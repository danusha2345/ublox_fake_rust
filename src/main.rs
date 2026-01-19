//! u-blox GNSS M8/M10 Emulator for RP2040/RP2350
//!
//! Dual-core architecture:
//! - Core0: Embassy executor, UART TX/RX, NAV/MON message generation
//! - Core1: LED control (PIO), SEC-SIGN ECDSA computation
//!
//! Author: Daniil, 2025

#![no_std]
#![no_main]

mod config;
mod coordinates;
mod flash_storage;
mod led;
mod passthrough;
mod sec_sign;
mod spoof_detector;
mod ubx;

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Flex, Level, Output, Pull};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::{FLASH, PIO0, UART0, UART1};
use embassy_rp::pio::Pio;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use embedded_io_async::{Read, Write};
use panic_probe as _;
use static_cell::StaticCell;

use config::*;
use sec_sign::{SecSignAccumulator, SecSignRequest, SecSignResult, Signature, DEFAULT_SESSION_ID, get_private_key};
use ubx::{
    AckAck, AckNak, MessageFlags, MonVer, NavDop, NavEoe, NavPvt, NavSol, NavStatus, SecSign, SecUniqid, UbxMessage,
    // Additional NAV messages
    NavPosecef, NavPosllh, NavVelned, NavVelecef, NavTimeutc, NavTimegps, NavClock, NavTimels,
    NavAopstatus, NavCov, NavHpposecef, NavSat, NavSvinfo,
    // MON messages
    MonHw, MonRf, MonComms,
    // TIM and RXM messages
    TimTp, RxmRawx,
    // DJI proprietary
    Cfg41,
    // CFG-VALGET response
    CfgValgetResponse,
};

// Interrupt bindings
bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

// ============================================================================
// UBX message sending macro
// ============================================================================

/// Macro to send a UBX message if the corresponding flag is enabled.
/// Reduces boilerplate for building and sending messages.
/// Logs warning on TX channel overflow.
///
/// Usage:
/// - `send_msg!(buf, flag, MsgType)` - sends default message
/// - `send_msg!(buf, flag, MsgType, { field1: val1, field2: val2 })` - with field overrides
macro_rules! send_msg {
    // Simple form: just default message
    ($buf:expr, $flag:expr, $msg_type:ty) => {
        if $flag {
            let msg = <$msg_type>::default();
            let len = msg.build(&mut $buf);
            if len > 0 {
                let mut vec = heapless::Vec::<u8, 1280>::new();
                let _ = vec.extend_from_slice(&$buf[..len]);
                if TX_CHANNEL.try_send(vec).is_err() {
                    warn!("TX channel full, dropped {} message", stringify!($msg_type));
                }
            }
        }
    };
    // With field overrides
    ($buf:expr, $flag:expr, $msg_type:ty, { $($field:ident : $value:expr),* $(,)? }) => {
        if $flag {
            let mut msg = <$msg_type>::default();
            $( msg.$field = $value; )*
            let len = msg.build(&mut $buf);
            if len > 0 {
                let mut vec = heapless::Vec::<u8, 1280>::new();
                let _ = vec.extend_from_slice(&$buf[..len]);
                if TX_CHANNEL.try_send(vec).is_err() {
                    warn!("TX channel full, dropped {} message", stringify!($msg_type));
                }
            }
        }
    };
}

// ============================================================================
// Inter-core communication
// ============================================================================

// TX channel for messages to drone (priority high)
static TX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 1280>, 32> = Channel::new();

// RX channel for messages from real GNSS (priority high)
// Increased buffer size to 1280 bytes for large UBX frames (RXM-RAWX with many satellites)
// Depth 128 prevents burst drops during SEC-SIGN and packet bursts (64 caused ~0.13% loss)
static GNSS_RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 1280>, 128> = Channel::new();

// RAW RX channel - intermediate buffer between UART1 RX and processing task
// Small chunks (256 bytes) to minimize time between UART reads, preventing overrun
// Depth 64 provides ~16KB buffer for burst absorption
static RAW_RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 256>, 64> = Channel::new();

/// Global message enable flags
static MSG_FLAGS_STATE: Mutex<CriticalSectionRawMutex, MessageFlags> = Mutex::new(MessageFlags::new_default());

/// Message output started flag (set 700ms after first config command)
/// Using AtomicBool instead of Signal because multiple tasks need to check this
static MSG_OUTPUT_STARTED: AtomicBool = AtomicBool::new(false);

/// Timestamp when first config command was received (for NAV delay)
/// 0 = no config yet, >0 = time of first config command
static FIRST_CONFIG_MILLIS: AtomicU32 = AtomicU32::new(0);

/// Global SEC-SIGN accumulator (accumulates sent messages)
static SEC_SIGN_ACC: Mutex<CriticalSectionRawMutex, Option<SecSignAccumulator>> = Mutex::new(None);

// Channels for SEC-SIGN (Replaced Signal with Channel to prevent race conditions/deadlocks)
static SEC_SIGN_REQUEST: Channel<CriticalSectionRawMutex, SecSignRequest, 2> = Channel::new();
static SEC_SIGN_RESULT: Channel<CriticalSectionRawMutex, SecSignResult, 2> = Channel::new();
static SEC_SIGN_DONE: Channel<CriticalSectionRawMutex, (), 2> = Channel::new();

/// SEC-SIGN computation in progress - pause TX to avoid race condition
/// When true: NAV/MON tasks skip sending, uart_tx_task waits for result
static SEC_SIGN_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

// ============================================================================
// Diagnostic counters for packet loss analysis (RTT logging)
// ============================================================================
static DIAG_RX_PACKETS: AtomicU32 = AtomicU32::new(0);   // Packets received from UART1
static DIAG_TX_PACKETS: AtomicU32 = AtomicU32::new(0);   // Packets sent to UART0
static DIAG_CHANNEL_DROPS: AtomicU32 = AtomicU32::new(0); // GNSS_RX_CHANNEL full drops
static DIAG_LOCAL_BUF_DROPS: AtomicU32 = AtomicU32::new(0); // Local buffer overflow during SEC-SIGN
static DIAG_SEC_SIGN_WAIT_MS: AtomicU32 = AtomicU32::new(0); // Last SEC-SIGN wait duration

/// Current operating mode (0 = Emulation, 1 = Passthrough)
static MODE: AtomicU8 = AtomicU8::new(2);

/// NAV message measurement period in milliseconds (default from config)
static NAV_MEAS_PERIOD_MS: AtomicU32 = AtomicU32::new(config::timers::NAV_MEAS_PERIOD_MS);

/// NAV message rate (cycles per navigation solution, default from config)
static NAV_RATE: AtomicU32 = AtomicU32::new(config::timers::NAV_RATE);

/// Time reference (0=UTC, 1=GPS, 2=GLONASS, etc.) - stored but not used
static NAV_TIMEREF: AtomicU8 = AtomicU8::new(0);

/// Drone model for SEC-SIGN key selection (0 = Air3, 1 = Mavic4Pro)
/// Default is Air 3, will be auto-detected from drone commands
static DRONE_MODEL: AtomicU8 = AtomicU8::new(0); // Air 3 (default)

/// Auto-detection state flags
/// Set to true once drone model is detected (prevents re-detection)
static DRONE_DETECTED: AtomicBool = AtomicBool::new(false);
/// Set to true if SEC-UNIQID poll received (Mavic 4 sends this before CFG-RST, Air 3 doesn't)
static SAW_SEC_UNIQID: AtomicBool = AtomicBool::new(false);
/// Set to true if CFG-VALGET received before CFG-VALSET (Mavic 4 pattern)
static SAW_CFG_VALGET: AtomicBool = AtomicBool::new(false);

/// Timestamp when message output started (for 20s invalid satellites timer)
static OUTPUT_START_MILLIS: AtomicU32 = AtomicU32::new(0);

/// Flag indicating satellites are invalid (for LED color change)
static SATELLITES_INVALID: AtomicBool = AtomicBool::new(false);

/// Signal for baudrate change (value = new baudrate)
static BAUDRATE_CHANGE: Signal<CriticalSectionRawMutex, u32> = Signal::new();

// ============================================================================
// Spoof Detection State (for passthrough mode)
// ============================================================================

/// Spoof detection active (satellites being masked)
pub static SPOOF_DETECTED: AtomicBool = AtomicBool::new(false);

/// Timestamp when spoof recovery started (for 5-second confirmation)
pub static SPOOF_RECOVERY_START_MS: AtomicU32 = AtomicU32::new(0);

/// Last known good coordinates before spoofing (lat * 1e-7)
pub static LAST_GOOD_LAT: portable_atomic::AtomicI32 = portable_atomic::AtomicI32::new(0);

/// Last known good coordinates before spoofing (lon * 1e-7)
pub static LAST_GOOD_LON: portable_atomic::AtomicI32 = portable_atomic::AtomicI32::new(0);

/// Last known good altitude before spoofing (mm)
pub static LAST_GOOD_ALT: portable_atomic::AtomicI32 = portable_atomic::AtomicI32::new(0);

/// Флаг для сброса спуф-детектора при смене режима (предотвращает ложные телепорты)
/// Устанавливается при переключении в Passthrough, сбрасывается в gnss_processing_task
pub static SPOOF_DETECTOR_RESET: AtomicBool = AtomicBool::new(false);

/// Flash mutex type for mode persistence
type FlashMutex = Mutex<CriticalSectionRawMutex, Flash<'static, FLASH, Async, { config::FLASH_SIZE_BYTES }>>;

/// Flash for mode persistence (initialized once in main)
static FLASH_CELL: StaticCell<FlashMutex> = StaticCell::new();

/// Core1 stack
static mut CORE1_STACK: Stack<8192> = Stack::new();

// ============================================================================
// Operating Mode
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub enum OperatingMode {
    #[default]
    Emulation = 0,
    Passthrough = 1,
    PassthroughRaw = 2,  // Passthrough without spoof detection
}

impl OperatingMode {
    fn load() -> Self {
        match MODE.load(Ordering::Acquire) {
            0 => Self::Emulation,
            1 => Self::Passthrough,
            _ => Self::PassthroughRaw,
        }
    }

    fn store(self) {
        MODE.store(self as u8, Ordering::Release);
    }
}

// ============================================================================
// Core0 Entry Point
// ============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize coordinate conversion (LLH -> ECEF) from config
    coordinates::init();

    // ===== MINIMAL WS2812 TEST - blink 3 times at startup =====
    {
        use embassy_rp::pio::Pio;
        use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program, Rgb};
        use smart_leds::RGB8;

        let mut pio0 = Pio::new(p.PIO0, Irqs);
        let program = PioWs2812Program::new(&mut pio0.common);
        let mut ws: PioWs2812<_, 0, 1, Rgb> = PioWs2812::with_color_order(&mut pio0.common, pio0.sm0, p.DMA_CH1, p.PIN_16, &program);

        for _ in 0..3 {
            ws.write(&[RGB8::new(0, 50, 0)]).await; // Green
            Timer::after(Duration::from_millis(200)).await;
            ws.write(&[RGB8::new(0, 0, 0)]).await; // Off
            Timer::after(Duration::from_millis(200)).await;
        }
    }
    // ===== END TEST =====

    // Re-init peripherals (PIO0 was consumed by test)
    let p = unsafe { embassy_rp::Peripherals::steal() };

    // Initialize flash for mode persistence
    let flash = Flash::<_, Async, { config::FLASH_SIZE_BYTES }>::new(p.FLASH, p.DMA_CH0);
    let flash_mutex = FLASH_CELL.init(Mutex::new(flash));

    // Load saved mode from flash
    {
        let mut flash = flash_mutex.lock().await;
        if let Some(mode_byte) = flash_storage::load_mode(&mut flash) {
            let mode = match mode_byte {
                0 => OperatingMode::Emulation,
                1 => OperatingMode::Passthrough,
                _ => OperatingMode::PassthroughRaw,
            };
            mode.store();
            info!("Loaded mode {} from flash: {:?}", mode_byte, mode);
        } else {
            // No saved mode, default to Emulation
            info!("No saved mode, defaulting to Emulation");
            OperatingMode::Emulation.store();
        }
    };

    // Initialize PIO0 for WS2812 LED (GPIO16 on RP2350-Tiny)
    let pio0 = Pio::new(p.PIO0, Irqs);
    let dma_ch1 = p.DMA_CH1;
    let led_pin = p.PIN_16;  // WS2812B on GPIO16 (RP2350-Tiny)

    // Mode button using Flex for E9 workaround (GPIO10=PWR, GPIO11=IN)
    let _btn_pwr = Output::new(p.PIN_10, Level::High);
    let btn_flex = Flex::new(p.PIN_11);

    // Spawn Core1 for LED, SEC-SIGN computation, and MON messages
    // MON runs on Core1 to balance load (Core0 handles NAV at higher rate)
    static EXECUTOR1: StaticCell<embassy_executor::Executor> = StaticCell::new();
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(embassy_executor::Executor::new());
            executor1.run(|spawner: embassy_executor::Spawner| {
                spawner.must_spawn(led_task(pio0, dma_ch1, led_pin));
                spawner.must_spawn(sec_sign_compute_task());
                spawner.must_spawn(mon_message_task());
            });
        },
    );

    // ========================================================================
    // UART0: Communication with drone/host (TX=GPIO0, RX=GPIO1)
    // ========================================================================
    static TX_BUF0: StaticCell<[u8; 2048]> = StaticCell::new();
    static RX_BUF0: StaticCell<[u8; 512]> = StaticCell::new();
    let tx_buf0 = &mut TX_BUF0.init([0; 2048])[..];
    let rx_buf0 = &mut RX_BUF0.init([0; 512])[..];

    let mut uart0_config = UartConfig::default();
    uart0_config.baudrate = DEFAULT_BAUDRATE;
    let uart0 = BufferedUart::new(
        p.UART0,
        p.PIN_0, // TX
        p.PIN_1, // RX
        Irqs,
        tx_buf0,
        rx_buf0,
        uart0_config,
    );
    let (uart0_tx, uart0_rx) = uart0.split();

    // ========================================================================
    // UART1: External GNSS module input (RX=GPIO5)
    // ========================================================================
    static TX_BUF1: StaticCell<[u8; 128]> = StaticCell::new();
    static RX_BUF1: StaticCell<[u8; 4096]> = StaticCell::new();
    let tx_buf1 = &mut TX_BUF1.init([0; 128])[..];
    let rx_buf1 = &mut RX_BUF1.init([0; 4096])[..];

    let mut uart1_config = UartConfig::default();
    uart1_config.baudrate = DEFAULT_BAUDRATE;
    let uart1 = BufferedUart::new(
        p.UART1,
        p.PIN_4, // TX (not used, but required)
        p.PIN_5, // RX from external GNSS
        Irqs,
        tx_buf1,
        rx_buf1,
        uart1_config,
    );

    // Reduce FIFO threshold from 7/8 (14 bytes) to 1/4 (4 bytes)
    // This triggers interrupt 3.4x more often, reducing overrun risk
    // Embassy-rp defaults to 7/8 which is too slow for 921600 baud
    rp_pac::UART1.uartifls().write(|w| {
        w.set_rxiflsel(0b001); // 1/4 full (4 bytes) - was 0b100 (14 bytes)
        w.set_txiflsel(0b000); // TX unchanged
    });
    info!("UART1 RX FIFO threshold set to 1/4 (4 bytes)");

    let (_uart1_tx, uart1_rx) = uart1.split();

    // Initialize SEC-SIGN accumulator
    {
        let mut acc = SEC_SIGN_ACC.try_lock().unwrap();
        *acc = Some(SecSignAccumulator::new());
    }

    // ========================================================================
    // Spawn all tasks - they check MODE internally for hot-switching
    // ========================================================================
    let mode = OperatingMode::load();
    info!("Starting in {:?} mode (hot-switchable)", mode);

    // Core communication tasks (always running)
    spawner.must_spawn(uart0_tx_task(uart0_tx));
    spawner.must_spawn(uart0_rx_task(uart0_rx));
    spawner.must_spawn(uart1_rx_task(uart1_rx));
    spawner.must_spawn(gnss_processing_task()); // Separated from uart1_rx to prevent overrun

    // Emulation tasks (check MODE internally, skip work in passthrough)
    // Note: mon_message_task runs on Core1 for load balancing
    spawner.must_spawn(nav_message_task());
    spawner.must_spawn(sec_sign_timer_task());

    // Button task for mode switching (no reboot!)
    // Button task for mode switching (no reboot!)
    spawner.must_spawn(button_task(btn_flex, flash_mutex));

    // Diagnostic stats task (every 10 seconds)
    spawner.must_spawn(diag_stats_task());

    info!("All tasks spawned, hot-switch enabled");

    // Core0 main loop - idle
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

// ============================================================================
// Core1 Tasks
// ============================================================================

/// LED control task - WS2812B on PIO (runs on Core1)
/// GPIO16 on RP2350-Tiny
/// Green = Emulation mode, Blue = Passthrough mode
/// Blinking Red = Spoofing detected (in passthrough mode)
#[embassy_executor::task]
async fn led_task(
    mut pio: Pio<'static, PIO0>,
    dma: embassy_rp::Peri<'static, embassy_rp::peripherals::DMA_CH1>,
    pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_16>,
) {
    use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program, Rgb};
    use smart_leds::RGB8;

    let program = PioWs2812Program::new(&mut pio.common);
    let mut ws: PioWs2812<_, 0, 1, Rgb> = PioWs2812::with_color_order(&mut pio.common, pio.sm0, dma, pin, &program);

    // Faster tick for smooth blinking during spoof detection
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut blink_counter: u8 = 0;

    loop {
        blink_counter = blink_counter.wrapping_add(1);
        let mode = OperatingMode::load();
        let sats_invalid = SATELLITES_INVALID.load(Ordering::Acquire);
        let spoof_detected = SPOOF_DETECTED.load(Ordering::Acquire);

        let color = match mode {
            OperatingMode::Passthrough => {
                if spoof_detected {
                    // Fast blinking red (200ms cycle = on 100ms, off 100ms)
                    if blink_counter.is_multiple_of(2) {
                        RGB8::new(50, 0, 0)  // Red ON
                    } else {
                        RGB8::new(0, 0, 0)   // OFF
                    }
                } else {
                    // Normal passthrough: solid blue (blink every 5 ticks = 500ms)
                    if blink_counter % 5 < 3 {
                        RGB8::new(0, 0, 30)  // Blue
                    } else {
                        RGB8::new(0, 0, 0)   // Off
                    }
                }
            }
            OperatingMode::PassthroughRaw => {
                // Raw passthrough (no spoof detection): purple (blink every 5 ticks = 500ms)
                if blink_counter % 5 < 3 {
                    RGB8::new(20, 0, 30)  // Purple
                } else {
                    RGB8::new(0, 0, 0)    // Off
                }
            }
            OperatingMode::Emulation => {
                // Slow blink (500ms cycle)
                if blink_counter % 5 < 3 {
                    if sats_invalid {
                        RGB8::new(30, 20, 0)  // Yellow (satellites invalid)
                    } else {
                        RGB8::new(0, 30, 0)   // Green (satellites valid)
                    }
                } else {
                    RGB8::new(0, 0, 0) // Off
                }
            }
        };

        let _ = ws.write(&[color]).await;
        ticker.next().await;
    }
}

/// SEC-SIGN computation task (runs on Core1 - CPU intensive)
#[embassy_executor::task]
async fn sec_sign_compute_task() {
    info!("SEC-SIGN compute task starting on Core1");

    // Initialize RNG for micro-ecc (uses RP2040 ROSC)
    sec_sign::init_rng();

    loop {
        // Wait for request from Core0
        let request = SEC_SIGN_REQUEST.receive().await;

        info!("Computing SEC-SIGN for {} packets", request.packet_count);

        // Compute z = fold(SHA256(hash || session_id))
        let z = sec_sign::compute_z(&request.sha256_hash, &request.session_id);

        // Get private key for current drone model
        let model = DroneModel::from_u8(DRONE_MODEL.load(Ordering::Acquire));
        let private_key = get_private_key(model);

        // Sign with ECDSA SECP192R1
        let signature = Signature::sign(&z, private_key).unwrap_or_default();

        // Send result back to Core0
        let result = SecSignResult {
            sha256_hash: request.sha256_hash,
            session_id: request.session_id,
            signature,
            packet_count: request.packet_count,
        };

        let _ = SEC_SIGN_RESULT.try_send(result);
        info!("SEC-SIGN computation complete");
    }
}

// ============================================================================
// Core0 Tasks
// ============================================================================

/// UART0 TX task - sends data to drone/host
/// In Emulation mode: sends generated UBX from TX_CHANNEL + SEC-SIGN
/// In Passthrough mode: forwards data from GNSS_RX_CHANNEL (external GNSS)
#[embassy_executor::task]
async fn uart0_tx_task(mut tx: embassy_rp::uart::BufferedUartTx) {
    use embassy_futures::select::{select, select3, Either, Either3};

    Timer::after(Duration::from_millis(50)).await;
    info!("UART0 TX task ready (hot-switch enabled)");

    loop {
        let mode = OperatingMode::load();

        match mode {
            OperatingMode::Emulation => {
                // Emulation: wait for TX_CHANNEL or SEC-SIGN result
                match select3(
                    SEC_SIGN_RESULT.receive(),
                    TX_CHANNEL.receive(),
                    GNSS_RX_CHANNEL.receive(), // Drain passthrough channel if any
                ).await {
                    Either3::First(result) => {
                        // SEC-SIGN result received - send signature
                        let sec_sign_msg = SecSign {
                            version: 0x01,
                            reserved1: 0,
                            msg_cnt: result.packet_count,
                            sha256_hash: result.sha256_hash,
                            session_id: result.session_id,
                            signature_r: result.signature.r,
                            signature_s: result.signature.s,
                        };

                        let mut buf = [0u8; 128];
                        let len = sec_sign_msg.build(&mut buf);
                        if len > 0 {
                            if let Err(e) = tx.write_all(&buf[..len]).await {
                                error!("UART TX SEC-SIGN error: {:?}", e);
                            }
                            info!("SEC-SIGN sent ({} packets)", result.packet_count);
                        }
                        // Clear flag and wake waiting tasks (NAV/MON)
                        SEC_SIGN_IN_PROGRESS.store(false, Ordering::Release);
                        let _ = SEC_SIGN_DONE.try_send(());
                    }
                    Either3::Second(msg) => {
                        // Regular emulation message
                        if SEC_SIGN_IN_PROGRESS.load(Ordering::Acquire) {
                            if TX_CHANNEL.try_send(msg).is_err() {
                                error!("Failed to re-queue message during SEC-SIGN!");
                            }
                            continue;
                        }

                        if let Err(e) = tx.write_all(&msg).await {
                            error!("UART TX error: {:?}", e);
                            continue;
                        }

                        // Accumulate for SEC-SIGN
                        let is_sec_sign = msg.len() >= 4 && msg[2] == 0x27 && msg[3] == 0x04;
                        if !is_sec_sign {
                            let mut acc = SEC_SIGN_ACC.lock().await;
                            if let Some(ref mut accumulator) = *acc {
                                accumulator.accumulate(&msg);
                            }
                        }
                    }
                    Either3::Third(_) => {
                        // Discard passthrough data in emulation mode
                    }
                }
            }
            OperatingMode::Passthrough => {
                // Passthrough mode with SEC-SIGN generation (both Spoof and Normal)
                // Use timeout to limit blocking time to 100ms for mode switching check
                match embassy_time::with_timeout(
                    Duration::from_millis(100),
                    select(
                        SEC_SIGN_RESULT.receive(),
                        GNSS_RX_CHANNEL.receive(),
                    )
                ).await {
                    Ok(Either::First(result)) => {
                        // SEC-SIGN computed - send it
                        let sec_sign_msg = SecSign {
                            version: 0x01,
                            reserved1: 0,
                            msg_cnt: result.packet_count,
                            sha256_hash: result.sha256_hash,
                            session_id: result.session_id,
                            signature_r: result.signature.r,
                            signature_s: result.signature.s,
                        };

                        let mut buf = [0u8; 128];
                        let len = sec_sign_msg.build(&mut buf);
                        if len > 0 {
                            if let Err(e) = tx.write_all(&buf[..len]).await {
                                error!("Passthrough SEC-SIGN TX error: {:?}", e);
                            }
                        }
                        SEC_SIGN_IN_PROGRESS.store(false, Ordering::Release);
                        let _ = SEC_SIGN_DONE.try_send(());
                    }
                    Ok(Either::Second(msg)) => {
                        // If SEC-SIGN is in progress, buffer packets locally while waiting
                        // This prevents GNSS_RX_CHANNEL overflow during ~59ms ECDSA computation
                        if SEC_SIGN_IN_PROGRESS.load(Ordering::Acquire) {
                            let wait_start = embassy_time::Instant::now();

                            // Local buffer for packets during signature wait
                            // 72 slots handles ~66ms of GNSS data (vs ~59ms ECDSA), covers high satellite count (32+ sats)
                            let mut pending: heapless::Vec<heapless::Vec<u8, 1280>, 72> = heapless::Vec::new();
                            let _ = pending.push(msg); // First packet goes to buffer

                            // Keep draining channel while waiting for signature (non-blocking!)
                            loop {
                                match select(
                                    SEC_SIGN_RESULT.receive(),
                                    GNSS_RX_CHANNEL.receive(),
                                ).await {
                                    Either::First(result) => {
                                        // Signature ready - send it FIRST (before buffered packets)
                                        let wait_ms = wait_start.elapsed().as_millis() as u32;
                                        DIAG_SEC_SIGN_WAIT_MS.store(wait_ms, Ordering::Relaxed);
                                        info!("SEC-SIGN wait={}ms buffered={} pkts", wait_ms, pending.len());

                                        let sec_sign_msg = SecSign {
                                            version: 0x01,
                                            reserved1: 0,
                                            msg_cnt: result.packet_count,
                                            sha256_hash: result.sha256_hash,
                                            session_id: result.session_id,
                                            signature_r: result.signature.r,
                                            signature_s: result.signature.s,
                                        };

                                        let mut buf = [0u8; 128];
                                        let len = sec_sign_msg.build(&mut buf);
                                        if len > 0 {
                                            if let Err(e) = tx.write_all(&buf[..len]).await {
                                                error!("Passthrough SEC-SIGN TX error: {:?}", e);
                                            }
                                        }
                                        SEC_SIGN_IN_PROGRESS.store(false, Ordering::Release);
                                        let _ = SEC_SIGN_DONE.try_send(());
                                        break;
                                    }
                                    Either::Second(next_msg) => {
                                        // Another packet arrived - buffer it
                                        let msg_len = next_msg.len();
                                        if pending.push(next_msg).is_err() {
                                            let drops = DIAG_LOCAL_BUF_DROPS.fetch_add(1, Ordering::Relaxed);
                                            warn!("LOCAL_BUF overflow! size={} drops={}", msg_len, drops + 1);
                                        }
                                    }
                                }
                            }

                            // Now send all buffered packets AFTER the signature
                            for buffered_msg in pending.into_iter() {
                                if let Err(e) = tx.write_all(&buffered_msg).await {
                                    error!("Passthrough TX error (buffered): {:?}", e);
                                    continue;
                                }
                                DIAG_TX_PACKETS.fetch_add(1, Ordering::Relaxed);
                                // Accumulate for next SEC-SIGN only if still spoofing
                                // (spoofing may have ended while waiting for ECDSA)
                                if SPOOF_DETECTED.load(Ordering::Acquire) {
                                    let is_sec_sign = buffered_msg.len() >= 4
                                        && buffered_msg[2] == 0x27 && buffered_msg[3] == 0x04;
                                    if !is_sec_sign {
                                        let mut acc = SEC_SIGN_ACC.lock().await;
                                        if let Some(ref mut accumulator) = *acc {
                                            accumulator.accumulate(&buffered_msg);
                                        }
                                    }
                                }
                            }
                            continue; // Skip normal msg handling (already processed)
                        }

                        // Normal path (no SEC-SIGN in progress): send immediately
                        if let Err(e) = tx.write_all(&msg).await {
                            error!("Passthrough TX error: {:?}", e);
                            continue;
                        }
                        DIAG_TX_PACKETS.fetch_add(1, Ordering::Relaxed);

                        // Accumulate hash only when spoofing (we generate our own SEC-SIGN)
                        // Without spoofing, original SEC-SIGN passes through - no accumulation needed
                        if SPOOF_DETECTED.load(Ordering::Acquire) {
                            let is_sec_sign = msg.len() >= 4 && msg[2] == 0x27 && msg[3] == 0x04;
                            if !is_sec_sign {
                                let mut acc = SEC_SIGN_ACC.lock().await;
                                if let Some(ref mut accumulator) = *acc {
                                    accumulator.accumulate(&msg);
                                }
                            }
                        }
                    }
                    Err(_) => {
                        // Timeout - just loop to check mode
                    }
                }
            }
            OperatingMode::PassthroughRaw => {
                // Raw passthrough: just forward without any processing
                // Use timeout to allow checking mode changes even if no data arrives
                match embassy_time::with_timeout(Duration::from_millis(100), GNSS_RX_CHANNEL.receive()).await {
                    Ok(msg) => {
                        if let Err(e) = tx.write_all(&msg).await {
                            error!("PassthroughRaw TX error: {:?}", e);
                        } else {
                            DIAG_TX_PACKETS.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    Err(_) => {
                        // Timeout, loop to check mode again
                    }
                }
            }
        }
    }
}

/// Periodic diagnostic statistics output (every 10 seconds)
/// Outputs RX/TX packet counts and drop statistics via RTT
#[embassy_executor::task]
async fn diag_stats_task() {
    let mut ticker = embassy_time::Ticker::every(Duration::from_secs(10));
    loop {
        ticker.next().await;

        let rx_pkts = DIAG_RX_PACKETS.load(Ordering::Relaxed);
        let tx_pkts = DIAG_TX_PACKETS.load(Ordering::Relaxed);
        let ch_drops = DIAG_CHANNEL_DROPS.load(Ordering::Relaxed);
        let buf_drops = DIAG_LOCAL_BUF_DROPS.load(Ordering::Relaxed);
        let sec_wait = DIAG_SEC_SIGN_WAIT_MS.load(Ordering::Relaxed);

        // Calculate in-flight packets (RX - TX)
        let in_flight = rx_pkts.saturating_sub(tx_pkts);

        info!("=== DIAG: RX={} TX={} in_flight={} ch_drops={} buf_drops={} sec_wait={}ms ===",
              rx_pkts, tx_pkts, in_flight, ch_drops, buf_drops, sec_wait);
    }
}

/// Set UART0 baudrate using direct register access
fn set_uart0_baudrate(baudrate: u32) {
    let clk_peri = embassy_rp::clocks::clk_peri_freq();
    let baud_rate_div = (8 * clk_peri) / baudrate;
    let mut baud_ibrd = baud_rate_div >> 7;
    let mut baud_fbrd = (baud_rate_div & 0x7f).div_ceil(2);

    if baud_ibrd == 0 {
        baud_ibrd = 1;
        baud_fbrd = 0;
    } else if baud_ibrd >= 65535 {
        baud_ibrd = 65535;
        baud_fbrd = 0;
    }

    let r = rp_pac::UART0;

    // Wait for UART to finish transmitting
    while r.uartfr().read().busy() {}

    // Disable UART
    r.uartcr().write(|w| w.set_uarten(false));

    // Set baud rate
    r.uartibrd().write(|w| w.set_baud_divint(baud_ibrd as u16));
    r.uartfbrd().write(|w| w.set_baud_divfrac(baud_fbrd as u8));

    // Dummy write to latch new baud rate
    r.uartlcr_h().modify(|_| {});

    // Re-enable UART
    r.uartcr().write(|w| {
        w.set_uarten(true);
        w.set_rxe(true);
        w.set_txe(true);
    });
}

/// UART0 RX task - receives and processes UBX commands from drone
/// Runs in BOTH modes to preserve settings during hot-switch
#[embassy_executor::task]
async fn uart0_rx_task(mut rx: embassy_rp::uart::BufferedUartRx) {
    let mut buf = [0u8; 128];
    let mut parser = ubx::UbxParser::new();

    loop {
        // Check for pending baudrate change
        if let Some(new_baudrate) = BAUDRATE_CHANGE.try_take() {
            info!("Applying baudrate change to {}", new_baudrate);
            set_uart0_baudrate(new_baudrate);
        }

        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                for &byte in &buf[..n] {
                    if let Some(cmd) = parser.parse_byte(byte) {
                        // Process commands in BOTH modes - preserves settings
                        handle_ubx_command(&cmd).await;
                    }
                }

                // Apply baudrate change after processing commands
                if let Some(new_baudrate) = BAUDRATE_CHANGE.try_take() {
                    info!("Applying baudrate change to {}", new_baudrate);
                    set_uart0_baudrate(new_baudrate);
                }
            }
            Ok(_) => {}
            Err(e) => {
                error!("UART0 RX error: {:?}", e);
            }
        }
    }
}

/// UART1 RX task - FAST path, only reads and forwards to RAW_RX_CHANNEL
/// Processing is done in gnss_processing_task to minimize overrun risk
#[embassy_executor::task]
async fn uart1_rx_task(mut rx: embassy_rp::uart::BufferedUartRx) {
    let mut buf = [0u8; 256]; // Smaller buffer = more frequent reads = less overrun

    info!("UART1 RX task ready (fast path)");

    loop {
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                // Minimal processing: just copy and send
                let mut vec = heapless::Vec::<u8, 256>::new();
                if vec.extend_from_slice(&buf[..n]).is_ok()
                    && RAW_RX_CHANNEL.try_send(vec).is_err()
                {
                    let drops = DIAG_CHANNEL_DROPS.fetch_add(1, Ordering::Relaxed);
                    warn!("RAW_RX_CHANNEL full! drops={}", drops + 1);
                }
            }
            Ok(_) => {}
            Err(e) => {
                error!("UART1 RX error: {:?}", e);
            }
        }
    }
}

/// GNSS processing task - handles parsing, spoof detection, NAV modification
/// Receives raw data from RAW_RX_CHANNEL, processes it, sends to GNSS_RX_CHANNEL
#[embassy_executor::task]
async fn gnss_processing_task() {
    use passthrough::{
        UbxFrameParser, PositionBuffer, extract_position_from_pvt, extract_gnss_time_from_pvt,
        extract_cno_from_nav_sat,
        modify_nav_pvt, modify_nav_sol, modify_nav_status, modify_nav_sat, modify_nav_svinfo,
        recalc_checksum,
    };
    use spoof_detector::{SpoofDetector, Position, AnalysisResult, FixType};

    // Lazy initialization of heavy structs
    let mut parser: Option<UbxFrameParser> = None;
    let mut detector: Option<SpoofDetector> = None;
    let mut pos_buffer: Option<PositionBuffer> = None;

    // CNO buffer - accumulates CNO values from NAV-SAT messages
    let mut last_cno_values: heapless::Vec<u8, 16> = heapless::Vec::new();

    info!("GNSS processing task ready");

    loop {
        let raw_data = RAW_RX_CHANNEL.receive().await;
        let mode = OperatingMode::load();

        if mode == OperatingMode::PassthroughRaw {
            // Raw passthrough: just forward data without any parsing or processing
            let mut vec = heapless::Vec::<u8, 1280>::new();
            if vec.extend_from_slice(&raw_data).is_ok() {
                if GNSS_RX_CHANNEL.try_send(vec).is_err() {
                    warn!("GNSS_RX_CHANNEL full (raw)!");
                } else {
                    DIAG_RX_PACKETS.fetch_add(1, Ordering::Relaxed);
                }
            }
        } else if mode == OperatingMode::Passthrough {
            // Initialize parsers if first time entering Passthrough
            if parser.is_none() {
                parser = Some(UbxFrameParser::new());
                detector = Some(SpoofDetector::new());
                pos_buffer = Some(PositionBuffer::new());
                info!("Initialized SpoofDetector and Parser for Passthrough mode");
            }

            // Проверяем флаг сброса детектора (при переключении режимов)
            if SPOOF_DETECTOR_RESET.swap(false, Ordering::AcqRel) {
                if let Some(ref mut d) = detector {
                    d.reset();
                    info!("SpoofDetector reset by mode switch (SPOOF_DETECTOR_RESET)");
                }
                // Также сбрасываем глобальные флаги спуфинга
                SPOOF_DETECTED.store(false, Ordering::Release);
                SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
            }

            let parser = parser.as_mut().unwrap();
            let detector = detector.as_mut().unwrap();
            let pos_buffer = pos_buffer.as_mut().unwrap();

            // Passthrough mode: parse UBX frames, detect spoofing
            for &byte in raw_data.iter() {
                if let Some(mut frame) = parser.feed(byte) {
                    let class = frame[2];
                    let id = frame[3];
                    let now_ms = embassy_time::Instant::now().as_millis() as u32;

                    // Extract CNO values from NAV-SAT (0x01, 0x35)
                    if class == 0x01 && id == 0x35 && frame.len() >= 14 {
                        last_cno_values = extract_cno_from_nav_sat(&frame[6..]);
                    }

                    // Detect spoofing ONLY from NAV-PVT (0x01, 0x07)
                    if class == 0x01 && id == 0x07 && frame.len() >= 100 {
                        if let Some((lat, lon, alt, h_acc, _speed, num_sv)) =
                            extract_position_from_pvt(&frame[6..])
                        {
                            pos_buffer.push(lat, lon, alt, now_ms);
                            let fix_type = FixType::from_u8(frame[6 + 20]);
                            let gnss_time = extract_gnss_time_from_pvt(&frame[6..], now_ms);

                            let pos = Position {
                                lat,
                                lon,
                                alt_mm: alt,
                                time_ms: now_ms,
                                fix_type,
                                h_acc_mm: h_acc,
                                num_sv,
                                pdop: 100,
                                gnss_time,
                                cno_values: last_cno_values.clone(),
                            };

                            let result = detector.analyze(pos);
                            let was_spoofed = SPOOF_DETECTED.load(Ordering::Acquire);
                            let is_spoofed = result == AnalysisResult::Spoofed;

                            if is_spoofed && !was_spoofed {
                                if let Some((good_lat, good_lon, good_alt)) =
                                    pos_buffer.get_position_at(2, now_ms)
                                {
                                    LAST_GOOD_LAT.store(good_lat, Ordering::Release);
                                    LAST_GOOD_LON.store(good_lon, Ordering::Release);
                                    LAST_GOOD_ALT.store(good_alt, Ordering::Release);
                                    info!("Spoof detected! Saved good coords: lat={}, lon={}",
                                          good_lat, good_lon);
                                }
                                SPOOF_DETECTED.store(true, Ordering::Release);
                                SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                                warn!("SPOOF DETECTED");
                            } else if !is_spoofed && was_spoofed {
                                let recovery_start = SPOOF_RECOVERY_START_MS.load(Ordering::Acquire);
                                if recovery_start == 0 {
                                    SPOOF_RECOVERY_START_MS.store(now_ms, Ordering::Release);
                                    info!("Spoof may have ended, starting 5s recovery timer");
                                } else {
                                    let elapsed = now_ms.wrapping_sub(recovery_start);
                                    if elapsed >= 5000 {
                                        SPOOF_DETECTED.store(false, Ordering::Release);
                                        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                                        info!("Spoof recovery complete after 5s of clean data");

                                        if let Ok(mut acc) = SEC_SIGN_ACC.try_lock() {
                                            if let Some(ref mut accumulator) = *acc {
                                                accumulator.reset();
                                                info!("SEC_SIGN_ACC reset after spoof recovery");
                                            }
                                        }
                                    }
                                }
                            } else if is_spoofed {
                                SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                            }
                        }
                    }

                    // Modify ALL NAV messages if spoofing detected
                    if class == 0x01 && SPOOF_DETECTED.load(Ordering::Acquire) {
                        match id {
                            0x07 => modify_nav_pvt(&mut frame),
                            0x06 => modify_nav_sol(&mut frame),
                            0x03 => modify_nav_status(&mut frame),
                            0x35 => modify_nav_sat(&mut frame),
                            0x30 => modify_nav_svinfo(&mut frame),
                            _ => {}
                        }
                        recalc_checksum(&mut frame);
                    }

                    // Filter SEC-SIGN in Passthrough based on spoof state:
                    // - Without spoofing: pass through original SEC-SIGN from real GNSS
                    // - With spoofing: filter out (NAV modified, we generate our own)
                    if class == 0x27 && id == 0x04 {
                        if SPOOF_DETECTED.load(Ordering::Acquire) {
                            // Spoofing active - filter original SEC-SIGN, our timer generates replacement
                            continue;
                        }
                        // No spoofing - let original SEC-SIGN pass through
                    }

                    // Send frame to TX channel
                    let mut vec = heapless::Vec::<u8, 1280>::new();
                    let copy_len = frame.len().min(1280);
                    if vec.extend_from_slice(&frame[..copy_len]).is_ok() {
                        if GNSS_RX_CHANNEL.try_send(vec).is_err() {
                            let drops = DIAG_CHANNEL_DROPS.fetch_add(1, Ordering::Relaxed);
                            warn!("GNSS_RX_CHANNEL full! cls={:#04x} id={:#04x} len={} drops={}",
                                  class, id, frame.len(), drops + 1);
                        } else {
                            DIAG_RX_PACKETS.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }

            // Send accumulated non-UBX data (NMEA, RTCM, etc.)
            if parser.is_idle() {
                if let Some(non_ubx_data) = parser.take_non_ubx_data() {
                    if GNSS_RX_CHANNEL.try_send(non_ubx_data).is_err() {
                        warn!("GNSS_RX_CHANNEL full (non-UBX)!");
                    } else {
                        DIAG_RX_PACKETS.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
        // Emulation mode: ignore raw data (uart0_tx_task generates its own)
    }
}

/// Send ACK-ACK response for a command
fn send_ack(cls_id: u8, msg_id: u8) {
    let ack = AckAck { cls_id, msg_id };
    let mut buf = [0u8; 16];
    let len = ack.build(&mut buf);
    if len > 0 {
        let mut vec = heapless::Vec::new();
        let _ = vec.extend_from_slice(&buf[..len]);
        if TX_CHANNEL.try_send(vec).is_err() {
            warn!("TX channel full, dropped ACK for {:02X}:{:02X}", cls_id, msg_id);
        }
    }
}

/// Record first config command time (called for all config commands)
/// This starts the 700ms countdown to NAV output
fn record_first_config() {
    // Only record if not already set (first command)
    if FIRST_CONFIG_MILLIS.load(Ordering::Acquire) == 0 {
        let now = embassy_time::Instant::now().as_millis() as u32;
        // Use compare_exchange to ensure only first call sets the value
        let _ = FIRST_CONFIG_MILLIS.compare_exchange(0, now, Ordering::AcqRel, Ordering::Acquire);
        info!("First config command received at {}ms", now);
    }
}

/// Send ACK-NAK response for unknown/failed command
#[allow(dead_code)]
fn send_nak(cls_id: u8, msg_id: u8) {
    let nak = AckNak { cls_id, msg_id };
    let mut buf = [0u8; 16];
    let len = nak.build(&mut buf);
    if len > 0 {
        let mut vec = heapless::Vec::new();
        let _ = vec.extend_from_slice(&buf[..len]);
        if TX_CHANNEL.try_send(vec).is_err() {
            warn!("TX channel full, dropped NAK for {:02X}:{:02X}", cls_id, msg_id);
        }
    }
}

/// Send a UBX message (for poll responses)
fn send_ubx_message<M: UbxMessage>(msg: &M) {
    let mut buf = [0u8; 256];
    let len = msg.build(&mut buf);
    if len > 0 {
        let mut vec = heapless::Vec::new();
        let _ = vec.extend_from_slice(&buf[..len]);
        if TX_CHANNEL.try_send(vec).is_err() {
            warn!("TX channel full, dropped UBX {:02X}:{:02X}", msg.class(), msg.id());
        }
    }
}

/// CFG-VALSET key IDs
mod valset_keys {
    // MSGOUT keys for UART1
    pub const NAV_PVT: u32 = 0x20910007;
    pub const NAV_SVINFO: u32 = 0x2091000C;
    pub const NAV_SAT: u32 = 0x20910016;
    pub const NAV_STATUS: u32 = 0x2091001B;
    pub const NAV_POSECEF: u32 = 0x20910025;
    pub const NAV_POSLLH: u32 = 0x2091002A;
    pub const NAV_HPPOSECEF: u32 = 0x20910036;
    pub const NAV_DOP: u32 = 0x20910039;
    pub const NAV_VELECEF: u32 = 0x2091003E;
    pub const NAV_VELNED: u32 = 0x20910043;
    pub const NAV_TIMEGPS: u32 = 0x20910048;
    pub const NAV_TIMEUTC: u32 = 0x2091005C;
    pub const NAV_TIMELS: u32 = 0x20910061;
    pub const NAV_CLOCK: u32 = 0x20910066;
    pub const NAV_AOPSTATUS: u32 = 0x2091007A;
    pub const NAV_COV: u32 = 0x20910084;
    pub const NAV_EOE: u32 = 0x20910160;
    pub const TIM_TP: u32 = 0x2091017E;
    pub const MON_HW: u32 = 0x209101B5;
    pub const RXM_RAWX: u32 = 0x209102A5;
    pub const MON_COMMS: u32 = 0x20910350;
    pub const MON_RF: u32 = 0x2091035A;

    // Rate configuration keys
    pub const RATE_MEAS: u32 = 0x30210001;     // Measurement period in ms (u16)
    pub const RATE_NAV: u32 = 0x30210002;      // Navigation rate cycles (u16)
    pub const RATE_TIMEREF: u32 = 0x20210003;  // Time reference (u8)

    // UART baudrate keys
    pub const UART1_BAUDRATE: u32 = 0x40520001;  // UART1 baudrate (u32)
}

/// Update message flags from CFG-VALSET key
fn update_flag_from_valset_key(flags: &mut MessageFlags, key: u32, val: u32) {
    let enabled = val > 0;
    match key {
        valset_keys::NAV_PVT => flags.nav_pvt = enabled,
        valset_keys::NAV_SVINFO => flags.nav_svinfo = enabled,
        valset_keys::NAV_SAT => flags.nav_sat = enabled,
        valset_keys::NAV_STATUS => flags.nav_status = enabled,
        valset_keys::NAV_POSECEF => flags.nav_posecef = enabled,
        valset_keys::NAV_POSLLH => flags.nav_posllh = enabled,
        valset_keys::NAV_HPPOSECEF => flags.nav_hpposecef = enabled,
        valset_keys::NAV_DOP => flags.nav_dop = enabled,
        valset_keys::NAV_VELECEF => flags.nav_velecef = enabled,
        valset_keys::NAV_VELNED => flags.nav_velned = enabled,
        valset_keys::NAV_TIMEGPS => flags.nav_timegps = enabled,
        valset_keys::NAV_TIMEUTC => flags.nav_timeutc = enabled,
        valset_keys::NAV_TIMELS => flags.nav_timels = enabled,
        valset_keys::NAV_CLOCK => flags.nav_clock = enabled,
        valset_keys::NAV_AOPSTATUS => flags.nav_aopstatus = enabled,
        valset_keys::NAV_COV => flags.nav_cov = enabled,
        valset_keys::NAV_EOE => flags.nav_eoe = enabled,
        valset_keys::TIM_TP => flags.tim_tp = enabled,
        valset_keys::MON_HW => flags.mon_hw = enabled,
        valset_keys::RXM_RAWX => flags.rxm_rawx = enabled,
        valset_keys::MON_COMMS => flags.mon_comms = enabled,
        valset_keys::MON_RF => flags.mon_rf = enabled,
        _ => {}
    }
}

/// Process CFG-VALSET rate and configuration keys
fn process_valset_config_key(key: u32, val: u32) {
    match key {
        valset_keys::RATE_MEAS => {
            // Measurement period in ms (clamp to 50-10000ms)
            let period = (val as u16).clamp(50, 10000) as u32;
            NAV_MEAS_PERIOD_MS.store(period, Ordering::Release);
            info!("VALSET: NAV meas period changed to {}ms", period);
        }
        valset_keys::RATE_NAV => {
            // Navigation rate (cycles per solution, clamp to 1-127)
            let rate = (val as u8).clamp(1, 127) as u32;
            NAV_RATE.store(rate, Ordering::Release);
            info!("VALSET: NAV rate changed to {}", rate);
        }
        valset_keys::RATE_TIMEREF => {
            // Time reference (0=UTC, 1=GPS, etc.)
            NAV_TIMEREF.store(val as u8, Ordering::Release);
            info!("VALSET: Time ref changed to {}", val);
        }
        valset_keys::UART1_BAUDRATE => {
            // UART baudrate
            if val > 0 {
                BAUDRATE_CHANGE.signal(val);
                info!("VALSET: Baudrate change to {}", val);
            }
        }
        _ => {}
    }
}

/// Handle incoming UBX command
async fn handle_ubx_command(cmd: &ubx::UbxCommand) {
    // Record first config command time (starts 700ms countdown to NAV output)
    // All UBX commands from drone count as config commands
    record_first_config();

    match cmd {
        ubx::UbxCommand::CfgPrt { baudrate } => {
            info!("CFG-PRT: baudrate={}", baudrate);
            send_ack(0x06, 0x00); // ACK for CFG-PRT

            // Apply baudrate change (signal to uart_rx_task)
            if *baudrate > 0 {
                BAUDRATE_CHANGE.signal(*baudrate);
            }
        }
        ubx::UbxCommand::CfgMsg { class, id, rate } => {
            info!("CFG-MSG: class=0x{:02X} id=0x{:02X} rate={}", class, id, rate);
            send_ack(0x06, 0x01); // ACK for CFG-MSG

            // Update global message flags
            {
                let mut flags = MSG_FLAGS_STATE.lock().await;
                flags.set_message(*class, *id, *rate > 0);
            }
        }
        ubx::UbxCommand::CfgRate { meas_rate, nav_rate, time_ref } => {
            info!("CFG-RATE: meas_rate={}, nav_rate={}, time_ref={}", meas_rate, nav_rate, time_ref);
            send_ack(0x06, 0x08); // ACK for CFG-RATE

            // Update measurement period (clamp to 50-10000ms)
            let period = (*meas_rate).clamp(50, 10000) as u32;
            NAV_MEAS_PERIOD_MS.store(period, Ordering::Release);

            // Update navigation rate (clamp to 1-127)
            let rate = (*nav_rate).clamp(1, 127) as u32;
            NAV_RATE.store(rate, Ordering::Release);

            // Store time reference
            NAV_TIMEREF.store(*time_ref as u8, Ordering::Release);

            info!("NAV config: meas={}ms, rate={}, timeref={}", period, rate, time_ref);
        }
        ubx::UbxCommand::CfgCfg => {
            info!("CFG-CFG received");
            send_ack(0x06, 0x09); // ACK for CFG-CFG
        }
        ubx::UbxCommand::CfgRst { reset_mode } => {
            // CFG-RST with reset_mode:
            // 0x00 = Hardware reset (watchdog)
            // 0x01 = Controlled software reset (GNSS only)
            // 0x02 = Software reset (GNSS + peripherals)
            // 0x08 = GNSS stop
            // 0x09 = GNSS start
            info!("CFG-RST received, reset_mode=0x{:02X}", reset_mode);
            // 20s timer starts from NAV output start, not from CFG-RST
            // No ACK for CFG-RST (per u-blox spec - device would normally reboot)
        }
        ubx::UbxCommand::CfgNav5 => {
            info!("CFG-NAV5 received");
            send_ack(0x06, 0x24); // ACK for CFG-NAV5
        }
        ubx::UbxCommand::CfgNavx5 => {
            info!("CFG-NAVX5 received");
            send_ack(0x06, 0x23); // ACK for CFG-NAVX5
        }
        ubx::UbxCommand::CfgGnss => {
            info!("CFG-GNSS received");
            send_ack(0x06, 0x3E); // ACK for CFG-GNSS
        }
        ubx::UbxCommand::CfgSbas => {
            info!("CFG-SBAS received");
            send_ack(0x06, 0x16); // ACK for CFG-SBAS
        }
        ubx::UbxCommand::CfgItfm => {
            info!("CFG-ITFM received");
            send_ack(0x06, 0x39); // ACK for CFG-ITFM
        }
        ubx::UbxCommand::CfgPms => {
            info!("CFG-PMS received");
            send_ack(0x06, 0x86); // ACK for CFG-PMS
        }
        ubx::UbxCommand::CfgValget { keys } => {
            // CFG-VALGET before CFG-VALSET is a Mavic 4 pattern
            // Mark this for auto-detection
            SAW_CFG_VALGET.store(true, Ordering::Release);

            info!("CFG-VALGET received for {} keys", keys.len());
            let response = CfgValgetResponse::for_keys(keys);
            send_ubx_message(&response);
            send_ack(0x06, 0x8B);
            // NAV output now starts 700ms after first config command (handled in nav_message_task)
        }
        ubx::UbxCommand::CfgValset { _layer: _, keys } => {
            // Auto-detect drone model on first CFG-VALSET
            // If we haven't detected yet, decide based on what we saw before
            if !DRONE_DETECTED.load(Ordering::Acquire) {
                let saw_uniqid = SAW_SEC_UNIQID.load(Ordering::Acquire);
                let saw_valget = SAW_CFG_VALGET.load(Ordering::Acquire);

                if saw_uniqid || saw_valget {
                    // Mavic 4 pattern: SEC-UNIQID or CFG-VALGET before CFG-VALSET
                    DRONE_MODEL.store(1, Ordering::Release); // Mavic 4 Pro
                    info!("AUTO-DETECT: Mavic 4 Pro (saw SEC-UNIQID={}, CFG-VALGET={})", saw_uniqid, saw_valget);
                } else {
                    // Air 3 pattern: CFG-VALSET immediately after MON-VER
                    DRONE_MODEL.store(0, Ordering::Release); // Air 3
                    info!("AUTO-DETECT: Air 3 (no SEC-UNIQID/CFG-VALGET before CFG-VALSET)");
                }
                DRONE_DETECTED.store(true, Ordering::Release);
            }

            info!("CFG-VALSET received with {} keys", keys.len());

            // Process all keys (message output starts on CFG-RST)
            {
                let mut flags = MSG_FLAGS_STATE.lock().await;
                for &(key, val) in keys.iter() {
                    // Update message flags for MSGOUT keys
                    update_flag_from_valset_key(&mut flags, key, val);
                    // Process rate and config keys (baudrate, measurement period)
                    process_valset_config_key(key, val);
                }
            }

            send_ack(0x06, 0x8A); // ACK for CFG-VALSET
        }
        ubx::UbxCommand::MonVerPoll => {
            info!("MON-VER poll received, sending version info");
            let ver = MonVer::default();
            send_ubx_message(&ver);
        }
        ubx::UbxCommand::SecUniqidPoll => {
            // SEC-UNIQID poll is sent by Mavic 4 before CFG-RST
            // Mark this for auto-detection (but don't change model here)
            SAW_SEC_UNIQID.store(true, Ordering::Release);

            let model = DroneModel::from_u8(DRONE_MODEL.load(Ordering::Acquire));
            info!("SEC-UNIQID poll received, sending unique ID for {:?}", model);
            let uniqid = SecUniqid::for_model(model);
            send_ubx_message(&uniqid);
        }
        ubx::UbxCommand::Cfg41Poll => {
            // DJI proprietary command - returns SEC-SIGN private key and configuration
            let model = DroneModel::from_u8(DRONE_MODEL.load(Ordering::Acquire));
            info!("CFG-0x41 poll received, sending SEC-SIGN config for {:?}", model);
            let cfg41 = Cfg41::for_model(model);
            send_ubx_message(&cfg41);
            send_ack(0x06, 0x41);
        }
        ubx::UbxCommand::Mga { id } => {
            // MGA-* messages: AssistNow assistance data upload
            // Real u-blox modules ACK these messages
            debug!("MGA message received: id=0x{:02X}", id);
            send_ack(0x13, *id);
        }
        ubx::UbxCommand::Poll { class, id } => {
            info!("Unhandled poll: class=0x{:02X} id=0x{:02X}", class, id);
        }
        ubx::UbxCommand::Unknown => {
            warn!("Unknown UBX command");
        }
    }
}

/// NAV message task - sends navigation messages at configurable rate
/// Starts 700ms after first config command from drone
/// Uses Timer::at() for precise timing without drift
#[embassy_executor::task]
async fn nav_message_task() {
    use embassy_time::Instant;

    info!("NAV message task waiting for first config command...");

    // Wait for first config command (FIRST_CONFIG_MILLIS != 0)
    while FIRST_CONFIG_MILLIS.load(Ordering::Acquire) == 0 {
        Timer::after(Duration::from_millis(10)).await;
    }

    // Get model-specific delay
    let model = DroneModel::from_u8(DRONE_MODEL.load(Ordering::Acquire));
    let config_to_nav_delay = match model {
        DroneModel::Air3 => config::timers::CONFIG_TO_NAV_AIR3_MS,
        DroneModel::Mavic4Pro => config::timers::CONFIG_TO_NAV_MAVIC4_MS,
    };

    // Wait remaining time to reach delay after first config
    let first_config_time = FIRST_CONFIG_MILLIS.load(Ordering::Acquire);
    let now = Instant::now().as_millis() as u32;
    let elapsed = now.wrapping_sub(first_config_time) as u64;
    if elapsed < config_to_nav_delay {
        let remaining = config_to_nav_delay - elapsed;
        info!("Waiting {}ms more for NAV start ({}ms from first config)", remaining, config_to_nav_delay);
        Timer::after(Duration::from_millis(remaining)).await;
    }

    // Set MSG_OUTPUT_STARTED flag (for other tasks like SEC-SIGN)
    MSG_OUTPUT_STARTED.store(true, Ordering::Release);
    // Record start time for 20s invalid satellites timer
    OUTPUT_START_MILLIS.store(Instant::now().as_millis() as u32, Ordering::Release);
    SATELLITES_INVALID.store(false, Ordering::Release);
    info!("NAV message output started ({}ms after first config)", config_to_nav_delay);

    // Use absolute timestamps to prevent timing drift
    // Timer::after() would cause drift because it waits AFTER processing completes
    let mut next_tick = Instant::now();

    loop {
        // Calculate effective period = meas_period * nav_rate
        let meas_period = NAV_MEAS_PERIOD_MS.load(Ordering::Acquire);
        let nav_rate = NAV_RATE.load(Ordering::Acquire);
        let effective_period = meas_period * nav_rate;

        // Schedule next tick at absolute time (no drift accumulation)
        next_tick += Duration::from_millis(effective_period as u64);
        Timer::at(next_tick).await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        // Wait for SEC-SIGN computation to complete using async Signal
        // This is more efficient than busy-wait yield_now loop
        if SEC_SIGN_IN_PROGRESS.load(Ordering::Acquire) {
            let _ = SEC_SIGN_DONE.receive().await;
        }

        // Get current message flags
        let flags = {
            let flags = MSG_FLAGS_STATE.lock().await;
            *flags
        };

        // Get system time - all time fields derived from same source
        let now = embassy_time::Instant::now();
        let itow = (now.as_millis() % (7 * 24 * 3600 * 1000)) as u32; // ms in week
        let total_secs = (now.as_millis() / 1000) as u32;
        let sec = (total_secs % 60) as u8;
        let min = ((total_secs / 60) % 60) as u8;
        let hour = ((total_secs / 3600) % 24) as u8;

        // Check if 20 seconds have passed since output started (satellites become invalid)
        // Using wrapping_sub to handle u32 overflow correctly (~49 days)
        let start_time = OUTPUT_START_MILLIS.load(Ordering::Acquire);
        let elapsed_ms = (now.as_millis() as u32).wrapping_sub(start_time);
        let satellites_invalid = elapsed_ms >= config::timers::SATELLITES_INVALID_AFTER_MS as u32;

        // Update global flag for LED indication (Core1)
        SATELLITES_INVALID.store(satellites_invalid, Ordering::Release);

        // Build and send enabled NAV messages using send_msg! macro
        let mut buf = [0u8; 256];

        // Helper macro for sending message with UbxMessage trait
        macro_rules! send_ubx {
            ($flag:expr, $msg:expr) => {
                if $flag {
                    let mut buf = [0u8; 1280];
                    let len = $msg.build(&mut buf);
                    if len > 0 {
                        let mut vec = heapless::Vec::<u8, 1280>::new();
                        let _ = vec.extend_from_slice(&buf[..len]);
                        if TX_CHANNEL.try_send(vec).is_err() {
                            warn!("TX channel full, dropped message");
                        }
                    }
                }
            };
        }

        // NAV-PVT (0x01 0x07) - with fix status
        if satellites_invalid {
            send_ubx!(flags.nav_pvt, NavPvt::invalid(itow, hour, min, sec));
        } else {
            send_msg!(buf, flags.nav_pvt, NavPvt, { itow: itow, hour: hour, min: min, sec: sec });
        }

        // NAV-POSECEF (0x01 0x01) - uses Default ECEF from C version
        send_msg!(buf, flags.nav_posecef, NavPosecef, { itow: itow });

        // NAV-POSLLH (0x01 0x02) - uses Default LLH from C version
        send_msg!(buf, flags.nav_posllh, NavPosllh, { itow: itow });

        // NAV-STATUS (0x01 0x03) - with fix status
        if satellites_invalid {
            send_ubx!(flags.nav_status, NavStatus::invalid(itow, itow));
        } else {
            send_msg!(buf, flags.nav_status, NavStatus, { itow: itow, msss: itow });
        }

        // NAV-DOP (0x01 0x04)
        send_msg!(buf, flags.nav_dop, NavDop, { itow: itow });

        // NAV-SOL (0x01 0x06) - legacy but important for many FCs, with fix status
        if satellites_invalid {
            send_ubx!(flags.nav_sol, NavSol::invalid(itow));
        } else {
            send_msg!(buf, flags.nav_sol, NavSol, { itow: itow });
        }

        // NAV-VELECEF (0x01 0x11)
        send_msg!(buf, flags.nav_velecef, NavVelecef, { itow: itow });

        // NAV-VELNED (0x01 0x12)
        send_msg!(buf, flags.nav_velned, NavVelned, { itow: itow });

        // NAV-HPPOSECEF (0x01 0x13) - uses Default ECEF from C version
        send_msg!(buf, flags.nav_hpposecef, NavHpposecef, { itow: itow });

        // NAV-TIMEGPS (0x01 0x20)
        send_msg!(buf, flags.nav_timegps, NavTimegps, { itow: itow });

        // NAV-TIMEUTC (0x01 0x21)
        send_msg!(buf, flags.nav_timeutc, NavTimeutc, { itow: itow, hour: hour, min: min, sec: sec });

        // NAV-CLOCK (0x01 0x22)
        send_msg!(buf, flags.nav_clock, NavClock, { itow: itow });

        // NAV-TIMELS (0x01 0x26)
        send_msg!(buf, flags.nav_timels, NavTimels, { itow: itow });

        // NAV-SVINFO (0x01 0x30) - legacy format, with satellite info
        if satellites_invalid {
            send_ubx!(flags.nav_svinfo, NavSvinfo::invalid(itow));
        } else {
            send_msg!(buf, flags.nav_svinfo, NavSvinfo, { itow: itow });
        }

        // NAV-SAT (0x01 0x35) - M10 format, with satellite info
        if satellites_invalid {
            send_ubx!(flags.nav_sat, NavSat::invalid(itow));
        } else {
            send_msg!(buf, flags.nav_sat, NavSat, { itow: itow });
        }

        // NAV-COV (0x01 0x36)
        send_msg!(buf, flags.nav_cov, NavCov, { itow: itow });

        // NAV-AOPSTATUS (0x01 0x60)
        send_msg!(buf, flags.nav_aopstatus, NavAopstatus, { itow: itow });

        // TIM-TP (0x0D 0x01)
        send_msg!(buf, flags.tim_tp, TimTp, { tow_ms: itow });

        // RXM-RAWX (0x02 0x15)
        send_msg!(buf, flags.rxm_rawx, RxmRawx, { rcv_tow: itow as f64 / 1000.0, week: 2349, leap_s: 18 });

        // NAV-EOE (0x01 0x61) - End of Epoch - always last
        send_msg!(buf, flags.nav_eoe, NavEoe, { itow: itow });
    }
}

/// MON message task - sends monitoring messages at configured rate when enabled
#[embassy_executor::task]
async fn mon_message_task() {
    // Wait for CFG-RST with reset_mode=0x09
    while !MSG_OUTPUT_STARTED.load(Ordering::Acquire) {
        Timer::after(Duration::from_millis(10)).await;
    }
    Timer::after(Duration::from_millis(config::timers::UART_TX_INIT_DELAY_MS)).await;
    info!("MON message task started");

    let mut ticker = Ticker::every(Duration::from_millis(config::timers::MON_PERIOD_MS));
    let mut buf = [0u8; 128];

    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        // Wait for SEC-SIGN computation to complete using async Signal
        // This is more efficient than busy-wait yield_now loop
        if SEC_SIGN_IN_PROGRESS.load(Ordering::Acquire) {
            let _ = SEC_SIGN_DONE.receive().await;
        }

        // Get current message flags
        let flags = {
            let flags = MSG_FLAGS_STATE.lock().await;
            *flags
        };

        // MON-HW (0x0A 0x09)
        send_msg!(buf, flags.mon_hw, MonHw);

        // MON-COMMS (0x0A 0x36)
        send_msg!(buf, flags.mon_comms, MonComms);

        // MON-RF (0x0A 0x38)
        send_msg!(buf, flags.mon_rf, MonRf);
    }
}

/// SEC-SIGN timer task - requests signature at configured interval
/// Only starts after message output begins (like C version)
/// Period depends on drone model: Air3=4s, Mavic4Pro=2s
#[embassy_executor::task]
async fn sec_sign_timer_task() {
    let session_id = DEFAULT_SESSION_ID;

    // Wait for message output to start
    // In Passthrough mode, don't wait for MSG_OUTPUT_STARTED - it's set by nav_message_task
    // which requires first UBX command from host. In Passthrough we start immediately.
    let start_time = embassy_time::Instant::now();
    loop {
        let mode = OperatingMode::load();
        if mode == OperatingMode::Passthrough {
            // In Passthrough, wait a fixed delay then start
            if start_time.elapsed().as_millis() >= 2000 {
                info!("SEC-SIGN timer: Passthrough mode, starting after 2s delay");
                break;
            }
        } else if mode == OperatingMode::PassthroughRaw {
            // Raw mode doesn't use SEC-SIGN, but we still need to wait
            // until mode changes or MSG_OUTPUT_STARTED
        } else {
            // Emulation mode - wait for MSG_OUTPUT_STARTED
            if MSG_OUTPUT_STARTED.load(Ordering::Acquire) {
                break;
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    // Wait for drone model auto-detection to complete (with timeout)
    // If detection doesn't happen within 500ms, use default model (Air 3)
    let detect_start = embassy_time::Instant::now();
    while !DRONE_DETECTED.load(Ordering::Acquire) {
        if detect_start.elapsed().as_millis() > 500 {
            info!("Auto-detect timeout, using default model (Air 3)");
            break;
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    // Read model once after detection is complete
    let model = DroneModel::from_u8(DRONE_MODEL.load(Ordering::Acquire));
    let (first_delay_ms, period_ms) = match model {
        DroneModel::Air3 => (
            config::timers::SEC_SIGN_FIRST_AIR3_MS,
            config::timers::SEC_SIGN_PERIOD_AIR3_MS,
        ),
        DroneModel::Mavic4Pro => (
            config::timers::SEC_SIGN_FIRST_MAVIC4_MS,
            config::timers::SEC_SIGN_PERIOD_MAVIC4_MS,
        ),
    };
    info!("SEC-SIGN timer task started (first={}ms, period={}ms for {:?})", first_delay_ms, period_ms, model);

    // First signature after model-specific delay from NAV start
    Timer::after(Duration::from_millis(first_delay_ms)).await;

    // Then at configured interval
    let mut ticker = Ticker::every(Duration::from_millis(period_ms));
    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode == OperatingMode::PassthroughRaw {
            continue;  // SEC-SIGN timer skipped in Raw mode
        }
        // In Passthrough without spoofing, original SEC-SIGN passes through - no need to generate
        if mode == OperatingMode::Passthrough && !SPOOF_DETECTED.load(Ordering::Acquire) {
            continue;
        }
        // Works for Emulation and Passthrough (with spoofing only)

        // CRITICAL: Pause TX before capturing hash to prevent race condition
        // Any packets sent after hash capture but before SEC-SIGN TX would not
        // be included in the hash -> verification fails on receiver side
        SEC_SIGN_IN_PROGRESS.store(true, Ordering::Release);

        // Get hash and count from global accumulator, reset it
        // Must capture BOTH atomically - packet count at same moment as hash
        let (hash, count) = {
            let mut acc = SEC_SIGN_ACC.lock().await;
            if let Some(ref mut accumulator) = *acc {
                accumulator.finalize_and_reset()
            } else {
                ([0u8; 32], 0)
            }
        };

        // Request computation from Core1
        let request = SecSignRequest {
            sha256_hash: hash,
            session_id,
            packet_count: count,
        };

        let _ = SEC_SIGN_REQUEST.try_send(request);
        info!("SEC-SIGN requested for {} packets (TX paused)", count);
    }
}

/// Mode button task - hot-switches mode without reboot
/// Выбор режима по количеству коротких нажатий:
/// 1 нажатие → Emulation (зелёный), 2 → Passthrough (синий), 3 → PassthroughRaw (фиолетовый)
/// Использует RP2350-E9 Errata Workaround (Input Latch-up fix)
#[embassy_executor::task]
async fn button_task(mut flex: Flex<'static>, flash_mutex: &'static FlashMutex) {
    use config::button::*;
    use embassy_time::Instant;

    // Инициализация GPIO
    flex.set_as_input();
    flex.set_pull(Pull::Down);
    
    // Ждём 1с при старте для игнорирования power-on noise
    Timer::after(Duration::from_millis(1000)).await;

    let mut click_count: u8 = 0;
    let mut last_click_time: Option<Instant> = None;
    let mut was_high = false;

    loop {
        Timer::after(Duration::from_millis(POLL_PERIOD_MS)).await;
        
        let is_high = flex.is_high();

        // Проверка таймаута серии нажатий
        if click_count > 0 {
            if let Some(last_time) = last_click_time {
                if last_time.elapsed().as_millis() >= MULTI_CLICK_TIMEOUT_MS {
                    // Таймаут - применяем режим по количеству нажатий
                    apply_mode_by_clicks(click_count, flash_mutex).await;
                    click_count = 0;
                    last_click_time = None;
                }
            }
        }

        // Обнаружение нажатия (0 → 1)
        if is_high && !was_high {
            info!("button_task: edge detected (0->1), debouncing");
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
            
            if flex.is_high() {
                click_count = click_count.saturating_add(1).min(3);
                info!("button_task: click #{}", click_count);

                // Ждём отпускания кнопки с E9 workaround и защитой от зависания
                info!("button_task: waiting for release...");
                let mut high_count: u32 = 0;
                const MAX_RELEASE_WAIT: u32 = 40; // 40 * 50ms = 2 секунды максимум
                loop {
                    if !flex.is_high() {
                        info!("button_task: released!");
                        break;
                    }
                    Timer::after(Duration::from_millis(50)).await;
                    high_count += 1;

                    // Защита от зависания - максимум 2 секунды ожидания
                    if high_count >= MAX_RELEASE_WAIT {
                        warn!("button_task: release wait timeout (2s), forcing exit");
                        break;
                    }

                    // E9 discharge kick каждые ~250мс (5 * 50мс)
                    if high_count % 5 == 0 {
                        info!("button_task: still HIGH (2.4V latch?), attempting E9 discharge kick...");
                        flex.set_as_output();
                        flex.set_low();
                        Timer::after(Duration::from_micros(50)).await;
                        flex.set_as_input();
                        flex.set_pull(Pull::Down);
                        Timer::after(Duration::from_micros(50)).await;
                    }
                }

                // Таймаут отсчитываем от момента ОТПУСКАНИЯ кнопки,
                // чтобы E9 workaround не съедал время на следующий клик
                last_click_time = Some(Instant::now());
            }
        }
        was_high = is_high;
    }
}

/// Применяет режим по количеству нажатий кнопки
async fn apply_mode_by_clicks(click_count: u8, flash_mutex: &'static FlashMutex) {
    let new_mode = match click_count {
        1 => OperatingMode::Emulation,
        2 => OperatingMode::Passthrough,
        3 => OperatingMode::PassthroughRaw,
        _ => return, // 0 нажатий - нечего делать
    };

    let current = OperatingMode::load();
    if current == new_mode {
        info!("Mode unchanged: {:?} (clicks={})", new_mode, click_count);
        return;
    }

    // Применяем режим
    new_mode.store();
    info!("HOT-SWITCH: {:?} -> {:?} (clicks={})", current, new_mode, click_count);

    // Reset таймера спутников при переключении в Emulation
    if new_mode == OperatingMode::Emulation {
        OUTPUT_START_MILLIS.store(embassy_time::Instant::now().as_millis() as u32, Ordering::Release);
        SATELLITES_INVALID.store(false, Ordering::Release);
        info!("Reset satellite validity timer (mode switch)");
    }

    // Сброс флагов спуфинга при переключении в PassthroughRaw
    if new_mode == OperatingMode::PassthroughRaw {
        SPOOF_DETECTED.store(false, Ordering::Release);
        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
        info!("Cleared spoof detection state (entering raw passthrough)");
    }

    // Сброс спуф-детектора при переключении в Passthrough
    // Это предотвращает ложную детекцию "телепорта" при переходе из Emulation
    // (координаты эмуляции отличаются от реальных GNSS координат)
    if new_mode == OperatingMode::Passthrough {
        SPOOF_DETECTOR_RESET.store(true, Ordering::Release);
        SPOOF_DETECTED.store(false, Ordering::Release);
        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
        info!("SPOOF_DETECTOR_RESET set (entering passthrough, prevents false teleport)");
    }

    // Сохраняем во flash
    {
        let mut flash = flash_mutex.lock().await;
        if flash_storage::save_mode(&mut flash, new_mode as u8).await {
            info!("Mode saved to flash");
        } else {
            warn!("Failed to save mode to flash");
        }
    }
}
