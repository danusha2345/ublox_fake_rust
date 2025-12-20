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
mod flash_storage;
mod led;
mod passthrough;
mod sec_sign;
mod ubx;

use core::sync::atomic::{AtomicU8, AtomicU32, AtomicPtr, Ordering};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::{FLASH, PIO0, PIO1, UART0};
use embassy_rp::pio::Pio;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use embedded_io_async::{Read, Write};
use cortex_m::peripheral::SCB;
use panic_probe as _;
use static_cell::StaticCell;

use config::*;
use sec_sign::{SecSignAccumulator, SecSignRequest, SecSignResult, Signature, DEFAULT_SESSION_ID, PRIVATE_KEY};
use ubx::{
    AckAck, MessageFlags, MonVer, NavDop, NavEoe, NavPvt, NavStatus, SecSign, SecUniqid, UbxMessage,
    // Additional NAV messages
    NavPosecef, NavPosllh, NavVelned, NavVelecef, NavTimeutc, NavTimegps, NavClock, NavTimels,
    NavAopstatus, NavCov, NavHpposecef, NavSat, NavSvinfo,
    // MON messages
    MonHw, MonRf, MonComms,
    // TIM and RXM messages
    TimTp, RxmRawx,
};

// Interrupt bindings
bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

// ============================================================================
// Inter-core communication
// ============================================================================

/// Channel for outgoing UBX messages (serialized, ready to send)
static TX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 256>, 16> = Channel::new();

/// Global message enable flags
static MSG_FLAGS_STATE: Mutex<CriticalSectionRawMutex, MessageFlags> = Mutex::new(MessageFlags::new_default());

/// Message output started flag (set after first CFG-MSG + 1 sec delay)
static MSG_OUTPUT_STARTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Global SEC-SIGN accumulator (accumulates sent messages)
static SEC_SIGN_ACC: Mutex<CriticalSectionRawMutex, Option<SecSignAccumulator>> = Mutex::new(None);

/// Request SEC-SIGN computation (Core0 -> Core1)
static SEC_SIGN_REQUEST: Signal<CriticalSectionRawMutex, SecSignRequest> = Signal::new();

/// SEC-SIGN result ready (Core1 -> Core0)
static SEC_SIGN_RESULT: Signal<CriticalSectionRawMutex, SecSignResult> = Signal::new();

/// Current operating mode (0 = Emulation, 1 = Passthrough)
static MODE: AtomicU8 = AtomicU8::new(0);

/// NAV message measurement period in milliseconds (default 200ms)
static NAV_MEAS_PERIOD_MS: AtomicU32 = AtomicU32::new(200);

/// NAV message rate (cycles per navigation solution, default 1)
static NAV_RATE: AtomicU32 = AtomicU32::new(1);

/// Time reference (0=UTC, 1=GPS, 2=GLONASS, etc.) - stored but not used
static NAV_TIMEREF: AtomicU8 = AtomicU8::new(0);

/// Signal for baudrate change (value = new baudrate)
static BAUDRATE_CHANGE: Signal<CriticalSectionRawMutex, u32> = Signal::new();

/// Flash for mode persistence
static FLASH_CELL: StaticCell<Mutex<CriticalSectionRawMutex, Flash<'static, FLASH, Async, { 2 * 1024 * 1024 }>>> = StaticCell::new();

/// Pointer to flash mutex (set after FLASH_CELL.init())
type FlashMutex = Mutex<CriticalSectionRawMutex, Flash<'static, FLASH, Async, { 2 * 1024 * 1024 }>>;
static FLASH_PTR: AtomicPtr<FlashMutex> = AtomicPtr::new(core::ptr::null_mut());

/// Core1 stack
static mut CORE1_STACK: Stack<4096> = Stack::new();

// ============================================================================
// Operating Mode
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum OperatingMode {
    Emulation = 0,
    Passthrough = 1,
}

impl OperatingMode {
    fn load() -> Self {
        match MODE.load(Ordering::Acquire) {
            0 => Self::Emulation,
            _ => Self::Passthrough,
        }
    }

    fn store(self) {
        MODE.store(self as u8, Ordering::Release);
    }
}

impl Default for OperatingMode {
    fn default() -> Self {
        Self::Emulation
    }
}

// ============================================================================
// Core0 Entry Point
// ============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("u-blox GNSS Emulator starting on Core0...");

    let p = embassy_rp::init(Default::default());

    // Initialize flash for mode persistence
    let flash = Flash::<_, Async, { 2 * 1024 * 1024 }>::new(p.FLASH, p.DMA_CH0);
    let flash_mutex = FLASH_CELL.init(Mutex::new(flash));
    // Store pointer for global access in button_task
    FLASH_PTR.store(flash_mutex as *const _ as *mut _, Ordering::Release);

    // Load saved mode from flash
    let is_passthrough = {
        let mut flash = flash_mutex.lock().await;
        if let Some(saved_mode) = flash_storage::load_mode(&mut flash) {
            if saved_mode == 1 {
                info!("Loaded passthrough mode from flash");
                OperatingMode::Passthrough.store();
                true
            } else {
                info!("Loaded emulation mode from flash");
                OperatingMode::Emulation.store();
                false
            }
        } else {
            info!("No saved mode, defaulting to emulation");
            OperatingMode::Emulation.store();
            false
        }
    };

    // Initialize PIO0 for WS2812 LED
    let pio0 = Pio::new(p.PIO0, Irqs);

    // Mode button (GPIO5 = power, GPIO6 = input)
    let _btn_pwr = Output::new(p.PIN_5, Level::High);
    let btn_input = Input::new(p.PIN_6, Pull::Down);

    // Spawn Core1 for LED and SEC-SIGN computation
    info!("Spawning Core1...");
    static EXECUTOR1: StaticCell<embassy_executor::Executor> = StaticCell::new();
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(embassy_executor::Executor::new());
            executor1.run(|spawner: embassy_executor::Spawner| {
                spawner.must_spawn(led_task(pio0, p.PIN_16));
                spawner.must_spawn(sec_sign_compute_task());
            });
        },
    );

    if is_passthrough {
        // Passthrough mode: PIO copies GPIO3 -> GPIO0
        info!("Starting in PASSTHROUGH mode");

        let mut pio1 = Pio::new(p.PIO1, Irqs);
        let mut passthrough = passthrough::Passthrough::new(
            &mut pio1.common,
            pio1.sm0,
            p.PIN_3,  // Input from external GNSS
            p.PIN_0,  // Output to host
        );
        passthrough.enable();

        // Prevent drop - keep PIO running (sm0 is owned by passthrough)
        core::mem::forget(passthrough);

        // Only button task in passthrough mode
        spawner.must_spawn(button_task(btn_input));

        info!("Passthrough active, press button to switch to emulation");
    } else {
        // Emulation mode: UART for UBX messages
        info!("Starting in EMULATION mode");

        static TX_BUF: StaticCell<[u8; 512]> = StaticCell::new();
        static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        let tx_buf = &mut TX_BUF.init([0; 512])[..];
        let rx_buf = &mut RX_BUF.init([0; 256])[..];

        let mut uart_config = UartConfig::default();
        uart_config.baudrate = DEFAULT_BAUDRATE;
        let uart = BufferedUart::new(
            p.UART0,
            p.PIN_0, // TX
            p.PIN_1, // RX
            Irqs,
            tx_buf,
            rx_buf,
            uart_config,
        );
        let (tx, rx) = uart.split();

        // All Core0 tasks for emulation
        spawner.must_spawn(uart_tx_task(tx));
        spawner.must_spawn(uart_rx_task(rx));
        spawner.must_spawn(nav_message_task());
        spawner.must_spawn(mon_message_task());
        spawner.must_spawn(sec_sign_timer_task());
        spawner.must_spawn(button_task(btn_input));
    }

    info!("All tasks spawned, running");

    // Core0 main loop - idle
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

// ============================================================================
// Core1 Tasks
// ============================================================================

/// LED control task - WS2812 on PIO (runs on Core1)
#[embassy_executor::task]
async fn led_task(
    mut pio: Pio<'static, PIO0>,
    pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_16>,
) {
    use led::Ws2812;

    info!("LED task starting on Core1");
    let mut ws2812 = Ws2812::new(&mut pio.common, pio.sm0, pin);
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut led_on = false;

    loop {
        let mode = OperatingMode::load();
        let color = match mode {
            OperatingMode::Emulation => led::Color::green(),
            OperatingMode::Passthrough => led::Color::blue(),
        };

        led_on = !led_on;
        if led_on {
            ws2812.write_color(color).await;
        } else {
            ws2812.write_color(led::Color::off()).await;
        }

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
        let request = SEC_SIGN_REQUEST.wait().await;

        info!("Computing SEC-SIGN for {} packets", request.packet_count);

        // Compute z = fold(SHA256(hash || session_id))
        let z = sec_sign::compute_z(&request.sha256_hash, &request.session_id);

        // Sign with ECDSA SECP192R1
        let signature = Signature::sign(&z, &PRIVATE_KEY).unwrap_or_default();

        // Send result back to Core0
        let result = SecSignResult {
            sha256_hash: request.sha256_hash,
            session_id: request.session_id,
            signature,
            packet_count: request.packet_count,
        };

        SEC_SIGN_RESULT.signal(result);
        info!("SEC-SIGN computation complete");
    }
}

// ============================================================================
// Core0 Tasks
// ============================================================================

/// UART TX task - sends UBX messages from TX_CHANNEL
#[embassy_executor::task]
async fn uart_tx_task(mut tx: embassy_rp::uart::BufferedUartTx) {
    // Wait for initial delay
    Timer::after(Duration::from_secs(1)).await;

    // Initialize global accumulator
    {
        let mut acc = SEC_SIGN_ACC.lock().await;
        *acc = Some(SecSignAccumulator::new());
    }

    info!("UART TX task ready");

    loop {
        // Wait for message from channel
        let msg = TX_CHANNEL.receive().await;

        // Send via UART
        if let Err(e) = tx.write_all(&msg).await {
            error!("UART TX error: {:?}", e);
            continue;
        }

        // Accumulate for SEC-SIGN (except SEC-SIGN messages themselves)
        // SEC-SIGN class = 0x27
        if msg.len() >= 3 && msg[2] != 0x27 {
            let mut acc = SEC_SIGN_ACC.lock().await;
            if let Some(ref mut accumulator) = *acc {
                accumulator.accumulate(&msg);
            }
        }

        // Check if SEC-SIGN result is ready
        if let Some(result) = SEC_SIGN_RESULT.try_take() {
            // Build and send SEC-SIGN message
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
                info!("SEC-SIGN message sent ({} packets)", result.packet_count);
            }
        }
    }
}

/// Set UART0 baudrate using direct register access
fn set_uart0_baudrate(baudrate: u32) {
    let clk_peri = embassy_rp::clocks::clk_peri_freq();
    let baud_rate_div = (8 * clk_peri) / baudrate;
    let mut baud_ibrd = baud_rate_div >> 7;
    let mut baud_fbrd = ((baud_rate_div & 0x7f) + 1) / 2;

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

/// UART RX task - receives and processes UBX commands
#[embassy_executor::task]
async fn uart_rx_task(mut rx: embassy_rp::uart::BufferedUartRx) {
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
                error!("UART RX error: {:?}", e);
            }
        }
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
        let _ = TX_CHANNEL.try_send(vec);
    }
}

/// Send a UBX message
fn send_ubx_message<M: UbxMessage>(msg: &M) {
    let mut buf = [0u8; 256];
    let len = msg.build(&mut buf);
    if len > 0 {
        let mut vec = heapless::Vec::new();
        let _ = vec.extend_from_slice(&buf[..len]);
        let _ = TX_CHANNEL.try_send(vec);
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
            let should_start = {
                let mut flags = MSG_FLAGS_STATE.lock().await;
                let was_empty = !flags.any_enabled();
                flags.set_message(*class, *id, *rate > 0);
                // Start message output if this is the first enabled message
                was_empty && flags.any_enabled()
            };

            if should_start {
                info!("First message enabled, starting output in 1 second...");
                MSG_OUTPUT_STARTED.signal(());
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
        ubx::UbxCommand::CfgPms => {
            info!("CFG-PMS received");
            send_ack(0x06, 0x86); // ACK for CFG-PMS
        }
        ubx::UbxCommand::CfgValset { layer: _, keys } => {
            info!("CFG-VALSET received with {} keys", keys.len());

            // Process all keys
            let should_start = {
                let mut flags = MSG_FLAGS_STATE.lock().await;
                let was_empty = !flags.any_enabled();

                for &(key, val) in keys.iter() {
                    // Update message flags for MSGOUT keys
                    update_flag_from_valset_key(&mut flags, key, val);
                    // Process rate and config keys (baudrate, measurement period)
                    process_valset_config_key(key, val);
                }

                // Start message output if this is the first enabled message
                was_empty && flags.any_enabled()
            };

            if should_start {
                info!("First message enabled via VALSET, starting output in 1 second...");
                MSG_OUTPUT_STARTED.signal(());
            }

            send_ack(0x06, 0x8A); // ACK for CFG-VALSET
        }
        ubx::UbxCommand::MonVerPoll => {
            info!("MON-VER poll received, sending version info");
            let ver = MonVer::default();
            send_ubx_message(&ver);
        }
        ubx::UbxCommand::SecUniqidPoll => {
            info!("SEC-UNIQID poll received, sending unique ID");
            let uniqid = SecUniqid::default();
            send_ubx_message(&uniqid);
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
/// Only starts after receiving first CFG-MSG command + 1 second delay
#[embassy_executor::task]
async fn nav_message_task() {
    info!("NAV message task waiting for CFG-MSG...");

    // Wait for first CFG-MSG to enable messages
    MSG_OUTPUT_STARTED.wait().await;

    // 1 second delay after first CFG-MSG (like C version)
    Timer::after(Duration::from_secs(1)).await;
    info!("NAV message output started");

    loop {
        // Calculate effective period = meas_period * nav_rate
        let meas_period = NAV_MEAS_PERIOD_MS.load(Ordering::Acquire);
        let nav_rate = NAV_RATE.load(Ordering::Acquire);
        let effective_period = meas_period * nav_rate;
        Timer::after(Duration::from_millis(effective_period as u64)).await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
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

        // Build and send enabled NAV messages
        // Use 256-byte buffer for larger messages like NAV-SAT
        let mut buf = [0u8; 256];

        // NAV-PVT (0x01 0x07)
        if flags.nav_pvt {
            let mut msg = NavPvt::default();
            msg.itow = itow;
            msg.hour = hour;
            msg.min = min;
            msg.sec = sec;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-POSECEF (0x01 0x01)
        if flags.nav_posecef {
            let mut msg = NavPosecef::default();
            msg.itow = itow;
            // ECEF coordinates for Moscow (approximately)
            msg.ecef_x = 278394700;  // cm
            msg.ecef_y = 182089200;
            msg.ecef_z = 523478500;
            msg.p_acc = 5000;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-POSLLH (0x01 0x02)
        if flags.nav_posllh {
            let mut msg = NavPosllh::default();
            msg.itow = itow;
            msg.lon = 376184230;   // 37.618423 * 1e7
            msg.lat = 557611990;   // 55.761199 * 1e7
            msg.height = 156000;   // mm
            msg.h_msl = 156000;
            msg.h_acc = 5000;
            msg.v_acc = 8000;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-STATUS (0x01 0x03)
        if flags.nav_status {
            let mut msg = NavStatus::default();
            msg.itow = itow;
            msg.msss = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-DOP (0x01 0x04)
        if flags.nav_dop {
            let mut msg = NavDop::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-VELECEF (0x01 0x11)
        if flags.nav_velecef {
            let mut msg = NavVelecef::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-VELNED (0x01 0x12)
        if flags.nav_velned {
            let mut msg = NavVelned::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-HPPOSECEF (0x01 0x13)
        if flags.nav_hpposecef {
            let mut msg = NavHpposecef::default();
            msg.itow = itow;
            msg.ecef_x = 278394700;
            msg.ecef_y = 182089200;
            msg.ecef_z = 523478500;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-TIMEGPS (0x01 0x20)
        if flags.nav_timegps {
            let mut msg = NavTimegps::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-TIMEUTC (0x01 0x21)
        if flags.nav_timeutc {
            let mut msg = NavTimeutc::default();
            msg.itow = itow;
            msg.hour = hour;
            msg.min = min;
            msg.sec = sec;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-CLOCK (0x01 0x22)
        if flags.nav_clock {
            let mut msg = NavClock::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-TIMELS (0x01 0x26)
        if flags.nav_timels {
            let mut msg = NavTimels::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-SVINFO (0x01 0x30) - legacy format
        if flags.nav_svinfo {
            let mut msg = NavSvinfo::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-SAT (0x01 0x35) - M10 format
        if flags.nav_sat {
            let mut msg = NavSat::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-COV (0x01 0x36)
        if flags.nav_cov {
            let mut msg = NavCov::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-AOPSTATUS (0x01 0x60)
        if flags.nav_aopstatus {
            let mut msg = NavAopstatus::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // TIM-TP (0x0D 0x01)
        if flags.tim_tp {
            let mut msg = TimTp::default();
            msg.tow_ms = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // RXM-RAWX (0x02 0x15)
        if flags.rxm_rawx {
            let mut msg = RxmRawx::default();
            msg.rcv_tow = itow as f64 / 1000.0;
            msg.week = 2349;
            msg.leap_s = 18;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-EOE (0x01 0x61) - End of Epoch - always last
        if flags.nav_eoe {
            let msg = NavEoe { itow };
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }
    }
}

/// MON message task - sends monitoring messages at 1Hz when enabled
#[embassy_executor::task]
async fn mon_message_task() {
    // Wait for message output to start
    MSG_OUTPUT_STARTED.wait().await;
    Timer::after(Duration::from_secs(1)).await;
    info!("MON message task started");

    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut buf = [0u8; 128];

    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        // Get current message flags
        let flags = {
            let flags = MSG_FLAGS_STATE.lock().await;
            *flags
        };

        // MON-HW (0x0A 0x09)
        if flags.mon_hw {
            let msg = MonHw::default();
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // MON-COMMS (0x0A 0x36)
        if flags.mon_comms {
            let msg = MonComms::default();
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // MON-RF (0x0A 0x38)
        if flags.mon_rf {
            let msg = MonRf::default();
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }
    }
}

/// SEC-SIGN timer task - requests signature every 4 seconds
/// Only starts after message output begins (like C version)
#[embassy_executor::task]
async fn sec_sign_timer_task() {
    let session_id = DEFAULT_SESSION_ID;

    // Wait for message output to start
    MSG_OUTPUT_STARTED.wait().await;

    // First signature at 3 seconds after start
    Timer::after(Duration::from_secs(3)).await;

    info!("SEC-SIGN timer task started");

    // Then every 4 seconds
    let mut ticker = Ticker::every(Duration::from_secs(4));
    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        // Get hash and count from global accumulator, reset it
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

        SEC_SIGN_REQUEST.signal(request);
        info!("SEC-SIGN requested for {} packets", count);
    }
}

/// Mode button task - switches mode, saves to flash, and reboots
#[embassy_executor::task]
async fn button_task(mut btn: Input<'static>) {
    loop {
        btn.wait_for_high().await;
        Timer::after(Duration::from_millis(50)).await; // Debounce

        if btn.is_high() {
            let current = OperatingMode::load();
            let new_mode = match current {
                OperatingMode::Emulation => OperatingMode::Passthrough,
                OperatingMode::Passthrough => OperatingMode::Emulation,
            };

            info!("Switching to {:?}, saving to flash...", new_mode);

            // Save new mode to flash
            {
                let flash_mutex = unsafe { &*FLASH_PTR.load(Ordering::Acquire) };
                let mut flash = flash_mutex.lock().await;
                flash_storage::save_mode(&mut flash, new_mode as u8).await;
            }

            info!("Mode saved, rebooting...");

            // Wait for button release before reboot
            btn.wait_for_low().await;
            Timer::after(Duration::from_millis(100)).await;

            // Reboot to apply new mode
            SCB::sys_reset();
        }
    }
}
