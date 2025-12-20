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
mod led;
mod sec_sign;
mod ubx;

use core::sync::atomic::{AtomicU8, Ordering};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::{PIO0, UART0};
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
use sec_sign::{SecSignAccumulator, SecSignRequest, SecSignResult, Signature, DEFAULT_SESSION_ID, PRIVATE_KEY};
use ubx::{AckAck, MessageFlags, MonVer, NavDop, NavEoe, NavPvt, NavStatus, SecSign, SecUniqid, UbxMessage};

// UART interrupt binding
bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
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

    // Initialize UART buffers
    static TX_BUF: StaticCell<[u8; 512]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 512])[..];
    let rx_buf = &mut RX_BUF.init([0; 256])[..];

    // Configure UART
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

    // Initialize PIO for WS2812 LED
    let pio0 = Pio::new(p.PIO0, Irqs);

    // Mode button (GPIO5 = power, GPIO6 = input)
    let _btn_pwr = Output::new(p.PIN_5, Level::High);
    let btn_input = Input::new(p.PIN_6, Pull::Down);

    // Set initial mode
    OperatingMode::Emulation.store();

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

    // Core0 tasks
    spawner.must_spawn(uart_tx_task(tx));
    spawner.must_spawn(uart_rx_task(rx));
    spawner.must_spawn(nav_message_task());
    spawner.must_spawn(mon_message_task());
    spawner.must_spawn(sec_sign_timer_task());
    spawner.must_spawn(button_task(btn_input));

    info!("All tasks spawned, emulator running");

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

/// UART RX task - receives and processes UBX commands
#[embassy_executor::task]
async fn uart_rx_task(mut rx: embassy_rp::uart::BufferedUartRx) {
    let mut buf = [0u8; 128];
    let mut parser = ubx::UbxParser::new();

    loop {
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                for &byte in &buf[..n] {
                    if let Some(cmd) = parser.parse_byte(byte) {
                        handle_ubx_command(&cmd).await;
                    }
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

/// CFG-VALSET MSGOUT key IDs for UART1
mod msgout_keys {
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
}

/// Update message flags from CFG-VALSET key
fn update_flag_from_valset_key(flags: &mut MessageFlags, key: u32, val: u8) {
    let enabled = val > 0;
    match key {
        msgout_keys::NAV_PVT => flags.nav_pvt = enabled,
        msgout_keys::NAV_SVINFO => flags.nav_svinfo = enabled,
        msgout_keys::NAV_SAT => flags.nav_sat = enabled,
        msgout_keys::NAV_STATUS => flags.nav_status = enabled,
        msgout_keys::NAV_POSECEF => flags.nav_posecef = enabled,
        msgout_keys::NAV_POSLLH => flags.nav_posllh = enabled,
        msgout_keys::NAV_HPPOSECEF => flags.nav_hpposecef = enabled,
        msgout_keys::NAV_DOP => flags.nav_dop = enabled,
        msgout_keys::NAV_VELECEF => flags.nav_velecef = enabled,
        msgout_keys::NAV_VELNED => flags.nav_velned = enabled,
        msgout_keys::NAV_TIMEGPS => flags.nav_timegps = enabled,
        msgout_keys::NAV_TIMEUTC => flags.nav_timeutc = enabled,
        msgout_keys::NAV_TIMELS => flags.nav_timels = enabled,
        msgout_keys::NAV_CLOCK => flags.nav_clock = enabled,
        msgout_keys::NAV_AOPSTATUS => flags.nav_aopstatus = enabled,
        msgout_keys::NAV_COV => flags.nav_cov = enabled,
        msgout_keys::NAV_EOE => flags.nav_eoe = enabled,
        msgout_keys::TIM_TP => flags.tim_tp = enabled,
        msgout_keys::MON_HW => flags.mon_hw = enabled,
        msgout_keys::RXM_RAWX => flags.rxm_rawx = enabled,
        msgout_keys::MON_COMMS => flags.mon_comms = enabled,
        msgout_keys::MON_RF => flags.mon_rf = enabled,
        _ => {}
    }
}

/// Handle incoming UBX command
async fn handle_ubx_command(cmd: &ubx::UbxCommand) {
    match cmd {
        ubx::UbxCommand::CfgPrt { baudrate } => {
            info!("CFG-PRT: baudrate={}", baudrate);
            send_ack(0x06, 0x00); // ACK for CFG-PRT
            // TODO: Change baudrate dynamically
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
        ubx::UbxCommand::CfgRate { meas_rate, .. } => {
            info!("CFG-RATE: meas_rate={}", meas_rate);
            send_ack(0x06, 0x08); // ACK for CFG-RATE
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

            // Update message flags from MSGOUT keys
            let should_start = {
                let mut flags = MSG_FLAGS_STATE.lock().await;
                let was_empty = !flags.any_enabled();

                for &(key, val) in keys.iter() {
                    update_flag_from_valset_key(&mut flags, key, val);
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
            // Could send ACK-NACK for unsupported polls
        }
        ubx::UbxCommand::Unknown => {
            warn!("Unknown UBX command");
        }
    }
}

/// NAV message task - sends navigation messages at 5Hz
/// Only starts after receiving first CFG-MSG command + 1 second delay
#[embassy_executor::task]
async fn nav_message_task() {
    info!("NAV message task waiting for CFG-MSG...");

    // Wait for first CFG-MSG to enable messages
    MSG_OUTPUT_STARTED.wait().await;

    // 1 second delay after first CFG-MSG (like C version)
    Timer::after(Duration::from_secs(1)).await;
    info!("NAV message output started");

    let mut ticker = Ticker::every(Duration::from_millis(200)); // 5Hz
    let mut itow: u32 = 0;

    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        itow = itow.wrapping_add(200);

        // Get current message flags
        let flags = {
            let flags = MSG_FLAGS_STATE.lock().await;
            *flags
        };

        // Build and send enabled NAV messages
        let mut buf = [0u8; 128];

        // NAV-PVT
        if flags.nav_pvt {
            let mut msg = NavPvt::default();
            msg.itow = itow;
            let len = msg.build(&mut buf);
            if len > 0 {
                let mut vec = heapless::Vec::new();
                let _ = vec.extend_from_slice(&buf[..len]);
                let _ = TX_CHANNEL.try_send(vec);
            }
        }

        // NAV-STATUS
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

        // NAV-DOP
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

        // NAV-EOE (End of Epoch) - always last
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

/// MON message task - sends monitoring messages at 1Hz
/// Only starts after receiving first CFG-MSG command + 1 second delay
#[embassy_executor::task]
async fn mon_message_task() {
    // Wait for message output to start
    MSG_OUTPUT_STARTED.wait().await;
    Timer::after(Duration::from_secs(1)).await;
    info!("MON message task started");

    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        // TODO: Implement MON-HW, MON-COMMS, MON-RF messages when enabled
        let flags = {
            let flags = MSG_FLAGS_STATE.lock().await;
            *flags
        };

        if flags.mon_hw || flags.mon_comms || flags.mon_rf {
            trace!("MON messages would be sent here");
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

/// Mode button task
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
            new_mode.store();
            info!("Mode changed to {:?}", new_mode);

            // Wait for button release
            btn.wait_for_low().await;
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}
