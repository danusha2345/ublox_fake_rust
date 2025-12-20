//! u-blox GNSS M8/M10 Emulator for RP2040/RP2350
//!
//! Dual-core architecture:
//! - Core0: Embassy async executor, UART, message transmission
//! - Core1: LED control, GPIO button, SEC-SIGN computation
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
use embassy_rp::peripherals::{PIO0, UART0};
use embassy_rp::pio::Pio;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
// Import traits for method resolution (v0.6 to match embassy-rp)
use embedded_io_async::{Read, Write};
use panic_probe as _;
use static_cell::StaticCell;

use config::*;

// UART interrupt binding
bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

// Inter-core communication
static SEC_SIGN_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static SEC_SIGN_READY: Signal<CriticalSectionRawMutex, [u8; 120]> = Signal::new();

// Message enable flags channel
static MSG_FLAGS: Channel<CriticalSectionRawMutex, ubx::MessageFlags, 4> = Channel::new();

// Current operating mode (0 = Emulation, 1 = Passthrough)
static MODE: AtomicU8 = AtomicU8::new(0);

/// Operating mode
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum OperatingMode {
    Emulation = 0,
    Passthrough = 1,
}

impl OperatingMode {
    fn load() -> Self {
        match MODE.load(Ordering::Relaxed) {
            0 => Self::Emulation,
            _ => Self::Passthrough,
        }
    }

    fn store(self) {
        MODE.store(self as u8, Ordering::Relaxed);
    }
}

impl Default for OperatingMode {
    fn default() -> Self {
        Self::Emulation
    }
}

/// Main entry point - runs on Core0
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("u-blox GNSS Emulator starting...");

    let p = embassy_rp::init(Default::default());

    // Initialize UART buffers
    static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 256])[..];
    let rx_buf = &mut RX_BUF.init([0; 256])[..];

    // Configure UART
    // Embassy 0.9 API: new(uart, tx_pin, rx_pin, irq, tx_buf, rx_buf, config)
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = DEFAULT_BAUDRATE;
    let uart = BufferedUart::new(
        p.UART0,
        p.PIN_0,  // TX
        p.PIN_1,  // RX
        Irqs,
        tx_buf,
        rx_buf,
        uart_config,
    );
    let (tx, rx) = uart.split();

    // Initialize PIO for WS2812 LED
    let pio0 = Pio::new(p.PIO0, Irqs);

    // Mode button
    let _btn_pwr = Output::new(p.PIN_5, Level::High);
    let btn_input = Input::new(p.PIN_6, Pull::Down);

    // Set initial mode
    OperatingMode::Emulation.store();

    // Spawn tasks
    spawner.must_spawn(led_task(pio0, p.PIN_16));
    spawner.must_spawn(uart_tx_task(tx));
    spawner.must_spawn(uart_rx_task(rx));
    spawner.must_spawn(nav_message_task());
    spawner.must_spawn(mon_message_task());
    spawner.must_spawn(sec_sign_timer_task());
    spawner.must_spawn(button_task(btn_input));

    info!("All tasks spawned, emulator running");

    // Core0 main loop - nothing to do, all work in tasks
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// LED control task - WS2812 on PIO
#[embassy_executor::task]
async fn led_task(mut pio: Pio<'static, PIO0>, pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_16>) {
    use led::Ws2812;

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

/// UART TX task - sends UBX messages
#[embassy_executor::task]
async fn uart_tx_task(mut tx: embassy_rp::uart::BufferedUartTx) {
    // Wait for initial delay before sending
    Timer::after(Duration::from_secs(1)).await;

    loop {
        // Check for SEC-SIGN ready
        if let Some(sec_sign_msg) = SEC_SIGN_READY.try_take() {
            if let Err(e) = tx.write_all(&sec_sign_msg).await {
                error!("UART TX error: {:?}", e);
            }
        }

        Timer::after(Duration::from_millis(10)).await;
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

/// Handle incoming UBX command
async fn handle_ubx_command(cmd: &ubx::UbxCommand) {
    match cmd {
        ubx::UbxCommand::CfgPrt { baudrate } => {
            info!("CFG-PRT: baudrate={}", baudrate);
            // TODO: Change baudrate dynamically
        }
        ubx::UbxCommand::CfgMsg { class, id, rate } => {
            info!("CFG-MSG: class=0x{:02X} id=0x{:02X} rate={}", class, id, rate);
            // Update message flags
            let flags = ubx::MessageFlags::from_cfg_msg(*class, *id, *rate > 0);
            let _ = MSG_FLAGS.try_send(flags);
        }
        ubx::UbxCommand::CfgRate { meas_rate, .. } => {
            info!("CFG-RATE: meas_rate={}", meas_rate);
        }
        ubx::UbxCommand::CfgValset { .. } => {
            info!("CFG-VALSET received");
        }
        ubx::UbxCommand::Poll { class, id } => {
            info!("Poll: class=0x{:02X} id=0x{:02X}", class, id);
        }
        ubx::UbxCommand::Unknown => {
            warn!("Unknown UBX command");
        }
    }
}

/// NAV message task - sends navigation messages at 5Hz
#[embassy_executor::task]
async fn nav_message_task() {
    let mut ticker = Ticker::every(Duration::from_millis(200)); // 5Hz
    let mut itow: u32 = 0;

    // Wait for startup
    Timer::after(Duration::from_secs(2)).await;

    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        itow = itow.wrapping_add(200);

        // TODO: Send enabled NAV messages
        // Check MSG_FLAGS for which messages to send
    }
}

/// MON message task - sends monitoring messages at 1Hz
#[embassy_executor::task]
async fn mon_message_task() {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    // Wait for startup
    Timer::after(Duration::from_secs(2)).await;

    loop {
        ticker.next().await;

        let mode = OperatingMode::load();
        if mode != OperatingMode::Emulation {
            continue;
        }

        // TODO: Send enabled MON messages
    }
}

/// SEC-SIGN timer task - requests signature every 4 seconds
#[embassy_executor::task]
async fn sec_sign_timer_task() {
    // First signature at 3 seconds
    Timer::after(Duration::from_secs(3)).await;
    SEC_SIGN_REQUEST.signal(());

    // Then every 4 seconds
    let mut ticker = Ticker::every(Duration::from_secs(4));
    loop {
        ticker.next().await;
        SEC_SIGN_REQUEST.signal(());
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
