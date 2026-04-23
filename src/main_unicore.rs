//! UC6580I emulator entry point (compile with `--features unicore --bin ublox_fake_uc`).
//!
//! Supports the same four operating modes as the u-blox build:
//!   0 Emulation          — fake UC6580I stream: boot-dump + NMEA + ExtRTCM 4074.
//!   1 Passthrough        — forward UART1 (real UC6580I) → UART0 (drone) verbatim.
//!   2 PassthroughRaw     — identical to Passthrough in this chip (no spoof detection yet).
//!   3 PassthroughOffset  — forward, but rewrite `$GxGGA`/`$GxRMC` latitude/longitude
//!                          to `config::offset_target` on the fly.
//!
//! Mode is persisted in the same flash slot as the u-blox build and changed
//! by clicking the mode button (GPIO13 power, GPIO14 sense) 1..4 times.

#![no_std]
#![no_main]

mod config;
mod coordinates;
mod flash_storage;
#[path = "unicore/mod.rs"]
mod unicore;

#[allow(dead_code)]
mod version {
    include!(concat!(env!("OUT_DIR"), "/version.rs"));
}

use core::sync::atomic::{AtomicU8, Ordering};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Flex, Level, Output, Pull};
use embassy_rp::peripherals::{FLASH, UART0, UART1};
#[cfg(not(feature = "rp2354"))]
use embassy_rp::peripherals::PIO0;
#[cfg(not(feature = "rp2354"))]
use embassy_rp::pio::Pio;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use panic_probe as _;
use static_cell::StaticCell;

use config::DEFAULT_BAUDRATE;

bind_interrupts!(pub struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
    #[cfg(not(feature = "rp2354"))]
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

// ---------------------------------------------------------------------------
// Operating mode
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
#[repr(u8)]
enum OperatingMode {
    Emulation = 0,
    Passthrough = 1,
    PassthroughRaw = 2,
    #[default]
    PassthroughOffset = 3,
}

impl OperatingMode {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Emulation,
            1 => Self::Passthrough,
            2 => Self::PassthroughRaw,
            _ => Self::PassthroughOffset,
        }
    }
    fn load() -> Self { Self::from_u8(MODE.load(Ordering::Acquire)) }
    fn store(self) { MODE.store(self as u8, Ordering::Release); }
}

static MODE: AtomicU8 = AtomicU8::new(OperatingMode::PassthroughOffset as u8);

// ---------------------------------------------------------------------------
// Shared channels
// ---------------------------------------------------------------------------

static TX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 1280>, 32> = Channel::new();
static RAW_RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 256>, 64> = Channel::new();

type FlashMutex = Mutex<CriticalSectionRawMutex, Flash<'static, FLASH, Async, { config::FLASH_SIZE_BYTES }>>;
static FLASH_CELL: StaticCell<FlashMutex> = StaticCell::new();

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("========================================");
    info!("UC6580I emulator starting ({})", version::FW_VERSION);
    info!("========================================");

    coordinates::init();

    // Flash: load persisted mode + save current firmware version.
    let flash = Flash::<_, Async, { config::FLASH_SIZE_BYTES }>::new(p.FLASH, p.DMA_CH0);
    let flash_mutex = FLASH_CELL.init(Mutex::new(flash));
    {
        let mut flash = flash_mutex.lock().await;
        let version_changed = flash_storage::load_version(&mut flash)
            .map(|s| s.as_str() != version::FW_VERSION)
            .unwrap_or(true);
        if version_changed {
            info!("New firmware version, resetting mode to PassthroughOffset");
            OperatingMode::PassthroughOffset.store();
            flash_storage::save_mode(&mut flash, OperatingMode::PassthroughOffset as u8).await;
        } else if let Some(b) = flash_storage::load_mode(&mut flash) {
            OperatingMode::from_u8(b).store();
            info!("Loaded mode from flash: {:?}", OperatingMode::load());
        }
        flash_storage::save_version(&mut flash, version::FW_VERSION);
    }

    // UART0: drone/host (921600, TX=GPIO0, RX=GPIO1).
    static TX_BUF0: StaticCell<[u8; 2048]> = StaticCell::new();
    static RX_BUF0: StaticCell<[u8; 512]> = StaticCell::new();
    let mut uart0_config = UartConfig::default();
    uart0_config.baudrate = DEFAULT_BAUDRATE;
    let uart0 = BufferedUart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs,
        &mut TX_BUF0.init([0; 2048])[..],
        &mut RX_BUF0.init([0; 512])[..],
        uart0_config,
    );
    let (uart0_tx, uart0_rx) = uart0.split();

    // UART1: real UC6580I source (921600, RX=GPIO5).
    static TX_BUF1: StaticCell<[u8; 128]> = StaticCell::new();
    static RX_BUF1: StaticCell<[u8; 4096]> = StaticCell::new();
    let mut uart1_config = UartConfig::default();
    uart1_config.baudrate = DEFAULT_BAUDRATE;
    let uart1 = BufferedUart::new(
        p.UART1, p.PIN_4, p.PIN_5, Irqs,
        &mut TX_BUF1.init([0; 128])[..],
        &mut RX_BUF1.init([0; 4096])[..],
        uart1_config,
    );
    let (_uart1_tx, uart1_rx) = uart1.split();

    // Button (GPIO13 = PWR, GPIO14 = sense) — same pinout as u-blox build.
    let _btn_pwr = Output::new(p.PIN_13, Level::High);
    let btn_flex = Flex::new(p.PIN_14);

    // LED: WS2812 on RP2350, simple GPIO on RP2354.
    #[cfg(not(feature = "rp2354"))]
    {
        let pio0 = Pio::new(p.PIO0, Irqs);
        spawner.must_spawn(led_task(pio0, p.DMA_CH1, p.PIN_25));
    }
    #[cfg(feature = "rp2354")]
    {
        let led_anode = Output::new(p.PIN_11, Level::Low);
        let _led_cathode = Output::new(p.PIN_12, Level::Low);
        spawner.must_spawn(simple_led_task(led_anode));
    }

    spawner.must_spawn(uart0_tx_task(uart0_tx));
    spawner.must_spawn(uart0_rx_task(uart0_rx));
    spawner.must_spawn(uart1_rx_task(uart1_rx));
    spawner.must_spawn(passthrough_forward_task());
    spawner.must_spawn(emulation_task());
    spawner.must_spawn(button_task(btn_flex, flash_mutex));

    info!("All tasks spawned — mode = {:?}", OperatingMode::load());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

// ---------------------------------------------------------------------------
// UART0 TX drain
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn uart0_tx_task(mut tx: BufferedUartTx) {
    loop {
        let frame = TX_CHANNEL.receive().await;
        if let Err(e) = tx.write_all(&frame).await {
            warn!("uart0 tx error: {:?}", defmt::Debug2Format(&e));
        }
    }
}

// ---------------------------------------------------------------------------
// UART0 RX — parse ASCII commands (active in Emulation mode), dispatch replies.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn uart0_rx_task(mut rx: BufferedUartRx) {
    let mut buf = [0u8; 128];
    let mut asm: unicore::nmea::LineAssembler<256> = unicore::nmea::LineAssembler::new();
    loop {
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                // Only respond to commands while we're acting as the chip.
                if OperatingMode::load() != OperatingMode::Emulation {
                    continue;
                }
                for &b in &buf[..n] {
                    if let Some(line) = asm.feed(b) {
                        handle_command(line).await;
                    }
                }
            }
            Ok(_) => {}
            Err(e) => {
                warn!("uart0 rx error: {:?}", defmt::Debug2Format(&e));
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}

async fn handle_command(line: &[u8]) {
    use unicore::cmd::{
        parse_command, reply_cfgkey, reply_cfgnav_default, reply_cfgsys_default, reply_fail,
        reply_gntxt_ack, reply_ok, reply_pdtinfo, reply_productinfo, CFGNAV_PREACK, CFGSYS_PREACK,
        Command, ParseError,
    };

    let mut scratch = [0u8; 256];
    let cmd = match parse_command(line) {
        Ok(c) => c,
        Err(ParseError::BadChecksum) => {
            let n = reply_fail(&mut scratch, 1);
            enqueue(&scratch[..n]);
            return;
        }
        Err(_) => {
            let n = reply_fail(&mut scratch, 0);
            enqueue(&scratch[..n]);
            return;
        }
    };

    let should_ack = matches!(
        cmd,
        Command::CfgSave | Command::CfgClr | Command::CfgMsg { rate: Some(_), .. } | Command::CfgNav { .. }
    );
    if should_ack {
        let mut end = line.len();
        while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
            end -= 1;
        }
        let start = if !line.is_empty() && line[0] == b'$' { 1 } else { 0 };
        let star = line[..end].iter().rposition(|&b| b == b'*').unwrap_or(end);
        let echo = &line[start..star.min(end)];
        let n = reply_gntxt_ack(&mut scratch, echo);
        if n > 0 { enqueue(&scratch[..n]); }
    }

    match cmd {
        Command::PdtInfo => {
            // Real chip emits no $GNTXT pre-ACK for $PDTINFO.
            let n = reply_pdtinfo(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::ProductInfo => {
            // Real chip emits no $GNTXT pre-ACK for $PRODUCTINFO either.
            let n = reply_productinfo(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::CfgSys(None) => {
            enqueue(CFGSYS_PREACK);
            let n = reply_cfgsys_default(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::CfgNav { meas_rate: 0, nav_rate: 0, dr_nav_rate: 0 } => {
            enqueue(CFGNAV_PREACK);
            let n = reply_cfgnav_default(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::CfgKey => {
            // No pre-ACK for $CFGKEY (verified on live chip).
            let n = reply_cfgkey(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        _ => {
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
    }
}

fn enqueue(bytes: &[u8]) {
    let mut v = heapless::Vec::<u8, 1280>::new();
    if v.extend_from_slice(bytes).is_ok() && TX_CHANNEL.try_send(v).is_err() {
        warn!("TX_CHANNEL full, dropped {} bytes", bytes.len());
    }
}

async fn enqueue_chunked(bytes: &[u8]) {
    let mut off = 0;
    while off < bytes.len() {
        let end = (off + 1280).min(bytes.len());
        let mut v = heapless::Vec::<u8, 1280>::new();
        if v.extend_from_slice(&bytes[off..end]).is_ok() {
            TX_CHANNEL.send(v).await;
        }
        off = end;
    }
}

// ---------------------------------------------------------------------------
// UART1 RX — always read; route to passthrough buffer.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn uart1_rx_task(mut rx: BufferedUartRx) {
    let mut buf = [0u8; 256];
    loop {
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                if OperatingMode::load() == OperatingMode::Emulation {
                    continue; // drop while we are emulating
                }
                let mut v = heapless::Vec::<u8, 256>::new();
                let _ = v.extend_from_slice(&buf[..n]);
                if RAW_RX_CHANNEL.try_send(v).is_err() {
                    // Buffer full — drop rather than stall UART.
                }
            }
            Ok(_) => {}
            Err(e) => {
                warn!("uart1 rx error: {:?}", defmt::Debug2Format(&e));
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Passthrough forwarding.
// Raw + Passthrough = byte-for-byte forward.
// Offset = reassemble NMEA lines, rewrite coordinates, forward; non-NMEA bytes pass through.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn passthrough_forward_task() {
    let mut asm: unicore::nmea::LineAssembler<256> = unicore::nmea::LineAssembler::new();
    loop {
        let chunk = RAW_RX_CHANNEL.receive().await;
        let mode = OperatingMode::load();
        match mode {
            OperatingMode::Emulation => { /* ignore */ }
            OperatingMode::PassthroughRaw | OperatingMode::Passthrough => {
                // Forward untouched in 1280-byte chunks (single chunk is always ≤256 here).
                let mut v = heapless::Vec::<u8, 1280>::new();
                let _ = v.extend_from_slice(&chunk);
                let _ = TX_CHANNEL.try_send(v);
            }
            OperatingMode::PassthroughOffset => {
                for &b in chunk.iter() {
                    // Bytes that are not part of an NMEA sentence flow through directly.
                    if !asm_in_progress(&asm) && b != b'$' {
                        let mut v = heapless::Vec::<u8, 1280>::new();
                        let _ = v.push(b);
                        let _ = TX_CHANNEL.try_send(v);
                        continue;
                    }
                    if let Some(line) = asm.feed(b) {
                        // Try to rewrite GGA/RMC; forward as-is on failure.
                        let mut work = [0u8; 320];
                        if line.len() <= work.len() {
                            work[..line.len()].copy_from_slice(line);
                            let lat = config::offset_target::LAT_1E7;
                            let lon = config::offset_target::LON_1E7;
                            let alt = config::offset_target::ALT_MM;
                            match unicore::nmea::rewrite_position_inplace(&mut work, line.len(), lat, lon, Some(alt)) {
                                Some(new_len) => {
                                    let mut v = heapless::Vec::<u8, 1280>::new();
                                    let _ = v.extend_from_slice(&work[..new_len]);
                                    let _ = TX_CHANNEL.try_send(v);
                                }
                                None => {
                                    let mut v = heapless::Vec::<u8, 1280>::new();
                                    let _ = v.extend_from_slice(line);
                                    let _ = TX_CHANNEL.try_send(v);
                                }
                            }
                        } else {
                            let mut v = heapless::Vec::<u8, 1280>::new();
                            let _ = v.extend_from_slice(line);
                            let _ = TX_CHANNEL.try_send(v);
                        }
                    }
                }
            }
        }
    }
}

fn asm_in_progress<const N: usize>(_asm: &unicore::nmea::LineAssembler<N>) -> bool {
    // LineAssembler does not expose its state; conservative answer is false,
    // which gives us the desired behaviour: any byte that isn't `$` while no
    // sentence has started falls through to passthrough, and once `feed` sees
    // `$` it starts a new sentence (the previous partial is discarded anyway).
    false
}

// ---------------------------------------------------------------------------
// Emulation task: boot-dump + 5 Hz NMEA + 1 Hz ExtRTCM bundle. Only runs when
// MODE == Emulation; other modes return without touching TX_CHANNEL.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn emulation_task() {
    let mut tick = Ticker::every(Duration::from_millis(200));
    let mut counter: u32 = 0;
    let mut buf = [0u8; 256];
    let mut booted = false;

    loop {
        tick.next().await;
        if OperatingMode::load() != OperatingMode::Emulation {
            booted = false;
            continue;
        }
        if !booted {
            enqueue_chunked(unicore::boot::BOOT_DUMP).await;
            booted = true;
        }
        counter = counter.wrapping_add(1);

        enqueue(b"$GNRMC,,V,,,,,,,,,,N,V*37\r\n");
        enqueue(b"$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n");
        enqueue(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1*33\r\n");
        enqueue(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,2*30\r\n");
        enqueue(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,3*31\r\n");
        enqueue(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,4*36\r\n");
        enqueue(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,5*37\r\n");

        if counter % 2 == 0 {
            enqueue(b"$GPGSV,1,1,00,1*64\r\n");
            enqueue(b"$GPGSV,1,1,00,8*6D\r\n");
            enqueue(b"$GBGSV,1,1,00,1*76\r\n");
            enqueue(b"$GBGSV,1,1,00,5*72\r\n");
            enqueue(b"$GAGSV,1,1,00,7*73\r\n");
            enqueue(b"$GAGSV,1,1,00,1*75\r\n");
            enqueue(b"$GLGSV,1,1,00,1*78\r\n");
            enqueue(b"$GQGSV,1,1,00,1*65\r\n");
            enqueue(b"$GQGSV,1,1,00,8*6C\r\n");
        }
        if counter % 10 == 0 {
            enqueue(b"$PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0*37\r\n");
        }

        // RTCM MSM7 + ephemerides — rates matched to real chip log 2026-04-23.
        if counter % 5 == 0 { enqueue(unicore::rtcm_samples::FRAME_MSG_1077); } // ≈1 Hz
        if counter % 3 == 0 { enqueue(unicore::rtcm_samples::FRAME_MSG_1097); } // ≈1.7 Hz
        if counter % 3 == 1 { enqueue(unicore::rtcm_samples::FRAME_MSG_1019); } // ≈1.7 Hz
        if counter % 8 == 0 { enqueue(unicore::rtcm_samples::FRAME_MSG_1046); } // ≈0.6 Hz

        // Extended RTCM 4074 status bundle — ≈1 Hz.
        if counter % 5 == 0 {
            for &(sub, data) in &[
                (0x0FEu16, unicore::extrtcm::DATA_SUB_0FE),
                (0x0E6,    unicore::extrtcm::DATA_SUB_0E6),
                (0x0F9,    unicore::extrtcm::DATA_SUB_0F9),
                (0x0E9,    unicore::extrtcm::DATA_SUB_0E9),
                (0x0FF,    unicore::extrtcm::DATA_SUB_0FF),
            ] {
                let n = unicore::extrtcm::build_sub(&mut buf, sub, data);
                if n > 0 { enqueue(&buf[..n]); }
            }
        }

        // $PPSInfo ≈ every 17 s; $SVEPH once per minute-ish.
        if counter % 85 == 40  { enqueue(unicore::rtcm_samples::PPS_INFO_SAMPLE); }
        if counter % 300 == 120 { enqueue(unicore::rtcm_samples::SVEPH_SAMPLE); }
    }
}

// ---------------------------------------------------------------------------
// LED task — WS2812 on RP2350, one colour per operating mode.
// ---------------------------------------------------------------------------

#[cfg(not(feature = "rp2354"))]
#[embassy_executor::task]
async fn led_task(
    mut pio: Pio<'static, PIO0>,
    dma: embassy_rp::Peri<'static, embassy_rp::peripherals::DMA_CH1>,
    pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_25>,
) {
    use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
    use smart_leds::RGB8;

    let program = PioWs2812Program::new(&mut pio.common);
    let mut ws: PioWs2812<_, 0, 1, Grb> =
        PioWs2812::with_color_order(&mut pio.common, pio.sm0, dma, pin, &program);

    let mut ticker = Ticker::every(Duration::from_millis(250));
    let mut phase: u8 = 0;
    loop {
        phase = phase.wrapping_add(1);
        // Distinct colour palette from the u-blox build so at a glance you
        // can tell which firmware variant is running.
        let colour = match OperatingMode::load() {
            OperatingMode::Emulation       => RGB8::new(40, 20, 0),  // orange
            OperatingMode::Passthrough     => RGB8::new(0, 30, 30),  // cyan
            OperatingMode::PassthroughRaw  => RGB8::new(30, 0, 30),  // magenta
            OperatingMode::PassthroughOffset => RGB8::new(40, 30, 0), // yellow
        };
        let value = if phase.is_multiple_of(2) { colour } else { RGB8::new(0, 0, 0) };
        let _ = ws.write(&[value]).await;
        ticker.next().await;
    }
}

// ---------------------------------------------------------------------------
// Simple GPIO LED for RP2354 — blink pattern carries mode info (1..4 blinks).
// ---------------------------------------------------------------------------

#[cfg(feature = "rp2354")]
#[embassy_executor::task]
async fn simple_led_task(mut led: Output<'static>) {
    loop {
        let blinks = (OperatingMode::load() as u8) + 1;
        for _ in 0..blinks {
            led.set_high();
            Timer::after(Duration::from_millis(120)).await;
            led.set_low();
            Timer::after(Duration::from_millis(120)).await;
        }
        Timer::after(Duration::from_millis(1200)).await;
    }
}

// ---------------------------------------------------------------------------
// Button task — count clicks in an 800 ms window, apply mode 1..4 → 0..3,
// persist to flash.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn button_task(mut btn: Flex<'static>, flash: &'static FlashMutex) {
    btn.set_as_input();
    btn.set_pull(Pull::Down);
    let mut last_level = btn.is_high();
    let mut press_count: u8 = 0;
    let mut series_start: Option<Instant> = None;

    loop {
        Timer::after(Duration::from_millis(config::button::POLL_PERIOD_MS)).await;
        let level = btn.is_high();

        // Rising edge (debounced by 50ms).
        if level && !last_level {
            Timer::after(Duration::from_millis(config::button::DEBOUNCE_MS)).await;
            if btn.is_high() {
                press_count += 1;
                if series_start.is_none() {
                    series_start = Some(Instant::now());
                }
            }
        }
        last_level = level;

        // Window expired — apply mode selection.
        if let Some(start) = series_start {
            if start.elapsed().as_millis() >= config::button::MULTI_CLICK_TIMEOUT_MS as u64
                && press_count > 0
            {
                let new_mode = match press_count {
                    1 => OperatingMode::Emulation,
                    2 => OperatingMode::Passthrough,
                    3 => OperatingMode::PassthroughRaw,
                    _ => OperatingMode::PassthroughOffset,
                };
                let current = OperatingMode::load();
                if new_mode != current {
                    new_mode.store();
                    let mut flash = flash.lock().await;
                    flash_storage::save_mode(&mut flash, new_mode as u8).await;
                    info!("Mode: {:?} → {:?}", current, new_mode);
                }
                press_count = 0;
                series_start = None;
            }
        }
    }
}
