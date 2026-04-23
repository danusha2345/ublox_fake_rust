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
mod spoof_detector;
#[path = "unicore/mod.rs"]
mod unicore;

#[allow(dead_code)]
mod version {
    include!(concat!(env!("OUT_DIR"), "/version.rs"));
}

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
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

/// Set to true when the spoof-detector flagged the current position as spoofed.
static SPOOF_DETECTED: AtomicBool = AtomicBool::new(false);

/// Detector instance — lives for the whole session, reset via `.reset()` on
/// mode changes that would otherwise pollute its internal `LAST_GOOD`.
static SPOOF_CELL: Mutex<CriticalSectionRawMutex, Option<spoof_detector::SpoofDetector>> =
    Mutex::new(None);

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

    // Initialise the spoof detector once.
    {
        let mut cell = SPOOF_CELL.lock().await;
        *cell = Some(spoof_detector::SpoofDetector::default());
    }

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
        build_cfgmsg_reply, default_cfgmsg_rate, is_known_cfgmsg, parse_command, reply_cfgkey,
        reply_cfgmsm, reply_cfgnav_default, reply_cfgnmea, reply_cfgprt_uart1, reply_cfgprt_uart2,
        reply_cfgsys_default, reply_fail, reply_gntxt_ack, reply_ok, reply_pdtinfo,
        reply_productinfo, CFGMSM_PREACK, CFGNAV_PREACK, CFGSYS_PREACK, Command, ParseError,
    };

    // Full echo body (between `$` and trailing \r\n, INCLUDING *HH if present).
    let mut end = line.len();
    while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
        end -= 1;
    }
    let start = if !line.is_empty() && line[0] == b'$' { 1 } else { 0 };
    let echo_full = &line[start..end];
    // Echo stripped of *HH, used as the $GNTXT body for valid replies.
    let star = echo_full.iter().rposition(|&b| b == b'*').unwrap_or(echo_full.len());
    let echo_clean = &echo_full[..star];

    let mut scratch = [0u8; 256];

    let cmd = match parse_command(line) {
        Ok(c) => c,
        Err(ParseError::BadChecksum) => {
            // Live chip echoes the full (invalid) body in $GNTXT, then replies $FAIL,1.
            let n = reply_gntxt_ack(&mut scratch, echo_full);
            if n > 0 { enqueue(&scratch[..n]); }
            let n = reply_fail(&mut scratch, 1);
            enqueue(&scratch[..n]);
            return;
        }
        Err(ParseError::Malformed) => {
            // Known CMD, bad args → same echo + $FAIL,0.
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { enqueue(&scratch[..n]); }
            let n = reply_fail(&mut scratch, 0);
            enqueue(&scratch[..n]);
            return;
        }
        Err(ParseError::NoDollarPrefix) => {
            return; // silently ignore stray bytes
        }
    };

    match cmd {
        Command::Unknown(_) => {
            // Live chip silently ignores commands it does not recognise.
        }
        Command::PdtInfo => {
            // Strict: extra args → $GNTXT + $FAIL,0 (confirmed on live chip).
            if echo_clean.contains(&b',') {
                let n = reply_gntxt_ack(&mut scratch, echo_clean);
                if n > 0 { enqueue(&scratch[..n]); }
                let n = reply_fail(&mut scratch, 0);
                enqueue(&scratch[..n]);
                return;
            }
            let n = reply_pdtinfo(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::ProductInfo => {
            if echo_clean.contains(&b',') {
                let n = reply_gntxt_ack(&mut scratch, echo_clean);
                if n > 0 { enqueue(&scratch[..n]); }
                let n = reply_fail(&mut scratch, 0);
                enqueue(&scratch[..n]);
                return;
            }
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
            // No $GNTXT pre-ACK for $CFGKEY on the live chip.
            let n = reply_cfgkey(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::CfgMsg { class, id, rate: None } => {
            // GET: $GNTXT echo + (either $CFGMSG,c,i,rate*cs + $OK, or $FAIL,0 for unknown).
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { enqueue(&scratch[..n]); }
            match default_cfgmsg_rate(class, id) {
                Some(r) => {
                    let n = build_cfgmsg_reply(&mut scratch, class, id, r);
                    if n > 0 { enqueue(&scratch[..n]); }
                    let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
                }
                None => {
                    let n = reply_fail(&mut scratch, 0); enqueue(&scratch[..n]);
                }
            }
        }
        Command::CfgMsg { class, id, rate: Some(_) } => {
            // SET: echo + ($OK if known pair, $FAIL,0 if not)
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { enqueue(&scratch[..n]); }
            if is_known_cfgmsg(class, id) {
                let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
            } else {
                let n = reply_fail(&mut scratch, 0); enqueue(&scratch[..n]);
            }
        }
        Command::CfgMsm => {
            enqueue(CFGMSM_PREACK);
            let n = reply_cfgmsm(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::CfgNmea => {
            // No $GNTXT pre-ACK for $CFGNMEA (verified live).
            let n = reply_cfgnmea(&mut scratch); enqueue(&scratch[..n]);
            let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
        }
        Command::CfgPrt(port) => {
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { enqueue(&scratch[..n]); }
            // Port 1 (UART1) default; port 2 (UART2) also valid; others $FAIL,0.
            match port {
                None | Some(1) => {
                    let n = reply_cfgprt_uart1(&mut scratch); enqueue(&scratch[..n]);
                    let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
                }
                Some(2) => {
                    let n = reply_cfgprt_uart2(&mut scratch); enqueue(&scratch[..n]);
                    let n = reply_ok(&mut scratch); enqueue(&scratch[..n]);
                }
                _ => {
                    let n = reply_fail(&mut scratch, 0); enqueue(&scratch[..n]);
                }
            }
        }
        // Remaining SET-ish commands (CFGSAVE, CFGCLR, CFGSYS with mask, etc.)
        // — echo in $GNTXT then $OK.
        _ => {
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { enqueue(&scratch[..n]); }
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

/// Canned no-fix replacements emitted when spoofing is detected.
const NOFIX_RMC: &[u8] = b"$GNRMC,,V,,,,,,,,,,N,V*37\r\n";
const NOFIX_GGA: &[u8] = b"$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n";

#[embassy_executor::task]
async fn passthrough_forward_task() {
    use unicore::nmea;

    let mut asm: nmea::LineAssembler<256> = nmea::LineAssembler::new();
    let mut pass_buf = heapless::Vec::<u8, 256>::new();
    let mut in_sentence = false;
    let mut last_mode = OperatingMode::load();

    loop {
        let chunk = RAW_RX_CHANNEL.receive().await;
        let mode = OperatingMode::load();

        // Reset assembler + detector when the operator flips modes, so
        // stale LAST_GOOD data cannot trigger false positives.
        if mode != last_mode {
            asm = nmea::LineAssembler::new();
            pass_buf.clear();
            in_sentence = false;
            SPOOF_DETECTED.store(false, Ordering::Release);
            let mut d = SPOOF_CELL.lock().await;
            if let Some(ref mut det) = *d {
                *det = spoof_detector::SpoofDetector::default();
            }
            last_mode = mode;
        }

        match mode {
            OperatingMode::Emulation => {
                // Drop incoming chip bytes while we are emitting our own.
            }
            OperatingMode::PassthroughRaw => {
                // Byte-for-byte forward, no inspection.
                let mut v = heapless::Vec::<u8, 1280>::new();
                let _ = v.extend_from_slice(&chunk);
                let _ = TX_CHANNEL.try_send(v);
            }
            OperatingMode::Passthrough | OperatingMode::PassthroughOffset => {
                for &b in chunk.iter() {
                    // Non-NMEA bytes (RTCM payload, noise) pass through directly.
                    if !in_sentence && b != b'$' {
                        // batch pass-through bytes into 256-byte TX frames
                        if pass_buf.push(b).is_err() {
                            let mut v = heapless::Vec::<u8, 1280>::new();
                            let _ = v.extend_from_slice(&pass_buf);
                            let _ = TX_CHANNEL.try_send(v);
                            pass_buf.clear();
                            let _ = pass_buf.push(b);
                        }
                        continue;
                    }
                    // Flush any queued pass-through bytes before we start on NMEA.
                    if b == b'$' && !pass_buf.is_empty() {
                        let mut v = heapless::Vec::<u8, 1280>::new();
                        let _ = v.extend_from_slice(&pass_buf);
                        let _ = TX_CHANNEL.try_send(v);
                        pass_buf.clear();
                    }
                    if b == b'$' { in_sentence = true; }

                    if let Some(line) = asm.feed(b) {
                        in_sentence = false;
                        process_nmea_line(line, mode).await;
                    }
                }
            }
        }
    }
}

async fn process_nmea_line(line: &[u8], mode: OperatingMode) {
    use unicore::nmea;

    // Try to extract a position from GGA / RMC and feed the detector.
    let fix = nmea::parse_gga(line).or_else(|| nmea::parse_rmc(line));
    if let Some(fix) = fix {
        if fix.checksum_ok && fix.fix_quality > 0 {
            let pos = spoof_detector::Position {
                lat: fix.lat_1e7,
                lon: fix.lon_1e7,
                alt_mm: fix.alt_mm,
                time_ms: embassy_time::Instant::now().as_millis() as u32,
                fix_type: spoof_detector::FixType::Fix3D,
                h_acc_mm: 0,
                num_sv: fix.nsats,
                pdop: 0,
                gnss_time: None,
                cno_values: heapless::Vec::new(),
            };
            let mut d = SPOOF_CELL.lock().await;
            if let Some(ref mut det) = *d {
                let result = det.analyze(pos);
                match result {
                    spoof_detector::AnalysisResult::Spoofed => {
                        SPOOF_DETECTED.store(true, Ordering::Release);
                    }
                    spoof_detector::AnalysisResult::Normal => {
                        SPOOF_DETECTED.store(false, Ordering::Release);
                    }
                    _ => {}
                }
            }
        }
    }

    let spoofed = SPOOF_DETECTED.load(Ordering::Acquire);
    let is_gga = line.len() >= 6 && &line[3..6] == b"GGA";
    let is_rmc = line.len() >= 6 && &line[3..6] == b"RMC";

    // Spoof state: replace GGA/RMC with no-fix so the drone fails over.
    if spoofed {
        if is_gga {
            let mut v = heapless::Vec::<u8, 1280>::new();
            let _ = v.extend_from_slice(NOFIX_GGA);
            let _ = TX_CHANNEL.try_send(v);
            return;
        }
        if is_rmc {
            let mut v = heapless::Vec::<u8, 1280>::new();
            let _ = v.extend_from_slice(NOFIX_RMC);
            let _ = TX_CHANNEL.try_send(v);
            return;
        }
        // Other NMEA (GSA/GSV/VTG/...) pass through — they do not carry a position.
    }

    // Offset: rewrite coords in GGA/RMC only.
    if mode == OperatingMode::PassthroughOffset && (is_gga || is_rmc) {
        let mut work = [0u8; 320];
        if line.len() <= work.len() {
            work[..line.len()].copy_from_slice(line);
            if let Some(new_len) = nmea::rewrite_position_inplace(
                &mut work,
                line.len(),
                config::offset_target::LAT_1E7,
                config::offset_target::LON_1E7,
                if is_gga { Some(config::offset_target::ALT_MM) } else { None },
            ) {
                let mut v = heapless::Vec::<u8, 1280>::new();
                let _ = v.extend_from_slice(&work[..new_len]);
                let _ = TX_CHANNEL.try_send(v);
                return;
            }
        }
    }

    // Default: forward verbatim.
    let mut v = heapless::Vec::<u8, 1280>::new();
    let _ = v.extend_from_slice(line);
    let _ = TX_CHANNEL.try_send(v);
}

// ---------------------------------------------------------------------------
// Emulation task: boot-dump + 5 Hz NMEA + 1 Hz ExtRTCM bundle. Only runs when
// MODE == Emulation; other modes return without touching TX_CHANNEL.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn emulation_task() {
    use unicore::nmea::{
        build_gga, build_gsa, build_gsv, build_rmc, GgaFields, GsaFields, GsaOpMode,
        GsvSat, NmeaDate, NmeaTime, RmcFields, Talker,
    };

    let mut tick = Ticker::every(Duration::from_millis(200));
    let mut counter: u32 = 0;
    let mut buf = [0u8; 256];
    let mut booted = false;

    // Simulated UTC time, reset at each (re)start of Emulation mode.
    // Increment by 200 ms per tick. Start at 03:01:55.00 UTC 2025-06-05
    // (matches the `$GNZDA` values we observed from the live chip probe).
    let start_centis: u64 = (3 * 3_600 + 1 * 60 + 55) * 100; // 10 865 500
    let mut centis: u64 = start_centis;
    let date = NmeaDate { day: 5, month: 6, year: 2025 };

    // Fake satellite roster — gives the drone something coherent to see
    // across GGA/GSA/GSV without needing real ephemeris.
    const FAKE_SATS: &[(u16, u8, u16, u8)] = &[
        // (PRN, elev_deg, azim_deg, cno_dbhz)
        (5, 45, 90, 45), (13, 60, 180, 47), (15, 30, 270, 42), (20, 80, 0, 50),
    ];

    loop {
        tick.next().await;
        if OperatingMode::load() != OperatingMode::Emulation {
            booted = false;
            centis = start_centis;
            counter = 0;
            continue;
        }
        if !booted {
            enqueue_chunked(unicore::boot::BOOT_DUMP).await;
            booted = true;
            counter = 0;
            centis = start_centis;
        }
        counter = counter.wrapping_add(1);
        centis += 20;

        // First 25 ticks (≈5 s) — cold start, no fix. After — 3D fix with
        // a growing sat count (4 → 12 over ~16 s) for realism.
        let cold = counter < 25;
        let valid = !cold;
        let fix_quality: u8 = if valid { 1 } else { 0 };
        let nsats: u8 = if valid { (4 + (counter - 25) / 10).min(12) as u8 } else { 0 };

        // Unpack simulated time
        let total_s = centis / 100;
        let cs = (centis % 100) as u8;
        let hour = ((total_s / 3600) % 24) as u8;
        let minute = ((total_s / 60) % 60) as u8;
        let second = (total_s % 60) as u8;
        let time = NmeaTime { hour, minute, second, centis: cs };

        let lat = if valid { config::default_position::LAT_1E7 } else { 0 };
        let lon = if valid { config::default_position::LON_1E7 } else { 0 };
        let alt = if valid { config::default_position::ALT_MM } else { 0 };

        // GGA every tick (rate=1).
        let gga = GgaFields {
            time,
            lat_1e7: lat,
            lon_1e7: lon,
            fix_quality,
            nsats,
            hdop_x100: if valid { 99 } else { 9999 },
            alt_mm: alt,
            geoid_sep_mm: -30_000,
        };
        let n = build_gga(&mut buf, Talker::Gn, &gga);
        if n > 0 { enqueue(&buf[..n]); }

        // RMC every tick (rate=1).
        let rmc = RmcFields {
            time,
            valid,
            lat_1e7: lat,
            lon_1e7: lon,
            sog_knots_x1000: 0,
            cog_deg_x100: 0,
            date,
            mode: if valid { b'A' } else { b'N' },
        };
        let n = build_rmc(&mut buf, Talker::Gn, &rmc);
        if n > 0 { enqueue(&buf[..n]); }

        // GSA every 5th tick (rate=5): 5 sentences, one per constellation.
        if counter % 5 == 0 {
            for (sys_id, prn_lo, prn_hi) in &[
                (1u8, 1u16, 32u16),    // GPS
                (2, 65, 96),            // GLONASS
                (3, 1, 30),             // Galileo
                (4, 1, 40),             // BeiDou
                (5, 193, 200),          // QZSS
            ] {
                let mut sats: [u16; 12] = [0; 12];
                if valid {
                    // Put a couple of our fake PRNs if they fall into the range,
                    // else pick plausible numbers inside the constellation span.
                    let a = prn_lo.saturating_add(4);
                    let b = prn_lo.saturating_add(8);
                    sats[0] = a.min(*prn_hi);
                    sats[1] = b.min(*prn_hi);
                }
                let gsa = GsaFields {
                    op_mode: GsaOpMode::Automatic,
                    fix_type: if valid { 3 } else { 1 },
                    sats,
                    pdop_x100: if valid { 99 } else { 9999 },
                    hdop_x100: if valid { 99 } else { 9999 },
                    vdop_x100: if valid { 99 } else { 9999 },
                    system_id: *sys_id,
                };
                let n = build_gsa(&mut buf, Talker::Gn, &gsa);
                if n > 0 { enqueue(&buf[..n]); }
            }
        }

        // GSV every 5th tick (rate=5): one page per talker per signal.
        if counter % 5 == 0 {
            let view = if valid { FAKE_SATS.len() as u8 } else { 0 };
            let pages: &[(Talker, u8)] = &[
                (Talker::Gp, 1), (Talker::Gp, 8),
                (Talker::Gb, 1), (Talker::Gb, 5),
                (Talker::Ga, 7), (Talker::Ga, 1),
                (Talker::Gl, 1),
                (Talker::Gq, 1), (Talker::Gq, 8),
            ];
            for (talker, sig) in pages {
                let sats_vec: [GsvSat; 4] = [
                    GsvSat { prn: FAKE_SATS[0].0, elevation_deg: FAKE_SATS[0].1,
                             azimuth_deg: FAKE_SATS[0].2, cno_dbhz: if valid { FAKE_SATS[0].3 } else { 0 } },
                    GsvSat { prn: FAKE_SATS[1].0, elevation_deg: FAKE_SATS[1].1,
                             azimuth_deg: FAKE_SATS[1].2, cno_dbhz: if valid { FAKE_SATS[1].3 } else { 0 } },
                    GsvSat { prn: FAKE_SATS[2].0, elevation_deg: FAKE_SATS[2].1,
                             azimuth_deg: FAKE_SATS[2].2, cno_dbhz: if valid { FAKE_SATS[2].3 } else { 0 } },
                    GsvSat { prn: FAKE_SATS[3].0, elevation_deg: FAKE_SATS[3].1,
                             azimuth_deg: FAKE_SATS[3].2, cno_dbhz: if valid { FAKE_SATS[3].3 } else { 0 } },
                ];
                let slice = if valid { &sats_vec[..] } else { &sats_vec[..0] };
                let n = build_gsv(&mut buf, *talker, 1, 1, view, slice, *sig);
                if n > 0 { enqueue(&buf[..n]); }
            }
        }

        // PNOISE every 10 ticks (~0.5 Hz).
        if counter % 10 == 0 {
            enqueue(b"$PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0*37\r\n");
        }

        // RTCM + ExtRTCM only when we have a fix — matches real chip behaviour.
        if valid {
            if counter % 5 == 0 { enqueue(unicore::rtcm_samples::FRAME_MSG_1077); }
            if counter % 3 == 0 { enqueue(unicore::rtcm_samples::FRAME_MSG_1097); }
            if counter % 3 == 1 { enqueue(unicore::rtcm_samples::FRAME_MSG_1019); }
            if counter % 8 == 0 { enqueue(unicore::rtcm_samples::FRAME_MSG_1046); }
            if counter % 5 == 2 { enqueue(unicore::rtcm_samples::FRAME_MSG_1013); }
            if counter % 5 == 3 { enqueue(unicore::rtcm_samples::CW_OUT_SAMPLE); }

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

            if counter % 85 == 40  { enqueue(unicore::rtcm_samples::PPS_INFO_SAMPLE); }
            if counter % 300 == 120 { enqueue(unicore::rtcm_samples::SVEPH_SAMPLE); }
        }
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

    let mut ticker = Ticker::every(Duration::from_millis(125));
    let mut phase: u8 = 0;
    loop {
        phase = phase.wrapping_add(1);
        let mode = OperatingMode::load();
        let spoofed = SPOOF_DETECTED.load(Ordering::Acquire);

        // Spoof detection in Passthrough/PassthroughOffset: fast-blink red.
        let value = if spoofed
            && matches!(mode, OperatingMode::Passthrough | OperatingMode::PassthroughOffset)
        {
            if phase.is_multiple_of(2) { RGB8::new(60, 0, 0) } else { RGB8::new(0, 0, 0) }
        } else {
            let colour = match mode {
                OperatingMode::Emulation       => RGB8::new(40, 20, 0),  // orange
                OperatingMode::Passthrough     => RGB8::new(0, 30, 30),  // cyan
                OperatingMode::PassthroughRaw  => RGB8::new(30, 0, 30),  // magenta
                OperatingMode::PassthroughOffset => RGB8::new(40, 30, 0), // yellow
            };
            // Slower base blink (~1 Hz: on 4 ticks, off 4 ticks).
            if phase % 8 < 4 { colour } else { RGB8::new(0, 0, 0) }
        };
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
