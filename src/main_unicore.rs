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

/// Last-known-good coordinates — saved on the spoof-onset edge (2 s prior
/// fix). Drone receives these while SPOOF_DETECTED is set, same semantics
/// as the u-blox build.
static LAST_GOOD_LAT: portable_atomic::AtomicI32 = portable_atomic::AtomicI32::new(0);
static LAST_GOOD_LON: portable_atomic::AtomicI32 = portable_atomic::AtomicI32::new(0);
static LAST_GOOD_ALT: portable_atomic::AtomicI32 = portable_atomic::AtomicI32::new(0);

/// Timestamp (ms since boot) when spoof recovery started. 0 = not recovering.
/// Recovery confirmed only after 5 s of clean fixes (u-blox parity).
static SPOOF_RECOVERY_START_MS: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(0);

/// Set when the mode flips — passthrough loop resets detector + pos_buffer
/// + dynamic_offset so stale LAST_GOOD cannot bleed into the new mode.
static SPOOF_DETECTOR_RESET: AtomicBool = AtomicBool::new(false);

/// Detector instance — lives for the whole session, reset via `.reset()` on
/// mode changes that would otherwise pollute its internal `LAST_GOOD`.
static SPOOF_CELL: Mutex<CriticalSectionRawMutex, Option<spoof_detector::SpoofDetector>> =
    Mutex::new(None);

// ---------------------------------------------------------------------------
// Position history ring-buffer + dynamic-offset state (inlined from the
// u-blox build's `passthrough::PositionBuffer` / `DynamicOffset`, which live
// in a UBX-only module we don't pull into this crate).
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
struct PositionEntry {
    lat: i32,
    lon: i32,
    alt: i32,
    ts_ms: u32,
}

struct PositionBuffer {
    entries: [PositionEntry; 15], // 3 s @ 5 Hz, matches u-blox
    write_idx: usize,
    count: usize,
}

impl PositionBuffer {
    const fn new() -> Self {
        Self {
            entries: [PositionEntry { lat: 0, lon: 0, alt: 0, ts_ms: 0 }; 15],
            write_idx: 0,
            count: 0,
        }
    }

    fn push(&mut self, lat: i32, lon: i32, alt: i32, ts_ms: u32) {
        self.entries[self.write_idx] = PositionEntry { lat, lon, alt, ts_ms };
        self.write_idx = (self.write_idx + 1) % 15;
        if self.count < 15 {
            self.count += 1;
        }
    }

    fn get_position_at(&self, seconds_ago: u32, now_ms: u32) -> Option<(i32, i32, i32)> {
        if self.count == 0 {
            return None;
        }
        let target = now_ms.wrapping_sub(seconds_ago * 1000);
        let mut best = 0usize;
        let mut best_diff = u32::MAX;
        for i in 0..self.count {
            let idx = (self.write_idx + 15 - 1 - i) % 15;
            let e = &self.entries[idx];
            let diff = e.ts_ms.abs_diff(target);
            if diff < best_diff {
                best_diff = diff;
                best = idx;
            }
        }
        let e = &self.entries[best];
        Some((e.lat, e.lon, e.alt))
    }
}

#[derive(Clone, Copy)]
struct DynamicOffset {
    lat_1e7: i32,
    lon_1e7: i32,
    alt_mm: i32,
}

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

    // Seed LAST_GOOD with the emulation-mode default position so spoof
    // replacement has something sane to point at before any real fix.
    LAST_GOOD_LAT.store(config::default_position::LAT_1E7, Ordering::Release);
    LAST_GOOD_LON.store(config::default_position::LON_1E7, Ordering::Release);
    LAST_GOOD_ALT.store(config::default_position::ALT_MM, Ordering::Release);

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

#[embassy_executor::task]
async fn passthrough_forward_task() {
    use unicore::nmea;

    let mut asm: nmea::LineAssembler<256> = nmea::LineAssembler::new();
    let mut pass_buf = heapless::Vec::<u8, 256>::new();
    let mut in_sentence = false;
    let mut last_mode = OperatingMode::load();

    // Lazy — created on first entry into Passthrough/PassthroughOffset.
    let mut pos_buffer: Option<PositionBuffer> = None;
    let mut dynamic_offset: Option<DynamicOffset> = None;

    loop {
        let chunk = RAW_RX_CHANNEL.receive().await;
        let mode = OperatingMode::load();

        // Mode change → cause a full reset on the next detector tick.
        if mode != last_mode {
            SPOOF_DETECTOR_RESET.store(true, Ordering::Release);
            last_mode = mode;
        }

        // Process the reset flag (also set explicitly by the button task).
        if SPOOF_DETECTOR_RESET.swap(false, Ordering::AcqRel) {
            asm = nmea::LineAssembler::new();
            pass_buf.clear();
            in_sentence = false;
            SPOOF_DETECTED.store(false, Ordering::Release);
            SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
            dynamic_offset = None;
            if let Some(ref mut pb) = pos_buffer {
                *pb = PositionBuffer::new();
            }
            let mut d = SPOOF_CELL.lock().await;
            if let Some(ref mut det) = *d {
                *det = spoof_detector::SpoofDetector::default();
            }
        }

        match mode {
            OperatingMode::Emulation => {
                // Drop incoming chip bytes while we are emitting our own.
            }
            OperatingMode::PassthroughRaw => {
                let mut v = heapless::Vec::<u8, 1280>::new();
                let _ = v.extend_from_slice(&chunk);
                let _ = TX_CHANNEL.try_send(v);
            }
            OperatingMode::Passthrough | OperatingMode::PassthroughOffset => {
                // Lazy init on first entry.
                if pos_buffer.is_none() {
                    pos_buffer = Some(PositionBuffer::new());
                }
                let apply_offset = mode == OperatingMode::PassthroughOffset;

                for &b in chunk.iter() {
                    if !in_sentence && b != b'$' {
                        if pass_buf.push(b).is_err() {
                            let mut v = heapless::Vec::<u8, 1280>::new();
                            let _ = v.extend_from_slice(&pass_buf);
                            let _ = TX_CHANNEL.try_send(v);
                            pass_buf.clear();
                            let _ = pass_buf.push(b);
                        }
                        continue;
                    }
                    if b == b'$' && !pass_buf.is_empty() {
                        let mut v = heapless::Vec::<u8, 1280>::new();
                        let _ = v.extend_from_slice(&pass_buf);
                        let _ = TX_CHANNEL.try_send(v);
                        pass_buf.clear();
                    }
                    if b == b'$' { in_sentence = true; }

                    if let Some(line) = asm.feed(b) {
                        in_sentence = false;
                        process_nmea_line(
                            line,
                            apply_offset,
                            pos_buffer.as_mut().unwrap(),
                            &mut dynamic_offset,
                        ).await;
                    }
                }
            }
        }
    }
}

/// Port of the u-blox `gnss_processing_task` NAV-PVT branch to NMEA.
/// Mirror for mirror:
///   - Only GGA feeds the detector (it carries lat/lon/alt/nsats like NAV-PVT).
///   - RMC is a coordinate-only duplicate; we rewrite it but never analyse it.
///   - PositionBuffer stores 3 s of 3D-fix history, used to look up the
///     "good" position 2 s before the spoof edge.
///   - 5 s recovery timer before clearing SPOOF_DETECTED.
///   - Dynamic offset computed once at the first valid fix; never recomputed.
///   - While in PassthroughOffset with the offset still unknown, coordinate-
///     carrying messages are **suppressed** (not forwarded) to avoid leaking
///     the real position. Non-coordinate messages (GSA/GSV/VTG/…) pass.
async fn process_nmea_line(
    line: &[u8],
    apply_offset: bool,
    pos_buffer: &mut PositionBuffer,
    dynamic_offset: &mut Option<DynamicOffset>,
) {
    use unicore::nmea;

    let now_ms = embassy_time::Instant::now().as_millis() as u32;
    let is_gga = line.len() >= 6 && &line[3..6] == b"GGA";
    let is_rmc = line.len() >= 6 && &line[3..6] == b"RMC";

    // --- Detector + pos_buffer: GGA only -------------------------------------
    if is_gga {
        if let Some(fix) = nmea::parse_gga(line) {
            if fix.checksum_ok && fix.fix_quality > 0 {
                // Store only valid 3D-fix samples (u-blox parity: skip no-fix).
                pos_buffer.push(fix.lat_1e7, fix.lon_1e7, fix.alt_mm, now_ms);

                let pos = spoof_detector::Position {
                    lat: fix.lat_1e7,
                    lon: fix.lon_1e7,
                    alt_mm: fix.alt_mm,
                    time_ms: now_ms,
                    fix_type: spoof_detector::FixType::Fix3D,
                    h_acc_mm: 0,
                    num_sv: fix.nsats,
                    pdop: 100,
                    gnss_time: None,
                    cno_values: heapless::Vec::new(),
                };

                let mut d = SPOOF_CELL.lock().await;
                if let Some(ref mut det) = *d {
                    let result = det.analyze(pos);
                    let was_spoofed = SPOOF_DETECTED.load(Ordering::Acquire);
                    let is_spoofed = result == spoof_detector::AnalysisResult::Spoofed;

                    if is_spoofed && !was_spoofed {
                        // Edge: capture the position from 2 s ago as LAST_GOOD.
                        if let Some((gl, go, ga)) = pos_buffer.get_position_at(2, now_ms) {
                            LAST_GOOD_LAT.store(gl, Ordering::Release);
                            LAST_GOOD_LON.store(go, Ordering::Release);
                            LAST_GOOD_ALT.store(ga, Ordering::Release);
                        }
                        SPOOF_DETECTED.store(true, Ordering::Release);
                        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                    } else if !is_spoofed && was_spoofed {
                        let start = SPOOF_RECOVERY_START_MS.load(Ordering::Acquire);
                        if start == 0 {
                            SPOOF_RECOVERY_START_MS.store(now_ms, Ordering::Release);
                        } else {
                            let elapsed = now_ms.wrapping_sub(start);
                            if elapsed >= 5000 {
                                SPOOF_DETECTED.store(false, Ordering::Release);
                                SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                                // dynamic_offset stays — computed once at takeoff,
                                // must remain constant for the whole flight.
                            }
                        }
                    } else if is_spoofed {
                        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                    }
                }

                // Compute dynamic offset at first real 3D fix (Offset mode only).
                if apply_offset && dynamic_offset.is_none() && fix.nsats >= 4 {
                    *dynamic_offset = Some(DynamicOffset {
                        lat_1e7: config::offset_target::LAT_1E7.saturating_sub(fix.lat_1e7),
                        lon_1e7: config::offset_target::LON_1E7.saturating_sub(fix.lon_1e7),
                        alt_mm:  config::offset_target::ALT_MM.saturating_sub(fix.alt_mm),
                    });
                }
            }
        }
    }

    let spoofed = SPOOF_DETECTED.load(Ordering::Acquire);

    // --- Decide what coords to emit in GGA/RMC -------------------------------
    //
    //   spoofed           → LAST_GOOD (+ offset in PassthroughOffset)
    //   !spoofed, Offset  → actual + offset   (suppress if offset not computed)
    //   !spoofed, plain   → verbatim
    //
    if is_gga || is_rmc {
        // Determine target coordinates for this message.
        let coords: Option<(i32, i32, i32)> = if spoofed {
            let base = (
                LAST_GOOD_LAT.load(Ordering::Acquire),
                LAST_GOOD_LON.load(Ordering::Acquire),
                LAST_GOOD_ALT.load(Ordering::Acquire),
            );
            if let Some(ref off) = *dynamic_offset {
                Some((
                    base.0.saturating_add(off.lat_1e7),
                    base.1.saturating_add(off.lon_1e7),
                    base.2.saturating_add(off.alt_mm),
                ))
            } else {
                Some(base)
            }
        } else if apply_offset {
            match (*dynamic_offset, nmea::parse_gga(line).or_else(|| nmea::parse_rmc(line))) {
                (Some(off), Some(fix)) if fix.checksum_ok => Some((
                    fix.lat_1e7.saturating_add(off.lat_1e7),
                    fix.lon_1e7.saturating_add(off.lon_1e7),
                    fix.alt_mm.saturating_add(off.alt_mm),
                )),
                (None, _) => {
                    // Offset not yet computed — suppress, don't leak real position.
                    return;
                }
                _ => None,
            }
        } else {
            None // plain Passthrough: forward verbatim
        };

        if let Some((lat, lon, alt)) = coords {
            // u-blox parity: under spoof, rebuild GGA/RMC with degraded status
            //   GGA: fix_quality=0, nsats=92 (spoof marker), hdop high
            //   RMC: status='V' invalid, sog/cog=0, mode='N'
            // Coordinates still come out as LAST_GOOD(+offset) so the drone sees
            // a "last-known position, fix lost" state and enters failsafe.
            if spoofed {
                let mut buf = [0u8; 256];
                if is_gga {
                    let existing = nmea::parse_gga(line).unwrap_or_default();
                    let g = nmea::GgaFields {
                        time: existing.time,
                        lat_1e7: lat, lon_1e7: lon,
                        fix_quality: 0,
                        nsats: 92,
                        hdop_x100: 9999,
                        alt_mm: alt,
                        geoid_sep_mm: -30_000,
                    };
                    let n = nmea::build_gga(&mut buf, nmea::Talker::Gn, &g);
                    if n > 0 {
                        let mut v = heapless::Vec::<u8, 1280>::new();
                        let _ = v.extend_from_slice(&buf[..n]);
                        let _ = TX_CHANNEL.try_send(v);
                    }
                    return;
                }
                if is_rmc {
                    let existing = nmea::parse_rmc(line).unwrap_or_default();
                    let r = nmea::RmcFields {
                        time: existing.time,
                        valid: false,
                        lat_1e7: lat, lon_1e7: lon,
                        sog_knots_x1000: 0,
                        cog_deg_x100: 0,
                        date: existing.date,
                        mode: b'N',
                    };
                    let n = nmea::build_rmc(&mut buf, nmea::Talker::Gn, &r);
                    if n > 0 {
                        let mut v = heapless::Vec::<u8, 1280>::new();
                        let _ = v.extend_from_slice(&buf[..n]);
                        let _ = TX_CHANNEL.try_send(v);
                    }
                    return;
                }
            }

            // Non-spoof: plain coordinate rewrite (offset mode).
            let mut work = [0u8; 320];
            if line.len() <= work.len() {
                work[..line.len()].copy_from_slice(line);
                if let Some(new_len) = nmea::rewrite_position_inplace(
                    &mut work,
                    line.len(),
                    lat,
                    lon,
                    if is_gga { Some(alt) } else { None },
                ) {
                    let mut v = heapless::Vec::<u8, 1280>::new();
                    let _ = v.extend_from_slice(&work[..new_len]);
                    let _ = TX_CHANNEL.try_send(v);
                    return;
                }
            }
        }
    }

    // Non-GGA/RMC, or rewrite skipped — forward verbatim.
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

        // Fix is reported from the very first tick — no warm-up.
        let valid = true;
        let fix_quality: u8 = 1;
        let nsats: u8 = 16;

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
        // PRN split below totals 16 satellites (matches GGA nsats=16).
        if counter % 5 == 0 {
            let sys_prns: &[(u8, &[u16])] = &[
                (1, &[5, 13, 15, 20]),      // GPS — 4
                (2, &[65, 70, 75]),          // GLONASS — 3
                (3, &[2, 11, 18]),           // Galileo — 3
                (4, &[6, 14, 20, 27]),       // BeiDou — 4
                (5, &[193, 194]),            // QZSS — 2
            ];
            for (sys_id, prn_list) in sys_prns {
                let mut sats: [u16; 12] = [0; 12];
                for (i, &prn) in prn_list.iter().enumerate().take(12) {
                    sats[i] = prn;
                }
                let gsa = GsaFields {
                    op_mode: GsaOpMode::Automatic,
                    fix_type: 3,
                    sats,
                    pdop_x100: 99,
                    hdop_x100: 99,
                    vdop_x100: 99,
                    system_id: *sys_id,
                };
                let n = build_gsa(&mut buf, Talker::Gn, &gsa);
                if n > 0 { enqueue(&buf[..n]); }
            }
        }

        // GSV every 5th tick (rate=5): per-talker pages advertising the same
        // 16 SVs split across constellations. View count matches GGA nsats.
        if counter % 5 == 0 {
            // (talker, signal_id, satellites for this talker's view)
            let pages: &[(Talker, u8, &[(u16, u8, u16, u8)])] = &[
                (Talker::Gp, 1, &[(5, 45, 90, 45), (13, 60, 180, 47), (15, 30, 270, 42), (20, 80, 0, 50)]),
                (Talker::Gb, 1, &[(6, 35, 60, 43), (14, 50, 150, 45), (20, 70, 210, 44), (27, 25, 300, 41)]),
                (Talker::Ga, 7, &[(2, 40, 45, 46), (11, 55, 135, 48), (18, 28, 225, 41)]),
                (Talker::Gl, 1, &[(65, 42, 110, 44), (70, 68, 200, 46), (75, 22, 330, 40)]),
                (Talker::Gq, 1, &[(193, 55, 180, 47), (194, 40, 240, 43)]),
            ];
            for (talker, sig, sats) in pages {
                let mut arr: [GsvSat; 4] = [GsvSat::default(); 4];
                for (i, s) in sats.iter().enumerate().take(4) {
                    arr[i] = GsvSat {
                        prn: s.0,
                        elevation_deg: s.1,
                        azimuth_deg: s.2,
                        cno_dbhz: s.3,
                    };
                }
                let n = build_gsv(&mut buf, *talker, 1, 1, nsats, &arr[..sats.len().min(4)], *sig);
                if n > 0 { enqueue(&buf[..n]); }
            }
        }

        // PNOISE every 10 ticks (~0.5 Hz).
        if counter % 10 == 0 {
            enqueue(b"$PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0*37\r\n");
        }

        // RTCM + ExtRTCM stream alongside NMEA (fix is immediate now).
        {
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
                    SPOOF_DETECTOR_RESET.store(true, Ordering::Release);
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
