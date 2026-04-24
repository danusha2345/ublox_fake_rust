//! UC6580I emulator entry point (compile with `--features unicore --bin ublox_fake_uc`).
//!
//! Supports the same four operating modes as the u-blox build:
//!   0 Emulation          — fake UC6580I stream: boot-dump + NMEA + ExtRTCM 4074.
//!   1 Passthrough        — forward UART1 (real UC6580I) → UART0 (drone); reassembles
//!                          `$GxGGA`/`$GxRMC` to run the shared spoof detector and,
//!                          when spoofing is flagged, rebuilds GGA/RMC with the
//!                          `SPOOF_NSATS_MARKER` fingerprint. RTCM3 frames (incl.
//!                          proprietary msg 4074) pass byte-for-byte.
//!   2 PassthroughRaw     — byte-for-byte forward, no parsing or spoof detection.
//!   3 PassthroughOffset  — same as Passthrough, plus rewrites GGA/RMC latitude/
//!                          longitude to `config::offset_target` on the fly.
//!
//! Mode is persisted in the same flash slot as the u-blox build and changed
//! by clicking the mode button (GPIO13 power, GPIO14 sense) 1..4 times.

#![no_std]
#![no_main]

mod config;
mod coordinates;
mod flash_storage;
mod pos_history;
mod spoof_detector;
#[path = "unicore/mod.rs"]
mod unicore;

#[allow(dead_code)]
mod version {
    include!(concat!(env!("OUT_DIR"), "/version.rs"));
}

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
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
use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
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

static MODE: AtomicU8 = AtomicU8::new(OperatingMode::Passthrough as u8);

// ---------------------------------------------------------------------------
// Shared channels
// ---------------------------------------------------------------------------

static TX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 1280>, 32> = Channel::new();
static RAW_RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 256>, 64> = Channel::new();

static OUTPUT_START_MILLIS: AtomicU32 = AtomicU32::new(0);
static SATELLITES_INVALID: AtomicBool = AtomicBool::new(false);

/// Boot-handshake capture: collect the first N checksum-ok config NMEA lines
/// (all non-streaming sentences — i.e. not GGA/RMC/GSA/GSV/GLL/VTG/ZDA/TXT/
/// PNOISE) in each direction into a RAM buffer, then stop. `boot_log_dump_task`
/// periodically prints the buffer so an `attach` after boot still shows the
/// captured handshake.
const BOOT_LOG_MAX: u8 = 15;
const BOOT_LOG_LINE_MAX: usize = 128;
const BOOT_LOG_TOTAL: usize = 3 * BOOT_LOG_MAX as usize;

#[derive(Clone, Copy)]
struct BootLogLine {
    direction: u8,
    len: u8,
    buf: [u8; BOOT_LOG_LINE_MAX],
}

impl BootLogLine {
    const EMPTY: Self = Self {
        direction: 0,
        len: 0,
        buf: [0; BOOT_LOG_LINE_MAX],
    };
}

struct BootLogBuf {
    entries: [BootLogLine; BOOT_LOG_TOTAL],
    count: usize,
}

impl BootLogBuf {
    const fn new() -> Self {
        Self {
            entries: [BootLogLine::EMPTY; BOOT_LOG_TOTAL],
            count: 0,
        }
    }
}

static BOOT_LOG: BlockingMutex<CriticalSectionRawMutex, RefCell<BootLogBuf>> =
    BlockingMutex::new(RefCell::new(BootLogBuf::new()));
static BOOT_LOG_DRONE_TO_CHIP: AtomicU8 = AtomicU8::new(0);
static BOOT_LOG_CHIP_TO_US: AtomicU8 = AtomicU8::new(0);
static BOOT_LOG_US_TO_DRONE: AtomicU8 = AtomicU8::new(0);

/// Append a captured line (already trimmed of CR/LF) into `BOOT_LOG`.
/// Direction: 0 = DRONE→CHIP, 1 = CHIP→US, 2 = US→DRONE.
fn boot_log_push(direction: u8, line: &[u8]) {
    BOOT_LOG.lock(|m| {
        let mut log = m.borrow_mut();
        if log.count >= BOOT_LOG_TOTAL {
            return;
        }
        let idx = log.count;
        let n = line.len().min(BOOT_LOG_LINE_MAX);
        log.entries[idx].direction = direction;
        log.entries[idx].len = n as u8;
        log.entries[idx].buf[..n].copy_from_slice(&line[..n]);
        log.count += 1;
    });
}

/// Periodically dump the entire boot log to RTT so late `probe-rs attach`
/// sessions can still see the handshake after the defmt ring has wrapped.
#[embassy_executor::task]
async fn boot_log_dump_task() {
    Timer::after(Duration::from_secs(4)).await;
    loop {
        BOOT_LOG.lock(|m| {
            let log = m.borrow();
            if log.count == 0 {
                return;
            }
            info!("=== BOOT LOG ({} entries) ===", log.count as u32);
            for i in 0..log.count {
                let e = &log.entries[i];
                let tag = match e.direction {
                    0 => "DRONE→CHIP",
                    1 => "CHIP→US",
                    _ => "US→DRONE",
                };
                let data = &e.buf[..e.len as usize];
                info!("#{} [{}] {=[u8]:a}", (i + 1) as u32, tag, data);
            }
            info!("=== END BOOT LOG ===");
        });
        Timer::after(Duration::from_secs(15)).await;
    }
}

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

use spoof_detector::SPOOF_NSATS_MARKER;

/// Minimum clean-fix window before we drop SPOOF_DETECTED back to false
/// (u-blox parity: one good reading is not enough).
const SPOOF_RECOVERY_TIMEOUT_MS: u32 = 5_000;

/// When spoofing starts, LAST_GOOD captures the fix from this many seconds
/// before the detector fired — jitter-resistant "pre-spoof" snapshot.
const SPOOF_LOOKBACK_SECONDS: u32 = 2;

/// Large HDOP value planted in spoofed GGA (tells the drone the fix is poor).
const SPOOF_HIGH_HDOP_X100: u16 = 9_999;

/// Placeholder geoid separation for rebuilt GGA under spoof. Real value would
/// require the original sentence's geoid_sep to round-trip, which the
/// rebuild path does not carry.
const SPOOF_DEFAULT_GEOID_SEP_MM: i32 = -30_000;

/// Detector instance — lives for the whole session, reset via `.reset()` on
/// mode changes that would otherwise pollute its internal `LAST_GOOD`.
static SPOOF_CELL: Mutex<CriticalSectionRawMutex, Option<spoof_detector::SpoofDetector>> =
    Mutex::new(None);

use pos_history::{DynamicOffset, PositionBuffer};

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
            info!("New firmware version, resetting mode to Passthrough");
            OperatingMode::Passthrough.store();
            flash_storage::save_mode(&mut flash, OperatingMode::Passthrough as u8).await;
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

    // Same electrical UART0 TX settings as the u-blox build. Default pad drive
    // is marginal at 921600 baud on the drone harness and can show up as
    // garbage in every Unicore mode, including raw passthrough.
    rp_pac::PADS_BANK0.gpio(0).modify(|w| {
        w.set_drive(rp_pac::pads::vals::Drive::_12M_A);
        w.set_pde(true);
        w.set_slewfast(true);
    });
    info!("GPIO0 (UART0 TX): 12mA drive, pull-down, fast slew");

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
    // Embassy's default RX FIFO interrupt threshold is too high for the dense
    // UC6580I NMEA+RTCM stream at 921600 baud. Keep it aligned with the
    // u-blox passthrough path and key-extraction path.
    rp_pac::UART1.uartifls().write(|w| {
        w.set_rxiflsel(0b001); // 1/4 full (4 bytes)
        w.set_txiflsel(0b000);
    });
    info!("UART1 RX FIFO threshold set to 1/4 (4 bytes)");
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
    spawner.must_spawn(boot_log_dump_task());

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
    let mut diag_asm: unicore::nmea::LineAssembler<320> = unicore::nmea::LineAssembler::new();
    loop {
        let frame = TX_CHANNEL.receive().await;
        // Boot-handshake capture: log only the first BOOT_LOG_MAX config lines
        // (non-streaming sentences). Guarded by counter so it's a no-op once
        // capped.
        if BOOT_LOG_US_TO_DRONE.load(Ordering::Relaxed) < BOOT_LOG_MAX {
            for &b in frame.iter() {
                if let Some(line) = diag_asm.feed(b) {
                    if is_config_line(line)
                        && BOOT_LOG_US_TO_DRONE.load(Ordering::Relaxed) < BOOT_LOG_MAX
                    {
                        let n = BOOT_LOG_US_TO_DRONE.fetch_add(1, Ordering::Relaxed);
                        if n < BOOT_LOG_MAX {
                            let trimmed = strip_crlf(line);
                            info!("[US→DRONE #{}] {=[u8]:a}", (n + 1) as u32, trimmed);
                            boot_log_push(2, trimmed);
                        }
                    }
                }
            }
        }
        if let Err(e) = tx.write_all(&frame).await {
            warn!("uart0 tx error: {:?}", defmt::Debug2Format(&e));
        }
    }
}

/// Strip trailing `\r\n` for prettier log output.
fn strip_crlf(line: &[u8]) -> &[u8] {
    let mut end = line.len();
    while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
        end -= 1;
    }
    &line[..end]
}

/// Is this NMEA line part of the config/boot handshake (as opposed to periodic
/// streaming data)? Filters out high-rate sentences the chip emits at 5 Hz.
fn is_config_line(line: &[u8]) -> bool {
    let body = strip_crlf(line);
    if body.len() < 4 || body[0] != b'$' {
        return false;
    }
    // Proprietary sentences ($P...) are interesting (PDTINFO, PSMT, …) except
    // for PNOISE which is a periodic telemetry stream.
    if body[1] == b'P' {
        return body.len() < 7 || &body[1..7] != *b"PNOISE";
    }
    // Standard NMEA: skip the noisy periodic sentences.
    if body.len() < 6 {
        return true;
    }
    match &body[3..6] {
        b"GGA" | b"RMC" | b"GSA" | b"GSV" | b"GLL" | b"VTG" | b"ZDA" | b"TXT" => false,
        _ => true,
    }
}

// ---------------------------------------------------------------------------
// UART0 RX — parse ASCII commands (active in Emulation mode), dispatch replies.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn uart0_rx_task(mut rx: BufferedUartRx) {
    let mut buf = [0u8; 128];
    let mut asm: unicore::nmea::LineAssembler<256> = unicore::nmea::LineAssembler::new();
    let mut diag_asm: unicore::nmea::LineAssembler<256> = unicore::nmea::LineAssembler::new();
    loop {
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                // Boot capture from drone bus (config lines only, capped).
                if BOOT_LOG_DRONE_TO_CHIP.load(Ordering::Relaxed) < BOOT_LOG_MAX {
                    for &b in &buf[..n] {
                        if let Some(line) = diag_asm.feed(b) {
                            if is_config_line(line)
                                && BOOT_LOG_DRONE_TO_CHIP.load(Ordering::Relaxed) < BOOT_LOG_MAX
                            {
                                let cnt = BOOT_LOG_DRONE_TO_CHIP.fetch_add(1, Ordering::Relaxed);
                                if cnt < BOOT_LOG_MAX {
                                    let trimmed = strip_crlf(line);
                                    info!("[DRONE→CHIP #{}] {=[u8]:a}", (cnt + 1) as u32, trimmed);
                                    boot_log_push(0, trimmed);
                                }
                            }
                        }
                    }
                }
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

/// Append one byte to `buf`; flush to TX if full, then retry. Used by the
/// RTCM/NMEA stream router to forward bytes without allocating.
#[inline]
fn buffered_forward(buf: &mut heapless::Vec<u8, 256>, byte: u8) {
    if buf.push(byte).is_err() {
        enqueue(buf);
        buf.clear();
        let _ = buf.push(byte);
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
    let mut diag_asm: unicore::nmea::LineAssembler<256> = unicore::nmea::LineAssembler::new();
    loop {
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                // Boot capture of config replies from the chip (capped; no-op
                // once we've gathered BOOT_LOG_MAX).
                if BOOT_LOG_CHIP_TO_US.load(Ordering::Relaxed) < BOOT_LOG_MAX {
                    for &b in &buf[..n] {
                        if let Some(line) = diag_asm.feed(b) {
                            if is_config_line(line)
                                && BOOT_LOG_CHIP_TO_US.load(Ordering::Relaxed) < BOOT_LOG_MAX
                            {
                                let cnt = BOOT_LOG_CHIP_TO_US.fetch_add(1, Ordering::Relaxed);
                                if cnt < BOOT_LOG_MAX {
                                    let trimmed = strip_crlf(line);
                                    info!("[CHIP→US #{}] {=[u8]:a}", (cnt + 1) as u32, trimmed);
                                    boot_log_push(1, trimmed);
                                }
                            }
                        }
                    }
                }
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

    // RTCM3 framing state (preamble 0xD3 + 2-byte big-endian length-header + payload
    // + 3-byte CRC24). Required so that a stray `$` (0x24) inside an RTCM payload
    // is NOT misread as the start of an NMEA sentence — that would hand binary
    // bytes to `asm.feed()` and lose them. RTCM wraps UC6580I proprietary
    // msg 4074, so this state covers both.
    let mut rtcm_hdr_have: u8 = 0;    // bytes of the 3-byte header already consumed
    let mut rtcm_hdr1: u8 = 0;        // header byte 1 (high bits of length)
    let mut rtcm_remaining: u16 = 0;  // payload + CRC bytes still to forward

    // Lazy — created on first entry into Passthrough/PassthroughOffset.
    let mut pos_buffer: Option<PositionBuffer> = None;
    let mut dynamic_offset: Option<DynamicOffset> = None;
    // Date from $GxRMC + midnight-rollover guard. Combined with the HH:MM:SS
    // from GGA to build `spoof_detector::GnssTime`, which enables time-jump
    // and system-clock-drift checks in the shared detector.
    let mut time_cache = NmeaTimeCache::default();

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
            rtcm_hdr_have = 0;
            rtcm_remaining = 0;
            time_cache = NmeaTimeCache::default();
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
                enqueue(&chunk);
            }
            OperatingMode::Passthrough | OperatingMode::PassthroughOffset => {
                if pos_buffer.is_none() {
                    pos_buffer = Some(PositionBuffer::new());
                }
                let apply_offset = mode == OperatingMode::PassthroughOffset;

                for &b in chunk.iter() {
                    // --- Inside an RTCM frame: forward payload/CRC verbatim. --
                    if rtcm_remaining > 0 {
                        buffered_forward(&mut pass_buf, b);
                        rtcm_remaining -= 1;
                        // Frame boundary = TX boundary: flush immediately so small
                        // RTCM frames (e.g. 1013 system params) don't sit in
                        // pass_buf waiting for the next `$` or a 256-byte fill.
                        if rtcm_remaining == 0 && !pass_buf.is_empty() {
                            enqueue(&pass_buf);
                            pass_buf.clear();
                        }
                        continue;
                    }

                    // --- Reading the 3-byte RTCM header (D3 + 2 length bytes) --
                    if rtcm_hdr_have == 1 {
                        // Byte index 1 of the header: top 6 bits MUST be zero
                        // (RTCM3 reserved). If not, the `0xD3` was noise/desync,
                        // not a real preamble. Abort RTCM state and re-route
                        // this byte through the NMEA logic below so that a `$`
                        // appearing here still starts a sentence correctly.
                        if b & 0xFC != 0 {
                            rtcm_hdr_have = 0;
                            // fall through without consuming `b`
                        } else {
                            buffered_forward(&mut pass_buf, b);
                            rtcm_hdr1 = b;
                            rtcm_hdr_have = 2;
                            continue;
                        }
                    } else if rtcm_hdr_have == 2 {
                        buffered_forward(&mut pass_buf, b);
                        // Payload length = low 10 bits of bytes [1..2] big-endian.
                        let payload_len = (((rtcm_hdr1 as u16) & 0x03) << 8) | (b as u16);
                        // Remaining = payload + 3-byte CRC24 (header already forwarded).
                        rtcm_remaining = payload_len + 3;
                        rtcm_hdr_have = 0;
                        continue;
                    }

                    // --- RTCM3 preamble only outside of an active NMEA sentence --
                    if !in_sentence && b == 0xD3 {
                        buffered_forward(&mut pass_buf, b);
                        rtcm_hdr_have = 1;
                        continue;
                    }

                    // --- NMEA routing ------------------------------------------
                    if !in_sentence && b != b'$' {
                        buffered_forward(&mut pass_buf, b);
                        continue;
                    }
                    if b == b'$' && !pass_buf.is_empty() {
                        enqueue(&pass_buf);
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
                            &mut time_cache,
                        ).await;
                    }
                }
            }
        }
    }
}

/// Build a spoofed GGA frame (degraded fix_quality + SPOOF_NSATS_MARKER). Returns
/// written byte count in `out`, or 0 on build failure.
fn build_spoofed_gga(existing: &unicore::nmea::NmeaFix, coords: (i32, i32, i32), out: &mut [u8]) -> usize {
    let g = unicore::nmea::GgaFields {
        time: existing.time,
        lat_1e7: coords.0, lon_1e7: coords.1,
        fix_quality: 0,
        nsats: SPOOF_NSATS_MARKER,
        hdop_x100: SPOOF_HIGH_HDOP_X100,
        alt_mm: coords.2,
        geoid_sep_mm: SPOOF_DEFAULT_GEOID_SEP_MM,
    };
    unicore::nmea::build_gga(out, unicore::nmea::Talker::Gn, &g)
}

/// Build a spoofed RMC frame (status='V', sog/cog zeroed, mode='N').
fn build_spoofed_rmc(existing: &unicore::nmea::NmeaFix, coords: (i32, i32, i32), out: &mut [u8]) -> usize {
    let r = unicore::nmea::RmcFields {
        time: existing.time,
        valid: false,
        lat_1e7: coords.0, lon_1e7: coords.1,
        sog_knots_x1000: 0,
        cog_deg_x100: 0,
        date: existing.date,
        mode: b'N',
    };
    unicore::nmea::build_rmc(out, unicore::nmea::Talker::Gn, &r)
}

/// Rewrite lat/lon/alt in place on an NMEA sentence. Returns new sentence
/// length on success, `None` when the rewrite failed (bad checksum, buffer
/// too small, …).
fn rewrite_coords(line: &[u8], coords: (i32, i32, i32), is_gga: bool, out: &mut [u8; 320]) -> Option<usize> {
    if line.len() > out.len() {
        return None;
    }
    out[..line.len()].copy_from_slice(line);
    unicore::nmea::rewrite_position_inplace(
        out,
        line.len(),
        coords.0,
        coords.1,
        if is_gga { Some(coords.2) } else { None },
    )
}

/// Port of the u-blox `gnss_processing_task` NAV-PVT branch to NMEA.
/// Mirror for mirror:
///   - Only GGA feeds the detector (it carries lat/lon/alt/nsats like NAV-PVT).
///   - RMC is a coordinate-only duplicate; we rewrite it but never analyse it.
///   - PositionBuffer stores 3 s of 3D-fix history, used to look up the
///     "good" position 2 s before the spoof edge.
///   - `SPOOF_RECOVERY_TIMEOUT_MS` of clean fixes before SPOOF_DETECTED drops.
///   - Dynamic offset computed once at the first valid fix; never recomputed.
///   - While in PassthroughOffset with the offset still unknown, coordinate-
///     carrying messages are **suppressed** (not forwarded) to avoid leaking
///     the real position. Auxiliary sentences (GSA/GSV/ZDA/GST/…) pass; GLL
///     and VTG are dropped while spoof or offset rewrite is active because
///     they would leak the chip's raw lat/lon (GLL) or velocity (VTG).
/// Tracks NMEA time/date state needed to synthesise `spoof_detector::GnssTime`
/// from the combined GGA (HH:MM:SS) + RMC (DD/MM/YY) streams.
///
/// `date_ms`: monotonic ms when `date` was last refreshed. Used to reject
/// stale cached dates (RMC stream paused, e.g. antenna lost fix).
///
/// `last_gga_sod`: previous GGA's second-of-day. Used to detect a midnight
/// rollover: if the new GGA is > 12 h *earlier* than the previous one, we've
/// crossed 00:00 UTC but haven't yet seen the post-midnight RMC — the cached
/// date would be yesterday's, so we skip this tick rather than fire a bogus
/// 24 h backward-jump at the detector.
///
/// Freshness window (1500 ms) covers the default 5 Hz RMC cadence with slack.
#[derive(Default)]
struct NmeaTimeCache {
    date: Option<unicore::nmea::NmeaDate>,
    date_ms: u32,
    last_gga_sod: Option<u32>,
    /// Last frame's GNSS unix timestamp (non-spoofed). Used by the time-jump
    /// secondary to catch backward time shifts on NoFix/V-status frames where
    /// the detector's `check_gnss_time` is gated behind `has_3d_fix`.
    last_frame_gnss_unix: Option<i64>,
}

const NMEA_DATE_FRESHNESS_MS: u32 = 1500;

/// Largest tolerated backward jump between consecutive GNSS timestamps. The
/// u-blox detector uses the same 1 s threshold (`thresholds::MAX_TIME_JUMP_BACK_S`
/// inside `check_gnss_time`). Forward jumps are allowed to be unbounded — chips
/// legitimately fast-forward by the gap duration when re-acquiring after a
/// long fix loss, so flagging forward jumps would produce false positives.
const TIME_JUMP_BACK_S: i64 = 1;

async fn process_nmea_line(
    line: &[u8],
    apply_offset: bool,
    pos_buffer: &mut PositionBuffer,
    dynamic_offset: &mut Option<DynamicOffset>,
    time_cache: &mut NmeaTimeCache,
) {
    use unicore::nmea;

    let now_ms = embassy_time::Instant::now().as_millis() as u32;
    let is_gga = line.len() >= 6 && &line[3..6] == b"GGA";
    let is_rmc = line.len() >= 6 && &line[3..6] == b"RMC";

    // Parse once; reuse for detector, offset-rewrite and spoof-rebuild.
    let parsed: Option<nmea::NmeaFix> = if is_gga { nmea::parse_gga(line) }
        else if is_rmc { nmea::parse_rmc(line) }
        else { None };

    // Cache the RMC date. GGA has no date of its own, so we combine it with
    // the last-seen RMC date to feed `GnssTime` into the shared detector.
    // Accept V-status RMC too — the chip still emits date+time during no-fix
    // ticks, and u-blox parity requires time-jump checks to run on the first
    // post-gap frame even before fq flips to 1. We only gate on structural
    // integrity (checksum + non-zero year), not on fix validity.
    if is_rmc {
        if let Some(fix) = parsed {
            if fix.checksum_ok && fix.date.year > 0 {
                time_cache.date = Some(fix.date);
                time_cache.date_ms = now_ms;
            }
        }
    }

    // --- Detector + pos_buffer: GGA (u-blox parity) --------------------------
    // u-blox calls `detector.analyze()` on every NAV-PVT regardless of fix_type;
    // the detector itself short-circuits on NoFix samples. Match that here: fire
    // on every checksum-ok GGA, passing the fix_type derived from fq. Only push
    // into pos_buffer on 3D-fix ticks to keep LAST_GOOD lookback uncontaminated.
    if is_gga {
        if let Some(fix) = parsed {
            if fix.checksum_ok {
                let fix_type = if fix.fix_quality > 0 {
                    spoof_detector::FixType::Fix3D
                } else {
                    spoof_detector::FixType::NoFix
                };

                if fix_type.has_3d_fix() {
                    pos_buffer.push(fix.lat_1e7, fix.lon_1e7, fix.alt_mm, now_ms);
                }

                // Build GnssTime from GGA time + cached RMC date so that
                // time-jump and system-clock-drift checks run on Unicore too.
                // Two guards:
                //   - stale cache: skip if no fresh RMC within 1500 ms (RMC
                //     stream paused → cached date may be wrong day).
                //   - midnight rollover: if GGA's second-of-day jumped back
                //     by > 12 h, we crossed 00:00 UTC but the RMC date is
                //     still yesterday; skip this tick rather than synthesise
                //     a 24 h backward jump.
                let sod = fix.time.hour as u32 * 3600
                        + fix.time.minute as u32 * 60
                        + fix.time.second as u32;
                let stale_date =
                    time_cache.date.is_none()
                    || now_ms.wrapping_sub(time_cache.date_ms) > NMEA_DATE_FRESHNESS_MS;
                let rolled_over = time_cache.last_gga_sod
                    .map_or(false, |prev| prev > sod && prev - sod > 12 * 3600);
                time_cache.last_gga_sod = Some(sod);

                let gnss_time = if stale_date || rolled_over {
                    None
                } else {
                    time_cache.date.map(|d| spoof_detector::GnssTime {
                        itow_ms: 0,
                        year: d.year,
                        month: d.month,
                        day: d.day,
                        hour: fix.time.hour,
                        min: fix.time.minute,
                        sec: fix.time.second,
                        system_time_ms: now_ms,
                    })
                };

                let pos = spoof_detector::Position {
                    lat: fix.lat_1e7,
                    lon: fix.lon_1e7,
                    alt_mm: fix.alt_mm,
                    time_ms: now_ms,
                    fix_type,
                    h_acc_mm: 0,
                    num_sv: fix.nsats,
                    pdop: 100,
                    gnss_time,
                    cno_values: heapless::Vec::new(),
                };

                let mut d = SPOOF_CELL.lock().await;
                if let Some(ref mut det) = *d {
                    let result = det.analyze(pos);
                    let was_spoofed = SPOOF_DETECTED.load(Ordering::Acquire);
                    let is_spoofed = result == spoof_detector::AnalysisResult::Spoofed;

                    if is_spoofed && !was_spoofed {
                        if let Some((gl, go, ga)) =
                            pos_buffer.get_position_at(SPOOF_LOOKBACK_SECONDS, now_ms)
                        {
                            LAST_GOOD_LAT.store(gl, Ordering::Release);
                            LAST_GOOD_LON.store(go, Ordering::Release);
                            LAST_GOOD_ALT.store(ga, Ordering::Release);
                        }
                        SPOOF_DETECTED.store(true, Ordering::Release);
                        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                        warn!(
                            "SPOOF [primary detector] fq={} lat={} lon={}",
                            fix.fix_quality, fix.lat_1e7, fix.lon_1e7
                        );
                    } else if !is_spoofed && was_spoofed {
                        let start = SPOOF_RECOVERY_START_MS.load(Ordering::Acquire);
                        if start == 0 {
                            SPOOF_RECOVERY_START_MS.store(now_ms, Ordering::Release);
                        } else if now_ms.wrapping_sub(start) >= SPOOF_RECOVERY_TIMEOUT_MS {
                            SPOOF_DETECTED.store(false, Ordering::Release);
                            SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                            // dynamic_offset stays — computed once at takeoff,
                            // must remain constant for the whole flight.
                        }
                    } else if is_spoofed {
                        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                    }
                }

                // Compute dynamic offset at first real 3D fix (Offset mode only).
                if fix_type.has_3d_fix()
                    && apply_offset
                    && dynamic_offset.is_none()
                    && fix.nsats >= 4
                {
                    *dynamic_offset = Some(DynamicOffset {
                        lat_1e7: config::offset_target::LAT_1E7.saturating_sub(fix.lat_1e7),
                        lon_1e7: config::offset_target::LON_1E7.saturating_sub(fix.lon_1e7),
                        alt_mm:  config::offset_target::ALT_MM.saturating_sub(fix.alt_mm),
                    });
                }
            }
        }
    }

    // Secondary trigger #2 — time-jump check on any frame with a derivable
    // GNSS timestamp. The detector's own `check_gnss_time` is gated behind
    // `has_3d_fix` (early-return at `spoof_detector.rs:388`), so a spoofer
    // that shifts date/time on V-status RMCs with empty coords never reaches
    // it. Compare the current frame's unix timestamp to the previous frame's;
    // a backward jump > `TIME_JUMP_BACK_S` flips SPOOF_DETECTED. Forward jumps
    // are left unbounded — chip legitimately fast-forwards after a long
    // fix-loss gap when it re-acquires real time.
    //
    // Gate: require at least one 3D-fix entry in pos_buffer before firing.
    // u-blox parity — the internal detector's state-modifying checks only
    // activate once `self.prev` is set (first Fix3D sample). Until then,
    // boot-time chip noise (date transitions like 010824→050625, temporary
    // time rewinds during satellite acquisition) should not be interpreted
    // as spoof. We still *track* `last_frame_gnss_unix` so that the first
    // post-3D-fix frame has a reference to compare against.
    let have_fix_reference = pos_buffer.get_position_at(0, now_ms).is_some();
    if (is_gga || is_rmc) && !SPOOF_DETECTED.load(Ordering::Acquire) {
        if let Some(fix) = parsed {
            if fix.checksum_ok {
                // Derive the frame's gnss_time. RMC carries its own date; for
                // GGA combine with the cached RMC date (skip if stale, skip on
                // midnight rollover to avoid a bogus 24 h backward jump).
                let frame_date = if is_rmc && fix.date.year > 0 {
                    Some(fix.date)
                } else if is_gga {
                    let sod = fix.time.hour as u32 * 3600
                            + fix.time.minute as u32 * 60
                            + fix.time.second as u32;
                    let stale = time_cache.date.is_none()
                        || now_ms.wrapping_sub(time_cache.date_ms) > NMEA_DATE_FRESHNESS_MS;
                    let rolled_over = time_cache
                        .last_gga_sod
                        .map_or(false, |prev| prev > sod && prev - sod > 12 * 3600);
                    if stale || rolled_over { None } else { time_cache.date }
                } else {
                    None
                };

                if let Some(d) = frame_date {
                    let curr_unix = spoof_detector::GnssTime {
                        itow_ms: 0,
                        year: d.year,
                        month: d.month,
                        day: d.day,
                        hour: fix.time.hour,
                        min: fix.time.minute,
                        sec: fix.time.second,
                        system_time_ms: now_ms,
                    }
                    .to_unix_timestamp();

                    if have_fix_reference {
                        if let Some(prev_unix) = time_cache.last_frame_gnss_unix {
                            if curr_unix < prev_unix - TIME_JUMP_BACK_S {
                                if let Some((gl, go, ga)) =
                                    pos_buffer.get_position_at(SPOOF_LOOKBACK_SECONDS, now_ms)
                                {
                                    LAST_GOOD_LAT.store(gl, Ordering::Release);
                                    LAST_GOOD_LON.store(go, Ordering::Release);
                                    LAST_GOOD_ALT.store(ga, Ordering::Release);
                                }
                                SPOOF_DETECTED.store(true, Ordering::Release);
                                SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                                warn!(
                                    "SPOOF [time-jump] prev_unix={} curr_unix={} diff={}s tag={}",
                                    prev_unix as i32,
                                    curr_unix as i32,
                                    (curr_unix - prev_unix) as i32,
                                    if is_gga { "GGA" } else { "RMC" },
                                );
                            }
                        }
                    }

                    if !SPOOF_DETECTED.load(Ordering::Acquire) {
                        time_cache.last_frame_gnss_unix = Some(curr_unix);
                    }
                }
            }
        }
    }

    // Secondary trigger #1 on any coord-carrying GGA/RMC. The GGA-driven
    // detector only runs on fq>0 samples, so two leaks slip past it:
    //   (a) RMC arriving before the paired GGA with status=A at a spoofed position
    //       (UC6580I epoch order can be RMC-first).
    //   (b) An `fq=0 / V-status` message that still carries non-zero coords —
    //       UC6580I emits such frames for ~1 tick before upgrading to fq>0, and
    //       the raw coords can still be consumed by the drone.
    // Fires on any checksum_ok message with non-zero coords whose jump vs the
    // most recent buffered 3D fix exceeds TELEPORT_M. Capture LAST_GOOD via the
    // lookback and flip SPOOF_DETECTED so the rebuild block below replaces the
    // coords regardless of the fix-quality field.
    if (is_gga || is_rmc) && !SPOOF_DETECTED.load(Ordering::Acquire) {
        if let Some(fix) = parsed {
            if fix.checksum_ok && (fix.lat_1e7 != 0 || fix.lon_1e7 != 0) {
                if let Some((prev_lat, prev_lon, _)) = pos_buffer.get_position_at(0, now_ms) {
                    let jump_m = spoof_detector::SpoofDetector::calc_distance(
                        prev_lat,
                        prev_lon,
                        fix.lat_1e7,
                        fix.lon_1e7,
                    );
                    if jump_m > spoof_detector::thresholds::TELEPORT_M {
                        if let Some((gl, go, ga)) =
                            pos_buffer.get_position_at(SPOOF_LOOKBACK_SECONDS, now_ms)
                        {
                            LAST_GOOD_LAT.store(gl, Ordering::Release);
                            LAST_GOOD_LON.store(go, Ordering::Release);
                            LAST_GOOD_ALT.store(ga, Ordering::Release);
                        }
                        SPOOF_DETECTED.store(true, Ordering::Release);
                        SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
                        warn!(
                            "SPOOF [coord-jump] jump_m={} prev_lat={} prev_lon={} lat={} lon={} tag={}",
                            jump_m as i32,
                            prev_lat,
                            prev_lon,
                            fix.lat_1e7,
                            fix.lon_1e7,
                            if is_gga { "GGA" } else { "RMC" },
                        );
                    }
                }
            }
        }
    }

    let spoofed = SPOOF_DETECTED.load(Ordering::Acquire);

    // GLL carries lat/lon, VTG carries course/speed. UC6580I keeps both off by
    // default but a drone can enable them via `$CFGMSG,0,1,1` / `$CFGMSG,0,5,1`
    // and persist that in NVM via `$CFGSAVE`. If we forward them verbatim while
    // spoof rebuild or offset rewrite is patching GGA/RMC, raw real coordinates
    // (GLL) or velocity (VTG) leak to the drone.
    let is_gll = line.len() >= 6 && &line[3..6] == b"GLL";
    let is_vtg = line.len() >= 6 && &line[3..6] == b"VTG";

    // Non-GGA/RMC messages (GSA/GSV/ZDA/GST/…) pass verbatim. Exceptions:
    //   - spoof active        → drop GLL+VTG to match the GGA/RMC rebuild;
    //   - offset mode active  → drop GLL (we don't rewrite its format and it
    //                            would leak the real chip coords next to the
    //                            offset GGA/RMC). VTG can stay — velocity is
    //                            invariant under a constant LLH offset.
    if !(is_gga || is_rmc) {
        if spoofed && (is_gll || is_vtg) {
            return;
        }
        if apply_offset && is_gll {
            return;
        }
        enqueue(line);
        return;
    }

    // Pick the coordinates to emit. Semantics:
    //   spoofed           → LAST_GOOD (+ dynamic_offset in PassthroughOffset)
    //   !spoofed, Offset  → actual + offset (suppress if offset not computed
    //                       yet, don't leak real position)
    //   !spoofed, plain   → verbatim
    let coords: Option<(i32, i32, i32)> = if spoofed {
        let base = (
            LAST_GOOD_LAT.load(Ordering::Acquire),
            LAST_GOOD_LON.load(Ordering::Acquire),
            LAST_GOOD_ALT.load(Ordering::Acquire),
        );
        Some(match *dynamic_offset {
            Some(ref off) => (
                base.0.saturating_add(off.lat_1e7),
                base.1.saturating_add(off.lon_1e7),
                base.2.saturating_add(off.alt_mm),
            ),
            None => base,
        })
    } else if apply_offset {
        match (*dynamic_offset, parsed) {
            (Some(off), Some(fix)) if fix.checksum_ok => Some((
                fix.lat_1e7.saturating_add(off.lat_1e7),
                fix.lon_1e7.saturating_add(off.lon_1e7),
                fix.alt_mm.saturating_add(off.alt_mm),
            )),
            (None, _) => return, // suppress: offset not yet computed
            _ => None,
        }
    } else {
        None
    };

    let Some(coords) = coords else {
        enqueue(line);
        return;
    };

    // Under spoof: rebuild the sentence from scratch with a degraded fix so the
    // drone sees "last-known position, fix lost" and enters failsafe.
    if spoofed {
        let existing = parsed.unwrap_or_default();
        let mut buf = [0u8; 256];
        let n = if is_gga {
            build_spoofed_gga(&existing, coords, &mut buf)
        } else {
            build_spoofed_rmc(&existing, coords, &mut buf)
        };
        if n > 0 {
            enqueue(&buf[..n]);
        }
        return;
    }

    // Plain offset rewrite — patch coords in place, fall back to verbatim on
    // buffer/parse failure.
    let mut work = [0u8; 320];
    if let Some(new_len) = rewrite_coords(line, coords, is_gga, &mut work) {
        enqueue(&work[..new_len]);
        return;
    }
    enqueue(line);
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
            SATELLITES_INVALID.store(false, Ordering::Release);
            continue;
        }
        if !booted {
            enqueue_chunked(unicore::boot::BOOT_DUMP).await;
            booted = true;
            counter = 0;
            centis = start_centis;
            OUTPUT_START_MILLIS.store(Instant::now().as_millis() as u32, Ordering::Release);
            SATELLITES_INVALID.store(false, Ordering::Release);
        }
        counter = counter.wrapping_add(1);
        centis += 20;

        // Keep mode-1 behavior aligned with the u-blox build: initially valid,
        // then force invalid satellites after the configured output timeout.
        let start_time = OUTPUT_START_MILLIS.load(Ordering::Acquire);
        let elapsed_ms = (Instant::now().as_millis() as u32).wrapping_sub(start_time);
        let satellites_invalid = elapsed_ms >= config::timers::SATELLITES_INVALID_AFTER_MS as u32;
        SATELLITES_INVALID.store(satellites_invalid, Ordering::Release);

        let valid = !satellites_invalid;
        let fix_quality: u8 = if valid { 1 } else { 0 };
        let nsats: u8 = if valid { 16 } else { 1 };

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
                if valid {
                    for (i, &prn) in prn_list.iter().enumerate().take(12) {
                        sats[i] = prn;
                    }
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

        // GSV every 5th tick (rate=5): per-talker pages advertising the same
        // 16 SVs split across constellations. View count matches GGA nsats.
        if counter % 5 == 0 {
            if valid {
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
            } else {
                let invalid_sat = [GsvSat {
                    prn: 1,
                    elevation_deg: 0,
                    azimuth_deg: 0,
                    cno_dbhz: 0,
                }];
                let n = build_gsv(&mut buf, Talker::Gp, 1, 1, nsats, &invalid_sat, 1);
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
        let sats_invalid = SATELLITES_INVALID.load(Ordering::Acquire);

        // Spoof detection in Passthrough/PassthroughOffset: fast-blink red.
        let value = if spoofed
            && matches!(mode, OperatingMode::Passthrough | OperatingMode::PassthroughOffset)
        {
            if phase.is_multiple_of(2) { RGB8::new(60, 0, 0) } else { RGB8::new(0, 0, 0) }
        } else {
            let colour = match mode {
                OperatingMode::Emulation       => {
                    if sats_invalid { RGB8::new(30, 20, 0) } else { RGB8::new(0, 30, 0) }
                }
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
                info!("btn press #{} counted", press_count);
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
                    _ => OperatingMode::Passthrough,
                };
                let current = OperatingMode::load();
                if new_mode != current {
                    new_mode.store();
                    if new_mode == OperatingMode::Emulation {
                        OUTPUT_START_MILLIS.store(Instant::now().as_millis() as u32, Ordering::Release);
                        SATELLITES_INVALID.store(false, Ordering::Release);
                    }
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
