//! UC6580I emulator entry point (compile with `--features unicore --bin ublox_fake_uc`).
//!
//! Supports the same four operating modes as the u-blox build:
//!   0 Emulation          — experiment: live UC6580I stream with targeted
//!                          forced-3D rewrites for GGA/RMC/GSA only.
//!   1 Passthrough        — forward UART1 (real UC6580I) → UART0 (drone); reassembles
//!                          `$GxGGA`/`$GxRMC` to run the shared spoof detector and,
//!                          when spoofing is flagged, rebuilds GGA/RMC as invalid
//!                          with one visible satellite. RTCM3 frames (incl.
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

#[derive(Clone, Copy, defmt::Format)]
enum ModeChangeSource {
    Button,
    Debug,
}

const DEBUG_MODE_REQUEST_IDLE: u8 = 0xFF;

#[unsafe(no_mangle)]
static MODE: AtomicU8 = AtomicU8::new(OperatingMode::Passthrough as u8);
#[unsafe(no_mangle)]
static DEBUG_MODE_REQUEST: AtomicU8 = AtomicU8::new(DEBUG_MODE_REQUEST_IDLE);

/// Wallclock (ms since boot) when the device entered Mode 0 / Forced3dFix.
/// 0 = uninitialised. Reset on every mode-change INTO Emulation. After
/// `config::timers::SATELLITES_INVALID_AFTER_MS` the rewriter switches from
/// forced 3D fix (fq=3, 16 sats) to forced no-fix (fq=0, 1 sat) — coordinates
/// stay at `config::offset_target` so the drone keeps the home point but
/// loses live tracking, mirroring the u-blox build's "satellites lost" event.
static FORCED3D_PHASE_START_MS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Shared channels
// ---------------------------------------------------------------------------

static TX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 1280>, 32> = Channel::new();
static RAW_RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 256>, 64> = Channel::new();

static SATELLITES_INVALID: AtomicBool = AtomicBool::new(false);

/// Boot-handshake capture: collect the first N checksum-ok config NMEA lines
/// (all non-streaming sentences — i.e. not GGA/RMC/GSA/GSV/GLL/VTG/ZDA/TXT/
/// PNOISE) in each direction into a RAM buffer, then stop. `boot_log_dump_task`
/// periodically prints the buffer so an `attach` after boot still shows the
/// captured handshake.
const BOOT_LOG_MAX: u8 = 100;
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

// ---------------------------------------------------------------------------
// nav-debug periodic summary (every 5 s when `nav-debug` is enabled)
// ---------------------------------------------------------------------------

#[cfg(feature = "nav-debug")]
#[embassy_executor::task]
async fn nav_debug_summary_task() {
    use core::sync::atomic::Ordering;
    Timer::after(Duration::from_secs(3)).await;
    let mut prev_total: u32 = 0;
    loop {
        let gga_ok = nav_debug::GGA_REWRITE.load(Ordering::Relaxed);
        let gga_bad = nav_debug::GGA_REJECT_CHECKSUM.load(Ordering::Relaxed);
        let rmc_ok = nav_debug::RMC_REWRITE.load(Ordering::Relaxed);
        let rmc_bad = nav_debug::RMC_REJECT_CHECKSUM.load(Ordering::Relaxed);
        let gsa_ok = nav_debug::GSA_REWRITE.load(Ordering::Relaxed);
        let gsa_bad = nav_debug::GSA_REJECT.load(Ordering::Relaxed);
        let gll_ok = nav_debug::GLL_REWRITE.load(Ordering::Relaxed);
        let gll_bad = nav_debug::GLL_REJECT_CHECKSUM.load(Ordering::Relaxed);
        let gntxt_ok = nav_debug::GNTXT_STATUS_REWRITE.load(Ordering::Relaxed);
        let gntxt_bad = nav_debug::GNTXT_STATUS_REJECT.load(Ordering::Relaxed);
        let gsv_synth = nav_debug::GSV_SYNTH.load(Ordering::Relaxed);
        let gsv_bad = nav_debug::GSV_REJECT.load(Ordering::Relaxed);
        let gsv = nav_debug::GSV_THROUGH.load(Ordering::Relaxed);
        let rtcm_dropped = nav_debug::RTCM_4074_DROPPED.load(Ordering::Relaxed);
        let rtcm_kept = nav_debug::RTCM_KEPT.load(Ordering::Relaxed);
        let vtg = nav_debug::VTG_THROUGH.load(Ordering::Relaxed);
        let zda = nav_debug::ZDA_THROUGH.load(Ordering::Relaxed);
        let txt = nav_debug::TXT_THROUGH.load(Ordering::Relaxed);
        let prop = nav_debug::PROP_THROUGH.load(Ordering::Relaxed);
        let other = nav_debug::OTHER_THROUGH.load(Ordering::Relaxed);
        let chip_fq = nav_debug::LAST_CHIP_FQ.load(Ordering::Relaxed);
        let chip_rmc = nav_debug::LAST_CHIP_RMC_VALID.load(Ordering::Relaxed);
        let chip_nsats = nav_debug::LAST_CHIP_NSATS.load(Ordering::Relaxed);
        let t_fresh = nav_debug::TIME_CACHE_FRESH.load(Ordering::Relaxed);
        let d_fresh = nav_debug::DATE_CACHE_FRESH.load(Ordering::Relaxed);

        let total = gga_ok + gga_bad + rmc_ok + rmc_bad + gsa_ok + gsa_bad
            + gll_ok + gll_bad + gntxt_ok + gntxt_bad
            + gsv_synth + gsv_bad + gsv + vtg + zda + txt + prop + other;
        let delta = total.wrapping_sub(prev_total);
        prev_total = total;

        info!(
            "[nav-debug] mode={:?} Δ5s_lines={=u32} | rewrite GGA={=u32}/{=u32} RMC={=u32}/{=u32} GSA={=u32}/{=u32} GLL={=u32}/{=u32} GNTXT={=u32}/{=u32} GSV_synth={=u32}/{=u32} | passthrough GSV={=u32} VTG={=u32} ZDA={=u32} TXT={=u32} P*={=u32} other={=u32} | RTCM kept={=u32} 4074_dropped={=u32} | chip fq={=u8} nsats={=u8} rmc={=u8} | cache t={=u8} d={=u8}",
            OperatingMode::load(),
            delta,
            gga_ok, gga_bad, rmc_ok, rmc_bad, gsa_ok, gsa_bad, gll_ok, gll_bad, gntxt_ok, gntxt_bad, gsv_synth, gsv_bad,
            gsv, vtg, zda, txt, prop, other,
            rtcm_kept, rtcm_dropped,
            chip_fq, chip_nsats, chip_rmc,
            t_fresh, d_fresh,
        );

        Timer::after(Duration::from_secs(5)).await;
    }
}

// ---------------------------------------------------------------------------
// Boot-stream raw capture (`boot-capture` feature)
// ---------------------------------------------------------------------------
//
// On power-up of the real UC6580I (UART1 RX, GPIO5), capture the first ~6.7 KB
// linearly into a static RAM buffer until either (a) the buffer fills,
// (b) ~4 s elapsed since the first byte, or (c) we see the steady-state marker
// `$GNRMC`. Then `raw_boot_dump_task` periodically re-emits the frozen buffer
// over defmt-rtt so a late `probe-rs attach` can still recover the bytes.
//
// Re-arm: a >500 ms silence followed by new bytes resets the buffer. This is
// how repeated chip power-cycles are picked up without resetting the board.
//
// Decode the RTT log with `tools/decode_rtt_boot_capture.py`.

#[cfg(feature = "boot-capture")]
mod raw_capture {
    use core::cell::UnsafeCell;
    use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32};

    pub const CAP_LEN: usize = 8192;
    pub const REARM_GAP_MS: u32 = 500;

    // ---- Chip → board side (UART1 RX) ----------------------------------
    /// Freeze ~4 s after first byte (boot dump completes in ~3 s).
    pub const FREEZE_AFTER_MS_CH: u32 = 4_000;
    /// First 6 bytes of `$GNRMC,…\r\n` — fires once the chip enters its 5 Hz
    /// stream so we don't pollute the buffer with steady-state NMEA.
    pub const SENTINEL_CH: &[u8] = b"$GNRMC";

    // ---- Drone → board side (UART0 RX) --------------------------------
    /// Long timeout because `$CFGKEY` arrives ~20 s after the first command
    /// on real drones (Air 3S / Mavic). 25 s gives margin.
    pub const FREEZE_AFTER_MS_DR: u32 = 25_000;
    /// `$CFGKEY` is the last config command in the standard handshake;
    /// freezing on it captures every preceding command without idle padding.
    pub const SENTINEL_DR: &[u8] = b"$CFGKEY";

    pub struct CapBuf(pub UnsafeCell<[u8; CAP_LEN]>);
    // SAFETY: each buffer has a single writer task on core 0. Readers
    // (`raw_boot_dump_task`) synchronise via `DONE.load(Acquire)` after writes
    // are made visible by `WRITE_IDX.store(Release)`. Single-core Unicore
    // build → no preemption.
    unsafe impl Sync for CapBuf {}

    /// Per-direction state — atomics + `UnsafeCell`-wrapped buffer.
    pub struct Channel {
        pub buf: CapBuf,
        pub write_idx: AtomicU16,
        pub done: AtomicBool,
        /// 0 = capture not started yet (no bytes seen).
        pub first_ms: AtomicU32,
        /// 0 = no bytes seen yet (also used as the silence-gap reference).
        pub last_ms: AtomicU32,
        /// Bumped on every re-arm. Consumers prefer the highest fully-emitted epoch.
        pub epoch: AtomicU16,
    }

    impl Channel {
        pub const fn new() -> Self {
            Self {
                buf: CapBuf(UnsafeCell::new([0u8; CAP_LEN])),
                write_idx: AtomicU16::new(0),
                done: AtomicBool::new(false),
                first_ms: AtomicU32::new(0),
                last_ms: AtomicU32::new(0),
                epoch: AtomicU16::new(0),
            }
        }
    }

    /// Chip-side (UART1 RX). Backwards-compat alias retained.
    pub static CH: Channel = Channel::new();
    /// Drone-side (UART0 RX).
    pub static DR: Channel = Channel::new();

    /// CRC32 (IEEE 802.3 / zlib polynomial 0xEDB88320), table-less.
    /// Tiny inline implementation — avoids pulling in the `crc` crate.
    pub fn crc32_ieee(data: &[u8]) -> u32 {
        let mut crc: u32 = 0xFFFF_FFFF;
        for &b in data {
            crc ^= b as u32;
            for _ in 0..8 {
                let mask = (crc & 1).wrapping_neg();
                crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
            }
        }
        !crc
    }

    /// True if `haystack` contains `needle`. Naive scan — buffers are small.
    pub fn contains_subseq(haystack: &[u8], needle: &[u8]) -> bool {
        if needle.is_empty() || haystack.len() < needle.len() {
            return false;
        }
        haystack.windows(needle.len()).any(|w| w == needle)
    }
}

/// Push a slice into a `raw_capture::Channel`, applying the freeze rules.
/// Called from inside a single writer task — `static` access is sound.
#[cfg(feature = "boot-capture")]
fn raw_capture_feed(
    ch: &raw_capture::Channel,
    tag: &'static str,
    freeze_after_ms: u32,
    sentinel: &[u8],
    bytes: &[u8],
) {
    use raw_capture::{contains_subseq, CAP_LEN, REARM_GAP_MS};
    let now_ms = Instant::now().as_millis() as u32;
    let last = ch.last_ms.swap(now_ms, Ordering::Relaxed);
    let mut done = ch.done.load(Ordering::Acquire);
    // Re-arm: long silence after a previous freeze + new bytes →
    // assume the source rebooted, start a fresh capture.
    if done && last != 0 && now_ms.wrapping_sub(last) > REARM_GAP_MS {
        ch.write_idx.store(0, Ordering::Release);
        ch.first_ms.store(0, Ordering::Relaxed);
        ch.done.store(false, Ordering::Release);
        ch.epoch.fetch_add(1, Ordering::Relaxed);
        done = false;
    }
    if done {
        return;
    }
    let first = ch.first_ms.load(Ordering::Relaxed);
    let first = if first == 0 {
        ch.first_ms.store(now_ms, Ordering::Relaxed);
        now_ms
    } else {
        first
    };
    let idx = ch.write_idx.load(Ordering::Relaxed) as usize;
    let room = CAP_LEN.saturating_sub(idx);
    let take = room.min(bytes.len());
    if take == 0 {
        // Buffer full but DONE not set yet — set it now.
        ch.done.store(true, Ordering::Release);
        return;
    }
    // SAFETY: single-writer task, single-core build.
    unsafe {
        let buf_mut: &mut [u8; CAP_LEN] = &mut *ch.buf.0.get();
        buf_mut[idx..idx + take].copy_from_slice(&bytes[..take]);
    }
    let new_idx = idx + take;
    ch.write_idx.store(new_idx as u16, Ordering::Release);
    let elapsed = now_ms.wrapping_sub(first);
    let saw_sentinel = unsafe {
        let buf_ref: &[u8; CAP_LEN] = &*ch.buf.0.get();
        contains_subseq(&buf_ref[..new_idx], sentinel)
    };
    if new_idx >= CAP_LEN || elapsed >= freeze_after_ms || saw_sentinel {
        ch.done.store(true, Ordering::Release);
        info!(
            "boot-capture frozen [{}]: epoch={=u16} len={=u16} elapsed_ms={=u32}",
            tag,
            ch.epoch.load(Ordering::Relaxed),
            new_idx as u16,
            elapsed,
        );
    }
}

/// Emit one channel's frozen buffer to RTT. `tag` selects the marker prefix
/// (`CHIP` → `RAWCAP …`, `DRONE` → `RAWCAP_DR …`); the parser script keys on it.
#[cfg(feature = "boot-capture")]
fn emit_channel(ch: &raw_capture::Channel, tag: &'static str) {
    use raw_capture::{crc32_ieee, CAP_LEN};
    let done = ch.done.load(Ordering::Acquire);
    let len = ch.write_idx.load(Ordering::Acquire) as usize;
    let epoch = ch.epoch.load(Ordering::Relaxed);
    if !done || len == 0 || len > CAP_LEN {
        return;
    }
    // SAFETY: writer task has reached `DONE=true`, so no further mutation.
    let slice: &[u8] = unsafe {
        let buf_ref: &[u8; CAP_LEN] = &*ch.buf.0.get();
        &buf_ref[..len]
    };
    let crc = crc32_ieee(slice);
    if tag == "DRONE" {
        info!(
            "==RAWCAP_DR BEGIN epoch={=u16} len={=u16} crc={=u32:08x}==",
            epoch, len as u16, crc,
        );
    } else {
        info!(
            "==RAWCAP BEGIN epoch={=u16} len={=u16} crc={=u32:08x}==",
            epoch, len as u16, crc,
        );
    }
    // 32 raw bytes per line → defmt {=[u8]:02x} renders as `[xx, xx, ...]`.
    let mut off = 0usize;
    while off < len {
        let end = (off + 32).min(len);
        if tag == "DRONE" {
            info!(
                "RAWCAP_DR {=u16} {=u16} {=[u8]:02x}",
                epoch,
                off as u16,
                &slice[off..end],
            );
        } else {
            info!(
                "RAWCAP {=u16} {=u16} {=[u8]:02x}",
                epoch,
                off as u16,
                &slice[off..end],
            );
        }
        off = end;
    }
    if tag == "DRONE" {
        info!("==RAWCAP_DR END epoch={=u16} len={=u16}==", epoch, len as u16);
    } else {
        info!("==RAWCAP END epoch={=u16} len={=u16}==", epoch, len as u16);
    }
}

/// Periodically re-emit the captured boot stream as hex-chunked defmt log lines.
/// Runs only with the `boot-capture` feature. Emits both directions (chip and
/// drone) when each is frozen.
#[cfg(feature = "boot-capture")]
#[embassy_executor::task]
async fn raw_boot_dump_task() {
    Timer::after(Duration::from_secs(6)).await;
    loop {
        emit_channel(&raw_capture::CH, "CHIP");
        emit_channel(&raw_capture::DR, "DRONE");
        Timer::after(Duration::from_secs(10)).await;
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

/// Minimum clean-fix window before we drop SPOOF_DETECTED back to false
/// (u-blox parity: one good reading is not enough).
const SPOOF_RECOVERY_TIMEOUT_MS: u32 = 5_000;

/// When spoofing starts, LAST_GOOD captures the fix from this many seconds
/// before the detector fired — jitter-resistant "pre-spoof" snapshot.
const SPOOF_LOOKBACK_SECONDS: u32 = 2;

/// Large HDOP value planted in spoofed GGA (tells the drone the fix is poor).
const SPOOF_HIGH_HDOP_X100: u16 = 9_999;

/// Satellite count reported in spoofed GGA. Keep the fix invalid while showing
/// one visible satellite on the Unicore NMEA path.
const SPOOF_INVALID_NSATS: u8 = 1;

/// Placeholder geoid separation for rebuilt GGA under spoof. Real value would
/// require the original sentence's geoid_sep to round-trip, which the
/// rebuild path does not carry.
const SPOOF_DEFAULT_GEOID_SEP_MM: i32 = -30_000;

/// Detector instance — lives for the whole session, reset via `.reset()` on
/// mode changes that would otherwise pollute its internal `LAST_GOOD`.
static SPOOF_CELL: Mutex<CriticalSectionRawMutex, Option<spoof_detector::SpoofDetector>> =
    Mutex::new(None);

use pos_history::{DynamicOffset, PositionBuffer};

#[derive(Clone, Copy, PartialEq, Eq)]
enum NmeaOutputPolicy {
    Plain,
    Offset,
    Forced3dFix,
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
    spawner.must_spawn(button_task(btn_flex, flash_mutex));
    spawner.must_spawn(debug_mode_request_task(flash_mutex));
    spawner.must_spawn(boot_log_dump_task());
    #[cfg(feature = "boot-capture")]
    spawner.must_spawn(raw_boot_dump_task());
    #[cfg(feature = "nav-debug")]
    spawner.must_spawn(nav_debug_summary_task());

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
    // Short-form `$GNTXT,01,01,00,…` preacks are part of the command/reply
    // handshake — keep them visible in the boot log even though their
    // talker+sentence prefix is `GxTXT` (which we otherwise drop as periodic).
    if body.starts_with(b"$GNTXT,01,01,00,") {
        return true;
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
// UART0 RX — capture drone command traffic; the live UC6580I sends replies.
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn uart0_rx_task(mut rx: BufferedUartRx) {
    let mut buf = [0u8; 128];
    let mut diag_asm: unicore::nmea::LineAssembler<256> = unicore::nmea::LineAssembler::new();
    loop {
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                // ---------------------------------------------------------------
                // boot-capture (drone side, UART0 RX): captures every byte the
                // drone sends, including the ~38-byte binary garbage prefix and
                // the late `$CFGKEY` (~20 s post-power). Active in any mode —
                // the drone-bus traffic is interesting in Emulation too.
                // ---------------------------------------------------------------
                #[cfg(feature = "boot-capture")]
                {
                    raw_capture_feed(
                        &raw_capture::DR,
                        "DRONE",
                        raw_capture::FREEZE_AFTER_MS_DR,
                        raw_capture::SENTINEL_DR,
                        &buf[..n],
                    );
                }

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
                // Mode 1 now keeps the live chip on the command bus. Do not
                // synthesize replies here; the UC6580I answers the drone.
            }
            Ok(_) => {}
            Err(e) => {
                warn!("uart0 rx error: {:?}", defmt::Debug2Format(&e));
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}

fn enqueue(bytes: &[u8]) {
    let mut v = heapless::Vec::<u8, 1280>::new();
    if v.extend_from_slice(bytes).is_ok() && TX_CHANNEL.try_send(v).is_err() {
        warn!("TX_CHANNEL full, dropped {} bytes", bytes.len());
    }
}

// ---------------------------------------------------------------------------
// nav-debug — Mode 0 (Forced3dFix) instrumentation. Counters + per-frame log.
// All zero-cost when the feature is off.
// ---------------------------------------------------------------------------

#[cfg(feature = "nav-debug")]
mod nav_debug {
    use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

    pub static GGA_REWRITE: AtomicU32 = AtomicU32::new(0);
    pub static GGA_REJECT_CHECKSUM: AtomicU32 = AtomicU32::new(0);
    pub static RMC_REWRITE: AtomicU32 = AtomicU32::new(0);
    pub static RMC_REJECT_CHECKSUM: AtomicU32 = AtomicU32::new(0);
    pub static GSA_REWRITE: AtomicU32 = AtomicU32::new(0);
    pub static GSA_REJECT: AtomicU32 = AtomicU32::new(0);
    pub static GLL_REWRITE: AtomicU32 = AtomicU32::new(0);
    pub static GLL_REJECT_CHECKSUM: AtomicU32 = AtomicU32::new(0);
    pub static GNTXT_STATUS_REWRITE: AtomicU32 = AtomicU32::new(0);
    pub static GNTXT_STATUS_REJECT: AtomicU32 = AtomicU32::new(0);
    pub static GSV_SYNTH: AtomicU32 = AtomicU32::new(0);
    pub static GSV_REJECT: AtomicU32 = AtomicU32::new(0);
    pub static GSV_THROUGH: AtomicU32 = AtomicU32::new(0);
    /// Frames matching `msg_id == 4074` (ExtRTCM proprietary) dropped in Mode 0.
    pub static RTCM_4074_DROPPED: AtomicU32 = AtomicU32::new(0);
    /// Standard RTCM3 frames (msg_id != 4074) forwarded.
    pub static RTCM_KEPT: AtomicU32 = AtomicU32::new(0);
    pub static VTG_THROUGH: AtomicU32 = AtomicU32::new(0);
    pub static ZDA_THROUGH: AtomicU32 = AtomicU32::new(0);
    pub static TXT_THROUGH: AtomicU32 = AtomicU32::new(0);
    /// `$P*` proprietary sentences forwarded verbatim.
    pub static PROP_THROUGH: AtomicU32 = AtomicU32::new(0);
    /// Anything else NMEA-shaped that we forwarded.
    pub static OTHER_THROUGH: AtomicU32 = AtomicU32::new(0);
    /// Latest fix_quality observed on a checksum-ok chip GGA (255 = unknown).
    pub static LAST_CHIP_FQ: AtomicU8 = AtomicU8::new(255);
    /// Latest RMC status: 1=A, 0=V, 255=unknown.
    pub static LAST_CHIP_RMC_VALID: AtomicU8 = AtomicU8::new(255);
    /// Latest nsats observed on a checksum-ok chip GGA.
    pub static LAST_CHIP_NSATS: AtomicU8 = AtomicU8::new(0);
    /// 1 = `time_cache.last_time` populated and fresh; 0 = falling back to default.
    pub static TIME_CACHE_FRESH: AtomicU8 = AtomicU8::new(0);
    /// 1 = `time_cache.date` populated and fresh; 0 = default fallback.
    pub static DATE_CACHE_FRESH: AtomicU8 = AtomicU8::new(0);

    #[inline]
    pub fn bump(c: &AtomicU32) {
        c.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(feature = "nav-debug")]
macro_rules! nav_dbg {
    ($($t:tt)*) => { defmt::info!($($t)*); };
}
#[cfg(not(feature = "nav-debug"))]
macro_rules! nav_dbg {
    ($($t:tt)*) => {};
}

/// Trace a TX-to-drone event when `SPOOF_DETECTED` is true. Used to chase
/// coordinate leaks during spoof experiments. Compile-time off in production.
#[cfg(feature = "tx-trace")]
fn tx_trace(tag: &'static str, bytes: &[u8]) {
    if !SPOOF_DETECTED.load(Ordering::Acquire) {
        return;
    }
    let n = bytes.len().min(48);
    info!(
        "[TX→DRONE/{}] len={=u16} first={=[u8]:02x}",
        tag,
        bytes.len() as u16,
        &bytes[..n],
    );
}
#[cfg(not(feature = "tx-trace"))]
#[inline(always)]
fn tx_trace(_tag: &'static str, _bytes: &[u8]) {}

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
                // ---------------------------------------------------------------
                // boot-capture (chip side): linear one-shot RAM buffer of the
                // chip's first ~6.7 KB. See raw_capture mod near top of file.
                // Active in every mode; Mode 1 now forwards the live chip too.
                // ---------------------------------------------------------------
                #[cfg(feature = "boot-capture")]
                raw_capture_feed(
                    &raw_capture::CH,
                    "CHIP",
                    raw_capture::FREEZE_AFTER_MS_CH,
                    raw_capture::SENTINEL_CH,
                    &buf[..n],
                );

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
                // Always forward chip bytes into the stream router. Mode 1
                // rewrites selected NMEA sentences downstream but never cuts
                // the live UC6580I stream after `$CFGKEY`.
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
    let mut rtcm_remaining: u16 = 0;  // payload + CRC bytes still to forward/modify
    // Scratch for first 6 frame bytes (D3 + LL_HI + LL_LO + 3 bytes encoding
    // msg_id[12] | sub_id[12]). Held without forwarding until msg_id+sub_id
    // are decoded; then either flushed to `pass_buf` (forward verbatim) or
    // kept in `rtcm_frame_buf` (modify in-place at frame end).
    let mut rtcm_scratch: [u8; 6] = [0; 6];
    let mut rtcm_scratch_n: u8 = 0;
    // Active when we're capturing a full frame for in-place modification.
    // Bytes go to `rtcm_frame_buf` instead of `pass_buf`; at frame end we
    // apply `unicore::extrtcm::modify_receiver_info` and enqueue the
    // modified frame as one TX chunk.
    let mut rtcm_modify_0ff: bool = false;
    // Buffer for the full 4074-0xFF frame being captured (D3+LL+MID+data+CRC =
    // 169 bytes). Sized to fit any RTCM frame up to MAX_PAYLOAD; only used
    // when `rtcm_modify_0ff = true`.
    let mut rtcm_frame_buf: heapless::Vec<u8, 256> = heapless::Vec::new();

    // Lazy — created on first entry into Passthrough/PassthroughOffset.
    let mut pos_buffer: Option<PositionBuffer> = None;
    let mut dynamic_offset: Option<DynamicOffset> = None;
    // Date from $GxRMC + midnight-rollover guard. Combined with the HH:MM:SS
    // from GGA to build `spoof_detector::GnssTime`, which enables time-jump
    // and system-clock-drift checks in the shared detector.
    let mut time_cache = NmeaTimeCache::default();

    // Cold-boot: if persisted mode is Emulation, arm the FORCED3D window now
    // so the 22 s timer starts at the very first chip frame (not the first
    // Mode-0 mode-change event, which never fires when persisted).
    if last_mode == OperatingMode::Emulation {
        FORCED3D_PHASE_START_MS.store(
            embassy_time::Instant::now().as_millis() as u32,
            Ordering::Release,
        );
        SATELLITES_INVALID.store(false, Ordering::Release);
    }

    loop {
        let chunk = RAW_RX_CHANNEL.receive().await;
        let mode = OperatingMode::load();

        // Mode change → cause a full reset on the next detector tick.
        if mode != last_mode {
            SPOOF_DETECTOR_RESET.store(true, Ordering::Release);
            // (Re)arm the FORCED3D window when we ENTER Emulation so the
            // 22 s "satellites lost" timer always counts from the moment the
            // drone could see fq=3 from us.
            if mode == OperatingMode::Emulation {
                FORCED3D_PHASE_START_MS.store(
                    embassy_time::Instant::now().as_millis() as u32,
                    Ordering::Release,
                );
                SATELLITES_INVALID.store(false, Ordering::Release);
            }
            last_mode = mode;
        }

        // Process the reset flag (also set explicitly by the button task).
        if SPOOF_DETECTOR_RESET.swap(false, Ordering::AcqRel) {
            asm = nmea::LineAssembler::new();
            pass_buf.clear();
            in_sentence = false;
            rtcm_remaining = 0;
            rtcm_scratch_n = 0;
            rtcm_modify_0ff = false;
            rtcm_frame_buf.clear();
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
            OperatingMode::PassthroughRaw => {
                tx_trace("raw-chunk", &chunk);
                enqueue(&chunk);
            }
            OperatingMode::Emulation | OperatingMode::Passthrough | OperatingMode::PassthroughOffset => {
                if pos_buffer.is_none() {
                    pos_buffer = Some(PositionBuffer::new());
                }
                let nmea_policy = match mode {
                    OperatingMode::Emulation => NmeaOutputPolicy::Forced3dFix,
                    OperatingMode::PassthroughOffset => NmeaOutputPolicy::Offset,
                    _ => NmeaOutputPolicy::Plain,
                };

                for &b in chunk.iter() {
                    // --- Inside an RTCM frame: forward bytes verbatim through
                    //     pass_buf (default) OR accumulate into rtcm_frame_buf
                    //     when we've decided to modify this frame in-place. --
                    if rtcm_remaining > 0 {
                        if rtcm_modify_0ff {
                            // Capacity reserved at scratch-flush time; push
                            // can't fail unless we mis-sized the buffer.
                            let _ = rtcm_frame_buf.push(b);
                        } else {
                            buffered_forward(&mut pass_buf, b);
                        }
                        rtcm_remaining -= 1;
                        if rtcm_remaining == 0 {
                            if rtcm_modify_0ff {
                                // DIAGNOSTIC MODE: drop 4074-0xFF entirely
                                // post-id-window. Other 4074 sub-IDs (incl.
                                // 0x0FC chip auth blob) still pass verbatim.
                                // Tests whether 0x0FC contains an HMAC over
                                // 0x0FF stream:
                                //   - If drone accepts module → 0x0FC is
                                //     independent (no HMAC binding to 0x0FF).
                                //   - If drone rejects → 0x0FC binds to 0x0FF.
                                let now_ms = embassy_time::Instant::now().as_millis() as u32;
                                let phase_start = FORCED3D_PHASE_START_MS.load(Ordering::Acquire);
                                let elapsed = if phase_start == 0 { 0 } else { now_ms.wrapping_sub(phase_start) };
                                let post_id_window = (elapsed as u64)
                                    >= config::timers::SATELLITES_INVALID_AFTER_MS;
                                if post_id_window {
                                    // DROP entire 0x0FF frame.
                                    tx_trace("rtcm-4074ff-dropped", &[]);
                                    #[cfg(feature = "nav-debug")]
                                    nav_debug::bump(&nav_debug::RTCM_4074_DROPPED);
                                    nav_dbg!(
                                        "[nav-debug] RTCM 4074-0xFF DROPPED (post-id, elapsed={=u32}ms)",
                                        elapsed,
                                    );
                                } else {
                                    // Within identify window — forward verbatim.
                                    tx_trace("rtcm-4074ff-id-window", &rtcm_frame_buf);
                                    enqueue(&rtcm_frame_buf);
                                    #[cfg(feature = "nav-debug")]
                                    nav_debug::bump(&nav_debug::RTCM_KEPT);
                                    nav_dbg!(
                                        "[nav-debug] RTCM 4074-0xFF id-window verbatim (elapsed={=u32}ms)",
                                        elapsed,
                                    );
                                }
                                rtcm_frame_buf.clear();
                                rtcm_modify_0ff = false;
                            } else {
                                // Pass-through path: flush pass_buf at frame boundary.
                                if !pass_buf.is_empty() {
                                    tx_trace("rtcm-flush", &pass_buf);
                                    enqueue(&pass_buf);
                                    pass_buf.clear();
                                }
                                #[cfg(feature = "nav-debug")]
                                nav_debug::bump(&nav_debug::RTCM_KEPT);
                            }
                        }
                        continue;
                    }

                    // --- Collecting frame leading 6 bytes:
                    //   [0]=D3, [1]=LL_HI, [2]=LL_LO,
                    //   [3..6] = msg_id[12] | sub_id[12] bit-packed:
                    //     [3] = msg_id[11:4]
                    //     [4] = (msg_id[3:0] << 4) | sub_id[11:8]
                    //     [5] = sub_id[7:0]
                    //
                    // Decision (forward-verbatim vs modify-0xFF) at index 5.
                    if rtcm_scratch_n > 0 && rtcm_scratch_n < 6 {
                        if rtcm_scratch_n == 1 && b & 0xFC != 0 {
                            // Top 6 bits of byte 1 must be zero. If not, the
                            // `0xD3` was noise — flush the stashed D3 as a
                            // regular byte and re-route current byte through
                            // normal NMEA logic.
                            buffered_forward(&mut pass_buf, rtcm_scratch[0]);
                            rtcm_scratch_n = 0;
                            // fall through without consuming `b`
                        } else {
                            rtcm_scratch[rtcm_scratch_n as usize] = b;
                            rtcm_scratch_n += 1;
                            if rtcm_scratch_n == 6 {
                                let payload_len = (((rtcm_scratch[1] as u16) & 0x03) << 8)
                                    | (rtcm_scratch[2] as u16);
                                let msg_id: u16 = ((rtcm_scratch[3] as u16) << 4)
                                    | ((rtcm_scratch[4] as u16) >> 4);
                                let sub_id: u16 = (((rtcm_scratch[4] as u16) & 0x0F) << 8)
                                    | (rtcm_scratch[5] as u16);
                                let modify_this = mode == OperatingMode::Emulation
                                    && msg_id == 4074
                                    && sub_id == unicore::extrtcm::SUB_RECEIVER_INFO;
                                if modify_this {
                                    // Stash all 6 leading bytes into the modify
                                    // buffer; remaining payload+CRC bytes will
                                    // accumulate into it via `rtcm_modify_0ff`.
                                    rtcm_frame_buf.clear();
                                    for &sb in rtcm_scratch.iter() {
                                        let _ = rtcm_frame_buf.push(sb);
                                    }
                                    rtcm_modify_0ff = true;
                                    nav_dbg!(
                                        "[nav-debug] RTCM 4074-0xFF capture (payload_len={=u16})",
                                        payload_len,
                                    );
                                } else {
                                    // Forward all 6 leading bytes verbatim
                                    // through pass_buf.
                                    for &sb in rtcm_scratch.iter() {
                                        buffered_forward(&mut pass_buf, sb);
                                    }
                                    nav_dbg!(
                                        "[nav-debug] RTCM keep msg_id={=u16} sub_id=0x{=u16:X} payload_len={=u16}",
                                        msg_id, sub_id, payload_len,
                                    );
                                }
                                // Remaining bytes after the 6 leading: data
                                // = payload_len - 3 (msg_id+sub_id consumed)
                                // plus 3-byte CRC.
                                rtcm_remaining = payload_len.saturating_sub(3) + 3;
                                rtcm_scratch_n = 0;
                            }
                            continue;
                        }
                    }

                    // --- RTCM3 preamble only outside of an active NMEA sentence --
                    if !in_sentence && b == 0xD3 && rtcm_scratch_n == 0 {
                        rtcm_scratch[0] = b;
                        rtcm_scratch_n = 1;
                        continue;
                    }

                    // --- NMEA routing ------------------------------------------
                    if !in_sentence && b != b'$' {
                        buffered_forward(&mut pass_buf, b);
                        continue;
                    }
                    if b == b'$' && !pass_buf.is_empty() {
                        tx_trace("inter-sentence", &pass_buf);
                        enqueue(&pass_buf);
                        pass_buf.clear();
                    }
                    if b == b'$' { in_sentence = true; }

                    if let Some(line) = asm.feed(b) {
                        in_sentence = false;
                        process_nmea_line(
                            line,
                            nmea_policy,
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

/// Build a spoofed GGA frame (invalid fix + one visible satellite). Returns
/// written byte count in `out`, or 0 on build failure.
fn build_spoofed_gga(existing: &unicore::nmea::NmeaFix, coords: (i32, i32, i32), out: &mut [u8]) -> usize {
    let g = unicore::nmea::GgaFields {
        time: existing.time,
        lat_1e7: coords.0, lon_1e7: coords.1,
        fix_quality: 0,
        nsats: SPOOF_INVALID_NSATS,
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
    /// Most recent parseable time (HH:MM:SS.cc) seen on any chip GGA/RMC.
    /// Used by Forced3dFix to fill the time field on chip frames whose own
    /// time is empty (typical pre-fix), so the rewrite still emits a valid
    /// sentence with target coords + fq=3 instead of falling back verbatim.
    last_time: Option<unicore::nmea::NmeaTime>,
    last_time_ms: u32,
}

const NMEA_DATE_FRESHNESS_MS: u32 = 1500;
/// `last_time` accepted as fallback for up to 60 s — covers extended fix-loss
/// windows. After that, fall back to a fixed default rather than emit a stale
/// timestamp that could trigger drone-side time-jump logic.
const NMEA_TIME_FRESHNESS_MS: u32 = 60_000;

/// Largest tolerated backward jump between consecutive GNSS timestamps. The
/// u-blox detector uses the same 1 s threshold (`thresholds::MAX_TIME_JUMP_BACK_S`
/// inside `check_gnss_time`). Forward jumps are allowed to be unbounded — chips
/// legitimately fast-forward by the gap duration when re-acquiring after a
/// long fix loss, so flagging forward jumps would produce false positives.
const TIME_JUMP_BACK_S: i64 = 1;

async fn process_nmea_line(
    line: &[u8],
    policy: NmeaOutputPolicy,
    pos_buffer: &mut PositionBuffer,
    dynamic_offset: &mut Option<DynamicOffset>,
    time_cache: &mut NmeaTimeCache,
) {
    use unicore::nmea;

    let now_ms = embassy_time::Instant::now().as_millis() as u32;
    let is_gga = line.len() >= 6 && &line[3..6] == b"GGA";
    let is_rmc = line.len() >= 6 && &line[3..6] == b"RMC";
    let is_gsa = line.len() >= 6 && &line[3..6] == b"GSA";
    let is_gll = line.len() >= 6 && &line[3..6] == b"GLL";
    // Long-form periodic status from UC6580I: `$xxTXT,01,01,01,…` (24 fields,
    // vendor-extended). Field 13 is the fix-progression indicator. Distinct
    // from the short `,00,…` preack which we leave alone.
    let is_gntxt_status = line.len() >= 16
        && &line[3..6] == b"TXT"
        && line[6..].starts_with(b",01,01,01,");
    let is_gsv = line.len() >= 6 && &line[3..6] == b"GSV";

    // Cache any parseable time/date from the chip's frame so the Forced3dFix
    // rewrite can fall back to a recent timestamp when the chip's own GGA/RMC
    // arrives with empty time field (cold-boot / fix-loss). Plain/Offset paths
    // also benefit (more accurate GnssTime feed for the detector).
    if is_gga || is_rmc {
        if let Some(fix) = if is_gga { nmea::parse_gga(line) } else { nmea::parse_rmc(line) } {
            if fix.checksum_ok && fix.time != nmea::NmeaTime::default() {
                time_cache.last_time = Some(fix.time);
                time_cache.last_time_ms = now_ms;
            }
            #[cfg(feature = "nav-debug")]
            {
                if is_gga && fix.checksum_ok {
                    nav_debug::LAST_CHIP_FQ.store(fix.fix_quality, Ordering::Relaxed);
                    nav_debug::LAST_CHIP_NSATS.store(fix.nsats, Ordering::Relaxed);
                }
                if is_rmc && fix.checksum_ok {
                    nav_debug::LAST_CHIP_RMC_VALID
                        .store(if fix.fix_quality > 0 { 1 } else { 0 }, Ordering::Relaxed);
                }
            }
        }
    }

    if policy == NmeaOutputPolicy::Forced3dFix {
        let target = (
            config::offset_target::LAT_1E7,
            config::offset_target::LON_1E7,
            config::offset_target::ALT_MM,
        );

        // Fallback time: cached chip time if fresh, else 00:00:00.
        let fallback_time = match time_cache.last_time {
            Some(t) if now_ms.wrapping_sub(time_cache.last_time_ms) <= NMEA_TIME_FRESHNESS_MS => t,
            _ => nmea::NmeaTime::default(),
        };
        // Fallback date: cached RMC date if fresh, else a fixed plausible date
        // (1 Jan 2025) — drone needs a non-zero date for RMC to validate.
        let fallback_date = match time_cache.date {
            Some(d) if now_ms.wrapping_sub(time_cache.date_ms) <= NMEA_DATE_FRESHNESS_MS => d,
            _ => nmea::NmeaDate { day: 1, month: 1, year: 2025 },
        };
        #[cfg(feature = "nav-debug")]
        {
            nav_debug::TIME_CACHE_FRESH.store(
                if fallback_time == nmea::NmeaTime::default() && time_cache.last_time.is_none() { 0 } else { 1 },
                Ordering::Relaxed,
            );
            nav_debug::DATE_CACHE_FRESH.store(
                if time_cache.date.is_some()
                    && now_ms.wrapping_sub(time_cache.date_ms) <= NMEA_DATE_FRESHNESS_MS { 1 } else { 0 },
                Ordering::Relaxed,
            );
        }

        // Identify-then-spoof window. First SATELLITES_INVALID_AFTER_MS
        // forwards chip's authentic stream verbatim so the drone can
        // fingerprint the module from real chip output (boot dump matches
        // expected pattern). After the window we flip to forced-valid
        // rewrites: GGA/RMC/GSA/GLL/GNTXT/GSV all advertise fq=3 + target
        // coords. The same flip drives the 4074-0xFF in-place modify in
        // the RTCM router (passthrough_forward_task).
        let phase_start = FORCED3D_PHASE_START_MS.load(Ordering::Acquire);
        let phase_start = if phase_start == 0 {
            FORCED3D_PHASE_START_MS.store(now_ms, Ordering::Release);
            now_ms
        } else {
            phase_start
        };
        let elapsed = now_ms.wrapping_sub(phase_start);
        let post_id_window = (elapsed as u64)
            >= config::timers::SATELLITES_INVALID_AFTER_MS;
        // LED green during identify window, blue/yellow when spoofing.
        // Reuse SATELLITES_INVALID flag (yellow) for the spoof phase to
        // keep visual feedback distinct from passthrough (green).
        SATELLITES_INVALID.store(post_id_window, Ordering::Release);

        if !post_id_window {
            // Identify window — pass chip's NMEA verbatim. Drone needs the
            // authentic chip stream to recognise the module type.
            #[cfg(feature = "nav-debug")]
            {
                let stype = if line.len() >= 6 { &line[3..6] } else { b"---" };
                let prop = line.len() >= 2 && line[1] == b'P';
                let c = match stype {
                    b"GSV" => &nav_debug::GSV_THROUGH,
                    b"VTG" => &nav_debug::VTG_THROUGH,
                    b"ZDA" => &nav_debug::ZDA_THROUGH,
                    b"TXT" => &nav_debug::TXT_THROUGH,
                    _ if prop => &nav_debug::PROP_THROUGH,
                    _ => &nav_debug::OTHER_THROUGH,
                };
                nav_debug::bump(c);
            }
            tx_trace("nmea-id-window", line);
            enqueue(line);
            return;
        }

        if is_gga || is_rmc || is_gsa || is_gll {
            let mut work = [0u8; 320];
            let kind = if is_gga { "GGA" }
                else if is_rmc { "RMC" }
                else if is_gsa { "GSA" }
                else { "GLL" };
            let result = nmea::force_3d_fix_sentence(
                line, target, fallback_time, fallback_date, &mut work,
            );
            if let Some(n) = result {
                tx_trace("force3d-rewrite", &work[..n]);
                enqueue(&work[..n]);
                #[cfg(feature = "nav-debug")]
                {
                    let c = if is_gga { &nav_debug::GGA_REWRITE }
                        else if is_rmc { &nav_debug::RMC_REWRITE }
                        else if is_gsa { &nav_debug::GSA_REWRITE }
                        else { &nav_debug::GLL_REWRITE };
                    nav_debug::bump(c);
                }
                nav_dbg!(
                    "[nav-debug] {=str} FIX3D ok len={=u16} t={=u8}:{=u8}:{=u8} cache_t={=u8} elapsed={=u32}ms",
                    kind,
                    n as u16,
                    fallback_time.hour, fallback_time.minute, fallback_time.second,
                    if time_cache.last_time.is_some() { 1u8 } else { 0u8 },
                    elapsed,
                );
            } else {
                // Only checksum failure or unknown sentence kind reach here now.
                tx_trace("force3d-fallback", line);
                enqueue(line);
                #[cfg(feature = "nav-debug")]
                {
                    let c = if is_gga { &nav_debug::GGA_REJECT_CHECKSUM }
                        else if is_rmc { &nav_debug::RMC_REJECT_CHECKSUM }
                        else if is_gsa { &nav_debug::GSA_REJECT }
                        else { &nav_debug::GLL_REJECT_CHECKSUM };
                    nav_debug::bump(c);
                }
                nav_dbg!(
                    "[nav-debug] {=str} rewrite REJECT (checksum/unknown) → verbatim len={=u16}",
                    kind, line.len() as u16
                );
            }
            return;
        }

        // `$xxGSV` synthesis: chip in cold-boot reports 0 visible satellites,
        // contradicting our forced `fq=3 + nsats=16` GGA/RMC. During the
        // valid window replace each chip GSV with a 1-of-1 sentence showing 4
        // visible sats per constellation at realistic CNR (45–48 dB-Hz).
        // Outside the valid window pass through verbatim — chip's "no sats"
        // is consistent with our NOFIX rewrite.
        if is_gsv {
            // Post-id-window only — synth 4 sats per talker.
            let mut work = [0u8; 256];
            if let Some(n) = nmea::synthesize_gsv(line, &mut work) {
                tx_trace("gsv-synth", &work[..n]);
                enqueue(&work[..n]);
                #[cfg(feature = "nav-debug")]
                nav_debug::bump(&nav_debug::GSV_SYNTH);
                nav_dbg!(
                    "[nav-debug] GSV SYNTH ok len={=u16} talker={=[u8]:a}",
                    n as u16,
                    &line[1..3],
                );
            } else {
                tx_trace("gsv-fallback", line);
                enqueue(line);
                #[cfg(feature = "nav-debug")]
                nav_debug::bump(&nav_debug::GSV_REJECT);
                nav_dbg!(
                    "[nav-debug] GSV synth REJECT (checksum/struct) → verbatim len={=u16}",
                    line.len() as u16,
                );
            }
            return;
        }

        // `$xxTXT,01,01,01,…` periodic status: rewrite field 13 (fix indicator)
        // to track the FIX3D / NOFIX phase of our forced output. Drone weights
        // this indicator alongside GGA fq when deciding whether the chip's
        // claim of fix is trustworthy. Without this rewrite the chip's `0`
        // (no-fix) leaks through and contradicts our `fq=3` GGA.
        if is_gntxt_status {
            // Post-id-window only — flip field 13 to '3' (3D fix indicator).
            let mut work = [0u8; 256];
            if let Some(n) = nmea::rewrite_gntxt_status(line, b'3', &mut work) {
                tx_trace("gntxt-fix3d", &work[..n]);
                enqueue(&work[..n]);
                #[cfg(feature = "nav-debug")]
                nav_debug::bump(&nav_debug::GNTXT_STATUS_REWRITE);
                nav_dbg!(
                    "[nav-debug] GNTXT FIX3D ok len={=u16}",
                    n as u16,
                );
            } else {
                tx_trace("gntxt-fallback", line);
                enqueue(line);
                #[cfg(feature = "nav-debug")]
                nav_debug::bump(&nav_debug::GNTXT_STATUS_REJECT);
                nav_dbg!(
                    "[nav-debug] GNTXT rewrite REJECT (checksum/struct) → verbatim len={=u16}",
                    line.len() as u16
                );
            }
            return;
        }

        // Non-coord NMEA — forwarded verbatim. Tag for stats.
        #[cfg(feature = "nav-debug")]
        {
            let stype = if line.len() >= 6 { &line[3..6] } else { b"---" };
            let prop = line.len() >= 2 && line[1] == b'P';
            let c = match stype {
                b"GSV" => &nav_debug::GSV_THROUGH,
                b"VTG" => &nav_debug::VTG_THROUGH,
                b"ZDA" => &nav_debug::ZDA_THROUGH,
                b"TXT" => &nav_debug::TXT_THROUGH,
                _ if prop => &nav_debug::PROP_THROUGH,
                _ => &nav_debug::OTHER_THROUGH,
            };
            nav_debug::bump(c);
            // First 8 bytes give a clear hint about message family.
            let head_len = line.len().min(8);
            nav_dbg!(
                "[nav-debug] passthrough head={=[u8]:a} len={=u16}",
                &line[..head_len], line.len() as u16
            );
        }
        tx_trace("nmea-passthrough", line);
        enqueue(line);
        return;
    }

    let apply_offset = policy == NmeaOutputPolicy::Offset;

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

    // Snapshot of `last_gga_sod` BEFORE the primary GGA branch updates it. Both
    // the primary's midnight-rollover guard and secondary trigger #2's own
    // rollover guard must compare the *current* GGA's second-of-day against
    // the *previous* GGA's, not against itself — without this snapshot the
    // primary would overwrite the cell first and secondary #2 would always see
    // `prev == sod`, defeating the rollover skip and synthesising a bogus
    // 24 h backward jump on every in-flight fix-loss with empty time field.
    let prev_gga_sod = time_cache.last_gga_sod;

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
                let rolled_over = prev_gga_sod
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
                    // Use the pre-update snapshot: the primary GGA branch
                    // has already overwritten `time_cache.last_gga_sod` to
                    // the current sod, so reading it here would always give
                    // `prev == sod` and break the rollover skip.
                    let rolled_over = prev_gga_sod
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
            tx_trace("nmea-drop-spoof", line);
            return;
        }
        if apply_offset && is_gll {
            tx_trace("nmea-drop-offset", line);
            return;
        }
        tx_trace("nmea-passthrough", line);
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
        tx_trace("nmea-verbatim", line);
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
            tx_trace(if is_gga { "spoof-rebuild-gga" } else { "spoof-rebuild-rmc" }, &buf[..n]);
            enqueue(&buf[..n]);
        }
        return;
    }

    // Plain offset rewrite — patch coords in place, fall back to verbatim on
    // buffer/parse failure.
    let mut work = [0u8; 320];
    if let Some(new_len) = rewrite_coords(line, coords, is_gga, &mut work) {
        tx_trace("offset-rewrite", &work[..new_len]);
        enqueue(&work[..new_len]);
        return;
    }
    tx_trace("nmea-fallback", line);
    enqueue(line);
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

async fn apply_operating_mode(
    new_mode: OperatingMode,
    source: ModeChangeSource,
    detail: u8,
    flash: &'static FlashMutex,
) {
    let mut flash = flash.lock().await;
    let current = OperatingMode::load();
    if new_mode == current {
        match source {
            ModeChangeSource::Button => info!("Mode unchanged: {:?} (clicks={})", new_mode, detail),
            ModeChangeSource::Debug => info!("Mode unchanged: {:?} (debug request={})", new_mode, detail),
        }
        return;
    }

    new_mode.store();
    SATELLITES_INVALID.store(false, Ordering::Release);
    SPOOF_DETECTED.store(false, Ordering::Release);
    SPOOF_RECOVERY_START_MS.store(0, Ordering::Release);
    SPOOF_DETECTOR_RESET.store(true, Ordering::Release);

    let saved = flash_storage::save_mode(&mut flash, new_mode as u8).await;
    match source {
        ModeChangeSource::Button => info!("Mode: {:?} -> {:?} (clicks={})", current, new_mode, detail),
        ModeChangeSource::Debug => info!("Mode: {:?} -> {:?} (debug request={})", current, new_mode, detail),
    }
    if !saved {
        warn!("Failed to save mode to flash");
    }
}

#[embassy_executor::task]
async fn debug_mode_request_task(flash: &'static FlashMutex) {
    loop {
        let request = DEBUG_MODE_REQUEST.swap(DEBUG_MODE_REQUEST_IDLE, Ordering::AcqRel);
        if request != DEBUG_MODE_REQUEST_IDLE {
            if request <= OperatingMode::PassthroughOffset as u8 {
                let new_mode = OperatingMode::from_u8(request);
                info!("debug mode request: {} -> {:?}", request, new_mode);
                apply_operating_mode(new_mode, ModeChangeSource::Debug, request, flash).await;
            } else {
                warn!("invalid debug mode request: {}", request);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn button_task(mut btn: Flex<'static>, flash: &'static FlashMutex) {
    use config::button::*;

    btn.set_as_input();
    btn.set_pull(Pull::Down);

    Timer::after(Duration::from_millis(1000)).await;

    let mut click_count: u8 = 0;
    let mut last_click_time: Option<Instant> = None;
    let mut was_high = false;

    loop {
        Timer::after(Duration::from_millis(POLL_PERIOD_MS)).await;
        let is_high = btn.is_high();

        if click_count > 0 {
            if let Some(last) = last_click_time {
                if last.elapsed().as_millis() >= MULTI_CLICK_TIMEOUT_MS {
                    let new_mode = match click_count {
                        1 => OperatingMode::Emulation,
                        2 => OperatingMode::Passthrough,
                        3 => OperatingMode::PassthroughRaw,
                        4 => OperatingMode::PassthroughOffset,
                        _ => OperatingMode::PassthroughOffset,
                    };
                    apply_operating_mode(new_mode, ModeChangeSource::Button, click_count, flash).await;
                    click_count = 0;
                    last_click_time = None;
                }
            }
        }

        if is_high && !was_high {
            info!("button_task: edge detected (0->1), debouncing");
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
            if btn.is_high() {
                click_count = click_count.saturating_add(1).min(4);
                info!("button_task: click #{}", click_count);
                info!("button_task: waiting for release...");

                let mut high_count: u32 = 0;
                loop {
                    if !btn.is_high() {
                        info!("button_task: released!");
                        break;
                    }
                    Timer::after(Duration::from_millis(50)).await;
                    high_count += 1;

                    if high_count.is_multiple_of(5) {
                        info!("button_task: still HIGH, attempting E9 discharge kick");
                        btn.set_as_output();
                        btn.set_low();
                        Timer::after(Duration::from_micros(50)).await;
                        btn.set_as_input();
                        btn.set_pull(Pull::Down);
                        Timer::after(Duration::from_micros(50)).await;
                    }
                }

                last_click_time = Some(Instant::now());
            }
        }
        was_high = is_high;
    }
}
