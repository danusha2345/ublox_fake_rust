//! End-to-end pipeline test for the Unicore NMEA spoof-detection branch.
//!
//! The host-side `Pipeline` struct below is a faithful mirror of
//! `src/main_unicore.rs::process_nmea_line` (passthrough mode, `apply_offset = false`),
//! lifted out of its async/embassy/global-state shell so we can drive a
//! deterministic multi-phase scenario:
//!
//!   Phase 0 (5 s) — no satellites: chip emits `fq=0` GGAs and V-status RMCs.
//!   Phase 1 (5 s) — 16 satellites static at position A (Golden Beach FL).
//!   Phase 2 (4 s) — satellites drop out again, mirroring Phase 0.
//!   Phase 3 (3 s) — fresh stream at position B (5 km north of A) with
//!                   `gnss_time` jumped backwards by ~1 hour.
//!
//! The post-processing output of every frame is captured and inspected.
//! The CRITICAL assertion is in Phase 3: after the spoofer flips the
//! incoming stream, **no Phase 3 coordinate must reach the drone**.
//! The only acceptable outputs in Phase 3 are spoof-rebuilt sentences
//! carrying the captured `LAST_GOOD` (≈ position A).

use spoof_detector_tests::pos_history::{DynamicOffset, PositionBuffer};
use spoof_detector_tests::spoof_detector::{
    self, AnalysisResult, FixType, GnssTime, Position, SpoofDetector,
};
use spoof_detector_tests::unicore_nmea as nmea;

// ---- constants mirrored from src/main_unicore.rs ---------------------------
const SPOOF_LOOKBACK_SECONDS: u32 = 2;
const SPOOF_RECOVERY_TIMEOUT_MS: u32 = 5_000;
const NMEA_DATE_FRESHNESS_MS: u32 = 1_500;
const TIME_JUMP_BACK_S: i64 = 1;
const SPOOF_INVALID_NSATS: u8 = 1;
const SPOOF_HIGH_HDOP_X100: u16 = 9_999;
const SPOOF_DEFAULT_GEOID_SEP_MM: i32 = -30_000;

// ---- pipeline state --------------------------------------------------------

#[derive(Default)]
struct TimeCache {
    date: Option<nmea::NmeaDate>,
    date_ms: u32,
    last_gga_sod: Option<u32>,
    last_frame_gnss_unix: Option<i64>,
}

struct Pipeline {
    detector: SpoofDetector,
    pos_buffer: PositionBuffer,
    spoofed: bool,
    last_good_lat: i32,
    last_good_lon: i32,
    last_good_alt: i32,
    recovery_start_ms: u32,
    time_cache: TimeCache,
    #[allow(dead_code)] // mirror of `apply_offset` arg, always false in this test
    dynamic_offset: Option<DynamicOffset>,
}

impl Pipeline {
    fn new() -> Self {
        Self {
            detector: SpoofDetector::new(),
            pos_buffer: PositionBuffer::new(),
            spoofed: false,
            last_good_lat: 0,
            last_good_lon: 0,
            last_good_alt: 0,
            recovery_start_ms: 0,
            time_cache: TimeCache::default(),
            dynamic_offset: None,
        }
    }

    fn capture_last_good(&mut self, now_ms: u32) {
        if let Some((gl, go, ga)) = self
            .pos_buffer
            .get_position_at(SPOOF_LOOKBACK_SECONDS, now_ms)
        {
            self.last_good_lat = gl;
            self.last_good_lon = go;
            self.last_good_alt = ga;
        }
    }

    /// Mirror of `process_nmea_line` for `apply_offset = false`. Returns
    /// `Some(bytes)` if the frame is forwarded downstream, `None` if dropped.
    fn process(&mut self, line: &[u8], now_ms: u32) -> Option<Vec<u8>> {
        let is_gga = line.len() >= 6 && &line[3..6] == b"GGA";
        let is_rmc = line.len() >= 6 && &line[3..6] == b"RMC";

        let parsed: Option<nmea::NmeaFix> = if is_gga {
            nmea::parse_gga(line)
        } else if is_rmc {
            nmea::parse_rmc(line)
        } else {
            None
        };

        // RMC date cache (year > 0 only)
        if is_rmc {
            if let Some(fix) = parsed {
                if fix.checksum_ok && fix.date.year > 0 {
                    self.time_cache.date = Some(fix.date);
                    self.time_cache.date_ms = now_ms;
                }
            }
        }

        // Snapshot the previous GGA's second-of-day BEFORE the primary GGA
        // branch updates the cache (mirrors `prev_gga_sod` in
        // `src/main_unicore.rs::process_nmea_line`). Both rollover guards
        // (primary and secondary #2) compare against this snapshot.
        let prev_gga_sod = self.time_cache.last_gga_sod;

        // Primary detector branch — GGA only.
        if is_gga {
            if let Some(fix) = parsed {
                if fix.checksum_ok {
                    let fix_type = if fix.fix_quality > 0 {
                        FixType::Fix3D
                    } else {
                        FixType::NoFix
                    };

                    if fix_type.has_3d_fix() {
                        self.pos_buffer
                            .push(fix.lat_1e7, fix.lon_1e7, fix.alt_mm, now_ms);
                    }

                    let sod = fix.time.hour as u32 * 3600
                        + fix.time.minute as u32 * 60
                        + fix.time.second as u32;
                    let stale_date = self.time_cache.date.is_none()
                        || now_ms.wrapping_sub(self.time_cache.date_ms)
                            > NMEA_DATE_FRESHNESS_MS;
                    let rolled_over = prev_gga_sod
                        .map_or(false, |prev| prev > sod && prev - sod > 12 * 3600);
                    self.time_cache.last_gga_sod = Some(sod);

                    let gnss_time = if stale_date || rolled_over {
                        None
                    } else {
                        self.time_cache.date.map(|d| GnssTime {
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

                    let pos = Position {
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

                    let result = self.detector.analyze(pos);
                    let was_spoofed = self.spoofed;
                    let is_spoofed = result == AnalysisResult::Spoofed;

                    if is_spoofed && !was_spoofed {
                        self.capture_last_good(now_ms);
                        self.spoofed = true;
                        self.recovery_start_ms = 0;
                    } else if !is_spoofed && was_spoofed {
                        if self.recovery_start_ms == 0 {
                            self.recovery_start_ms = now_ms;
                        } else if now_ms.wrapping_sub(self.recovery_start_ms)
                            >= SPOOF_RECOVERY_TIMEOUT_MS
                        {
                            self.spoofed = false;
                            self.recovery_start_ms = 0;
                        }
                    } else if is_spoofed {
                        self.recovery_start_ms = 0;
                    }
                }
            }
        }

        // Secondary trigger #2 — time-jump (catches no-fix/V-status time spoof).
        let have_fix_reference = self.pos_buffer.get_position_at(0, now_ms).is_some();
        if (is_gga || is_rmc) && !self.spoofed {
            if let Some(fix) = parsed {
                if fix.checksum_ok {
                    let frame_date = if is_rmc && fix.date.year > 0 {
                        Some(fix.date)
                    } else if is_gga {
                        let sod = fix.time.hour as u32 * 3600
                            + fix.time.minute as u32 * 60
                            + fix.time.second as u32;
                        let stale = self.time_cache.date.is_none()
                            || now_ms.wrapping_sub(self.time_cache.date_ms)
                                > NMEA_DATE_FRESHNESS_MS;
                        // Same snapshot as the primary branch — see
                        // `prev_gga_sod` above.
                        let rolled_over = prev_gga_sod
                            .map_or(false, |prev| prev > sod && prev - sod > 12 * 3600);
                        if stale || rolled_over {
                            None
                        } else {
                            self.time_cache.date
                        }
                    } else {
                        None
                    };

                    if let Some(d) = frame_date {
                        let curr_unix = GnssTime {
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
                            if let Some(prev_unix) = self.time_cache.last_frame_gnss_unix {
                                if curr_unix < prev_unix - TIME_JUMP_BACK_S {
                                    self.capture_last_good(now_ms);
                                    self.spoofed = true;
                                    self.recovery_start_ms = 0;
                                }
                            }
                        }

                        if !self.spoofed {
                            self.time_cache.last_frame_gnss_unix = Some(curr_unix);
                        }
                    }
                }
            }
        }

        // Secondary trigger #1 — coord-jump vs newest pos_buffer entry.
        if (is_gga || is_rmc) && !self.spoofed {
            if let Some(fix) = parsed {
                if fix.checksum_ok && (fix.lat_1e7 != 0 || fix.lon_1e7 != 0) {
                    if let Some((prev_lat, prev_lon, _)) =
                        self.pos_buffer.get_position_at(0, now_ms)
                    {
                        let jump_m = SpoofDetector::calc_distance(
                            prev_lat,
                            prev_lon,
                            fix.lat_1e7,
                            fix.lon_1e7,
                        );
                        if jump_m > spoof_detector::thresholds::TELEPORT_M {
                            self.capture_last_good(now_ms);
                            self.spoofed = true;
                            self.recovery_start_ms = 0;
                        }
                    }
                }
            }
        }

        // ---- output decision ------------------------------------------------
        if !(is_gga || is_rmc) {
            // GSA/GSV/etc. — forward verbatim (we don't drop GLL/VTG in this test).
            return Some(line.to_vec());
        }

        if !self.spoofed {
            // Passthrough mode + not spoofed → forward verbatim.
            return Some(line.to_vec());
        }

        // Spoofed: rebuild GGA/RMC with LAST_GOOD coords + degraded fix.
        let coords = (self.last_good_lat, self.last_good_lon, self.last_good_alt);
        let existing = parsed.unwrap_or_default();
        let mut buf = [0u8; 256];
        let n = if is_gga {
            let g = nmea::GgaFields {
                time: existing.time,
                lat_1e7: coords.0,
                lon_1e7: coords.1,
                fix_quality: 0,
                nsats: SPOOF_INVALID_NSATS,
                hdop_x100: SPOOF_HIGH_HDOP_X100,
                alt_mm: coords.2,
                geoid_sep_mm: SPOOF_DEFAULT_GEOID_SEP_MM,
            };
            nmea::build_gga(&mut buf, nmea::Talker::Gn, &g)
        } else {
            let r = nmea::RmcFields {
                time: existing.time,
                valid: false,
                lat_1e7: coords.0,
                lon_1e7: coords.1,
                sog_knots_x1000: 0,
                cog_deg_x100: 0,
                date: existing.date,
                mode: b'N',
            };
            nmea::build_rmc(&mut buf, nmea::Talker::Gn, &r)
        };
        if n > 0 {
            Some(buf[..n].to_vec())
        } else {
            None
        }
    }
}

// ---- frame builders --------------------------------------------------------

const A_LAT: i32 = 259_664_430; // 25.9664430°N — Golden Beach FL
const A_LON: i32 = -801_223_710; // 80.1223710°W
const A_ALT_MM: i32 = 100_000;

// B = ~5 km north of A (well past TELEPORT_M = 2 km)
const B_LAT: i32 = A_LAT + 449_204;
const B_LON: i32 = A_LON;
const B_ALT_MM: i32 = 100_000;

fn make_gga(
    lat_1e7: i32,
    lon_1e7: i32,
    alt_mm: i32,
    fq: u8,
    nsats: u8,
    h: u8,
    m: u8,
    s: u8,
) -> Vec<u8> {
    let mut buf = [0u8; 128];
    let n = nmea::build_gga(
        &mut buf,
        nmea::Talker::Gn,
        &nmea::GgaFields {
            time: nmea::NmeaTime {
                hour: h,
                minute: m,
                second: s,
                centis: 0,
            },
            lat_1e7,
            lon_1e7,
            fix_quality: fq,
            nsats,
            hdop_x100: 99,
            alt_mm,
            geoid_sep_mm: -30_000,
        },
    );
    assert!(n > 0);
    buf[..n].to_vec()
}

fn make_rmc(
    lat_1e7: i32,
    lon_1e7: i32,
    valid: bool,
    h: u8,
    m: u8,
    s: u8,
    day: u8,
    mon: u8,
    year: u16,
) -> Vec<u8> {
    let mut buf = [0u8; 128];
    let n = nmea::build_rmc(
        &mut buf,
        nmea::Talker::Gn,
        &nmea::RmcFields {
            time: nmea::NmeaTime {
                hour: h,
                minute: m,
                second: s,
                centis: 0,
            },
            valid,
            lat_1e7,
            lon_1e7,
            sog_knots_x1000: 0,
            cog_deg_x100: 0,
            date: nmea::NmeaDate { day, month: mon, year },
            mode: if valid { b'A' } else { b'N' },
        },
    );
    assert!(n > 0);
    buf[..n].to_vec()
}

/// Real UC6580I cold-start no-fix sentences (Phase 0 only — boot, no time/date).
/// Using `build_rmc(.., valid=false, year=0)` would produce `000000` for date,
/// and the parser turns that into `year=2000`, breaking the `year > 0` cache
/// gate. The chip emits truly empty fields, so we use its exact bytes here.
const NO_FIX_RMC_BOOT: &[u8] = b"$GNRMC,,V,,,,,,,,,,N,V*37";
const NO_FIX_GGA_BOOT: &[u8] = b"$GNGGA,,,,,,0,00,99.99,,,,,,*56";

/// In-flight fix-loss sentences (Phase 2): chip stays in TimeOnly mode and
/// keeps emitting time (and RMC date) verbatim while reporting `fq=0` / `V`
/// status with empty coords. We keep the `last_*_unix`/`last_gga_sod` caches
/// consistent with Phase 1 so the dropout itself is not interpreted as a
/// time anomaly.
fn make_no_fix_rmc_with_time(h: u8, m: u8, s: u8, day: u8, mon: u8, year: u16) -> Vec<u8> {
    // build_rmc writes lat/lon as `0.0,N,0.0,E` when fields are zero — those
    // round-trip to `lat_1e7=0, lon_1e7=0` so the secondary coord-jump check's
    // non-zero-coord guard skips them. RMC keeps a valid date so the cache
    // stays fresh.
    make_rmc(0, 0, false, h, m, s, day, mon, year)
}

fn make_no_fix_gga_with_time(h: u8, m: u8, s: u8) -> Vec<u8> {
    make_gga(0, 0, 0, 0, 0, h, m, s)
}

// ---- the test --------------------------------------------------------------

/// Reproduces a real-world scenario the user observed:
///   - 5 s of no-fix
///   - 5 s of static fixes (16 sats) at position A
///   - 4 s of no-fix (sat dropout)
///   - sudden static stream at position B with backward-jumped time
///
/// The spoof-detection pipeline must not let any Phase-3 coordinate reach
/// the drone. Acceptable Phase-3 outputs: spoof-rebuilt GGA/RMC carrying
/// `LAST_GOOD` (≈ position A), or dropped frames.
#[test]
fn unicore_pipeline_filters_spoof_after_dropout() {
    let mut pipe = Pipeline::new();
    let mut now: u32 = 1_000;

    // ----- Phase 0: 5 s cold-start no satellites (25 epochs @ 5 Hz) ---------
    // Cold start: empty time + empty date. time_cache.date stays None so no
    // time-based check can fire even on the GGA branch.
    for _ in 0..25 {
        pipe.process(NO_FIX_RMC_BOOT, now);
        pipe.process(NO_FIX_GGA_BOOT, now + 50);
        now += 200;
    }
    assert!(!pipe.spoofed, "phase 0: no spoof should be raised on cold no-fix");

    // ----- Phase 1: 5 s @ 5 Hz, 16 sats, static at A ------------------------
    // GNSS time advances 1 s per 5 system frames (5 Hz NMEA cadence).
    for i in 0..25 {
        let s = (i / 5) as u8;
        let rmc = make_rmc(A_LAT, A_LON, true, 12, 0, s, 22, 4, 2026);
        let gga = make_gga(A_LAT, A_LON, A_ALT_MM, 1, 16, 12, 0, s);

        let out_rmc = pipe.process(&rmc, now).expect("phase1 rmc forwarded");
        let out_gga = pipe.process(&gga, now + 50).expect("phase1 gga forwarded");

        // Real coords must reach the drone verbatim.
        let p_g = nmea::parse_gga(&out_gga).expect("parse phase1 gga");
        assert_eq!(p_g.lat_1e7, A_LAT, "phase1 frame {}: GGA lat mismatch", i);
        assert_eq!(p_g.lon_1e7, A_LON);
        assert_eq!(p_g.fix_quality, 1);
        assert_eq!(p_g.nsats, 16);

        let p_r = nmea::parse_rmc(&out_rmc).expect("parse phase1 rmc");
        assert_eq!(p_r.lat_1e7, A_LAT, "phase1 frame {}: RMC lat mismatch", i);
        assert_eq!(p_r.lon_1e7, A_LON);
        assert_eq!(p_r.fix_quality, 1);

        now += 200;
    }
    assert!(!pipe.spoofed, "phase 1: static real fix must not raise spoof");

    // ----- Phase 2: 4 s in-flight fix-loss (TimeOnly) -----------------------
    // Chip preserves the last valid time and date but drops to fq=0 / V status.
    // GNSS time advances by 1 s per 5 frames continuing from Phase 1's last
    // value (12:00:04 → 12:00:08 by the end of Phase 2).
    let mut hold_s = 4u8; // last value seen at end of phase 1
    for i in 0..20 {
        if i > 0 && i % 5 == 0 {
            hold_s = hold_s.wrapping_add(1);
        }
        let rmc = make_no_fix_rmc_with_time(12, 0, hold_s, 22, 4, 2026);
        let gga = make_no_fix_gga_with_time(12, 0, hold_s);
        let _ = pipe.process(&rmc, now);
        let _ = pipe.process(&gga, now + 50);
        // We do NOT assert on the per-frame output: depending on timing edge
        // cases the pipeline may emit either a forwarded no-fix sentence
        // (lat=0) or a spoof rebuild carrying LAST_GOOD≈A. Either is safe
        // because no Phase-3 coordinate exists yet to leak. The Phase-3
        // assertions below are what matters.
        now += 200;
    }

    // ----- Phase 3: spoofed static at B, backward time, 3 s of frames -------
    // Time jumped backwards by 1 hour vs Phase 1 (12:00 → 11:00).
    let spoof_h = 11u8;
    let spoof_m = 0u8;
    let spoof_d = 22u8;
    let spoof_mon = 4u8;
    let spoof_year = 2026u16;

    let mut leaked_lat_count = 0;
    let mut leaked_fix_quality_count = 0;
    let mut total_phase3_outputs = 0;

    for i in 0..15 {
        let s = (i / 5) as u8;
        let rmc = make_rmc(B_LAT, B_LON, true, spoof_h, spoof_m, s, spoof_d, spoof_mon, spoof_year);
        let gga = make_gga(B_LAT, B_LON, B_ALT_MM, 1, 16, spoof_h, spoof_m, s);

        let out_rmc = pipe.process(&rmc, now).expect("phase3 rmc forwarded");
        let out_gga = pipe.process(&gga, now + 50).expect("phase3 gga forwarded");

        // CRITICAL — no Phase 3 coordinate must reach the drone.
        let p_g = nmea::parse_gga(&out_gga).expect("parse phase3 gga");
        let p_r = nmea::parse_rmc(&out_rmc).expect("parse phase3 rmc");
        total_phase3_outputs += 2;

        if p_g.lat_1e7 == B_LAT || p_r.lat_1e7 == B_LAT {
            leaked_lat_count += 1;
        }
        if p_g.fix_quality > 0 || p_r.fix_quality > 0 {
            leaked_fix_quality_count += 1;
        }

        // For frames after the first detection, the pipeline must emit the
        // degraded spoof rebuild (fq=0, nsats=SPOOF_INVALID_NSATS, coords=A).
        if pipe.spoofed {
            assert_eq!(
                p_g.fix_quality, 0,
                "phase3 frame {}: spoofed GGA must report fq=0", i
            );
            assert_eq!(
                p_g.nsats, SPOOF_INVALID_NSATS,
                "phase3 frame {}: spoofed GGA must report nsats={}", i, SPOOF_INVALID_NSATS
            );
            assert_ne!(p_g.lat_1e7, B_LAT, "phase3 frame {}: GGA leaked B_LAT", i);
            assert_ne!(p_r.lat_1e7, B_LAT, "phase3 frame {}: RMC leaked B_LAT", i);
            // LAST_GOOD captured from the lookback should be ≈ A.
            assert!(
                (p_g.lat_1e7 - A_LAT).abs() < 50_000, // tolerance ~50 m
                "phase3 frame {}: GGA lat {} not near A {}", i, p_g.lat_1e7, A_LAT
            );
        }

        now += 200;
    }

    assert!(
        pipe.spoofed,
        "phase 3: spoof must be detected by the end of the 3 s window"
    );
    assert_eq!(
        leaked_lat_count, 0,
        "phase 3: {} of {} outputs leaked the spoofed B_LAT downstream",
        leaked_lat_count, total_phase3_outputs
    );
    assert_eq!(
        leaked_fix_quality_count, 0,
        "phase 3: {} of {} outputs reported fq>0 instead of degraded fix",
        leaked_fix_quality_count, total_phase3_outputs
    );

    // LAST_GOOD captured at the spoof edge should be near A.
    assert!(
        (pipe.last_good_lat - A_LAT).abs() < 50_000,
        "LAST_GOOD lat {} not near A {}",
        pipe.last_good_lat, A_LAT
    );
    assert!(
        (pipe.last_good_lon - A_LON).abs() < 50_000,
        "LAST_GOOD lon {} not near A {}",
        pipe.last_good_lon, A_LON
    );
}

/// Case B regression — single-frame GGA leak after long fix-loss.
///
/// Conditions for the leak to slip past every detection layer (pre-fix):
///   - 7 s of in-flight no-fix with empty time/date (stale RMC cache).
///   - First Phase-3 frame is GGA (no RMC ahead of it in the same epoch).
///   - B is < TELEPORT_M (2 km) from A so the gap-branch teleport check
///     does not fire.
///
/// Pre-fix path:
///   primary gap branch → no teleport, no gnss_time → GapReset
///   secondary #2 → stale cache → frame_date None → skip
///   secondary #1 → pos_buffer.get_position_at(0) returns the just-pushed B
///                   → distance B↔B == 0 → skip
///   output → forward GGA verbatim → leak.
///
/// Post-fix: gap branch's new average-speed check (Check 1b in
/// `spoof_detector::analyze`) catches the implausible 1500m / 7s ≈ 214 m/s
/// average ground speed → SPOOF → output rebuilt with LAST_GOOD ≈ A.
#[test]
fn unicore_pipeline_blocks_case_b_one_frame_leak() {
    let mut pipe = Pipeline::new();
    let mut now: u32 = 1_000;

    // Phase 1 — establish A with valid time, calibrate detector.
    for i in 0..25 {
        let s = (i / 5) as u8;
        pipe.process(&make_rmc(A_LAT, A_LON, true, 12, 0, s, 22, 4, 2026), now);
        pipe.process(&make_gga(A_LAT, A_LON, A_ALT_MM, 1, 16, 12, 0, s), now + 50);
        now += 200;
    }
    assert!(!pipe.spoofed);

    // Phase 2 — 7 s of cold-style no-fix (RMC has empty date, so the cache
    // stops being refreshed and goes stale after NMEA_DATE_FRESHNESS_MS).
    for _ in 0..35 {
        pipe.process(NO_FIX_RMC_BOOT, now);
        pipe.process(NO_FIX_GGA_BOOT, now + 50);
        now += 200;
    }

    // Phase 3 first frame — GGA only, B = A + 1.5 km north (< TELEPORT_M).
    // No RMC sent before it, so the cache stays stale and secondary #2
    // cannot derive a frame_date.
    let b_close_lat = A_LAT + 134_761; // ~1.5 km north
    let b_close_lon = A_LON;
    let gga = make_gga(b_close_lat, b_close_lon, A_ALT_MM, 1, 16, 11, 0, 0);
    let out = pipe.process(&gga, now).expect("forwarded");
    let p = nmea::parse_gga(&out).expect("parse");

    assert_ne!(
        p.lat_1e7, b_close_lat,
        "Case B: GGA-first sub-teleport jump after long gap LEAKED B_LAT downstream"
    );
    assert!(
        pipe.spoofed,
        "Case B: spoof flag must be set on the first leaking frame"
    );
}

/// Third-artefact regression — no false positive on in-flight fix-loss with
/// an empty GGA time field while the RMC date cache is still fresh.
///
/// Pre-fix: the primary GGA branch updated `time_cache.last_gga_sod` BEFORE
/// secondary trigger #2 read it, so secondary #2's rollover guard always saw
/// `prev == sod` and never skipped a midnight rollover. The first no-fix GGA
/// after a valid Phase 1 (sod 43204 → 0) was then interpreted as a 12 h
/// backward jump and SPOOF was raised even though nothing was actually wrong.
///
/// Post-fix: both rollover guards read the same pre-update snapshot; the no-
/// fix GGA is correctly identified as a midnight-rollover case (sod jump
/// > 12 h while RMC date hasn't caught up), `frame_date` is None, no time-
/// jump check runs, and SPOOF stays clear.
#[test]
fn unicore_pipeline_no_false_spoof_on_in_flight_fix_loss() {
    let mut pipe = Pipeline::new();
    let mut now: u32 = 1_000;

    // Phase 1 — establish A with valid time, calibrate detector.
    for i in 0..25 {
        let s = (i / 5) as u8;
        pipe.process(&make_rmc(A_LAT, A_LON, true, 12, 0, s, 22, 4, 2026), now);
        pipe.process(&make_gga(A_LAT, A_LON, A_ALT_MM, 1, 16, 12, 0, s), now + 50);
        now += 200;
    }
    assert!(!pipe.spoofed);

    // Single epoch of cold-style no-fix immediately after Phase 1. Cache is
    // still fresh (250 ms < 1500 ms), GGA time field is empty.
    pipe.process(NO_FIX_RMC_BOOT, now);
    pipe.process(NO_FIX_GGA_BOOT, now + 50);

    assert!(
        !pipe.spoofed,
        "in-flight fix-loss with empty time spuriously raised SPOOF \
         (rolled_over read-after-update bug in secondary #2)"
    );
}
