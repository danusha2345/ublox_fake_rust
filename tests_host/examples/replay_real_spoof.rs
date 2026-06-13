// Throwaway harness: replay REAL recorded spoofing (NMEA logs) through the
// production SpoofDetector::analyze(). NOT part of `make test`.
//
//   cargo run --example replay_real_spoof -- <file1.log> [file2.log ...]
//
// Caveat: the recordings are NMEA from a MediaTek MT6686 chip; the firmware
// detector normally consumes UBX NAV-PVT/NAV-SAT from a u-blox. We map NMEA ->
// Position as faithfully as we can. The detector's relative anchors (origin /
// last_good drift) assume a genuine first fix; these captures are spoofed from
// the first epoch, so those anchors calibrate on fake data (known limitation).
// Absolute checks (speed, alt-jump, CNO uniformity, reported-velocity ceiling)
// are unaffected by that and are what we're really probing here.

use spoof_detector_tests::spoof_detector::*;
use std::collections::BTreeMap;

const KNOTS_TO_MS: f64 = 1852.0 / 3600.0; // 0.514444 m/s

#[derive(Default, Clone)]
struct Epoch {
    // time
    hms: Option<(u8, u8, u8, u16)>, // h,m,s,centi
    date: Option<(u8, u8, u16)>,    // day, month, year
    // position
    lat_1e7: Option<i32>,
    lon_1e7: Option<i32>,
    alt_mm: Option<i32>,
    speed_knots: Option<f64>,
    course: Option<f64>,
    fix_quality: u8, // GGA field 6 (0 = no fix)
    nsats: u8,       // GGA field 7
    fix_mode: u8,    // GSA: max over constellations (1 nofix, 2 = 2D, 3 = 3D)
    // CNO grouped by constellation talker (GP/GL/GB/GA/GQ...)
    cno: BTreeMap<String, Vec<u8>>,
}

fn parse_latlon(val: &str, hemi: &str) -> Option<i32> {
    if val.is_empty() {
        return None;
    }
    let v: f64 = val.parse().ok()?;
    let deg = (v / 100.0).trunc();
    let min = v - deg * 100.0;
    let mut dec = deg + min / 60.0;
    if hemi == "S" || hemi == "W" {
        dec = -dec;
    }
    Some((dec * 1e7).round() as i32)
}

fn parse_hms(field: &str) -> Option<(u8, u8, u8, u16)> {
    if field.len() < 6 {
        return None;
    }
    let h = field[0..2].parse().ok()?;
    let m = field[2..4].parse().ok()?;
    let s = field[4..6].parse().ok()?;
    let centi = if field.len() > 7 {
        (field[7..].parse::<f64>().ok()? * 10.0_f64.powi(2 - (field.len() as i32 - 7))) as u16
    } else {
        0
    };
    Some((h, m, s, centi))
}

fn checksum_ok(line: &str) -> bool {
    // $....*HH
    let body = line.strip_prefix('$').unwrap_or(line);
    if let Some(star) = body.rfind('*') {
        let (data, csum) = body.split_at(star);
        let csum = &csum[1..];
        let want = u8::from_str_radix(csum.trim(), 16).ok();
        let got = data.bytes().fold(0u8, |a, b| a ^ b);
        return want == Some(got);
    }
    false
}

fn talker(typ: &str) -> String {
    // "GPGSV" -> "GP", "GNRMC" -> "GN"
    typ.chars().take(2).collect()
}

fn flush(ep: &Epoch, idx: &mut u32, det: &mut SpoofDetector, log: &str, stats: &mut Stats) {
    // Need at least lat/lon to build a Position; otherwise skip.
    let (lat, lon) = match (ep.lat_1e7, ep.lon_1e7) {
        (Some(a), Some(b)) => (a, b),
        _ => return,
    };
    let (h, m, s, centi) = match ep.hms {
        Some(t) => t,
        None => return,
    };
    let (day, month, year) = ep.date.unwrap_or((13, 6, 2026));

    // system monotonic ms derived from UTC-of-day (incl. centiseconds, since the
    // log is 2 Hz: .00/.50 within one second must NOT collapse to dt=0). Faithful
    // to a time-honest spoofer; lets real >5s gaps trigger the gap path.
    let utc_secs = h as u32 * 3600 + m as u32 * 60 + s as u32;
    let time_ms = utc_secs.wrapping_mul(1000) + (centi as u32) * 10;

    // fix type: GGA quality 0 => NoFix; else use GSA mode
    let fix_type = if ep.fix_quality == 0 {
        FixType::NoFix
    } else {
        FixType::from_u8(ep.fix_mode)
    };

    let reported_speed_mms = ep
        .speed_knots
        .map(|k| (k * KNOTS_TO_MS * 1000.0).round() as u32)
        .unwrap_or(0);

    // Build (gnssId, cno) pairs in u-blox gnssId order (mirrors NAV-SAT layout),
    // capped at 16 like the firmware's extract_cno_from_nav_sat. all_cno keeps
    // every value (uncapped) for the pooled-σ diagnostic only.
    // talker -> u-blox gnssId: GP=0, GA=2, GB=3, GQ=5, GL=6, GI=7.
    let con_order: [(u8, &str); 6] = [
        (0, "GP"), (2, "GA"), (3, "GB"), (5, "GQ"), (6, "GL"), (7, "GI"),
    ];
    let mut cno_values: heapless::Vec<(u8, u8), 16> = heapless::Vec::new();
    let mut all_cno: Vec<u8> = Vec::new();
    for (gid, k) in con_order {
        if let Some(v) = ep.cno.get(k) {
            for &c in v {
                all_cno.push(c);
                let _ = cno_values.push((gid, c));
            }
        }
    }

    // NOTIME=1 drops gnss_time to test whether the clock-drift-recovery path is
    // what keeps clearing the spoof latch.
    let gnss_time = if std::env::var("NOTIME").is_ok() {
        None
    } else {
        Some(GnssTime {
            itow_ms: 0,
            year,
            month,
            day,
            hour: h,
            min: m,
            sec: s,
            system_time_ms: time_ms,
        })
    };

    let pos = Position {
        lat,
        lon,
        alt_mm: ep.alt_mm.unwrap_or(0),
        time_ms,
        fix_type,
        h_acc_mm: 1000,
        num_sv: ep.nsats,
        pdop: 0,
        gnss_time,
        cno_values,
        reported_speed_mms,
    };

    let res = det.analyze(pos);
    *idx += 1;

    // per-constellation CNO std (the doc's D1 detector) vs pooled std (ours)
    let pooled_std = std_of(&all_cno);
    let pooled_mean = mean_of(&all_cno);

    match res {
        AnalysisResult::Spoofed => stats.spoofed += 1,
        AnalysisResult::Normal => stats.normal += 1,
        AnalysisResult::Initializing => stats.init += 1,
        AnalysisResult::GapReset => stats.gap += 1,
    }
    if res == AnalysisResult::Spoofed && stats.first_spoof_epoch.is_none() {
        stats.first_spoof_epoch = Some(*idx);
        stats.first_spoof_hms = Some((h, m, s));
    }

    // ---- replicate main.rs SPOOF_DETECTED + 5s recovery timer ----
    let is_spoofed = res == AnalysisResult::Spoofed;
    let now_ms = time_ms;
    // Mode 3 (recovery)
    if is_spoofed {
        stats.m3_detected = true;
        stats.m3_recovery_start = 0;
    } else if stats.m3_detected {
        // !is_spoofed && was_spoofed
        if stats.m3_recovery_start == 0 {
            stats.m3_recovery_start = now_ms.max(1);
        } else if now_ms.wrapping_sub(stats.m3_recovery_start) >= 5000 {
            stats.m3_detected = false;
            stats.m3_recovery_start = 0;
        }
    }
    if stats.m3_detected {
        stats.m3_blocked += 1;
    }
    // Mode 4 (latched): never auto-clears
    if is_spoofed {
        stats.m4_detected = true;
    }
    if stats.m4_detected {
        stats.m4_blocked += 1;
    }

    // verbose line on state transitions / spoof
    let label = match res {
        AnalysisResult::Spoofed => "SPOOFED",
        AnalysisResult::Normal => "normal",
        AnalysisResult::Initializing => "init",
        AnalysisResult::GapReset => "gap",
    };
    if res != stats.last || res == AnalysisResult::Spoofed && stats.spoofed <= 3 {
        println!(
            "[{log}] ep{:>4} {:02}:{:02}:{:02} fix={} nsv={:>2} spd={:>5.1}kt cno(n={:>2} mean={:>4.1} pooledσ={:>4.1}) -> {}",
            *idx, h, m, s, fix_type as u8, ep.nsats,
            ep.speed_knots.unwrap_or(0.0), all_cno.len(), pooled_mean, pooled_std, label
        );
    }
    stats.last = res;

    // collect per-constellation std for D1 reporting (max-N constellation)
    for (k, v) in &ep.cno {
        if v.len() >= 6 {
            let e = stats.per_con.entry(k.clone()).or_default();
            e.0 += std_of(v) as f64;
            e.1 += 1;
        }
    }

    // ---- raw-signal tallies (would-fire, independent of the state machine) ----
    let has_fix = fix_type.has_3d_fix();
    if has_fix {
        stats.n_fix += 1;
        // pos-derived speed vs prev valid fix
        if let Some((plat, plon, pt)) = stats.prev_fix {
            let dt = time_ms.wrapping_sub(pt) as f32 / 1000.0;
            if dt > 0.0 && dt <= 5.0 {
                let avg_lat = ((plat + lat) / 2) as f32 * 1e-7 * std::f32::consts::PI / 180.0;
                let dlat = (lat - plat) as f32 * 1e-7 * 111320.0;
                let dlon = (lon - plon) as f32 * 1e-7 * 111320.0 * avg_lat.cos();
                let spd = (dlat * dlat + dlon * dlon).sqrt() / dt;
                if spd > 30.0 {
                    stats.n_speed_fire += 1;
                }
            }
        }
        stats.prev_fix = Some((lat, lon, time_ms));
        // reported velocity ceiling (REPORTED_SPEED_MAX_MS = 35 m/s)
        if reported_speed_mms as f32 / 1000.0 > 35.0 {
            stats.n_velceil_fire += 1;
        }
        // CNO: pooled (ours) vs per-constellation (doc D1)
        if all_cno.len() >= 6 && pooled_std < 3.0 && pooled_mean > 40.0 {
            stats.n_cno_pooled += 1;
        }
        // mirror the detector's new per-constellation flatness test (floor 20)
        let percon = ep.cno.values().any(|v| {
            v.len() >= 6 && std_of(v) < 3.0 && mean_of(v) >= 20.0
        });
        if percon {
            stats.n_cno_percon += 1;
        }
    }
}

fn mean_of(v: &[u8]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.iter().map(|&x| x as f64).sum::<f64>() / v.len() as f64
}
fn std_of(v: &[u8]) -> f64 {
    if v.len() < 2 {
        return 0.0;
    }
    let m = mean_of(v);
    (v.iter().map(|&x| (x as f64 - m).powi(2)).sum::<f64>() / v.len() as f64).sqrt()
}

struct Stats {
    spoofed: u32,
    normal: u32,
    init: u32,
    gap: u32,
    first_spoof_epoch: Option<u32>,
    first_spoof_hms: Option<(u8, u8, u8)>,
    last: AnalysisResult,
    per_con: BTreeMap<String, (f64, u32)>, // sum of std, count
    // raw signal tallies (how often each primitive detector WOULD fire),
    // independent of the latch/warmup/counter state machine:
    n_fix: u32,           // epochs with a valid 3D fix
    n_speed_fire: u32,    // pos-derived speed > MAX_SPEED_MS
    n_velceil_fire: u32,  // reported speed > REPORTED_SPEED_MAX_MS
    n_cno_pooled: u32,    // pooled σ<3 & mean>40 & n>=6  (OUR implementation)
    n_cno_percon: u32,    // ANY constellation N>=6 with σ<3 & mean>40 (doc D1)
    prev_fix: Option<(i32, i32, u32)>, // lat, lon, time_ms of last valid fix
    // Simulate main.rs SPOOF_DETECTED + 5s recovery timer over the verdict stream.
    // Mode 3 (recovery): latch clears after 5s of continuous Normal.
    // Mode 4 (latched):  latch never auto-clears.
    m3_detected: bool,
    m3_recovery_start: u32, // 0 = not running
    m3_blocked: u32,        // epochs where SPOOF_DETECTED true (fake coords blocked)
    m4_detected: bool,
    m4_blocked: u32,
}

impl Default for Stats {
    fn default() -> Self {
        Stats {
            spoofed: 0,
            normal: 0,
            init: 0,
            gap: 0,
            first_spoof_epoch: None,
            first_spoof_hms: None,
            last: AnalysisResult::Initializing,
            per_con: BTreeMap::new(),
            n_fix: 0,
            n_speed_fire: 0,
            n_velceil_fire: 0,
            n_cno_pooled: 0,
            n_cno_percon: 0,
            prev_fix: None,
            m3_detected: false,
            m3_recovery_start: 0,
            m3_blocked: 0,
            m4_detected: false,
            m4_blocked: 0,
        }
    }
}

fn process(path: &str) {
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cannot read {path}: {e}");
            return;
        }
    };
    let log = std::path::Path::new(path)
        .file_name()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| path.to_string());

    let mut det = SpoofDetector::new();
    let mut idx = 0u32;
    let mut stats = Stats::default();
    let mut ep = Epoch::default();
    let mut cur_time: Option<String> = None;
    let mut prev_line = String::new();

    for raw in content.lines() {
        let line = raw.trim();
        if !line.starts_with('$') {
            continue; // skip generator #SATELLITEA etc
        }
        if line == prev_line {
            continue; // dedup exact duplicate sentence
        }
        prev_line = line.to_string();
        if !checksum_ok(line) {
            continue;
        }
        let body = &line[1..line.rfind('*').unwrap_or(line.len())];
        let f: Vec<&str> = body.split(',').collect();
        if f.is_empty() {
            continue;
        }
        let typ = f[0];
        if typ.len() < 5 {
            continue;
        }
        let kind = &typ[2..]; // RMC, GGA, GSA, GSV, ...

        match kind {
            "RMC" => {
                // $..RMC,time,status,lat,N,lon,E,speed,course,date,...
                let t = f.get(1).copied().unwrap_or("");
                if cur_time.as_deref() != Some(t) {
                    flush(&ep, &mut idx, &mut det, &log, &mut stats);
                    ep = Epoch::default();
                    cur_time = Some(t.to_string());
                }
                ep.hms = parse_hms(t);
                if let (Some(lat), Some(lon)) = (
                    parse_latlon(f.get(3).copied().unwrap_or(""), f.get(4).copied().unwrap_or("")),
                    parse_latlon(f.get(5).copied().unwrap_or(""), f.get(6).copied().unwrap_or("")),
                ) {
                    ep.lat_1e7 = Some(lat);
                    ep.lon_1e7 = Some(lon);
                }
                ep.speed_knots = f.get(7).and_then(|s| s.parse().ok());
                ep.course = f.get(8).and_then(|s| s.parse().ok());
                if let Some(d) = f.get(9) {
                    if d.len() == 6 {
                        let day = d[0..2].parse().unwrap_or(13);
                        let mon = d[2..4].parse().unwrap_or(6);
                        let yr: u16 = d[4..6].parse().unwrap_or(26);
                        ep.date = Some((day, mon, 2000 + yr));
                    }
                }
            }
            "GGA" => {
                // $..GGA,time,lat,N,lon,E,quality,nsats,hdop,alt,M,...
                let t = f.get(1).copied().unwrap_or("");
                if cur_time.is_none() {
                    cur_time = Some(t.to_string());
                }
                if ep.hms.is_none() {
                    ep.hms = parse_hms(t);
                }
                if ep.lat_1e7.is_none() {
                    if let (Some(lat), Some(lon)) = (
                        parse_latlon(f.get(2).copied().unwrap_or(""), f.get(3).copied().unwrap_or("")),
                        parse_latlon(f.get(4).copied().unwrap_or(""), f.get(5).copied().unwrap_or("")),
                    ) {
                        ep.lat_1e7 = Some(lat);
                        ep.lon_1e7 = Some(lon);
                    }
                }
                ep.fix_quality = f.get(6).and_then(|s| s.parse().ok()).unwrap_or(0);
                ep.nsats = f.get(7).and_then(|s| s.parse().ok()).unwrap_or(0);
                if let Some(a) = f.get(9).and_then(|s| s.parse::<f64>().ok()) {
                    ep.alt_mm = Some((a * 1000.0).round() as i32);
                }
            }
            "GSA" => {
                // $..GSA,mode,fix(1/2/3),sat...,pdop,hdop,vdop,systemid
                let mode = f.get(2).and_then(|s| s.parse::<u8>().ok()).unwrap_or(1);
                if mode > ep.fix_mode {
                    ep.fix_mode = mode;
                }
            }
            "GSV" => {
                // $xxGSV,total,seq,totalSats,(prn,elev,az,cno)x4,...
                let tk = talker(typ);
                let entry = ep.cno.entry(tk).or_default();
                let mut i = 4;
                while i + 3 < f.len() + 1 && i + 3 <= f.len() {
                    // cno is f[i+3] within each group; group starts at i (prn)
                    if let Some(c) = f.get(i + 3).and_then(|s| s.parse::<u8>().ok()) {
                        if c > 0 {
                            entry.push(c);
                        }
                    }
                    i += 4;
                }
            }
            _ => {}
        }
    }
    flush(&ep, &mut idx, &mut det, &log, &mut stats);

    println!("\n===== SUMMARY [{log}] =====");
    println!(
        "epochs={} | init={} gap={} normal={} SPOOFED={}",
        idx, stats.init, stats.gap, stats.normal, stats.spoofed
    );
    match (stats.first_spoof_epoch, stats.first_spoof_hms) {
        (Some(e), Some((h, m, s))) => println!(
            "first SPOOFED verdict: epoch {} at {:02}:{:02}:{:02} UTC",
            e, h, m, s
        ),
        _ => println!("NEVER flagged as spoofed"),
    }
    println!("per-constellation mean C/N0 σ (doc's D1 detector, N>=6 only):");
    for (k, (sum, n)) in &stats.per_con {
        if *n > 0 {
            println!("   {k}: avg σ = {:.2} dB over {} epochs", sum / *n as f64, n);
        }
    }
    println!("(detector pools all constellations into one σ; D1 wants per-constellation σ)");
    let f = stats.n_fix.max(1) as f64;
    println!("--- raw 'would-fire' rates over {} fixed epochs ---", stats.n_fix);
    println!(
        "  pos-derived speed >30 m/s : {:>4} ({:.0}%)",
        stats.n_speed_fire, 100.0 * stats.n_speed_fire as f64 / f
    );
    println!(
        "  reported speed   >35 m/s : {:>4} ({:.0}%)   [vel-ceiling detector]",
        stats.n_velceil_fire, 100.0 * stats.n_velceil_fire as f64 / f
    );
    println!(
        "  CNO pooled  σ<3 & mean>40: {:>4} ({:.0}%)   [OUR cno detector]",
        stats.n_cno_pooled, 100.0 * stats.n_cno_pooled as f64 / f
    );
    println!(
        "  CNO per-con σ<3 & mean>40: {:>4} ({:.0}%)   [doc D1, NOT implemented]",
        stats.n_cno_percon, 100.0 * stats.n_cno_percon as f64 / f
    );
    let tot = idx.max(1) as f64;
    println!("--- effective protection (SPOOF_DETECTED held => fake coords blocked) ---");
    println!(
        "  Mode 3 (Passthrough+offset, recovery): blocked {} / {} epochs ({:.0}%) -- fake coords LEAK {:.0}%",
        stats.m3_blocked, idx, 100.0 * stats.m3_blocked as f64 / tot,
        100.0 * (idx - stats.m3_blocked) as f64 / tot
    );
    println!(
        "  Mode 4 (NoRecovery, latched):          blocked {} / {} epochs ({:.0}%)",
        stats.m4_blocked, idx, 100.0 * stats.m4_blocked as f64 / tot
    );
    println!();
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!("usage: replay_real_spoof <nmea.log> [more.log ...]");
        std::process::exit(1);
    }
    for a in &args {
        process(a);
    }
}
