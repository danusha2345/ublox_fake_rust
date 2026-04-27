//! Host-side tests for `src/unicore/nmea.rs`.
//!
//! The reference sentences come from the real UC6580I log
//! `uart_term/target/release/log_2026-04-22_17-13-32.txt`.

use spoof_detector_tests::{config, unicore_nmea::*};

// ---------------------------------------------------------------------------
// Checksum — real UC6580I sentences observed on the wire
// ---------------------------------------------------------------------------

#[test]
fn real_gnrmc_cs_matches() {
    assert!(verify_sentence(b"$GNRMC,,V,,,,,,,,,,N,V*37"));
}

#[test]
fn real_gngga_cs_matches() {
    assert!(verify_sentence(b"$GNGGA,,,,,,0,00,99.99,,,,,,*56"));
}

#[test]
fn real_gngsa_all_systems() {
    assert!(verify_sentence(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1*33"));
    assert!(verify_sentence(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,2*30"));
    assert!(verify_sentence(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,3*31"));
    assert!(verify_sentence(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,4*36"));
    assert!(verify_sentence(b"$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,5*37"));
}

#[test]
fn real_gsv_empty_all_talkers() {
    assert!(verify_sentence(b"$GPGSV,1,1,00,1*64"));
    assert!(verify_sentence(b"$GPGSV,1,1,00,8*6D"));
    assert!(verify_sentence(b"$GBGSV,1,1,00,1*76"));
    assert!(verify_sentence(b"$GBGSV,1,1,00,5*72"));
    assert!(verify_sentence(b"$GAGSV,1,1,00,7*73"));
    assert!(verify_sentence(b"$GAGSV,1,1,00,1*75"));
    assert!(verify_sentence(b"$GLGSV,1,1,00,1*78"));
    assert!(verify_sentence(b"$GQGSV,1,1,00,1*65"));
    assert!(verify_sentence(b"$GQGSV,1,1,00,8*6C"));
}

#[test]
fn real_pnoise_cs() {
    assert!(verify_sentence(
        b"$PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0*37"
    ));
    assert!(verify_sentence(
        b"$PNOISE,67,84,13555,11463,9616,35638,10000,10000,10000,10000,0,0*36"
    ));
    assert!(verify_sentence(
        b"$PNOISE,61,85,11769,9784,9335,33636,10000,10000,10000,10000,0,0*01"
    ));
}

#[test]
fn real_ok_reply_cs() {
    // `$OK*04` — 'O' XOR 'K' = 0x04
    assert!(verify_sentence(b"$OK*04"));
}

#[test]
fn real_gntxt_ack_cs() {
    assert!(verify_sentence(b"$GNTXT,01,01,00,PDTINFO*1F"));
    assert!(verify_sentence(b"$GNTXT,01,01,00,CFGSAVE*12"));
    assert!(verify_sentence(b"$GNTXT,01,01,00,CFGSYS*4A"));
}

// ---------------------------------------------------------------------------
// Builder — sanity
// ---------------------------------------------------------------------------

#[test]
fn build_gga_golden_beach_position() {
    let mut buf = [0u8; 128];
    let n = build_gga(&mut buf, Talker::Gn, &GgaFields {
        time: NmeaTime::new(12, 34, 56),
        lat_1e7: 259_664_430,  // 25.966443°N — Golden Beach FL
        lon_1e7: -801_223_710, // 80.122371°W
        fix_quality: 1,
        nsats: 10,
        hdop_x100: 123,
        alt_mm: 100_000,
        geoid_sep_mm: -30_000,
    });
    assert!(n > 0);
    assert!(verify_sentence(&buf[..n]));
    // Must start with `$GNGGA,`
    assert_eq!(&buf[..7], b"$GNGGA,");
    // Round-trip parse preserves integer fields
    let p = parse_gga(&buf[..n]).expect("parse");
    assert_eq!(p.lat_1e7, 259_664_430);
    assert_eq!(p.lon_1e7, -801_223_710);
    assert_eq!(p.alt_mm, 100_000);
    assert_eq!(p.nsats, 10);
    assert_eq!(p.fix_quality, 1);
}

#[test]
fn build_rmc_valid_fix() {
    let mut buf = [0u8; 128];
    let n = build_rmc(&mut buf, Talker::Gn, &RmcFields {
        time: NmeaTime { hour: 11, minute: 22, second: 33, centis: 45 },
        valid: true,
        lat_1e7: 259_664_430,
        lon_1e7: -801_223_710,
        sog_knots_x1000: 1_234,  // 1.234 knots
        cog_deg_x100: 9_000,     // 90.00°
        date: NmeaDate { day: 22, month: 4, year: 2026 },
        mode: b'A',
    });
    assert!(n > 0);
    assert!(verify_sentence(&buf[..n]));
    assert_eq!(&buf[..7], b"$GNRMC,");
    let p = parse_rmc(&buf[..n]).expect("parse");
    assert!(p.checksum_ok);
    assert_eq!(p.lat_1e7, 259_664_430);
    assert_eq!(p.lon_1e7, -801_223_710);
    assert_eq!(p.fix_quality, 1);
    assert_eq!(p.time, NmeaTime { hour: 11, minute: 22, second: 33, centis: 45 });
}

#[test]
fn build_gsa_preserves_system_id() {
    let mut buf = [0u8; 128];
    let n = build_gsa(&mut buf, Talker::Gn, &GsaFields {
        op_mode: GsaOpMode::Automatic,
        fix_type: 3,
        sats: [5, 13, 0, 15, 0, 0, 20, 0, 0, 0, 0, 0],
        pdop_x100: 150,
        hdop_x100: 99,
        vdop_x100: 122,
        system_id: 1,
    });
    assert!(n > 0);
    assert!(verify_sentence(&buf[..n]));
    let s = core::str::from_utf8(&buf[..n]).expect("ascii");
    // PRN present, op_mode = A, fix_type = 3, trailing `,1*cs\r\n`
    assert!(s.starts_with("$GNGSA,A,3,"));
    assert!(s.contains(",15,"));
    assert!(s.contains(",1.50,"));      // PDOP
    assert!(s.contains(",1*"));          // system_id = 1
}

#[test]
fn build_gsv_single_sentence() {
    let mut buf = [0u8; 128];
    let sats = [GsvSat { prn: 5, elevation_deg: 30, azimuth_deg: 180, cno_dbhz: 45 }];
    let n = build_gsv(&mut buf, Talker::Gp, 1, 1, 1, &sats, 1);
    assert!(n > 0);
    assert!(verify_sentence(&buf[..n]));
    assert!(core::str::from_utf8(&buf[..n]).unwrap().starts_with("$GPGSV,1,1,01,5,30,180,45,1*"));
}

#[test]
fn build_pnoise_matches_real_format() {
    let mut buf = [0u8; 128];
    let n = build_pnoise(&mut buf, 65, 85,
        &[13663, 11506, 9643, 34974],
        &[10000, 10000, 10000, 10000],
        0, 0);
    assert!(n > 0);
    // The sentence should be the exact one observed in the real log.
    assert_eq!(
        &buf[..n],
        b"$PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0*37\r\n"
    );
}

#[test]
fn build_gntxt_matches_real_ack() {
    let mut buf = [0u8; 128];
    let n = build_gntxt(&mut buf, 1, 1, 0, b"PDTINFO");
    assert!(n > 0);
    assert_eq!(&buf[..n], b"$GNTXT,01,01,00,PDTINFO*1F\r\n");
}

// ---------------------------------------------------------------------------
// Parser — robustness
// ---------------------------------------------------------------------------

#[test]
fn parse_gga_handles_empty_fields() {
    let p = parse_gga(b"$GNGGA,,,,,,0,00,99.99,,,,,,*56").expect("parse");
    assert!(p.checksum_ok);
    assert_eq!(p.fix_quality, 0);
    assert_eq!(p.nsats, 0);
    assert_eq!(p.lat_1e7, 0);
    assert_eq!(p.lon_1e7, 0);
}

#[test]
fn parse_gga_rejects_non_gga_tag() {
    assert!(parse_gga(b"$GNRMC,,V,,,,,,,,,,N,V*37").is_none());
    assert!(parse_rmc(b"$GNRMC,,V,,,,,,,,,,N,V*37").is_some());
    // Any talker is accepted as long as the 3-letter kind matches.
    assert!(parse_gga(b"$GPGGA,,,,,,0,00,99.99,,,,,,*57").is_some());
}

#[test]
fn parse_rmc_picks_up_status_flag() {
    let p = parse_rmc(b"$GNRMC,,V,,,,,,,,,,N,V*37").expect("parse");
    assert_eq!(p.fix_quality, 0);      // V → invalid
}

#[test]
fn verify_rejects_bad_checksum() {
    // The real checksum is *37; anything else should fail.
    assert!(!verify_sentence(b"$GNRMC,,V,,,,,,,,,,N,V*00"));
    assert!(!verify_sentence(b"$GNRMC,,V,,,,,,,,,,N,V*FF"));
    assert!(!verify_sentence(b"$PNOISE,1,2*99"));
}

// ---------------------------------------------------------------------------
// Rewrite
// ---------------------------------------------------------------------------

#[test]
fn rewrite_gga_position_roundtrip() {
    let mut buf = [0u8; 256];
    // Build a GGA at Golden Beach first.
    let n = build_gga(&mut buf, Talker::Gn, &GgaFields {
        time: NmeaTime::new(10, 20, 30),
        lat_1e7: 259_664_430,
        lon_1e7: -801_223_710,
        fix_quality: 1,
        nsats: 12,
        hdop_x100: 77,
        alt_mm: 100_000,
        geoid_sep_mm: -10_000,
    });
    // Rewrite to Seney, MI.
    let new = rewrite_position_inplace(&mut buf, n, 463_407_000, -859_407_000, Some(100_000))
        .expect("rewrite");
    assert!(verify_sentence(&buf[..new]));
    let p = parse_gga(&buf[..new]).expect("parse");
    assert_eq!(p.lat_1e7, 463_407_000);
    assert_eq!(p.lon_1e7, -859_407_000);
    assert_eq!(p.alt_mm, 100_000);
    assert_eq!(p.nsats, 12);
    assert_eq!(p.fix_quality, 1);
    assert_eq!(p.time, NmeaTime::new(10, 20, 30));
}

#[test]
fn rewrite_rmc_keeps_time_date_valid() {
    let mut buf = [0u8; 256];
    let n = build_rmc(&mut buf, Talker::Gn, &RmcFields {
        time: NmeaTime { hour: 5, minute: 15, second: 25, centis: 50 },
        valid: true,
        lat_1e7: 100_000_000,
        lon_1e7: 200_000_000,
        sog_knots_x1000: 5_000,
        cog_deg_x100: 18_000,
        date: NmeaDate { day: 22, month: 4, year: 2026 },
        mode: b'A',
    });
    let new = rewrite_position_inplace(&mut buf, n, -500_000_000, -500_000_000, None)
        .expect("rewrite");
    assert!(verify_sentence(&buf[..new]));
    let p = parse_rmc(&buf[..new]).expect("parse");
    assert_eq!(p.lat_1e7, -500_000_000);
    assert_eq!(p.lon_1e7, -500_000_000);
    assert_eq!(p.fix_quality, 1);
    assert_eq!(p.time, NmeaTime { hour: 5, minute: 15, second: 25, centis: 50 });
}

#[test]
fn rewrite_rejects_non_gga_non_rmc() {
    let mut buf = [0u8; 256];
    let n = build_gsa(&mut buf, Talker::Gn, &GsaFields {
        op_mode: GsaOpMode::Automatic, fix_type: 3,
        sats: [0; 12], pdop_x100: 100, hdop_x100: 100, vdop_x100: 100,
        system_id: 1,
    });
    assert!(rewrite_position_inplace(&mut buf, n, 0, 0, None).is_none());
}

#[test]
fn force_3d_gga_uses_target_fix_and_live_time() {
    let target = (
        config::offset_target::LAT_1E7,
        config::offset_target::LON_1E7,
        config::offset_target::ALT_MM,
    );
    let mut src = [0u8; 128];
    let n = build_gga(&mut src, Talker::Gn, &GgaFields {
        time: NmeaTime { hour: 3, minute: 4, second: 5, centis: 67 },
        lat_1e7: 259_664_430,
        lon_1e7: -801_223_710,
        fix_quality: 0,
        nsats: 0,
        hdop_x100: 9_999,
        alt_mm: 0,
        geoid_sep_mm: 0,
    });
    let mut out = [0u8; 160];
    let m = force_3d_fix_sentence(
        &src[..n], target,
        NmeaTime::default(), NmeaDate { day: 1, month: 1, year: 2025 },
        &mut out,
    ).expect("force gga");

    assert!(verify_sentence(&out[..m]));
    let p = parse_gga(&out[..m]).expect("parse forced gga");
    assert_eq!(p.time, NmeaTime { hour: 3, minute: 4, second: 5, centis: 67 });
    assert_eq!(p.lat_1e7, target.0);
    assert_eq!(p.lon_1e7, target.1);
    assert_eq!(p.alt_mm, target.2);
    assert_eq!(p.fix_quality, 3);
    assert_eq!(p.nsats, 16);
    let s = core::str::from_utf8(&out[..m]).unwrap();
    assert!(s.contains(",0.99,"));
    assert!(s.contains(",-30.000,M,,"));
}

#[test]
fn force_3d_rmc_preserves_live_time_date_and_sets_valid_mode() {
    let target = (
        config::offset_target::LAT_1E7,
        config::offset_target::LON_1E7,
        config::offset_target::ALT_MM,
    );
    let mut src = [0u8; 128];
    let n = build_rmc(&mut src, Talker::Gn, &RmcFields {
        time: NmeaTime { hour: 11, minute: 22, second: 33, centis: 44 },
        valid: false,
        lat_1e7: 0,
        lon_1e7: 0,
        sog_knots_x1000: 12_345,
        cog_deg_x100: 27_000,
        date: NmeaDate { day: 22, month: 4, year: 2026 },
        mode: b'N',
    });
    let mut out = [0u8; 160];
    let m = force_3d_fix_sentence(
        &src[..n], target,
        NmeaTime::default(), NmeaDate { day: 1, month: 1, year: 2025 },
        &mut out,
    ).expect("force rmc");

    assert!(verify_sentence(&out[..m]));
    let p = parse_rmc(&out[..m]).expect("parse forced rmc");
    assert_eq!(p.time, NmeaTime { hour: 11, minute: 22, second: 33, centis: 44 });
    assert_eq!(p.date, NmeaDate { day: 22, month: 4, year: 2026 });
    assert_eq!(p.lat_1e7, target.0);
    assert_eq!(p.lon_1e7, target.1);
    assert_eq!(p.fix_quality, 1);

    let s = core::str::from_utf8(&out[..m]).unwrap();
    let body = s.trim_end().trim_start_matches('$').split('*').next().unwrap();
    let fields: Vec<_> = body.split(',').collect();
    assert_eq!(fields[2], "A");
    assert_eq!(fields[7], "0.000");
    assert_eq!(fields[8], "0.00");
    assert_eq!(fields[9], "220426");
    assert_eq!(fields[12], "A");
}

#[test]
fn force_3d_gsa_preserves_talker_system_and_prns() {
    let mut src = [0u8; 128];
    let n = build_gsa(&mut src, Talker::Gp, &GsaFields {
        op_mode: GsaOpMode::Manual,
        fix_type: 1,
        sats: [5, 13, 0, 15, 0, 0, 20, 0, 0, 0, 0, 0],
        pdop_x100: 9_999,
        hdop_x100: 9_999,
        vdop_x100: 9_999,
        system_id: 4,
    });
    let mut out = [0u8; 160];
    let target = (
        config::offset_target::LAT_1E7,
        config::offset_target::LON_1E7,
        config::offset_target::ALT_MM,
    );
    let m = force_3d_fix_sentence(
        &src[..n], target,
        NmeaTime::default(), NmeaDate { day: 1, month: 1, year: 2025 },
        &mut out,
    ).expect("force gsa");

    assert!(verify_sentence(&out[..m]));
    let s = core::str::from_utf8(&out[..m]).unwrap();
    let body = s.trim_end().trim_start_matches('$').split('*').next().unwrap();
    let fields: Vec<_> = body.split(',').collect();
    assert_eq!(fields[0], "GPGSA");
    assert_eq!(fields[1], "M");
    assert_eq!(fields[2], "3");
    assert_eq!(fields[3], "5");
    assert_eq!(fields[4], "13");
    assert_eq!(fields[6], "15");
    assert_eq!(fields[9], "20");
    assert_eq!(fields[15], "0.99");
    assert_eq!(fields[16], "0.99");
    assert_eq!(fields[17], "0.99");
    assert_eq!(fields[18], "4");
}

#[test]
fn force_3d_rejects_bad_checksum_only() {
    // After "passthrough + immediate coord substitution" semantics, force_3d
    // succeeds even when chip's own time/date fields are empty (cold-boot
    // case): the rewrite uses the caller-provided fallback. Only checksum
    // failure or unknown sentence kind keep returning `None`.
    let target = (
        config::offset_target::LAT_1E7,
        config::offset_target::LON_1E7,
        config::offset_target::ALT_MM,
    );
    let fb_time = NmeaTime::default();
    let fb_date = NmeaDate { day: 1, month: 1, year: 2025 };
    let mut out = [0u8; 160];

    // Bad checksum on RMC → still rejected.
    assert!(force_3d_fix_sentence(
        b"$GNRMC,112233.00,V,,,,,,,220426,,,N,V*00",
        target, fb_time, fb_date, &mut out,
    ).is_none());

    // Unknown sentence kind → rejected.
    assert!(force_3d_fix_sentence(
        b"$GNVTG,,T,,M,0.000,N,0.000,K,N*2C",
        target, fb_time, fb_date, &mut out,
    ).is_none());

    // Empty-time GGA → succeeds, falls back to fb_time.
    let n = force_3d_fix_sentence(
        b"$GNGGA,,,,,,0,00,99.99,,,,,,*56",
        target, fb_time, fb_date, &mut out,
    ).expect("empty-time GGA must rewrite");
    let p = parse_gga(&out[..n]).expect("parse rewritten gga");
    assert!(p.checksum_ok);
    assert_eq!(p.fix_quality, 3);
    assert_eq!(p.lat_1e7, target.0);
    assert_eq!(p.lon_1e7, target.1);
    assert_eq!(p.time, fb_time);

    // Empty-time/date RMC → succeeds, falls back.
    let n = force_3d_fix_sentence(
        b"$GNRMC,,V,,,,,,,,,,N,V*37",
        target, fb_time, fb_date, &mut out,
    ).expect("empty RMC must rewrite");
    let p = parse_rmc(&out[..n]).expect("parse rewritten rmc");
    assert!(p.checksum_ok);
    assert_eq!(p.lat_1e7, target.0);
    assert_eq!(p.lon_1e7, target.1);
    assert_eq!(p.time, fb_time);
    assert_eq!(p.date, fb_date);
    let s = core::str::from_utf8(&out[..n]).unwrap();
    let body = s.trim_end().trim_start_matches('$').split('*').next().unwrap();
    let fields: Vec<_> = body.split(',').collect();
    assert_eq!(fields[2], "A"); // status flipped to valid
    assert_eq!(fields[12], "A"); // mode set to autonomous
}

#[test]
fn force_no_fix_zeroes_satellites_keeps_target_coords() {
    let target = (
        config::offset_target::LAT_1E7,
        config::offset_target::LON_1E7,
        config::offset_target::ALT_MM,
    );
    let fb_time = NmeaTime { hour: 12, minute: 34, second: 56, centis: 0 };
    let fb_date = NmeaDate { day: 1, month: 1, year: 2025 };
    let mut out = [0u8; 160];

    // GGA: fq=0, nsats=1, target coords still substituted.
    let mut src = [0u8; 128];
    let n = build_gga(&mut src, Talker::Gn, &GgaFields {
        time: fb_time,
        lat_1e7: 259_664_430,
        lon_1e7: -801_223_710,
        fix_quality: 1,
        nsats: 8,
        hdop_x100: 100,
        alt_mm: 50_000,
        geoid_sep_mm: -10_000,
    });
    let m = force_no_fix_sentence(&src[..n], target, fb_time, fb_date, &mut out)
        .expect("force_no_fix gga");
    let p = parse_gga(&out[..m]).expect("parse no-fix gga");
    assert!(p.checksum_ok);
    assert_eq!(p.fix_quality, 0);
    assert_eq!(p.nsats, 1);
    assert_eq!(p.lat_1e7, target.0);
    assert_eq!(p.lon_1e7, target.1);

    // RMC: status=V, mode=N.
    let n = build_rmc(&mut src, Talker::Gn, &RmcFields {
        time: fb_time,
        valid: true,
        lat_1e7: 259_664_430,
        lon_1e7: -801_223_710,
        sog_knots_x1000: 0,
        cog_deg_x100: 0,
        date: fb_date,
        mode: b'A',
    });
    let m = force_no_fix_sentence(&src[..n], target, fb_time, fb_date, &mut out)
        .expect("force_no_fix rmc");
    let s = core::str::from_utf8(&out[..m]).unwrap();
    let body = s.trim_end().trim_start_matches('$').split('*').next().unwrap();
    let fields: Vec<_> = body.split(',').collect();
    assert_eq!(fields[2], "V");
    assert_eq!(fields[12], "N");

    // GLL under no-fix: status=V, mode=N, target coords.
    let m = force_no_fix_sentence(b"$GNGLL,,,,,,V,N*7A", target, fb_time, fb_date, &mut out)
        .expect("force_no_fix gll");
    let s = core::str::from_utf8(&out[..m]).unwrap();
    let body = s.trim_end().trim_start_matches('$').split('*').next().unwrap();
    let fields: Vec<_> = body.split(',').collect();
    assert_eq!(fields[6], "V");
    assert_eq!(fields[7], "N");
    assert!(!fields[1].is_empty());
}

#[test]
fn force_3d_gll_rewrites_target_and_status() {
    let target = (
        config::offset_target::LAT_1E7,
        config::offset_target::LON_1E7,
        config::offset_target::ALT_MM,
    );
    let fb_time = NmeaTime { hour: 7, minute: 8, second: 9, centis: 0 };
    let fb_date = NmeaDate { day: 1, month: 1, year: 2025 };
    let mut out = [0u8; 160];

    // Real-style GLL with V status and empty fields.
    let n = force_3d_fix_sentence(
        b"$GNGLL,,,,,,V,N*7A",
        target, fb_time, fb_date, &mut out,
    ).expect("force gll on empty");
    assert!(verify_sentence(&out[..n]));
    let s = core::str::from_utf8(&out[..n]).unwrap();
    let body = s.trim_end().trim_start_matches('$').split('*').next().unwrap();
    let fields: Vec<_> = body.split(',').collect();
    assert_eq!(fields[0], "GNGLL");
    // lat,N/S — non-empty, contains hemisphere letter
    assert!(!fields[1].is_empty());
    assert!(fields[2] == "N" || fields[2] == "S");
    // lon,E/W
    assert!(!fields[3].is_empty());
    assert!(fields[4] == "E" || fields[4] == "W");
    // status=A, mode=A
    assert_eq!(fields[6], "A");
    assert_eq!(fields[7], "A");
}

#[test]
fn offset_rewrite_keeps_gga_fix_status_and_sat_count() {
    let mut buf = [0u8; 256];
    let n = build_gga(&mut buf, Talker::Gn, &GgaFields {
        time: NmeaTime::new(10, 20, 30),
        lat_1e7: 259_664_430,
        lon_1e7: -801_223_710,
        fix_quality: 3,
        nsats: 16,
        hdop_x100: 42,
        alt_mm: 50_000,
        geoid_sep_mm: -10_000,
    });
    let new = rewrite_position_inplace(
        &mut buf,
        n,
        config::offset_target::LAT_1E7,
        config::offset_target::LON_1E7,
        Some(config::offset_target::ALT_MM),
    )
    .expect("rewrite");

    assert!(verify_sentence(&buf[..new]));
    let p = parse_gga(&buf[..new]).expect("parse rewritten gga");
    assert_eq!(p.lat_1e7, config::offset_target::LAT_1E7);
    assert_eq!(p.lon_1e7, config::offset_target::LON_1E7);
    assert_eq!(p.alt_mm, config::offset_target::ALT_MM);
    assert_eq!(p.fix_quality, 3);
    assert_eq!(p.nsats, 16);
}
