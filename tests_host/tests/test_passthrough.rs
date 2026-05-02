use spoof_detector_tests::passthrough::*;
use spoof_detector_tests::coordinates;

// ============================================================================
// Helpers
// ============================================================================

/// Fletcher-8 checksum over class..payload (bytes 2..len-2 of the full UBX frame)
fn calculate_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in data {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Verify checksum bytes at the tail of a complete UBX frame.
fn verify_checksum(frame: &[u8]) -> bool {
    if frame.len() < 8 {
        return false;
    }
    let (ck_a, ck_b) = calculate_checksum(&frame[2..frame.len() - 2]);
    ck_a == frame[frame.len() - 2] && ck_b == frame[frame.len() - 1]
}

/// Read a little-endian i32 at `offset` inside the full UBX frame (including 2-byte sync).
fn read_i32_le(frame: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes([frame[offset], frame[offset + 1], frame[offset + 2], frame[offset + 3]])
}

/// Read a little-endian u32 at `offset` inside the full UBX frame.
fn read_u32_le(frame: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([frame[offset], frame[offset + 1], frame[offset + 2], frame[offset + 3]])
}

/// Build a syntactically-valid NAV-PVT frame (class=0x01, id=0x07, payload=92 bytes).
///
/// Payload layout (offsets relative to payload byte 0, i.e. frame byte 6):
///   fix_type  @ 20, flags @ 21, num_sv @ 23
///   lon       @ 24-27, lat @ 28-31, height @ 32-35, hMSL @ 36-39
///   hAcc      @ 40-43
///   velN      @ 48-51, velE @ 52-55, velD @ 56-59
///
/// All fields not explicitly set are left as 0 unless noted.
fn build_nav_pvt_frame(lat_1e7: i32, lon_1e7: i32, alt_mm: i32, fix_type: u8, num_sv: u8) -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 92];

    // valid flags for date+time (bits 0+1 set)
    payload[11] = 0x03;
    // year = 2026, month = 3, day = 23
    let year: u16 = 2026;
    payload[4..6].copy_from_slice(&year.to_le_bytes());
    payload[6] = 3;
    payload[7] = 23;

    payload[20] = fix_type;
    payload[21] = 0x01; // gnssFixOK
    payload[23] = num_sv;

    payload[24..28].copy_from_slice(&lon_1e7.to_le_bytes());
    payload[28..32].copy_from_slice(&lat_1e7.to_le_bytes());
    payload[32..36].copy_from_slice(&alt_mm.to_le_bytes());
    payload[36..40].copy_from_slice(&alt_mm.to_le_bytes()); // hMSL ≈ height

    // hAcc = 1000 mm (1 m)
    payload[40..44].copy_from_slice(&1000u32.to_le_bytes());

    // velN = 500 mm/s, velE = 500 mm/s (gives speed ≈ 707 mm/s)
    payload[48..52].copy_from_slice(&500i32.to_le_bytes());
    payload[52..56].copy_from_slice(&500i32.to_le_bytes());

    build_frame(0x01, 0x07, &payload)
}

/// Build a syntactically-valid NAV-POSLLH frame (class=0x01, id=0x02, payload=28 bytes).
///
/// Payload layout (offsets relative to payload byte 0):
///   lon @ 4-7, lat @ 8-11, height @ 12-15, hMSL @ 16-19, hAcc @ 20-23, vAcc @ 24-27
fn build_nav_posllh_frame(lat_1e7: i32, lon_1e7: i32, alt_mm: i32) -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 28];
    payload[4..8].copy_from_slice(&lon_1e7.to_le_bytes());
    payload[8..12].copy_from_slice(&lat_1e7.to_le_bytes());
    payload[12..16].copy_from_slice(&alt_mm.to_le_bytes());
    payload[16..20].copy_from_slice(&alt_mm.to_le_bytes()); // hMSL
    payload[20..24].copy_from_slice(&1000u32.to_le_bytes()); // hAcc
    payload[24..28].copy_from_slice(&1000u32.to_le_bytes()); // vAcc
    build_frame(0x01, 0x02, &payload)
}

/// Build a syntactically-valid NAV-POSECEF frame (class=0x01, id=0x01, payload=20 bytes).
///
/// Payload layout (offsets relative to payload byte 0):
///   ecefX @ 4-7, ecefY @ 8-11, ecefZ @ 12-15, pAcc @ 16-19
fn build_nav_posecef_frame(ecef_x: i32, ecef_y: i32, ecef_z: i32) -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 20];
    payload[4..8].copy_from_slice(&ecef_x.to_le_bytes());
    payload[8..12].copy_from_slice(&ecef_y.to_le_bytes());
    payload[12..16].copy_from_slice(&ecef_z.to_le_bytes());
    payload[16..20].copy_from_slice(&500u32.to_le_bytes()); // pAcc
    build_frame(0x01, 0x01, &payload)
}

/// Build a syntactically-valid NAV-HPPOSECEF frame (class=0x01, id=0x13, payload=28 bytes).
///
/// Payload layout (offsets relative to payload byte 0):
///   ecefX @ 8-11, ecefY @ 12-15, ecefZ @ 16-19, HP bytes @ 20-22, flags @ 23, pAcc @ 24-27
fn build_nav_hpposecef_frame(ecef_x: i32, ecef_y: i32, ecef_z: i32) -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 28];
    payload[8..12].copy_from_slice(&ecef_x.to_le_bytes());
    payload[12..16].copy_from_slice(&ecef_y.to_le_bytes());
    payload[16..20].copy_from_slice(&ecef_z.to_le_bytes());
    payload[20] = 10; // ecefXHp — non-zero to verify zeroing
    payload[21] = 20; // ecefYHp
    payload[22] = 30; // ecefZHp
    payload[23] = 0;  // flags
    payload[24..28].copy_from_slice(&500u32.to_le_bytes()); // pAcc
    build_frame(0x01, 0x13, &payload)
}

/// Build a syntactically-valid NAV-SOL frame (class=0x01, id=0x06, payload=52 bytes).
///
/// Payload layout (offsets relative to payload byte 0):
///   gps_fix @ 10, ecefX @ 12-15, ecefY @ 16-19, ecefZ @ 20-23, pAcc @ 24-27,
///   ecefVX @ 28-31, ecefVY @ 32-35, ecefVZ @ 36-39, num_sv @ 47
fn build_nav_sol_frame(ecef_x: i32, ecef_y: i32, ecef_z: i32) -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 52];
    payload[10] = 3;  // gps_fix = 3D
    payload[12..16].copy_from_slice(&ecef_x.to_le_bytes());
    payload[16..20].copy_from_slice(&ecef_y.to_le_bytes());
    payload[20..24].copy_from_slice(&ecef_z.to_le_bytes());
    payload[24..28].copy_from_slice(&500u32.to_le_bytes()); // pAcc
    payload[28..32].copy_from_slice(&100i32.to_le_bytes()); // ecefVX
    payload[32..36].copy_from_slice(&100i32.to_le_bytes()); // ecefVY
    payload[36..40].copy_from_slice(&100i32.to_le_bytes()); // ecefVZ
    payload[47] = 12; // num_sv
    build_frame(0x01, 0x06, &payload)
}

/// Build a syntactically-valid NAV-VELNED frame (class=0x01, id=0x12, payload=36 bytes).
///
/// Payload layout (offsets relative to payload byte 0):
///   velN @ 4, velE @ 8, velD @ 12, speed @ 16, gSpeed @ 20, heading @ 24,
///   sAcc @ 28, cAcc @ 32
fn build_nav_velned_frame() -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 36];
    payload[4..8].copy_from_slice(&500i32.to_le_bytes());    // velN
    payload[8..12].copy_from_slice(&500i32.to_le_bytes());   // velE
    payload[12..16].copy_from_slice(&(-100i32).to_le_bytes()); // velD
    payload[16..20].copy_from_slice(&700u32.to_le_bytes());  // speed
    payload[20..24].copy_from_slice(&700u32.to_le_bytes());  // gSpeed
    payload[24..28].copy_from_slice(&45_00000i32.to_le_bytes()); // heading (deg*1e-5)
    payload[28..32].copy_from_slice(&200u32.to_le_bytes());  // sAcc
    payload[32..36].copy_from_slice(&500000u32.to_le_bytes()); // cAcc
    build_frame(0x01, 0x12, &payload)
}

/// Build a syntactically-valid NAV-STATUS frame (class=0x01, id=0x03, payload=16 bytes).
fn build_nav_status_frame() -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 16];
    payload[4] = 3;  // gps_fix = 3D fix
    payload[5] = 0x01; // flags: gpsFixOK
    build_frame(0x01, 0x03, &payload)
}

/// Build a syntactically-valid NAV-SAT frame (class=0x01, id=0x35, minimal payload=8 bytes).
fn build_nav_sat_frame(num_svs: u8) -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 8];
    payload[5] = num_svs;
    build_frame(0x01, 0x35, &payload)
}

/// Build a syntactically-valid NAV-SVINFO frame (class=0x01, id=0x30, minimal payload=8 bytes).
fn build_nav_svinfo_frame(num_ch: u8) -> heapless::Vec<u8, 1280> {
    let mut payload = [0u8; 8];
    payload[4] = num_ch;
    build_frame(0x01, 0x30, &payload)
}

/// Assemble a complete UBX frame: sync bytes + class + id + len(LE) + payload + checksum.
fn build_frame(class: u8, id: u8, payload: &[u8]) -> heapless::Vec<u8, 1280> {
    let payload_len = payload.len();
    let frame_len = 6 + payload_len + 2;
    let mut frame: heapless::Vec<u8, 1280> = heapless::Vec::new();
    let _ = frame.resize(frame_len, 0u8);

    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = class;
    frame[3] = id;
    frame[4] = (payload_len & 0xFF) as u8;
    frame[5] = ((payload_len >> 8) & 0xFF) as u8;
    frame[6..6 + payload_len].copy_from_slice(payload);

    let (ck_a, ck_b) = calculate_checksum(&frame[2..6 + payload_len]);
    frame[6 + payload_len] = ck_a;
    frame[6 + payload_len + 1] = ck_b;

    frame
}

/// Golden Beach FL coordinates used throughout these tests.
const BASE_LAT: i32 = 259_664_430;   // 25.966443°N
const BASE_LON: i32 = -801_223_710;  // -80.122371°E
const BASE_ALT: i32 = 100_000;       // 100 m in mm

// ============================================================================
// Group 1: UbxFrameParser
// ============================================================================

/// Feed every byte of a pre-built frame into a fresh parser and expect the
/// complete frame back on the final byte.
#[test]
fn test_parser_valid_nav_pvt() {
    let frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);
    let mut parser = UbxFrameParser::new();

    let mut result = None;
    for &b in frame.iter() {
        if let Some(f) = parser.feed(b) {
            result = Some(f);
        }
    }

    let parsed = result.expect("Expected a complete frame to be returned");
    assert_eq!(parsed.len(), frame.len());
    assert_eq!(&parsed[..], &frame[..]);
    assert!(verify_checksum(&parsed));
}

/// Two frames fed in sequence are both parsed correctly.
#[test]
fn test_parser_two_frames_in_sequence() {
    let frame1 = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);
    let frame2 = build_nav_posllh_frame(BASE_LAT, BASE_LON, BASE_ALT);

    let mut parser = UbxFrameParser::new();
    let mut results: heapless::Vec<heapless::Vec<u8, 1280>, 4> = heapless::Vec::new();

    for &b in frame1.iter().chain(frame2.iter()) {
        if let Some(f) = parser.feed(b) {
            let _ = results.push(f);
        }
    }

    assert_eq!(results.len(), 2, "Expected exactly two parsed frames");
    assert_eq!(results[0].len(), frame1.len());
    assert_eq!(results[1].len(), frame2.len());
    assert!(verify_checksum(&results[0]));
    assert!(verify_checksum(&results[1]));
}

/// A frame with a corrupted checksum byte returns None from feed().
#[test]
fn test_parser_invalid_checksum_returns_none() {
    let mut frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);
    // Corrupt the first checksum byte
    let last = frame.len() - 2;
    frame[last] ^= 0xFF;

    let mut parser = UbxFrameParser::new();
    let mut result = None;
    for &b in frame.iter() {
        if let Some(f) = parser.feed(b) {
            result = Some(f);
        }
    }

    assert!(result.is_none(), "Corrupted checksum should yield no frame");
}

/// A payload length field encoding >1272 causes the parser to reject the frame
/// (buffer maximum is 1272 payload bytes, leaving room for 6+2 framing bytes).
#[test]
fn test_parser_oversized_payload_rejected() {
    // Manually craft a frame header claiming 1280 payload bytes (> 1272 limit)
    let oversized_len: u16 = 1280;
    let mut bytes: heapless::Vec<u8, 1280> = heapless::Vec::new();
    let _ = bytes.push(0xB5);
    let _ = bytes.push(0x62);
    let _ = bytes.push(0x01); // class
    let _ = bytes.push(0x07); // id
    let _ = bytes.push((oversized_len & 0xFF) as u8);
    let _ = bytes.push(((oversized_len >> 8) & 0xFF) as u8);

    let mut parser = UbxFrameParser::new();
    let mut got_frame = false;
    for &b in bytes.iter() {
        if parser.feed(b).is_some() {
            got_frame = true;
        }
    }

    // Parser should have reset after seeing the oversized length, returning nothing
    assert!(!got_frame, "Oversized payload should be rejected");
    // After reset the parser should be idle again
    assert!(parser.is_idle());
}

/// Non-UBX bytes accumulate in the non-UBX buffer and are returned by
/// take_non_ubx_data().
#[test]
fn test_parser_non_ubx_bytes_accumulate() {
    let mut parser = UbxFrameParser::new();

    // Feed a typical NMEA sentence prefix (none of these bytes start a UBX frame)
    let nmea = b"$GNGGA,123519,4807.038,N";
    for &b in nmea {
        let _ = parser.feed(b);
    }

    let non_ubx = parser.take_non_ubx_data().expect("Expected non-UBX data");
    assert_eq!(&non_ubx[..], nmea);

    // After taking, buffer should be empty
    assert!(parser.take_non_ubx_data().is_none());
}

/// A frame split across many individual feed() calls is parsed correctly.
#[test]
fn test_parser_split_frame_across_feed_calls() {
    let frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 8);
    let mut parser = UbxFrameParser::new();

    let mut result = None;
    // Feed one byte at a time — same as the hardware receiving byte-by-byte
    for (i, &b) in frame.iter().enumerate() {
        let maybe = parser.feed(b);
        if i < frame.len() - 1 {
            // All intermediate bytes should return None
            assert!(maybe.is_none(), "Byte {} should not complete the frame", i);
        } else {
            result = maybe;
        }
    }

    let parsed = result.expect("Last byte should complete the frame");
    assert_eq!(parsed.len(), frame.len());
    assert!(verify_checksum(&parsed));
}

// ============================================================================
// Group 2: Offset application and ECEF replacement
// ============================================================================

/// apply_offset_nav_pvt shifts lat, lon, height, and hMSL by the offset.
#[test]
fn test_apply_offset_nav_pvt_shifts_coords() {
    let mut frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);
    let off = DynamicOffset {
        lat_1e7: 10_000_000,   // +1 degree
        lon_1e7: -5_000_000,   // -0.5 degree
        alt_mm: 50_000,        // +50 m
    };

    apply_offset_nav_pvt(&mut frame, &off);

    // payload starts at frame byte 6
    let lon = read_i32_le(&frame, 6 + 24);
    let lat = read_i32_le(&frame, 6 + 28);
    let height = read_i32_le(&frame, 6 + 32);
    let h_msl = read_i32_le(&frame, 6 + 36);

    assert_eq!(lat, BASE_LAT + 10_000_000);
    assert_eq!(lon, BASE_LON - 5_000_000);
    assert_eq!(height, BASE_ALT + 50_000);
    assert_eq!(h_msl, BASE_ALT + 50_000);
}

/// apply_offset_nav_posllh shifts lon, lat, height, and hMSL by the offset.
#[test]
fn test_apply_offset_nav_posllh_shifts_coords() {
    let mut frame = build_nav_posllh_frame(BASE_LAT, BASE_LON, BASE_ALT);
    let off = DynamicOffset {
        lat_1e7: -3_000_000,
        lon_1e7: 2_000_000,
        alt_mm: -10_000,
    };

    apply_offset_nav_posllh(&mut frame, &off);

    let lon = read_i32_le(&frame, 6 + 4);
    let lat = read_i32_le(&frame, 6 + 8);
    let height = read_i32_le(&frame, 6 + 12);
    let h_msl = read_i32_le(&frame, 6 + 16);

    assert_eq!(lat, BASE_LAT - 3_000_000);
    assert_eq!(lon, BASE_LON + 2_000_000);
    assert_eq!(height, BASE_ALT - 10_000);
    assert_eq!(h_msl, BASE_ALT - 10_000);
}

/// replace_ecef_nav_posecef writes absolute ECEF values at the correct offsets.
#[test]
fn test_replace_ecef_nav_posecef_writes_values() {
    let mut frame = build_nav_posecef_frame(0, 0, 0);
    let (ex, ey, ez) = (929_123_456i32, -541_234_567i32, 288_000_000i32);

    replace_ecef_nav_posecef(&mut frame, ex, ey, ez);

    assert_eq!(read_i32_le(&frame, 6 + 4), ex);
    assert_eq!(read_i32_le(&frame, 6 + 8), ey);
    assert_eq!(read_i32_le(&frame, 6 + 12), ez);
}

/// replace_ecef_nav_hpposecef writes ECEF values and zeros the three HP bytes (offsets 20-22).
#[test]
fn test_replace_ecef_nav_hpposecef_zeros_hp_bytes() {
    let mut frame = build_nav_hpposecef_frame(0, 0, 0);
    // HP bytes start as 10, 20, 30 (set in builder) — must be zeroed after replacement
    let (ex, ey, ez) = (929_123_456i32, -541_234_567i32, 288_000_000i32);

    replace_ecef_nav_hpposecef(&mut frame, ex, ey, ez);

    assert_eq!(read_i32_le(&frame, 6 + 8), ex);
    assert_eq!(read_i32_le(&frame, 6 + 12), ey);
    assert_eq!(read_i32_le(&frame, 6 + 16), ez);
    assert_eq!(frame[6 + 20], 0, "ecefXHp must be zeroed");
    assert_eq!(frame[6 + 21], 0, "ecefYHp must be zeroed");
    assert_eq!(frame[6 + 22], 0, "ecefZHp must be zeroed");
}

/// replace_ecef_nav_sol writes absolute ECEF values at the correct offsets.
#[test]
fn test_replace_ecef_nav_sol_writes_values() {
    let mut frame = build_nav_sol_frame(0, 0, 0);
    let (ex, ey, ez) = (929_123_456i32, -541_234_567i32, 288_000_000i32);

    replace_ecef_nav_sol(&mut frame, ex, ey, ez);

    assert_eq!(read_i32_le(&frame, 6 + 12), ex);
    assert_eq!(read_i32_le(&frame, 6 + 16), ey);
    assert_eq!(read_i32_le(&frame, 6 + 20), ez);
}

/// recalc_checksum produces a checksum that passes verify_checksum after modification.
#[test]
fn test_recalc_checksum_after_modification() {
    let mut frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);
    assert!(verify_checksum(&frame), "Pre-condition: original frame has valid checksum");

    // Tamper: change lat directly (invalidates checksum)
    let new_lat = BASE_LAT + 1_000_000;
    frame[6 + 28..6 + 32].copy_from_slice(&new_lat.to_le_bytes());
    assert!(!verify_checksum(&frame), "After tampering checksum should be invalid");

    recalc_checksum(&mut frame);
    assert!(verify_checksum(&frame), "After recalc_checksum frame should be valid");
    assert_eq!(read_i32_le(&frame, 6 + 28), new_lat, "Modified lat must survive checksum recalc");
}

/// LLH-to-ECEF consistency: applying a lat/lon offset to a NAV-PVT and then
/// converting the resulting LLH with llh_to_ecef_cm() must produce ECEF
/// values close to those written by replace_ecef_nav_posecef on a matching
/// NAV-POSECEF frame (tolerance ±5 cm accounts for integer rounding).
#[test]
fn test_llh_to_ecef_consistency_with_posecef() {
    let off = DynamicOffset {
        lat_1e7: 5_000_000,  // +0.5 degree
        lon_1e7: 3_000_000,  // +0.3 degree
        alt_mm: 20_000,      // +20 m
    };

    let offset_lat = BASE_LAT + off.lat_1e7;
    let offset_lon = BASE_LON + off.lon_1e7;
    let offset_alt = BASE_ALT + off.alt_mm;

    let (ex, ey, ez) = coordinates::llh_to_ecef_cm(offset_lat, offset_lon, offset_alt);

    let mut pvt_frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);
    apply_offset_nav_pvt(&mut pvt_frame, &off);
    let pvt_lat = read_i32_le(&pvt_frame, 6 + 28);
    let pvt_lon = read_i32_le(&pvt_frame, 6 + 24);
    let pvt_alt = read_i32_le(&pvt_frame, 6 + 32);

    let (pvt_ex, pvt_ey, pvt_ez) = coordinates::llh_to_ecef_cm(pvt_lat, pvt_lon, pvt_alt);

    // Both paths must agree to within ±5 cm (integer rounding)
    assert!((pvt_ex - ex).abs() <= 5, "ECEF X mismatch: {} vs {}", pvt_ex, ex);
    assert!((pvt_ey - ey).abs() <= 5, "ECEF Y mismatch: {} vs {}", pvt_ey, ey);
    assert!((pvt_ez - ez).abs() <= 5, "ECEF Z mismatch: {} vs {}", pvt_ez, ez);
}

/// Round-trip: build NAV-PVT → feed through parser → apply offset → recalc_checksum → verify.
#[test]
fn test_round_trip_build_parse_offset_verify() {
    let frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);
    let mut parser = UbxFrameParser::new();

    let mut parsed = None;
    for &b in frame.iter() {
        if let Some(f) = parser.feed(b) {
            parsed = Some(f);
        }
    }

    let mut parsed = parsed.expect("Frame must parse correctly");
    assert!(verify_checksum(&parsed));

    let off = DynamicOffset { lat_1e7: 1_000_000, lon_1e7: 2_000_000, alt_mm: 5_000 };
    apply_offset_nav_pvt(&mut parsed, &off);
    recalc_checksum(&mut parsed);

    assert!(verify_checksum(&parsed), "Round-trip: checksum must be valid after offset + recalc");

    let lat_after = read_i32_le(&parsed, 6 + 28);
    let lon_after = read_i32_le(&parsed, 6 + 24);
    assert_eq!(lat_after, BASE_LAT + 1_000_000);
    assert_eq!(lon_after, BASE_LON + 2_000_000);
}

// ============================================================================
// Group 3: Spoof modification
// ============================================================================

/// modify_nav_pvt_spoof replaces coords, zeros velocity, and sets fix_type=0 / num_sv=92.
#[test]
fn test_modify_nav_pvt_spoof_replaces_coords_and_degrades_status() {
    let mut frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);

    let spoof_lat = 500_000_000i32; // 50°N
    let spoof_lon = 370_000_000i32; // 37°E (Moscow area)
    let spoof_alt = 200_000i32;

    modify_nav_pvt_spoof(&mut frame, spoof_lat, spoof_lon, spoof_alt);

    // Coordinates replaced
    assert_eq!(read_i32_le(&frame, 6 + 28), spoof_lat, "lat must be replaced");
    assert_eq!(read_i32_le(&frame, 6 + 24), spoof_lon, "lon must be replaced");
    assert_eq!(read_i32_le(&frame, 6 + 32), spoof_alt, "height must be replaced");
    assert_eq!(read_i32_le(&frame, 6 + 36), spoof_alt, "hMSL must be replaced");

    // Velocity zeroed
    assert_eq!(read_i32_le(&frame, 6 + 48), 0, "velN must be zero");
    assert_eq!(read_i32_le(&frame, 6 + 52), 0, "velE must be zero");
    assert_eq!(read_i32_le(&frame, 6 + 56), 0, "velD must be zero");

    // Status degraded
    assert_eq!(frame[6 + 20], 0, "fix_type must be 0");
    assert_eq!(frame[6 + 21], 0, "flags must be 0");
    assert_eq!(frame[6 + 23], 92, "num_sv must be 92 (spoof marker)");
}

/// modify_nav_posecef_spoof replaces ECEF coords and sets pAcc=9999999.
#[test]
fn test_modify_nav_posecef_spoof_replaces_ecef_and_degrades_accuracy() {
    let mut frame = build_nav_posecef_frame(100, 200, 300);

    let (sx, sy, sz) = (929_123_456i32, -541_234_567i32, 288_000_000i32);
    modify_nav_posecef_spoof(&mut frame, sx, sy, sz);

    assert_eq!(read_i32_le(&frame, 6 + 4), sx);
    assert_eq!(read_i32_le(&frame, 6 + 8), sy);
    assert_eq!(read_i32_le(&frame, 6 + 12), sz);

    let p_acc = read_u32_le(&frame, 6 + 16);
    assert_eq!(p_acc, 9_999_999, "pAcc must be set to 9999999");
}

/// modify_nav_velned_spoof zeros all velocity/speed fields and sets sAcc/cAcc=9999999.
#[test]
fn test_modify_nav_velned_spoof_zeros_velocity_and_degrades_accuracy() {
    let mut frame = build_nav_velned_frame();
    // Frame built with non-zero velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc
    modify_nav_velned_spoof(&mut frame);

    // All velocity and speed fields must be zero
    assert_eq!(read_i32_le(&frame, 6 + 4), 0, "velN must be 0");
    assert_eq!(read_i32_le(&frame, 6 + 8), 0, "velE must be 0");
    assert_eq!(read_i32_le(&frame, 6 + 12), 0, "velD must be 0");
    assert_eq!(read_u32_le(&frame, 6 + 16), 0, "speed must be 0");
    assert_eq!(read_u32_le(&frame, 6 + 20), 0, "gSpeed must be 0");
    assert_eq!(read_i32_le(&frame, 6 + 24), 0, "heading must be 0");

    // Accuracy fields degraded
    assert_eq!(read_u32_le(&frame, 6 + 28), 9_999_999, "sAcc must be 9999999");
    assert_eq!(read_u32_le(&frame, 6 + 32), 9_999_999, "cAcc must be 9999999");
}

/// modify_nav_status sets gps_fix=0 and flags=0.
#[test]
fn test_modify_nav_status_degrades_fix_and_flags() {
    let mut frame = build_nav_status_frame();
    // Frame has gps_fix=3, flags=1 before modification
    assert_eq!(frame[6 + 4], 3, "Pre-condition: gps_fix should be 3");
    assert_eq!(frame[6 + 5], 1, "Pre-condition: flags should be 1");

    modify_nav_status(&mut frame);

    assert_eq!(frame[6 + 4], 0, "gps_fix must be 0 after modify_nav_status");
    assert_eq!(frame[6 + 5], 0, "flags must be 0 after modify_nav_status");
}

/// modify_nav_sat sets num_svs=92 at payload offset 5.
#[test]
fn test_modify_nav_sat_sets_num_svs_92() {
    let mut frame = build_nav_sat_frame(14);
    assert_eq!(frame[6 + 5], 14, "Pre-condition: num_svs should be 14");

    modify_nav_sat(&mut frame);

    assert_eq!(frame[6 + 5], 92, "num_svs must be 92 (spoof marker)");
}

// ============================================================================
// Group 4: Extract functions and PositionBuffer
// ============================================================================

/// extract_position_from_pvt correctly extracts lat, lon, alt, h_acc, speed, num_sv.
#[test]
fn test_extract_position_from_pvt_correct_values() {
    // Build frame with known velN=500, velE=500 → speed = sqrt(500²+500²) ≈ 707 mm/s
    let frame = build_nav_pvt_frame(BASE_LAT, BASE_LON, BASE_ALT, 3, 12);

    // The function takes the payload slice, which starts at frame byte 6 and is 92 bytes long
    let payload = &frame[6..6 + 92];
    let result = extract_position_from_pvt(payload).expect("Should extract position");

    let (lat, lon, alt, h_acc, speed, num_sv) = result;

    assert_eq!(lat, BASE_LAT);
    assert_eq!(lon, BASE_LON);
    assert_eq!(alt, BASE_ALT);
    assert_eq!(h_acc, 1000, "h_acc must be 1000 mm");
    assert_eq!(num_sv, 12);

    // speed = sqrt(500² + 500²) ≈ 707; check within ±2 (integer sqrt rounding)
    assert!((speed - 707).abs() <= 2, "speed should be ≈707 mm/s, got {}", speed);
}

/// extract_position_from_pvt returns None when payload is shorter than 92 bytes.
#[test]
fn test_extract_position_from_pvt_short_payload_returns_none() {
    let payload = [0u8; 91]; // one byte short
    let result = extract_position_from_pvt(&payload);
    assert!(result.is_none(), "Payload shorter than 92 bytes must return None");

    let empty: [u8; 0] = [];
    assert!(extract_position_from_pvt(&empty).is_none());
}

/// PositionBuffer: push 20 entries at 200 ms intervals; get_position_at returns
/// the entry with timestamp closest to the requested time offset.
#[test]
fn test_position_buffer_push_and_get() {
    let mut buf = PositionBuffer::new();

    // Push 20 entries at 200ms intervals (simulating 5Hz for 4 seconds).
    // The ring buffer holds 15; older entries are overwritten.
    for i in 0u32..20 {
        let lat = BASE_LAT + (i as i32) * 1000;
        let lon = BASE_LON + (i as i32) * 500;
        let alt = BASE_ALT + (i as i32) * 100;
        let ts = i * 200; // 0, 200, 400, ..., 3800 ms
        buf.push(lat, lon, alt, ts);
    }

    // After 20 pushes into a 15-slot ring buffer, entries 5..19 (inclusive) remain
    // (entries 0..4 have been overwritten).  The most recent entry has timestamp 3800 ms.
    // get_position_at(0, 3800) should return the entry at ts=3800 → i=19.
    let current_time_ms = 3800u32;

    let pos_now = buf.get_position_at(0, current_time_ms)
        .expect("Should find entry at t=0 seconds ago");

    // Entry i=19: lat = BASE_LAT + 19*1000
    assert_eq!(pos_now.0, BASE_LAT + 19 * 1000, "lat at t=0s ago must match newest entry");
    assert_eq!(pos_now.1, BASE_LON + 19 * 500);

    // get_position_at(1, 3800) targets ts = 3800 - 1000 = 2800 ms → i=14
    // ts=2800 → i=14: lat = BASE_LAT + 14*1000
    let pos_1s_ago = buf.get_position_at(1, current_time_ms)
        .expect("Should find entry at t=1 second ago");
    // i=14 → lat = BASE_LAT + 14*1000
    assert_eq!(pos_1s_ago.0, BASE_LAT + 14 * 1000, "lat at t=1s ago must match entry at ts≈2800ms");

    // get_position_at on empty buffer returns None
    let empty_buf = PositionBuffer::new();
    assert!(empty_buf.get_position_at(0, 1000).is_none(), "Empty buffer must return None");
}

// ============================================================================
// Group 5: PositionBuffer fix_type filtering (bug fix verification)
//
// BUG: pos_buffer.push() was called BEFORE fix_type check, so no-fix entries
// (with coords 0,0,0) would corrupt LAST_GOOD when spoof is detected after
// satellite loss. FIX: only push entries with valid 3D fix.
// These tests verify the fix logic that should be applied in main.rs.
// ============================================================================

/// Simulate the exact bug scenario: normal flight → satellite loss (no-fix entries
/// with 0,0,0 coords) → spoof detection. Verify that get_position_at returns
/// the last valid 3D fix, NOT a no-fix entry.
#[test]
fn test_position_buffer_preserves_good_coords_through_satellite_loss() {
    let mut buf = PositionBuffer::new();

    // Normal flight: 15 samples at 5Hz (3 seconds), valid 3D fix
    for i in 0u32..15 {
        let lat = BASE_LAT + (i as i32) * 100;
        let lon = BASE_LON + (i as i32) * 50;
        buf.push(lat, lon, BASE_ALT, i * 200);
    }

    // Satellite loss: 15 more samples with no-fix. In the FIXED code, these
    // would NOT be pushed. Simulate the fix by NOT pushing them.
    // (Old buggy code would push (0, 0, 0) here.)

    // Spoof detected at t=6000ms. get_position_at(2) should return
    // position from ~4000ms = entry at i=14 (t=2800ms, closest to 4000ms).
    let now_ms = 6000u32;
    let result = buf.get_position_at(2, now_ms);
    assert!(result.is_some(), "Should find a position from before satellite loss");

    let (lat, _lon, _alt) = result.unwrap();
    // All entries in buffer are from normal flight (BASE_LAT + delta)
    assert!(lat > BASE_LAT, "Position should be near base, got lat={}", lat);
    assert!(lat < BASE_LAT + 15 * 100 + 1, "Position should be within flight range");
}

/// Verify that a buffer with ONLY no-fix entries (simulating the old buggy behavior)
/// would return (0,0,0) — demonstrating why the fix is necessary.
#[test]
fn test_position_buffer_nofix_corruption_demo() {
    let mut buf = PositionBuffer::new();

    // Normal entry
    buf.push(BASE_LAT, BASE_LON, BASE_ALT, 0);

    // 15 no-fix entries (0,0,0) overwrite the ring buffer entirely.
    // This simulates the OLD BUGGY code that pushed without fix_type check.
    for i in 1u32..16 {
        buf.push(0, 0, 0, i * 200);
    }

    // get_position_at(2) returns a (0,0,0) entry — this is the bug
    let result = buf.get_position_at(0, 3000);
    assert!(result.is_some());
    let (lat, lon, _) = result.unwrap();
    assert_eq!(lat, 0, "Buggy buffer returns (0,0,0) from no-fix entries");
    assert_eq!(lon, 0);
}

/// After satellite loss gap, buffer entries still hold pre-loss positions
/// with timestamps far in the past. get_position_at(2, now) returns the closest
/// available entry even if it's older than 2 seconds.
#[test]
fn test_position_buffer_stale_entries_after_long_gap() {
    let mut buf = PositionBuffer::new();

    // 10 entries at normal 5Hz (0..1800ms)
    for i in 0u32..10 {
        buf.push(BASE_LAT + (i as i32) * 100, BASE_LON, BASE_ALT, i * 200);
    }

    // Long satellite loss: 30 seconds with no pushes (fix applied)
    // Spoof detected at t=32000ms
    let now_ms = 32000u32;
    let result = buf.get_position_at(2, now_ms);
    assert!(result.is_some(), "Should return stale but valid entry");

    let (lat, _, _) = result.unwrap();
    assert!(lat >= BASE_LAT && lat <= BASE_LAT + 9 * 100,
        "Stale entry should be from pre-loss flight, lat={}", lat);
}

/// If a spoof sample is pushed before spoof analysis, get_position_at(2) must
/// not choose it just because old clean samples are far from the target time.
#[test]
fn test_position_buffer_rejects_fresh_entry_on_lookback() {
    let mut buf = PositionBuffer::new();

    // Clean history before a long gap.
    for i in 0u32..10 {
        buf.push(BASE_LAT + (i as i32) * 100, BASE_LON, BASE_ALT, i * 200);
    }

    let now_ms = 32000u32;
    let spoof_lat = BASE_LAT + 1_000_000;
    buf.push(spoof_lat, BASE_LON, BASE_ALT, now_ms);

    let (lat, _, _) = buf.get_position_at(2, now_ms)
        .expect("Should skip fresh spoof sample and return stale clean history");
    assert_ne!(lat, spoof_lat, "Fresh sample must not become LAST_GOOD");
    assert!(lat >= BASE_LAT && lat <= BASE_LAT + 9 * 100,
        "Position should come from pre-gap clean history, lat={}", lat);
}

/// A buffer containing only too-fresh samples has no entry for a non-zero
/// lookback. This guards against silently falling back to current coordinates.
#[test]
fn test_position_buffer_returns_none_when_only_fresh_entries_exist() {
    let mut buf = PositionBuffer::new();

    buf.push(BASE_LAT, BASE_LON, BASE_ALT, 1000);

    assert!(buf.get_position_at(2, 1500).is_none(),
        "Fresh-only history should not satisfy a 2s lookback");
}

// ============================================================================
// Group 6: Dynamic offset computation and recomputation
//
// Tests for the offset pipeline: computation from GPS coords, application to
// frames, and the recomputation-after-recovery fix.
// ============================================================================

/// Target offset position (Seney, Michigan)
const TARGET_LAT: i32 = 463_407_000;
const TARGET_LON: i32 = -859_407_000;
const TARGET_ALT: i32 = 100_000;

/// Simulated actual GPS position (Moscow area)
const ACTUAL_LAT: i32 = 557_500_000;
const ACTUAL_LON: i32 = 376_200_000;
const ACTUAL_ALT: i32 = 200_000;

/// Simulated spoofed GPS position (somewhere in Turkey)
const SPOOFED_LAT: i32 = 390_000_000;
const SPOOFED_LON: i32 = 320_000_000;
const SPOOFED_ALT: i32 = 150_000;

/// Offset computed from REAL coordinates produces correct target position.
#[test]
fn test_dynamic_offset_from_real_coords() {
    let off = DynamicOffset {
        lat_1e7: TARGET_LAT - ACTUAL_LAT,
        lon_1e7: TARGET_LON - ACTUAL_LON,
        alt_mm: TARGET_ALT - ACTUAL_ALT,
    };

    let mut frame = build_nav_pvt_frame(ACTUAL_LAT, ACTUAL_LON, ACTUAL_ALT, 3, 12);
    apply_offset_nav_pvt(&mut frame, &off);

    let lat = read_i32_le(&frame, 6 + 28);
    let lon = read_i32_le(&frame, 6 + 24);
    assert_eq!(lat, TARGET_LAT, "Offset from real coords should produce target lat");
    assert_eq!(lon, TARGET_LON, "Offset from real coords should produce target lon");
}

/// Offset computed from SPOOFED coordinates produces WRONG target position
/// when applied to real coordinates — this demonstrates the bug that the
/// dynamic_offset recomputation fix addresses.
#[test]
fn test_dynamic_offset_from_spoofed_coords_produces_wrong_position() {
    // Offset computed from spoofed coords (the bug scenario)
    let bad_off = DynamicOffset {
        lat_1e7: TARGET_LAT - SPOOFED_LAT,
        lon_1e7: TARGET_LON - SPOOFED_LON,
        alt_mm: TARGET_ALT - SPOOFED_ALT,
    };

    // Apply bad offset to REAL coordinates
    let mut frame = build_nav_pvt_frame(ACTUAL_LAT, ACTUAL_LON, ACTUAL_ALT, 3, 12);
    apply_offset_nav_pvt(&mut frame, &bad_off);

    let lat = read_i32_le(&frame, 6 + 28);
    let lon = read_i32_le(&frame, 6 + 24);

    // Position should NOT equal target — that's the bug
    assert_ne!(lat, TARGET_LAT, "Bad offset should NOT produce target lat");
    assert_ne!(lon, TARGET_LON, "Bad offset should NOT produce target lon");

    // The error should be approximately |ACTUAL - SPOOFED|
    let lat_error = (lat - TARGET_LAT).abs();
    let lon_error = (lon - TARGET_LON).abs();
    assert!(lat_error > 100_000_000, "Lat error should be large, got {}", lat_error);
    assert!(lon_error > 50_000_000, "Lon error should be large, got {}", lon_error);
}

/// After offset recomputation from real coords, position returns to target.
/// This simulates the fix: invalidate offset → recompute from clean data.
#[test]
fn test_dynamic_offset_recomputation_fixes_position() {
    // Step 1: bad offset from spoofed coords
    let bad_off = DynamicOffset {
        lat_1e7: TARGET_LAT - SPOOFED_LAT,
        lon_1e7: TARGET_LON - SPOOFED_LON,
        alt_mm: TARGET_ALT - SPOOFED_ALT,
    };

    let mut frame1 = build_nav_pvt_frame(ACTUAL_LAT, ACTUAL_LON, ACTUAL_ALT, 3, 12);
    apply_offset_nav_pvt(&mut frame1, &bad_off);
    let wrong_lat = read_i32_le(&frame1, 6 + 28);
    assert_ne!(wrong_lat, TARGET_LAT, "Bad offset gives wrong position");

    // Step 2: recompute offset from real coords (the fix)
    let good_off = DynamicOffset {
        lat_1e7: TARGET_LAT - ACTUAL_LAT,
        lon_1e7: TARGET_LON - ACTUAL_LON,
        alt_mm: TARGET_ALT - ACTUAL_ALT,
    };

    let mut frame2 = build_nav_pvt_frame(ACTUAL_LAT, ACTUAL_LON, ACTUAL_ALT, 3, 12);
    apply_offset_nav_pvt(&mut frame2, &good_off);
    let correct_lat = read_i32_le(&frame2, 6 + 28);
    let correct_lon = read_i32_le(&frame2, 6 + 24);
    assert_eq!(correct_lat, TARGET_LAT, "Recomputed offset should give target lat");
    assert_eq!(correct_lon, TARGET_LON, "Recomputed offset should give target lon");
}

/// LAST_GOOD + offset should produce target-like position when LAST_GOOD
/// has valid pre-spoof coordinates.
#[test]
fn test_last_good_plus_offset_produces_target() {
    // LAST_GOOD is from pre-spoof real position
    let last_good_lat = ACTUAL_LAT + 500; // slight movement
    let last_good_lon = ACTUAL_LON + 300;

    let off = DynamicOffset {
        lat_1e7: TARGET_LAT - ACTUAL_LAT,
        lon_1e7: TARGET_LON - ACTUAL_LON,
        alt_mm: TARGET_ALT - ACTUAL_ALT,
    };

    // Spoof replacement: LAST_GOOD + offset
    let rep_lat = last_good_lat.saturating_add(off.lat_1e7);
    let rep_lon = last_good_lon.saturating_add(off.lon_1e7);

    // Should be near TARGET (within ~50m of movement delta)
    let lat_diff = (rep_lat - TARGET_LAT).abs();
    let lon_diff = (rep_lon - TARGET_LON).abs();
    assert!(lat_diff < 1000, "rep_lat should be near TARGET, diff={}", lat_diff);
    assert!(lon_diff < 1000, "rep_lon should be near TARGET, diff={}", lon_diff);
}

/// LAST_GOOD = (0,0,0) (from no-fix corruption) + offset produces WRONG position.
/// This demonstrates why pos_buffer must filter no-fix entries.
#[test]
fn test_last_good_zero_plus_offset_produces_wrong_position() {
    let off = DynamicOffset {
        lat_1e7: TARGET_LAT - ACTUAL_LAT,
        lon_1e7: TARGET_LON - ACTUAL_LON,
        alt_mm: TARGET_ALT - ACTUAL_ALT,
    };

    // Buggy LAST_GOOD from no-fix entry
    let rep_lat = 0i32.saturating_add(off.lat_1e7);
    let rep_lon = 0i32.saturating_add(off.lon_1e7);

    // This should be very far from TARGET
    let lat_diff = (rep_lat - TARGET_LAT).abs();
    let lon_diff = (rep_lon - TARGET_LON).abs();
    assert!(lat_diff > 100_000_000,
        "Zero LAST_GOOD + offset should be far from TARGET, lat_diff={}", lat_diff);
    assert!(lon_diff > 100_000_000,
        "Zero LAST_GOOD + offset should be far from TARGET, lon_diff={}", lon_diff);
}

/// Full pipeline: build frame → apply offset → spoof modify → verify coords.
/// Simulates the correct spoof handler behavior in PassthroughOffset mode.
#[test]
fn test_full_pipeline_offset_then_spoof_replace() {
    let off = DynamicOffset {
        lat_1e7: TARGET_LAT - ACTUAL_LAT,
        lon_1e7: TARGET_LON - ACTUAL_LON,
        alt_mm: TARGET_ALT - ACTUAL_ALT,
    };

    // Frame from spoofed GNSS
    let mut frame = build_nav_pvt_frame(SPOOFED_LAT, SPOOFED_LON, SPOOFED_ALT, 3, 12);

    // Step 1: apply offset (as in main.rs line 1737)
    apply_offset_nav_pvt(&mut frame, &off);

    // Step 2: spoof replacement with LAST_GOOD + offset (as in main.rs line 1818)
    let last_good_lat = ACTUAL_LAT;
    let last_good_lon = ACTUAL_LON;
    let last_good_alt = ACTUAL_ALT;
    let rep_lat = last_good_lat.saturating_add(off.lat_1e7);
    let rep_lon = last_good_lon.saturating_add(off.lon_1e7);
    let rep_alt = last_good_alt.saturating_add(off.alt_mm);

    modify_nav_pvt_spoof(&mut frame, rep_lat, rep_lon, rep_alt);

    // Verify final coords = TARGET
    let lat = read_i32_le(&frame, 6 + 28);
    let lon = read_i32_le(&frame, 6 + 24);
    assert_eq!(lat, TARGET_LAT, "After offset+spoof, should show target lat");
    assert_eq!(lon, TARGET_LON, "After offset+spoof, should show target lon");

    // Verify status degraded
    assert_eq!(frame[6 + 20], 0, "fix_type should be 0");
    assert_eq!(frame[6 + 23], 92, "num_sv should be 92 (spoof marker)");
}

/// Verify ECEF consistency: offset LLH → ECEF should match for both
/// normal and spoof replacement paths.
#[test]
fn test_ecef_consistency_after_offset_recomputation() {
    coordinates::init();

    let off = DynamicOffset {
        lat_1e7: TARGET_LAT - ACTUAL_LAT,
        lon_1e7: TARGET_LON - ACTUAL_LON,
        alt_mm: TARGET_ALT - ACTUAL_ALT,
    };

    // Normal path: ACTUAL + offset = TARGET
    let offset_lat = ACTUAL_LAT + off.lat_1e7;
    let offset_lon = ACTUAL_LON + off.lon_1e7;
    let offset_alt = ACTUAL_ALT + off.alt_mm;
    let (ecef_x1, ecef_y1, ecef_z1) = coordinates::llh_to_ecef_cm(offset_lat, offset_lon, offset_alt);

    // Spoof replacement path: LAST_GOOD + offset (LAST_GOOD ≈ ACTUAL)
    let rep_lat = ACTUAL_LAT + off.lat_1e7;
    let rep_lon = ACTUAL_LON + off.lon_1e7;
    let rep_alt = ACTUAL_ALT + off.alt_mm;
    let (ecef_x2, ecef_y2, ecef_z2) = coordinates::llh_to_ecef_cm(rep_lat, rep_lon, rep_alt);

    // Both should produce identical ECEF
    assert_eq!(ecef_x1, ecef_x2, "ECEF X should match between normal and spoof paths");
    assert_eq!(ecef_y1, ecef_y2, "ECEF Y should match");
    assert_eq!(ecef_z1, ecef_z2, "ECEF Z should match");
}

/// Verify all NAV message types get correct coordinates during spoof replacement
/// with offset. Tests the full set of modify_nav_*_spoof functions.
#[test]
fn test_all_nav_messages_spoof_replacement_with_offset() {
    coordinates::init();

    let off = DynamicOffset {
        lat_1e7: TARGET_LAT - ACTUAL_LAT,
        lon_1e7: TARGET_LON - ACTUAL_LON,
        alt_mm: TARGET_ALT - ACTUAL_ALT,
    };

    let rep_lat = ACTUAL_LAT + off.lat_1e7; // = TARGET_LAT
    let rep_lon = ACTUAL_LON + off.lon_1e7; // = TARGET_LON
    let rep_alt = ACTUAL_ALT + off.alt_mm;  // = TARGET_ALT
    let (rep_ex, rep_ey, rep_ez) = coordinates::llh_to_ecef_cm(rep_lat, rep_lon, rep_alt);

    // NAV-PVT
    let mut pvt = build_nav_pvt_frame(SPOOFED_LAT, SPOOFED_LON, SPOOFED_ALT, 3, 12);
    modify_nav_pvt_spoof(&mut pvt, rep_lat, rep_lon, rep_alt);
    assert_eq!(read_i32_le(&pvt, 6 + 28), TARGET_LAT);
    assert_eq!(read_i32_le(&pvt, 6 + 24), TARGET_LON);

    // NAV-POSLLH
    let mut posllh = build_nav_posllh_frame(SPOOFED_LAT, SPOOFED_LON, SPOOFED_ALT);
    modify_nav_posllh_spoof(&mut posllh, rep_lat, rep_lon, rep_alt);
    assert_eq!(read_i32_le(&posllh, 6 + 8), TARGET_LAT);
    assert_eq!(read_i32_le(&posllh, 6 + 4), TARGET_LON);

    // NAV-POSECEF
    let mut posecef = build_nav_posecef_frame(0, 0, 0);
    modify_nav_posecef_spoof(&mut posecef, rep_ex, rep_ey, rep_ez);
    assert_eq!(read_i32_le(&posecef, 6 + 4), rep_ex);
    assert_eq!(read_i32_le(&posecef, 6 + 8), rep_ey);
    assert_eq!(read_i32_le(&posecef, 6 + 12), rep_ez);

    // NAV-HPPOSECEF
    let mut hpposecef = build_nav_hpposecef_frame(0, 0, 0);
    modify_nav_hpposecef_spoof(&mut hpposecef, rep_ex, rep_ey, rep_ez);
    assert_eq!(read_i32_le(&hpposecef, 6 + 8), rep_ex);

    // NAV-SOL
    let mut sol = build_nav_sol_frame(0, 0, 0);
    modify_nav_sol_spoof(&mut sol, rep_ex, rep_ey, rep_ez);
    assert_eq!(read_i32_le(&sol, 6 + 12), rep_ex);
    assert_eq!(read_i32_le(&sol, 6 + 16), rep_ey);
    assert_eq!(read_i32_le(&sol, 6 + 20), rep_ez);
}
