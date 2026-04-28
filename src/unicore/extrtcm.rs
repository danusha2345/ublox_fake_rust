//! Extended RTCM 4074 builder (Unicore proprietary).
//!
//! Frame layout for message type 4074:
//! ```text
//! [0xD3][reserved(6)|len(10)]  [MsgType(12)=4074 | SubID(12)] [Data N bytes]  [CRC24-Q]
//!  \________ 3 bytes _______/   \_________ 3 bytes _________/  \__ N bytes __/ \_ 3 _/
//! ```
//!
//! Data fields for Sub-IDs observed in production
//! (`uart_term/target/release/log_2026-04-22_17-27-43.txt`) are captured here
//! verbatim — the emulator replays them byte-for-byte. This is adequate for
//! drones that treat the bundle as an opaque heartbeat; the inner semantics
//! are not parsed by any consumer we target.
//!
//! Reference: docs/UC6580I.md §6.3.

#![allow(dead_code)]

use super::rtcm3;

/// RTCM message type reserved for Unicore proprietary frames.
pub const MSG_TYPE: u16 = 4074;

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

/// Build an RTCM 4074 frame with the given Sub-ID and `data` into `buf`.
/// Returns the total bytes written, or 0 on overflow.
pub fn build_sub(buf: &mut [u8], sub_id: u16, data: &[u8]) -> usize {
    // payload = MsgType(12b) + SubID(12b) + data = 3 + data.len() bytes
    let payload_len = 3 + data.len();
    if payload_len > rtcm3::MAX_PAYLOAD {
        return 0;
    }
    let total = rtcm3::HEADER_LEN + payload_len + rtcm3::CRC_LEN;
    if buf.len() < total {
        return 0;
    }
    // Header.
    buf[0] = rtcm3::PREAMBLE;
    buf[1] = ((payload_len >> 8) as u8) & 0x03;
    buf[2] = (payload_len & 0xFF) as u8;
    // MsgType + SubID, big-endian bit-packed:
    //   payload[0] = top 8 bits of MsgType
    //   payload[1] = low 4 bits of MsgType | top 4 bits of SubID
    //   payload[2] = low 8 bits of SubID
    buf[3] = ((MSG_TYPE >> 4) & 0xFF) as u8;
    buf[4] = (((MSG_TYPE & 0x0F) << 4) as u8) | ((sub_id >> 8) as u8 & 0x0F);
    buf[5] = (sub_id & 0xFF) as u8;
    // Data.
    buf[6..6 + data.len()].copy_from_slice(data);
    // CRC over header + payload.
    let crc = rtcm3::crc24_q(&buf[..rtcm3::HEADER_LEN + payload_len]);
    let off = rtcm3::HEADER_LEN + payload_len;
    buf[off] = ((crc >> 16) & 0xFF) as u8;
    buf[off + 1] = ((crc >> 8) & 0xFF) as u8;
    buf[off + 2] = (crc & 0xFF) as u8;
    total
}

// ---------------------------------------------------------------------------
// In-place modifier for sub 0x0FF (Receiver Information)
// ---------------------------------------------------------------------------

/// Sub-ID for Receiver Information (Doc §5.2.2.1).
pub const SUB_RECEIVER_INFO: u16 = 0x0FF;

/// Total expected payload length for sub 0x0FF: 3-byte msg+sub header +
/// 160-byte data field.
pub const RECEIVER_INFO_PAYLOAD_LEN: usize = 3 + 160;

/// Total frame length for sub 0x0FF: D3+LL header (3) + 163-byte payload
/// + CRC24Q (3) = 169 bytes.
pub const RECEIVER_INFO_FRAME_LEN: usize = 3 + RECEIVER_INFO_PAYLOAD_LEN + 3;

/// Convert lat/lon from `1e-7 deg` (our internal/u-blox unit) to the
/// `2^-32 deg` units used by 4074-0xFF Lat/Lon S64 fields.
#[inline]
pub fn llh_e7_to_2pn32(coord_1e7: i32) -> i64 {
    // (coord_1e7 / 1e7) * 2^32  =  (coord_1e7 << 32) / 10_000_000
    // Use i128 to avoid overflow; lat range ±90 → coord_1e7 ≤ 9e8.
    ((coord_1e7 as i128) * (1i128 << 32) / 10_000_000) as i64
}

/// Modify a Receiver Information frame in-place to advertise either a
/// forced-valid fix or a forced-no-fix at the target coordinates.
///
/// Always rewrites Lat/Lon/Hae/Hmsl/ECEF to `target` so the drone never
/// sees the chip's real-or-empty position. `Quality` and `SatNum` flip on
/// the `valid` flag — same semantic as the NMEA Forced3dFix → NOFIX
/// 22 s window: first 22 s `valid=true` (drone commits home point),
/// then `valid=false` (drone enters "satellites lost" but coords stay
/// at target). CRC24Q is recomputed.
///
/// `frame` must be exactly `RECEIVER_INFO_FRAME_LEN` (169) bytes
/// containing a verified RTCM3 4074-0xFF frame. Returns `Some(())` on
/// success, or `None` on length / msg-id / sub-id mismatch.
///
/// Coordinate conversions performed inside:
/// - Lat/Lon S64: `coord_1e7 → 2^-32 deg`
/// - Hae/Hmsl S32: target altitude mm
/// - ECEF X/Y/Z S64: caller-provided WGS84 conversion of LLH
pub fn modify_receiver_info(
    frame: &mut [u8],
    valid: bool,
    target_lat_1e7: i32,
    target_lon_1e7: i32,
    target_alt_mm: i32,
    target_ecef_xyz_mm: (i64, i64, i64),
) -> Option<()> {
    if frame.len() != RECEIVER_INFO_FRAME_LEN {
        return None;
    }
    if frame[0] != rtcm3::PREAMBLE {
        return None;
    }
    let payload_len = (((frame[1] as u16) & 0x03) << 8) | (frame[2] as u16);
    if payload_len as usize != RECEIVER_INFO_PAYLOAD_LEN {
        return None;
    }
    // msg_type (12 bits) + sub_id (12 bits) at frame[3..6]
    let msg_type = ((frame[3] as u16) << 4) | ((frame[4] as u16) >> 4);
    let sub_id = (((frame[4] as u16) & 0x0F) << 8) | (frame[5] as u16);
    if msg_type != MSG_TYPE || sub_id != SUB_RECEIVER_INFO {
        return None;
    }

    // Data starts at frame[6] (after D3+LL+MID).
    // Doc Table 5-11 offsets are within the data field (not payload).
    const DATA_BASE: usize = 6;
    const O_SATNUM: usize = DATA_BASE + 7;
    const O_LON: usize = DATA_BASE + 8;
    const O_LAT: usize = DATA_BASE + 16;
    const O_HAE: usize = DATA_BASE + 24;
    const O_HMSL: usize = DATA_BASE + 28;
    const O_ECEF_X: usize = DATA_BASE + 32;
    const O_ECEF_Y: usize = DATA_BASE + 40;
    const O_ECEF_Z: usize = DATA_BASE + 48;
    const O_QUALITY: usize = DATA_BASE + 56;
    const O_VEL_E: usize = DATA_BASE + 57;
    const O_VEL_N: usize = DATA_BASE + 61;
    const O_VEL_U: usize = DATA_BASE + 65;
    const O_SPEED: usize = DATA_BASE + 69;
    const O_HEADING: usize = DATA_BASE + 73;
    const O_HDOP: usize = DATA_BASE + 75;
    const O_VDOP: usize = DATA_BASE + 77;
    const O_PDOP: usize = DATA_BASE + 79;
    const O_GDOP: usize = DATA_BASE + 81;
    const O_TDOP: usize = DATA_BASE + 83;
    const O_EACC: usize = DATA_BASE + 85;
    const O_NACC: usize = DATA_BASE + 89;
    const O_UACC: usize = DATA_BASE + 93;
    const O_TACC: usize = DATA_BASE + 97;
    const O_XACC: usize = DATA_BASE + 101;
    const O_YACC: usize = DATA_BASE + 105;
    const O_ZACC: usize = DATA_BASE + 109;
    const O_VEACC: usize = DATA_BASE + 113;
    const O_VNACC: usize = DATA_BASE + 117;
    const O_VUACC: usize = DATA_BASE + 121;

    // SatNum and Quality flip on `valid`. Coordinates stay at target in both.
    frame[O_SATNUM] = if valid { 16 } else { 1 };
    frame[O_QUALITY] = if valid { 1 } else { 0 };

    // Lat/Lon as S64 in 2^-32 deg units, big-endian.
    let lon_2pn32 = llh_e7_to_2pn32(target_lon_1e7);
    let lat_2pn32 = llh_e7_to_2pn32(target_lat_1e7);
    frame[O_LON..O_LON + 8].copy_from_slice(&lon_2pn32.to_be_bytes());
    frame[O_LAT..O_LAT + 8].copy_from_slice(&lat_2pn32.to_be_bytes());

    // Hae/Hmsl as S32 mm, big-endian. Use target_alt_mm for both.
    let hae_be = target_alt_mm.to_be_bytes();
    frame[O_HAE..O_HAE + 4].copy_from_slice(&hae_be);
    frame[O_HMSL..O_HMSL + 4].copy_from_slice(&hae_be);

    // ECEF X/Y/Z as S64 mm, big-endian.
    frame[O_ECEF_X..O_ECEF_X + 8].copy_from_slice(&target_ecef_xyz_mm.0.to_be_bytes());
    frame[O_ECEF_Y..O_ECEF_Y + 8].copy_from_slice(&target_ecef_xyz_mm.1.to_be_bytes());
    frame[O_ECEF_Z..O_ECEF_Z + 8].copy_from_slice(&target_ecef_xyz_mm.2.to_be_bytes());

    // Internal consistency: when Quality≠0 the chip never emits invalid
    // markers (`0x80000000` / `0xFFFFFFFF`) for velocity / DOP / accuracy
    // fields — so drone-side sanity checks reject "fix valid but kinematics
    // invalid". Fill plausible stationary values.
    if valid {
        // Stationary: all velocities = 0, speed = 0, heading = 0.
        let z32 = 0i32.to_be_bytes();
        frame[O_VEL_E..O_VEL_E + 4].copy_from_slice(&z32);
        frame[O_VEL_N..O_VEL_N + 4].copy_from_slice(&z32);
        frame[O_VEL_U..O_VEL_U + 4].copy_from_slice(&z32);
        frame[O_SPEED..O_SPEED + 4].copy_from_slice(&z32);
        frame[O_HEADING..O_HEADING + 2].copy_from_slice(&0u16.to_be_bytes());
        // DOPs: HDOP=1.0, VDOP=1.5, PDOP=2.0, GDOP=2.0, TDOP=1.0 (×0.01).
        frame[O_HDOP..O_HDOP + 2].copy_from_slice(&100u16.to_be_bytes());
        frame[O_VDOP..O_VDOP + 2].copy_from_slice(&150u16.to_be_bytes());
        frame[O_PDOP..O_PDOP + 2].copy_from_slice(&200u16.to_be_bytes());
        frame[O_GDOP..O_GDOP + 2].copy_from_slice(&200u16.to_be_bytes());
        frame[O_TDOP..O_TDOP + 2].copy_from_slice(&100u16.to_be_bytes());
        // Accuracies (mean square error): position ~5 m = 5000 mm; velocity
        // ~1 m/s = 1000 mm/s; time ~50 ns.
        let pos_acc = 5000u32.to_be_bytes();
        frame[O_EACC..O_EACC + 4].copy_from_slice(&pos_acc);
        frame[O_NACC..O_NACC + 4].copy_from_slice(&pos_acc);
        frame[O_UACC..O_UACC + 4].copy_from_slice(&pos_acc);
        frame[O_TACC..O_TACC + 4].copy_from_slice(&50u32.to_be_bytes());
        frame[O_XACC..O_XACC + 4].copy_from_slice(&pos_acc);
        frame[O_YACC..O_YACC + 4].copy_from_slice(&pos_acc);
        frame[O_ZACC..O_ZACC + 4].copy_from_slice(&pos_acc);
        let vel_acc = 1000u32.to_be_bytes();
        frame[O_VEACC..O_VEACC + 4].copy_from_slice(&vel_acc);
        frame[O_VNACC..O_VNACC + 4].copy_from_slice(&vel_acc);
        frame[O_VUACC..O_VUACC + 4].copy_from_slice(&vel_acc);
    }
    // Note: for `valid=false` we deliberately leave velocities/DOPs/accuracies
    // at the chip's invalid markers — that's consistent with Quality=0.

    // Recompute CRC24Q over header + payload (everything except trailing CRC).
    let crc_off = rtcm3::HEADER_LEN + payload_len as usize;
    let crc = rtcm3::crc24_q(&frame[..crc_off]);
    frame[crc_off] = ((crc >> 16) & 0xFF) as u8;
    frame[crc_off + 1] = ((crc >> 8) & 0xFF) as u8;
    frame[crc_off + 2] = (crc & 0xFF) as u8;
    Some(())
}

// ---------------------------------------------------------------------------
// Data-field samples (byte-exact captures from real chip)
// ---------------------------------------------------------------------------

/// Sub 0x000 — unknown (not in public R1.2). 2-byte data field from log.
pub const DATA_SUB_000: &[u8] = &[0xA8, 0x5A];

/// Sub 0x001 — unknown (3-byte RTCM payload total; no data bytes).
pub const DATA_SUB_001: &[u8] = &[];

/// Sub 0x002 — unknown. 45-byte data field from log.
pub const DATA_SUB_002: &[u8] = &[
    0xA8, 0x5A, 0x00, 0x00, 0x01, 0x49, 0x00, 0xC8, 0x00, 0x01, 0x01, 0x08,
    0x00, 0x09, 0x00, 0x01, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

/// Sub 0x0E6 — Hardware Status (doc §5.2.2.9). 28-byte data field.
pub const DATA_SUB_0E6: &[u8] = &[
    0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x02, 0x00, 0x3E, 0x01,
    0x55, 0x04, 0x00, 0x2C, 0xC6, 0x01, 0x21, 0xD6, 0x02, 0x24, 0x74, 0x03,
    0x8A, 0x39, 0x23, 0x00,
];

/// Sub 0x0E9 — Jamming & Spoofing Detection (doc §5.2.2.7). 15-byte data field.
pub const DATA_SUB_0E9: &[u8] = &[
    0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x01, 0x01, 0x01, 0x00,
    0x00, 0x00, 0x00,
];

/// Sub 0x0F9 — Protection Level Information (doc §5.2.2.5). 54-byte data field.
pub const DATA_SUB_0F9: &[u8] = &[
    0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
    0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
];

/// Sub 0x0FC — unknown (likely Protocol/Firmware Info). 130-byte data field.
pub const DATA_SUB_0FC: &[u8] = &[
    0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x36,
    0xFC, 0x5E, 0x2C, 0xCD, 0x8A, 0x41, 0x0F, 0xC6, 0xD8, 0xD1, 0x44, 0x99,
    0x0D, 0xA9, 0xE0, 0xFC, 0x22, 0x4F, 0x8C, 0x61, 0xAB, 0xF8, 0xBB, 0x6F,
    0x7C, 0xCA, 0xF6, 0xC1, 0x9E, 0xA0, 0x24, 0x0B, 0xC4, 0x6D, 0x31, 0x94,
    0x57, 0x4B, 0xD4, 0xA9, 0x49, 0x78, 0xE2, 0x02, 0x9D, 0x55, 0x94, 0x57,
    0xAB, 0xCA, 0x2D, 0x9B, 0xDB, 0xBB, 0x62, 0x4A, 0xC6, 0xCA, 0x99, 0xD3,
    0x37, 0x9D, 0x3C, 0xF2, 0x71, 0x14, 0x29, 0x21, 0xE4, 0x19, 0xF7, 0x6E,
    0x8B, 0xC1, 0x16, 0x19, 0xE7, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

/// Sub 0x0FE — Signal Information (doc §5.2.2.2). 11-byte data field.
pub const DATA_SUB_0FE: &[u8] = &[
    0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
];

/// Sub 0x0FF — Receiver Information (doc §5.2.2.1). 160-byte data field.
pub const DATA_SUB_0FF: &[u8] = &[
    0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
    0x72, 0x00, 0x00, 0x00,
];

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Reference full frames captured from the real chip. Each must match
    // byte-for-byte what `build_sub` produces when fed the corresponding
    // DATA_SUB_* constant.

    const REF_000: &[u8] = &[
        0xD3, 0x00, 0x05, 0xFE, 0xA0, 0x00, 0xA8, 0x5A, 0xC2, 0x2E, 0x06,
    ];
    const REF_001: &[u8] = &[
        0xD3, 0x00, 0x03, 0xFE, 0xA0, 0x01, 0xFA, 0x8D, 0x7A,
    ];
    const REF_002: &[u8] = &[
        0xD3, 0x00, 0x30, 0xFE, 0xA0, 0x02, 0xA8, 0x5A, 0x00, 0x00, 0x01, 0x49,
        0x00, 0xC8, 0x00, 0x01, 0x01, 0x08, 0x00, 0x09, 0x00, 0x01, 0x01, 0x08,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x2A, 0xFD, 0x09,
    ];
    const REF_0E6: &[u8] = &[
        0xD3, 0x00, 0x1F, 0xFE, 0xA0, 0xE6, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0x00, 0x02, 0x00, 0x3E, 0x01, 0x55, 0x04, 0x00, 0x2C, 0xC6, 0x01,
        0x21, 0xD6, 0x02, 0x24, 0x74, 0x03, 0x8A, 0x39, 0x23, 0x00, 0xD1, 0x53,
        0xC4,
    ];
    const REF_0E9: &[u8] = &[
        0xD3, 0x00, 0x12, 0xFE, 0xA0, 0xE9, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xE9, 0x3F, 0x2A,
    ];
    const REF_0F9: &[u8] = &[
        0xD3, 0x00, 0x39, 0xFE, 0xA0, 0xF9, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x54, 0x87,
    ];
    const REF_0FC: &[u8] = &[
        0xD3, 0x00, 0x85, 0xFE, 0xA0, 0xFC, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xBF, 0x36, 0xFC, 0x5E, 0x2C, 0xCD, 0x8A, 0x41,
        0x0F, 0xC6, 0xD8, 0xD1, 0x44, 0x99, 0x0D, 0xA9, 0xE0, 0xFC, 0x22, 0x4F,
        0x8C, 0x61, 0xAB, 0xF8, 0xBB, 0x6F, 0x7C, 0xCA, 0xF6, 0xC1, 0x9E, 0xA0,
        0x24, 0x0B, 0xC4, 0x6D, 0x31, 0x94, 0x57, 0x4B, 0xD4, 0xA9, 0x49, 0x78,
        0xE2, 0x02, 0x9D, 0x55, 0x94, 0x57, 0xAB, 0xCA, 0x2D, 0x9B, 0xDB, 0xBB,
        0x62, 0x4A, 0xC6, 0xCA, 0x99, 0xD3, 0x37, 0x9D, 0x3C, 0xF2, 0x71, 0x14,
        0x29, 0x21, 0xE4, 0x19, 0xF7, 0x6E, 0x8B, 0xC1, 0x16, 0x19, 0xE7, 0x92,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x0B, 0xAF, 0x90,
    ];
    const REF_0FE: &[u8] = &[
        0xD3, 0x00, 0x0E, 0xFE, 0xA0, 0xFE, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x91, 0x3A,
    ];
    const REF_0FF: &[u8] = &[
        0xD3, 0x00, 0xA3, 0xFE, 0xA0, 0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00,
        0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
        0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x72, 0x00, 0x00, 0x00, 0xAD, 0xFB,
        0x9E,
    ];

    fn bit_exact(sub_id: u16, data: &[u8], reference: &[u8]) {
        let mut buf = [0u8; 256];
        let n = build_sub(&mut buf, sub_id, data);
        assert_eq!(n, reference.len(), "sub 0x{sub_id:03X} length");
        assert_eq!(&buf[..n], reference, "sub 0x{sub_id:03X} bit-exact");
        assert_eq!(rtcm3::verify_frame(&buf[..n]), Some(n));
        assert_eq!(rtcm3::message_type(&buf[..n]), Some(MSG_TYPE));
    }

    #[test] fn sub_000() { bit_exact(0x000, DATA_SUB_000, REF_000); }
    #[test] fn sub_001() { bit_exact(0x001, DATA_SUB_001, REF_001); }
    #[test] fn sub_002() { bit_exact(0x002, DATA_SUB_002, REF_002); }
    #[test] fn sub_0e6() { bit_exact(0x0E6, DATA_SUB_0E6, REF_0E6); }
    #[test] fn sub_0e9() { bit_exact(0x0E9, DATA_SUB_0E9, REF_0E9); }
    #[test] fn sub_0f9() { bit_exact(0x0F9, DATA_SUB_0F9, REF_0F9); }
    #[test] fn sub_0fc() { bit_exact(0x0FC, DATA_SUB_0FC, REF_0FC); }
    #[test] fn sub_0fe() { bit_exact(0x0FE, DATA_SUB_0FE, REF_0FE); }
    #[test] fn sub_0ff() { bit_exact(0x0FF, DATA_SUB_0FF, REF_0FF); }

    #[test]
    fn build_sub_rejects_overflow() {
        let mut small = [0u8; 10];
        assert_eq!(build_sub(&mut small, 0x0FF, DATA_SUB_0FF), 0);
    }

    #[test]
    fn modify_receiver_info_valid_writes_target_and_quality_1() {
        // Start from the chip's no-signal capture (Quality=0, all coord
        // fields = invalid markers).
        let mut frame = [0u8; RECEIVER_INFO_FRAME_LEN];
        let n = build_sub(&mut frame, SUB_RECEIVER_INFO, DATA_SUB_0FF);
        assert_eq!(n, RECEIVER_INFO_FRAME_LEN);

        let lat_1e7: i32 = 25_966_443; // 25.9664430° N
        let lon_1e7: i32 = -801_223_710_i32; // approx -80.122 W
        let alt_mm: i32 = 50_000;
        let ecef_mm = (5_000_000_000i64, -1_000_000_000i64, 2_700_000_000i64);

        modify_receiver_info(&mut frame, true, lat_1e7, lon_1e7, alt_mm, ecef_mm)
            .expect("modify ok");
        // CRC must verify.
        assert_eq!(rtcm3::verify_frame(&frame), Some(RECEIVER_INFO_FRAME_LEN));
        // Read back fields and confirm.
        const D: usize = 6;
        assert_eq!(frame[D + 7], 16, "SatNum");
        assert_eq!(frame[D + 56], 1, "Quality");
        let lon_be = i64::from_be_bytes(frame[D + 8..D + 16].try_into().unwrap());
        let lat_be = i64::from_be_bytes(frame[D + 16..D + 24].try_into().unwrap());
        assert_eq!(lon_be, llh_e7_to_2pn32(lon_1e7));
        assert_eq!(lat_be, llh_e7_to_2pn32(lat_1e7));
        let hae = i32::from_be_bytes(frame[D + 24..D + 28].try_into().unwrap());
        let hmsl = i32::from_be_bytes(frame[D + 28..D + 32].try_into().unwrap());
        assert_eq!(hae, alt_mm);
        assert_eq!(hmsl, alt_mm);
        assert_eq!(
            i64::from_be_bytes(frame[D + 32..D + 40].try_into().unwrap()),
            ecef_mm.0
        );
        assert_eq!(
            i64::from_be_bytes(frame[D + 40..D + 48].try_into().unwrap()),
            ecef_mm.1
        );
        assert_eq!(
            i64::from_be_bytes(frame[D + 48..D + 56].try_into().unwrap()),
            ecef_mm.2
        );
    }

    #[test]
    fn modify_receiver_info_invalid_keeps_target_coords_quality_0() {
        let mut frame = [0u8; RECEIVER_INFO_FRAME_LEN];
        build_sub(&mut frame, SUB_RECEIVER_INFO, DATA_SUB_0FF);
        let lat_1e7 = 25_966_443_i32;
        let lon_1e7 = -801_223_710_i32;
        let alt_mm = 50_000_i32;
        let ecef = (1i64, 2i64, 3i64);

        modify_receiver_info(&mut frame, false, lat_1e7, lon_1e7, alt_mm, ecef).expect("ok");
        assert_eq!(rtcm3::verify_frame(&frame), Some(RECEIVER_INFO_FRAME_LEN));
        const D: usize = 6;
        assert_eq!(frame[D + 7], 1, "SatNum=1 under invalid");
        assert_eq!(frame[D + 56], 0, "Quality=0 under invalid");
        // Coords still target.
        let lat_be = i64::from_be_bytes(frame[D + 16..D + 24].try_into().unwrap());
        assert_eq!(lat_be, llh_e7_to_2pn32(lat_1e7));
    }

    #[test]
    fn modify_receiver_info_rejects_wrong_sub_id() {
        // Build a 4074-0xFE frame and try to modify it as 0xFF.
        let mut frame = [0u8; 256];
        let n = build_sub(&mut frame, 0x0FE, DATA_SUB_0FE);
        // Pad to RECEIVER_INFO_FRAME_LEN — modify_receiver_info checks length first.
        // So we expect None either due to length OR (if same length) sub_id mismatch.
        let _ = n;
        // Use the actual 0x0FE frame length (which is shorter than 169) — modify
        // should reject on the length precondition.
        let mut short = vec![0u8; RECEIVER_INFO_FRAME_LEN];
        short[..n].copy_from_slice(&frame[..n]);
        // Force length check by truncating to actual 0xFE size:
        assert!(modify_receiver_info(
            &mut short[..n].to_vec(),
            true,
            0,
            0,
            0,
            (0, 0, 0),
        )
        .is_none());
    }

    #[test]
    fn llh_e7_to_2pn32_roundtrip_keeps_sign_and_magnitude() {
        // Equator + prime meridian = 0
        assert_eq!(llh_e7_to_2pn32(0), 0);
        // 90 deg = 9e8 in 1e-7
        let n90 = llh_e7_to_2pn32(900_000_000);
        assert_eq!(n90, (1i64 << 32) * 90);
        let neg = llh_e7_to_2pn32(-900_000_000);
        assert_eq!(neg, -(1i64 << 32) * 90);
        // 180 deg lon
        let l180 = llh_e7_to_2pn32(1_800_000_000);
        assert_eq!(l180, (1i64 << 32) * 180);
    }
}
