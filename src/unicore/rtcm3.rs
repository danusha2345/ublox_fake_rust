//! RTCM v3 frame utilities — header + CRC24-Q only.
//!
//! This module intentionally does not implement any specific RTCM message type.
//! For UC6580I emulation we emit RTCM frames (standard 1077 and proprietary
//! 4074) by wrapping a pre-built payload into a `0xD3 + length + payload +
//! CRC24` envelope. Parsing is limited to identifying frame boundaries and
//! the 12-bit message type so the frame router can dispatch incoming data.
//!
//! Reference: RTCM 10403.3 §3.1 (frame format) and docs/UC6580I.md §8.

#![allow(dead_code)]

/// RTCM frame preamble byte (always `0xD3`).
pub const PREAMBLE: u8 = 0xD3;

/// Three fixed-size bytes at the start of every RTCM frame:
/// `[0xD3][reserved(6)|len_hi(2)][len_lo(8)]`. Payload length is 10 bits so
/// the maximum payload is 1023 bytes.
pub const HEADER_LEN: usize = 3;

/// CRC24-Q appended at the end of every frame.
pub const CRC_LEN: usize = 3;

/// Maximum RTCM frame payload length (10-bit field).
pub const MAX_PAYLOAD: usize = 1023;

// ---------------------------------------------------------------------------
// CRC24-Q (RTCM polynomial 0x1864CFB, init = 0, no reflection, no xor-out)
// ---------------------------------------------------------------------------

/// Compute CRC24-Q over `data`. Returns a 24-bit value in the low bits.
pub fn crc24_q(data: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &b in data {
        crc ^= (b as u32) << 16;
        for _ in 0..8 {
            crc <<= 1;
            if crc & 0x0100_0000 != 0 {
                crc ^= 0x0186_4CFB;
            }
        }
    }
    crc & 0x00FF_FFFF
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

/// Write an RTCM frame into `buf`: preamble + length + payload + CRC24-Q.
/// Returns the total byte count, or 0 on overflow / invalid payload length.
pub fn build_frame(buf: &mut [u8], payload: &[u8]) -> usize {
    if payload.len() > MAX_PAYLOAD {
        return 0;
    }
    let total = HEADER_LEN + payload.len() + CRC_LEN;
    if buf.len() < total {
        return 0;
    }
    buf[0] = PREAMBLE;
    buf[1] = ((payload.len() >> 8) as u8) & 0x03; // top 2 bits of 10-bit length
    buf[2] = (payload.len() & 0xFF) as u8;
    buf[3..3 + payload.len()].copy_from_slice(payload);
    let crc = crc24_q(&buf[..HEADER_LEN + payload.len()]);
    buf[HEADER_LEN + payload.len()] = ((crc >> 16) & 0xFF) as u8;
    buf[HEADER_LEN + payload.len() + 1] = ((crc >> 8) & 0xFF) as u8;
    buf[HEADER_LEN + payload.len() + 2] = (crc & 0xFF) as u8;
    total
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/// Decoded RTCM frame header: payload length + total frame length.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameHeader {
    pub payload_len: u16,
    pub total_len: u16,
}

/// Read the first 3 bytes as an RTCM header.
/// Returns `None` if the preamble is wrong or the top 6 "reserved" bits are
/// non-zero (anything other than 0 is either noise or a future extension).
pub fn parse_header(bytes: &[u8]) -> Option<FrameHeader> {
    if bytes.len() < HEADER_LEN {
        return None;
    }
    if bytes[0] != PREAMBLE {
        return None;
    }
    // RTCM fixes the top 6 bits of byte 1 to zero. Any other value is not
    // a well-formed frame.
    if bytes[1] & 0xFC != 0 {
        return None;
    }
    let payload_len = ((bytes[1] as u16 & 0x03) << 8) | bytes[2] as u16;
    Some(FrameHeader {
        payload_len,
        total_len: (HEADER_LEN + payload_len as usize + CRC_LEN) as u16,
    })
}

/// If `bytes` begins with a complete, CRC-valid RTCM frame, return its length.
pub fn verify_frame(bytes: &[u8]) -> Option<usize> {
    let h = parse_header(bytes)?;
    let total = h.total_len as usize;
    if bytes.len() < total {
        return None;
    }
    let body_end = HEADER_LEN + h.payload_len as usize;
    let expected = crc24_q(&bytes[..body_end]);
    let actual = ((bytes[body_end] as u32) << 16)
        | ((bytes[body_end + 1] as u32) << 8)
        | (bytes[body_end + 2] as u32);
    if actual == expected { Some(total) } else { None }
}

/// Read the RTCM message type from the first 12 bits of the payload.
pub fn message_type(bytes: &[u8]) -> Option<u16> {
    let h = parse_header(bytes)?;
    if (h.payload_len as usize) < 2 || bytes.len() < HEADER_LEN + 2 {
        return None;
    }
    let hi = bytes[HEADER_LEN] as u16;
    let lo = bytes[HEADER_LEN + 1] as u16;
    Some(((hi << 4) | (lo >> 4)) & 0x0FFF)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference CRC24-Q result for RTCM frame `[0xD3, 0x00, 0x03, 0x43, 0x50, 0x00]`
    /// is `0x8D44BE` — confirmed from real UC6580I log at byte offset of a
    /// short 3-byte payload frame.
    #[test]
    fn crc24_known_vector() {
        let v = [0xD3, 0x00, 0x03, 0x43, 0x50, 0x00];
        assert_eq!(crc24_q(&v), 0x8D_44_BE);
    }

    #[test]
    fn crc24_empty_is_zero() {
        assert_eq!(crc24_q(&[]), 0);
    }

    #[test]
    fn build_and_verify_roundtrip() {
        let payload = [0x43, 0x50, 0x00];
        let mut buf = [0u8; 16];
        let n = build_frame(&mut buf, &payload);
        assert_eq!(n, 9);
        // Must begin with D3 00 03 and end with 8D 44 BE
        assert_eq!(&buf[..n], &[0xD3, 0x00, 0x03, 0x43, 0x50, 0x00, 0x8D, 0x44, 0xBE]);
        assert_eq!(verify_frame(&buf[..n]), Some(9));
    }

    #[test]
    fn verify_rejects_bad_crc() {
        let mut frame = [0xD3, 0x00, 0x03, 0x43, 0x50, 0x00, 0x8D, 0x44, 0xBF];
        assert!(verify_frame(&frame).is_none());
        frame[8] = 0xBE;
        assert_eq!(verify_frame(&frame), Some(9));
    }

    #[test]
    fn parse_header_rejects_wrong_preamble() {
        assert!(parse_header(&[0xD0, 0x00, 0x03]).is_none());
        assert!(parse_header(&[0xD3, 0xFC, 0x03]).is_none());
        assert_eq!(
            parse_header(&[0xD3, 0x03, 0xFF]),
            Some(FrameHeader { payload_len: 1023, total_len: 1029 })
        );
    }

    #[test]
    fn message_type_from_first_12_bits() {
        // Payload starts with 0x43 0x50 -> 12 bits = 0x435 (decimal 1077)
        let frame = [0xD3, 0x00, 0x03, 0x43, 0x50, 0x00, 0x8D, 0x44, 0xBE];
        assert_eq!(message_type(&frame), Some(1077));
    }

    #[test]
    fn build_frame_respects_max_payload() {
        let payload = [0u8; MAX_PAYLOAD];
        let mut buf = [0u8; MAX_PAYLOAD + 6];
        assert!(build_frame(&mut buf, &payload) > 0);

        let too_big = [0u8; MAX_PAYLOAD + 1];
        let mut buf = [0u8; MAX_PAYLOAD + 10];
        assert_eq!(build_frame(&mut buf, &too_big), 0);
    }
}
