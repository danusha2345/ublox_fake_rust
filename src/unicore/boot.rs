//! Static boot-dump replay.
//!
//! `BOOT_DUMP` is the first ~6.7 KB that the real UC6580I emits on UART1
//! between power-on and the start of the steady NMEA stream. It contains:
//! - Identification block (`UC6580I-00 …`, `PN`, `SN`, `HWVer`, `FWVer`,
//!   `Reset Count`, `Init time`).
//! - `$$$_PVT_INIT_INFO_START_$$$` marker.
//! - Static config: `$RECVCFG,120`, `$PECFG,22`, `$CHCFG`, `$PTINFOPKG,34`.
//! - UTC tables: `$GPSUTC`, `$BDSUTC`, `$GALUTC`, `$GLOUTC`, `$IRNUTC`.
//! - Iono tables: `$GPSIONO`, `$BDSIONO`, `$GALIONO`.
//! - 20 ephemeris summaries (`$SVEPH_003 … $SVEPH_131`).
//! - First steady-state burst: empty RTCM 1077 + ExtRTCM 4074 subs
//!   0xFF / 0xFE / 0xF9 / 0xE9 / 0xE6.
//!
//! Captured from `logs_analiser/2026-04-26_165533_neo_hex.txt` on 2026-04-26
//! (Neo drone power-on with live UC6580I), extracted via
//! `.scratch/extract_boot_dump.py`. Stream is cut just before the first
//! `$GNRMC` of the periodic 5 Hz cycle.

#![allow(dead_code)]

/// Raw boot-dump bytes (~6.7 KB).
pub const BOOT_DUMP: &[u8] = include_bytes!("boot_dump.bin");

#[cfg(test)]
mod tests {
    use super::*;

    fn contains(needle: &[u8]) -> bool {
        BOOT_DUMP.windows(needle.len()).any(|w| w == needle)
    }

    #[test]
    fn boot_dump_size_reasonable() {
        // Live capture is ~6.7 KB. Be tolerant — tighten to 6000..8000.
        assert!(BOOT_DUMP.len() > 6000, "len={}", BOOT_DUMP.len());
        assert!(BOOT_DUMP.len() < 8000, "len={}", BOOT_DUMP.len());
    }

    #[test]
    fn boot_dump_starts_with_clean_header() {
        // Live chip emits an empty CRLF, then the identification banner.
        assert!(BOOT_DUMP.starts_with(b"\r\nUC6580I-00 G1B1L1E1 COM1\r\n"));
    }

    #[test]
    fn boot_dump_contains_identification_block() {
        assert!(contains(b"PN 2400615000514\r\n"));
        assert!(contains(b"SN 20250427080308004\r\n"));
        assert!(contains(b"HWVer 00\r\n"));
        assert!(contains(b"FWVer R6.1.1.1Build9074\r\n"));
        assert!(contains(b"Reset Count:0\r\n"));
        assert!(contains(b"Init time:"));
    }

    #[test]
    fn boot_dump_contains_pvt_init_marker() {
        assert!(contains(b"$$$_PVT_INIT_INFO_START_$$$\r\n"));
    }

    #[test]
    fn boot_dump_contains_static_config_blocks() {
        assert!(contains(b"$RECVCFG,120,\r\n"));
        assert!(contains(b"$PECFG,22,\r\n"));
        assert!(contains(b"$CHCFG,"));
        assert!(contains(b"$PTINFOPKG,34,\r\n"));
    }

    #[test]
    fn boot_dump_contains_all_utc_tables() {
        assert!(contains(b"$GPSUTC,10,"));
        assert!(contains(b"$BDSUTC,10,"));
        assert!(contains(b"$GALUTC,10,"));
        assert!(contains(b"$GLOUTC,10,"));
        assert!(contains(b"$IRNUTC,10,"));
    }

    #[test]
    fn boot_dump_contains_all_iono_tables() {
        assert!(contains(b"$GPSIONO,14,"));
        assert!(contains(b"$BDSIONO,14,"));
        assert!(contains(b"$GALIONO,5,"));
    }

    #[test]
    fn boot_dump_contains_sveph_burst() {
        // First and last SV in the captured boot burst.
        assert!(contains(b"$SVEPH_003,17,"));
        assert!(contains(b"$SVEPH_131,18,"));
    }

    #[test]
    fn boot_dump_contains_no_steady_state_rmc() {
        // Boundary check: cut should fall before the periodic 5 Hz stream.
        assert!(!contains(b"$GNRMC"));
        assert!(!contains(b"$GNGGA"));
        assert!(!contains(b"$PNOISE"));
    }
}
