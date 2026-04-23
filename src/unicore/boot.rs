//! Static boot-dump replay.
//!
//! `BOOT_DUMP` is the first ~2.3 KB that the real UC6580I emits on UART1
//! between power-on and the start of the steady NMEA stream. It contains
//! `$PDTINFO`, `Reset Count`, `Init time`, `$$$_PVT_INIT_INFO_START_`,
//! `$CHCFG`, `$PTINFOPKG`, `$BDSUTC`, `$GALUTC`, `$SVEPH_*` messages and a
//! short burst of RTCM 4074 status packets.
//!
//! Captured from `uart_term/target/release/log_2026-04-22_17-27-43.txt` on
//! 2026-04-22. We replay the bytes verbatim — drones that look for any of
//! these boot markers see the exact sequence a production chip emits.

#![allow(dead_code)]

/// Raw boot-dump bytes (2286 bytes).
pub const BOOT_DUMP: &[u8] = include_bytes!("boot_dump.bin");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boot_dump_nonempty() {
        assert!(BOOT_DUMP.len() > 2000);
        assert!(BOOT_DUMP.len() < 4000);
    }

    #[test]
    fn boot_dump_contains_fw_version_marker() {
        // Real chip embeds "1.1Build9074" tail of $PDTINFO inside the very
        // first bytes of the dump.
        let needle = b"1.1Build9074";
        assert!(BOOT_DUMP.windows(needle.len()).any(|w| w == needle));
    }

    #[test]
    fn boot_dump_contains_reset_count() {
        let needle = b"Reset Count:0";
        assert!(BOOT_DUMP.windows(needle.len()).any(|w| w == needle));
    }
}
