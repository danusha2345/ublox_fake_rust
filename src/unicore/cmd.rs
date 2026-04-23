//! ASCII command parser + reply builder for Unicore UC6580I emulation.
//!
//! Commands arrive as `$<CMD>[,<arg>,...][*HH]<CR><LF>`. The XOR checksum is
//! optional — the real chip accepts commands without it. Bad checksums reply
//! `$FAIL,1*cs`; malformed parameters reply `$FAIL,0*cs`.
//!
//! Replies we emit match §5.2 and §5.8 of docs/UC6580I.md.

#![allow(dead_code)]

use super::nmea;

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/// Parsed command. `&'a [u8]` ties back to the input buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command<'a> {
    PdtInfo,
    ProductInfo,
    /// Get: `Some(args=empty)`. Set: `Some(sys_mask)`.
    CfgSys(Option<u32>),
    CfgMsg {
        class: u8,
        id: u8,
        rate: Option<u8>,
    },
    CfgNav {
        meas_rate: u32,
        nav_rate: u32,
        dr_nav_rate: u32,
    },
    CfgKey,
    CfgSave,
    CfgClr,
    Unknown(&'a [u8]),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// No leading `$`.
    NoDollarPrefix,
    /// Checksum present but does not match the body XOR.
    BadChecksum,
    /// Known command but numeric argument could not be parsed.
    Malformed,
}

/// Parse a NMEA-style command line. `line` may include trailing `\r\n`.
/// Checksum (`*HH`) is optional.
pub fn parse_command(line: &[u8]) -> Result<Command<'_>, ParseError> {
    // Strip trailing CR/LF.
    let mut end = line.len();
    while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
        end -= 1;
    }
    if end == 0 || line[0] != b'$' {
        return Err(ParseError::NoDollarPrefix);
    }
    let body = &line[1..end];
    // Split off *HH if present.
    let (fields_slice, has_cs) = match body.iter().rposition(|&b| b == b'*') {
        Some(star) => {
            if body.len() < star + 3 {
                return Err(ParseError::Malformed);
            }
            if !nmea::verify_sentence(line) {
                return Err(ParseError::BadChecksum);
            }
            (&body[..star], true)
        }
        None => (body, false),
    };
    let _ = has_cs;

    // Split by comma. Field 0 is the command name.
    let mut fields: [&[u8]; 16] = [&[][..]; 16];
    let mut count = 0;
    let mut last = 0;
    for i in 0..fields_slice.len() {
        if fields_slice[i] == b',' {
            if count < fields.len() {
                fields[count] = &fields_slice[last..i];
                count += 1;
            }
            last = i + 1;
        }
    }
    if count < fields.len() {
        fields[count] = &fields_slice[last..];
        count += 1;
    }
    let name = fields[0];
    let args = &fields[1..count];

    match name {
        b"PDTINFO" => Ok(Command::PdtInfo),
        b"PRODUCTINFO" => Ok(Command::ProductInfo),
        b"CFGSYS" => match args.len() {
            0 => Ok(Command::CfgSys(None)),
            1 => {
                let mask = parse_hex_prefixed(args[0]).ok_or(ParseError::Malformed)?;
                Ok(Command::CfgSys(Some(mask)))
            }
            _ => Err(ParseError::Malformed),
        },
        b"CFGMSG" => {
            if args.len() < 2 || args.len() > 3 {
                return Err(ParseError::Malformed);
            }
            let class = parse_dec_u8(args[0]).ok_or(ParseError::Malformed)?;
            let id = parse_dec_u8(args[1]).ok_or(ParseError::Malformed)?;
            let rate = if args.len() == 3 {
                Some(parse_dec_u8(args[2]).ok_or(ParseError::Malformed)?)
            } else {
                None
            };
            Ok(Command::CfgMsg { class, id, rate })
        }
        b"CFGNAV" => match args.len() {
            0 => Ok(Command::CfgNav { meas_rate: 0, nav_rate: 0, dr_nav_rate: 0 }),
            3 => {
                let a = parse_dec_u32(args[0]).ok_or(ParseError::Malformed)?;
                let b = parse_dec_u32(args[1]).ok_or(ParseError::Malformed)?;
                let c = parse_dec_u32(args[2]).ok_or(ParseError::Malformed)?;
                Ok(Command::CfgNav { meas_rate: a, nav_rate: b, dr_nav_rate: c })
            }
            _ => Err(ParseError::Malformed),
        },
        b"CFGKEY" => Ok(Command::CfgKey),
        b"CFGSAVE" => Ok(Command::CfgSave),
        b"CFGCLR" => Ok(Command::CfgClr),
        _ => Ok(Command::Unknown(name)),
    }
}

fn parse_dec_u8(s: &[u8]) -> Option<u8> {
    let v = parse_dec_u32(s)?;
    if v > 255 { None } else { Some(v as u8) }
}

fn parse_dec_u32(s: &[u8]) -> Option<u32> {
    if s.is_empty() {
        return None;
    }
    let mut v: u32 = 0;
    for &c in s {
        if !(b'0'..=b'9').contains(&c) {
            return None;
        }
        v = v.checked_mul(10)?.checked_add((c - b'0') as u32)?;
    }
    Some(v)
}

/// Parse `h<hex>` or `H<hex>` (Unicore convention for hex values).
fn parse_hex_prefixed(s: &[u8]) -> Option<u32> {
    if s.is_empty() || (s[0] != b'h' && s[0] != b'H') {
        return None;
    }
    let mut v: u32 = 0;
    for &c in &s[1..] {
        let d = match c {
            b'0'..=b'9' => c - b'0',
            b'a'..=b'f' => c - b'a' + 10,
            b'A'..=b'F' => c - b'A' + 10,
            _ => return None,
        };
        v = v.checked_mul(16)?.checked_add(d as u32)?;
    }
    Some(v)
}

// ---------------------------------------------------------------------------
// Reply builders
// ---------------------------------------------------------------------------

/// `$PDTINFO,UC6580I-00,G1B1L1E1,V00,R6.1.1.1Build9074,2400615000514,20250427080308004*4D\r\n`.
pub const PDTINFO_REPLY: &[u8] =
    b"$PDTINFO,UC6580I-00,G1B1L1E1,V00,R6.1.1.1Build9074,2400615000514,20250427080308004*4D\r\n";

/// `$CFGSYS,H55155*4E\r\n` — default mask for our chip
/// (L1+L5 dual-band across GPS/BDS/GLO/GAL/QZSS).
pub const CFGSYS_DEFAULT_REPLY: &[u8] = b"$CFGSYS,H55155*4E\r\n";

/// `$CFGKEY,...` — factory key, byte-exact from live chip probe 2026-04-23.
pub const CFGKEY_REPLY: &[u8] =
    b"$CFGKEY,,6adae970f433a1c9c55f3bd7092b1400f40dddc4c715c39a5aed529158f0010101010101010101010101010101010101,D0645C*3B\r\n";

/// `$PRODUCTINFO,...*cs\r\n` — extended identity reply. Adds protocol
/// version (PTV=1.1) + 4 reserved fields after the $PDTINFO payload.
pub const PRODUCTINFO_REPLY: &[u8] =
    b"$PRODUCTINFO,UC6580I-00,G1B1L1E1,V00,R6.1.1.1Build9074,2400615000514,20250427080308004,1.1,,,,*44\r\n";

/// `$CFGNAV,200,200,0*07\r\n` — navigation rate reply (5 Hz, no DR).
pub const CFGNAV_DEFAULT_REPLY: &[u8] = b"$CFGNAV,200,200,0*07\r\n";

/// `$GNTXT,01,01,00,CFGSYS*4A\r\n` — pre-ACK emitted by the real chip
/// *before* the `$CFGSYS` value on a GET request.
pub const CFGSYS_PREACK: &[u8] = b"$GNTXT,01,01,00,CFGSYS*4A\r\n";

/// `$GNTXT,01,01,00,CFGNAV*4A\r\n` — pre-ACK for `$CFGNAV` GET.
pub const CFGNAV_PREACK: &[u8] = b"$GNTXT,01,01,00,CFGNAV*4A\r\n";

/// Generic `$OK*04\r\n` success reply.
pub const OK_REPLY: &[u8] = b"$OK*04\r\n";

/// `$FAIL,0*cs` — invalid parameters. Precomputed.
pub const FAIL_PARAMS: &[u8] = b"$FAIL,0*1E\r\n";

/// `$FAIL,1*cs` — bad checksum. Precomputed.
pub const FAIL_CHECKSUM: &[u8] = b"$FAIL,1*1F\r\n";

/// Copy `src` into `buf`, returning bytes written or 0 on overflow.
fn copy_to(buf: &mut [u8], src: &[u8]) -> usize {
    if buf.len() < src.len() {
        return 0;
    }
    buf[..src.len()].copy_from_slice(src);
    src.len()
}

/// Emit the canned PDTINFO reply.
pub fn reply_pdtinfo(buf: &mut [u8]) -> usize {
    copy_to(buf, PDTINFO_REPLY)
}

/// Emit the canned CFGSYS reply (default mask).
pub fn reply_cfgsys_default(buf: &mut [u8]) -> usize {
    copy_to(buf, CFGSYS_DEFAULT_REPLY)
}

/// Emit the canned CFGKEY reply.
pub fn reply_cfgkey(buf: &mut [u8]) -> usize {
    copy_to(buf, CFGKEY_REPLY)
}

/// Emit the canned PRODUCTINFO reply.
pub fn reply_productinfo(buf: &mut [u8]) -> usize {
    copy_to(buf, PRODUCTINFO_REPLY)
}

/// Emit the canned CFGNAV GET reply.
pub fn reply_cfgnav_default(buf: &mut [u8]) -> usize {
    copy_to(buf, CFGNAV_DEFAULT_REPLY)
}

/// Emit `$GNTXT,01,01,00,<name>*cs` pre-ACK for a GET command.
/// `name` is the command name without `$`, e.g. `b"CFGSYS"`.
pub fn reply_preack(buf: &mut [u8], name: &[u8]) -> usize {
    // Reuse build_gntxt to keep the CS in sync with the echoed command.
    nmea::build_gntxt(buf, 1, 1, 0, name)
}

/// Build `$CFGMSG,<class>,<id>,<rate>*cs\r\n` — used as the GET reply for
/// `$CFGMSG,class,id`. Returns bytes written, 0 on overflow.
pub fn build_cfgmsg_reply(buf: &mut [u8], class: u8, id: u8, rate: u8) -> usize {
    // Body: "CFGMSG,<class>,<id>,<rate>" with `class`/`id`/`rate` in decimal.
    fn write_u32(buf: &mut [u8], mut pos: usize, mut v: u32) -> usize {
        let mut tmp = [0u8; 10]; let mut n = 0;
        if v == 0 { tmp[0] = b'0'; n = 1; }
        while v > 0 { tmp[n] = b'0' + (v % 10) as u8; v /= 10; n += 1; }
        if pos + n > buf.len() { return 0; }
        for i in 0..n { buf[pos + i] = tmp[n - 1 - i]; }
        pos += n;
        pos
    }
    if buf.len() < 32 { return 0; }
    let mut p = 0;
    buf[p] = b'$'; p += 1;
    for &b in b"CFGMSG," { buf[p] = b; p += 1; }
    p = write_u32(buf, p, class as u32); if p == 0 { return 0; }
    buf[p] = b','; p += 1;
    p = write_u32(buf, p, id as u32); if p == 0 { return 0; }
    buf[p] = b','; p += 1;
    p = write_u32(buf, p, rate as u32); if p == 0 { return 0; }
    // Checksum over body
    let cs = nmea::nmea_checksum(&buf[1..p]);
    buf[p] = b'*'; p += 1;
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    buf[p] = HEX[(cs >> 4) as usize]; p += 1;
    buf[p] = HEX[(cs & 0x0F) as usize]; p += 1;
    buf[p] = b'\r'; p += 1;
    buf[p] = b'\n'; p += 1;
    p
}

/// Known `$CFGMSG,class,id` pairs observed on live UC6580I (2026-04-23).
/// Unknown pairs are rejected with `$FAIL,0` by the real chip.
pub fn is_known_cfgmsg(class: u8, id: u8) -> bool {
    matches!((class, id),
        // Standard NMEA
        (0, 0..=8) |
        // RTCM 3 standard
        (2, 3) | (2, 4) | (2, 5) | (2, 14) |
        // ExtRTCM 4074 (empirically-verified mapping)
        (9, 1) | (9, 2) | (9, 3) | (9, 4) |
        (9, 7) | (9, 8) | (9, 9) |
        (9, 11) | (9, 12) |
        (9, 15) | (9, 16))
}

/// Emit `$OK*04\r\n`.
pub fn reply_ok(buf: &mut [u8]) -> usize {
    copy_to(buf, OK_REPLY)
}

/// Emit `$FAIL,N*cs\r\n`. `error_code` is 0 (bad params) or 1 (bad checksum).
pub fn reply_fail(buf: &mut [u8], error_code: u8) -> usize {
    match error_code {
        0 => copy_to(buf, FAIL_PARAMS),
        1 => copy_to(buf, FAIL_CHECKSUM),
        _ => 0,
    }
}

/// Emit `$GNTXT,01,01,00,<echoed_cmd>*cs\r\n` — the implicit ACK observed
/// after SET commands. `echoed_cmd` is the full body after the leading `$`,
/// without `*HH` or `\r\n`.
pub fn reply_gntxt_ack(buf: &mut [u8], echoed_cmd: &[u8]) -> usize {
    nmea::build_gntxt(buf, 1, 1, 0, echoed_cmd)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_pdtinfo_bare() {
        let cmd = parse_command(b"$PDTINFO\r\n").unwrap();
        assert_eq!(cmd, Command::PdtInfo);
    }

    #[test]
    fn parse_cfgsys_get_and_set() {
        assert_eq!(parse_command(b"$CFGSYS").unwrap(), Command::CfgSys(None));
        assert_eq!(
            parse_command(b"$CFGSYS,h55155").unwrap(),
            Command::CfgSys(Some(0x55155))
        );
        assert_eq!(
            parse_command(b"$CFGSYS,H55155").unwrap(),
            Command::CfgSys(Some(0x55155))
        );
    }

    #[test]
    fn parse_cfgmsg_with_and_without_rate() {
        assert_eq!(
            parse_command(b"$CFGMSG,9,12,1\r\n").unwrap(),
            Command::CfgMsg { class: 9, id: 12, rate: Some(1) }
        );
        assert_eq!(
            parse_command(b"$CFGMSG,9,12").unwrap(),
            Command::CfgMsg { class: 9, id: 12, rate: None }
        );
    }

    #[test]
    fn parse_cfgnav_three_rates() {
        assert_eq!(
            parse_command(b"$CFGNAV,200,200,0").unwrap(),
            Command::CfgNav { meas_rate: 200, nav_rate: 200, dr_nav_rate: 0 }
        );
    }

    #[test]
    fn parse_accepts_valid_checksum() {
        // XOR of "CFGSYS" bytes is 0x1B.
        assert_eq!(
            parse_command(b"$CFGSYS*1B\r\n").unwrap(),
            Command::CfgSys(None)
        );
    }

    #[test]
    fn parse_rejects_bad_checksum() {
        assert_eq!(
            parse_command(b"$CFGSYS*FF\r\n"),
            Err(ParseError::BadChecksum)
        );
    }

    #[test]
    fn parse_rejects_missing_dollar() {
        assert_eq!(
            parse_command(b"CFGSYS\r\n"),
            Err(ParseError::NoDollarPrefix)
        );
    }

    #[test]
    fn parse_rejects_malformed_numbers() {
        assert_eq!(
            parse_command(b"$CFGMSG,x,12"),
            Err(ParseError::Malformed)
        );
    }

    #[test]
    fn unknown_command_is_passthrough() {
        match parse_command(b"$XYZ,1,2").unwrap() {
            Command::Unknown(name) => assert_eq!(name, b"XYZ"),
            other => panic!("expected Unknown, got {:?}", other),
        }
    }

    #[test]
    fn reply_pdtinfo_checksum_valid() {
        let mut buf = [0u8; 128];
        let n = reply_pdtinfo(&mut buf);
        assert!(n > 0);
        assert!(nmea::verify_sentence(&buf[..n]));
        assert!(buf[..n].starts_with(b"$PDTINFO,UC6580I-00"));
    }

    #[test]
    fn reply_cfgsys_checksum_valid() {
        let mut buf = [0u8; 64];
        let n = reply_cfgsys_default(&mut buf);
        assert_eq!(&buf[..n], b"$CFGSYS,H55155*4E\r\n");
        assert!(nmea::verify_sentence(&buf[..n]));
    }

    #[test]
    fn reply_cfgkey_checksum_valid() {
        let mut buf = [0u8; 256];
        let n = reply_cfgkey(&mut buf);
        assert!(n > 0);
        assert!(nmea::verify_sentence(&buf[..n]));
        assert!(buf[..n].ends_with(b"*3B\r\n"));
    }

    #[test]
    fn fail_codes_match_precomputed() {
        let mut buf = [0u8; 32];
        let n = reply_fail(&mut buf, 0);
        assert!(nmea::verify_sentence(&buf[..n]));
        let n = reply_fail(&mut buf, 1);
        assert!(nmea::verify_sentence(&buf[..n]));
    }

    #[test]
    fn gntxt_ack_matches_real_format() {
        let mut buf = [0u8; 64];
        let n = reply_gntxt_ack(&mut buf, b"PDTINFO");
        assert_eq!(&buf[..n], b"$GNTXT,01,01,00,PDTINFO*1F\r\n");
    }

    // ---- bit-exact checks against live chip replies captured 2026-04-23 ----

    #[test]
    fn cfgsys_reply_bitexact_live() {
        assert_eq!(CFGSYS_DEFAULT_REPLY, b"$CFGSYS,H55155*4E\r\n");
        assert!(nmea::verify_sentence(CFGSYS_DEFAULT_REPLY));
    }

    #[test]
    fn cfgsys_preack_bitexact_live() {
        assert_eq!(CFGSYS_PREACK, b"$GNTXT,01,01,00,CFGSYS*4A\r\n");
        assert!(nmea::verify_sentence(CFGSYS_PREACK));
    }

    #[test]
    fn cfgnav_reply_bitexact_live() {
        assert_eq!(CFGNAV_DEFAULT_REPLY, b"$CFGNAV,200,200,0*07\r\n");
        assert!(nmea::verify_sentence(CFGNAV_DEFAULT_REPLY));
    }

    #[test]
    fn cfgnav_preack_bitexact_live() {
        assert_eq!(CFGNAV_PREACK, b"$GNTXT,01,01,00,CFGNAV*4A\r\n");
        assert!(nmea::verify_sentence(CFGNAV_PREACK));
    }

    #[test]
    fn productinfo_reply_bitexact_live() {
        let expected = b"$PRODUCTINFO,UC6580I-00,G1B1L1E1,V00,R6.1.1.1Build9074,2400615000514,20250427080308004,1.1,,,,*44\r\n";
        assert_eq!(PRODUCTINFO_REPLY, expected);
        assert!(nmea::verify_sentence(PRODUCTINFO_REPLY));
    }

    #[test]
    fn cfgkey_reply_bitexact_live() {
        // Exact bytes captured from live UC6580I chip on 2026-04-23.
        let expected = b"$CFGKEY,,6adae970f433a1c9c55f3bd7092b1400f40dddc4c715c39a5aed529158f0010101010101010101010101010101010101,D0645C*3B\r\n";
        assert_eq!(CFGKEY_REPLY, expected);
        assert!(nmea::verify_sentence(CFGKEY_REPLY));
    }

    #[test]
    fn parse_cfgnav_get_form() {
        assert_eq!(
            parse_command(b"$CFGNAV\r\n").unwrap(),
            Command::CfgNav { meas_rate: 0, nav_rate: 0, dr_nav_rate: 0 }
        );
    }

    #[test]
    fn cfgmsg_reply_matches_live_chip() {
        // Live captures from UC6580I on 2026-04-23:
        //   $CFGMSG,0,0      -> $CFGMSG,0,0,1*06
        //   $CFGMSG,2,3      -> $CFGMSG,2,3,1*07
        //   $CFGMSG,9,15     -> $CFGMSG,9,15,1*3B
        let mut buf = [0u8; 32];
        let n = build_cfgmsg_reply(&mut buf, 0, 0, 1);
        assert_eq!(&buf[..n], b"$CFGMSG,0,0,1*06\r\n");
        let n = build_cfgmsg_reply(&mut buf, 2, 3, 1);
        assert_eq!(&buf[..n], b"$CFGMSG,2,3,1*07\r\n");
        let n = build_cfgmsg_reply(&mut buf, 9, 15, 1);
        assert_eq!(&buf[..n], b"$CFGMSG,9,15,1*3B\r\n");
    }

    #[test]
    fn known_cfgmsg_table() {
        // Sanity on the live-verified subset.
        assert!(is_known_cfgmsg(0, 0));   // GGA
        assert!(is_known_cfgmsg(0, 8));   // GBS
        assert!(is_known_cfgmsg(2, 3));   // MSM
        assert!(is_known_cfgmsg(9, 12));  // Sub 0x0FC
        assert!(is_known_cfgmsg(9, 15));  // Sub 0x0E6
        assert!(!is_known_cfgmsg(0, 99));
        assert!(!is_known_cfgmsg(2, 0));
        assert!(!is_known_cfgmsg(9, 0));
        assert!(!is_known_cfgmsg(99, 0));
    }
}
