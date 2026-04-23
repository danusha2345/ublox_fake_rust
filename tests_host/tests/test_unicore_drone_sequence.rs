//! Drone boot command sequence replay.
//!
//! Captured 2026-04-22 17:27:43 from a live DJI drone talking to its
//! embedded UC6580I. The chip's responses were recorded at the same time;
//! we replay the commands here through our emulator's handler primitives
//! and assert that the bytes we would emit are **identical** to what the
//! real chip emitted.
//!
//! If any assertion fails, the emulator will desynchronise with the drone
//! during boot — the whole point of this test is to catch that before a
//! flight.

use spoof_detector_tests::unicore_cmd::*;

/// Build the full response for a single drone command (replays the branch
/// structure of `handle_command` in `src/main_unicore.rs`).
/// Returns the concatenated reply bytes.
fn emulate_reply(line: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut scratch = [0u8; 256];

    // Extract echo_full / echo_clean the same way handle_command does.
    let mut end = line.len();
    while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
        end -= 1;
    }
    let start = if !line.is_empty() && line[0] == b'$' { 1 } else { 0 };
    let echo_full = &line[start..end];
    let star = echo_full.iter().rposition(|&b| b == b'*').unwrap_or(echo_full.len());
    let echo_clean = &echo_full[..star];

    let cmd = match parse_command(line) {
        Ok(c) => c,
        Err(ParseError::BadChecksum) => {
            let n = reply_gntxt_ack(&mut scratch, echo_full);
            if n > 0 { out.extend_from_slice(&scratch[..n]); }
            let n = reply_fail(&mut scratch, 1);
            out.extend_from_slice(&scratch[..n]);
            return out;
        }
        Err(ParseError::Malformed) => {
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { out.extend_from_slice(&scratch[..n]); }
            let n = reply_fail(&mut scratch, 0);
            out.extend_from_slice(&scratch[..n]);
            return out;
        }
        Err(ParseError::NoDollarPrefix) => return out,
    };

    match cmd {
        Command::Unknown(_) => {}
        Command::PdtInfo => {
            if echo_clean.contains(&b',') {
                let n = reply_gntxt_ack(&mut scratch, echo_clean);
                if n > 0 { out.extend_from_slice(&scratch[..n]); }
                let n = reply_fail(&mut scratch, 0);
                out.extend_from_slice(&scratch[..n]);
                return out;
            }
            let n = reply_pdtinfo(&mut scratch); out.extend_from_slice(&scratch[..n]);
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
        Command::ProductInfo => {
            if echo_clean.contains(&b',') {
                let n = reply_gntxt_ack(&mut scratch, echo_clean);
                if n > 0 { out.extend_from_slice(&scratch[..n]); }
                let n = reply_fail(&mut scratch, 0);
                out.extend_from_slice(&scratch[..n]);
                return out;
            }
            let n = reply_productinfo(&mut scratch); out.extend_from_slice(&scratch[..n]);
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
        Command::CfgSys(None) => {
            out.extend_from_slice(CFGSYS_PREACK);
            let n = reply_cfgsys_default(&mut scratch); out.extend_from_slice(&scratch[..n]);
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
        Command::CfgNav { meas_rate: 0, nav_rate: 0, dr_nav_rate: 0 } => {
            out.extend_from_slice(CFGNAV_PREACK);
            let n = reply_cfgnav_default(&mut scratch); out.extend_from_slice(&scratch[..n]);
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
        Command::CfgKey => {
            let n = reply_cfgkey(&mut scratch); out.extend_from_slice(&scratch[..n]);
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
        Command::CfgMsg { class, id, rate: None } => {
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { out.extend_from_slice(&scratch[..n]); }
            match default_cfgmsg_rate(class, id) {
                Some(r) => {
                    let n = build_cfgmsg_reply(&mut scratch, class, id, r);
                    if n > 0 { out.extend_from_slice(&scratch[..n]); }
                    let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
                }
                None => {
                    let n = reply_fail(&mut scratch, 0); out.extend_from_slice(&scratch[..n]);
                }
            }
        }
        Command::CfgMsg { class, id, rate: Some(_) } => {
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { out.extend_from_slice(&scratch[..n]); }
            if is_known_cfgmsg(class, id) {
                let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
            } else {
                let n = reply_fail(&mut scratch, 0); out.extend_from_slice(&scratch[..n]);
            }
        }
        Command::CfgMsm => {
            out.extend_from_slice(CFGMSM_PREACK);
            let n = reply_cfgmsm(&mut scratch); out.extend_from_slice(&scratch[..n]);
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
        Command::CfgNmea => {
            let n = reply_cfgnmea(&mut scratch); out.extend_from_slice(&scratch[..n]);
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
        Command::CfgPrt(port) => {
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { out.extend_from_slice(&scratch[..n]); }
            match port {
                None | Some(1) => {
                    let n = reply_cfgprt_uart1(&mut scratch); out.extend_from_slice(&scratch[..n]);
                    let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
                }
                Some(2) => {
                    let n = reply_cfgprt_uart2(&mut scratch); out.extend_from_slice(&scratch[..n]);
                    let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
                }
                _ => {
                    let n = reply_fail(&mut scratch, 0); out.extend_from_slice(&scratch[..n]);
                }
            }
        }
        _ => {
            // Remaining SET-ish commands (CfgSave, CfgClr, CfgSys with mask): echo + $OK.
            let n = reply_gntxt_ack(&mut scratch, echo_clean);
            if n > 0 { out.extend_from_slice(&scratch[..n]); }
            let n = reply_ok(&mut scratch); out.extend_from_slice(&scratch[..n]);
        }
    }
    out
}

/// Assert `emulate_reply(cmd)` contains `needle` (order-preserving substring).
/// The real log merges periodic chip output with command replies, so we
/// only check the reply-specific bytes appear.
fn assert_reply_contains(cmd: &[u8], needle: &[u8]) {
    let out = emulate_reply(cmd);
    let found = out.windows(needle.len()).any(|w| w == needle);
    if !found {
        panic!(
            "reply to {:?} missing {:?}\ngot: {:?}",
            core::str::from_utf8(cmd).unwrap_or("<non-utf8>"),
            core::str::from_utf8(needle).unwrap_or("<non-utf8>"),
            core::str::from_utf8(&out).unwrap_or("<non-utf8>")
        );
    }
}

// ---------------------------------------------------------------------------
// Exact drone boot sequence from log_2026-04-22_17-27-43.txt ttyUSB1 <
// ---------------------------------------------------------------------------

#[test]
fn drone_cmd_01_cfgmsg_9_12_1() {
    // SET: $GNTXT echo + $OK
    assert_reply_contains(b"$CFGMSG,9,12,1\r\n", b"$GNTXT,01,01,00,CFGMSG,9,12,1*6D\r\n");
    assert_reply_contains(b"$CFGMSG,9,12,1\r\n", b"$OK*04\r\n");
}

#[test]
fn drone_cmd_02_cfgmsg_9_12_get() {
    // GET: $GNTXT echo + $CFGMSG,9,12,1*3C + $OK
    assert_reply_contains(b"$CFGMSG,9,12\r\n", b"$GNTXT,01,01,00,CFGMSG,9,12*70\r\n");
    assert_reply_contains(b"$CFGMSG,9,12\r\n", b"$CFGMSG,9,12,1*3C\r\n");
    assert_reply_contains(b"$CFGMSG,9,12\r\n", b"$OK*04\r\n");
}

#[test]
fn drone_cmd_03_cfgmsg_9_15_1() {
    assert_reply_contains(b"$CFGMSG,9,15,1\r\n", b"$GNTXT,01,01,00,CFGMSG,9,15,1*6A\r\n");
    assert_reply_contains(b"$CFGMSG,9,15,1\r\n", b"$OK*04\r\n");
}

#[test]
fn drone_cmd_04_cfgmsg_9_15_get() {
    assert_reply_contains(b"$CFGMSG,9,15\r\n", b"$GNTXT,01,01,00,CFGMSG,9,15*77\r\n");
    assert_reply_contains(b"$CFGMSG,9,15\r\n", b"$CFGMSG,9,15,1*3B\r\n");
    assert_reply_contains(b"$CFGMSG,9,15\r\n", b"$OK*04\r\n");
}

#[test]
fn drone_cmd_05_cfgsave() {
    // Live chip: $GNTXT,01,01,00,CFGSAVE*12 + $OK*04 (observed as combined window reply)
    assert_reply_contains(b"$CFGSAVE\r\n", b"$GNTXT,01,01,00,CFGSAVE*12\r\n");
    assert_reply_contains(b"$CFGSAVE\r\n", b"$OK*04\r\n");
}

#[test]
fn drone_cmd_06_cfgsys_get() {
    assert_reply_contains(b"$CFGSYS\r\n", b"$GNTXT,01,01,00,CFGSYS*4A\r\n");
    assert_reply_contains(b"$CFGSYS\r\n", b"$CFGSYS,H55155*4E\r\n");
    assert_reply_contains(b"$CFGSYS\r\n", b"$OK*04\r\n");
}

#[test]
fn drone_cmd_07_cfgkey_get() {
    let out = emulate_reply(b"$CFGKEY\r\n");
    // Byte-exact check of the full response.
    let expected = b"$CFGKEY,,6adae970f433a1c9c55f3bd7092b1400f40dddc4c715c39a5aed529158f0010101010101010101010101010101010101,D0645C*3B\r\n$OK*04\r\n";
    assert_eq!(&out[..], expected);
}
