# 4074-0xFF cryptographic chip authentication (2026-04-27 finding)

## Bottom line

UC6580 4074-0xFF stream is **cryptographically signed** by chip firmware. DJI Neo verifies and **rejects ANY** modification or drop of this binary. Direct spoofing of 0x0FF (which carries the authoritative `Quality` byte for fix state) is **impossible without firmware reverse engineering** of the chip's signing key.

## Experimental matrix

| Action on 4074-0xFF | Drone result |
|---|---|
| Forward verbatim | ✅ accepts module (chip Quality=0 → no fix) |
| Modify in-place (CRC24Q correct) | ❌ "module not accepted" |
| Drop entirely (post-id-window) | ❌ "module not accepted" |
| Drop ALL 4074 sub-IDs | ❌ "module missing" |
| NMEA modify, 4074 verbatim | ✅ accepts module, ❌ no fix (drone reads 0x0FF.Quality=0 over our spoofed NMEA) |

## CFGKEY exchange (~20s)

Drone sends `$CFGKEY` (empty query), chip responds with **constant** string across all sessions:
```
$CFGKEY,,6adae970f433a1c9c55f3bd7092b1400f40dddc4c715c39a5aed529158f0010101010101010101010101010101010101,D0645C*3B
```

= 30 bytes high-entropy + 18 bytes `0x01` filler + 24-bit fingerprint. **Static chip-model identifier**, not per-session challenge or signing key.

## Trust model split

- **NMEA**: trusted-by-association — modify accepted (Mode 4 works for coord spoofing)
- **RTCM 4074**: signature-verified, byte-perfect required

## Where the signature lives

Most likely in **4074-0xFC** (130 byte payload, undocumented in public R1.2):
- `01 01` version+type
- 32 zero bytes (SHA-256 hash placeholder?)
- **80 bytes high-entropy** (signature/HMAC carrier)
- 16 zero bytes padding

80-byte high-entropy block is textbook signature size. Public Unicore docs do NOT mention signing/auth — proprietary mechanism, likely DJI-OEM contract.

## Implications for Mode 0

Mode 0 fix-spoof (no antenna, fake fix at target coords) is **functionally blocked at the protocol layer**. Cannot:
- Synthesize 0x0FF.Quality=1 (no signing key)
- Drop chip's 0x0FF (heartbeat missing → reject)
- Inject duplicates (likely detected)

## Working alternative

**Mode 4 (PassthroughOffset) + real antenna** — chip gets real lock naturally, we shift only NMEA coords. Drone signature check is NOT triggered (4074 untouched). Documented working in `unicore_emulation_mode_status` memory.

## Future paths

1. Mode 4 + antenna (CURRENT WORKING)
2. Reverse-engineer UC6580 firmware blob for signing key (major undertaking)
3. Replay captured Quality=1 frames from a session with real signal (risk: timestamp mismatch)
4. Use Air 3S (u-blox M10 + UBX SEC-SIGN) — different protocol, key extraction already implemented

## Key files

- Live test logs: `.scratch/drone_attach_modify4074_*.log`, `.scratch/drone_attach_drop0xff_*.log`, `.scratch/drone_attach_idspoof_*.log`
- `src/unicore/extrtcm.rs::modify_receiver_info` — host-validated bit-correct, drone-rejected
- `src/main_unicore.rs::passthrough_forward_task` — RTCM router with 5-byte msg_id+sub_id scratch
- Auto memory: `project_unicore_4074_hmac_signature.md` (full experimental log)
- HEAD: branch `feature/unicore-chip`, commit `6de5491` (working tree has WIP modify code, not committed)

## Test count

248 host tests green throughout experiments. Modification function `modify_receiver_info` passes `verify_frame()` byte-for-byte — host-side correctness confirmed. Drone-side rejection means the issue is OUTSIDE what public spec / our test coverage can catch.
