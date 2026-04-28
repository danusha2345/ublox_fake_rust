# Mode 1 (Passthrough, 2-click, default) — RTCM drop under spoof + GLL rebuild

## Behavior

- **Default mode** (2-click, `OperatingMode::Passthrough` carries `#[default]`).
- **No spoof**: NMEA verbatim, RTCM verbatim — pure passthrough.
- **Spoof detected** (`SPOOF_DETECTED == true`):
  - All RTCM frames silently dropped at the RTCM router (every msg_id incl. 1077/1087/1097/1117/1127, 4074 with all sub-IDs incl. 0x0FC chip-auth).
  - NMEA: GGA/RMC rebuilt at LAST_GOOD with `fq=0`/`status=V`, **GLL also rebuilt** via new `build_spoofed_gll` (was drop), VTG verbatim, GSA/GSV/ZDA/TXT/$P* verbatim.
  - Decision re-evaluated per RTCM frame (each 0xD3) — when spoof clears, RTCM resumes.

## Implementation

- `src/main_unicore.rs::passthrough_forward_task` — `rtcm_drop_for_spoof: bool` set when `mode == Passthrough && SPOOF_DETECTED` at scratch decision (after 6-byte D3+LL+MID parsed).
- `src/main_unicore.rs::build_spoofed_gll(time, coords, out)` — mirror of `build_spoofed_gga`/`build_spoofed_rmc` for GLL.
- `src/main_unicore.rs::process_nmea_line` Plain branch — GLL under spoof OR offset → rebuild (was drop).
- nav-debug counter `RTCM_DROPPED_SPOOF` (separate from `RTCM_4074_DROPPED` for Mode 0 and `RTCM_KEPT`).

## Live test 2026-04-27 (DJI Neo + UC6580I)

`.scratch/drone_attach_mode1_rtcm_drop_111133.log` — 6678 lines, 150s session:
- 0..67.5s: chip got real fix `fq=1 nsats=19 rmc=A`, RTCM kept=3223
- 67.53s: detector fired `GNSS TIME anomaly + SYSTEM CLOCK DRIFT`
- 67.5s+: all RTCM dropped (1077, 1097, 4074-FF/E6/0/1/2/...), `drop_spoof` counter +60/s
- 148s: kept=3223, drop_spoof=4856 — 80s spoof window, RTCM stream fully gated

Algorithm verified working end-to-end.

## Counter-spoof rationale

When RF-level GNSS spoofing affects the chip, the chip itself sees "real" satellites at the spoofed location and emits a CRYPTOGRAPHICALLY SIGNED 4074-0xFF with the fake position. Drone would trust it (signature valid — chip really did sign that data, see `extrtcm_4074_signature` memory). Mode 1's NMEA-side telejump detector fires in time and shuts off the entire RTCM channel, denying the drone access to the (chip-authenticated but RF-spoofed) raw fix data. NMEA simultaneously holds LAST_GOOD coords so drone interprets it as "fix lost, hold last position" rather than confusing "I see a chip-signed valid fix at a wrong location".

This complements Mode 4 (PassthroughOffset, real-antenna spoof testing) and Mode 0 (cosmetic fix-claim, blocked by signature).

## Mode 2 (PassthroughRaw, 3-click)

Reverted to **byte-for-byte raw forward** (`enqueue(&chunk)` with no parsing). Earlier draft confused 2-click vs internal enum 2 — fixed.

## Test count

248 host tests green. RTCM-drop-under-spoof is runtime-only behavior using existing detector + RTCM router; no new tests needed.
