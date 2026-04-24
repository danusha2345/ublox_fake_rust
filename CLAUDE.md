# u-blox GNSS M10 Emulator (RP2350/RP2354, Embassy)

Two chip targets:
- **u-blox M10** (default) — `make rp2350` / `make rp2354`. UBX + SEC-SIGN ECDSA.
- **Unicore UC6580I** — `make rp2350-unicore` / `make rp2354-unicore` (feature `unicore`, bin `ublox_fake_uc`). NMEA + RTCM + proprietary msg 4074. No SEC-SIGN. Entry point `src/main_unicore.rs`, protocol stack in `src/unicore/`. See `docs/UC6580I.md` for live-verified protocol reference.

## CRITICAL: Version Dependencies

| Crate | Version | Why |
|-------|---------|-----|
| `embedded-io-async` | **0.6** | MUST match embassy-rp internals. v0.7 breaks! |
| `pio` | **0.3** | Only v0.3 exports `pio_asm!` macro |
| `embassy-rp` | 0.9 | API changed from 0.8 |

## Build & Test

**Always use Makefile for UF2!** Manual objcopy loses address info.

```bash
PATH="/home2/.cargo/bin:$PATH"
make rp2350 / make rp2354              # Build UF2
make flash / make flash-rp2354         # Flash via probe-rs
make test                              # All 220 host-side tests
cd tests_host && cargo test            # spoof_detector + passthrough + coordinates + unicore
```

`tests_host/` is standalone crate (NOT in workspace), uses `#[path]` to `../../src/spoof_detector.rs`.

## Architecture

Dual-core Embassy async. Core0: UART TX/RX, NAV generation, button. Core1: LED (PIO), SEC-SIGN ECDSA.
Inter-core: `Signal`/`Channel` (embassy-sync). Mode state: `AtomicU8`.

### Modules
- `src/ubx/` — UBX protocol (messages, parser)
- `src/unicore/` — NMEA / RTCM / ExtRTCM 4074 protocol (unicore feature only)
- `src/sec_sign.rs` — SHA256 + ECDSA SECP192R1
- `src/spoof_detector.rs` — GPS spoofing detection (pure logic, host-testable). Exports `SPOOF_NSATS_MARKER = 92` consumed by both builds.
- `src/pos_history.rs` — shared `PositionBuffer` (3 s ring @ 5 Hz) + `DynamicOffset`, used by both UBX and NMEA paths
- `src/passthrough.rs` — UBX frame parser + NAV modification (re-exports `pos_history` types for backward compat)
- `src/config.rs` — pins, timing, default position
- `src/coordinates.rs` — LLH/ECEF (WGS84)
- `src/flash_storage.rs` — mode + key persistence
- `src/key_extract.rs` — runtime key extraction via CFG-0x41
- `src/led.rs` — WS2812 (PIO)

### Operating Modes
| Mode | ID | LED | Description |
|------|----|-----|-------------|
| Emulation | 0 | green/yellow | Fake GNSS + SEC-SIGN; yellow after invalid-satellite timeout |
| Passthrough | 1 | blue | Forward real GNSS, spoof detection |
| PassthroughRaw | 2 | purple | Transparent forwarding |
| PassthroughOffset | 3 | white | Passthrough + coordinate offset |

Mode persisted to flash. Button: 1-4 clicks. Timeout: 800ms.

## Hardware Pins

**RP2350A** (SpotPear): UART0 TX=GPIO0, RX=GPIO1 (921600). UART1 RX=GPIO5. WS2812B=GPIO25. Button=GPIO14/13.
**RP2354A** (`--features rp2354`): Same UARTs. GPIO LED=GPIO11/12. Button=GPIO14/13.

## Key Gotchas

- **SEC-UNIQID race**: Mavic 4 Pro sends SEC-UNIQID BEFORE CFG-VALSET. Workaround: set `DRONE_MODEL` in `main.rs` (default=2 Air 3S)
- **accumulate() BEFORE write_all()**: prevents hash race between uart0_tx and sec_sign_timer on Core1
- **Coord recovery = 6 samples** (not 5): returning from spoofed position is itself a teleport
- **Spoof marker**: `spoof_detector::SPOOF_NSATS_MARKER` (=92) — impossible sat count planted in NAV-PVT/SOL/SAT/SVINFO and NMEA GGA under spoof
- **Diagnostic mode**: `DIAG_MSG_DETAIL = true` in main.rs
- **Unicore UART signal integrity**: `src/main_unicore.rs` must mirror the proven u-blox hardware UART settings: GPIO0 UART0 TX pad = 12mA drive + pull-down + fast slew, and UART1 RX FIFO threshold = 1/4. If raw passthrough also outputs garbage at 921600, check these settings before protocol logic.
- **Unicore mode 1 parity**: Emulation mirrors u-blox mode 1 timing: valid fix + green LED first, then invalid fix + yellow LED after `SATELLITES_INVALID_AFTER_MS`.
- **Unicore passthrough router is RTCM-aware** (`passthrough_forward_task` in `src/main_unicore.rs`): detect `0xD3` preamble before splitting on `$`, otherwise a `$` (0x24) byte inside an RTCM payload steals the rest of the frame into `LineAssembler` and it is silently dropped. Covers ExtRTCM 4074.
- **pos_buffer lookback age-gate**: callers `pos_buffer.push()` **before** `detector.analyze()`. After a long fix-loss gap (> 3 s ring capacity), the first new sample wins "closest-to-now-2s" against stale pre-gap entries if the gate is missing → `LAST_GOOD` captures the spoofed position. `pos_history::get_position_at` must only return entries with `age >= seconds_ago*1000`. Shared by u-blox and Unicore branches; sibling to the 2026-04-03 `has_3d_fix()` push-site guard but orthogonal. Regression log: `log_2026-04-24_14-38-32.txt`.
- **Unicore detector-feed parity with u-blox** (`process_nmea_line` in `src/main_unicore.rs`): mirror the u-blox NAV-PVT contract on NMEA. (a) Cache the RMC date on **any** checksum-ok RMC with `year > 0` — do **not** gate on `fix_quality > 0`. The chip still emits date/time in V-status RMCs during no-fix ticks, and without the cache `gnss_time` synthesises as `None`, killing `check_gnss_time` and `check_system_clock_drift`. (b) Call `detector.analyze()` on **every** checksum-ok GGA, setting `fix_type = Fix3D` when `fq > 0` else `NoFix`. The detector's internal `!has_3d_fix` early-return handles the NoFix case (no harm). `pos_buffer.push` and `dynamic_offset` compute stay gated on `has_3d_fix` so LAST_GOOD and the takeoff offset baseline don't get polluted.
- **Unicore NMEA coord-jump secondary trigger** (`process_nmea_line`): NMEA splits position across GGA and RMC, and UC6580I can emit (i) RMC with `A` status ahead of the paired GGA, or (ii) GGA/RMC with `fq=0 / V` but non-zero coords for ~1 tick before upgrading to `fq>0`. The detector's internal `!has_3d_fix` early-return skips both cases, so raw SPB coords leak to the drone before the next valid-fix GGA. Mitigation: on any checksum-ok GGA/RMC with non-zero coords, compute haversine distance from `pos_buffer`'s newest entry via `SpoofDetector::calc_distance` (made `pub` for this path). Jump > `thresholds::TELEPORT_M` (2 km) flips SPOOF_DETECTED, captures LAST_GOOD via the age-gated lookback, and resets the recovery timer. Catches scenarios u-blox doesn't face because NAV-PVT carries fix-type+coords atomically. Regression logs: `log_2026-04-24_15-25-33.txt` (A-RMC-first), `log_2026-04-24_15-41-48.txt` (fq=0 coord leak).
- **Unicore GLL/VTG drop under spoof or offset rewrite** (`process_nmea_line` non-GGA/RMC branch): UC6580I keeps GLL (`$CFGMSG,0,1`) and VTG (`$CFGMSG,0,5`) off by default but the drone can enable them and persist the choice in chip NVM via `$CFGSAVE`. `$GxGLL` carries lat/lon, `$GxVTG` carries speed/course — forwarding either verbatim while we're rebuilding GGA/RMC leaks the chip's raw position (GLL) or movement state (VTG, equivalent to u-blox `NAV-VELNED`/`VELECEF` zeroing). Fix: in spoof, drop both; in `apply_offset`, drop GLL too (we don't rewrite GLL formatting and it would expose real coords next to the offset GGA/RMC). VTG is left alone in plain offset mode — velocity is invariant under a constant LLH offset.

## Detailed Reference (Serena memories)

`sec_sign_protocol` — SEC-SIGN algorithm, drone timing, TX pause mechanism
`cfg_0x41_key_extraction` — CFG-0x41 format, auto/manual key extraction
`spoof_detection_algorithm` — full algorithm, state variables, warmup phases, NAV modification
`ubx_messages` — all implemented UBX message IDs and payloads
