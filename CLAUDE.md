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
- `src/spoof_detector.rs` — GPS spoofing detection (pure logic, host-testable). Exports `SPOOF_NSATS_MARKER = 2` for the UBX path.
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
- **Spoof marker**: `spoof_detector::SPOOF_NSATS_MARKER` (=2) is planted in UBX NAV-PVT/SOL/SAT/SVINFO under spoof; Unicore spoofed GGA reports invalid fix with 1 visible satellite
- **Diagnostic mode**: `DIAG_MSG_DETAIL = true` in main.rs
- **Unicore UART signal integrity**: `src/main_unicore.rs` must mirror the proven u-blox hardware UART settings: GPIO0 UART0 TX pad = 12mA drive + pull-down + fast slew, and UART1 RX FIFO threshold = 1/4. If raw passthrough also outputs garbage at 921600, check these settings before protocol logic.
- **Unicore Mode 0 = passthrough + immediate coord substitution + 22 s window** (2026-04-27 redesign, replaces old hybrid CFGKEY-trigger logic). `passthrough_forward_task` `Forced3dFix` branch dispatches GGA/RMC/GSA/**GLL** through `unicore::nmea::force_3d_fix_sentence`/`force_no_fix_sentence` into `config::offset_target`. First `SATELLITES_INVALID_AFTER_MS=22_000` ms (timer = `FORCED3D_PHASE_START_MS`, armed on cold-boot if persisted Mode 0 OR on mode-change INTO Mode 0) emit `fq=3, 16 sats, status=A, mode=A, fix_type=3` (green LED). After window: `fq=0, 1 sat, V, N, fix_type=1` — coords stay target, `SATELLITES_INVALID=true` (yellow LED). `force_*` accept fallback time/date and **never reject on empty time/date** — use `NmeaTimeCache.last_time` (60 s TTL) or `NmeaTime::default()` + `01.01.2025`; only checksum failure / unknown sentence kind returns `None`. **Empirical 2026-04-27** (DJI Neo + UC6580I): even with `$GNTXT,01,01,01,…` field 13 also rewritten to `'3'` (run #2), drone doesn't lock 3D fix. Most likely it weights RTCM/ExtRTCM 4074 raw observations as primary source, which we still pass verbatim with chip's empty/no-signal payload. See memory `project_mode0_forced3dfix_redesign.md`.
- **Unicore Mode 0 GNTXT status rewrite** (`src/unicore/nmea.rs::rewrite_gntxt_status`, `process_nmea_line` `is_gntxt_status` branch): in Forced3dFix policy, `$xxTXT,01,01,01,…` periodic status (24-field UC6580I-extended TXT) has its field 13 (fix progression indicator) replaced with `'3'` during the 22 s FIX3D window and `'0'` after. Validates checksum + first 3 fields = `01,01,01` (skips `,00,…` preack). Counter `nav_debug::GNTXT_STATUS_REWRITE`. Did not flip drone behaviour — see memory above.
- **`nav-debug` Cargo feature** (`Cargo.toml`, `src/main_unicore.rs::nav_debug`): per-frame `info!` of every Forced3dFix decision (rewrite kind, FIX3D|NOFIX, elapsed, cache freshness) + 5 s summary task counting GGA/RMC/GSA/GLL rewrites, GSV/VTG/ZDA/TXT/`$P*`/other passthrough, plus chip's last fq/nsats/RMC-status. Build via `make flash-unicore-nav-debug`. Production OFF.
- **Unicore passthrough router is RTCM-aware** (`passthrough_forward_task` in `src/main_unicore.rs`): detect `0xD3` preamble before splitting on `$`, otherwise a `$` (0x24) byte inside an RTCM payload steals the rest of the frame into `LineAssembler` and it is silently dropped. Covers ExtRTCM 4074.
- **pos_buffer lookback age-gate**: callers `pos_buffer.push()` **before** `detector.analyze()`. After a long fix-loss gap (> 3 s ring capacity), the first new sample wins "closest-to-now-2s" against stale pre-gap entries if the gate is missing → `LAST_GOOD` captures the spoofed position. `pos_history::get_position_at` must only return entries with `age >= seconds_ago*1000`. Shared by u-blox and Unicore branches; sibling to the 2026-04-03 `has_3d_fix()` push-site guard but orthogonal. Regression log: `log_2026-04-24_14-38-32.txt`.
- **Unicore detector-feed parity with u-blox** (`process_nmea_line` in `src/main_unicore.rs`): mirror the u-blox NAV-PVT contract on NMEA. (a) Cache the RMC date on **any** checksum-ok RMC with `year > 0` — do **not** gate on `fix_quality > 0`. The chip still emits date/time in V-status RMCs during no-fix ticks, and without the cache `gnss_time` synthesises as `None`, killing `check_gnss_time` and `check_system_clock_drift`. (b) Call `detector.analyze()` on **every** checksum-ok GGA, setting `fix_type = Fix3D` when `fq > 0` else `NoFix`. The detector's internal `!has_3d_fix` early-return handles the NoFix case (no harm). `pos_buffer.push` and `dynamic_offset` compute stay gated on `has_3d_fix` so LAST_GOOD and the takeoff offset baseline don't get polluted.
- **Unicore NMEA coord-jump secondary trigger** (`process_nmea_line`): NMEA splits position across GGA and RMC, and UC6580I can emit (i) RMC with `A` status ahead of the paired GGA, or (ii) GGA/RMC with `fq=0 / V` but non-zero coords for ~1 tick before upgrading to `fq>0`. The detector's internal `!has_3d_fix` early-return skips both cases, so raw SPB coords leak to the drone before the next valid-fix GGA. Mitigation: on any checksum-ok GGA/RMC with non-zero coords, compute haversine distance from `pos_buffer`'s newest entry via `SpoofDetector::calc_distance` (made `pub` for this path). Jump > `thresholds::TELEPORT_M` (2 km) flips SPOOF_DETECTED, captures LAST_GOOD via the age-gated lookback, and resets the recovery timer. Catches scenarios u-blox doesn't face because NAV-PVT carries fix-type+coords atomically. Regression logs: `log_2026-04-24_15-25-33.txt` (A-RMC-first), `log_2026-04-24_15-41-48.txt` (fq=0 coord leak).
- **Unicore GLL/VTG drop under spoof or offset rewrite** (`process_nmea_line` non-GGA/RMC branch): UC6580I keeps GLL (`$CFGMSG,0,1`) and VTG (`$CFGMSG,0,5`) off by default but the drone can enable them and persist the choice in chip NVM via `$CFGSAVE`. `$GxGLL` carries lat/lon, `$GxVTG` carries speed/course — forwarding either verbatim while we're rebuilding GGA/RMC leaks the chip's raw position (GLL) or movement state (VTG, equivalent to u-blox `NAV-VELNED`/`VELECEF` zeroing). Fix: in spoof, drop both; in `apply_offset`, drop GLL too (we don't rewrite GLL formatting and it would expose real coords next to the offset GGA/RMC). VTG is left alone in plain offset mode — velocity is invariant under a constant LLH offset.
- **`prev_gga_sod` snapshot** (`process_nmea_line`): the primary GGA branch updates `time_cache.last_gga_sod = Some(sod)` early, **before** secondary trigger #2 reads it for its own midnight-rollover guard. Without a snapshot, secondary #2 always sees `prev == sod` so its rollover skip never fires — and a no-fix GGA with empty time field arriving while the RMC date cache is still fresh (<1500 ms after Phase-1 end) synthesises `gnss_time = {date, 00:00:00}` and looks like a 12 h backward jump → spurious SPOOF on every in-flight fix-loss. Take a single `let prev_gga_sod = time_cache.last_gga_sod;` snapshot at the top of the function and use it in **both** rollover checks. Regression: `tests_host/tests/test_unicore_spoof_pipeline.rs::unicore_pipeline_no_false_spoof_on_in_flight_fix_loss`.
- **Gap-branch speed check** (`SpoofDetector::analyze`, gap branch ≥ `MAX_GAP_MS=5000`): the original gap branch only had a teleport check (`dist_from_good > TELEPORT_M`) and `gnss_time` checks. A spoofer that places B within 2 km of `last_good` and either preserves the GNSS time or arrives on a GGA-first frame with stale RMC cache (`gnss_time = None`) skipped both gates → `gap_spoof = false` → `GapReset` → SPOOF stays clear → next frame's `prev` is the spoofed B and the world looks normal again. Closes the leak by adding "Check 1b": `dist_from_good / dt_s > MAX_SPEED_MS` (30 m/s). 1500 m / 7 s ≈ 214 m/s is implausible for any drone, while a legit reacquisition at ≤ 30 m/s after a real gap stays under the threshold. Regression: `tests_host/tests/test_unicore_spoof_pipeline.rs::unicore_pipeline_blocks_case_b_one_frame_leak`.

## Detailed Reference (Serena memories)

`sec_sign_protocol` — SEC-SIGN algorithm, drone timing, TX pause mechanism
`cfg_0x41_key_extraction` — CFG-0x41 format, auto/manual key extraction
`spoof_detection_algorithm` — full algorithm, state variables, warmup phases, NAV modification
`ubx_messages` — all implemented UBX message IDs and payloads
