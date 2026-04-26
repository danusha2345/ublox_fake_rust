# Spoof Detection Algorithm

## Data Flow
UART1 RX → uart1_rx_task → RAW_RX_CHANNEL → gnss_processing_task (parse+detect+modify) → GNSS_RX_CHANNEL → uart0_tx_task → UART0 TX

## Key State Variables
- `origin` — first GPS fix, never updated (only on reset()), anti-gradient-drift + leash anchor
- `last_good` — last trusted position, updated only when !spoofed && dist < 2km && within_leash(origin, 5km)
- `prev` — previous sample, always updated (for velocity calc)
- `last_good_gnss_time` / `calibrated_at_system_ms` — time refs, updated only when !spoofed

## Anomaly Flags (SpoofDetector::analyze(), 5Hz on NAV-PVT)
- coord_anomaly: teleport >2km OR speed >30m/s OR alt_jump >10m/sample [disabled in recovery warmup]
- time_anomaly: time_jump_back >1s OR time_jump_fwd >5s OR clock_drift >10s [disabled in startup warmup]
- last_good_anomaly: dist(last_good→curr) > 2km [disabled in warmups]
- origin_drift: dist(origin→curr) > 10km [disabled in startup warmup]

## Gap Branch (`dt_ms > MAX_GAP_MS=5000`)
Three independent checks; any of them flips `gap_spoof = true`:
1. **Teleport** — `dist(last_good, pos) > TELEPORT_M` (2 km).
2. **Average speed** (added 2026-04-26) — `dist(last_good, pos) / dt_s > MAX_SPEED_MS` (30 m/s). Catches sub-teleport jumps that imply unrealistic ground speed across the gap. Closes the leak window when a smart spoofer keeps B within 2 km of `last_good` and either preserves GNSS time or hits a GGA-first frame with stale RMC date cache (`gnss_time = None`) so neither check 1 nor check 3 fire. Legit reacquisition at ≤ 30 m/s after a real fix-loss stays under the threshold. Regression: `tests_host/tests/test_unicore_spoof_pipeline.rs::unicore_pipeline_blocks_case_b_one_frame_leak`.
3. **GNSS time anomalies** — `check_system_clock_drift` + `check_gnss_time` (only if `gnss_time.is_some()`).

If all three are clean, gap branch returns `GapReset`, sets `prev = pos`, and the **next** sample is compared against this fresh `prev` in the normal branch.

## Warmup Phases
- Startup (10 samples, ~2s): time checks disabled, coord checks active
- Recovery (10 samples, ~2s): coord + last_good checks disabled, time checks active

## Recovery
Coord recovery requires 6 samples (not 5) — returning from spoofed position is itself a teleport that resets normal_count.

## LAST_GOOD Capture on Spoof Edge
When `analyze()` transitions to Spoofed, caller reads `pos_buffer.get_position_at(2, now_ms)` and stores it as `LAST_GOOD`. Three non-obvious invariants:
- Push-site guard (2026-04-03): only `pos_buffer.push()` when `fix_type.has_3d_fix()` — otherwise no-fix `(0,0,0)` entries would overwrite the ring and poison LAST_GOOD.
- Lookback age-gate (2026-04-24, `src/pos_history.rs`): `get_position_at(seconds_ago, now)` must only consider entries with `age >= seconds_ago*1000`. Callers `push()` **before** `analyze()`; after a > 3 s fix-loss gap, the first new sample (already spoofed) is numerically closest to `now - 2 s` and would win the closest-match race against stale pre-gap entries without the gate. Shared by u-blox (`main.rs:1651/1674`) and Unicore (`main_unicore.rs`) branches. Regression log: `log_2026-04-24_14-38-32.txt`. Test: `tests_host::test_position_buffer_rejects_fresh_entry_on_lookback`.
- Unicore detector-feed parity (2026-04-24, `main_unicore.rs::process_nmea_line`): must mirror u-blox's NAV-PVT contract on NMEA so time-based checks can fire on the first post-gap frame. (a) RMC date cache updates on any checksum-ok RMC with `year > 0` — no `fq > 0` gate (chip keeps emitting date/time in V-status RMCs during no-fix ticks). (b) `detector.analyze()` is called on every checksum-ok GGA with `fix_type` derived from `fq` (Fix3D if > 0, else NoFix); the detector's own early-return handles NoFix. Without this parity, `gnss_time` stayed `None` across gaps and `check_system_clock_drift` / `check_gnss_time` never ran.

## Unicore NMEA Secondary Spoof Trigger
NMEA splits lat/lon across GGA and RMC, and UC6580I can emit an `A`-RMC before its paired GGA or an `fq=0 / V-status` frame with non-zero coords for ~1 tick before `fq` flips. The detector's internal `!has_3d_fix` early-return skips both cases → raw SPB coords leak to the drone. Mitigation in `process_nmea_line` (after the primary GGA branch): on any checksum-ok GGA/RMC with non-zero coords, compute `SpoofDetector::calc_distance` (made `pub`) from `pos_buffer`'s newest entry; jump > `thresholds::TELEPORT_M` (2 km) sets SPOOF_DETECTED, captures LAST_GOOD from the lookback window, and resets the recovery timer. u-blox doesn't need this because NAV-PVT carries fix-type and coords in one atomic message. Regression logs: `log_2026-04-24_15-25-33.txt` (A-RMC-first), `log_2026-04-24_15-41-48.txt` (fq=0 coord leak).

## `prev_gga_sod` snapshot (2026-04-26, `process_nmea_line`)
Take a single `let prev_gga_sod = time_cache.last_gga_sod;` snapshot at the top of the function, BEFORE the primary GGA branch overwrites the cell. Both rollover guards (the primary's `gnss_time` build, and secondary trigger #2's `frame_date` build) **must** read this snapshot, not the live `time_cache.last_gga_sod`. Without it, the primary updates the cell first and secondary #2 always sees `prev == sod` → midnight-rollover skip never fires → an in-flight fix-loss with empty time field synthesises a 12 h backward jump while the RMC date cache is still fresh (<1500 ms) → spurious SPOOF on every fix-loss tick. Regression: `tests_host/tests/test_unicore_spoof_pipeline.rs::unicore_pipeline_no_false_spoof_on_in_flight_fix_loss`.

## Return-to-Real Recovery (u-blox parity)
Recovery is identical on both branches (`main.rs:1690-1712` / `main_unicore.rs`): any `result != Spoofed` while `was_spoofed` starts `SPOOF_RECOVERY_START_MS` (system-time ms); after `SPOOF_RECOVERY_TIMEOUT_MS` (5 s) of continuous non-Spoofed returns, `SPOOF_DETECTED` clears and pass-through resumes verbatim. During active Unicore spoof, the coord-jump secondary resets `SPOOF_RECOVERY_START_MS` on every leaking frame, so the 5 s timer cannot advance until the chip actually stops shipping SPB coords. `dynamic_offset` (computed once at takeoff) is **not** invalidated — it must stay constant for the whole flight. Detector's `calibrated_at_system_ms` / `calibrated_gnss_unix` (`check_system_clock_drift`) stop updating during spoof (`!self.spoofed` gate), so when real coords return the embassy-tracked system-time delta vs calibrated GNSS time gives drift ≈ 0 → `is_drift_recovery`.

## NAV Modification During Spoofing (10 message types)
NAV-PVT, POSLLH, POSECEF, HPPOSECEF, SOL: coords→LAST_GOOD, vel→0
NAV-VELECEF, VELNED: vel→0, speed→0
NAV-STATUS: gps_fix=0, flags=0
NAV-SAT, SVINFO: num_sv=92 (impossible value = spoof marker)

## PassthroughOffset Mode
Dynamic offset computed ONCE at first 3D fix, never recomputed.
- LLH: linear offset = target - actual
- ECEF: recomputed per-frame via llh_to_ecef_cm() for geometric consistency
- Startup protection: coord messages suppressed until offset computed
- Spoof detection uses ORIGINAL coords, offset applied to output only
- dynamic_offset never invalidated after spoof recovery
