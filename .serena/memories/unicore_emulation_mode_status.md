## Mode 0 (Emulation, 1-click) — current state 2026-04-27

**Strategy:** Mode 4 (PassthroughOffset) + forced 3D fix + 22 s window + GNTXT field 13 rewrite. Old hybrid CFGKEY-trigger + emulation_task is **gone**. Live chip stream stays intact except coord-bearing NMEA, which we substitute.

### What's substituted

| Sentence/frame | Action | Function |
|---|---|---|
| `$xxGGA / RMC / GSA / GLL` | rewrite to `config::offset_target` + `fq=3, 16 sats, hdop=0.99, status=A, mode=A, fix_type=3` (FIX3D) or `fq=0, 1 sat, V, N, fix_type=1` (NOFIX) | `unicore::nmea::force_3d_fix_sentence` / `force_no_fix_sentence` |
| `$xxTXT,01,01,01,…` (24-field UC6580I status) | rewrite field 13 (fix indicator) → `'3'` (FIX3D) or `'0'` (NOFIX) | `unicore::nmea::rewrite_gntxt_status` |
| All RTCM3 (1005/1006/1019/1046/1077/1087/1097/1117/1127) | **verbatim** | RTCM router in `passthrough_forward_task` |
| ExtRTCM 4074 sub-IDs (proprietary raw obs) | **verbatim** | ditto |
| `$GxGSV / VTG / ZDA / TXT (preack ,00,) / $PNOISE / $P*` | verbatim | passthrough branch |
| `$<binary>` proprietary frames | verbatim | passthrough |

### 22 s window timing

- `FORCED3D_PHASE_START_MS: AtomicU32`. Armed at cold-boot if persisted mode = Emulation, AND on mode-change INTO Emulation. Lazy init on first frame if cold-boot left it 0.
- 0..22 s → `force_3d_fix_sentence` + GNTXT field13='3' (green LED).
- ≥22 s → `force_no_fix_sentence` + GNTXT field13='0', `SATELLITES_INVALID=true` (yellow LED). Coords stay target.
- Window length: `config::timers::SATELLITES_INVALID_AFTER_MS = 22_000`.

### Time/date fallback (for empty chip fields)

`force_*` accept `fallback_time` (uses `NmeaTimeCache.last_time`, 60 s TTL, else `NmeaTime::default()`) and `fallback_date` (uses `NmeaTimeCache.date`, 1500 ms TTL, else `01.01.2025`). Only bad checksum or unknown sentence kind returns None — empty time/date no longer leak chip's `fq=0` to drone via verbatim fallback.

### `nav-debug` Cargo feature (off by default)

Per-frame info! at every Forced3dFix decision + 5 s summary task with atomic counters: GGA/RMC/GSA/GLL/GNTXT rewrite/reject; GSV/VTG/ZDA/TXT/$P*/other passthrough; chip's last fq/nsats/RMC-status; cache freshness. Build: `make flash-unicore-nav-debug`.

### Live results (DJI **Neo** + UC6580I, 2026-04-27)

Run #1 (no GNTXT rewrite): drone did not lock 3D fix. Run #2 (with GNTXT field 13 rewrite to '3'): drone STILL did not lock 3D fix. In both runs chip was cold-boot no-signal (`fq=0 nsats=0` throughout). 11.5 s of FIX3D rewrites observed before window closed.

**Conclusion:** Neo most likely uses RTCM/ExtRTCM 4074 raw observations as primary fix source. Our GGA/RMC/GSA/GLL/GNTXT rewrite is strictly NMEA-side; raw obs flow verbatim with empty chip payload. Drone sees "no real sats observed" → ignores our claimed fq=3 / field13=3.

### Recommendation

For working spoof experiments — **Mode 4 (PassthroughOffset)** with target in `src/config.rs::offset_target`. Mode 0 is experimental.

### Next experiments (TODO)

- ✅ A. GNTXT field 13 rewrite — done, no effect.
- B. Drop ExtRTCM 4074 (proprietary) — keep standard RTCM. Most likely candidate.
- C. Drop full observation family (1077/1087/1097/1117/1127). Stronger.
- D. Synthesise ExtRTCM 4074 with fake obs — heaviest, requires format reverse-engineering.
- E. Rewrite `$<binary>` proprietary frames (~44/5s in `nav-debug` `other` counter) — format unknown.

### Key files

- `src/main_unicore.rs` — `process_nmea_line`, `passthrough_forward_task`, `nav_debug_summary_task`, `FORCED3D_PHASE_START_MS`, `is_gntxt_status` branch.
- `src/unicore/nmea.rs` — `force_3d_fix_sentence`, `force_no_fix_sentence`, `force_fix_inner`, `rewrite_gntxt_status`, `force_gga/rmc/gsa/gll`, `GllFields`, `build_gll`.
- `Cargo.toml` — `nav-debug = []` feature.
- `Makefile` — target `flash-unicore-nav-debug`.
- `docs/UC6580I.md` §4.4 + §4.4-bis.
- Auto memory `project_mode0_forced3dfix_redesign.md`.
- Live logs: `.scratch/drone_attach_190805.log`, `.scratch/drone_attach_gntxt_192804.log`.
- HEAD: branch `feature/unicore-chip`, commit `d2ff707`.
