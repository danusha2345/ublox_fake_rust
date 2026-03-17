# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

u-blox GNSS M10 emulator written in Rust for RP2350/RP2354 microcontrollers. Uses Embassy async framework instead of FreeRTOS.

**Note**: RP2040 no longer supported — requires ~221 KB RAM for buffers, exceeding available 264 KB.

**Original C version**: `../ublox_fake_unified/` - FreeRTOS based, for reference.

## CRITICAL: Version Dependencies

These version mismatches caused build failures - DO NOT change without testing:

| Crate | Version | Why |
|-------|---------|-----|
| `embedded-io-async` | **0.6** | MUST match embassy-rp internals. v0.7 breaks trait resolution! |
| `pio` | **0.3** | Only v0.3 exports `pio_asm!` macro. v0.2 has only struct API |
| `embassy-rp` | 0.9 | Latest as of Dec 2024. API changed from 0.8 |

## Build Environment

```bash
# Rust may not be in PATH, check:
PATH="/home2/.cargo/bin:$PATH"

# Required target
rustup target add thumbv8m.main-none-eabihf  # RP2350
```

## Required Files

- `build.rs` - Build script that auto-generates `memory.x` linker script + firmware version from git
- `.cargo/config.toml` - Target selection and linker flags

## Firmware Version Storage

Firmware version is automatically extracted from git at build time and stored in flash on the board.

**How it works:**
1. `build.rs` runs `git rev-parse --short HEAD` + dirty check → generates `$OUT_DIR/version.rs`
2. Version format: `"0.1.0-a72ddc5"` or `"0.1.0-a72ddc5-dirty"` (Cargo.toml version + git hash)
3. At startup, firmware writes version to flash (third-to-last sector), skips write if unchanged
4. defmt log prints version at boot: `"Firmware version: 0.1.0-a72ddc5"`

**Flash layout for version data** (sector = 4KB = `ERASE_SIZE`):
| Sector | Offset (4MB) | Offset (2MB) | Content |
|--------|-------------|-------------|---------|
| Last-2 | 0x3FE000 | 0x1FE000 | Mode data (ModeData) |
| Last-3 | 0x3FD000 | 0x1FD000 | **Version data** |

**Version data format in flash** (37 bytes):
| Offset | Size | Content |
|--------|------|---------|
| 0 | 4 | Magic `0x56455253` ("VERS", LE) |
| 4 | 1 | String length |
| 5 | 32 | Version string (null-padded) |

**Reading version from a flashed board** (via probe-rs):
```bash
# RP2350 (4MB flash): version at 0x103FD000
probe-rs read --chip RP2350 0x103FD000 37

# RP2354 (2MB flash): version at 0x101FD000
probe-rs read --chip RP2354 0x101FD000 37
```

**Build-time regeneration**: `build.rs` watches `.git/HEAD` and `.git/index` — version updates automatically on new commits.

## Build Commands

**CRITICAL: Always use Makefile to build UF2 files!**

```bash
# Build UF2 for RP2350 (external QSPI flash)
make rp2350

# Build UF2 for RP2354 (2MB internal flash)
make rp2354

# Flash via probe-rs
make flash          # RP2350 (default)
make flash-rp2354   # RP2354

# Or directly
cargo run --release                                                              # RP2350
CARGO_TARGET_THUMBV8M_MAIN_NONE_EABIHF_RUNNER="probe-rs run --chip RP2354" cargo run --release  # RP2354
```

**WARNING:** Do NOT use manual uf2conv.py or objcopy commands!
The Makefile uses `elf2uf2-rs` (takes addresses from ELF sections) + patches Family ID for RP2350/RP2354.
Manual conversion with objcopy loses address info and creates broken firmware.

```bash
# Aliases defined in .cargo/config.toml:
cargo rb        # run release binary
cargo rp2350    # build for RP2350 (ELF only, no UF2)
cargo rp2354    # build for RP2354 (ELF only, no UF2)
```

## Host-Side Tests

`spoof_detector.rs` is pure logic (no embassy/cortex-m deps) and can be tested on the host via a standalone crate in `tests_host/`. Uses a mock `defmt` crate (no-op macros) to satisfy `use defmt::*`.

```bash
make test                              # Run all 70 tests
cd tests_host && cargo test            # Same, directly
cd tests_host && cargo test vuln       # Only regression tests (Mar 2026 vulnerabilities)
cd tests_host && cargo test -- -v      # Verbose output
```

**Structure** (zero firmware code changes):
```
tests_host/
  .cargo/config.toml          # Override target to x86_64 (parent sets thumbv8m)
  Cargo.toml                  # Standalone crate, NOT in workspace
  defmt_mock/                 # Fake defmt: 5 no-op macros (info/warn/error/trace/debug)
  src/lib.rs                  # #[path = "../../src/spoof_detector.rs"] + Debug impl
  tests/test_spoof_detector.rs  # 64 tests, 12 groups
```

**Test groups** (70 tests):
| Group | Tests | Coverage |
|-------|-------|----------|
| 0: Utilities | 8 | GnssTime, calc_distance, FixType |
| 1: Basic flow | 6 | init, warmup, normal flight |
| 2: Teleportation | 5 | >2km threshold, all directions |
| 3: Speed | 2 | 31 vs 29 m/s boundary |
| 4: GNSS time | 6 | forward/backward jumps, warmup suppression |
| 5: Clock drift | 5 | calibration, 10s threshold, recovery |
| 6: Recovery | 9 | coord (5+1 samples), time (immediate), warmup |
| 7: Gap handling | 6 | >5s gap, gap+teleport/drift/time |
| 8: Regressions | 6 | Mar 2026 vulns: immediate detect, last_good guard |
| 9: last_good | 4 | Invariant: frozen during spoof, survives recovery |
| 10: Complex | 4 | Spoof/recovery cycles, realistic attack |
| 11: Reset | 2 | Clean state after reset |
| 12: Bug fixes | 6 | Time recovery+distance, last_good check, origin drift |

**Note**: Coord recovery requires 6 samples (not 5) — returning from spoofed position is itself a teleport that resets normal_count.

## Architecture

### Dual-core async design (Embassy)
- **Core0**: Embassy executor - UART TX/RX, NAV message generation, button handling
- **Core1**: Embassy executor - LED control (PIO), SEC-SIGN ECDSA
- Inter-core communication via `Signal` and `Channel` from `embassy-sync`
- Mode state shared via `AtomicU8` with `Acquire/Release` ordering
- TX_CHANNEL capacity: 32 messages (buffer for SEC-SIGN computation delays)

### Core0 Tasks (src/main.rs)
| Task | Rate | Purpose |
|------|------|---------|
| `uart0_tx_task` | async | Sends UBX messages from TX_CHANNEL (Emulation) or GNSS_RX_CHANNEL (Passthrough), accumulates SHA256 for SEC-SIGN. Drains TX_CHANNEL in all Passthrough modes (ACK/poll responses discarded — real GNSS handles drone commands) |
| `uart0_rx_task` | async | Parses incoming UBX commands, updates MSG_FLAGS |
| `uart1_rx_task` | async | Fast UART1 read, forwards raw chunks to RAW_RX_CHANNEL (minimal processing to prevent overrun) |
| `gnss_processing_task` | async | Parses UBX frames from RAW_RX_CHANNEL, spoof detection, forwards to GNSS_RX_CHANNEL |
| `nav_message_task` | 200ms (5Hz) | Sends NAV-* messages (uses Timer::at for drift-free timing) |
| `mon_message_task` | 1s | Sends MON-HW, MON-RF, MON-COMMS (moved from Core1 to avoid yield_now contention with ECDSA) |
| `sec_sign_timer_task` | 2-4s | Requests SEC-SIGN from Core1, waits via SEC_SIGN_DONE Signal |
| `button_task` | async | Mode selection by click count (1/2/3/4 clicks → Emulation/Passthrough/Raw/Offset) |

### Core1 Tasks
| Task | Rate | Purpose |
|------|------|---------|
| `led_task` (RP2350) | 100ms | WS2812 LED blinking (green/yellow=emulation, blue=passthrough, purple=raw, red=spoof) |
| `simple_led_task` (RP2354) | 50ms | Simple GPIO LED blink code (1-4 blinks = mode number, fast blink = spoof) |
| `sec_sign_compute_task` | async | ECDSA signature computation (CPU intensive) |

### Module Structure
- `src/ubx/` - UBX protocol implementation
  - `mod.rs` - Message classes, IDs, Fletcher checksum
  - `messages.rs` - NAV-PVT, NAV-POSECEF, SEC-SIGN structs
  - `parser.rs` - State machine for parsing incoming UBX frames
- `src/led.rs` - WS2812 driver using PIO (`pio::pio_asm!` macro)
- `src/sec_sign.rs` - SHA256 accumulator for SEC-SIGN authentication
- `src/config.rs` - Pin assignments, timing constants, default position
- `src/coordinates.rs` - LLH→ECEF conversion, cached at startup
- `src/passthrough.rs` - UBX frame parser, position buffer, NAV modification for spoof detection
- `src/spoof_detector.rs` - GPS spoofing detection algorithms (teleportation, speed, altitude, acceleration)
- `src/flash_storage.rs` - Flash persistence for operating mode, extracted keys, extraction requests
- `src/key_extract.rs` - Runtime key extraction from real GNSS module via CFG-0x41 poll

### Coordinate System

Two sets of coordinates in `config.rs`, converted to all required formats at startup:

```rust
// config.rs
pub mod default_position {           // Emulation mode (mode 1)
    pub const LATITUDE: f64 = 25.966443;    // Golden Beach, FL
    pub const LONGITUDE: f64 = -80.122371;
}
pub mod offset_target {              // PassthroughOffset mode (mode 4)
    pub const LATITUDE: f64 = 46.3407;      // Seney, Michigan
    pub const LONGITUDE: f64 = -85.9407;
}
```

The `coordinates` module computes once at init:
- `lat_1e7()`, `lon_1e7()` - Emulation target (deg × 1e-7)
- `offset_lat_1e7()`, `offset_lon_1e7()` - PassthroughOffset target
- `alt_mm()` - altitude in mm
- `ecef_x_cm()`, `ecef_y_cm()`, `ecef_z_cm()` - for NAV-POSECEF, NAV-SOL, NAV-HPPOSECEF

Uses WGS84 ellipsoid parameters for LLH→ECEF conversion.

### Operating Modes
- **Emulation** (0): Generates fake GNSS data with SEC-SIGN authentication (LED green→yellow)
- **Passthrough** (1): Forwards data from real GNSS module with spoof detection (LED blue, blinking red on spoof)
- **PassthroughRaw** (2): Raw passthrough without spoof detection - pure transparent forwarding (LED purple)
- **PassthroughOffset** (3): Passthrough with coordinate offset + spoof detection (LED white, blinking red on spoof)

Mode is persisted to flash and survives reboots. Button selects mode by click count:
- **1 click** → Emulation (green LED)
- **2 clicks** → Passthrough (blue LED)
- **3 clicks** → PassthroughRaw (purple LED)
- **4 clicks** → PassthroughOffset (white LED)

Timeout between clicks: 800ms. Hot-switch without reboot.

### PassthroughOffset Mode

PassthroughOffset works like Passthrough (spoof detection enabled), but applies a dynamic coordinate offset to all NAV messages. The offset is computed **once** at the first 3D GPS fix: `offset = default_position - actual_gps_position`. This means it works from **any location** — no hardcoded source/target offsets needed.

**Target position**: `offset_target` in `config.rs` (Seney, Michigan — separate from Emulation mode's `default_position`)

**Dynamic offset computation** (in `gnss_processing_task`):
1. Wait for first NAV-PVT with `fix_type >= 3` (3D fix) and `num_sv >= 4`
2. Extract actual lat/lon/alt from PVT payload
3. Compute ECEF for actual position via `llh_to_ecef_cm()` (~50µs, one-time)
4. `offset = target (cached at init) - actual` for all 6 components (lat, lon, alt, ecef_x/y/z)
5. Offset locked in `dynamic_offset: Option<DynamicOffset>` — reset on mode switch

**Modified NAV messages**:
| Message | Offsets applied |
|---------|----------------|
| NAV-PVT (0x07) | lon, lat, height, hMSL |
| NAV-POSLLH (0x02) | lon, lat, height, hMSL |
| NAV-POSECEF (0x01) | ecefX, ecefY, ecefZ |
| NAV-HPPOSECEF (0x13) | ecefX, ecefY, ecefZ |
| NAV-SOL (0x06) | ecefX, ecefY, ecefZ |

**Important**: Spoof detection analyzes original (unmodified) coordinates. Offset is applied only to output.

**Offset + spoofing order** (Feb 2026 fix): Offset is applied ALWAYS (even during spoofing), BEFORE spoof modification. This prevents real coordinates from leaking without offset. During spoofing, LAST_GOOD coordinates also get offset applied before being written into NAV messages. Checksum is recalculated only once: after offset (if no spoofing) or after spoof modification (which overwrites offset coordinates anyway).

**SEC-SIGN handling**: In both Passthrough and PassthroughOffset modes, we **always** generate our own SEC-SIGN (real GNSS SEC-SIGN filtered, hash accumulation always enabled, our timer always generates). This eliminates dirty hash problems (per-packet spoof checks causing partial accumulation) and timing gaps (phase mismatch between real GNSS timer and ours on spoof transitions).

**Bug fix (Feb 2026)**: SEC-SIGN rejected in PassthroughOffset mode. Root cause: non-UBX data (NMEA sentences, garbage bytes from GNSS module boot) was forwarded through `GNSS_RX_CHANNEL` and accumulated into SHA256 hash by `uart0_tx_task`. The drone only hashes UBX frames → hash mismatch → signature rejected. Fix: (1) `gnss_processing_task` no longer forwards non-UBX data — `take_non_ubx_data()` drains buffer without sending to channel; (2) `uart0_tx_task` checks for UBX sync bytes (`0xB5 0x62`) before accumulating — non-UBX data is transmitted but not hashed (defense in depth).

### Spoof Detection in Passthrough Mode

**Data flow**: UART1 RX → `uart1_rx_task` → `RAW_RX_CHANNEL` → `gnss_processing_task` (parse + detect + modify) → `GNSS_RX_CHANNEL` → `uart0_tx_task` → UART0 TX

**Algorithm flowchart** (`SpoofDetector::analyze()`, called on every NAV-PVT at 5Hz):

```
                            ┌─────────────────┐
                            │  NAV-PVT frame   │
                            │  from GNSS (5Hz) │
                            └────────┬─────────┘
                                     │
                              ┌──────▼──────┐
                              │ has_3d_fix? │
                              └──┬───────┬──┘
                               NO│       │YES
                    ┌────────────▼┐      │
                    │ Initializing│      │
                    │ (skip frame)│      │
                    └─────────────┘      │
                                  ┌──────▼──────┐
                                  │ first sample?│
                                  └──┬───────┬──┘
                                   YES       │NO
                     ┌─────────────▼─┐       │
                     │ Set prev,     │       │
                     │ last_good,    │       │
                     │ origin        │       │
                     │→ Initializing │       │
                     └───────────────┘       │
                                      ┌──────▼──────┐
                                      │dt > 5s gap? │
                                      └──┬───────┬──┘
                                       YES       │NO
                              ┌────────▼────────┐│
                              │ GAP PATH:       ││
                              │ check vs        ││
                              │ last_good >2km? ││
                              │ clock drift?    ││
                              │ GNSS time jump? ││
                              └──┬───────┬──────┘│
                              ANY│    NONE│       │
                             ┌───▼───┐ ┌──▼────┐  │
                             │Spoofed│ │GapRest│  │
                             │(immed)│ │prev=  │  │
                             └───────┘ │curr   │  │
                                       └───────┘  │
               ┌──────────────────────────────────▼──────────────────────┐
               │                    NORMAL PATH                          │
               │  1. calculate_movement(prev→curr): dist, speed         │
               │  2. check_gnss_time(): time_spoof, time_recovery       │
               │  3. check_system_clock_drift(): drift_spoof, drift_rec │
               └────────────────────────┬────────────────────────────────┘
                                        │
                   ┌────────────────────▼────────────────────┐
                   │ TIME/CLOCK RECOVERY? (spoofed=true)     │
                   │ time_recovery OR clock_drift_recovery   │
                   └────┬──────────────────────────┬────────┘
                      YES                          │NO
               ┌───────▼────────┐                  │
               │dist(last_good, │                  │
               │    curr) < 2km?│                  │
               └──┬──────────┬──┘                  │
                YES        NO│                     │
         ┌──────▼──────┐ ┌──▼───────────┐         │
         │spoofed=false│ │stay spoofed  │         │
         │warmup reset │ │recalibrate   │         │
         │→ Normal     │ │time only     │         │
         └─────────────┘ │→ Spoofed     │         │
                         └──────────────┘         │
                                           ┌──────▼──────────────────────────┐
                                           │ COMPUTE ANOMALY FLAGS:          │
                                           │                                 │
                                           │ coord_anomaly =                 │
                                           │   teleport(prev→curr >2km)      │
                                           │   OR speed(>30 m/s)             │
                                           │   [disabled in recovery warmup] │
                                           │                                 │
                                           │ time_anomaly =                  │
                                           │   time_jump_back(>1s)           │
                                           │   OR time_jump_fwd(>5s)         │
                                           │   OR clock_drift(>10s)          │
                                           │   [disabled in startup warmup]  │
                                           │                                 │
                                           │ last_good_anomaly =             │
                                           │   dist(last_good→curr) > 2km   │
                                           │   [disabled in warmups]         │
                                           │                                 │
                                           │ origin_drift =                  │
                                           │   dist(origin→curr) > 50km     │
                                           │   [disabled in startup warmup]  │
                                           └───────────────┬─────────────────┘
                                                           │
                                                    ┌──────▼──────┐
                                                    │ ANY anomaly?│
                                                    └──┬───────┬──┘
                                                     YES       │NO
                                              ┌───────▼──────┐ │
                                              │ anomaly_cnt++│ │
                                              │ if cnt>=1:   │ │
                                              │ spoofed=true │ │
                                              │ → Spoofed    │ │
                                              └──────────────┘ │
                                                        ┌──────▼──────────────────┐
                                                        │ NORMAL SAMPLE:          │
                                                        │ normal_cnt++            │
                                                        │                         │
                                                        │ if spoofed &&           │
                                                        │   normal_cnt>=5 &&      │
                                                        │   dist(last_good)<2km:  │
                                                        │   spoofed=false         │
                                                        │   recovery_warmup=0     │
                                                        │                         │
                                                        │ if !spoofed &&          │
                                                        │   dist(last_good)<2km:  │
                                                        │   last_good=curr        │
                                                        │                         │
                                                        │ prev=curr               │
                                                        │ → Normal or Spoofed     │
                                                        └─────────────────────────┘
```

**Key state variables:**
- `origin` — first GPS fix, never updated (only on `reset()`), anti-gradient-drift anchor
- `last_good` — last trusted position, updated only when `!spoofed && dist < 2km`
- `prev` — previous sample, always updated (for velocity calculation)
- `last_good_gnss_time` — last trusted GNSS time, updated only when `!spoofed`
- `calibrated_at_system_ms` / `calibrated_gnss_unix` — system clock ↔ GNSS time mapping

**Warmup phases:**
- **Startup warmup** (10 samples, ~2s): time checks disabled (need calibration), coord checks active
- **Recovery warmup** (10 samples, ~2s): coord + last_good checks disabled (GPS re-acquisition jitter), time checks active

**What happens in `gnss_processing_task` after `analyze()` returns:**

| Result | `SPOOF_DETECTED` | NAV modification | LED |
|--------|-------------------|------------------|-----|
| `Normal` | `false` (after 5s timer) | offset only (Mode 4) | blue/white |
| `Spoofed` | `true` (immediate) | coords→LAST_GOOD, vel→0, status degraded, +offset | red blink |
| `Initializing` | unchanged | none | unchanged |
| `GapReset` | unchanged | none | unchanged |

When in passthrough mode, the device parses incoming UBX frames and detects GPS spoofing:

**Active detection algorithms** (in `spoof_detector.rs`):
- **Teleportation**: Position jump > 2km (increased from 500m to reduce false positives during GPS re-acquisition)
- **Speed anomaly**: Ground speed > 30 m/s (108 km/h)
- **GNSS time jump backwards**: Time going backwards > 1 second
- **GNSS time jump forwards**: Unrealistic jump > 5 seconds (matches MAX_GAP_MS)
- **System clock drift**: GNSS time vs calibrated internal clock > 10 seconds
- **Last good distance**: Position > 2km from `last_good` (catches drift after GapReset or time recovery)
- **Origin drift**: Position > 50km from first GPS fix (catches slow gradient attacks that evade per-sample checks)

**Disabled algorithms** (code kept for future use):
- Altitude anomaly: Jump > 10m or vertical speed > 15 m/s
- Acceleration anomaly: Speed change > 20 m/s² (2G)
- CNO uniformity: All satellites with same high CNO (spoof indicator)

**Time-based detection mechanism**:
1. Calibrate system clock with GNSS time (first 5 seconds)
2. Project expected GNSS time using monotonic system clock
3. If incoming GNSS time drifts > 10s from projection → spoofing
4. If drift returns to ≤ 3s → automatic recovery

**On spoofing detected**:
1. Save last good coordinates (2 seconds before spoofing started) including ECEF (computed via `llh_to_ecef_cm()`)
2. Replace coordinates with LAST_GOOD in ALL 5 positional NAV messages (PVT, POSLLH, POSECEF, HPPOSECEF, SOL)
3. Zero velocity in ALL velocity messages (PVT velN/E/D, VELECEF ecefVX/Y/Z, VELNED velN/E/D/speed/gSpeed/heading)
4. Degrade status: `num_sv=92` (impossible value as spoof marker), `fix_type=0`, `flags=0`, high accuracy values (9999999)
5. NAV-STATUS, NAV-SAT, NAV-SVINFO: degrade fix/satellite counts
5. LED: Fast blinking red (200ms cycle)
6. Recalculate Fletcher-8 checksum after modification
7. SEC-SIGN is unaffected — already generated by our timer (always active in Passthrough)

**Recovery**:
- Time-based recovery: Only clears spoofed if GNSS time returns to normal AND position is within 2km of `last_good`. If time ok but coords far → stays spoofed, time is recalibrated (prevents re-trigger). `last_good` is NOT updated on time recovery.
- Coordinate-based recovery: 5 seconds of clean data + position within 2km of last good
- **Recovery warmup** (10 samples, ~2 seconds): After recovery, coordinate anomalies are ignored to prevent false positives from GPS re-acquisition jitter. Time-based checks remain active during warmup.
- SEC_SIGN_ACC hash is NOT reset on spoof recovery (hash is self-consistent: modified NAV bytes are already accumulated). Reset only on mode switch.
- SEC-SIGN continues from our timer (no transition needed — always generating)

**`last_good` invariant** (Mar 2026): `last_good` is ONLY updated when: (1) `!self.spoofed`, AND (2) distance from current `last_good` is < TELEPORT_M (2km). This prevents a spoofed position from ever becoming the reference. Time-based recovery does NOT update `last_good`. `SPOOF_CONFIRM_COUNT = 1` ensures immediate detection on first anomalous frame (no leaked frames).

**`origin` tracking** (Mar 2026): `origin` is set on first GPS fix and never updated (only on `reset()`). Detects slow gradient drift attacks where spoofer moves position < 30 m/s (evading speed check) and < 2km per step (evading teleport check) but accumulates > 50km from the original position over minutes. `MAX_DRIFT_FROM_ORIGIN_M = 50km` (DJI max range ~30km).

**Bug fix (Jan 2026)**: Fixed false positive triggering during GPS re-acquisition after spoofing ends. Previously, `last_good` was overwritten during time recovery, making detector vulnerable to coordinate jumps. Now `last_good` is preserved and recovery warmup ignores coordinate anomalies.

**Bug fix (Jan 2026 #2)**: Fixed false teleport detection when switching from Emulation to Passthrough mode. Added `SPOOF_DETECTOR_RESET` atomic flag that triggers detector reset via `apply_mode_by_clicks()`. This prevents the detector from seeing emulation coordinates as the previous position when real GNSS data starts arriving.

**Bug fix (Mar 2026)**: Spoof detection bypassed during GPS signal gap. When a spoofer captures the GNSS module, it typically causes a brief loss of 3D fix (fix_type < 3). The detector ignores samples without 3D fix. If this gap exceeds 5 seconds (MAX_GAP_MS), the `GapReset` path fired — it updated `prev` to the spoofed position and returned **without checking teleportation, GNSS time, or clock drift**. Result: spoofed coordinates were silently accepted as the new reference. The next sample compared against the spoofed position → distance=0 → Normal. Detection only triggered later via clock drift (10+ seconds). Bug existed since the first spoof_detector commit (56f28f7). Fix: during gap, check distance from `last_good` (teleportation), system clock drift, and GNSS time jump. If any anomaly detected → immediate spoofing confirmation (no consecutive count required). Affects both Passthrough (mode 2) and PassthroughOffset (mode 4).

**Bug fix (Mar 2026 #2)**: Three vulnerabilities allowed spoof bypass: (1) `SPOOF_CONFIRM_COUNT=2` leaked 1 frame and corrupted `last_good` via `prev` update; (2) time-based recovery unconditionally overwrote `last_good` with current (possibly spoofed) position; (3) `last_good` updated without distance check, allowing spoofed position to become the reference. Fix: `SPOOF_CONFIRM_COUNT=1` (immediate detection), removed `last_good` update from time recovery, added distance guard (`< TELEPORT_M`) on `last_good` updates.

**Bug fix (Mar 2026 #3)**: Four interrelated vulnerabilities in spoof detection: (1) Time recovery cleared `spoofed` even when coordinates were >2km from `last_good` — spoofer could return time to normal while keeping fake coords, causing "alternating coordinates" effect. Fix: time recovery requires proximity to `last_good` (<2km), otherwise stays spoofed with time recalibration. (2) No `last_good → curr` distance check — only `prev → curr` was checked, so after GapReset or time recovery, `prev` at spoofed position made `dist≈0 → Normal`. Fix: added `last_good_anomaly` check (>2km from `last_good` after warmup). (3) Slow gradient attack evaded all per-sample checks by keeping speed <30 m/s and distance <2km per step, gradually moving `last_good` kilometers away. Fix: added `origin` position (first fix, never updated except reset) with `MAX_DRIFT_FROM_ORIGIN_M=50km` check. (4) NAV-VELECEF (0x11) and NAV-VELNED (0x12) were not zeroed during spoofing — leaked real velocity while NAV-PVT showed zero velocity. Fix: added `modify_nav_velecef_spoof` and `modify_nav_velned_spoof` functions.

**Known bug (Jan 2026)**: SEC-UNIQID race condition in drone auto-detection. When Mavic 4 Pro connects, it sends SEC-UNIQID poll BEFORE CFG-VALSET, but auto-detection triggers on CFG-VALSET. Result: Mavic 4 receives Air 3's chip ID (`0xE0 0x95 0x65 0x0F 0x2A` instead of `0xEB 0xB9 0x91 0x0F 0x2B`), causing signature rejection. **Workaround**: Set default `DRONE_MODEL` to target model in `main.rs` (currently Air 3S = 2). Auto-detection is skipped when Air 3S or Mavic 3 Pro is set manually.

**Global state** (atomics in `main.rs`):
- `SPOOF_DETECTED: AtomicBool` - spoofing active flag
- `SPOOF_RECOVERY_START_MS: AtomicU32` - recovery timer start
- `LAST_GOOD_LAT/LON/ALT: AtomicI32` - saved good coordinates (initialized with default position from config at startup)
- `LAST_GOOD_ECEF_X/Y/Z: AtomicI32` - saved good ECEF coordinates in cm (computed via `llh_to_ecef_cm()`)
- `SEC_SIGN_ACC_NEEDS_RESET: AtomicBool` - deferred SEC_SIGN_ACC reset (set by mode switch, consumed by `sec_sign_timer_task`)

**Implementation** (in `passthrough.rs`):
- `UbxFrameParser` - state machine for UBX frame parsing
- `PositionBuffer` - ring buffer for 3 seconds of position history (15 samples at 5Hz)
- `modify_nav_pvt/sol/status/sat/svinfo` - in-place NAV status degradation
- `modify_nav_pvt_spoof/posllh_spoof/posecef_spoof/hpposecef_spoof/sol_spoof` - coordinate replacement with LAST_GOOD + status degradation
- `modify_nav_velecef_spoof/velned_spoof` - velocity zeroing + accuracy degradation during spoofing
- `recalc_checksum` - Fletcher-8 recalculation

**NAV messages modified during spoofing**:
| Message | Size | Fields replaced/degraded |
|---------|------|--------------------------|
| NAV-PVT | 92B | lon, lat, height, hMSL → LAST_GOOD; velN/E/D → 0; fix_type=0, flags=0, num_sv=92 |
| NAV-POSLLH | 28B | lon, lat, height, hMSL → LAST_GOOD; hAcc/vAcc → 9999999 |
| NAV-POSECEF | 20B | ecefX/Y/Z → LAST_GOOD_ECEF; pAcc → 9999999 |
| NAV-HPPOSECEF | 28B | ecefX/Y/Z → LAST_GOOD_ECEF; Hp → 0; invalidEcef flag; pAcc → 9999999 |
| NAV-SOL | 52B | ecefX/Y/Z → LAST_GOOD_ECEF; ecefVX/Y/Z → 0; gps_fix=0, num_sv=92; pAcc → 9999999 |
| NAV-VELECEF | 20B | ecefVX/Y/Z → 0; sAcc → 9999999 |
| NAV-VELNED | 36B | velN/E/D → 0; speed/gSpeed → 0; heading → 0; sAcc/cAcc → 9999999 |
| NAV-STATUS | 16B | gps_fix=0, flags=0 |
| NAV-SAT | 8+12n | num_svs=92 |
| NAV-SVINFO | 8+12n | num_ch=92 |

### NAV Output Start Timing

NAV messages start after a model-specific delay from the first UBX command (any command: MON-VER poll, CFG-VALSET, etc.):

| Model | Real Timing | Config Delay |
|-------|-------------|--------------|
| Air 3 | 666ms | 700ms |
| Air 3S | 780ms | 780ms |
| Mavic 4 Pro | 399ms | 400ms |
| Mavic 3 Pro | ~780ms (est.) | 780ms |

```
First UBX command → +delay → NAV output starts → +first_delay → First SEC-SIGN
```
First delay: 1000ms for Air 3, 650ms for Air 3S/Mavic 4 Pro/Mavic 3 Pro.

Implementation:
- `FIRST_CONFIG_MILLIS` records timestamp of first command
- `nav_message_task` gets delay based on `DRONE_MODEL` AtomicU8
- `CONFIG_TO_NAV_AIR3_MS = 700`, `CONFIG_TO_NAV_AIR3S_MS = 780`, `CONFIG_TO_NAV_MAVIC4_MS = 400` in config.rs

### 20-Second Invalid Satellites Timer

After 20 seconds from NAV output start, satellites become invalid to simulate signal loss:

| Message | Invalid State |
|---------|---------------|
| NAV-PVT | fix_type=0, flags=0, num_sv=1 |
| NAV-STATUS | gps_fix=0, flags=0 |
| NAV-SOL | gps_fix=0, num_sv=1 |
| NAV-SAT | 1 satellite, cno=8 dBHz, not used |
| NAV-SVINFO | 1 satellite, low quality |

Timer resets **only** on:
- Mode switch from Passthrough → Emulation (button)

Timer does **NOT** reset on:
- CFG-RST command from drone

Implementation: `OUTPUT_START_MILLIS` (AtomicU32) + `wrapping_sub` for overflow safety.

### Passthrough Implementation

**Current implementation (algos-v2):**
- Uses UART1 RX (GPIO5) for receiving external GNSS data
- Data flows: UART1 RX → `uart1_rx_task` → `RAW_RX_CHANNEL` → `gnss_processing_task` → `UbxFrameParser` → `GNSS_RX_CHANNEL` → `uart0_tx_task` → UART0 TX
- `uart1_rx_task` and `gnss_processing_task` split for overrun prevention (UART read must not block on frame parsing)
- `RAW_RX_CHANNEL`: 256-byte chunks, depth 64 (~16KB buffer for burst absorption)
- Allows frame-by-frame parsing for spoof detection and NAV modification

**Legacy PIO passthrough (in passthrough.rs, NOT USED):**
- `struct Passthrough` exists but is not instantiated in main.rs
- Was designed for pure bit-level copying GPIO3→GPIO0 via PIO
- PIO clock: 8MHz (~8 samples per bit at 921600 baud)
- Could be used for future ultra-low-latency transparent mode:
```asm
.wrap_target
    wait 1 pin 0    ; wait for input high
    set pins, 1     ; set output high
    wait 0 pin 0    ; wait for input low
    set pins, 0     ; set output low
.wrap
```

### Packet Loss Fix in Passthrough Mode (Jan 2026)

**Original problem**: ~57.5% packet loss for RXM-RAWX, ~10% overall loss.

**Root causes identified and fixed**:

1. **RXM-RAWX header loss** (commit 6d0fd4f):
   - Large packets (720+ bytes) lost UBX headers due to `take_non_ubx_data()` called mid-frame
   - Fix: Added `is_idle()` to `UbxFrameParser`, only drain non-UBX when in WaitSync1 state

2. **GNSS_RX_CHANNEL overflow during ECDSA** (commit eec6e24):
   - `SEC_SIGN_IN_PROGRESS` blocked TX while Core1 computed signature (~700ms originally)
   - Fix: Local buffering in `uart0_tx_task` using `select()` pattern - buffer packets during wait

3. **Slow ECDSA computation** (commit a61110d, extended Jan 2026):
   - `opt-level="z"` made p192 crypto very slow (~700ms per signature)
   - Fix: Added `opt-level=3` for crypto crates in Cargo.toml:
     - p192, sha2, hmac, subtle (original)
     - crypto-bigint, elliptic-curve, primeorder, ff, group, digest, block-buffer (Jan 2026)
   - Added `#[inline(always)]` to `compute_z()`, `generate_k()`, `ct_option_to_option()`
   - Result: ECDSA ~59ms (was ~67ms, originally ~700ms)

4. **Channel depth insufficient** (commit 50cf77b):
   - 64 slots still caused ~0.13% loss during packet bursts
   - Fix: Increased `GNSS_RX_CHANNEL` depth to 128

**Final test results** (307 seconds, 19547 packets):
| Metric | Before | After |
|--------|--------|-------|
| RXM-RAWX loss | 57.5% | **0.07%** |
| Overall loss | ~10% | **0.03%** (6 packets) |
| ECDSA time | ~700ms | **~59ms** |
| SEC-SIGN | unstable | **stable 4.00s** |

Remaining 6 lost packets: MON-VER(1), NAV-SAT(1), NAV-STATUS(1), RXM-RAWX(1), RXM-SFRBX(2) - acceptable for Passthrough mode.

**Key code changes**:
```rust
// uart0_tx_task: local buffering during SEC-SIGN wait
if SEC_SIGN_IN_PROGRESS.load(Ordering::Acquire) {
    let mut pending: heapless::Vec<heapless::Vec<u8, 1280>, 64> = heapless::Vec::new();
    loop {
        match select(SEC_SIGN_RESULT.receive(), GNSS_RX_CHANNEL.receive()).await {
            Either::First(result) => { /* send signature */ break; }
            Either::Second(msg) => { pending.push(msg); }
        }
    }
    // Send buffered packets AFTER signature
}
```

**Warning**: Do NOT use `try_lock()` instead of `lock().await` for SEC_SIGN_ACC - it breaks signature verification by skipping hash accumulation.

**Bug fix (Feb 2026)**: SEC_SIGN_ACC not reset on mode switch or spoof recovery. Previously, `apply_mode_by_clicks()` didn't reset the hash accumulator, so the first SEC-SIGN after mode switch contained stale hashes → invalid signature. Spoof recovery used `try_lock()` which silently skipped reset when mutex was held by `uart0_tx_task`. Fix: `SEC_SIGN_ACC_NEEDS_RESET` atomic flag — set by mode switch and spoof recovery, consumed by `sec_sign_timer_task` before mode-skip checks (ensures reset even in modes that normally skip SEC-SIGN generation).

### UART Overrun Fix (Jan 2026) — 100% RELIABILITY ACHIEVED

**Problem diagnosed**: 0.02-0.05% packet loss from UART hardware overrun (NOT software channels).

**Root cause**: UART1 16-byte FIFO fills in ~175µs at 921600 baud. Embassy-rp defaults to 7/8 full (14 bytes) trigger → only ~152µs between interrupts → insufficient for processing.

**Solution**: Reduce FIFO RX threshold from 7/8 to 1/4 using `rp_pac` register access:
```rust
// After BufferedUart::new():
rp_pac::UART1.uartifls().write(|w| {
    w.set_rxiflsel(0b001); // 1/4 full (4 bytes) - was 0b100 (14 bytes)
    w.set_txiflsel(0b000); // TX unchanged
});
```

**Test results** (commit 23741d0):
| Metric | Before | After |
|--------|--------|-------|
| Overrun errors | 4-10 per 5 min | **0** |
| Test duration | 5 minutes | 5 minutes |
| Packets processed | 18281 | 18538 |
| ch_drops / buf_drops | 0 / 0 | 0 / 0 |
| **Result** | 0.02-0.05% loss | **100% reliability** |

**Why it works**: 1/4 threshold (4 bytes) triggers interrupt every ~44µs (3.4x more often), giving ample time to empty FIFO before overflow.

**Alternative solutions researched but not needed**:
- PIO UART: embassy-rp 0.9 has `PioUartRxProgram`, 32-byte FIFO possible
- DMA Ring Buffer + Watchdog Timer: 100% guaranteed but complex (see plan file)

## Hardware Pins

### RP2350A — SpotPear RP2350-Core-A (default)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud, к дрону/хосту)
- UART1: RX=GPIO5 (от внешнего GNSS для passthrough)
- WS2812B LED: GPIO25 (PIO0)
- Mode button: GPIO14 (input), GPIO13 (power)

### RP2354A (`--features rp2354`)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud)
- UART1: RX=GPIO5 (passthrough)
- Simple GPIO LED: GPIO11 (анод), GPIO12 (катод/земля) — blink code индикация
- Mode button: GPIO14 (input), GPIO13 (power)

### Simple LED Blink Code (RP2354)
Количество вспышек = номер режима (цикл ~1.5 сек):
| Режим | Вспышек | Паттерн |
|-------|---------|---------|
| Emulation (1) | 1 | 50ms ON, 1400ms OFF |
| Passthrough (2) | 2 | 50ms ON, 150ms OFF, 50ms ON, 1200ms OFF |
| PassthroughRaw (3) | 3 | 3× (50ms ON, 150ms OFF), затем пауза |
| PassthroughOffset (4) | 4 | 4× (50ms ON, 150ms OFF), затем пауза |

При спуфинге (Passthrough/PassthroughOffset): быстрое мигание 50ms ON / 50ms OFF.

## Key Dependencies
- `embassy-rp 0.9` - RP2350 HAL
- `embedded-io-async 0.6` - Must match embassy-rp version exactly
- `pio 0.3` - For `pio_asm!` macro (not 0.2)
- `p192` + `sha2` - ECDSA SECP192R1 for SEC-SIGN

## UBX Protocol Notes
- All messages use 0xB5 0x62 sync header
- Checksum: Fletcher 8-bit over class, id, length, payload
- SEC-SIGN uses SHA256 hash folded to 24 bytes, signed with SECP192R1

## SEC-SIGN Cryptography

**Private keys** location: `src/sec_sign.rs`

| Drone Model | Constant | First Delay | SEC-SIGN Period |
|-------------|----------|-------------|-----------------|
| DJI Air 3 | `PRIVATE_KEY_AIR3` | 1000ms | 4 seconds |
| DJI Air 3S | `PRIVATE_KEY_AIR3S` | 650ms | 2 seconds |
| DJI Mavic 4 Pro | `PRIVATE_KEY_MAVIC4PRO` | 650ms | 2 seconds |
| DJI Mavic 3 Pro | `PRIVATE_KEY_MAVIC3PRO` | 650ms | 2 seconds |

Model selection: `DRONE_MODEL` static variable in `main.rs` (0=Air3, 1=Mavic4Pro, 2=Air3S, 3=Mavic3Pro). Default: Air 3S (2). Auto-detection skipped when Air 3S or Mavic 3 Pro is set manually.

**Note**: Mavic 4 Pro has two known private keys (different units have different keys):
- Key 1 (hardcoded): `90 89 a2 18 14 a6 2f c3 3a f5 d6 eb 61 16 1c e1 86 36 f5 48 d0 71 d6 9f`
- Key 2: `55 14 01 BC 89 05 7B E6 3C 84 13 FA 37 30 7F 88 B4 BE 42 73 3A AD FF DA`
- Key 3: `1F DE D3 9B 4F F6 98 A1 95 D9 02 A2 69 0F 9A BB 8B 48 74 F9 B5 74 89 66`

Each physical GNSS module has its own unique key burned into OTP. Use key extraction to get the correct key for a specific unit.

**Implementation**: Pure Rust using `p192` crate primitives (no C dependencies)

Algorithm:
1. Accumulate all transmitted UBX messages in SHA256 hasher
2. Compute `z = fold(SHA256(sha256_field || session_id))` - fold 32→24 bytes via XOR
3. Generate deterministic `k` using HMAC-SHA256 (simplified RFC6979)
4. Compute R = k * G, r = R.x mod n
5. Compute s = k^(-1) * (z + r * d) mod n
6. Output: 48-byte signature (r=24, s=24) in UBX-SEC-SIGN message

Key crates: `p192` (elliptic curve), `sha2` (hashing), `hmac` (deterministic k)

### TX Pause During SEC-SIGN Computation (Dec 2025 fix)

**Problem**: Race condition where packets sent AFTER hash capture but BEFORE SEC-SIGN TX are not included in the signature hash → verification fails on receiver side.

**Solution**: `SEC_SIGN_IN_PROGRESS` atomic flag coordinates TX while SEC-SIGN is being computed:

```
sec_sign_timer_task: set SEC_SIGN_IN_PROGRESS=true → capture hash → signal Core1
nav_message_task:    check flag → wait with yield_now() until false
mon_message_task:    check flag → wait with yield_now() until false
uart_tx_task:        check flag → wait for SEC-SIGN result → send → clear flag
Core1:               compute ECDSA → signal result
```

Critical synchronization points:
- `sec_sign_timer_task` sets `SEC_SIGN_IN_PROGRESS = true` BEFORE capturing hash
- `nav_message_task` and `mon_message_task` use cooperative `yield_now()` loop to wait (like C version's busy-wait, prevents packet drops)
- `uart_tx_task` waits for `SEC_SIGN_RESULT` signal when flag is set
- Flag is cleared by `uart_tx_task` AFTER sending SEC-SIGN message

**Bug fix (Feb 2026)**: SEC-SIGN timer misses in Passthrough modes. `sec_sign_timer_task` uses `Ticker::every(2000ms)` but blocked on `SEC_SIGN_ACC.lock().await` while `uart0_tx_task` held the mutex during slow UART `write_all()` (~10-50ms per packet). Result: only 55% of SEC-SIGN at 2s interval, mean 3.78s (all intervals exact multiples of 2s: 250×2s, 104×4s, 44×6s...). Fix: reorder operations in `uart0_tx_task` — `write_all()` first (no lock), then `lock().await` + `accumulate()` (~microseconds). Safe because `SEC_SIGN_IN_PROGRESS` flag already prevents other tasks from sending during hash capture.

**Bug fix (Feb 2026 #2)**: Same lock-during-write issue existed in Emulation mode. `uart0_tx_task` held `SEC_SIGN_ACC` lock during `write_all()` (1-5ms per message, ~15-20 messages per 200ms NAV cycle → 30-60ms lock contention per cycle). Over 2s SEC-SIGN period: 15-30% probability timer hits locked mutex. Fix: same reorder as Passthrough — `write_all()` first, then brief `lock().await` + `accumulate()`. Also moved `mon_message_task` from Core1 to Core0 to eliminate `yield_now()` contention with `sec_sign_compute_task` on Core1.

**Hash accumulation**: ALL transmitted UBX messages are accumulated except SEC-SIGN itself (0x27, 0x04). This includes:
- All NAV-* messages (PVT, POSECEF, STATUS, DOP, SAT, EOE, etc.)
- All MON-* messages (HW, RF, COMMS, VER)
- All ACK-ACK responses to CFG commands
- SEC-UNIQID (0x27, 0x03) - IS included in hash
- TIM-TP, RXM-RAWX, RXM-SFRBX

## CFG-0x41 (OTP Configuration / DJI Proprietary)

CFG-0x41 is u-blox's **OTP (One-Time Programmable)** configuration command for M10 modules.
DJI extended this format for SEC-SIGN private key storage and ROM patches.

**Standard u-blox OTP format**:
```
B5 62 06 41 [len] 04 01 A4 [size] [hash:4] 28 EF 12 05 [config_data] [checksum]
```
- `04 01 A4` — OTP header
- `28 EF 12 05` — constant marker in all OTP messages
- Source: https://github.com/cturvey/RandomNinjaChef/tree/main/uBloxM10OTPCodes

**DJI extension**: Poll (0x06, 0x41) with zero-length payload returns 256-byte response with SEC-SIGN config.

**Payload structure (256 bytes) - detailed breakdown**:

| Section | Offset | Size | Description |
|---------|--------|------|-------------|
| 1. Bitmasks | 0 | 26 | Signal enable bitmasks |
| 2. ROM Patch #1 | 26 | 28 | file 0x82, ARM Thumb-2 code |
| 3. ROM Patch #2 | 54 | 42 | file 0x83, ARM Thumb-2 code |
| 4. CFG-SIGNAL | 96 | ~20 | group 0x31, signal config |
| 5. CFG-RINV | ~116 | ~50 | group 0xC7, Remote Inventory (Air 3/Mavic 4 Pro only, absent in Air 3S/Mavic 3 Pro) |
| 6. SEC/KEY | ~166 | 26 | group 0xA6, **Private Key** (offset 175 or 115 for Air 3S/Mavic 3 Pro) |
| 7. CFG-UART1 | ~192 | 10 | group 0x52, baudrate |
| 8. CFG-CLOCK | ~202 | 40 | group 0xA4, clock frequencies |
| 9. Padding | ~242 | 14 | 0xFF fill |

**Section 5 - CFG-RINV (Remote Inventory)**:
```
C7 10 01                              ← DUMP = 1
03 00 C7 20 1E                        ← SIZE = 30 bytes
04 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA0 (8 bytes)
05 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA1 (8 bytes)
06 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA2 (8 bytes)
07 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA3 (8 bytes)
```

**Section 6 - SEC/KEY (Private Key)**:
```
A6 18                                 ← Group tag + length
xx xx xx xx xx xx xx xx xx xx xx xx   ← Private Key P-192
xx xx xx xx xx xx xx xx xx xx xx xx   ← (24 bytes, big-endian)
```

**Section 8 - CFG-CLOCK frequencies**:
```
A4 20 01
00 A4 40 00 B0 71 0B                  ← item 00 = 192 MHz
03 00 A4 40 00 B0 71 0B               ← item 03 = 192 MHz
05 00 A4 40 00 B0 71 0B               ← item 05 = 192 MHz
0A 00 A4 40 00 D8 B8 05               ← item 0A = 96 MHz
```

**Implementation**: `src/ubx/messages.rs` → `Cfg41`, `cfg41_templates`
- `PRIVATE_KEY_OFFSET = 175` - where key is inserted in template (Air 3, Mavic 4 Pro)
- `PRIVATE_KEY_OFFSET_AIR3S = 115` - key offset for Air 3S and Mavic 3 Pro (no CFG-RINV section)
- `TEMPLATE` captured from real Mavic 4 Pro GNSS module
- `TEMPLATE_AIR3S` captured from real Air 3S GNSS module (no CFG-RINV, more FF padding)
- `TEMPLATE_MAVIC3PRO` captured from real Mavic 3 Pro GNSS module (no CFG-RINV, no CFG-UART1/CLOCK, unique bitmask)

**Security note**: This command exposes the private key, allowing key extraction from real DJI GNSS modules.

## Runtime Key Extraction

Extracts the 24-byte SEC-SIGN private key from a real u-blox GNSS module connected to the board. Uses CFG-0x41 poll command to read OTP configuration containing the key.

**Trigger**: Long button press (3+ seconds) during normal operation → saves flash flag → reboots → extraction runs at boot BEFORE UART init.

**Hardware connections for extraction**:
- TX: PIO1 SM0 on GPIO1 (to GNSS module RX) — 921600 baud
- RX: Hardware UART1 on GPIO5 (from GNSS module TX) — BufferedUartRx, 1KB buffer, FIFO 1/4

**Why Hardware UART1 RX (not PIO RX)**: PIO UART RX lost ~99.6% of bytes at 921600 baud. The async `read_u8().await` per-byte approach caused PIO FIFO overflow — 264-byte response took 735ms instead of 2.86ms with corrupted data. Hardware UART1 with DMA-backed BufferedUartRx handles the full data rate reliably.

**Algorithm** (`src/key_extract.rs`):
1. Init PIO1 TX on GPIO1, Hardware UART1 RX on GPIO5
2. Wait 2s for GNSS module to stabilize after power-on
3. Flush RX buffer (GNSS outputs NMEA at startup)
4. Loop up to 5 attempts:
   - Send `B5 62 06 41 00 00 47 DB` (CFG-0x41 poll, 8 bytes)
   - Collect up to 1024 bytes with 2s timeout (offline into buffer)
   - Search buffer for UBX frame `B5 62 06 41 00 01` (class=0x06, id=0x41, len=256)
   - Validate Fletcher-8 checksum
   - Extract key: try offset 113 (Air3S/Mavic3Pro), then 173 (Air3/Mavic4Pro), then fallback scan for `A6 18` tag
5. Return `Some(key)` or `None`

**Key storage** (`src/flash_storage.rs`):
- Extracted key saved to flash (Last-1 sector, magic `0x4B455953` = "KEYS")
- Loaded on every subsequent boot — overrides hardcoded keys in `sec_sign.rs`
- If no key in flash → fallback to hardcoded `PRIVATE_KEY_*` by `DroneModel`

**LED feedback after extraction**:
| Board | Success | Failure |
|-------|---------|---------|
| RP2350 (WS2812) | 5x green blink (100ms) | 5x red blink (100ms) |
| RP2354 (GPIO) | 5x blink (100ms) | 5x blink (100ms) |

Normal startup (no extraction): 3x green blink (200ms) as usual.

**Flash layout for extraction**:
| Sector | Content | Magic |
|--------|---------|-------|
| Last-1 | Extracted key (24 bytes) | `0x4B455953` ("KEYS") |
| Last-4 | Extraction request flag | `0x45585452` ("EXTR") |

**Debug flag**: `FORCE_KEY_EXTRACT` in `main.rs` (default `false`) — forces extraction on every boot, bypassing button/flash mechanism. For SWD debugging only.

**Error handling**: If GNSS not connected or doesn't respond → 5 attempts × 2s = ~12s delay at boot → `None` → program continues with hardcoded or previously saved key. Never blocks indefinitely.

## Diagnostic Mode (Per-Message Counters)

Built-in diagnostic for analyzing UBX message flow in Passthrough modes. Disabled by default.

**Enable**: Set `DIAG_MSG_DETAIL = true` in `src/main.rs` (~line 167), rebuild and flash.

**What it provides** (output in `diag_stats_task` every 10 seconds):
- Per-message-type RX counters (from GNSS on UART1): NAV-PVT, NAV-POSLLH, etc.
- Per-message-type TX counters (to drone on UART0)
- CFG-VALSET key=value logging (each key printed as `key={:#010x} val={}`)
- Stored VALSET buffer (256 keys) — captures keys from boot, dumps on first diag cycle (survives probe-rs late attach)
- VALSET packet counter in DIAG line

**Modules** (in `src/main.rs`):
- `diag_msg_counts` — 28 known UBX message types, atomic RX/TX counters, `log_and_reset()`
- `diag_valset_store` — static buffer for CFG-VALSET keys, one-time dump via `log_stored()`

**Air 3S baseline** (Feb 2026, verified):
- 32 CFG-VALSET packets, 39 unique keys at boot
- Drone enables: PVT, POSLLH, STATUS, DOP, VELNED, TIMEGPS, TIMEUTC, CLOCK, SAT, AOPSTATUS, MON-RF, RXM-RAWX, TIM-TP, NAV-SOL
- Drone disables: NAV-POSECEF
- TIM-TP and NAV-SOL: enabled by drone but GNSS module doesn't generate them (not in OTP)
- Steady state: RX=TX for all types, 0% loss

## RAM/Flash Usage Comparison (vs C/FreeRTOS)

| Metric | C/FreeRTOS | Rust/Embassy |
|--------|------------|--------------|
| text (code) | 55.8 KB | **97.3 KB** |
| bss (RAM) | **133 KB** | **13.5 KB** |

Notes:
- Code size increased due to pure Rust P-192 elliptic curve arithmetic (vs C micro-ecc)
- RAM usage remains 10x lower - stackless coroutines vs FreeRTOS task stacks
- Trade-off: larger code, no C dependencies, fully auditable Rust crypto

## Dynamic Configuration

NAV message rate and UART baudrate can be changed at runtime via:

### CFG-RATE (0x06, 0x08)
- `meas_rate` (u16): Measurement period in ms (50-10000)
- `nav_rate` (u16): Cycles per navigation solution (1-127)
- `time_ref` (u16): Time reference (0=UTC, 1=GPS, etc.)

Effective NAV period = `meas_rate × nav_rate`

### CFG-VALSET (0x06, 0x8A) Keys
| Key | Description | Type |
|-----|-------------|------|
| `0x30210001` | CFG-RATE-MEAS | u16 |
| `0x30210002` | CFG-RATE-NAV | u16 |
| `0x20210003` | CFG-RATE-TIMEREF | u8 |
| `0x40520001` | CFG-UART1-BAUDRATE | u32 |
| `0x20910007` | NAV-PVT enable | u8 |
| `0x2091001B` | NAV-STATUS enable | u8 |
| ... | (see `valset_keys` module in main.rs) | |

### Baudrate Change Implementation
Uses direct UART0 register access via `rp-pac` crate (embassy-rp doesn't expose baudrate change on BufferedUart).

## Implementation Status

All core features complete:

1. ✅ NAV messages - All 18 NAV message types implemented (including NAV-SOL)
2. ✅ MON messages - MON-HW, MON-COMMS, MON-RF, MON-VER (poll)
3. ✅ Passthrough mode - UART1 RX with frame parsing, spoof detection, flash persistence
4. ✅ ECDSA signing - Pure Rust p192 with RFC6979 deterministic k
5. ✅ CFG-PRT baudrate - Direct PAC register access
6. ✅ CFG-VALSET - Full M10 configuration support
7. ✅ ACK-ACK/ACK-NAK - Command acknowledgement
8. ✅ SEC-UNIQID - Unique ID poll response
9. ✅ CFG-0x41 - DJI proprietary SEC-SIGN config (256-byte response with private key)
10. ✅ MGA-* - AssistNow assistance data (ACK-ACK response)
11. ✅ Key extraction - Runtime private key extraction from real GNSS via CFG-0x41
12. ✅ Release build - 0 warnings, pure Rust

## Implemented UBX Messages

### NAV Class (0x01) - Navigation
| ID | Name | Payload | Description |
|----|------|---------|-------------|
| 0x01 | NAV-POSECEF | 20 | Position in ECEF |
| 0x02 | NAV-POSLLH | 28 | Position in LLH |
| 0x03 | NAV-STATUS | 16 | Receiver status |
| 0x04 | NAV-DOP | 18 | Dilution of precision |
| 0x06 | NAV-SOL | 52 | Navigation solution (legacy) |
| 0x07 | NAV-PVT | 92 | Position/Velocity/Time |
| 0x11 | NAV-VELECEF | 20 | Velocity in ECEF |
| 0x12 | NAV-VELNED | 36 | Velocity in NED |
| 0x13 | NAV-HPPOSECEF | 28 | High precision ECEF |
| 0x20 | NAV-TIMEGPS | 16 | GPS time solution |
| 0x21 | NAV-TIMEUTC | 20 | UTC time solution |
| 0x22 | NAV-CLOCK | 20 | Clock solution |
| 0x26 | NAV-TIMELS | 24 | Leap second info |
| 0x30 | NAV-SVINFO | 8+12n | Satellite info (legacy) |
| 0x35 | NAV-SAT | 8+12n | Satellite info (M10) |
| 0x36 | NAV-COV | 64 | Covariance matrices |
| 0x60 | NAV-AOPSTATUS | 16 | AssistNow status |
| 0x61 | NAV-EOE | 4 | End of epoch |

### MON Class (0x0A) - Monitoring
| ID | Name | Payload | Description |
|----|------|---------|-------------|
| 0x04 | MON-VER | 160 | Version info (poll) |
| 0x09 | MON-HW | 60 | Hardware status |
| 0x36 | MON-COMMS | 8 | Communication port info |
| 0x38 | MON-RF | 24 | RF information |

### Other Classes
| Class | ID | Name | Payload | Description |
|-------|-----|------|---------|-------------|
| 0x02 | 0x13 | RXM-SFRBX | 8+n | Subframe buffer (passthrough only) |
| 0x02 | 0x15 | RXM-RAWX | 16+ | Raw measurements |
| 0x05 | 0x00 | ACK-NAK | 2 | Negative acknowledgement |
| 0x05 | 0x01 | ACK-ACK | 2 | Acknowledgement |
| 0x06 | 0x41 | CFG-0x41 | 256 | DJI proprietary (private key at offset 175) |
| 0x0D | 0x01 | TIM-TP | 16 | Timepulse |
| 0x13 | * | MGA-* | var | AssistNow data (ACK-ACK response) |
| 0x27 | 0x03 | SEC-UNIQID | 10 | Unique ID (poll) |
| 0x27 | 0x04 | SEC-SIGN | 108 | Signature |

## Embassy 0.9 API Notes

- `BufferedUartTx` / `BufferedUartRx` have no lifetime parameters (changed from 0.8)
- `Signal::try_get()` doesn't exist - use `AtomicU8` for shared mode state
- PIO pin wrapping: `Peri<'d, PIN>` not raw pin type
- Interrupt binding: `bind_interrupts!` macro required
