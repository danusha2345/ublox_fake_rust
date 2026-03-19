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

## Build Commands

**CRITICAL: Always use Makefile to build UF2 files!** Manual objcopy loses address info → broken firmware.

```bash
PATH="/home2/.cargo/bin:$PATH"       # Rust may not be in PATH

make rp2350                           # Build UF2 for RP2350
make rp2354                           # Build UF2 for RP2354
make flash                            # Flash via probe-rs (RP2350)
make flash-rp2354                     # Flash via probe-rs (RP2354)
make test                             # Run all 75 host-side tests

cargo rb                              # run release binary (alias)
cargo rp2350                          # build ELF only (alias)
cargo rp2354                          # build ELF only (alias)
```

## Host-Side Tests

`spoof_detector.rs` is pure logic (no embassy/cortex-m deps) and can be tested on host via `tests_host/`.

```bash
cd tests_host && cargo test            # All 75 tests
cd tests_host && cargo test vuln       # Only regression tests
```

**Structure**: `tests_host/` is a standalone crate (NOT in workspace) with `defmt_mock/` (no-op macros) and `#[path]` to `../../src/spoof_detector.rs`.

**Test groups** (75 tests):
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
| 13: Leash+altitude | 5 | Gradual drift, leash freeze, altitude jump |

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
| `uart0_tx_task` | async | Sends UBX messages from TX_CHANNEL (Emulation) or GNSS_RX_CHANNEL (Passthrough), accumulates SHA256 for SEC-SIGN |
| `uart0_rx_task` | async | Parses incoming UBX commands, updates MSG_FLAGS |
| `uart1_rx_task` | async | Fast UART1 read → RAW_RX_CHANNEL (minimal processing to prevent overrun) |
| `gnss_processing_task` | async | Parses UBX frames, spoof detection, forwards to GNSS_RX_CHANNEL |
| `nav_message_task` | 200ms (5Hz) | Sends NAV-* messages (Timer::at for drift-free timing) |
| `mon_message_task` | 1s | Sends MON-HW, MON-RF, MON-COMMS |
| `button_task` | async | Mode selection by click count (1/2/3/4 clicks) |

### Core1 Tasks
| Task | Purpose |
|------|---------|
| `led_task` (RP2350) | WS2812 LED blinking (green/yellow=emulation, blue=passthrough, purple=raw, red=spoof) |
| `simple_led_task` (RP2354) | GPIO LED blink code (1-4 blinks = mode number, fast blink = spoof) |
| `sec_sign_compute_task` | ECDSA signature computation (~59ms per signature) |
| `sec_sign_timer_task` | Waits for FIRST_CONFIG_MILLIS or 2s fallback, then first_delay → immediate first SEC-SIGN → period ticker (moved from Core0 to avoid UART interrupt starvation) |

### Module Structure
- `src/ubx/` - UBX protocol (`mod.rs`, `messages.rs`, `parser.rs`)
- `src/led.rs` - WS2812 driver (PIO `pio_asm!`)
- `src/sec_sign.rs` - SHA256 accumulator + ECDSA for SEC-SIGN
- `src/config.rs` - Pin assignments, timing constants, default position
- `src/coordinates.rs` - LLH→ECEF conversion (WGS84), cached at startup
- `src/passthrough.rs` - UBX frame parser, position buffer, NAV modification
- `src/spoof_detector.rs` - GPS spoofing detection algorithms
- `src/flash_storage.rs` - Flash persistence for mode, extracted keys
- `src/key_extract.rs` - Runtime key extraction from real GNSS via CFG-0x41

### Operating Modes
| Mode | ID | LED | Spoof Detection | Description |
|------|----|-----|-----------------|-------------|
| Emulation | 0 | green→yellow | No | Generates fake GNSS data with SEC-SIGN |
| Passthrough | 1 | blue | Yes | Forwards real GNSS data |
| PassthroughRaw | 2 | purple | No | Pure transparent forwarding |
| PassthroughOffset | 3 | white | Yes | Passthrough + coordinate offset |

Mode persisted to flash. Button: 1-4 clicks → mode 0-3. Timeout: 800ms. Hot-switch.

### PassthroughOffset Mode

Dynamic offset computed **once** at first 3D GPS fix: `offset = offset_target - actual_gps_position`. Target: `offset_target` in `config.rs` (Seney, Michigan). Offset applied to NAV-PVT, POSLLH, POSECEF, HPPOSECEF, SOL.

**Important rules**:
- Spoof detection uses **original** coordinates. Offset applied only to output.
- Offset applied ALWAYS (even during spoofing), BEFORE spoof modification.
- SEC-SIGN: always generate our own in Passthrough modes (real GNSS SEC-SIGN filtered).

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
                                           │   OR alt_jump(>10m/sample)      │
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
                                           │   dist(origin→curr) > 10km     │
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
                                                        │   dist(last_good)<2km   │
                                                        │   && within_leash:      │
                                                        │   spoofed=false         │
                                                        │   recovery_warmup=0     │
                                                        │                         │
                                                        │ if !spoofed &&          │
                                                        │   dist(last_good)<2km   │
                                                        │   && within_leash:      │
                                                        │   last_good=curr        │
                                                        │                         │
                                                        │ prev=curr               │
                                                        │ → Normal or Spoofed     │
                                                        └─────────────────────────┘
```

**Key state variables:**
- `origin` — first GPS fix, never updated (only on `reset()`), anti-gradient-drift anchor + leash anchor
- `last_good` — last trusted position, updated only when `!spoofed && dist < 2km && within_leash(origin, 5km)`
- `prev` — previous sample, always updated (for velocity calculation)
- `last_good_gnss_time` / `calibrated_at_system_ms` — time references, updated only when `!spoofed`

**Warmup phases:**
- **Startup warmup** (10 samples, ~2s): time checks disabled (need calibration), coord checks active
- **Recovery warmup** (10 samples, ~2s): coord + last_good checks disabled (GPS re-acquisition jitter), time checks active

**What happens after `analyze()` returns:**

| Result | `SPOOF_DETECTED` | NAV modification | LED |
|--------|-------------------|------------------|-----|
| `Normal` | `false` (after 5s timer) | offset only (Mode 4) | blue/white |
| `Spoofed` | `true` (immediate) | coords→LAST_GOOD, vel→0, status degraded, +offset | red blink |
| `Initializing` | unchanged | none | unchanged |
| `GapReset` | unchanged | none | unchanged |

**NAV messages modified during spoofing** (10 message types):

| Message | Fields replaced/degraded |
|---------|--------------------------|
| NAV-PVT (0x07) | lon, lat, height, hMSL → LAST_GOOD; velN/E/D → 0; fix_type=0, flags=0, num_sv=92 |
| NAV-POSLLH (0x02) | lon, lat, height, hMSL → LAST_GOOD; hAcc/vAcc → 9999999 |
| NAV-POSECEF (0x01) | ecefX/Y/Z → LAST_GOOD_ECEF; pAcc → 9999999 |
| NAV-HPPOSECEF (0x13) | ecefX/Y/Z → LAST_GOOD_ECEF; Hp → 0; invalidEcef; pAcc → 9999999 |
| NAV-SOL (0x06) | ecefX/Y/Z → LAST_GOOD_ECEF; ecefVX/Y/Z → 0; gps_fix=0, num_sv=92 |
| NAV-VELECEF (0x11) | ecefVX/Y/Z → 0; sAcc → 9999999 |
| NAV-VELNED (0x12) | velN/E/D → 0; speed/gSpeed → 0; heading → 0; sAcc/cAcc → 9999999 |
| NAV-STATUS (0x03) | gps_fix=0, flags=0 |
| NAV-SAT (0x35) | num_svs=92 |
| NAV-SVINFO (0x30) | num_ch=92 |

`num_sv=92` is an impossible value (max real ~40) used as spoof marker.

**Known bug (Jan 2026)**: SEC-UNIQID race in drone auto-detection. Mavic 4 Pro sends SEC-UNIQID poll BEFORE CFG-VALSET (auto-detect trigger). **Workaround**: Set `DRONE_MODEL` to target model in `main.rs` (default Air 3S = 2).

### NAV Output Start Timing

| Model | Delay from first UBX cmd | First SEC-SIGN delay |
|-------|--------------------------|----------------------|
| Air 3 | 700ms | 1000ms |
| Air 3S | 780ms | 650ms |
| Mavic 4 Pro | 400ms | 650ms |
| Mavic 3 Pro | 780ms | 650ms |

### SEC-SIGN Timer Start (Passthrough)

In Passthrough/PassthroughOffset modes, `sec_sign_timer_task` triggers on `FIRST_CONFIG_MILLIS` (set by first drone UBX command, typically SEC-UNIQID at ~15ms after connect) with 2s fallback if no drone commands. After trigger, waits `first_delay` then fires first SEC-SIGN **immediately** (no extra period wait). Subsequent SEC-SIGN via `Ticker::every(period)` at loop bottom.

After 20 seconds from NAV output start, satellites become invalid (fix_type=0, num_sv=1). Timer resets only on mode switch from Passthrough → Emulation.

### Passthrough Implementation

- Data flow: UART1 RX → `uart1_rx_task` → `RAW_RX_CHANNEL` (256B chunks, depth 64) → `gnss_processing_task` → `GNSS_RX_CHANNEL` (depth 128) → `uart0_tx_task` → UART0 TX
- `uart1_rx_task` and `gnss_processing_task` split for overrun prevention
- UART1 FIFO RX threshold: 1/4 (4 bytes) via `rp_pac` — prevents hardware overrun at 921600 baud
- Non-UBX data (NMEA) drained by `take_non_ubx_data()`, not forwarded to hash
- ECDSA ~59ms (opt-level=3 for crypto crates). During SEC-SIGN wait: `uart0_tx_task` stops consuming GNSS_RX_CHANNEL, packets stay queued (depth 128, ~5 pkts during ECDSA — no overflow risk).

**CRITICAL**: `uart0_tx_task` must `accumulate()` BEFORE `write_all()` to prevent hash race with `sec_sign_timer_task` on Core1. Lock is held briefly (not during `write_all`) so timer task is not blocked.

### SEC-SIGN TX Pause

`SEC_SIGN_IN_PROGRESS` atomic flag coordinates TX during ECDSA:
1. `sec_sign_timer_task`: if flag stuck → reuse it (no clear/set gap); else set flag → capture hash → `try_send()` request to Core1 (non-blocking, drops if full)
2. `nav_message_task` / `mon_message_task`: yield_now() loop until flag clear
3. `uart0_tx_task`: **two-layer guard** — pre-check flag at loop top → if true, wait ONLY for `SEC_SIGN_RESULT` (100ms timeout, no channel consumption) → send signature → clear flag. **Additionally**, both Emulation and Passthrough branches re-check flag after `select3` returns a GNSS/NAV packet — if flag became true during `select3` await, packet is dropped (`continue`) to prevent hash mismatch (packet sent to drone but not covered by SEC-SIGN hash).
4. `sec_sign_compute_task` (Core1): `try_send()` result back (non-blocking, drops if full)

**Packet drop cost**: The post-select3 guard drops ~1 GNSS packet per 2s SEC-SIGN cycle. Measured over 20min (68029 packets, 597 SEC-SIGN cycles): 59 RXM-RAWX (1.0%) + 6 NAV-PVT (0.1%) + 1 MGA-ACK — total 0.1% loss. RXM-RAWX (largest packet) hits the race window most often; NAV-PVT drops are rare (~1 per 200s). All other NAV-* types: zero loss.

**Non-blocking channels**: Both `SEC_SIGN_REQUEST.send()` and `SEC_SIGN_RESULT.send()` use `try_send()` to prevent cascading deadlocks. If channels are full, request/result is dropped and retried on next tick.

**Stuck flag guard**: If `SEC_SIGN_IN_PROGRESS` still true on next ticker tick (~2s later), flag is **not cleared** — left true and falls through to capture hash immediately (avoids TOCTOU window where TX task could sneak packets). Max SEC-SIGN gap = 2s.

**MGA-ACK**: `send_ack(0x13, id)` only in Emulation mode. In Passthrough, real GNSS sends its own MGA-ACK (MGA-0x60). Generating duplicate ACKs flooded TX_CHANNEL and starved executor.

`SEC_SIGN_ACC_NEEDS_RESET` atomic flag: set by mode switch, consumed by `sec_sign_timer_task`.

**Mode switch**: `apply_mode_by_clicks()` drains `TX_CHANNEL` after mode store to prevent stale message leaks (e.g. NAV-VELNED fragments from Emulation appearing in Passthrough output).

## Hardware Pins

### RP2350A — SpotPear RP2350-Core-A (default)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud, к дрону/хосту)
- UART1: RX=GPIO5 (от внешнего GNSS для passthrough)
- WS2812B LED: GPIO25 (PIO0)
- Mode button: GPIO14 (input), GPIO13 (power)

### RP2354A (`--features rp2354`)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud)
- UART1: RX=GPIO5 (passthrough)
- Simple GPIO LED: GPIO11 (анод), GPIO12 (катод/земля)
- Mode button: GPIO14 (input), GPIO13 (power)

## SEC-SIGN Cryptography

**Private keys** location: `src/sec_sign.rs`

| Drone Model | ID | First Delay | Period |
|-------------|-----|-------------|--------|
| Air 3 | 0 | 1000ms | 4s |
| Mavic 4 Pro | 1 | 650ms | 2s |
| Air 3S | 2 | 650ms | 2s |
| Mavic 3 Pro | 3 | 650ms | 2s |

Default: `DRONE_MODEL=2` (Air 3S). Mavic 4 Pro: per-unit unique keys (3 known). Use key extraction for correct key.

**Algorithm**: SHA256 over all transmitted UBX (except SEC-SIGN itself) → fold 32→24 bytes → ECDSA SECP192R1 sign with deterministic k (HMAC-SHA256, simplified RFC6979). Key crates: `p192`, `sha2`, `hmac`.

## CFG-0x41 (OTP / DJI Proprietary)

Poll `(0x06, 0x41)` with zero-length payload → 256-byte response with SEC-SIGN config.

**Key sections in payload**:

| Section | Description |
|---------|-------------|
| Bitmasks (0-25) | Signal enable bitmasks |
| ROM Patches (26-95) | files 0x82, 0x83 — ARM Thumb-2 code |
| CFG-SIGNAL (96+) | group 0x31 |
| CFG-RINV (~116) | group 0xC7, Remote Inventory (Air 3/Mavic 4 Pro only) |
| **SEC/KEY** | group 0xA6, **24-byte Private Key P-192** |
| CFG-UART1/CLOCK | groups 0x52, 0xA4 |

Key offset: 175 (Air 3, Mavic 4 Pro) or 115 (Air 3S, Mavic 3 Pro — no CFG-RINV section). Fallback: scan for `A6 18` tag.

## Runtime Key Extraction

Extracts 24-byte SEC-SIGN private key from real u-blox GNSS module via CFG-0x41 poll.

**Trigger**: Long button press (3+ sec) → flash flag → reboot → extraction at boot.
**HW**: TX via PIO1 on GPIO1, RX via UART1 on GPIO5. 5 attempts × 2s timeout.
**Storage**: Flash Last-1 sector (magic `0x4B455953` "KEYS"). Overrides hardcoded keys.
**Debug**: `FORCE_KEY_EXTRACT` in `main.rs` forces extraction on every boot.

## Diagnostic Mode

Set `DIAG_MSG_DETAIL = true` in `src/main.rs`, rebuild. Outputs per-message-type RX/TX counters and CFG-VALSET key logging every 10s.

## Dynamic Configuration

NAV rate: CFG-RATE (0x06, 0x08) or CFG-VALSET keys `0x30210001` (meas_rate), `0x30210002` (nav_rate).
Baudrate: CFG-VALSET key `0x40520001`. Uses direct UART0 register access via `rp-pac`.

## Firmware Version Storage

`build.rs` extracts git hash → `$OUT_DIR/version.rs` → written to flash (Last-3 sector, magic `0x56455253` "VERS").

```bash
probe-rs read --chip RP2350 0x103FD000 37   # RP2350 (4MB flash)
probe-rs read --chip RP2354 0x101FD000 37   # RP2354 (2MB flash)
```

## Implemented UBX Messages

### NAV Class (0x01)
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

### Other Classes
| Class | ID | Name | Payload | Description |
|-------|-----|------|---------|-------------|
| 0x02 | 0x13 | RXM-SFRBX | 8+n | Subframe buffer (passthrough) |
| 0x02 | 0x15 | RXM-RAWX | 16+ | Raw measurements |
| 0x05 | 0x00/0x01 | ACK-NAK/ACK | 2 | Acknowledgement |
| 0x06 | 0x41 | CFG-0x41 | 256 | DJI proprietary (private key) |
| 0x0A | 0x04/0x09/0x36/0x38 | MON-VER/HW/COMMS/RF | var | Monitoring |
| 0x0D | 0x01 | TIM-TP | 16 | Timepulse |
| 0x13 | * | MGA-* | var | AssistNow data |
| 0x27 | 0x03 | SEC-UNIQID | 10 | Unique ID (poll) |
| 0x27 | 0x04 | SEC-SIGN | 108 | Signature |

## Embassy 0.9 API Notes

- `BufferedUartTx` / `BufferedUartRx` have no lifetime parameters (changed from 0.8)
- `Signal::try_get()` doesn't exist - use `AtomicU8` for shared mode state
- PIO pin wrapping: `Peri<'d, PIN>` not raw pin type
- Interrupt binding: `bind_interrupts!` macro required
