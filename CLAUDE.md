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
| Emulation | 0 | green | Fake GNSS + SEC-SIGN |
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
- **Unicore passthrough router is RTCM-aware** (`passthrough_forward_task` in `src/main_unicore.rs`): detect `0xD3` preamble before splitting on `$`, otherwise a `$` (0x24) byte inside an RTCM payload steals the rest of the frame into `LineAssembler` and it is silently dropped. Covers ExtRTCM 4074.

## Detailed Reference (Serena memories)

`sec_sign_protocol` — SEC-SIGN algorithm, drone timing, TX pause mechanism
`cfg_0x41_key_extraction` — CFG-0x41 format, auto/manual key extraction
`spoof_detection_algorithm` — full algorithm, state variables, warmup phases, NAV modification
`ubx_messages` — all implemented UBX message IDs and payloads
