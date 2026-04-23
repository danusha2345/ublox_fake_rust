# u-blox GNSS M10 Emulator (RP2350/RP2354, Embassy)

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
make test                              # All 128 host-side tests
cd tests_host && cargo test            # spoof_detector + passthrough + coordinates
```

`tests_host/` is standalone crate (NOT in workspace), uses `#[path]` to `../../src/spoof_detector.rs`.

## Architecture

Dual-core Embassy async. Core0: UART TX/RX, NAV generation, button. Core1: LED (PIO), SEC-SIGN ECDSA.
Inter-core: `Signal`/`Channel` (embassy-sync). Mode state: `AtomicU8`.

### Modules
- `src/ubx/` — UBX protocol (messages, parser)
- `src/sec_sign.rs` — SHA256 + ECDSA SECP192R1
- `src/spoof_detector.rs` — GPS spoofing detection (pure logic, host-testable)
- `src/passthrough.rs` — UBX frame parser, position buffer, NAV modification
- `src/config.rs` — pins, timing, default position
- `src/coordinates.rs` — LLH/ECEF (WGS84)
- `src/flash_storage.rs` — mode + key persistence
- `src/key_extract.rs` — runtime key extraction via CFG-0x41
- `src/led.rs` — WS2812 (PIO)

### Operating Modes
| Mode | ID | LED | Description |
|------|----|-----|-------------|
| Emulation | 0 | green/yellow | Fake GNSS + SEC-SIGN |
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
- **num_sv=92**: impossible value used as spoof marker in NAV modification
- **Diagnostic mode**: `DIAG_MSG_DETAIL = true` in main.rs

## Detailed Reference (Serena memories)

`sec_sign_protocol` — SEC-SIGN algorithm, drone timing, TX pause mechanism
`cfg_0x41_key_extraction` — CFG-0x41 format, auto/manual key extraction
`spoof_detection_algorithm` — full algorithm, state variables, warmup phases, NAV modification
`ubx_messages` — all implemented UBX message IDs and payloads
