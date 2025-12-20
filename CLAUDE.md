# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

u-blox GNSS M8/M10 emulator written in Rust for RP2040/RP2350 microcontrollers. Uses Embassy async framework instead of FreeRTOS.

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
rustup target add thumbv6m-none-eabi      # RP2040
rustup target add thumbv8m.main-none-eabihf  # RP2350
```

## Required Files

- `memory.x` - Linker script defining RP2040 memory layout (BOOT2, FLASH, RAM). Build fails without it!
- `.cargo/config.toml` - Target selection and linker flags

## Build Commands

```bash
# Build for RP2040 (default)
cargo build

# Build release for RP2040
cargo build --release

# Build for RP2350
cargo build --release --features rp2350 --target thumbv8m.main-none-eabihf

# Flash and run (requires probe-rs)
cargo run --release

# Aliases defined in .cargo/config.toml:
cargo rb        # run release binary
cargo rp2350    # build for RP2350
```

## Architecture

### Dual-core async design (Embassy)
- **Core0**: Embassy executor - UART TX/RX, NAV/MON message generation, button handling
- **Core1**: Embassy executor - LED control (PIO), SEC-SIGN ECDSA computation
- Inter-core communication via `Signal` and `Channel` from `embassy-sync`
- Mode state shared via `AtomicU8` with `Acquire/Release` ordering

### Core0 Tasks (src/main.rs)
| Task | Rate | Purpose |
|------|------|---------|
| `uart_tx_task` | async | Sends UBX messages from TX_CHANNEL, accumulates for SEC-SIGN |
| `uart_rx_task` | async | Parses incoming UBX commands, updates MSG_FLAGS |
| `nav_message_task` | 200ms (5Hz) | Sends NAV-PVT, NAV-STATUS, NAV-DOP, NAV-EOE |
| `mon_message_task` | 1s | Sends MON-* messages (TODO: implement message structs) |
| `sec_sign_timer_task` | 4s | Requests SEC-SIGN from Core1 |
| `button_task` | async | Mode toggle on GPIO button press |

### Core1 Tasks
| Task | Rate | Purpose |
|------|------|---------|
| `led_task` | 500ms | WS2812 LED blinking (green=emulation, blue=passthrough) |
| `sec_sign_compute_task` | async | ECDSA signature computation (CPU intensive) |

### Module Structure
- `src/ubx/` - UBX protocol implementation
  - `mod.rs` - Message classes, IDs, Fletcher checksum
  - `messages.rs` - NAV-PVT, NAV-POSECEF, SEC-SIGN structs
  - `parser.rs` - State machine for parsing incoming UBX frames
- `src/led.rs` - WS2812 driver using PIO (`pio::pio_asm!` macro)
- `src/sec_sign.rs` - SHA256 accumulator for SEC-SIGN authentication
- `src/config.rs` - Pin assignments, timing constants, default position

### Operating Modes
- **Emulation**: Generates fake GNSS data with SEC-SIGN authentication
- **Passthrough**: Forwards data from real GNSS module (not yet implemented)

## Hardware Pins (RP2040)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud default)
- WS2812 LED: GPIO16 (PIO0)
- Mode button: GPIO6 (input), GPIO5 (power)

## Key Dependencies
- `embassy-rp 0.9` - RP2040/RP2350 HAL
- `embedded-io-async 0.6` - Must match embassy-rp version exactly
- `pio 0.3` - For `pio_asm!` macro (not 0.2)
- `p192` + `sha2` - ECDSA SECP192R1 for SEC-SIGN

## UBX Protocol Notes
- All messages use 0xB5 0x62 sync header
- Checksum: Fletcher 8-bit over class, id, length, payload
- SEC-SIGN uses SHA256 hash folded to 24 bytes, signed with SECP192R1

## SEC-SIGN Cryptography

Private key location: `src/sec_sign.rs` line 8 (`PRIVATE_KEY` constant)

Algorithm:
1. Accumulate all transmitted UBX messages in SHA256 hasher
2. Compute `z = fold(SHA256(sha256_field || session_id))` - fold 32→24 bytes via XOR
3. Sign `z` with ECDSA SECP192R1 (P-192 curve)
4. Output: 48-byte signature (r=24, s=24) in UBX-SEC-SIGN message

## RAM Usage Comparison (vs C/FreeRTOS)

| Metric | C/FreeRTOS | Rust/Embassy |
|--------|------------|--------------|
| text (code) | 55.8 KB | 52.7 KB |
| bss (RAM) | **133 KB** | **3.8 KB** |

Embassy async uses stackless coroutines - no separate stack per task. FreeRTOS allocates fixed stacks for each task.

## Known TODOs

1. ~~`nav_message_task` - message sending~~ ✅ Implemented: NAV-PVT, NAV-STATUS, NAV-DOP, NAV-EOE
2. `mon_message_task` - MON-HW, MON-COMMS, MON-RF message structs not implemented
3. Passthrough mode - not implemented
4. ECDSA signing - placeholder only (p192 v0.13 lacks SignPrimitive trait), generates deterministic but not cryptographically valid signatures
5. CFG-PRT baudrate change - parsed but not applied
6. Release build needs `cc` in PATH for build scripts

## Embassy 0.9 API Notes

- `BufferedUartTx` / `BufferedUartRx` have no lifetime parameters (changed from 0.8)
- `Signal::try_get()` doesn't exist - use `AtomicU8` for shared mode state
- PIO pin wrapping: `Peri<'d, PIN>` not raw pin type
- Interrupt binding: `bind_interrupts!` macro required
