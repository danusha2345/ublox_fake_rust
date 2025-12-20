# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

u-blox GNSS M8/M10 emulator written in Rust for RP2040/RP2350 microcontrollers. Uses Embassy async framework instead of FreeRTOS.

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

### Task-based async design (Embassy)
- **Core0**: Embassy executor runs all async tasks
- All tasks communicate via `Signal` and `Channel` from `embassy-sync`
- Mode state shared via `AtomicU8` (not Signal, due to embassy 0.9 API)

### Main Tasks (src/main.rs)
| Task | Rate | Purpose |
|------|------|---------|
| `led_task` | 500ms | WS2812 LED blinking (green=emulation, blue=passthrough) |
| `uart_tx_task` | 10ms | Sends UBX messages from SEC_SIGN_READY signal |
| `uart_rx_task` | async | Parses incoming UBX commands |
| `nav_message_task` | 200ms (5Hz) | Sends NAV-* messages |
| `mon_message_task` | 1s | Sends MON-* messages |
| `sec_sign_timer_task` | 4s | Triggers SEC-SIGN computation |
| `button_task` | async | Mode toggle on GPIO button press |

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
- UART0: TX=GPIO0, RX=GPIO1 (38400 baud default)
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
