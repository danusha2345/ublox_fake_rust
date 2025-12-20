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
| text (code) | 55.8 KB | **34.8 KB** |
| bss (RAM) | **133 KB** | **13.5 KB** |

Rust/Embassy advantages:
- Stackless coroutines - no separate stack per task (FreeRTOS allocates fixed stacks)
- Messages built dynamically from struct fields instead of static byte arrays

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

## Known TODOs

1. ~~`nav_message_task` - message sending~~ ✅ Implemented: All NAV, TIM, RXM messages
2. ~~`mon_message_task` - MON messages~~ ✅ Implemented: MON-HW, MON-COMMS, MON-RF
3. Passthrough mode - not implemented
4. ECDSA signing - placeholder only (p192 v0.13 lacks SignPrimitive trait), generates deterministic but not cryptographically valid signatures
5. ~~CFG-PRT baudrate change~~ ✅ Implemented via direct PAC register access
6. Release build needs `cc` in PATH for build scripts

## Implemented UBX Messages

### NAV Class (0x01) - Navigation
| ID | Name | Payload | Description |
|----|------|---------|-------------|
| 0x01 | NAV-POSECEF | 20 | Position in ECEF |
| 0x02 | NAV-POSLLH | 28 | Position in LLH |
| 0x03 | NAV-STATUS | 16 | Receiver status |
| 0x04 | NAV-DOP | 18 | Dilution of precision |
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
| 0x02 | 0x15 | RXM-RAWX | 16+ | Raw measurements |
| 0x05 | 0x01 | ACK-ACK | 2 | Acknowledgement |
| 0x0D | 0x01 | TIM-TP | 16 | Timepulse |
| 0x27 | 0x01 | SEC-SIGN | 108 | Signature |
| 0x27 | 0x03 | SEC-UNIQID | 10 | Unique ID (poll) |

## Embassy 0.9 API Notes

- `BufferedUartTx` / `BufferedUartRx` have no lifetime parameters (changed from 0.8)
- `Signal::try_get()` doesn't exist - use `AtomicU8` for shared mode state
- PIO pin wrapping: `Peri<'d, PIN>` not raw pin type
- Interrupt binding: `bind_interrupts!` macro required
