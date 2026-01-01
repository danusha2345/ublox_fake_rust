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

**CRITICAL: Always use Makefile to build UF2 files!**

```bash
# Build UF2 for RP2350 (ALWAYS use this!)
make rp2350

# Build UF2 for RP2040
make rp2040

# Flash and run (requires probe-rs)
cargo run --release
make flash
```

**WARNING:** Do NOT use manual uf2conv.py or objcopy commands!
The Makefile uses `elf2uf2-rs` (takes addresses from ELF sections) + patches Family ID for RP2350.
Manual conversion with objcopy loses address info and creates broken firmware.

```bash
# Aliases defined in .cargo/config.toml:
cargo rb        # run release binary
cargo rp2350    # build for RP2350 (ELF only, no UF2)
```

## Architecture

### Dual-core async design (Embassy)
- **Core0**: Embassy executor - UART TX/RX, NAV message generation, button handling
- **Core1**: Embassy executor - LED control (PIO), SEC-SIGN ECDSA, MON messages
- Inter-core communication via `Signal` and `Channel` from `embassy-sync`
- Mode state shared via `AtomicU8` with `Acquire/Release` ordering
- TX_CHANNEL capacity: 32 messages (buffer for SEC-SIGN computation delays)

### Core0 Tasks (src/main.rs)
| Task | Rate | Purpose |
|------|------|---------|
| `uart_tx_task` | async | Sends UBX messages from TX_CHANNEL, accumulates SHA256 for SEC-SIGN |
| `uart_rx_task` | async | Parses incoming UBX commands, updates MSG_FLAGS |
| `nav_message_task` | 200ms (5Hz) | Sends NAV-* messages (uses Timer::at for drift-free timing) |
| `sec_sign_timer_task` | 2-4s | Requests SEC-SIGN from Core1, waits via SEC_SIGN_DONE Signal |
| `button_task` | async | Mode toggle on GPIO button press |

### Core1 Tasks
| Task | Rate | Purpose |
|------|------|---------|
| `led_task` | 500ms | WS2812 LED blinking (green/yellow=emulation, blue=passthrough) |
| `sec_sign_compute_task` | async | ECDSA signature computation (CPU intensive) |
| `mon_message_task` | 1s | Sends MON-HW, MON-RF, MON-COMMS |

### Module Structure
- `src/ubx/` - UBX protocol implementation
  - `mod.rs` - Message classes, IDs, Fletcher checksum
  - `messages.rs` - NAV-PVT, NAV-POSECEF, SEC-SIGN structs
  - `parser.rs` - State machine for parsing incoming UBX frames
- `src/led.rs` - WS2812 driver using PIO (`pio::pio_asm!` macro)
- `src/sec_sign.rs` - SHA256 accumulator for SEC-SIGN authentication
- `src/config.rs` - Pin assignments, timing constants, default position
- `src/coordinates.rs` - LLH→ECEF conversion, cached at startup
- `src/passthrough.rs` - PIO-based UART passthrough driver
- `src/flash_storage.rs` - Flash persistence for operating mode

### Coordinate System

Default coordinates are set in `config.rs` and automatically converted to all required formats at startup:

```rust
// config.rs
pub mod default_position {
    pub const LATITUDE: f64 = 25.7889186;   // degrees
    pub const LONGITUDE: f64 = -80.1919471; // degrees
    pub const ALTITUDE_M: i32 = 101;        // meters
}
```

The `coordinates` module computes once at init:
- `lat_1e7()`, `lon_1e7()` - for NAV-PVT, NAV-POSLLH (deg × 1e-7)
- `alt_mm()` - altitude in mm
- `ecef_x_cm()`, `ecef_y_cm()`, `ecef_z_cm()` - for NAV-POSECEF, NAV-SOL, NAV-HPPOSECEF

Uses WGS84 ellipsoid parameters for LLH→ECEF conversion.

### Operating Modes
- **Emulation**: Generates fake GNSS data with SEC-SIGN authentication (LED green→yellow)
- **Passthrough**: Forwards data from real GNSS module via PIO (LED blue)

Mode is persisted to flash and survives reboots. Button press toggles mode (hot-switch, no reboot).

### NAV Output Start Timing

NAV messages start after a model-specific delay from the first UBX command (any command: MON-VER poll, CFG-VALSET, etc.):

| Model | Real Timing | Config Delay |
|-------|-------------|--------------|
| Air 3 | 666ms | 700ms |
| Mavic 4 Pro | 399ms | 400ms |

```
First UBX command → +delay → NAV output starts → +650ms → First SEC-SIGN
```

Implementation:
- `FIRST_CONFIG_MILLIS` records timestamp of first command
- `nav_message_task` gets delay based on `DRONE_MODEL` AtomicU8
- `CONFIG_TO_NAV_AIR3_MS = 700`, `CONFIG_TO_NAV_MAVIC4_MS = 400` in config.rs

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
- Uses PIO1 state machine 0 for signal copying
- Input: GPIO3 (from external GNSS TX)
- Output: GPIO0 (to host, same as UART TX)
- PIO clock: 8MHz (~8 samples per bit at 921600 baud)
- PIO program waits for pin state changes and copies them:
```asm
.wrap_target
    wait 1 pin 0    ; wait for input high
    set pins, 1     ; set output high
    wait 0 pin 0    ; wait for input low
    set pins, 0     ; set output low
.wrap
```

## Hardware Pins (RP2350A - Spotpear RP2350-Core-A)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud, к дрону/хосту)
- UART1: RX=GPIO5 (от внешнего GNSS для passthrough)
- WS2812B LED: GPIO25 (PIO0)
- Mode button: GPIO7 (input), GPIO6 (power)

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

**Private keys** location: `src/sec_sign.rs`

| Drone Model | Constant | First Delay | SEC-SIGN Period |
|-------------|----------|-------------|-----------------|
| DJI Air 3 | `PRIVATE_KEY_AIR3` | 1000ms | 4 seconds |
| DJI Mavic 4 Pro | `PRIVATE_KEY_MAVIC4PRO` | 650ms | 2 seconds |

Model selection: `DRONE_MODEL` static variable in `main.rs` (0=Air3, 1=Mavic4Pro)

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

**Hash accumulation**: ALL transmitted UBX messages are accumulated except SEC-SIGN itself (0x27, 0x04). This includes:
- All NAV-* messages (PVT, POSECEF, STATUS, DOP, SAT, EOE, etc.)
- All MON-* messages (HW, RF, COMMS, VER)
- All ACK-ACK responses to CFG commands
- SEC-UNIQID (0x27, 0x03) - IS included in hash
- TIM-TP, RXM-RAWX

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
| 5. CFG-RINV | ~116 | ~50 | group 0xC7, Remote Inventory |
| 6. SEC/KEY | ~166 | 26 | group 0xA6, **Private Key** |
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
- `PRIVATE_KEY_OFFSET = 175` - where key is inserted in template
- Template captured from real Mavic 4 Pro GNSS module

**Security note**: This command exposes the private key, allowing key extraction from real DJI GNSS modules.

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

1. ✅ NAV messages - All 17 NAV message types implemented
2. ✅ MON messages - MON-HW, MON-COMMS, MON-RF, MON-VER (poll)
3. ✅ Passthrough mode - PIO-based GPIO3→GPIO0 with flash persistence
4. ✅ ECDSA signing - Pure Rust p192 with RFC6979 deterministic k
5. ✅ CFG-PRT baudrate - Direct PAC register access
6. ✅ CFG-VALSET - Full M10 configuration support
7. ✅ ACK-ACK/ACK-NAK - Command acknowledgement
8. ✅ SEC-UNIQID - Unique ID poll response
9. ✅ CFG-0x41 - DJI proprietary SEC-SIGN config (256-byte response with private key)
10. ✅ MGA-* - AssistNow assistance data (ACK-ACK response)
11. ✅ Release build - 0 warnings, pure Rust

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
