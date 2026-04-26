---
name: flash
description: Flash firmware to RP2350/RP2354 via probe-rs and show defmt log output. Use when user asks to flash, upload, program, run on hardware, or deploy firmware.
disable-model-invocation: true
allowed-tools: Bash, Read
---

# /flash — Flash Firmware via probe-rs

Build and flash the ublox_fake firmware to the board via SWD, then show defmt log output.

## Arguments

- No arguments or `rp2350`: Flash RP2350 (default)
- `rp2354`: Flash RP2354

## Steps

1. Ensure PATH includes cargo:
```bash
export PATH="/home2/.cargo/bin:$PATH"
```

2. Flash the appropriate target:

For RP2350 (default):
```bash
cd /home/danik/Projects/ublox_fake_rust && make flash 2>&1
```

For RP2354:
```bash
cd /home/danik/Projects/ublox_fake_rust && make flash-rp2354 2>&1
```

Timeout: 120 seconds.

IMPORTANT: probe-rs will show defmt log output after flashing. The process runs until interrupted (Ctrl+C). Let the log run for ~10 seconds to capture startup messages, then report results.

3. Parse output:
   - Look for `Firmware version:` line — confirms boot success
   - Look for `Mode:` line — shows current operating mode
   - Look for `Drone model:` — shows configured drone
   - Check for any `ERROR` or `panic` messages

4. Output summary:
```
Flash OK: RP2350

Version:  0.1.0-de58680
Mode:     Emulation
Drone:    Air 3S
Boot log: (first 10 lines of defmt output)
```

5. If flashing fails:
   - `Error: probe not found` → "No probe-rs debugger connected. Check SWD cable."
   - `Error: connecting to target` → "Cannot connect to target. Check power and SWD wiring."
   - Build error → show compiler output.
