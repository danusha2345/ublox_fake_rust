---
name: build
description: Build UF2 firmware for RP2350 or RP2354 via Makefile. Use when user asks to build, compile, make UF2, or prepare firmware for flashing.
disable-model-invocation: true
allowed-tools: Bash, Read
---

# /build — Build UF2 Firmware

Build the ublox_fake firmware UF2 file for a specific target chip.

## Arguments

- No arguments or `rp2350`: Build for RP2350 (external QSPI flash, WS2812B LED)
- `rp2354`: Build for RP2354 (2MB internal flash, GPIO LED)
- `all`: Build both RP2350 and RP2354

## Steps

1. Ensure PATH includes cargo:
```bash
export PATH="/home2/.cargo/bin:$PATH"
```

2. Run the appropriate make target:

For RP2350 (default):
```bash
cd /home/danik/Projects/ublox_fake_rust && make rp2350 2>&1
```

For RP2354:
```bash
cd /home/danik/Projects/ublox_fake_rust && make rp2354 2>&1
```

For `all`: run both sequentially, stop on first failure.

Timeout: 120 seconds.

3. Parse output:
   - Count `warning` lines (exclude `warning: unused` in dependencies)
   - Check for `error[EXXXX]` lines
   - Verify UF2 file was created: `ls -la ublox_fake_rp2350.uf2` or `ublox_fake_rp2354.uf2`

4. Output summary:
```
Build complete: RP2350

UF2:      ublox_fake_rp2350.uf2 (XXX KB)
Warnings: 0 | Errors: 0
```

For `all`:
```
Build complete: RP2350 + RP2354

| Target | UF2 File                   | Size    | Warnings |
|--------|----------------------------|---------|----------|
| RP2350 | ublox_fake_rp2350.uf2      | XXX KB  | 0        |
| RP2354 | ublox_fake_rp2354.uf2      | XXX KB  | 0        |
```

5. If build fails, show the last 30 lines of compiler output as error context.
