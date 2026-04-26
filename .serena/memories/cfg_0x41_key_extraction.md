# CFG-0x41 (OTP / DJI Proprietary) & Key Extraction

## CFG-0x41 Format
Poll `(0x06, 0x41)` with zero-length payload → 256-byte response.

| Section | Description |
|---------|-------------|
| Bitmasks (0-25) | Signal enable bitmasks |
| ROM Patches (26-95) | files 0x82, 0x83 — ARM Thumb-2 code |
| CFG-SIGNAL (96+) | group 0x31 |
| CFG-RINV (~116) | group 0xC7, Remote Inventory (Air 3/Mavic 4 Pro only) |
| **SEC/KEY** | group 0xA6, **24-byte Private Key P-192** |
| CFG-UART1/CLOCK | groups 0x52, 0xA4 |

Key offset: 175 (Air 3, Mavic 4 Pro) or 115 (Air 3S, Mavic 3 Pro — no CFG-RINV). Fallback: scan for `A6 18` tag.

## Auto Extraction (at boot)
If no key in flash, `auto_extract()` runs:
1. Detect GNSS on UART1 (500ms timeout)
2. Retries on framing/break errors
3. CFG-0x41 poll (2 attempts × 500ms)
4. Save to flash

Timings: GNSS detected ~120-280ms, key extracted ~430-600ms total.
No GNSS → 500ms timeout → fallback to hardcoded keys.
Key in flash → skipped (0ms).

## Manual Extraction
Long button press (3+ sec) → flash flag → reboot → 5 attempts × 2s.
LED: green=success, red=fail.

## Storage
Flash Last-1 sector (magic `0x4B455953` "KEYS"). Overrides hardcoded keys.
HW: TX via PIO1 on GPIO1, RX via UART1 on GPIO5.
Debug: `FORCE_KEY_EXTRACT` in `main.rs`.
