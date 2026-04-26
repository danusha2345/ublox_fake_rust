#!/usr/bin/env python3
"""Extract a clean UC6580I boot dump from a DSLogic-style hex capture.

Input format (one entry per line):
    [   COM12] [HH:MM:SS:mmm] AA BB CC DD ...
We keep only COM12 (chip -> drone). Output is the concatenated bytes,
trimmed to the boot-phase: from the chip's very first emission (leading
CRLF + 'UC6580I-00 ...' header) up to the LAST byte before the first
'$GNRMC' sentence of the steady-state stream.
"""
import re
import sys
from pathlib import Path

LOG = Path("/home/danik/Projects/ublox_fake_rust/logs_analiser/2026-04-26_165533_neo_hex.txt")
OUT = Path("/home/danik/Projects/ublox_fake_rust/src/unicore/boot_dump.bin")

LINE_RE = re.compile(r"^\[\s*COM12\]\s*\[\d+:\d+:\d+:\d+\]\s*(.*?)\s*$")

def main() -> int:
    blob = bytearray()
    with LOG.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = LINE_RE.match(line.rstrip("\n"))
            if not m:
                continue
            hex_str = m.group(1)
            tokens = hex_str.split()
            for tok in tokens:
                if len(tok) != 2:
                    continue
                try:
                    blob.append(int(tok, 16))
                except ValueError:
                    pass

    print(f"Total COM12 bytes: {len(blob)}", file=sys.stderr)

    # Boundary: cut just before the first '$GNRMC' which marks
    # the start of the steady-state 5 Hz stream.
    marker = b"$GNRMC"
    end = blob.find(marker)
    if end < 0:
        print("ERROR: $GNRMC not found in stream", file=sys.stderr)
        return 1
    # Trim trailing whitespace/CRLF padding back to nearest '\r\n' so we
    # don't cut into the previous frame's CRC.
    boot = bytes(blob[:end])
    # The very first $GNRMC is preceded by a clean CRLF — keep boot data
    # ending exactly there (no trailing partial bytes).
    print(f"Boot-phase length: {len(boot)} bytes", file=sys.stderr)
    print(f"First 32 bytes:    {boot[:32].hex(' ')}", file=sys.stderr)
    print(f"Last 32 bytes:     {boot[-32:].hex(' ')}", file=sys.stderr)

    OUT.write_bytes(boot)
    print(f"Wrote {OUT} ({len(boot)} bytes)", file=sys.stderr)
    return 0

if __name__ == "__main__":
    sys.exit(main())
