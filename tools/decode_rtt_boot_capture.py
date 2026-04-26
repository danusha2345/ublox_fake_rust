#!/usr/bin/env python3
"""Decode UC6580I boot stream from a probe-rs RTT log.

The board (built with `--features unicore,boot-capture`) captures the chip's
first ~6.7 KB into RAM and re-emits it every 10 s as defmt log lines:

    ==RAWCAP BEGIN epoch=N len=L crc=XXXXXXXX==
    RAWCAP N 0 d3 00 03 ...
    RAWCAP N 32 fe a0 00 ...
    ...
    ==RAWCAP END epoch=N len=L==

This script reconstructs the byte stream, verifies CRC32, and writes a binary
file suitable for `src/unicore/boot_dump.bin`.

Usage:
    python3 decode_rtt_boot_capture.py --input cap.log --out new_dump.bin
    python3 decode_rtt_boot_capture.py --input cap.log --out new_dump.bin \
        --diff src/unicore/boot_dump.bin
"""
from __future__ import annotations

import argparse
import re
import sys
import zlib
from pathlib import Path

BEGIN_RE = re.compile(
    r"==RAWCAP BEGIN epoch=(?P<epoch>\d+) len=(?P<len>\d+) crc=(?P<crc>[0-9a-fA-F]+)=="
)
END_RE = re.compile(r"==RAWCAP END epoch=(?P<epoch>\d+) len=(?P<len>\d+)==")
LINE_RE = re.compile(
    r"(?:^|\s)RAWCAP\s+(?P<epoch>\d+)\s+(?P<offset>\d+)\s+(?P<hex>[0-9a-fA-F\s]+?)\s*$"
)


class Epoch:
    __slots__ = ("epoch", "expected_len", "expected_crc", "chunks", "ended")

    def __init__(self, epoch: int, expected_len: int, expected_crc: int) -> None:
        self.epoch = epoch
        self.expected_len = expected_len
        self.expected_crc = expected_crc
        self.chunks: dict[int, bytes] = {}
        self.ended = False

    def assemble(self) -> bytes | None:
        if not self.chunks:
            return None
        out = bytearray(self.expected_len)
        covered = bytearray(self.expected_len)
        for off, data in self.chunks.items():
            end = min(off + len(data), self.expected_len)
            n = end - off
            if n <= 0:
                continue
            out[off:end] = data[:n]
            for i in range(off, end):
                covered[i] = 1
        if any(c == 0 for c in covered):
            return None
        return bytes(out)


def parse_log(text: str) -> list[Epoch]:
    epochs: dict[int, Epoch] = {}
    current: Epoch | None = None

    for line in text.splitlines():
        m = BEGIN_RE.search(line)
        if m:
            ep = int(m["epoch"])
            ln = int(m["len"])
            crc = int(m["crc"], 16)
            current = Epoch(ep, ln, crc)
            epochs[ep] = current
            continue
        m = END_RE.search(line)
        if m:
            ep = int(m["epoch"])
            if ep in epochs:
                epochs[ep].ended = True
            current = None
            continue
        m = LINE_RE.search(line)
        if m:
            ep = int(m["epoch"])
            off = int(m["offset"])
            hex_str = m["hex"].replace(" ", "").replace("\t", "")
            try:
                data = bytes.fromhex(hex_str)
            except ValueError:
                continue
            target = epochs.get(ep)
            if target is None:
                # RAWCAP line without matching BEGIN — skip (RTT may have
                # dropped earlier lines; the next BEGIN/END cycle will recover).
                continue
            target.chunks[off] = data

    return list(epochs.values())


def pick_epoch(epochs: list[Epoch], wanted: int | None) -> Epoch | None:
    candidates = [e for e in epochs if e.ended and e.assemble() is not None]
    if not candidates:
        return None
    if wanted is not None:
        for c in candidates:
            if c.epoch == wanted:
                return c
        return None
    # Prefer the highest fully-emitted epoch.
    candidates.sort(key=lambda e: e.epoch)
    return candidates[-1]


def diff_against(blob: bytes, ref_path: Path) -> str:
    if not ref_path.exists():
        return f"diff: reference {ref_path} not found"
    ref = ref_path.read_bytes()
    same = blob == ref
    n = min(len(blob), len(ref))
    first_diff = -1
    for i in range(n):
        if blob[i] != ref[i]:
            first_diff = i
            break
    if first_diff < 0 and len(blob) != len(ref):
        first_diff = n
    return (
        f"diff: equal={same} new_len={len(blob)} ref_len={len(ref)} "
        f"first_diff_offset={first_diff}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True, type=Path, help="probe-rs RTT log file")
    ap.add_argument("--out", required=True, type=Path, help="output .bin path")
    ap.add_argument("--epoch", type=int, default=None, help="specific epoch to extract (default: latest complete)")
    ap.add_argument("--diff", type=Path, default=None, help="compare against existing boot_dump.bin")
    args = ap.parse_args()

    text = args.input.read_text(encoding="utf-8", errors="replace")
    epochs = parse_log(text)
    if not epochs:
        print("ERROR: no RAWCAP markers found in input", file=sys.stderr)
        return 1

    print(f"Found {len(epochs)} epoch(s):", file=sys.stderr)
    for e in epochs:
        blob = e.assemble()
        ok = "complete" if (e.ended and blob is not None) else "incomplete"
        print(
            f"  epoch={e.epoch} len={e.expected_len} chunks={len(e.chunks)} "
            f"ended={e.ended} {ok}",
            file=sys.stderr,
        )

    chosen = pick_epoch(epochs, args.epoch)
    if chosen is None:
        print("ERROR: no complete epoch found", file=sys.stderr)
        return 1

    blob = chosen.assemble()
    if blob is None or len(blob) != chosen.expected_len:
        print("ERROR: assembly failed (gaps in chunk coverage)", file=sys.stderr)
        return 1

    actual_crc = zlib.crc32(blob) & 0xFFFF_FFFF
    if actual_crc != chosen.expected_crc:
        print(
            f"WARN: CRC mismatch: header={chosen.expected_crc:08x} "
            f"actual={actual_crc:08x}",
            file=sys.stderr,
        )
    else:
        print(f"CRC32 ok: {actual_crc:08x}", file=sys.stderr)

    args.out.write_bytes(blob)
    print(f"Wrote {args.out} ({len(blob)} bytes, epoch={chosen.epoch})", file=sys.stderr)

    if args.diff is not None:
        print(diff_against(blob, args.diff), file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
