#!/usr/bin/env python3
"""Decode UC6580I boot stream(s) from a probe-rs RTT log.

The board (built with `--features unicore,boot-capture`) captures up to two
streams into separate RAM buffers and re-emits each every 10 s as defmt log
lines. Two marker prefixes select the side:

    chip side (UART1 RX, ~6.7 KB, freezes on `$GNRMC`):
        ==RAWCAP BEGIN epoch=N len=L crc=XXXXXXXX==
        RAWCAP N 0 [d3, 00, 03, ...]
        ==RAWCAP END epoch=N len=L==

    drone side (UART0 RX, ≤8 KB, freezes on `$CFGKEY` or 25 s timeout):
        ==RAWCAP_DR BEGIN epoch=N len=L crc=XXXXXXXX==
        RAWCAP_DR N 0 [ff, 7f, 7f, ...]
        ==RAWCAP_DR END epoch=N len=L==

This script reconstructs the byte stream(s), verifies CRC32, and writes a
binary file. Pick the side with `--side chip` (default) or `--side drone`.

Usage:
    python3 decode_rtt_boot_capture.py --input cap.log --out chip.bin
    python3 decode_rtt_boot_capture.py --input cap.log --out drone.bin --side drone
    python3 decode_rtt_boot_capture.py --input cap.log --out chip.bin \
        --diff src/unicore/boot_dump.bin
"""
from __future__ import annotations

import argparse
import re
import sys
import zlib
from pathlib import Path

def _build_regex(tag: str) -> tuple[re.Pattern, re.Pattern, re.Pattern]:
    """Build the BEGIN / END / LINE regex trio for marker prefix `tag`."""
    begin = re.compile(
        rf"=={tag} BEGIN epoch=(?P<epoch>\d+) len=(?P<len>\d+) crc=(?P<crc>[0-9a-fA-F]+)=="
    )
    end = re.compile(
        rf"=={tag} END epoch=(?P<epoch>\d+) len=(?P<len>\d+)=="
    )
    line = re.compile(
        # Defmt {=[u8]:02x} formatter renders bytes as `[d3, 00, 03, ...]`.
        # Accept both that form and bare space-separated hex (`d3 00 03`).
        rf"(?:^|\s){tag}\s+(?P<epoch>\d+)\s+(?P<offset>\d+)\s+(?P<hex>[\[\]0-9a-fA-F,\s]+?)\s*(?:\(.*\))?$"
    )
    return begin, end, line


# Side → (begin, end, line) regex tuple.
_SIDE_TAGS = {"chip": "RAWCAP", "drone": "RAWCAP_DR"}
_REGEX = {side: _build_regex(tag) for side, tag in _SIDE_TAGS.items()}


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


def parse_log(text: str, side: str) -> list[Epoch]:
    """Extract epochs for one side (`chip` or `drone`)."""
    if side not in _REGEX:
        raise ValueError(f"unknown side {side!r}, expected one of {list(_REGEX)}")
    begin_re, end_re, line_re = _REGEX[side]
    # Other side's regex is used to skip lines that belong to the OTHER tag —
    # otherwise `RAWCAP_DR` lines would also match the `chip` LINE_RE because
    # `RAWCAP` is a prefix of `RAWCAP_DR`.
    other_tag = "RAWCAP_DR" if side == "chip" else "RAWCAP"
    other_marker = re.compile(rf"(?:^|\s){other_tag}(?:\s|=)")

    epochs: dict[int, Epoch] = {}

    for line in text.splitlines():
        # Skip lines that belong to the other side first.
        if side == "chip" and other_marker.search(line):
            continue
        m = begin_re.search(line)
        if m:
            ep = int(m["epoch"])
            ln = int(m["len"])
            crc = int(m["crc"], 16)
            epochs[ep] = Epoch(ep, ln, crc)
            continue
        m = end_re.search(line)
        if m:
            ep = int(m["epoch"])
            if ep in epochs:
                epochs[ep].ended = True
            continue
        m = line_re.search(line)
        if m:
            ep = int(m["epoch"])
            off = int(m["offset"])
            # Defmt formats `{=[u8]:02x}` as `[d3, 00, 03, ...]` (commas + brackets).
            # Strip both and the older bare-hex form before fromhex().
            hex_str = (
                m["hex"]
                .replace("[", "")
                .replace("]", "")
                .replace(",", "")
                .replace(" ", "")
                .replace("\t", "")
            )
            if not hex_str or len(hex_str) % 2 != 0:
                continue
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
    ap.add_argument(
        "--side",
        choices=("chip", "drone"),
        default="chip",
        help="which capture buffer to decode (default: chip)",
    )
    ap.add_argument("--epoch", type=int, default=None, help="specific epoch to extract (default: latest complete)")
    ap.add_argument("--diff", type=Path, default=None, help="compare against existing boot_dump.bin")
    args = ap.parse_args()

    text = args.input.read_text(encoding="utf-8", errors="replace")
    epochs = parse_log(text, args.side)
    if not epochs:
        tag = _SIDE_TAGS[args.side]
        print(f"ERROR: no {tag} markers found in input", file=sys.stderr)
        return 1

    print(f"Found {len(epochs)} {args.side} epoch(s):", file=sys.stderr)
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
