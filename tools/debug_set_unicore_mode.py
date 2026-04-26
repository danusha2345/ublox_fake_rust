#!/usr/bin/env python3
"""Set the UC6580I firmware operating mode through the debug mailbox."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ELF_CANDIDATES = (
    REPO_ROOT / "target/thumbv8m.main-none-eabihf/release/ublox_fake_uc",
    REPO_ROOT / "target/thumbv8m.main-none-eabihf/debug/ublox_fake_uc",
)

MODES = {
    "0": 0,
    "emulation": 0,
    "emu": 0,
    "1": 1,
    "passthrough": 1,
    "pass": 1,
    "2": 2,
    "passthroughraw": 2,
    "passthrough-raw": 2,
    "raw": 2,
    "3": 3,
    "passthroughoffset": 3,
    "passthrough-offset": 3,
    "offset": 3,
}
MODE_NAMES = ("emulation", "passthrough", "raw", "offset")


def parse_mode(value: str) -> int:
    key = value.strip().lower().replace("_", "-")
    try:
        return MODES[key]
    except KeyError as exc:
        expected = ", ".join(("0", "1", "2", "3", *MODE_NAMES))
        raise argparse.ArgumentTypeError(f"unknown mode {value!r}; expected one of: {expected}") from exc


def default_elf() -> Path:
    for path in DEFAULT_ELF_CANDIDATES:
        if path.exists():
            return path
    return DEFAULT_ELF_CANDIDATES[0]


def find_symbol(nm: str, elf: Path, symbol: str) -> int:
    if not elf.exists():
        raise SystemExit(
            f"ELF not found: {elf}\n"
            "Build it first, or pass --elf. Example:\n"
            '  cargo build --release --features "rp2350 unicore" '
            "--bin ublox_fake_uc --target thumbv8m.main-none-eabihf"
        )

    result = subprocess.run(
        [nm, "--defined-only", str(elf)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[-1] == symbol:
            return int(parts[0], 16)

    raise SystemExit(f"symbol {symbol!r} not found in {elf}")


def probe_options(args: argparse.Namespace) -> list[str]:
    cmd: list[str] = []
    if args.probe:
        cmd.extend(["--probe", args.probe])
    if args.speed:
        cmd.extend(["--speed", str(args.speed)])
    return cmd


def run_probe(args: argparse.Namespace, command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["probe-rs", command[0], *probe_options(args), *command[1:]],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )


def read_u8(args: argparse.Namespace, address: int) -> int:
    result = run_probe(args, ["read", "--chip", args.chip, "b8", hex(address), "1"])
    tokens = result.stdout.split()
    if not tokens:
        raise RuntimeError("probe-rs read returned no data")
    return int(tokens[-1], 16)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Write DEBUG_MODE_REQUEST in RAM and let the UC6580I firmware apply the mode safely.",
    )
    parser.add_argument("--mode", required=True, type=parse_mode, help="0/emulation, 1/passthrough, 2/raw, 3/offset")
    parser.add_argument("--elf", type=Path, default=default_elf(), help="ELF containing DEBUG_MODE_REQUEST and MODE")
    parser.add_argument("--chip", default="RP2350", help="probe-rs chip name")
    parser.add_argument("--probe", help="probe selector passed to probe-rs")
    parser.add_argument("--speed", type=int, help="SWD/JTAG speed in kHz passed to probe-rs")
    parser.add_argument("--nm", default="nm", help="nm-compatible tool")
    parser.add_argument("--timeout-ms", type=int, default=1500, help="MODE readback timeout")
    parser.add_argument("--poll-ms", type=int, default=100, help="MODE readback poll interval")
    parser.add_argument("--no-verify", action="store_true", help="do not read MODE after writing the request")
    args = parser.parse_args()

    request_addr = find_symbol(args.nm, args.elf, "DEBUG_MODE_REQUEST")
    mode_addr = find_symbol(args.nm, args.elf, "MODE") if not args.no_verify else None

    run_probe(args, ["write", "--chip", args.chip, "b8", hex(request_addr), str(args.mode)])
    print(f"DEBUG_MODE_REQUEST[{hex(request_addr)}] <- {args.mode} ({MODE_NAMES[args.mode]})")

    if args.no_verify:
        return 0

    deadline = time.monotonic() + args.timeout_ms / 1000.0
    last_value: int | None = None
    while time.monotonic() <= deadline:
        last_value = read_u8(args, mode_addr)
        if last_value == args.mode:
            print(f"MODE[{hex(mode_addr)}] == {last_value} ({MODE_NAMES[last_value]})")
            return 0
        time.sleep(args.poll_ms / 1000.0)

    print(
        f"MODE did not become {args.mode} ({MODE_NAMES[args.mode]}) within {args.timeout_ms} ms; "
        f"last read: {last_value}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
