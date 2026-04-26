#!/usr/bin/env python3
"""Decode UART from DSLogic CSV (transition-based) and find SEC-SIGN (B5 62 27 04)."""

import sys
import bisect

BAUD = 921600
BIT_PERIOD = 1.0 / BAUD  # ~1.0851 µs


def load_transitions(filename):
    """Load CSV, return per-channel transition lists: {ch: ([times], [states])}"""
    ch_times = {0: [], 1: [], 2: [], 3: []}
    ch_states = {0: [], 1: [], 2: [], 3: []}
    prev = [None, None, None, None]

    with open(filename, 'r') as f:
        for line in f:
            if line.startswith(';') or line.startswith('Time'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 5:
                continue
            t = float(parts[0])
            for ch in range(4):
                val = int(parts[ch + 1])
                if val != prev[ch]:
                    ch_times[ch].append(t)
                    ch_states[ch].append(val)
                    prev[ch] = val

    return ch_times, ch_states


def get_state(times, states, t):
    """Get signal state at time t using binary search."""
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return 0  # before any transition
    return states[idx]


def decode_uart_channel(times, states):
    """Decode 8N1 UART. Returns [(byte_start_time, byte_value), ...]"""
    if not times:
        return []

    result = []
    n = len(times)

    # Find falling edges (1→0) = start bits
    i = 0
    skip_until = 0.0

    while i < n:
        t = times[i]
        s = states[i]

        # Skip if we're still in a previous byte frame
        if t < skip_until:
            i += 1
            continue

        # Look for falling edge (1→0)
        if s == 0 and i > 0 and states[i - 1] == 1:
            start_t = t

            # Verify start bit at center
            start_center = start_t + 0.5 * BIT_PERIOD
            if get_state(times, states, start_center) != 0:
                i += 1
                continue

            # Sample 8 data bits
            byte_val = 0
            valid = True
            for bit in range(8):
                sample_t = start_t + (1.5 + bit) * BIT_PERIOD
                bit_val = get_state(times, states, sample_t)
                byte_val |= (bit_val << bit)

            # Check stop bit
            stop_t = start_t + 9.5 * BIT_PERIOD
            if get_state(times, states, stop_t) != 1:
                # Framing error — skip
                skip_until = start_t + 10.0 * BIT_PERIOD
                i += 1
                continue

            result.append((start_t, byte_val))

            # Skip past this byte frame
            skip_until = start_t + 9.5 * BIT_PERIOD
            i += 1
        else:
            i += 1

    return result


def find_ubx_messages(decoded):
    """Find all UBX messages (B5 62 class id). Return {(cls,id): [times]}"""
    data = [b for _, b in decoded]
    times = [t for t, _ in decoded]
    msgs = {}

    i = 0
    while i < len(data) - 5:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls = data[i + 2]
            msg_id = data[i + 3]
            length = data[i + 4] | (data[i + 5] << 8)
            key = (cls, msg_id)
            if key not in msgs:
                msgs[key] = []
            msgs[key].append((times[i], length, i))
            # Skip past this message (header + payload + checksum)
            i += 6 + length + 2
        else:
            i += 1
    return msgs


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else '/home/danik/Projects/ublox_fake_rust/logs_analiser/DSLogic_air3s.csv'

    print(f"Loading transitions from {filename}...")
    ch_times, ch_states = load_transitions(filename)

    for ch in range(4):
        print(f"  Channel {ch}: {len(ch_times[ch])} transitions", end="")
        if ch_times[ch]:
            print(f"  (first: {ch_times[ch][0]:.6f}s, last: {ch_times[ch][-1]:.6f}s)")
        else:
            print()

    ch_names = {
        0: "Unknown",
        1: "Drone→GNSS (config)",
        2: "GNSS→Board (u-blox)",
        3: "Board→Drone (output)"
    }

    for ch in (1, 2, 3):
        print(f"\n{'=' * 70}")
        print(f"Channel {ch}: {ch_names[ch]}")
        print(f"{'=' * 70}")

        decoded = decode_uart_channel(ch_times[ch], ch_states[ch])
        print(f"Decoded {len(decoded)} bytes")

        if not decoded:
            continue

        # Show first 32 bytes as hex
        first_bytes = decoded[:32]
        hex_str = ' '.join(f'{b:02X}' for _, b in first_bytes)
        print(f"First bytes: {hex_str}")

        # Find UBX messages
        msgs = find_ubx_messages(decoded)

        # Print known UBX classes
        CLASS_NAMES = {
            0x01: "NAV", 0x02: "RXM", 0x05: "ACK", 0x06: "CFG",
            0x0A: "MON", 0x0D: "TIM", 0x13: "MGA", 0x27: "SEC"
        }
        MSG_NAMES = {
            (0x01, 0x01): "NAV-POSECEF", (0x01, 0x02): "NAV-POSLLH",
            (0x01, 0x03): "NAV-STATUS", (0x01, 0x04): "NAV-DOP",
            (0x01, 0x06): "NAV-SOL", (0x01, 0x07): "NAV-PVT",
            (0x01, 0x11): "NAV-VELECEF", (0x01, 0x12): "NAV-VELNED",
            (0x01, 0x13): "NAV-HPPOSECEF", (0x01, 0x20): "NAV-TIMEGPS",
            (0x01, 0x21): "NAV-TIMEUTC", (0x01, 0x22): "NAV-CLOCK",
            (0x01, 0x26): "NAV-TIMELS", (0x01, 0x30): "NAV-SVINFO",
            (0x01, 0x35): "NAV-SAT", (0x01, 0x36): "NAV-COV",
            (0x01, 0x60): "NAV-AOPSTATUS", (0x01, 0x61): "NAV-EOE",
            (0x02, 0x13): "RXM-SFRBX", (0x02, 0x15): "RXM-RAWX",
            (0x05, 0x00): "ACK-NAK", (0x05, 0x01): "ACK-ACK",
            (0x06, 0x08): "CFG-RATE", (0x06, 0x8A): "CFG-VALSET",
            (0x06, 0x41): "CFG-0x41",
            (0x0A, 0x04): "MON-VER", (0x0A, 0x09): "MON-HW",
            (0x0A, 0x36): "MON-COMMS", (0x0A, 0x38): "MON-RF",
            (0x0D, 0x01): "TIM-TP",
            (0x13, 0x40): "MGA-INI", (0x13, 0x60): "MGA-ACK",
            (0x27, 0x03): "SEC-UNIQID", (0x27, 0x04): "SEC-SIGN",
        }

        print(f"\nUBX messages:")
        for (cls, mid), entries in sorted(msgs.items()):
            cls_name = CLASS_NAMES.get(cls, f"0x{cls:02X}")
            msg_name = MSG_NAMES.get((cls, mid), f"0x{mid:02X}")
            print(f"  {cls_name}-{msg_name} ({cls:02X}-{mid:02X}): {len(entries)} msgs, payload={entries[0][1]}")

        # SEC-SIGN details
        sec_sign_key = (0x27, 0x04)
        if sec_sign_key in msgs:
            entries = msgs[sec_sign_key]
            print(f"\n--- SEC-SIGN (27-04) details: {len(entries)} signatures ---")
            print(f"{'#':>4}  {'Time(s)':>12}  {'Interval(ms)':>12}  {'Payload':>7}")
            print(f"{'─' * 4}  {'─' * 12}  {'─' * 12}  {'─' * 7}")
            for i, (t, length, _) in enumerate(entries):
                if i == 0:
                    print(f"{i + 1:4d}  {t:12.6f}  {'—':>12}  {length:7d}")
                else:
                    dt = (t - entries[i - 1][0]) * 1000
                    print(f"{i + 1:4d}  {t:12.6f}  {dt:12.1f}  {length:7d}")

            if len(entries) > 1:
                intervals = [(entries[i][0] - entries[i - 1][0]) * 1000 for i in range(1, len(entries))]
                print(f"\nStatistics:")
                print(f"  Count:    {len(entries)}")
                print(f"  Min:      {min(intervals):.1f} ms")
                print(f"  Max:      {max(intervals):.1f} ms")
                print(f"  Avg:      {sum(intervals) / len(intervals):.1f} ms")
                print(f"  Median:   {sorted(intervals)[len(intervals) // 2]:.1f} ms")
                print(f"  First at: {entries[0][0]:.6f} s")
                print(f"  Last at:  {entries[-1][0]:.6f} s")

                # Histogram of intervals
                print(f"\n  Interval distribution:")
                buckets = {}
                for iv in intervals:
                    bucket = int(iv / 100) * 100  # 100ms buckets
                    buckets[bucket] = buckets.get(bucket, 0) + 1
                for b in sorted(buckets.keys()):
                    bar = '█' * buckets[b]
                    print(f"    {b:5d}-{b + 99:5d} ms: {buckets[b]:3d} {bar}")


if __name__ == '__main__':
    main()
