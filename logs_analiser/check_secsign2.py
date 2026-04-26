#!/usr/bin/env python3
"""Universal DSLogic CSV UART decoder — auto-detects channel layout."""

import sys
import bisect

BAUD = 921600
BIT_PERIOD = 1.0 / BAUD

def load_transitions(filename):
    """Auto-detect channel columns from header, return {ch_name: (times, states)}."""
    ch_times = {}
    ch_states = {}
    ch_prev = {}
    ch_cols = {}  # col_index -> channel_name

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith(';') or not line:
                continue
            if line.startswith('Time'):
                # Parse header: "Time(s), 0, 1, 2, 3" or "Time(s), 1, 2, 3"
                parts = [p.strip() for p in line.split(',')]
                for col_idx, name in enumerate(parts[1:], start=1):
                    ch_cols[col_idx] = int(name)
                    ch_times[int(name)] = []
                    ch_states[int(name)] = []
                    ch_prev[int(name)] = None
                continue

            parts = line.split(',')
            if len(parts) < 2:
                continue
            t = float(parts[0])
            for col_idx, ch_name in ch_cols.items():
                if col_idx < len(parts):
                    val = int(parts[col_idx])
                    if val != ch_prev[ch_name]:
                        ch_times[ch_name].append(t)
                        ch_states[ch_name].append(val)
                        ch_prev[ch_name] = val

    return ch_times, ch_states

def get_state(times, states, t):
    idx = bisect.bisect_right(times, t) - 1
    return states[idx] if idx >= 0 else 0

def decode_uart_channel(times, states):
    if not times:
        return []
    result = []
    n = len(times)
    i = 0
    skip_until = 0.0
    while i < n:
        t = times[i]
        if t < skip_until:
            i += 1
            continue
        s = states[i]
        if s == 0 and i > 0 and states[i - 1] == 1:
            start_t = t
            if get_state(times, states, start_t + 0.5 * BIT_PERIOD) != 0:
                i += 1
                continue
            byte_val = 0
            for bit in range(8):
                bit_val = get_state(times, states, start_t + (1.5 + bit) * BIT_PERIOD)
                byte_val |= (bit_val << bit)
            if get_state(times, states, start_t + 9.5 * BIT_PERIOD) != 1:
                skip_until = start_t + 10.0 * BIT_PERIOD
                i += 1
                continue
            result.append((start_t, byte_val))
            skip_until = start_t + 9.5 * BIT_PERIOD
        i += 1
    return result

def ubx_checksum(data):
    ck_a = ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

CLASS_NAMES = {
    0x01: "NAV", 0x02: "RXM", 0x05: "ACK", 0x06: "CFG",
    0x0A: "MON", 0x0D: "TIM", 0x13: "MGA", 0x27: "SEC"
}
MSG_NAMES = {
    (0x01, 0x01): "POSECEF", (0x01, 0x02): "POSLLH",
    (0x01, 0x03): "STATUS", (0x01, 0x04): "DOP",
    (0x01, 0x06): "SOL", (0x01, 0x07): "PVT",
    (0x01, 0x11): "VELECEF", (0x01, 0x12): "VELNED",
    (0x01, 0x13): "HPPOSECEF", (0x01, 0x20): "TIMEGPS",
    (0x01, 0x21): "TIMEUTC", (0x01, 0x22): "CLOCK",
    (0x01, 0x26): "TIMELS", (0x01, 0x30): "SVINFO",
    (0x01, 0x35): "SAT", (0x01, 0x36): "COV",
    (0x01, 0x60): "AOPSTATUS", (0x01, 0x61): "EOE",
    (0x02, 0x13): "SFRBX", (0x02, 0x15): "RAWX",
    (0x05, 0x00): "NAK", (0x05, 0x01): "ACK",
    (0x06, 0x08): "RATE", (0x06, 0x8A): "VALSET",
    (0x06, 0x41): "0x41",
    (0x0A, 0x04): "VER", (0x0A, 0x09): "HW",
    (0x0A, 0x36): "COMMS", (0x0A, 0x38): "RF",
    (0x0D, 0x01): "TP",
    (0x13, 0x40): "INI", (0x13, 0x60): "ACK",
    (0x13, 0x80): "DBD",
    (0x27, 0x03): "UNIQID", (0x27, 0x04): "SIGN",
}

def analyze_channel(decoded, ch_name):
    data = [b for _, b in decoded]
    times = [t for t, _ in decoded]
    total = len(data)

    print(f"\n{'=' * 70}")
    print(f"{ch_name}  ({total} bytes decoded)")
    print(f"{'=' * 70}")

    if total < 4:
        print("  Too few bytes")
        return

    # First bytes
    hex_str = ' '.join(f'{b:02X}' for b in data[:40])
    print(f"First bytes: {hex_str}")

    # Find all UBX messages
    msgs = {}
    i = 0
    while i < total - 5:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls, mid = data[i + 2], data[i + 3]
            length = data[i + 4] | (data[i + 5] << 8)
            end = i + 6 + length + 2
            complete = end <= total

            if complete:
                ck_data = data[i + 2:i + 6 + length]
                exp_a, exp_b = ubx_checksum(ck_data)
                ck_ok = (data[end - 2] == exp_a and data[end - 1] == exp_b)
            else:
                ck_ok = False

            key = (cls, mid)
            if key not in msgs:
                msgs[key] = []
            msgs[key].append({
                'time': times[i], 'length': length, 'idx': i,
                'complete': complete, 'ck_ok': ck_ok, 'full_len': 6 + length + 2
            })
            i = end if complete else i + 1
        else:
            i += 1

    # Print message summary
    print(f"\nUBX messages:")
    for (cls, mid), entries in sorted(msgs.items()):
        cls_name = CLASS_NAMES.get(cls, f"0x{cls:02X}")
        msg_name = MSG_NAMES.get((cls, mid), f"0x{mid:02X}")
        bad = sum(1 for e in entries if not e['complete'] or not e['ck_ok'])
        bad_str = f"  ({bad} BAD)" if bad else ""
        print(f"  {cls_name}-{msg_name:12s} ({cls:02X}-{mid:02X}): {len(entries):4d} msgs, payload={entries[0]['length']}{bad_str}")

    # SEC-SIGN details
    sec_key = (0x27, 0x04)
    if sec_key not in msgs:
        print(f"\n  SEC-SIGN not found on this channel")
        return

    entries = msgs[sec_key]
    print(f"\n--- SEC-SIGN: {len(entries)} signatures ---")
    print(f"{'#':>4}  {'Time(s)':>12}  {'Interval':>10}  {'Len':>4}  {'Size':>5}  {'CRC':>4}")
    print(f"{'─' * 4}  {'─' * 12}  {'─' * 10}  {'─' * 4}  {'─' * 5}  {'─' * 4}")

    bad_count = 0
    for i, e in enumerate(entries):
        interval = ""
        if i > 0:
            dt = (e['time'] - entries[i - 1]['time']) * 1000
            interval = f"{dt:.1f}ms"

        ok = e['complete'] and e['ck_ok'] and e['length'] == 108
        if not ok:
            bad_count += 1

        # Print all if few, otherwise first/last 3 + bad ones
        show = (len(entries) <= 20) or (i < 3) or (i >= len(entries) - 3) or (not ok)
        if show:
            mark = "" if ok else " <<<BAD"
            print(f"{i + 1:4d}  {e['time']:12.6f}  {interval:>10}  {e['length']:4d}  {e['full_len']:5d}  {'OK' if e['ck_ok'] else 'FAIL':>4}{mark}")
        elif i == 3:
            print(f"  ... ({len(entries) - 6} more) ...")

    # Stats
    if len(entries) > 1:
        intervals = [(entries[i]['time'] - entries[i - 1]['time']) * 1000 for i in range(1, len(entries))]
        print(f"\nStatistics:")
        print(f"  Total:   {len(entries)},  OK: {len(entries) - bad_count},  Bad: {bad_count}")
        print(f"  Payload: {sorted(set(e['length'] for e in entries))}")
        print(f"  Sizes:   {sorted(set(e['full_len'] for e in entries))}")
        print(f"  Min interval: {min(intervals):.1f} ms")
        print(f"  Max interval: {max(intervals):.1f} ms")
        print(f"  Avg interval: {sum(intervals) / len(intervals):.1f} ms")
        print(f"  First at: {entries[0]['time']:.6f} s")
        print(f"  Last at:  {entries[-1]['time']:.6f} s")


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else '/home/danik/Projects/ublox_fake_rust/logs_analiser/DSLogic U2Pro16-la-260319-114030.csv'

    print(f"Loading {filename}...")
    ch_times, ch_states = load_transitions(filename)

    for ch in sorted(ch_times.keys()):
        n = len(ch_times[ch])
        t_range = f"{ch_times[ch][0]:.3f}-{ch_times[ch][-1]:.3f}s" if n else "empty"
        print(f"  Channel {ch}: {n} transitions  ({t_range})")

    CH_NAMES = {
        1: "Drone→GNSS (config)",
        2: "GNSS→Board (u-blox)",
        3: "Board→Drone (output)"
    }

    for ch in sorted(ch_times.keys()):
        name = CH_NAMES.get(ch, f"Channel {ch}")
        print(f"\nDecoding channel {ch}...")
        decoded = decode_uart_channel(ch_times[ch], ch_states[ch])
        analyze_channel(decoded, f"Channel {ch}: {name}")


if __name__ == '__main__':
    main()
