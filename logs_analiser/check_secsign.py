#!/usr/bin/env python3
"""Check SEC-SIGN integrity: verify each signature is 116 bytes (6 hdr + 108 payload + 2 cksum)."""

import sys
import bisect

BAUD = 921600
BIT_PERIOD = 1.0 / BAUD

def load_transitions(filename):
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
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return 0
    return states[idx]

def decode_uart_channel(times, states):
    if not times:
        return []
    result = []
    n = len(times)
    i = 0
    skip_until = 0.0
    while i < n:
        t = times[i]
        s = states[i]
        if t < skip_until:
            i += 1
            continue
        if s == 0 and i > 0 and states[i - 1] == 1:
            start_t = t
            start_center = start_t + 0.5 * BIT_PERIOD
            if get_state(times, states, start_center) != 0:
                i += 1
                continue
            byte_val = 0
            for bit in range(8):
                sample_t = start_t + (1.5 + bit) * BIT_PERIOD
                bit_val = get_state(times, states, sample_t)
                byte_val |= (bit_val << bit)
            stop_t = start_t + 9.5 * BIT_PERIOD
            if get_state(times, states, stop_t) != 1:
                skip_until = start_t + 10.0 * BIT_PERIOD
                i += 1
                continue
            result.append((start_t, byte_val))
            skip_until = start_t + 9.5 * BIT_PERIOD
            i += 1
        else:
            i += 1
    return result


def ubx_checksum(data):
    """Fletcher-8 checksum over class, id, length, payload."""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def check_sec_sign(decoded, ch_name):
    data = [b for _, b in decoded]
    times = [t for t, _ in decoded]
    total = len(data)

    print(f"\n{'=' * 70}")
    print(f"{ch_name}")
    print(f"{'=' * 70}")

    # Find all SEC-SIGN starts
    signatures = []
    i = 0
    while i < total - 5:
        if data[i] == 0xB5 and data[i + 1] == 0x62 and data[i + 2] == 0x27 and data[i + 3] == 0x04:
            length = data[i + 4] | (data[i + 5] << 8)
            full_len = 6 + length + 2  # sync(2) + class(1) + id(1) + len(2) + payload + cksum(2)

            # Extract full message bytes
            end_idx = i + full_len
            if end_idx <= total:
                msg_bytes = data[i:end_idx]
                msg_complete = True
            else:
                msg_bytes = data[i:total]
                msg_complete = False

            # Verify checksum (over class, id, length, payload)
            if msg_complete:
                ck_data = data[i + 2:i + 6 + length]  # class + id + length + payload
                expected_a, expected_b = ubx_checksum(ck_data)
                actual_a = data[end_idx - 2]
                actual_b = data[end_idx - 1]
                ck_ok = (expected_a == actual_a and expected_b == actual_b)
            else:
                ck_ok = False

            signatures.append({
                'idx': i,
                'time': times[i],
                'length_field': length,
                'full_len': full_len,
                'complete': msg_complete,
                'ck_ok': ck_ok,
                'bytes': msg_bytes,
            })
            i = end_idx if msg_complete else i + 1
        else:
            i += 1

    print(f"Found {len(signatures)} SEC-SIGN messages\n")

    # Print table
    print(f"{'#':>4}  {'Time(s)':>12}  {'Interval':>10}  {'LenField':>8}  {'Total':>5}  {'Complete':>8}  {'CRC':>5}  {'Notes'}")
    print(f"{'─' * 4}  {'─' * 12}  {'─' * 10}  {'─' * 8}  {'─' * 5}  {'─' * 8}  {'─' * 5}  {'─' * 20}")

    bad_count = 0
    for i, s in enumerate(signatures):
        interval = ""
        if i > 0:
            dt = (s['time'] - signatures[i - 1]['time']) * 1000
            interval = f"{dt:.1f}ms"

        notes = []
        if not s['complete']:
            notes.append("TRUNCATED")
            bad_count += 1
        if s['length_field'] != 108:
            notes.append(f"len={s['length_field']}!=108")
            bad_count += 1
        if s['complete'] and not s['ck_ok']:
            notes.append("BAD CRC")
            bad_count += 1
        if s['full_len'] != 116:
            notes.append(f"total={s['full_len']}!=116")

        status = "OK" if (s['complete'] and s['ck_ok'] and s['length_field'] == 108) else "FAIL"

        # Only print details for bad ones or first/last few
        if notes or i < 3 or i >= len(signatures) - 3:
            note_str = ", ".join(notes) if notes else ""
            print(f"{i + 1:4d}  {s['time']:12.6f}  {interval:>10}  {s['length_field']:8d}  {s['full_len']:5d}  {'yes' if s['complete'] else 'NO':>8}  {('OK' if s['ck_ok'] else 'FAIL'):>5}  {note_str}")
        elif i == 3:
            print(f"  ... ({len(signatures) - 6} more OK entries) ...")

    # Summary
    ok_count = len(signatures) - bad_count
    print(f"\nSummary:")
    print(f"  Total:    {len(signatures)}")
    print(f"  OK:       {ok_count}  (complete, 108-byte payload, valid CRC)")
    print(f"  Bad:      {bad_count}")

    # Check all lengths
    lengths = set(s['length_field'] for s in signatures)
    print(f"  Payload lengths seen: {sorted(lengths)}")
    totals = set(s['full_len'] for s in signatures)
    print(f"  Total sizes seen:     {sorted(totals)}")

    if bad_count > 0:
        print(f"\n  BAD SIGNATURES:")
        for i, s in enumerate(signatures):
            if not s['complete'] or not s['ck_ok'] or s['length_field'] != 108:
                hex_preview = ' '.join(f'{b:02X}' for b in s['bytes'][:20])
                print(f"    #{i + 1} at {s['time']:.6f}s: {hex_preview}...")


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else '/home/danik/Projects/ublox_fake_rust/logs_analiser/DSLogic_air3s.csv'

    print(f"Loading...")
    ch_times, ch_states = load_transitions(filename)

    for ch, name in [(2, "Channel 2: GNSS→Board (u-blox)"), (3, "Channel 3: Board→Drone (output)")]:
        print(f"\nDecoding channel {ch}...")
        decoded = decode_uart_channel(ch_times[ch], ch_states[ch])
        print(f"Decoded {len(decoded)} bytes")
        check_sec_sign(decoded, name)


if __name__ == '__main__':
    main()
