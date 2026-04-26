#!/usr/bin/env python3
"""Detailed SEC-SIGN payload dump + check for non-UBX data leaking into hash stream."""

import sys
import bisect

BAUD = 921600
BIT_PERIOD = 1.0 / BAUD

def load_transitions(filename):
    ch_times = {}; ch_states = {}; ch_prev = {}; ch_cols = {}
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith(';') or not line: continue
            if line.startswith('Time'):
                parts = [p.strip() for p in line.split(',')]
                for col_idx, name in enumerate(parts[1:], start=1):
                    ch_cols[col_idx] = int(name)
                    ch_times[int(name)] = []; ch_states[int(name)] = []; ch_prev[int(name)] = None
                continue
            parts = line.split(',')
            if len(parts) < 2: continue
            t = float(parts[0])
            for col_idx, ch_name in ch_cols.items():
                if col_idx < len(parts):
                    val = int(parts[col_idx])
                    if val != ch_prev[ch_name]:
                        ch_times[ch_name].append(t); ch_states[ch_name].append(val); ch_prev[ch_name] = val
    return ch_times, ch_states

def get_state(times, states, t):
    idx = bisect.bisect_right(times, t) - 1
    return states[idx] if idx >= 0 else 0

def decode_uart_channel(times, states):
    if not times: return []
    result = []; n = len(times); i = 0; skip_until = 0.0
    while i < n:
        t = times[i]
        if t < skip_until: i += 1; continue
        s = states[i]
        if s == 0 and i > 0 and states[i - 1] == 1:
            start_t = t
            if get_state(times, states, start_t + 0.5 * BIT_PERIOD) != 0: i += 1; continue
            byte_val = 0
            for bit in range(8):
                bit_val = get_state(times, states, start_t + (1.5 + bit) * BIT_PERIOD)
                byte_val |= (bit_val << bit)
            if get_state(times, states, start_t + 9.5 * BIT_PERIOD) != 1:
                skip_until = start_t + 10.0 * BIT_PERIOD; i += 1; continue
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

def extract_all(decoded):
    """Return list of all UBX messages with index range in decoded stream."""
    data = [b for _, b in decoded]
    times = [t for t, _ in decoded]
    total = len(data)
    msgs = []
    i = 0
    while i < total - 5:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls, mid = data[i + 2], data[i + 3]
            length = data[i + 4] | (data[i + 5] << 8)
            end = i + 6 + length + 2
            if end <= total:
                payload = bytes(data[i + 6:i + 6 + length])
                full = bytes(data[i:end])
                ck_data = data[i + 2:i + 6 + length]
                exp_a, exp_b = ubx_checksum(ck_data)
                ck_ok = (data[end - 2] == exp_a and data[end - 1] == exp_b)
                msgs.append({
                    'time': times[i], 'cls': cls, 'mid': mid,
                    'length': length, 'payload': payload,
                    'full': full, 'ck_ok': ck_ok,
                    'start_idx': i, 'end_idx': end,
                })
                i = end
            else:
                i += 1
        else:
            i += 1
    return msgs


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else '/home/danik/Projects/ublox_fake_rust/logs_analiser/DSLogic U2Pro16-la-260319-114030.csv'

    print(f"Loading...")
    ch_times, ch_states = load_transitions(filename)

    print("Decoding ch2...")
    dec2 = decode_uart_channel(ch_times[2], ch_states[2])
    print("Decoding ch3...")
    dec3 = decode_uart_channel(ch_times[3], ch_states[3])

    msgs2 = extract_all(dec2)
    msgs3 = extract_all(dec3)

    # ── 1. Check for non-UBX gaps (NMEA etc) in ch3 stream ──
    print(f"\n{'=' * 70}")
    print("Non-UBX data check on ch3 (bytes between UBX messages)")
    print(f"{'=' * 70}")

    data3 = [b for _, b in dec3]
    covered = set()
    for m in msgs3:
        for j in range(m['start_idx'], m['end_idx']):
            covered.add(j)

    gaps = []
    gap_start = None
    for j in range(len(data3)):
        if j not in covered:
            if gap_start is None:
                gap_start = j
        else:
            if gap_start is not None:
                gap_bytes = bytes(data3[gap_start:j])
                gaps.append((gap_start, j, gap_bytes, dec3[gap_start][0] if gap_start < len(dec3) else 0))
                gap_start = None
    if gap_start is not None:
        gaps.append((gap_start, len(data3), bytes(data3[gap_start:]), dec3[gap_start][0]))

    if gaps:
        print(f"  Found {len(gaps)} non-UBX gaps:")
        for start, end, gap_bytes, t in gaps:
            hex_preview = ' '.join(f'{b:02X}' for b in gap_bytes[:50])
            ascii_preview = ''.join(chr(b) if 32 <= b < 127 else '.' for b in gap_bytes[:50])
            print(f"    [{start}-{end}) {len(gap_bytes)} bytes at ~{t:.3f}s")
            print(f"      hex:   {hex_preview}")
            print(f"      ascii: {ascii_preview}")
    else:
        print("  No non-UBX data on ch3 — clean UBX-only stream")

    # ── 2. Count messages between each SEC-SIGN pair on both channels ──
    print(f"\n{'=' * 70}")
    print("Messages between SEC-SIGN pairs (ch2 vs ch3)")
    print(f"{'=' * 70}")

    signs2 = [m for m in msgs2 if m['cls'] == 0x27 and m['mid'] == 0x04]
    signs3 = [m for m in msgs3 if m['cls'] == 0x27 and m['mid'] == 0x04]
    nonsign2 = [m for m in msgs2 if not (m['cls'] == 0x27 and m['mid'] == 0x04)]
    nonsign3 = [m for m in msgs3 if not (m['cls'] == 0x27 and m['mid'] == 0x04)]

    print(f"\n{'#':>4}  {'Ch3 time':>10}  {'Ch2 msgs':>8}  {'Ch3 msgs':>8}  {'Ch2 bytes':>9}  {'Ch3 bytes':>9}  {'Diff msgs':>9}  {'Diff bytes':>10}  Note")
    print(f"{'─' * 4}  {'─' * 10}  {'─' * 8}  {'─' * 8}  {'─' * 9}  {'─' * 9}  {'─' * 9}  {'─' * 10}  {'─' * 20}")

    for i in range(len(signs3)):
        if i == 0:
            t_start = 0
        else:
            t_start = signs3[i - 1]['time']
        t_end = signs3[i]['time']

        # Count non-SEC-SIGN messages in this interval on each channel
        cnt2 = 0; bytes2 = 0
        for m in nonsign2:
            if t_start < m['time'] <= t_end:
                cnt2 += 1
                bytes2 += len(m['full'])

        cnt3 = 0; bytes3 = 0
        for m in nonsign3:
            if t_start < m['time'] <= t_end:
                cnt3 += 1
                bytes3 += len(m['full'])

        diff_msgs = cnt3 - cnt2
        diff_bytes = bytes3 - bytes2
        note = ""
        if diff_msgs != 0:
            note = f"<<< {diff_msgs:+d} msgs"
        if diff_bytes != 0 and diff_msgs == 0:
            note = f"<<< {diff_bytes:+d} bytes"

        print(f"{i + 1:4d}  {t_end:10.3f}  {cnt2:8d}  {cnt3:8d}  {bytes2:9d}  {bytes3:9d}  {diff_msgs:+9d}  {diff_bytes:+10d}  {note}")

    # ── 3. Dump SEC-SIGN payloads for last 5 signatures ──
    print(f"\n{'=' * 70}")
    print("Last 5 SEC-SIGN payload dump (ch3 = our board)")
    print(f"{'=' * 70}")
    # SEC-SIGN payload (108 bytes):
    # [0]     version
    # [1-3]   reserved
    # [4-7]   message count (u32 LE)
    # [8-11]  session/reserved
    # [12-43] SHA-256 hash (32 bytes) — folded to 24 bytes for ECDSA
    # [44-107] ECDSA signature (48+padding or 64 bytes)
    # Actually for M10, the layout might be different. Let me just hex dump

    for i in range(max(0, len(signs3) - 5), len(signs3)):
        s = signs3[i]
        p = s['payload']
        print(f"\n  #{i + 1} at {s['time']:.6f}s (CRC {'OK' if s['ck_ok'] else 'FAIL'}):")
        for row in range(0, len(p), 16):
            chunk = p[row:row + 16]
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            print(f"    {row:3d}: {hex_str}")

    print(f"\n{'=' * 70}")
    print("Last 5 SEC-SIGN payload dump (ch2 = u-blox)")
    print(f"{'=' * 70}")
    for i in range(max(0, len(signs2) - 5), len(signs2)):
        s = signs2[i]
        p = s['payload']
        print(f"\n  #{i + 1} at {s['time']:.6f}s (CRC {'OK' if s['ck_ok'] else 'FAIL'}):")
        for row in range(0, len(p), 16):
            chunk = p[row:row + 16]
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            print(f"    {row:3d}: {hex_str}")

    # ── 4. Check bytes in ch3 stream that are between UBX messages ──
    # This catches any stray bytes that would affect hash
    print(f"\n{'=' * 70}")
    print("Byte count: ch2 total vs ch3 total (UBX only, excluding SEC-SIGN from u-blox)")
    print(f"{'=' * 70}")

    # Total UBX bytes on ch2 (excluding SEC-SIGN from u-blox)
    total_ubx2 = sum(len(m['full']) for m in msgs2 if not (m['cls'] == 0x27 and m['mid'] == 0x04))
    total_ubx3_nosign = sum(len(m['full']) for m in msgs3 if not (m['cls'] == 0x27 and m['mid'] == 0x04))
    total_ubx3_all = sum(len(m['full']) for m in msgs3)

    print(f"  Ch2 UBX bytes (excl SEC-SIGN): {total_ubx2}")
    print(f"  Ch3 UBX bytes (excl SEC-SIGN): {total_ubx3_nosign}")
    print(f"  Ch3 UBX bytes (all):           {total_ubx3_all}")
    print(f"  Difference (excl SEC-SIGN):     {total_ubx3_nosign - total_ubx2}")


if __name__ == '__main__':
    main()
