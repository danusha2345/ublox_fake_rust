#!/usr/bin/env python3
"""Compare channels 2 and 3: find missing packets and check SEC-SIGN hash correspondence."""

import sys
import bisect

BAUD = 921600
BIT_PERIOD = 1.0 / BAUD

def load_transitions(filename):
    ch_times = {}
    ch_states = {}
    ch_prev = {}
    ch_cols = {}
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith(';') or not line:
                continue
            if line.startswith('Time'):
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

def extract_ubx_messages(decoded):
    """Extract all UBX messages with full payload bytes."""
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
                ck_data = data[i + 2:i + 6 + length]
                exp_a, exp_b = ubx_checksum(ck_data)
                ck_ok = (data[end - 2] == exp_a and data[end - 1] == exp_b)
                full_bytes = bytes(data[i:end])
                msgs.append({
                    'time': times[i],
                    'cls': cls, 'mid': mid,
                    'length': length,
                    'payload': payload,
                    'ck_ok': ck_ok,
                    'full_bytes': full_bytes,
                    'idx': i,
                })
                i = end
            else:
                i += 1
        else:
            i += 1
    return msgs


MSG_NAMES = {
    (0x01, 0x02): "NAV-POSLLH", (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x04): "NAV-DOP", (0x01, 0x07): "NAV-PVT",
    (0x01, 0x12): "NAV-VELNED", (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x21): "NAV-TIMEUTC", (0x01, 0x22): "NAV-CLOCK",
    (0x01, 0x35): "NAV-SAT", (0x01, 0x60): "NAV-AOPSTATUS",
    (0x02, 0x13): "RXM-SFRBX", (0x02, 0x15): "RXM-RAWX",
    (0x05, 0x01): "ACK-ACK", (0x06, 0x8B): "CFG-0x8B",
    (0x0A, 0x04): "MON-VER", (0x0A, 0x38): "MON-RF",
    (0x0D, 0x01): "TIM-TP", (0x13, 0x60): "MGA-ACK",
    (0x27, 0x03): "SEC-UNIQID", (0x27, 0x04): "SEC-SIGN",
}

def msg_name(cls, mid):
    return MSG_NAMES.get((cls, mid), f"{cls:02X}-{mid:02X}")


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else '/home/danik/Projects/ublox_fake_rust/logs_analiser/DSLogic U2Pro16-la-260319-114030.csv'

    print(f"Loading {filename}...")
    ch_times, ch_states = load_transitions(filename)

    print("Decoding channel 2 (u-blox)...")
    dec2 = decode_uart_channel(ch_times[2], ch_states[2])
    print(f"  {len(dec2)} bytes")

    print("Decoding channel 3 (our board)...")
    dec3 = decode_uart_channel(ch_times[3], ch_states[3])
    print(f"  {len(dec3)} bytes")

    msgs2 = extract_ubx_messages(dec2)
    msgs3 = extract_ubx_messages(dec3)
    print(f"\nChannel 2: {len(msgs2)} UBX messages")
    print(f"Channel 3: {len(msgs3)} UBX messages")

    # ── 1. Compare message counts per type ──
    print(f"\n{'=' * 70}")
    print("Message count comparison (ch2 vs ch3)")
    print(f"{'=' * 70}")

    types2 = {}
    types3 = {}
    for m in msgs2:
        k = (m['cls'], m['mid'])
        types2[k] = types2.get(k, 0) + 1
    for m in msgs3:
        k = (m['cls'], m['mid'])
        types3[k] = types3.get(k, 0) + 1

    all_types = sorted(set(list(types2.keys()) + list(types3.keys())))
    print(f"  {'Message':20s}  {'Ch2':>5}  {'Ch3':>5}  {'Diff':>5}")
    print(f"  {'─' * 20}  {'─' * 5}  {'─' * 5}  {'─' * 5}")
    for k in all_types:
        c2 = types2.get(k, 0)
        c3 = types3.get(k, 0)
        diff = c3 - c2
        mark = " <<<" if diff != 0 else ""
        print(f"  {msg_name(*k):20s}  {c2:5d}  {c3:5d}  {diff:+5d}{mark}")

    # ── 2. Find packet gaps: messages on ch2 not on ch3 ──
    # Group messages into epochs by NAV-PVT timestamp on each channel
    # Simpler: compare message-by-message for each type
    print(f"\n{'=' * 70}")
    print("Per-type timeline comparison (finding dropped packets)")
    print(f"{'=' * 70}")

    for msg_type in all_types:
        list2 = [m for m in msgs2 if (m['cls'], m['mid']) == msg_type]
        list3 = [m for m in msgs3 if (m['cls'], m['mid']) == msg_type]

        if len(list2) != len(list3):
            name = msg_name(*msg_type)
            print(f"\n  {name}: ch2={len(list2)}, ch3={len(list3)}, diff={len(list3) - len(list2)}")

            # Find which ones are missing by matching timestamps
            # Ch3 timestamps will be slightly later than ch2 (processing delay)
            # Match by finding closest ch3 time for each ch2 time
            times2 = [m['time'] for m in list2]
            times3 = [m['time'] for m in list3]

            # Find ch2 messages without a close match in ch3
            matched3 = set()
            unmatched2 = []
            for i2, t2 in enumerate(times2):
                # Find closest ch3 time
                best_j = None
                best_dt = 999
                for j3, t3 in enumerate(times3):
                    if j3 in matched3:
                        continue
                    dt = abs(t3 - t2)
                    if dt < best_dt:
                        best_dt = dt
                        best_j = j3
                if best_j is not None and best_dt < 0.5:  # within 500ms
                    matched3.add(best_j)
                else:
                    unmatched2.append((i2, t2))

            if unmatched2:
                print(f"    Missing from ch3 (present on ch2 only):")
                for idx, t in unmatched2:
                    print(f"      #{idx + 1} at {t:.6f}s")

            # Find ch3 messages without match in ch2
            matched2 = set()
            unmatched3 = []
            for j3, t3 in enumerate(times3):
                best_i = None
                best_dt = 999
                for i2, t2 in enumerate(times2):
                    if i2 in matched2:
                        continue
                    dt = abs(t3 - t2)
                    if dt < best_dt:
                        best_dt = dt
                        best_i = i2
                if best_i is not None and best_dt < 0.5:
                    matched2.add(best_i)
                else:
                    unmatched3.append((j3, t3))

            if unmatched3:
                print(f"    Extra on ch3 (not on ch2):")
                for idx, t in unmatched3:
                    print(f"      #{idx + 1} at {t:.6f}s")

    # ── 3. SEC-SIGN hash comparison ──
    print(f"\n{'=' * 70}")
    print("SEC-SIGN payload comparison (ch2 vs ch3)")
    print(f"{'=' * 70}")

    signs2 = [m for m in msgs2 if m['cls'] == 0x27 and m['mid'] == 0x04]
    signs3 = [m for m in msgs3 if m['cls'] == 0x27 and m['mid'] == 0x04]

    # SEC-SIGN payload structure (108 bytes):
    # 0x00: version (1)
    # 0x01: reserved (3)
    # 0x04: hash bytes or signature data
    # The exact layout: version(1), reserved(3), session_id(2), reserved(2),
    # msg_count(4), hash(32), signature(64)
    # Actually for u-blox M10 SEC-SIGN:
    # Bytes 0-3: version, reserved
    # Bytes 4-7: session/msg info
    # Bytes 8-11: msg_count or similar
    # Bytes 12-43: SHA256 hash (32 bytes)
    # Bytes 44-107: ECDSA signature (64 bytes)

    print(f"\nChannel 2: {len(signs2)} SEC-SIGN")
    print(f"Channel 3: {len(signs3)} SEC-SIGN")

    # Match by closest time
    print(f"\n{'#':>4}  {'Ch2 time':>12}  {'Ch3 time':>12}  {'Delay(ms)':>9}  {'Payload match':>14}  {'Details'}")
    print(f"{'─' * 4}  {'─' * 12}  {'─' * 12}  {'─' * 9}  {'─' * 14}  {'─' * 40}")

    j3 = 0
    for i2, s2 in enumerate(signs2):
        # Find closest ch3 sign
        best_j = None
        best_dt = 999
        for j in range(max(0, j3 - 2), min(len(signs3), j3 + 5)):
            dt = signs3[j]['time'] - s2['time']
            if abs(dt) < abs(best_dt):
                best_dt = dt
                best_j = j

        if best_j is not None and abs(best_dt) < 2.0:
            s3 = signs3[best_j]
            j3 = best_j + 1

            payload_match = s2['payload'] == s3['payload']

            details = ""
            if not payload_match:
                # Find first differing byte
                p2, p3 = s2['payload'], s3['payload']
                diffs = []
                for bi in range(min(len(p2), len(p3))):
                    if p2[bi] != p3[bi]:
                        diffs.append(bi)

                if diffs:
                    first_diff = diffs[0]
                    details = f"diff at byte {first_diff}, {len(diffs)} bytes differ"

                    # Show the differing regions
                    # Group consecutive diffs
                    regions = []
                    start = diffs[0]
                    prev = diffs[0]
                    for d in diffs[1:]:
                        if d == prev + 1:
                            prev = d
                        else:
                            regions.append((start, prev))
                            start = d
                            prev = d
                    regions.append((start, prev))
                    details += f"  regions: {regions}"

            mark = "SAME" if payload_match else "DIFFER"
            show = not payload_match or i2 < 3 or i2 >= len(signs2) - 3
            if show:
                print(f"{i2 + 1:4d}  {s2['time']:12.6f}  {s3['time']:12.6f}  {best_dt * 1000:9.1f}  {mark:>14}  {details}")
            elif i2 == 3:
                print(f"  ... (checking all, showing only DIFFER) ...")
        else:
            print(f"{i2 + 1:4d}  {s2['time']:12.6f}  {'—':>12}  {'—':>9}  {'NO MATCH':>14}")

    # ── 4. Detailed look at ALL intervals on ch3 ──
    print(f"\n{'=' * 70}")
    print("ALL SEC-SIGN intervals on Channel 3 (our board)")
    print(f"{'=' * 70}")
    print(f"{'#':>4}  {'Time(s)':>12}  {'Interval(ms)':>12}  {'Note'}")
    print(f"{'─' * 4}  {'─' * 12}  {'─' * 12}  {'─' * 20}")
    for i, s in enumerate(signs3):
        if i == 0:
            print(f"{i + 1:4d}  {s['time']:12.6f}  {'—':>12}")
        else:
            dt = (s['time'] - signs3[i - 1]['time']) * 1000
            note = ""
            if abs(dt - 2000) > 10:
                note = f"  <<< ANOMALY ({dt - 2000:+.1f}ms)"
            print(f"{i + 1:4d}  {s['time']:12.6f}  {dt:12.1f}{note}")


if __name__ == '__main__':
    main()
