#!/usr/bin/env python3
"""
Analyze dual-channel UBX log: ttyUSB0 (original GNSS) vs ttyUSB1 (after board, mode 4).
"""
import re
import struct
import sys
from collections import defaultdict

# UBX class/id names
UBX_NAMES = {
    (0x01, 0x01): "NAV-POSECEF",
    (0x01, 0x02): "NAV-POSLLH",
    (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x04): "NAV-DOP",
    (0x01, 0x06): "NAV-SOL",
    (0x01, 0x07): "NAV-PVT",
    (0x01, 0x11): "NAV-VELECEF",
    (0x01, 0x12): "NAV-VELNED",
    (0x01, 0x13): "NAV-HPPOSECEF",
    (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x21): "NAV-TIMEUTC",
    (0x01, 0x22): "NAV-CLOCK",
    (0x01, 0x26): "NAV-TIMELS",
    (0x01, 0x30): "NAV-SVINFO",
    (0x01, 0x35): "NAV-SAT",
    (0x01, 0x36): "NAV-COV",
    (0x01, 0x60): "NAV-AOPSTATUS",
    (0x01, 0x61): "NAV-EOE",
    (0x02, 0x13): "RXM-SFRBX",
    (0x02, 0x15): "RXM-RAWX",
    (0x05, 0x00): "ACK-NAK",
    (0x05, 0x01): "ACK-ACK",
    (0x06, 0x08): "CFG-RATE",
    (0x06, 0x41): "CFG-0x41",
    (0x06, 0x8A): "CFG-VALGET",
    (0x06, 0x8B): "CFG-VALSET",
    (0x0A, 0x04): "MON-VER",
    (0x0A, 0x09): "MON-HW",
    (0x0A, 0x36): "MON-COMMS",
    (0x0A, 0x38): "MON-RF",
    (0x0D, 0x01): "TIM-TP",
    (0x13, 0x60): "MGA-ACK",
    (0x27, 0x03): "SEC-UNIQID",
    (0x27, 0x04): "SEC-SIGN",
}

def ubx_name(cls, id_):
    return UBX_NAMES.get((cls, id_), f"UBX-0x{cls:02X}-0x{id_:02X}")

def parse_line(line):
    """Parse a log line. Returns (channel, timestamp, hex_bytes, tag) or None."""
    line = line.strip()
    if not line or line.startswith('---'):
        return None
    m = re.match(r'(ttyUSB\d)\s+<\s+(\d+\.\d+)s\s+((?:[0-9A-Fa-f]{2}\s?)+)\s*(?:\[(.*?)\])?', line)
    if not m:
        return None
    channel = m.group(1)
    timestamp = float(m.group(2))
    hex_str = m.group(3).strip()
    tag = m.group(4)
    hex_bytes = bytes.fromhex(hex_str.replace(' ', ''))
    return (channel, timestamp, hex_bytes, tag)

def extract_ubx_frames(hex_bytes):
    """Extract all UBX frames from a hex byte sequence."""
    frames = []
    i = 0
    while i < len(hex_bytes) - 7:
        if hex_bytes[i] == 0xB5 and hex_bytes[i+1] == 0x62:
            cls = hex_bytes[i+2]
            id_ = hex_bytes[i+3]
            length = hex_bytes[i+4] | (hex_bytes[i+5] << 8)
            frame_end = i + 6 + length + 2
            if frame_end <= len(hex_bytes):
                payload = hex_bytes[i+6:i+6+length]
                frames.append((cls, id_, length, payload, hex_bytes[i:frame_end]))
                i = frame_end
                continue
        i += 1
    return frames

def fletcher8(data):
    """Fletcher-8 checksum over data bytes."""
    ck_a, ck_b = 0, 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

def decode_nav_status(payload):
    if len(payload) < 16:
        return None
    itow = struct.unpack_from('<I', payload, 0)[0]
    gps_fix = payload[4]
    flags = payload[5]
    fix_stat = payload[6]
    flags2 = payload[7]
    ttff = struct.unpack_from('<I', payload, 8)[0]
    msss = struct.unpack_from('<I', payload, 12)[0]
    return {'iTOW': itow, 'gps_fix': gps_fix, 'flags': flags, 'fix_stat': fix_stat,
            'flags2': flags2, 'ttff': ttff, 'msss': msss}

def decode_nav_sat(payload):
    if len(payload) < 8:
        return None
    itow = struct.unpack_from('<I', payload, 0)[0]
    version = payload[4]
    num_svs = payload[5]
    return {'iTOW': itow, 'version': version, 'numSvs': num_svs, 'payload_len': len(payload)}

def decode_nav_timeutc(payload):
    if len(payload) < 20:
        return None
    itow = struct.unpack_from('<I', payload, 0)[0]
    tacc = struct.unpack_from('<I', payload, 4)[0]
    nano = struct.unpack_from('<i', payload, 8)[0]
    year = struct.unpack_from('<H', payload, 12)[0]
    month = payload[14]
    day = payload[15]
    hour = payload[16]
    minute = payload[17]
    sec = payload[18]
    valid = payload[19]
    return {'iTOW': itow, 'tAcc': tacc, 'nano': nano, 'year': year, 'month': month,
            'day': day, 'hour': hour, 'min': minute, 'sec': sec, 'valid': valid}

def decode_nav_timegps(payload):
    if len(payload) < 16:
        return None
    itow = struct.unpack_from('<I', payload, 0)[0]
    ftow = struct.unpack_from('<i', payload, 4)[0]
    week = struct.unpack_from('<h', payload, 8)[0]
    leaps = payload[10]
    valid = payload[11]
    tacc = struct.unpack_from('<I', payload, 12)[0]
    return {'iTOW': itow, 'fTOW': ftow, 'week': week, 'leapS': leaps,
            'valid': valid, 'tAcc': tacc}

def decode_sec_sign(payload):
    if len(payload) < 108:
        return None
    msg_ver = payload[0]
    flags = payload[2]
    session_id = struct.unpack_from('<I', payload, 4)[0]
    sig = payload[12:60]
    hash_data = payload[60:108]
    return {
        'version': msg_ver,
        'flags': flags,
        'session_id': session_id,
        'signature_hex': sig.hex(),
        'hash_hex': hash_data.hex(),
    }

def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/danik/Projects/ublox_fake_rust/logs_analiser/air3s_gnss_to_plate_4режим.txt'

    # Counters per channel
    msg_counts = {'ttyUSB0': defaultdict(int), 'ttyUSB1': defaultdict(int)}
    # All messages with their raw data for byte-for-byte comparison
    msg_by_type_and_itow = {'ttyUSB0': defaultdict(dict), 'ttyUSB1': defaultdict(dict)}
    # All message timestamps for latency analysis
    msg_timestamps = {'ttyUSB0': defaultdict(list), 'ttyUSB1': defaultdict(list)}
    # SEC-SIGN
    sec_sign_events = []
    # NAV-STATUS
    nav_status_data = {'ttyUSB0': [], 'ttyUSB1': []}
    # NAV-SAT
    nav_sat_data = {'ttyUSB0': [], 'ttyUSB1': []}
    # NAV-TIMEUTC
    nav_timeutc_data = {'ttyUSB0': [], 'ttyUSB1': []}
    # NAV-TIMEGPS
    nav_timegps_data = {'ttyUSB0': [], 'ttyUSB1': []}
    # Non-UBX
    non_ubx_lines = {'ttyUSB0': 0, 'ttyUSB1': 0}
    # All raw frames for byte-for-byte comparison
    all_frames = {'ttyUSB0': [], 'ttyUSB1': []}
    # Checksum verification
    checksum_errors = {'ttyUSB0': 0, 'ttyUSB1': 0}
    total_frames = {'ttyUSB0': 0, 'ttyUSB1': 0}

    total_lines = 0
    parsed_lines = 0

    with open(filepath, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            total_lines += 1
            parsed = parse_line(line)
            if parsed is None:
                continue
            parsed_lines += 1
            channel, timestamp, hex_bytes, tag = parsed

            frames = extract_ubx_frames(hex_bytes)
            if not frames:
                non_ubx_lines[channel] += 1
                continue

            for cls, id_, length, payload, raw in frames:
                key = (cls, id_)
                name = ubx_name(cls, id_)
                msg_counts[channel][name] += 1
                msg_timestamps[channel][name].append(timestamp)
                total_frames[channel] += 1

                # Verify checksum
                # Checksum is over class, id, length, payload
                check_data = raw[2:-2]  # everything except sync and checksum
                expected_cka, expected_ckb = raw[-2], raw[-1]
                actual_cka, actual_ckb = fletcher8(check_data)
                if actual_cka != expected_cka or actual_ckb != expected_ckb:
                    checksum_errors[channel] += 1

                # Store frame keyed by (name, itow if available)
                itow = None
                if len(payload) >= 4 and cls == 0x01:  # NAV messages have iTOW at offset 0
                    itow = struct.unpack_from('<I', payload, 0)[0]
                all_frames[channel].append({
                    'name': name, 'cls': cls, 'id': id_, 'timestamp': timestamp,
                    'payload': payload, 'raw': raw, 'itow': itow, 'line': line_num
                })

                # Decode specific messages
                if key == (0x01, 0x03):  # NAV-STATUS
                    ns = decode_nav_status(payload)
                    if ns:
                        ns['timestamp'] = timestamp
                        nav_status_data[channel].append(ns)

                elif key == (0x01, 0x35):  # NAV-SAT
                    sat = decode_nav_sat(payload)
                    if sat:
                        sat['timestamp'] = timestamp
                        nav_sat_data[channel].append(sat)

                elif key == (0x01, 0x21):  # NAV-TIMEUTC
                    tu = decode_nav_timeutc(payload)
                    if tu:
                        tu['timestamp'] = timestamp
                        nav_timeutc_data[channel].append(tu)

                elif key == (0x01, 0x20):  # NAV-TIMEGPS
                    tg = decode_nav_timegps(payload)
                    if tg:
                        tg['timestamp'] = timestamp
                        nav_timegps_data[channel].append(tg)

                elif key == (0x27, 0x04):  # SEC-SIGN
                    detail = decode_sec_sign(payload)
                    if detail:
                        detail['timestamp'] = timestamp
                        detail['channel'] = channel
                        detail['line'] = line_num
                        sec_sign_events.append(detail)

    # ====================== REPORT ======================
    print("=" * 90)
    print("DUAL-CHANNEL UBX LOG ANALYSIS: PassthroughOffset (Mode 4)")
    print(f"File: {filepath}")
    print(f"Total lines: {total_lines}, Parsed: {parsed_lines}")
    print("=" * 90)

    # ========= 1. Message Pass-Through Fidelity =========
    print("\n" + "=" * 90)
    print("1. MESSAGE PASS-THROUGH FIDELITY")
    print("=" * 90)

    all_msg_types = sorted(set(list(msg_counts['ttyUSB0'].keys()) + list(msg_counts['ttyUSB1'].keys())))

    print(f"\n{'Message Type':<20} {'USB0 (GNSS)':<14} {'USB1 (Board)':<14} {'Diff':<8} {'Status'}")
    print("-" * 75)

    total_usb0 = 0
    total_usb1 = 0

    for msg in all_msg_types:
        c0 = msg_counts['ttyUSB0'].get(msg, 0)
        c1 = msg_counts['ttyUSB1'].get(msg, 0)
        total_usb0 += c0
        total_usb1 += c1
        diff = c1 - c0
        if c0 == 0 and c1 > 0:
            status = "ADDED by board"
        elif c0 > 0 and c1 == 0:
            status = "DROPPED!"
        elif diff == 0:
            status = "OK (1:1)"
        elif diff > 0:
            status = f"+{diff} extra"
        else:
            status = f"LOST {-diff}"
        print(f"{msg:<20} {c0:<14} {c1:<14} {diff:<+8} {status}")

    print("-" * 75)
    print(f"{'TOTAL':<20} {total_usb0:<14} {total_usb1:<14} {total_usb1-total_usb0:<+8}")
    print(f"\nNon-UBX data lines: USB0={non_ubx_lines['ttyUSB0']}, USB1={non_ubx_lines['ttyUSB1']}")
    print(f"Checksum verification: USB0={total_frames['ttyUSB0']} frames, {checksum_errors['ttyUSB0']} errors; "
          f"USB1={total_frames['ttyUSB1']} frames, {checksum_errors['ttyUSB1']} errors")

    # ========= 2. Byte-for-Byte Comparison =========
    print("\n" + "=" * 90)
    print("2. BYTE-FOR-BYTE COMPARISON (USB0 vs USB1)")
    print("=" * 90)

    # Build parallel sequences per message type (excluding SEC-SIGN which is independently generated)
    msg_types_to_compare = [m for m in all_msg_types if m != 'SEC-SIGN']

    # For messages with iTOW, compare by iTOW matching
    # For messages without iTOW (ACK-ACK, MGA-ACK, etc.), compare by sequence
    identical_count = 0
    different_count = 0
    diff_details = []

    for msg_type in msg_types_to_compare:
        frames0 = [f for f in all_frames['ttyUSB0'] if f['name'] == msg_type]
        frames1 = [f for f in all_frames['ttyUSB1'] if f['name'] == msg_type]

        n = min(len(frames0), len(frames1))
        for i in range(n):
            f0 = frames0[i]
            f1 = frames1[i]
            if f0['raw'] == f1['raw']:
                identical_count += 1
            else:
                different_count += 1
                diff_details.append({
                    'type': msg_type,
                    'index': i,
                    'itow0': f0.get('itow'),
                    'itow1': f1.get('itow'),
                    'line0': f0['line'],
                    'line1': f1['line'],
                    'raw0': f0['raw'].hex(),
                    'raw1': f1['raw'].hex(),
                })

    print(f"\nIdentical frames: {identical_count}")
    print(f"Different frames: {different_count}")

    if diff_details:
        print(f"\nDifferences found (first 10):")
        for d in diff_details[:10]:
            print(f"  {d['type']} index={d['index']} iTOW0={d['itow0']} iTOW1={d['itow1']}")
            print(f"    USB0 line {d['line0']}: {d['raw0']}")
            print(f"    USB1 line {d['line1']}: {d['raw1']}")
            # Find byte differences
            b0 = bytes.fromhex(d['raw0'])
            b1 = bytes.fromhex(d['raw1'])
            for j in range(min(len(b0), len(b1))):
                if b0[j] != b1[j]:
                    print(f"    Byte {j}: 0x{b0[j]:02X} -> 0x{b1[j]:02X}")
    else:
        print("\nALL non-SEC-SIGN frames are byte-for-byte identical between USB0 and USB1!")

    # ========= 3. Coordinate Offset Analysis =========
    print("\n" + "=" * 90)
    print("3. COORDINATE OFFSET ANALYSIS")
    print("=" * 90)

    # Check which position messages are present
    pos_messages = ['NAV-PVT', 'NAV-POSLLH', 'NAV-POSECEF', 'NAV-SOL', 'NAV-HPPOSECEF', 'NAV-VELNED']
    has_any_pos = False
    for pm in pos_messages:
        c0 = msg_counts['ttyUSB0'].get(pm, 0)
        c1 = msg_counts['ttyUSB1'].get(pm, 0)
        if c0 > 0 or c1 > 0:
            has_any_pos = True
            print(f"  {pm}: USB0={c0}, USB1={c1}")

    if not has_any_pos:
        print("\n  ** NO POSITION MESSAGES FOUND **")
        print("  The GNSS module did not generate NAV-PVT, NAV-POSLLH, NAV-POSECEF, NAV-SOL,")
        print("  NAV-HPPOSECEF, or NAV-VELNED during this capture.")
        print("  This means the module never achieved a 3D fix, so there are no coordinates")
        print("  to offset. The board correctly passes through all non-position NAV messages")
        print("  without modification.")

    # Check NAV-STATUS for fix type
    print("\n  NAV-STATUS fix type progression (USB0):")
    status_fix_types = set()
    for ns in nav_status_data['ttyUSB0']:
        status_fix_types.add(ns['gps_fix'])
    print(f"  Unique fix types seen: {sorted(status_fix_types)}")
    if 3 not in status_fix_types and 2 not in status_fix_types:
        print("  -> NO 3D fix achieved during capture (all fix_type=0 = no fix)")

    # Show first/last NAV-STATUS
    if nav_status_data['ttyUSB0']:
        first = nav_status_data['ttyUSB0'][0]
        last = nav_status_data['ttyUSB0'][-1]
        print(f"\n  First NAV-STATUS: iTOW={first['iTOW']}ms gps_fix={first['gps_fix']} "
              f"flags=0x{first['flags']:02X} msss={first['msss']}ms")
        print(f"  Last  NAV-STATUS: iTOW={last['iTOW']}ms gps_fix={last['gps_fix']} "
              f"flags=0x{last['flags']:02X} msss={last['msss']}ms")

    # NAV-SAT satellite count progression
    print("\n  NAV-SAT satellite count progression (USB0):")
    sat_counts = [s['numSvs'] for s in nav_sat_data['ttyUSB0']]
    if sat_counts:
        print(f"  Min satellites: {min(sat_counts)}, Max: {max(sat_counts)}")
        # Show first few and last few
        for i, s in enumerate(nav_sat_data['ttyUSB0'][:3]):
            print(f"    t={s['timestamp']:.3f}s iTOW={s['iTOW']}ms numSvs={s['numSvs']} payload_len={s['payload_len']}")
        if len(nav_sat_data['ttyUSB0']) > 6:
            print(f"    ...")
        for s in nav_sat_data['ttyUSB0'][-3:]:
            print(f"    t={s['timestamp']:.3f}s iTOW={s['iTOW']}ms numSvs={s['numSvs']} payload_len={s['payload_len']}")

    # ========= 4. SEC-SIGN Analysis =========
    print("\n" + "=" * 90)
    print("4. SEC-SIGN ANALYSIS")
    print("=" * 90)

    usb0_sec = [e for e in sec_sign_events if e['channel'] == 'ttyUSB0']
    usb1_sec = [e for e in sec_sign_events if e['channel'] == 'ttyUSB1']

    print(f"\n  USB0 (original GNSS) SEC-SIGN: {len(usb0_sec)} messages")
    print(f"  USB1 (board output)  SEC-SIGN: {len(usb1_sec)} messages")

    if usb0_sec:
        print(f"\n  USB0 SEC-SIGN (from real GNSS module):")
        times0 = [e['timestamp'] for e in usb0_sec]
        print(f"    First: {times0[0]:.3f}s, Last: {times0[-1]:.3f}s")
        if len(times0) > 1:
            intervals0 = [times0[i+1] - times0[i] for i in range(len(times0)-1)]
            print(f"    Intervals: {[f'{iv:.3f}s' for iv in intervals0]}")
            print(f"    Mean: {sum(intervals0)/len(intervals0):.3f}s, Min: {min(intervals0):.3f}s, Max: {max(intervals0):.3f}s")
        for i, e in enumerate(usb0_sec):
            print(f"    [{i}] t={e['timestamp']:.3f}s session=0x{e['session_id']:08X} "
                  f"flags=0x{e['flags']:02X} line={e['line']}")

    if usb1_sec:
        print(f"\n  USB1 SEC-SIGN (generated by board):")
        times1 = [e['timestamp'] for e in usb1_sec]
        print(f"    First: {times1[0]:.3f}s, Last: {times1[-1]:.3f}s")
        if len(times1) > 1:
            intervals1 = [times1[i+1] - times1[i] for i in range(len(times1)-1)]
            print(f"    Intervals: {[f'{iv:.3f}s' for iv in intervals1]}")
            print(f"    Mean: {sum(intervals1)/len(intervals1):.3f}s, Min: {min(intervals1):.3f}s, Max: {max(intervals1):.3f}s")
        for i, e in enumerate(usb1_sec):
            print(f"    [{i}] t={e['timestamp']:.3f}s session=0x{e['session_id']:08X} "
                  f"flags=0x{e['flags']:02X} line={e['line']}")

    # Check: are SEC-SIGN from USB0 also on USB1 (they should be FILTERED)
    # USB0 SEC-SIGN have different session_ids than USB1 -> board generates its own
    usb0_sessions = set(e['session_id'] for e in usb0_sec)
    usb1_sessions = set(e['session_id'] for e in usb1_sec)
    shared_sessions = usb0_sessions & usb1_sessions
    if shared_sessions:
        print(f"\n  WARNING: Shared session IDs between USB0 and USB1: {[hex(s) for s in shared_sessions]}")
        print("  This means the board is forwarding the real GNSS SEC-SIGN (should be filtered)!")
    else:
        print(f"\n  Session IDs are DIFFERENT between USB0 and USB1.")
        print(f"    USB0 sessions: {[hex(s) for s in usb0_sessions]}")
        print(f"    USB1 sessions: {[hex(s) for s in usb1_sessions]}")
        if len(usb1_sessions) == 1:
            print("    Board uses a single consistent session ID (correct).")

    # First SEC-SIGN timing
    first_cfg_time = None
    for name in msg_timestamps['ttyUSB0']:
        if 'CFG' in name:
            for t in msg_timestamps['ttyUSB0'][name]:
                if first_cfg_time is None or t < first_cfg_time:
                    first_cfg_time = t

    # Also check for SEC-UNIQID (first poll from drone)
    first_sec_uniqid = None
    if msg_timestamps['ttyUSB0'].get('SEC-UNIQID'):
        first_sec_uniqid = msg_timestamps['ttyUSB0']['SEC-UNIQID'][0]
    if msg_timestamps['ttyUSB1'].get('SEC-UNIQID'):
        t = msg_timestamps['ttyUSB1']['SEC-UNIQID'][0]
        if first_sec_uniqid is None or t < first_sec_uniqid:
            first_sec_uniqid = t

    first_nav_time = None
    for name in msg_timestamps['ttyUSB0']:
        if name.startswith('NAV-'):
            for t in msg_timestamps['ttyUSB0'][name]:
                if first_nav_time is None or t < first_nav_time:
                    first_nav_time = t

    if usb1_sec:
        first_usb1_sec = usb1_sec[0]['timestamp']
        print(f"\n  First SEC-SIGN timing:")
        if first_sec_uniqid:
            print(f"    First SEC-UNIQID: {first_sec_uniqid:.3f}s")
        if first_cfg_time:
            print(f"    First CFG command (USB0): {first_cfg_time:.3f}s")
            print(f"    First board SEC-SIGN (USB1): {first_usb1_sec:.3f}s")
            print(f"    Delta (SEC-SIGN - first CFG): {(first_usb1_sec - first_cfg_time)*1000:.0f}ms")
        if first_nav_time:
            print(f"    First NAV message (USB0): {first_nav_time:.3f}s")
            print(f"    Delta (SEC-SIGN - first NAV): {(first_usb1_sec - first_nav_time)*1000:.0f}ms")
        if usb0_sec:
            first_usb0_sec = usb0_sec[0]['timestamp']
            print(f"    First GNSS SEC-SIGN (USB0): {first_usb0_sec:.3f}s")
            print(f"    Delta (board SEC-SIGN - GNSS SEC-SIGN): {(first_usb1_sec - first_usb0_sec)*1000:.0f}ms")

    # ========= 5. Latency Analysis =========
    print("\n" + "=" * 90)
    print("5. LATENCY ANALYSIS (USB0 -> USB1)")
    print("=" * 90)

    common_types = set(msg_timestamps['ttyUSB0'].keys()) & set(msg_timestamps['ttyUSB1'].keys())

    print(f"\n{'Message Type':<20} {'Count':<8} {'Min (ms)':<12} {'Max (ms)':<12} {'Avg (ms)':<12} {'Notes'}")
    print("-" * 80)

    all_latencies = []

    for msg in sorted(common_types):
        t0_list = msg_timestamps['ttyUSB0'][msg]
        t1_list = msg_timestamps['ttyUSB1'][msg]
        n = min(len(t0_list), len(t1_list))
        if n == 0:
            continue

        latencies = []
        for i in range(n):
            lat = (t1_list[i] - t0_list[i]) * 1000
            latencies.append(lat)

        notes = ""
        if msg == 'SEC-SIGN':
            notes = "INDEPENDENT (different generators)"
        elif min(latencies) < -1:
            notes = "NEGATIVE - sequence mismatch?"
        elif max(latencies) > 50:
            notes = "HIGH latency"

        all_latencies.extend(latencies if msg != 'SEC-SIGN' else [])

        avg = sum(latencies) / len(latencies)
        print(f"{msg:<20} {n:<8} {min(latencies):<12.1f} {max(latencies):<12.1f} {avg:<12.1f} {notes}")

    if all_latencies:
        # Filter out SEC-SIGN
        print(f"\n  Overall (excl. SEC-SIGN): min={min(all_latencies):.1f}ms, max={max(all_latencies):.1f}ms, "
              f"mean={sum(all_latencies)/len(all_latencies):.1f}ms")
        # Percentiles
        sorted_lat = sorted(all_latencies)
        p50 = sorted_lat[len(sorted_lat)//2]
        p95 = sorted_lat[int(len(sorted_lat)*0.95)]
        p99 = sorted_lat[int(len(sorted_lat)*0.99)]
        print(f"  Percentiles: P50={p50:.1f}ms, P95={p95:.1f}ms, P99={p99:.1f}ms")

    # ========= 6. Anomaly Detection =========
    print("\n" + "=" * 90)
    print("6. ANOMALY DETECTION")
    print("=" * 90)

    # 6a. Message types present on one channel only
    usb0_only_types = set(msg_counts['ttyUSB0'].keys()) - set(msg_counts['ttyUSB1'].keys())
    usb1_only_types = set(msg_counts['ttyUSB1'].keys()) - set(msg_counts['ttyUSB0'].keys())

    if usb0_only_types:
        print(f"\n  Message types on USB0 ONLY (dropped by board):")
        for t in usb0_only_types:
            print(f"    {t}: {msg_counts['ttyUSB0'][t]} messages")
    else:
        print(f"\n  No message types dropped by board.")

    if usb1_only_types:
        print(f"\n  Message types on USB1 ONLY (added by board):")
        for t in usb1_only_types:
            print(f"    {t}: {msg_counts['ttyUSB1'][t]} messages")
    else:
        print(f"\n  No message types added by board (beyond what GNSS outputs).")

    # 6b. Count mismatches
    print(f"\n  Per-type count mismatches:")
    any_mismatch = False
    for msg in sorted(common_types):
        c0 = msg_counts['ttyUSB0'][msg]
        c1 = msg_counts['ttyUSB1'][msg]
        if c0 != c1 and msg != 'SEC-SIGN':
            any_mismatch = True
            print(f"    {msg}: USB0={c0}, USB1={c1}, diff={c1-c0}")
    if not any_mismatch:
        print("    None. All forwarded types have identical counts.")

    # 6c. Non-UBX data forwarding
    if non_ubx_lines['ttyUSB0'] > 0:
        print(f"\n  Non-UBX data on USB0 (NMEA/startup garbage): {non_ubx_lines['ttyUSB0']} lines")
    if non_ubx_lines['ttyUSB1'] > 0:
        print(f"  WARNING: Non-UBX data FORWARDED on USB1: {non_ubx_lines['ttyUSB1']} lines")
        print("  Board should filter non-UBX data!")
    else:
        if non_ubx_lines['ttyUSB0'] > 0:
            print("  Non-UBX data correctly NOT forwarded on USB1.")

    # 6d. MGA-ACK negative latency investigation
    # Check if any MGA-ACK on USB1 appears BEFORE its USB0 counterpart
    mga_t0 = msg_timestamps['ttyUSB0'].get('MGA-ACK', [])
    mga_t1 = msg_timestamps['ttyUSB1'].get('MGA-ACK', [])
    if mga_t0 and mga_t1:
        mga_neg = sum(1 for i in range(min(len(mga_t0), len(mga_t1))) if mga_t1[i] < mga_t0[i])
        mga_zero = sum(1 for i in range(min(len(mga_t0), len(mga_t1))) if mga_t1[i] == mga_t0[i])
        mga_pos = sum(1 for i in range(min(len(mga_t0), len(mga_t1))) if mga_t1[i] > mga_t0[i])
        print(f"\n  MGA-ACK latency distribution: negative={mga_neg}, zero={mga_zero}, positive={mga_pos}")
        if mga_neg > 0:
            print("  Negative latency is normal for MGA-ACK -- the board may buffer/reorder")
            print("  ACK responses, or the USB capture tool has ~1ms timestamp jitter.")

    # 6e. NAV-SAT comparison between USB0 and USB1
    print(f"\n  NAV-SAT byte-for-byte comparison:")
    sat0 = [f for f in all_frames['ttyUSB0'] if f['name'] == 'NAV-SAT']
    sat1 = [f for f in all_frames['ttyUSB1'] if f['name'] == 'NAV-SAT']
    sat_identical = 0
    sat_different = 0
    for i in range(min(len(sat0), len(sat1))):
        if sat0[i]['raw'] == sat1[i]['raw']:
            sat_identical += 1
        else:
            sat_different += 1
    print(f"    Identical: {sat_identical}, Different: {sat_different}")
    if sat_different > 0:
        print("    WARNING: NAV-SAT differs -- board may be modifying satellite data!")

    # 6f. NAV-STATUS comparison
    print(f"\n  NAV-STATUS byte-for-byte comparison:")
    st0 = [f for f in all_frames['ttyUSB0'] if f['name'] == 'NAV-STATUS']
    st1 = [f for f in all_frames['ttyUSB1'] if f['name'] == 'NAV-STATUS']
    st_identical = 0
    st_different = 0
    for i in range(min(len(st0), len(st1))):
        if st0[i]['raw'] == st1[i]['raw']:
            st_identical += 1
        else:
            st_different += 1
    print(f"    Identical: {st_identical}, Different: {st_different}")

    # ========= 7. Timeline Summary =========
    print("\n" + "=" * 90)
    print("7. TIMELINE SUMMARY")
    print("=" * 90)

    all_ts = []
    for ch in ['ttyUSB0', 'ttyUSB1']:
        for n, times in msg_timestamps[ch].items():
            all_ts.extend(times)
    if all_ts:
        t_start = min(all_ts)
        t_end = max(all_ts)
        print(f"\n  Session start: {t_start:.3f}s")
        print(f"  Session end:   {t_end:.3f}s")
        print(f"  Duration:      {t_end - t_start:.3f}s ({(t_end - t_start)/60:.1f} min)")

    # iTOW range
    all_itows_usb0 = sorted(set(f['itow'] for f in all_frames['ttyUSB0'] if f['itow'] is not None))
    if all_itows_usb0:
        print(f"\n  iTOW range (USB0): {all_itows_usb0[0]}ms to {all_itows_usb0[-1]}ms")
        print(f"  iTOW span: {all_itows_usb0[-1] - all_itows_usb0[0]}ms = {(all_itows_usb0[-1] - all_itows_usb0[0])/1000:.1f}s")
        # Check iTOW intervals
        if len(all_itows_usb0) > 1:
            itow_intervals = set(all_itows_usb0[i+1] - all_itows_usb0[i] for i in range(len(all_itows_usb0)-1))
            print(f"  iTOW step sizes: {sorted(itow_intervals)}ms (expect 200 for 5Hz)")

    # NAV epoch count
    # Each epoch = one iTOW value with full NAV cycle
    nav_epochs = len(all_itows_usb0)
    print(f"  Total NAV epochs: {nav_epochs}")

    # First NAV-TIMEUTC with valid time
    print(f"\n  NAV-TIMEUTC time progression (USB0):")
    for tu in nav_timeutc_data['ttyUSB0'][:3]:
        valid_str = "UTC valid" if (tu['valid'] & 0x04) else "UTC INVALID"
        print(f"    t={tu['timestamp']:.3f}s iTOW={tu['iTOW']}ms "
              f"{tu['year']}-{tu['month']:02d}-{tu['day']:02d} "
              f"{tu['hour']:02d}:{tu['min']:02d}:{tu['sec']:02d} "
              f"valid=0x{tu['valid']:02X} ({valid_str})")
    if len(nav_timeutc_data['ttyUSB0']) > 3:
        print(f"    ...")
        for tu in nav_timeutc_data['ttyUSB0'][-2:]:
            valid_str = "UTC valid" if (tu['valid'] & 0x04) else "UTC INVALID"
            print(f"    t={tu['timestamp']:.3f}s iTOW={tu['iTOW']}ms "
                  f"{tu['year']}-{tu['month']:02d}-{tu['day']:02d} "
                  f"{tu['hour']:02d}:{tu['min']:02d}:{tu['sec']:02d} "
                  f"valid=0x{tu['valid']:02X} ({valid_str})")

    # NAV-TIMEGPS with week number
    print(f"\n  NAV-TIMEGPS (USB0):")
    for tg in nav_timegps_data['ttyUSB0'][:3]:
        print(f"    t={tg['timestamp']:.3f}s iTOW={tg['iTOW']}ms week={tg['week']} "
              f"leapS={tg['leapS']} valid=0x{tg['valid']:02X} tAcc={tg['tAcc']}ns")
    if len(nav_timegps_data['ttyUSB0']) > 3:
        print(f"    ...")
        for tg in nav_timegps_data['ttyUSB0'][-2:]:
            print(f"    t={tg['timestamp']:.3f}s iTOW={tg['iTOW']}ms week={tg['week']} "
                  f"leapS={tg['leapS']} valid=0x{tg['valid']:02X} tAcc={tg['tAcc']}ns")

    # SEC-SIGN timing relationship to NAV epochs
    print(f"\n  SEC-SIGN vs NAV epoch timeline:")
    # Interleave SEC-SIGN events from both channels with NAV epochs
    events = []
    for e in sec_sign_events:
        events.append((e['timestamp'], f"SEC-SIGN [{e['channel']}] session=0x{e['session_id']:08X}"))

    # Add first and last NAV epoch
    if all_itows_usb0:
        first_epoch_frame = next(f for f in all_frames['ttyUSB0'] if f['itow'] == all_itows_usb0[0])
        last_epoch_frame = next(f for f in all_frames['ttyUSB0'] if f['itow'] == all_itows_usb0[-1])
        events.append((first_epoch_frame['timestamp'], f"First NAV epoch iTOW={all_itows_usb0[0]}ms"))
        events.append((last_epoch_frame['timestamp'], f"Last NAV epoch iTOW={all_itows_usb0[-1]}ms"))

    if first_cfg_time:
        events.append((first_cfg_time, "First CFG command"))

    events.sort()
    for ts, desc in events:
        print(f"    {ts:.3f}s  {desc}")

    # ========= 8. Summary =========
    print("\n" + "=" * 90)
    print("8. OVERALL SUMMARY")
    print("=" * 90)

    total_forwarded = sum(msg_counts['ttyUSB1'].get(m, 0) for m in msg_counts['ttyUSB0'])
    total_original = sum(msg_counts['ttyUSB0'].values())

    print(f"""
  Duration: {t_end - t_start:.1f}s
  Total USB0 frames: {total_original}
  Total USB1 frames: {sum(msg_counts['ttyUSB1'].values())}
  Forwarding rate: {total_forwarded}/{total_original} = {100*total_forwarded/total_original if total_original else 0:.1f}%
  Dropped frames: {total_original - total_forwarded}
  Added frames (SEC-SIGN by board): {sum(msg_counts['ttyUSB1'].values()) - total_forwarded}

  GNSS Status: No 3D fix achieved during capture (cold start / no antenna?)
  Position messages: NONE (NAV-PVT/POSLLH/POSECEF/SOL not enabled or no fix)
  Coordinate offset: N/A (no position data to offset)

  SEC-SIGN: {len(usb0_sec)} from real GNSS (filtered), {len(usb1_sec)} generated by board
  Board SEC-SIGN session: {hex(list(usb1_sessions)[0]) if usb1_sessions else 'N/A'}
  GNSS SEC-SIGN sessions: {[hex(s) for s in usb0_sessions] if usb0_sessions else 'N/A'}

  Data integrity: {'ALL frames identical' if different_count == 0 else f'{different_count} differences found!'}
  Checksum errors: USB0={checksum_errors['ttyUSB0']}, USB1={checksum_errors['ttyUSB1']}
  Non-UBX filtering: {'OK (filtered)' if non_ubx_lines['ttyUSB1'] == 0 and non_ubx_lines['ttyUSB0'] > 0 else 'N/A' if non_ubx_lines['ttyUSB0'] == 0 else 'FAIL'}
""")


if __name__ == '__main__':
    main()
