#!/usr/bin/env python3
"""
Analyze Mavic 3 Pro GNSS UART log.
Format: ttyUSBx < timestamp_s hex_bytes [MessageType]
"""

import re
import sys
from collections import defaultdict

LOG_FILE = "/home/danik/Projects/ublox_fake_rust/logs_analiser/mavic3pro_real_gnss_log_2026-02-28_19-17-02.txt"

# Parse all lines
line_re = re.compile(r'^(ttyUSB\d+)\s*<\s*([\d.]+)s\s+((?:[0-9A-Fa-f]{2}\s*)+)\[([^\]]+)\]')

gnss_msgs = []  # (timestamp, msg_type, hex_bytes)
drone_msgs = []  # (timestamp, msg_type, hex_bytes)

with open(LOG_FILE, 'r') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('---'):
            continue
        m = line_re.match(line)
        if not m:
            continue
        port = m.group(1)
        ts = float(m.group(2))
        hex_str = m.group(3).strip()
        msg_type = m.group(4)
        hex_bytes = bytes.fromhex(hex_str.replace(' ', ''))

        entry = (ts, msg_type, hex_bytes)
        if port == 'ttyUSB1':
            gnss_msgs.append(entry)
        elif port == 'ttyUSB0':
            drone_msgs.append(entry)

print(f"Total GNSS messages (ttyUSB1): {len(gnss_msgs)}")
print(f"Total Drone messages (ttyUSB0): {len(drone_msgs)}")
print()

# ============================================================
# 1. Message types from GNSS (ttyUSB1)
# ============================================================
print("=" * 60)
print("1. MESSAGE TYPES FROM GNSS (ttyUSB1)")
print("=" * 60)
gnss_counts = defaultdict(int)
for ts, msg_type, hb in gnss_msgs:
    gnss_counts[msg_type] += 1
for k, v in sorted(gnss_counts.items(), key=lambda x: -x[1]):
    print(f"  {k:30s}: {v}")
print()

# ============================================================
# 2. Message types from Drone (ttyUSB0)
# ============================================================
print("=" * 60)
print("2. MESSAGE TYPES FROM DRONE (ttyUSB0)")
print("=" * 60)
drone_counts = defaultdict(int)
for ts, msg_type, hb in drone_msgs:
    drone_counts[msg_type] += 1
for k, v in sorted(drone_counts.items(), key=lambda x: -x[1]):
    print(f"  {k:30s}: {v}")
print()

# ============================================================
# 3. NAV message rate (ttyUSB1 only)
# ============================================================
print("=" * 60)
print("3. NAV MESSAGE RATES (ttyUSB1)")
print("=" * 60)
nav_types_of_interest = [
    'NAV-0x01',  # POSECEF
    'NAV-0x03',  # STATUS
    'NAV-0x04',  # DOP
    'NAV-0x21',  # TIMEUTC
    'NAV-0x22',  # CLOCK
    'NAV-0x60',  # AOPSTATUS
    'NAV-0x35',  # SAT
]
# Also collect all NAV types
all_nav_types = sorted(set(t for ts, t, hb in gnss_msgs if t.startswith('NAV-')))

nav_timestamps = defaultdict(list)
for ts, msg_type, hb in gnss_msgs:
    if msg_type.startswith('NAV-'):
        nav_timestamps[msg_type].append(ts)

for nav_type in all_nav_types:
    tss = nav_timestamps[nav_type]
    if len(tss) < 2:
        print(f"  {nav_type:20s}: {len(tss)} message(s), not enough for interval")
        continue
    intervals = [tss[i+1] - tss[i] for i in range(len(tss)-1)]
    avg = sum(intervals) / len(intervals)
    mn = min(intervals)
    mx = max(intervals)
    print(f"  {nav_type:20s}: count={len(tss):4d}, avg_interval={avg:.3f}s, min={mn:.3f}s, max={mx:.3f}s")
print()

# ============================================================
# 4. SEC-SIGN (SEC-0x04) timing
# ============================================================
print("=" * 60)
print("4. SEC-SIGN (SEC-0x04) TIMING (ttyUSB1)")
print("=" * 60)
sec_sign_ts = [ts for ts, msg_type, hb in gnss_msgs if msg_type == 'SEC-0x04']
if sec_sign_ts:
    print(f"  Count: {len(sec_sign_ts)}")
    intervals = [sec_sign_ts[i+1] - sec_sign_ts[i] for i in range(len(sec_sign_ts)-1)]
    if intervals:
        avg = sum(intervals) / len(intervals)
        mn = min(intervals)
        mx = max(intervals)
        print(f"  Average period: {avg:.3f}s, min={mn:.3f}s, max={mx:.3f}s")
    print(f"  First SEC-SIGN at: {sec_sign_ts[0]:.3f}s")
    print(f"  Last  SEC-SIGN at: {sec_sign_ts[-1]:.3f}s")
    print(f"  All timestamps:")
    for i, t in enumerate(sec_sign_ts):
        if i == 0:
            print(f"    [{i:3d}] {t:.3f}s")
        else:
            print(f"    [{i:3d}] {t:.3f}s  (+{t - sec_sign_ts[i-1]:.3f}s)")
else:
    print("  No SEC-SIGN messages found.")
print()

# ============================================================
# 5. Timing from first drone command to first NAV message
# ============================================================
print("=" * 60)
print("5. TIMING: FIRST DRONE COMMAND -> FIRST NAV")
print("=" * 60)
first_drone_cmd = drone_msgs[0] if drone_msgs else None
first_nav_gnss = next(((ts, t, hb) for ts, t, hb in gnss_msgs if t.startswith('NAV-')), None)

if first_drone_cmd:
    print(f"  First drone command: {first_drone_cmd[1]} at {first_drone_cmd[0]:.3f}s")
if first_nav_gnss:
    print(f"  First NAV from GNSS: {first_nav_gnss[1]} at {first_nav_gnss[0]:.3f}s")
if first_drone_cmd and first_nav_gnss:
    print(f"  Delay: {first_nav_gnss[0] - first_drone_cmd[0]:.3f}s")
print()

# ============================================================
# 6. Timing from first NAV to first SEC-SIGN
# ============================================================
print("=" * 60)
print("6. TIMING: FIRST NAV -> FIRST SEC-SIGN")
print("=" * 60)
if first_nav_gnss and sec_sign_ts:
    delay = sec_sign_ts[0] - first_nav_gnss[0]
    print(f"  First NAV:      {first_nav_gnss[0]:.3f}s  ({first_nav_gnss[1]})")
    print(f"  First SEC-SIGN: {sec_sign_ts[0]:.3f}s")
    print(f"  Delay:          {delay:.3f}s")
else:
    print("  Not enough data.")
print()

# ============================================================
# 7. CFG-VALSET keys (CFG-0x8A from ttyUSB0)
# ============================================================
print("=" * 60)
print("7. CFG-VALSET KEYS (ttyUSB0, CFG-0x8A)")
print("=" * 60)

# Known key names (from CLAUDE.md and UBX M10 docs)
KNOWN_KEYS = {
    0x20910007: "CFG-MSGOUT-NAV-PVT",
    0x2091001B: "CFG-MSGOUT-NAV-STATUS",
    0x20910039: "CFG-MSGOUT-NAV-DOP",
    0x20910065: "CFG-MSGOUT-NAV-SAT",
    0x20910025: "CFG-MSGOUT-NAV-POSLLH",
    0x2091002A: "CFG-MSGOUT-NAV-VELNED",
    0x2091007E: "CFG-MSGOUT-NAV-POSECEF",
    0x20910043: "CFG-MSGOUT-NAV-TIMEGPS",
    0x20910048: "CFG-MSGOUT-NAV-TIMEUTC",
    0x2091005C: "CFG-MSGOUT-NAV-CLOCK",
    0x20910066: "CFG-MSGOUT-NAV-SOL",
    0x209100A5: "CFG-MSGOUT-NAV-AOPSTATUS",
    0x209100A8: "CFG-MSGOUT-RXM-RAWX",
    0x20910035: "CFG-MSGOUT-NAV-SVINFO",
    0x20910038: "CFG-MSGOUT-MON-RF",
    0x209100B0: "CFG-MSGOUT-TIM-TP",
    0x30210001: "CFG-RATE-MEAS",
    0x30210002: "CFG-RATE-NAV",
    0x20210003: "CFG-RATE-TIMEREF",
    0x40520001: "CFG-UART1-BAUDRATE",
    0x10930001: "CFG-SEC-SIGN-SESSTRIG",
    0x10930002: "CFG-SEC-SIGN-SESS-HASHEN",
    0x2091003E: "CFG-MSGOUT-NAV-HPPOSECEF",
    0x209100B5: "CFG-MSGOUT-NAV-COV",
    0x2091006B: "CFG-MSGOUT-NAV-TIMELS",
    0x20910025: "CFG-MSGOUT-NAV-POSLLH",
}

# Collect single-key VALSET packets (payload = 9 bytes: 00 01 00 00 + key4 + val1)
# Full packet: B5 62 06 8A [len_lo len_hi] [payload] [ck_a ck_b]
# payload length is in bytes 4-5 of the packet (little-endian)
valset_keys = {}  # key -> list of (ts, value)
valset_all = []   # (ts, key, value, keylen_guess)

for ts, msg_type, hb in drone_msgs:
    if msg_type != 'CFG-0x8A':
        continue
    # hb = full UBX packet: B5 62 06 8A len_lo len_hi payload ck_a ck_b
    if len(hb) < 8:
        continue
    payload_len = hb[4] | (hb[5] << 8)
    payload = hb[6:6+payload_len]
    # Minimal payload: 4-byte header + 4-byte key + 1-byte val = 9 bytes
    # Multi-key packets have longer payloads
    if len(payload) < 8:
        continue
    # First 4 bytes of payload: version(1) layers(1) reserved(2)
    if payload[0] != 0x00 or payload[1] != 0x01:
        continue  # skip non-standard
    # Extract all key-value pairs (key = 4 bytes, then value size depends on key type)
    # For simplicity, handle single-key (payload = 9 bytes)
    if len(payload) == 9:
        key = int.from_bytes(payload[4:8], 'little')
        val = payload[8]
        valset_all.append((ts, key, val, 1))
        if key not in valset_keys:
            valset_keys[key] = []
        valset_keys[key].append((ts, val))
    elif len(payload) >= 12:
        # Try to parse: 4 header + potentially multiple 5-byte (key4+val1) items
        # For u16 keys: 4+2, for u32: 4+4
        # We'll try heuristic: look at key type bits [29:28] for size
        # bits [29:28]: 00=1byte, 01=1byte, 10=2byte, 11=4byte (actually [29:28] in key)
        # Key size encoding: bits 28-29 of key (big-endian key numbering)
        offset = 4
        while offset + 8 <= len(payload):
            key = int.from_bytes(payload[offset:offset+4], 'little')
            size_bits = (key >> 28) & 0xF
            # Determine value size from key type in bits [31:28]
            # 0x1 = bit/bool (1 byte), 0x2 = byte (1 byte), 0x3 = short (2 bytes), 0x4 = long (4 bytes)
            if size_bits == 0x1 or size_bits == 0x2:
                val_size = 1
            elif size_bits == 0x3:
                val_size = 2
            elif size_bits == 0x4:
                val_size = 4
            else:
                val_size = 1
            if offset + 4 + val_size > len(payload):
                break
            val_bytes = payload[offset+4:offset+4+val_size]
            val = int.from_bytes(val_bytes, 'little')
            valset_all.append((ts, key, val, val_size))
            if key not in valset_keys:
                valset_keys[key] = []
            valset_keys[key].append((ts, val))
            offset += 4 + val_size

print(f"  Unique CFG-VALSET keys found: {len(valset_keys)}")
print(f"  All key-value assignments:")
for key in sorted(valset_keys.keys()):
    name = KNOWN_KEYS.get(key, "???")
    vals = valset_keys[key]
    val_summary = ', '.join(f"{v[1]}" for v in vals)
    print(f"    0x{key:08X}  ({name:35s}): {val_summary}")
print()

# ============================================================
# 8. CFG-RATE measurement rate
# ============================================================
print("=" * 60)
print("8. CFG-RATE MEAS RATE (key 0x30210001)")
print("=" * 60)
meas_rate_key = 0x30210001
if meas_rate_key in valset_keys:
    for ts, val in valset_keys[meas_rate_key]:
        print(f"  At {ts:.3f}s: meas_rate = {val} ms  ({1000/val:.1f} Hz)")
else:
    # Check for multi-byte key (u16)
    print("  Not found as single-byte value, searching raw...")
    # Search in all drone CFG-0x8A packets for key 0x30210001
    found_rate = False
    for ts, msg_type, hb in drone_msgs:
        if msg_type != 'CFG-0x8A':
            continue
        if len(hb) < 8:
            continue
        payload_len = hb[4] | (hb[5] << 8)
        payload = hb[6:6+payload_len]
        # Search for key bytes: 01 00 21 30 (little-endian)
        key_bytes = bytes([0x01, 0x00, 0x21, 0x30])
        idx = payload.find(key_bytes)
        if idx >= 0 and idx + 6 <= len(payload):
            val = int.from_bytes(payload[idx+4:idx+6], 'little')
            print(f"  At {ts:.3f}s: meas_rate = {val} ms  ({1000/val:.1f} Hz if nonzero)")
            found_rate = True
    if not found_rate:
        print("  CFG-RATE-MEAS not found in any VALSET.")
print()

# ============================================================
# 9. NAV-SAT (0x35) satellite count evolution
# ============================================================
print("=" * 60)
print("9. NAV-SAT (NAV-0x35) SATELLITE COUNT EVOLUTION")
print("=" * 60)
nav_sat_entries = [(ts, hb) for ts, t, hb in gnss_msgs if t == 'NAV-0x35']
print(f"  Total NAV-SAT messages: {len(nav_sat_entries)}")
if nav_sat_entries:
    # UBX frame: B5 62 01 35 len_lo len_hi [payload] ck_a ck_b
    # payload offset 5 (0-indexed in payload) = numSvs (after 4-byte iTOW, 1-byte version, ... )
    # NAV-SAT payload: iTOW(4) version(1) numSvs(1) reserved1(1) reserved2(1) ...
    print("  timestamp -> numSvs:")
    prev_num = None
    for ts, hb in nav_sat_entries:
        if len(hb) < 14:
            continue
        payload_len = hb[4] | (hb[5] << 8)
        payload = hb[6:6+payload_len]
        if len(payload) < 6:
            continue
        num_svs = payload[5]
        marker = " <-- CHANGED" if prev_num is not None and num_svs != prev_num else ""
        if prev_num is None or num_svs != prev_num or nav_sat_entries.index((ts, hb)) < 5 or nav_sat_entries.index((ts, hb)) >= len(nav_sat_entries) - 3:
            print(f"    {ts:.3f}s: numSvs={num_svs}{marker}")
        prev_num = num_svs
print()

# ============================================================
# 10. Presence of specific NAV messages
# ============================================================
print("=" * 60)
print("10. PRESENCE CHECK: NAV-POSLLH(0x02), NAV-VELNED(0x12), NAV-PVT(0x07),")
print("    NAV-TIMEGPS(0x20), NAV-SOL(0x06)")
print("=" * 60)
check_types = ['NAV-0x02', 'NAV-0x12', 'NAV-0x07', 'NAV-0x20', 'NAV-0x06']
for ct in check_types:
    count = sum(1 for ts, t, hb in gnss_msgs if t == ct)
    print(f"  {ct:15s}: {'PRESENT (' + str(count) + ' messages)' if count > 0 else 'NOT PRESENT'}")
print()

# ============================================================
# 11. MON-RF (MON-0x38) timing
# ============================================================
print("=" * 60)
print("11. MON-RF (MON-0x38) TIMING (ttyUSB1)")
print("=" * 60)
mon_rf_ts = [ts for ts, t, hb in gnss_msgs if t == 'MON-0x38']
print(f"  Count: {len(mon_rf_ts)}")
if mon_rf_ts:
    print(f"  First: {mon_rf_ts[0]:.3f}s")
    print(f"  Last:  {mon_rf_ts[-1]:.3f}s")
    if len(mon_rf_ts) > 1:
        intervals = [mon_rf_ts[i+1] - mon_rf_ts[i] for i in range(len(mon_rf_ts)-1)]
        avg = sum(intervals) / len(intervals)
        print(f"  Avg interval: {avg:.3f}s, min={min(intervals):.3f}s, max={max(intervals):.3f}s")
print()

# ============================================================
# 12. CFG-RST (0x06, 0x04) from drone
# ============================================================
print("=" * 60)
print("12. CFG-RST (CFG-0x04) FROM DRONE (ttyUSB0)")
print("=" * 60)
cfg_rst = [(ts, t, hb) for ts, t, hb in drone_msgs if t == 'CFG-0x04']
if cfg_rst:
    print(f"  Found {len(cfg_rst)} CFG-RST:")
    for ts, t, hb in cfg_rst:
        print(f"    at {ts:.3f}s")
else:
    print("  No CFG-RST found.")
print()

# ============================================================
# 13. MGA messages
# ============================================================
print("=" * 60)
print("13. MGA MESSAGES")
print("=" * 60)
mga_drone = [(ts, t) for ts, t, hb in drone_msgs if t.startswith('MGA-')]
mga_gnss = [(ts, t) for ts, t, hb in gnss_msgs if t.startswith('MGA-')]

mga_drone_counts = defaultdict(int)
for ts, t in mga_drone:
    mga_drone_counts[t] += 1

mga_gnss_counts = defaultdict(int)
for ts, t in mga_gnss:
    mga_gnss_counts[t] += 1

print(f"  MGA from Drone (ttyUSB0):")
for k, v in sorted(mga_drone_counts.items()):
    print(f"    {k}: {v}")

print(f"  MGA from GNSS (ttyUSB1):")
for k, v in sorted(mga_gnss_counts.items()):
    print(f"    {k}: {v}")

mga_80 = sum(1 for ts, t in mga_drone if t == 'MGA-0x80')
mga_60 = sum(1 for ts, t in mga_gnss if t == 'MGA-0x60')
print(f"\n  MGA-0x80 from drone: {mga_80}")
print(f"  MGA-0x60 ACK from GNSS: {mga_60}")
print()

# ============================================================
# BONUS: Session time range
# ============================================================
print("=" * 60)
print("SESSION INFO")
print("=" * 60)
all_ts = [ts for ts, t, hb in gnss_msgs + drone_msgs]
if all_ts:
    print(f"  Start: {min(all_ts):.3f}s")
    print(f"  End:   {max(all_ts):.3f}s")
    print(f"  Duration: {max(all_ts) - min(all_ts):.3f}s")
print()

# ============================================================
# BONUS: SEC-SIGN from GNSS + first NAV detail
# ============================================================
print("=" * 60)
print("BONUS: FIRST 5 GNSS MESSAGES AND FIRST 5 DRONE MESSAGES")
print("=" * 60)
print("  First 5 GNSS:")
for ts, t, hb in gnss_msgs[:5]:
    print(f"    {ts:.3f}s {t}")
print("  First 5 Drone:")
for ts, t, hb in drone_msgs[:5]:
    print(f"    {ts:.3f}s {t}")
