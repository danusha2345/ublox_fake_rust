#!/usr/bin/env python3
"""
Verify SEC-SIGN signatures from UBX logs using private keys.

Algorithm (from src/sec_sign.rs):
  1. z = fold(SHA256(sha256_hash || session_id))
     - SHA256 of 56-byte concatenation
     - Fold: z = hash[0:24]; z[0:8] ^= hash[24:32]
  2. ECDSA verify(Q, z, r, s) on NIST P-192

Usage:
  pip install ecdsa
  python3 logs_analiser/verify_sec_sign.py
"""

import hashlib
import re
import sys
from pathlib import Path

try:
    from ecdsa import NIST192p, BadSignatureError
    from ecdsa.util import sigdecode_string
    from ecdsa.keys import SigningKey, VerifyingKey
except ImportError:
    print("ERROR: pip install ecdsa")
    sys.exit(1)

# Private keys from src/sec_sign.rs
KEYS = {
    "mavic4pro": bytes([
        0x90, 0x89, 0xa2, 0x18, 0x14, 0xa6, 0x2f, 0xc3,
        0x3a, 0xf5, 0xd6, 0xeb, 0x61, 0x16, 0x1c, 0xe1,
        0x86, 0x36, 0xf5, 0x48, 0xd0, 0x71, 0xd6, 0x9f,
    ]),
    "mavic3pro": bytes([
        0xBD, 0x53, 0xD2, 0x32, 0x19, 0x4D, 0x92, 0xB4,
        0xA0, 0x50, 0x73, 0xAE, 0x64, 0x48, 0x33, 0x4F,
        0x2B, 0x80, 0x27, 0x69, 0x9B, 0x3E, 0x71, 0x81,
    ]),
    "air3": bytes([
        0xea, 0xa5, 0xc0, 0x11, 0x1e, 0x18, 0xdb, 0xd1,
        0x7a, 0xdb, 0x3d, 0xc9, 0x39, 0x4b, 0xfb, 0x45,
        0x1f, 0x9d, 0x5e, 0x83, 0xf9, 0x38, 0x22, 0xc7,
    ]),
    "air3s": bytes([
        0x37, 0xBB, 0xEA, 0xF7, 0x8D, 0xD1, 0xEE, 0x96,
        0x31, 0x36, 0x43, 0xA8, 0x63, 0x62, 0xFE, 0x4A,
        0x20, 0x2A, 0xAB, 0x5D, 0xBA, 0x7E, 0x73, 0x5F,
    ]),
}

# Log file → key name mapping
LOG_CONFIGS = [
    ("лог мавик 4 для проверки ключа.txt", "mavic4pro"),
    ("ublox_mavic3_pro.txt", "mavic3pro"),
]


def compute_z(sha256_hash: bytes, session_id: bytes) -> bytes:
    """Compute z = fold(SHA256(sha256_hash || session_id))"""
    assert len(sha256_hash) == 32
    assert len(session_id) == 24

    to_sign = sha256_hash + session_id  # 56 bytes
    final_hash = hashlib.sha256(to_sign).digest()  # 32 bytes

    # Fold 32 → 24: XOR bytes 0-7 with bytes 24-31
    z = bytearray(final_hash[:24])
    for i in range(8):
        z[i] ^= final_hash[24 + i]
    return bytes(z)


def derive_public_key(private_key: bytes) -> VerifyingKey:
    """Derive public key Q = d * G from private key bytes."""
    sk = SigningKey.from_string(private_key, curve=NIST192p)
    return sk.get_verifying_key()


def verify_signature(vk: VerifyingKey, z: bytes, r: bytes, s: bytes) -> bool:
    """Verify ECDSA signature (r, s) over digest z."""
    sig = r + s  # 48 bytes
    try:
        return vk.verify_digest(sig, z, sigdecode=sigdecode_string)
    except BadSignatureError:
        return False


def parse_sec_sign_packets(filepath: str) -> list:
    """
    Parse SEC-SIGN packets from log file.
    Supports both formats:
      - "< B5 62 27 04 6C 00 ..."  (mavic 4 log)
      - "< 32.990s B5 62 27 04 6C 00 ... [SEC-0x04]"  (mavic 3 pro log)
    """
    packets = []

    with open(filepath, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if "B5 62 27 04 6C 00" not in line:
                continue

            # Extract hex starting from B5 62 27 04
            idx = line.index("B5 62 27 04 6C 00")
            hex_str = line[idx:]

            # Remove non-hex: brackets, letters in tags like [SEC-0x04], etc.
            hex_str = re.sub(r'\[.*?\]', '', hex_str)  # remove [tags]
            hex_str = re.sub(r'[^0-9A-Fa-f\s]', '', hex_str)
            hex_bytes = bytes.fromhex(hex_str.replace(' ', ''))

            # Need at least 6 (header) + 108 (payload) = 114 bytes
            if len(hex_bytes) < 114:
                print(f"  Line {line_no}: SKIP (truncated, {len(hex_bytes)} bytes)")
                continue

            payload = hex_bytes[6:]  # skip B5 62 27 04 6C 00

            sha256_hash = payload[4:36]
            session_id = payload[36:60]
            r = payload[60:84]
            s = payload[84:108]
            msg_cnt = int.from_bytes(payload[2:4], 'little')

            packets.append({
                'line': line_no,
                'msg_cnt': msg_cnt,
                'sha256_hash': sha256_hash,
                'session_id': session_id,
                'r': r,
                's': s,
            })

    return packets


def verify_log(log_file: Path, key_name: str) -> int:
    """Verify all SEC-SIGN packets in a log file. Returns number of failures."""
    private_key = KEYS[key_name]

    print(f"{'=' * 70}")
    print(f"Log:  {log_file.name}")
    print(f"Key:  {key_name} ({private_key.hex()})")

    vk = derive_public_key(private_key)
    print(f"Pub:  {vk.to_string().hex()}")
    print()

    packets = parse_sec_sign_packets(str(log_file))
    print(f"Found {len(packets)} complete SEC-SIGN packets")
    print("-" * 70)

    passed = 0
    failed = 0

    for pkt in packets:
        z = compute_z(pkt['sha256_hash'], pkt['session_id'])
        ok = verify_signature(vk, z, pkt['r'], pkt['s'])

        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        else:
            failed += 1

        print(f"  Line {pkt['line']:4d} | msg_cnt={pkt['msg_cnt']:4d} | "
              f"sha256={pkt['sha256_hash'][:8].hex()}... | "
              f"r={pkt['r'][:6].hex()}... | {status}")

    print("-" * 70)
    print(f"Results: {passed} PASS, {failed} FAIL out of {passed + failed} packets")

    if failed == 0 and passed > 0:
        print(">>> Key is VALID — all signatures verified.")
    elif failed > 0:
        print(f">>> WARNING: {failed} signature(s) FAILED!")
    else:
        print(">>> No complete packets found.")
    print()

    return failed


def main():
    base = Path(__file__).parent
    total_failures = 0

    for log_name, key_name in LOG_CONFIGS:
        log_file = base / log_name
        if not log_file.exists():
            print(f"SKIP: {log_name} (not found)")
            continue
        total_failures += verify_log(log_file, key_name)

    return 0 if total_failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
