
import sys
import csv
import struct

def parse_ubx_packets(filename):
    packets = {}
    byte_stream = bytearray()
    
    print(f"Reading {filename}...")
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            next(reader) # Skip header
            for row in reader:
                if len(row) >= 3:
                    try:
                        byte_val = int(row[2], 16)
                        byte_stream.append(byte_val)
                    except ValueError:
                        pass
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return {}

    print(f"Total bytes in {filename}: {len(byte_stream)}")
    
    i = 0
    total_packets = 0
    while i < len(byte_stream) - 6:
        if byte_stream[i] == 0xB5 and byte_stream[i+1] == 0x62:
            cls = byte_stream[i+2]
            id_ = byte_stream[i+3]
            length = byte_stream[i+4] | (byte_stream[i+5] << 8)
            
            key = (cls, id_)
            packets[key] = packets.get(key, 0) + 1
            total_packets += 1
            
            # Skip payload + checksum (2 bytes)
            i += 6 + length + 2
        else:
            i += 1
            
    print(f"Found {total_packets} UBX packets in {filename}")
    return packets

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 analyze_packets.py <source_log> <output_log>")
        return

    source_packets = parse_ubx_packets(sys.argv[1])
    output_packets = parse_ubx_packets(sys.argv[2])

    print("\nComparison (Source -> Output):")
    print(f"{'Class/ID':<15} | {'Source':<10} | {'Output':<10} | {'Diff':<10} | {'Name'}")
    print("-" * 65)

    all_keys = set(source_packets.keys()) | set(output_packets.keys())
    
    for key in sorted(all_keys):
        cls, id_ = key
        src_count = source_packets.get(key, 0)
        out_count = output_packets.get(key, 0)
        diff = out_count - src_count
        
        name = "Unknown"
        if cls == 0x01: name = "NAV"
        elif cls == 0x0A: name = "MON"
        elif cls == 0x27: name = "SEC"
        elif cls == 0x06: name = "CFG"
        
        if cls == 0x01 and id_ == 0x07: name = "NAV-PVT"
        if cls == 0x01 and id_ == 0x35: name = "NAV-SAT"
        if cls == 0x01 and id_ == 0x14: name = "NAV-HPPOSLLH"
        if cls == 0x27 and id_ == 0x03: name = "SEC-UNIQID"
        if cls == 0x27 and id_ == 0x04: name = "SEC-SIGN"
        
        print(f"0x{cls:02X} 0x{id_:02X}     | {src_count:<10} | {out_count:<10} | {diff:<10} | {name}")

if __name__ == "__main__":
    main()
