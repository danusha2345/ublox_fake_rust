#!/usr/bin/env python3
"""Поиск ответов на CFGKEY"""

def decode_uart_log(filepath):
    """Декодирует hex байты из CSV файла"""
    messages = []
    current_msg = []
    start_time = None

    with open(filepath, 'r') as f:
        lines = f.readlines()

    for line in lines[1:]:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            try:
                time_ns = float(parts[1])
                hex_byte = parts[2].strip()
                byte_val = int(hex_byte, 16)

                if not current_msg:
                    start_time = time_ns

                if byte_val == 0x0A:  # LF
                    if current_msg:
                        messages.append((start_time, bytes(current_msg)))
                        current_msg = []
                        start_time = None
                else:
                    current_msg.append(byte_val)
            except ValueError:
                continue

    if current_msg:
        messages.append((start_time, bytes(current_msg)))

    return messages

def format_message(msg_bytes):
    """Форматирует сообщение"""
    if msg_bytes and msg_bytes[-1] == 0x0D:
        msg_bytes = msg_bytes[:-1]
    try:
        text = msg_bytes.decode('ascii')
        if text.isprintable() or text.startswith('$'):
            return text
        else:
            return f"[BINARY] {msg_bytes.hex(' ').upper()}"
    except:
        return f"[BINARY] {msg_bytes.hex(' ').upper()}"

def main():
    data_file = '/home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser/mini5_decoder--251227-192129.txt'
    data_messages = decode_uart_log(data_file)

    # Ищем сообщения в диапазоне времени CFGKEY (23-26 секунд)
    print("=" * 80)
    print("Сообщения в период отправки CFGKEY (23-26 секунд)")
    print("=" * 80)

    for time_ns, msg_bytes in data_messages:
        time_ms = time_ns / 1_000_000
        if 23000 <= time_ms <= 26000:
            formatted = format_message(msg_bytes)
            # Показываем только не-бинарные или короткие
            if not formatted.startswith('[BINARY]') or len(formatted) < 100:
                print(f"[{time_ms:12.3f} ms] {formatted}")

if __name__ == '__main__':
    main()
