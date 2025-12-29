#!/usr/bin/env python3
"""Точный декодер конфигурационных сообщений DJI Mini 5"""

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
    # Убираем \r в конце
    if msg_bytes and msg_bytes[-1] == 0x0D:
        msg_bytes = msg_bytes[:-1]

    # Проверяем, ASCII ли это
    try:
        text = msg_bytes.decode('ascii')
        if text.startswith('$'):
            return text
        else:
            # Смешанные данные
            return f"[BINARY] {msg_bytes.hex(' ').upper()}"
    except:
        return f"[BINARY] {msg_bytes.hex(' ').upper()}"

def main():
    set_file = '/home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser/mini5_set_decoder--251227-192129.txt'
    data_file = '/home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser/mini5_decoder--251227-192129.txt'

    print("=" * 80)
    print("КОМАНДЫ НАСТРОЙКИ ОТ DJI Mini 5 -> GNSS модуль (полные сообщения)")
    print("=" * 80)

    set_messages = decode_uart_log(set_file)

    for i, (time_ns, msg_bytes) in enumerate(set_messages, 1):
        time_ms = time_ns / 1_000_000
        formatted = format_message(msg_bytes)
        print(f"{i:2}. [{time_ms:12.3f} ms] {formatted}")

    print("\n")
    print("=" * 80)
    print("ПОИСК ОТВЕТОВ В ДАННЫХ ОТ GNSS МОДУЛЯ")
    print("=" * 80)

    data_messages = decode_uart_log(data_file)

    # Ищем конфигурационные ответы
    config_keywords = ['$OK', '$PDTINFO', '$CFGMSG', '$CFGSYS', '$CFGSAVE', '$CFGKEY', '$GNTXT']

    config_responses = []
    for time_ns, msg_bytes in data_messages:
        formatted = format_message(msg_bytes)
        if any(kw in formatted for kw in config_keywords):
            config_responses.append((time_ns, formatted))

    print(f"\nНайдено {len(config_responses)} конфигурационных ответов:\n")

    for i, (time_ns, msg) in enumerate(config_responses[:50], 1):  # Первые 50
        time_ms = time_ns / 1_000_000
        print(f"{i:2}. [{time_ms:12.3f} ms] {msg}")

    # Создаём итоговый файл
    output_file = '/home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser/mini5_full_config.txt'
    with open(output_file, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("DJI Mini 5 GNSS - ПОЛНЫЕ КОНФИГУРАЦИОННЫЕ СООБЩЕНИЯ\n")
        f.write("=" * 80 + "\n\n")

        f.write("-" * 80 + "\n")
        f.write("КОМАНДЫ ОТ DJI Mini 5 -> GNSS модуль\n")
        f.write("-" * 80 + "\n\n")

        for i, (time_ns, msg_bytes) in enumerate(set_messages, 1):
            time_ms = time_ns / 1_000_000
            formatted = format_message(msg_bytes)
            f.write(f"{i:2}. [{time_ms:12.3f} ms] {formatted}\n")

        f.write("\n\n")
        f.write("-" * 80 + "\n")
        f.write("ОТВЕТЫ ОТ GNSS модуля -> DJI Mini 5 (конфигурационные)\n")
        f.write("-" * 80 + "\n\n")

        for i, (time_ns, msg) in enumerate(config_responses, 1):
            time_ms = time_ns / 1_000_000
            f.write(f"{i:3}. [{time_ms:12.3f} ms] {msg}\n")

    print(f"\n\nСохранено в: {output_file}")

if __name__ == '__main__':
    main()
