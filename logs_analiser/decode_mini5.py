#!/usr/bin/env python3
"""Декодер сырых UART данных DJI Mini 5"""

import sys

def decode_uart_log(filepath):
    """Декодирует hex байты из CSV файла в читаемые сообщения"""
    messages = []
    current_msg = []

    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Пропускаем заголовок
    for line in lines[1:]:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            try:
                hex_byte = parts[2].strip()
                byte_val = int(hex_byte, 16)

                # Собираем байты до \r\n или \n
                if byte_val == 0x0A:  # LF
                    if current_msg:
                        # Декодируем сообщение
                        msg_bytes = bytes(current_msg)
                        try:
                            msg_str = msg_bytes.decode('ascii', errors='replace')
                            msg_str = msg_str.rstrip('\r\n')
                            if msg_str:
                                # Получаем время первого байта сообщения
                                messages.append(msg_str)
                        except:
                            # Если не ASCII - показываем hex
                            hex_str = ' '.join(f'{b:02X}' for b in current_msg)
                            messages.append(f"[HEX] {hex_str}")
                        current_msg = []
                else:
                    current_msg.append(byte_val)
            except ValueError:
                continue

    # Остаток
    if current_msg:
        msg_bytes = bytes(current_msg)
        try:
            msg_str = msg_bytes.decode('ascii', errors='replace')
            messages.append(msg_str)
        except:
            hex_str = ' '.join(f'{b:02X}' for b in current_msg)
            messages.append(f"[HEX] {hex_str}")

    return messages

def categorize_message(msg):
    """Категоризирует сообщение"""
    if msg.startswith('$CFG'):
        return 'CONFIG'
    elif msg.startswith('$PD'):
        return 'PROPRIETARY'
    elif msg.startswith('$G') or msg.startswith('$B') or msg.startswith('$Q'):
        return 'NMEA'
    elif msg.startswith('[HEX]') or not msg.startswith('$'):
        return 'BINARY/OTHER'
    else:
        return 'UNKNOWN'

def main():
    # Файл настроек от DJI (TX к GNSS модулю)
    set_file = '/home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser/mini5_set_decoder--251227-192129.txt'

    # Файл данных от GNSS модуля (RX от GNSS модуля)
    data_file = '/home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser/mini5_decoder--251227-192129.txt'

    print("=" * 70)
    print("СООБЩЕНИЯ НАСТРОЙКИ (DJI -> GNSS модуль)")
    print("=" * 70)

    set_messages = decode_uart_log(set_file)

    config_msgs = []
    for msg in set_messages:
        cat = categorize_message(msg)
        config_msgs.append((cat, msg))
        print(f"[{cat:12}] {msg}")

    print("\n")
    print("=" * 70)
    print("СООБЩЕНИЯ ОТ GNSS МОДУЛЯ (GNSS -> DJI)")
    print("=" * 70)

    data_messages = decode_uart_log(data_file)

    # Группируем по типам
    nmea_msgs = []
    other_msgs = []

    for msg in data_messages:
        cat = categorize_message(msg)
        if cat == 'NMEA':
            nmea_msgs.append(msg)
        else:
            other_msgs.append((cat, msg))

    print("\n--- Другие сообщения ---")
    for cat, msg in other_msgs[:100]:  # Ограничиваем вывод
        print(f"[{cat:12}] {msg}")

    print(f"\n--- NMEA сообщения (всего {len(nmea_msgs)}) ---")
    # Показываем уникальные типы NMEA
    nmea_types = {}
    for msg in nmea_msgs:
        if msg.startswith('$'):
            parts = msg.split(',')
            nmea_type = parts[0]
            if nmea_type not in nmea_types:
                nmea_types[nmea_type] = 0
            nmea_types[nmea_type] += 1

    print("\nТипы NMEA сообщений:")
    for nmea_type, count in sorted(nmea_types.items()):
        print(f"  {nmea_type}: {count} сообщений")

    # Сохраняем в файл
    output_file = '/home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser/mini5_decoded_messages.txt'
    with open(output_file, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write("СООБЩЕНИЯ НАСТРОЙКИ (DJI Mini 5 -> GNSS модуль)\n")
        f.write("=" * 70 + "\n\n")

        for cat, msg in config_msgs:
            f.write(f"[{cat:12}] {msg}\n")

        f.write("\n\n")
        f.write("=" * 70 + "\n")
        f.write("СООБЩЕНИЯ ОТ GNSS МОДУЛЯ (GNSS модуль -> DJI Mini 5)\n")
        f.write("=" * 70 + "\n\n")

        f.write("--- Информация о модуле и конфигурационные ответы ---\n")
        for cat, msg in other_msgs:
            f.write(f"[{cat:12}] {msg}\n")

        f.write("\n--- Статистика NMEA сообщений ---\n")
        for nmea_type, count in sorted(nmea_types.items()):
            f.write(f"  {nmea_type}: {count} сообщений\n")

        f.write("\n--- Примеры NMEA сообщений (первые 20) ---\n")
        for msg in nmea_msgs[:20]:
            f.write(f"{msg}\n")

    print(f"\n\nРезультат сохранён в: {output_file}")

if __name__ == '__main__':
    main()
