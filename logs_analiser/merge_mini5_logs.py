#!/usr/bin/env python3
"""
Скрипт для объединения логов Mini 5 в формат, аналогичный mavic4_merged_ubx.txt

Mini 5 использует чип UC6580 (Unicore) с NMEA-подобным текстовым протоколом,
а не UBX как у ublox.

Парсит сырые CSV данные с осциллографа и извлекает текстовые сообщения.
"""

import os
import re
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Message:
    """Сообщение"""
    timestamp_ns: float  # Время первого байта в наносекундах
    timestamp_end_ns: float  # Время последнего байта
    text: str  # Текстовое содержимое
    raw_bytes: bytes  # Сырые байты
    source_file: str  # Из какого файла


def parse_csv_to_bytes(filepath: str) -> List[Tuple[float, int]]:
    """
    Парсит CSV файл и возвращает список (timestamp_ns, byte_value)
    """
    result = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('Id,'):  # Пропускаем заголовок
                continue
            
            parts = line.split(',')
            if len(parts) >= 3:
                try:
                    timestamp_ns = float(parts[1])
                    byte_hex = parts[2].strip()
                    byte_val = int(byte_hex, 16)
                    result.append((timestamp_ns, byte_val))
                except (ValueError, IndexError):
                    continue
    
    return result


def extract_messages(byte_stream: List[Tuple[float, int]], source_file: str) -> List[Message]:
    """
    Извлекает сообщения из потока байтов
    
    Сообщения разделяются переводом строки (0x0A) или начинаются с '$'
    """
    messages = []
    current_bytes = []
    start_ts = None
    end_ts = None
    
    for ts, byte_val in byte_stream:
        if start_ts is None:
            start_ts = ts
        end_ts = ts
        current_bytes.append(byte_val)
        
        # Если встретили перевод строки - завершаем сообщение
        if byte_val == 0x0A:
            if current_bytes:
                raw = bytes(current_bytes)
                # Декодируем как ASCII, заменяя непечатные символы
                text = ''
                for b in current_bytes:
                    if b == 0x0D or b == 0x0A:
                        continue  # Пропускаем CR/LF
                    elif 32 <= b <= 126:
                        text += chr(b)
                    else:
                        text += f'[{b:02X}]'
                
                if text.strip():  # Только если есть текст
                    messages.append(Message(
                        timestamp_ns=start_ts,
                        timestamp_end_ns=end_ts,
                        text=text.strip(),
                        raw_bytes=raw,
                        source_file=source_file
                    ))
                
                current_bytes = []
                start_ts = None
    
    # Последнее сообщение без перевода строки
    if current_bytes:
        raw = bytes(current_bytes)
        text = ''
        for b in current_bytes:
            if b == 0x0D or b == 0x0A:
                continue
            elif 32 <= b <= 126:
                text += chr(b)
            else:
                text += f'[{b:02X}]'
        
        if text.strip():
            messages.append(Message(
                timestamp_ns=start_ts,
                timestamp_end_ns=end_ts,
                text=text.strip(),
                raw_bytes=raw,
                source_file=source_file
            ))
    
    return messages


def get_msg_type(text: str) -> str:
    """Определяет тип сообщения по тексту"""
    if text.startswith('$'):
        # NMEA-подобное сообщение
        # Извлекаем имя команды
        match = re.match(r'\$([A-Z0-9_]+)', text)
        if match:
            return match.group(1)
    elif text.startswith('[') and ']' in text:
        return "BINARY"
    elif 'UC6580' in text or 'FWVer' in text or 'HWVer' in text:
        return "INFO"
    elif 'PN ' in text or 'SN ' in text:
        return "ID"
    elif 'Init time' in text or 'Reset Count' in text:
        return "STATUS"
    
    return "TEXT"


def format_bytes_to_hex(data: bytes, bytes_per_line: int = 48) -> List[str]:
    """Форматирует байты в строки HEX"""
    lines = []
    for i in range(0, len(data), bytes_per_line):
        chunk = data[i:i + bytes_per_line]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        lines.append(hex_str)
    return lines


def main():
    # Пути к файлам
    base_dir = "/mnt/danikserver_home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/logs_analiser"
    
    set_file = os.path.join(base_dir, "mini5_set_decoder--251227-192129.txt")
    decoder_file = os.path.join(base_dir, "mini5_decoder--251227-192129.txt")
    output_file = os.path.join(base_dir, "mini5_merged_ubx.txt")
    
    print(f"Парсинг {set_file}...")
    set_bytes = parse_csv_to_bytes(set_file)
    print(f"  Прочитано {len(set_bytes)} байтов")
    
    print(f"Парсинг {decoder_file}...")
    decoder_bytes = parse_csv_to_bytes(decoder_file)
    print(f"  Прочитано {len(decoder_bytes)} байтов")
    
    # Извлекаем сообщения
    print("\nИзвлечение сообщений...")
    set_file_short = os.path.basename(set_file)
    decoder_file_short = os.path.basename(decoder_file)
    
    set_messages = extract_messages(set_bytes, set_file_short)
    print(f"  Из {set_file_short}: {len(set_messages)} сообщений")
    
    decoder_messages = extract_messages(decoder_bytes, decoder_file_short)
    print(f"  Из {decoder_file_short}: {len(decoder_messages)} сообщений")
    
    # Объединяем и сортируем по времени
    all_messages = set_messages + decoder_messages
    all_messages.sort(key=lambda m: m.timestamp_ns)
    
    print(f"\nВсего сообщений: {len(all_messages)}")
    
    # Записываем результат
    print(f"Запись в {output_file}...")
    
    with open(output_file, 'w', encoding='utf-8') as f:
        # Заголовок
        f.write("=" * 100 + "\n")
        f.write("MERGED LOG - Mini 5 (UC6580)\n")
        f.write("Source files:\n")
        f.write(f"  1. {set_file}\n")
        f.write(f"  2. {decoder_file}\n")
        f.write(f"Total messages: {len(all_messages)}\n")
        f.write("=" * 100 + "\n")
        f.write("\n")
        
        # Сообщения
        for idx, msg in enumerate(all_messages, 1):
            time_ms = msg.timestamp_ns / 1_000_000.0
            msg_type = get_msg_type(msg.text)
            
            # Заголовок сообщения
            f.write(f"[{idx:4d}] {time_ms:12.3f} ms | {msg_type:15s} | Len={len(msg.raw_bytes):4d} | {msg.source_file}\n")
            
            # Текстовое содержимое
            f.write(f"       TEXT: {msg.text}\n")
            
            # HEX данные
            hex_lines = format_bytes_to_hex(msg.raw_bytes)
            for hex_line in hex_lines:
                f.write(f"       {hex_line}\n")
            
            f.write("\n")
    
    print("Готово!")


if __name__ == "__main__":
    main()
