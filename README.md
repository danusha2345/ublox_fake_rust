# u-blox GNSS M10 Emulator (Rust/Embassy)

Эмулятор GNSS приёмника u-blox серии M10 на микроконтроллере RP2350.
Написан на Rust с использованием асинхронного фреймворка Embassy.

## Назначение

Устройство эмулирует работу GNSS приёмника u-blox и может работать в двух режимах:
- **Emulation** — генерация фиктивных GNSS данных с криптографической подписью SEC-SIGN
- **Passthrough** — прозрачная ретрансляция данных от реального GNSS модуля

Основное применение — тестирование и исследование систем, использующих u-blox GNSS с аутентификацией SEC-SIGN (например, дроны DJI).

## Особенности

- Полная реализация протокола UBX (17 NAV сообщений, 4 MON сообщения, SEC-SIGN)
- Криптографическая подпись ECDSA SECP192R1 (чистый Rust, без C зависимостей)
- Поддержка приватных ключей DJI Air 3 и Mavic 4 Pro
- Двухядерная асинхронная архитектура Embassy
- Hot-switch режимов без перезагрузки
- Сохранение режима во flash память
- Динамическое изменение baudrate и частоты NAV сообщений
- WS2812B LED индикация режима работы

## Аппаратные требования

- **MCU**: RP2350A (Spotpear RP2350-Core-A)
- **Flash**: 2 МБ
- **UART0**: TX=GPIO0, RX=GPIO1 — к дрону/хосту (921600 бод по умолчанию)
- **UART1**: TX=GPIO4 (не используется), RX=GPIO5 — от внешнего GNSS модуля (для passthrough)
- **LED**: WS2812B на GPIO16 (RP2350-Core-A)
- **Кнопка**: GPIO11 (вход), GPIO10 (питание) — переключение режимов

**Настройка пинов**: все GPIO пины можно изменить в файле [`src/config.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/config.rs) (модуль `pins`).

## Сборка и прошивка

### Требования

```bash
# Установка Rust (если не установлен)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Установка целевой платформы
rustup target add thumbv8m.main-none-eabihf  # RP2350

# Установка probe-rs для прошивки через отладчик
cargo install probe-rs-tools

# Установка elf2uf2-rs для конвертации в UF2
cargo install elf2uf2-rs
```

### Сборка (через Makefile)

**ВАЖНО:** Всегда используйте Makefile для сборки UF2!

```bash
# Сборка UF2 для RP2350
make rp2350

# Очистка
make clean
```

#### Почему Makefile, а не ручная конвертация?

`elf2uf2-rs` по умолчанию генерирует UF2 с Family ID для RP2040 (`0xe48bff56`).
Для RP2350 требуется патчинг Family ID на `0xe48bff59`.

Makefile автоматически:
1. Собирает ELF с правильным target
2. Конвертирует ELF → UF2 через `elf2uf2-rs`
3. Патчит Family ID для RP2350 (Python скрипт)

**НЕ используйте** ручную конвертацию через `objcopy` — она теряет информацию об адресах из ELF секций!

### Прошивка

```bash
# Способ 1: UF2 через BOOTSEL (без отладчика)
# 1. Зажать кнопку BOOT и подключить USB
# 2. Скопировать ublox_fake_rp2350.uf2 на появившийся диск RPI-RP2

# Способ 2: Через probe-rs (требуется отладочный адаптер)
make flash
# или
cargo run --release
```

## Архитектура

### Двухядерная модель

```
┌─────────────────────────────────────────────────────────────────┐
│                         RP2350/RP2040                           │
├───────────────────────────────┬─────────────────────────────────┤
│           CORE 0              │            CORE 1               │
│      (Embassy Executor)       │       (Embassy Executor)        │
├───────────────────────────────┼─────────────────────────────────┤
│  uart0_tx_task                │  led_task (500ms)               │
│    └─ TX_CHANNEL → UART0 TX   │    └─ WS2812B управление        │
│    └─ SHA256 накопление       │                                 │
│    └─ SEC_SIGN_RESULT обрабо. │  sec_sign_compute_task          │
│                               │    └─ ECDSA P-192 подпись       │
│  uart0_rx_task                │                                 │
│    └─ UBX парсер              │  mon_message_task (1Hz)         │
│    └─ CFG команды             │    └─ MON-HW, RF, COMMS         │
│                               │                                 │
│  uart1_rx_task                │                                 │
│    └─ Внешний GNSS вход       │                                 │
│                               │                                 │
│  nav_message_task (5Hz)       │                                 │
│    └─ NAV-PVT, STATUS, DOP... │                                 │
│    └─ Timer::at (без дрейфа)  │                                 │
│                               │                                 │
│  sec_sign_timer_task (2-4s)   │                                 │
│    └─ Запрос SEC-SIGN         │                                 │
│                               │                                 │
│  button_task                  │                                 │
│    └─ Переключение режимов    │                                 │
└───────────────────────────────┴─────────────────────────────────┘
```

### Межъядерная синхронизация

| Канал/Сигнал | Направление | Назначение |
|--------------|-------------|------------|
| `TX_CHANNEL` (32 msg) | Tasks → uart0_tx | Очередь UBX сообщений для отправки |
| `GNSS_RX_CHANNEL` | uart1_rx → uart0_tx | Данные от внешнего GNSS (passthrough) |
| `SEC_SIGN_REQUEST` | Core0 → Core1 | Запрос вычисления подписи |
| `SEC_SIGN_RESULT` | Core1 → Core0 | Результат вычисления (r, s) |
| `SEC_SIGN_IN_PROGRESS` | Атомик | Пауза TX во время вычисления |
| `SEC_SIGN_DONE` | Signal | Уведомление о завершении SEC-SIGN |
| `MODE` | Атомик | Текущий режим работы |

### Поток данных SEC-SIGN

```
1. nav_message_task (Core0) / mon_message_task (Core1)
   └─→ TX_CHANNEL (сериализованные UBX)

2. uart0_tx_task (Core0)
   ├─→ UART0 TX (отправка)
   └─→ SecSignAccumulator.accumulate() (SHA256)

3. sec_sign_timer_task (каждые 2-4 сек)
   ├─ SEC_SIGN_IN_PROGRESS = true (пауза TX)
   ├─ Захват SHA256 хэша + msg_count
   └─→ SEC_SIGN_REQUEST (→ Core1)

4. sec_sign_compute_task (Core1)
   ├─ z = fold(SHA256(hash || session_id))
   ├─ k = HMAC-SHA256(private_key, z) mod n
   ├─ R = k * G, r = R.x mod n
   ├─ s = k^(-1) * (z + r * d) mod n
   └─→ SEC_SIGN_RESULT (→ Core0)

5. uart0_tx_task (Core0)
   ├─ Отправка SEC-SIGN (r, s, hash, msg_cnt)
   ├─ SEC_SIGN_IN_PROGRESS = false
   └─ SEC_SIGN_DONE.signal() (разблокирует NAV/MON)
```

## Режимы работы

### Emulation (LED зелёный/жёлтый)

Генерация синтетических GNSS данных:
- Координаты по умолчанию: настраиваемы в `config.rs`
- 18 спутников: 9 GPS + 3 SBAS + 6 Galileo
- 3D fix с DOP=1.0
- SEC-SIGN подпись: Air 3 = каждые 4 сек, Mavic 4 Pro = каждые 2 сек

LED индикация в режиме Emulation:
- **Зелёный**: валидные спутники (первые 20 сек)
- **Жёлтый**: невалидные спутники (после 20 сек)

### Passthrough (LED синий)

Прозрачная ретрансляция:
- Вход: UART1 RX (GPIO5) от реального GNSS
- Выход: UART0 TX (GPIO0) к хосту
- UBX команды от хоста обрабатываются (настройки сохраняются)

### Переключение режимов

- **Кнопка**: нажатие переключает режим (hot-switch без перезагрузки)
- **Индикация**: LED мигает зелёным/жёлтым (Emulation) или синим (Passthrough)
- **Персистентность**: режим сохраняется во flash и восстанавливается после питания

### Таймер невалидных спутников (20 сек)

Через 20 секунд после старта эмуляции спутники становятся невалидными:

| Сообщение | Невалидное состояние |
|-----------|----------------------|
| NAV-PVT | fix_type=0, flags=0, num_sv=1 |
| NAV-STATUS | gps_fix=0, flags=0 |
| NAV-SOL | gps_fix=0, num_sv=1 |
| NAV-SAT | 1 спутник, cno=8 dBHz, не используется |
| NAV-SVINFO | 1 спутник, низкое качество |

Таймер сбрасывается при:
- Команде CFG-RST от дрона
- Переключении режима Passthrough → Emulation

## Поддерживаемые UBX сообщения

### Выходные (генерируемые/ретранслируемые)

| Класс | ID | Название | Размер | Описание |
|-------|-----|----------|--------|----------|
| 0x01 | 0x07 | NAV-PVT | 92 | Position/Velocity/Time (основное) |
| 0x01 | 0x01 | NAV-POSECEF | 20 | Позиция в ECEF |
| 0x01 | 0x02 | NAV-POSLLH | 28 | Позиция в LLH |
| 0x01 | 0x03 | NAV-STATUS | 16 | Статус приёмника |
| 0x01 | 0x04 | NAV-DOP | 18 | Dilution of Precision |
| 0x01 | 0x06 | NAV-SOL | 52 | Navigation Solution (legacy M8) |
| 0x01 | 0x11 | NAV-VELECEF | 20 | Скорость в ECEF |
| 0x01 | 0x12 | NAV-VELNED | 36 | Скорость в NED |
| 0x01 | 0x13 | NAV-HPPOSECEF | 28 | High Precision ECEF |
| 0x01 | 0x20 | NAV-TIMEGPS | 16 | GPS время |
| 0x01 | 0x21 | NAV-TIMEUTC | 20 | UTC время |
| 0x01 | 0x22 | NAV-CLOCK | 20 | Clock solution |
| 0x01 | 0x26 | NAV-TIMELS | 24 | Leap second info |
| 0x01 | 0x30 | NAV-SVINFO | 8+12n | Satellite info (M8) |
| 0x01 | 0x35 | NAV-SAT | 8+12n | Satellite info (M10) |
| 0x01 | 0x36 | NAV-COV | 64 | Covariance matrices |
| 0x01 | 0x60 | NAV-AOPSTATUS | 16 | AssistNow status |
| 0x01 | 0x61 | NAV-EOE | 4 | End of Epoch |
| 0x02 | 0x15 | RXM-RAWX | 16 | Raw measurements |
| 0x05 | 0x00 | ACK-NAK | 2 | Отрицательное подтверждение |
| 0x05 | 0x01 | ACK-ACK | 2 | Подтверждение |
| 0x0A | 0x04 | MON-VER | 160 | Version info (poll response) |
| 0x0A | 0x09 | MON-HW | 60 | Hardware status |
| 0x0A | 0x36 | MON-COMMS | 8 | Communication port info |
| 0x0A | 0x38 | MON-RF | 24 | RF information |
| 0x0D | 0x01 | TIM-TP | 16 | Timepulse |
| 0x27 | 0x03 | SEC-UNIQID | 10 | Unique ID (poll response) |
| 0x27 | 0x04 | SEC-SIGN | 108 | Подпись |

### Входные (обрабатываемые команды)

| Класс | ID | Название | Описание |
|-------|-----|----------|----------|
| 0x06 | 0x00 | CFG-PRT | Настройка порта (baudrate) |
| 0x06 | 0x01 | CFG-MSG | Включение/отключение сообщений |
| 0x06 | 0x04 | CFG-RST | Reset (0x09 запускает выход сообщений) |
| 0x06 | 0x08 | CFG-RATE | Частота NAV сообщений |
| 0x06 | 0x09 | CFG-CFG | Сохранение конфигурации |
| 0x06 | 0x16 | CFG-SBAS | SBAS настройки |
| 0x06 | 0x23 | CFG-NAVX5 | Navigation expert settings |
| 0x06 | 0x24 | CFG-NAV5 | Navigation settings |
| 0x06 | 0x39 | CFG-ITFM | Jamming monitor |
| 0x06 | 0x3E | CFG-GNSS | GNSS system config |
| 0x06 | 0x86 | CFG-PMS | Power management |
| 0x06 | 0x8A | CFG-VALSET | Value set (M10 style) |
| 0x06 | 0x41 | CFG-0x41 | DJI proprietary SEC-SIGN config (см. ниже) |
| 0x0A | 0x04 | MON-VER | Poll версии |
| 0x27 | 0x03 | SEC-UNIQID | Poll unique ID |

## SEC-SIGN криптография

### Алгоритм

1. **Аккумуляция**: все отправленные UBX сообщения (кроме SEC-SIGN) накапливаются в SHA256 хэшере
2. **Хэш**: `z = fold(SHA256(sha256_field || session_id))`
   - `sha256_field` = текущий хэш накопленных сообщений
   - `session_id` = 24 нулевых байта (M10 совместимость)
   - `fold` = XOR байтов 0-7 с байтами 24-31 (32→24 байта)
3. **Подпись ECDSA P-192**:
   - `k` = детерминистический nonce (RFC6979 упрощённый)
   - `R = k * G`
   - `r = R.x mod n`
   - `s = k^(-1) * (z + r * d) mod n`
4. **Выход**: SEC-SIGN сообщение с (hash, session_id, r, s, msg_count)

### Приватные ключи и тайминги

| Модель | Константа | Период SEC-SIGN | Задержка до NAV |
|--------|-----------|-----------------|-----------------|
| DJI Air 3 | `PRIVATE_KEY_AIR3` | 4 секунды | 700 мс |
| DJI Mavic 4 Pro (по умолчанию) | `PRIVATE_KEY_MAVIC4PRO` | 2 секунды | 400 мс |

**Настройка модели дрона**:
- Изменить константу `DRONE_MODEL` в [`src/main.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/main.rs#L168) (строка 168)
- Значения: `0` = Air 3, `1` = Mavic 4 Pro
- По умолчанию: `1` (DJI Mavic 4 Pro)
- Приватные ключи хранятся в [`src/sec_sign.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/sec_sign.rs)

### CFG-0x41 (OTP / DJI Proprietary)

CFG-0x41 — это команда u-blox для **OTP (One-Time Programmable)** конфигурации M10 модулей.
DJI расширил этот формат для хранения SEC-SIGN ключей и ROM патчей.

**Стандартный u-blox OTP формат**:
```
B5 62 06 41 [len] 04 01 A4 [size] [hash:4] 28 EF 12 05 [config_data] [checksum]
```
- `04 01 A4` — OTP header
- `28 EF 12 05` — константа во всех OTP сообщениях
- Источник: https://github.com/cturvey/RandomNinjaChef/tree/main/uBloxM10OTPCodes

**DJI расширение**: Poll запрос (0x06, 0x41) с нулевым payload возвращает 256-байтный ответ.

**Детальная структура ответа (256 байт)**:

| Секция | Смещение | Размер | Описание |
|--------|----------|--------|----------|
| 1. Bitmasks | 0 | 26 | Битовые маски включения сигналов |
| 2. ROM Patch #1 | 26 | 28 | file 0x82, ARM Thumb-2 код |
| 3. ROM Patch #2 | 54 | 42 | file 0x83, ARM Thumb-2 код |
| 4. CFG-SIGNAL | 96 | ~20 | group 0x31, конфиг сигналов |
| 5. CFG-RINV | ~116 | ~50 | group 0xC7, Remote Inventory |
| 6. SEC/KEY | ~166 | 26 | group 0xA6, **Приватный ключ** |
| 7. CFG-UART1 | ~192 | 10 | group 0x52, baudrate |
| 8. CFG-CLOCK | ~202 | 40 | group 0xA4, частоты |
| 9. Padding | ~242 | 14 | 0xFF заполнение |

**Секция 5 - CFG-RINV (Remote Inventory)**:
```
C7 10 01                              ← DUMP = 1
03 00 C7 20 1E                        ← SIZE = 30 bytes
04 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA0 (8 bytes)
05 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA1 (8 bytes)
06 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA2 (8 bytes)
07 00 C7 50 xx xx xx xx xx xx xx xx   ← DATA3 (8 bytes)
```

**Секция 6 - SEC/KEY (Приватный ключ)**:
```
A6 18                                 ← Group tag (0xA6) + length (24)
xx xx xx xx xx xx xx xx xx xx xx xx   ← Private Key P-192
xx xx xx xx xx xx xx xx xx xx xx xx   ← (24 байта, big-endian)
```

**Секция 8 - CFG-CLOCK (частоты)**:
```
A4 20 01
00 A4 40 00 B0 71 0B                  ← item 00 = 192 MHz
03 00 A4 40 00 B0 71 0B               ← item 03 = 192 MHz
05 00 A4 40 00 B0 71 0B               ← item 05 = 192 MHz
0A 00 A4 40 00 D8 B8 05               ← item 0A = 96 MHz
```

**Практическое применение**: DJI использует эту команду для:
- Выгрузки приватных ключей из GNSS модулей на производстве
- Верификации аутентичности модуля
- Получения ключей из других дронов для анализа

**Реализация**: `src/ubx/messages.rs` → `Cfg41`, `cfg41_templates`
- `PRIVATE_KEY_OFFSET = 175` - смещение для вставки ключа в шаблон

## Конфигурация

### Настройка пинов GPIO ([`src/config.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/config.rs))

```rust
// UART0: к дрону/хосту
pub const UART0_TX: u8 = 0;
pub const UART0_RX: u8 = 1;

// UART1: от внешнего GNSS модуля (passthrough source)
pub const UART1_TX: u8 = 4;   // не используется, но резервируется
pub const UART1_RX: u8 = 5;   // вход от внешнего GNSS

// Mode button
pub const MODE_BTN_PWR: u8 = 6;     // Устаревшее, используется GPIO10
pub const MODE_BTN_INPUT: u8 = 7;   // Устаревшее, используется GPIO11

// WS2812B LED (RP2350-Core-A: GPIO16)
pub const WS2812_LED: u8 = 16;
```

> [!NOTE]
> **Реальные пины в main.rs**: Кнопка использует GPIO10 (PWR) и GPIO11 (INPUT) из-за RP2350-E9 workaround

> [!TIP]
> **Изменение WS2812 LED пина**: Пин светодиода настраивается через **type alias** в одном месте!
> 
> Измените в [`src/main.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/main.rs#L70) строку ~70:
> ```rust
> type WS2812LedPin = embassy_rp::peripherals::PIN_16;  // <- Измените PIN_XX
> ```
> Пример для GPIO25: `type WS2812LedPin = embassy_rp::peripherals::PIN_25;`

### Настройка модели дрона ([`src/main.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/main.rs#L168))

```rust
/// Drone model for SEC-SIGN key selection (0 = Air3, 1 = Mavic4Pro)
static DRONE_MODEL: AtomicU8 = AtomicU8::new(1); // Mavic 4 Pro по умолчанию
```

**Изменить модель**: замените `1` на `0` для Air 3

### Настройка размера flash памяти ([`src/config.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/config.rs#L8))

```rust
/// Flash memory size in bytes
/// Изменить для плат с другим размером flash (например, 2MB = 2 * 1024 * 1024)
pub const FLASH_SIZE_BYTES: usize = 4 * 1024 * 1024; // 4MB по умолчанию
```

**Важно**: При изменении размера памяти офсет для записи конфигурации во flash автоматически пересчитывается в [`src/flash_storage.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/flash_storage.rs)
- Для 4MB: офсет = `0x3FE000` (сектор 1022 из 1024)
- Для 2MB: офсет = `0x1FE000` (сектор 510 из 512)

### Константы времени и координат ([`src/config.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/config.rs))


```rust
// UART baudrate
pub const DEFAULT_BAUDRATE: u32 = 921600;

// Тайминги
pub const NAV_MEAS_PERIOD_MS: u32 = 200;  // 5Hz
pub const NAV_RATE: u32 = 1;
pub const MON_PERIOD_MS: u64 = 1000;      // 1Hz
pub const SEC_SIGN_PERIOD_AIR3_MS: u64 = 4000;   // Air 3: каждые 4 сек
pub const SEC_SIGN_PERIOD_MAVIC4_MS: u64 = 2000; // Mavic 4 Pro: каждые 2 сек

// Координаты по умолчанию (автоматически конвертируются в ECEF)
pub const LATITUDE: f64 = 25.7860556;   // Flamingo Park, Miami Beach
pub const LONGITUDE: f64 = -80.1380556;
pub const ALTITUDE_M: i32 = 3;
```

При изменении координат в `config.rs` автоматически обновляются все NAV сообщения:
- NAV-PVT, NAV-POSLLH — lat, lon, height
- NAV-POSECEF, NAV-SOL, NAV-HPPOSECEF — ECEF координаты (конвертация WGS84)

### Динамическая конфигурация (runtime)

Через UBX команды можно изменить:
- **Baudrate**: CFG-PRT или CFG-VALSET (ключ `0x40520001`)
- **NAV частота**: CFG-RATE или CFG-VALSET (ключи `0x30210001`, `0x30210002`)
- **Сообщения**: CFG-MSG или CFG-VALSET (ключи `0x2091xxxx`)

## Структура проекта

```
ublox_fake_rust/
├── Cargo.toml              # Зависимости и настройки сборки
├── build.rs                # Build script (memory.x генерация)
├── memory.x                # Linker script для RP2350
├── .cargo/config.toml      # Cargo настройки (target, runner)
├── CLAUDE.md               # Документация для Claude Code
└── src/
    ├── main.rs             # Точка входа, tasks, межъядерная связь
    ├── config.rs           # Константы: пины, тайминги, координаты
    ├── coordinates.rs      # LLH→ECEF конвертация (WGS84)
    ├── ubx/
    │   ├── mod.rs          # UBX протокол: классы, checksum, traits
    │   ├── messages.rs     # Структуры всех UBX сообщений
    │   └── parser.rs       # State machine парсер входящих команд
    ├── sec_sign.rs         # ECDSA P-192 подпись, SHA256 аккумулятор
    ├── led.rs              # WS2812 LED драйвер (обёртка над embassy-rp)
    ├── passthrough.rs      # PIO UART passthrough (GPIO→GPIO)
    └── flash_storage.rs    # Сохранение режима во flash
```

## Зависимости

| Крейт | Версия | Назначение |
|-------|--------|------------|
| embassy-executor | 0.9 | Асинхронный runtime |
| embassy-rp | 0.9 | HAL для RP2040/RP2350 |
| embassy-sync | 0.7 | Channel, Signal, Mutex |
| embassy-time | 0.5 | Timer, Ticker |
| embassy-futures | 0.1 | select, yield_now |
| embedded-io-async | 0.6 | Async IO traits (**критично: версия 0.6!**) |
| p192 | 0.13 | Эллиптическая кривая P-192 |
| sha2 | 0.10 | SHA256 хэширование |
| hmac | 0.12 | HMAC для RFC6979 k |
| pio | 0.3 | PIO макрос (**критично: версия 0.3!**) |
| heapless | 0.9 | Статические коллекции |
| libm | 0.2 | Математика для LLH→ECEF (sin, cos, sqrt) |
| defmt + defmt-rtt | 1.0+ | Отладочный вывод |

### Критичные версии

| Крейт | Требуется | Причина |
|-------|-----------|---------|
| `embedded-io-async` | 0.6 | v0.7 ломает trait resolution с embassy-rp |
| `pio` | 0.3 | только v0.3 экспортирует макрос `pio_asm!` |

## Использование RAM/Flash

| Метрика | Значение | Примечание |
|---------|----------|------------|
| .text (код) | ~97 КБ | Pure Rust P-192 (vs 56 КБ C/micro-ecc) |
| .bss (RAM) | ~13.5 КБ | Stackless coroutines (vs 133 КБ FreeRTOS) |

## Известные ограничения

1. **Координаты статичны** — настраиваются в `config.rs`, но не меняются во время работы (нет симуляции движения)
2. **Нет GPS week rollover** — week hardcoded (2349)
3. **Passthrough без фильтрации** — все данные ретранслируются как есть
4. **Один LED** — нет отдельной индикации ошибок

## Лицензия

MIT

## Автор

Daniil, 2025

---

Связанный проект: `../ublox_fake_unified/` — оригинальная версия на C/FreeRTOS
