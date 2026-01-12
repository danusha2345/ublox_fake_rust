# Инструкция по работе с платой

## Подключение

```
┌─────────────────────────────────────┐
│          RP2350 плата               │
├─────────────────────────────────────┤
│  GPIO0 (TX) ────► К дрону (RX)      │
│  GPIO1 (RX) ◄──── От дрона (TX)     │
│  GPIO5 (RX) ◄──── От GNSS (TX)      │  ← только для Passthrough
│  GND ────────────── GND             │
└─────────────────────────────────────┘
```

**Baudrate**: 921600 бод (8N1)

## Режимы работы

| Режим | LED | Описание |
|-------|-----|----------|
| **Emulation** | Зелёный | Генерация фейковых GNSS данных |
| **Emulation** (>20 сек) | Жёлтый | Спутники стали "невалидными" |
| **Passthrough** | Синий | Прозрачная передача от внешнего GNSS |

LED мигает с частотой 1 Гц (500ms вкл / 500ms выкл).

## Переключение режимов

**Нажать кнопку** — режим переключается циклически:
```
Emulation → Passthrough → Emulation → ...
```

Режим сохраняется во flash и восстанавливается после перезагрузки.

## Поведение в режиме Emulation

```
Включение платы ───────────────► LED зелёный (мигает)
     │
     ▼
Ожидание команды от дрона
     │
     ▼ (первая UBX команда)
     │
Задержка 400-700ms
     │
     ▼
NAV данные начинают передаваться
(3D fix, 18 спутников)
     │
     ▼ (+20 секунд)
     │
Спутники невалидны ────────────► LED жёлтый
(fix_type=0, num_sv=1)
```

## Сброс таймера 20 секунд

Переключить режим кнопкой: **Passthrough → Emulation**

---

## Настраиваемые параметры

### 1. Размер flash памяти

**Файл**: [`src/config.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/config.rs#L8) (строка 8)

```rust
pub const FLASH_SIZE_BYTES: usize = 4 * 1024 * 1024; // 4MB по умолчанию
```

**Изменить для**: плат с 2MB flash → `2 * 1024 * 1024`

**Автоматически пересчитывается**: офсет для записи конфигурации во flash

### 2. GPIO пины

**Файл**: [`src/config.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/config.rs) (модуль `pins`)

| Назначение | По умолчанию | Где менять |
|------------|--------------|------------|
| UART0 TX (к дрону) | GPIO0 | `config.rs` → `UART0_TX` |
| UART0 RX (от дрону) | GPIO1 | `config.rs` → `UART0_RX` |
| UART1 RX (от GNSS) | GPIO5 | `config.rs` → `UART1_RX` |
| WS2812 LED | GPIO16 | `main.rs` строка ~70 → `WS2812LedPin` |
| Кнопка PWR | GPIO10 | `main.rs` строка ~264 |
| Кнопка INPUT | GPIO11 | `main.rs` строка ~265 |

**WS2812 LED пин** — изменяется через type alias:

**Файл**: [`src/main.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/main.rs#L70) (строка ~70)

```rust
type WS2812LedPin = embassy_rp::peripherals::PIN_16; // ← Изменить PIN_XX
```

Пример для GPIO25: `PIN_25`

### 3. Модель дрона (SEC-SIGN ключ)

**Файл**: [`src/main.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/main.rs#L168) (строка 168)

```rust
static DRONE_MODEL: AtomicU8 = AtomicU8::new(1); // 0 = Air 3, 1 = Mavic 4 Pro
```

| Модель | Значение | Период SEC-SIGN | Задержка NAV |
|--------|----------|-----------------|--------------|
| DJI Air 3 | `0` | 4 секунды | 700 мс |
| DJI Mavic 4 Pro (по умолчанию) | `1` | 2 секунды | 400 мс |

### 4. Координаты по умолчанию

**Файл**: [`src/config.rs`](file:///home2/Git_projects/ublox_gnss_emulator/ublox_fake_rust/src/config.rs) (модуль `default_position`)

```rust
pub const LATITUDE: f64 = 25.7860556;   // Flamingo Park, Miami Beach
pub const LONGITUDE: f64 = -80.1380556;
pub const ALTITUDE_M: i32 = 3;
```

**После изменения**: пересобрать проект → `make rp2350`
