# UBX Protocol Commands Reference

Полная документация по обрабатываемым UBX командам в эмуляторе u-blox GNSS M8/M10.

## Структура UBX протокола

Каждое UBX сообщение имеет структуру:
```
| Sync1 | Sync2 | Class | ID | Length (2B LE) | Payload | CK_A | CK_B |
| 0xB5  | 0x62  |  u8   | u8 |     u16        |   ...   |  u8  |  u8  |
```

Контрольная сумма - Fletcher 8-bit по полям: class, id, length, payload.

---

## Входящие команды (от дрона/хоста)

### CFG Class (0x06) - Конфигурация

| ID | Имя | Размер | Описание |
|----|-----|--------|----------|
| 0x00 | **CFG-PRT** | ≥20 | Конфигурация порта. Payload[8:12] = baudrate (u32 LE). Изменяет скорость UART0. |
| 0x01 | **CFG-MSG** | 3 или 8 | Включение/отключение сообщений. Формат: class(1) + id(1) + rate(1 или 6). Rate[1] для UART1. |
| 0x04 | **CFG-RST** | ≥3 | Reset команда. Payload[2] = reset_mode: 0x08=GNSS stop, 0x09=GNSS start. Без ACK. |
| 0x08 | **CFG-RATE** | 6 | Частота навигации. meas_rate(u16) + nav_rate(u16) + time_ref(u16). Period = meas_rate × nav_rate. |
| 0x09 | **CFG-CFG** | var | Сохранение/загрузка конфигурации. Просто ACK. |
| 0x16 | **CFG-SBAS** | var | Конфигурация SBAS. Просто ACK. |
| 0x23 | **CFG-NAVX5** | var | Расширенные настройки навигации. Просто ACK. |
| 0x24 | **CFG-NAV5** | var | Настройки навигационного движка. Просто ACK. |
| 0x39 | **CFG-ITFM** | var | Мониторинг помех/джамминга. Просто ACK. |
| 0x3E | **CFG-GNSS** | var | Конфигурация GNSS систем. Просто ACK. |
| 0x41 | **CFG-0x41** | 0 (poll) | **DJI проприетарная**: возвращает 256 байт с приватным ключом SEC-SIGN на offset 175. |
| 0x86 | **CFG-PMS** | var | Управление питанием. Просто ACK. |
| 0x8A | **CFG-VALSET** | ≥4 | M10: установка значений. Header(4) + key-value пары. Размер value определяется по bits 28-30 key. |
| 0x8B | **CFG-VALGET** | ≥8 | M10: запрос значений. Возвращает CFG-VALGET response с запрошенными ключами. |

#### CFG-VALSET/VALGET Keys (M10)

| Key ID | Имя | Тип | Описание |
|--------|-----|-----|----------|
| 0x30210001 | CFG-RATE-MEAS | U2 | Период измерений (ms) |
| 0x30210002 | CFG-RATE-NAV | U2 | Циклов на решение |
| 0x20210003 | CFG-RATE-TIMEREF | U1 | Источник времени |
| 0x40520001 | CFG-UART1-BAUDRATE | U4 | Скорость UART1 |
| 0x20910007 | NAV-PVT enable | U1 | Включить NAV-PVT |
| 0x2091001B | NAV-STATUS enable | U1 | Включить NAV-STATUS |
| 0x40A40001 | CFG-HW-RF_LNA_MODE | U4 | Частота кристалла (192 MHz) |
| 0x10C70001 | CFG-RINV-DUMP | L | Дамп данных при старте |
| 0x20C70003 | CFG-RINV-SIZE | U1 | Размер RINV данных (30) |
| 0x50C70004-7 | CFG-RINV-DATA0-3 | U8 | Remote Inventory данные |

---

### MON Class (0x0A) - Мониторинг

| ID | Имя | Размер | Описание |
|----|-----|--------|----------|
| 0x04 | **MON-VER** (poll) | 0 | Запрос версии. Возвращает 160 байт: SW версия + HW версия + Extensions. |

---

### SEC Class (0x27) - Безопасность

| ID | Имя | Размер | Описание |
|----|-----|--------|----------|
| 0x03 | **SEC-UNIQID** (poll) | 0 | Запрос уникального ID чипа. Возвращает 10 байт с 5-байтовым chip ID. |

---

### MGA Class (0x13) - AssistNow

| ID | Имя | Размер | Описание |
|----|-----|--------|----------|
| 0x00 | **MGA-GPS** | var | GPS эфемериды |
| 0x02 | **MGA-GAL** | var | Galileo эфемериды |
| 0x03 | **MGA-BDS** | var | BeiDou эфемериды |
| 0x06 | **MGA-GLO** | var | GLONASS эфемериды |
| 0x20 | **MGA-ANO** | var | AssistNow Offline |
| 0x40 | **MGA-INI** | var | Начальные данные |
| 0x80 | **MGA-DBD** | var | База данных навигации |

Все MGA-* сообщения подтверждаются ACK-ACK.

---

## Исходящие сообщения (к дрону/хосту)

### NAV Class (0x01) - Навигация (5 Hz)

| ID | Имя | Payload | Описание |
|----|-----|---------|----------|
| 0x01 | **NAV-POSECEF** | 20 | Позиция в ECEF: iTOW(4) + ecefX/Y/Z(4×3) + pAcc(4) cm |
| 0x02 | **NAV-POSLLH** | 28 | Позиция в LLH: iTOW + lon/lat(1e-7°) + height/hMSL(mm) + hAcc/vAcc |
| 0x03 | **NAV-STATUS** | 16 | Статус: iTOW + gpsFix + flags + fixStat + flags2 + ttff + msss |
| 0x04 | **NAV-DOP** | 18 | DOP значения: iTOW + gDOP/pDOP/tDOP/vDOP/hDOP/nDOP/eDOP (×0.01) |
| 0x06 | **NAV-SOL** | 52 | Legacy M8: полное решение с ECEF позицией и скоростью |
| 0x07 | **NAV-PVT** | 92 | Position/Velocity/Time: полные данные навигации |
| 0x11 | **NAV-VELECEF** | 20 | Скорость в ECEF: iTOW + ecefVX/VY/VZ(cm/s) + sAcc |
| 0x12 | **NAV-VELNED** | 36 | Скорость в NED: iTOW + velN/E/D + speed + gSpeed + heading + sAcc + cAcc |
| 0x13 | **NAV-HPPOSECEF** | 28 | Высокоточная позиция ECEF с 0.1mm дополнением |
| 0x20 | **NAV-TIMEGPS** | 16 | GPS время: iTOW + fTOW + week + leapS + valid + tAcc |
| 0x21 | **NAV-TIMEUTC** | 20 | UTC время: iTOW + tAcc + nano + year/month/day/hour/min/sec + valid |
| 0x22 | **NAV-CLOCK** | 20 | Часы: iTOW + clkB(ns) + clkD(ns/s) + tAcc + fAcc |
| 0x26 | **NAV-TIMELS** | 24 | Leap second информация |
| 0x30 | **NAV-SVINFO** | 8+12n | Legacy: информация о спутниках (18 SVs) |
| 0x35 | **NAV-SAT** | 8+12n | M10: информация о спутниках (18 SVs: 9 GPS + 3 SBAS + 6 Galileo) |
| 0x36 | **NAV-COV** | 64 | Ковариационные матрицы позиции и скорости |
| 0x60 | **NAV-AOPSTATUS** | 16 | Статус AssistNow Autonomous |
| 0x61 | **NAV-EOE** | 4 | End of Epoch маркер |

#### Состояние "Invalid" (после 20 секунд)

После 20 секунд от старта NAV выхода спутники становятся невалидными:

| Сообщение | Изменения |
|-----------|-----------|
| NAV-PVT | fix_type=0, flags=0, num_sv=1 |
| NAV-STATUS | gps_fix=0, flags=0 |
| NAV-SOL | gps_fix=0, num_sv=1 |
| NAV-SAT | 1 спутник, cno=8 dBHz, не используется |
| NAV-SVINFO | 1 спутник, низкое качество |

---

### MON Class (0x0A) - Мониторинг (1 Hz)

| ID | Имя | Payload | Описание |
|----|-----|---------|----------|
| 0x04 | **MON-VER** | 160 | Версия: SW(30) + HW(10) + Extensions(4×30) |
| 0x09 | **MON-HW** | 60 | Hardware статус: пины, шум, AGC, джамминг |
| 0x36 | **MON-COMMS** | 8 | Информация о порте: version + nPorts + txErrors |
| 0x38 | **MON-RF** | 24 | RF информация: шум, AGC, джамминг индикатор |

---

### ACK Class (0x05) - Подтверждения

| ID | Имя | Payload | Описание |
|----|-----|---------|----------|
| 0x00 | **ACK-NAK** | 2 | Отрицательное подтверждение: clsId + msgId |
| 0x01 | **ACK-ACK** | 2 | Подтверждение: clsId + msgId |

---

### SEC Class (0x27) - Безопасность

| ID | Имя | Payload | Описание |
|----|-----|---------|----------|
| 0x03 | **SEC-UNIQID** | 10 | Уникальный ID: version(1) + reserved(3) + uniqueId(5) + reserved(1) |
| 0x04 | **SEC-SIGN** | 108 | ECDSA подпись: version + msgCnt + sha256Hash(32) + sessionId(24) + sigR(24) + sigS(24) |

#### SEC-SIGN Криптография

- Алгоритм: ECDSA SECP192R1 (P-192)
- Hash: SHA256 складывается по XOR в 24 байта
- k: Детерминистический по RFC6979 (HMAC-SHA256)
- Период: 4 сек (Air 3), 2 сек (Mavic 4 Pro)
- Первая задержка: 1000ms (Air 3), 650ms (Mavic 4 Pro)

---

### TIM Class (0x0D) - Время

| ID | Имя | Payload | Описание |
|----|-----|---------|----------|
| 0x01 | **TIM-TP** | 16 | Timepulse: towMs + towSubMs + qErr + week + flags + refInfo |

---

### RXM Class (0x02) - Receiver Manager

| ID | Имя | Payload | Описание |
|----|-----|---------|----------|
| 0x15 | **RXM-RAWX** | 16 | Raw measurements (без измерений): rcvTow(f64) + week + leapS + numMeas + recStat + version |

---

## DJI Проприетарные расширения

### CFG-0x41 (0x06, 0x41)

Poll-запрос (payload=0) возвращает 256-байтный ответ со структурой:

| Секция | Offset | Size | Описание |
|--------|--------|------|----------|
| Bitmasks | 0 | 26 | Enable bitmasks сигналов |
| ROM Patch #1 | 26 | 28 | file 0x82, ARM Thumb-2 |
| ROM Patch #2 | 54 | 42 | file 0x83, ARM Thumb-2 |
| CFG-SIGNAL | 96 | ~20 | group 0x31 |
| CFG-RINV | ~116 | ~50 | group 0xC7, Remote Inventory |
| **SEC/KEY** | **175** | **26** | group 0xA6, **Приватный ключ P-192** |
| CFG-UART1 | ~192 | 10 | group 0x52, baudrate |
| CFG-CLOCK | ~202 | 40 | group 0xA4, частоты |
| Padding | ~242 | 14 | 0xFF fill |

**Приватный ключ** (24 байта, big-endian) находится на offset 175.

---

## Тайминги

### Старт NAV выхода

| Модель | Реальный тайминг | Задержка от первой команды |
|--------|------------------|---------------------------|
| DJI Air 3 | 666ms | 700ms |
| DJI Mavic 4 Pro | 399ms | 400ms |

```
Первая UBX команда → +delay → NAV выход → +650ms/1000ms → Первый SEC-SIGN
```

### SEC-SIGN период

| Модель | Период | Первая задержка |
|--------|--------|-----------------|
| DJI Air 3 | 4 сек | 1000ms |
| DJI Mavic 4 Pro | 2 сек | 650ms |

---

## Message Enable Flags

Управляются через CFG-MSG и CFG-VALSET:

```rust
// NAV messages (5Hz)
nav_pvt, nav_posecef, nav_posllh, nav_status, nav_dop, nav_sol,
nav_velned, nav_velecef, nav_timeutc, nav_timegps, nav_timels,
nav_clock, nav_sat, nav_svinfo, nav_cov, nav_hpposecef, nav_aopstatus, nav_eoe

// RXM messages
rxm_rawx

// MON messages (1Hz)
mon_hw, mon_comms, mon_rf

// TIM messages
tim_tp
```

Все сообщения изначально отключены и включаются командами от дрона.

---

## Уникальные ID чипов

| Модель | Unique ID (5 байт) |
|--------|-------------------|
| DJI Air 3 | E0 95 65 0F 2A |
| DJI Mavic 4 Pro | EB B9 91 0F 2B |

---

## Версии прошивки (MON-VER)

```
SW Version: ROM SPG 5.10 (7b202e)
HW Version: 000A0000

Extensions:
  FWVER=SPG 5.10
  PROTVER=34.10
  GPS;GLO;GAL;BDS
  SBAS;QZSS
```
