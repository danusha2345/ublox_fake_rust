# CLAUDE.md

Этот файл даёт guidance для Claude Code (claude.ai/code) при работе с кодом в этом репозитории.

## Обзор проекта

Эмулятор u-blox GNSS M10 на Rust для микроконтроллеров RP2350/RP2354. Использует асинхронный фреймворк Embassy вместо FreeRTOS.

**Замечание**: поддерживаются только RP2350/RP2354; старый Cortex-M0+ target удалён, поскольку текущим буферам нужно слишком много RAM.

**Оригинальная C-версия**: `../ublox_fake_unified/` — на FreeRTOS, для reference.

## КРИТИЧНО: зависимости и версии

Эти расхождения версий ломали сборку — НЕ менять без тестирования:

| Crate | Версия | Почему |
|-------|---------|-----|
| `embedded-io-async` | **0.6** | ДОЛЖНО совпадать с embassy-rp internals. v0.7 ломает trait resolution! |
| `pio` | **0.3** | Только v0.3 экспортирует макрос `pio_asm!`. v0.2 имеет только struct API |
| `embassy-rp` | 0.9 | Последняя на декабрь 2024. API изменился с 0.8 |

## Команды сборки

**КРИТИЧНО: всегда используйте Makefile для сборки UF2!** Ручной objcopy теряет address info → broken firmware.

```bash
PATH="/home2/.cargo/bin:$PATH"       # Rust может не быть в PATH

make rp2350                           # Build UF2 for RP2350
make rp2354                           # Build UF2 for RP2354
make flash                            # Flash via probe-rs (RP2350)
make flash-rp2354                     # Flash via probe-rs (RP2354)
make test                             # Run all 154 host-side tests

cargo rb                              # run release binary (alias)
cargo rp2350                          # build ELF only (alias)
cargo rp2354                          # build ELF only (alias)
```

## Хост-сайд тесты

`spoof_detector.rs` — чистая логика (без зависимостей от embassy/cortex-m), может тестироваться на хосте через `tests_host/`.

```bash
cd tests_host && cargo test            # All 154 tests
cd tests_host && cargo test vuln       # Только регрессионные тесты
```

**Структура**: `tests_host/` — standalone-крейт (НЕ в workspace) с `defmt_mock/` (no-op макросы) и `#[path]` к `../../src/spoof_detector.rs`.

**Группы тестов** (154 теста — 104 spoof_detector + 42 passthrough + 8 coordinates):

Spoof detector (104 теста):
| Группа | Тесты | Покрытие |
|--------|-------|----------|
| 0: Утилиты | 8 | GnssTime, calc_distance, FixType |
| 1: Базовый flow | 6 | init, warmup, normal flight |
| 2: Телепортация | 5 | порог >2km, все направления |
| 3: Скорость | 2 | граница 31 vs 29 m/s |
| 4: GNSS time | 6 | forward/backward jumps, warmup suppression |
| 5: Clock drift | 6 | calibration, порог 10s, recovery |
| 6: Recovery | 9 | coord (5+1 samples), time (immediate), warmup |
| 7: Gap handling | 8 | gap >5s, gap+teleport/drift/time, gap-aware time (чистый гэп ≠ спуф) |
| 8: Регрессии | 7 | март 2026 vulns: immediate detect, last_good guard; honest-time coord-спуф остаётся latched (июнь 2026) |
| 9: last_good | 4 | Invariant: заморожен при spoof, переживает recovery |
| 10: Complex | 4 | Циклы spoof/recovery, реалистичная атака |
| 11: Reset | 2 | Чистое состояние после reset |
| 12: Bug fixes | 6 | Time recovery+distance, last_good check, origin drift |
| 13: Leash+altitude | 5 | Постепенный drift, leash freeze, altitude jump |
| 14: Satellite loss | 12 | Циклы потеря+spoof+recovery, gap thresholds, NoFix persistence |
| 15: CNO uniformity | 8 | Устойчивый uniform C/N0 = спуф (per-constellation σ, не маскируется реальным остатком; плоский низкий C/N0 тоже спуф); разброс / мало спутников / transient = норма; счётчик через gap |
| 16: Velocity inject | 6 | Потолок reported>35 м/с, рассинхрон reported−track>15 м/с; consistent fast = норма; счётчик через gap |

Passthrough (42 теста):
| Группа | Тесты | Покрытие |
|--------|-------|----------|
| 1: UBX parser | 7 | Парсинг кадра, checksum, split-кадры, resync на B5 B5 |
| 2: Offset+ECEF | 8 | LLH offset, ECEF замена, round-trip |
| 3: Spoof modify | 5 | Замена координат, velocity zero, status degrade |
| 4: Extract+Buffer | 3 | Извлечение позиции, ring PositionBuffer |
| 5: Buffer fix_type | 5 | Фильтрация no-fix, stale/fresh entry guards, corruption demo |
| 6: Dynamic offset | 8 | Пересчёт offset, spoofed-base баг, full pipeline |
| 7: Offset boundary | 6 | apply_offset: i32 overflow, out-of-range LLH |

**Замечание**: для coord recovery нужно 6 samples (не 5) — возврат из spoofed позиции — сам по себе телепорт, который сбрасывает normal_count.

## Архитектура

### Двухъядерный async-дизайн (Embassy)
- **Core0**: Embassy executor — UART TX/RX, генерация NAV-сообщений, обработка кнопки
- **Core1**: Embassy executor — управление LED (PIO), SEC-SIGN ECDSA
- Межъядерная коммуникация через `Signal` и `Channel` из `embassy-sync`
- Mode state расшарен через `AtomicU8` с `Acquire/Release` ordering
- Capacity TX_CHANNEL: 32 сообщения (buffer на задержки SEC-SIGN computation)

### Core0 Tasks (src/main.rs)
| Task | Rate | Назначение |
|------|------|------------|
| `uart0_tx_task` | async | Шлёт UBX из TX_CHANNEL (Emulation) или GNSS_RX_CHANNEL (Passthrough), накапливает SHA256 для SEC-SIGN |
| `uart0_rx_task` | async | Парсит входящие UBX-команды, обновляет MSG_FLAGS |
| `uart1_rx_task` | async | Быстрое чтение UART1 → RAW_RX_CHANNEL (минимум обработки, против overrun) |
| `gnss_processing_task` | async | Парсит UBX-кадры, spoof detection, форвардит в GNSS_RX_CHANNEL |
| `nav_message_task` | 200ms (5Hz) | Шлёт NAV-* (Timer::at для drift-free timing) |
| `mon_message_task` | 1s | Шлёт MON-HW, MON-RF, MON-COMMS |
| `button_task` | async | Выбор режима по количеству кликов (1-4, 6 кликов; 5 кликов = выбор модели) |

### Core1 Tasks
| Task | Назначение |
|------|------------|
| `led_task` (RP2350) | WS2812 LED blinking (зелёный/жёлтый=emulation, синий=passthrough, пурпурный=raw, белый=offset, янтарный=no-recovery, красный=spoof) |
| `simple_led_task` (RP2354) | GPIO LED blink code (1-4/6 вспышек = режим, быстрое мигание = spoof) |
| `sec_sign_compute_task` | ECDSA signature computation (~59ms на подпись) |
| `sec_sign_timer_task` | Ждёт FIRST_CONFIG_MILLIS или 2s fallback, затем first_delay → немедленный first SEC-SIGN → period ticker (перенесён с Core0, чтобы не голодал UART interrupt) |

### Структура модулей
- `src/ubx/` — протокол UBX (`mod.rs`, `messages.rs`, `parser.rs`)
- `src/led.rs` — драйвер WS2812 (PIO `pio_asm!`)
- `src/sec_sign.rs` — аккумулятор SHA256 + ECDSA для SEC-SIGN
- `src/config.rs` — назначение пинов, тайминги, дефолтная позиция
- `src/coordinates.rs` — конверсия LLH→ECEF (WGS84), кэшируется при старте
- `src/passthrough.rs` — парсер UBX-кадров, position buffer, модификация NAV
- `src/spoof_detector.rs` — алгоритмы детекции GPS-спуфинга
- `src/flash_storage.rs` — flash-персистентность для режима и извлечённых ключей
- `src/key_extract.rs` — runtime извлечение ключа из реального GNSS через CFG-0x41

### Режимы работы
| Режим | ID | LED | Spoof Detection | Описание |
|-------|----|-----|-----------------|----------|
| Emulation | 0 | green→yellow | Нет анализатора | Защищает, заменяя GNSS-источник на сгенерированный NAV + SEC-SIGN; реальных координат нет |
| Passthrough | 1 | blue | Да | Форвардит реальные GNSS-данные |
| PassthroughRaw | 2 | purple | Нет | Чисто прозрачный форвардинг |
| PassthroughOffset | 3 | white | Да | Passthrough + смещение координат |
| PassthroughOffsetNoRecovery | 4 | amber | Да, latched | PassthroughOffset без сброса `SPOOF_DETECTED` по clean/recovery |

Режим персистится во flash. Кнопка: 1-4 кликов → режимы 0-3, 5 кликов → выбор модели, 6 кликов → PassthroughOffsetNoRecovery (байт режима 4). Таймаут: 800ms. Hot-switch.

Emulation не запускает `SpoofDetector::analyze()`, т.к. не потребляет внешний GNSS-поток. Защита от внешнего spoofing достигается полной заменой источника (генерация UBX/NAV + SEC-SIGN); цена — нет реальных координат.

### Режимы PassthroughOffset

`PassthroughOffset` и `PassthroughOffsetNoRecovery` разделяют один offset, spoof detection, NAV-модификацию и SEC-SIGN путь. Динамический offset вычисляется **один раз** при первом 3D GPS-фиксе и не пересчитывается (фиксирован на весь полёт). Target: `offset_target` в `config.rs` (Seney, Michigan).

**LLH offset** (NAV-PVT, NAV-POSLLH): линейный `offset = target - actual`, применяется через сложение. Всегда точен.

**ECEF offset** (NAV-POSECEF, NAV-HPPOSECEF, NAV-SOL): пересчитывается на каждом кадре из offset LLH через `llh_to_ecef_cm()` (~50µs). Это обеспечивает геометрическую консистентность — фиксированный аддитивный ECEF offset расходится, поскольку `llh_to_ecef` нелинейна (разная широта → разное metres/degree).

**Защита при старте**: координатные сообщения (0x07, 0x02, 0x01, 0x13, 0x06) подавляются, пока offset не вычислен (предотвращает утечку реальных координат в первые 0-2 эпохи до 3D fix).

**Важные правила**:
- Spoof detection работает с **оригинальными** координатами. Offset применяется только на output.
- Offset применяется ВСЕГДА (даже во время spoofing), ДО модификации spoof.
- Во время spoofing: LAST_GOOD ECEF также пересчитывается из offset LLH для консистентности.
- SEC-SIGN: всегда генерируем свой в Passthrough-режимах (реальный SEC-SIGN GNSS фильтруется).
- Структура `DynamicOffset` содержит только LLH-поля (lat_1e7, lon_1e7, alt_mm). Без ECEF-полей.
- `cached_offset_llh` кэширует последний offset LLH для пересчёта ECEF в той же эпохе.
- `pos_buffer` хранит только entries с валидным 3D fix (`fix_type >= 3`). No-fix entries (0,0,0 при потере спутников) исключаются, чтобы не порушить LAST_GOOD.
- `dynamic_offset` **никогда не инвалидируется** после spoof recovery — пересчёт из другой позиции сломал бы коорд-маппинг (одна и та же физическая точка дала бы разные выходные координаты до и после spoof).
- `PassthroughOffsetNoRecovery` не запускает 5s recovery timer и не сбрасывает `SPOOF_DETECTED` по clean/recovery; сброс защёлки — только сменой режима или reboot.

### Spoof Detection в Passthrough

**Поток данных**: UART1 RX → `uart1_rx_task` → `RAW_RX_CHANNEL` → `gnss_processing_task` (parse + detect + modify) → `GNSS_RX_CHANNEL` → `uart0_tx_task` → UART0 TX

**Алгоритм** (`SpoofDetector::analyze()`, вызывается на каждом NAV-PVT 5Hz):

```
                            ┌─────────────────┐
                            │  NAV-PVT frame   │
                            │  from GNSS (5Hz) │
                            └────────┬─────────┘
                                     │
                              ┌──────▼──────┐
                              │ has_3d_fix? │
                              └──┬───────┬──┘
                               NO│       │YES
                    ┌────────────▼┐      │
                    │ Initializing│      │
                    │ (skip frame)│      │
                    └─────────────┘      │
                                  ┌──────▼──────┐
                                  │ first sample?│
                                  └──┬───────┬──┘
                                   YES       │NO
                     ┌─────────────▼─┐       │
                     │ Set prev,     │       │
                     │ last_good,    │       │
                     │ origin        │       │
                     │→ Initializing │       │
                     └───────────────┘       │
                                      ┌──────▼──────┐
                                      │dt > 5s gap? │
                                      └──┬───────┬──┘
                                       YES       │NO
                              ┌────────▼────────┐│
                              │ GAP PATH:       ││
                              │ check vs        ││
                              │ last_good >2km? ││
                              │ clock drift?    ││
                              │ GNSS time jump? ││
                              │ CNO uniform?    ││
                              │ vel ceiling?    ││
                              └──┬───────┬──────┘│
                              ANY│    NONE│       │
                             ┌───▼───┐ ┌──▼────┐  │
                             │Spoofed│ │GapRest│  │
                             │(immed)│ │prev=  │  │
                             └───────┘ │curr   │  │
                                       └───────┘  │
               ┌──────────────────────────────────▼──────────────────────┐
               │                    NORMAL PATH                          │
               │  1. calculate_movement(prev→curr): dist, speed         │
               │  2. check_gnss_time(): time_spoof, time_recovery       │
               │  3. check_system_clock_drift(): drift_spoof, drift_rec │
               └────────────────────────┬────────────────────────────────┘
                                        │
                   ┌────────────────────▼────────────────────┐
                   │ TIME/CLOCK RECOVERY? (spoofed=true)     │
                   │ time_recovery OR clock_drift_recovery   │
                   └────┬──────────────────────────┬────────┘
                      YES                          │NO
               ┌───────▼────────┐                  │
               │near last_good &│                  │
               │NO coord_anomaly│                  │
               └──┬──────────┬──┘                  │
                YES        NO│                     │
         ┌──────▼──────┐ ┌──▼───────────┐         │
         │spoofed=false│ │stay spoofed  │         │
         │warmup reset │ │recalibrate   │         │
         │→ Normal     │ │time only     │         │
         └─────────────┘ │→ Spoofed     │         │
                         └──────────────┘         │
                                           ┌──────▼──────────────────────────┐
                                           │ COMPUTE ANOMALY FLAGS:          │
                                           │                                 │
                                           │ coord_anomaly =                 │
                                           │   teleport(prev→curr >2km)      │
                                           │   OR speed(>30 m/s)             │
                                           │   OR alt_jump(>10m/sample)      │
                                           │   [disabled in recovery warmup] │
                                           │                                 │
                                           │ time_anomaly =                  │
                                           │   time_jump_back(>1s)           │
                                           │   OR time_jump_fwd(>5s)         │
                                           │   OR clock_drift(>10s)          │
                                           │   [disabled in startup warmup]  │
                                           │                                 │
                                           │ last_good_anomaly =             │
                                           │   dist(last_good→curr) > 2km   │
                                           │   [disabled in warmups]         │
                                           │                                 │
                                           │ origin_drift =                  │
                                           │   dist(origin→curr) > 10km     │
                                           │   [disabled in startup warmup]  │
                                           │                                 │
                                           │ cno_spoof =                     │
                                           │   uniform C/N0 per-con          │
                                           │   (std<3, mean>20, >=6/con)     │
                                           │   sustained >=10 epochs         │
                                           │                                 │
                                           │ vel_spoof =                     │
                                           │   reported >35 m/s OR           │
                                           │   reported-track >15 m/s        │
                                           │   sustained >=5 epochs          │
                                           └───────────────┬─────────────────┘
                                                           │
                                                    ┌──────▼──────┐
                                                    │ ANY anomaly?│
                                                    └──┬───────┬──┘
                                                     YES       │NO
                                              ┌───────▼──────┐ │
                                              │ anomaly_cnt++│ │
                                              │ if cnt>=1:   │ │
                                              │ spoofed=true │ │
                                              │ → Spoofed    │ │
                                              └──────────────┘ │
                                                        ┌──────▼──────────────────┐
                                                        │ NORMAL SAMPLE:          │
                                                        │ normal_cnt++            │
                                                        │                         │
                                                        │ if spoofed &&           │
                                                        │   normal_cnt>=5 &&      │
                                                        │   dist(last_good)<2km   │
                                                        │   && within_leash:      │
                                                        │   spoofed=false         │
                                                        │   recovery_warmup=0     │
                                                        │                         │
                                                        │ if !spoofed &&          │
                                                        │   dist(last_good)<2km   │
                                                        │   && within_leash:      │
                                                        │   last_good=curr        │
                                                        │                         │
                                                        │ prev=curr               │
                                                        │ → Normal or Spoofed     │
                                                        └─────────────────────────┘
```

**Ключевые state-переменные:**
- `origin` — первый GPS fix, не обновляется (только на `reset()`), anti-gradient-drift якорь + якорь leash
- `last_good` — последняя надёжная позиция, обновляется только когда `!spoofed && dist < 2km && within_leash(origin, 5km)`
- `prev` — предыдущий sample, обновляется всегда (для расчёта velocity)
- `last_good_gnss_time` / `calibrated_at_system_ms` — time references, обновляются только когда `!spoofed`

**Фазы warmup:**
- **Startup warmup** (10 samples, ~2s): time-проверки отключены (нужна калибровка), coord-проверки активны
- **Recovery warmup** (10 samples, ~2s): coord + last_good проверки отключены (GPS re-acquisition jitter), time-проверки активны

**Что происходит после возврата `analyze()`:**

| Результат | `SPOOF_DETECTED` | Модификация NAV | LED |
|-----------|------------------|-----------------|-----|
| `Normal` | `false` (после 5s таймера) | только offset (Mode 4) | blue/white |
| `Spoofed` | `true` (сразу) | coords→LAST_GOOD, vel→0, status degraded, +offset | red blink |
| `Initializing` | без изменений | нет | без изменений |
| `GapReset` | без изменений | нет | без изменений |

**NAV-сообщения, модифицируемые при spoofing** (10 типов):

| Сообщение | Замена/деградация полей |
|-----------|--------------------------|
| NAV-PVT (0x07) | lon, lat, height, hMSL → LAST_GOOD; velN/E/D → 0; fix_type=0, flags=0, num_sv=2 |
| NAV-POSLLH (0x02) | lon, lat, height, hMSL → LAST_GOOD; hAcc/vAcc → 9999999 |
| NAV-POSECEF (0x01) | ecefX/Y/Z → LAST_GOOD_ECEF; pAcc → 9999999 |
| NAV-HPPOSECEF (0x13) | ecefX/Y/Z → LAST_GOOD_ECEF; Hp → 0; invalidEcef; pAcc → 9999999 |
| NAV-SOL (0x06) | ecefX/Y/Z → LAST_GOOD_ECEF; ecefVX/Y/Z → 0; gps_fix=0, num_sv=2 |
| NAV-VELECEF (0x11) | ecefVX/Y/Z → 0; sAcc → 9999999 |
| NAV-VELNED (0x12) | velN/E/D → 0; speed/gSpeed → 0; heading → 0; sAcc/cAcc → 9999999 |
| NAV-STATUS (0x03) | gps_fix=0, flags=0 |
| NAV-SAT (0x35) | num_svs=2 |
| NAV-SVINFO (0x30) | num_ch=2 |

`num_sv=2` — слишком мало спутников для валидного фикса (вместе с fix_type=0), используется как маркер spoof.

**Историческое замечание (январь 2026)**: авто-детект модели через SEC-UNIQID может рейсить, потому что Mavic 4 Pro может прислать SEC-UNIQID poll до CFG-VALSET. Теперь это не контролирует штатный путь private key: `sec_sign::get_private_key()` сначала использует flash-извлечённый ключ. Выбор модели влияет в основном на тайминги NAV/SEC-SIGN, SEC-UNIQID, шаблон CFG-0x41 и hardcoded fallback ключи.

### NAV Output Start Timing

| Модель | Задержка от первой UBX-команды | Первая SEC-SIGN задержка |
|--------|--------------------------------|---------------------------|
| Air 3 | 700ms | 1000ms |
| Air 3S | 780ms | 650ms |
| Mavic 4 Pro | 400ms | 650ms |
| Mavic 3 Pro | 780ms | 650ms |

### SEC-SIGN Timer Start (Passthrough)

В обрабатывающих Passthrough-режимах (`Passthrough`, `PassthroughOffset`, `PassthroughOffsetNoRecovery`) `sec_sign_timer_task` триггерится по `FIRST_CONFIG_MILLIS` (выставляется первой UBX-командой дрона, обычно SEC-UNIQID на ~15ms после connect) с 2s fallback, если команд нет. После триггера ждёт `first_delay`, потом запускает первый SEC-SIGN **немедленно** (без дополнительного period wait). Дальше SEC-SIGN идёт через `Ticker::every(period)` в конце цикла.

Через 20 секунд после старта NAV-вывода спутники становятся невалидными (fix_type=0, num_sv=1). Таймер сбрасывается только при переключении Passthrough → Emulation.

### Реализация Passthrough

- Поток данных: UART1 RX → `uart1_rx_task` → `RAW_RX_CHANNEL` (256B чанки, глубина 64) → `gnss_processing_task` → `GNSS_RX_CHANNEL` (глубина 128) → `uart0_tx_task` → UART0 TX
- `uart1_rx_task` и `gnss_processing_task` разделены против overrun
- UART1 FIFO RX threshold: 1/4 (4 байта) через `rp_pac` — предотвращает hardware overrun на 921600 baud
- Non-UBX данные (NMEA) сливаются через `take_non_ubx_data()`, в hash не форвардятся
- ECDSA ~59ms (opt-level=3 для crypto-крейтов). Во время SEC-SIGN: `uart0_tx_task` перестаёт потреблять GNSS_RX_CHANNEL, пакеты копятся (глубина 128, ~5 pkts во время ECDSA — без риска переполнения).

**КРИТИЧНО**: `uart0_tx_task` должен `accumulate()` ДО `write_all()`, чтобы избежать гонки hash с `sec_sign_timer_task` на Core1. Лок держится кратко (не во время `write_all`), поэтому timer task не блокируется.

### SEC-SIGN TX Pause

Атомик `SEC_SIGN_IN_PROGRESS` координирует TX во время ECDSA:
1. `sec_sign_timer_task`: если флаг застрял → переиспользовать (нет clear/set gap); иначе set flag → захват hash → `try_send()` request к Core1 (non-blocking, дропит при full)
2. `nav_message_task` / `mon_message_task`: цикл yield_now() пока флаг не очистится
3. `uart0_tx_task`: **двуслойный guard** — pre-check флага в начале цикла → если true, ждать ТОЛЬКО `SEC_SIGN_RESULT` (100ms timeout, без потребления канала) → отправить подпись → очистить флаг. **Дополнительно** обе ветки (Emulation и Passthrough) re-check флага после возврата `select3` с GNSS/NAV-пакетом — если флаг стал true во время `select3` await, пакет дропается (`continue`), чтобы не было hash mismatch (пакет отправлен дрону, но не покрыт SEC-SIGN hash).
4. `sec_sign_compute_task` (Core1): `try_send()` результата обратно (non-blocking, дропит при full)

**Цена дропа пакетов**: post-select3 guard дропит ~1 GNSS-пакет за каждый 2s SEC-SIGN цикл. Замерено за 20 минут (68029 пакетов, 597 SEC-SIGN циклов): 59 RXM-RAWX (1.0%) + 6 NAV-PVT (0.1%) + 1 MGA-ACK — итого 0.1% loss. RXM-RAWX (самый большой пакет) чаще попадает в race window; дропы NAV-PVT редки (~1 на 200s). Все остальные NAV-* типы: ноль потерь.

**Non-blocking каналы**: `SEC_SIGN_REQUEST.send()` и `SEC_SIGN_RESULT.send()` оба через `try_send()`, чтобы исключить каскадные deadlock'и. При заполненных каналах request/result дропается и повторяется на следующем тике.

**Stuck flag guard**: если `SEC_SIGN_IN_PROGRESS` всё ещё true на следующем ticker (~2s позже), флаг **не очищается** — оставляется true и проваливается в захват hash сразу же (исключает TOCTOU-окно, где TX task мог бы протащить пакеты). Максимальный SEC-SIGN gap = 2s.

**MGA-ACK**: `send_ack(0x13, id)` только в Emulation. В Passthrough реальный GNSS присылает свой MGA-ACK (MGA-0x60). Генерация дубликатов ACK переполняла TX_CHANNEL и голодала executor.

`SEC_SIGN_ACC_NEEDS_RESET` атомик: ставится сменой режима, потребляется `sec_sign_timer_task`.

**Смена режима**: `apply_mode_by_clicks()` сливает `TX_CHANNEL` после сохранения режима, чтобы не утекали stale-сообщения (например, фрагменты NAV-VELNED из Emulation в Passthrough output).

## Аппаратные пины

### RP2350A — SpotPear RP2350-Core-A (default)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud, к дрону/хосту)
- UART1: RX=GPIO5 (от внешнего GNSS для passthrough)
- WS2812B LED: GPIO25 (PIO0)
- Кнопка режима: GPIO14 (input), GPIO13 (power)

### RP2354A (`--features rp2354`)
- UART0: TX=GPIO0, RX=GPIO1 (921600 baud)
- UART1: RX=GPIO5 (passthrough)
- Simple GPIO LED: GPIO11 (анод), GPIO12 (катод/земля)
- Кнопка режима: GPIO14 (input), GPIO13 (power)

## Криптография SEC-SIGN

**Приватные ключи** хранятся в: `src/sec_sign.rs`

| Drone Model | ID | First Delay | Period |
|-------------|-----|-------------|--------|
| Air 3 | 0 | 1000ms | 2s |
| Mavic 4 Pro | 1 | 650ms | 2s |
| Air 3S | 2 | 650ms | 2s |
| Mavic 3 Pro | 3 | 650ms | 2s |

Источник приватного ключа: сначала flash-извлечённый, hardcoded ключ модели — только fallback. `DRONE_MODEL` для текущей u-blox DJI задачи второстепенен; он по-прежнему выбирает тайминги, SEC-UNIQID, шаблон CFG-0x41 и fallback-ключ. У Mavic 4 Pro могут быть уникальные ключи на единицу — извлечение ключа это авторитетный путь.

**Алгоритм**: SHA256 по всем переданным UBX (кроме самого SEC-SIGN) → fold 32→24 байта → ECDSA SECP192R1 подпись с deterministic k (HMAC-SHA256, упрощённый RFC6979). Ключевые крейты: `p192`, `sha2`, `hmac`.

## CFG-0x41 (OTP / DJI Proprietary)

Poll `(0x06, 0x41)` с нулевым payload → 256-байтный ответ с конфигом SEC-SIGN.

**Ключевые секции payload**:

| Секция | Описание |
|--------|----------|
| Bitmasks (0-25) | Битовые маски включения сигналов |
| ROM Patches (26-95) | files 0x82, 0x83 — ARM Thumb-2 код |
| CFG-SIGNAL (96+) | group 0x31 |
| CFG-RINV (~116) | group 0xC7, Remote Inventory (только Air 3 / Mavic 4 Pro) |
| **SEC/KEY** | group 0xA6, **24-байтный Private Key P-192** |
| CFG-UART1/CLOCK | groups 0x52, 0xA4 |

Offset ключа: 175 (Air 3, Mavic 4 Pro) или 115 (Air 3S, Mavic 3 Pro — без секции CFG-RINV). Fallback: сканирование `A6 18` тега.

## Runtime извлечение ключа

Извлекает 24-байтный приватный ключ SEC-SIGN из реального u-blox GNSS-модуля через CFG-0x41 poll.

### Автоматическое извлечение (на старте)

Если в flash нет ключа, `auto_extract()` запускается автоматически при старте:
1. Детектит GNSS-модуль, ожидая NMEA-данные на UART1 (500ms timeout)
2. Повторяет при framing/break ошибках (типично на старте UART)
3. Шлёт CFG-0x41 poll, читает ответ (2 попытки × 500ms)
4. Извлекает и сохраняет ключ во flash

**Тайминги** (замерено): GNSS detected ~120-280ms, ключ извлечён за ~430-600ms суммарно.
Если GNSS не подключён — 500ms timeout, fallback на hardcoded ключи.
Если ключ уже во flash — полностью пропускается (overhead 0ms).

### Ручное извлечение (long-press)

**Триггер**: длинное нажатие (3+ сек) → flash-флаг → reboot → извлечение при старте.
5 попыток × 2s timeout. LED-фидбэк (зелёный=успех, красный=fail).

### Общее

**HW**: TX через PIO1 на GPIO1, RX через UART1 на GPIO5.
**Хранилище**: Flash Last-1 сектор (magic `0x4B455953` "KEYS"). Перекрывает hardcoded ключи.
**Дебаг**: `FORCE_KEY_EXTRACT` в `main.rs` форсит ручное извлечение на каждом старте.

### Boot log (deferred)

Подробности boot сохраняются в атомиках и выводятся через `diag_stats_task` на первом тике (~10s после старта), поскольку probe-rs не может подключиться до power-on, если плата питается от дрона.

## Diagnostic Mode

Установить `DIAG_MSG_DETAIL = true` в `src/main.rs`, пересобрать. Выводит per-message-type RX/TX счётчики и логирование ключей CFG-VALSET каждые 10s.

## Dynamic Configuration

NAV rate: CFG-RATE (0x06, 0x08) или CFG-VALSET ключи `0x30210001` (meas_rate), `0x30210002` (nav_rate).
Baudrate: CFG-VALSET ключ `0x40520001`. Использует прямой доступ к регистрам UART0 через `rp-pac`.

## Firmware Version Storage

`build.rs` извлекает git hash → `$OUT_DIR/version.rs` → пишется во flash (Last-3 сектор, magic `0x56455253` "VERS").

```bash
probe-rs read --chip RP2350 0x103FD000 37   # RP2350 (4MB flash)
probe-rs read --chip RP2354 0x101FD000 37   # RP2354 (2MB flash)
```

## Реализованные UBX-сообщения

### NAV Class (0x01)
| ID | Name | Payload | Описание |
|----|------|---------|----------|
| 0x01 | NAV-POSECEF | 20 | Position in ECEF |
| 0x02 | NAV-POSLLH | 28 | Position in LLH |
| 0x03 | NAV-STATUS | 16 | Receiver status |
| 0x04 | NAV-DOP | 18 | Dilution of precision |
| 0x06 | NAV-SOL | 52 | Navigation solution (legacy) |
| 0x07 | NAV-PVT | 92 | Position/Velocity/Time |
| 0x11 | NAV-VELECEF | 20 | Velocity in ECEF |
| 0x12 | NAV-VELNED | 36 | Velocity in NED |
| 0x13 | NAV-HPPOSECEF | 28 | High precision ECEF |
| 0x20 | NAV-TIMEGPS | 16 | GPS time solution |
| 0x21 | NAV-TIMEUTC | 20 | UTC time solution |
| 0x22 | NAV-CLOCK | 20 | Clock solution |
| 0x26 | NAV-TIMELS | 24 | Leap second info |
| 0x30 | NAV-SVINFO | 8+12n | Satellite info (legacy) |
| 0x35 | NAV-SAT | 8+12n | Satellite info (M10) |
| 0x36 | NAV-COV | 64 | Covariance matrices |
| 0x60 | NAV-AOPSTATUS | 16 | AssistNow status |
| 0x61 | NAV-EOE | 4 | End of epoch |

### Прочие классы
| Class | ID | Name | Payload | Описание |
|-------|-----|------|---------|----------|
| 0x02 | 0x13 | RXM-SFRBX | 8+n | Subframe buffer (passthrough) |
| 0x02 | 0x15 | RXM-RAWX | 16+ | Raw measurements |
| 0x05 | 0x00/0x01 | ACK-NAK/ACK | 2 | Acknowledgement |
| 0x06 | 0x41 | CFG-0x41 | 256 | DJI proprietary (private key) |
| 0x0A | 0x04/0x09/0x36/0x38 | MON-VER/HW/COMMS/RF | var | Monitoring |
| 0x0D | 0x01 | TIM-TP | 16 | Timepulse |
| 0x13 | * | MGA-* | var | AssistNow data |
| 0x27 | 0x03 | SEC-UNIQID | 10 | Unique ID (poll) |
| 0x27 | 0x04 | SEC-SIGN | 108 | Signature |

## Заметки по Embassy 0.9 API

- У `BufferedUartTx` / `BufferedUartRx` больше нет lifetime-параметров (изменено с 0.8)
- `Signal::try_get()` не существует — для shared mode state использовать `AtomicU8`
- PIO pin wrapping: `Peri<'d, PIN>`, не сырой pin type
- Interrupt binding: требуется макрос `bind_interrupts!`

# GitNexus — Code Intelligence

Этот проект индексирован GitNexus как **ublox_fake_rust** (1668 символов, 3294 связи, 20 execution flows). Используйте MCP-инструменты GitNexus для понимания кода, оценки impact и безопасной навигации.

> Если какой-то инструмент GitNexus сообщает, что индекс устарел — сначала запустите в терминале `npx gitnexus analyze`.

## Что делать всегда

- **ОБЯЗАТЕЛЬНО запускать impact analysis перед правкой любого символа.** Перед изменением функции, класса или метода вызывайте `gitnexus_impact({target: "symbolName", direction: "upstream"})` и сообщайте пользователю blast radius (прямые вызывающие, затронутые процессы, уровень риска).
- **ОБЯЗАТЕЛЬНО запускать `gitnexus_detect_changes()` перед коммитом**, чтобы убедиться, что изменения затронули только ожидаемые символы и execution flows.
- **ОБЯЗАТЕЛЬНО предупреждать пользователя**, если impact analysis возвращает риск HIGH или CRITICAL, до начала правок.
- При исследовании незнакомого кода используйте `gitnexus_query({query: "concept"})` для поиска execution flows вместо grep. Он возвращает результаты, сгруппированные по процессам и отсортированные по релевантности.
- Когда нужен полный контекст по конкретному символу — вызывающие, вызываемые, в каких execution flows он участвует — используйте `gitnexus_context({name: "symbolName"})`.

## Что делать никогда

- НИКОГДА не править функцию, класс или метод, не запустив сначала `gitnexus_impact`.
- НИКОГДА не игнорировать предупреждения HIGH или CRITICAL от impact analysis.
- НИКОГДА не переименовывать символы через find-and-replace — используйте `gitnexus_rename`, который понимает call graph.
- НИКОГДА не коммитить изменения без `gitnexus_detect_changes()` для проверки затронутого scope.

## Ресурсы

| Ресурс | Использование |
|--------|---------------|
| `gitnexus://repo/ublox_fake_rust/context` | Обзор кодовой базы, проверка свежести индекса |
| `gitnexus://repo/ublox_fake_rust/clusters` | Все функциональные области |
| `gitnexus://repo/ublox_fake_rust/processes` | Все execution flows |
| `gitnexus://repo/ublox_fake_rust/process/{name}` | Пошаговая трассировка выполнения |

## CLI

| Задача | Какой skill-файл читать |
|--------|------------------------|
| Понять архитектуру / «Как работает X?» | `.claude/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / «Что сломается, если изменить X?» | `.claude/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Трассировка багов / «Почему X падает?» | `.claude/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / рефакторинг | `.claude/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Инструменты, ресурсы, справка по схеме | `.claude/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Команды CLI: index, status, clean, wiki | `.claude/skills/gitnexus/gitnexus-cli/SKILL.md` |

# Serena — символьные инструменты для кода

Этот проект сконфигурирован для Serena (`.serena/`). MCP-инструменты Serena
дают символьный доступ к коду через language-server. Предпочитать их
полному чтению файлов или grep.

## Когда использовать Serena
- **Обзор файла** — `get_symbols_overview` перед полным чтением файла.
- **Поиск символа** — `find_symbol` по name path вместо grep.
- **Трассировка использования** — `find_referencing_symbols` для поиска вызовов/зависимостей.
- **Точные правки** — `replace_symbol_body`, `insert_after_symbol`,
  `insert_before_symbol` вместо ручных правок по строкам.

Полное тело файла читать только если символьные инструменты не покрывают задачу.

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **ublox_fake_rust** (1712 symbols, 3401 relationships, 20 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> If any GitNexus tool warns the index is stale, run `npx gitnexus analyze` in terminal first.

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `gitnexus_impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `gitnexus_detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `gitnexus_query({query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `gitnexus_context({name: "symbolName"})`.

## Never Do

- NEVER edit a function, class, or method without first running `gitnexus_impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `gitnexus_rename` which understands the call graph.
- NEVER commit changes without running `gitnexus_detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/ublox_fake_rust/context` | Codebase overview, check index freshness |
| `gitnexus://repo/ublox_fake_rust/clusters` | All functional areas |
| `gitnexus://repo/ublox_fake_rust/processes` | All execution flows |
| `gitnexus://repo/ublox_fake_rust/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.claude/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.claude/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.claude/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.claude/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.claude/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.claude/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
