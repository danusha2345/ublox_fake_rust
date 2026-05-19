# TODO: добавление эмуляции u-blox M8 (Mavic 3 Classic и др.)

Текущий код эмулирует только M10. Для дронов на M8 (Mavic 3 Classic и
прочие) нужна отдельная сборка с правильным паспортом чипа и (вероятно)
без SEC-SIGN.

## Этап 0. Разведка по дампу (блокер для всего остального)

С реального M8 на конкретном дроне снять и распарсить:

| Что | Зачем |
|-----|-------|
| MON-VER (0x0A 0x04) ответ | swVer/hwVer строки + extension lines (PROTVER, MOD, FWVER) — паспорт чипа |
| Последовательность CFG-* команд от дрона при boot | Понять что дрон ждёт от M8: классические CFG-MSG/PRT/RATE/NAV5/GNSS или уже CFG-VALSET ключи |
| Есть ли запрос SEC-UNIQID (0x27 0x03) / CFG-0x41 / SEC-SIGN | Если нет — весь crypto-путь (p192/sha2/hmac) отключаем под cfg, экономим ~30% флеша и сильно упрощаем код |
| Набор включённых NAV-* сообщений и rate | На M8 типично NAV-SOL/NAV-SVINFO вместо/вместе с NAV-PVT/NAV-SAT — подтвердить |
| baudrate, CFG-PRT параметры | Может отличаться от 921600 |

Без дампа дальше не идём — будем гадать.

**Open question:** Mavic 3 Classic точно M8? Подтвердится только из MON-VER `hwVer == "00080000"`.

## Этап 1. Стратегия: compile-time feature (не runtime)

Причины:
- SEC-SIGN на M8 почти наверняка не нужен. Compile-time убирает p192/sha2/hmac/elliptic-curve целиком — это и память, и упрощение Core1, и нулевой риск что чужой SEC-SIGN таск выстрелит при работе с M8.
- Кнопочный UI уже плотный (1–4 → modes, 5 → model select, 6 → NoRecovery). Втиснуть туда ещё «версия чипа» получится только длинными жестами/комбинациями.
- Под M8 и M10 целевые ситуации разные (старый дрон vs новый), переключать в полёте смысла нет.
- На выходе будет 4 UF2 вместо 2 — приемлемо.

Runtime-вариант оставляем как этап 6, если реально понадобится один универсальный бинарь.

```toml
# Cargo.toml
[features]
default = ["rp2350", "ubx-m10"]
ubx-m10 = []
ubx-m8  = []
rp2350  = ["embassy-rp/rp235xa", "embassy-rp/imagedef-secure-exe"]
rp2354  = ["rp2350"]
```

В `build.rs` — guard: запретить одновременно `ubx-m8`+`ubx-m10` и потребовать ровно одну версию.

## Этап 2. Изолировать version-specific данные

Новый модуль `src/ubx/version.rs` с двумя плоскими наборами констант под `cfg(feature = …)`:

```
PROTVER           "34.10"    vs  "18.00"
HW_VER_STR        "000A0000" vs  "00080000"
MON_VER_SW        "EXT CORE …"
MON_VER_EXT[]     extension lines (MOD=, FWVER=, …)
DEFAULT_NAV_FLAGS какие NAV-* включены при старте
DEFAULT_MON_FLAGS
SUPPORTS_SEC_SIGN bool (compile-time)
SUPPORTS_VALGET   bool (M8 → NAK на VALSET/VALGET)
```

Текущие M10-строки/набор переезжают сюда. Удалить мёртвый `UbloxVersion` enum из `config.rs` — он не нужен.

## Этап 3. Ветвление кода

| Точка | Что меняется |
|-------|--------------|
| `MonVer::default()` (`ubx/messages.rs`) | Берёт строки из `version::*` |
| `handle_ubx_command` для `CFG-VALSET/VALGET/0x41` | На M8 → NAK. На M10 — как сейчас |
| `sec_sign_compute_task`, `sec_sign_timer_task`, `sec_sign::*` | Целиком под `#[cfg(feature = "ubx-m10")]`. На M8 не спавнится. SEC-SIGN-фильтрация в `gnss_processing_task` тоже отключается |
| `MessageFlags::new_default()` | Под cfg — M8 включает `nav_sol`/`nav_svinfo` обязательно |
| `nav_message_task` композиция | По наборам из `version::DEFAULT_NAV_FLAGS` |
| `key_extract::*`, `flash_storage::load_key/save_key` | Под `cfg(m10)`. На M8 — ни key-extract, ни авто-extract на boot |
| `passthrough.rs`, `spoof_detector.rs`, offset-pipeline | **Не трогаем** — версия чипа не влияет на координаты |
| `apply_mode_by_clicks` / `apply_model_by_clicks` | На M8 model select можно скрыть (модели нужны только для SEC-SIGN ключа) или оставить пустым |

## Этап 4. Сборка

Makefile получит 4 цели:

```
rp2350-m10, rp2350-m8, rp2354-m10, rp2354-m8
```

+ объединённый `make all` собирает все четыре. Выходные UF2 — с суффиксом версии:

```
ublox_fake_rp2350_m10.uf2
ublox_fake_rp2350_m8.uf2
ublox_fake_rp2354_m10.uf2
ublox_fake_rp2354_m8.uf2
```

`build.rs` пишет фичу в `version.rs` → попадает в флеш-секцию VERS, чтобы `probe-rs read 0x103FD000 37` всегда показывал какая прошивка. Полезно при switching между билдами.

## Этап 5. Тесты

`tests_host/Cargo.toml` получает те же features (`ubx-m10`/`ubx-m8`). Новые тесты:

- `test_monver_m8_protver` — что MON-VER содержит `PROTVER=18.00` под `cfg(ubx-m8)`.
- `test_monver_m10_protver` — что под `cfg(ubx-m10)` остаётся `PROTVER=34.10` (regression).
- `test_cfg_valset_nak_on_m8` — `handle_ubx_command` на M8 шлёт NAK.
- `test_cfg_msg_ack_on_m8` — классические CFG-MSG/PRT/RATE возвращают ACK.
- `test_default_nav_flags_m8` — `nav_sol`/`nav_svinfo` включены по умолчанию.

CI/локально гонять `cargo test --no-default-features --features ubx-m8` и `--features ubx-m10` отдельно. Обе сборки прошивки тоже под обе фичи.

## Этап 6 (опционально, позже). Runtime switch

Только если появится требование «один UF2 на всё». Тогда:

- enum в флеш (VERSION_FLAG рядом с MODE / MODEL)
- SEC-SIGN таск спавнится всегда, но `is_active()` проверяет флаг
- UI: длительное удержание (3+ сек) переключает M8↔M10, индикация двумя-тремя миганиями. Или «6-й уровень» в model select (после Air3/Mavic4Pro/Air3S/Mavic3Pro добавить «M8-generic»).

Не делаем без явной нужды — runtime удваивает footprint и усложняет логику.

## Что точно НЕ ломаем

- Дефолтный билд `make rp2350` — это **M10**, как сейчас.
- Все 136 host-тестов проходят без правок при `--features ubx-m10`.
- Spoof-detection, passthrough offset, latched-режим — версия чипа на них не влияет, общий код.
- Auto-extract ключа M10 при boot — тоже без изменений на M10-билде.

## Открытые вопросы (закрывать на этапе 0)

1. Mavic 3 Classic точно M8? Подтверждение из MON-VER hwVer строки.
2. Запрашивает ли он SEC-*? Если да — это гибрид (старый чип с новой аутентификацией), SEC-SIGN под cfg не убрать.
3. Какой NAV-набор ждёт дрон по умолчанию? Нужно ли NAV-SOL обязательно, или PVT хватает.
4. CFG-VALSET шлёт? Если нет — на M8 этот класс можно вообще не парсить (упрощает `parser.rs`).
5. Baudrate в M8-дампе? Возможно `38400` или `115200`, а не наш дефолт `921600` — тогда `DEFAULT_BAUDRATE` тоже под cfg.
