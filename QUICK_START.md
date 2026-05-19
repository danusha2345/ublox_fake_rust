# Краткая инструкция по работе с платой

## Подключение

```
┌─────────────────────────────────────┐
│          RP2350/RP2354 плата        │
├─────────────────────────────────────┤
│  GPIO0 (UART0 TX) ─► К дрону RX     │
│  GPIO1 (UART0 RX) ◄─ От дрона TX    │
│  GPIO5 (UART1 RX) ◄─ От GNSS TX     │
│  GPIO13 ───────────► PWR кнопки     │
│  GPIO14 ◄─────────── INPUT кнопки   │
│  GND ──────────────► GND            │
└─────────────────────────────────────┘
```

**Baudrate**: 460800 бод (8N1).

LED:
- RP2350: WS2812B на GPIO25.
- RP2354: простой GPIO LED, GPIO11 анод и GPIO12 катод/земля.

## Режимы и кнопка

Короткие нажатия считаются серией с таймаутом 800 мс после отпускания кнопки.

| Нажатия | Режим | Что делает |
|---------|-------|------------|
| 1 | Emulation | Полностью заменяет GNSS поток своими UBX/NAV и SEC-SIGN |
| 2 | Passthrough | Пропускает внешний GNSS с spoof detection и собственной SEC-SIGN |
| 3 | PassthroughRaw | Полностью прозрачный мост без парсинга, spoof detection и своей SEC-SIGN |
| 4 | PassthroughOffset | Passthrough + фиксированный offset координат |
| 5 | Model select | Вход в выбор модели для таймингов/SEC-UNIQID/CFG-0x41/fallback |
| 6 | PassthroughOffsetNoRecovery | Как PassthroughOffset, но `SPOOF_DETECTED` не очищается по recovery |

Длинное удержание 3+ секунды запускает ручное извлечение SEC-SIGN ключа через reboot-флаг.

## LED-индикация

RP2350:
- Зелёный/жёлтый blink — Emulation; жёлтый после 20 секунд, когда спутники становятся невалидными.
- Синий blink — Passthrough.
- Белый blink — PassthroughRaw.
- Белый blink — PassthroughOffset.
- Янтарный короткий blink — PassthroughOffsetNoRecovery.
- Быстрый красный blink — spoof detected в processed passthrough режимах.

RP2354:
- 1/2/3/4/6 вспышек — режимы 1/2/3/4/6.
- Быстрое непрерывное мигание — spoof detected.

## Emulation и spoofing

В режиме Emulation внешний GNSS поток не используется: прошивка полностью заменяет его своими сообщениями. Поэтому внешний spoofing реального GNSS-модуля не может попасть к дрону. Это защита изоляцией/заменой источника, а не нормальная работа с реальными координатами и не работа `SpoofDetector::analyze()`.

## SEC-SIGN ключ

Для текущего u-blox пути модель дрона не является главным способом выбрать приватный ключ. При загрузке прошивка:

1. Загружает ранее извлечённый ключ из flash, если он есть.
2. Если ключа нет, пытается автоматически обнаружить GNSS на UART1, опросить `CFG-0x41`, извлечь ключ и сохранить его во flash.
3. Только если flash/auto key недоступен, использует hardcoded fallback по `DRONE_MODEL`.

Model select остаётся полезен для таймингов NAV/SEC-SIGN, `SEC-UNIQID`, шаблона `CFG-0x41` и fallback ключей.

| Модель | Нажатия в model select | SEC-SIGN period | Config→NAV delay |
|--------|-------------------------|-----------------|------------------|
| DJI Air 3 | 1 | 2 секунды | 700 мс |
| DJI Mavic 4 Pro | 2 | 2 секунды | 400 мс |
| DJI Air 3S | 3 | 2 секунды | 780 мс |
| DJI Mavic 3 Pro | 4 | 2 секунды | 780 мс |

## Настраиваемые параметры

Основные параметры находятся в `src/config.rs`:

| Параметр | Где |
|----------|-----|
| Flash size | `FLASH_SIZE_BYTES`: 4 МБ для RP2350, 2 МБ для RP2354 |
| Пины | `pins` module |
| Тайминги | `timers` module |
| Координаты Emulation | `default_position` |
| Целевая точка PassthroughOffset | `offset_target` |

После изменения прошивку пересобрать через `make rp2350` или `make rp2354`.
