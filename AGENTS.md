<!-- gitnexus:start -->
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

<!-- gitnexus:end -->
