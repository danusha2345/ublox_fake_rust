# Unicore UC6580I boot dump and `boot-capture` feature

## What `boot_dump.bin` is

`src/unicore/boot_dump.bin` (6692 bytes after refresh on 2026-04-26) — byte-exact replay of the chip's first ~6.7 KB on UART1 between power-on and the start of the steady 5 Hz NMEA stream. The Emulation mode tick loop (`main_unicore.rs::emulation_task`) sends it once via `enqueue_chunked(unicore::boot::BOOT_DUMP)`.

Contents (verified by `src/unicore/boot.rs` tests, 9 assertions):
- Leading `\r\nUC6580I-00 G1B1L1E1 COM1\r\n` header line.
- `PN`, `SN`, `HWVer`, `FWVer`, `Reset Count`, `Init time:` identification.
- `$$$_PVT_INIT_INFO_START_$$$` marker.
- `$RECVCFG,120,` + 8 continuation lines (1 KB of receiver config).
- `$PECFG,22,` + 2 continuation lines.
- `$CHCFG,...`, `$PTINFOPKG,34,` + 2 continuation lines.
- All five UTC tables: `$GPSUTC`, `$BDSUTC`, `$GALUTC`, `$GLOUTC`, `$IRNUTC`.
- All three IONO tables: `$GPSIONO`, `$BDSIONO`, `$GALIONO`.
- 20 ephemeris summaries (`$SVEPH_003 ... $SVEPH_131`).
- First steady-state burst: empty RTCM 1077 + ExtRTCM 4074 subs 0xFF/0xFE/0xF9/0xE9/0xE6.

The dump is cut just before the first `$GNRMC` of the periodic 5 Hz stream.

## History of captures

1. **2026-04-22**: original 2286 B dump captured via standalone `uart_term` USB↔UART logger. Started mid-`$PDTINFO`, missing many fields. Committed in `939ea0f` predecessor work.
2. **2026-04-26 morning**: refreshed to 6692 B by extracting from `logs_analiser/2026-04-26_165533_neo_hex.txt` via `.scratch/extract_boot_dump.py`. Commit `939ea0f feat(unicore): refresh UC6580I boot replay`.
3. **2026-04-26 afternoon**: added on-board capture facility via `boot-capture` cargo feature (commit `056e469 feat(unicore): on-board boot-stream capture via defmt-rtt`).
4. **2026-04-26 afternoon**: extended `boot-capture` to ALSO record drone-side UART0 RX in a separate buffer (commit `375ace1 feat(unicore): boot-capture also records drone-side UART0 RX`).

## `boot-capture` feature workflow — TWO sides

Cargo feature `boot-capture = []` declared in `Cargo.toml`. Opt-in only — costs ~16 KB `.bss` for two static buffers + the dump task.

### Implementation in `src/main_unicore.rs`

- `mod raw_capture`: two `raw_capture::Channel` instances, one per direction:
  - `raw_capture::CH` — chip → board (UART1 RX). Freeze: 4 s timeout / `$GNRMC` sentinel / 8192 buffer full.
  - `raw_capture::DR` — drone → board (UART0 RX). Freeze: 25 s timeout / `$CFGKEY` sentinel / 8192 buffer full. `FREEZE_AFTER_MS_DR=25_000` chosen because `$CFGKEY` is the last command and arrives ~19.3 s after first `$PDTINFO`.
- Each `Channel` = `BUF: UnsafeCell<[u8; 8192]>` + atomics `WRITE_IDX/DONE/FIRST_MS/LAST_MS/EPOCH`.
- `fn raw_capture_feed(ch, tag, freeze_after_ms, sentinel, bytes)` — common state machine with re-arm on >500 ms silence (handles repeated source power-cycles without resetting the board).
- `fn emit_channel(ch, tag)` — emits hex chunks with marker prefixes:
  - `RAWCAP` for chip side (back-compat with original boot-capture)
  - `RAWCAP_DR` for drone side
- `raw_boot_dump_task`: spawns under feature gate. After 6 s startup delay, emits both channels every 10 s.
- Capture state machine inserted in `uart1_rx_task` (chip side) and `uart0_rx_task` (drone side), both gated by `#[cfg(feature = "boot-capture")]`.

### Soundness of `static mut`/`UnsafeCell` access

Sound because (a) Unicore build is single-core, (b) each buffer has a dedicated single-writer task, (c) reader (`raw_boot_dump_task`) waits on `DONE.load(Acquire)` after writes are made visible by `WRITE_IDX.store(Release)`. No critical section needed.

### Decoder script `tools/decode_rtt_boot_capture.py`

Parses RTT log file (`probe-rs attach … | tee cap.log`). Supports `--side {chip,drone}` to pick the marker prefix (`RAWCAP` vs `RAWCAP_DR`). Reassembles chunks by offset, verifies CRC32 with `zlib.crc32`, picks the latest fully-emitted epoch by default.

**Important format note**: defmt formats `{=[u8]:02x}` as `[d3, 00, 03, ...]` (commas + brackets). The decoder strips both before `bytes.fromhex()`. Earlier versions of the decoder used a regex that only accepted bare-space hex — that produced "chunks=0" failures. Fixed.

### End-to-end refresh procedure

1. `cargo build --release --features unicore,boot-capture --bin ublox_fake_uc`
2. `make flash`
3. Switch board to Passthrough (double-click button), connect real UC6580I to UART1 RX (GPIO5).
4. `probe-rs attach --chip RP235x ./target/.../ublox_fake_uc | tee /tmp/cap.log` — wait ≥30 s for chip side, ≥25 s for drone side post-power-on.
5. Decode each side:
   - `python3 tools/decode_rtt_boot_capture.py --input /tmp/cap.log --out /tmp/chip.bin --side chip`
   - `python3 tools/decode_rtt_boot_capture.py --input /tmp/cap.log --out /tmp/drone.bin --side drone`
6. Optional: `--diff src/unicore/boot_dump.bin` to spot-check.
7. `cp /tmp/chip.bin src/unicore/boot_dump.bin && (cd tests_host && cargo test boot_dump)` — must pass all 9 structural assertions.
8. Production rebuild **without** the feature: `make rp2350-unicore` / `make rp2354-unicore`.

## Verified results (2026-04-26)

- Chip side (UART1 RX): captured **6841 bytes**, CRC `eb0da459`. Bytes `[0..6669)` are byte-identical to the existing `boot_dump.bin`.
- Drone side (UART0 RX): captured **118 bytes**, CRC `56e8d879`. Contains UART glitch prefix `86 98 86 98 98 FE 98 E6 80 98 80` (11 bytes) followed by `$PDTINFO`/`$CFGMSG×4`/`$CFGSAVE`/`$CFGSYS×2`/`$CFGKEY` exactly matching the DSLogic capture. Sentinel `$CFGKEY` triggered freeze at 19.76 seconds.

## $CFGKEY timing (critical for capture window)

Drone sends `$CFGKEY` last in standard handshake — **~19.3 seconds** after first `$PDTINFO`. Chip replies in ~9 ms. Reference from DSLogic capture `logs_analiser/2026-04-26_165030_neo_asc2.txt`:
- `16:51:09:842` first `$PDTINFO` (drone)
- `16:51:29:109` `$CFGKEY` (drone) — Δ = 19.27 s

`FREEZE_AFTER_MS_DR=25_000` ms in `raw_capture::DR` covers this.

## Documentation locations

- README.md — section under "Unicore UC6580I target" → "Обновление boot dump через `boot-capture`" (chip side + drone side + tx-trace).
- docs/UC6580I.md — §4.1 updated for new size, §4.3 capture procedure (both sides), §4.4 tx-trace usage, §5.8 documents `$CFGKEY` timing.
