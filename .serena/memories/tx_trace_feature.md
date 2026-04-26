# `tx-trace` feature — diagnosing coordinate leaks under spoof

## What it is

Opt-in cargo feature `tx-trace` (declared in `Cargo.toml`) that emits a
single `info!()` line through defmt-rtt for **every** `enqueue()` to
UART0 TX (the drone bus) while `SPOOF_DETECTED == true`. Production OFF —
LTO drops the entire trace path.

## Implementation

`src/main_unicore.rs`:
- `fn tx_trace(tag: &'static str, bytes: &[u8])` — gated helper. Logs
  `[TX→DRONE/<tag>] len=N first=[xx, ...]` (first 48 bytes hex). No-op
  branch under non-feature.
- Call sites at every `enqueue()` in the passthrough TX path:
  - `process_nmea_line` rebuild path (`spoof-rebuild-gga`/`-rmc`)
  - `process_nmea_line` non-GGA/RMC verbatim forward (`nmea-passthrough`)
  - `process_nmea_line` GLL/VTG drop (`nmea-drop-spoof`, `nmea-drop-offset`)
  - `process_nmea_line` offset rewrite (`offset-rewrite`)
  - `process_nmea_line` fallback (`nmea-fallback`, `nmea-verbatim`)
  - `passthrough_forward_task` RTCM frame flush (`rtcm-flush`)
  - `passthrough_forward_task` between-sentence flush (`inter-sentence`)
  - `passthrough_forward_task` PassthroughRaw chunk (`raw-chunk`)

## Build / use

```
cargo build --release --features unicore,tx-trace --bin ublox_fake_uc \
    --target thumbv8m.main-none-eabihf
make flash
probe-rs attach --chip RP2350 ./target/.../ublox_fake_uc | tee /tmp/spoof.log
```

User runs spoof experiment. After, grep the log:

```
grep '\[TX→DRONE/' /tmp/spoof.log | sort | uniq -c   # counts per tag
grep '\[TX→DRONE/spoof-rebuild-gga\]' /tmp/spoof.log # rebuild outputs
```

## Regression baseline (2026-04-26)

Air 3S + real UC6580I, RF spoof Phase-1 (Montana, 48.49°N/-114.28°E) →
Phase-2 (SPB, 59.94°N/30.33°E). 60-second capture window:

| tag | count | content |
|---|---|---|
| `rtcm-flush` | 1929 | RTCM 1019/1077/1097/1046/1013 + 4074 sub-IDs |
| `nmea-passthrough` | 788 | `$GNGSA/$GNTXT/$PNOISE/$GxGSV/$PPSInfo` |
| `inter-sentence` | 229 | CRLF + UART artifacts |
| `spoof-rebuild-gga` | 227 | `4829.569,N / 11417.081,W` (Phase-1 + 75 m drift) |
| `spoof-rebuild-rmc` | 226 | same Phase-1 + 75 m drift |

**Searches for Phase-2 in TX bytes — all zero hits**:
- ASCII (`,E,`, `,5956.`, `,03019.`, `59.94`, `30.33`)
- int32 LE/BE for lat=599435895 (`23 B5 8E 37` / `37 8E B5 23`)
- int32 LE/BE for lon=303316483 (`12 15 0F 03` / `03 0F 15 12`)
- f64 BE/LE first/last 4 bytes

## Conclusion (2026-04-26 spoof leak hunt)

The plate's NMEA + RTCM TX path is **clean** under spoof. If a user
pilot still sees Phase-2 coordinates on the remote, the source is **not
the plate**. Likely candidates:
- Drone has its own GPS receiver besides the UC6580I-on-UART → RF spoof
  reaches that antenna directly.
- Sensor fusion (IMU + vision + aiding data).
- External aiding via RTK base / A-GPS / cellular network correction.

The `LAST_GOOD` value used for rebuild updated **once** during the
experiment (at `t=114.889` time-jump trigger), shifting from
`(48.4928194°, -114.2846959°)` to `(48.4928141°, -114.2846935°)` — a
~75 m drift typical of stationary GPS noise within 30 seconds. Same
order of magnitude as expected, no leak.

## $CFGKEY timing

Drone sends `$CFGKEY` ~19.3 seconds after first `$PDTINFO` — last
command in the standard handshake. Chip replies in ~9 ms. To capture
this via `boot-capture` or `tx-trace`, hold `probe-rs attach` open for
≥25 seconds after drone power-on. `FREEZE_AFTER_MS_DR=25_000ms` in
`raw_capture::DR` covers this.
