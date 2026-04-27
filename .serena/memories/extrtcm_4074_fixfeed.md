# ExtRTCM 4074 — authoritative fix-feed binary for Neo (UC6580I)

Discovered 2026-04-27 after NMEA-only Mode 0 (Forced3dFix + GNTXT + GSA fill + GSV synth) failed to give Neo a 3D fix.

## Why it matters

Drone reads 4074 sub-ID `0x0FF` (Receiver Information) for fix Quality. While chip has no signal, this binary says `Quality=0, SatNum=0` and our entire NMEA rewrite stack is ignored. NMEA fq=3 alone won't convince Neo.

## Frame envelope

Standard RTCM3 — preamble `D3 00 LL` (10-bit length, big-endian top 6 bits zero) + payload + CRC24Q. Inside payload first 24 bits = `[msg_type:12 = 4074][sub_id:12]`. Helpers exist in `src/unicore/rtcm3.rs` (CRC24Q + verify) and `src/unicore/extrtcm.rs` (bit-exact Sub-ID samples with golden frames).

## 4074-0xFF Receiver Information (160 bytes payload after sub-id)

Big Endian. Critical fields:
- offset 7: SatNum U8 (sats used in positioning)
- offset 8: Lon S64 (units 2^-32 deg, positive=East)
- offset 16: Lat S64 (same units, positive=North)
- offset 24: Hae S32 (mm)
- offset 28: Hmsl S32 (mm)
- offset 32: ECEF X S64 mm
- offset 40: ECEF Y S64 mm
- offset 48: ECEF Z S64 mm
- **offset 56: Quality U8 (0=invalid, 1=single, 2=DGPS, 4=RTK fixed, 5=RTK float, 6=INS)**
- offset 57: Vel_E/N/U S32×3 (mm/s)
- offset 73: Heading U16 (×0.01 deg)
- offsets 75–83: HDOP/VDOP/PDOP/GDOP/TDOP U16×5 (×0.01)
- offsets 85–124: position+velocity accuracies (E/N/U + ECEF + Vel*Acc) U32×9 (mm/mm/s)
- offset 125: ClkErr S32 ns; offset 129: ClkDrift S32 ×0.1 Hz
- offsets 133–140: Year U16 / Month U8 / Day U8 / Hour U8 / Min U8 / mSec U16 (UTC)
- offsets 141–145: StationID U16 / DiffAge U8 / CACC U16
- offsets 146–159: Reserved U16×7

## 4074-0xFE Signal Information (variable length)

`11 + SatNum × (8 + 6×Nf)` bytes payload. Header = Version U8, Week U16, Tow U32, SatNum U32. Per sat = Prn U8, System U8 (1=GPS, 2=GLO, 3=GAL, 4=BDS, 5=QZSS, 6=SBAS), El U16 (×0.1 deg), Az U16 (×0.1 deg), **InUse U8 (0/1)**, Nf U8, then per freq: FreqID U8, CN0 U8, PrResi U16, DpResi U16.

## Source

Unicore Firebird II Series Protocol Specification R1.2, §5.2.2.1 / §5.2.2.2. Local copy `.scratch/unicore_doc/firebird_ii_protocol.txt`. Spec URL: https://en.unicore.com/uploads/file/UFirebirdII%20Series%20Protocol%20Specification_EN_R1.2.pdf

## Conversion target → 4074-FF

- `lat_2pn32 = (lat_1e7 as i64) * (1<<32) / 10_000_000`
- `lon_2pn32 = (lon_1e7 as i64) * (1<<32) / 10_000_000`
- `hmsl_mm = alt_mm`

## Other 4074 sub-IDs observed in session (verbatim, not authoritative for fix)

0x0FB (Iono), 0x0F9 (Protection Level), 0x0EA (Leap), 0x0E9 (Jamming/Spoofing), 0x0E8 (SBAS), 0x0E6 (Hardware Status), 0x0E4 (PPS). Plus undocumented short subs 0x000/0x001/0x002 (3-5 byte payloads).

Sensor-fusion subs 0x00B–0x014 (GYOACC, NAVATT, IMURAW, INSPVA, IMUVEH, DR Protection) — UM621/UM681A only, NOT emitted by UC6580.

## Action plan

1. Test B (drop 4074 in RTCM router by msg-id filter) — simple ~10 lines, verifies whether dropping the binary helps drone fall back to NMEA.
2. If B doesn't help → option D: synthesize 4074-FF + 4074-FE with target state. ~150-250 lines.

## Updated docs

- `docs/UC6580I.md` §6.3.1 (full 0x0FF layout), §6.3.2 (full 0x0FE layout), §4.4 TODO with `Найден блокер` callout.
- `CLAUDE.md` Key Gotchas — Neo's authoritative fix feed bullet.
- Auto memory `project_unicore_extrtcm_4074_fixfeed.md`.
