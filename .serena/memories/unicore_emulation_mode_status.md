# UC6580I emulation modes — drone acceptance status (2026-04-26 evening)

## Drone↔chip mapping (important context)

- **DJI Air 3S** uses **u-blox M10** (target `ublox_fake`, `src/main.rs`).
- **DJI Neo** uses **Unicore UC6580I** (target `ublox_fake_uc`, `src/main_unicore.rs`).

This memory + everything in `docs/UC6580I.md` is about **Neo + Unicore**.
Air 3S spoof tests use the u-blox build and a different code path entirely.

## Live-tested with DJI Neo

| Mode | Strategy | Drone accepts |
|---|---|---|
| 1 — Emulation | plate generates entire stream itself (hybrid: passthrough until $CFGKEY/timeout, then override with own NMEA + replayed RTCM/ExtRTCM) | ❌ rejects right at the passthrough→override switch |
| 2 — Passthrough | chip→plate→drone 1:1, no modification | ✅ |
| 3 — PassthroughRaw | byte-for-byte forward, no NMEA reassembly | ✅ presumed (not tested separately) |
| 4 — PassthroughOffset | chip stream forwarded, only `$GxGGA`/`$GxRMC` get coords rewritten to `config::offset_target` | ✅ **verified** |

## Why Mode 4 works and Mode 1 doesn't

Mode 4 sends drone **modified coordinates** (different from what chip physically sees) and drone still accepts. This works because:
- All other streams (RTCM, ExtRTCM 4074, GSA, GSV, TXT, PNOISE) flow from chip **as-is** — drone doesn't lose a byte of expected chip output.
- Only `$GxGGA` and `$GxRMC` are rewritten via `process_nmea_line` / `rewrite_coords` (`src/main_unicore.rs`) — coords get replaced with `config::offset_target`.
- Minimally-invasive substitution — drone recognises the chip by the totality of its outputs; rewriting only coordinate NMEA is tolerable.

Mode 1 (full stream replacement) is a dead-end. Even when we replay byte-exact RTCM/ExtRTCM 4074 frames, drone detects "different entity" and disconnects on the first post-handshake tick.

## Recommendation for spoofing experiments

To establish the home point at chosen coordinates on **Neo**, use **Mode 4 (PassthroughOffset)** with target set via `src/config.rs::offset_target` (currently Seney, Michigan default). Refactoring Mode 1 to match Mode 4 semantics + post-N-seconds fix-loss is a separate task in TODO.

For Air 3S (u-blox target) the picture is different — see u-blox-side memories.

## Code state at this point (commit `1f6501c` + uncommitted Mode 1 hybrid logic)

`src/main_unicore.rs` Mode 1 currently has:
- `EMULATION_VALID_UNTIL_MS: AtomicU32` — passthrough/override phase tracker (0 = passthrough)
- `EMULATION_VALID_WINDOW_MS = 10_000` ms — duration of `fq=3` valid fix in override
- `EMULATION_FORCE_OVERRIDE_AFTER_MS = 25_000` ms — fallback when drone doesn't send `$CFGKEY`
- `EMULATION_PASSTHROUGH_START_MS: AtomicU32` — for the timeout fallback
- `CFGKEY_TRIGGER = b"$CFGKEY,,"` — substring trigger in chip→drone byte stream
- In override phase: short RTCM 1077 (9 B) + ExtRTCM 4074 subs 0xFF/0xFE/0xF9/0xE6/0xE9/0x000/0x001/0x002 emitted every tick at 5 Hz, plus our generated `$GxGGA/RMC/GSA/GSV/TXT/PNOISE`.

This logic is committed-pending — Mode 1 still doesn't work despite all this. Decision: leave Mode 1 hybrid logic in place but do NOT use it; production use is Mode 4 instead.

## Documentation pointers

- `docs/UC6580I.md §4.4` — full investigation log, hypothesis chain, Mode 1 hybrid implementation details.
- This memory captures the verdict for quick lookup.
