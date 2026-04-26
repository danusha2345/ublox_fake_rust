# Unicore debug mode mailbox

Use this when working on `/home/danik/Projects/ublox_fake_rust` Unicore mode switching, board debug commands, or docs around mode persistence.

- Firmware path: `src/main_unicore.rs`.
- Unicore mode enum: `0=Emulation`, `1=Passthrough`, `2=PassthroughRaw`, `3=PassthroughOffset`.
- Button mapping is click count `1..4` to the same enum `0..3`.
- Debug mailbox: exported RAM symbol `DEBUG_MODE_REQUEST: AtomicU8`, idle value `0xFF`.
- Runtime readback symbol: exported RAM symbol `MODE: AtomicU8`.
- `debug_mode_request_task` periodically `swap(0xFF)`, accepts only `0..=3`, logs invalid values, then calls the same mode-apply helper as `button_task`.
- The shared apply path stores `MODE`, resets emulation/spoof state as needed, sets `SPOOF_DETECTOR_RESET`, saves the mode via `flash_storage::save_mode()`, and logs `Mode: old -> new`.
- Host utility: `tools/debug_set_unicore_mode.py --mode emulation|passthrough|raw|offset`.
- The utility uses `nm --defined-only` on the ELF to find `DEBUG_MODE_REQUEST` and `MODE`; it does not hardcode RAM addresses.
- It writes with `probe-rs write --chip RP2350 b8 <DEBUG_MODE_REQUEST_ADDR> <mode>` and then polls `probe-rs read --chip RP2350 b8 <MODE_ADDR> 1` unless `--no-verify` is passed.
- Do not recommend direct `probe-rs write` to `MODE` as the normal control path because it bypasses reset/save side effects.
- Verified on board: `passthrough -> MODE 1`, `offset -> MODE 3`, reset preserved `03`, `emulation -> MODE 0`, `raw -> MODE 2`; board was later left in Emulation (`MODE 0`) by user request.
- Docs updated in `README.md` and `docs/UC6580I.md`; `docs/UC6580I.md` also corrected Unicore spoof `GGA nsats` from stale `92` to current `1`.

Useful commands:

```bash
cargo build --release --features "rp2350 unicore" --bin ublox_fake_uc --target thumbv8m.main-none-eabihf
probe-rs download --chip RP2350 --verify target/thumbv8m.main-none-eabihf/release/ublox_fake_uc
tools/debug_set_unicore_mode.py --mode emulation
```