# TODO: Improvements for algos-v2 (Passthrough Mode)

## 1. NMEA & Generic Data Passthrough ✅ DONE
- [x] **Passthrough Non-UBX Data**: In `Passthrough` (Blue) mode, modified `UbxFrameParser` to accumulate non-UBX bytes in a buffer.
    - **Implementation**: Added `non_ubx_buffer` and `take_non_ubx_data()` method
    - **Result**: NMEA, RTCM and other non-UBX data is now forwarded through `GNSS_RX_CHANNEL`

## 2. Advanced Spoof Detection: Time-based ✅ DONE
- [x] **GNSS Time-based Detection**:
    - Added `GnssTime` struct extracted from NAV-PVT
    - Detection: time jumps backwards (>1s) or forwards (>5s) = spoofing
    - Recovery: when GNSS time matches projected system clock time (±5s)
- [x] **System Clock Drift Detection**:
    - Calibrates internal clock with GNSS time (first 5 seconds)
    - Projects expected GNSS time using system clock
    - Drift > 10 seconds = spoofing indicator
    - Recovery when drift ≤ 3 seconds

## 3. Bug Fixes ✅ DONE
- [x] **SEC-SIGN not enabled in Passthrough**: Fixed by removing `MSG_OUTPUT_STARTED` wait condition for Passthrough mode
- [x] **Hash Mismatch / Packet Drops**: **FIXED** (Jan 2026)
    - Root causes: RXM-RAWX header loss, ECDSA blocking, slow crypto, channel overflow, UART FIFO overrun
    - Fixes: `is_idle()` check, `select()` buffering, crypto opt-level=3, channel depth 128, FIFO threshold 1/4
    - **Result**: 100% packet reliability (0 overruns in 5-minute test)

## Current Active Detection Algorithms
1. ✅ Teleportation (>2000m position jump)
2. ✅ Speed anomaly (>30 m/s)
3. ✅ GNSS time jumps (backwards/forward)
4. ✅ System clock drift (calibrated internal clock vs GNSS time)
5. ✅ Altitude anomaly (>10m jump)
6. ✅ Last-good/origin drift guards
7. ❌ Acceleration anomaly (DISABLED - kept in code)
8. ❌ CNO uniformity (DISABLED - kept in code)

## Historical Caveat: SEC-UNIQID Model Race (Jan 2026)

**Status**: Not the main key-selection path anymore. Current u-blox workflow auto-extracts the private SEC-SIGN key at boot and stores it in flash; `sec_sign::get_private_key()` uses that flash key before any hardcoded model key.

**Remaining impact**: Model selection still affects NAV/SEC-SIGN timings, SEC-UNIQID, CFG-0x41 template and hardcoded fallback if no flash key exists.

**Possible cleanup**: Keep or remove old model auto-detection notes only after validating whether SEC-UNIQID/template differences matter for the current drone set when a real extracted key is present.

## Completed Features
- [x] **DJI Air 3S support** (Jan 2026): Private key, SEC-UNIQID, CFG-0x41 template (no CFG-RINV, key @offset 115), timers (SEC-SIGN 2s, Config→NAV 780ms). DroneModel::Air3S = 2.

## Future Improvements
- [ ] Test NMEA passthrough with real u-blox module
- [x] Test time-based detection with simulated spoofing (tested Jan 2026)
- [ ] Add signal-level anti-jam detection (AGC analysis)
- [ ] Implement carrier-phase divergence detection
- [ ] Re-evaluate whether model auto-detection is still needed when flash key extraction succeeds
