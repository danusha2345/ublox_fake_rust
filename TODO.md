# TODO: Improvements for algos-v2 (Passthrough Mode)

## 1. NMEA & Generic Data Passthrough ✅ DONE
- [x] **Passthrough Non-UBX Data**: In `Passthrough` (Blue) mode, modified `UbxFrameParser` to accumulate non-UBX bytes in a buffer.
    - **Implementation**: Added `non_ubx_buffer` and `take_non_ubx_data()` method
    - **Result**: NMEA, RTCM and other non-UBX data is now forwarded through `GNSS_RX_CHANNEL`

## 2. Advanced Spoof Detection: Time-based ✅ DONE
- [x] **GNSS Time-based Detection**:
    - Added `GnssTime` struct extracted from NAV-PVT
    - Detection: time jumps backwards (>1s) or forwards (>30s) = spoofing
    - Recovery: when GNSS time matches projected system clock time (±5s)
- [x] **System Clock Drift Detection**:
    - Calibrates internal clock with GNSS time (first 5 seconds)
    - Projects expected GNSS time using system clock
    - Drift > 10 seconds = spoofing indicator
    - Recovery when drift ≤ 3 seconds

## Current Active Detection Algorithms
1. ✅ Teleportation (>500m position jump)
2. ✅ Speed anomaly (>30 m/s)
3. ✅ GNSS time jumps (backwards/forward)
4. ✅ System clock drift (calibrated internal clock vs GNSS time)
5. ❌ Altitude anomaly (DISABLED - kept in code)
6. ❌ Acceleration anomaly (DISABLED - kept in code)
7. ❌ CNO uniformity (DISABLED - kept in code)

## Future Improvements
- [ ] Test NMEA passthrough with real u-blox module
- [ ] Test time-based detection with simulated spoofing
- [ ] Add signal-level anti-jam detection (AGC analysis)
- [ ] Implement carrier-phase divergence detection
