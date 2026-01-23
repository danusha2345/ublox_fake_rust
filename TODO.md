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
5. ❌ Altitude anomaly (DISABLED - kept in code)
6. ❌ Acceleration anomaly (DISABLED - kept in code)
7. ❌ CNO uniformity (DISABLED - kept in code)

## 🐛 Known Bug: SEC-UNIQID Race Condition (Jan 2026)

**Problem**: When Mavic 4 Pro connects, auto-detection triggers at CFG-VALSET, but SEC-UNIQID is sent BEFORE that (in response to SEC-UNIQID poll). Result: Mavic 4 receives Air 3's chip ID.

**Symptom**: Auto-detection shows "Mavic 4 Pro", SEC-SIGN uses correct key, but signatures rejected by drone.

**Timeline**:
1. Mavic sends SEC-UNIQID poll → We respond with **Air 3 chip ID** (wrong!)
2. Mavic sends CFG-VALGET → SAW_CFG_VALGET = true
3. Mavic sends CFG-VALSET → **Detection triggers** → DRONE_MODEL = Mavic4Pro (too late)
4. SEC-SIGN computed with Mavic 4 key → Signature OK, but drone already has wrong ID

**Workaround**: Set `DRONE_MODEL` default to Mavic 4 Pro in `main.rs:172`

**Fix (TODO)**: Detect Mavic 4 Pro immediately on SEC-UNIQID poll (Air 3 never sends this):
```rust
ubx::UbxCommand::SecUniqidPoll => {
    if !DRONE_DETECTED.load(Ordering::Acquire) {
        DRONE_MODEL.store(1, Ordering::Release);  // Mavic 4 Pro
        DRONE_DETECTED.store(true, Ordering::Release);
    }
    // ... send correct SEC-UNIQID
}
```

## Future Improvements
- [ ] Test NMEA passthrough with real u-blox module
- [x] Test time-based detection with simulated spoofing (tested Jan 2026)
- [ ] Add signal-level anti-jam detection (AGC analysis)
- [ ] Implement carrier-phase divergence detection
- [ ] **Fix SEC-UNIQID auto-detection race condition**
