# TODO: Improvements for algos-v2 (Passthrough Mode)

## 1. NMEA & Generic Data Passthrough
- [ ] **Passthrough Non-UBX Data**: In `Passthrough` (Blue) mode, currently only valid UBX frames (`0xB5 0x62 ...`) are parsed and forwarded. All other bytes (NMEA `$GNGGA`, RTCM, etc.) are discarded.
    - **Goal**: Implement a fallback mechanism to forward bytes that are not part of a UBX frame.
    - **Implementation**: If `parser.feed(byte)` returns `None` and doesn't consume the byte into the internal buffer (need to check parser logic), or if we track "non-matching" bytes, they should be accumulated and sent to `GNSS_RX_CHANNEL`.
    - **Note**: NMEA messages usually come *before* UBX messages in the burst.

## 2. Advanced Spoof Detection: Time/Date Jumps
- [ ] **Time-based Recovery Detection**:
    - **Problem**: Distance-based recovery check (`TELEPORT_M`) might be insufficient if the spoofer smoothly drags the position back.
    - **Solution**: Use GNSS time for validation.
    - **Logic**:
        1. Capture "Real Time" (from `NAV-PVT`) before spoofing is detected (e.g., maintain a "Last Good Time" + internal monotonic clock offset).
        2. When Spoofing is active: Compare incoming packet time with the projected "Real Time".
        3. **Detection**: If incoming time sharply jumps *back* to the "Real Time" (or matches the projected real time consistently), it indicates the spoofer has stopped (or we are receiving real signals again).
        4. **Action**: Use this as a strong signal to clear `SPOOF_DETECTED` status.
