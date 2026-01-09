//! PIO-based UART passthrough for transparent forwarding
//! + UBX frame parsing and spoof detection

#![allow(dead_code)]

use embassy_rp::pio::{Common, Config, Instance, PioPin, StateMachine};
use embassy_rp::Peri;
use fixed::traits::ToFixed;
use fixed::types::U24F8;
use fixed_macro::fixed;

// ============================================================================
// UBX Frame Parser - state machine for parsing UBX frames from byte stream
// ============================================================================

/// State of UBX parser
#[derive(Clone, Copy, PartialEq)]
enum ParseState {
    WaitSync1,
    WaitSync2,
    WaitClass,
    WaitId,
    WaitLen1,
    WaitLen2,
    CollectPayload,
    WaitCkA,
    WaitCkB,
}

/// UBX frame parser - accumulates bytes and returns complete frames
/// Returns owned Vec to avoid borrow checker issues
pub struct UbxFrameParser {
    buffer: [u8; 1024],
    len: usize,
    state: ParseState,
    payload_len: usize,
    payload_collected: usize,
    // Buffer for non-UBX data (NMEA, RTCM, etc.)
    // Size matches GNSS_RX_CHANNEL capacity (1024)
    non_ubx_buffer: [u8; 1024],
    non_ubx_len: usize,
}

impl UbxFrameParser {
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; 1024],
            len: 0,
            state: ParseState::WaitSync1,
            payload_len: 0,
            payload_collected: 0,
            non_ubx_buffer: [0u8; 1024],
            non_ubx_len: 0,
        }
    }

    /// Reset parser state
    fn reset(&mut self) {
        self.len = 0;
        self.state = ParseState::WaitSync1;
        self.payload_len = 0;
        self.payload_collected = 0;
        // Don't reset non_ubx_buffer - it accumulates between UBX frames
    }

    /// Feed a byte to the parser. Returns complete frame as owned Vec if available.
    pub fn feed(&mut self, byte: u8) -> Option<heapless::Vec<u8, 1024>> {
        match self.state {
            ParseState::WaitSync1 => {
                if byte == 0xB5 {
                    self.buffer[0] = byte;
                    self.len = 1;
                    self.state = ParseState::WaitSync2;
                } else {
                    // Not a UBX sync byte - accumulate as non-UBX data  
                    if self.non_ubx_len < self.non_ubx_buffer.len() {
                        self.non_ubx_buffer[self.non_ubx_len] = byte;
                        self.non_ubx_len += 1;
                    }
                }
                None
            }
            ParseState::WaitSync2 => {
                if byte == 0x62 {
                    self.buffer[1] = byte;
                    self.len = 2;
                    self.state = ParseState::WaitClass;
                } else {
                    self.reset();
                }
                None
            }
            ParseState::WaitClass => {
                self.buffer[2] = byte;
                self.len = 3;
                self.state = ParseState::WaitId;
                None
            }
            ParseState::WaitId => {
                self.buffer[3] = byte;
                self.len = 4;
                self.state = ParseState::WaitLen1;
                None
            }
            ParseState::WaitLen1 => {
                self.buffer[4] = byte;
                self.len = 5;
                self.state = ParseState::WaitLen2;
                None
            }
            ParseState::WaitLen2 => {
                self.buffer[5] = byte;
                self.len = 6;
                self.payload_len = (self.buffer[4] as usize) | ((byte as usize) << 8);
                self.payload_collected = 0;

                // Sanity check: payload too large
                if self.payload_len > 500 {
                    self.reset();
                    return None;
                }

                if self.payload_len == 0 {
                    self.state = ParseState::WaitCkA;
                } else {
                    self.state = ParseState::CollectPayload;
                }
                None
            }
            ParseState::CollectPayload => {
                if self.len < self.buffer.len() {
                    self.buffer[self.len] = byte;
                    self.len += 1;
                    self.payload_collected += 1;
                }

                if self.payload_collected >= self.payload_len {
                    self.state = ParseState::WaitCkA;
                }
                None
            }
            ParseState::WaitCkA => {
                if self.len < self.buffer.len() {
                    self.buffer[self.len] = byte;
                    self.len += 1;
                }
                self.state = ParseState::WaitCkB;
                None
            }
            ParseState::WaitCkB => {
                if self.len < self.buffer.len() {
                    self.buffer[self.len] = byte;
                    self.len += 1;
                }

                // Complete frame received - verify checksum and return copy
                let frame_len = self.len;
                let result = if self.verify_checksum() {
                    let mut vec = heapless::Vec::new();
                    let _ = vec.extend_from_slice(&self.buffer[..frame_len]);
                    Some(vec)
                } else {
                    None
                };

                self.reset();
                result
            }
        }
    }

    /// Verify Fletcher-8 checksum
    fn verify_checksum(&self) -> bool {
        if self.len < 8 {
            return false;
        }

        let mut ck_a: u8 = 0;
        let mut ck_b: u8 = 0;

        // Checksum over class, id, length, payload (bytes 2 to len-2)
        for &b in &self.buffer[2..self.len - 2] {
            ck_a = ck_a.wrapping_add(b);
            ck_b = ck_b.wrapping_add(ck_a);
        }

        ck_a == self.buffer[self.len - 2] && ck_b == self.buffer[self.len - 1]
    }

    /// Take accumulated non-UBX data and clear the buffer
    /// Returns None if no data accumulated
    pub fn take_non_ubx_data(&mut self) -> Option<heapless::Vec<u8, 1024>> {
        if self.non_ubx_len > 0 {
            let mut vec = heapless::Vec::new();
            if vec.extend_from_slice(&self.non_ubx_buffer[..self.non_ubx_len]).is_ok() {
                self.non_ubx_len = 0;
                Some(vec)
            } else {
                None
            }
        } else {
            None
        }
    }
}

// ============================================================================
// Position Buffer - ring buffer for 3 seconds of position history (5Hz = 15 samples)
// ============================================================================

/// Position entry with timestamp
#[derive(Clone, Copy, Default)]
pub struct PositionEntry {
    pub lat: i32,      // degrees * 1e-7
    pub lon: i32,      // degrees * 1e-7
    pub alt: i32,      // mm
    pub timestamp_ms: u32,
}

/// Ring buffer for position history
pub struct PositionBuffer {
    entries: [PositionEntry; 15],  // 3 seconds at 5Hz
    write_idx: usize,
    count: usize,
}

impl PositionBuffer {
    pub const fn new() -> Self {
        Self {
            entries: [PositionEntry { lat: 0, lon: 0, alt: 0, timestamp_ms: 0 }; 15],
            write_idx: 0,
            count: 0,
        }
    }

    /// Add new position to buffer
    pub fn push(&mut self, lat: i32, lon: i32, alt: i32, timestamp_ms: u32) {
        self.entries[self.write_idx] = PositionEntry { lat, lon, alt, timestamp_ms };
        self.write_idx = (self.write_idx + 1) % 15;
        if self.count < 15 {
            self.count += 1;
        }
    }

    /// Get position from N seconds ago (approximate)
    /// Returns None if not enough history
    pub fn get_position_at(&self, seconds_ago: u32, current_time_ms: u32) -> Option<(i32, i32, i32)> {
        if self.count == 0 {
            return None;
        }

        let target_time = current_time_ms.wrapping_sub(seconds_ago * 1000);
        let mut best_idx = 0;
        let mut best_diff = u32::MAX;

        // Find entry closest to target time
        for i in 0..self.count {
            let idx = (self.write_idx + 15 - 1 - i) % 15;
            let entry = &self.entries[idx];
            let diff = entry.timestamp_ms.abs_diff(target_time);

            if diff < best_diff {
                best_diff = diff;
                best_idx = idx;
            }
        }

        let entry = &self.entries[best_idx];
        Some((entry.lat, entry.lon, entry.alt))
    }
}

// ============================================================================
// NAV Message Modification Functions
// ============================================================================

/// Recalculate Fletcher-8 checksum for UBX frame
pub fn recalc_checksum(frame: &mut [u8]) {
    if frame.len() < 8 {
        return;
    }

    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;

    // Checksum over class, id, length, payload (bytes 2 to len-2)
    for &b in &frame[2..frame.len() - 2] {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    let len = frame.len();
    frame[len - 2] = ck_a;
    frame[len - 1] = ck_b;
}

/// Modify NAV-PVT (0x01 0x07): set fix_type=0, flags=0, num_sv=2
/// Payload: 92 bytes, offsets: fix_type=20, flags=21, num_sv=23
pub fn modify_nav_pvt(frame: &mut [u8]) {
    // Frame = sync(2) + class(1) + id(1) + len(2) + payload(92) + ck(2) = 100 bytes
    if frame.len() >= 100 {
        frame[6 + 20] = 0;  // fix_type = 0 (no fix)
        frame[6 + 21] = 0;  // flags = 0
        frame[6 + 23] = 2;  // num_sv = 2
    }
}

/// Modify NAV-SOL (0x01 0x06): set gps_fix=0, num_sv=2
/// Payload: 52 bytes, offsets: gps_fix=10, num_sv=47
pub fn modify_nav_sol(frame: &mut [u8]) {
    // Frame = sync(2) + class(1) + id(1) + len(2) + payload(52) + ck(2) = 60 bytes
    if frame.len() >= 60 {
        frame[6 + 10] = 0;  // gps_fix = 0
        frame[6 + 47] = 2;  // num_sv = 2
    }
}

/// Modify NAV-STATUS (0x01 0x03): set gps_fix=0, flags=0
/// Payload: 16 bytes, offsets: gps_fix=4, flags=5
pub fn modify_nav_status(frame: &mut [u8]) {
    // Frame = sync(2) + class(1) + id(1) + len(2) + payload(16) + ck(2) = 24 bytes
    if frame.len() >= 24 {
        frame[6 + 4] = 0;   // gps_fix = 0
        frame[6 + 5] = 0;   // flags = 0
    }
}

/// Modify NAV-SAT (0x01 0x35): set num_svs=2
/// Payload: 8 + 12*n bytes, offset: num_svs=5
pub fn modify_nav_sat(frame: &mut [u8]) {
    // Frame header = 8 bytes minimum
    if frame.len() >= 14 {  // 6 + 8 minimum
        frame[6 + 5] = 2;   // num_svs = 2
    }
}

/// Modify NAV-SVINFO (0x01 0x30): set num_ch=2
/// Payload: 8 + 12*n bytes, offset: num_ch=4
pub fn modify_nav_svinfo(frame: &mut [u8]) {
    // Frame header = 8 bytes minimum
    if frame.len() >= 14 {  // 6 + 8 minimum
        frame[6 + 4] = 2;   // num_ch = 2
    }
}

/// Extract position data from NAV-PVT payload
/// Returns (lat, lon, alt, h_acc, speed, num_sv)
pub fn extract_position_from_pvt(payload: &[u8]) -> Option<(i32, i32, i32, u32, i32, u8)> {
    if payload.len() < 92 {
        return None;
    }

    // Offsets in NAV-PVT payload:
    // lon: 24-27 (i32, 1e-7 deg)
    // lat: 28-31 (i32, 1e-7 deg)
    // height: 32-35 (i32, mm)
    // hAcc: 40-43 (u32, mm)
    // velN: 48-51 (i32, mm/s)
    // velE: 52-55 (i32, mm/s)
    // numSV: 23 (u8)

    let lon = i32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]);
    let lat = i32::from_le_bytes([payload[28], payload[29], payload[30], payload[31]]);
    let height = i32::from_le_bytes([payload[32], payload[33], payload[34], payload[35]]);
    let h_acc = u32::from_le_bytes([payload[40], payload[41], payload[42], payload[43]]);
    let vel_n = i32::from_le_bytes([payload[48], payload[49], payload[50], payload[51]]);
    let vel_e = i32::from_le_bytes([payload[52], payload[53], payload[54], payload[55]]);
    let num_sv = payload[23];

    // Calculate ground speed in mm/s
    let vel_n_f = vel_n as f32;
    let vel_e_f = vel_e as f32;
    let speed = libm::sqrtf(vel_n_f * vel_n_f + vel_e_f * vel_e_f) as i32;

    Some((lat, lon, height, h_acc, speed, num_sv))
}

/// Extract GNSS time from NAV-PVT payload
/// Returns GnssTime if date and time are valid
pub fn extract_gnss_time_from_pvt(payload: &[u8], system_time_ms: u32) -> Option<crate::spoof_detector::GnssTime> {
    if payload.len() < 92 {
        return None;
    }
    
    // NAV-PVT payload offsets (relative to payload start):
    // iTOW: 0-3 (u32, ms) - GPS time of week
    // year: 4-5 (u16) - UTC year
    // month: 6 (u8) - UTC month 1-12
    // day: 7 (u8) - UTC day 1-31
    // hour: 8 (u8) - UTC hour 0-23
    // min: 9 (u8) - UTC minute 0-59
    // sec: 10 (u8) - UTC second 0-60
    // valid: 11 (u8) - validity flags (bit 0: validDate, bit 1: validTime, bit 2: fullyResolved)
    
    let itow_ms = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
    let year = u16::from_le_bytes([payload[4], payload[5]]);
    let month = payload[6];
    let day = payload[7];
    let hour = payload[8];
    let min = payload[9];
    let sec = payload[10];
    let valid = payload[11];
    
    // Check if date and time are valid (bits 0 and 1 set)
    if (valid & 0x03) != 0x03 {
        return None;
    }
    
    // Basic sanity checks
    if !(1980..=2100).contains(&year) {
        return None;
    }
    if !(1..=12).contains(&month) {
        return None;
    }
    if !(1..=31).contains(&day) {
        return None;
    }
    if hour > 23 || min > 59 || sec > 60 {
        return None;
    }
    
    Some(crate::spoof_detector::GnssTime {
        itow_ms,
        year,
        month,
        day,
        hour,
        min,
        sec,
        system_time_ms,
    })
}

/// Extract CNO (Carrier-to-Noise) values from NAV-SAT payload
/// Returns Vec of CNO values (dB-Hz) for satellites with valid signal
/// Used for detecting uniform high CNO (spoof indicator)
pub fn extract_cno_from_nav_sat(payload: &[u8]) -> heapless::Vec<u8, 16> {
    let mut cno_values: heapless::Vec<u8, 16> = heapless::Vec::new();
    
    // NAV-SAT structure:
    // Header: 8 bytes (iTOW[4], version[1], numSvs[1], reserved[2])
    // Per-satellite block: 12 bytes each
    //   - gnssId[1], svId[1], cno[1], elev[1], azim[2], prRes[2], flags[4]
    
    if payload.len() < 8 {
        return cno_values;
    }
    
    let num_svs = payload[5] as usize;
    
    // Check payload size
    let expected_size = 8 + num_svs * 12;
    if payload.len() < expected_size {
        return cno_values;
    }
    
    // Extract CNO for each satellite with valid signal
    for i in 0..num_svs.min(16) {
        let offset = 8 + i * 12;
        let cno = payload[offset + 2]; // CNO at offset 2 within satellite block
        let flags = u32::from_le_bytes([
            payload[offset + 8],
            payload[offset + 9],
            payload[offset + 10],
            payload[offset + 11],
        ]);
        
        // Check if satellite is used in solution (bit 3 of flags)
        // Only consider satellites that are actually being used
        let sv_used = (flags & 0x08) != 0;
        
        // Only include satellites with valid CNO (not 0) that are used
        if cno > 0 && sv_used {
            let _ = cno_values.push(cno);
        }
    }
    
    cno_values
}

/// PIO passthrough driver - copies input pin state to output pin
pub struct Passthrough<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> Passthrough<'d, P, S> {
    /// Create new passthrough driver
    /// - in_pin: GPIO3 (PIO input from external GNSS)
    /// - out_pin: GPIO0 (UART TX, output to host)
    pub fn new<IN: PioPin, OUT: PioPin>(
        common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        in_pin: Peri<'d, IN>,
        out_pin: Peri<'d, OUT>,
    ) -> Self {
        // PIO program: copy input pin state to output pin
        // wait 1 pin 0  - wait for input high
        // set pins, 1   - set output high
        // wait 0 pin 0  - wait for input low
        // set pins, 0   - set output low
        let prg = pio::pio_asm!(
            ".wrap_target",
            "    wait 1 pin 0",
            "    set pins, 1",
            "    wait 0 pin 0",
            "    set pins, 0",
            ".wrap"
        );

        let pio_in_pin = common.make_pio_pin(in_pin);
        let pio_out_pin = common.make_pio_pin(out_pin);

        let mut cfg = Config::default();
        cfg.use_program(&common.load_program(&prg.program), &[]);
        cfg.set_in_pins(&[&pio_in_pin]);
        cfg.set_set_pins(&[&pio_out_pin]);

        // Fast clock for accurate signal copying at 921600 baud
        // 8MHz gives ~8 samples per bit at 921600 baud
        let clock_freq = fixed!(8_000_000: U24F8);
        cfg.clock_divider = (U24F8::from_num(125_000_000u32) / clock_freq).to_fixed();

        sm.set_config(&cfg);
        sm.set_pin_dirs(embassy_rp::pio::Direction::In, &[&pio_in_pin]);
        sm.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&pio_out_pin]);

        // Start disabled - will be enabled when switching to passthrough mode
        sm.set_enable(false);

        Self { sm }
    }

    /// Enable passthrough
    pub fn enable(&mut self) {
        self.sm.set_enable(true);
    }

    /// Disable passthrough
    pub fn disable(&mut self) {
        self.sm.set_enable(false);
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        // StateMachine doesn't have is_enabled, track externally
        false
    }
}
