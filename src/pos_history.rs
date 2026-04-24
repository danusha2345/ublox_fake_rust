//! Position history ring buffer and dynamic coordinate offset.
//!
//! Shared between the u-blox UBX build (`main.rs` + `passthrough.rs`) and
//! the Unicore NMEA build (`main_unicore.rs`). The logic is protocol-agnostic:
//! coordinates come in as `i32` (lat/lon in degrees × 1e-7, altitude in mm)
//! paired with a monotonic millisecond timestamp.

const CAPACITY: usize = 15; // 3 s @ 5 Hz

#[derive(Clone, Copy, Default)]
pub struct PositionEntry {
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub timestamp_ms: u32,
}

pub struct PositionBuffer {
    entries: [PositionEntry; CAPACITY],
    write_idx: usize,
    count: usize,
}

impl PositionBuffer {
    pub const fn new() -> Self {
        Self {
            entries: [PositionEntry { lat: 0, lon: 0, alt: 0, timestamp_ms: 0 }; CAPACITY],
            write_idx: 0,
            count: 0,
        }
    }

    pub fn push(&mut self, lat: i32, lon: i32, alt: i32, timestamp_ms: u32) {
        self.entries[self.write_idx] = PositionEntry { lat, lon, alt, timestamp_ms };
        self.write_idx = (self.write_idx + 1) % CAPACITY;
        if self.count < CAPACITY {
            self.count += 1;
        }
    }

    /// Returns the entry whose timestamp is closest to `seconds_ago` before `current_time_ms`,
    /// considering only entries that are **at least** `seconds_ago` old. The age gate matters
    /// when a spoof sample was just pushed (callers `push()` before analysing): without it,
    /// the fresh spoofed entry wins as "closest to 2 s ago" against stale pre-gap fixes and
    /// then gets stored as `LAST_GOOD`. `None` if no eligible entry exists.
    pub fn get_position_at(&self, seconds_ago: u32, current_time_ms: u32) -> Option<(i32, i32, i32)> {
        if self.count == 0 {
            return None;
        }
        let lookback_ms = seconds_ago * 1000;
        let target = current_time_ms.wrapping_sub(lookback_ms);
        let mut best_idx: Option<usize> = None;
        let mut best_diff = u32::MAX;
        for i in 0..self.count {
            let idx = (self.write_idx + CAPACITY - 1 - i) % CAPACITY;
            let age = current_time_ms.wrapping_sub(self.entries[idx].timestamp_ms);
            if age < lookback_ms {
                continue;
            }
            let diff = self.entries[idx].timestamp_ms.abs_diff(target);
            if diff < best_diff {
                best_diff = diff;
                best_idx = Some(idx);
            }
        }
        best_idx.map(|idx| {
            let e = &self.entries[idx];
            (e.lat, e.lon, e.alt)
        })
    }
}

/// Coordinate offset (lat/lon in 1e-7 deg, altitude in mm). Computed once per
/// flight at the first valid 3D fix and kept constant so distance/heading
/// calculations aboard the drone stay self-consistent.
#[derive(Clone, Copy)]
pub struct DynamicOffset {
    pub lat_1e7: i32,
    pub lon_1e7: i32,
    pub alt_mm: i32,
}
