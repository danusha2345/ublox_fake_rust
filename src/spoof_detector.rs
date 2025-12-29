//! Dynamic spoofing detector for passthrough mode
//!
//! Detects GPS spoofing by analyzing coordinate changes:
//! - Teleportation (sudden position jumps)
//! - Unrealistic speed (too fast movement)
//! - Altitude anomalies (sudden height changes)
//!
//! When spoofing is detected, blocks coordinate transmission.
//! When data normalizes, unblocks transmission.

use defmt::*;

/// Spoofing detection thresholds
pub mod thresholds {
    /// Teleportation threshold in meters (jump > 1 km = spoof)
    pub const TELEPORT_M: f32 = 1000.0;

    /// Altitude jump threshold in meters (> 5 m instant jump = spoof)
    pub const TELEPORT_ALT_M: f32 = 5.0;

    /// Maximum realistic speed in m/s (25 m/s = 90 km/h for DJI drones)
    pub const MAX_SPEED_MS: f32 = 25.0;

    /// Maximum realistic vertical speed in m/s (10 m/s for consumer drones)
    pub const MAX_VERTICAL_SPEED_MS: f32 = 10.0;

    /// Minimum samples to confirm spoofing (1 = immediate detection)
    pub const SPOOF_CONFIRM_COUNT: u8 = 1;

    /// Minimum normal samples to clear spoofing flag
    pub const NORMAL_CONFIRM_COUNT: u8 = 5;

    /// Maximum gap before resetting reference (ms)
    /// If no data for this long, don't trigger false positive on first packet
    pub const MAX_GAP_MS: u32 = 5000;
}

/// GPS fix type from NAV-PVT
#[derive(Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum FixType {
    #[default]
    NoFix = 0,
    DeadReckoning = 1,
    Fix2D = 2,
    Fix3D = 3,
    GnssDr = 4,
    TimeOnly = 5,
}

impl FixType {
    pub fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::DeadReckoning,
            2 => Self::Fix2D,
            3 => Self::Fix3D,
            4 => Self::GnssDr,
            5 => Self::TimeOnly,
            _ => Self::NoFix,
        }
    }

    /// Returns true if fix has reliable 3D position (altitude valid)
    pub fn has_3d_fix(&self) -> bool {
        matches!(self, Self::Fix3D | Self::GnssDr)
    }
}

/// Position sample for analysis
#[derive(Clone, Copy, Default)]
pub struct Position {
    /// Latitude in 1e-7 degrees
    pub lat: i32,
    /// Longitude in 1e-7 degrees
    pub lon: i32,
    /// Altitude in millimeters
    pub alt_mm: i32,
    /// Timestamp in milliseconds
    pub time_ms: u32,
    /// GPS fix type (need 3D fix for altitude checks)
    pub fix_type: FixType,
}

/// Spoofing detector state
pub struct SpoofDetector {
    /// Last known good position (before spoofing)
    last_good: Option<Position>,

    /// Previous position (for velocity calculation)
    prev: Option<Position>,

    /// Currently detected as spoofed
    spoofed: bool,

    /// Consecutive anomaly counter
    anomaly_count: u8,

    /// Consecutive normal counter (for clearing spoof flag)
    normal_count: u8,

    /// Total anomalies detected (statistics)
    total_anomalies: u32,
}

/// Result of position analysis
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AnalysisResult {
    /// Normal position, pass through
    Normal,
    /// Spoofing detected, block transmission
    Spoofed,
    /// First sample, need more data
    Initializing,
    /// Gap in data, reference reset
    GapReset,
}

impl SpoofDetector {
    pub const fn new() -> Self {
        Self {
            last_good: None,
            prev: None,
            spoofed: false,
            anomaly_count: 0,
            normal_count: 0,
            total_anomalies: 0,
        }
    }

    /// Check if currently in spoofed state
    pub fn is_spoofed(&self) -> bool {
        self.spoofed
    }

    /// Get total anomaly count (for diagnostics)
    pub fn total_anomalies(&self) -> u32 {
        self.total_anomalies
    }

    /// Analyze new position and determine if it's spoofed
    pub fn analyze(&mut self, pos: Position) -> AnalysisResult {
        // Skip samples without 3D fix - altitude unreliable during acquisition
        if !pos.fix_type.has_3d_fix() {
            debug!("No 3D fix (type={}), skipping", pos.fix_type as u8);
            return AnalysisResult::Initializing;
        }

        // First sample with 3D fix - initialize
        let prev = match self.prev {
            Some(p) => p,
            None => {
                self.prev = Some(pos);
                self.last_good = Some(pos);
                info!("Spoof detector initialized with 3D fix: lat={}, lon={}, alt={}mm",
                      pos.lat, pos.lon, pos.alt_mm);
                return AnalysisResult::Initializing;
            }
        };

        // Previous sample didn't have 3D fix - reinitialize
        if !prev.fix_type.has_3d_fix() {
            self.prev = Some(pos);
            self.last_good = Some(pos);
            info!("First 3D fix after acquisition: lat={}, lon={}", pos.lat, pos.lon);
            return AnalysisResult::Initializing;
        }

        // Check for data gap
        let dt_ms = pos.time_ms.wrapping_sub(prev.time_ms);
        if dt_ms > thresholds::MAX_GAP_MS {
            // Long gap - reset reference, don't trigger false positive
            info!("GPS gap detected ({}ms), resetting reference", dt_ms);
            self.prev = Some(pos);
            // Don't update last_good - keep it for comparison after gap
            return AnalysisResult::GapReset;
        }

        if dt_ms == 0 {
            // Same timestamp, skip
            return if self.spoofed { AnalysisResult::Spoofed } else { AnalysisResult::Normal };
        }

        // Calculate movement
        let dt_s = dt_ms as f32 / 1000.0;
        let analysis = self.calculate_movement(&prev, &pos, dt_s);

        // Check for anomalies
        let is_anomaly = analysis.is_teleport
            || analysis.is_speed_anomaly
            || analysis.is_alt_anomaly;

        if is_anomaly {
            self.anomaly_count = self.anomaly_count.saturating_add(1);
            self.normal_count = 0;
            self.total_anomalies += 1;

            if analysis.is_teleport {
                warn!("TELEPORT detected: {}m jump", analysis.distance_m as i32);
            }
            if analysis.is_speed_anomaly {
                warn!("SPEED anomaly: {} m/s", analysis.speed_ms as i32);
            }
            if analysis.is_alt_anomaly {
                warn!("ALTITUDE anomaly: {}m jump, {} m/s vertical",
                      analysis.alt_diff_m as i32, analysis.vertical_speed_ms as i32);
            }

            // Confirm spoofing after consecutive anomalies
            if self.anomaly_count >= thresholds::SPOOF_CONFIRM_COUNT && !self.spoofed {
                self.spoofed = true;
                error!("SPOOFING CONFIRMED - blocking transmission");
            }
        } else {
            // Normal sample
            self.normal_count = self.normal_count.saturating_add(1);
            self.anomaly_count = 0;

            // Clear spoofing after enough normal samples
            if self.spoofed && self.normal_count >= thresholds::NORMAL_CONFIRM_COUNT {
                // Additional check: are we back near last_good position?
                if let Some(good) = self.last_good {
                    let dist_from_good = Self::calc_distance(
                        good.lat, good.lon, pos.lat, pos.lon
                    );

                    if dist_from_good < thresholds::TELEPORT_M {
                        self.spoofed = false;
                        self.last_good = Some(pos);
                        info!("Spoofing cleared - back to normal position");
                    } else {
                        // Still far from last good position, might be legitimate travel
                        // or spoofing stopped at fake position
                        debug!("Normal data but {}m from last good pos", dist_from_good as i32);
                    }
                } else {
                    self.spoofed = false;
                    info!("Spoofing cleared - normalized");
                }
            }

            // Update last_good if not spoofed
            if !self.spoofed {
                self.last_good = Some(pos);
            }
        }

        // Always update prev for next iteration
        self.prev = Some(pos);

        if self.spoofed {
            AnalysisResult::Spoofed
        } else {
            AnalysisResult::Normal
        }
    }

    /// Calculate distance between two points in meters
    fn calc_distance(lat1: i32, lon1: i32, lat2: i32, lon2: i32) -> f32 {
        // Convert from 1e-7 degrees to meters
        // 1 degree latitude ≈ 111 km
        // dlat in 1e-7 deg * 0.0111 = meters
        let dlat_m = (lat2 - lat1) as f32 * 0.0111;

        // Longitude correction for latitude (cos approximation for 45-55°)
        let dlon_m = (lon2 - lon1) as f32 * 0.0111 * 0.7;

        libm::sqrtf(dlat_m * dlat_m + dlon_m * dlon_m)
    }

    /// Analyze movement between two positions
    fn calculate_movement(&self, prev: &Position, curr: &Position, dt_s: f32) -> MovementAnalysis {
        let distance_m = Self::calc_distance(prev.lat, prev.lon, curr.lat, curr.lon);
        let alt_diff_m = (curr.alt_mm - prev.alt_mm) as f32 / 1000.0;
        let speed_ms = distance_m / dt_s;
        let vertical_speed_ms = libm::fabsf(alt_diff_m) / dt_s;

        MovementAnalysis {
            distance_m,
            alt_diff_m: libm::fabsf(alt_diff_m),
            speed_ms,
            vertical_speed_ms,
            is_teleport: distance_m > thresholds::TELEPORT_M,
            is_speed_anomaly: speed_ms > thresholds::MAX_SPEED_MS,
            is_alt_anomaly: libm::fabsf(alt_diff_m) > thresholds::TELEPORT_ALT_M
                || vertical_speed_ms > thresholds::MAX_VERTICAL_SPEED_MS,
        }
    }

    /// Force reset detector state (e.g., on mode change)
    pub fn reset(&mut self) {
        self.last_good = None;
        self.prev = None;
        self.spoofed = false;
        self.anomaly_count = 0;
        self.normal_count = 0;
        info!("Spoof detector reset");
    }

    /// Get last known good position (for comparison/display)
    pub fn last_good_position(&self) -> Option<Position> {
        self.last_good
    }
}

/// Movement analysis results
struct MovementAnalysis {
    distance_m: f32,
    alt_diff_m: f32,
    speed_ms: f32,
    vertical_speed_ms: f32,
    is_teleport: bool,
    is_speed_anomaly: bool,
    is_alt_anomaly: bool,
}

impl Default for SpoofDetector {
    fn default() -> Self {
        Self::new()
    }
}
