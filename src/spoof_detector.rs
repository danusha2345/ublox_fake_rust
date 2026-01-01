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
    /// Teleportation threshold in meters (jump > 500m = spoof)
    /// Reduced from 1km for better sensitivity
    pub const TELEPORT_M: f32 = 500.0;

    /// Altitude jump threshold in meters (> 10m instant jump = spoof)
    /// Increased from 5m to reduce false positives from GPS noise
    pub const TELEPORT_ALT_M: f32 = 10.0;

    /// Maximum realistic speed in m/s (30 m/s = 108 km/h for racing drones)
    /// DJI FPV: 140 km/h in M mode, but typical flight ~60 km/h
    pub const MAX_SPEED_MS: f32 = 30.0;

    /// Maximum realistic vertical speed in m/s (15 m/s for aggressive maneuvers)
    pub const MAX_VERTICAL_SPEED_MS: f32 = 15.0;

    /// Maximum realistic acceleration in m/s² (20 m/s² = ~2G)
    /// Helps detect impossible acceleration changes
    pub const MAX_ACCEL_MS2: f32 = 20.0;

    /// Minimum samples to confirm spoofing (2 = require consecutive anomalies)
    pub const SPOOF_CONFIRM_COUNT: u8 = 2;

    /// Minimum normal samples to clear spoofing flag
    pub const NORMAL_CONFIRM_COUNT: u8 = 5;

    /// Maximum gap before resetting reference (ms)
    /// If no data for this long, don't trigger false positive on first packet
    pub const MAX_GAP_MS: u32 = 5000;

    /// EMA smoothing factor for velocity estimation (0.0-1.0)
    /// Lower = more smoothing, higher = more responsive
    pub const VELOCITY_ALPHA: f32 = 0.3;

    /// Degrees to meters conversion factor at equator
    /// 1 degree ≈ 111.32 km at equator
    pub const DEG_TO_M: f32 = 111320.0;
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
    /// Horizontal accuracy estimate in mm (from NAV-PVT hAcc)
    pub h_acc_mm: u32,
    /// Number of satellites used
    pub num_sv: u8,
    /// PDOP * 100 (from NAV-DOP)
    pub pdop: u16,
}

/// Velocity state for EMA filtering
#[derive(Clone, Copy, Default)]
struct VelocityState {
    /// Smoothed velocity north (m/s)
    vel_n: f32,
    /// Smoothed velocity east (m/s)
    vel_e: f32,
    /// Smoothed vertical velocity (m/s)
    vel_d: f32,
    /// Is velocity state initialized
    initialized: bool,
}

/// Spoofing detector state
pub struct SpoofDetector {
    /// Last known good position (before spoofing)
    last_good: Option<Position>,

    /// Previous position (for velocity calculation)
    prev: Option<Position>,

    /// EMA-filtered velocity state
    velocity: VelocityState,

    /// Previous speed for acceleration detection
    prev_speed_ms: f32,

    /// Currently detected as spoofed
    spoofed: bool,

    /// Consecutive anomaly counter
    anomaly_count: u8,

    /// Consecutive normal counter (for clearing spoof flag)
    normal_count: u8,

    /// Total anomalies detected (statistics)
    total_anomalies: u32,

    /// Circular buffer for pattern detection (last 10 positions)
    position_history: [Option<Position>; 10],
    history_idx: usize,
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
            velocity: VelocityState {
                vel_n: 0.0,
                vel_e: 0.0,
                vel_d: 0.0,
                initialized: false,
            },
            prev_speed_ms: 0.0,
            spoofed: false,
            anomaly_count: 0,
            normal_count: 0,
            total_anomalies: 0,
            position_history: [None; 10],
            history_idx: 0,
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
        // NOTE: 3D fix filter disabled - during dynamic spoofing fix_type may not
        // transition properly, causing missed detections. Altitude checks still work
        // because spoofer sends consistent fix_type with fake coordinates.
        //
        // if !pos.fix_type.has_3d_fix() {
        //     debug!("No 3D fix (type={}), skipping", pos.fix_type as u8);
        //     return AnalysisResult::Initializing;
        // }

        // First sample - initialize
        let prev = match self.prev {
            Some(p) => p,
            None => {
                self.prev = Some(pos);
                self.last_good = Some(pos);
                info!("Spoof detector initialized: lat={}, lon={}, alt={}mm",
                      pos.lat, pos.lon, pos.alt_mm);
                return AnalysisResult::Initializing;
            }
        };

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

        // Check for anomalies (including acceleration)
        let is_anomaly = analysis.is_teleport
            || analysis.is_speed_anomaly
            || analysis.is_alt_anomaly
            || analysis.is_accel_anomaly;

        // Check for signal quality anomalies (potential spoof indicator)
        let signal_anomaly = self.check_signal_quality(&pos);

        if is_anomaly || signal_anomaly {
            self.anomaly_count = self.anomaly_count.saturating_add(1);
            self.normal_count = 0;
            self.total_anomalies += 1;

            if analysis.is_teleport {
                warn!("TELEPORT detected: {}m jump", analysis.distance_m as i32);
            }
            if analysis.is_speed_anomaly {
                warn!("SPEED anomaly: {} m/s (max {})",
                      analysis.speed_ms as i32, thresholds::MAX_SPEED_MS as i32);
            }
            if analysis.is_alt_anomaly {
                warn!("ALTITUDE anomaly: {}m jump, {} m/s vertical",
                      analysis.alt_diff_m as i32, analysis.vertical_speed_ms as i32);
            }
            if analysis.is_accel_anomaly {
                warn!("ACCELERATION anomaly: {} m/s² (max {})",
                      analysis.accel_ms2 as i32, thresholds::MAX_ACCEL_MS2 as i32);
            }
            if signal_anomaly {
                warn!("SIGNAL quality anomaly: sats={} acc={}mm pdop={}",
                      pos.num_sv, pos.h_acc_mm, pos.pdop);
            }

            // Confirm spoofing after consecutive anomalies
            if self.anomaly_count >= thresholds::SPOOF_CONFIRM_COUNT && !self.spoofed {
                self.spoofed = true;
                error!("SPOOFING CONFIRMED - blocking transmission (count={})",
                       self.anomaly_count);
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

    /// Calculate distance between two points in meters using proper lat/lon scaling
    fn calc_distance(lat1: i32, lon1: i32, lat2: i32, lon2: i32) -> f32 {
        // Convert from 1e-7 degrees to radians for cos calculation
        // Average latitude for longitude scaling
        let avg_lat_rad = ((lat1 + lat2) / 2) as f32 * 1e-7 * core::f32::consts::PI / 180.0;
        let cos_lat = libm::cosf(avg_lat_rad);

        // Delta in 1e-7 degrees, convert to meters
        // 1 degree = 111320 meters at equator
        let dlat_m = (lat2 - lat1) as f32 * 1e-7 * thresholds::DEG_TO_M;
        let dlon_m = (lon2 - lon1) as f32 * 1e-7 * thresholds::DEG_TO_M * cos_lat;

        libm::sqrtf(dlat_m * dlat_m + dlon_m * dlon_m)
    }

    /// Calculate velocity components (North, East) in m/s
    fn calc_velocity(lat1: i32, lon1: i32, lat2: i32, lon2: i32, dt_s: f32) -> (f32, f32) {
        let avg_lat_rad = ((lat1 + lat2) / 2) as f32 * 1e-7 * core::f32::consts::PI / 180.0;
        let cos_lat = libm::cosf(avg_lat_rad);

        let dlat_m = (lat2 - lat1) as f32 * 1e-7 * thresholds::DEG_TO_M;
        let dlon_m = (lon2 - lon1) as f32 * 1e-7 * thresholds::DEG_TO_M * cos_lat;

        (dlat_m / dt_s, dlon_m / dt_s) // (vel_n, vel_e)
    }

    /// Analyze movement between two positions
    fn calculate_movement(&mut self, prev: &Position, curr: &Position, dt_s: f32) -> MovementAnalysis {
        let distance_m = Self::calc_distance(prev.lat, prev.lon, curr.lat, curr.lon);
        let alt_diff_m = (curr.alt_mm - prev.alt_mm) as f32 / 1000.0;
        let speed_ms = distance_m / dt_s;
        let vertical_speed_ms = libm::fabsf(alt_diff_m) / dt_s;

        // Calculate velocity components for EMA filtering
        let (vel_n, vel_e) = Self::calc_velocity(prev.lat, prev.lon, curr.lat, curr.lon, dt_s);
        let vel_d = alt_diff_m / dt_s;

        // Apply EMA filtering to velocity
        if self.velocity.initialized {
            let alpha = thresholds::VELOCITY_ALPHA;
            self.velocity.vel_n = alpha * vel_n + (1.0 - alpha) * self.velocity.vel_n;
            self.velocity.vel_e = alpha * vel_e + (1.0 - alpha) * self.velocity.vel_e;
            self.velocity.vel_d = alpha * vel_d + (1.0 - alpha) * self.velocity.vel_d;
        } else {
            self.velocity.vel_n = vel_n;
            self.velocity.vel_e = vel_e;
            self.velocity.vel_d = vel_d;
            self.velocity.initialized = true;
        }

        // Calculate acceleration (change in speed magnitude)
        let accel_ms2 = libm::fabsf(speed_ms - self.prev_speed_ms) / dt_s;
        self.prev_speed_ms = speed_ms;

        // Store in history buffer for pattern detection
        self.position_history[self.history_idx] = Some(*curr);
        self.history_idx = (self.history_idx + 1) % 10;

        MovementAnalysis {
            distance_m,
            alt_diff_m: libm::fabsf(alt_diff_m),
            speed_ms,
            vertical_speed_ms,
            accel_ms2,
            is_teleport: distance_m > thresholds::TELEPORT_M,
            is_speed_anomaly: speed_ms > thresholds::MAX_SPEED_MS,
            is_alt_anomaly: libm::fabsf(alt_diff_m) > thresholds::TELEPORT_ALT_M
                || vertical_speed_ms > thresholds::MAX_VERTICAL_SPEED_MS,
            is_accel_anomaly: accel_ms2 > thresholds::MAX_ACCEL_MS2,
        }
    }

    /// Force reset detector state (e.g., on mode change)
    pub fn reset(&mut self) {
        self.last_good = None;
        self.prev = None;
        self.velocity = VelocityState::default();
        self.prev_speed_ms = 0.0;
        self.spoofed = false;
        self.anomaly_count = 0;
        self.normal_count = 0;
        self.position_history = [None; 10];
        self.history_idx = 0;
        info!("Spoof detector reset");
    }

    /// Get last known good position (for comparison/display)
    pub fn last_good_position(&self) -> Option<Position> {
        self.last_good
    }

    /// Check signal quality for spoofing indicators
    /// Returns true if signal quality is suspiciously bad/good
    fn check_signal_quality(&self, pos: &Position) -> bool {
        // Suspicious: too few satellites with "perfect" accuracy
        // Real GPS rarely has <5 sats with <1m accuracy
        if pos.num_sv < 5 && pos.h_acc_mm < 1000 && pos.h_acc_mm > 0 {
            return true;
        }

        // Suspicious: way too many satellites (unlikely real scenario)
        if pos.num_sv > 30 {
            return true;
        }

        // Suspicious: perfect accuracy with high PDOP
        // PDOP > 5 means poor geometry, should have worse accuracy
        if pos.pdop > 500 && pos.h_acc_mm < 2000 && pos.h_acc_mm > 0 {
            return true;
        }

        false
    }

    /// Detect linear interpolation patterns (common in spoofers)
    /// Returns linearity score (0.0 = random, 1.0 = perfectly linear)
    #[allow(dead_code)]
    pub fn detect_linear_pattern(&self) -> f32 {
        // Need at least 5 positions in history
        let positions: heapless::Vec<Position, 10> = self.position_history
            .iter()
            .filter_map(|p| *p)
            .collect();

        if positions.len() < 5 {
            return 0.0;
        }

        // Calculate velocity variance
        // Linear interpolation has very low velocity variance
        let mut vel_n_sum = 0.0f32;
        let mut vel_e_sum = 0.0f32;
        let mut vel_n_sq_sum = 0.0f32;
        let mut vel_e_sq_sum = 0.0f32;
        let mut count = 0u32;

        for i in 1..positions.len() {
            let prev = &positions[i - 1];
            let curr = &positions[i];
            let dt_ms = curr.time_ms.wrapping_sub(prev.time_ms);

            if dt_ms > 0 && dt_ms < 1000 {
                let dt_s = dt_ms as f32 / 1000.0;
                let (vel_n, vel_e) = Self::calc_velocity(
                    prev.lat, prev.lon, curr.lat, curr.lon, dt_s
                );

                vel_n_sum += vel_n;
                vel_e_sum += vel_e;
                vel_n_sq_sum += vel_n * vel_n;
                vel_e_sq_sum += vel_e * vel_e;
                count += 1;
            }
        }

        if count < 3 {
            return 0.0;
        }

        let n = count as f32;
        let vel_n_mean = vel_n_sum / n;
        let vel_e_mean = vel_e_sum / n;
        let vel_n_var = (vel_n_sq_sum / n) - (vel_n_mean * vel_n_mean);
        let vel_e_var = (vel_e_sq_sum / n) - (vel_e_mean * vel_e_mean);

        // Low variance = linear pattern (suspicious)
        // Normalize to 0-1 range: 1.0 = perfectly linear
        let variance = vel_n_var + vel_e_var;
        let linearity = 1.0 / (1.0 + variance * 0.01);

        linearity
    }

    /// Get smoothed velocity (from EMA filter)
    pub fn smoothed_velocity(&self) -> Option<(f32, f32, f32)> {
        if self.velocity.initialized {
            Some((self.velocity.vel_n, self.velocity.vel_e, self.velocity.vel_d))
        } else {
            None
        }
    }
}

/// Movement analysis results
struct MovementAnalysis {
    distance_m: f32,
    alt_diff_m: f32,
    speed_ms: f32,
    vertical_speed_ms: f32,
    accel_ms2: f32,
    is_teleport: bool,
    is_speed_anomaly: bool,
    is_alt_anomaly: bool,
    is_accel_anomaly: bool,
}

impl Default for SpoofDetector {
    fn default() -> Self {
        Self::new()
    }
}
