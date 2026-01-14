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
    /// Teleportation threshold in meters (jump > 2km = spoof)
    /// Increased from 500m to reduce false positives during GPS re-acquisition
    /// Time-based detection catches smaller coordinate jumps via clock drift
    pub const TELEPORT_M: f32 = 2000.0;

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
    
    // ===== Time-based detection thresholds =====
    
    /// Maximum realistic GNSS time jump forward in seconds (30s)
    /// GPS receivers should maintain continuous time
    pub const MAX_TIME_JUMP_FORWARD_S: i64 = 30;
    
    /// Time jump backwards threshold (even 1s backwards is suspicious)
    /// Real GNSS time should never go backwards
    pub const TIME_BACKWARDS_THRESHOLD_S: i64 = 1;
    
    /// Tolerance for time projection match during recovery (±5s)
    /// If spoofed time jumps back within this range of projected real time, likely recovery
    pub const TIME_RECOVERY_TOLERANCE_S: i64 = 5;
    
    // ===== CNO (Carrier-to-Noise) detection thresholds =====
    // Currently disabled but kept for future use
    
    /// Minimum CNO standard deviation (dB-Hz)
    #[allow(dead_code)]
    pub const MIN_CNO_STDDEV: f32 = 3.0;
    
    /// High CNO threshold (dB-Hz)
    #[allow(dead_code)]
    pub const HIGH_CNO_THRESHOLD: u8 = 45;
    
    /// Minimum satellites needed for CNO variance analysis
    #[allow(dead_code)]
    pub const MIN_SATS_FOR_CNO_CHECK: u8 = 4;
    
    // ===== System clock drift detection thresholds =====
    
    /// Maximum allowed drift between system clock and GNSS time (seconds)
    /// If |system_time - gnss_time| > this, likely spoofing
    pub const MAX_CLOCK_DRIFT_S: i64 = 10;
    
    /// Calibration period - how long to wait before trusting system clock (ms)
    /// Need valid GNSS time for at least this long to calibrate
    pub const CLOCK_CALIBRATION_MS: u32 = 5000;

    /// Warmup samples after recovery to ignore coordinate jumps during GPS re-acquisition
    /// GPS needs time to stabilize after re-acquiring satellites
    pub const RECOVERY_WARMUP_COUNT: u8 = 10;  // ~2 seconds at 5 Hz
}

/// GNSS time from NAV-PVT (UTC time with GPS week reference)
#[derive(Clone, Copy, Debug)]
pub struct GnssTime {
    /// GPS time of week in milliseconds
    #[allow(dead_code)]
    pub itow_ms: u32,
    /// UTC year (e.g., 2026)
    pub year: u16,
    /// UTC month (1-12)
    pub month: u8,
    /// UTC day (1-31)
    pub day: u8,
    /// UTC hour (0-23)
    pub hour: u8,
    /// UTC minute (0-59)
    pub min: u8,
    /// UTC second (0-60, 60 is for leap second)
    pub sec: u8,
    /// System monotonic time when this GNSS time was captured (ms)
    pub system_time_ms: u32,
}

impl GnssTime {
    /// Convert GNSS time to Unix timestamp (seconds since 1970-01-01 UTC)
    /// Simplified calculation (ignores leap seconds for simplicity in embedded)
    #[allow(clippy::wrong_self_convention)]
    pub fn to_unix_timestamp(&self) -> i64 {
        // Days in each month (non-leap year)
        const DAYS_IN_MONTH: [i32; 12] = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
        
        // Calculate days since Unix epoch (1970-01-01)
        let mut days: i64 = 0;
        
        // Add full years
        for y in 1970..(self.year as i32) {
            days += if is_leap_year(y) { 366 } else { 365 };
        }
        
        // Add full months in current year
        for &month_days in DAYS_IN_MONTH.iter().take(self.month as usize - 1) {
            days += month_days as i64;
        }
        
        // Add leap day if current year is leap and past February
        if is_leap_year(self.year as i32) && self.month > 2 {
            days += 1;
        }
        
        // Add days in current month
        days += (self.day - 1) as i64;
        
        // Convert to seconds and add time of day
        days * 86400 + (self.hour as i64) * 3600 + (self.min as i64) * 60 + (self.sec as i64)
    }
    
    /// Project GNSS time forward using elapsed system time
    /// Returns projected Unix timestamp
    pub fn project(&self, current_system_ms: u32) -> i64 {
        let elapsed_ms = current_system_ms.wrapping_sub(self.system_time_ms);
        self.to_unix_timestamp() + (elapsed_ms as i64 / 1000)
    }
}

/// Check if year is a leap year
fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
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

#[allow(dead_code)]
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
#[derive(Clone, Default)]
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
    #[allow(dead_code)]
    pub fix_type: FixType,
    /// Horizontal accuracy estimate in mm (from NAV-PVT hAcc)
    #[allow(dead_code)]
    pub h_acc_mm: u32,
    /// Number of satellites used
    #[allow(dead_code)]
    pub num_sv: u8,
    /// PDOP * 100 (from NAV-DOP)
    #[allow(dead_code)]
    pub pdop: u16,
    /// GNSS time (optional, extracted from NAV-PVT)
    pub gnss_time: Option<GnssTime>,
    /// CNO values from satellites (up to 16 values, from NAV-SAT)
    /// Used for detecting uniform high CNO (spoof indicator)
    #[allow(dead_code)]
    pub cno_values: heapless::Vec<u8, 16>,
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
    
    /// Last known good GNSS time (before spoofing)
    last_good_gnss_time: Option<GnssTime>,

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
    
    // ===== System clock calibration =====
    
    /// System time (ms) when clock was calibrated with valid GNSS time
    calibrated_at_system_ms: u32,
    
    /// GNSS Unix timestamp at calibration moment
    calibrated_gnss_unix: i64,
    
    /// Whether system clock is calibrated
    clock_calibrated: bool,
    
    /// How long we've had valid GNSS time (ms) for calibration
    valid_gnss_duration_ms: u32,
    
    /// Warmup sample counter - ignore first N positions to avoid false positives
    /// on startup when GNSS time and coordinates are first established
    warmup_samples: u8,

    /// Recovery warmup counter - ignore coordinate anomalies after spoofing ends
    /// to allow GPS to stabilize during re-acquisition
    recovery_warmup_samples: u8,
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
    /// Warmup period - ignore first N positions to avoid false positives
    const WARMUP_SAMPLES: u8 = 10;
    
    pub const fn new() -> Self {
        Self {
            last_good: None,
            prev: None,
            last_good_gnss_time: None,
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
            position_history: [const { None }; 10],
            history_idx: 0,
            // System clock calibration
            calibrated_at_system_ms: 0,
            calibrated_gnss_unix: 0,
            clock_calibrated: false,
            valid_gnss_duration_ms: 0,
            // Warmup counter
            warmup_samples: 0,
            // Recovery warmup counter (starts at max to disable until first recovery)
            recovery_warmup_samples: thresholds::RECOVERY_WARMUP_COUNT,
        }
    }

    /// Check if currently in spoofed state
    #[allow(dead_code)]
    pub fn is_spoofed(&self) -> bool {
        self.spoofed
    }

    /// Get total anomaly count (for diagnostics)
    #[allow(dead_code)]
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

        // Ignore positions without valid 3D fix - prevents false teleport on first fix
        if !pos.fix_type.has_3d_fix() {
            return AnalysisResult::Initializing;
        }

        // First sample - initialize (only with valid 3D fix)
        let prev = match &self.prev {
            Some(p) => p.clone(),
            None => {
                self.prev = Some(pos.clone());
                self.last_good = Some(pos.clone());
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

        // NEW: Check GNSS time for anomalies (if available)
        let mut time_spoof = false;
        let mut time_recovery = false;
        let mut clock_drift_spoof = false;
        let mut clock_drift_recovery = false;
        
        if let Some(ref gnss_time) = pos.gnss_time {
            // Check time jumps
            let (is_spoof, is_recovery) = self.check_gnss_time(gnss_time, pos.time_ms);
            time_spoof = is_spoof;
            time_recovery = is_recovery;
            
            // Check system clock drift
            let (drift_spoof, drift_recovery) = self.check_system_clock_drift(gnss_time, pos.time_ms);
            clock_drift_spoof = drift_spoof;
            clock_drift_recovery = drift_recovery;
            
            if time_spoof {
                warn!("GNSS TIME anomaly detected");
            }
            if clock_drift_spoof {
                warn!("SYSTEM CLOCK DRIFT detected - GNSS time doesn't match system clock");
            }
            if time_recovery || clock_drift_recovery {
                info!("GNSS TIME recovery detected - time back to normal");
            }
        }
        
        // DISABLED: CNO anomaly check - too many false positives, keeping code for future use
        // let cno_anomaly = self.check_cno_anomaly(&pos);
        // if cno_anomaly {
        //     warn!("CNO ANOMALY detected - uniform high signal strength across satellites");
        // }

        // Warmup period - ignore anomalies during first N samples
        // This prevents false positives when GNSS first acquires fix
        let in_warmup = self.warmup_samples < Self::WARMUP_SAMPLES;
        if in_warmup {
            self.warmup_samples += 1;
            if self.warmup_samples == Self::WARMUP_SAMPLES {
                info!("Spoof detector warmup complete ({} samples) - time checks enabled", Self::WARMUP_SAMPLES);
            }
        }

        // Recovery warmup - ignore coordinate anomalies after spoofing ends
        // to allow GPS to stabilize during re-acquisition
        let in_recovery_warmup = self.recovery_warmup_samples < thresholds::RECOVERY_WARMUP_COUNT;
        if in_recovery_warmup && !self.spoofed {
            self.recovery_warmup_samples += 1;
            if self.recovery_warmup_samples == thresholds::RECOVERY_WARMUP_COUNT {
                info!("Recovery warmup complete - coordinate checks re-enabled");
            }
        }

        // Check for anomalies - SIMPLIFIED: only coordinate + time based checks
        //
        // WARMUP LOGIC:
        // - Coordinate-based checks (teleport, speed) work IMMEDIATELY
        //   because if drone "moves" from first position, it's likely spoofing
        // - Time-based checks (time_spoof, clock_drift) are DELAYED during warmup
        //   because GNSS time needs calibration first (5 seconds)
        // - During RECOVERY warmup, coordinate anomalies are ignored to prevent
        //   false positives from GPS re-acquisition jitter
        //
        // Disabled: is_alt_anomaly, is_accel_anomaly, cno_anomaly

        // Coordinate anomalies - disabled during recovery warmup
        let coord_anomaly = analysis.is_teleport || analysis.is_speed_anomaly;
        let coord_anomaly_effective = if in_recovery_warmup && !self.spoofed {
            false  // Ignore coordinate jumps during GPS re-acquisition
        } else {
            coord_anomaly
        };

        // Time anomalies - only after warmup (need calibration first)
        // Note: Time checks remain active during recovery warmup!
        let time_anomaly = if in_warmup {
            false  // Ignore time anomalies during startup warmup
        } else {
            time_spoof || clock_drift_spoof
        };

        let is_anomaly = coord_anomaly_effective || time_anomaly;

        // DEBUG: Детальное логирование состояния детектора
        debug!("SPOOF_DBG: spoofed={} anom_cnt={} norm_cnt={} warmup={}/{} rec_warmup={}/{}",
            self.spoofed as u8,
            self.anomaly_count,
            self.normal_count,
            self.warmup_samples,
            Self::WARMUP_SAMPLES,
            self.recovery_warmup_samples,
            thresholds::RECOVERY_WARMUP_COUNT
        );

        debug!("SPOOF_DBG: coord={} coord_eff={} time_sp={} time_rec={} drift_sp={} drift_rec={}",
            coord_anomaly as u8,
            coord_anomaly_effective as u8,
            time_spoof as u8,
            time_recovery as u8,
            clock_drift_spoof as u8,
            clock_drift_recovery as u8
        );

        debug!("SPOOF_DBG: dist={}m speed={}m/s teleport={} speed_anom={}",
            analysis.distance_m as i32,
            analysis.speed_ms as i32,
            analysis.is_teleport as u8,
            analysis.is_speed_anomaly as u8
        );

        // DEBUG: Log current position and last_good for comparison
        debug!("POS_DBG: curr lat={} lon={} alt={}mm",
            pos.lat, pos.lon, pos.alt_mm);
        if let Some(ref good) = self.last_good {
            let dist_from_good = Self::calc_distance(good.lat, good.lon, pos.lat, pos.lon);
            debug!("POS_DBG: good lat={} lon={} dist_from_good={}m",
                good.lat, good.lon, dist_from_good as i32);
        } else {
            debug!("POS_DBG: last_good=None");
        }

        // NEW: Prioritize time-based recovery (stronger signal than coordinates)
        if (time_recovery || clock_drift_recovery) && self.spoofed {
            self.spoofed = false;
            self.normal_count = 0;
            self.anomaly_count = 0;
            // DO NOT update last_good here - keep the original good position!
            // Start recovery warmup to ignore coordinate jumps during GPS re-acquisition
            self.recovery_warmup_samples = 0;
            info!("Spoofing cleared by GNSS time/clock recovery - starting recovery warmup");
            self.prev = Some(pos);
            return AnalysisResult::Normal;
        }

        if is_anomaly {
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
                if let Some(ref good) = self.last_good {
                    let dist_from_good = Self::calc_distance(
                        good.lat, good.lon, pos.lat, pos.lon
                    );

                    if dist_from_good < thresholds::TELEPORT_M {
                        self.spoofed = false;
                        self.last_good = Some(pos.clone());
                        self.recovery_warmup_samples = 0;  // Start recovery warmup
                        info!("Spoofing cleared - back to normal position, starting recovery warmup");
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
                self.last_good = Some(pos.clone());
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
        self.position_history[self.history_idx] = Some(curr.clone());
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
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.last_good = None;
        self.prev = None;
        self.last_good_gnss_time = None;
        self.velocity = VelocityState::default();
        self.prev_speed_ms = 0.0;
        self.spoofed = false;
        self.anomaly_count = 0;
        self.normal_count = 0;
        self.position_history = [const { None }; 10];
        self.history_idx = 0;
        // Reset system clock calibration
        self.calibrated_at_system_ms = 0;
        self.calibrated_gnss_unix = 0;
        self.clock_calibrated = false;
        self.valid_gnss_duration_ms = 0;
        // Reset warmup counter
        self.warmup_samples = 0;
        // Reset recovery warmup (start at max to disable until first recovery)
        self.recovery_warmup_samples = thresholds::RECOVERY_WARMUP_COUNT;
        info!("Spoof detector reset");
    }

    /// Get last known good position (for comparison/display)
    #[allow(dead_code)]
    pub fn last_good_position(&self) -> Option<Position> {
        self.last_good.clone()
    }

    /// Check GNSS time for anomalies (spoofing and recovery detection)
    /// Returns (is_time_spoof, is_time_recovery)
    fn check_gnss_time(&mut self, curr_time: &GnssTime, system_ms: u32) -> (bool, bool) {
        let last_good = match self.last_good_gnss_time {
            Some(t) => t,
            None => {
                // First time sample - store as reference
                self.last_good_gnss_time = Some(*curr_time);
                return (false, false);
            }
        };
        
        let curr_unix = curr_time.to_unix_timestamp();
        let last_unix = last_good.to_unix_timestamp();
        let time_diff = curr_unix - last_unix;
        
        // Check 1: Time jumped backwards (strong spoof indicator)
        // Real GPS time should never go backwards
        let time_backwards = time_diff < -thresholds::TIME_BACKWARDS_THRESHOLD_S;
        
        // Check 2: Time jumped forward too much (unrealistic)
        // GPS should maintain continuous time within reasonable bounds
        let time_forward_jump = time_diff > thresholds::MAX_TIME_JUMP_FORWARD_S;
        
        let is_spoof = time_backwards || time_forward_jump;
        
        // Check 3: If currently spoofed, check for recovery
        // If GNSS time jumps back close to projected real time, likely recovered
        let (is_recovery, diff_from_real) = if self.spoofed {
            // Project what real time should be now based on last good time
            let projected_real = last_good.project(system_ms);
            let diff = (curr_unix - projected_real).abs();

            // If current GNSS time is close to projected real time, likely recovered
            (diff <= thresholds::TIME_RECOVERY_TOLERANCE_S, diff)
        } else {
            (false, 0)
        };

        // DEBUG: Log time check details
        debug!("TIME_DBG: curr={} last={} diff={} diff_from_proj={} spoof={} rec={}",
            curr_unix, last_unix, time_diff, diff_from_real, is_spoof as u8, is_recovery as u8);

        // Update last good GNSS time if not spoofed
        if !is_spoof && !self.spoofed {
            self.last_good_gnss_time = Some(*curr_time);
        }

        (is_spoof, is_recovery)
    }

    /// Check CNO (Carrier-to-Noise) values for spoofing indicators
    /// Spoofers often have uniform high CNO across all satellites
    /// Returns true if CNO pattern is suspiciously uniform
    /// NOTE: Currently disabled in analyze() but kept for future use
    #[allow(dead_code)]
    fn check_cno_anomaly(&self, pos: &Position) -> bool {
        let cno_count = pos.cno_values.len();
        
        // Need enough satellites for meaningful analysis
        if cno_count < thresholds::MIN_SATS_FOR_CNO_CHECK as usize {
            return false;
        }
        
        // Calculate mean CNO
        let sum: u32 = pos.cno_values.iter().map(|&v| v as u32).sum();
        let mean = sum as f32 / cno_count as f32;
        
        // Calculate standard deviation
        let variance: f32 = pos.cno_values.iter()
            .map(|&v| {
                let diff = v as f32 - mean;
                diff * diff
            })
            .sum::<f32>() / cno_count as f32;
        let stddev = libm::sqrtf(variance);
        
        // Check 1: All satellites have suspiciously high CNO (>45 dB-Hz)
        let all_high = pos.cno_values.iter().all(|&v| v > thresholds::HIGH_CNO_THRESHOLD);
        
        // Check 2: Very low variance with high mean (uniform high signal)
        // Real satellites have varying signal strengths due to atmosphere, multipath, etc.
        let uniform_high = stddev < thresholds::MIN_CNO_STDDEV && mean > 40.0;
        
        if all_high || uniform_high {
            return true;
        }
        
        false
    }
    
    /// Check and update system clock calibration
    /// Uses GNSS time to calibrate internal clock, then detects drift during spoofing
    /// Returns (is_drift_spoof, is_drift_recovery)
    fn check_system_clock_drift(&mut self, gnss_time: &GnssTime, system_ms: u32) -> (bool, bool) {
        let gnss_unix = gnss_time.to_unix_timestamp();
        
        // Phase 1: Calibration
        // Need valid GNSS time for at least CLOCK_CALIBRATION_MS before trusting
        if !self.clock_calibrated {
            self.valid_gnss_duration_ms = self.valid_gnss_duration_ms.saturating_add(200); // ~5 Hz
            
            if self.valid_gnss_duration_ms >= thresholds::CLOCK_CALIBRATION_MS {
                // Calibrate system clock
                self.calibrated_at_system_ms = system_ms;
                self.calibrated_gnss_unix = gnss_unix;
                self.clock_calibrated = true;
                info!("System clock calibrated with GNSS time");
            }
            return (false, false);
        }
        
        // Phase 2: Detection
        // Calculate expected GNSS time based on system clock elapsed time
        let elapsed_system_ms = system_ms.wrapping_sub(self.calibrated_at_system_ms);
        let expected_gnss_unix = self.calibrated_gnss_unix + (elapsed_system_ms as i64 / 1000);
        
        // Calculate drift
        let drift_s = (gnss_unix - expected_gnss_unix).abs();

        // DEBUG: Log clock drift check details
        debug!("DRIFT_DBG: gnss={} expected={} drift={}s calib={} spoofed={}",
            gnss_unix, expected_gnss_unix, drift_s, self.clock_calibrated as u8, self.spoofed as u8);

        // Check for spoofing: large drift indicates fake GNSS time
        let is_drift_spoof = drift_s > thresholds::MAX_CLOCK_DRIFT_S;
        
        // Check for recovery: drift back to normal
        let is_drift_recovery = if self.spoofed {
            drift_s <= 3 // Within 3 seconds of expected time = recovered
        } else {
            false
        };
        
        // If not spoofed, update calibration (keeps clock in sync)
        if !is_drift_spoof && !self.spoofed {
            self.calibrated_at_system_ms = system_ms;
            self.calibrated_gnss_unix = gnss_unix;
        }
        
        (is_drift_spoof, is_drift_recovery)
    }

    /// Detect linear interpolation patterns (common in spoofers)
    /// Returns linearity score (0.0 = random, 1.0 = perfectly linear)
    #[allow(dead_code)]
    pub fn detect_linear_pattern(&self) -> f32 {
        // Need at least 5 positions in history
        let positions: heapless::Vec<Position, 10> = self.position_history
            .iter()
            .filter_map(|p| p.clone())
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
        

        1.0 / (1.0 + variance * 0.01)
    }

    /// Get smoothed velocity (from EMA filter)
    #[allow(dead_code)]
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
