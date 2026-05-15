//! Coordinate conversion utilities (LLH <-> ECEF)
//!
//! Uses WGS84 ellipsoid parameters for geodetic calculations.
//! Default position and offset target are computed once at startup
//! and cached in atomics (lock-free reads, no `unsafe`).

use core::sync::atomic::{AtomicBool, Ordering};
use libm::{sin, cos, sqrt};
use portable_atomic::AtomicI32;

/// WGS84 ellipsoid parameters
const WGS84_A: f64 = 6378137.0;           // Semi-major axis (m)
const WGS84_E2: f64 = 0.00669437999014;   // Eccentricity squared

// Default position cache (Emulation mode / LAST_GOOD seed)
static CACHED_LAT_1E7: AtomicI32 = AtomicI32::new(0);
static CACHED_LON_1E7: AtomicI32 = AtomicI32::new(0);
static CACHED_ALT_MM: AtomicI32 = AtomicI32::new(0);
static CACHED_ECEF_X_CM: AtomicI32 = AtomicI32::new(0);
static CACHED_ECEF_Y_CM: AtomicI32 = AtomicI32::new(0);
static CACHED_ECEF_Z_CM: AtomicI32 = AtomicI32::new(0);

// Offset target cache (PassthroughOffset modes)
static OFFSET_LAT_1E7: AtomicI32 = AtomicI32::new(0);
static OFFSET_LON_1E7: AtomicI32 = AtomicI32::new(0);
static OFFSET_ALT_MM: AtomicI32 = AtomicI32::new(0);

static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Initialize cached coordinates from config (call once at startup)
pub fn init() {
    if INITIALIZED.swap(true, Ordering::SeqCst) {
        return; // Already initialized
    }

    // === Default position ===
    {
        use crate::config::default_position as pos;

        let lat_rad = pos::LATITUDE.to_radians();
        let lon_rad = pos::LONGITUDE.to_radians();

        let sin_lat = sin(lat_rad);
        let cos_lat = cos(lat_rad);
        let sin_lon = sin(lon_rad);
        let cos_lon = cos(lon_rad);

        // Prime vertical radius of curvature
        let n = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
        let alt_m = pos::ALTITUDE_M as f64;

        let x_m = (n + alt_m) * cos_lat * cos_lon;
        let y_m = (n + alt_m) * cos_lat * sin_lon;
        let z_m = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat;

        CACHED_LAT_1E7.store((pos::LATITUDE * 1e7) as i32, Ordering::Release);
        CACHED_LON_1E7.store((pos::LONGITUDE * 1e7) as i32, Ordering::Release);
        CACHED_ALT_MM.store(pos::ALTITUDE_M * 1000, Ordering::Release);
        CACHED_ECEF_X_CM.store((x_m * 100.0) as i32, Ordering::Release);
        CACHED_ECEF_Y_CM.store((y_m * 100.0) as i32, Ordering::Release);
        CACHED_ECEF_Z_CM.store((z_m * 100.0) as i32, Ordering::Release);
    }

    // === Offset target (Seney, Michigan) ===
    // ECEF is recomputed per-frame from offset LLH via `llh_to_ecef_cm()` for
    // geometric consistency, so we only need to cache the LLH triple.
    {
        use crate::config::offset_target as ot;

        OFFSET_LAT_1E7.store((ot::LATITUDE * 1e7) as i32, Ordering::Release);
        OFFSET_LON_1E7.store((ot::LONGITUDE * 1e7) as i32, Ordering::Release);
        OFFSET_ALT_MM.store(ot::ALTITUDE_M * 1000, Ordering::Release);
    }
}

/// Get latitude in UBX format (deg * 1e-7)
#[inline]
pub fn lat_1e7() -> i32 {
    CACHED_LAT_1E7.load(Ordering::Acquire)
}

/// Get longitude in UBX format (deg * 1e-7)
#[inline]
pub fn lon_1e7() -> i32 {
    CACHED_LON_1E7.load(Ordering::Acquire)
}

/// Get altitude in mm
#[inline]
pub fn alt_mm() -> i32 {
    CACHED_ALT_MM.load(Ordering::Acquire)
}

/// Get ECEF X in cm
#[inline]
pub fn ecef_x_cm() -> i32 {
    CACHED_ECEF_X_CM.load(Ordering::Acquire)
}

/// Get ECEF Y in cm
#[inline]
pub fn ecef_y_cm() -> i32 {
    CACHED_ECEF_Y_CM.load(Ordering::Acquire)
}

/// Get ECEF Z in cm
#[inline]
pub fn ecef_z_cm() -> i32 {
    CACHED_ECEF_Z_CM.load(Ordering::Acquire)
}

// === Offset target getters (for PassthroughOffset mode) ===

#[inline]
pub fn offset_lat_1e7() -> i32 { OFFSET_LAT_1E7.load(Ordering::Acquire) }
#[inline]
pub fn offset_lon_1e7() -> i32 { OFFSET_LON_1E7.load(Ordering::Acquire) }
#[inline]
pub fn offset_alt_mm() -> i32 { OFFSET_ALT_MM.load(Ordering::Acquire) }

/// Runtime LLH → ECEF conversion (for spoof detection LAST_GOOD coordinates and per-frame offset)
/// Uses f64 trig (software-emulated on Cortex-M33, ~50µs per call — acceptable for rare spoof events)
pub fn llh_to_ecef_cm(lat_1e7: i32, lon_1e7: i32, alt_mm: i32) -> (i32, i32, i32) {
    let lat_rad = (lat_1e7 as f64 * 1e-7_f64).to_radians();
    let lon_rad = (lon_1e7 as f64 * 1e-7_f64).to_radians();

    let sin_lat = sin(lat_rad);
    let cos_lat = cos(lat_rad);
    let sin_lon = sin(lon_rad);
    let cos_lon = cos(lon_rad);

    let n = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    let alt_m = alt_mm as f64 / 1000.0;

    let x_cm = ((n + alt_m) * cos_lat * cos_lon * 100.0) as i32;
    let y_cm = ((n + alt_m) * cos_lat * sin_lon * 100.0) as i32;
    let z_cm = ((n * (1.0 - WGS84_E2) + alt_m) * sin_lat * 100.0) as i32;

    (x_cm, y_cm, z_cm)
}
