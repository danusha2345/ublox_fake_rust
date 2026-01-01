//! Coordinate conversion utilities (LLH <-> ECEF)
//!
//! Uses WGS84 ellipsoid parameters for geodetic calculations.
//! Coordinates are computed once at startup and cached in static variables.

use core::sync::atomic::{AtomicBool, Ordering};
use libm::{sin, cos, sqrt};

/// WGS84 ellipsoid parameters
const WGS84_A: f64 = 6378137.0;           // Semi-major axis (m)
const WGS84_E2: f64 = 0.00669437999014;   // Eccentricity squared

/// Cached coordinate values (computed once at startup)
struct CachedCoords {
    lat_1e7: i32,
    lon_1e7: i32,
    alt_mm: i32,
    ecef_x_cm: i32,
    ecef_y_cm: i32,
    ecef_z_cm: i32,
}

static mut CACHED: CachedCoords = CachedCoords {
    lat_1e7: 0,
    lon_1e7: 0,
    alt_mm: 0,
    ecef_x_cm: 0,
    ecef_y_cm: 0,
    ecef_z_cm: 0,
};

static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Initialize cached coordinates from config (call once at startup)
pub fn init() {
    if INITIALIZED.swap(true, Ordering::SeqCst) {
        return; // Already initialized
    }

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

    // ECEF in meters -> centimeters
    let x_m = (n + alt_m) * cos_lat * cos_lon;
    let y_m = (n + alt_m) * cos_lat * sin_lon;
    let z_m = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat;

    // Safety: single-threaded init before any reads
    unsafe {
        CACHED.lat_1e7 = (pos::LATITUDE * 1e7) as i32;
        CACHED.lon_1e7 = (pos::LONGITUDE * 1e7) as i32;
        CACHED.alt_mm = pos::ALTITUDE_M * 1000;
        CACHED.ecef_x_cm = (x_m * 100.0) as i32;
        CACHED.ecef_y_cm = (y_m * 100.0) as i32;
        CACHED.ecef_z_cm = (z_m * 100.0) as i32;
    }
}

/// Get latitude in UBX format (deg * 1e-7)
#[inline]
pub fn lat_1e7() -> i32 {
    unsafe { CACHED.lat_1e7 }
}

/// Get longitude in UBX format (deg * 1e-7)
#[inline]
pub fn lon_1e7() -> i32 {
    unsafe { CACHED.lon_1e7 }
}

/// Get altitude in mm
#[inline]
pub fn alt_mm() -> i32 {
    unsafe { CACHED.alt_mm }
}

/// Get ECEF X in cm
#[inline]
pub fn ecef_x_cm() -> i32 {
    unsafe { CACHED.ecef_x_cm }
}

/// Get ECEF Y in cm
#[inline]
pub fn ecef_y_cm() -> i32 {
    unsafe { CACHED.ecef_y_cm }
}

/// Get ECEF Z in cm
#[inline]
pub fn ecef_z_cm() -> i32 {
    unsafe { CACHED.ecef_z_cm }
}
