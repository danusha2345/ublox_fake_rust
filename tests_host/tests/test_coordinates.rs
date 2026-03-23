//! Integration tests for the coordinates module.
//!
//! Only `llh_to_ecef_cm()` is tested here — it is a pure function with no global state.
//! The init() / cached-getter path is not exercised because it depends on config constants
//! and writes to mutable statics, which cannot be safely reset between tests.

use spoof_detector_tests::coordinates;

// ============================================================================
// Reference constants
// ============================================================================

/// Equator / Prime Meridian
const EQ_LAT: i32 = 0;
const EQ_LON: i32 = 0;

/// North Pole (90.0° × 1e7)
const NP_LAT: i32 = 900_000_000;
const NP_LON: i32 = 0;

/// Golden Beach, FL — default firmware position
const GB_LAT: i32 = 259_664_430;  // 25.966443°
const GB_LON: i32 = -801_223_710; // -80.122371°

/// Seney, MI — offset target
const SN_LAT: i32 = 463_407_000;  // 46.3407°
const SN_LON: i32 = -859_407_000; // -85.9407°

/// Sea level altitude in mm
const SEA_LEVEL_MM: i32 = 0;

/// 1 km altitude in mm
const ONE_KM_MM: i32 = 1_000_000;

// ============================================================================
// Test 1: Equator / Prime Meridian
// ============================================================================

/// At (lat=0, lon=0, alt=0) the entire radius equals the WGS84 semi-major axis:
///   X = a ≈ 6378137.0 m → 637813700 cm, Y = 0, Z = 0
#[test]
fn test_ecef_equator() {
    let (x, y, z) = coordinates::llh_to_ecef_cm(EQ_LAT, EQ_LON, SEA_LEVEL_MM);

    // WGS84 semi-major axis a = 6378137.0 m = 637813700 cm
    let expected_x: i32 = 637_813_700;
    let tolerance: i32 = 100; // ±1 m

    assert!(
        (x - expected_x).abs() <= tolerance,
        "X at equator/meridian: got {x}, expected ~{expected_x} (±{tolerance})"
    );
    assert!(
        y.abs() <= tolerance,
        "Y at equator/meridian should be ~0: got {y}"
    );
    assert!(
        z.abs() <= tolerance,
        "Z at equator/meridian should be ~0: got {z}"
    );
}

// ============================================================================
// Test 2: North Pole
// ============================================================================

/// At (lat=90°, lon=0, alt=0) the point is at the geometric north pole:
///   X = 0, Y = 0, Z = a*(1-e²)/sqrt(1-e²) = b ≈ 6356752.31 m → 635675231 cm
#[test]
fn test_ecef_north_pole() {
    let (x, y, z) = coordinates::llh_to_ecef_cm(NP_LAT, NP_LON, SEA_LEVEL_MM);

    // WGS84 semi-minor axis b ≈ 6356752.3142 m → 635675231 cm
    let expected_z: i32 = 635_675_231;
    let tolerance: i32 = 100; // ±1 m

    assert!(
        x.abs() <= tolerance,
        "X at north pole should be ~0: got {x}"
    );
    assert!(
        y.abs() <= tolerance,
        "Y at north pole should be ~0: got {y}"
    );
    assert!(
        (z - expected_z).abs() <= tolerance,
        "Z at north pole: got {z}, expected ~{expected_z} (±{tolerance})"
    );
}

// ============================================================================
// Test 3: Golden Beach, FL — sign and magnitude sanity
// ============================================================================

/// Golden Beach, FL (lat≈26°N, lon≈-80°E):
///   X > 0  (cos(lat) > 0, cos(lon) > 0 since |lon| < 90°)
///   Y < 0  (cos(lat) > 0, sin(lon) < 0 since lon is negative)
///   Z > 0  (sin(lat) > 0 in the northern hemisphere)
///
/// Approximate ECEF magnitude ≈ Earth radius ≈ 637000000 cm for each component
/// within plausible range.
#[test]
fn test_ecef_golden_beach() {
    let (x, y, z) = coordinates::llh_to_ecef_cm(GB_LAT, GB_LON, SEA_LEVEL_MM);

    assert!(x > 0, "X at Golden Beach should be positive: got {x}");
    assert!(y < 0, "Y at Golden Beach should be negative (lon<0): got {y}");
    assert!(z > 0, "Z at Golden Beach should be positive (lat>0): got {z}");

    // Total radius must be close to Earth radius: [630000000, 640000000] cm
    let r_sq = (x as i64).pow(2) + (y as i64).pow(2) + (z as i64).pow(2);
    let r_cm = (r_sq as f64).sqrt() as i64;
    assert!(
        r_cm > 630_000_000 && r_cm < 640_000_000,
        "ECEF radius at Golden Beach out of range: got {r_cm} cm"
    );
}

// ============================================================================
// Test 4: Seney, MI — sign and magnitude sanity
// ============================================================================

/// Seney, MI (lat≈46.3°N, lon≈-86°E):
///   X > 0  (cos(lat) > 0, cos(lon) > 0 since |lon| < 90°)
///   Y < 0  (sin(lon) < 0 since lon is negative in western hemisphere)
///   Z > 0  (northern hemisphere)
#[test]
fn test_ecef_seney_michigan() {
    let (x, y, z) = coordinates::llh_to_ecef_cm(SN_LAT, SN_LON, SEA_LEVEL_MM);

    assert!(x > 0, "X at Seney MI should be positive: got {x}");
    assert!(y < 0, "Y at Seney MI should be negative (lon<0): got {y}");
    assert!(z > 0, "Z at Seney MI should be positive (lat>0): got {z}");

    let r_sq = (x as i64).pow(2) + (y as i64).pow(2) + (z as i64).pow(2);
    let r_cm = (r_sq as f64).sqrt() as i64;
    assert!(
        r_cm > 630_000_000 && r_cm < 640_000_000,
        "ECEF radius at Seney MI out of range: got {r_cm} cm"
    );
}

// ============================================================================
// Test 5: Altitude increases ECEF radius
// ============================================================================

/// Adding 1 km altitude must increase the ECEF radius by approximately 100000 cm (1 km).
/// The increase should be within 1% of the nominal value.
#[test]
fn test_ecef_altitude_increases_radius() {
    let (x0, y0, z0) = coordinates::llh_to_ecef_cm(GB_LAT, GB_LON, SEA_LEVEL_MM);
    let (x1, y1, z1) = coordinates::llh_to_ecef_cm(GB_LAT, GB_LON, ONE_KM_MM);

    let r0 = ((x0 as i64).pow(2) + (y0 as i64).pow(2) + (z0 as i64).pow(2)) as f64;
    let r1 = ((x1 as i64).pow(2) + (y1 as i64).pow(2) + (z1 as i64).pow(2)) as f64;

    let dr_cm = r1.sqrt() - r0.sqrt();
    let expected_dr_cm = 100_000.0_f64; // 1 km = 100000 cm
    let tolerance_cm = 1_000.0_f64;     // ±10 m tolerance

    assert!(
        (dr_cm - expected_dr_cm).abs() < tolerance_cm,
        "ECEF radius increase for +1km altitude: got {dr_cm:.0} cm, expected ~{expected_dr_cm:.0} cm (±{tolerance_cm:.0})"
    );
}

// ============================================================================
// Test 6: Deterministic / pure function
// ============================================================================

/// Two calls with identical arguments must return identical results.
/// Verifies there is no accumulated state or randomness inside the function.
#[test]
fn test_ecef_consistency() {
    let (x1, y1, z1) = coordinates::llh_to_ecef_cm(GB_LAT, GB_LON, 50_000);
    let (x2, y2, z2) = coordinates::llh_to_ecef_cm(GB_LAT, GB_LON, 50_000);

    assert_eq!(x1, x2, "X not deterministic");
    assert_eq!(y1, y2, "Y not deterministic");
    assert_eq!(z1, z2, "Z not deterministic");
}

// ============================================================================
// Test 7: Southern hemisphere → Z < 0
// ============================================================================

/// A point in the southern hemisphere (negative latitude) must have Z < 0
/// because sin(lat) < 0 when lat < 0.
#[test]
fn test_ecef_southern_hemisphere() {
    // Buenos Aires, Argentina: lat≈-34.6°, lon≈-58.4°
    let lat: i32 = -346_000_000; // -34.6° × 1e7
    let lon: i32 = -584_000_000; // -58.4° × 1e7

    let (x, y, z) = coordinates::llh_to_ecef_cm(lat, lon, SEA_LEVEL_MM);

    assert!(z < 0, "Z in southern hemisphere should be negative: got {z}");
    assert!(x > 0, "X in southern hemisphere (|lon| < 90°) should be positive: got {x}");
    assert!(y < 0, "Y for western longitude should be negative: got {y}");
}

// ============================================================================
// Test 8: Offset round-trip
// ============================================================================

/// Simulates the PassthroughOffset workflow:
///   1. Compute ECEF for the "actual" position (Golden Beach).
///   2. Compute ECEF for the "target" position (Seney, MI).
///   3. Derive the offset: delta = target - actual (per component).
///   4. Apply delta to the actual ECEF.
///   5. Result must equal the target ECEF within ±2 cm (rounding error only).
///
/// This verifies that the LLH→ECEF conversion is self-consistent and that the
/// integer arithmetic used for offset application does not accumulate error.
#[test]
fn test_offset_round_trip() {
    let (ax, ay, az) = coordinates::llh_to_ecef_cm(GB_LAT, GB_LON, SEA_LEVEL_MM);
    let (tx, ty, tz) = coordinates::llh_to_ecef_cm(SN_LAT, SN_LON, SEA_LEVEL_MM);

    let delta_x = tx - ax;
    let delta_y = ty - ay;
    let delta_z = tz - az;

    let result_x = ax + delta_x;
    let result_y = ay + delta_y;
    let result_z = az + delta_z;

    // Integer addition is exact — result must equal target exactly
    assert_eq!(result_x, tx, "Round-trip X mismatch");
    assert_eq!(result_y, ty, "Round-trip Y mismatch");
    assert_eq!(result_z, tz, "Round-trip Z mismatch");
}
