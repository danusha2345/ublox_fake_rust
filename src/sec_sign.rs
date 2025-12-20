//! SEC-SIGN cryptographic signature implementation
//!
//! Implements ECDSA SECP192R1 signature for u-blox authentication
//! Pure Rust implementation using p192 primitives

use sha2::{Digest, Sha256};
use p192::{ProjectivePoint, Scalar};
use p192::elliptic_curve::group::GroupEncoding;
use p192::elliptic_curve::Field;
use subtle::CtOption;
use hmac::{Hmac, Mac};

type HmacSha256 = Hmac<Sha256>;

/// SEC-SIGN private key (SECP192R1 - 24 bytes)
/// WARNING: Hardcoded for development. Production should use secure storage.
pub const PRIVATE_KEY: [u8; 24] = [
    0xea, 0xa5, 0xc0, 0x11, 0x1e, 0x18, 0xdb, 0xd1,
    0x7a, 0xdb, 0x3d, 0xc9, 0x39, 0x4b, 0xfb, 0x45,
    0x1f, 0x9d, 0x5e, 0x83, 0xf9, 0x38, 0x22, 0xc7,
];

/// Default session ID (zeros for M10 compatibility, same as C version)
pub const DEFAULT_SESSION_ID: [u8; 24] = [0u8; 24];

/// SEC-SIGN accumulator for hashing transmitted messages
pub struct SecSignAccumulator {
    hasher: Sha256,
    packet_count: u16,
}

impl SecSignAccumulator {
    pub fn new() -> Self {
        Self {
            hasher: Sha256::new(),
            packet_count: 0,
        }
    }

    /// Const constructor for static initialization (empty, must call reset before use)
    pub const fn new_const() -> Self {
        Self {
            hasher: unsafe { core::mem::zeroed() },
            packet_count: 0,
        }
    }

    /// Initialize or reset the accumulator
    pub fn reset(&mut self) {
        self.hasher = Sha256::new();
        self.packet_count = 0;
    }

    /// Accumulate a transmitted UBX message
    pub fn accumulate(&mut self, data: &[u8]) {
        self.hasher.update(data);
        self.packet_count = self.packet_count.wrapping_add(1);
    }

    /// Get current packet count
    pub fn packet_count(&self) -> u16 {
        self.packet_count
    }

    /// Finalize hash and get SHA256 result (non-destructive)
    pub fn finalize_hash(&self) -> [u8; 32] {
        let hasher_copy = self.hasher.clone();
        let result = hasher_copy.finalize();
        let mut hash = [0u8; 32];
        hash.copy_from_slice(&result);
        hash
    }

    /// Finalize hash, reset accumulator, return hash
    pub fn finalize_and_reset(&mut self) -> ([u8; 32], u16) {
        let hash = self.finalize_hash();
        let count = self.packet_count;
        self.reset();
        (hash, count)
    }
}

impl Default for SecSignAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the z value for ECDSA signing
/// z = fold(SHA256(sha256_field || session_id))
pub fn compute_z(sha256_field: &[u8; 32], session_id: &[u8; 24]) -> [u8; 24] {
    // Concatenate sha256_field and session_id
    let mut to_sign = [0u8; 56];
    to_sign[..32].copy_from_slice(sha256_field);
    to_sign[32..].copy_from_slice(session_id);

    // Hash the concatenation
    let mut hasher = Sha256::new();
    hasher.update(&to_sign);
    let final_hash = hasher.finalize();

    // Fold 32 bytes to 24 bytes: XOR bytes 0-7 with bytes 24-31
    let mut z = [0u8; 24];
    z.copy_from_slice(&final_hash[..24]);
    for i in 0..8 {
        z[i] ^= final_hash[24 + i];
    }

    z
}

/// SEC-SIGN signature result
#[derive(Clone, Default)]
pub struct Signature {
    pub r: [u8; 24],
    pub s: [u8; 24],
}

/// Generate deterministic k using RFC6979 (simplified)
/// k = HMAC-SHA256(private_key || z) mod n
fn generate_k(private_key: &[u8; 24], z: &[u8; 24]) -> Scalar {
    // Simplified RFC6979: k = HMAC(key=d, data=z)
    let mut mac = HmacSha256::new_from_slice(private_key)
        .expect("HMAC key length is valid");
    mac.update(z);

    // Add counter for retry mechanism
    let mut counter = 0u8;
    loop {
        let mut mac_clone = mac.clone();
        mac_clone.update(&[counter]);
        let result = mac_clone.finalize().into_bytes();

        // Take first 24 bytes and try to create valid scalar
        let mut k_bytes = [0u8; 24];
        k_bytes.copy_from_slice(&result[..24]);

        // Convert to Scalar (reduce mod n)
        let k_opt: CtOption<Scalar> = Scalar::from_bytes(&k_bytes.into());
        if bool::from(k_opt.is_some()) {
            let k = k_opt.unwrap();
            // Check k != 0
            if bool::from(!k.is_zero()) {
                return k;
            }
        }

        counter += 1;
        if counter > 100 {
            // Fallback - shouldn't happen
            return Scalar::ONE;
        }
    }
}

impl Signature {
    /// Sign the z value using ECDSA SECP192R1
    /// Pure Rust implementation using p192 primitives
    pub fn sign(z: &[u8; 24], private_key: &[u8; 24]) -> Option<Self> {
        // Convert private key to Scalar
        let d = ct_option_to_option(Scalar::from_bytes(&(*private_key).into()))?;

        // Convert z to Scalar (message hash)
        let z_scalar = ct_option_to_option(Scalar::from_bytes(&(*z).into()))?;

        // Generate deterministic k (RFC6979 simplified)
        let k = generate_k(private_key, z);

        // Compute R = k * G
        let r_point = ProjectivePoint::GENERATOR * k;

        // Get x coordinate of R as bytes
        let r_affine = r_point.to_affine();
        let r_bytes = r_affine.to_bytes();

        // r = x mod n (take first 24 bytes as scalar)
        // SEC1 encoded point: 0x04 || x || y for uncompressed
        // or 0x02/0x03 || x for compressed
        let x_bytes: [u8; 24] = if r_bytes[0] == 0x04 {
            // Uncompressed: skip tag, take x (24 bytes)
            r_bytes[1..25].try_into().ok()?
        } else {
            // Compressed: skip tag, take x
            r_bytes[1..25].try_into().ok()?
        };

        let r = ct_option_to_option(Scalar::from_bytes(&x_bytes.into()))?;

        // Check r != 0
        if bool::from(r.is_zero()) {
            return None;
        }

        // Compute s = k^-1 * (z + r * d) mod n
        let k_inv = ct_option_to_option(k.invert())?;
        let rd = r * d;
        let z_plus_rd = z_scalar + rd;
        let s = k_inv * z_plus_rd;

        // Check s != 0
        if bool::from(s.is_zero()) {
            return None;
        }

        // Convert r and s to bytes
        let mut sig = Self::default();
        sig.r.copy_from_slice(&r.to_bytes());
        sig.s.copy_from_slice(&s.to_bytes());

        Some(sig)
    }
}

/// RNG initialization placeholder (not needed for deterministic signing)
pub fn init_rng() {
    // No-op: using deterministic RFC6979 k generation
}

/// Data passed from Core0 to Core1 for SEC-SIGN computation
#[derive(Clone)]
pub struct SecSignRequest {
    pub sha256_hash: [u8; 32],
    pub session_id: [u8; 24],
    pub packet_count: u16,
}

/// Result from Core1 SEC-SIGN computation
#[derive(Clone)]
pub struct SecSignResult {
    pub sha256_hash: [u8; 32],
    pub session_id: [u8; 24],
    pub signature: Signature,
    pub packet_count: u16,
}

/// Helper to convert CtOption to Option
fn ct_option_to_option<T>(ct: CtOption<T>) -> Option<T> {
    if bool::from(ct.is_some()) {
        Some(ct.unwrap())
    } else {
        None
    }
}
