//! SEC-SIGN cryptographic signature implementation
//!
//! Implements ECDSA SECP192R1 signature for u-blox authentication

use sha2::{Digest, Sha256};

/// SEC-SIGN private key (SECP192R1 - 24 bytes)
/// WARNING: Hardcoded for development. Production should use secure storage.
pub const PRIVATE_KEY: [u8; 24] = [
    0xea, 0xa5, 0xc0, 0x11, 0x1e, 0x18, 0xdb, 0xd1,
    0x7a, 0xdb, 0x3d, 0xc9, 0x39, 0x4b, 0xfb, 0x45,
    0x1f, 0x9d, 0x5e, 0x83, 0xf9, 0x38, 0x22, 0xc7,
];

/// Default session ID (should be random in production)
pub const DEFAULT_SESSION_ID: [u8; 24] = [
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
];

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
        // This creates an uninitialized state - MUST call reset() before use
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

impl Signature {
    /// Sign the z value using ECDSA SECP192R1
    ///
    /// TODO: p192 crate v0.13 doesn't implement SignPrimitive trait.
    /// For now, generates a deterministic placeholder signature.
    /// Real implementation needs p192 with full ecdsa support or a different crate.
    pub fn sign(z: &[u8; 24], private_key: &[u8; 24]) -> Option<Self> {
        // Placeholder: XOR z with private key to generate deterministic r and s
        // This is NOT cryptographically secure, just for protocol testing
        let mut sig = Self::default();

        // Generate r from first part
        for i in 0..24 {
            sig.r[i] = z[i] ^ private_key[i];
        }

        // Generate s from hashed combination
        let mut hasher = Sha256::new();
        hasher.update(z);
        hasher.update(private_key);
        hasher.update(&sig.r);
        let hash = hasher.finalize();
        sig.s.copy_from_slice(&hash[..24]);

        Some(sig)
    }
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
