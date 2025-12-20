//! SEC-SIGN cryptographic signature implementation
//!
//! Implements ECDSA SECP192R1 signature for u-blox authentication

use sha2::{Digest, Sha256};

/// SEC-SIGN private key (SECP192R1 - 24 bytes)
pub const PRIVATE_KEY: [u8; 24] = [
    0xea, 0xa5, 0xc0, 0x11, 0x1e, 0x18, 0xdb, 0xd1,
    0x7a, 0xdb, 0x3d, 0xc9, 0x39, 0x4b, 0xfb, 0x45,
    0x1f, 0x9d, 0x5e, 0x83, 0xf9, 0x38, 0x22, 0xc7,
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

    /// Reset the accumulator
    pub fn reset(&mut self) {
        self.hasher = Sha256::new();
        self.packet_count = 0;
    }

    /// Accumulate a transmitted UBX message
    pub fn accumulate(&mut self, data: &[u8]) {
        self.hasher.update(data);
        self.packet_count += 1;
    }

    /// Get current packet count
    pub fn packet_count(&self) -> u16 {
        self.packet_count
    }

    /// Finalize hash and get SHA256 result
    pub fn finalize_hash(&self) -> [u8; 32] {
        let hasher_copy = self.hasher.clone();
        let result = hasher_copy.finalize();
        let mut hash = [0u8; 32];
        hash.copy_from_slice(&result);
        hash
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

// Note: Actual ECDSA signing requires the p192 crate which may need
// additional setup for no_std. For now, this is a placeholder structure.
// The actual signing would be done using:
//
// use p192::ecdsa::{SigningKey, signature::Signer};
//
// pub fn sign(z: &[u8; 24], private_key: &[u8; 24]) -> Option<Signature> {
//     let signing_key = SigningKey::from_bytes(private_key.into()).ok()?;
//     let signature: p192::ecdsa::Signature = signing_key.sign(z);
//     let (r, s) = signature.split_bytes();
//     Some(Signature {
//         r: r.into(),
//         s: s.into(),
//     })
// }
