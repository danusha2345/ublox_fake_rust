//! Flash storage for persistent mode and firmware version

use defmt::*;
use embassy_rp::flash::{Async, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

/// Magic value to identify valid flash data
const FLASH_MAGIC: u32 = 0xDEADBEEF;

/// Magic value for firmware version data
const VERSION_MAGIC: u32 = 0x56455253; // "VERS" in ASCII

/// Flash offset for mode data - second-to-last sector
/// Для 4MB: 0x3FE000 (sector 1022 of 1024)
/// Для 2MB: 0x1FE000 (sector 510 of 512)
const FLASH_OFFSET: u32 = (crate::config::FLASH_SIZE_BYTES as u32) - (2 * ERASE_SIZE as u32);

/// Flash offset for version data - third-to-last sector
/// Для 4MB: 0x3FD000 (sector 1021 of 1024)
/// Для 2MB: 0x1FD000 (sector 509 of 512)
const VERSION_FLASH_OFFSET: u32 = (crate::config::FLASH_SIZE_BYTES as u32) - (3 * ERASE_SIZE as u32);

/// Mode data stored in flash
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ModeData {
    pub magic: u32,
    pub mode: u8,  // 0 = emulation, 1 = passthrough
    pub _reserved: [u8; 3],
}

impl Default for ModeData {
    fn default() -> Self {
        Self {
            magic: FLASH_MAGIC,
            mode: 0,
            _reserved: [0; 3],
        }
    }
}

/// Save mode to flash. Returns true on success, false on error.
pub async fn save_mode(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>, mode: u8) -> bool {
    let mut data = [0u8; ERASE_SIZE];

    // Prepare data
    let mode_data = ModeData {
        magic: FLASH_MAGIC,
        mode,
        _reserved: [0; 3],
    };

    // Copy to buffer
    unsafe {
        let ptr = &mode_data as *const ModeData as *const u8;
        core::ptr::copy_nonoverlapping(ptr, data.as_mut_ptr(), core::mem::size_of::<ModeData>());
    }

    // Erase sector
    if let Err(e) = flash.blocking_erase(FLASH_OFFSET, FLASH_OFFSET + ERASE_SIZE as u32) {
        error!("Flash erase failed: {:?}", e);
        return false;
    }

    // Write data
    if let Err(e) = flash.blocking_write(FLASH_OFFSET, &data[..256]) {
        error!("Flash write failed: {:?}", e);
        return false;
    }

    info!("Mode {} saved to flash successfully", mode);
    true
}

/// Load mode from flash, returns None if no valid data
#[allow(dead_code)]
pub fn load_mode(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>) -> Option<u8> {
    let mut buf = [0u8; 8];

    if flash.blocking_read(FLASH_OFFSET, &mut buf).is_ok() {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic == FLASH_MAGIC {
            return Some(buf[4]);
        }
    }

    None
}

/// Max firmware version string length stored in flash
const VERSION_MAX_LEN: usize = 32;

/// Version data layout in flash:
///   [0..4]   magic (VERSION_MAGIC, LE)
///   [4]      string length
///   [5..37]  version string (up to 32 bytes, null-padded)
/// Total: 37 bytes, padded to 256 for flash write alignment

/// Save firmware version string to flash.
/// Only writes if version differs from what's already stored — avoids unnecessary flash wear.
pub fn save_version(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>, version: &str) -> bool {
    // Check if already stored with same version
    if let Some(stored) = load_version(flash) {
        if stored.as_str() == version {
            info!("Firmware version unchanged in flash: {}", version);
            return true;
        }
    }

    let version_bytes = version.as_bytes();
    let len = version_bytes.len().min(VERSION_MAX_LEN);

    let mut data = [0xFFu8; ERASE_SIZE];

    // Write magic
    data[0..4].copy_from_slice(&VERSION_MAGIC.to_le_bytes());
    // Write string length
    data[4] = len as u8;
    // Write version string
    data[5..5 + len].copy_from_slice(&version_bytes[..len]);

    // Erase sector
    if let Err(e) = flash.blocking_erase(VERSION_FLASH_OFFSET, VERSION_FLASH_OFFSET + ERASE_SIZE as u32) {
        error!("Version flash erase failed: {:?}", e);
        return false;
    }

    // Write data
    if let Err(e) = flash.blocking_write(VERSION_FLASH_OFFSET, &data[..256]) {
        error!("Version flash write failed: {:?}", e);
        return false;
    }

    info!("Firmware version saved to flash: {}", version);
    true
}

/// Load firmware version from flash, returns None if no valid data
pub fn load_version(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>) -> Option<heapless::String<VERSION_MAX_LEN>> {
    let mut buf = [0u8; 5 + VERSION_MAX_LEN]; // magic + len + string

    if flash.blocking_read(VERSION_FLASH_OFFSET, &mut buf).is_ok() {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic == VERSION_MAGIC {
            let len = buf[4] as usize;
            if len <= VERSION_MAX_LEN {
                if let Ok(s) = core::str::from_utf8(&buf[5..5 + len]) {
                    let mut result = heapless::String::new();
                    let _ = result.push_str(s);
                    return Some(result);
                }
            }
        }
    }

    None
}

/// Magic value for extracted private key data
const KEY_MAGIC: u32 = 0x4B455953; // "KEYS" in ASCII

/// Flash offset for key data - last sector
/// For 4MB: 0x3FF000, For 2MB: 0x1FF000
const KEY_FLASH_OFFSET: u32 = (crate::config::FLASH_SIZE_BYTES as u32) - (1 * ERASE_SIZE as u32);

/// Key data layout in flash:
///   [0..4]   magic (KEY_MAGIC, LE)
///   [4]      key length (24 for SECP192R1)
///   [5..29]  private key (24 bytes)
/// Total: 29 bytes, padded to 256 for flash write alignment

/// Save extracted private key to flash.
/// Skips write if key is unchanged — avoids unnecessary flash wear.
pub fn save_key(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>, key: &[u8; 24]) -> bool {
    // Check if already stored with same key
    if let Some(stored) = load_key(flash) {
        if stored == *key {
            info!("Key unchanged in flash, skipping write");
            return true;
        }
    }

    let mut data = [0xFFu8; ERASE_SIZE];
    data[0..4].copy_from_slice(&KEY_MAGIC.to_le_bytes());
    data[4] = 24;
    data[5..29].copy_from_slice(key);

    if let Err(e) = flash.blocking_erase(KEY_FLASH_OFFSET, KEY_FLASH_OFFSET + ERASE_SIZE as u32) {
        error!("Key flash erase failed: {:?}", e);
        return false;
    }

    if let Err(e) = flash.blocking_write(KEY_FLASH_OFFSET, &data[..256]) {
        error!("Key flash write failed: {:?}", e);
        return false;
    }

    info!("Private key saved to flash successfully");
    true
}

/// Load extracted private key from flash, returns None if no valid data
pub fn load_key(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>) -> Option<[u8; 24]> {
    let mut buf = [0u8; 29]; // magic + len + key

    if flash.blocking_read(KEY_FLASH_OFFSET, &mut buf).is_ok() {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic == KEY_MAGIC {
            let len = buf[4] as usize;
            if len == 24 {
                let mut key = [0u8; 24];
                key.copy_from_slice(&buf[5..29]);
                // Validate not all zeros or all FF
                let all_zero = key.iter().all(|&b| b == 0x00);
                let all_ff = key.iter().all(|&b| b == 0xFF);
                if !all_zero && !all_ff {
                    return Some(key);
                }
            }
        }
    }

    None
}

// ============================================================================
// Key extraction request flag
// ============================================================================

/// Magic value for key extraction request
const EXTRACT_REQUEST_MAGIC: u32 = 0x45585452; // "EXTR" in ASCII

/// Flash offset for extraction request flag - fourth-to-last sector
/// For 4MB: 0x3FC000, For 2MB: 0x1FC000
const EXTRACT_REQUEST_FLASH_OFFSET: u32 = (crate::config::FLASH_SIZE_BYTES as u32) - (4 * ERASE_SIZE as u32);

/// Save key extraction request flag to flash (before reboot).
pub fn save_extract_request(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>) -> bool {
    let mut data = [0xFFu8; ERASE_SIZE];
    data[0..4].copy_from_slice(&EXTRACT_REQUEST_MAGIC.to_le_bytes());

    if let Err(e) = flash.blocking_erase(EXTRACT_REQUEST_FLASH_OFFSET, EXTRACT_REQUEST_FLASH_OFFSET + ERASE_SIZE as u32) {
        error!("Extract request flash erase failed: {:?}", e);
        return false;
    }

    if let Err(e) = flash.blocking_write(EXTRACT_REQUEST_FLASH_OFFSET, &data[..256]) {
        error!("Extract request flash write failed: {:?}", e);
        return false;
    }

    info!("Key extraction request saved to flash");
    true
}

// ============================================================================
// Drone model persistence
// ============================================================================

/// Magic value for drone model data
const DRONE_MODEL_MAGIC: u32 = 0x4D4F444C; // "MODL" in ASCII

/// Flash offset for drone model - fifth-to-last sector
/// For 4MB: 0x3FB000, For 2MB: 0x1FB000
const DRONE_MODEL_FLASH_OFFSET: u32 = (crate::config::FLASH_SIZE_BYTES as u32) - (5 * ERASE_SIZE as u32);

/// Save drone model to flash. Returns true on success.
pub fn save_drone_model(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>, model: u8) -> bool {
    // Check if already stored with same model
    if let Some(stored) = load_drone_model(flash) {
        if stored == model {
            info!("Drone model unchanged in flash: {}", model);
            return true;
        }
    }

    let mut data = [0xFFu8; ERASE_SIZE];
    data[0..4].copy_from_slice(&DRONE_MODEL_MAGIC.to_le_bytes());
    data[4] = model;

    if let Err(e) = flash.blocking_erase(DRONE_MODEL_FLASH_OFFSET, DRONE_MODEL_FLASH_OFFSET + ERASE_SIZE as u32) {
        error!("Drone model flash erase failed: {:?}", e);
        return false;
    }

    if let Err(e) = flash.blocking_write(DRONE_MODEL_FLASH_OFFSET, &data[..256]) {
        error!("Drone model flash write failed: {:?}", e);
        return false;
    }

    info!("Drone model {} saved to flash successfully", model);
    true
}

/// Load drone model from flash, returns None if no valid data.
/// Valid values: 0 (Air3), 1 (Mavic4Pro), 2 (Air3S), 3 (Mavic3Pro).
pub fn load_drone_model(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>) -> Option<u8> {
    let mut buf = [0u8; 8];

    if flash.blocking_read(DRONE_MODEL_FLASH_OFFSET, &mut buf).is_ok() {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic == DRONE_MODEL_MAGIC {
            let model = buf[4];
            if model <= 3 {
                return Some(model);
            }
        }
    }

    None
}

/// Check if key extraction was requested, and clear the flag.
/// Returns true if extraction was requested.
pub fn load_and_clear_extract_request(flash: &mut Flash<'_, FLASH, Async, { crate::config::FLASH_SIZE_BYTES }>) -> bool {
    let mut buf = [0u8; 4];

    if flash.blocking_read(EXTRACT_REQUEST_FLASH_OFFSET, &mut buf).is_ok() {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic == EXTRACT_REQUEST_MAGIC {
            // Clear the flag by erasing the sector
            let _ = flash.blocking_erase(EXTRACT_REQUEST_FLASH_OFFSET, EXTRACT_REQUEST_FLASH_OFFSET + ERASE_SIZE as u32);
            info!("Key extraction request found and cleared");
            return true;
        }
    }

    false
}
