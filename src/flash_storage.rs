//! Flash storage for persistent mode

use defmt::*;
use embassy_rp::flash::{Async, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

/// Magic value to identify valid flash data
const FLASH_MAGIC: u32 = 0xDEADBEEF;

/// Flash offset - second-to-last sector (автоматически вычисляется от размера памяти)
/// Last sector causes erase to fail (end == FLASH_SIZE)
/// Using second-to-last sector for safety
/// Для 4MB: 0x3FE000 (sector 1022 of 1024)
/// Для 2MB: 0x1FE000 (sector 510 of 512)
const FLASH_OFFSET: u32 = (crate::config::FLASH_SIZE_BYTES as u32) - (2 * ERASE_SIZE as u32);

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
