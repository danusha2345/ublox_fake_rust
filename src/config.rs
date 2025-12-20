//! Hardware configuration and constants

/// Default UART baudrate
pub const DEFAULT_BAUDRATE: u32 = 921600;

/// GPIO pin assignments
pub mod pins {
    pub const UART_TX: u8 = 0;
    pub const UART_RX: u8 = 1;
    pub const PIO_PASSTHROUGH_IN: u8 = 3;
    pub const MODE_BTN_PWR: u8 = 5;
    pub const MODE_BTN_INPUT: u8 = 6;
    pub const WS2812_LED: u8 = 16;
}

/// Timer periods in milliseconds
pub mod timers {
    pub const NAV_PERIOD_MS: u64 = 200;   // 5Hz
    pub const MON_PERIOD_MS: u64 = 1000;  // 1Hz
    pub const SEC_SIGN_FIRST_MS: u64 = 3000;
    pub const SEC_SIGN_PERIOD_MS: u64 = 4000;
}

/// Default coordinates (Moscow, Red Square)
pub mod default_position {
    pub const LATITUDE: f64 = 55.761199;
    pub const LONGITUDE: f64 = 37.618423;
    pub const ALTITUDE_M: i32 = 156;
}

/// UBX protocol version emulation
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum UbloxVersion {
    M8,   // PROTVER 18.00
    M10,  // PROTVER 34.10
}

impl Default for UbloxVersion {
    fn default() -> Self {
        Self::M10
    }
}
