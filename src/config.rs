//! Hardware configuration and constants

#![allow(dead_code)]

/// Default UART baudrate
pub const DEFAULT_BAUDRATE: u32 = 921600;

/// GPIO pin assignments for RP2350A (Spotpear RP2350-Core-A)
pub mod pins {
    // UART0: к дрону/хосту
    pub const UART0_TX: u8 = 0;
    pub const UART0_RX: u8 = 1;

    // UART1: от внешнего GNSS модуля (passthrough source)
    pub const UART1_TX: u8 = 4;   // не используется, но резервируем
    pub const UART1_RX: u8 = 5;   // вход от внешнего GNSS

    // Mode button (перенесено с GPIO5/6)
    pub const MODE_BTN_PWR: u8 = 6;
    pub const MODE_BTN_INPUT: u8 = 7;

    // WS2812B LED (RP2350-Core-A: GPIO25)
    pub const WS2812_LED: u8 = 25;

    // Legacy alias
    pub const UART_TX: u8 = UART0_TX;
    pub const UART_RX: u8 = UART0_RX;
}

/// Timer periods in milliseconds
pub mod timers {
    /// Default NAV measurement period (5Hz = 200ms)
    pub const NAV_MEAS_PERIOD_MS: u32 = 200;
    /// Default NAV rate (cycles per navigation solution)
    pub const NAV_RATE: u32 = 1;
    /// MON message period (1Hz)
    pub const MON_PERIOD_MS: u64 = 1000;
    /// LED blink period
    pub const LED_BLINK_MS: u64 = 500;
    /// First SEC-SIGN delay after start
    pub const SEC_SIGN_FIRST_MS: u64 = 3000;
    /// SEC-SIGN interval for DJI Air 3
    pub const SEC_SIGN_PERIOD_AIR3_MS: u64 = 4000;
    /// SEC-SIGN interval for DJI Mavic 4 Pro
    pub const SEC_SIGN_PERIOD_MAVIC4_MS: u64 = 2000;
    /// Delay after CFG-RST before message output starts (0 = immediate)
    pub const UART_TX_INIT_DELAY_MS: u64 = 0;
    /// Time after start when satellites become invalid (ms)
    pub const SATELLITES_INVALID_AFTER_MS: u64 = 20_000;
}

/// Default coordinates (configurable)
pub mod default_position {
    pub const LATITUDE: f64 = 25.7889186;
    pub const LONGITUDE: f64 = -80.1919471;
    pub const ALTITUDE_M: i32 = 101;
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

/// Drone model for SEC-SIGN key selection
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum DroneModel {
    Air3 = 0,
    Mavic4Pro = 1,
}

impl DroneModel {
    pub fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::Mavic4Pro,
            _ => Self::Air3,
        }
    }
}

impl Default for DroneModel {
    fn default() -> Self {
        Self::Air3
    }
}
