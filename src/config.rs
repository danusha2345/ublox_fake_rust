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
    /// First SEC-SIGN delay after NAV start for Air 3 (real: ~1025ms)
    pub const SEC_SIGN_FIRST_AIR3_MS: u64 = 1000;
    /// First SEC-SIGN delay after NAV start for Mavic 4 Pro (real: ~614ms)
    pub const SEC_SIGN_FIRST_MAVIC4_MS: u64 = 650;
    /// SEC-SIGN interval for DJI Air 3
    pub const SEC_SIGN_PERIOD_AIR3_MS: u64 = 4000;
    /// SEC-SIGN interval for DJI Mavic 4 Pro
    pub const SEC_SIGN_PERIOD_MAVIC4_MS: u64 = 2000;
    /// Delay from first config to NAV start for Air 3 (real: 666ms)
    pub const CONFIG_TO_NAV_AIR3_MS: u64 = 700;
    /// Delay from first config to NAV start for Mavic 4 Pro (real: 399ms)
    pub const CONFIG_TO_NAV_MAVIC4_MS: u64 = 400;
    /// Delay after CFG-RST before message output starts (0 = immediate)
    pub const UART_TX_INIT_DELAY_MS: u64 = 0;
    /// Time after start when satellites become invalid (ms)
    pub const SATELLITES_INVALID_AFTER_MS: u64 = 20_000;
}

/// Default coordinates (configurable)
/// Flamingo Park, Miami Beach: 25°47'09.8"N 80°08'17.0"W
pub mod default_position {
    pub const LATITUDE: f64 = 25.7860556;
    pub const LONGITUDE: f64 = -80.1380556;
    pub const ALTITUDE_M: i32 = 3;
}

/// UBX protocol version emulation
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum UbloxVersion {
    M8,   // PROTVER 18.00
    #[default]
    M10,  // PROTVER 34.10
}

/// Drone model for SEC-SIGN key selection
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, defmt::Format)]
#[repr(u8)]
pub enum DroneModel {
    #[default]
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
