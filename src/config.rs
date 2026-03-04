//! Hardware configuration and constants

#![allow(dead_code)]

/// Default UART baudrate
pub const DEFAULT_BAUDRATE: u32 = 921600;

/// Flash memory size in bytes
#[cfg(not(feature = "rp2354"))]
pub const FLASH_SIZE_BYTES: usize = 4 * 1024 * 1024; // 4MB external QSPI flash (RP2350)

#[cfg(feature = "rp2354")]
pub const FLASH_SIZE_BYTES: usize = 2 * 1024 * 1024; // 2MB internal flash (RP2354)

/// GPIO pin assignments
#[cfg(not(feature = "rp2354"))]
pub mod pins {
    // RP2350A (Spotpear RP2350-Core-A)

    // UART0: к дрону/хосту
    pub const UART0_TX: u8 = 0;
    pub const UART0_RX: u8 = 1;

    // UART1: от внешнего GNSS модуля (passthrough source)
    pub const UART1_TX: u8 = 4;   // не используется, но резервируем
    pub const UART1_RX: u8 = 5;   // вход от внешнего GNSS

    // Mode button (RP2350: GPIO13=PWR, GPIO14=INPUT)
    pub const MODE_BTN_PWR: u8 = 13;
    pub const MODE_BTN_INPUT: u8 = 14;

    // WS2812B LED
    pub const WS2812_LED: u8 = 25;
    pub const HAS_WS2812_LED: bool = true;

    // Legacy alias
    pub const UART_TX: u8 = UART0_TX;
    pub const UART_RX: u8 = UART0_RX;
}

#[cfg(feature = "rp2354")]
pub mod pins {
    // RP2354A - отдельная конфигурация пинов

    // UART0: к дрону/хосту
    pub const UART0_TX: u8 = 0;
    pub const UART0_RX: u8 = 1;

    // UART1: от внешнего GNSS модуля (passthrough source)
    pub const UART1_TX: u8 = 4;   // не используется, но резервируем
    pub const UART1_RX: u8 = 5;   // вход от внешнего GNSS

    // Mode button (RP2354: GPIO13/14)
    pub const MODE_BTN_PWR: u8 = 13;
    pub const MODE_BTN_INPUT: u8 = 14;

    // WS2812B LED - отключен для RP2354
    pub const WS2812_LED: u8 = 16;  // не используется
    pub const HAS_WS2812_LED: bool = false;

    // Simple GPIO LED для RP2354
    // Схема: GPIO11 (анод +) --- LED --- GPIO12 (катод -, земля)
    pub const SIMPLE_LED_ANODE: u8 = 11;   // управляющий пин (HIGH = включен)
    pub const SIMPLE_LED_CATHODE: u8 = 12; // земля (постоянно LOW)

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
    /// First SEC-SIGN delay after NAV start for Air 3S (same as Mavic 4 Pro)
    pub const SEC_SIGN_FIRST_AIR3S_MS: u64 = 650;
    /// SEC-SIGN interval for DJI Air 3S
    pub const SEC_SIGN_PERIOD_AIR3S_MS: u64 = 2000;
    /// Delay from first config to NAV start for Air 3S (real: 780ms)
    pub const CONFIG_TO_NAV_AIR3S_MS: u64 = 780;
    /// First SEC-SIGN delay after NAV start for Mavic 3 Pro (estimated, same as Air 3S)
    pub const SEC_SIGN_FIRST_MAVIC3PRO_MS: u64 = 650;
    /// SEC-SIGN interval for DJI Mavic 3 Pro (confirmed from logs)
    pub const SEC_SIGN_PERIOD_MAVIC3PRO_MS: u64 = 2000;
    /// Delay from first config to NAV start for Mavic 3 Pro (estimated, same as Air 3S)
    pub const CONFIG_TO_NAV_MAVIC3PRO_MS: u64 = 780;
    /// Delay after CFG-RST before message output starts (0 = immediate)
    pub const UART_TX_INIT_DELAY_MS: u64 = 0;
    /// Time after start when satellites become invalid (ms)
    pub const SATELLITES_INVALID_AFTER_MS: u64 = 20_000;
}

/// Константы для логики кнопки переключения режимов
pub mod button {
    /// Максимальное время между нажатиями для серии (мс)
    /// 800мс даёт достаточно времени для 3 кликов (было 500мс - слишком мало)
    pub const MULTI_CLICK_TIMEOUT_MS: u64 = 800;
    /// Время debounce (мс)
    pub const DEBOUNCE_MS: u64 = 50;
    /// Период опроса кнопки (мс)
    pub const POLL_PERIOD_MS: u64 = 20;
    /// Длительность удержания для запуска key extraction (мс)
    pub const LONG_PRESS_MS: u64 = 3000;
    /// Таймаут режима выбора модели дрона без кликов (мс)
    pub const MODEL_SELECT_TIMEOUT_MS: u64 = 5000;
}

/// Default coordinates (configurable) — Golden Beach, FL
pub mod default_position {
    pub const LATITUDE: f64 = 25.966443;
    pub const LONGITUDE: f64 = -80.122371;
    pub const ALTITUDE_M: i32 = 100;
}

/// Coordinate offset for PassthroughOffset mode
/// Transforms: Saint Petersburg (59.9343°N, 30.3351°E) → Austin, Nevada (39.4933°N, 117.0686°W)
pub mod coordinate_offset {
    /// Latitude offset in 1e-7 degrees: (39.4933 - 59.9343) × 1e7
    pub const LAT_OFFSET_1E7: i32 = -204_410_000;
    /// Longitude offset in 1e-7 degrees: (-117.0686 - 30.3351) × 1e7
    pub const LON_OFFSET_1E7: i32 = -1_474_037_000;
    /// Altitude offset in mm (no change)
    pub const ALT_OFFSET_MM: i32 = 0;

    /// Pre-computed ECEF offset in centimeters
    /// Saint Petersburg ECEF: X=2825884m, Y=1671063m, Z=5479661m
    /// Austin, Nevada ECEF:   X=-2242830m, Y=-4388803m, Z=4034730m
    pub const ECEF_OFFSET_X_CM: i32 = -506_871_400;  // (-2242830 - 2825884) * 100
    pub const ECEF_OFFSET_Y_CM: i32 = -605_986_600;  // (-4388803 - 1671063) * 100
    pub const ECEF_OFFSET_Z_CM: i32 = -144_493_100;  // (4034730 - 5479661) * 100
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
    Air3S = 2,
    Mavic3Pro = 3,
}

impl DroneModel {
    pub fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::Mavic4Pro,
            2 => Self::Air3S,
            3 => Self::Mavic3Pro,
            _ => Self::Air3,
        }
    }
}
