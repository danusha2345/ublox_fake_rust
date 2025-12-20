//! WS2812 LED driver using PIO

use embassy_rp::pio::{Common, Config, Instance, PioPin, ShiftDirection, StateMachine};
use embassy_rp::Peri;
use fixed::traits::ToFixed;
use fixed::types::U24F8;
use fixed_macro::fixed;

/// RGB color
#[derive(Clone, Copy, Default)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const fn off() -> Self {
        Self { r: 0, g: 0, b: 0 }
    }

    pub const fn green() -> Self {
        Self { r: 0, g: 50, b: 0 }
    }

    pub const fn blue() -> Self {
        Self { r: 0, g: 0, b: 100 }
    }

    pub const fn red() -> Self {
        Self { r: 100, g: 0, b: 0 }
    }

    /// Convert to GRB format for WS2812
    pub fn to_grb(self) -> u32 {
        ((self.g as u32) << 16) | ((self.r as u32) << 8) | (self.b as u32)
    }
}

/// WS2812 driver using PIO
pub struct Ws2812<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> Ws2812<'d, P, S> {
    /// Create new WS2812 driver
    pub fn new<PIN: PioPin>(
        common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        pin: Peri<'d, PIN>,
    ) -> Self {
        // WS2812 PIO program
        let prg = pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "bitloop:",
            "    out x, 1       side 0 [2]",
            "    jmp !x do_zero side 1 [1]",
            "    jmp bitloop    side 1 [4]",
            "do_zero:",
            "    nop            side 0 [4]",
            ".wrap"
        );

        let pio_pin = common.make_pio_pin(pin);

        let mut cfg = Config::default();
        cfg.use_program(&common.load_program(&prg.program), &[&pio_pin]);
        cfg.set_out_pins(&[&pio_pin]);
        cfg.set_set_pins(&[&pio_pin]);
        cfg.shift_out.auto_fill = true;
        cfg.shift_out.threshold = 24;
        cfg.shift_out.direction = ShiftDirection::Left;

        // WS2812 timing: 800kHz base, 8 cycles per bit = 6.25MHz
        let clock_freq = fixed!(6_250_000: U24F8);
        cfg.clock_divider = (U24F8::from_num(125_000_000u32) / clock_freq).to_fixed();

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self { sm }
    }

    /// Write a color to the LED
    pub async fn write_color(&mut self, color: Color) {
        let word = color.to_grb() << 8;
        self.sm.tx().wait_push(word).await;
    }
}
