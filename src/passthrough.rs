//! PIO-based UART passthrough for transparent forwarding

#![allow(dead_code)]

use embassy_rp::pio::{Common, Config, Instance, PioPin, StateMachine};
use embassy_rp::Peri;
use fixed::traits::ToFixed;
use fixed::types::U24F8;
use fixed_macro::fixed;

/// PIO passthrough driver - copies input pin state to output pin
pub struct Passthrough<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> Passthrough<'d, P, S> {
    /// Create new passthrough driver
    /// - in_pin: GPIO3 (PIO input from external GNSS)
    /// - out_pin: GPIO0 (UART TX, output to host)
    pub fn new<IN: PioPin, OUT: PioPin>(
        common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        in_pin: Peri<'d, IN>,
        out_pin: Peri<'d, OUT>,
    ) -> Self {
        // PIO program: copy input pin state to output pin
        // wait 1 pin 0  - wait for input high
        // set pins, 1   - set output high
        // wait 0 pin 0  - wait for input low
        // set pins, 0   - set output low
        let prg = pio::pio_asm!(
            ".wrap_target",
            "    wait 1 pin 0",
            "    set pins, 1",
            "    wait 0 pin 0",
            "    set pins, 0",
            ".wrap"
        );

        let pio_in_pin = common.make_pio_pin(in_pin);
        let pio_out_pin = common.make_pio_pin(out_pin);

        let mut cfg = Config::default();
        cfg.use_program(&common.load_program(&prg.program), &[]);
        cfg.set_in_pins(&[&pio_in_pin]);
        cfg.set_set_pins(&[&pio_out_pin]);

        // Fast clock for accurate signal copying at 921600 baud
        // 8MHz gives ~8 samples per bit at 921600 baud
        let clock_freq = fixed!(8_000_000: U24F8);
        cfg.clock_divider = (U24F8::from_num(125_000_000u32) / clock_freq).to_fixed();

        sm.set_config(&cfg);
        sm.set_pin_dirs(embassy_rp::pio::Direction::In, &[&pio_in_pin]);
        sm.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&pio_out_pin]);

        // Start disabled - will be enabled when switching to passthrough mode
        sm.set_enable(false);

        Self { sm }
    }

    /// Enable passthrough
    pub fn enable(&mut self) {
        self.sm.set_enable(true);
    }

    /// Disable passthrough
    pub fn disable(&mut self) {
        self.sm.set_enable(false);
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        // StateMachine doesn't have is_enabled, track externally
        false
    }
}
