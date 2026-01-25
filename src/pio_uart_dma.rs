//! PIO UART RX with fast polling for high-throughput data reception
//!
//! This module provides a custom PIO UART RX driver that uses fast polling
//! to read data from the PIO FIFO, solving the packet loss problem caused by FIFO overflow.
//!
//! The standard PioUartRx reads byte-by-byte with wait_pull(), which is too slow
//! at 921600 baud (FIFO overflows in ~44µs with 4-byte FIFO).
//!
//! This implementation provides try_read_available() for fast polling
//! to drain the FIFO as quickly as possible.

use defmt::*;
use embassy_rp::dma::Channel;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{clocks, Peri};
use fixed::traits::ToFixed;

/// PIO UART RX with fast polling support
///
/// This driver provides fast FIFO access methods for high-throughput UART reception.
/// Instead of waiting for each byte, it drains the entire FIFO in tight polling loops.
pub struct PioUartRxDma<'d, PIO: Instance, const SM: usize, DMA: Channel> {
    sm: StateMachine<'d, PIO, SM>,
    _dma: Peri<'d, DMA>,
}

impl<'d, PIO: Instance, const SM: usize, DMA: Channel> PioUartRxDma<'d, PIO, SM, DMA> {
    /// Create a new PIO UART RX with DMA channel
    ///
    /// # Arguments
    /// * `baud` - Baud rate (e.g., 921600)
    /// * `common` - PIO common instance for configuration
    /// * `sm` - State machine to use
    /// * `dma` - DMA channel (reserved for future use)
    /// * `pin` - GPIO pin for RX input (as Peri wrapper)
    pub fn new<PIN: embassy_rp::gpio::Pin + PioPin>(
        baud: u32,
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        dma: Peri<'d, DMA>,
        pin: impl Into<Peri<'d, PIN>>,
    ) -> Self {
        // UART RX program assembly (8N1 with framing error handling)
        // Same as embassy-rp's uart.rs program
        let prg = pio::pio_asm!(
            ".side_set 0 opt"
            ".wrap_target"
            "start:"
            "    wait 0 pin 0"           // Wait for start bit (pin goes low)
            "    set x, 7 [10]"          // Bit counter = 7, delay to middle of first data bit
            "rx_bitloop:"
            "    in pins, 1"             // Shift in one data bit
            "    jmp x-- rx_bitloop [6]" // Loop until all 8 bits received
            "    jmp pin good_rx_stop"   // Check stop bit (should be high)
            "    irq 4 rel"              // Framing error: set IRQ flag
            "    wait 1 pin 0"           // Wait for line to go idle
            "    jmp start"              // Restart without pushing
            "good_rx_stop:"
            "    in null, 24"            // Pad ISR to 32 bits (data in MSB)
            "    push"                   // Push to RX FIFO
            ".wrap"
        );

        // Load program into PIO instruction memory
        let loaded_program = common.load_program(&prg.program);

        // Configure the pin as PIO input
        let pin = common.make_pio_pin(pin.into());
        sm.set_pin_dirs(Direction::In, &[&pin]);

        // Configure state machine
        let mut cfg = Config::default();

        // Set program
        cfg.use_program(&loaded_program, &[]);

        // Clock divider: sys_clk / (8 * baud)
        // PIO program runs at 8 cycles per bit
        let clock_freq = clocks::clk_sys_freq();
        cfg.clock_divider = (clock_freq as f64 / (8.0 * baud as f64)).to_fixed();

        // Shift configuration: right shift, autopush at 32 bits
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Right,
            threshold: 32,
        };

        // FIFO: join for RX only (8-word FIFO instead of 4)
        cfg.fifo_join = FifoJoin::RxOnly;

        // Set input pin for both data and JMP PIN instruction
        cfg.set_in_pins(&[&pin]);
        cfg.set_jmp_pin(&pin);

        // Apply configuration and enable
        sm.set_config(&cfg);
        sm.set_enable(true);

        info!("PIO UART RX initialized: baud={}, pin={}", baud, pin.pin());

        Self { sm, _dma: dma }
    }

    /// Read a single byte (blocking until available)
    ///
    /// Uses wait_pull() internally.
    #[allow(dead_code)]
    pub async fn read_byte(&mut self) -> u8 {
        let word = self.sm.rx().wait_pull().await;
        // Data is in upper 8 bits due to right shift + null padding
        (word >> 24) as u8
    }

    /// Try to read available bytes from FIFO without blocking
    ///
    /// Returns number of bytes read into the output buffer.
    /// This is the preferred method for high-throughput UART reception.
    /// Call this in a tight polling loop to prevent FIFO overflow.
    pub fn try_read_available(&mut self, output: &mut [u8]) -> usize {
        let mut count = 0;
        while count < output.len() {
            if let Some(word) = self.sm.rx().try_pull() {
                // Data is in upper 8 bits due to right shift + null padding
                output[count] = (word >> 24) as u8;
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    /// Check if FIFO has data available
    #[allow(dead_code)]
    pub fn has_data(&mut self) -> bool {
        !self.sm.rx().empty()
    }

    /// Get the number of words in FIFO (0-8 with RxOnly join)
    #[allow(dead_code)]
    pub fn fifo_level(&mut self) -> u8 {
        self.sm.rx().level()
    }
}

/// Convert raw PIO words to bytes
///
/// PIO UART program shifts data right and pads with 24 null bits,
/// so the received byte is in the upper 8 bits of each u32 word.
#[inline]
#[allow(dead_code)]
pub fn words_to_bytes(words: &[u32], output: &mut [u8]) -> usize {
    let count = words.len().min(output.len());
    for i in 0..count {
        // Data is in upper 8 bits
        output[i] = (words[i] >> 24) as u8;
    }
    count
}
