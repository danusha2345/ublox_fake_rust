//! Key extraction from a real u-blox GNSS module via PIO UART.
//!
//! Triggered by long button press (3+ sec) during runtime:
//! 1. button_task detects long press → saves request flag → reboots
//! 2. On boot, main checks flag → calls `extract()` before UART init
//! 3. PIO1 UART (TX on GPIO0, RX on GPIO5) at 921600 baud
//! 4. Sends CFG-0x41 poll command (8 bytes)
//! 5. Receives 264-byte response with 2s timeout
//! 6. Parses the response and extracts the 24-byte private key

use defmt::*;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::uart::{
    PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram,
};
use embassy_rp::peripherals::{PIN_0, PIN_5};
use embassy_rp::Peri;
use embassy_time::{Duration, Timer, with_timeout};

use crate::config::DEFAULT_BAUDRATE;
use crate::ubx::calculate_checksum;

bind_interrupts!(struct Pio1Irqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

// ============================================================================
// Protocol constants
// ============================================================================

/// CFG-0x41 poll command: B5 62 06 41 00 00 47 05
const CFG41_POLL: [u8; 8] = [0xB5, 0x62, 0x06, 0x41, 0x00, 0x00, 0x47, 0x05];

/// Total expected response length: 2 (sync) + 4 (header) + 256 (payload) + 2 (cksum) = 264
const RESPONSE_LEN: usize = 264;

/// Byte offset where the payload begins inside the 264-byte response buffer.
const PAYLOAD_OFFSET: usize = 6;

/// Expected 256-byte payload length.
const EXPECTED_PAYLOAD_LEN: usize = 256;

// ---- Key location constants -----------------------------------------------

const SEC_KEY_TAG: u8 = 0xA6;
const SEC_KEY_LEN: u8 = 0x18;
const KEY_LEN: usize = 24;

/// Air 3S / Mavic 3 Pro layout: A6 18 header at payload[113].
const KEY_HEADER_OFFSET_AIR3S: usize = 113;
const KEY_DATA_OFFSET_AIR3S: usize = KEY_HEADER_OFFSET_AIR3S + 2;

/// Air 3 / Mavic 4 Pro layout: A6 18 header at payload[173].
const KEY_HEADER_OFFSET_AIR3: usize = 173;
const KEY_DATA_OFFSET_AIR3: usize = KEY_HEADER_OFFSET_AIR3 + 2;

// ============================================================================
// Response parser state machine
// ============================================================================

#[derive(Clone, Copy, PartialEq)]
enum RxState {
    WaitSync1,
    WaitSync2,
    Collect,
}

// ============================================================================
// Public API
// ============================================================================

/// Extract the 24-byte private key from a real u-blox GNSS module.
///
/// Must be called BEFORE UART0/UART1 init — GPIO0 and GPIO5 must be free.
/// Uses PIO1 to create a software UART at 921600 baud.
///
/// Returns `Some([u8; 24])` if a valid key was found, `None` otherwise.
pub async fn extract(
    pio1: Peri<'static, PIO1>,
    tx_pin: Peri<'static, PIN_0>,
    rx_pin: Peri<'static, PIN_5>,
) -> Option<[u8; 24]> {
    info!("KEY EXTRACTION MODE ACTIVATED");

    // ---- Initialize PIO1 UART ---------------------------------------------
    let Pio { mut common, sm0, sm1, .. } = Pio::new(pio1, Pio1Irqs);

    // TX on sm0 (GPIO0)
    let tx_program = PioUartTxProgram::new(&mut common);
    let mut uart_tx: PioUartTx<'_, PIO1, 0> =
        PioUartTx::new(DEFAULT_BAUDRATE, &mut common, sm0, tx_pin, &tx_program);

    // RX on sm1 (GPIO5)
    let rx_program = PioUartRxProgram::new(&mut common);
    let mut uart_rx: PioUartRx<'_, PIO1, 1> =
        PioUartRx::new(DEFAULT_BAUDRATE, &mut common, sm1, rx_pin, &rx_program);

    // Wait for GNSS module to be ready after power-on
    Timer::after(Duration::from_millis(500)).await;

    // ---- Send CFG-0x41 poll command ----------------------------------------
    info!("KEY EXTRACT: sending CFG-0x41 poll ({} bytes)", CFG41_POLL.len());
    for &b in &CFG41_POLL {
        uart_tx.write_u8(b).await;
    }
    info!("KEY EXTRACT: poll sent, waiting for response (2s timeout)...");

    // ---- Receive 264-byte response with timeout ----------------------------
    let result = with_timeout(
        Duration::from_millis(2000),
        receive_response(&mut uart_rx),
    )
    .await;

    let response_buf = match result {
        Ok(Some(buf)) => buf,
        Ok(None) => {
            error!("KEY EXTRACT: response rejected (class/id/length/checksum error)");
            return None;
        }
        Err(_elapsed) => {
            error!("KEY EXTRACT: timed out waiting for GNSS module response");
            return None;
        }
    };

    // ---- Extract key from 256-byte payload ---------------------------------
    let payload = &response_buf[PAYLOAD_OFFSET..PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN];
    extract_key(payload)
}

// ============================================================================
// Internal helpers
// ============================================================================

async fn receive_response(uart_rx: &mut PioUartRx<'_, PIO1, 1>) -> Option<[u8; RESPONSE_LEN]> {
    let mut buf = [0u8; RESPONSE_LEN];
    let mut state = RxState::WaitSync1;
    let mut idx: usize = 0;

    loop {
        let byte = uart_rx.read_u8().await;

        match state {
            RxState::WaitSync1 => {
                if byte == 0xB5 {
                    buf[0] = byte;
                    state = RxState::WaitSync2;
                }
            }

            RxState::WaitSync2 => {
                if byte == 0x62 {
                    buf[1] = byte;
                    idx = 2;
                    state = RxState::Collect;
                } else if byte == 0xB5 {
                    buf[0] = byte;
                } else {
                    state = RxState::WaitSync1;
                }
            }

            RxState::Collect => {
                if idx >= RESPONSE_LEN {
                    warn!("KEY EXTRACT: collect buffer overflow, restarting scan");
                    state = RxState::WaitSync1;
                    continue;
                }

                buf[idx] = byte;
                idx += 1;

                if idx == RESPONSE_LEN {
                    return validate_response(buf);
                }
            }
        }
    }
}

fn validate_response(buf: [u8; RESPONSE_LEN]) -> Option<[u8; RESPONSE_LEN]> {
    let class = buf[2];
    let id = buf[3];
    let payload_len = u16::from_le_bytes([buf[4], buf[5]]) as usize;

    if class != 0x06 {
        warn!("KEY EXTRACT: unexpected class {:#04x} (expected 0x06)", class);
        return None;
    }
    if id != 0x41 {
        warn!("KEY EXTRACT: unexpected id {:#04x} (expected 0x41)", id);
        return None;
    }
    if payload_len != EXPECTED_PAYLOAD_LEN {
        warn!("KEY EXTRACT: unexpected payload len {} (expected {})", payload_len, EXPECTED_PAYLOAD_LEN);
        return None;
    }

    let checksum_range = &buf[2..PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN];
    let (exp_ck_a, exp_ck_b) = calculate_checksum(checksum_range);
    let got_ck_a = buf[PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN];
    let got_ck_b = buf[PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN + 1];

    if got_ck_a != exp_ck_a || got_ck_b != exp_ck_b {
        warn!("KEY EXTRACT: checksum mismatch got={:#04x}{:#04x} exp={:#04x}{:#04x}",
            got_ck_a, got_ck_b, exp_ck_a, exp_ck_b);
        return None;
    }

    info!("KEY EXTRACT: 264-byte response validated (checksum OK)");
    Some(buf)
}

fn extract_key(payload: &[u8]) -> Option<[u8; KEY_LEN]> {
    // Try known fixed offsets first
    if let Some(key) = try_key_at(payload, KEY_HEADER_OFFSET_AIR3S, KEY_DATA_OFFSET_AIR3S, "Air3S/Mavic3Pro") {
        return Some(key);
    }
    if let Some(key) = try_key_at(payload, KEY_HEADER_OFFSET_AIR3, KEY_DATA_OFFSET_AIR3, "Air3/Mavic4Pro") {
        return Some(key);
    }

    // Fallback: scan for A6 18 anywhere
    info!("KEY EXTRACT: known offsets miss, scanning for A6 18...");
    let scan_limit = payload.len().saturating_sub(1 + KEY_LEN);
    for i in 0..scan_limit {
        if payload[i] == SEC_KEY_TAG && payload[i + 1] == SEC_KEY_LEN {
            let data_start = i + 2;
            let data_end = data_start + KEY_LEN;
            if data_end > payload.len() {
                continue;
            }
            let mut key = [0u8; KEY_LEN];
            key.copy_from_slice(&payload[data_start..data_end]);
            if validate_key_bytes(&key) {
                info!("KEY EXTRACT: key found via scan at payload[{}..{}]", data_start, data_end);
                log_key(&key);
                return Some(key);
            }
        }
    }

    error!("KEY EXTRACT: private key not found in payload");
    None
}

fn try_key_at(payload: &[u8], header_offset: usize, data_offset: usize, layout_name: &str) -> Option<[u8; KEY_LEN]> {
    let data_end = data_offset + KEY_LEN;

    if header_offset + 1 >= payload.len() || data_end > payload.len() {
        return None;
    }

    if payload[header_offset] != SEC_KEY_TAG || payload[header_offset + 1] != SEC_KEY_LEN {
        return None;
    }

    let mut key = [0u8; KEY_LEN];
    key.copy_from_slice(&payload[data_offset..data_end]);

    if !validate_key_bytes(&key) {
        warn!("KEY EXTRACT: {} header matched but key invalid", layout_name);
        return None;
    }

    info!("KEY EXTRACT: {} layout, key extracted OK", layout_name);
    log_key(&key);
    Some(key)
}

fn validate_key_bytes(key: &[u8; KEY_LEN]) -> bool {
    let all_zero = key.iter().all(|&b| b == 0x00);
    let all_ff = key.iter().all(|&b| b == 0xFF);
    !all_zero && !all_ff
}

fn log_key(key: &[u8; KEY_LEN]) {
    info!(
        "KEY EXTRACT: key first4={:#04x} {:#04x} {:#04x} {:#04x}  last4={:#04x} {:#04x} {:#04x} {:#04x}",
        key[0], key[1], key[2], key[3],
        key[20], key[21], key[22], key[23],
    );
}
