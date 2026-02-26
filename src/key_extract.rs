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
use embassy_time::{Duration, Instant, Timer, with_timeout};

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

/// Number of extraction attempts before giving up
const MAX_ATTEMPTS: u8 = 3;

/// Timeout for each receive attempt (ms)
const RX_TIMEOUT_MS: u64 = 3000;

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
/// Retries up to MAX_ATTEMPTS times. Each attempt sends CFG-0x41 and waits
/// for response with RX_TIMEOUT_MS timeout.
///
/// Returns `Some([u8; 24])` if a valid key was found, `None` otherwise.
pub async fn extract(
    pio1: Peri<'static, PIO1>,
    tx_pin: Peri<'static, PIN_0>,
    rx_pin: Peri<'static, PIN_5>,
) -> Option<[u8; 24]> {
    let t0 = Instant::now();
    info!("========================================");
    info!("KEY EXTRACTION MODE ACTIVATED");
    info!("  baudrate: {}", DEFAULT_BAUDRATE);
    info!("  TX: GPIO0 (PIO1 SM0)");
    info!("  RX: GPIO5 (PIO1 SM1)");
    info!("  attempts: {}, timeout: {}ms each", MAX_ATTEMPTS, RX_TIMEOUT_MS);
    info!("========================================");

    // ---- Initialize PIO1 UART ---------------------------------------------
    info!("KEY EXTRACT: initializing PIO1 UART...");
    let t_pio = Instant::now();
    let Pio { mut common, sm0, sm1, .. } = Pio::new(pio1, Pio1Irqs);

    let tx_program = PioUartTxProgram::new(&mut common);
    let mut uart_tx: PioUartTx<'_, PIO1, 0> =
        PioUartTx::new(DEFAULT_BAUDRATE, &mut common, sm0, tx_pin, &tx_program);

    let rx_program = PioUartRxProgram::new(&mut common);
    let mut uart_rx: PioUartRx<'_, PIO1, 1> =
        PioUartRx::new(DEFAULT_BAUDRATE, &mut common, sm1, rx_pin, &rx_program);
    info!("KEY EXTRACT: PIO1 UART initialized in {}ms", t_pio.elapsed().as_millis());

    // Wait for GNSS module to be ready after power-on
    info!("KEY EXTRACT: waiting 500ms for GNSS module...");
    Timer::after(Duration::from_millis(500)).await;

    // ---- Multiple attempts -------------------------------------------------
    for attempt in 1..=MAX_ATTEMPTS {
        info!("KEY EXTRACT: === attempt {}/{} ===", attempt, MAX_ATTEMPTS);

        // Send CFG-0x41 poll command
        let t_tx = Instant::now();
        info!("KEY EXTRACT: sending CFG-0x41 poll: {:02x}", CFG41_POLL.as_slice());
        for &b in &CFG41_POLL {
            uart_tx.write_u8(b).await;
        }
        info!("KEY EXTRACT: poll sent in {}us, waiting for 264-byte response...",
            t_tx.elapsed().as_micros());

        // Receive response with timeout
        let t_rx = Instant::now();
        let result = with_timeout(
            Duration::from_millis(RX_TIMEOUT_MS),
            receive_response(&mut uart_rx),
        )
        .await;

        match result {
            Ok(Some(buf)) => {
                let rx_ms = t_rx.elapsed().as_millis();
                info!("KEY EXTRACT: response received in {}ms", rx_ms);

                // Dump full response header
                info!("KEY EXTRACT: header: {:02x}", &buf[..6]);

                // Extract and dump payload
                let payload = &buf[PAYLOAD_OFFSET..PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN];
                dump_payload(payload);

                // Extract key
                if let Some(key) = extract_key(payload) {
                    let total_ms = t0.elapsed().as_millis();
                    info!("========================================");
                    info!("KEY EXTRACT: SUCCESS in {}ms (attempt {})", total_ms, attempt);
                    log_key_full(&key);
                    info!("========================================");
                    return Some(key);
                }

                error!("KEY EXTRACT: payload received but key not found (attempt {})", attempt);
            }
            Ok(None) => {
                let rx_ms = t_rx.elapsed().as_millis();
                error!("KEY EXTRACT: response validation failed after {}ms (attempt {})", rx_ms, attempt);
            }
            Err(_elapsed) => {
                error!("KEY EXTRACT: timeout after {}ms (attempt {})", RX_TIMEOUT_MS, attempt);
            }
        }

        // Pause between retries
        if attempt < MAX_ATTEMPTS {
            info!("KEY EXTRACT: retrying in 500ms...");
            Timer::after(Duration::from_millis(500)).await;
        }
    }

    let total_ms = t0.elapsed().as_millis();
    error!("KEY EXTRACT: FAILED after {} attempts ({}ms total)", MAX_ATTEMPTS, total_ms);
    None
}

// ============================================================================
// Internal helpers
// ============================================================================

async fn receive_response(uart_rx: &mut PioUartRx<'_, PIO1, 1>) -> Option<[u8; RESPONSE_LEN]> {
    let mut buf = [0u8; RESPONSE_LEN];
    let mut state = RxState::WaitSync1;
    let mut idx: usize = 0;
    let mut total_rx_bytes: u32 = 0;
    let mut garbage_bytes: u32 = 0;
    let t_start = Instant::now();

    loop {
        let byte = uart_rx.read_u8().await;
        total_rx_bytes += 1;

        match state {
            RxState::WaitSync1 => {
                if byte == 0xB5 {
                    buf[0] = byte;
                    state = RxState::WaitSync2;
                    if garbage_bytes > 0 {
                        warn!("KEY EXTRACT: {} garbage bytes before sync", garbage_bytes);
                    }
                    debug!("KEY EXTRACT: sync1 (0xB5) at byte #{}", total_rx_bytes);
                } else {
                    garbage_bytes += 1;
                    // Log first few garbage bytes for debugging
                    if garbage_bytes <= 16 {
                        debug!("KEY EXTRACT: garbage[{}]: {:#04x}", garbage_bytes, byte);
                    }
                }
            }

            RxState::WaitSync2 => {
                if byte == 0x62 {
                    buf[1] = byte;
                    idx = 2;
                    state = RxState::Collect;
                    debug!("KEY EXTRACT: sync2 (0x62) OK, collecting payload...");
                } else if byte == 0xB5 {
                    buf[0] = byte;
                    warn!("KEY EXTRACT: double 0xB5, restarting sync2 wait");
                } else {
                    warn!("KEY EXTRACT: bad sync2 byte {:#04x} (expected 0x62), restart", byte);
                    state = RxState::WaitSync1;
                }
            }

            RxState::Collect => {
                if idx >= RESPONSE_LEN {
                    warn!("KEY EXTRACT: collect overflow, restarting scan");
                    state = RxState::WaitSync1;
                    garbage_bytes = 0;
                    continue;
                }

                buf[idx] = byte;
                idx += 1;

                // Progress log every 64 bytes
                if idx % 64 == 0 {
                    debug!("KEY EXTRACT: collected {}/{} bytes ({}ms)",
                        idx, RESPONSE_LEN, t_start.elapsed().as_millis());
                }

                if idx == RESPONSE_LEN {
                    info!("KEY EXTRACT: all {} bytes received ({}ms, {} total RX bytes, {} garbage)",
                        RESPONSE_LEN, t_start.elapsed().as_millis(), total_rx_bytes, garbage_bytes);
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

    info!("KEY EXTRACT: validating: class={:#04x} id={:#04x} len={}", class, id, payload_len);

    if class != 0x06 {
        error!("KEY EXTRACT: wrong class {:#04x} (expected 0x06)", class);
        return None;
    }
    if id != 0x41 {
        error!("KEY EXTRACT: wrong id {:#04x} (expected 0x41)", id);
        return None;
    }
    if payload_len != EXPECTED_PAYLOAD_LEN {
        error!("KEY EXTRACT: wrong payload len {} (expected {})", payload_len, EXPECTED_PAYLOAD_LEN);
        return None;
    }

    let checksum_range = &buf[2..PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN];
    let (exp_ck_a, exp_ck_b) = calculate_checksum(checksum_range);
    let got_ck_a = buf[PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN];
    let got_ck_b = buf[PAYLOAD_OFFSET + EXPECTED_PAYLOAD_LEN + 1];

    if got_ck_a != exp_ck_a || got_ck_b != exp_ck_b {
        error!("KEY EXTRACT: CHECKSUM FAIL got={:#04x},{:#04x} exp={:#04x},{:#04x}",
            got_ck_a, got_ck_b, exp_ck_a, exp_ck_b);
        // Dump first and last 16 bytes for debugging
        info!("KEY EXTRACT: buf[0..16]:  {:02x}", &buf[..16]);
        info!("KEY EXTRACT: buf[248..264]: {:02x}", &buf[248..264]);
        return None;
    }

    info!("KEY EXTRACT: checksum OK ({:#04x},{:#04x})", got_ck_a, got_ck_b);
    Some(buf)
}

/// Dump the full 256-byte payload in 16-byte rows for RTT analysis
fn dump_payload(payload: &[u8]) {
    info!("KEY EXTRACT: ---- payload dump (256 bytes) ----");
    for row in 0..16 {
        let off = row * 16;
        info!("  [{:03}]: {:02x}", off, &payload[off..off + 16]);
    }
    info!("KEY EXTRACT: ---- end payload dump ----");
}

fn extract_key(payload: &[u8]) -> Option<[u8; KEY_LEN]> {
    info!("KEY EXTRACT: searching for A6 18 key header...");

    // Check known offsets
    info!("KEY EXTRACT: checking Air3S/Mavic3Pro offset (payload[{}]): {:#04x} {:#04x}",
        KEY_HEADER_OFFSET_AIR3S,
        payload[KEY_HEADER_OFFSET_AIR3S], payload[KEY_HEADER_OFFSET_AIR3S + 1]);
    if let Some(key) = try_key_at(payload, KEY_HEADER_OFFSET_AIR3S, KEY_DATA_OFFSET_AIR3S, "Air3S/Mavic3Pro") {
        return Some(key);
    }

    info!("KEY EXTRACT: checking Air3/Mavic4Pro offset (payload[{}]): {:#04x} {:#04x}",
        KEY_HEADER_OFFSET_AIR3,
        payload[KEY_HEADER_OFFSET_AIR3], payload[KEY_HEADER_OFFSET_AIR3 + 1]);
    if let Some(key) = try_key_at(payload, KEY_HEADER_OFFSET_AIR3, KEY_DATA_OFFSET_AIR3, "Air3/Mavic4Pro") {
        return Some(key);
    }

    // Fallback: scan for A6 18 anywhere
    warn!("KEY EXTRACT: known offsets miss, scanning entire payload for A6 18...");
    let scan_limit = payload.len().saturating_sub(1 + KEY_LEN);
    for i in 0..scan_limit {
        if payload[i] == SEC_KEY_TAG && payload[i + 1] == SEC_KEY_LEN {
            info!("KEY EXTRACT: A6 18 found at payload[{}]", i);
            let data_start = i + 2;
            let data_end = data_start + KEY_LEN;
            if data_end > payload.len() {
                continue;
            }
            let mut key = [0u8; KEY_LEN];
            key.copy_from_slice(&payload[data_start..data_end]);
            if validate_key_bytes(&key) {
                info!("KEY EXTRACT: key found via scan at payload[{}..{}]", data_start, data_end);
                log_key_full(&key);
                return Some(key);
            } else {
                warn!("KEY EXTRACT: A6 18 at [{}] but key invalid (all-zero or all-FF)", i);
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
    log_key_full(&key);
    Some(key)
}

fn validate_key_bytes(key: &[u8; KEY_LEN]) -> bool {
    let all_zero = key.iter().all(|&b| b == 0x00);
    let all_ff = key.iter().all(|&b| b == 0xFF);
    !all_zero && !all_ff
}

/// Log full 24-byte key as hex (for SWD/RTT debugging)
fn log_key_full(key: &[u8; KEY_LEN]) {
    info!("KEY EXTRACT: key[0..12]:  {:02x}", &key[..12]);
    info!("KEY EXTRACT: key[12..24]: {:02x}", &key[12..24]);
}
