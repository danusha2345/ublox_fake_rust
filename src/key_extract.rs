//! Key extraction from a real u-blox GNSS module.
//!
//! TX: PIO1 UART on GPIO1 (к GNSS RX) — 921600 baud
//! RX: Hardware UART1 on GPIO5 (от GNSS TX) — BufferedUart with 1KB buffer
//!
//! Triggered by long button press (3+ sec) or FORCE_KEY_EXTRACT flag.
//! Must be called BEFORE main UART init — GPIO1 and GPIO5 must be free.

use defmt::*;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO1, PIN_1, PIN_5, UART1};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::uart::{PioUartTx, PioUartTxProgram};
use embassy_rp::uart::{BufferedUartRx, Config as UartConfig};
use embassy_rp::Peri;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::Read;
use static_cell::StaticCell;

use crate::config::DEFAULT_BAUDRATE;

bind_interrupts!(struct Pio1Irqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

// UART1_IRQ is bound in main.rs crate::Irqs

// ============================================================================
// Protocol constants
// ============================================================================

/// CFG-0x41 poll command: B5 62 06 41 00 00 47 DB
const CFG41_POLL: [u8; 8] = [0xB5, 0x62, 0x06, 0x41, 0x00, 0x00, 0x47, 0xDB];

const EXPECTED_PAYLOAD_LEN: usize = 256;
const MAX_ATTEMPTS: u8 = 5;

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
// Public API
// ============================================================================

/// Extract the 24-byte private key from a real u-blox GNSS module.
///
/// TX: PIO1 SM0 on GPIO1 (to GNSS RX)
/// RX: Hardware UART1 on GPIO5 (from GNSS TX) with 1KB buffered RX
pub async fn extract(
    pio1: Peri<'static, PIO1>,
    tx_pin: Peri<'static, PIN_1>,
    uart1: Peri<'static, UART1>,
    rx_pin: Peri<'static, PIN_5>,
) -> Option<[u8; 24]> {
    let t0 = Instant::now();
    info!("========================================");
    info!("KEY EXTRACTION MODE ACTIVATED");
    info!("  baudrate: {}", DEFAULT_BAUDRATE);
    info!("  TX: GPIO1 (PIO1 SM0)");
    info!("  RX: GPIO5 (Hardware UART1, buffered)");
    info!("  attempts: {}", MAX_ATTEMPTS);
    info!("========================================");

    // ---- Initialize PIO1 TX on GPIO1 ----------------------------------------
    info!("KEY EXTRACT: initializing PIO1 TX...");
    let Pio { mut common, sm0, .. } = Pio::new(pio1, Pio1Irqs);
    let tx_program = PioUartTxProgram::new(&mut common);
    let mut uart_tx: PioUartTx<'_, PIO1, 0> =
        PioUartTx::new(DEFAULT_BAUDRATE, &mut common, sm0, tx_pin, &tx_program);

    // ---- Initialize Hardware UART1 RX on GPIO5 ------------------------------
    info!("KEY EXTRACT: initializing Hardware UART1 RX...");
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf = RX_BUF.init([0u8; 1024]);

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = DEFAULT_BAUDRATE;

    let mut uart_rx = BufferedUartRx::new(
        uart1,
        crate::Irqs,
        rx_pin,
        rx_buf,
        uart_config,
    );

    // Set FIFO RX threshold to 1/4 (same as passthrough mode)
    rp_pac::UART1.uartifls().write(|w| {
        w.set_rxiflsel(0b001); // 1/4 full (4 bytes)
        w.set_txiflsel(0b000);
    });
    info!("KEY EXTRACT: UART1 RX FIFO threshold set to 1/4");

    // Wait for GNSS module to stabilize
    info!("KEY EXTRACT: waiting 2s for GNSS module...");
    Timer::after(Duration::from_millis(2000)).await;

    // Flush any initial data
    let mut flush_buf = [0u8; 256];
    let mut flushed: usize = 0;
    loop {
        match with_timeout(Duration::from_millis(100), uart_rx.read(&mut flush_buf)).await {
            Ok(Ok(n)) => { flushed += n; }
            _ => break,
        }
    }
    info!("KEY EXTRACT: flushed {} initial bytes", flushed);

    // ---- Send + capture loop -----
    for attempt in 1..=MAX_ATTEMPTS {
        info!("KEY EXTRACT: === attempt {}/{} ===", attempt, MAX_ATTEMPTS);

        // Send CFG-0x41 poll via PIO TX
        info!("KEY EXTRACT: sending CFG-0x41 poll");
        for &b in &CFG41_POLL {
            uart_tx.write_u8(b).await;
        }

        // Read response into buffer with 2s timeout
        let mut raw = [0u8; 1024];
        let mut count: usize = 0;
        let t_rx = Instant::now();

        loop {
            if t_rx.elapsed().as_millis() >= 2000 { break; }
            if count >= raw.len() { break; }

            let remaining = raw.len() - count;
            match with_timeout(
                Duration::from_millis(500),
                uart_rx.read(&mut raw[count..count + remaining])
            ).await {
                Ok(Ok(n)) if n > 0 => {
                    count += n;
                }
                _ => {
                    if count > 0 { break; }
                }
            }
        }

        let rx_ms = t_rx.elapsed().as_millis();
        info!("KEY EXTRACT: captured {} bytes in {}ms", count, rx_ms);

        // Dump first 32 bytes
        if count >= 32 {
            info!("KEY EXTRACT: raw[0..32]: {:02x}", &raw[..32]);
        } else if count > 0 {
            info!("KEY EXTRACT: raw[0..{}]: {:02x}", count, &raw[..count]);
        }

        // Search for CFG-0x41 UBX frame in captured data
        if let Some(key) = find_cfg41_in_buffer(&raw[..count]) {
            let total_ms = t0.elapsed().as_millis();
            info!("========================================");
            info!("KEY EXTRACT: SUCCESS in {}ms (attempt {})", total_ms, attempt);
            log_key_full(&key);
            info!("========================================");
            return Some(key);
        }

        error!("KEY EXTRACT: CFG-0x41 not found in buffer (attempt {})", attempt);
    }

    let total_ms = t0.elapsed().as_millis();
    error!("KEY EXTRACT: FAILED after {} attempts ({}ms total)", MAX_ATTEMPTS, total_ms);
    None
}

// ============================================================================
// Auto-extraction: silent key extraction at boot (no long-press needed)
// ============================================================================

const AUTO_GNSS_DETECT_TIMEOUT_MS: u64 = 500;
const AUTO_POLL_TIMEOUT_MS: u64 = 500;
const AUTO_MAX_ATTEMPTS: u8 = 2;

/// Auto-extract key at boot: detect GNSS via NMEA, poll CFG-0x41, extract key.
/// Returns None if no GNSS connected (timeout) or key not found.
/// Must be called BEFORE main UART init — GPIO1 and GPIO5 must be free.
pub async fn auto_extract(
    pio1: Peri<'static, PIO1>,
    tx_pin: Peri<'static, PIN_1>,
    uart1: Peri<'static, UART1>,
    rx_pin: Peri<'static, PIN_5>,
) -> Option<[u8; 24]> {
    let t0 = Instant::now();
    info!("AUTO KEY: attempting silent key extraction...");

    // ---- Initialize PIO1 TX on GPIO1 ----------------------------------------
    let Pio { mut common, sm0, .. } = Pio::new(pio1, Pio1Irqs);
    let tx_program = PioUartTxProgram::new(&mut common);
    let mut uart_tx: PioUartTx<'_, PIO1, 0> =
        PioUartTx::new(DEFAULT_BAUDRATE, &mut common, sm0, tx_pin, &tx_program);

    // ---- Initialize Hardware UART1 RX on GPIO5 ------------------------------
    static AUTO_RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf = AUTO_RX_BUF.init([0u8; 1024]);

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = DEFAULT_BAUDRATE;

    let mut uart_rx = BufferedUartRx::new(
        uart1,
        crate::Irqs,
        rx_pin,
        rx_buf,
        uart_config,
    );

    // Set FIFO RX threshold to 1/4
    rp_pac::UART1.uartifls().write(|w| {
        w.set_rxiflsel(0b001);
        w.set_txiflsel(0b000);
    });

    // ---- NMEA detection: wait for any data from GNSS module -----------------
    let mut detect_buf = [0u8; 64];
    match with_timeout(
        Duration::from_millis(AUTO_GNSS_DETECT_TIMEOUT_MS),
        uart_rx.read(&mut detect_buf),
    ).await {
        Ok(Ok(n)) => {
            info!("AUTO KEY: GNSS detected ({} bytes in {}ms)", n, t0.elapsed().as_millis());
        }
        _ => {
            info!("AUTO KEY: no GNSS detected ({}ms timeout), skipping", AUTO_GNSS_DETECT_TIMEOUT_MS);
            return None;
        }
    }

    // Flush remaining NMEA data
    let mut flush_buf = [0u8; 256];
    loop {
        match with_timeout(Duration::from_millis(100), uart_rx.read(&mut flush_buf)).await {
            Ok(Ok(_)) => {}
            _ => break,
        }
    }

    // ---- Send CFG-0x41 poll + capture response ------------------------------
    for attempt in 1..=AUTO_MAX_ATTEMPTS {
        info!("AUTO KEY: attempt {}/{}", attempt, AUTO_MAX_ATTEMPTS);

        // Send CFG-0x41 poll via PIO TX
        for &b in &CFG41_POLL {
            uart_tx.write_u8(b).await;
        }

        // Read response
        let mut raw = [0u8; 1024];
        let mut count: usize = 0;
        let t_rx = Instant::now();

        loop {
            if t_rx.elapsed().as_millis() >= AUTO_POLL_TIMEOUT_MS { break; }
            if count >= raw.len() { break; }

            let remaining = raw.len() - count;
            match with_timeout(
                Duration::from_millis(200),
                uart_rx.read(&mut raw[count..count + remaining]),
            ).await {
                Ok(Ok(n)) if n > 0 => {
                    count += n;
                }
                _ => {
                    if count > 0 { break; }
                }
            }
        }

        info!("AUTO KEY: captured {} bytes in {}ms", count, t_rx.elapsed().as_millis());

        if let Some(key) = find_cfg41_in_buffer(&raw[..count]) {
            let total_ms = t0.elapsed().as_millis();
            info!("AUTO KEY: SUCCESS in {}ms (attempt {})", total_ms, attempt);
            return Some(key);
        }

        warn!("AUTO KEY: CFG-0x41 not found (attempt {})", attempt);
    }

    let total_ms = t0.elapsed().as_millis();
    warn!("AUTO KEY: failed after {} attempts ({}ms)", AUTO_MAX_ATTEMPTS, total_ms);
    None
}

// ============================================================================
// Offline buffer search — find CFG-0x41 in raw captured bytes
// ============================================================================

fn find_cfg41_in_buffer(data: &[u8]) -> Option<[u8; KEY_LEN]> {
    let min_frame_len = 6 + EXPECTED_PAYLOAD_LEN + 2; // 264

    for i in 0..data.len().saturating_sub(min_frame_len) {
        if data[i] != 0xB5 || data[i + 1] != 0x62 {
            continue;
        }
        let class = data[i + 2];
        let id = data[i + 3];
        let len = u16::from_le_bytes([data[i + 4], data[i + 5]]) as usize;

        if class != 0x06 || id != 0x41 || len != EXPECTED_PAYLOAD_LEN {
            continue;
        }

        let frame_end = i + 6 + len + 2;
        if frame_end > data.len() {
            warn!("KEY EXTRACT: CFG-0x41 header at [{}] but buffer too short ({} need {})",
                i, data.len(), frame_end);
            continue;
        }

        info!("KEY EXTRACT: CFG-0x41 frame at offset [{}]", i);

        // Validate Fletcher-8 checksum
        let checksum_data = &data[i + 2..i + 6 + len];
        let (exp_a, exp_b) = crate::ubx::calculate_checksum(checksum_data);
        let got_a = data[i + 6 + len];
        let got_b = data[i + 6 + len + 1];

        if got_a != exp_a || got_b != exp_b {
            error!("KEY EXTRACT: checksum FAIL at [{}]: got={:#04x},{:#04x} exp={:#04x},{:#04x}",
                i, got_a, got_b, exp_a, exp_b);
            continue;
        }

        info!("KEY EXTRACT: checksum OK");
        let payload = &data[i + 6..i + 6 + len];
        dump_payload(payload);
        return extract_key(payload);
    }

    None
}

// ============================================================================
// Key extraction from payload
// ============================================================================

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

    if let Some(key) = try_key_at(payload, KEY_HEADER_OFFSET_AIR3S, KEY_DATA_OFFSET_AIR3S, "Air3S/Mavic3Pro") {
        return Some(key);
    }
    if let Some(key) = try_key_at(payload, KEY_HEADER_OFFSET_AIR3, KEY_DATA_OFFSET_AIR3, "Air3/Mavic4Pro") {
        return Some(key);
    }

    // Fallback: scan for A6 18 anywhere
    warn!("KEY EXTRACT: known offsets miss, scanning payload for A6 18...");
    for i in 0..payload.len().saturating_sub(1 + KEY_LEN) {
        if payload[i] == SEC_KEY_TAG && payload[i + 1] == SEC_KEY_LEN {
            let data_start = i + 2;
            let data_end = data_start + KEY_LEN;
            if data_end > payload.len() { continue; }
            let mut key = [0u8; KEY_LEN];
            key.copy_from_slice(&payload[data_start..data_end]);
            if validate_key_bytes(&key) {
                info!("KEY EXTRACT: key via scan at payload[{}..{}]", data_start, data_end);
                log_key_full(&key);
                return Some(key);
            }
        }
    }

    error!("KEY EXTRACT: private key not found in payload");
    None
}

fn try_key_at(payload: &[u8], header_offset: usize, data_offset: usize, name: &str) -> Option<[u8; KEY_LEN]> {
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
        warn!("KEY EXTRACT: {} header matched but key invalid", name);
        return None;
    }
    info!("KEY EXTRACT: {} layout, key OK", name);
    log_key_full(&key);
    Some(key)
}

fn validate_key_bytes(key: &[u8; KEY_LEN]) -> bool {
    !key.iter().all(|&b| b == 0x00) && !key.iter().all(|&b| b == 0xFF)
}

fn log_key_full(key: &[u8; KEY_LEN]) {
    info!("KEY EXTRACT: key[0..12]:  {:02x}", &key[..12]);
    info!("KEY EXTRACT: key[12..24]: {:02x}", &key[12..24]);
}
