# SEC-SIGN Protocol Details

## Algorithm
SHA256 over all transmitted UBX (except SEC-SIGN itself) → fold 32→24 bytes → ECDSA SECP192R1 sign with deterministic k (HMAC-SHA256, simplified RFC6979). Key crates: `p192`, `sha2`, `hmac`.

## Drone Models
| Model | ID | First Delay | Period |
|-------|-----|-------------|--------|
| Air 3 | 0 | 1000ms | 2s |
| Mavic 4 Pro | 1 | 650ms | 2s |
| Air 3S | 2 | 650ms | 2s |
| Mavic 3 Pro | 3 | 650ms | 2s |

Default: `DRONE_MODEL=2` (Air 3S). Mavic 4 Pro: per-unit unique keys (3 known).
Private keys location: `src/sec_sign.rs`

## NAV Output Start Timing
| Model | Delay from first UBX cmd | First SEC-SIGN delay |
|-------|--------------------------|----------------------|
| Air 3 | 700ms | 1000ms |
| Air 3S | 780ms | 650ms |
| Mavic 4 Pro | 400ms | 650ms |
| Mavic 3 Pro | 780ms | 650ms |

## TX Pause Mechanism
`SEC_SIGN_IN_PROGRESS` atomic flag coordinates TX during ECDSA (~59ms):
1. `sec_sign_timer_task`: set flag → capture hash → try_send() to Core1
2. `nav/mon_message_task`: yield_now() loop until flag clear
3. `uart0_tx_task`: two-layer guard — pre-check + post-select3 drop
4. `sec_sign_compute_task` (Core1): try_send() result back

Packet drop: ~0.1% (59 RXM-RAWX + 6 NAV-PVT per 20min/597 cycles).
Non-blocking channels prevent deadlocks. Stuck flag guard: no clear on re-entry.

## SEC-SIGN Timer Start (Passthrough)
Triggers on FIRST_CONFIG_MILLIS (first drone UBX command, ~15ms) with 2s fallback.
After 20s from NAV start, satellites invalidate (fix_type=0, num_sv=1).
