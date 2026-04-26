# Implemented UBX Messages

## NAV Class (0x01)
| ID | Name | Payload | Description |
|----|------|---------|-------------|
| 0x01 | NAV-POSECEF | 20 | Position ECEF |
| 0x02 | NAV-POSLLH | 28 | Position LLH |
| 0x03 | NAV-STATUS | 16 | Receiver status |
| 0x04 | NAV-DOP | 18 | Dilution of precision |
| 0x06 | NAV-SOL | 52 | Nav solution (legacy) |
| 0x07 | NAV-PVT | 92 | Position/Velocity/Time |
| 0x11 | NAV-VELECEF | 20 | Velocity ECEF |
| 0x12 | NAV-VELNED | 36 | Velocity NED |
| 0x13 | NAV-HPPOSECEF | 28 | High precision ECEF |
| 0x20 | NAV-TIMEGPS | 16 | GPS time |
| 0x21 | NAV-TIMEUTC | 20 | UTC time |
| 0x22 | NAV-CLOCK | 20 | Clock solution |
| 0x26 | NAV-TIMELS | 24 | Leap second |
| 0x30 | NAV-SVINFO | 8+12n | Sat info (legacy) |
| 0x35 | NAV-SAT | 8+12n | Sat info (M10) |
| 0x36 | NAV-COV | 64 | Covariance |
| 0x60 | NAV-AOPSTATUS | 16 | AssistNow |
| 0x61 | NAV-EOE | 4 | End of epoch |

## Other Classes
| Class:ID | Name | Description |
|----------|------|-------------|
| 02:13 | RXM-SFRBX | Subframe buffer |
| 02:15 | RXM-RAWX | Raw measurements |
| 05:00/01 | ACK-NAK/ACK | Acknowledgement |
| 06:41 | CFG-0x41 | DJI proprietary (private key) |
| 0A:04/09/36/38 | MON-* | Monitoring |
| 0D:01 | TIM-TP | Timepulse |
| 13:* | MGA-* | AssistNow data |
| 27:03 | SEC-UNIQID | Unique ID |
| 27:04 | SEC-SIGN | Signature (108B) |
