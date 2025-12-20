//! UBX protocol parser

use super::{calculate_checksum, SYNC1, SYNC2};

/// Parsed UBX command from host
#[derive(Clone, Debug)]
pub enum UbxCommand {
    /// CFG-PRT: Port configuration
    CfgPrt { baudrate: u32 },

    /// CFG-MSG: Message configuration
    CfgMsg { class: u8, id: u8, rate: u8 },

    /// CFG-RATE: Navigation rate
    CfgRate { meas_rate: u16, nav_rate: u16, time_ref: u16 },

    /// CFG-VALSET: Value set (M10)
    CfgValset { layer: u8, keys: heapless::Vec<(u32, u32), 16> },

    /// Poll request
    Poll { class: u8, id: u8 },

    /// Unknown command
    Unknown,
}

/// UBX parser state machine
pub struct UbxParser {
    state: ParserState,
    class: u8,
    id: u8,
    len: u16,
    payload: [u8; 256],
    payload_idx: usize,
    ck_a: u8,
    ck_b: u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ParserState {
    Sync1,
    Sync2,
    Class,
    Id,
    LenLow,
    LenHigh,
    Payload,
    CkA,
    CkB,
}

impl UbxParser {
    pub fn new() -> Self {
        Self {
            state: ParserState::Sync1,
            class: 0,
            id: 0,
            len: 0,
            payload: [0; 256],
            payload_idx: 0,
            ck_a: 0,
            ck_b: 0,
        }
    }

    /// Reset parser state
    pub fn reset(&mut self) {
        self.state = ParserState::Sync1;
        self.payload_idx = 0;
    }

    /// Parse a single byte, returns command if complete
    pub fn parse_byte(&mut self, byte: u8) -> Option<UbxCommand> {
        match self.state {
            ParserState::Sync1 => {
                if byte == SYNC1 {
                    self.state = ParserState::Sync2;
                }
            }
            ParserState::Sync2 => {
                if byte == SYNC2 {
                    self.state = ParserState::Class;
                } else {
                    self.reset();
                }
            }
            ParserState::Class => {
                self.class = byte;
                self.state = ParserState::Id;
            }
            ParserState::Id => {
                self.id = byte;
                self.state = ParserState::LenLow;
            }
            ParserState::LenLow => {
                self.len = byte as u16;
                self.state = ParserState::LenHigh;
            }
            ParserState::LenHigh => {
                self.len |= (byte as u16) << 8;
                self.payload_idx = 0;

                if self.len == 0 {
                    self.state = ParserState::CkA;
                } else if self.len > 256 {
                    // Payload too large
                    self.reset();
                } else {
                    self.state = ParserState::Payload;
                }
            }
            ParserState::Payload => {
                if self.payload_idx < self.len as usize {
                    self.payload[self.payload_idx] = byte;
                    self.payload_idx += 1;
                }

                if self.payload_idx >= self.len as usize {
                    self.state = ParserState::CkA;
                }
            }
            ParserState::CkA => {
                self.ck_a = byte;
                self.state = ParserState::CkB;
            }
            ParserState::CkB => {
                self.ck_b = byte;

                // Verify checksum
                let mut data = [0u8; 260];
                data[0] = self.class;
                data[1] = self.id;
                data[2] = (self.len & 0xFF) as u8;
                data[3] = ((self.len >> 8) & 0xFF) as u8;
                data[4..4 + self.len as usize].copy_from_slice(&self.payload[..self.len as usize]);

                let (calc_a, calc_b) = calculate_checksum(&data[..4 + self.len as usize]);

                self.reset();

                if calc_a == self.ck_a && calc_b == self.ck_b {
                    return Some(self.decode_command());
                }
            }
        }

        None
    }

    /// Decode parsed message into command
    fn decode_command(&self) -> UbxCommand {
        match (self.class, self.id) {
            // CFG-PRT
            (0x06, 0x00) if self.len >= 20 => {
                let baudrate = u32::from_le_bytes([
                    self.payload[8],
                    self.payload[9],
                    self.payload[10],
                    self.payload[11],
                ]);
                UbxCommand::CfgPrt { baudrate }
            }

            // CFG-MSG (short form: 3 bytes)
            (0x06, 0x01) if self.len == 3 => {
                UbxCommand::CfgMsg {
                    class: self.payload[0],
                    id: self.payload[1],
                    rate: self.payload[2],
                }
            }

            // CFG-MSG (long form: 8 bytes)
            (0x06, 0x01) if self.len == 8 => {
                UbxCommand::CfgMsg {
                    class: self.payload[0],
                    id: self.payload[1],
                    rate: self.payload[2], // Port 0 rate
                }
            }

            // CFG-RATE
            (0x06, 0x08) if self.len == 6 => {
                let meas_rate = u16::from_le_bytes([self.payload[0], self.payload[1]]);
                let nav_rate = u16::from_le_bytes([self.payload[2], self.payload[3]]);
                let time_ref = u16::from_le_bytes([self.payload[4], self.payload[5]]);
                UbxCommand::CfgRate { meas_rate, nav_rate, time_ref }
            }

            // CFG-VALSET (M10)
            (0x06, 0x8A) if self.len >= 4 => {
                let layer = self.payload[1];
                // Parse key-value pairs (simplified)
                let keys = heapless::Vec::new();
                UbxCommand::CfgValset { layer, keys }
            }

            // Poll requests (zero length)
            (class, id) if self.len == 0 => {
                UbxCommand::Poll { class, id }
            }

            _ => UbxCommand::Unknown,
        }
    }
}

impl Default for UbxParser {
    fn default() -> Self {
        Self::new()
    }
}
