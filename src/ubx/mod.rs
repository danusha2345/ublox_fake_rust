//! UBX protocol implementation

mod messages;
mod parser;

pub use messages::*;
pub use parser::*;

/// UBX sync bytes
pub const SYNC1: u8 = 0xB5;
pub const SYNC2: u8 = 0x62;

/// UBX message classes
#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Class {
    Nav = 0x01,
    Rxm = 0x02,
    Inf = 0x04,
    Ack = 0x05,
    Cfg = 0x06,
    Upd = 0x09,
    Mon = 0x0A,
    Tim = 0x0D,
    Sec = 0x27,
}

/// UBX message IDs for NAV class
#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum NavId {
    Posecef = 0x01,
    Posllh = 0x02,
    Status = 0x03,
    Dop = 0x04,
    Pvt = 0x07,
    Velned = 0x12,
    Velecef = 0x11,
    Timegps = 0x20,
    Timeutc = 0x21,
    Clock = 0x22,
    Timels = 0x26,
    Svinfo = 0x30,
    Sat = 0x35,
    Cov = 0x36,
    Hpposecef = 0x13,
    Aopstatus = 0x60,
    Eoe = 0x61,
}

/// UBX message IDs for CFG class
#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum CfgId {
    Prt = 0x00,
    Msg = 0x01,
    Rate = 0x08,
    Cfg = 0x09,
    Nav5 = 0x24,
    Navx5 = 0x23,
    Gnss = 0x3E,
    Pms = 0x86,
    Valset = 0x8A,
}

/// Calculate UBX Fletcher checksum
pub fn calculate_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;

    for &byte in data {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    (ck_a, ck_b)
}

/// Trait for UBX messages
pub trait UbxMessage {
    /// Get the message class
    fn class(&self) -> u8;

    /// Get the message ID
    fn id(&self) -> u8;

    /// Get the payload length
    fn payload_len(&self) -> u16;

    /// Write payload to buffer, returns bytes written
    fn write_payload(&self, buf: &mut [u8]) -> usize;

    /// Build complete UBX frame into buffer, returns total bytes
    fn build(&self, buf: &mut [u8]) -> usize {
        let payload_len = self.payload_len() as usize;
        let total_len = 6 + payload_len + 2; // sync(2) + class(1) + id(1) + len(2) + payload + cksum(2)

        if buf.len() < total_len {
            return 0;
        }

        buf[0] = SYNC1;
        buf[1] = SYNC2;
        buf[2] = self.class();
        buf[3] = self.id();
        buf[4] = (payload_len & 0xFF) as u8;
        buf[5] = ((payload_len >> 8) & 0xFF) as u8;

        self.write_payload(&mut buf[6..6 + payload_len]);

        // Calculate checksum over class, id, length, and payload
        let (ck_a, ck_b) = calculate_checksum(&buf[2..6 + payload_len]);
        buf[6 + payload_len] = ck_a;
        buf[7 + payload_len] = ck_b;

        total_len
    }
}

/// Message enable flags
#[derive(Clone, Copy, Default)]
pub struct MessageFlags {
    // NAV messages (5Hz)
    pub nav_pvt: bool,
    pub nav_posecef: bool,
    pub nav_posllh: bool,
    pub nav_status: bool,
    pub nav_dop: bool,
    pub nav_sol: bool,       // NAV-SOL (0x01 0x06) - legacy M8, important for many FCs
    pub nav_velned: bool,
    pub nav_velecef: bool,
    pub nav_timeutc: bool,
    pub nav_timegps: bool,
    pub nav_timels: bool,
    pub nav_clock: bool,
    pub nav_sat: bool,
    pub nav_svinfo: bool,
    pub nav_cov: bool,
    pub nav_hpposecef: bool,
    pub nav_aopstatus: bool,
    pub nav_eoe: bool,

    // RXM messages
    pub rxm_rawx: bool,

    // MON messages (1Hz)
    pub mon_hw: bool,
    pub mon_comms: bool,
    pub mon_rf: bool,

    // TIM messages
    pub tim_tp: bool,
}

impl MessageFlags {
    /// Const constructor - all messages disabled by default
    /// Messages are enabled only after receiving CFG-MSG commands
    pub const fn new_default() -> Self {
        Self {
            nav_pvt: false,
            nav_posecef: false,
            nav_posllh: false,
            nav_status: false,
            nav_dop: false,
            nav_sol: false,
            nav_velned: false,
            nav_velecef: false,
            nav_timeutc: false,
            nav_timegps: false,
            nav_timels: false,
            nav_clock: false,
            nav_sat: false,
            nav_svinfo: false,
            nav_cov: false,
            nav_hpposecef: false,
            nav_aopstatus: false,
            nav_eoe: false,
            rxm_rawx: false,
            mon_hw: false,
            mon_comms: false,
            mon_rf: false,
            tim_tp: false,
        }
    }

    /// Update a single message flag by class/id
    pub fn set_message(&mut self, class: u8, id: u8, enabled: bool) {
        match (class, id) {
            (0x01, 0x07) => self.nav_pvt = enabled,
            (0x01, 0x01) => self.nav_posecef = enabled,
            (0x01, 0x02) => self.nav_posllh = enabled,
            (0x01, 0x03) => self.nav_status = enabled,
            (0x01, 0x04) => self.nav_dop = enabled,
            (0x01, 0x06) => self.nav_sol = enabled,
            (0x01, 0x12) => self.nav_velned = enabled,
            (0x01, 0x11) => self.nav_velecef = enabled,
            (0x01, 0x21) => self.nav_timeutc = enabled,
            (0x01, 0x20) => self.nav_timegps = enabled,
            (0x01, 0x26) => self.nav_timels = enabled,
            (0x01, 0x22) => self.nav_clock = enabled,
            (0x01, 0x35) => self.nav_sat = enabled,
            (0x01, 0x30) => self.nav_svinfo = enabled,
            (0x01, 0x36) => self.nav_cov = enabled,
            (0x01, 0x13) => self.nav_hpposecef = enabled,
            (0x01, 0x60) => self.nav_aopstatus = enabled,
            (0x01, 0x61) => self.nav_eoe = enabled,
            (0x02, 0x15) => self.rxm_rawx = enabled,
            (0x0A, 0x09) => self.mon_hw = enabled,
            (0x0A, 0x36) => self.mon_comms = enabled,
            (0x0A, 0x38) => self.mon_rf = enabled,
            (0x0D, 0x01) => self.tim_tp = enabled,
            _ => {}
        }
    }
}

