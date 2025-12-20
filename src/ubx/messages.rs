//! UBX message definitions

use super::UbxMessage;

/// UBX-NAV-PVT message (Position Velocity Time)
#[derive(Clone)]
pub struct NavPvt {
    pub itow: u32,      // GPS time of week (ms)
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub valid: u8,
    pub t_acc: u32,
    pub nano: i32,
    pub fix_type: u8,
    pub flags: u8,
    pub flags2: u8,
    pub num_sv: u8,
    pub lon: i32,       // deg * 1e-7
    pub lat: i32,       // deg * 1e-7
    pub height: i32,    // mm above ellipsoid
    pub h_msl: i32,     // mm above MSL
    pub h_acc: u32,
    pub v_acc: u32,
    pub vel_n: i32,     // mm/s
    pub vel_e: i32,
    pub vel_d: i32,
    pub g_speed: i32,   // mm/s
    pub head_mot: i32,  // deg * 1e-5
    pub s_acc: u32,
    pub head_acc: u32,
    pub p_dop: u16,
    pub flags3: u8,
    pub reserved1: [u8; 5],
    pub head_veh: i32,
    pub mag_dec: i16,
    pub mag_acc: u16,
}

impl Default for NavPvt {
    fn default() -> Self {
        Self {
            itow: 0,
            year: 2025,
            month: 1,
            day: 1,
            hour: 12,
            min: 0,
            sec: 0,
            valid: 0x37,  // Valid date, time, fully resolved
            t_acc: 20,
            nano: 0,
            fix_type: 3,  // 3D fix
            flags: 0x01,  // gnssFixOK
            flags2: 0x20,
            num_sv: 12,
            lon: 376184230,   // 37.618423 * 1e7
            lat: 557611990,   // 55.761199 * 1e7
            height: 156000,
            h_msl: 156000,
            h_acc: 5000,
            v_acc: 8000,
            vel_n: 0,
            vel_e: 0,
            vel_d: 0,
            g_speed: 0,
            head_mot: 0,
            s_acc: 100,
            head_acc: 500000,
            p_dop: 120,
            flags3: 0,
            reserved1: [0; 5],
            head_veh: 0,
            mag_dec: 0,
            mag_acc: 0,
        }
    }
}

impl UbxMessage for NavPvt {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x07 }
    fn payload_len(&self) -> u16 { 92 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..6].copy_from_slice(&self.year.to_le_bytes());
        buf[6] = self.month;
        buf[7] = self.day;
        buf[8] = self.hour;
        buf[9] = self.min;
        buf[10] = self.sec;
        buf[11] = self.valid;
        buf[12..16].copy_from_slice(&self.t_acc.to_le_bytes());
        buf[16..20].copy_from_slice(&self.nano.to_le_bytes());
        buf[20] = self.fix_type;
        buf[21] = self.flags;
        buf[22] = self.flags2;
        buf[23] = self.num_sv;
        buf[24..28].copy_from_slice(&self.lon.to_le_bytes());
        buf[28..32].copy_from_slice(&self.lat.to_le_bytes());
        buf[32..36].copy_from_slice(&self.height.to_le_bytes());
        buf[36..40].copy_from_slice(&self.h_msl.to_le_bytes());
        buf[40..44].copy_from_slice(&self.h_acc.to_le_bytes());
        buf[44..48].copy_from_slice(&self.v_acc.to_le_bytes());
        buf[48..52].copy_from_slice(&self.vel_n.to_le_bytes());
        buf[52..56].copy_from_slice(&self.vel_e.to_le_bytes());
        buf[56..60].copy_from_slice(&self.vel_d.to_le_bytes());
        buf[60..64].copy_from_slice(&self.g_speed.to_le_bytes());
        buf[64..68].copy_from_slice(&self.head_mot.to_le_bytes());
        buf[68..72].copy_from_slice(&self.s_acc.to_le_bytes());
        buf[72..76].copy_from_slice(&self.head_acc.to_le_bytes());
        buf[76..78].copy_from_slice(&self.p_dop.to_le_bytes());
        buf[78] = self.flags3;
        buf[79..84].copy_from_slice(&self.reserved1);
        buf[84..88].copy_from_slice(&self.head_veh.to_le_bytes());
        buf[88..90].copy_from_slice(&self.mag_dec.to_le_bytes());
        buf[90..92].copy_from_slice(&self.mag_acc.to_le_bytes());
        92
    }
}

/// UBX-NAV-POSECEF message
#[derive(Clone, Default)]
pub struct NavPosecef {
    pub itow: u32,
    pub ecef_x: i32,  // cm
    pub ecef_y: i32,
    pub ecef_z: i32,
    pub p_acc: u32,   // cm
}

impl UbxMessage for NavPosecef {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x01 }
    fn payload_len(&self) -> u16 { 20 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.ecef_x.to_le_bytes());
        buf[8..12].copy_from_slice(&self.ecef_y.to_le_bytes());
        buf[12..16].copy_from_slice(&self.ecef_z.to_le_bytes());
        buf[16..20].copy_from_slice(&self.p_acc.to_le_bytes());
        20
    }
}

/// UBX-NAV-POSLLH message
#[derive(Clone, Default)]
pub struct NavPosllh {
    pub itow: u32,
    pub lon: i32,     // deg * 1e-7
    pub lat: i32,     // deg * 1e-7
    pub height: i32,  // mm
    pub h_msl: i32,   // mm
    pub h_acc: u32,   // mm
    pub v_acc: u32,   // mm
}

impl UbxMessage for NavPosllh {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x02 }
    fn payload_len(&self) -> u16 { 28 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.lon.to_le_bytes());
        buf[8..12].copy_from_slice(&self.lat.to_le_bytes());
        buf[12..16].copy_from_slice(&self.height.to_le_bytes());
        buf[16..20].copy_from_slice(&self.h_msl.to_le_bytes());
        buf[20..24].copy_from_slice(&self.h_acc.to_le_bytes());
        buf[24..28].copy_from_slice(&self.v_acc.to_le_bytes());
        28
    }
}

/// UBX-NAV-STATUS message
#[derive(Clone)]
pub struct NavStatus {
    pub itow: u32,
    pub gps_fix: u8,
    pub flags: u8,
    pub fix_stat: u8,
    pub flags2: u8,
    pub ttff: u32,
    pub msss: u32,
}

impl Default for NavStatus {
    fn default() -> Self {
        Self {
            itow: 0,
            gps_fix: 3,  // 3D fix
            flags: 0x0D,
            fix_stat: 0x00,
            flags2: 0x08,
            ttff: 25000,
            msss: 0,
        }
    }
}

impl UbxMessage for NavStatus {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x03 }
    fn payload_len(&self) -> u16 { 16 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4] = self.gps_fix;
        buf[5] = self.flags;
        buf[6] = self.fix_stat;
        buf[7] = self.flags2;
        buf[8..12].copy_from_slice(&self.ttff.to_le_bytes());
        buf[12..16].copy_from_slice(&self.msss.to_le_bytes());
        16
    }
}

/// UBX-NAV-DOP message (Dilution of Precision)
#[derive(Clone)]
pub struct NavDop {
    pub itow: u32,
    pub g_dop: u16,
    pub p_dop: u16,
    pub t_dop: u16,
    pub v_dop: u16,
    pub h_dop: u16,
    pub n_dop: u16,
    pub e_dop: u16,
}

impl Default for NavDop {
    fn default() -> Self {
        Self {
            itow: 0,
            g_dop: 150,
            p_dop: 120,
            t_dop: 80,
            v_dop: 100,
            h_dop: 80,
            n_dop: 70,
            e_dop: 60,
        }
    }
}

impl UbxMessage for NavDop {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x04 }
    fn payload_len(&self) -> u16 { 18 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..6].copy_from_slice(&self.g_dop.to_le_bytes());
        buf[6..8].copy_from_slice(&self.p_dop.to_le_bytes());
        buf[8..10].copy_from_slice(&self.t_dop.to_le_bytes());
        buf[10..12].copy_from_slice(&self.v_dop.to_le_bytes());
        buf[12..14].copy_from_slice(&self.h_dop.to_le_bytes());
        buf[14..16].copy_from_slice(&self.n_dop.to_le_bytes());
        buf[16..18].copy_from_slice(&self.e_dop.to_le_bytes());
        18
    }
}

/// UBX-NAV-EOE message (End of Epoch)
#[derive(Clone, Default)]
pub struct NavEoe {
    pub itow: u32,
}

impl UbxMessage for NavEoe {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x61 }
    fn payload_len(&self) -> u16 { 4 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        4
    }
}

/// UBX-ACK-ACK message
#[derive(Clone)]
pub struct AckAck {
    pub cls_id: u8,
    pub msg_id: u8,
}

impl UbxMessage for AckAck {
    fn class(&self) -> u8 { 0x05 }
    fn id(&self) -> u8 { 0x01 }
    fn payload_len(&self) -> u16 { 2 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.cls_id;
        buf[1] = self.msg_id;
        2
    }
}

/// UBX-SEC-SIGN message
#[derive(Clone)]
pub struct SecSign {
    pub version: u8,
    pub reserved1: u8,
    pub msg_cnt: u16,
    pub sha256_hash: [u8; 32],
    pub session_id: [u8; 24],
    pub signature_r: [u8; 24],
    pub signature_s: [u8; 24],
}

impl Default for SecSign {
    fn default() -> Self {
        Self {
            version: 0x01,
            reserved1: 0,
            msg_cnt: 0,
            sha256_hash: [0; 32],
            session_id: [0; 24],
            signature_r: [0; 24],
            signature_s: [0; 24],
        }
    }
}

impl UbxMessage for SecSign {
    fn class(&self) -> u8 { 0x27 }
    fn id(&self) -> u8 { 0x01 }
    fn payload_len(&self) -> u16 { 108 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.version;
        buf[1] = self.reserved1;
        buf[2..4].copy_from_slice(&self.msg_cnt.to_le_bytes());
        buf[4..36].copy_from_slice(&self.sha256_hash);
        buf[36..60].copy_from_slice(&self.session_id);
        buf[60..84].copy_from_slice(&self.signature_r);
        buf[84..108].copy_from_slice(&self.signature_s);
        108
    }
}
