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
            flags2: 0x0E,  // Same as C version
            num_sv: 18,   // 9 GPS + 3 SBAS + 6 Galileo
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

/// UBX-ACK-NAK message (negative acknowledgement)
#[allow(dead_code)]
#[derive(Clone)]
pub struct AckNak {
    pub cls_id: u8,
    pub msg_id: u8,
}

impl UbxMessage for AckNak {
    fn class(&self) -> u8 { 0x05 }
    fn id(&self) -> u8 { 0x00 }  // NAK = 0x00, ACK = 0x01
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
    fn id(&self) -> u8 { 0x04 }
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

/// UBX-MON-VER message (160 byte payload)
/// Response to MON-VER poll request
#[derive(Clone)]
pub struct MonVer {
    /// Software version string (30 bytes, null-terminated)
    pub sw_version: [u8; 30],
    /// Hardware version string (10 bytes, null-terminated)
    pub hw_version: [u8; 10],
    /// Extended version strings (4 x 30 bytes)
    pub extensions: [[u8; 30]; 4],
}

impl Default for MonVer {
    fn default() -> Self {
        // M10: "ROM SPG 5.10 (7b202e)"
        let mut sw_version = [0u8; 30];
        let sw = b"ROM SPG 5.10 (7b202e)";
        sw_version[..sw.len()].copy_from_slice(sw);

        let mut hw_version = [0u8; 10];
        let hw = b"00190000";
        hw_version[..hw.len()].copy_from_slice(hw);

        // Extension strings
        let mut ext0 = [0u8; 30];
        let e0 = b"FWVER=SPG 5.10";
        ext0[..e0.len()].copy_from_slice(e0);

        let mut ext1 = [0u8; 30];
        let e1 = b"PROTVER=34.10";
        ext1[..e1.len()].copy_from_slice(e1);

        let mut ext2 = [0u8; 30];
        let e2 = b"GPS;GLO;GAL;BDS";
        ext2[..e2.len()].copy_from_slice(e2);

        let mut ext3 = [0u8; 30];
        let e3 = b"SBAS;QZSS";
        ext3[..e3.len()].copy_from_slice(e3);

        Self {
            sw_version,
            hw_version,
            extensions: [ext0, ext1, ext2, ext3],
        }
    }
}

impl UbxMessage for MonVer {
    fn class(&self) -> u8 { 0x0A }
    fn id(&self) -> u8 { 0x04 }
    fn payload_len(&self) -> u16 { 160 }  // 30 + 10 + 4*30 = 160

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..30].copy_from_slice(&self.sw_version);
        buf[30..40].copy_from_slice(&self.hw_version);
        buf[40..70].copy_from_slice(&self.extensions[0]);
        buf[70..100].copy_from_slice(&self.extensions[1]);
        buf[100..130].copy_from_slice(&self.extensions[2]);
        buf[130..160].copy_from_slice(&self.extensions[3]);
        160
    }
}

/// UBX-SEC-UNIQID message (10 byte payload)
/// Response to SEC-UNIQID poll request
#[derive(Clone)]
pub struct SecUniqid {
    /// Version (0x02 for M10)
    pub version: u8,
    /// Reserved bytes
    pub reserved: [u8; 3],
    /// Chip unique ID (5 bytes)
    pub unique_id: [u8; 5],
    /// Reserved byte
    pub reserved2: u8,
}

impl Default for SecUniqid {
    fn default() -> Self {
        Self {
            version: 0x02,
            reserved: [0x00, 0x00, 0x00],
            unique_id: [0xE0, 0x95, 0x65, 0x0F, 0x2A],  // Example chip ID
            reserved2: 0x54,
        }
    }
}

impl UbxMessage for SecUniqid {
    fn class(&self) -> u8 { 0x27 }
    fn id(&self) -> u8 { 0x03 }
    fn payload_len(&self) -> u16 { 10 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.version;
        buf[1..4].copy_from_slice(&self.reserved);
        buf[4..9].copy_from_slice(&self.unique_id);
        buf[9] = self.reserved2;
        10
    }
}

// ============================================================================
// Additional NAV messages
// ============================================================================

/// UBX-NAV-VELNED (0x01 0x12) - Velocity in NED frame
#[derive(Clone, Default)]
pub struct NavVelned {
    pub itow: u32,
    pub vel_n: i32,     // cm/s
    pub vel_e: i32,     // cm/s
    pub vel_d: i32,     // cm/s
    pub speed: u32,     // cm/s (3D speed)
    pub g_speed: u32,   // cm/s (ground speed)
    pub heading: i32,   // deg * 1e-5
    pub s_acc: u32,     // cm/s
    pub c_acc: u32,     // deg * 1e-5
}

impl UbxMessage for NavVelned {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x12 }
    fn payload_len(&self) -> u16 { 36 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.vel_n.to_le_bytes());
        buf[8..12].copy_from_slice(&self.vel_e.to_le_bytes());
        buf[12..16].copy_from_slice(&self.vel_d.to_le_bytes());
        buf[16..20].copy_from_slice(&self.speed.to_le_bytes());
        buf[20..24].copy_from_slice(&self.g_speed.to_le_bytes());
        buf[24..28].copy_from_slice(&self.heading.to_le_bytes());
        buf[28..32].copy_from_slice(&self.s_acc.to_le_bytes());
        buf[32..36].copy_from_slice(&self.c_acc.to_le_bytes());
        36
    }
}

/// UBX-NAV-VELECEF (0x01 0x11) - Velocity in ECEF coordinates
#[derive(Clone, Default)]
pub struct NavVelecef {
    pub itow: u32,
    pub ecef_vx: i32,   // cm/s
    pub ecef_vy: i32,   // cm/s
    pub ecef_vz: i32,   // cm/s
    pub s_acc: u32,     // cm/s
}

impl UbxMessage for NavVelecef {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x11 }
    fn payload_len(&self) -> u16 { 20 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.ecef_vx.to_le_bytes());
        buf[8..12].copy_from_slice(&self.ecef_vy.to_le_bytes());
        buf[12..16].copy_from_slice(&self.ecef_vz.to_le_bytes());
        buf[16..20].copy_from_slice(&self.s_acc.to_le_bytes());
        20
    }
}

/// UBX-NAV-TIMEUTC (0x01 0x21) - UTC Time Solution
#[derive(Clone)]
pub struct NavTimeutc {
    pub itow: u32,
    pub t_acc: u32,     // ns
    pub nano: i32,      // ns
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub valid: u8,
}

impl Default for NavTimeutc {
    fn default() -> Self {
        Self {
            itow: 0,
            t_acc: 20,
            nano: 0,
            year: 2025,
            month: 1,
            day: 1,
            hour: 12,
            min: 0,
            sec: 0,
            valid: 0xF7,  // UTC valid
        }
    }
}

impl UbxMessage for NavTimeutc {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x21 }
    fn payload_len(&self) -> u16 { 20 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.t_acc.to_le_bytes());
        buf[8..12].copy_from_slice(&self.nano.to_le_bytes());
        buf[12..14].copy_from_slice(&self.year.to_le_bytes());
        buf[14] = self.month;
        buf[15] = self.day;
        buf[16] = self.hour;
        buf[17] = self.min;
        buf[18] = self.sec;
        buf[19] = self.valid;
        20
    }
}

/// UBX-NAV-TIMEGPS (0x01 0x20) - GPS Time Solution
#[derive(Clone)]
pub struct NavTimegps {
    pub itow: u32,
    pub f_tow: i32,     // ns (fractional)
    pub week: i16,
    pub leap_s: i8,
    pub valid: u8,
    pub t_acc: u32,     // ns
}

impl Default for NavTimegps {
    fn default() -> Self {
        Self {
            itow: 0,
            f_tow: 0,
            week: 2349,  // Example GPS week
            leap_s: 18,
            valid: 0x07, // towValid|weekValid|leapSValid
            t_acc: 20,
        }
    }
}

impl UbxMessage for NavTimegps {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x20 }
    fn payload_len(&self) -> u16 { 16 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.f_tow.to_le_bytes());
        buf[8..10].copy_from_slice(&self.week.to_le_bytes());
        buf[10] = self.leap_s as u8;
        buf[11] = self.valid;
        buf[12..16].copy_from_slice(&self.t_acc.to_le_bytes());
        16
    }
}

/// UBX-NAV-CLOCK (0x01 0x22) - Clock Solution
#[derive(Clone, Default)]
pub struct NavClock {
    pub itow: u32,
    pub clk_b: i32,     // ns (clock bias)
    pub clk_d: i32,     // ns/s (clock drift)
    pub t_acc: u32,     // ns
    pub f_acc: u32,     // ps/s
}

impl UbxMessage for NavClock {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x22 }
    fn payload_len(&self) -> u16 { 20 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.clk_b.to_le_bytes());
        buf[8..12].copy_from_slice(&self.clk_d.to_le_bytes());
        buf[12..16].copy_from_slice(&self.t_acc.to_le_bytes());
        buf[16..20].copy_from_slice(&self.f_acc.to_le_bytes());
        20
    }
}

/// UBX-NAV-TIMELS (0x01 0x26) - Leap Second Event Information
#[derive(Clone)]
pub struct NavTimels {
    pub itow: u32,
    pub version: u8,
    pub reserved1: [u8; 3],
    pub src_of_curr_ls: u8,
    pub curr_ls: i8,
    pub src_of_ls_change: u8,
    pub ls_change: i8,
    pub time_to_ls_event: i32,
    pub date_of_ls_gps_wn: u16,
    pub date_of_ls_gps_dn: u16,
    pub reserved2: [u8; 3],
    pub valid: u8,
}

impl Default for NavTimels {
    fn default() -> Self {
        Self {
            itow: 0,
            version: 0x01,
            reserved1: [0; 3],
            src_of_curr_ls: 0x02,  // GPS
            curr_ls: 18,
            src_of_ls_change: 0x02,
            ls_change: 0,
            time_to_ls_event: 0,
            date_of_ls_gps_wn: 0,
            date_of_ls_gps_dn: 0,
            reserved2: [0; 3],
            valid: 0x03,
        }
    }
}

impl UbxMessage for NavTimels {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x26 }
    fn payload_len(&self) -> u16 { 24 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4] = self.version;
        buf[5..8].copy_from_slice(&self.reserved1);
        buf[8] = self.src_of_curr_ls;
        buf[9] = self.curr_ls as u8;
        buf[10] = self.src_of_ls_change;
        buf[11] = self.ls_change as u8;
        buf[12..16].copy_from_slice(&self.time_to_ls_event.to_le_bytes());
        buf[16..18].copy_from_slice(&self.date_of_ls_gps_wn.to_le_bytes());
        buf[18..20].copy_from_slice(&self.date_of_ls_gps_dn.to_le_bytes());
        buf[20..23].copy_from_slice(&self.reserved2);
        buf[23] = self.valid;
        24
    }
}

/// UBX-NAV-AOPSTATUS (0x01 0x60) - AssistNow Autonomous Status
#[derive(Clone)]
pub struct NavAopstatus {
    pub itow: u32,
    pub config: u8,
    pub status: u8,
    pub reserved1: [u8; 10],
}

impl Default for NavAopstatus {
    fn default() -> Self {
        Self {
            itow: 0,
            config: 0x01,  // AOP enabled
            status: 0x00,  // idle
            reserved1: [0; 10],
        }
    }
}

impl UbxMessage for NavAopstatus {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x60 }
    fn payload_len(&self) -> u16 { 16 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4] = self.config;
        buf[5] = self.status;
        buf[6..16].copy_from_slice(&self.reserved1);
        16
    }
}

/// UBX-NAV-COV (0x01 0x36) - Covariance Matrices
#[derive(Clone)]
pub struct NavCov {
    pub itow: u32,
    pub version: u8,
    pub pos_cov_valid: u8,
    pub vel_cov_valid: u8,
    pub reserved1: [u8; 9],
    pub pos_cov_nn: f32,
    pub pos_cov_ne: f32,
    pub pos_cov_nd: f32,
    pub pos_cov_ee: f32,
    pub pos_cov_ed: f32,
    pub pos_cov_dd: f32,
    pub vel_cov_nn: f32,
    pub vel_cov_ne: f32,
    pub vel_cov_nd: f32,
    pub vel_cov_ee: f32,
    pub vel_cov_ed: f32,
    pub vel_cov_dd: f32,
}

impl Default for NavCov {
    fn default() -> Self {
        Self {
            itow: 0,
            version: 0,
            pos_cov_valid: 0x03,
            vel_cov_valid: 0x03,
            reserved1: [0; 9],
            pos_cov_nn: 1.0,
            pos_cov_ne: 0.0,
            pos_cov_nd: 0.0,
            pos_cov_ee: 1.0,
            pos_cov_ed: 0.0,
            pos_cov_dd: 1.0,
            vel_cov_nn: 1.0,
            vel_cov_ne: 0.0,
            vel_cov_nd: 0.0,
            vel_cov_ee: 1.0,
            vel_cov_ed: 0.0,
            vel_cov_dd: 1.0,
        }
    }
}

impl UbxMessage for NavCov {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x36 }
    fn payload_len(&self) -> u16 { 64 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4] = self.version;
        buf[5] = self.pos_cov_valid;
        buf[6] = self.vel_cov_valid;
        buf[7..16].copy_from_slice(&self.reserved1);
        buf[16..20].copy_from_slice(&self.pos_cov_nn.to_le_bytes());
        buf[20..24].copy_from_slice(&self.pos_cov_ne.to_le_bytes());
        buf[24..28].copy_from_slice(&self.pos_cov_nd.to_le_bytes());
        buf[28..32].copy_from_slice(&self.pos_cov_ee.to_le_bytes());
        buf[32..36].copy_from_slice(&self.pos_cov_ed.to_le_bytes());
        buf[36..40].copy_from_slice(&self.pos_cov_dd.to_le_bytes());
        buf[40..44].copy_from_slice(&self.vel_cov_nn.to_le_bytes());
        buf[44..48].copy_from_slice(&self.vel_cov_ne.to_le_bytes());
        buf[48..52].copy_from_slice(&self.vel_cov_nd.to_le_bytes());
        buf[52..56].copy_from_slice(&self.vel_cov_ee.to_le_bytes());
        buf[56..60].copy_from_slice(&self.vel_cov_ed.to_le_bytes());
        buf[60..64].copy_from_slice(&self.vel_cov_dd.to_le_bytes());
        64
    }
}

/// UBX-NAV-HPPOSECEF (0x01 0x13) - High Precision Position ECEF
#[derive(Clone, Default)]
pub struct NavHpposecef {
    pub version: u8,
    pub reserved1: [u8; 3],
    pub itow: u32,
    pub ecef_x: i32,     // cm
    pub ecef_y: i32,     // cm
    pub ecef_z: i32,     // cm
    pub ecef_x_hp: i8,   // 0.1mm
    pub ecef_y_hp: i8,   // 0.1mm
    pub ecef_z_hp: i8,   // 0.1mm
    pub flags: u8,
    pub p_acc: u32,      // 0.1mm
}

impl UbxMessage for NavHpposecef {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x13 }
    fn payload_len(&self) -> u16 { 28 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.version;
        buf[1..4].copy_from_slice(&self.reserved1);
        buf[4..8].copy_from_slice(&self.itow.to_le_bytes());
        buf[8..12].copy_from_slice(&self.ecef_x.to_le_bytes());
        buf[12..16].copy_from_slice(&self.ecef_y.to_le_bytes());
        buf[16..20].copy_from_slice(&self.ecef_z.to_le_bytes());
        buf[20] = self.ecef_x_hp as u8;
        buf[21] = self.ecef_y_hp as u8;
        buf[22] = self.ecef_z_hp as u8;
        buf[23] = self.flags;
        buf[24..28].copy_from_slice(&self.p_acc.to_le_bytes());
        28
    }
}

/// Satellite info entry for NAV-SAT
#[derive(Clone, Copy, Default)]
pub struct SatInfo {
    pub gnss_id: u8,
    pub sv_id: u8,
    pub cno: u8,        // dBHz
    pub elev: i8,       // degrees
    pub azim: i16,      // degrees
    pub pr_res: i16,    // 0.1m
    pub flags: u32,
}

/// UBX-NAV-SAT (0x01 0x35) - Satellite Information (M10)
/// 18 satellites: 9 GPS + 3 SBAS + 6 Galileo (from real GNSS log)
#[derive(Clone)]
pub struct NavSat {
    pub itow: u32,
    pub version: u8,
    pub num_svs: u8,
    pub reserved1: [u8; 2],
    pub sats: [SatInfo; 18],  // 18 satellites
}

impl Default for NavSat {
    fn default() -> Self {
        // Satellite constellation from real GNSS log (18 satellites)
        let mut sats = [SatInfo::default(); 18];
        let flags = 0x0000191F_u32; // qualityInd=7, svUsed=1, health=1, ephAvail=1

        // GPS satellites (gnss_id=0)
        sats[0] = SatInfo { gnss_id: 0, sv_id: 1, cno: 35, elev: 44, azim: 242, pr_res: 0, flags };
        sats[1] = SatInfo { gnss_id: 0, sv_id: 2, cno: 35, elev: 26, azim: 214, pr_res: 0, flags };
        sats[2] = SatInfo { gnss_id: 0, sv_id: 3, cno: 35, elev: 57, azim: 303, pr_res: 0, flags };
        sats[3] = SatInfo { gnss_id: 0, sv_id: 10, cno: 30, elev: 2, azim: 131, pr_res: 0, flags };
        sats[4] = SatInfo { gnss_id: 0, sv_id: 12, cno: 30, elev: 3, azim: 20, pr_res: 0, flags };
        sats[5] = SatInfo { gnss_id: 0, sv_id: 26, cno: 35, elev: 16, azim: 146, pr_res: 0, flags };
        sats[6] = SatInfo { gnss_id: 0, sv_id: 28, cno: 35, elev: 59, azim: 70, pr_res: 0, flags };
        sats[7] = SatInfo { gnss_id: 0, sv_id: 31, cno: 35, elev: 70, azim: 144, pr_res: 0, flags };
        sats[8] = SatInfo { gnss_id: 0, sv_id: 32, cno: 35, elev: 27, azim: 77, pr_res: 0, flags };
        // SBAS satellites (gnss_id=1) - boosted signal
        sats[9] = SatInfo { gnss_id: 1, sv_id: 131, cno: 35, elev: 34, azim: 184, pr_res: 0, flags };
        sats[10] = SatInfo { gnss_id: 1, sv_id: 133, cno: 35, elev: 33, azim: 199, pr_res: 0, flags };
        sats[11] = SatInfo { gnss_id: 1, sv_id: 138, cno: 35, elev: 34, azim: 171, pr_res: 0, flags };
        // Galileo satellites (gnss_id=2)
        sats[12] = SatInfo { gnss_id: 2, sv_id: 5, cno: 35, elev: 45, azim: 69, pr_res: 0, flags };
        sats[13] = SatInfo { gnss_id: 2, sv_id: 8, cno: 35, elev: 25, azim: 261, pr_res: 0, flags };
        sats[14] = SatInfo { gnss_id: 2, sv_id: 13, cno: 35, elev: 29, azim: 314, pr_res: 0, flags };
        sats[15] = SatInfo { gnss_id: 2, sv_id: 15, cno: 35, elev: 50, azim: 246, pr_res: 0, flags };
        sats[16] = SatInfo { gnss_id: 2, sv_id: 16, cno: 35, elev: 45, azim: 180, pr_res: 0, flags };
        sats[17] = SatInfo { gnss_id: 2, sv_id: 31, cno: 35, elev: 10, azim: 30, pr_res: 0, flags };

        Self {
            itow: 0,
            version: 0x01,
            num_svs: 18,
            reserved1: [0; 2],
            sats,
        }
    }
}

impl UbxMessage for NavSat {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x35 }
    fn payload_len(&self) -> u16 { 8 + 12 * self.num_svs as u16 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4] = self.version;
        buf[5] = self.num_svs;
        buf[6..8].copy_from_slice(&self.reserved1);

        let mut offset = 8;
        for i in 0..self.num_svs as usize {
            let sat = &self.sats[i];
            buf[offset] = sat.gnss_id;
            buf[offset + 1] = sat.sv_id;
            buf[offset + 2] = sat.cno;
            buf[offset + 3] = sat.elev as u8;
            buf[offset + 4..offset + 6].copy_from_slice(&sat.azim.to_le_bytes());
            buf[offset + 6..offset + 8].copy_from_slice(&sat.pr_res.to_le_bytes());
            buf[offset + 8..offset + 12].copy_from_slice(&sat.flags.to_le_bytes());
            offset += 12;
        }
        offset
    }
}

/// Satellite info entry for NAV-SVINFO (legacy)
#[derive(Clone, Copy, Default)]
pub struct SvInfo {
    pub chn: u8,
    pub svid: u8,
    pub flags: u8,
    pub quality: u8,
    pub cno: u8,
    pub elev: i8,
    pub azim: i16,
    pub pr_res: i32,    // cm
}

/// UBX-NAV-SVINFO (0x01 0x30) - Satellite Information (legacy M8 format)
/// 18 satellites: 9 GPS + 3 SBAS + 6 Galileo (from real GNSS log)
/// Note: Galileo svid encoding = 211 + sv_id - 1 (E01=211, E31=241)
#[derive(Clone)]
pub struct NavSvinfo {
    pub itow: u32,
    pub num_ch: u8,
    pub global_flags: u8,
    pub reserved1: [u8; 2],
    pub svs: [SvInfo; 18],
}

impl Default for NavSvinfo {
    fn default() -> Self {
        let mut svs = [SvInfo::default(); 18];
        let flags = 0x0D;   // svUsed + orbitAvail + orbitEph
        let quality = 0x07; // best quality

        // GPS satellites (svid = PRN)
        svs[0] = SvInfo { chn: 0, svid: 1, flags, quality, cno: 35, elev: 44, azim: 242, pr_res: -1536 };
        svs[1] = SvInfo { chn: 1, svid: 2, flags, quality, cno: 35, elev: 26, azim: 214, pr_res: -1536 };
        svs[2] = SvInfo { chn: 2, svid: 3, flags, quality, cno: 35, elev: 57, azim: 303, pr_res: -1536 };
        svs[3] = SvInfo { chn: 3, svid: 10, flags, quality, cno: 30, elev: 2, azim: 131, pr_res: -1536 };
        svs[4] = SvInfo { chn: 4, svid: 12, flags, quality, cno: 30, elev: 3, azim: 20, pr_res: -1536 };
        svs[5] = SvInfo { chn: 5, svid: 26, flags, quality, cno: 35, elev: 16, azim: 146, pr_res: -1536 };
        svs[6] = SvInfo { chn: 6, svid: 28, flags, quality, cno: 35, elev: 59, azim: 70, pr_res: -1536 };
        svs[7] = SvInfo { chn: 7, svid: 31, flags, quality, cno: 35, elev: 70, azim: 144, pr_res: -1536 };
        svs[8] = SvInfo { chn: 8, svid: 32, flags, quality, cno: 35, elev: 27, azim: 77, pr_res: -1536 };
        // SBAS satellites (svid = PRN, 120-158)
        svs[9] = SvInfo { chn: 9, svid: 131, flags, quality, cno: 35, elev: 34, azim: 184, pr_res: -1536 };
        svs[10] = SvInfo { chn: 10, svid: 133, flags, quality, cno: 35, elev: 33, azim: 199, pr_res: -1536 };
        svs[11] = SvInfo { chn: 11, svid: 138, flags, quality, cno: 35, elev: 34, azim: 171, pr_res: -1536 };
        // Galileo satellites (svid = 211 + E_num - 1: E05=215, E08=218, E13=223, E15=225, E16=226, E31=241)
        svs[12] = SvInfo { chn: 12, svid: 215, flags, quality, cno: 35, elev: 45, azim: 69, pr_res: -1536 };   // E05
        svs[13] = SvInfo { chn: 13, svid: 218, flags, quality, cno: 35, elev: 25, azim: 261, pr_res: -1536 };  // E08
        svs[14] = SvInfo { chn: 14, svid: 223, flags, quality, cno: 35, elev: 29, azim: 314, pr_res: -1536 };  // E13
        svs[15] = SvInfo { chn: 15, svid: 225, flags, quality, cno: 35, elev: 50, azim: 246, pr_res: -1536 };  // E15
        svs[16] = SvInfo { chn: 16, svid: 226, flags, quality, cno: 35, elev: 45, azim: 180, pr_res: -1536 };  // E16
        svs[17] = SvInfo { chn: 17, svid: 241, flags, quality, cno: 35, elev: 10, azim: 30, pr_res: -1536 };   // E31

        Self {
            itow: 0,
            num_ch: 18,
            global_flags: 0x04,
            reserved1: [0; 2],
            svs,
        }
    }
}

impl UbxMessage for NavSvinfo {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x30 }
    fn payload_len(&self) -> u16 { 8 + 12 * self.num_ch as u16 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4] = self.num_ch;
        buf[5] = self.global_flags;
        buf[6..8].copy_from_slice(&self.reserved1);

        let mut offset = 8;
        for i in 0..self.num_ch as usize {
            let sv = &self.svs[i];
            buf[offset] = sv.chn;
            buf[offset + 1] = sv.svid;
            buf[offset + 2] = sv.flags;
            buf[offset + 3] = sv.quality;
            buf[offset + 4] = sv.cno;
            buf[offset + 5] = sv.elev as u8;
            buf[offset + 6..offset + 8].copy_from_slice(&sv.azim.to_le_bytes());
            buf[offset + 8..offset + 12].copy_from_slice(&sv.pr_res.to_le_bytes());
            offset += 12;
        }
        offset
    }
}

// ============================================================================
// MON messages
// ============================================================================

/// UBX-MON-HW (0x0A 0x09) - Hardware Status
#[derive(Clone)]
pub struct MonHw {
    pub pin_sel: u32,
    pub pin_bank: u32,
    pub pin_dir: u32,
    pub pin_val: u32,
    pub noise_per_ms: u16,
    pub agc_cnt: u16,
    pub a_status: u8,
    pub a_power: u8,
    pub flags: u8,
    pub reserved1: u8,
    pub used_mask: u32,
    pub vp: [u8; 17],
    pub jam_ind: u8,
    pub reserved2: [u8; 2],
    pub pin_irq: u32,
    pub pull_h: u32,
    pub pull_l: u32,
}

impl Default for MonHw {
    fn default() -> Self {
        Self {
            pin_sel: 0x00000020,
            pin_bank: 0x00000003,
            pin_dir: 0,
            pin_val: 0,
            noise_per_ms: 231,
            agc_cnt: 66,
            a_status: 76,
            a_power: 0,
            flags: 0xCE,
            reserved1: 0x0D,
            used_mask: 0x0FFC,
            vp: [0x01, 0x02, 0x25, 0x86, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0E, 0x10, 0x12, 0x0A, 0x0B, 0x13],
            jam_ind: 0x13,
            reserved2: [0x0E, 0x5A],
            pin_irq: 0,
            pull_h: 0x000000ED,
            pull_l: 0,
        }
    }
}

impl UbxMessage for MonHw {
    fn class(&self) -> u8 { 0x0A }
    fn id(&self) -> u8 { 0x09 }
    fn payload_len(&self) -> u16 { 60 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.pin_sel.to_le_bytes());
        buf[4..8].copy_from_slice(&self.pin_bank.to_le_bytes());
        buf[8..12].copy_from_slice(&self.pin_dir.to_le_bytes());
        buf[12..16].copy_from_slice(&self.pin_val.to_le_bytes());
        buf[16..18].copy_from_slice(&self.noise_per_ms.to_le_bytes());
        buf[18..20].copy_from_slice(&self.agc_cnt.to_le_bytes());
        buf[20] = self.a_status;
        buf[21] = self.a_power;
        buf[22] = self.flags;
        buf[23] = self.reserved1;
        buf[24..28].copy_from_slice(&self.used_mask.to_le_bytes());
        buf[28..45].copy_from_slice(&self.vp);
        buf[45] = self.jam_ind;
        buf[46..48].copy_from_slice(&self.reserved2);
        buf[48..52].copy_from_slice(&self.pin_irq.to_le_bytes());
        buf[52..56].copy_from_slice(&self.pull_h.to_le_bytes());
        buf[56..60].copy_from_slice(&self.pull_l.to_le_bytes());
        60
    }
}

/// UBX-MON-RF (0x0A 0x38) - RF Information
#[derive(Clone)]
pub struct MonRf {
    pub version: u8,
    pub n_blocks: u8,
    pub reserved1: [u8; 2],
    // One RF block:
    pub block_id: u8,
    pub flags: u8,
    pub ant_status: u8,
    pub ant_power: u8,
    pub post_status: u32,
    pub reserved2: [u8; 2],
    pub noise_per_ms: u16,
    pub agc_cnt: u16,
    pub jam_ind: u8,
    pub ofs_i: i8,
    pub mag_i: u8,
    pub ofs_q: i8,
    pub mag_q: u8,
    pub reserved3: u8,
}

impl Default for MonRf {
    fn default() -> Self {
        Self {
            version: 0,
            n_blocks: 1,
            reserved1: [0; 2],
            block_id: 0,
            flags: 0,
            ant_status: 0,
            ant_power: 0,
            post_status: 0,
            reserved2: [0; 2],
            noise_per_ms: 90,
            agc_cnt: 1488,
            jam_ind: 8,
            ofs_i: 0,
            mag_i: 0,
            ofs_q: 0,
            mag_q: 0,
            reserved3: 0,
        }
    }
}

impl UbxMessage for MonRf {
    fn class(&self) -> u8 { 0x0A }
    fn id(&self) -> u8 { 0x38 }
    fn payload_len(&self) -> u16 { 24 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.version;
        buf[1] = self.n_blocks;
        buf[2..4].copy_from_slice(&self.reserved1);
        buf[4] = self.block_id;
        buf[5] = self.flags;
        buf[6] = self.ant_status;
        buf[7] = self.ant_power;
        buf[8..12].copy_from_slice(&self.post_status.to_le_bytes());
        buf[12..14].copy_from_slice(&self.reserved2);
        buf[14..16].copy_from_slice(&self.noise_per_ms.to_le_bytes());
        buf[16..18].copy_from_slice(&self.agc_cnt.to_le_bytes());
        buf[18] = self.jam_ind;
        buf[19] = self.ofs_i as u8;
        buf[20] = self.mag_i;
        buf[21] = self.ofs_q as u8;
        buf[22] = self.mag_q;
        buf[23] = self.reserved3;
        24
    }
}

/// UBX-MON-COMMS (0x0A 0x36) - Communication Port Information (minimal)
#[derive(Clone, Default)]
pub struct MonComms {
    pub version: u8,
    pub n_ports: u8,
    pub tx_errors: u8,
    pub reserved1: u8,
    pub reserved2: [u8; 4],
}

impl UbxMessage for MonComms {
    fn class(&self) -> u8 { 0x0A }
    fn id(&self) -> u8 { 0x36 }
    fn payload_len(&self) -> u16 { 8 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.version;
        buf[1] = self.n_ports;
        buf[2] = self.tx_errors;
        buf[3] = self.reserved1;
        buf[4..8].copy_from_slice(&self.reserved2);
        8
    }
}

// ============================================================================
// TIM and RXM messages
// ============================================================================

/// UBX-TIM-TP (0x0D 0x01) - Timepulse
#[derive(Clone)]
pub struct TimTp {
    pub tow_ms: u32,
    pub tow_sub_ms: u32,
    pub q_err: i32,
    pub week: u16,
    pub flags: u8,
    pub ref_info: u8,
}

impl Default for TimTp {
    fn default() -> Self {
        Self {
            tow_ms: 0,
            tow_sub_ms: 0,
            q_err: 0,
            week: 2349,
            flags: 0x1A,
            ref_info: 0,
        }
    }
}

impl UbxMessage for TimTp {
    fn class(&self) -> u8 { 0x0D }
    fn id(&self) -> u8 { 0x01 }
    fn payload_len(&self) -> u16 { 16 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.tow_ms.to_le_bytes());
        buf[4..8].copy_from_slice(&self.tow_sub_ms.to_le_bytes());
        buf[8..12].copy_from_slice(&self.q_err.to_le_bytes());
        buf[12..14].copy_from_slice(&self.week.to_le_bytes());
        buf[14] = self.flags;
        buf[15] = self.ref_info;
        16
    }
}

/// UBX-RXM-RAWX (0x02 0x15) - Raw Measurement Data (minimal, no measurements)
#[derive(Clone, Default)]
pub struct RxmRawx {
    pub rcv_tow: f64,
    pub week: u16,
    pub leap_s: i8,
    pub num_meas: u8,
    pub rec_stat: u8,
    pub version: u8,
    pub reserved1: [u8; 2],
}

impl UbxMessage for RxmRawx {
    fn class(&self) -> u8 { 0x02 }
    fn id(&self) -> u8 { 0x15 }
    fn payload_len(&self) -> u16 { 16 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..8].copy_from_slice(&self.rcv_tow.to_le_bytes());
        buf[8..10].copy_from_slice(&self.week.to_le_bytes());
        buf[10] = self.leap_s as u8;
        buf[11] = self.num_meas;
        buf[12] = self.rec_stat;
        buf[13] = self.version;
        buf[14..16].copy_from_slice(&self.reserved1);
        16
    }
}
