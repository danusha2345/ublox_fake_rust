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
        // Values copied exactly from C version (massivs.h UBX_NAV_PVT)
        Self {
            itow: 0,
            year: 2025,
            month: 1,
            day: 12,             // C: 0x0C
            hour: 12,            // C: 0x0C
            min: 43,             // C: 0x2B
            sec: 2,              // C: 0x02
            valid: 0x37,         // C: 0x37
            t_acc: 6,            // C: 0x00000006
            nano: 0x11DE89FD_u32 as i32,  // C: from real GNSS
            fix_type: 3,         // C: 0x03
            flags: 0x01,         // C: 0x01
            flags2: 0x0E,        // C: 0x0E
            num_sv: 18,          // C: 0x12
            lon: -801919471,     // C: 0xD03BAA11 (real GNSS coordinates)
            lat: 257889186,      // C: 0x0F5E13A2
            height: 73449,       // C: 0x00011EE9
            h_msl: 100902,       // C: 0x00018626
            h_acc: 1435,         // C: 0x0000059B
            v_acc: 2073,         // C: 0x00000819
            vel_n: -2,           // C: 0xFFFFFFFE
            vel_e: 1,            // C: 0x00000001
            vel_d: 1,            // C: 0x00000001
            g_speed: 0,          // C: 0x00000000
            head_mot: 0x00C78ADA_u32 as i32,  // C: from real GNSS
            s_acc: 8,            // C: 0x00000008
            head_acc: 0x00F0D89E,  // C: from real GNSS
            p_dop: 110,          // C: 0x006E
            flags3: 0,
            reserved1: [0; 5],
            head_veh: 0x00234AE0_u32 as i32,  // C: from real GNSS
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
#[derive(Clone)]
pub struct NavPosecef {
    pub itow: u32,
    pub ecef_x: i32,  // cm
    pub ecef_y: i32,
    pub ecef_z: i32,
    pub p_acc: u32,   // cm
}

impl Default for NavPosecef {
    fn default() -> Self {
        // Values from C version (massivs.h UBX_NAV_POSECEF)
        Self {
            itow: 0,
            ecef_x: 0x05DDB07B_u32 as i32,  // 98230395 cm
            ecef_y: 0xDE407016_u32 as i32,  // -565854186 cm
            ecef_z: 0x106F7693_u32 as i32,  // 276297363 cm
            p_acc: 252,                      // C: 0x000000FC
        }
    }
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
#[derive(Clone)]
pub struct NavPosllh {
    pub itow: u32,
    pub lon: i32,     // deg * 1e-7
    pub lat: i32,     // deg * 1e-7
    pub height: i32,  // mm
    pub h_msl: i32,   // mm
    pub h_acc: u32,   // mm
    pub v_acc: u32,   // mm
}

impl Default for NavPosllh {
    fn default() -> Self {
        // Values from C version (massivs.h UBX_NAV_POSLLH)
        Self {
            itow: 0,
            lon: 0xD03BAA11_u32 as i32,   // -801919471 (deg * 1e-7)
            lat: 0x0F5E13A2_u32 as i32,   // 257889186 (deg * 1e-7)
            height: 73449,                 // C: 0x00011EE9 mm
            h_msl: 100902,                 // C: 0x00018626 mm
            h_acc: 1435,                   // C: 0x0000059B mm
            v_acc: 2073,                   // C: 0x00000819 mm
        }
    }
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
        // Values from C version (massivs.h UBX_NAV_STATUS)
        Self {
            itow: 0,
            gps_fix: 3,   // C: 0x03
            flags: 0x0F,  // C: 0x0F
            fix_stat: 0x00,
            flags2: 0x08, // C: 0x08
            ttff: 0,      // C: 0x00000000
            msss: 0,      // C: 0x00000000
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
            g_dop: 100,  // 1.00 (same as C)
            p_dop: 100,
            t_dop: 100,
            v_dop: 100,
            h_dop: 100,
            n_dop: 100,
            e_dop: 100,
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

/// UBX-NAV-SOL (0x01 0x06) - Navigation Solution Information (legacy M8)
/// This message is deprecated in M10 but many flight controllers still use it
#[derive(Clone)]
pub struct NavSol {
    pub itow: u32,
    pub f_tow: i32,      // ns (fractional TOW)
    pub week: i16,
    pub gps_fix: u8,     // 0=no, 1=DR, 2=2D, 3=3D, 4=GPS+DR, 5=time only
    pub flags: u8,       // bit0=GPSfixOK, bit1=DiffSoln, bit2=WKNSET, bit3=TOWSET
    pub ecef_x: i32,     // cm
    pub ecef_y: i32,     // cm
    pub ecef_z: i32,     // cm
    pub p_acc: u32,      // cm
    pub ecef_vx: i32,    // cm/s
    pub ecef_vy: i32,    // cm/s
    pub ecef_vz: i32,    // cm/s
    pub s_acc: u32,      // cm/s
    pub p_dop: u16,      // *0.01
    pub reserved1: u8,
    pub num_sv: u8,
    pub reserved2: u32,
}

impl Default for NavSol {
    fn default() -> Self {
        // ECEF coordinates from C version (same as NAV-POSECEF)
        Self {
            itow: 0,
            f_tow: 0,
            week: 2349,                          // GPS week (0x092D)
            gps_fix: 3,                          // 3D fix
            flags: 0x0F,                         // GPSfixOK + DiffSoln + WKNSET + TOWSET
            ecef_x: 0x05DDB07B_u32 as i32,       // 98230395 cm
            ecef_y: 0xDE407016_u32 as i32,       // -565854186 cm
            ecef_z: 0x106F7693_u32 as i32,       // 276297363 cm
            p_acc: 252,                          // C: 0x000000FC cm
            ecef_vx: 0,
            ecef_vy: 0,
            ecef_vz: 0,
            s_acc: 100,                          // 1 m/s (0x64)
            p_dop: 110,                          // 1.10 (0x6E)
            reserved1: 0,
            num_sv: 18,
            reserved2: 0,
        }
    }
}

impl UbxMessage for NavSol {
    fn class(&self) -> u8 { 0x01 }
    fn id(&self) -> u8 { 0x06 }
    fn payload_len(&self) -> u16 { 52 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0..4].copy_from_slice(&self.itow.to_le_bytes());
        buf[4..8].copy_from_slice(&self.f_tow.to_le_bytes());
        buf[8..10].copy_from_slice(&self.week.to_le_bytes());
        buf[10] = self.gps_fix;
        buf[11] = self.flags;
        buf[12..16].copy_from_slice(&self.ecef_x.to_le_bytes());
        buf[16..20].copy_from_slice(&self.ecef_y.to_le_bytes());
        buf[20..24].copy_from_slice(&self.ecef_z.to_le_bytes());
        buf[24..28].copy_from_slice(&self.p_acc.to_le_bytes());
        buf[28..32].copy_from_slice(&self.ecef_vx.to_le_bytes());
        buf[32..36].copy_from_slice(&self.ecef_vy.to_le_bytes());
        buf[36..40].copy_from_slice(&self.ecef_vz.to_le_bytes());
        buf[40..44].copy_from_slice(&self.s_acc.to_le_bytes());
        buf[44..46].copy_from_slice(&self.p_dop.to_le_bytes());
        buf[46] = self.reserved1;
        buf[47] = self.num_sv;
        buf[48..52].copy_from_slice(&self.reserved2.to_le_bytes());
        52
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
        let hw = b"000A0000";  // M10 hardware version (must match C version)
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

/// Unique IDs for different drone models (from real GNSS logs)
pub mod uniqid {
    /// DJI Air 3 chip unique ID
    pub const AIR3: [u8; 5] = [0xE0, 0x95, 0x65, 0x0F, 0x2A];
    /// DJI Mavic 4 Pro chip unique ID
    pub const MAVIC4PRO: [u8; 5] = [0xEB, 0xB9, 0x91, 0x0F, 0x2B];
}

impl SecUniqid {
    /// Create SEC-UNIQID for specific drone model
    pub fn for_model(model: crate::config::DroneModel) -> Self {
        let unique_id = match model {
            crate::config::DroneModel::Air3 => uniqid::AIR3,
            crate::config::DroneModel::Mavic4Pro => uniqid::MAVIC4PRO,
        };
        Self {
            version: 0x02,
            reserved: [0x00, 0x00, 0x00],
            unique_id,
            reserved2: 0x54,
        }
    }
}

impl Default for SecUniqid {
    fn default() -> Self {
        Self {
            version: 0x02,
            reserved: [0x00, 0x00, 0x00],
            unique_id: uniqid::AIR3,  // Default to Air 3
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
#[derive(Clone)]
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

impl Default for NavVelned {
    fn default() -> Self {
        Self {
            itow: 0,
            vel_n: 0,
            vel_e: 0,
            vel_d: 0,
            speed: 0,
            g_speed: 0,
            heading: 0,
            s_acc: 100,  // Same as C version (0x64)
            c_acc: 0,
        }
    }
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
#[derive(Clone)]
pub struct NavVelecef {
    pub itow: u32,
    pub ecef_vx: i32,   // cm/s
    pub ecef_vy: i32,   // cm/s
    pub ecef_vz: i32,   // cm/s
    pub s_acc: u32,     // cm/s
}

impl Default for NavVelecef {
    fn default() -> Self {
        Self {
            itow: 0,
            ecef_vx: 0,
            ecef_vy: 0,
            ecef_vz: 0,
            s_acc: 100,  // Same as C version (0x64)
        }
    }
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
        // Values from C version (massivs.h UBX_NAV_TIMEUTC)
        Self {
            itow: 0,
            t_acc: 0,     // C: 0x00000000
            nano: 0,      // C: 0x00000000
            year: 2025,   // C: 0x07E9
            month: 1,     // C: 0x01
            day: 12,      // C: 0x0C
            hour: 12,     // C: 0x0C
            min: 0,       // C: 0x00
            sec: 0,       // C: 0x00
            valid: 0xF7,  // C: 0xF7
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
        // Values from C version (massivs.h UBX_NAV_TIMEGPS)
        Self {
            itow: 0,
            f_tow: 0,
            week: 2349,   // C: 0x092D
            leap_s: 18,   // C: 0x12
            valid: 0x07,  // C: 0x07
            t_acc: 5,     // C: 0x00000005
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
#[derive(Clone)]
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

impl Default for NavHpposecef {
    fn default() -> Self {
        // Values from C version (massivs.h UBX_NAV_HPPOSECEF)
        Self {
            version: 0,
            reserved1: [0; 3],
            itow: 0,
            ecef_x: 0x05DDB07B_u32 as i32,  // 98230395 cm
            ecef_y: 0xDE407016_u32 as i32,  // -565854186 cm
            ecef_z: 0x106F7693_u32 as i32,  // 276297363 cm
            ecef_x_hp: 0,
            ecef_y_hp: 0,
            ecef_z_hp: 0,
            flags: 0,
            p_acc: 252,                      // C: 0x000000FC (0.1mm units)
        }
    }
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
#[derive(Clone)]
pub struct MonComms {
    pub version: u8,
    pub n_ports: u8,
    pub tx_errors: u8,
    pub reserved1: u8,
    pub reserved2: [u8; 4],
}

impl Default for MonComms {
    fn default() -> Self {
        Self {
            version: 1,   // Same as C version
            n_ports: 1,   // Same as C version
            tx_errors: 0,
            reserved1: 0,
            reserved2: [0; 4],
        }
    }
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
#[derive(Clone)]
pub struct RxmRawx {
    pub rcv_tow: f64,
    pub week: u16,
    pub leap_s: i8,
    pub num_meas: u8,
    pub rec_stat: u8,
    pub version: u8,
    pub reserved1: [u8; 2],
}

impl Default for RxmRawx {
    fn default() -> Self {
        Self {
            rcv_tow: 0.0,
            week: 0,
            leap_s: 0,
            num_meas: 0,
            rec_stat: 1,   // Same as C version
            version: 0,
            reserved1: [0; 2],
        }
    }
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

// ============================================================================
// DJI Proprietary Messages
// ============================================================================

/// CFG-0x41 (0x06 0x41) - DJI proprietary SEC-SIGN configuration
/// 256-byte response containing private key and configuration data
/// Private key is located at offset 175 (0xAF) in payload
#[derive(Clone)]
pub struct Cfg41 {
    /// Full 256-byte payload (template with private key embedded)
    pub payload: [u8; 256],
}

/// CFG-0x41 payload templates for different drone models
/// Captured from real DJI GNSS modules
pub mod cfg41_templates {
    /// Private key offset in CFG-0x41 payload
    pub const PRIVATE_KEY_OFFSET: usize = 175;

    /// Base template from Mavic 4 Pro (private key will be replaced)
    pub const TEMPLATE: [u8; 256] = [
        0xC4, 0x90, 0xF3, 0xFF, 0xAE, 0xFF, 0xFE, 0xFF, 0xF7, 0xFF, 0x9F, 0xFF, 0xFF, 0xF7, 0xBF, 0xFF,
        0xEF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x82, 0x18, 0xB0, 0x00, 0x00, 0x00,
        0xF0, 0xA0, 0xF7, 0xF7, 0x00, 0x22, 0x42, 0x60, 0xC0, 0xF8, 0xBC, 0x10, 0x4F, 0xF4, 0x00, 0x31,
        0xC1, 0x67, 0x70, 0x47, 0x00, 0x00, 0x83, 0x28, 0x8C, 0x16, 0x00, 0x00, 0xF1, 0xA0, 0xF7, 0xF7,
        0x23, 0xF4, 0x40, 0x03, 0x8E, 0x68, 0x46, 0xF4, 0x40, 0x06, 0xC1, 0xF8, 0xBC, 0x20, 0x8E, 0x60,
        0xCE, 0x68, 0xC1, 0xF8, 0xBC, 0x20, 0x46, 0xF4, 0xC0, 0x06, 0xCE, 0x60, 0x70, 0x47, 0x00, 0x00,
        0xA4, 0x0F, 0x0F, 0x00, 0x31, 0x10, 0x00, 0x0D, 0x00, 0x31, 0x10, 0x01, 0x22, 0x00, 0x31, 0x10,
        0x01, 0xA4, 0x3A, 0x02, 0x00, 0xC7, 0x10, 0x01, 0x03, 0x00, 0xC7, 0x20, 0x1E, 0x04, 0x00, 0xC7,
        0x50, 0x6A, 0xFD, 0xDD, 0xD2, 0x50, 0x21, 0xEB, 0xB3, 0x05, 0x00, 0xC7, 0x50, 0x1D, 0xEB, 0xED,
        0x47, 0x60, 0x44, 0x12, 0x42, 0x06, 0x00, 0xC7, 0x50, 0x6A, 0xC4, 0x43, 0x75, 0x06, 0x5A, 0x7A,
        0xFD, 0x07, 0x00, 0xC7, 0x50, 0x41, 0x77, 0xD2, 0xE9, 0xA1, 0xF0, 0x00, 0x00, 0xA6, 0x18,
        // Private key starts at offset 175 (next 24 bytes) - placeholder zeros
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // Remaining config data
        0xA4, 0x08, 0x01, 0x00, 0x52, 0x40, 0x00, 0x10, 0x0E, 0x00, 0xA4, 0x20, 0x01, 0x00, 0xA4, 0x40,
        0x00, 0xB0, 0x71, 0x0B, 0x03, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B, 0x05, 0x00, 0xA4, 0x40,
        0x00, 0xB0, 0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    ];
}

impl Cfg41 {
    /// Create CFG-0x41 response for specific drone model
    pub fn for_model(model: crate::config::DroneModel) -> Self {
        use crate::sec_sign::{PRIVATE_KEY_AIR3, PRIVATE_KEY_MAVIC4PRO};

        let mut payload = cfg41_templates::TEMPLATE;

        // Insert private key at offset 175
        let key = match model {
            crate::config::DroneModel::Air3 => &PRIVATE_KEY_AIR3,
            crate::config::DroneModel::Mavic4Pro => &PRIVATE_KEY_MAVIC4PRO,
        };
        payload[cfg41_templates::PRIVATE_KEY_OFFSET..cfg41_templates::PRIVATE_KEY_OFFSET + 24]
            .copy_from_slice(key);

        Self { payload }
    }
}

impl Default for Cfg41 {
    fn default() -> Self {
        Self::for_model(crate::config::DroneModel::Air3)
    }
}

impl UbxMessage for Cfg41 {
    fn class(&self) -> u8 { 0x06 }
    fn id(&self) -> u8 { 0x41 }
    fn payload_len(&self) -> u16 { 256 }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[..256].copy_from_slice(&self.payload);
        256
    }
}

/// CFG-VALGET (0x06 0x8B) response - returns requested configuration values
#[derive(Clone)]
pub struct CfgValgetResponse {
    /// Version (0x01)
    pub version: u8,
    /// Layer (0x00=RAM, 0x01=BBR, 0x02=Flash, 0x07=default)
    pub layer: u8,
    /// Key-value pairs (key_id, value) - value stored as u64 to support all sizes
    pub values: heapless::Vec<(u32, u64), 8>,
}

impl Default for CfgValgetResponse {
    fn default() -> Self {
        Self {
            version: 0x01,
            layer: 0x04, // Default layer
            values: heapless::Vec::new(),
        }
    }
}

/// Known CFG-VALGET keys and their default values
pub mod valget_defaults {
    /// CFG-UART1-BAUDRATE: 921600 (U4)
    pub const UART1_BAUDRATE: (u32, u32) = (0x40520001, 921600);
    /// CFG-HW-RF_LNA_MODE (crystal frequency): 192 MHz (U4)
    pub const XTAL_FREQ: (u32, u32) = (0x40A40001, 192_000_000);
    /// CFG-RINV-DUMP: Dump data at startup (L/1-bit)
    pub const RINV_DUMP: (u32, u32) = (0x10C70001, 0);
    /// CFG-RINV-SIZE: Size of RINV data (U1) = 30 bytes
    pub const RINV_SIZE: (u32, u32) = (0x20C70003, 0x1E);

    /// CFG-RINV-DATA0..DATA3: Remote Inventory data (U8 each)
    /// These contain DJI identification data (from Mavic 4 Pro log)
    pub const RINV_DATA0: (u32, u64) = (0x50C70004, 0xB3EB2150D2DDFD6A);
    pub const RINV_DATA1: (u32, u64) = (0x50C70005, 0x4212446047EDEB1D);
    pub const RINV_DATA2: (u32, u64) = (0x50C70006, 0xFD7A5A067543C46A);
    pub const RINV_DATA3: (u32, u64) = (0x50C70007, 0x0000F0A1E9D27741);
}

/// Get value size in bytes from key (bits 28-30)
fn key_value_size(key: u32) -> usize {
    match (key >> 28) & 0x07 {
        1 | 2 => 1,  // L (1-bit) or U1 (1-byte)
        3 => 2,      // U2 (2-byte)
        4 => 4,      // U4 (4-byte)
        5 => 8,      // U8 (8-byte)
        _ => 1,      // default to 1 byte
    }
}

impl CfgValgetResponse {
    /// Create response for requested keys
    pub fn for_keys(keys: &[u32]) -> Self {
        let mut resp = Self::default();
        for &key in keys {
            let value: u64 = match key {
                // 4-byte values (U4)
                0x40520001 => valget_defaults::UART1_BAUDRATE.1 as u64,
                0x40A40001 => valget_defaults::XTAL_FREQ.1 as u64,
                // 1-byte values (L/U1)
                0x10C70001 => valget_defaults::RINV_DUMP.1 as u64,
                0x20C70003 => valget_defaults::RINV_SIZE.1 as u64,
                // 8-byte values (U8) - RINV data
                0x50C70004 => valget_defaults::RINV_DATA0.1,
                0x50C70005 => valget_defaults::RINV_DATA1.1,
                0x50C70006 => valget_defaults::RINV_DATA2.1,
                0x50C70007 => valget_defaults::RINV_DATA3.1,
                _ => 0, // Unknown key - return 0
            };
            let _ = resp.values.push((key, value));
        }
        resp
    }
}

impl UbxMessage for CfgValgetResponse {
    fn class(&self) -> u8 { 0x06 }
    fn id(&self) -> u8 { 0x8B }
    fn payload_len(&self) -> u16 {
        // Header: version(1) + layer(1) + position(2) = 4
        // Each key-value: key(4) + value(size depends on key type)
        let mut len = 4u16;
        for &(key, _) in self.values.iter() {
            len += 4 + key_value_size(key) as u16;
        }
        len
    }

    fn write_payload(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.version;
        buf[1] = self.layer;
        buf[2] = 0x00; // Position low
        buf[3] = 0x00; // Position high

        let mut offset = 4;
        for &(key, value) in self.values.iter() {
            buf[offset..offset + 4].copy_from_slice(&key.to_le_bytes());
            offset += 4;

            // Write value with correct size based on key type
            let val_size = key_value_size(key);
            match val_size {
                1 => buf[offset] = value as u8,
                2 => buf[offset..offset + 2].copy_from_slice(&(value as u16).to_le_bytes()),
                4 => buf[offset..offset + 4].copy_from_slice(&(value as u32).to_le_bytes()),
                8 => buf[offset..offset + 8].copy_from_slice(&value.to_le_bytes()),
                _ => buf[offset] = value as u8,
            }
            offset += val_size;
        }
        offset
    }
}
