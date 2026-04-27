//! NMEA 0183 builder + parser for Unicore UC6580I emulation.
//!
//! Builders write into `&mut [u8]` and return the number of bytes written
//! (including the trailing `\r\n`). The emitted sentence always terminates
//! with a correct XOR checksum. Output is ASCII-only — safe to transmit as-is.
//!
//! Parsers accept raw NMEA lines (with or without trailing CR/LF) and extract
//! lat/lon/alt/fix/nsats. Nothing allocates.
//!
//! `rewrite_position_inplace` mutates a `$GxGGA` or `$GxRMC` sentence so it
//! reports a replaced latitude/longitude (and altitude for GGA), recomputing
//! the XOR checksum in place. Used by `PassthroughOffset` mode.
//!
//! All floating-point conversions go through `libm` to stay `no_std`-friendly.
//!
//! Reference: NMEA 0183 v4.11, docs/UC6580I.md §6.1.

#![allow(dead_code)]

// ---------------------------------------------------------------------------
// Checksum
// ---------------------------------------------------------------------------

/// XOR-checksum over bytes between `$` and `*` (both exclusive).
/// Accepts the raw sentence body slice — caller must strip the leading `$`.
pub fn nmea_checksum(body: &[u8]) -> u8 {
    let mut cs: u8 = 0;
    for &b in body {
        cs ^= b;
    }
    cs
}

/// Append `*HH\r\n` to `buf` starting at `pos`, where `HH` is the XOR of
/// bytes `buf[1..pos]` (i.e. the sentence body after the leading `$`).
/// Returns the total length including the terminator.
fn finalize_sentence(buf: &mut [u8], pos: usize) -> usize {
    let cs = nmea_checksum(&buf[1..pos]);
    if pos + 5 > buf.len() {
        return 0;
    }
    buf[pos] = b'*';
    buf[pos + 1] = hex_nibble(cs >> 4);
    buf[pos + 2] = hex_nibble(cs & 0x0F);
    buf[pos + 3] = b'\r';
    buf[pos + 4] = b'\n';
    pos + 5
}

#[inline]
fn hex_nibble(n: u8) -> u8 {
    match n {
        0..=9 => b'0' + n,
        10..=15 => b'A' + (n - 10),
        _ => b'?',
    }
}

#[inline]
fn parse_hex_nibble(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'A'..=b'F' => Some(c - b'A' + 10),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    }
}

/// Verify a full NMEA sentence checksum. Accepts sentence with optional
/// trailing `\r\n` and with or without the leading `$`. Returns `true` if
/// the embedded `*HH` matches the XOR of the body.
pub fn verify_sentence(line: &[u8]) -> bool {
    let mut end = line.len();
    while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
        end -= 1;
    }
    let mut start = 0;
    if end > 0 && line[0] == b'$' {
        start = 1;
    }
    let slice = &line[start..end];
    let star_pos = match slice.iter().rposition(|&b| b == b'*') {
        Some(p) => p,
        None => return false,
    };
    if slice.len() < star_pos + 3 {
        return false;
    }
    let hi = match parse_hex_nibble(slice[star_pos + 1]) {
        Some(v) => v,
        None => return false,
    };
    let lo = match parse_hex_nibble(slice[star_pos + 2]) {
        Some(v) => v,
        None => return false,
    };
    let expected = (hi << 4) | lo;
    nmea_checksum(&slice[..star_pos]) == expected
}

// ---------------------------------------------------------------------------
// Writer helpers (no alloc, no float formatting from libstd)
// ---------------------------------------------------------------------------

/// Write raw bytes, return new cursor or 0 on overflow.
fn write_bytes(buf: &mut [u8], pos: usize, data: &[u8]) -> usize {
    if pos + data.len() > buf.len() {
        return 0;
    }
    buf[pos..pos + data.len()].copy_from_slice(data);
    pos + data.len()
}

fn write_byte(buf: &mut [u8], pos: usize, b: u8) -> usize {
    if pos >= buf.len() {
        return 0;
    }
    buf[pos] = b;
    pos + 1
}

/// Write unsigned integer as decimal, returns new cursor.
fn write_u32(buf: &mut [u8], pos: usize, mut val: u32) -> usize {
    if val == 0 {
        return write_byte(buf, pos, b'0');
    }
    let mut tmp = [0u8; 10];
    let mut n = 0;
    while val > 0 {
        tmp[n] = b'0' + (val % 10) as u8;
        val /= 10;
        n += 1;
    }
    if pos + n > buf.len() {
        return 0;
    }
    for i in 0..n {
        buf[pos + i] = tmp[n - 1 - i];
    }
    pos + n
}

/// Write unsigned integer zero-padded to a fixed width.
fn write_u32_pad(buf: &mut [u8], pos: usize, val: u32, width: usize) -> usize {
    let mut tmp = [0u8; 12];
    let mut n = 0;
    let mut v = val;
    if v == 0 {
        tmp[0] = b'0';
        n = 1;
    } else {
        while v > 0 {
            tmp[n] = b'0' + (v % 10) as u8;
            v /= 10;
            n += 1;
        }
    }
    let pad = if width > n { width - n } else { 0 };
    let total = pad + n;
    if pos + total > buf.len() {
        return 0;
    }
    for i in 0..pad {
        buf[pos + i] = b'0';
    }
    for i in 0..n {
        buf[pos + pad + i] = tmp[n - 1 - i];
    }
    pos + total
}

/// Write `lat` in NMEA ddmm.mmmmmm format. Returns new cursor.
/// `lat_1e7` is signed degrees × 1e7. `hemi_buf` will receive b'N' or b'S'.
fn write_lat(buf: &mut [u8], pos: usize, lat_1e7: i32, hemi: &mut u8) -> usize {
    let abs = lat_1e7.unsigned_abs();
    *hemi = if lat_1e7 < 0 { b'S' } else { b'N' };
    // degrees = abs / 10_000_000
    // minutes = (abs % 10_000_000) * 60 / 10_000_000 = abs % 1e7 * 6e-6 — need 6 decimals
    let deg = abs / 10_000_000;
    let frac = abs % 10_000_000; // degrees fractional × 1e7
    // minutes_1e6 = frac * 60 / 10 = frac * 6
    let minutes_1e6: u64 = frac as u64 * 6;
    let minutes_int = minutes_1e6 / 1_000_000;
    let minutes_frac = minutes_1e6 % 1_000_000;
    let mut p = write_u32_pad(buf, pos, deg, 2);
    if p == 0 {
        return 0;
    }
    p = write_u32_pad(buf, p, minutes_int as u32, 2);
    if p == 0 {
        return 0;
    }
    p = write_byte(buf, p, b'.');
    if p == 0 {
        return 0;
    }
    p = write_u32_pad(buf, p, minutes_frac as u32, 6);
    p
}

/// Write `lon` in NMEA dddmm.mmmmmm format.
fn write_lon(buf: &mut [u8], pos: usize, lon_1e7: i32, hemi: &mut u8) -> usize {
    let abs = lon_1e7.unsigned_abs();
    *hemi = if lon_1e7 < 0 { b'W' } else { b'E' };
    let deg = abs / 10_000_000;
    let frac = abs % 10_000_000;
    let minutes_1e6: u64 = frac as u64 * 6;
    let minutes_int = minutes_1e6 / 1_000_000;
    let minutes_frac = minutes_1e6 % 1_000_000;
    let mut p = write_u32_pad(buf, pos, deg, 3);
    if p == 0 {
        return 0;
    }
    p = write_u32_pad(buf, p, minutes_int as u32, 2);
    if p == 0 {
        return 0;
    }
    p = write_byte(buf, p, b'.');
    if p == 0 {
        return 0;
    }
    p = write_u32_pad(buf, p, minutes_frac as u32, 6);
    p
}

/// Write signed integer as decimal with optional leading `-`.
fn write_i32(buf: &mut [u8], pos: usize, val: i32) -> usize {
    let mut p = pos;
    let abs = if val < 0 {
        p = write_byte(buf, p, b'-');
        if p == 0 {
            return 0;
        }
        val.unsigned_abs()
    } else {
        val as u32
    };
    write_u32(buf, p, abs)
}

/// Write fixed-point i32 as `int.dec` with `decimals` places after the dot.
/// `scale` is 10^decimals (caller provides it).
fn write_fixed(buf: &mut [u8], pos: usize, val: i32, scale: u32, decimals: usize) -> usize {
    let neg = val < 0;
    let abs = val.unsigned_abs();
    let int_part = abs / scale;
    let frac_part = abs % scale;
    let mut p = pos;
    if neg {
        p = write_byte(buf, p, b'-');
        if p == 0 {
            return 0;
        }
    }
    p = write_u32(buf, p, int_part);
    if p == 0 {
        return 0;
    }
    p = write_byte(buf, p, b'.');
    if p == 0 {
        return 0;
    }
    write_u32_pad(buf, p, frac_part, decimals)
}

// ---------------------------------------------------------------------------
// Builder structs
// ---------------------------------------------------------------------------

/// Time of day.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct NmeaTime {
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    /// Hundredths of a second (0..99).
    pub centis: u8,
}

impl NmeaTime {
    pub const fn new(hour: u8, minute: u8, second: u8) -> Self {
        Self { hour, minute, second, centis: 0 }
    }
}

/// Gregorian date.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct NmeaDate {
    pub day: u8,
    pub month: u8,
    /// Full year (e.g. 2026). Converted to two-digit for RMC/ZDA.
    pub year: u16,
}

/// Fields for `$GxGGA` (Global Positioning System Fix Data).
#[derive(Clone, Copy, Debug)]
pub struct GgaFields {
    pub time: NmeaTime,
    pub lat_1e7: i32,
    pub lon_1e7: i32,
    /// 0 = no fix, 1 = GPS fix, 2 = DGPS, 4 = RTK fix, 5 = RTK float.
    pub fix_quality: u8,
    pub nsats: u8,
    /// HDOP × 100 (e.g. 123 = 1.23).
    pub hdop_x100: u16,
    /// Altitude (MSL) in millimetres.
    pub alt_mm: i32,
    /// Geoidal separation in millimetres.
    pub geoid_sep_mm: i32,
}

/// Fields for `$GxRMC` (Recommended Minimum Specific).
#[derive(Clone, Copy, Debug)]
pub struct RmcFields {
    pub time: NmeaTime,
    /// `true` = data valid (emits 'A'), `false` = invalid ('V').
    pub valid: bool,
    pub lat_1e7: i32,
    pub lon_1e7: i32,
    /// Speed over ground in knots × 1000.
    pub sog_knots_x1000: u32,
    /// Course over ground in degrees × 100.
    pub cog_deg_x100: u32,
    pub date: NmeaDate,
    /// NMEA mode indicator: 'A' = autonomous, 'D' = differential, 'N' = none.
    pub mode: u8,
}

/// GSA operating mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GsaOpMode {
    /// 'A' — automatic 2D/3D switching.
    Automatic,
    /// 'M' — manual, forced to 2D or 3D.
    Manual,
}

/// Fields for `$GxGLL` (Geographic position, latitude/longitude).
#[derive(Clone, Copy, Debug)]
pub struct GllFields {
    pub lat_1e7: i32,
    pub lon_1e7: i32,
    pub time: NmeaTime,
    /// `true` = data valid (emits 'A'), `false` = invalid ('V').
    pub valid: bool,
    /// NMEA mode indicator: 'A' = autonomous, 'D' = differential, 'N' = none.
    pub mode: u8,
}

/// Fields for `$GxGSA` (GNSS DOP and active satellites).
#[derive(Clone, Copy, Debug)]
pub struct GsaFields {
    pub op_mode: GsaOpMode,
    /// 1 = no fix, 2 = 2D, 3 = 3D.
    pub fix_type: u8,
    /// Up to 12 PRNs, 0 = empty slot.
    pub sats: [u16; 12],
    /// PDOP × 100.
    pub pdop_x100: u16,
    /// HDOP × 100.
    pub hdop_x100: u16,
    /// VDOP × 100.
    pub vdop_x100: u16,
    /// System ID per NMEA v4.11 (1=GPS, 2=GLONASS, 3=Galileo, 4=BeiDou, 5=QZSS, 6=NavIC).
    pub system_id: u8,
}

/// One satellite record in `$GxGSV`.
#[derive(Clone, Copy, Debug, Default)]
pub struct GsvSat {
    pub prn: u16,
    pub elevation_deg: u8,
    pub azimuth_deg: u16,
    /// Carrier-to-noise ratio in dB-Hz. 0 = not tracking.
    pub cno_dbhz: u8,
}

/// Talker ID — first two characters after `$`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Talker {
    Gn,
    Gp,
    Gl,
    Ga,
    Gb,
    Gq,
    Gi,
}

impl Talker {
    pub const fn as_bytes(self) -> &'static [u8; 2] {
        match self {
            Talker::Gn => b"GN",
            Talker::Gp => b"GP",
            Talker::Gl => b"GL",
            Talker::Ga => b"GA",
            Talker::Gb => b"GB",
            Talker::Gq => b"GQ",
            Talker::Gi => b"GI",
        }
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        match bytes {
            b"GN" => Some(Talker::Gn),
            b"GP" => Some(Talker::Gp),
            b"GL" => Some(Talker::Gl),
            b"GA" => Some(Talker::Ga),
            b"GB" => Some(Talker::Gb),
            b"GQ" => Some(Talker::Gq),
            b"GI" => Some(Talker::Gi),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Builders
// ---------------------------------------------------------------------------

/// Build `$<talker>GGA,...*cs\r\n` into `buf`. Returns total bytes written, or 0 on overflow.
pub fn build_gga(buf: &mut [u8], talker: Talker, f: &GgaFields) -> usize {
    let mut p = 0;
    p = write_byte(buf, p, b'$');
    if p == 0 { return 0; }
    p = write_bytes(buf, p, talker.as_bytes());
    if p == 0 { return 0; }
    p = write_bytes(buf, p, b"GGA,");
    if p == 0 { return 0; }
    // hhmmss.ss
    p = write_u32_pad(buf, p, f.time.hour as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.minute as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.second as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b'.');
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.centis as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // lat,N/S
    let mut hemi = 0u8;
    p = write_lat(buf, p, f.lat_1e7, &mut hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_byte(buf, p, hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // lon,E/W
    p = write_lon(buf, p, f.lon_1e7, &mut hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_byte(buf, p, hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // fix quality
    p = write_u32(buf, p, f.fix_quality as u32);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // nsats (two-digit)
    p = write_u32_pad(buf, p, f.nsats as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // hdop x.xx
    p = write_fixed(buf, p, f.hdop_x100 as i32, 100, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // alt (metres with 3 decimals) + ,M,
    p = write_fixed(buf, p, f.alt_mm, 1000, 3);
    if p == 0 { return 0; }
    p = write_bytes(buf, p, b",M,");
    if p == 0 { return 0; }
    // geoid sep + ,M,,
    p = write_fixed(buf, p, f.geoid_sep_mm, 1000, 3);
    if p == 0 { return 0; }
    p = write_bytes(buf, p, b",M,,");
    if p == 0 { return 0; }
    finalize_sentence(buf, p)
}

/// Build `$<talker>RMC,...*cs\r\n` into `buf`. Returns total bytes written, or 0 on overflow.
pub fn build_rmc(buf: &mut [u8], talker: Talker, f: &RmcFields) -> usize {
    let mut p = 0;
    p = write_byte(buf, p, b'$');
    if p == 0 { return 0; }
    p = write_bytes(buf, p, talker.as_bytes());
    if p == 0 { return 0; }
    p = write_bytes(buf, p, b"RMC,");
    if p == 0 { return 0; }
    // time
    p = write_u32_pad(buf, p, f.time.hour as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.minute as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.second as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b'.');
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.centis as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // status
    p = write_byte(buf, p, if f.valid { b'A' } else { b'V' });
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // lat,N/S
    let mut hemi = 0u8;
    p = write_lat(buf, p, f.lat_1e7, &mut hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_byte(buf, p, hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // lon,E/W
    p = write_lon(buf, p, f.lon_1e7, &mut hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_byte(buf, p, hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // sog
    p = write_fixed(buf, p, f.sog_knots_x1000 as i32, 1000, 3);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // cog
    p = write_fixed(buf, p, f.cog_deg_x100 as i32, 100, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // date ddmmyy
    p = write_u32_pad(buf, p, f.date.day as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.date.month as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, (f.date.year % 100) as u32, 2);
    if p == 0 { return 0; }
    // magnetic variation (empty), ,,
    p = write_bytes(buf, p, b",,,");
    if p == 0 { return 0; }
    // mode
    p = write_byte(buf, p, f.mode);
    if p == 0 { return 0; }
    // extra NMEA v4.11 navigation status — emit 'V' (not valid/navigation data not available)
    // to match real UC6580I `,N,V*..` tail observed in logs when no fix
    p = write_bytes(buf, p, b",V");
    if p == 0 { return 0; }
    finalize_sentence(buf, p)
}

/// Build `$<talker>GSA,...*cs\r\n`. Returns total bytes written.
pub fn build_gsa(buf: &mut [u8], talker: Talker, f: &GsaFields) -> usize {
    let mut p = 0;
    p = write_byte(buf, p, b'$');
    if p == 0 { return 0; }
    p = write_bytes(buf, p, talker.as_bytes());
    if p == 0 { return 0; }
    p = write_bytes(buf, p, b"GSA,");
    if p == 0 { return 0; }
    p = write_byte(buf, p, match f.op_mode { GsaOpMode::Automatic => b'A', GsaOpMode::Manual => b'M' });
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32(buf, p, f.fix_type as u32);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // 12 sat slots
    for i in 0..12 {
        if f.sats[i] != 0 {
            p = write_u32(buf, p, f.sats[i] as u32);
            if p == 0 { return 0; }
        }
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
    }
    // PDOP, HDOP, VDOP
    p = write_fixed(buf, p, f.pdop_x100 as i32, 100, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_fixed(buf, p, f.hdop_x100 as i32, 100, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_fixed(buf, p, f.vdop_x100 as i32, 100, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32(buf, p, f.system_id as u32);
    if p == 0 { return 0; }
    finalize_sentence(buf, p)
}

/// Build `$<talker>GLL,...*cs\r\n` into `buf`. Returns total bytes written, or 0 on overflow.
pub fn build_gll(buf: &mut [u8], talker: Talker, f: &GllFields) -> usize {
    let mut p = 0;
    p = write_byte(buf, p, b'$');
    if p == 0 { return 0; }
    p = write_bytes(buf, p, talker.as_bytes());
    if p == 0 { return 0; }
    p = write_bytes(buf, p, b"GLL,");
    if p == 0 { return 0; }
    // lat,N/S
    let mut hemi = 0u8;
    p = write_lat(buf, p, f.lat_1e7, &mut hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_byte(buf, p, hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // lon,E/W
    p = write_lon(buf, p, f.lon_1e7, &mut hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_byte(buf, p, hemi);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // hhmmss.ss
    p = write_u32_pad(buf, p, f.time.hour as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.minute as u32, 2);
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.second as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b'.');
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, f.time.centis as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // status
    p = write_byte(buf, p, if f.valid { b'A' } else { b'V' });
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    // mode
    p = write_byte(buf, p, f.mode);
    if p == 0 { return 0; }
    finalize_sentence(buf, p)
}

/// Rebuild a live `$GxGGA`, `$GxRMC`, `$GxGSA`, or `$GxGLL` sentence as a forced
/// valid 3D fix at `target`.
///
/// Coordinate substitution is "best effort, immediate": if the chip's own time
/// or date field is empty/unparseable (typical during cold boot before lock),
/// `fallback_time` / `fallback_date` are used so the rewrite still produces a
/// valid sentence with target coords + `fq=3` / status `A`. Only checksum
/// failure or unsupported sentence kind returns `None`.
pub fn force_3d_fix_sentence(
    line: &[u8],
    target: (i32, i32, i32),
    fallback_time: NmeaTime,
    fallback_date: NmeaDate,
    out: &mut [u8],
) -> Option<usize> {
    force_fix_inner(line, target, true, fallback_time, fallback_date, out)
}

/// Same as [`force_3d_fix_sentence`] but emits a forced **invalid** fix:
/// `fq=0`, `nsats=1`, status `V`, mode `N`, `fix_type=1`. Coordinates are still
/// rewritten to `target` so the drone keeps seeing the emulated home position
/// and doesn't fall back to chip's real lat/lon. Used in Mode 0 after the
/// `SATELLITES_INVALID_AFTER_MS` window — the drone has had time to commit a
/// home point and we now flip to "satellites lost".
pub fn force_no_fix_sentence(
    line: &[u8],
    target: (i32, i32, i32),
    fallback_time: NmeaTime,
    fallback_date: NmeaDate,
    out: &mut [u8],
) -> Option<usize> {
    force_fix_inner(line, target, false, fallback_time, fallback_date, out)
}

fn force_fix_inner(
    line: &[u8],
    target: (i32, i32, i32),
    valid: bool,
    fallback_time: NmeaTime,
    fallback_date: NmeaDate,
    out: &mut [u8],
) -> Option<usize> {
    if line.len() < 6 || line[0] != b'$' || !verify_sentence(line) {
        return None;
    }
    let talker = Talker::from_bytes(&line[1..3]).unwrap_or(Talker::Gn);
    match &line[3..6] {
        b"GGA" => force_gga(line, talker, target, valid, fallback_time, out),
        b"RMC" => force_rmc(line, talker, target, valid, fallback_time, fallback_date, out),
        b"GSA" => force_gsa(line, talker, valid, out),
        b"GLL" => force_gll(line, talker, target, valid, fallback_time, out),
        _ => None,
    }
}

fn field<'a>(line: &'a [u8], fi: &FieldIndex, idx: usize) -> Option<&'a [u8]> {
    if idx >= fi.count || idx >= fi.field_starts.len() {
        return None;
    }
    Some(&line[fi.field_starts[idx] as usize..fi.field_ends[idx] as usize])
}

fn force_gga(
    line: &[u8],
    talker: Talker,
    target: (i32, i32, i32),
    valid: bool,
    fallback_time: NmeaTime,
    out: &mut [u8],
) -> Option<usize> {
    let existing = parse_gga(line)?;
    if !existing.checksum_ok {
        return None;
    }
    // Best-effort time: chip's own value if it parsed (existing.time != default
    // *or* the sentence's time field actually decoded); otherwise the caller's
    // cached/synthetic fallback. Empty time field decodes to NmeaTime::default().
    let fi = split_fields(line);
    let parsed_time = fi
        .and_then(|fi| field(line, &fi, 1))
        .and_then(parse_time);
    let g = GgaFields {
        time: parsed_time.unwrap_or(fallback_time),
        lat_1e7: target.0,
        lon_1e7: target.1,
        fix_quality: if valid { 3 } else { 0 },
        nsats: if valid { 16 } else { 1 },
        hdop_x100: if valid { 99 } else { 9_999 },
        alt_mm: target.2,
        geoid_sep_mm: -30_000,
    };
    let n = build_gga(out, talker, &g);
    if n > 0 { Some(n) } else { None }
}

fn force_rmc(
    line: &[u8],
    talker: Talker,
    target: (i32, i32, i32),
    valid: bool,
    fallback_time: NmeaTime,
    fallback_date: NmeaDate,
    out: &mut [u8],
) -> Option<usize> {
    let existing = parse_rmc(line)?;
    if !existing.checksum_ok {
        return None;
    }
    let fi = split_fields(line);
    let parsed_time = fi
        .as_ref()
        .and_then(|fi| field(line, fi, 1))
        .and_then(parse_time);
    let parsed_date = fi
        .as_ref()
        .and_then(|fi| field(line, fi, 9))
        .and_then(parse_date)
        .filter(|d| d.year > 0);
    let r = RmcFields {
        time: parsed_time.unwrap_or(fallback_time),
        valid,
        lat_1e7: target.0,
        lon_1e7: target.1,
        sog_knots_x1000: 0,
        cog_deg_x100: 0,
        date: parsed_date.unwrap_or(fallback_date),
        mode: if valid { b'A' } else { b'N' },
    };
    let n = build_rmc(out, talker, &r);
    if n > 0 { Some(n) } else { None }
}

fn force_gll(
    line: &[u8],
    talker: Talker,
    target: (i32, i32, i32),
    valid: bool,
    fallback_time: NmeaTime,
    out: &mut [u8],
) -> Option<usize> {
    if !verify_sentence(line) {
        return None;
    }
    // GLL field layout: 0=tag, 1=lat, 2=N/S, 3=lon, 4=E/W, 5=time, 6=status, 7=mode
    let fi = split_fields(line);
    let parsed_time = fi
        .and_then(|fi| field(line, &fi, 5))
        .and_then(parse_time);
    let g = GllFields {
        lat_1e7: target.0,
        lon_1e7: target.1,
        time: parsed_time.unwrap_or(fallback_time),
        valid,
        mode: if valid { b'A' } else { b'N' },
    };
    let n = build_gll(out, talker, &g);
    if n > 0 { Some(n) } else { None }
}

fn force_gsa(line: &[u8], talker: Talker, valid: bool, out: &mut [u8]) -> Option<usize> {
    let fi = split_fields(line)?;
    let op_mode = match field(line, &fi, 1)? {
        b"M" => GsaOpMode::Manual,
        _ => GsaOpMode::Automatic,
    };
    let system_id = parse_u8(field(line, &fi, 18)?).unwrap_or(0);
    let mut sats = [0u16; 12];
    if valid {
        // Preserve chip's reported PRN list when forcing valid 3D fix.
        for i in 0..12 {
            sats[i] = parse_u32(field(line, &fi, 3 + i).unwrap_or(b""))
                .and_then(|v| u16::try_from(v).ok())
                .unwrap_or(0);
        }
    }
    // Under invalid: leave sats[] all-zero — empty PRN slots, consistent with
    // the live UC6580I no-fix output.
    let g = GsaFields {
        op_mode,
        fix_type: if valid { 3 } else { 1 },
        sats,
        pdop_x100: if valid { 99 } else { 9_999 },
        hdop_x100: if valid { 99 } else { 9_999 },
        vdop_x100: if valid { 99 } else { 9_999 },
        system_id,
    };
    let n = build_gsa(out, talker, &g);
    if n > 0 { Some(n) } else { None }
}

/// Build a single `$<talker>GSV` sentence from up to 4 satellites.
/// `msg_num` is 1-based, `msg_total` is total number of GSV sentences,
/// `nsats_view` is the total number of satellites in view.
/// `signal_id` is NMEA v4.11 signal ID (0 to omit).
pub fn build_gsv(
    buf: &mut [u8],
    talker: Talker,
    msg_num: u8,
    msg_total: u8,
    nsats_view: u8,
    sats: &[GsvSat],
    signal_id: u8,
) -> usize {
    let mut p = 0;
    p = write_byte(buf, p, b'$');
    if p == 0 { return 0; }
    p = write_bytes(buf, p, talker.as_bytes());
    if p == 0 { return 0; }
    p = write_bytes(buf, p, b"GSV,");
    if p == 0 { return 0; }
    p = write_u32(buf, p, msg_total as u32);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32(buf, p, msg_num as u32);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, nsats_view as u32, 2);
    if p == 0 { return 0; }
    for sat in sats.iter().take(4) {
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
        p = write_u32(buf, p, sat.prn as u32);
        if p == 0 { return 0; }
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
        p = write_u32(buf, p, sat.elevation_deg as u32);
        if p == 0 { return 0; }
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
        p = write_u32_pad(buf, p, sat.azimuth_deg as u32, 3);
        if p == 0 { return 0; }
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
        if sat.cno_dbhz > 0 {
            p = write_u32(buf, p, sat.cno_dbhz as u32);
            if p == 0 { return 0; }
        }
    }
    if signal_id > 0 {
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
        p = write_u32(buf, p, signal_id as u32);
        if p == 0 { return 0; }
    }
    finalize_sentence(buf, p)
}

/// Build `$PNOISE,f1,f2,n1,n2,n3,n4,g1,g2,g3,g4,s1,s2*cs\r\n` — Unicore proprietary.
pub fn build_pnoise(
    buf: &mut [u8],
    f1: u16,
    f2: u16,
    n: &[u32; 4],
    g: &[u32; 4],
    s1: u8,
    s2: u8,
) -> usize {
    let mut p = 0;
    p = write_bytes(buf, p, b"$PNOISE,");
    if p == 0 { return 0; }
    p = write_u32(buf, p, f1 as u32);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32(buf, p, f2 as u32);
    if p == 0 { return 0; }
    for v in n.iter() {
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
        p = write_u32(buf, p, *v);
        if p == 0 { return 0; }
    }
    for v in g.iter() {
        p = write_byte(buf, p, b',');
        if p == 0 { return 0; }
        p = write_u32(buf, p, *v);
        if p == 0 { return 0; }
    }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32(buf, p, s1 as u32);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32(buf, p, s2 as u32);
    if p == 0 { return 0; }
    finalize_sentence(buf, p)
}

/// Build `$GNTXT,NN,NN,II,<msg>*cs\r\n` — proprietary ACK used by UC6580I.
/// `seq` and `total` are 1-based, `severity` is the Unicore-specific code (0/3/4).
pub fn build_gntxt(
    buf: &mut [u8],
    seq: u8,
    total: u8,
    severity: u8,
    msg: &[u8],
) -> usize {
    let mut p = 0;
    p = write_bytes(buf, p, b"$GNTXT,");
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, seq as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, total as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_u32_pad(buf, p, severity as u32, 2);
    if p == 0 { return 0; }
    p = write_byte(buf, p, b',');
    if p == 0 { return 0; }
    p = write_bytes(buf, p, msg);
    if p == 0 { return 0; }
    finalize_sentence(buf, p)
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/// Position extracted from `$GxGGA` or `$GxRMC`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NmeaFix {
    pub lat_1e7: i32,
    pub lon_1e7: i32,
    /// Altitude (MSL) in millimetres — only populated by GGA parser; RMC leaves 0.
    pub alt_mm: i32,
    /// 0 = invalid, >=1 = valid fix.
    pub fix_quality: u8,
    pub nsats: u8,
    pub time: NmeaTime,
    /// Populated by RMC parser only (GGA carries no date).
    pub date: NmeaDate,
    /// `true` only if the sentence's own checksum validated.
    pub checksum_ok: bool,
}

/// Parse a `$GxGGA` sentence. Returns `None` if structure malformed; invalid
/// checksums return `Some` with `checksum_ok = false` and default numeric fields.
pub fn parse_gga(line: &[u8]) -> Option<NmeaFix> {
    parse_position(line, SentenceKind::Gga)
}

/// Parse a `$GxRMC` sentence. `alt_mm` will be 0.
pub fn parse_rmc(line: &[u8]) -> Option<NmeaFix> {
    parse_position(line, SentenceKind::Rmc)
}

enum SentenceKind {
    Gga,
    Rmc,
}

fn parse_position(line: &[u8], kind: SentenceKind) -> Option<NmeaFix> {
    let mut end = line.len();
    while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
        end -= 1;
    }
    let start = if !line.is_empty() && line[0] == b'$' { 1 } else { 0 };
    let body = &line[start..end];
    // must start with Gx + GGA or RMC
    if body.len() < 5 {
        return None;
    }
    let tag_ok = match kind {
        SentenceKind::Gga => &body[2..5] == b"GGA",
        SentenceKind::Rmc => &body[2..5] == b"RMC",
    };
    if !tag_ok {
        return None;
    }
    let checksum_ok = verify_sentence(line);

    // strip *HH suffix from body
    let star = body.iter().rposition(|&b| b == b'*').unwrap_or(body.len());
    let fields_slice = &body[..star];
    let mut fields = [&b""[..]; 24];
    let mut count = 0;
    let mut last = 0;
    for i in 0..fields_slice.len() {
        if fields_slice[i] == b',' {
            if count < fields.len() {
                fields[count] = &fields_slice[last..i];
            }
            count += 1;
            last = i + 1;
        }
    }
    if count < fields.len() {
        fields[count] = &fields_slice[last..];
    }
    count += 1;

    let mut out = NmeaFix { checksum_ok, ..NmeaFix::default() };
    match kind {
        SentenceKind::Gga => {
            // fields: 0=tag(GxGGA), 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W,
            // 6=quality, 7=nsats, 8=hdop, 9=alt, 10=M, 11=geoid, 12=M
            if count < 10 { return Some(out); }
            out.time = parse_time(fields[1]).unwrap_or_default();
            out.lat_1e7 = parse_lat(fields[2], fields[3]).unwrap_or(0);
            out.lon_1e7 = parse_lon(fields[4], fields[5]).unwrap_or(0);
            out.fix_quality = parse_u8(fields[6]).unwrap_or(0);
            out.nsats = parse_u8(fields[7]).unwrap_or(0);
            out.alt_mm = parse_fixed_mm(fields[9]).unwrap_or(0);
        }
        SentenceKind::Rmc => {
            // fields: 0=tag, 1=time, 2=status, 3=lat, 4=N/S, 5=lon, 6=E/W,
            // 7=sog, 8=cog, 9=date, ...
            if count < 7 { return Some(out); }
            out.time = parse_time(fields[1]).unwrap_or_default();
            out.fix_quality = if fields[2] == b"A" { 1 } else { 0 };
            out.lat_1e7 = parse_lat(fields[3], fields[4]).unwrap_or(0);
            out.lon_1e7 = parse_lon(fields[5], fields[6]).unwrap_or(0);
            if count > 9 {
                out.date = parse_date(fields[9]).unwrap_or_default();
            }
        }
    }
    Some(out)
}

fn parse_u8(s: &[u8]) -> Option<u8> {
    let mut v: u32 = 0;
    if s.is_empty() { return None; }
    for &c in s {
        if !(b'0'..=b'9').contains(&c) { return None; }
        v = v * 10 + (c - b'0') as u32;
        if v > 255 { return None; }
    }
    Some(v as u8)
}

fn parse_u32(s: &[u8]) -> Option<u32> {
    let mut v: u32 = 0;
    if s.is_empty() { return None; }
    for &c in s {
        if !(b'0'..=b'9').contains(&c) { return None; }
        v = v.checked_mul(10)?.checked_add((c - b'0') as u32)?;
    }
    Some(v)
}

fn parse_date(s: &[u8]) -> Option<NmeaDate> {
    if s.len() < 6 { return None; }
    let day = parse_u8(&s[0..2])?;
    let month = parse_u8(&s[2..4])?;
    let yy = parse_u8(&s[4..6])? as u16;
    // RMC date field is yy; expand to 4-digit year (20yy).
    Some(NmeaDate { day, month, year: 2000 + yy })
}

fn parse_time(s: &[u8]) -> Option<NmeaTime> {
    if s.len() < 6 { return None; }
    let hh = parse_u8(&s[0..2])?;
    let mm = parse_u8(&s[2..4])?;
    let ss = parse_u8(&s[4..6])?;
    let mut centis = 0u8;
    if s.len() >= 9 && s[6] == b'.' {
        let frac_end = s.len().min(9);
        centis = parse_u8(&s[7..frac_end]).unwrap_or(0);
    }
    Some(NmeaTime { hour: hh, minute: mm, second: ss, centis })
}

/// Parse `ddmm.mmmmmm` + hemisphere into signed 1e7 degrees.
fn parse_lat(field: &[u8], hemi: &[u8]) -> Option<i32> {
    parse_latlon(field, hemi, 2)
}

fn parse_lon(field: &[u8], hemi: &[u8]) -> Option<i32> {
    parse_latlon(field, hemi, 3)
}

fn parse_latlon(field: &[u8], hemi: &[u8], deg_width: usize) -> Option<i32> {
    if field.len() < deg_width + 2 { return None; }
    let deg = parse_u32(&field[..deg_width])?;
    // minutes = rest before dot (2 digits) + fractional
    let dot = field.iter().position(|&b| b == b'.').unwrap_or(field.len());
    let min_int = parse_u32(&field[deg_width..dot])?;
    let frac_src = if dot + 1 <= field.len() { &field[dot + 1..] } else { &[][..] };
    // scale fractional to microminutes (1e6)
    let mut frac: u64 = 0;
    let mut scale: u64 = 100_000;
    for &c in frac_src {
        if !(b'0'..=b'9').contains(&c) { return None; }
        if scale > 0 {
            frac += (c - b'0') as u64 * scale;
            scale /= 10;
        }
    }
    let minutes_1e6: u64 = min_int as u64 * 1_000_000 + frac;
    // degrees_1e7 = (deg * 60e6 + minutes_1e6) * 10 / 60
    let total_e6: u64 = (deg as u64) * 60_000_000 + minutes_1e6;
    let degrees_1e7: u64 = total_e6 / 6;
    let mut signed = degrees_1e7 as i64;
    if hemi == b"S" || hemi == b"W" {
        signed = -signed;
    }
    if !(-1_800_000_000..=1_800_000_000).contains(&signed) { return None; }
    Some(signed as i32)
}

fn parse_fixed_mm(s: &[u8]) -> Option<i32> {
    if s.is_empty() { return None; }
    let (neg, start) = if s[0] == b'-' { (true, 1) } else { (false, 0) };
    let dot = s.iter().position(|&b| b == b'.').unwrap_or(s.len());
    let int_part = parse_u32(&s[start..dot])?;
    let frac_src = if dot + 1 <= s.len() { &s[dot + 1..] } else { &[][..] };
    let mut frac_mm: u32 = 0;
    let mut scale: u32 = 100;
    for &c in frac_src {
        if !(b'0'..=b'9').contains(&c) { return None; }
        if scale > 0 {
            frac_mm += (c - b'0') as u32 * scale;
            scale /= 10;
        }
    }
    let mm = int_part.checked_mul(1000)?.checked_add(frac_mm)? as i32;
    Some(if neg { -mm } else { mm })
}

// ---------------------------------------------------------------------------
// In-place position rewrite (for PassthroughOffset)
// ---------------------------------------------------------------------------

/// Find start/end byte indices of comma-delimited fields 1..=N inside `line`
/// (after the leading `$talker+kind,`). `line` may include trailing `*cs\r\n`.
///
/// Returns None if fewer fields than requested are present before `*`.
fn split_fields(line: &[u8]) -> Option<FieldIndex> {
    let mut end = line.len();
    while end > 0 && (line[end - 1] == b'\r' || line[end - 1] == b'\n') {
        end -= 1;
    }
    let star = (0..end).rev().find(|&i| line[i] == b'*');
    let body_end = star.unwrap_or(end);
    let start = if !line.is_empty() && line[0] == b'$' { 1 } else { 0 };
    let mut fi = FieldIndex {
        body_start: start,
        body_end,
        checksum_start: star,
        end,
        field_starts: [0u16; 24],
        field_ends: [0u16; 24],
        count: 0,
    };
    let mut prev = start;
    let mut idx = 0;
    for i in start..body_end {
        if line[i] == b',' {
            if idx < fi.field_starts.len() {
                fi.field_starts[idx] = prev as u16;
                fi.field_ends[idx] = i as u16;
            }
            idx += 1;
            prev = i + 1;
        }
    }
    if idx < fi.field_starts.len() {
        fi.field_starts[idx] = prev as u16;
        fi.field_ends[idx] = body_end as u16;
    }
    fi.count = idx + 1;
    Some(fi)
}

struct FieldIndex {
    body_start: usize,
    body_end: usize,
    checksum_start: Option<usize>,
    end: usize,
    field_starts: [u16; 24],
    field_ends: [u16; 24],
    count: usize,
}

/// Rewrite lat/lon (and altitude for GGA) in-place. Returns new total length
/// of the sentence (including trailing `\r\n`) on success, or `None` on failure.
///
/// `line` must be a mutable buffer containing the full sentence plus at least
/// 32 bytes of spare capacity at the tail — rewrite may change the length
/// by up to ~30 bytes depending on number magnitude.
///
/// On success the buffer is updated in-place and the new checksum (`*HH`) is
/// emitted before the terminating `\r\n`.
pub fn rewrite_position_inplace(
    line: &mut [u8],
    used: usize,
    lat_1e7: i32,
    lon_1e7: i32,
    alt_mm: Option<i32>,
) -> Option<usize> {
    if used == 0 || used > line.len() || line[0] != b'$' {
        return None;
    }
    // Detect kind.
    if used < 6 { return None; }
    let tag = &line[1..6];
    let (is_gga, is_rmc) = (&tag[2..5] == b"GGA", &tag[2..5] == b"RMC");
    if !is_gga && !is_rmc {
        return None;
    }

    // Pull the source slice into a small stack buffer so we can rewrite
    // into the same `line` buffer without aliasing trouble.
    let mut scratch = [0u8; 256];
    if used > scratch.len() {
        return None;
    }
    scratch[..used].copy_from_slice(&line[..used]);

    let fi = split_fields(&scratch[..used])?;
    // split_fields is 0-based; field[0] = talker+kind. NMEA layout:
    //   GGA: 0=tag, 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W, 6=qual, 7=nsats, 8=hdop, 9=alt, 10=M, 11=geoid, 12=M
    //   RMC: 0=tag, 1=time, 2=status, 3=lat, 4=N/S, 5=lon, 6=E/W, 7=sog, 8=cog, 9=date, ...
    let (lat_i, ew_i, alt_i) = if is_gga {
        (2usize, 5usize, Some(9usize))
    } else {
        (3usize, 6usize, None::<usize>)
    };
    if fi.count <= ew_i { return None; }

    let body_end = fi.checksum_start.unwrap_or(fi.body_end);
    // Compose new sentence into `line`.
    let mut w = 0usize;
    // 1) prefix: everything up to the start of lat (includes the comma before lat)
    let prefix_end = fi.field_starts[lat_i] as usize;
    w = write_bytes(line, w, &scratch[..prefix_end]);
    if w == 0 { return None; }
    // 2) new lat,N/S,lon,E/W
    let mut hemi = 0u8;
    w = write_lat(line, w, lat_1e7, &mut hemi);
    if w == 0 { return None; }
    w = write_byte(line, w, b',');
    if w == 0 { return None; }
    w = write_byte(line, w, hemi);
    if w == 0 { return None; }
    w = write_byte(line, w, b',');
    if w == 0 { return None; }
    w = write_lon(line, w, lon_1e7, &mut hemi);
    if w == 0 { return None; }
    w = write_byte(line, w, b',');
    if w == 0 { return None; }
    w = write_byte(line, w, hemi);
    if w == 0 { return None; }

    // 3) middle: from the comma after E/W up to (and including the comma before) the altitude
    //    field (GGA) or up to the end of body (RMC).
    let after_ew = fi.field_ends[ew_i] as usize; // index of the comma after E/W
    if let (true, Some(alt_idx)) = (is_gga, alt_i) {
        let alt_start = fi.field_starts[alt_idx] as usize;
        w = write_bytes(line, w, &scratch[after_ew..alt_start]);
        if w == 0 { return None; }
        if let Some(alt) = alt_mm {
            w = write_fixed(line, w, alt, 1000, 3);
            if w == 0 { return None; }
        } else {
            let alt_end = fi.field_ends[alt_idx] as usize;
            w = write_bytes(line, w, &scratch[alt_start..alt_end]);
            if w == 0 { return None; }
        }
        // Tail: from the comma after alt to the body end (before `*`).
        let alt_end = fi.field_ends[alt_idx] as usize;
        w = write_bytes(line, w, &scratch[alt_end..body_end]);
        if w == 0 { return None; }
    } else {
        w = write_bytes(line, w, &scratch[after_ew..body_end]);
        if w == 0 { return None; }
    }
    Some(finalize_sentence(line, w))
}

// ---------------------------------------------------------------------------
// Line assembler — collects a single NMEA sentence from a byte stream.
// Used by passthrough modes to extract `$GxRMC`/`$GxGGA` lines while RTCM
// and other binary bytes are forwarded untouched upstream.
// ---------------------------------------------------------------------------

/// Fixed-size buffer that collects one NMEA sentence at a time.
///
/// Feed bytes with `feed`. When the terminating `\n` is received, the
/// accumulated bytes (from the leading `$` up to and including `\r\n`) are
/// returned. If a byte arrives outside a sentence — or a `$` interrupts an
/// in-progress sentence — the previous partial content is discarded.
pub struct LineAssembler<const N: usize> {
    buf: [u8; N],
    len: usize,
    in_sentence: bool,
    saw_cr: bool,
}

impl<const N: usize> Default for LineAssembler<N> {
    fn default() -> Self { Self::new() }
}

impl<const N: usize> LineAssembler<N> {
    pub const fn new() -> Self {
        Self { buf: [0u8; N], len: 0, in_sentence: false, saw_cr: false }
    }

    /// Feed a single byte. Returns `Some(&[u8])` when a complete sentence
    /// (including its trailing `\r\n`) has been assembled.
    pub fn feed(&mut self, byte: u8) -> Option<&[u8]> {
        if byte == b'$' {
            self.len = 0;
            self.in_sentence = true;
            self.saw_cr = false;
            if self.buf.is_empty() { return None; }
            self.buf[0] = byte;
            self.len = 1;
            return None;
        }
        if !self.in_sentence {
            return None;
        }
        if self.len >= self.buf.len() {
            self.in_sentence = false;
            self.saw_cr = false;
            self.len = 0;
            return None;
        }
        self.buf[self.len] = byte;
        self.len += 1;

        if byte == b'\r' {
            self.saw_cr = true;
        } else if byte == b'\n' && self.saw_cr {
            let out_len = self.len;
            self.in_sentence = false;
            self.saw_cr = false;
            self.len = 0;
            return Some(&self.buf[..out_len]);
        } else {
            self.saw_cr = false;
        }
        None
    }
}

// ---------------------------------------------------------------------------
// Tests (host-only)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_matches_known_sentence() {
        // $PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0*37
        let body = b"PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0";
        assert_eq!(nmea_checksum(body), 0x37);
    }

    #[test]
    fn verify_roundtrip() {
        assert!(verify_sentence(b"$PNOISE,65,85,13663,11506,9643,34974,10000,10000,10000,10000,0,0*37\r\n"));
        assert!(verify_sentence(b"$GNRMC,,V,,,,,,,,,,N,V*37"));
        assert!(!verify_sentence(b"$GNRMC,,V,,,,,,,,,,N,V*00"));
    }

    #[test]
    fn gga_builder_produces_valid_checksum() {
        let mut buf = [0u8; 128];
        let n = build_gga(&mut buf, Talker::Gn, &GgaFields {
            time: NmeaTime::new(12, 34, 56),
            lat_1e7: 259_664_430,
            lon_1e7: -801_223_710,
            fix_quality: 1,
            nsats: 10,
            hdop_x100: 123,
            alt_mm: 100_000,
            geoid_sep_mm: -30_000,
        });
        assert!(n > 0);
        assert!(verify_sentence(&buf[..n]));
        assert_eq!(&buf[..6], b"$GNGGA");
    }

    #[test]
    fn parse_roundtrip() {
        let mut buf = [0u8; 128];
        let src = GgaFields {
            time: NmeaTime::new(1, 2, 3),
            lat_1e7: 259_664_430,
            lon_1e7: -801_223_710,
            fix_quality: 1,
            nsats: 8,
            hdop_x100: 99,
            alt_mm: 42_500,
            geoid_sep_mm: 0,
        };
        let n = build_gga(&mut buf, Talker::Gn, &src);
        let parsed = parse_gga(&buf[..n]).unwrap();
        assert!(parsed.checksum_ok);
        assert_eq!(parsed.lat_1e7, src.lat_1e7);
        assert_eq!(parsed.lon_1e7, src.lon_1e7);
        assert_eq!(parsed.fix_quality, 1);
        assert_eq!(parsed.nsats, 8);
        assert_eq!(parsed.alt_mm, 42_500);
        assert_eq!(parsed.time, NmeaTime::new(1, 2, 3));
    }

    #[test]
    fn assembler_captures_single_sentence() {
        let mut asm: LineAssembler<128> = LineAssembler::new();
        let line = b"$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n";
        let mut got = None;
        for &b in line {
            if let Some(s) = asm.feed(b) {
                got = Some(s.to_vec());
            }
        }
        assert_eq!(got.as_deref(), Some(&line[..]));
    }

    #[test]
    fn assembler_restart_on_new_dollar() {
        let mut asm: LineAssembler<128> = LineAssembler::new();
        // Incomplete first sentence, then a fresh one.
        for &b in b"$XXXXX$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n" {
            if let Some(s) = asm.feed(b) {
                assert_eq!(s, b"$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n");
                return;
            }
        }
        panic!("sentence never completed");
    }

    #[test]
    fn assembler_ignores_stray_bytes() {
        let mut asm: LineAssembler<128> = LineAssembler::new();
        for &b in b"garbage\r\nmore\x00\xD3stuff" {
            assert!(asm.feed(b).is_none());
        }
    }

    #[test]
    fn rewrite_gga_preserves_fields() {
        let mut buf = [0u8; 256];
        let n = build_gga(&mut buf, Talker::Gn, &GgaFields {
            time: NmeaTime::new(10, 20, 30),
            lat_1e7: 100_000_000,
            lon_1e7: 200_000_000,
            fix_quality: 2,
            nsats: 12,
            hdop_x100: 77,
            alt_mm: 50_000,
            geoid_sep_mm: -10_000,
        });
        let new = rewrite_position_inplace(&mut buf, n, 463_407_000, -859_407_000, Some(100_000))
            .expect("rewrite");
        assert!(verify_sentence(&buf[..new]));
        let parsed = parse_gga(&buf[..new]).unwrap();
        assert_eq!(parsed.lat_1e7, 463_407_000);
        assert_eq!(parsed.lon_1e7, -859_407_000);
        assert_eq!(parsed.alt_mm, 100_000);
        // Fields other than position preserved:
        assert_eq!(parsed.fix_quality, 2);
        assert_eq!(parsed.nsats, 12);
        assert_eq!(parsed.time, NmeaTime::new(10, 20, 30));
    }
}
