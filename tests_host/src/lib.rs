#[path = "../../src/spoof_detector.rs"]
pub mod spoof_detector;

#[path = "../../src/config.rs"]
pub mod config;

#[path = "../../src/coordinates.rs"]
pub mod coordinates;

#[path = "../../src/passthrough.rs"]
pub mod passthrough;

#[path = "../../src/unicore/mod.rs"]
pub mod unicore;

// Convenience aliases so existing/external tests can import shorter paths.
pub use unicore::nmea as unicore_nmea;
pub use unicore::rtcm3 as unicore_rtcm3;
pub use unicore::extrtcm as unicore_extrtcm;
pub use unicore::cmd as unicore_cmd;
pub use unicore::boot as unicore_boot;

// AnalysisResult doesn't derive Debug in firmware (no std), add it here for assert_eq!
impl core::fmt::Debug for spoof_detector::AnalysisResult {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            spoof_detector::AnalysisResult::Normal => write!(f, "Normal"),
            spoof_detector::AnalysisResult::Spoofed => write!(f, "Spoofed"),
            spoof_detector::AnalysisResult::Initializing => write!(f, "Initializing"),
            spoof_detector::AnalysisResult::GapReset => write!(f, "GapReset"),
        }
    }
}
