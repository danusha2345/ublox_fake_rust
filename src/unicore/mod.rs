//! Unicore UC6580I emulation protocol stack.
//!
//! This module is only active under `#[cfg(feature = "unicore")]` in firmware,
//! but the submodules are pure `no_std` and are compiled unconditionally by
//! the host-side test harness via `#[path = "../../src/unicore/..."]`.

#![allow(dead_code)]

#[path = "nmea.rs"]
pub mod nmea;

#[path = "rtcm3.rs"]
pub mod rtcm3;

#[path = "extrtcm.rs"]
pub mod extrtcm;

#[path = "cmd.rs"]
pub mod cmd;

#[path = "boot.rs"]
pub mod boot;

#[path = "rtcm_samples.rs"]
pub mod rtcm_samples;
