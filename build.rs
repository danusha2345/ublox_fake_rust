//! Build script for ublox_fake
//!
//! Embassy-rp provides memory.x via its build script, so we just need
//! to set up the linker search path.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
}
