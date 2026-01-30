//! Build script for ublox_fake
//!
//! Automatically selects memory layout based on target

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let target = env::var("TARGET").unwrap_or_default();

    // Detect RP2354 feature flag for flash size selection
    let is_rp2354 = env::var("CARGO_FEATURE_RP2354").is_ok();

    // Select memory layout based on target architecture
    let memory_x = if target.contains("thumbv8m") {
        // RP2350/RP2354 (Cortex-M33)
        let flash_size = if is_rp2354 { "2048K" } else { "4096K" };
        let chip_comment = if is_rp2354 {
            "RP2354A (2MB internal flash)"
        } else {
            "RP2350A (Spotpear RP2350-Core-A)"
        };

        // Pattern from working RP2350 examples - start_block AFTER vector_table
        format!(
            "/* Memory layout for {} */\n\
            MEMORY {{\n\
            \x20   FLASH : ORIGIN = 0x10000000, LENGTH = {}\n\
            \x20   RAM   : ORIGIN = 0x20000000, LENGTH = 512K\n\
            \x20   SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K\n\
            \x20   SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K\n\
            }}\n\
            \n\
            _stack_start = ORIGIN(RAM) + LENGTH(RAM);\n\
            \n\
            SECTIONS {{\n\
            \x20   .start_block : ALIGN(4)\n\
            \x20   {{\n\
            \x20       __start_block_addr = .;\n\
            \x20       KEEP(*(.start_block));\n\
            \x20       KEEP(*(.boot_info));\n\
            \x20   }} > FLASH\n\
            }} INSERT AFTER .vector_table;\n\
            \n\
            _stext = ADDR(.start_block) + SIZEOF(.start_block);\n\
            \n\
            SECTIONS {{\n\
            \x20   .bi_entries : ALIGN(4)\n\
            \x20   {{\n\
            \x20       __bi_entries_start = .;\n\
            \x20       KEEP(*(.bi_entries));\n\
            \x20       . = ALIGN(4);\n\
            \x20       __bi_entries_end = .;\n\
            \x20   }} > FLASH\n\
            }} INSERT AFTER .text;\n\
            \n\
            SECTIONS {{\n\
            \x20   .end_block : ALIGN(4)\n\
            \x20   {{\n\
            \x20       __end_block_addr = .;\n\
            \x20       KEEP(*(.end_block));\n\
            \x20   }} > FLASH\n\
            }} INSERT AFTER .uninit;\n\
            \n\
            PROVIDE(start_to_end = __end_block_addr - __start_block_addr);\n\
            PROVIDE(end_to_start = __start_block_addr - __end_block_addr);\n",
            chip_comment, flash_size
        )
    } else {
        // RP2040 (Cortex-M0+) - 264KB RAM, 2MB Flash
        String::from(r#"/* Memory layout for RP2040 */
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
"#)
    };

    let mut f = File::create(out.join("memory.x")).unwrap();
    f.write_all(memory_x.as_bytes()).unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
}
