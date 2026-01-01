# Makefile for ublox_fake firmware

.PHONY: all clean rp2040 rp2350 rp2354 flash

# Default target
all: rp2350

# Path to cargo binaries
CARGO_BIN = $(HOME)/.cargo/bin

# RP2350 Family ID for UF2
RP2040_FAMILY = 0xe48bff56
RP2350_FAMILY = 0xe48bff59

# Python script for patching UF2 Family ID
define PATCH_UF2_FAMILY
python3 -c "import sys; \
	OLD=int(sys.argv[3],16); NEW=int(sys.argv[4],16); \
	d=bytearray(open(sys.argv[1],'rb').read()); \
	[d.__setitem__(slice(i*512+0x1C,i*512+0x20), NEW.to_bytes(4,'little')) for i in range(len(d)//512) if int.from_bytes(d[i*512+0x1C:i*512+0x20],'little')==OLD]; \
	open(sys.argv[2],'wb').write(d)" $(1) $(2) $(RP2040_FAMILY) $(RP2350_FAMILY)
endef

# Build for RP2040 (external flash, Cortex-M0+)
rp2040:
	cargo build --release --features rp2040 --target thumbv6m-none-eabi
	$(CARGO_BIN)/elf2uf2-rs target/thumbv6m-none-eabi/release/ublox_fake ublox_fake_rp2040.uf2
	@echo "Built: ublox_fake_rp2040.uf2"

# Build for RP2350 (external flash, Cortex-M33)
rp2350:
	cargo build --release --features rp2350 --target thumbv8m.main-none-eabihf
	$(CARGO_BIN)/elf2uf2-rs target/thumbv8m.main-none-eabihf/release/ublox_fake /tmp/temp.uf2
	$(call PATCH_UF2_FAMILY,/tmp/temp.uf2,ublox_fake_rp2350.uf2)
	@echo "Built: ublox_fake_rp2350.uf2"

# Build for RP2354 (2MB internal flash, Cortex-M33)
# Same binary as RP2350, different output name for clarity
rp2354:
	cargo build --release --features rp2350 --target thumbv8m.main-none-eabihf
	$(CARGO_BIN)/elf2uf2-rs target/thumbv8m.main-none-eabihf/release/ublox_fake /tmp/temp.uf2
	$(call PATCH_UF2_FAMILY,/tmp/temp.uf2,ublox_fake_rp2354.uf2)
	@echo "Built: ublox_fake_rp2354.uf2 (RP2354A with 2MB internal flash)"

# Flash via probe-rs
flash:
	cargo run --release

clean:
	cargo clean
	rm -f *.uf2
