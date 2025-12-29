# Makefile for ublox_fake firmware

.PHONY: all clean rp2040 rp2350 flash

# Default target
all: rp2350

# Path to cargo binaries
CARGO_BIN = $(HOME)/.cargo/bin

# Build for RP2040
rp2040:
	cargo build --release --features rp2040 --target thumbv6m-none-eabi
	$(CARGO_BIN)/elf2uf2-rs target/thumbv6m-none-eabi/release/ublox_fake ublox_fake_rp2040.uf2

# Build for RP2350
rp2350:
	cargo build --release --features rp2350 --target thumbv8m.main-none-eabihf
	$(CARGO_BIN)/elf2uf2-rs target/thumbv8m.main-none-eabihf/release/ublox_fake /tmp/temp.uf2
	python3 -c "import sys; \
		RP2040=0xe48bff56; RP2350=0xe48bff59; \
		d=bytearray(open(sys.argv[1],'rb').read()); \
		[d.__setitem__(slice(i*512+0x1C,i*512+0x20), RP2350.to_bytes(4,'little')) for i in range(len(d)//512) if int.from_bytes(d[i*512+0x1C:i*512+0x20],'little')==RP2040]; \
		open(sys.argv[2],'wb').write(d)" /tmp/temp.uf2 ublox_fake_rp2350.uf2
	@echo "Built: ublox_fake_rp2350.uf2"

# Flash via probe-rs
flash:
	cargo run --release

clean:
	cargo clean
	rm -f *.uf2
