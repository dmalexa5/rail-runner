PORT ?= /dev/ttyUSB0
BAUD ?= 115200
SCALE ?= 0.1
CC = avr-gcc
HOST_CC ?= cc
ELF = build/rail-teleop.elf

.PHONY: build clean flash test FORCE
build: $(ELF)

# Preserve mtime when configuration is unchanged; changing SCALE rebuilds the ELF.
build/config.h: FORCE
	@mkdir -p build
	@python3 -c 'import math,pathlib; s=float("$(SCALE)"); assert math.isfinite(s) and s > 0, "SCALE must be positive and finite"; p=pathlib.Path("$@"); text="#define SCALE " + repr(s) + "f\n"; p.write_text(text) if not p.exists() or p.read_text()!=text else None'

$(ELF): main.c motion.c motion.h Makefile build/config.h
	$(CC) -mmcu=atmega328p -DF_CPU=16000000UL -include build/config.h -Os -Wall -Wextra -Werror -std=c11 main.c motion.c -lm -o $@
	avr-size $@

flash: $(ELF)
	avrdude -p atmega328p -c arduino -P "$(PORT)" -b $(BAUD) -D -U flash:w:$(ELF):e

test:
	@mkdir -p build
	$(HOST_CC) -Wall -Wextra -Werror -std=c11 tests/motion_test.c motion.c -lm -o build/motion-test
	./build/motion-test
	$(HOST_CC) -Wall -Wextra -Werror -std=c11 -DSCALE=1.0f tests/motion_test.c motion.c -lm -o build/motion-test-unit
	./build/motion-test-unit
	$(HOST_CC) -Wall -Wextra -Werror -std=c11 tests/firmware_test.c motion.c -lm -o build/firmware-test
	./build/firmware-test

clean:
	rm -f build/rail-teleop.elf build/config.h build/motion-test build/motion-test-unit build/firmware-test
