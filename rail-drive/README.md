# rail-drive

This is the firmware for the physical system, controlled by a `NUCLEPO_F446RE` development board.

## `make` commands
```sh
make clean                  # remove `build/`
make build                  # build `build/rail-drive.elf`
make flash                  # flash the drive board with OpenOCD
make debug                  # debug the drive board with gdb-multiarch
```

Rail position integrates measured CAN velocity at the 1 kHz control rate,
using 5 mm of travel per output-shaft revolution. Calibration times out after
90 seconds. The motor's existing CAN configuration is used without modification.

Host-side checks:
```sh
cc -Wall -Wextra -Werror -Iinclude tests/position_test.c src/ak60.c src/motion.c -lm -o /tmp/rail-position-test
/tmp/rail-position-test
cc -Wall -Wextra -Werror -Iinclude tests/control_test.c src/motion.c -lm -o /tmp/rail-control-test
/tmp/rail-control-test
```
