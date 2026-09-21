# rail-firmware

This is the firmware for the physical system, controlled by a `NUCLEPO_F446RE` development board.

## `make` commands
```sh
make clean                  # remove `build/`
make BOARD=drive build      # build `build/rail-drive.elf`
make BOARD=teleop build     # build `build/rail-teleop.elf`
make BOARD=drive flash      # flash the selected board with OpenOCD
make BOARD=drive debug      # debug the selected board with gdb-multiarch
```
