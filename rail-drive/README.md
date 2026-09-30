# rail-drive

This is the firmware for the physical system, controlled by a `NUCLEPO_F446RE` development board.

## `make` commands
```sh
make clean                  # remove `build/`
make BOARD=drive build      # build `build/rail-drive.elf`
make BOARD=drive flash      # flash the drive board with OpenOCD
make BOARD=drive debug      # debug the drive board with gdb-multiarch
```
