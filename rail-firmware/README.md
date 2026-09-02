# rail-firmware

This is the firmware for the physical system, controlled by a `NUCLEPO_F446RE` development board.

## `make` commands
```sh
make clean      # remove `build/`
make build      # build `build/board.elf` 
make flash      # flash `build/board.elf` with OpenOCD
make debug      # start `gdb-multiarch` against `localhost:3333` 
```

