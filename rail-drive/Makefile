TARGET = rail-drive

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

BUILD_DIR = build
ELF = $(BUILD_DIR)/$(TARGET).elf

CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

DEFINES = \
    -DSTM32F446xx \
    -DUSE_HAL_DRIVER

CFLAGS = \
    $(MCU) \
    -Wall \
    -Wextra \
    -O0 \
    -g3 \
    -ffunction-sections \
    -fdata-sections \
    $(DEFINES)

LDFLAGS = \
    $(MCU) \
    -Tvendor/STM32CubeF4/Projects/STM32446E-Nucleo/Templates/STM32CubeIDE/STM32F446RETX_FLASH.ld \
    -Wl,--gc-sections

INCLUDES = \
    -Iinclude \
    -Ivendor/STM32CubeF4/Drivers/CMSIS/Include \
    -Ivendor/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
    -Ivendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc

SOURCES = \
    src/control_timer.c \
    src/main.c \
    src/motion.c \
    src/uart.c \
    vendor/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
    vendor/STM32CubeF4/Projects/STM32446E-Nucleo/Templates/STM32CubeIDE/Example/User/syscalls.c \
    vendor/STM32CubeF4/Projects/STM32446E-Nucleo/Templates/STM32CubeIDE/Example/User/sysmem.c \
    src/ak60.c \
    src/board.c \
    src/can.c \
    src/control.c \
    src/protocol.c \
    vendor/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c

ASM_SOURCES = \
    startup/startup_stm32f446xx.s

OBJECTS = $(addprefix $(BUILD_DIR)/,$(SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

.PHONY: build clean flash debug

build: $(ELF)

$(ELF): $(OBJECTS)
	mkdir -p $(dir $@)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

flash: $(ELF)
	openocd \
	-f interface/stlink-dap.cfg \
	-f target/stm32f4x.cfg \
	-c "reset_config srst_only srst_nogate connect_assert_srst" \
	-c "program $(ELF) verify reset exit"

debug: $(ELF)
	gdb-multiarch \
	-ex "target extended-remote localhost:3333" \
	-ex "monitor reset halt" \
	$(ELF)

clean:
	rm -rf $(BUILD_DIR)
