######################################
# STM32F446RET6 Makefile
# PayLord FreeRTOS Project
# Cross-platform (Windows & Linux)
######################################

######################################
# Target
######################################
TARGET = PayLordFreeRTOS

######################################
# Build Path
######################################
BUILD_DIR = build

######################################
# Source Files
######################################

# C sources
C_SOURCES = \
Core/Src/bme280.c \
Core/Src/bmi088.c \
Core/Src/data_logger.c \
Core/Src/e22_lib.c \
Core/Src/flight_algorithm.c \
Core/Src/freertos.c \
Core/Src/kalman.c \
Core/Src/l86_gnss.c \
Core/Src/main.c \
Core/Src/packet.c \
Core/Src/quaternion.c \
Core/Src/queternion.c \
Core/Src/sensor_fusion.c \
Core/Src/stm32f4xx_hal_msp.c \
Core/Src/stm32f4xx_hal_timebase_tim.c \
Core/Src/stm32f4xx_it.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/system_stm32f4xx.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c \
FATFS/App/fatfs.c \
FATFS/Target/user_diskio.c \
FATFS/Target/user_diskio_spi.c \
Middlewares/Third_Party/FatFs/src/diskio.c \
Middlewares/Third_Party/FatFs/src/ff.c \
Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
Middlewares/Third_Party/FatFs/src/option/syscall.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Middlewares/Third_Party/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c \
Middlewares/Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c \
Middlewares/Third_Party/SEGGER/SEGGER/SEGGER_RTT.c \
Middlewares/Third_Party/SEGGER/SEGGER/SEGGER_SYSVIEW.c

# ASM sources
ASM_SOURCES = \
Core/Startup/startup_stm32f446retx.s

######################################
# Binaries
######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

######################################
# MCU Flags
######################################
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

######################################
# C Defines
######################################
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F446xx

######################################
# C Includes
######################################
C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IFATFS/Target \
-IFATFS/App \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IMiddlewares/Third_Party/FatFs/src \
-IMiddlewares/Third_Party/SEGGER/Config \
-IMiddlewares/Third_Party/SEGGER/OS \
-IMiddlewares/Third_Party/SEGGER/SEGGER

######################################
# Compiler Flags
######################################
# Debug build
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -O0 -g3 -Wall -fdata-sections -ffunction-sections

# Release build (uncomment for release)
# CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -O2 -Wall -fdata-sections -ffunction-sections

# Generate dependency info
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

######################################
# Linker Flags
######################################
# Linker script
LDSCRIPT = STM32F446RETX_FLASH.ld

# Libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -u _printf_float

######################################
# Build Object List
######################################
# List of objects (subdirectory yapısını koru)
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))

# List of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

######################################
# Build Rules
######################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# Build object files from C sources
$(BUILD_DIR)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	@echo "CC $<"
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Build object files from ASM sources
$(BUILD_DIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	@echo "AS $<"
	@$(AS) -c $(CFLAGS) $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "LINK $(TARGET).elf"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@echo ""
	@echo "✅ Build successful!"
	@$(SZ) $@
	@echo ""

# Generate HEX file
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "HEX $@"
	@$(HEX) $< $@

# Generate BIN file
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "BIN $@"
	@$(BIN) $< $@

# Create build directory
$(BUILD_DIR):
	@mkdir -p $@

######################################
# Clean
######################################
clean:
	-rm -fR $(BUILD_DIR)

######################################
# Flash (OpenOCD)
######################################
flash: all
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

######################################
# Dependencies
######################################
-include $(wildcard $(BUILD_DIR)/*.d)

######################################
# Phony Targets
######################################
.PHONY: all clean flash
