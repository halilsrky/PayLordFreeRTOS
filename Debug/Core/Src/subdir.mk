################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/bme280.c \
../Core/Src/bmi088.c \
../Core/Src/data_logger.c \
../Core/Src/dma.c \
../Core/Src/e22_lib.c \
../Core/Src/flight_algorithm.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/kalman.c \
../Core/Src/l86_gnss.c \
../Core/Src/main.c \
../Core/Src/packet.c \
../Core/Src/quaternion.c \
../Core/Src/queternion.c \
../Core/Src/sensor_fusion.c \
../Core/Src/sensor_mailbox.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/test_modes.c \
../Core/Src/tim.c \
../Core/Src/uart_handler.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/bme280.o \
./Core/Src/bmi088.o \
./Core/Src/data_logger.o \
./Core/Src/dma.o \
./Core/Src/e22_lib.o \
./Core/Src/flight_algorithm.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/kalman.o \
./Core/Src/l86_gnss.o \
./Core/Src/main.o \
./Core/Src/packet.o \
./Core/Src/quaternion.o \
./Core/Src/queternion.o \
./Core/Src/sensor_fusion.o \
./Core/Src/sensor_mailbox.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/test_modes.o \
./Core/Src/tim.o \
./Core/Src/uart_handler.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/bme280.d \
./Core/Src/bmi088.d \
./Core/Src/data_logger.d \
./Core/Src/dma.d \
./Core/Src/e22_lib.d \
./Core/Src/flight_algorithm.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/kalman.d \
./Core/Src/l86_gnss.d \
./Core/Src/main.d \
./Core/Src/packet.d \
./Core/Src/quaternion.d \
./Core/Src/queternion.d \
./Core/Src/sensor_fusion.d \
./Core/Src/sensor_mailbox.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/test_modes.d \
./Core/Src/tim.d \
./Core/Src/uart_handler.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/Halil/STM32CubeIDE/workspace_1.14.1/PayLordFreeRTOS/Middlewares/Third_Party/SEGGER/Config" -I"C:/Users/Halil/STM32CubeIDE/workspace_1.14.1/PayLordFreeRTOS/Middlewares/Third_Party/SEGGER/OS" -I"C:/Users/Halil/STM32CubeIDE/workspace_1.14.1/PayLordFreeRTOS/Middlewares/Third_Party/SEGGER/Patch" -I"C:/Users/Halil/STM32CubeIDE/workspace_1.14.1/PayLordFreeRTOS/Middlewares/Third_Party/SEGGER/SEGGER" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/bme280.cyclo ./Core/Src/bme280.d ./Core/Src/bme280.o ./Core/Src/bme280.su ./Core/Src/bmi088.cyclo ./Core/Src/bmi088.d ./Core/Src/bmi088.o ./Core/Src/bmi088.su ./Core/Src/data_logger.cyclo ./Core/Src/data_logger.d ./Core/Src/data_logger.o ./Core/Src/data_logger.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/e22_lib.cyclo ./Core/Src/e22_lib.d ./Core/Src/e22_lib.o ./Core/Src/e22_lib.su ./Core/Src/flight_algorithm.cyclo ./Core/Src/flight_algorithm.d ./Core/Src/flight_algorithm.o ./Core/Src/flight_algorithm.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/l86_gnss.cyclo ./Core/Src/l86_gnss.d ./Core/Src/l86_gnss.o ./Core/Src/l86_gnss.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/packet.cyclo ./Core/Src/packet.d ./Core/Src/packet.o ./Core/Src/packet.su ./Core/Src/quaternion.cyclo ./Core/Src/quaternion.d ./Core/Src/quaternion.o ./Core/Src/quaternion.su ./Core/Src/queternion.cyclo ./Core/Src/queternion.d ./Core/Src/queternion.o ./Core/Src/queternion.su ./Core/Src/sensor_fusion.cyclo ./Core/Src/sensor_fusion.d ./Core/Src/sensor_fusion.o ./Core/Src/sensor_fusion.su ./Core/Src/sensor_mailbox.cyclo ./Core/Src/sensor_mailbox.d ./Core/Src/sensor_mailbox.o ./Core/Src/sensor_mailbox.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/test_modes.cyclo ./Core/Src/test_modes.d ./Core/Src/test_modes.o ./Core/Src/test_modes.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/uart_handler.cyclo ./Core/Src/uart_handler.d ./Core/Src/uart_handler.o ./Core/Src/uart_handler.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

