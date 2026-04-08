################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_bmi330.c \
../Core/Src/app_bmp384.c \
../Core/Src/app_can_GPS16.c \
../Core/Src/app_can_IMU_BARO.c \
../Core/Src/app_neo6m.c \
../Core/Src/bmi3.c \
../Core/Src/bmi330.c \
../Core/Src/bmp3.c \
../Core/Src/bmp3_port.c \
../Core/Src/fdcan.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/spi.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/app_bmi330.o \
./Core/Src/app_bmp384.o \
./Core/Src/app_can_GPS16.o \
./Core/Src/app_can_IMU_BARO.o \
./Core/Src/app_neo6m.o \
./Core/Src/bmi3.o \
./Core/Src/bmi330.o \
./Core/Src/bmp3.o \
./Core/Src/bmp3_port.o \
./Core/Src/fdcan.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/spi.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/app_bmi330.d \
./Core/Src/app_bmp384.d \
./Core/Src/app_can_GPS16.d \
./Core/Src/app_can_IMU_BARO.d \
./Core/Src/app_neo6m.d \
./Core/Src/bmi3.d \
./Core/Src/bmi330.d \
./Core/Src/bmp3.d \
./Core/Src/bmp3_port.d \
./Core/Src/fdcan.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/spi.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_bmi330.cyclo ./Core/Src/app_bmi330.d ./Core/Src/app_bmi330.o ./Core/Src/app_bmi330.su ./Core/Src/app_bmp384.cyclo ./Core/Src/app_bmp384.d ./Core/Src/app_bmp384.o ./Core/Src/app_bmp384.su ./Core/Src/app_can_GPS16.cyclo ./Core/Src/app_can_GPS16.d ./Core/Src/app_can_GPS16.o ./Core/Src/app_can_GPS16.su ./Core/Src/app_can_IMU_BARO.cyclo ./Core/Src/app_can_IMU_BARO.d ./Core/Src/app_can_IMU_BARO.o ./Core/Src/app_can_IMU_BARO.su ./Core/Src/app_neo6m.cyclo ./Core/Src/app_neo6m.d ./Core/Src/app_neo6m.o ./Core/Src/app_neo6m.su ./Core/Src/bmi3.cyclo ./Core/Src/bmi3.d ./Core/Src/bmi3.o ./Core/Src/bmi3.su ./Core/Src/bmi330.cyclo ./Core/Src/bmi330.d ./Core/Src/bmi330.o ./Core/Src/bmi330.su ./Core/Src/bmp3.cyclo ./Core/Src/bmp3.d ./Core/Src/bmp3.o ./Core/Src/bmp3.su ./Core/Src/bmp3_port.cyclo ./Core/Src/bmp3_port.d ./Core/Src/bmp3_port.o ./Core/Src/bmp3_port.su ./Core/Src/fdcan.cyclo ./Core/Src/fdcan.d ./Core/Src/fdcan.o ./Core/Src/fdcan.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

