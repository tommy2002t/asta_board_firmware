################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CANBUS_C/app_can_BMI330.c \
../Core/Src/CANBUS_C/app_can_BMP384.c 

OBJS += \
./Core/Src/CANBUS_C/app_can_BMI330.o \
./Core/Src/CANBUS_C/app_can_BMP384.o 

C_DEPS += \
./Core/Src/CANBUS_C/app_can_BMI330.d \
./Core/Src/CANBUS_C/app_can_BMP384.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/CANBUS_C/%.o Core/Src/CANBUS_C/%.su Core/Src/CANBUS_C/%.cyclo: ../Core/Src/CANBUS_C/%.c Core/Src/CANBUS_C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-CANBUS_C

clean-Core-2f-Src-2f-CANBUS_C:
	-$(RM) ./Core/Src/CANBUS_C/app_can_BMI330.cyclo ./Core/Src/CANBUS_C/app_can_BMI330.d ./Core/Src/CANBUS_C/app_can_BMI330.o ./Core/Src/CANBUS_C/app_can_BMI330.su ./Core/Src/CANBUS_C/app_can_BMP384.cyclo ./Core/Src/CANBUS_C/app_can_BMP384.d ./Core/Src/CANBUS_C/app_can_BMP384.o ./Core/Src/CANBUS_C/app_can_BMP384.su

.PHONY: clean-Core-2f-Src-2f-CANBUS_C

