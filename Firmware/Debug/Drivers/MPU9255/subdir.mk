################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MPU9255/mpu925x_spi.c 

OBJS += \
./Drivers/MPU9255/mpu925x_spi.o 

C_DEPS += \
./Drivers/MPU9255/mpu925x_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MPU9255/%.o Drivers/MPU9255/%.su: ../Drivers/MPU9255/%.c Drivers/MPU9255/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=48000000 -DEXTERNALSAI1_CLOCK_VALUE=2097000 -DEXTERNALSAI2_CLOCK_VALUE=2097000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/EPOS4 -I../Drivers/MCP25625 -I../Drivers/AS5145B -I../Core/Inc -I../Drivers/MPU9255 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MPU9255

clean-Drivers-2f-MPU9255:
	-$(RM) ./Drivers/MPU9255/mpu925x_spi.d ./Drivers/MPU9255/mpu925x_spi.o ./Drivers/MPU9255/mpu925x_spi.su

.PHONY: clean-Drivers-2f-MPU9255

