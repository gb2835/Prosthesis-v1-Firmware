################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MX/adc.c \
../Drivers/MX/gpio.c \
../Drivers/MX/lptim.c \
../Drivers/MX/spi.c \
../Drivers/MX/stm32l4xx_it.c \
../Drivers/MX/syscalls.c \
../Drivers/MX/sysmem.c \
../Drivers/MX/system_stm32l4xx.c \
../Drivers/MX/usart.c 

OBJS += \
./Drivers/MX/adc.o \
./Drivers/MX/gpio.o \
./Drivers/MX/lptim.o \
./Drivers/MX/spi.o \
./Drivers/MX/stm32l4xx_it.o \
./Drivers/MX/syscalls.o \
./Drivers/MX/sysmem.o \
./Drivers/MX/system_stm32l4xx.o \
./Drivers/MX/usart.o 

C_DEPS += \
./Drivers/MX/adc.d \
./Drivers/MX/gpio.d \
./Drivers/MX/lptim.d \
./Drivers/MX/spi.d \
./Drivers/MX/stm32l4xx_it.d \
./Drivers/MX/syscalls.d \
./Drivers/MX/sysmem.d \
./Drivers/MX/system_stm32l4xx.d \
./Drivers/MX/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MX/%.o Drivers/MX/%.su: ../Drivers/MX/%.c Drivers/MX/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=48000000 -DEXTERNALSAI1_CLOCK_VALUE=2097000 -DEXTERNALSAI2_CLOCK_VALUE=2097000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MX -I../Drivers/EPOS4 -I../Drivers/MCP25625 -I../Drivers/MPU-9255 -I../Drivers/AS5145B -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MX

clean-Drivers-2f-MX:
	-$(RM) ./Drivers/MX/adc.d ./Drivers/MX/adc.o ./Drivers/MX/adc.su ./Drivers/MX/gpio.d ./Drivers/MX/gpio.o ./Drivers/MX/gpio.su ./Drivers/MX/lptim.d ./Drivers/MX/lptim.o ./Drivers/MX/lptim.su ./Drivers/MX/spi.d ./Drivers/MX/spi.o ./Drivers/MX/spi.su ./Drivers/MX/stm32l4xx_it.d ./Drivers/MX/stm32l4xx_it.o ./Drivers/MX/stm32l4xx_it.su ./Drivers/MX/syscalls.d ./Drivers/MX/syscalls.o ./Drivers/MX/syscalls.su ./Drivers/MX/sysmem.d ./Drivers/MX/sysmem.o ./Drivers/MX/sysmem.su ./Drivers/MX/system_stm32l4xx.d ./Drivers/MX/system_stm32l4xx.o ./Drivers/MX/system_stm32l4xx.su ./Drivers/MX/usart.d ./Drivers/MX/usart.o ./Drivers/MX/usart.su

.PHONY: clean-Drivers-2f-MX

