################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MCP25625/mcp25625.c 

OBJS += \
./Drivers/MCP25625/mcp25625.o 

C_DEPS += \
./Drivers/MCP25625/mcp25625.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MCP25625/%.o Drivers/MCP25625/%.su Drivers/MCP25625/%.cyclo: ../Drivers/MCP25625/%.c Drivers/MCP25625/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DMSI_VALUE=48000000 -DEXTERNALSAI1_CLOCK_VALUE=2097000 -DEXTERNALSAI2_CLOCK_VALUE=2097000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/EPOS4 -I../Drivers/MCP25625 -I../Drivers/MPU-9255 -I../Drivers/AS5145B -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MCP25625

clean-Drivers-2f-MCP25625:
	-$(RM) ./Drivers/MCP25625/mcp25625.cyclo ./Drivers/MCP25625/mcp25625.d ./Drivers/MCP25625/mcp25625.o ./Drivers/MCP25625/mcp25625.su

.PHONY: clean-Drivers-2f-MCP25625

