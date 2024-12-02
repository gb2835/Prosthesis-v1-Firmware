################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/EPOS4/epos4.c 

OBJS += \
./Drivers/EPOS4/epos4.o 

C_DEPS += \
./Drivers/EPOS4/epos4.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/EPOS4/%.o Drivers/EPOS4/%.su: ../Drivers/EPOS4/%.c Drivers/EPOS4/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/EPOS4 -I../Drivers/MCP25625 -I../Drivers/AS5145B -I../Core/Inc -I../Drivers/MPU-9255 -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-EPOS4

clean-Drivers-2f-EPOS4:
	-$(RM) ./Drivers/EPOS4/epos4.d ./Drivers/EPOS4/epos4.o ./Drivers/EPOS4/epos4.su

.PHONY: clean-Drivers-2f-EPOS4

