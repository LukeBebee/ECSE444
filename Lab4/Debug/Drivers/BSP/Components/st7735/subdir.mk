################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/st7735/st7735.c 

OBJS += \
./Drivers/BSP/Components/st7735/st7735.o 

C_DEPS += \
./Drivers/BSP/Components/st7735/st7735.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/st7735/%.o Drivers/BSP/Components/st7735/%.su: ../Drivers/BSP/Components/st7735/%.c Drivers/BSP/Components/st7735/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP" -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP/B-L4S5I-IOT01" -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP/Components/hts221" -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-st7735

clean-Drivers-2f-BSP-2f-Components-2f-st7735:
	-$(RM) ./Drivers/BSP/Components/st7735/st7735.d ./Drivers/BSP/Components/st7735/st7735.o ./Drivers/BSP/Components/st7735/st7735.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-st7735

