################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/m24sr/m24sr.c 

OBJS += \
./Drivers/BSP/Components/m24sr/m24sr.o 

C_DEPS += \
./Drivers/BSP/Components/m24sr/m24sr.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/m24sr/%.o Drivers/BSP/Components/m24sr/%.su: ../Drivers/BSP/Components/m24sr/%.c Drivers/BSP/Components/m24sr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP" -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP/B-L4S5I-IOT01" -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP/Components/hts221" -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-m24sr

clean-Drivers-2f-BSP-2f-Components-2f-m24sr:
	-$(RM) ./Drivers/BSP/Components/m24sr/m24sr.d ./Drivers/BSP/Components/m24sr/m24sr.o ./Drivers/BSP/Components/m24sr/m24sr.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-m24sr

