################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lis3mdl/lis3mdl.c 

OBJS += \
./Drivers/BSP/Components/lis3mdl/lis3mdl.o 

C_DEPS += \
./Drivers/BSP/Components/lis3mdl/lis3mdl.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lis3mdl/%.o Drivers/BSP/Components/lis3mdl/%.su: ../Drivers/BSP/Components/lis3mdl/%.c Drivers/BSP/Components/lis3mdl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP" -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP/B-L4S5I-IOT01" -I"/Users/chefluke/STM32Cube/ECSE444/Lab4/Drivers/BSP/Components/hts221" -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lis3mdl

clean-Drivers-2f-BSP-2f-Components-2f-lis3mdl:
	-$(RM) ./Drivers/BSP/Components/lis3mdl/lis3mdl.d ./Drivers/BSP/Components/lis3mdl/lis3mdl.o ./Drivers/BSP/Components/lis3mdl/lis3mdl.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lis3mdl

