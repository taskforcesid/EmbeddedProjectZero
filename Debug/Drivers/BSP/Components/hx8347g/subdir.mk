################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/hx8347g/hx8347g.c 

OBJS += \
./Drivers/BSP/Components/hx8347g/hx8347g.o 

C_DEPS += \
./Drivers/BSP/Components/hx8347g/hx8347g.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/hx8347g/hx8347g.o: ../Drivers/BSP/Components/hx8347g/hx8347g.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32L475xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I"C:/Users/ALADIN/STM32CubeIDE/workspace_1.3.0/EmbeddedProjectZero/Drivers/BSP/B-L475E-IOT01" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hx8347g/hx8347g.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

