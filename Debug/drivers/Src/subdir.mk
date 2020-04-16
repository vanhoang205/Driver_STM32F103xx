################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/drv_gpio.c 

OBJS += \
./drivers/Src/drv_gpio.o 

C_DEPS += \
./drivers/Src/drv_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/drv_gpio.o: ../drivers/Src/drv_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"F:/STM32_work/CubeIDE_Workspace/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/drv_gpio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

