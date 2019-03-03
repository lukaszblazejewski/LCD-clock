################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/LCD.c 

OBJS += \
./Inc/LCD.o 

C_DEPS += \
./Inc/LCD.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/%.o: ../Inc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/£ukasz/Desktop/Semestr VI/STM_1/LCD_ostatnua_proba/LCD_ostatnua_proba/Inc" -I"C:/Users/£ukasz/Desktop/Semestr VI/STM_1/LCD_ostatnua_proba/LCD_ostatnua_proba/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/£ukasz/Desktop/Semestr VI/STM_1/LCD_ostatnua_proba/LCD_ostatnua_proba/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/£ukasz/Desktop/Semestr VI/STM_1/LCD_ostatnua_proba/LCD_ostatnua_proba/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/£ukasz/Desktop/Semestr VI/STM_1/LCD_ostatnua_proba/LCD_ostatnua_proba/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


