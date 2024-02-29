################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/led_toggle.c 

OBJS += \
./Src/led_toggle.o 

C_DEPS += \
./Src/led_toggle.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/wardawg/Desktop/TEAM AUTOMATONS/STM_32_implementation/stm32_f407_drivers/driver_practice/INC" -I"C:/Users/wardawg/Desktop/TEAM AUTOMATONS/STM_32_implementation/stm32_f407_drivers" -I"C:/Users/wardawg/Desktop/TEAM AUTOMATONS/STM_32_implementation/stm32_f407_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/led_toggle.cyclo ./Src/led_toggle.d ./Src/led_toggle.o ./Src/led_toggle.su

.PHONY: clean-Src

