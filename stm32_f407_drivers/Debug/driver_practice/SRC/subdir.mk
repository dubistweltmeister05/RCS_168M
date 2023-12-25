################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver_practice/SRC/i2c_driver_practice.c \
../driver_practice/SRC/spi_drivers_practice.c 

OBJS += \
./driver_practice/SRC/i2c_driver_practice.o \
./driver_practice/SRC/spi_drivers_practice.o 

C_DEPS += \
./driver_practice/SRC/i2c_driver_practice.d \
./driver_practice/SRC/spi_drivers_practice.d 


# Each subdirectory must supply rules for building sources it contributes
driver_practice/SRC/%.o driver_practice/SRC/%.su driver_practice/SRC/%.cyclo: ../driver_practice/SRC/%.c driver_practice/SRC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/wardawg/Desktop/TEAM AUTOMATONS/STM_32_implementation/stm32_f407_drivers/driver_practice/INC" -I"C:/Users/wardawg/Desktop/TEAM AUTOMATONS/STM_32_implementation/stm32_f407_drivers" -I"C:/Users/wardawg/Desktop/TEAM AUTOMATONS/STM_32_implementation/stm32_f407_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-driver_practice-2f-SRC

clean-driver_practice-2f-SRC:
	-$(RM) ./driver_practice/SRC/i2c_driver_practice.cyclo ./driver_practice/SRC/i2c_driver_practice.d ./driver_practice/SRC/i2c_driver_practice.o ./driver_practice/SRC/i2c_driver_practice.su ./driver_practice/SRC/spi_drivers_practice.cyclo ./driver_practice/SRC/spi_drivers_practice.d ./driver_practice/SRC/spi_drivers_practice.o ./driver_practice/SRC/spi_drivers_practice.su

.PHONY: clean-driver_practice-2f-SRC

