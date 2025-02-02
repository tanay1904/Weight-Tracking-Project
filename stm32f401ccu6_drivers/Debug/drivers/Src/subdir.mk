################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f401ccu6_gpio_drivers.c \
../drivers/Src/stm32f401ccu6_spi_drivers.c \
../drivers/Src/stm32f401ccu6_uart_drivers.c 

OBJS += \
./drivers/Src/stm32f401ccu6_gpio_drivers.o \
./drivers/Src/stm32f401ccu6_spi_drivers.o \
./drivers/Src/stm32f401ccu6_uart_drivers.o 

C_DEPS += \
./drivers/Src/stm32f401ccu6_gpio_drivers.d \
./drivers/Src/stm32f401ccu6_spi_drivers.d \
./drivers/Src/stm32f401ccu6_uart_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F401CCUx -DSTM32 -DSTM32F4 -c -I../Inc -I"E:/ST/STProjects/stm32f401ccu6_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f401ccu6_gpio_drivers.cyclo ./drivers/Src/stm32f401ccu6_gpio_drivers.d ./drivers/Src/stm32f401ccu6_gpio_drivers.o ./drivers/Src/stm32f401ccu6_gpio_drivers.su ./drivers/Src/stm32f401ccu6_spi_drivers.cyclo ./drivers/Src/stm32f401ccu6_spi_drivers.d ./drivers/Src/stm32f401ccu6_spi_drivers.o ./drivers/Src/stm32f401ccu6_spi_drivers.su ./drivers/Src/stm32f401ccu6_uart_drivers.cyclo ./drivers/Src/stm32f401ccu6_uart_drivers.d ./drivers/Src/stm32f401ccu6_uart_drivers.o ./drivers/Src/stm32f401ccu6_uart_drivers.su

.PHONY: clean-drivers-2f-Src

