################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Dmitrii/Downloads/ssd1306-stm32HAL-master/ssd1306-stm32HAL-master/ssd1306/fonts.c \
C:/Users/Dmitrii/Downloads/ssd1306-stm32HAL-master/ssd1306-stm32HAL-master/ssd1306/ssd1306.c 

OBJS += \
./ssd1306/fonts.o \
./ssd1306/ssd1306.o 

C_DEPS += \
./ssd1306/fonts.d \
./ssd1306/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
ssd1306/fonts.o: C:/Users/Dmitrii/Downloads/ssd1306-stm32HAL-master/ssd1306-stm32HAL-master/ssd1306/fonts.c ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Dmitrii/Downloads/ssd1306-stm32HAL-master/ssd1306-stm32HAL-master/ssd1306" -I"C:/Users/Dmitrii/Desktop/stm32projs/sdio_SSD1306_test/HTU21D" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ssd1306/ssd1306.o: C:/Users/Dmitrii/Downloads/ssd1306-stm32HAL-master/ssd1306-stm32HAL-master/ssd1306/ssd1306.c ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Dmitrii/Downloads/ssd1306-stm32HAL-master/ssd1306-stm32HAL-master/ssd1306" -I"C:/Users/Dmitrii/Desktop/stm32projs/sdio_SSD1306_test/HTU21D" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ssd1306

clean-ssd1306:
	-$(RM) ./ssd1306/fonts.cyclo ./ssd1306/fonts.d ./ssd1306/fonts.o ./ssd1306/fonts.su ./ssd1306/ssd1306.cyclo ./ssd1306/ssd1306.d ./ssd1306/ssd1306.o ./ssd1306/ssd1306.su

.PHONY: clean-ssd1306
