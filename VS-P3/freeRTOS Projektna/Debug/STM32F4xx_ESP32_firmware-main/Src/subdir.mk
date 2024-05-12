################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32F4xx_ESP32_firmware-main/Src/MQTT_dispatcher.c \
../STM32F4xx_ESP32_firmware-main/Src/Wifi_MQTT_Example.c \
../STM32F4xx_ESP32_firmware-main/Src/esp32_manager.c \
../STM32F4xx_ESP32_firmware-main/Src/usart_rx_queue.c 

OBJS += \
./STM32F4xx_ESP32_firmware-main/Src/MQTT_dispatcher.o \
./STM32F4xx_ESP32_firmware-main/Src/Wifi_MQTT_Example.o \
./STM32F4xx_ESP32_firmware-main/Src/esp32_manager.o \
./STM32F4xx_ESP32_firmware-main/Src/usart_rx_queue.o 

C_DEPS += \
./STM32F4xx_ESP32_firmware-main/Src/MQTT_dispatcher.d \
./STM32F4xx_ESP32_firmware-main/Src/Wifi_MQTT_Example.d \
./STM32F4xx_ESP32_firmware-main/Src/esp32_manager.d \
./STM32F4xx_ESP32_firmware-main/Src/usart_rx_queue.d 


# Each subdirectory must supply rules for building sources it contributes
STM32F4xx_ESP32_firmware-main/Src/%.o STM32F4xx_ESP32_firmware-main/Src/%.su STM32F4xx_ESP32_firmware-main/Src/%.cyclo: ../STM32F4xx_ESP32_firmware-main/Src/%.c STM32F4xx_ESP32_firmware-main/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/Uporabnik/Desktop/freeRTOS Projektna/STM32F4xx_ESP32_firmware-main/Src" -I"C:/Users/Uporabnik/Desktop/freeRTOS Projektna/STM32F4xx_ESP32_firmware-main/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32F4xx_ESP32_firmware-2d-main-2f-Src

clean-STM32F4xx_ESP32_firmware-2d-main-2f-Src:
	-$(RM) ./STM32F4xx_ESP32_firmware-main/Src/MQTT_dispatcher.cyclo ./STM32F4xx_ESP32_firmware-main/Src/MQTT_dispatcher.d ./STM32F4xx_ESP32_firmware-main/Src/MQTT_dispatcher.o ./STM32F4xx_ESP32_firmware-main/Src/MQTT_dispatcher.su ./STM32F4xx_ESP32_firmware-main/Src/Wifi_MQTT_Example.cyclo ./STM32F4xx_ESP32_firmware-main/Src/Wifi_MQTT_Example.d ./STM32F4xx_ESP32_firmware-main/Src/Wifi_MQTT_Example.o ./STM32F4xx_ESP32_firmware-main/Src/Wifi_MQTT_Example.su ./STM32F4xx_ESP32_firmware-main/Src/esp32_manager.cyclo ./STM32F4xx_ESP32_firmware-main/Src/esp32_manager.d ./STM32F4xx_ESP32_firmware-main/Src/esp32_manager.o ./STM32F4xx_ESP32_firmware-main/Src/esp32_manager.su ./STM32F4xx_ESP32_firmware-main/Src/usart_rx_queue.cyclo ./STM32F4xx_ESP32_firmware-main/Src/usart_rx_queue.d ./STM32F4xx_ESP32_firmware-main/Src/usart_rx_queue.o ./STM32F4xx_ESP32_firmware-main/Src/usart_rx_queue.su

.PHONY: clean-STM32F4xx_ESP32_firmware-2d-main-2f-Src

