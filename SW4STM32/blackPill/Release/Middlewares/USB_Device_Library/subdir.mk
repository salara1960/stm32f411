################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/USB_Device_Library/usbd_cdc.o \
./Middlewares/USB_Device_Library/usbd_core.o \
./Middlewares/USB_Device_Library/usbd_ctlreq.o \
./Middlewares/USB_Device_Library/usbd_ioreq.o 

C_DEPS += \
./Middlewares/USB_Device_Library/usbd_cdc.d \
./Middlewares/USB_Device_Library/usbd_core.d \
./Middlewares/USB_Device_Library/usbd_ctlreq.d \
./Middlewares/USB_Device_Library/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/USB_Device_Library/usbd_cdc.o: /home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F411xE -I"/home/alarm/Project/STM32/TEST/blackPill/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/SW4STM32/blackPill/Application/User" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Device_Library/usbd_core.o: /home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F411xE -I"/home/alarm/Project/STM32/TEST/blackPill/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/SW4STM32/blackPill/Application/User" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Device_Library/usbd_ctlreq.o: /home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F411xE -I"/home/alarm/Project/STM32/TEST/blackPill/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/SW4STM32/blackPill/Application/User" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/USB_Device_Library/usbd_ioreq.o: /home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F411xE -I"/home/alarm/Project/STM32/TEST/blackPill/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/SW4STM32/blackPill/Application/User" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Drivers/CMSIS/Include" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/alarm/Project/STM32/TEST/blackPill/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


