################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Application/User/ST7789/subdir.mk
-include Application/User/NET/multicast/subdir.mk
-include Application/User/NET/loopback/subdir.mk
-include Application/User/NET/httpServer/subdir.mk
-include Application/User/NET/W5500/subdir.mk
-include Application/User/NET/TFTP/subdir.mk
-include Application/User/NET/SNTP/subdir.mk
-include Application/User/NET/SNMP/subdir.mk
-include Application/User/NET/MQTT/MQTTPacket/src/subdir.mk
-include Application/User/NET/MQTT/subdir.mk
-include Application/User/NET/FTPServer/subdir.mk
-include Application/User/NET/FTPClient/subdir.mk
-include Application/User/NET/DNS/subdir.mk
-include Application/User/NET/DHCP/subdir.mk
-include Application/User/NET/subdir.mk
-include Application/User/BLE/subdir.mk
-include Application/User/subdir.mk
-include Application/SW4STM32/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: blackPill.elf

# Tool invocations
blackPill.elf: $(OBJS) $(USER_OBJS) ../STM32F411CEUx_FLASH.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nosys.specs -specs=nano.specs -u _printf_float -T"../STM32F411CEUx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "blackPill.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating hex and Printing size information:'
	arm-none-eabi-objcopy -O ihex "blackPill.elf" "blackPill.hex"
	arm-none-eabi-objcopy -O binary "blackPill.elf" "blackPill.bin"
	ls -la | grep "blackPill.*"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
