################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
exit_escape_box_stm32f4.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
exit_escape_box_stm32f4.list \

OBJCOPY_BIN += \
exit_escape_box_stm32f4.bin \


# All Target
all: exit_escape_box_stm32f4.elf secondary-outputs

# Tool invocations
exit_escape_box_stm32f4.elf: $(OBJS) $(USER_OBJS) C:\STM32Toolchain\projects\exit_escape_box_stm32f4\STM32F411RETX_FLASH.ld
	arm-none-eabi-gcc -o "exit_escape_box_stm32f4.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\STM32Toolchain\projects\exit_escape_box_stm32f4\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="exit_escape_box_stm32f4.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

exit_escape_box_stm32f4.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "exit_escape_box_stm32f4.list"
	@echo 'Finished building: $@'
	@echo ' '

exit_escape_box_stm32f4.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "exit_escape_box_stm32f4.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
