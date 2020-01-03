################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
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
FullF051Bldc.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FullF051Bldc.list \


# All Target
all: FullF051Bldc.elf secondary-outputs

# Tool invocations
FullF051Bldc.elf: $(OBJS) $(USER_OBJS) C:\Users\Alka\STM32CubeIDE\workspace_1.0.2\FullF051Bldc\STM32F051K6TX_FLASH.ld
	arm-none-eabi-gcc -o "FullF051Bldc.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\Alka\STM32CubeIDE\workspace_1.0.2\FullF051Bldc\STM32F051K6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FullF051Bldc.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FullF051Bldc.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FullF051Bldc.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
