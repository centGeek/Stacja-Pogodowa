################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Stacja.axf

# Tool invocations
Stacja.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\247832\Documents\Stacja\Stacja pogodowa\Lib_CMSISv1p30_LPC17xx\Debug" -L"C:\Users\247832\Documents\Stacja\Stacja pogodowa\Lib_EaBaseBoard\Debug" -L"C:\Users\247832\Documents\Stacja\Stacja pogodowa\Lib_MCU\Debug" -Xlinker --gc-sections -Xlinker -Map=Stacja.map -mcpu=cortex-m3 -mthumb -T "Program_Debug.ld" -o "Stacja.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) Stacja.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size Stacja.axf; # arm-none-eabi-objdump -h -S Stacja.axf >Stacja.lss
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
