################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/Programing_envs/CodeComposerStudio5/ccsv5/tools/compiler/c5500_4.4.1

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./vectors.obj" \
"./unpacki32.obj" \
"./unpacki.obj" \
"./unpack32.obj" \
"./unpack.obj" \
"./twiddle32.obj" \
"./twiddle.obj" \
"./sqrtv.obj" \
"./sine.obj" \
"./q15tofl.obj" \
"./main.obj" \
"./i2c.obj" \
"./gpio.obj" \
"./fltoq15.obj" \
"./cifft_scale.obj" \
"./cifft_noscale.obj" \
"./cifft32_scale.obj" \
"./cfft_scale.obj" \
"./cfft_noscale.obj" \
"./cfft32_scale.obj" \
"./cbrev32.obj" \
"./cbrev.obj" \
"./aic3204.obj" \
"./C5515.obj" \
"../C5515.cmd" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: audio_proj2.out

# Tool invocations
audio_proj2.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C5500 Linker'
	"D:/Programing_envs/CodeComposerStudio5/ccsv5/tools/compiler/c5500_4.4.1/bin/cl55" -v5515 --memory_model=large -g --define=c5515 --display_error_number --diag_warning=225 --ptrdiff_size=16 -z -m"audio_proj2.map" --stack_size=0x200 --heap_size=0x400 -i"D:/Programing_envs/CodeComposerStudio5/ccsv5/tools/compiler/c5500_4.4.1/lib" -i"D:/Programing_envs/CodeComposerStudio5/ccsv5/tools/compiler/c5500_4.4.1/include" --reread_libs --display_error_number --warn_sections --xml_link_info="audio_proj2_linkInfo.xml" --rom_model --sys_stacksize=0x200 -o "audio_proj2.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C5500_EXECUTABLE_OUTPUTS__QUOTED) "audio_proj2.out"
	-$(RM) "C5515.pp" "aic3204.pp" "gpio.pp" "i2c.pp" "main.pp" 
	-$(RM) "C5515.obj" "aic3204.obj" "cbrev.obj" "cbrev32.obj" "cfft32_scale.obj" "cfft_noscale.obj" "cfft_scale.obj" "cifft32_scale.obj" "cifft_noscale.obj" "cifft_scale.obj" "fltoq15.obj" "gpio.obj" "i2c.obj" "main.obj" "q15tofl.obj" "sine.obj" "sqrtv.obj" "twiddle.obj" "twiddle32.obj" "unpack.obj" "unpack32.obj" "unpacki.obj" "unpacki32.obj" "vectors.obj" 
	-$(RM) "cbrev.pp" "cbrev32.pp" "cfft32_scale.pp" "cfft_noscale.pp" "cfft_scale.pp" "cifft32_scale.pp" "cifft_noscale.pp" "cifft_scale.pp" "fltoq15.pp" "q15tofl.pp" "sine.pp" "sqrtv.pp" "twiddle.pp" "twiddle32.pp" "unpack.pp" "unpack32.pp" "unpacki.pp" "unpacki32.pp" "vectors.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

