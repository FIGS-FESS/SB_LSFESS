################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
-l"rts2800_fpu32_fast_supplement.lib" \
-l"rts2800_fpu32.lib" \
"./main.obj" \
"./DSP2833x_usDelay.obj" \
"./DSP2833x_SysCtrl.obj" \
"./DSP2833x_PieVect.obj" \
"./DSP2833x_PieCtrl.obj" \
"./DSP2833x_GlobalVariableDefs.obj" \
"./DSP2833x_DefaultIsr.obj" \
"./DSP2833x_CodeStartBranch.obj" \
"./DSP2833x_Adc.obj" \
"./DSP2833x_ADC_cal.obj" \
"C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/cmd/28335_RAM_lnk.cmd" \
"C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/cmd/DSP2833x_Headers_nonBIOS.cmd" \
-l"IQmath_fpu32.lib" \

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
all: test_v5.out

# Tool invocations
test_v5.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data -z -m"test_v5.map" --stack_size=0x380 --warn_sections -i"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/lib" -i"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" -i"C:/Users/Brent/workspace_v5_5/SB_Final_Version" -i"C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/lib" -i"C:/ti/controlSUITE/libs/math/IQmath/v15c/lib" -i"C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/lib" --reread_libs --priority --verbose_diagnostics --issue_remarks --xml_link_info="test_v5_linkInfo.xml" --entry_point=code_start --rom_model -o "test_v5.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "test_v5.out"
	-$(RM) "DSP2833x_Adc.pp" "DSP2833x_DefaultIsr.pp" "DSP2833x_GlobalVariableDefs.pp" "DSP2833x_PieCtrl.pp" "DSP2833x_PieVect.pp" "DSP2833x_SysCtrl.pp" "main.pp" 
	-$(RM) "DSP2833x_ADC_cal.obj" "DSP2833x_Adc.obj" "DSP2833x_CodeStartBranch.obj" "DSP2833x_DefaultIsr.obj" "DSP2833x_GlobalVariableDefs.obj" "DSP2833x_PieCtrl.obj" "DSP2833x_PieVect.obj" "DSP2833x_SysCtrl.obj" "DSP2833x_usDelay.obj" "main.obj" 
	-$(RM) "DSP2833x_ADC_cal.pp" "DSP2833x_CodeStartBranch.pp" "DSP2833x_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

