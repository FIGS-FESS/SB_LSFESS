################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
DSP2833x_ADC_cal.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_ADC_cal.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_ADC_cal.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_Adc.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_Adc.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_Adc.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_CodeStartBranch.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_CodeStartBranch.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_DefaultIsr.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_DefaultIsr.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_DefaultIsr.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_GlobalVariableDefs.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_GlobalVariableDefs.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_PieCtrl.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_PieCtrl.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_PieCtrl.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_PieVect.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_PieVect.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_PieVect.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_SysCtrl.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_SysCtrl.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_SysCtrl.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_usDelay.obj: C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/source/DSP2833x_usDelay.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="DSP2833x_usDelay.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/ti/ccsv5p5/ccsv5/tools/compiler/c2000_6.2.4/include" --include_path="C:/ti/xdais_7_21_01_07/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2833x/v133/DSP2833x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


