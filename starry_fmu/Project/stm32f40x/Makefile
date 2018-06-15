SOURCES=$(wildcard ../../Application/*.c) ../../Driver/gpio/gpio.c
OBJS=$(addprefix build/, $(notdir $(SOURCES:.c=.o)))

all:$(OBJS)

test:
	echo $(SOURCES)
	echo $(OBJS)

%.o:
	armclang -xc -std=gnu99 --target=arm-arm-none-eabi -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -gdwarf-3 -O3 -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -I ../../Library/STM_Lib/STM32F4xx_StdPeriph_Driver/inc -I ../../Library/STM_Lib/CMSIS/Include -I ../../Library/STM_Lib/CMSIS/Device/ST/STM32F4xx/Include -I ../../Library/mavlink/v2.0 -I ../../Library/mavlink/v2.0/common -I ../../Library/Fatfs -I ../../Driver/usb/inc -I ../../Driver/include -I ../../RTOS/components/finsh -I ../../RTOS/libcpu/arm/common -I ../../RTOS/libcpu/arm/cortex-m4 -I ../../RTOS/include -I ../../HAL/include -I ../../Framework/include -I ../stm32f40x -I./RTE/_rt-thread_stm32f4xx -IE:/App/Keil_v5/ARM/PACK/ARM/CMSIS/5.0.1/CMSIS/Include -IE:/App/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/2.12.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -D__UVISION_VERSION="523" -D_RTE_ -DSTM32F427xx -DUSE_STDPERIPH_DRIVER -DSTM32F427X -D__VFP_FP__ -DARM_MATH_MATRIX_CHECK -DARM_MATH_CM4 -D__FPU_PRESENT="1" -D__FPU_USED="1" -O3 -o $@ -MD $<

build/starry_fmu.axf:$(OBJS)
	armlink --cpu=Cortex-M4.fp --library_type=microlib --strict --scatter ".\build\starry_fmu.sct" --keep *.o(FSymTab) --keep *.o(VSymTab) --summary_stderr --info summarysizes --map --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --list ".\build\starry_fmu.map" $< -o $@
