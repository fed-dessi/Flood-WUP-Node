################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/croutine.c \
C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/event_groups.c \
C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/list.c \
C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/queue.c \
C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/stream_buffer.c \
C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/tasks.c \
C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/timers.c 

OBJS += \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/croutine.o \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/event_groups.o \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/list.o \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/queue.o \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/stream_buffer.o \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/tasks.o \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/timers.o 

C_DEPS += \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/croutine.d \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/event_groups.d \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/list.d \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/queue.d \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/stream_buffer.d \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/tasks.d \
./gecko_sdk_3.1.0/util/third_party/freertos/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_3.1.0/util/third_party/freertos/Source/croutine.o: C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/croutine.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32MG12P433F1024GM68=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/EFR32MG12P/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/CMSIS/RTOS2/FreeRTOS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/portable/GCC/ARM_CM4F" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/board/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//app/flex/component/common/sl_flex_assert" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/RTOS2/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/memlcd/src/ls013b7dh03" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\config" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\autogen" -Og -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.1.0/util/third_party/freertos/Source/croutine.d" -MT"gecko_sdk_3.1.0/util/third_party/freertos/Source/croutine.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_3.1.0/util/third_party/freertos/Source/event_groups.o: C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/event_groups.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32MG12P433F1024GM68=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/EFR32MG12P/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/CMSIS/RTOS2/FreeRTOS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/portable/GCC/ARM_CM4F" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/board/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//app/flex/component/common/sl_flex_assert" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/RTOS2/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/memlcd/src/ls013b7dh03" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\config" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\autogen" -Og -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.1.0/util/third_party/freertos/Source/event_groups.d" -MT"gecko_sdk_3.1.0/util/third_party/freertos/Source/event_groups.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_3.1.0/util/third_party/freertos/Source/list.o: C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/list.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32MG12P433F1024GM68=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/EFR32MG12P/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/CMSIS/RTOS2/FreeRTOS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/portable/GCC/ARM_CM4F" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/board/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//app/flex/component/common/sl_flex_assert" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/RTOS2/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/memlcd/src/ls013b7dh03" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\config" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\autogen" -Og -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.1.0/util/third_party/freertos/Source/list.d" -MT"gecko_sdk_3.1.0/util/third_party/freertos/Source/list.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_3.1.0/util/third_party/freertos/Source/queue.o: C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/queue.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32MG12P433F1024GM68=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/EFR32MG12P/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/CMSIS/RTOS2/FreeRTOS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/portable/GCC/ARM_CM4F" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/board/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//app/flex/component/common/sl_flex_assert" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/RTOS2/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/memlcd/src/ls013b7dh03" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\config" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\autogen" -Og -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.1.0/util/third_party/freertos/Source/queue.d" -MT"gecko_sdk_3.1.0/util/third_party/freertos/Source/queue.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_3.1.0/util/third_party/freertos/Source/stream_buffer.o: C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/stream_buffer.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32MG12P433F1024GM68=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/EFR32MG12P/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/CMSIS/RTOS2/FreeRTOS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/portable/GCC/ARM_CM4F" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/board/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//app/flex/component/common/sl_flex_assert" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/RTOS2/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/memlcd/src/ls013b7dh03" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\config" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\autogen" -Og -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.1.0/util/third_party/freertos/Source/stream_buffer.d" -MT"gecko_sdk_3.1.0/util/third_party/freertos/Source/stream_buffer.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_3.1.0/util/third_party/freertos/Source/tasks.o: C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/tasks.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32MG12P433F1024GM68=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/EFR32MG12P/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/CMSIS/RTOS2/FreeRTOS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/portable/GCC/ARM_CM4F" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/board/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//app/flex/component/common/sl_flex_assert" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/RTOS2/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/memlcd/src/ls013b7dh03" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\config" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\autogen" -Og -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.1.0/util/third_party/freertos/Source/tasks.d" -MT"gecko_sdk_3.1.0/util/third_party/freertos/Source/tasks.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_3.1.0/util/third_party/freertos/Source/timers.o: C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1/util/third_party/freertos/Source/timers.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DEFR32MG12P433F1024GM68=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/toolchain/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/system/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/Device/SiliconLabs/EFR32MG12P/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/device_init/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/mpu/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/CMSIS/RTOS2/FreeRTOS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//util/third_party/freertos/Source/portable/GCC/ARM_CM4F" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/gpiointerrupt/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/emdrv/uartdrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/udelay/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/board/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/power_manager/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/common" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ble" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/ieee802154" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/zwave" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/protocol/mfm" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/chip/efr32/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//app/flex/component/common/sl_flex_assert" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/RTOS2/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/pa-conversions/efr32xg1x" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//hardware/driver/memlcd/src/ls013b7dh03" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/service/sleeptimer/inc" -I"C:/SiliconLabs/SimplicityStudio/v5_2/developer/sdks/gecko_sdk_suite/v3.1//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\config" -I"C:\Users\Federico\SimplicityStudio\v5_workspace\flood wup node\autogen" -Og -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_3.1.0/util/third_party/freertos/Source/timers.d" -MT"gecko_sdk_3.1.0/util/third_party/freertos/Source/timers.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


