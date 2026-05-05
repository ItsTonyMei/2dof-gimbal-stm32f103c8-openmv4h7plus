#!/bin/bash
# GCC 交叉编译脚本 — 二自由度云台 STM32F103C8T6 (OpenMV 追踪模式)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$SCRIPT_DIR/stm32"
BUILD="$PROJECT/build_gcc"

CPU="-mcpu=cortex-m3 -mthumb"
FPU="-msoft-float"
SPECS="--specs=nosys.specs --specs=nano.specs"
DEFINES="-DUSE_HAL_DRIVER -DSTM32F103xB"
CFLAGS="$CPU $FPU -Wall -Wextra -Os -ffunction-sections -fdata-sections -fno-common"
LDFLAGS="$CPU $FPU $SPECS -T$PROJECT/MDK-ARM/STM32F103C8TX_FLASH.ld -Wl,--gc-sections -Wl,-Map=$BUILD/output.map"

INC=(
    -I$PROJECT/Core/Inc
    -I$PROJECT/Drivers/CMSIS/Include
    -I$PROJECT/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    -I$PROJECT/Drivers/STM32F1xx_HAL_Driver/Inc
    -I$PROJECT/MiniBalance/CONTROL
    -I$PROJECT/MiniBalance/show
    -I$PROJECT/MiniBalance_HARDWARE/OLED
    -I$PROJECT/MiniBalance_HARDWARE/LED
    -I$PROJECT/SYSTEM/delay
    -I$PROJECT/SYSTEM/sys
)

rm -rf "$BUILD"
mkdir -p "$BUILD"

SRCS=(
    Core/Src/main.c
    Core/Src/usart.c
    Core/Src/gpio.c
    Core/Src/adc.c
    Core/Src/tim.c
    Core/Src/stm32f1xx_it.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/system_stm32f1xx.c
    MiniBalance/CONTROL/control.c
    MiniBalance/show/show.c
    MiniBalance_HARDWARE/OLED/oled.c
    MiniBalance_HARDWARE/LED/LED.C
    SYSTEM/delay/delay.c
    SYSTEM/sys/sys.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

echo "=== 编译 ${#SRCS[@]} 个文件 (STM32F103C8T6) ==="

OBJS=""
for src in "${SRCS[@]}"; do
    obj="$BUILD/$(basename "${src%.*}").o"
    printf "  编译 %s\n" "${src##*/}"
    arm-none-eabi-gcc $CFLAGS $DEFINES "${INC[@]}" -c "$PROJECT/$src" -o "$obj" 2>&1
    OBJS="$OBJS $obj"
done

echo "  汇编 startup_stm32f103xb.s"
arm-none-eabi-gcc $CPU -c "$PROJECT/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s" -o "$BUILD/startup.o" 2>&1

echo "=== 链接中 ==="
arm-none-eabi-gcc $LDFLAGS $BUILD/startup.o $OBJS -o "$BUILD/output.elf" 2>&1

echo ""
echo "=== 编译成功 ==="
arm-none-eabi-size "$BUILD/output.elf"
