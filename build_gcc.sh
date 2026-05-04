#!/bin/bash
# GCC build for 2DOF-Gimbal STM32F103C8T6
# Verifies compilation only (no flash target)

set -e
PROJECT="/home/tonymei/桌面/2dof-gimbal-stm32f103c8-openmv4h7plus/stm32"
BUILD="$PROJECT/build_gcc"
CPU="-mcpu=cortex-m3 -mthumb"
FPU="-msoft-float"
SPECS="--specs=nosys.specs --specs=nano.specs"
DEFINES="-DUSE_HAL_DRIVER -DSTM32F103xB"
CFLAGS="$CPU $FPU -Wall -Wextra -Os -ffunction-sections -fdata-sections -fno-common"
LDFLAGS="$CPU $FPU $SPECS -T$PROJECT/MDK-ARM/STM32F103C8TX_FLASH.ld -Wl,--gc-sections -Wl,-Map=$BUILD/output.map"
INC=""
for d in \
    Core/Inc \
    Drivers/CMSIS/Include \
    Drivers/CMSIS/Device/ST/STM32F1xx/Include \
    Drivers/STM32F1xx_HAL_Driver/Inc \
    MiniBalance/CONTROL \
    MiniBalance/show \
    MiniBalance_HARDWARE/MOTOR \
    MiniBalance_HARDWARE/OLED \
    MiniBalance_HARDWARE/KEY \
    MiniBalance_HARDWARE/ADC \
    MiniBalance_HARDWARE/LED \
    MiniBalance_HARDWARE/BEEP \
    MiniBalance_HARDWARE/IIC \
    MiniBalance_HARDWARE/PS2 \
    MiniBalance_HARDWARE/USART2 \
    MiniBalance_HARDWARE/USART3 \
    MiniBalance_HARDWARE/DataScope_DP \
    MiniBalance_HARDWARE/STMFLASH \
    MiniBalance_HARDWARE/TIMER \
    MiniBalance_HARDWARE/RING_BUFFER \
    SYSTEM/delay \
    SYSTEM/sys \
    SYSTEM/usart \
; do
    INC="$INC -I$PROJECT/$d"
done

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
    MiniBalance_HARDWARE/KEY/key.c
    MiniBalance_HARDWARE/LED/LED.C
    MiniBalance_HARDWARE/PS2/pstwo.c
    MiniBalance_HARDWARE/DataScope_DP/DataScope_DP.C
    # stmflash.c excluded: references MiniBalance variables not present in gimbal project
    MiniBalance_HARDWARE/RING_BUFFER/ring_buffer.c
    SYSTEM/delay/delay.c
    SYSTEM/sys/sys.c
    # SPL-legacy drivers excluded (replaced by HAL equivalents in Core/Src/):
    #   motor.c→tim.c, adc.c→adc.c(HAL), timer.c→tim.c, IOI2C.c→HAL,
    #   usart2.c→HAL, BEEP.C→tim.c(HAL)
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

echo "=== Compiling $((${#SRCS[@]} + 1)) files for STM32F103C8T6 ==="

OBJS=""
for src in "${SRCS[@]}"; do
    obj="$BUILD/$(basename "${src%.*}").o"
    echo "  CC  ${src##*/}"
    arm-none-eabi-gcc $CFLAGS $DEFINES $INC -c "$PROJECT/$src" -o "$obj" 2>&1
    OBJS="$OBJS $obj"
done

# Startup assembly (GCC syntax from CMSIS)
echo "  AS  startup_stm32f103xb.s"
arm-none-eabi-gcc $CPU -c "$PROJECT/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s" -o "$BUILD/startup.o" 2>&1

echo "=== Linking ==="
arm-none-eabi-gcc $LDFLAGS $BUILD/startup.o $OBJS -o "$BUILD/output.elf" 2>&1

echo ""
echo "=== BUILD SUCCESS ==="
arm-none-eabi-size "$BUILD/output.elf"
