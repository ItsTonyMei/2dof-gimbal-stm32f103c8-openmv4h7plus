# 2-DOF Gimbal (STM32F103C8T6 + OpenMV4 H7 Plus) — Copilot Instructions

## Architecture

Single control mode: OpenMV color blob tracking → UART3 → STM32 PD controller → dual servo PWM.

- **STM32F103C8T6**: Cortex-M3, 72 MHz, TIM2 ISR at 100 Hz (10 ms)
- **OpenMV4 H7 Plus**: LAB color threshold tracking at 30-60 fps
- **Servos**: 270° base (PB8/TIM4 CH3), 180° arm (PB9/TIM4 CH4)

## Build

```bash
./build_gcc.sh
# Output: stm32/build_gcc/output.elf
# arm-none-eabi-gcc, -mcpu=cortex-m3 -mthumb -msoft-float
```

## Key Files

| File | Role |
|------|------|
| `stm32/Core/Src/main.c` | Init, main loop (OLED, battery, debug printf) |
| `stm32/Core/Src/usart.c` | USART3 OpenMV protocol parser + USART1 printf |
| `stm32/MiniBalance/CONTROL/control.c` | `OpenMV_Control()` PD, `Set_Pwm()`, servo limits |
| `stm32/MiniBalance/CONTROL/control.h` | PD gains, servo PWM limits, state flags |
| `stm32/MiniBalance/show/show.c` | OLED display (OpenMV mode layout) |
| `openmv/main.py` | Color blob detection + 5-byte UART tx |

## OpenMV Protocol (5 bytes, UART3 115200 8N1)

```
[0xFF][0xFE][hasBlob][tx][ty]
```
- `hasBlob`: 0x01 = detected, 0x00 = lost
- `tx/ty`: normalized 0-255, 128 = center (QVGA 320×240)

## Control Flow

```
TIM2 ISR @ 100 Hz:
  OpenMV_Control() → soft deadzone → PD (KP=0.05) → velocity clamp (±10) → Set_Pwm()

Main loop:
  OLED refresh → battery ADC → delay_ms(5)
```

## PD Parameters (control.h)

- `YAW_KP = PITCH_KP = 0.05`, `YAW_KD = PITCH_KD = 0.0`
- `INNER_DEADZONE = 5`, `OUTER_DEADZONE = 15` (quadratic easing)
- `OPENMV_MAX_DELTA = 10.0`, `OPENMV_STALE_TIMEOUT_MS = 300`

## Servo Limits (control.h)

- Base: `SERVO_BASE_MIN/MAX_PWM = 250/1250` (270°)
- Arm: `SERVO_ARM_MIN/MAX_PWM = 300/1200` (180°)

## Common Tasks

- **Tune gains**: Edit `YAW_KP`/`YAW_KD` etc. in `control.h`
- **Change color**: Edit `COLOR_THRESHOLDS` in `openmv/main.py`
- **Adjust deadzone**: Edit `INNER_DEADZONE`/`OUTER_DEADZONE` in `control.c`
- **Servo limits**: Edit `SERVO_*_MIN/MAX_PWM` in `control.h`
- **Debug output**: Add `-DDEBUG_PRINTF` to build flags
