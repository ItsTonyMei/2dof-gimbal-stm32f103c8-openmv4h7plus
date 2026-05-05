# 2-DOF Gimbal — Color Tracking Platform

Two-axis gimbal with **OpenMV4 H7 Plus** color blob tracking + **STM32F103C8T6** servo control.

## Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| MCU | STM32F103C8T6 | 72 MHz Cortex-M3, 64 KB Flash, 20 KB RAM |
| Vision | OpenMV4 H7 Plus | QVGA 320×240, color tracking at 30-60 fps |
| Base servo | 270° (TIM4 CH3, PB8) | PWM range 250-1250 |
| Arm servo | 180° (TIM4 CH4, PB9) | PWM range 300-1200 |
| UART to OpenMV | USART3 (PB11/PB10) | 115200 8N1 |
| Debug UART | USART1 (PA9/PA10) | printf redirect, 115200 8N1 |
| Display | SSD1306 OLED 128×64 | Software SPI (PB3/RST, PA15/DC, PB5/SCL, PB4/SDA) |
| Battery sense | ADC1 IN1 (PA1) | Voltage divider, factor ~11 |

## Wiring

```
OpenMV4 H7 Plus          STM32F103C8T6
  TX (Pin 4)   ────────> PB11 (USART3_RX)
  GND          ────────> GND

CH9102 USB-UART          STM32F103C8T6
  RX           ────────> PA9  (USART1_TX)
  TX           ────────> PA10 (USART1_RX)
```

## Protocol (5-byte, OpenMV → STM32 via USART3)

```
[0xFF] [0xFE] [hasBlob] [tx] [ty]
```

| Byte | Field | Description |
|------|-------|-------------|
| 0 | `0xFF` | Frame sync 1 |
| 1 | `0xFE` | Frame sync 2 |
| 2 | `hasBlob` | `0x01` = target detected, `0x00` = lost |
| 3 | `tx` | Normalized X: 0-255, 128 = image center |
| 4 | `ty` | Normalized Y: 0-255, 128 = image center |

**Coordinate mapping** (QVGA 320×240): `tx = round(cx / 320 × 255)`, `ty = round(cy / 240 × 255)`

**Target lost**: OpenMV sends `hasBlob=0x00` with center coordinates. STM32 holds current position via `OpenMV_Hold_Current_Position()`. If no valid frame for 300 ms, velocity zeroes out.

There is **no checksum** — the 2-byte header provides self-synchronization on frame loss.

## Control Architecture

```
OpenMV (30-60 fps)                  STM32 (TIM2 @ 100 Hz)
  color blob detection                OpenMV_Control() PD controller
  → normalize to 0-255                → normalized error (-0.5 ~ +0.5)
  → send 5-byte frame via UART3       → soft deadzone (5 px inner, 15 px outer)
                                      → P gain (KP=0.05) → velocity output
                                      → velocity clamp (±10)
                                      → Set_Pwm() integrates velocity → position
                                      → TIM4 CCR3/CCR4 → servos
```

### PID Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `YAW_KP` / `PITCH_KP` | 0.05 | Proportional gain on normalized error |
| `YAW_KD` / `PITCH_KD` | 0.0 | Derivative gain (disabled by default) |
| `INNER_DEADZONE` | 5 px | Complete deadband around center |
| `OUTER_DEADZONE` | 15 px | Quadratic easing transition zone |
| `OPENMV_MAX_DELTA` | 10.0 | Max velocity per control cycle |
| `OPENMV_STALE_TIMEOUT_MS` | 300 | Stale frame timeout |

### Servo Limits

| Servo | Min PWM | Max PWM | Angle |
|-------|---------|---------|-------|
| Base (PB8) | 250 | 1250 | 270° |
| Arm (PB9) | 300 | 1200 | 180° |

## Directory Structure

```
├── openmv/main.py          # OpenMV color tracking firmware
├── stm32/
│   ├── Core/                # HAL drivers (main, usart, tim, adc, gpio)
│   ├── Drivers/             # CMSIS + STM32F1xx HAL
│   ├── MiniBalance/CONTROL/ # PD controller + servo output
│   ├── MiniBalance/show/    # OLED display
│   ├── MiniBalance_HARDWARE/
│   │   ├── OLED/            # SSD1306 128×64 driver
│   │   └── LED/             # Status LED driver
│   ├── SYSTEM/              # delay, sys (bit-band, type defs)
│   └── MDK-ARM/             # Keil project + linker script
├── build_gcc.sh             # GCC cross-compilation script
└── README.md
```

## Build & Flash

### STM32 (GCC)

```bash
# Requires: arm-none-eabi-gcc
./build_gcc.sh
# Output: stm32/build_gcc/output.elf
```

### STM32 (Keil MDK)

Open `stm32/MDK-ARM/MiniBalance.uvprojx`, build (F7), download (F8).

### OpenMV

Copy `openmv/main.py` to the OpenMV flash root via OpenMV IDE.

## Configuration

### Color Thresholds (openmv/main.py)

Edit `COLOR_THRESHOLDS` in LAB format: `(L_min, L_max, A_min, A_max, B_min, B_max)`.

```python
# Use OpenMV IDE threshold editor to tune
COLOR_THRESHOLDS = [(0, 100, -128, -15, 0, 127)]
```

### Camera orientation

Set `SENSOR_HMIRROR` and `SENSOR_VFLIP` to match physical mounting.

### PD Gains (stm32/MiniBalance/CONTROL/control.h)

Increase `YAW_KP` / `PITCH_KP` for faster response, enable `YAW_KD` / `PITCH_KD` for damping.

### Servo PWM Limits (stm32/MiniBalance/CONTROL/control.h)

Adjust `SERVO_BASE_MIN/MAX_PWM` and `SERVO_ARM_MIN/MAX_PWM` to match servo specs.

## Debug

Define `DEBUG_PRINTF` to enable serial debugging via USART1 (printf at 115200 baud):

```bash
# In build_gcc.sh, add: -DDEBUG_PRINTF
```

Prints target coordinates, normalized error, and target-lost status every 500 ms.
