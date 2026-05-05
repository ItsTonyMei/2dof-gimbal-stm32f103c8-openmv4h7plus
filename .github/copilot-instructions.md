# 二自由度云台 (STM32F103C8T6 + OpenMV4 H7 Plus) — Copilot 指令

## 架构

单一控制模式: OpenMV 颜色色块追踪 → UART3 → STM32 PD 控制器 → 双舵机 PWM。

- **STM32F103C8T6**: Cortex-M3, 72 MHz, TIM2 中断 100 Hz (10 ms)
- **OpenMV4 H7 Plus**: LAB 颜色阈值追踪, 30-60 fps
- **舵机**: 270° 底舵机 (PB8/TIM4 CH3), 180° 摇臂舵机 (PB9/TIM4 CH4)

## 构建

```bash
./build_gcc.sh
# 输出: stm32/build_gcc/output.elf
# arm-none-eabi-gcc, -mcpu=cortex-m3 -mthumb -msoft-float
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `stm32/Core/Src/main.c` | 初始化, 主循环 (OLED, 电池, 调试 printf) |
| `stm32/Core/Src/usart.c` | USART3 OpenMV 协议解析 + USART1 printf |
| `stm32/MiniBalance/CONTROL/control.c` | `OpenMV_Control()` PD 控制器, `Set_Pwm()`, 舵机限幅 |
| `stm32/MiniBalance/CONTROL/control.h` | PD 增益, 舵机 PWM 限幅, 状态标志 |
| `stm32/MiniBalance/show/show.c` | OLED 显示 (OpenMV 模式布局) |
| `openmv/main.py` | 颜色色块检测 + 5 字节 UART 发送 |

## OpenMV 协议 (5 字节, UART3 115200 8N1)

```
[0xFF][0xFE][hasBlob][tx][ty]
```
- `hasBlob`: 0x01=检测到, 0x00=丢失
- `tx/ty`: 归一化 0-255, 128=中心 (QVGA 320×240)

## 控制流程

```
TIM2 中断 @ 100 Hz:
  OpenMV_Control() → 软死区 → PD (KP=0.05) → 速度限幅 (±10) → Set_Pwm()

主循环:
  OLED 刷新 → 电池 ADC → delay_ms(5)
```

## PD 参数 (control.h)

- `YAW_KP = PITCH_KP = 0.05`, `YAW_KD = PITCH_KD = 0.0`
- `INNER_DEADZONE = 5`, `OUTER_DEADZONE = 15` (二次缓动)
- `OPENMV_MAX_DELTA = 10.0`, `OPENMV_STALE_TIMEOUT_MS = 300`

## 舵机限幅 (control.h)

- 底舵机: `SERVO_BASE_MIN/MAX_PWM = 250/1250` (270°)
- 摇臂舵机: `SERVO_ARM_MIN/MAX_PWM = 300/1200` (180°)

## 常见任务

- **调节增益**: 编辑 `control.h` 中的 `YAW_KP`/`YAW_KD` 等
- **更换颜色**: 编辑 `openmv/main.py` 中的 `COLOR_THRESHOLDS`
- **调整死区**: 编辑 `control.c` 中的 `INNER_DEADZONE`/`OUTER_DEADZONE`
- **舵机限幅**: 编辑 `control.h` 中的 `SERVO_*_MIN/MAX_PWM`
- **调试输出**: 在构建参数中添加 `-DDEBUG_PRINTF`
