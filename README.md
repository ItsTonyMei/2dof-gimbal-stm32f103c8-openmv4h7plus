# 二自由度云台 — 颜色追踪平台

基于 **OpenMV4 H7 Plus** 颜色色块追踪 + **STM32F103C8T6** 舵机控制的二轴云台系统。

## 硬件配置

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | STM32F103C8T6 | 72 MHz Cortex-M3, 64 KB Flash, 20 KB RAM |
| 视觉 | OpenMV4 H7 Plus | QVGA 320×240, 颜色追踪 30-60 fps |
| 底舵机 | 270° (TIM4 CH3, PB8) | PWM 范围 250-1250 |
| 摇臂舵机 | 180° (TIM4 CH4, PB9) | PWM 范围 300-1200 |
| 视觉通信 | USART3 (PB11/PB10) | 115200 8N1 |
| 调试串口 | USART1 (PA9/PA10) | printf 重定向, 115200 8N1 |
| 显示屏 | SSD1306 OLED 128×64 | 软件 SPI (PB3/RST, PA15/DC, PB5/SCL, PB4/SDA) |
| 电池检测 | ADC1 IN1 (PA1) | 电阻分压, 系数约 11 |

## 接线

```
OpenMV4 H7 Plus          STM32F103C8T6
  TX (Pin 4)   ────────> PB11 (USART3_RX)
  GND          ────────> GND

CH9102 USB-UART          STM32F103C8T6
  RX           ────────> PA9  (USART1_TX)
  TX           ────────> PA10 (USART1_RX)
```

## 通信协议 (5字节, OpenMV → STM32, USART3)

```
[0xFF] [0xFE] [hasBlob] [tx] [ty]
```

| 字节 | 字段 | 说明 |
|------|------|------|
| 0 | `0xFF` | 帧头 1（同步） |
| 1 | `0xFE` | 帧头 2（同步） |
| 2 | `hasBlob` | `0x01`=检测到目标, `0x00`=目标丢失 |
| 3 | `tx` | 归一化 X 坐标: 0-255, 128=图像中心 |
| 4 | `ty` | 归一化 Y 坐标: 0-255, 128=图像中心 |

**坐标映射** (QVGA 320×240): `tx = round(cx / 320 × 255)`, `ty = round(cy / 240 × 255)`

**目标丢失处理**: OpenMV 发送 `hasBlob=0x00` 和中心坐标。STM32 通过 `OpenMV_Hold_Current_Position()` 保持当前位置。若超过 300 ms 未收到有效帧，速度归零。

**无校验和** — 2 字节帧头在数据丢失时自动重新同步。

## 控制架构

```
OpenMV (30-60 fps)                  STM32 (TIM2 @ 100 Hz)
  颜色色块检测                         OpenMV_Control() PD 控制器
  → 归一化到 0-255                    → 归一化误差 (-0.5 ~ +0.5)
  → 通过 UART3 发送 5 字节帧           → 软死区 (5 px 内死区, 15 px 外过渡)
                                      → P 增益 (KP=0.05) → 速度输出
                                      → 速度限幅 (±10)
                                      → Set_Pwm() 积分速度 → 位置
                                      → TIM4 CCR3/CCR4 → 舵机
```

### PID 参数

| 参数 | 值 | 说明 |
|------|------|------|
| `YAW_KP` / `PITCH_KP` | 0.05 | 归一化误差的比例增益 |
| `YAW_KD` / `PITCH_KD` | 0.0 | 微分增益（默认禁用） |
| `INNER_DEADZONE` | 5 px | 中心区域完全死区 |
| `OUTER_DEADZONE` | 15 px | 二次缓动过渡区 |
| `OPENMV_MAX_DELTA` | 10.0 | 每控制周期最大速度 |
| `OPENMV_STALE_TIMEOUT_MS` | 300 | 数据超时时间(ms) |

### 舵机限幅

| 舵机 | 最小 PWM | 最大 PWM | 角度 |
|------|----------|----------|------|
| 底舵机 (PB8) | 250 | 1250 | 270° |
| 摇臂舵机 (PB9) | 300 | 1200 | 180° |

## 目录结构

```
├── openmv/main.py          # OpenMV 颜色追踪固件
├── stm32/
│   ├── Core/                # HAL 驱动 (main, usart, tim, adc, gpio)
│   ├── Drivers/             # CMSIS + STM32F1xx HAL
│   ├── MiniBalance/CONTROL/ # PD 控制器 + 舵机输出
│   ├── MiniBalance/show/    # OLED 显示
│   ├── MiniBalance_HARDWARE/
│   │   ├── OLED/            # SSD1306 128×64 驱动
│   │   └── LED/             # 状态 LED 驱动
│   ├── SYSTEM/              # 延时, sys (位带, 类型定义)
│   └── MDK-ARM/             # Keil 工程 + 链接脚本
├── build_gcc.sh             # GCC 交叉编译脚本
└── README.md
```

## 编译与烧录

### STM32 (GCC)

```bash
# 需要: arm-none-eabi-gcc
./build_gcc.sh
# 输出: stm32/build_gcc/output.elf
```

### STM32 (Keil MDK)

打开 `stm32/MDK-ARM/MiniBalance.uvprojx`，编译 (F7)，下载 (F8)。

### OpenMV

通过 OpenMV IDE 将 `openmv/main.py` 复制到 OpenMV 闪存根目录。

## 配置指南

### 颜色阈值 (openmv/main.py)

编辑 `COLOR_THRESHOLDS`，LAB 格式: `(L_min, L_max, A_min, A_max, B_min, B_max)`。

```python
# 使用 OpenMV IDE 阈值编辑器进行调试
COLOR_THRESHOLDS = [(0, 100, -128, -15, 0, 127)]
```

### 摄像头方向

根据实际安装方向设置 `SENSOR_HMIRROR` 和 `SENSOR_VFLIP`。

### PD 增益 (stm32/MiniBalance/CONTROL/control.h)

增大 `YAW_KP` / `PITCH_KP` 可加快响应，启用 `YAW_KD` / `PITCH_KD` 可增加阻尼。

### 舵机 PWM 限幅 (stm32/MiniBalance/CONTROL/control.h)

根据舵机规格调整 `SERVO_BASE_MIN/MAX_PWM` 和 `SERVO_ARM_MIN/MAX_PWM`。

## 调试

定义 `DEBUG_PRINTF` 以通过 USART1 启用串口调试 (printf, 115200 波特)：

```bash
# 在 build_gcc.sh 中添加: -DDEBUG_PRINTF
```

每 500 ms 打印一次目标坐标、归一化误差和目标丢失状态。
