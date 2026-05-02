# 2DOF-Gimbal (STM32F103C8T6 + OpenMV4 H7 Plus)

二自由度颜色追踪云台固件 + 文档。

## 硬件

- **MCU**: STM32F103C8T6 (ARM Cortex-M3)
- **视觉**: OpenMV4 H7 Plus
- **舵机**: 270° 底舵 + 180° 摇臂（可配置）
- **控制方式**: PS2 手柄 / UART / OpenMV 颜色追踪

## 目录结构

```
├── stm32/                  # STM32 Keil MDK 工程
│   ├── MDK-ARM/            # Keil 工程文件 (.uvprojx)
│   ├── Core/                # HAL 驱动源码
│   ├── Drivers/             # CMSIS + STM32F1xx_HAL_Driver
│   ├── MiniBalance/         # 业务逻辑 (CONTROL, show)
│   └── MiniBalance_HARDWARE/ # 外设驱动 (MOTOR, OLED, KEY, USART...)
├── openmv/
│   └── main.py             # OpenMV4 H7 Plus 颜色追踪固件
├── docs/                   # 项目文档
│   ├── 01-硬件与接线/
│   ├── 02-开发环境/
│   ├── 03-快速开始/
│   ├── 04-工程源码解析/
│   ├── 05-通讯协议/
│   ├── 06-ROS与Python例程/
│   ├── 07-开发笔记/
│   ├── 08-C06B-360舵机驱动例程/
│   └── 09-OpenMV人形追踪/
└── README.md
```

## OpenMV 协议（5字节）

```
[0xFF][0xFE][hasBlob][tx][ty]
```

| 字段 | 说明 |
|---|---|
| `hasBlob` | `0x01`=检测到目标，`0x00`=未检测 |
| `tx/ty` | 归一化坐标 0-255，128=画面中心 |
| 波特率 | 115200 8N1 |

## 快速开始

### STM32 固件

1. 用 Keil MDK 打开 `stm32/MDK-ARM/MiniBalance.uvprojx`
2. 编译（F7）
3. 下载（F8）
4. 按 KEY_S 切换到 Mode 2（OpenMV 模式）

### OpenMV 固件

1. 将 `openmv/main.py` 放入 OpenMV 盘符根目录
2. 重启 OpenMV，自动运行颜色追踪

## 控制参数（STM32）

| 参数 | 值 |
|---|---|
| YAW_KP / PITCH_KP | 0.05 |
| INNER_DEADZONE | ±5px |
| OUTER_DEADZONE | 5-15px（scale² 缓动） |

详见 [docs/09-OpenMV人形追踪/openmv-tracking.md](docs/09-OpenMV人形追踪/openmv-tracking.md)。

## 编译依赖

- **Keil MDK** 5.41+（需自行安装 ARM Compiler 6 和 STM32F1xx Device Pack）
- **OpenMV IDE** 8.0+（用于 OpenMV 固件烧录）
