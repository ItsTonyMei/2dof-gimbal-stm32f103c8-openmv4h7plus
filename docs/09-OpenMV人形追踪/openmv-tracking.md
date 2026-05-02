# OpenMV 颜色追踪

## 概述

STM32F103C8T6 云台 + OpenMV4 H7 Plus 颜色追踪模式。STM32 保留三种控制模式：

- Mode 0: PS2 手柄控制
- Mode 1: USART1/USB PC 角度控制
- Mode 2: USART3 OpenMV 视觉追踪

## 接线

| OpenMV4 H7 Plus | STM32F103C8T6 | 说明 |
|---|---|---|
| TX (P4) | PB11 (USART3_RX) | OpenMV 单向发送目标坐标 |
| GND | GND | 必须共地 |
| 5V | 5V | 电流不足时请独立供电并共地 |

串口参数：`115200 8N1`，无流控。

## 部署

1. 将 `openmv/main.py` 放入 OpenMV 盘符根目录
2. 上电后 OpenMV 自动以 30-60 FPS 发送目标坐标
3. STM32 只在 Mode 2（KEY_S 切换）消费 USART3 的有效帧

## 模式切换

按 KEY_S 单击循环切换：Mode 0 → Mode 1 → Mode 2 → Mode 0

OLED 分别显示 `PS2`、`UART`、`OpenMV`。

## 协议（5字节）

STM32 接收完整帧：`[0xFF][0xFE][hasBlob][tx][ty]`

| 字节 | 字段 | 说明 |
|---|---|---|
| 0 | `0xFF` | 帧头1（同步） |
| 1 | `0xFE` | 帧头2（同步） |
| 2 | `hasBlob` | `0x01`=检测到目标，`0x00`=未检测 |
| 3 | `tx` | 目标X坐标，归一化 0-255 |
| 4 | `ty` | 目标Y坐标，归一化 0-255 |

### 坐标说明

QVGA 分辨率 320×240：
- tx = cx / 320 × 255（像素X → 归一化）
- ty = cy / 240 × 255（像素Y → 归一化）
- 图像中心点 (cx=160, cy=120) → (tx=128, ty=128)

| tx/ty 值 | 含义 |
|---|---|
| 0 | 左/上边界 |
| 128 | 画面中心 |
| 255 | 右/下边界 |

STM32 端 `OPENMV_CENTER_X = OPENMV_CENTER_Y = 128`，与 OpenMV 端 `IMAGE_CENTER = 128` 对齐。

### 丢失目标处理

目标丢失时，OpenMV 保持发送上一帧坐标最多 5 帧（LAST_KNOWN_FRAMES=5），之后回传中心坐标 (128, 128)。STM32 端若超过约 200ms 未收到有效帧，Velocity1/2 清零，云台停止动作。

## 颜色阈值配置

`openmv/main.py` 中 `COLOR_THRESHOLDS` 为 LAB 颜色空间阈值，默认为绿色：

```python
COLOR_THRESHOLDS = [
    (0, 100, -128, -15, 0, 127),   # 绿色
]
```

常用阈值参考：

| 颜色 | LAB 阈值 |
|---|---|
| 绿色 | `(30, 50, -20, 20, 20, 70)` |
| 蓝色 | `(20, 40, 10, 30, -60, -20)` |
| 红色 | `(60, 80, 30, 70, 10, 50)` |

## 控制参数（STM32）

| 参数 | 值 | 说明 |
|---|---|---|
| YAW_KP | 0.05 | 偏航比例增益 |
| PITCH_KP | 0.05 | 俯仰比例增益 |
| YAW_KD / PITCH_KD | 0.0 | 微分增益（已禁用） |
| INNER_DEADZONE | 5 | 完全死区 ±5px |
| OUTER_DEADZONE | 15 | 过渡区 5-15px（scale² 缓动） |
| MAX_TARGET_DELTA | 40 | 单帧最大位移限制 |

死区公式（二次缓动）：

```c
scale = ((abs_dx - INNER_DEADZONE) / (OUTER_DEADZONE - INNER_DEADZONE))²
new_error = raw_error × scale
```
