# 通信协议 (Communication Protocol) — 已废弃 (Deprecated)

> **当前项目已不再使用此协议。** N6 单板方案中，检测 (Detection) 和控制 (Control) 都在同一块 OpenMV N6 上完成，无外部通信。
>
> 此协议适用于旧架构 (OpenMV4 H7 Plus → UART → STM32F103C8T6)，保留作为历史参考。

---

## 物理层 (Physical Layer, 旧)

| 参数 (Parameter) | 值 (Value) |
| ------ | ------ |
| 串口 (UART) | USART3 |
| 引脚 (Pin) | OpenMV TX(P4) → STM32 PB11(RX), GND ↔ GND |
| 波特率 (Baud Rate) | 115200 8N1 |

## 帧格式 (Frame Format, 旧, 5 字节 / 5-Byte)

```text
[0xFF] [0xFE] [hasBlob] [tx] [ty]
```

- `hasBlob`: 0x01=检测到目标 (Detected), 0x00=目标丢失 (Lost)
- `tx/ty`: 归一化坐标 (Normalized Coordinates) 0-255, 128=中心 (Center, QVGA 320×240)
