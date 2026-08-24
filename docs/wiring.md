# d36ax1 接线表 (Wiring)

Wheeltec 双路步进电机驱动模块(带稳压版 d36ax1)与 OpenMV N6 的连接。
信号为脉冲/方向 (STEP/DIR) 接口,N6 直接生成步进脉冲,**无外部 MCU**。

> ⚠️ 下表 N6 引脚为**待确认占位 (TBD)**,请按实际接线填写。

## 信号定义 (Signal Table)

| 模块信号 | 功能 (Function) | 通道 | N6 引脚 (待确认) | 备注 (Notes) |
|---|---|---|---|---|
| `ST1` | 步进脉冲 (Step Pulse) | Yaw | | 建议 PWM 引脚,高电平有效 |
| `DIR1` | 方向 (Direction) | Yaw | | GPIO,电平决定正反转 |
| `EN1` | 使能 (Enable) | Yaw | | 有效电平待确认(低有效常见) |
| `ST2` | 步进脉冲 (Step Pulse) | Pitch | | 建议 PWM 引脚 |
| `DIR2` | 方向 (Direction) | Pitch | | GPIO |
| `EN2` | 使能 (Enable) | Pitch | | |
| `ADC` | 模拟反馈 (Analog Feedback) | 共用 | | 位置反馈?用途待确认 |
| `5V` | 电源 (Power) | — | — | 稳压版 5V 输出/输入 |
| `GND` | 地 (Ground) | — | — | 共地,必须与 N6 共地 |

## 接线注意事项 (Notes)

1. **共地 (Common Ground)**:N6 与驱动模块必须共 GND,否则脉冲信号无参考电平。
2. **脉冲信号电平 (Signal Level)**:确认 N6 GPIO (3.3V) 与驱动模块输入电平兼容;
   如需要 5V 电平,加电平转换或确认模块输入带施密特触发。
3. **ADC 反馈 (ADC Feedback)**:确认模拟输入量程(0~3.3V 或 0~5V),超量程需分压。
4. **使能逻辑 (Enable Logic)**:EN 悬空/高/低分别是什么状态,建议默认使能、软件可关断。

## 调试记录 (Debug Log)

_待补充:实际接线后的验证结果、脉冲频率/细分设置。_
