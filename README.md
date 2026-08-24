# 2-DOF Gimbal — OpenMV N6 + Wheeltec Stepper 双轴云台

基于 OpenMV N6 的单板方案:BlazeFace 人脸检测 + PD 控制,直接驱动
Wheeltec d36ax1 双路步进电机驱动模块(带稳压),控制 2 轴步进云台。

> ⚠️ **重建中 (Rebuilding)**:硬件已从「STM32 + 舵机」改为「N6 直驱步进」,
> 固件控制层尚未重写,当前 `openmv/main.py` 仍是舵机 PWM 版本。

## 架构 (Architecture)

```
┌──────────────────────┐    STEP1/DIR1/EN1 (Yaw)     ┌────────────────────────┐
│  OpenMV N6 (单板)     │ ───────────────────────────▶│  Wheeltec d36ax1      │
│  BlazeFace 检测       │    STEP2/DIR2/EN2 (Pitch)   │  双路步进驱动 (带稳压)  │
│  PD 控制器            │ ───────────────────────────▶│                        │
│  步进脉冲生成 (待重写) │    ADC (位置反馈)            └───────────┬────────────┘
│                      │ ◀───────────────────────────             │
└──────────────────────┘   5V / GND                               ▼
                                                          2 轴步进云台电机
```

- **视觉 (Vision)**:OpenMV N6,Google MediaPipe BlazeFace (128×128, ROM 内置)
- **控制 (Control)**:PD 控制器运行在 N6 上,输出两路步进脉冲 (STEP/DIR/EN)
- **驱动 (Driver)**:Wheeltec d36ax1 双路步进驱动模块(带稳压),无外部 MCU
- **通信 (Link)**:STEP/DIR/EN 脉冲信号 + ADC 反馈,见 [docs/wiring.md](docs/wiring.md)

## 目录结构 (Layout)

```
├── openmv/          # N6 固件 (MicroPython)
│   └── main.py      # BlazeFace + PD + 舵机 PWM (待重写为步进控制)
├── docs/
│   └── wiring.md    # d36ax1 接线表 (待确认引脚)
├── README.md
└── CLAUDE.md        # 项目说明 (给 Claude 的操作手册)
```

## 烧录 (Flashing)

- OpenMV N6:MicroPython,通过 OpenMV IDE 将 `openmv/main.py` 复制到相机闪存
- 烧录工具:OpenMV IDE / dfu 工具(原 `tools/FlyMcu` 已随 STM32 架构移除)

## 状态 (Status)

| 模块 | 状态 |
|---|---|
| 视觉检测 (BlazeFace) | ✅ 可用 |
| PD 控制器 | ✅ 可用(舵机版参数) |
| 步进脉冲输出 | ⏳ 待重写 |
| ADC 位置反馈 | ⏳ 待实现 |
| 接线文档 | 📝 骨架,引脚待确认 |
