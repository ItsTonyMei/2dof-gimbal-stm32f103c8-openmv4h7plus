# 二自由度云台 (OpenMV N6 单板) — Copilot 指令

## 架构

单板方案: OpenMV N6 BlazeFace 人脸检测 → PD 控制器 → PWM 直驱双舵机。

- **OpenMV N6**: STM32N6, 1GHz/600GOPS NPU, OpenMV SDK 3.4, MicroPython 1.28
- **模型**: ROM 内置 `blazeface_front_128.tflite` (186KB, 128×128)
- **舵机**: P7 (底/Yaw), P9 (臂/Pitch), 50Hz PWM

## 关键文件

| 文件 | 作用 |
|------|------|
| `openmv/main.py` | BlazeFace 追踪 + PD 控制 + PWM 驱动 |
| `openmv/color-tracking.py` | H7 Plus 兼容固件 (旧硬件) |

## 控制链路

```
cam.snapshot() → model.predict() → get_target(最大脸) → servo_control(PD) → _pwm_write(P7/P9)
    70+ FPS             128×128 NPU       空间一致性防跳         像素误差→ns        5us 输出死区
```

## 调参 (openmv/main.py 文件头)

- `YAW_DIR / PITCH_DIR`: 舵机方向, 1 或 -1
- `KP / KD`: PD 增益
- `PD_DEAD_INNER / PD_DEAD_OUTER`: 像素死区
- `SERVO_GAIN / MAX_STEP_NS`: 速度换算
- `BLAZEFACE_THRESHOLD`: 检测置信度
- `DRAW_ENABLE`: IDE 调试绘图开关

## 调试

```bash
mpremote connect /dev/ttyACM0 run openmv/main.py   # 热加载运行
mpremote connect /dev/ttyACM0 cp openmv/main.py :/flash/main.py  # 固化到 flash
```
