# 二自由度云台 (2-DOF Gimbal) — OpenMV N6 单板 — Copilot 指令

## 架构 (Architecture)

单板方案 (Single-Board): OpenMV N6 BlazeFace 人脸检测 (Face Detection) → PD 控制器 (PD Controller) → PWM 直驱双舵机 (Direct Servo Drive)。

- **OpenMV N6**: STM32N6, 1GHz/600GOPS NPU, OpenMV SDK 3.4, MicroPython 1.28
- **模型 (Model)**: ROM 内置 `blazeface_front_128.tflite` (186KB, 128×128)
- **舵机 (Servo)**: P7 (底 / Yaw), P9 (臂 / Pitch), 50Hz PWM

## 关键文件 (Key Files)

| 文件 (File) | 作用 (Purpose) |
|------|------|
| `openmv/main.py` | BlazeFace 追踪 (Tracking) + PD 控制 (Control) + PWM 驱动 |
| `openmv/color-tracking.py` | H7 Plus 兼容固件 (旧硬件 / Legacy) |

## 控制链路 (Control Pipeline)

```
cam.snapshot() → model.predict() → get_target(最近脸 Nearest Face) → servo_control(PD) → _pwm_write(P7/P9)
    70+ FPS             128×128 NPU       空间一致性防跳 (Spatial Consistency)   像素误差→ns (Pixel Error→ns)   5us 输出死区 (Output Deadzone)
```

## 调参 (Tuning) — openmv/main.py 文件头

- `YAW_DIR / PITCH_DIR`: 舵机方向 (Servo Direction), 1 或 -1
- `KP / KD`: PD 增益 (PD Gain)
- `PD_DEAD_INNER / PD_DEAD_OUTER`: 像素死区 (Pixel Deadzone)
- `SERVO_GAIN / MAX_STEP_NS`: 速度换算 (Speed Scaling)
- `BLAZEFACE_THRESHOLD`: 检测置信度 (Detection Confidence)
- `DRAW_ENABLE`: IDE 调试绘图开关 (Debug Overlay Toggle)

## 调试 (Debug)

```bash
mpremote connect /dev/ttyACM0 run openmv/main.py   # 热加载运行 (Hot Reload)
mpremote connect /dev/ttyACM0 cp openmv/main.py :/flash/main.py  # 固化到 flash (Flash)
```
