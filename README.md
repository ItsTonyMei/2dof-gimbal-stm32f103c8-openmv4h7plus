# 二自由度云台 (2-DOF Gimbal) — BlazeFace 人脸追踪 (Face Tracking)

基于 **OpenMV N6**（STM32N6 NPU）单板方案，BlazeFace 人脸检测 (Face Detection) + PD 舵机直驱 (Direct Servo Drive)。

## 硬件 (Hardware)

| 组件 (Component) | 型号 (Model) | 说明 |
| ------ | ------ | ------ |
| 视觉+主控 (Vision+MCU) | OpenMV N6 | STM32N6, 1GHz/600GOPS NPU, OpenMV SDK 3.4 |
| 扩展板 (Expansion Board) | OpenMV 传感器扩展板 | 3 路舵机接口 (Servo Ports) |
| 底舵机 (Base Servo) | P7 (舵机口 1) | Yaw 轴, 180° 500-2500us |
| 摇臂舵机 (Arm Servo) | P9 (舵机口 3) | Pitch 轴, 180° 限幅 1000-2000us |

## 软件架构 (Software Architecture)

```text
摄像头 VGA(640x480) → 320x320 裁剪 (Crop) → BlazeFace 128 → 最近脸追踪 (Nearest Face) → PD 控制器 ~70Hz → P7/P9
```

- **320x320 裁剪窗口 (Crop Window)**：提升模型空间分辨率 (Spatial Resolution)，中距离（2-3m）识别更强
- **最近脸优先 (Nearest Face First)**：防止多人/误检时目标跳变 (Target Jump)
- **丢失复位 (Lost Reset)**：目标丢失超时后自动回中 (Return-to-Center) + 重置内部状态，允许重捕获 (Reacquire)
- **多层死区 (Multi-Layer Deadzone)**：位置死区 (Position Deadzone)、导数死区 (Derivative Deadzone)、PWM 输出死区 (Output Deadzone) 三级级联，抑制检测噪声 (Detection Noise) 导致的微振 (Micro-Jitter)
- **线性死区过渡 (Linear Deadzone)**：`deadzone()` 使用线性缓动 (Linear Easing)（非 t²），消除边界增益尖峰 (Gain Spike)

## 追踪流程 (Tracking Flow)

```text
采集 (Capture) → BlazeFace 检测 (Detection) → get_target():
  ├─ 有检测框 (Has Detection) → 已锁定 (Locked)? 选最近脸 (Nearest Face) : 选最大脸 (Largest Face)
  │            → 边界越界 (Out of Bounds)? 丢弃
  │            → 跳变过大 (Jump Too Large)? 累加 track_lost, 超限则复位重捕获 (Reset & Reacquire)
  │            → 通过 → 锁定 (Locked), track_lost=0
  └─ 无检测框 (No Detection) → track_lost 累加, 超限后复位 last_cx/cy 并进入回中流程 (Return-to-Center)

servo_control():
  ├─ 有目标 (Has Target) → PD (死区误差×KP + 原始导数×KD) → 速度积分 (Velocity Integration) → PWM
  ├─ 丢失 <1.5s → 保持位置 (Hold Position)
  └─ 丢失 >1.5s → 匀速回中 (Constant-Speed Return)
```

## 目录 (Directory)

```text
├── openmv/
│   ├── main.py                    # N6 BlazeFace 追踪固件 (Firmware, 当前)
│   └── color-tracking.py          # H7 Plus 兼容固件 (旧硬件用)
├── stm32/                         # STM32F103 固件 (旧架构, 已废弃 / Deprecated)
└── README.md
```

## 调参 (Tuning)

所有参数集中在 `openmv/main.py` 文件头：

```python
# 舵机方向 (Servo Direction) — 1=正向 (Normal), -1=反向 (Reversed)
YAW_DIR   = -1
PITCH_DIR = 1

# 视觉 (Vision)
CAMERA_WINDOW_W  = 320   # 窗口越小, 模型空间分辨率越高
CAMERA_WINDOW_H  = 320
BLAZEFACE_THRESHOLD = 0.5  # 置信度阈值 (Confidence Threshold) — 降低=更远可检

# PD 控制 (PD Controller)
KP = 0.2            # 比例增益 (Proportional Gain) — 大=跟手
KD = 0.1            # 微分增益 (Derivative Gain) — 大=阻尼 (Damping)
PD_DEAD_INNER = 13  # 位置死区内径 (Position Deadzone Inner, px)
PD_DEAD_OUTER = 27  # 位置死区外径 (Position Deadzone Outer, px)
DERIV_DEAD = 5      # 导数死区 (Derivative Deadzone, px)
SERVO_GAIN = 18000  # 像素→舵机速度换算 (Error-to-Speed Gain)
MAX_STEP_NS = 50000 # 单帧最大步进 (Max Step per Tick, ns)
PWM_DEAD_NS = 5000  # PWM 输出死区 (Output Deadzone, ns)

# 追踪 (Tracking)
TRACK_LOST_MAX  = 30   # 连续丢失帧数阈值 (Lost Frame Threshold)
TRACK_MAX_JUMP  = 55   # 跳变判定距离 (Jump Distance Threshold, px)

# 回中 (Return-to-Center)
RETURN_HOLD_TIME_MS = 1500  # 丢失后保持时间 (Hold Time, ms)
RETURN_SPEED_NS_MS  = 500   # 回中速度 (Return Speed, ns/ms)
```

## 状态 LED (Status LED)

| LED | 含义 |
| ----- | ------ |
| 绿灯 (Green) | 目标锁定 (Locked) |
| 蓝灯 (Blue) | 目标丢失 (Lost) |
| 红灯 (Red) | 初始化失败 (Init Failed) |
