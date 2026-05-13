# 二自由度云台 — BlazeFace 人脸追踪

基于 **OpenMV N6**（STM32N6 NPU）单板方案，BlazeFace 人脸检测 + PD 舵机直驱。

## 硬件

| 组件 | 型号 | 说明 |
|------|------|------|
| 视觉+主控 | OpenMV N6 | STM32N6, 1GHz/600GOPS NPU, OpenMV SDK 3.4 |
| 扩展板 | OpenMV 传感器扩展板 | 3 路舵机接口 |
| 底舵机 | P7 (舵机口 1) | Yaw 轴, 180° 500-2500us |
| 摇臂舵机 | P9 (舵机口 3) | Pitch 轴, 180° 限幅 1000-2000us |

## 软件架构

```
摄像头 VGA(640x480) → 320x320 裁剪 → BlazeFace 128 → 最近脸追踪 → PD 控制器 ~70Hz → P7/P9
```

- **320x320 裁剪窗口**：提升模型空间分辨率，中距离（2-3m）识别更强
- **最近脸优先**：防止多人/误检时目标跳变
- **丢失复位**：目标丢失超时后自动回中 + 重置内部状态，允许重捕获
- **多层死区**：位置死区、导数死区、PWM 输出死区三级级联，抑制检测噪声导致的微振
- **线性死区过渡**：`deadzone()` 使用线性缓动（非 t²），消除边界增益尖峰

## 追踪流程

```
采集 → BlazeFace 检测 → get_target():
  ├─ 有检测框 → 已锁定? 选最近脸 : 选最大脸
  │            → 边界越界? 丢弃
  │            → 跳变过大? 累加 track_lost, 超限则复位重捕获
  │            → 通过 → 锁定, track_lost=0
  └─ 无检测框 → track_lost 累加, 超限后复位 last_cx/cy 并进入回中流程

servo_control():
  ├─ 有目标 → PD (死区误差×KP + 原始导数×KD) → 速度积分 → PWM
  ├─ 丢失 <1.5s → 保持位置
  └─ 丢失 >1.5s → 匀速回中
```

## 目录

```
├── openmv/
│   ├── main.py                    # N6 BlazeFace 追踪固件 (当前)
│   └── color-tracking.py          # H7 Plus 兼容固件 (旧硬件用)
├── stm32/                         # STM32F103 固件 (旧架构, 已废弃)
└── README.md
```

## 调参

所有参数集中在 `openmv/main.py` 文件头：

```python
# 舵机方向 (1=正向, -1=反向)
YAW_DIR   = -1
PITCH_DIR = 1

# 视觉
CAMERA_WINDOW_W  = 320   # 窗口越小, 模型空间分辨率越高
CAMERA_WINDOW_H  = 320
BLAZEFACE_THRESHOLD = 0.25  # 置信度阈值 (降低=更远可检)

# PD 控制
KP = 0.8            # 比例增益 (大=跟手)
KD = 0.1            # 微分增益 (大=阻尼)
PD_DEAD_INNER = 13  # 位置死区内径 (px)
PD_DEAD_OUTER = 27  # 位置死区外径 (px)
DERIV_DEAD = 5      # 导数死区 (px)
SERVO_GAIN = 18000  # 像素→舵机速度换算
MAX_STEP_NS = 50000 # 单帧最大步进
PWM_DEAD_NS = 5000  # PWM 输出死区

# 追踪
TRACK_LOST_MAX  = 30   # 连续丢失帧数阈值
TRACK_MAX_JUMP  = 55   # 跳变判定距离 (px)

# 回中
RETURN_HOLD_TIME_MS = 1500  # 丢失后保持时间
RETURN_SPEED_NS_MS  = 500   # 回中速度
```

## 状态 LED

| LED | 含义 |
|-----|------|
| 绿色 | 目标锁定 |
| 蓝色 | 目标丢失 |
| 红色 | 初始化失败 |
