# 二自由度云台 — BlazeFace 人脸追踪

基于 **OpenMV N6**（STM32N6 NPU）单板方案，BlazeFace 人脸检测 + PD 舵机直驱。

## 硬件

| 组件 | 型号 | 说明 |
|------|------|------|
| 视觉+主控 | OpenMV N6 | STM32N6, 1GHz/600GOPS NPU, OpenMV SDK 3.4 |
| 扩展板 | OpenMV 传感器扩展板 | 3 路舵机接口 |
| 底舵机 | P7 (舵机口 1) | Yaw 轴 |
| 摇臂舵机 | P9 (舵机口 3) | Pitch 轴 |

> 已不再使用 STM32F103C8T6。N6 单板完成检测+控制+驱动。

## 软件架构

```
摄像头 (VGA→480×480) → BlazeFace (128×128, 186KB ROM内置) → 选最大脸 → PD 控制器 (~70Hz) → PWM 直驱 P7/P9
```

## 目录

```
├── openmv/
│   ├── main.py                          # N6 BlazeFace 追踪固件 (当前)
│   ├── color-tracking.py                # H7 Plus 兼容固件 (旧硬件用)
│   ├── crowdhuman_head_person_int8.*    # YOLO 模型文件 (已废弃, 仅存档)
│   └── main_yolo_backup.py             # YOLO 固件备份
├── stm32/                              # STM32F103 固件 (旧架构, 已废弃)
├── docs/
│   ├── crowdhuman-int8-export-guide.md  # YOLO int8 量化指南 (存档)
│   └── 05-通讯协议/protocol.md          # UART 协议 (旧架构, 已废弃)
└── README.md
```

## 调参

所有参数集中在 `openmv/main.py` 文件头：

```python
# 舵机方向 (1=正向, -1=反向)
YAW_DIR   = -1
PITCH_DIR = 1

# PD 控制
KP = 1.5            # 比例增益 (大=跟手)
KD = 0.05           # 微分增益 (大=阻尼强)
PD_DEAD_INNER = 18  # 死区 (px)

# 速度
SERVO_GAIN = 12000  # 像素→速度换算
MAX_STEP_NS = 50000 # 单帧最大步进

# 视觉
BLAZEFACE_THRESHOLD = 0.35  # 检测门槛
DRAW_ENABLE = False         # IDE 调试时开
```

## 状态 LED

| LED | 含义 |
|-----|------|
| 绿色 | 目标锁定 |
| 蓝色 | 目标丢失 |
| 红色闪烁 | 初始化失败 |
