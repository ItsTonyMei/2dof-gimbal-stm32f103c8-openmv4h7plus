# 软死区调参日志

**日期**: 2026-05-02
**项目**: 2级舵机云台 - OpenMV 视觉追踪稳定性优化
**当前状态**: ✓ 测试效果不错，待后续优化

---

## 问题背景

目标：让 OpenMV 视觉跟踪云台稳定追踪目标，消除死区边缘的极限环振荡（目标不动但云台在死区边界反复修正）。

---

## 调参轨迹

### Phase 1: 架构重构（解决根因）

| 提交 | 内容 | 说明 |
|------|------|------|
| `6f0db3c` | 消除双层积分抖动 | OpenMV_Control 不再修改 Target1/2，直接写 Velocity1/2，设 OpenMV_Armed=1 跳过 Position_PID |
| `72a9abe` | 修复 Velocity1/2 类型冲突 | sys.h 是 float，control.h 误声明为 int |
| `9d136bb` | 反转 velocity 符号 | Velocity1 = -yaw_pwm, Velocity2 = -pitch_pwm，从"躲目标"变为"追目标" |
| `b4c7c9b` | 降低 KP/KD | KP 0.1→0.08, KD 1.0→1.0, 死区 ±20→±40, alpha 0.2→0.1 |
| `f94be52` | 纯 P 控制禁用 D 项 | KD=0, KP=0.05, 死区 ±50px |

**关键发现**: D 项在高速帧率下（30-60fps，dt≈33ms）被放大 3-6 倍（计算用固定 dt=0.1s），导致高频抖动。

---

### Phase 2: 软死区实现（核心优化）

| 提交 | 内容 | 参数 | 效果 |
|------|------|------|------|
| `16d0486` | 硬死区→二次缓动软死区 | ±30/70 | 极限环基本消除 |
| `34ff0bd` | 缩小死区 | ±25/60 | 响应更灵敏 |
| `6e432d6` | 扩大过渡区比例 | ±15/55 | 边界更平滑 |
| `7576a7b` | 恢复 3:4 比例缩小范围 | ±10/24 | 比例恢复，更紧凑 |
| `95533f7` | 进一步缩小 | **±5/15** | 当前最终参数 |

**最终参数（commit `95533f7`）**:
```c
#define INNER_DEADZONE  5     // ±5px 完全死区
#define OUTER_DEADZONE  15    // 5-15px 二次缓动过渡; 15px+ 全速响应
```

**软死区公式**:
```c
scale = ((abs_dx - INNER_DEADZONE) / (OUTER_DEADZONE - INNER_DEADZONE))²
new_error = raw_error * scale   // 二次缓动，边界处修正量递增更缓
```

---

## 当前参数总览

### control.h
```c
#define YAW_KP     0.05f
#define YAW_KD     0.0f    // 禁用
#define PITCH_KP   0.05f
#define PITCH_KD   0.0f    // 禁用
#define MAX_TARGET_DELTA  40
```

### control.c (OpenMV_Control)
```c
#define INNER_DEADZONE  5     // 中心死区阈值
#define OUTER_DEADZONE  15    // 外环阈值(超出此值全速)
```

### OpenMV (main.py)
```python
IMAGE_H = 320
IMAGE_V = 240
IMAGE_CENTER = 128   # QVGA 中心 (160,120) 归一化到 0-255 = 128
```

### STM32 (sys.h / control.h)
```c
#define OPENMV_CENTER_X  128
#define OPENMV_CENTER_Y  128
```

---

## 架构说明

```
OpenMV 检测目标 → 归一化坐标(0-255)通过 UART → STM32
                                              ↓
                              OpenMV_Control() 计算误差 + 软死区
                                              ↓
                              Velocity1/2 (yaw/pitch 速度)
                                              ↓
                              OpenMV_Armed=1 → 跳过 Position_PID
                                              ↓
                              Xianfu_Velocity() → 舵机输出
```

**关键设计**:
- OpenMV 直接写 Velocity1/2，不经过 Position_PID（避免双积分冲突）
- OpenMV 超时/丢失时清零 Velocity1/2
- velocity-mode 架构统一控制流

---

## 待优化方向

1. **极限区平滑**: 如果 5/15 还有边界振荡，可继续缩小 INNER 或扩大过渡区
2. **EMA 平滑输出**: 在 velocity 输出端加一阶低通滤波替代 scale² 缓动
3. **D 项重评估**: 帧率稳定后可尝试加回 D 项（需用真实帧率 dt 而非固定 0.1s）
4. **串口参数调节**: 运行时通过串口调节 KP/死区参数，无需重新烧录

---

## 历史参数对比

| 阶段 | INNER | OUTER | 比例(inner:过渡) | KP | KD | 效果 |
|------|-------|-------|-----------------|----|----|------|
| 硬死区(原始) | 50 | 50 | 1:0 | 0.05 | 0.0 | 边界极限环振荡 |
| 软死区v1 | 30 | 70 | 3:4 | 0.05 | 0.0 | 极限环消除 |
| 软死区v2 | 25 | 60 | 5:7 | 0.05 | 0.0 | 响应更灵敏 |
| 软死区v3 | 15 | 55 | 3:8 | 0.05 | 0.0 | 过渡更平滑 |
| 软死区v4 | 10 | 24 | 3:4 | 0.05 | 0.0 | 比例恢复 |
| **软死区v5(当前)** | **5** | **15** | **1:2** | **0.05** | **0.0** | **测试效果不错** |
