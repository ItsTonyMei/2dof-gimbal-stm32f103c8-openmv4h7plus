# 2-DOF Gimbal — OpenMV N6 + Wheeltec Stepper

## 项目概述 (Project Overview)

双轴步进云台,OpenMV N6 单板方案:BlazeFace 人脸检测 + PD 控制,直接驱动
Wheeltec d36ax1 双路步进电机驱动模块(带稳压)。**无外部 MCU**。

| 组成部分 | 型号 | 说明 |
|---|---|---|
| 视觉+控制 | OpenMV N6 | BlazeFace 检测 + PD 控制器,直驱步进 |
| 驱动模块 | Wheeltec d36ax1 双路步进驱动(带稳压) | STEP/DIR/EN 脉冲接口 + ADC 反馈 |
| 机械 | 2 轴云台 | Yaw (底) + Pitch (臂),步进电机 |

## 架构变更历史 (History)

- **2026-08**:硬件换代 — 移除 STM32F103C8T6 + 舵机方案,改为 N6 直驱步进。
  仓库从 `2dof-gimbal-stm32f103c8-openmv4h7plus` 更名为 `2dof-gimbal-openmv-n6-stepper`。
- **2026-07 之前 (git 历史)**:H7+ → STM32 UART3 5 字节协议 → PD → TIM4 PWM(已废弃,可从历史找回)。

## 构建与烧录 (Build & Flash)

- **OpenMV**:MicroPython,IDE 里把 `openmv/main.py` 复制到相机闪存
- 无 Keil/GCC 工程(ST M32 架构已移除)

## 关键文件 (Key Files)

- `openmv/main.py` — 当前固件:**仍是舵机 PWM 版本,控制层待重写为步进脉冲**
- `docs/wiring.md` — d36ax1 接线表(引脚待确认)

## 待办 (TODO)

- [ ] `openmv/main.py` 控制层:舵机 PWM → 两路步进脉冲 (STEP/DIR/EN) + ADC 反馈
- [ ] `docs/wiring.md` 填入实际 N6 引脚
- [ ] PD 参数针对步进电机重新整定
- [ ] 步进细分/加减速策略(可选)
