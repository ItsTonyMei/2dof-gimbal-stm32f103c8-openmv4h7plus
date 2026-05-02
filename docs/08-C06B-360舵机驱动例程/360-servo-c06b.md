# C06B 360° 舵机驱动例程

## 1. 概述

本文档介绍 **C06B 主控板（STM32F103C8T6）** 的 360° 舵机基础驱动例程。这是一个独立的入门级例程，专注于演示如何通过按键控制 360° 舵机的转速和方向。

**注意**：此例程是独立的 360° 舵机基础演示，与二自由度云台源码（8种配置）不同。它仅含单舵机控制逻辑，不涉及云台的姿态控制。

**源码位置**：
```
附送资料/360°舵机基础控制例程/360°舵机基础控制例程/WHEELTEC_C06B（STM32F103C8T6）_360°舵机驱动例程.zip
```

**实现效果**：
- 单击：舵机加速（每单击 +50）
- 双击：舵机减速（每双击 -50）
- 长按：舵机停止（target_pwm = 0）

---

## 2. 硬件接线

| 引脚 | 连接 | 说明 |
|------|------|------|
| PB9 | 舵机信号线 | TIM4_CH4 PWM 输出 |
| GND | 舵机 GND | 共地 |
| 5V | 舵机 5V | 供电 |

**PWM 引脚分配**：
- PB8 → TIM4_CH3（备用）
- PB9 → TIM4_CH4（主用）

---

## 3. PWM 配置

### 3.1 TIM4 初始化参数

```c
TIM4_PWM_Init(9999, 71);  // 50Hz, 周期 20ms
```

| 参数 | 值 | 说明 |
|------|----|------|
| ARR (自动重装载值) | 9999 | 计数周期 10000 |
| PSC (预分频) | 71 | 72MHz / 72 = 1MHz |
| PWM 频率 | 50Hz | 1MHz / 10000 = 100kHz / 10000 |
| 周期 | 20ms | 1 / 50Hz |

### 3.2 舵机中值（PTZ_MIDDLE）

```c
#define PTZ_MIDDLE 750   // 1.5ms 脉宽，360° 舵机停止位置
```

### 3.3 PWM 脉宽与转速关系

```
脉宽 = CCR4 值（TIM4->CCR4）

脉宽 < 750  → 反向旋转（速度与偏离程度成正比）
脉宽 = 750  → 停止
脉宽 > 750  → 正向旋转（速度与偏离程度成正比）
```

### 3.4 速度控制函数

```c
// speed 范围：-400 ~ 400
int servo_speed_control(int speed) {
    if (speed > 400)  speed = 400;
    if (speed < -400) speed = -400;

    if (speed > 0)
        pwm = 1600 + speed;   // 正向：1500 ~ 1900
    else if (speed < 0)
        pwm = 1400 + speed;   // 反向：1100 ~ 1500
    else
        pwm = 1500;           // 停止

    return pwm;
}
```

| speed 值 | PWM 输出 | 舵机行为 |
|----------|----------|----------|
| +400 | 1900 | 最快正向 |
| +200 | 1700 | 中速正向 |
| 0 | 1500 | 停止 |
| -200 | 1300 | 中速反向 |
| -400 | 1100 | 最快反向 |

---

## 4. 按键控制逻辑

### 4.1 按键定义

```c
#define KEY PAin(5)   // PA5 作为按键输入

enum {
    key_stateless,   // 无按键
    single_click,    // 单击
    double_click,    // 双击
    long_click       // 长按
};
```

### 4.2 按键扫描时间参数

| 参数 | 值 | 说明 |
|------|----|------|
| 扫描频率 | 100Hz（由 TIM2 中断 10ms 调用一次） | KEY_Scan(100, 0) |
| 双击判定 | 50ms ~ 300ms 内再次按下 | 超出 300ms 视为单击 |
| 长按判定 | 超过 500ms | 返回 long_click |

### 4.3 按键处理（TIM2 中断）

```c
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        uint8_t keystate = KEY_Scan(100, 0);
        switch(keystate) {
            case single_click:
                target_pwm += 50;   // 加速
                break;
            case double_click:
                target_pwm -= 50;   // 减速
                break;
            case long_click:
                target_pwm = 0;     // 停止
                break;
        }

        if (target_pwm > 400)  target_pwm = 400;
        if (target_pwm < -400) target_pwm = -400;

        TIM4->CCR3 = servo_speed_control(target_pwm);
        TIM4->CCR4 = servo_speed_control(target_pwm);
    }
}
```

### 4.4 target_pwm 限幅

| 限制 | 值 |
|------|-----|
| 上限 | +400 |
| 下限 | -400 |
| 步进（单击/双击） | ±50 |

---

## 5. OLED 显示

主循环中实时刷新 OLED 显示以下信息：

| 位置 | 内容 | 说明 |
|------|------|------|
| (0,0) | target_pwm | 当前目标 PWM 值（4位，小数2位） |
| (0,10) | TIM4->CCR3 | 实际 PWM 比较值 |
| (0,20) | TIM4->CCR4 | 实际 PWM 比较值 |
| (80,50) | Voltage | 电源电压（2位小数） |
| (120,50) | V | 单位 |

---

## 6. 电压监测

```c
// TIM2 每 100 次中断（约 1 秒）计算一次平均电压
Voltage_All += Get_battery_volt();
if (++Voltage_Count == 100)
    Voltage = (float)Voltage_All / 10000.0f;
```

---

## 7. 目录结构

```
WHEELTEC_C06B_360°舵机驱动例程/
├── USER/
│   ├── MiniBalance.c           # 主程序入口
│   └── MiniBalance.uvprojx    # Keil 工程文件
├── MiniBalance_HARDWARE/
│   ├── servo.c / servo.h      # TIM4 PWM 舵机驱动
│   ├── timer.c / timer.h      # TIM2 定时器（按键扫描）
│   ├── key.c / key.h          # 按键扫描
│   ├── adc.c / adc.h          # ADC 电池电压监测
│   ├── led.c / led.h          # LED 指示
│   └── oled.c / oled.h        # OLED 显示
├── MiniBalance/CONTROL/
│   ├── control.c              # 控制逻辑、TIM2 中断
│   └── control.h
├── SYSTEM/
│   ├── delay/                 # 延时
│   ├── sys/                   # 系统初始化
│   └── usart/                 # 串口（usart1, 230400bps）
├── OBJ/
│   └── MiniBalance.hex        # 编译产物（可直接烧录）
└── WHEELTEC.bin
```

---

## 8. 与云台源码的关键差异

| 项目 | C06B 360° 舵机基础例程 | 二自由度云台源码 |
|------|------------------------|------------------|
| 舵机数量 | 1 个 | 2 个（底舵机 + 摇臂） |
| 控制方式 | 按键 + 速度值 | PS2手柄 / 串口 / ROS / Python |
| 控制目标 | 转速（-400~400） | 角度（0~180° 或 0~270°） |
| 360° 处理 | 开环速度控制 | 开环速度控制（仅 PS2） |
| 串口协议 | 无 | 10字节帧格式（0xFF 0xFE ...） |
| 适用场景 | 360° 舵机入门学习 | 完整云台控制 |

---

## 9. 快速验证步骤

1. 解压例程，找到 `OBJ/MiniBalance.hex`
2. 使用 FlyMcu 通过 USB 一键下载到 C06B 板
3. 确认 PB9 → 舵机信号线，GND → 舵机GND，5V → 舵机5V
4. 给舵机供电（7-12V）
5. 单击 PA5 按键，观察舵机加速；双击减速；长按停止
6. 观察 OLED 上 target_pwm 和 CCR3/CCR4 数值变化
