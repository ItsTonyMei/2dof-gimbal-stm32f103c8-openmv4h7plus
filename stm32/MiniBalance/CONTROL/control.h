#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

// ---- OpenMV 通讯协议 ----
// 5字节帧: [0xFF][0xFE][hasBlob][tx][ty]
//   hasBlob: 0x01=检测到目标, 0x00=目标丢失
//   tx/ty:   归一化坐标 0-255, 128=中心 (QVGA 320x240: 中心(160,120)→128)
#define OPENMV_CENTER_X          128
#define OPENMV_CENTER_Y          128
#define OPENMV_STALE_TIMEOUT_MS  300U
#define OPENMV_MAX_DELTA         10.0f
#define OPENMV_PAYLOAD_LEN       3

// ---- 舵机 PWM 限幅 ----
#define SERVO_BASE_MIN_PWM       250.0f
#define SERVO_BASE_MAX_PWM       1250.0f
#define SERVO_ARM_MIN_PWM        300.0f
#define SERVO_ARM_MAX_PWM        1200.0f

// ---- PD 控制增益 (归一化误差 → 速度输出) ----
#define YAW_KP     0.05f
#define YAW_KD     0.0f
#define PITCH_KP   0.05f
#define PITCH_KD   0.0f

// ---- PD 软死区 (二次缓动过渡) ----
#define PD_INNER_DEADZONE        5      // px, 完全死区半径
#define PD_OUTER_DEADZONE        15     // px, 死区过渡带外边界
#define PD_EMA_ALPHA             0.1f   // 微分项 EMA 滤波系数

// ---- ADC 电池监测 ----
#define BATTERY_ADC_RAW_LOW      1000   // 低电压关断阈值 (ADC 原始值)
#define BATTERY_SAMPLE_COUNT     100    // 电压滑动平均采样数

// ---- 调试输出周期 ----
#define DEBUG_PRINTF_INTERVAL_MS 500U

// OpenMV 状态标志
extern u8 OpenMV_Target_Lost;
extern u8 OpenMV_Data_Stale;
extern u8 OpenMV_Armed;
extern u32 OpenMV_Last_Update;
extern float OpenMV_Error_X;
extern float OpenMV_Error_Y;

void Set_Pwm(float velocity1, float velocity2);
void Xianfu_Pwm(void);
void Xianfu_Velocity(void);
u8 Turn_Off(int voltage);
void OpenMV_Control(void);

#endif
