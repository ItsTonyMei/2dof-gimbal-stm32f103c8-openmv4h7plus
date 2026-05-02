/***********************************************
广州万至达科技有限公司
产品名称: WHEELTEC
官网: wheeltec.net
淘宝店: shop114407458.taobao.com
阿里通: https://minibalance.aliexpress.com/store/4455017
版本: V1.0
修改时间: 2022-10-13

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update: 2022-10-13

All rights reserved
***********************************************/
#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265
#define FILTERING_TIMES  10

// 全局变量（Velocity1/2 在 sys.h 中声明为 float）
extern int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
extern u8 Flag_Target;
extern u32 Flash_R_Count;
extern int Voltage_Temp, Voltage_Count, Voltage_All;

// OpenMV状态标志 (可在OLED等显示)
extern u8 OpenMV_Target_Lost;       // 目标丢失标志: 0=检测到, 1=未检测到
extern u8 OpenMV_Data_Stale;        // 数据陈旧标志: 0=数据新鲜, 1=数据超时
extern u8 OpenMV_Armed;             // OpenMV控制标志: 1=OpenMV正在接管, Position_PID应跳过
extern u32 OpenMV_Last_Update;     // 最后更新时间(ms)
extern int Last_Mode;              // 上次的控制模式,用于检测模式切换

// 函数声明
int EXTI15_10_IRQHandler(void);
u8 Kinematic_Analysis(float x, float y, float Beta, float Alpha, float Gamma);
void Set_Pwm(float velocity1, float velocity2);
void Xianfu_Pwm(void);
void Xianfu_Velocity(void);
u8 Turn_Off(int voltage);
void Get_Angle(u8 way);
int myabs(int a);
void Control(float Step);
float Position_PID(float Position, float Target, u8 servo);

// 兼容宏: 底舵机和摇臂舵机分别使用索引0和1
#define Position_PID_1(pos, tgt) Position_PID(pos, tgt, 0)
#define Position_PID_2(pos, tgt) Position_PID(pos, tgt, 1)
void Usart_Control(void);
void Key_Scan(void);

/**************************************************************************
  OpenMV人形追踪相关
**************************************************************************/
void OpenMV_Control(void);                 // OpenMV视觉伺服控制函数

// PD控制参数 (归一化误差 -> PWM)
// 高速帧率下D项会放大噪声，改用纯P+大死区抑制抖动
// 若需更快响应可适当加大KP，但死区优先保证稳态不抖
#define YAW_KP     0.05f
#define YAW_KD     0.0f   // 禁用D项: 高速帧率下D项放大噪声
#define PITCH_KP   0.05f
#define PITCH_KD   0.0f   // 禁用D项: 高速帧率下D项放大噪声

// 归一化误差（-0.5~+0.5）
extern float OpenMV_Error_X;  // 水平偏差
extern float OpenMV_Error_Y;  // 垂直偏差

#endif
