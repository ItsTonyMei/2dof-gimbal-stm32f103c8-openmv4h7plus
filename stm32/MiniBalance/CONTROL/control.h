#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

// OpenMV 5-byte protocol: [0xFF][0xFE][hasBlob][tx][ty]
// hasBlob: 0x01=target detected, 0x00=lost
// tx/ty: normalized 0-255, 128=center (QVGA 320x240 → center(160,120)→128)
#define OPENMV_CENTER_X        128
#define OPENMV_CENTER_Y        128
#define OPENMV_STALE_TIMEOUT_MS 300U
#define OPENMV_MAX_DELTA       10.0f
#define OPENMV_PAYLOAD_LEN     3

#define SERVO_BASE_MIN_PWM     250.0f
#define SERVO_BASE_MAX_PWM     1250.0f
#define SERVO_ARM_MIN_PWM      300.0f
#define SERVO_ARM_MAX_PWM      1200.0f

// PD gains (normalized error → velocity)
#define YAW_KP     0.05f
#define YAW_KD     0.0f
#define PITCH_KP   0.05f
#define PITCH_KD   0.0f

// OpenMV status flags
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
