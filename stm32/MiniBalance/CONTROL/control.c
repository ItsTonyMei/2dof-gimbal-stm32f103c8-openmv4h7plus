#include "control.h"

// Global state
int Voltage_Temp, Voltage_Count, Voltage_All;
u8 OpenMV_Target_Lost = 0;
u8 OpenMV_Data_Stale = 0;
u8 OpenMV_Armed = 0;
u32 OpenMV_Last_Update = 0;
float OpenMV_Error_X = 0;
float OpenMV_Error_Y = 0;

static float Clamp_Float(float value, float min_value, float max_value);
static void OpenMV_Hold_Current_Position(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim2)
    {
        if(delay_flag == 1)
        {
            if(++delay_50 == 5) delay_50 = 0, delay_flag = 0;
        }

        if(Turn_Off(Voltage) == 0)
        {
            OpenMV_Control();
            Xianfu_Pwm();
            Set_Pwm(Velocity1, Velocity2);
        }

        Led_Flash(100);
    }
}

void Set_Pwm(float velocity1, float velocity2)
{
    Position1 += velocity1;
    Position2 += velocity2;

    Position1 = Clamp_Float(Position1, SERVO_BASE_MIN_PWM, SERVO_BASE_MAX_PWM);
    Position2 = Clamp_Float(Position2, SERVO_ARM_MIN_PWM, SERVO_ARM_MAX_PWM);

    TIM4->CCR3 = Position1;  // PB8, base servo
    TIM4->CCR4 = Position2;  // PB9, arm servo
}

u8 Turn_Off(int voltage)
{
    return (voltage < 1000) ? 1 : 0;
}

void Xianfu_Pwm(void)
{
    if(Target1 < SERVO_BASE_MIN_PWM) Target1 = SERVO_BASE_MIN_PWM;
    if(Target1 > SERVO_BASE_MAX_PWM) Target1 = SERVO_BASE_MAX_PWM;
    if(Target2 < SERVO_ARM_MIN_PWM) Target2 = SERVO_ARM_MIN_PWM;
    if(Target2 > SERVO_ARM_MAX_PWM) Target2 = SERVO_ARM_MAX_PWM;
}

void Xianfu_Velocity(void)
{
    if(OpenMV_Armed) return;
    if(Velocity1 < -OPENMV_MAX_DELTA) Velocity1 = -OPENMV_MAX_DELTA;
    if(Velocity1 > OPENMV_MAX_DELTA) Velocity1 = OPENMV_MAX_DELTA;
    if(Velocity2 < -OPENMV_MAX_DELTA) Velocity2 = -OPENMV_MAX_DELTA;
    if(Velocity2 > OPENMV_MAX_DELTA) Velocity2 = OPENMV_MAX_DELTA;
}

static float Clamp_Float(float value, float min_value, float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static void OpenMV_Hold_Current_Position(void)
{
    Velocity1 = 0;
    Velocity2 = 0;
    Target1 = Position1;
    Target2 = Position2;
    OpenMV_Armed = 1;
    OpenMV_Target_Lost = 1;
    OpenMV_Error_X = 0.0f;
    OpenMV_Error_Y = 0.0f;
}

// OpenMV visual servo PD controller
// Normalized error (-0.5~+0.5) → PD → velocity output
// Soft deadzone: 5px inner deadband, 5-15px quadratic easing, 15px+ full response
void OpenMV_Control(void)
{
    static uint32_t last_frame_tick = 0;
    static float last_error_x = 0.0f, last_error_y = 0.0f;
    static float yaw_deriv = 0.0f, pitch_deriv = 0.0f;

    u8 payload[OPENMV_PAYLOAD_LEN];
    u8 i;
    uint32_t primask;
    uint32_t now = HAL_GetTick();

    if(OpenMV_Usart_Compelet == 0)
    {
        Velocity1 = 0;
        Velocity2 = 0;
        Target1 = Position1;
        Target2 = Position2;
        OpenMV_Armed = 1;
        if(last_frame_tick == 0U || (now - last_frame_tick) > OPENMV_STALE_TIMEOUT_MS)
        {
            OpenMV_Data_Stale = 1;
            OpenMV_Target_Lost = 1;
            OpenMV_Error_X = 0.0f;
            OpenMV_Error_Y = 0.0f;
            yaw_deriv = 0.0f;
            pitch_deriv = 0.0f;
            last_error_x = 0.0f;
            last_error_y = 0.0f;
        }
        else
        {
            OpenMV_Data_Stale = 0;
        }
        return;
    }

    // Copy payload with IRQ protection
    primask = __get_PRIMASK();
    __disable_irq();
    for(i = 0; i < OPENMV_PAYLOAD_LEN; i++)
        payload[i] = OpenMV_Rxbuf[i];
    OpenMV_Usart_Compelet = 0;
    if(primask == 0U) __enable_irq();

    u8 hasBlob   = payload[0];
    u8 target_x  = payload[1];
    u8 target_y  = payload[2];

    if(hasBlob != 0x00 && hasBlob != 0x01)
    {
        OpenMV_Hold_Current_Position();
        OpenMV_Data_Stale = 1;
        last_frame_tick = 0U;
        yaw_deriv = 0.0f;
        pitch_deriv = 0.f;
        last_error_x = 0.0f;
        last_error_y = 0.0f;
        return;
    }

    last_frame_tick = now;
    OpenMV_Last_Update = now;
    OpenMV_Data_Stale = 0;

    if(hasBlob == 0x01)
    {
        float new_error_x = ((float)target_x - OPENMV_CENTER_X) / 255.0f;
        float new_error_y = ((float)target_y - OPENMV_CENTER_Y) / 255.0f;

        // Soft deadzone with quadratic easing
        #define INNER_DEADZONE  5
        #define OUTER_DEADZONE  15

        int raw_dx = (int)(target_x - OPENMV_CENTER_X);
        int raw_dy = (int)(target_y - OPENMV_CENTER_Y);
        int abs_dx = raw_dx < 0 ? -raw_dx : raw_dx;
        int abs_dy = raw_dy < 0 ? -raw_dy : raw_dy;

        float scale_x = 1.0f;
        if(abs_dx <= INNER_DEADZONE) {
            scale_x = 0.0f;
        } else if(abs_dx < OUTER_DEADZONE) {
            scale_x = (float)(abs_dx - INNER_DEADZONE) / (float)(OUTER_DEADZONE - INNER_DEADZONE);
            scale_x = scale_x * scale_x;
        }
        new_error_x = ((float)raw_dx) / 255.0f * scale_x;

        float scale_y = 1.0f;
        if(abs_dy <= INNER_DEADZONE) {
            scale_y = 0.0f;
        } else if(abs_dy < OUTER_DEADZONE) {
            scale_y = (float)(abs_dy - INNER_DEADZONE) / (float)(OUTER_DEADZONE - INNER_DEADZONE);
            scale_y = scale_y * scale_y;
        }
        new_error_y = ((float)raw_dy) / 255.0f * scale_y;

        OpenMV_Error_X = new_error_x;
        OpenMV_Error_Y = new_error_y;

        // PD control (KD=0 by default → P-only)
        float alpha = 0.1f;
        float filtered_yaw   = alpha * ((new_error_x - last_error_x) / 0.1f) + (1.0f - alpha) * yaw_deriv;
        float filtered_pitch = alpha * ((new_error_y - last_error_y) / 0.1f) + (1.0f - alpha) * pitch_deriv;
        yaw_deriv = filtered_yaw;
        pitch_deriv = filtered_pitch;

        float yaw_out   = YAW_KP * new_error_x + YAW_KD * yaw_deriv;
        float pitch_out = PITCH_KP * new_error_y + PITCH_KD * pitch_deriv;

        float yaw_vel   = -(yaw_out * 500.0f);
        float pitch_vel = -(pitch_out * 500.0f);
        Velocity1 = Clamp_Float(yaw_vel,   -OPENMV_MAX_DELTA, OPENMV_MAX_DELTA);
        Velocity2 = Clamp_Float(pitch_vel, -OPENMV_MAX_DELTA, OPENMV_MAX_DELTA);
        OpenMV_Armed = 1;

        last_error_x = new_error_x;
        last_error_y = new_error_y;
        OpenMV_Target_Lost = 0;
    }
    else
    {
        OpenMV_Hold_Current_Position();
        yaw_deriv = 0.0f;
        pitch_deriv = 0.0f;
        last_error_x = 0.0f;
        last_error_y = 0.0f;
    }
}
