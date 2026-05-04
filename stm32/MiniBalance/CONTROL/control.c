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
#include "control.h"

// OpenMV人形追踪协议定义 (5字节帧)
// 硬件连接: OpenMV TX -> PB11 (USART3_RX)
// 帧格式: [0xFF][0xFE][hasBlob][tx][ty]
//   hasBlob: 0x01=检测到人, 0x00=未检测
//   tx/ty:   归一化坐标 0-255, 128=中心 (QVGA 320x240: 中心(160,120)→128)
// 注意: OpenMV QVGA(320x240), 中心(160,120) → (128,128)
#define OPENMV_CENTER_X        128     // 图像中心X坐标（协议坐标系，0-255范围，128为中心）
#define OPENMV_CENTER_Y        128     // 图像中心Y坐标
#define OPENMV_STALE_TIMEOUT_MS 300U   // 数据陈旧超时时间(ms)
#define OPENMV_MAX_DELTA       10.0f   // 每次最大增量限制,防止跳变
#define OPENMV_PAYLOAD_LEN     3
#define PC_PAYLOAD_LEN         8
#define SERVO_BASE_MIN_PWM     250.0f
#define SERVO_BASE_MAX_PWM     1250.0f
#define SERVO_ARM_MIN_PWM      300.0f
#define SERVO_ARM_MAX_PWM      1200.0f

// 全局变量
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
u8 Flag_Target;
u32 Flash_R_Count;
int Voltage_Temp, Voltage_Count, Voltage_All;

// OpenMV状态标志 (用于OLED显示等)
u8 OpenMV_Target_Lost = 0;      // 目标丢失标志: 0=检测到, 1=未检测到
u8 OpenMV_Data_Stale = 0;       // 数据陈旧标志: 0=数据新鲜, 1=数据超时
u8 OpenMV_Armed = 0;            // OpenMV控制标志: 1=OpenMV正在接管, Position_PID应跳过
u32 OpenMV_Last_Update = 0;    // 最后更新时间(ms)

// 归一化误差（-0.5~+0.5）
float OpenMV_Error_X = 0;       // 水平偏差
float OpenMV_Error_Y = 0;       // 垂直偏差

static float Clamp_Float(float value, float min_value, float max_value);
static void OpenMV_Hold_Current_Position(void);

/**************************************************************************
  函数功能: 定时器中断回调 - 所有控制逻辑在此执行
  说明: 保持此回调简短,让UART RX保持响应
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim2)
    {
        Last_Mode = Mode_Usart_PS2;
        Key_Scan();

        if(delay_flag==1)
        {
            if(++delay_50==5) delay_50=0, delay_flag=0;
        }

        if(Turn_Off(Voltage)==0)
        {
            if(Mode_Usart_PS2 == 0)
            {
                // 模式0: PS2手柄按钮控制 (默认模式)
                Control(Speed/3);
            }
            else if(Mode_Usart_PS2 == 1)
            {
                // 模式1: PC串口角度控制
                Usart_Control();
            }
            else if(Mode_Usart_PS2 == 2)
            {
                // 模式2: OpenMV视觉追踪
                OpenMV_Control();
            }
            else
            {
                // 未知模式,默认PS2控制
                Control(Speed/3);
            }

            Xianfu_Pwm();
            if(OpenMV_Armed)
            {
                // OpenMV模式: OpenMV_Control已设置Velocity1/2,Position_PID应跳过
                OpenMV_Armed = 0;  // 每帧重置，由OpenMV_Control重新设置
            }
            else
            {
                Velocity1 = Position_PID_1(Position1, Target1);
                Velocity2 = Position_PID_2(Position2, Target2);
            }
            Xianfu_Velocity();
            Set_Pwm(Velocity1, Velocity2);
        }

        Led_Flash(100);
    }
}

/**************************************************************************
  函数功能: 设置舵机PWM值,驱动电机转动
  输入参数: velocity1=底舵机PWM增量, velocity2=摇臂舵机PWM增量
  返回值:  无
**************************************************************************/
void Set_Pwm(float velocity1, float velocity2)
{
    Position1 += velocity1;     // 速度的积分,得到当前位置
    Position2 += velocity2;

    Position1 = Clamp_Float(Position1, SERVO_BASE_MIN_PWM, SERVO_BASE_MAX_PWM);
    Position2 = Clamp_Float(Position2, SERVO_ARM_MIN_PWM, SERVO_ARM_MAX_PWM);

    // 写入到STM32的寄存器
    TIM4->CCR3 = Position1;      // GPIOB8,底舵机控制
    TIM4->CCR4 = Position2;       // GPIOB9,摇臂舵机
}

/**************************************************************************
  函数功能: 异常关闭函数 - 低电压保护
  输入参数: voltage=电池电压
  返回值: 1=异常, 0=正常
**************************************************************************/
u8 Turn_Off(int voltage)
{
    u8 temp;
    if(voltage < 1000)  // 电池电压低于10V,停止所有输出
    {
        temp = 1;
    }
    else
    {
        temp = 0;
    }
    return temp;
}

/**************************************************************************
  函数功能: 角度转换为PWM脉宽值 (180度舵机)
  输入参数: angle=目标角度(度)
  返回值: 对应PWM值
**************************************************************************/
int angle_to_pwm_180(float angle)
{
    int pwm = 250 + (1000 * angle) / 180;
    return pwm;
}

/**************************************************************************
  函数功能: 角度转换为PWM脉宽值 (270度舵机)
  输入参数: angle=目标角度(度)
  返回值: 对应PWM值
**************************************************************************/
int angle_to_pwm_270(float angle)
{
    int pwm = 250 + (1000 * angle) / 270;
    return pwm;
}

/**************************************************************************
  函数功能: 限制PWM值在安全范围内
  输入参数: 无
  返回值: 无
**************************************************************************/
void Xianfu_Pwm(void)
{
    // Target1为底舵机, Target2为摇臂舵机
    if(Target1 < SERVO_BASE_MIN_PWM) Target1 = SERVO_BASE_MIN_PWM;
    if(Target1 > SERVO_BASE_MAX_PWM) Target1 = SERVO_BASE_MAX_PWM;

    if(Target2 < SERVO_ARM_MIN_PWM) Target2 = SERVO_ARM_MIN_PWM;
    if(Target2 > SERVO_ARM_MAX_PWM) Target2 = SERVO_ARM_MAX_PWM;
}

/**************************************************************************
  函数功能: 限制速度幅值
  输入参数: 无
  返回值: 无
**************************************************************************/
void Xianfu_Velocity(void)
{
    // OpenMV模式: OpenMV_Control已按OPENMV_MAX_DELTA做独立限幅
    if(OpenMV_Armed) return;

    int Amplitude_H = Speed, Amplitude_L = -Speed;
    if(Velocity1 < Amplitude_L) Velocity1 = Amplitude_L;
    if(Velocity1 > Amplitude_H) Velocity1 = Amplitude_H;
    if(Velocity2 < Amplitude_L) Velocity2 = Amplitude_L;
    if(Velocity2 > Amplitude_H) Velocity2 = Amplitude_H;
}

/**************************************************************************
  函数功能: PS2手柄控制函数
  输入参数: Step=每次按键的角度增量
  返回值: 无
**************************************************************************/
void Control(float Step)
{
    if(PS2_KEY==8)      Target1 += Step;      // 底舵机向左
    else if(PS2_KEY==6) Target1 -= Step;      // 底舵机向右
    else if(PS2_KEY==5) Target2 -= Step;      // 摇臂向下
    else if(PS2_KEY==7) Target2 += Step;      // 摇臂向上

    if(PS2_KEY==11) Speed += 0.05;             // 速度增加
    else if(PS2_KEY==9) Speed -= 0.05;        // 速度减少
    if(Speed <= 3) Speed = 3;
    if(Speed >= 30) Speed = 30;
}

/*************************************************************************
  函数功能: 位置式PID控制器
  输入参数: Position=当前位置, Target=目标位置, servo=舵机索引(0=底舵机,1=摇臂舵机)
  返回值: PWM增量值
**************************************************************************/
float Position_PID(float Position, float Target, u8 servo)
{
    // 每个舵机独立的PID状态
    static float Bias[2] = {0}, Pwm[2] = {0}, Integral_bias[2] = {0}, Last_Bias[2] = {0};
    float Bias_kp, Bias_ki, Bias_kd;

    // 积分限幅 (Anti-Windup)
    #define INTEGRAL_LIMIT 1000
    #define INTEGRAL_DEADZONE 50

    Bias[servo] = Target - Position;  // 位置偏差

    // 积分分离: 偏差大于阈值时清零积分项防止积分饱和,偏差小于阈值时累加积分
    if(myabs(Bias[servo]) > INTEGRAL_DEADZONE)
    {
        Integral_bias[servo] = 0;
    }
    else
    {
        Integral_bias[servo] += Bias[servo];
    }

    // 积分限幅防止积分饱和
    if(Integral_bias[servo] > INTEGRAL_LIMIT) Integral_bias[servo] = INTEGRAL_LIMIT;
    if(Integral_bias[servo] < -INTEGRAL_LIMIT) Integral_bias[servo] = -INTEGRAL_LIMIT;

    // PID计算
    Bias_kp = Position_KP * Bias[servo] / 100;
    Bias_ki = Position_KI * Integral_bias[servo] / 100;
    Bias_kd = Position_KD * (Bias[servo] - Last_Bias[servo]) / 100;

    Pwm[servo] = Bias_kp + Bias_ki + Bias_kd;
    Last_Bias[servo] = Bias[servo];  // 保存本次偏差

    return Pwm[servo];
}

// 兼容宏: 底舵机和摇臂舵机分别使用索引0和1
#define Position_PID_1(pos, tgt) Position_PID(pos, tgt, 0)
#define Position_PID_2(pos, tgt) Position_PID(pos, tgt, 1)

/**************************************************************************
  函数功能: 绝对值函数
  输入参数: a=int类型数值
  返回值: unsigned int
**************************************************************************/
int myabs(int a)
{
    int temp;
    if(a < 0) temp = -a;
    else temp = a;
    return temp;
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

/**************************************************************************
  函数功能: PC串口控制 - 通过BCC校验解析目标角度对应PWM值
  输入参数: 无
  返回值: 无
**************************************************************************/
void Usart_Control()
{
    u8 payload[PC_PAYLOAD_LEN];
    u8 i;
    uint32_t primask;

    if(Pc_Usart_Compelet == 0)
    {
        return;
    }

    // 临界区保护
    primask = __get_PRIMASK();
    __disable_irq();
    for(i = 0; i < PC_PAYLOAD_LEN; i++)
    {
        payload[i] = Pc_Rxbuf[i];
    }
    Pc_Usart_Compelet = 0;
    if(primask == 0U)
    {
        __enable_irq();
    }

    if(payload[0] != 0)
    {
        Target1 = payload[0] * 1000;
        Target1 = Target1 / 270 + 250;
    }
    if(payload[1] != 0)
    {
        Target2 = payload[1] * 1000;
        Target2 = Target2 / 180 + 250;
    }
}

/**************************************************************************
  函数功能: 按键扫描 - 支持三种控制模式循环切换
  调用位置: TIM2定时器中断 (100Hz/10ms周期)
  模式定义: Mode_Usart_PS2 = 0 -> PS2手柄控制 (默认)
                               1 -> PC串口控制
                               2 -> OpenMV视觉追踪
  切换逻辑: KEY_S单按键单击时,Mode_Usart_PS2 = (Mode_Usart_PS2 + 1) % 3
**************************************************************************/
void Key_Scan(void)
{
    u8 temp;
    temp = click_N_Double(50);

    if(temp == 1)  // 单击: 切换到下一个模式
    {
        Mode_Usart_PS2 = (Mode_Usart_PS2 + 1) % 3;
    }
}

/**************************************************************************
  函数功能: OpenMV视觉伺服控制 - PD控制方式
  调用位置: TIM2定时器中断中,当Mode_Usart_PS2==2时调用
  控制策略: 归一化误差(-0.5~+0.5) → PD控制 → 直接设角度
            替代原来的简单偏差叠加方式,避免双重积分震荡
  调试说明: YAW_KP/KD, PITCH_KP/KD影响跟踪响应，KD已启用
  注意事项: OPENMV_CENTER_X/Y = 128 为图像中心
**************************************************************************/
void OpenMV_Control(void)
{
    static uint32_t last_frame_tick = 0;
    static float last_error_x = 0.0f, last_error_y = 0.0f;
    static float yaw_deriv = 0.0f, pitch_deriv = 0.0f;  // D项低通滤波状态
    float error_x = 0.0f, error_y = 0.0f;

    u8 payload[OPENMV_PAYLOAD_LEN];
    u8 i;
    u8 hasBlob;
    u8 target_x;
    u8 target_y;
    u8 detect_flag;
    uint32_t primask;
    uint32_t now;

    now = HAL_GetTick();

    // 检查是否有新数据
    if(OpenMV_Usart_Compelet == 0)
    {
        // 无新数据时也保持OpenMV接管,避免在视觉帧间回落到Position_PID
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

    // 复制数据到本地缓冲区 (临界区保护)
    primask = __get_PRIMASK();
    __disable_irq();
    for(i = 0; i < OPENMV_PAYLOAD_LEN; i++)
    {
        payload[i] = OpenMV_Rxbuf[i];
    }
    OpenMV_Usart_Compelet = 0;
    if(primask == 0U)
    {
        __enable_irq();
    }

    // payload[0]=hasBlob, [1]=tx, [2]=ty (5字节协议: 帧头2字节在状态机中已跳过)
    hasBlob = payload[0];
    target_x = payload[1];
    target_y = payload[2];
    detect_flag = hasBlob;  // hasBlob 0x01=检测到, 0x00=未检测

    if(detect_flag != 0x00 && detect_flag != 0x01)
    {
        OpenMV_Hold_Current_Position();
        OpenMV_Data_Stale = 1;
        last_frame_tick = 0U;
        yaw_deriv = 0.0f;
        pitch_deriv = 0.0f;
        last_error_x = 0.0f;
        last_error_y = 0.0f;
        return;
    }

    last_frame_tick = now;
    OpenMV_Last_Update = now;
    OpenMV_Data_Stale = 0;  // 数据新鲜

    if(detect_flag == 0x01)
    {
        // 检测到人形目标

        // 归一化误差: 0=左/上边界, 128=中心, 255=右/下边界
        // 误差 = (raw - 128) / 255.0 → -0.5 ~ +0.5
        float new_error_x = ((float)target_x - OPENMV_CENTER_X) / 255.0f;
        float new_error_y = ((float)target_y - OPENMV_CENTER_Y) / 255.0f;

        // 软死区(带滞后): 中心区完全抑制,边界区平滑过渡,消除极限环振荡
        // 内环: ±5px 完全死区; 外环: 5-15px 平滑缩减; 15px+ 全速响应
        #define INNER_DEADZONE  5     // 中心死区阈值
        #define OUTER_DEADZONE  15    // 外环阈值(超出此值全速)
        int raw_dx = (int)((float)target_x - OPENMV_CENTER_X);
        int raw_dy = (int)((float)target_y - OPENMV_CENTER_Y);
        int abs_dx = myabs(raw_dx);
        int abs_dy = myabs(raw_dy);

        // X轴: 计算缩放系数(0~1),在内环死区快速衰减到0
        float scale_x = 1.0f;
        if(abs_dx <= INNER_DEADZONE) {
            scale_x = 0.0f;  // 中心死区: 完全不响应
        } else if(abs_dx < OUTER_DEADZONE) {
            // 外环内: 线性或二次缓动过渡到1.0
            scale_x = (float)(abs_dx - INNER_DEADZONE) / (float)(OUTER_DEADZONE - INNER_DEADZONE);
            scale_x = scale_x * scale_x;  // 二次缓动,边界处修正更轻柔
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

        error_x = new_error_x;  // 直接用raw error，无滤波滞后
        error_y = new_error_y;

        // 更新全局误差值（供显示/调试）
        OpenMV_Error_X = error_x;
        OpenMV_Error_Y = error_y;

        // PD控制: P项响应误差, D项(一阶低通滤波)抑制overshoot/震荡
        // 注意: dt假定=0.1s (100ms), 但实际帧间隔约17-33ms (30-60fps)
        //       若将来启用D项(KD>0), 需用真实帧间隔替换硬编码的0.1f
        //       当前KD=0所以此硬编码dt不影响控制输出
        float alpha = 0.1f;  // 滤波系数小,D项更平滑,抑制高速帧率下的噪声放大
        float filtered_yaw = alpha * ((error_x - last_error_x) / 0.1f) + (1.0f - alpha) * yaw_deriv;
        float filtered_pitch = alpha * ((error_y - last_error_y) / 0.1f) + (1.0f - alpha) * pitch_deriv;
        yaw_deriv = filtered_yaw;
        pitch_deriv = filtered_pitch;

        float yaw_out = YAW_KP * error_x + YAW_KD * yaw_deriv;
        float pitch_out = PITCH_KP * error_y + PITCH_KD * pitch_deriv;

        // 转换为 Velocity（velocity-mode），直接设速度不经过 Target/Position_PID
        // PD输出(归一化) → velocity单位, 统一经 OPENMV_MAX_DELTA 限幅
        // 注意: velocity-mode符号与位置积分模式相反，取反以匹配追踪方向
        float yaw_vel = -(yaw_out * 500.0f);
        float pitch_vel = -(pitch_out * 500.0f);
        Velocity1 = Clamp_Float(yaw_vel, -OPENMV_MAX_DELTA, OPENMV_MAX_DELTA);
        Velocity2 = Clamp_Float(pitch_vel, -OPENMV_MAX_DELTA, OPENMV_MAX_DELTA);
        OpenMV_Armed = 1;

        // Target 限幅由主循环 Xianfu_Pwm() 统一处理
        last_error_x = error_x;
        last_error_y = error_y;
        OpenMV_Target_Lost = 0;  // 目标存在
    }
    else
    {
        // 目标丢失: 停在当前位置
        OpenMV_Hold_Current_Position();
        yaw_deriv = 0.0f;
        pitch_deriv = 0.0f;
        last_error_x = 0.0f;
        last_error_y = 0.0f;
    }
}
