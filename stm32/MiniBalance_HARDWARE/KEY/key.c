/***********************************************
 * 广州万至达科技有限公司
 * 产品名称: WHEELTEC
 * 官网: wheeltec.net
 * 淘宝店: shop114407458.taobao.com
 * 阿里通: https://minibalance.aliexpress.com/store/4455017
 * 版本: V1.0
 * 修改时间: 2022-10-13
 *
 * Brand: WHEELTEC
 * Website: wheeltec.net
 * Taobao shop: shop114407458.taobao.com
 * Aliexpress: https://minibalance.aliexpress.com/store/4455017
 * Version: V1.0
 * Update: 2022-10-13
 *
 * All rights reserved
 ***********************************************/
#include "key.h"

/**************************************************************************
 * 函数功能: 按键初始化
 * 入口参数: 无
 * 返回值:  无
 **************************************************************************/
void KEY_Init(void)
{
    HAL_PWR_EnableBkUpAccess();        // 允许修改RTC和备份寄存器
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF); // 关闭外部低速时钟，降低PC13/14/15功耗
    __HAL_RCC_BACKUPRESET_FORCE();     // 复位备份域
    __HAL_RCC_BACKUPRESET_RELEASE();    // 释放备份域复位
}

/**************************************************************************
 * 函数功能: 按键扫描 (支持单击和双击检测)
 * 入口参数: time=双击等待时间
 * 返回值:  0=无动作, 1=单击, 2=双击
 **************************************************************************/
u8 click_N_Double(u8 time)
{
    static u8 flag_key, count_key, double_key;
    static u16 count_single, Forever_count;

    if(KEY_S == 0)  Forever_count++;   // 长按计时
    else             Forever_count = 0;

    if(0 == KEY_S && 0 == flag_key)    flag_key = 1;

    if(0 == count_key)
    {
        if(flag_key == 1)
        {
            double_key++;
            count_key = 1;
        }
        if(double_key == 2)
        {
            double_key = 0;
            count_single = 0;
            return 2; // 双击执行指令
        }
    }
    if(1 == KEY_S)  flag_key = 0, count_key = 0;

    if(1 == double_key)
    {
        count_single++;
        if(count_single > time && Forever_count < time)
        {
            double_key = 0;
            count_single = 0;
            return 1; // 单击执行指令
        }
        if(Forever_count > time)
        {
            double_key = 0;
            count_single = 0;
        }
    }
    return 0;
}

/**************************************************************************
 * 函数功能: 按键扫描 (带消抖的单击检测)
 * 入口参数: 无
 * 返回值:  0=无动作, 1-5=对应按键
 **************************************************************************/
u8 click(void)
{
    int temp;
    static u8 flag_key = 1; // 按键释放标志

    if(flag_key && (KEY_S==0 || KEY_P==0 || KEY_J==0 || KEY_M==0 || KEY_X==0))
    {
        flag_key = 0; // 已按下

        // 10ms 软件消抖 (机械按键典型抖动<10ms)
        delay_us(10000);

        if(KEY_S == 0)  temp = 1;
        else if(KEY_P == 0)  temp = 2;
        else if(KEY_X == 0)  temp = 3;
        else if(KEY_J == 0)  temp = 4;
        else if(KEY_M == 0)  temp = 5;
        else    temp = 0;

        return temp;
    }
    // 所有按键释放后才允许下次触发
    else if(KEY_S==1 && KEY_P==1 && KEY_J==1 && KEY_M==1 && KEY_X==1)
    {
        flag_key = 1;
    }
    return 0; // 无按键动作
}

/**************************************************************************
 * 函数功能: 长按检测
 * 入口参数: 无
 * 返回值:  0=无动作, 1=长按超过2秒
 **************************************************************************/
u8 Long_Press(void)
{
    static u16 Long_Press_count, Long_Press;

    if(Long_Press == 0 && KEY_S == 0)  Long_Press_count++;
    else                                Long_Press_count = 0;

    if(Long_Press_count > 200)  // 约2秒 (200 × 10ms周期)
    {
        Long_Press = 1;
        Long_Press_count = 0;
        return 1;
    }

    if(Long_Press == 1)  Long_Press = 0; // 松开后清除标志

    return 0;
}
