#include "show.h"

void oled_show(void)
{
    // OpenMV 追踪模式显示 (128x64 OLED, 纯 ASCII 字库, 直接覆写避免闪烁)
    OLED_ShowString(0, 0,  "MODE: OpenMV");

    extern volatile uint32_t OpenMV_Frame_Count;
    OLED_ShowString(0, 10, "F:");
    OLED_ShowNumber(12, 10, OpenMV_Frame_Count, 8, 12);

    extern u8 OpenMV_Target_Lost;
    extern volatile uint8_t OpenMV_Rxbuf[3];
    OLED_ShowString(0, 20, "T:");
    OLED_ShowNumber(12, 20, OpenMV_Target_Lost, 1, 12);
    OLED_ShowNumber(24, 20, OpenMV_Rxbuf[1], 3, 12);
    OLED_ShowNumber(52, 20, OpenMV_Rxbuf[2], 3, 12);

    extern float OpenMV_Error_X, OpenMV_Error_Y;
    OLED_ShowString(0, 30, "EX:");
    OLED_ShowNumber(20, 30, (int)(OpenMV_Error_X * 100), 6, 12);
    OLED_ShowString(70, 30, "EY:");
    OLED_ShowNumber(90, 30, (int)(OpenMV_Error_Y * 100), 6, 12);

    OLED_ShowString(0, 42, "T1:");
    OLED_ShowNumber(20, 42, Target1, 6, 12);
    OLED_ShowString(70, 42, "T2:");
    OLED_ShowNumber(90, 42, Target2, 6, 12);

    OLED_ShowString(0, 54, "L:");
    OLED_ShowNumber(12, 54, OpenMV_Target_Lost, 1, 12);

    OLED_Refresh_Gram();
}
