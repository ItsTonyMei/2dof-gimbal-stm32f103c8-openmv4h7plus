#include "show.h"

void oled_show(void)
{
    // OpenMV 追踪模式显示 (Tracking Display, 128x64 OLED, 纯 ASCII 字库, 直接覆写避免闪烁)
    OLED_ShowString(0, 0,  "MODE: OpenMV");

    extern volatile uint32_t OpenMV_Frame_Count;
    static uint32_t last_fc = 0, last_tick = 0, fps = 0;
    uint32_t now = HAL_GetTick();
    if(now - last_tick >= 1000U) {
        last_tick = now;
        fps = OpenMV_Frame_Count - last_fc;
        last_fc = OpenMV_Frame_Count;
    }
    OLED_ShowString(0, 10, "FPS:");
    OLED_ShowNumber(28, 10, fps, 3, 12);

    extern u8 OpenMV_Target_Lost;
    extern volatile uint8_t OpenMV_Rxbuf[3];
    OLED_ShowString(0, 20, "T:");
    OLED_ShowNumber(12, 20, OpenMV_Target_Lost, 1, 12);
    OLED_ShowNumber(24, 20, OpenMV_Rxbuf[1], 3, 12);
    OLED_ShowNumber(52, 20, OpenMV_Rxbuf[2], 3, 12);

    extern float OpenMV_Error_X, OpenMV_Error_Y;
    int ex_raw = (int)(OpenMV_Error_X * 100);
    int ey_raw = (int)(OpenMV_Error_Y * 100);
    OLED_ShowString(0, 30, "EX:");
    if(ex_raw < 0) { OLED_ShowString(20, 30, "-"); OLED_ShowNumber(28, 30, -ex_raw, 5, 12); }
    else          { OLED_ShowString(20, 30, "+"); OLED_ShowNumber(28, 30,  ex_raw, 5, 12); }
    OLED_ShowString(70, 30, "EY:");
    if(ey_raw < 0) { OLED_ShowString(90, 30, "-"); OLED_ShowNumber(98, 30, -ey_raw, 5, 12); }
    else          { OLED_ShowString(90, 30, "+"); OLED_ShowNumber(98, 30,  ey_raw, 5, 12); }

    OLED_ShowString(0, 42, "T1:");
    OLED_ShowNumber(20, 42, Target1, 6, 12);
    OLED_ShowString(70, 42, "T2:");
    OLED_ShowNumber(90, 42, Target2, 6, 12);

    OLED_ShowString(0, 54, "L:");
    OLED_ShowNumber(12, 54, OpenMV_Target_Lost, 1, 12);

    OLED_Refresh_Gram();
}
