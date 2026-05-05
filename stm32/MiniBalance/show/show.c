#include "show.h"

void oled_show(void)
{
    // OpenMV 追踪模式显示 (128x64 OLED, 6行布局)
    OLED_Clear();

    OLED_ShowString(0, 0,  "模式:");
    OLED_ShowString(65, 0, "OpenMV");

    extern volatile uint32_t OpenMV_Frame_Count;
    OLED_ShowString(0, 10, "帧:");
    OLED_ShowNumber(65, 10, OpenMV_Frame_Count, 8, 12);

    extern u8 OpenMV_Target_Lost;
    extern volatile uint8_t OpenMV_Rxbuf[3];
    OLED_ShowString(0, 20, "检测:");
    OLED_ShowNumber(65, 20, OpenMV_Target_Lost, 1, 12);
    OLED_ShowNumber(75, 20, OpenMV_Rxbuf[1], 3, 12);
    OLED_ShowNumber(100, 20, OpenMV_Rxbuf[2], 3, 12);

    extern float OpenMV_Error_X, OpenMV_Error_Y;
    OLED_ShowString(0,  30, "误差X ");
    OLED_ShowString(60, 30, "误差Y ");

    OLED_ShowString(0,  40, "+");
    OLED_ShowNumber(15, 40, (int)(OpenMV_Error_X * 100), 6, 12);
    OLED_ShowString(60, 40, "+");
    OLED_ShowNumber(75, 40, Target1, 6, 12);

    OLED_ShowString(0,  50, "+");
    OLED_ShowNumber(15, 50, (int)(OpenMV_Error_Y * 100), 6, 12);
    OLED_ShowString(60, 50, "+");
    OLED_ShowNumber(75, 50, Target2, 6, 12);

    OLED_Refresh_Gram();
}
