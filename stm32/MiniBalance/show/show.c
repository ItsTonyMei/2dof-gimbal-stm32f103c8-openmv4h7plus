/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-10-13

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2022-10-13

All rights reserved
***********************************************/
#include "show.h"

unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
    if (Mode_Usart_PS2 == 2)
    {
        // ================================================================
        // OpenMV 模式 (Mode 2) - 6行布局，与PS2/UART模式保持一致
        OLED_Clear();  // 清除残留的Mode 0/1显示内容
        // ================================================================
        // OpenMV模式 (Mode 2) - 6行布局
        // y=0,10,20,30,40,50 全部在128x64 OLED可视范围内
        // ================================================================
        OLED_ShowString(0, 0,  "Mode:");
        OLED_ShowString(65, 0, "OpenMV");

        extern volatile uint32_t OpenMV_Frame_Count;
        OLED_ShowString(0, 10, "Frm:");
        OLED_ShowNumber(65, 10, OpenMV_Frame_Count, 8, 12);

        extern u8 OpenMV_Target_Lost;
        extern volatile uint8_t OpenMV_Rxbuf[3];
        OLED_ShowString(0, 20, "Det:");
        OLED_ShowNumber(65, 20, OpenMV_Target_Lost, 1, 12);  // 0=检测到目标,1=目标丢失
        OLED_ShowNumber(75, 20, OpenMV_Rxbuf[1], 3, 12);    // 原始target_x
        OLED_ShowNumber(100, 20, OpenMV_Rxbuf[2], 3, 12);   // 原始target_y

        extern float OpenMV_Error_X, OpenMV_Error_Y;
        // EX/EY 标签行：完全覆盖 Mode 0/1 的 "Target" (7字符→6字符) 和 "Position" 残留
        OLED_ShowString(0,  30, "EX    ");
        OLED_ShowString(60, 30, "EY    ");

        // 对应 Target1/Position1 → OpenMV_Error_X / Target1
        OLED_ShowString(0,  40, "+");
        OLED_ShowNumber(15, 40, (int)(OpenMV_Error_X * 100), 6, 12);
        OLED_ShowString(60, 40, "+");
        OLED_ShowNumber(75, 40, Target1, 6, 12);

        // 对应 Target2/Position2 → OpenMV_Error_Y / Target2
        OLED_ShowString(0,  50, "+");
        OLED_ShowNumber(15, 50, (int)(OpenMV_Error_Y * 100), 6, 12);
        OLED_ShowString(60, 50, "+");
        OLED_ShowNumber(75, 50, Target2, 6, 12);

        OLED_Refresh_Gram();
        return;
    }

    // ================================================================
    // PS2 / UART 模式 (Mode 0/1) - 原有6行布局
    // ================================================================
    OLED_ShowString(0, 0,  "Mode:");
    if (Mode_Usart_PS2 == 0) OLED_ShowString(65, 0, "PS2   ");
    else                     OLED_ShowString(65, 0, "UART  ");

    OLED_ShowString(0, 10, "PS2KEY:");
    OLED_ShowNumber(65, 10, PS2_KEY, 2, 12);

    OLED_ShowString(0, 20, "Voltage:");
    OLED_ShowNumber(65, 20, Voltage / 100, 2, 12);
    OLED_ShowString(88, 20, ".");
    OLED_ShowNumber(100, 20, Voltage % 100, 2, 12);
    if (Voltage % 100 < 10) OLED_ShowNumber(82, 20, 0, 2, 12);
    OLED_ShowString(118, 20, "V");

    OLED_ShowString(0, 30, "Target");
    OLED_ShowString(60, 30, "Position");

    OLED_ShowString(0,  40, "+");
    OLED_ShowNumber(15, 40, Target1, 6, 12);
    OLED_ShowString(60, 40, "+");
    OLED_ShowNumber(75, 40, Position1, 6, 12);

    OLED_ShowString(0,  50, "+");
    OLED_ShowNumber(15, 50, Target2, 6, 12);
    OLED_ShowString(60, 50, "+");
    OLED_ShowNumber(75, 50, Position2, 6, 12);

    OLED_Refresh_Gram();
}
/**************************************************************************
�������ܣ���APP��������
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void APP_Show(void)
{    
		static u8 flag;
	  int app_2,app_3,app_4;
		app_4=(Voltage-1110)*2/3;		if(app_4<0)app_4=0;if(app_4>100)app_4=100;   //�Ե�ѹ���ݽ��д���
//    app_3=Moto1/50; if(app_3<0)app_3=-app_3;			                           //�Ա��������ݾ������ݴ�������ͼ�λ�
//		app_2=Moto2/50;  if(app_2<0)app_2=-app_2;
	  flag=!flag;
   if(flag==0)// 
   printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,(u8)app_4,0);//��ӡ��APP����
	 else
	 printf("{B%d:%d}$",(int)Position1,(int)Position2);//��ӡ��APP���� ��ʾ����
}

