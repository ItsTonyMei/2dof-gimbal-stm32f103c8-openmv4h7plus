/***********************************************
公司：深圳市趣玩电子有限公司
产品：WHEELTEC
网站：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
阿里店: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-10-13

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-10-13

All rights reserved
***********************************************/
#include "pstwo.h"

#define DELAY_TIME               delay_us(5);
#define PS2_ANALOG_MODE_RESPONSE 0x73 
u16 Handkey;	// 按键键值读取，临时存储变量
u8 Comd[2]={0x01,0x42};	// 起始命令。发送这两个字节
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // 数据存储数组
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};
	// 按键值对应表

// PS2手柄接口初始化   
void PS2_Init(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//  
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB,&GPIO_InitStructure);

//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB,&GPIO_InitStructure);	
}

// 向手柄发送命令
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   // 输出一位，数据位置位
		}
		else DO_L;

		CLK_H;                        // 时钟线上升沿
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}
// 判断是否为模拟模式,0x41=模拟手柄，0x73=模拟手柄
// 返回值为0代表模拟模式
//	           返回值为1代表数字模式
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  // 开始命令
	PS2_Cmd(Comd[1]);  // 发送数据
	CS_H;
	if( Data[1] == PS2_ANALOG_MODE_RESPONSE)   return 0;
	else return 1;

}
// 读取手柄数据
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  // 开始命令
	PS2_Cmd(Comd[1]);  // 发送数据
	for(byte=2;byte<9;byte++)          // 开始接收数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}

// 自动填充PS2手柄数据接收,只供一个遥控器使用  
// 只有一个按键按下时键值为0，未按下为1
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     // 组成16位数据，按键为0， 未按键为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          // 没有任何按键按下则返回
}

// 得到摇杆的模拟量	  范围0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

// 清除数据缓冲区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动 0x00关，其他值开
		   motor2:左侧大震动 0x40~0xFF 数值越大震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  // 开始命令
	PS2_Cmd(0x42);  // 发送数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
// short poll
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
// 进入配置
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
// 发送模式配置
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00); //analog=0x01;digital=0x00  选择手柄模式类型
	PS2_Cmd(0x03); //0x03：红石手柄配置，模拟手柄通过MODE键切换模式；
				         //0xEE：配置后不保存，模拟手柄通过MODE键切换模式；
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
// 震动设置
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
// 完成配置并初始化
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
// 手柄发出初始化
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		// 进入配置模式
	PS2_TurnOnAnalogMode();	// 开启摇杆、模拟模式，并选择是否保存
	//PS2_VibrationMode();	// 开启震动模式
	PS2_ExitConfing();		// 完成配置初始化
}

