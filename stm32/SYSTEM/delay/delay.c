/***********************************************
 * 延时函数 (Delay, SysTick 实现)
 ***********************************************/

#include "delay.h"

static u8  fac_us=0;							//us延时倍乘数 (us Delay Multiplier)
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数

void delay_init(void)
{
	SysTick->CTRL &= ~(1<<2) ;   //配置SysTick使用外部时钟源，是AHB总线时钟的1/8  有 72MHz/8 = 9MHz
	fac_us= 9;                   //SysTick计算一个数需要 1/9MHz 秒 ， 计算9个数则需要 9* 1/9MHz = 1us  ，所以延时函数delay_us传入的数值是“需要多少个1us”,delay_ms同理
	fac_ms=(u16)fac_us*1000;     //1ms = 1000us
}

/**************************************************************************
函数功能 (Function)：微秒延时 (Microsecond Delay)
入口参数 (Input)：nus：要延时的微秒数 (us)
返回  值 (Return)：无 (None)
**************************************************************************/
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us; 								//时间加载 (Load Value)
	SysTick->VAL=0x00;        								//清空计数器 (Clear Counter)
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数 (Start Countdown)
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//等待时间到达 (Wait for Countdown)
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器 (Stop Counter)
	SysTick->VAL =0X00;      					 				//清空计数器 (Clear Counter)
}
/**************************************************************************
函数功能 (Function)：毫秒延时 (Millisecond Delay)
入口参数 (Input)：nms：要延时的毫秒数 (ms)
返回  值 (Return)：无 (None)
**************************************************************************/
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864
void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD=(u32)nms*fac_ms;						//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;												//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//等待时间到达
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       								//清空计数器
} 
