#include "sys.h"

// THUMB指令不支持 Thumb-2 PSP
// 实现方式二:使用 WFI 指令 (Wait For Interrupt)
__attribute__((naked)) void WFI_SET(void)
{
	__asm volatile("WFI");
}
// 关闭全局中断 (Disable Global Interrupts)
__attribute__((naked)) void INTX_DISABLE(void)
{
	__asm volatile("CPSID I");
}
// 开启全局中断 (Enable Global Interrupts)
__attribute__((naked)) void INTX_ENABLE(void)
{
	__asm volatile("CPSIE I");
}
// 设置堆栈指针地址 (Set Stack Pointer)
// addr:堆栈地址 (Stack Address)
__attribute__((naked)) void MSR_Msp(u32 addr)
{
	__asm volatile("MSR MSP, r0");
	__asm volatile("BX r14");
}

/**************************************************************************
 * 函数功能 (Function): 设置JTAG模式 (Set JTAG Mode)
 * 入口参数 (Input): mode: jtag,swd模式设置; 00,全使能; 01,使能SWD; 10,全关闭;
 * 返 回 值 (Return): 无 (None)
 **************************************************************************/
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2ENR |= 1<<0;      // 使能APB2时钟 (Enable APB2 Clock)
	AFIO->MAPR &= 0XF8FFFFFF;  // 清除MAPR的[26:24] (Clear MAPR bits)
	AFIO->MAPR |= temp;         // 设置jtag模式 (Set JTAG Mode)
}
