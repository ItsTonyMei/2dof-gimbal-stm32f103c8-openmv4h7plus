/***********************************************
 * 延时函数头文件 (SysTick 实现)
 ***********************************************/
#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  

void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif
















