/***********************************************
 * LED 驱动头文件
 ***********************************************/
#ifndef __LED_H
#define __LED_H
#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED PAout(4)
void LED_Init(void);
void Led_Flash(u16 time);

#ifdef __cplusplus
}
#endif
#endif
