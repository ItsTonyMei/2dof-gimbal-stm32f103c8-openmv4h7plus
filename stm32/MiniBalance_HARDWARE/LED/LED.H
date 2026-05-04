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
#ifndef __LED_H
#define __LED_H
#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

//LED ?????
#define LED PAout(4)  
void LED_Init(void);  //?????
void Led_Flash(u16 time);

#ifdef __cplusplus
}
#endif
#endif
