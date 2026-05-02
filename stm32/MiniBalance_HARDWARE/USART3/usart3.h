/***********************************************
 * 广州万至达科技有限公司
 * 产品名称: WHEELTEC
 * 官网: wheeltec.net
 * 淘宝店: shop114407458.taobao.com
 * 阿里通: https://minibalance.aliexpress.com/store/4455017
 * 版本: V1.0
 * 修改时间: 2022-10-13
 *
 * Brand: WHEELTEC
 * Website: wheeltec.net
 * Taobao shop: shop114407458.taobao.com
 * Aliexpress: https://minibalance.aliexpress.com/store/4455017
 * Version: V1.0
 * Update: 2022-10-13
 *
 * All rights reserved
 ***********************************************/
#ifndef __USRAT3_H
#define __USRAT3_H
#include "sys.h"

void usart3_send(u8 data);
void usart3_sendAngleBlock(int Angle_A, int Angle_B);
#endif
