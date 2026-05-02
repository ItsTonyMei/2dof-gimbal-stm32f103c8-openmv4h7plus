/***********************************************
 * 广州万至达科技有限公司
 * 产品名称: WHEELTEC
 * 官网: wheeltec.net
 * 淘宝店: shop114407458.taobao.com
 * 阿里通: https://minibalance.aliexpress.com/store/4455017
 * 版本: V1.0
 * 修改时间: 2022-10-13
 *
 * 注意: 此文件为死代码，实际 USART3 通信在 Core/Src/usart.c 中实现。
 *       此处保留的函数接口用于参考，实际不会被编译。
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

#include "usart3.h"

/**************************************************************************
 * 函数功能: USART3发送单个字节 (死代码)
 * 说明: 实际实现位于 Core/Src/usart.c::usart3_send()
 **************************************************************************/
void usart3_send(u8 data)
{
    USART3->DR = data;
    while((USART3->SR & 0x40) == 0);    // 等待发送完成
}

/**************************************************************************
 * 函数功能: USART3发送角度数据包 (死代码)
 * 说明: 实际实现位于 Core/Src/usart.c::usart3_sendAngleBlock()
 * 协议格式: [0xFF][0xFE][Angle_A][Angle_B][0][0][0][0][0][BCC]
 **************************************************************************/
void usart3_sendAngleBlock(int Angle_A, int Angle_B)
{
    int BlockCheck = 0;

    BlockCheck = Angle_A ^ BlockCheck;
    BlockCheck = Angle_B ^ BlockCheck;    // 异或校验位

    usart3_send(0xFF);       // 帧头
    usart3_send(0xFE);       // 帧头
    usart3_send(Angle_A);    // 云台A角度
    usart3_send(Angle_B);    // 云台B角度
    usart3_send(0);
    usart3_send(0);
    usart3_send(0);
    usart3_send(0);
    usart3_send(0);
    usart3_send(BlockCheck);  // BCC校验位
}
