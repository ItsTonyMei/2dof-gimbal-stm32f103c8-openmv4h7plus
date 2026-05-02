/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#if SYSTEM_SUPPORT_OS
#include "includes.h"				// uCOS 使用
#endif
//////////////////////////////////////////////////////////////////
//重定向此文件,支持printf函数,不需要勾选use MicroLIB
#if 1
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6000000)
#pragma import(__use_no_semihosting)
#endif
//标准库需要的支持函数
// ARM/Keil stdio.h defines FILE type, so we don't redefine it
#if !defined(FILE)
typedef struct __FILE FILE;
#endif

#ifndef __stdout
FILE __stdout;
#define __stdout __stdout
#endif
//定义 _sys_exit() 以避免使用半主机模式
_sys_exit(int x)
{
	x = x;
	while(1);  // never returns — halts CPU on semihosting exit
}
//重定向 fputc 函数
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);	// 等待发送完成
	USART1->DR = (u8) ch;
	return ch;
}
#endif
u8 Usart3_Receive_buf[1];          // 串口3单字节中断接收缓冲
u8 Usart1_Receive_buf[1];          // 串口1单字节中断接收缓冲
volatile uint32_t OpenMV_Frame_Count = 0;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1,Usart1_Receive_buf,sizeof(Usart1_Receive_buf)); //启动串口1接收中断
  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf)); //启动串口3接收中断
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**************************************************************************
 * 函数功能: USART1发送单个字节
 * 入口参数: data-要发送的数据
 * 返 回 值: 无
 **************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	// 等待发送完成
}

/**************************************************************************
 * 函数功能: USART1发送角度数据包
 * 入口参数: Angle_A-A角度, Angle_B-B角度
 * 返 回 值: 无
 **************************************************************************/
void usart1_sendAngleBlock(int Angle_A, int Angle_B)
{
	int BlockCheck = 0;

	BlockCheck = Angle_A ^ BlockCheck;
	BlockCheck = Angle_B ^ BlockCheck; // 用于校验位

	usart1_send(0xff);       // 帧头
	usart1_send(0xfe);       // 帧头
	usart1_send(Angle_A);    // 云台A角度
	usart1_send(Angle_B);    // 云台B角度
	usart1_send(0);
	usart1_send(0);
	usart1_send(0);
	usart1_send(0);
	usart1_send(0);
	usart1_send(BlockCheck);    // BCC校验位
}
/**************************************************************************
 * 函数功能: USART3发送单个字节
 * 入口参数: data-要发送的数据
 * 返 回 值: 无
 **************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	// 等待发送完成
}
/**************************************************************************
 * 函数功能: USART3发送角度数据包
 * 入口参数: Angle_A-A角度, Angle_B-B角度
 * 返 回 值: 无
 **************************************************************************/
void usart3_sendAngleBlock(int Angle_A, int Angle_B)
{
	int BlockCheck = 0;

	BlockCheck = Angle_A ^ BlockCheck;
	BlockCheck = Angle_B ^ BlockCheck; // 用于校验位

	usart3_send(0xff);       // 帧头
	usart3_send(0xfe);       // 帧头
	usart3_send(Angle_A);    // 云台A角度
	usart3_send(Angle_B);    // 云台B角度
	usart3_send(0);
	usart3_send(0);
	usart3_send(0);
	usart3_send(0);
	usart3_send(0);
	usart3_send(BlockCheck);    // BCC校验位
}

// ============================================================
// USART3 OpenMV 视觉协议 (5字节帧)
// 帧格式: [0xFF][0xFE][hasBlob][tx][ty]
//   hasBlob: 0x01=检测到目标, 0x00=未检测
//   tx/ty:   归一化坐标 0-255, 127=中心 (与OpenMV侧IMAGE_CENTER对齐)
// ============================================================
typedef struct {
    u8 count;            // 已接收payload字节数 (0-3)
    u8 last_data;         // 上一个接收字节
    u8 last_last_data;    // 上上一个接收字节
    u8 head_received;     // 帧头已锁定标志
    u8 payload[3];        // [hasBlob, tx, ty]
} UART_RxState;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
{
    if(huart == &huart3)
    {
        u8 temp;
        static UART_RxState usart3_state = {0};

        temp = Usart3_Receive_buf[0];

        // 双重帧头检测: 0xFF 0xFE
        if(usart3_state.head_received == 0)
        {
            if(usart3_state.last_data == 0xFE && usart3_state.last_last_data == 0xFF)
            {
                usart3_state.head_received = 1;
                usart3_state.count = 0;
            }
        }

        // 接收 payload (hasBlob + tx + ty = 3字节)
        if(usart3_state.head_received == 1)
        {
            // 安全检查: count必须在有效范围0-2
            if(usart3_state.count >= 3)
            {
                // 超出范围,重置状态机
                usart3_state.head_received = 0;
                usart3_state.count = 0;
            }
            else
            {
                usart3_state.payload[usart3_state.count] = temp;
                usart3_state.count++;
            }

            // 收齐3字节后写入全局缓冲区，唤醒控制任务
            if(usart3_state.count == 3)
            {
                // OpenMV_Rxbuf[0]=hasBlob, [1]=tx, [2]=ty
                OpenMV_Rxbuf[0] = usart3_state.payload[0];
                OpenMV_Rxbuf[1] = usart3_state.payload[1];
                OpenMV_Rxbuf[2] = usart3_state.payload[2];
                OpenMV_Usart_Compelet = 1;
                OpenMV_Frame_Count++;

                usart3_state.head_received = 0;
                usart3_state.count = 0;
            }
        }

        usart3_state.last_last_data = usart3_state.last_data;
        usart3_state.last_data = temp;

        // 重新启动下一次接收中断
        HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf));
    }
	else if(huart == &huart1)
	{
		u8 temp;
		u8 i;
		u8 check;
		static UART_RxState usart1_state = {0};

		temp=Usart1_Receive_buf[0];
		if(usart1_state.head_received==0)
		{
			if(usart1_state.last_data==0xfe&&usart1_state.last_last_data==0xff)
			{
				usart1_state.head_received=1;
				usart1_state.count=0;
			}
		}
		if(usart1_state.head_received==1)
		{
			// 安全检查: count必须在有效范围0-8
			if(usart1_state.count >= 9)
			{
				usart1_state.head_received = 0;
				usart1_state.count = 0;
			}
			else
			{
				usart1_state.payload[usart1_state.count] = temp;
				usart1_state.count++;
			}
			if(usart1_state.count==9)   // 等待收到完整的9字节后再进行BCC校验（10字节帧：header 2 + 数据8）
			{
				check=0;
				for(i=0; i<8; i++)
				{
					check ^= usart1_state.payload[i];
				}
				if(check==usart1_state.payload[8])   // payload[8]是BCC校验字节
				{
					for(i=0; i<8; i++)
					{
						Pc_Rxbuf[i]=usart1_state.payload[i];
					}
					Pc_Usart_Compelet=1;
				}
				usart1_state.head_received=0;
				usart1_state.count=0;
			}
		}
		usart1_state.last_last_data=usart1_state.last_data;
		usart1_state.last_data=temp;
		HAL_UART_Receive_IT(&huart1,Usart1_Receive_buf,sizeof(Usart1_Receive_buf));
	}
}



// UART overrun recovery
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if( huart==&huart1 )
	{
		//串口1溢出错误处理
		if ( huart->ErrorCode & HAL_UART_ERROR_ORE  )
		{
			__HAL_UART_CLEAR_OREFLAG(huart);
			HAL_UART_Receive_IT(&huart1,Usart1_Receive_buf,sizeof(Usart1_Receive_buf));
		}
	}
	else if( huart==&huart3 )
	{
		//串口3错误处理: 溢出/帧错误/噪声/校验错误
		if ( huart->ErrorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE | HAL_UART_ERROR_NE | HAL_UART_ERROR_PE) )
		{
			// 清除所有错误标志
			__HAL_UART_CLEAR_OREFLAG(huart);
			__HAL_UART_CLEAR_FEFLAG(huart);
			__HAL_UART_CLEAR_NEFLAG(huart);
			__HAL_UART_CLEAR_PEFLAG(huart);
			// 重新启动接收,下一帧0xFF 0xFE会自动同步
			HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));
		}
	}
}
/* USER CODE END 1 */
