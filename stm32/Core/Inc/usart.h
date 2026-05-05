#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sys.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern volatile uint32_t OpenMV_Frame_Count;
extern volatile uint32_t OpenMV_Error_ORE;
extern volatile uint32_t OpenMV_Error_FE;
extern volatile uint32_t OpenMV_Error_NE;
extern volatile uint32_t OpenMV_Error_PE;

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif
