#include "usart.h"

// ---- printf 重定向到 USART1 ----
#ifdef __ARMCC_VERSION
  #if (__ARMCC_VERSION < 6000000)
  #pragma import(__use_no_semihosting)
  #endif
  #if !defined(FILE)
  typedef struct __FILE FILE;
  #endif
  #ifndef __stdout
  FILE __stdout;
  #define __stdout __stdout
  #endif
#endif

void _sys_exit(int x)
{
    x = x;
    while(1);
}

int fputc(int ch, FILE *f)
{
    (void)f;
    while((USART1->SR & 0x40) == 0);
    USART1->DR = (u8)ch;
    return ch;
}

u8 Usart3_Receive_buf[1];
u8 Usart1_Receive_buf[1];
volatile uint32_t OpenMV_Frame_Count = 0;
volatile uint32_t OpenMV_Error_ORE = 0;
volatile uint32_t OpenMV_Error_FE  = 0;
volatile uint32_t OpenMV_Error_NE  = 0;
volatile uint32_t OpenMV_Error_PE  = 0;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

// ---- USART1 初始化 (仅用于调试 printf) ----
void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
    HAL_UART_Receive_IT(&huart1, Usart1_Receive_buf, sizeof(Usart1_Receive_buf));
}

// ---- USART3 初始化 (OpenMV 协议) ----
void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart3) != HAL_OK)
        Error_Handler();
    HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf));
}

// ---- MSP 初始化 ----
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if(uartHandle->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
    if(uartHandle->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
    else if(uartHandle->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
}

// ---- OpenMV 5字节协议帧解析 (仅 USART3) ----
// 帧格式: [0xFF][0xFE][hasBlob][tx][ty]
#define OMV_PAYLOAD_LEN 3U

typedef struct {
    u8 count;
    u8 last_data;
    u8 last_last_data;
    u8 head_received;
    u8 payload[OMV_PAYLOAD_LEN];
} OmvRxState;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3)
    {
        u8 temp = Usart3_Receive_buf[0];
        static OmvRxState st = {0};

        if(st.head_received == 0)
        {
            if(st.last_data == 0xFE && st.last_last_data == 0xFF)
            {
                st.head_received = 1;
                st.count = 0;
            }
        }

        if(st.head_received == 1)
        {
            if(st.count >= OMV_PAYLOAD_LEN)
            {
                st.head_received = 0;
                st.count = 0;
            }
            else
            {
                st.payload[st.count] = temp;
                st.count++;
            }

            if(st.count == OMV_PAYLOAD_LEN)
            {
                OpenMV_Rxbuf[0] = st.payload[0];
                OpenMV_Rxbuf[1] = st.payload[1];
                OpenMV_Rxbuf[2] = st.payload[2];
                OpenMV_Usart_Compelet = 1;
                OpenMV_Frame_Count++;
                st.head_received = 0;
                st.count = 0;
            }
        }

        st.last_last_data = st.last_data;
        st.last_data = temp;
        HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf));
    }
    else if(huart == &huart1)
    {
        // USART1: 不做协议解析, 仅清空接收防止溢出
        HAL_UART_Receive_IT(&huart1, Usart1_Receive_buf, sizeof(Usart1_Receive_buf));
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        if(huart->ErrorCode & HAL_UART_ERROR_ORE)
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            HAL_UART_Receive_IT(&huart1, Usart1_Receive_buf, sizeof(Usart1_Receive_buf));
        }
    }
    else if(huart == &huart3)
    {
        uint32_t err = huart->ErrorCode;
        if(err & HAL_UART_ERROR_ORE) {
            __HAL_UART_CLEAR_OREFLAG(huart);
            OpenMV_Error_ORE++;
        }
        if(err & HAL_UART_ERROR_FE)  {
            __HAL_UART_CLEAR_FEFLAG(huart);
            OpenMV_Error_FE++;
        }
        if(err & HAL_UART_ERROR_NE)  {
            __HAL_UART_CLEAR_NEFLAG(huart);
            OpenMV_Error_NE++;
        }
        if(err & HAL_UART_ERROR_PE)  {
            __HAL_UART_CLEAR_PEFLAG(huart);
            OpenMV_Error_PE++;
        }
        HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf));
    }
}
