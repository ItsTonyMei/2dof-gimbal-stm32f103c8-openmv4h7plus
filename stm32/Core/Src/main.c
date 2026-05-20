#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sys.h"
#include "control.h"

// 调试开关 (Debug Switch): 编译时定义 DEBUG_PRINTF 以启用串口调试输出 (UART Debug Output)
// (在 build_gcc.sh 中添加 -DDEBUG_PRINTF)

u8 Flag_Show = 0;

float Velocity1, Velocity2;
float Position1 = 750;
float Position2 = 750;
float Target1 = 750;
float Target2 = 750;

int Voltage;
u8 delay_50, delay_flag;

volatile uint8_t OpenMV_Rxbuf[3];
volatile int OpenMV_Usart_Compelet;

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_TIM4_Init();
    MX_TIM2_Init();
    MX_USART3_UART_Init();

    OLED_Init();

    // 独立看门狗 (IWDG): LSI 40kHz / 64 = 625 Hz, 重装载 (Reload) 62 → ~100ms 超时 (Timeout)
    // TIM2 回调 (Callback) 每 10ms 喂狗 (Kick), 连续 10 次中断丢失则复位 (Reset)
    IWDG->KR = 0x5555;
    IWDG->PR = 0x04;
    IWDG->RLR = 62;
    IWDG->KR = 0xCCCC;
    IWDG->KR = 0xAAAA;

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim2);

    while(1)
    {
        oled_show();

        Voltage_All += Get_battery_volt();
        if(++Voltage_Count == BATTERY_SAMPLE_COUNT)
            Voltage = Voltage_All / BATTERY_SAMPLE_COUNT, Voltage_All = 0, Voltage_Count = 0;

#ifdef DEBUG_PRINTF
        {
            static uint32_t dbg_tick = 0;
            if(HAL_GetTick() - dbg_tick >= DEBUG_PRINTF_INTERVAL_MS) {
                dbg_tick = HAL_GetTick();
                uint8_t rx1, rx2;
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                rx1 = OpenMV_Rxbuf[1];
                rx2 = OpenMV_Rxbuf[2];
                if(primask == 0U) __enable_irq();
                printf("[调试] tx=%d ty=%d ex=%.3f ey=%.3f T1=%.0f T2=%.0f lost=%d\r\n",
                    rx1, rx2,
                    (double)OpenMV_Error_X, (double)OpenMV_Error_Y,
                    (double)Target1, (double)Target2,
                    OpenMV_Target_Lost);
            }
        }
#endif
        HAL_Delay(5);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
