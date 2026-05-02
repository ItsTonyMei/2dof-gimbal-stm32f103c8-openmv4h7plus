/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**************************************************************************
  全局变量定义 - 云台控制系统

  说明: 这些变量在多个文件中使用,定义在main.c但需要extern声明
**************************************************************************/

// 显示控制标志位
u8 Flag_Show = 0;                 // OLED显示标志位: 0=停止显示,1=显示

// PID速度输出 (由Position_PID计算得到,送到舵机PWM)
float Velocity1, Velocity2;       // Velocity1=底舵机速度, Velocity2=摇臂舵机速度

// 舵机当前位置 (PWM脉宽值,非角度值)
// 范围: 底舵机250-1250, 摇臂舵机250-1200
float Position1 = 750;            // 底舵机当前位置 (初始值750=中点)
float Position2 = 750;            // 摇臂舵机当前位置

// 运动速度参数 (控制PID输出幅度,影响响应速度)
float Speed = 10;                 // 舵机运动速度限制值: 3-30

// 电池电压监测
int Voltage;                      // 电池电压(mV),用于低压保护

// 延时与模式切换辅助变量
u8 delay_50, delay_flag;          // 50ms延时计数器与标志
u8 Bi_zhang = 0;                  // 壁障标志(未使用)
u8 PID_Send, Flash_Send;          // PID数据发送,Flash发送辅助

// 目标位置 (期望达到的位置)
float Target1 = 750;              // 底舵机目标位置 (初始值750=中点)
float Target2 = 750;              // 摇臂舵机目标位置

// 位置环PID参数 (比例/积分/微分)
float Position_KP = 1.5;           // 比例系数: 值越大响应越快,但可能震荡
float Position_KI = 0;            // 积分系数: 消除静差,本版本设为0
float Position_KD = 3;            // 微分系数: 抑制震荡,改善动态特性

// PS2手柄数据
int PS2_LX, PS2_LY;               // 左摇杆X/Y (本版本未使用)
int PS2_RX, PS2_RY;               // 右摇杆X/Y (本版本未使用)
int PS2_KEY;                      // 手柄按键值: 5=上,7=下,6=右,8=左等

// 串口接收缓冲区与状态
uint8_t Urxbuf[8];               // 串口接收缓冲,存放一帧数据(8字节)
                                   // 格式: [功能字][数据1][数据2][数据3][数据4][数据5][数据6][校验码]
int Usart_Compelet;               // 串口接收完成标志: 1=接收完成,可解析
volatile uint8_t Pc_Rxbuf[8];     // USART1 PC angle payload after BCC validation
volatile uint8_t OpenMV_Rxbuf[3]; // USART3 OpenMV tracking payload: [hasBlob, tx, ty]
volatile int Pc_Usart_Compelet;   // USART1 valid frame ready
volatile int OpenMV_Usart_Compelet; // USART3 valid frame ready

// 控制模式标志 (核心状态变量)
int Mode_Usart_PS2;               // 0=PS2手柄模式,1=PC串口模式,2=OpenMV视觉模式
int Last_Mode;                    // 上次的控制模式,用于检测模式切换
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	delay_init();
	OLED_Init();
	KEY_Init();
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim2);       // 通用定时器2,以更新中断的方式工作  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	printf("\r\n");
	printf("底舵机角度: %.0f\r\n", (Target1-250)/1000*270);
	printf("摇臂角度: %.0f\r\n", (Target2-250)/1000*180);
	PS2_LX=PS2_AnologData(PSS_LX);
	PS2_LY=PS2_AnologData(PSS_LY);
	PS2_KEY=PS2_DataKey();
	oled_show();          // 更新OLED显示
	Voltage_All+=Get_battery_volt();
	if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;

	// OpenMV调试打印 (每500ms一次)
	{
		static uint32_t dbg_tick = 0;
		if(HAL_GetTick() - dbg_tick >= 500U) {
			dbg_tick = HAL_GetTick();
			printf("[DBG] tx=%d ty=%d ex=%.3f ey=%.3f T1=%.0f T2=%.0f lost=%d\r\n",
				OpenMV_Rxbuf[1], OpenMV_Rxbuf[2],
				(double)OpenMV_Error_X, (double)OpenMV_Error_Y,
				(double)Target1, (double)Target2,
				OpenMV_Target_Lost);
		}
	}

	delay_ms(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

