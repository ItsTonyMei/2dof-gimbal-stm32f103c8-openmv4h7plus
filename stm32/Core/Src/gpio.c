#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Output pin initial levels
    HAL_GPIO_WritePin(GPIOA, LED_Pin | OLED_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, OLED_RTC_Pin | OLED_SDA_Pin | OLED_SCL_Pin, GPIO_PIN_RESET);

    // LED (PA4) - output
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // OLED DC (PA15) - output
    GPIO_InitStruct.Pin = OLED_DC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);

    // OLED control (PB3=RST, PB4=SDA, PB5=SCL)
    GPIO_InitStruct.Pin = OLED_RTC_Pin | OLED_SDA_Pin | OLED_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
