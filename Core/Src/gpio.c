#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* LED: 开漏，低亮，默认灭 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /* 上轴：PB0 / PB1 / PB10 */
  HAL_GPIO_WritePin(GPIOB, EN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, Step1_Pin | Dir1_Pin, GPIO_PIN_RESET);

  /* 下轴：PA8 / PA9 / PA10 */
  HAL_GPIO_WritePin(GPIOA, EN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, Step2_Pin | Dir2_Pin, GPIO_PIN_RESET);

  /* 激光控制：PB15 */
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);

  /* LED */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* 上轴 GPIO */
  GPIO_InitStruct.Pin = EN1_Pin | Step1_Pin | Dir1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 下轴 GPIO */
  GPIO_InitStruct.Pin = EN2_Pin | Step2_Pin | Dir2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 激光控制 GPIO */
  GPIO_InitStruct.Pin = LASER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LASER_GPIO_Port, &GPIO_InitStruct);
}