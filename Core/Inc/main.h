#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

/* LED */
#define LED_Pin         GPIO_PIN_13
#define LED_GPIO_Port   GPIOC

/* 上轴：EN=PB10, STEP=PB1, DIR=PB0 */
#define EN1_Pin         GPIO_PIN_10
#define EN1_GPIO_Port   GPIOB

#define Step1_Pin       GPIO_PIN_1
#define Step1_GPIO_Port GPIOB

#define Dir1_Pin        GPIO_PIN_0
#define Dir1_GPIO_Port  GPIOB

/* 激光控制：PB15 */
#define LASER_Pin       GPIO_PIN_15
#define LASER_GPIO_Port GPIOB

/* 下轴：EN=PA8, STEP=PA9, DIR=PA10 */
#define EN2_Pin         GPIO_PIN_8
#define EN2_GPIO_Port   GPIOA

#define Step2_Pin       GPIO_PIN_9
#define Step2_GPIO_Port GPIOA

#define Dir2_Pin        GPIO_PIN_10
#define Dir2_GPIO_Port  GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
