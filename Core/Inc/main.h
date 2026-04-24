#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Pin assignments — both motors use TIM3 for hardware PWM step generation:
 *   Motor1 STEP = PA6 (TIM3_CH1), Motor2 STEP = PB1 (TIM3_CH4) */

/* LED */
#define LED_Pin         GPIO_PIN_13
#define LED_GPIO_Port   GPIOC

/* Motor 1 (Vertical Axis): EN=PA1, STEP=PA6, DIR=PA0 */
#define EN1_Pin         GPIO_PIN_1
#define EN1_GPIO_Port   GPIOA

#define Step1_Pin       GPIO_PIN_6
#define Step1_GPIO_Port GPIOA

#define Dir1_Pin        GPIO_PIN_0
#define Dir1_GPIO_Port  GPIOA

/* Motor 2 (Horizontal Axis): EN=PB10, STEP=PB1, DIR=PB0 */
#define EN2_Pin         GPIO_PIN_10
#define EN2_GPIO_Port   GPIOB

#define Step2_Pin       GPIO_PIN_1
#define Step2_GPIO_Port GPIOB

#define Dir2_Pin        GPIO_PIN_0
#define Dir2_GPIO_Port  GPIOB

/* Laser control: PB15 */
#define LASER_Pin       GPIO_PIN_15
#define LASER_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
