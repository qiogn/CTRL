/**
  ******************************************************************************
  * @file    stepper_config.h
  * @brief   Stepper motor configuration file
  *          Hardware-specific pin mappings and configuration
  ******************************************************************************
  */

#ifndef __STEPPER_CONFIG_H
#define __STEPPER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Hardware configuration ----------------------------------------------------*/
/* Both motors use TIM3 for hardware PWM step generation:
 *   Motor1 STEP = PA6 (TIM3_CH1), Motor2 STEP = PB1 (TIM3_CH4) */

/* Motor 1 (Vertical Axis) */
#define MOTOR1_ENABLE_PORT      EN1_GPIO_Port
#define MOTOR1_ENABLE_PIN       EN1_Pin
#define MOTOR1_STEP_PORT        Step1_GPIO_Port
#define MOTOR1_STEP_PIN         Step1_Pin
#define MOTOR1_DIR_PORT         Dir1_GPIO_Port
#define MOTOR1_DIR_PIN          Dir1_Pin

/* Motor 2 (Horizontal Axis) */
#define MOTOR2_ENABLE_PORT      EN2_GPIO_Port
#define MOTOR2_ENABLE_PIN       EN2_Pin
#define MOTOR2_STEP_PORT        Step2_GPIO_Port
#define MOTOR2_STEP_PIN         Step2_Pin
#define MOTOR2_DIR_PORT         Dir2_GPIO_Port
#define MOTOR2_DIR_PIN          Dir2_Pin

/* Enable/disable logic levels */
#define STEPPER_ENABLE_LEVEL    GPIO_PIN_RESET  /* Active low */
#define STEPPER_DISABLE_LEVEL   GPIO_PIN_SET

/* Direction logic levels */
#define DIRECTION_FORWARD       GPIO_PIN_RESET
#define DIRECTION_REVERSE       GPIO_PIN_SET

/* Timer configuration -------------------------------------------------------*/
/* TIM3: CH1(PA6)=Motor1 STEP, CH4(PB1)=Motor2 STEP */
#define STEPPER_TIMER_INSTANCE      TIM3
#define STEPPER_TIMER_CLK_ENABLE()  __HAL_RCC_TIM3_CLK_ENABLE()
#define STEPPER_TIMER_IRQn          TIM3_IRQn

/* Timer clock configuration */
/*
 * STM32F1 timer clock behavior:
 * - System clock: 72MHz
 * - APB1 prescaler: DIV2 (36MHz)
 * - When APB1 prescaler != 1: Timer clock = APB1 clock x 2 = 72MHz
 * - Timer prescaler: 71
 * - Timer counter clock: 72MHz / (71+1) = 1MHz
 */
#define TIMER_CLOCK_FREQ_HZ         72000000U  /* APB1 timer clock = 72MHz (APB1x2) */
#define TIMER_PRESCALER             71U        /* 72MHz / (71+1) = 1MHz timer clock */

/* PWM channel assignments */
#include "stm32f1xx_hal_tim.h"
#define MOTOR1_PWM_CHANNEL          TIM_CHANNEL_1  /* PA6: TIM3_CH1 */
#define MOTOR2_PWM_CHANNEL          TIM_CHANNEL_4  /* PB1: TIM3_CH4 */

/* Timer channel aliases for stepper_timer.c */
#define STEPPER_TIMER_CHANNEL_1     MOTOR1_PWM_CHANNEL
#define STEPPER_TIMER_CHANNEL_2     MOTOR2_PWM_CHANNEL

/* Performance limits --------------------------------------------------------*/

/* Speed limits (Hz) */
#define MIN_SPEED_HZ                10U      /* Minimum speed 10Hz */
#define MAX_SPEED_HZ                50000U   /* Maximum speed 50kHz */
#define DEFAULT_SPEED_HZ            1000U    /* Default speed 1kHz */

/* Acceleration limits (Hz/s) */
#define MIN_ACCELERATION_HZ_S       100U     /* Minimum acceleration */
#define MAX_ACCELERATION_HZ_S       100000U  /* Maximum acceleration */
#define DEFAULT_ACCELERATION_HZ_S   10000U   /* Default acceleration 10kHz/s */

/* Deceleration limits (Hz/s) */
#define MIN_DECELERATION_HZ_S       100U     /* Minimum deceleration */
#define MAX_DECELERATION_HZ_S       100000U  /* Maximum deceleration */
#define DEFAULT_DECELERATION_HZ_S   10000U   /* Default deceleration 10kHz/s */

/* Position limits (steps) */
#define MAX_POSITIVE_POSITION       1000000L  /* Maximum positive position */
#define MAX_NEGATIVE_POSITION       -1000000L /* Maximum negative position */

/* Safety settings -----------------------------------------------------------*/

/* Maximum allowed pulse width (microseconds) */
#define MAX_PULSE_WIDTH_US          10000U    /* 10ms maximum pulse width */

/* Minimum allowed pulse width (microseconds) */
#define MIN_PULSE_WIDTH_US          20U       /* 20us minimum pulse width */

/* Direction change settling time (microseconds) */
#define DIR_SETTLING_TIME_US        1300U

/* Thermal protection */
#define MAX_CONTINUOUS_CURRENT_MA   1000U     /* Maximum continuous current */
#define MAX_DUTY_CYCLE              80U       /* Maximum duty cycle (%) */

/* Debug and monitoring ------------------------------------------------------*/

/* Enable debug output */
#define STEPPER_DEBUG_ENABLED       0

/* Performance monitoring */
#define ENABLE_PERFORMANCE_MONITOR  1

/* Error reporting */
#define ENABLE_ERROR_REPORTING      1

/* Utility macros ------------------------------------------------------------*/

#define US_TO_HZ(us)                (1000000U / (us))
#define HZ_TO_US(hz)                (1000000U / (hz))
#define CLAMP(value, min, max)      (((value) < (min)) ? (min) : (((value) > (max)) ? (max) : (value)))
#define ABS(x)                      (((x) < 0) ? -(x) : (x))

/* Configuration validation --------------------------------------------------*/

#if (MIN_SPEED_HZ >= MAX_SPEED_HZ)
#error "MIN_SPEED_HZ must be less than MAX_SPEED_HZ"
#endif

#if (MIN_ACCELERATION_HZ_S >= MAX_ACCELERATION_HZ_S)
#error "MIN_ACCELERATION_HZ_S must be less than MAX_ACCELERATION_HZ_S"
#endif

#if (MIN_DECELERATION_HZ_S >= MAX_DECELERATION_HZ_S)
#error "MIN_DECELERATION_HZ_S must be less than MAX_DECELERATION_HZ_S"
#endif

#if (MAX_PULSE_WIDTH_US <= MIN_PULSE_WIDTH_US)
#error "MAX_PULSE_WIDTH_US must be greater than MIN_PULSE_WIDTH_US"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_CONFIG_H */
