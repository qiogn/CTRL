/**
  ******************************************************************************
  * @file    stepper_timer.h
  * @brief   Hardware timer PWM driver for dual stepper motor control
  *          Uses TIM3 CH1(PA6) + CH4(PB1) for precise step pulses
  ******************************************************************************
  */

#ifndef __STEPPER_TIMER_H
#define __STEPPER_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Channel identifiers */
#define STEPPER_CH_MOTOR1  0U
#define STEPPER_CH_MOTOR2  1U
#define STEPPER_CH_BOTH    0xFFU

/* Initialise TIM3 dual-channel PWM (call once during startup) */
void StepperTimer_Init(void);

/* Start N step pulses on the given channel (non-blocking).
 * freq_hz  : pulse frequency (steps per second)
 * steps    : number of steps to generate            */
void StepperTimer_Start(uint8_t channel, uint32_t freq_hz, uint32_t steps);

/* Stop pulses on the given channel immediately */
void StepperTimer_Stop(uint8_t channel);

/* Return 1 if the given channel is still generating pulses */
uint8_t StepperTimer_IsBusy(uint8_t channel);

/* Interrupt handler — call from TIM3_IRQHandler */
void StepperTimer_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_TIMER_H */
