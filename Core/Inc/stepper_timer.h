/**
  ******************************************************************************
  * @file    stepper_timer.h
  * @brief   Hardware timer PWM driver for stepper motor control
  *          Uses TIM3 for generating precise step pulses
  ******************************************************************************
  */

#ifndef __STEPPER_TIMER_H
#define __STEPPER_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stepper_config.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Stepper timer status enumeration
  */
typedef enum {
    STEPPER_TIMER_OK = 0,
    STEPPER_TIMER_ERROR,
    STEPPER_TIMER_BUSY,
    STEPPER_TIMER_INVALID_PARAM,
    STEPPER_TIMER_NOT_INITIALIZED
} StepperTimer_Status_t;

/**
  * @brief  Stepper motor axis enumeration
  */
typedef enum {
    STEPPER_AXIS_1 = 0,    /* Motor 1 (vertical axis) */
    STEPPER_AXIS_2,        /* Motor 2 (horizontal axis) */
    STEPPER_AXIS_BOTH      /* Both motors */
} Stepper_Axis_t;

/**
  * @brief  Stepper timer configuration structure
  */
typedef struct {
    uint32_t timer_frequency_hz;      /* Timer clock frequency in Hz */
    uint32_t min_pulse_frequency_hz;  /* Minimum pulse frequency (10Hz) */
    uint32_t max_pulse_frequency_hz;  /* Maximum pulse frequency (50kHz) */
    uint32_t default_frequency_hz;    /* Default frequency (1kHz) */
    uint8_t  pulse_duty_cycle;        /* Pulse duty cycle (50%) */
} StepperTimer_Config_t;

/**
  * @brief  Stepper timer handle structure
  */
typedef struct {
    TIM_HandleTypeDef* htim;          /* Timer handle */
    StepperTimer_Config_t config;     /* Timer configuration */
    volatile uint8_t is_initialized;  /* Initialization flag */
    volatile uint8_t is_running;      /* Running flag */
    volatile uint32_t current_freq_hz;/* Current frequency */
} StepperTimer_Handle_t;

/* Exported constants --------------------------------------------------------*/

/* Default configuration values */
#define STEPPER_TIMER_DEFAULT_FREQ_HZ     DEFAULT_SPEED_HZ    /* Default frequency */
#define STEPPER_TIMER_MIN_FREQ_HZ         MIN_SPEED_HZ        /* Minimum frequency */
#define STEPPER_TIMER_MAX_FREQ_HZ         MAX_SPEED_HZ        /* Maximum frequency */
#define STEPPER_TIMER_PULSE_DUTY_CYCLE    50U                 /* 50% duty cycle */

/* Exported functions --------------------------------------------------------*/

/* Initialization and configuration */
StepperTimer_Status_t StepperTimer_Init(void);
StepperTimer_Status_t StepperTimer_DeInit(void);
StepperTimer_Status_t StepperTimer_GetConfig(StepperTimer_Config_t* config);
StepperTimer_Status_t StepperTimer_SetConfig(const StepperTimer_Config_t* config);

/* Frequency control */
StepperTimer_Status_t StepperTimer_SetFrequency(Stepper_Axis_t axis, uint32_t frequency_hz);
StepperTimer_Status_t StepperTimer_GetFrequency(Stepper_Axis_t axis, uint32_t* frequency_hz);
StepperTimer_Status_t StepperTimer_SetFrequencyBoth(uint32_t freq_motor1_hz, uint32_t freq_motor2_hz);

/* Pulse generation control */
StepperTimer_Status_t StepperTimer_Start(Stepper_Axis_t axis);
StepperTimer_Status_t StepperTimer_Stop(Stepper_Axis_t axis);
StepperTimer_Status_t StepperTimer_StartBoth(void);
StepperTimer_Status_t StepperTimer_StopBoth(void);

/* Status and information */
StepperTimer_Status_t StepperTimer_GetStatus(Stepper_Axis_t axis, uint8_t* is_running);
StepperTimer_Status_t StepperTimer_IsInitialized(uint8_t* is_initialized);
StepperTimer_Status_t StepperTimer_GetErrorCount(uint32_t* error_count);

/* Utility functions */
StepperTimer_Status_t StepperTimer_GenerateSinglePulse(Stepper_Axis_t axis);
StepperTimer_Status_t StepperTimer_SetPulseCount(Stepper_Axis_t axis, uint32_t pulse_count);
StepperTimer_Status_t StepperTimer_GetRemainingPulses(Stepper_Axis_t axis, uint32_t* remaining_pulses);

/* Interrupt handlers */
void StepperTimer_TIM3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_TIMER_H */