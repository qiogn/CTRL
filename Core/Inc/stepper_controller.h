/**
  ******************************************************************************
  * @file    stepper_controller.h
  * @brief   Trapezoidal acceleration controller for stepper motors
  *          Provides smooth acceleration/deceleration profiles
  ******************************************************************************
  */

#ifndef __STEPPER_CONTROLLER_H
#define __STEPPER_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stepper_timer.h"
#include "stepper_config.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Stepper controller status enumeration
  */
typedef enum {
    STEPPER_CTRL_OK = 0,
    STEPPER_CTRL_ERROR,
    STEPPER_CTRL_BUSY,
    STEPPER_CTRL_INVALID_PARAM,
    STEPPER_CTRL_NOT_INITIALIZED,
    STEPPER_CTRL_MOTOR_STALLED
} StepperCtrl_Status_t;

/**
  * @brief  Motion profile type enumeration
  */
typedef enum {
    PROFILE_CONSTANT_SPEED = 0,  /* Constant speed movement */
    PROFILE_TRAPEZOIDAL,         /* Trapezoidal acceleration */
    PROFILE_S_CURVE              /* S-curve acceleration (future) */
} MotionProfile_Type_t;

/**
  * @brief  Motor state enumeration
  */
typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ACCELERATING,
    MOTOR_CRUISING,
    MOTOR_DECELERATING,
    MOTOR_STOPPING,
    MOTOR_ERROR
} Motor_State_t;

/**
  * @brief  Motion profile configuration structure
  */
typedef struct {
    uint32_t max_speed_hz;        /* Maximum speed in Hz */
    uint32_t start_speed_hz;      /* Starting speed in Hz */
    uint32_t acceleration_hz_s;   /* Acceleration in Hz/s */
    uint32_t deceleration_hz_s;   /* Deceleration in Hz/s */
    MotionProfile_Type_t profile_type; /* Profile type */
} MotionProfile_Config_t;

/**
  * @brief  Motor control structure
  */
typedef struct {
    /* Configuration */
    MotionProfile_Config_t profile_config;

    /* Current state */
    Motor_State_t state;
    int32_t target_position;      /* Target position in steps */
    int32_t current_position;     /* Current position in steps */
    uint32_t current_speed_hz;    /* Current speed in Hz */

    /* Movement parameters */
    uint32_t acceleration_steps;  /* Steps needed for acceleration */
    uint32_t deceleration_steps;  /* Steps needed for deceleration */
    uint32_t cruise_steps;        /* Steps at constant speed */

    /* Timing control */
    uint32_t step_counter;        /* Step counter */
    uint32_t speed_update_counter; /* Counter for speed updates */
    uint32_t speed_update_interval; /* Interval between speed updates */

    /* Status flags */
    uint8_t is_moving : 1;        /* Movement in progress */
    uint8_t direction : 1;        /* Movement direction (0=forward, 1=reverse) */
    uint8_t homing_complete : 1;  /* Homing completed flag */
} Motor_Control_t;

/**
  * @brief  Stepper controller handle structure
  */
typedef struct {
    Motor_Control_t motor1;       /* Motor 1 control */
    Motor_Control_t motor2;       /* Motor 2 control */
    volatile uint8_t is_initialized;
    volatile uint32_t error_count;
} StepperCtrl_Handle_t;

/* Exported constants --------------------------------------------------------*/

/* Default configuration values */
#define DEFAULT_MAX_SPEED_HZ         DEFAULT_SPEED_HZ          /* Default maximum speed */
#define DEFAULT_START_SPEED_HZ       MIN_SPEED_HZ              /* Starting speed */
/* Note: DEFAULT_ACCELERATION_HZ_S and DEFAULT_DECELERATION_HZ_S are defined in stepper_config.h */

/* Timing constants */
#define SPEED_UPDATE_INTERVAL_MS     10U                       /* Update speed every 10ms */

/* Position limits */
#define MAX_POSITION_STEPS           MAX_POSITIVE_POSITION     /* Maximum position */
#define MIN_POSITION_STEPS           MAX_NEGATIVE_POSITION     /* Minimum position */

/* Exported functions --------------------------------------------------------*/

/* Initialization and configuration */
StepperCtrl_Status_t StepperCtrl_Init(void);
StepperCtrl_Status_t StepperCtrl_DeInit(void);
StepperCtrl_Status_t StepperCtrl_ConfigureProfile(Stepper_Axis_t axis, const MotionProfile_Config_t* config);
StepperCtrl_Status_t StepperCtrl_GetProfileConfig(Stepper_Axis_t axis, MotionProfile_Config_t* config);

/* Movement control */
StepperCtrl_Status_t StepperCtrl_MoveToPosition(Stepper_Axis_t axis, int32_t target_position);
StepperCtrl_Status_t StepperCtrl_MoveRelative(Stepper_Axis_t axis, int32_t steps);
StepperCtrl_Status_t StepperCtrl_MoveAxes(int32_t motor1_steps, int32_t motor2_steps);
StepperCtrl_Status_t StepperCtrl_Stop(Stepper_Axis_t axis);
StepperCtrl_Status_t StepperCtrl_StopAll(void);
StepperCtrl_Status_t StepperCtrl_EmergencyStop(void);

/* Speed control */
StepperCtrl_Status_t StepperCtrl_SetSpeed(Stepper_Axis_t axis, uint32_t speed_hz);
StepperCtrl_Status_t StepperCtrl_GetSpeed(Stepper_Axis_t axis, uint32_t* speed_hz);
StepperCtrl_Status_t StepperCtrl_SetMaxSpeed(Stepper_Axis_t axis, uint32_t max_speed_hz);

/* Position control */
StepperCtrl_Status_t StepperCtrl_GetPosition(Stepper_Axis_t axis, int32_t* position);
StepperCtrl_Status_t StepperCtrl_SetPosition(Stepper_Axis_t axis, int32_t position);
StepperCtrl_Status_t StepperCtrl_ResetPosition(Stepper_Axis_t axis);

/* Status and monitoring */
StepperCtrl_Status_t StepperCtrl_GetState(Stepper_Axis_t axis, Motor_State_t* state);
StepperCtrl_Status_t StepperCtrl_IsMoving(Stepper_Axis_t axis, uint8_t* is_moving);
StepperCtrl_Status_t StepperCtrl_GetErrorCount(uint32_t* error_count);
StepperCtrl_Status_t StepperCtrl_ClearErrors(void);

/* Update function (to be called periodically) */
StepperCtrl_Status_t StepperCtrl_Update(void);

/* Utility functions */
StepperCtrl_Status_t StepperCtrl_CalculateMoveTime(Stepper_Axis_t axis, int32_t steps, uint32_t* time_ms);
StepperCtrl_Status_t StepperCtrl_GetRemainingSteps(Stepper_Axis_t axis, uint32_t* remaining_steps);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_CONTROLLER_H */