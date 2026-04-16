/**
  ******************************************************************************
  * @file    stepper_controller.c
  * @brief   Trapezoidal acceleration controller implementation
  *          Provides smooth acceleration/deceleration profiles for stepper motors
  ******************************************************************************
  */

#include "stepper_controller.h"
#include "stepper_timer.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/* Math constants */
#define PI 3.14159265358979323846f

/* Safety limits */
#define MAX_ACCELERATION_HZ_S     100000U  /* Maximum acceleration 100kHz/s */
#define MAX_DECELERATION_HZ_S     100000U  /* Maximum deceleration 100kHz/s */

/* Update timing */
#define UPDATE_INTERVAL_MS        1U       /* Controller update interval */

/* Private variables ---------------------------------------------------------*/

static StepperCtrl_Handle_t g_stepper_ctrl = {
    .motor1 = {
        .profile_config = {
            .max_speed_hz = DEFAULT_MAX_SPEED_HZ,
            .start_speed_hz = DEFAULT_START_SPEED_HZ,
            .acceleration_hz_s = DEFAULT_ACCELERATION_HZ_S,
            .deceleration_hz_s = DEFAULT_DECELERATION_HZ_S,
            .profile_type = PROFILE_TRAPEZOIDAL
        },
        .state = MOTOR_IDLE,
        .target_position = 0,
        .current_position = 0,
        .current_speed_hz = 0,
        .acceleration_steps = 0,
        .deceleration_steps = 0,
        .cruise_steps = 0,
        .step_counter = 0,
        .speed_update_counter = 0,
        .speed_update_interval = SPEED_UPDATE_INTERVAL_MS / UPDATE_INTERVAL_MS,
        .is_moving = 0,
        .direction = 0,
        .homing_complete = 0
    },
    .motor2 = {
        .profile_config = {
            .max_speed_hz = DEFAULT_MAX_SPEED_HZ,
            .start_speed_hz = DEFAULT_START_SPEED_HZ,
            .acceleration_hz_s = DEFAULT_ACCELERATION_HZ_S,
            .deceleration_hz_s = DEFAULT_DECELERATION_HZ_S,
            .profile_type = PROFILE_TRAPEZOIDAL
        },
        .state = MOTOR_IDLE,
        .target_position = 0,
        .current_position = 0,
        .current_speed_hz = 0,
        .acceleration_steps = 0,
        .deceleration_steps = 0,
        .cruise_steps = 0,
        .step_counter = 0,
        .speed_update_counter = 0,
        .speed_update_interval = SPEED_UPDATE_INTERVAL_MS / UPDATE_INTERVAL_MS,
        .is_moving = 0,
        .direction = 0,
        .homing_complete = 0
    },
    .is_initialized = 0,
    .error_count = 0
};

/* System tick counter for timing */
static volatile uint32_t g_system_tick = 0;

/* Private function prototypes -----------------------------------------------*/
static StepperCtrl_Status_t validate_profile_config(const MotionProfile_Config_t* config);
static StepperCtrl_Status_t calculate_trapezoidal_profile(Motor_Control_t* motor);
static StepperCtrl_Status_t update_motor_state(Motor_Control_t* motor, Stepper_Axis_t axis);
static StepperCtrl_Status_t set_motor_speed(Stepper_Axis_t axis, uint32_t speed_hz);
static StepperCtrl_Status_t set_motor_direction(Stepper_Axis_t axis, uint8_t direction);
static void handle_controller_error(const char* error_msg);
static uint32_t calculate_acceleration_time(uint32_t start_speed, uint32_t end_speed, uint32_t acceleration);
static uint32_t calculate_acceleration_distance(uint32_t start_speed, uint32_t end_speed, uint32_t acceleration);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the stepper controller
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_Init(void)
{
    /* Initialize timer driver */
    if (StepperTimer_Init() != STEPPER_TIMER_OK)
    {
        handle_controller_error("Timer initialization failed");
        return STEPPER_CTRL_ERROR;
    }

    /* Reset motor states */
    memset(&g_stepper_ctrl.motor1, 0, sizeof(Motor_Control_t));
    memset(&g_stepper_ctrl.motor2, 0, sizeof(Motor_Control_t));

    /* Set default configuration */
    g_stepper_ctrl.motor1.profile_config.max_speed_hz = DEFAULT_MAX_SPEED_HZ;
    g_stepper_ctrl.motor1.profile_config.start_speed_hz = DEFAULT_START_SPEED_HZ;
    g_stepper_ctrl.motor1.profile_config.acceleration_hz_s = DEFAULT_ACCELERATION_HZ_S;
    g_stepper_ctrl.motor1.profile_config.deceleration_hz_s = DEFAULT_DECELERATION_HZ_S;
    g_stepper_ctrl.motor1.profile_config.profile_type = PROFILE_TRAPEZOIDAL;
    g_stepper_ctrl.motor1.state = MOTOR_IDLE;
    g_stepper_ctrl.motor1.current_speed_hz = 0;

    g_stepper_ctrl.motor2.profile_config.max_speed_hz = DEFAULT_MAX_SPEED_HZ;
    g_stepper_ctrl.motor2.profile_config.start_speed_hz = DEFAULT_START_SPEED_HZ;
    g_stepper_ctrl.motor2.profile_config.acceleration_hz_s = DEFAULT_ACCELERATION_HZ_S;
    g_stepper_ctrl.motor2.profile_config.deceleration_hz_s = DEFAULT_DECELERATION_HZ_S;
    g_stepper_ctrl.motor2.profile_config.profile_type = PROFILE_TRAPEZOIDAL;
    g_stepper_ctrl.motor2.state = MOTOR_IDLE;
    g_stepper_ctrl.motor2.current_speed_hz = 0;

    g_stepper_ctrl.is_initialized = 1;
    g_stepper_ctrl.error_count = 0;

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Deinitialize the stepper controller
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_DeInit(void)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    /* Stop all motors */
    StepperCtrl_StopAll();

    /* Deinitialize timer */
    StepperTimer_DeInit();

    /* Reset controller state */
    memset(&g_stepper_ctrl, 0, sizeof(g_stepper_ctrl));

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Configure motion profile for specific axis
  * @param  axis: Motor axis
  * @param  config: Pointer to profile configuration
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_ConfigureProfile(Stepper_Axis_t axis, const MotionProfile_Config_t* config)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (config == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Validate configuration */
    StepperCtrl_Status_t status = validate_profile_config(config);
    if (status != STEPPER_CTRL_OK)
    {
        return status;
    }

    /* Update configuration */
    if (axis == STEPPER_AXIS_1 || axis == STEPPER_AXIS_BOTH)
    {
        memcpy(&g_stepper_ctrl.motor1.profile_config, config, sizeof(MotionProfile_Config_t));
    }

    if (axis == STEPPER_AXIS_2 || axis == STEPPER_AXIS_BOTH)
    {
        memcpy(&g_stepper_ctrl.motor2.profile_config, config, sizeof(MotionProfile_Config_t));
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Get current profile configuration
  * @param  axis: Motor axis
  * @param  config: Pointer to store configuration
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_GetProfileConfig(Stepper_Axis_t axis, MotionProfile_Config_t* config)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (config == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (axis == STEPPER_AXIS_1)
    {
        memcpy(config, &g_stepper_ctrl.motor1.profile_config, sizeof(MotionProfile_Config_t));
    }
    else if (axis == STEPPER_AXIS_2)
    {
        memcpy(config, &g_stepper_ctrl.motor2.profile_config, sizeof(MotionProfile_Config_t));
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Move to absolute position
  * @param  axis: Motor axis
  * @param  target_position: Target position in steps
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_MoveToPosition(Stepper_Axis_t axis, int32_t target_position)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    /* Check position limits */
    if (target_position > MAX_POSITION_STEPS || target_position < MIN_POSITION_STEPS)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    Motor_Control_t* motor = NULL;

    if (axis == STEPPER_AXIS_1)
    {
        motor = &g_stepper_ctrl.motor1;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        motor = &g_stepper_ctrl.motor2;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Check if motor is already moving */
    if (motor->is_moving)
    {
        return STEPPER_CTRL_BUSY;
    }

    /* Calculate movement parameters */
    int32_t steps_to_move = target_position - motor->current_position;

    if (steps_to_move == 0)
    {
        return STEPPER_CTRL_OK;  /* Already at target */
    }

    /* Set direction */
    motor->direction = (steps_to_move > 0) ? 0 : 1;
    set_motor_direction(axis, motor->direction);

    /* Set target position */
    motor->target_position = target_position;

    /* Calculate trapezoidal profile */
    motor->step_counter = 0;
    motor->speed_update_counter = 0;
    motor->current_speed_hz = motor->profile_config.start_speed_hz;

    /* For now, use simple constant speed movement */
    /* In a complete implementation, we would calculate the full profile */
    motor->is_moving = 1;
    motor->state = MOTOR_CRUISING;

    /* Set initial speed */
    set_motor_speed(axis, motor->current_speed_hz);

    /* Start timer */
    StepperTimer_Start(axis);

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Move relative number of steps
  * @param  axis: Motor axis
  * @param  steps: Number of steps to move (positive = forward, negative = reverse)
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_MoveRelative(Stepper_Axis_t axis, int32_t steps)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    Motor_Control_t* motor = NULL;

    if (axis == STEPPER_AXIS_1)
    {
        motor = &g_stepper_ctrl.motor1;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        motor = &g_stepper_ctrl.motor2;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Calculate target position */
    int32_t target_position = motor->current_position + steps;

    return StepperCtrl_MoveToPosition(axis, target_position);
}

/**
  * @brief  Move both axes simultaneously
  * @param  motor1_steps: Steps for motor 1
  * @param  motor2_steps: Steps for motor 2
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_MoveAxes(int32_t motor1_steps, int32_t motor2_steps)
{
    StepperCtrl_Status_t status1 = STEPPER_CTRL_OK;
    StepperCtrl_Status_t status2 = STEPPER_CTRL_OK;

    /* Move motor 1 if needed */
    if (motor1_steps != 0)
    {
        status1 = StepperCtrl_MoveRelative(STEPPER_AXIS_1, motor1_steps);
    }

    /* Move motor 2 if needed */
    if (motor2_steps != 0)
    {
        status2 = StepperCtrl_MoveRelative(STEPPER_AXIS_2, motor2_steps);
    }

    /* Return worst status */
    if (status1 != STEPPER_CTRL_OK)
    {
        return status1;
    }

    return status2;
}

/**
  * @brief  Stop motor movement
  * @param  axis: Motor axis
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_Stop(Stepper_Axis_t axis)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    Motor_Control_t* motor = NULL;

    if (axis == STEPPER_AXIS_1)
    {
        motor = &g_stepper_ctrl.motor1;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        motor = &g_stepper_ctrl.motor2;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Stop timer */
    StepperTimer_Stop(axis);

    /* Update motor state */
    motor->is_moving = 0;
    motor->state = MOTOR_IDLE;
    motor->current_speed_hz = 0;

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Stop all motors
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_StopAll(void)
{
    StepperCtrl_Status_t status1 = StepperCtrl_Stop(STEPPER_AXIS_1);
    StepperCtrl_Status_t status2 = StepperCtrl_Stop(STEPPER_AXIS_2);

    if (status1 != STEPPER_CTRL_OK)
    {
        return status1;
    }

    return status2;
}

/**
  * @brief  Emergency stop - immediate halt
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_EmergencyStop(void)
{
    /* Immediately stop timer */
    StepperTimer_StopBoth();

    /* Reset motor states */
    g_stepper_ctrl.motor1.is_moving = 0;
    g_stepper_ctrl.motor1.state = MOTOR_IDLE;
    g_stepper_ctrl.motor1.current_speed_hz = 0;

    g_stepper_ctrl.motor2.is_moving = 0;
    g_stepper_ctrl.motor2.state = MOTOR_IDLE;
    g_stepper_ctrl.motor2.current_speed_hz = 0;

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Set constant speed for motor
  * @param  axis: Motor axis
  * @param  speed_hz: Speed in Hz
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_SetSpeed(Stepper_Axis_t axis, uint32_t speed_hz)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    /* Validate speed */
    if (speed_hz < MIN_SPEED_HZ || speed_hz > MAX_SPEED_HZ)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    Motor_Control_t* motor = NULL;

    if (axis == STEPPER_AXIS_1)
    {
        motor = &g_stepper_ctrl.motor1;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        motor = &g_stepper_ctrl.motor2;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Update speed */
    motor->current_speed_hz = speed_hz;

    /* If motor is moving, update timer frequency */
    if (motor->is_moving)
    {
        return set_motor_speed(axis, speed_hz);
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Get current motor speed
  * @param  axis: Motor axis
  * @param  speed_hz: Pointer to store speed
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_GetSpeed(Stepper_Axis_t axis, uint32_t* speed_hz)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (speed_hz == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (axis == STEPPER_AXIS_1)
    {
        *speed_hz = g_stepper_ctrl.motor1.current_speed_hz;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        *speed_hz = g_stepper_ctrl.motor2.current_speed_hz;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Set maximum speed for motor
  * @param  axis: Motor axis
  * @param  max_speed_hz: Maximum speed in Hz
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_SetMaxSpeed(Stepper_Axis_t axis, uint32_t max_speed_hz)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    /* Validate speed */
    if (max_speed_hz < MIN_SPEED_HZ || max_speed_hz > MAX_SPEED_HZ)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (axis == STEPPER_AXIS_1)
    {
        g_stepper_ctrl.motor1.profile_config.max_speed_hz = max_speed_hz;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        g_stepper_ctrl.motor2.profile_config.max_speed_hz = max_speed_hz;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Get current motor position
  * @param  axis: Motor axis
  * @param  position: Pointer to store position
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_GetPosition(Stepper_Axis_t axis, int32_t* position)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (position == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (axis == STEPPER_AXIS_1)
    {
        *position = g_stepper_ctrl.motor1.current_position;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        *position = g_stepper_ctrl.motor2.current_position;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Set motor position (for homing or manual adjustment)
  * @param  axis: Motor axis
  * @param  position: New position value
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_SetPosition(Stepper_Axis_t axis, int32_t position)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    /* Check position limits */
    if (position > MAX_POSITION_STEPS || position < MIN_POSITION_STEPS)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (axis == STEPPER_AXIS_1)
    {
        g_stepper_ctrl.motor1.current_position = position;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        g_stepper_ctrl.motor2.current_position = position;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Reset motor position to zero
  * @param  axis: Motor axis
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_ResetPosition(Stepper_Axis_t axis)
{
    return StepperCtrl_SetPosition(axis, 0);
}

/**
  * @brief  Get motor state
  * @param  axis: Motor axis
  * @param  state: Pointer to store state
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_GetState(Stepper_Axis_t axis, Motor_State_t* state)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (state == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (axis == STEPPER_AXIS_1)
    {
        *state = g_stepper_ctrl.motor1.state;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        *state = g_stepper_ctrl.motor2.state;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Check if motor is moving
  * @param  axis: Motor axis
  * @param  is_moving: Pointer to store moving status
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_IsMoving(Stepper_Axis_t axis, uint8_t* is_moving)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (is_moving == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (axis == STEPPER_AXIS_1)
    {
        *is_moving = g_stepper_ctrl.motor1.is_moving;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        *is_moving = g_stepper_ctrl.motor2.is_moving;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Get error count
  * @param  error_count: Pointer to store error count
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_GetErrorCount(uint32_t* error_count)
{
    if (error_count == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    *error_count = g_stepper_ctrl.error_count;

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Clear error count
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_ClearErrors(void)
{
    g_stepper_ctrl.error_count = 0;

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Update controller state (to be called periodically)
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_Update(void)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    /* Update system tick */
    g_system_tick++;

    /* Update motor 1 state */
    if (g_stepper_ctrl.motor1.is_moving)
    {
        update_motor_state(&g_stepper_ctrl.motor1, STEPPER_AXIS_1);
    }

    /* Update motor 2 state */
    if (g_stepper_ctrl.motor2.is_moving)
    {
        update_motor_state(&g_stepper_ctrl.motor2, STEPPER_AXIS_2);
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Calculate estimated move time
  * @param  axis: Motor axis
  * @param  steps: Number of steps
  * @param  time_ms: Pointer to store time in milliseconds
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_CalculateMoveTime(Stepper_Axis_t axis, int32_t steps, uint32_t* time_ms)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (time_ms == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    Motor_Control_t* motor = NULL;

    if (axis == STEPPER_AXIS_1)
    {
        motor = &g_stepper_ctrl.motor1;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        motor = &g_stepper_ctrl.motor2;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Simplified calculation: time = steps / speed */
    uint32_t abs_steps = (steps > 0) ? steps : -steps;
    uint32_t avg_speed_hz = (motor->profile_config.start_speed_hz + motor->profile_config.max_speed_hz) / 2;

    if (avg_speed_hz == 0)
    {
        *time_ms = 0;
    }
    else
    {
        *time_ms = (abs_steps * 1000) / avg_speed_hz;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Get remaining steps to target
  * @param  axis: Motor axis
  * @param  remaining_steps: Pointer to store remaining steps
  * @retval StepperCtrl_Status_t Status code
  */
StepperCtrl_Status_t StepperCtrl_GetRemainingSteps(Stepper_Axis_t axis, uint32_t* remaining_steps)
{
    if (!g_stepper_ctrl.is_initialized)
    {
        return STEPPER_CTRL_NOT_INITIALIZED;
    }

    if (remaining_steps == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    int32_t remaining = 0;

    if (axis == STEPPER_AXIS_1)
    {
        remaining = g_stepper_ctrl.motor1.target_position - g_stepper_ctrl.motor1.current_position;
    }
    else if (axis == STEPPER_AXIS_2)
    {
        remaining = g_stepper_ctrl.motor2.target_position - g_stepper_ctrl.motor2.current_position;
    }
    else
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    *remaining_steps = (remaining > 0) ? remaining : -remaining;

    return STEPPER_CTRL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Validate profile configuration
  * @param  config: Profile configuration
  * @retval StepperCtrl_Status_t Status code
  */
static StepperCtrl_Status_t validate_profile_config(const MotionProfile_Config_t* config)
{
    if (config == NULL)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Check speed limits */
    if (config->max_speed_hz < MIN_SPEED_HZ || config->max_speed_hz > MAX_SPEED_HZ)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (config->start_speed_hz < MIN_SPEED_HZ || config->start_speed_hz > config->max_speed_hz)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    /* Check acceleration limits */
    if (config->acceleration_hz_s == 0 || config->acceleration_hz_s > MAX_ACCELERATION_HZ_S)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    if (config->deceleration_hz_s == 0 || config->deceleration_hz_s > MAX_DECELERATION_HZ_S)
    {
        return STEPPER_CTRL_INVALID_PARAM;
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Calculate trapezoidal motion profile
  * @param  motor: Motor control structure
  * @retval StepperCtrl_Status_t Status code
  */
static StepperCtrl_Status_t calculate_trapezoidal_profile(Motor_Control_t* motor)
{
    /* This is a simplified implementation */
    /* In a complete implementation, we would calculate:
       - Acceleration time and distance
       - Deceleration time and distance
       - Cruise distance
       - Check if trapezoidal profile is possible
    */

    /* For now, just set reasonable defaults */
    motor->acceleration_steps = 100;
    motor->deceleration_steps = 100;
    motor->cruise_steps = 200;

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Update motor state during movement
  * @param  motor: Motor control structure
  * @param  axis: Motor axis
  * @retval StepperCtrl_Status_t Status code
  */
static StepperCtrl_Status_t update_motor_state(Motor_Control_t* motor, Stepper_Axis_t axis)
{
    /* Check if target reached */
    if (motor->current_position == motor->target_position)
    {
        motor->is_moving = 0;
        motor->state = MOTOR_IDLE;
        motor->current_speed_hz = 0;

        /* Stop timer */
        StepperTimer_Stop(axis);

        return STEPPER_CTRL_OK;
    }

    /* Update position based on direction */
    if (motor->direction == 0)
    {
        motor->current_position++;
    }
    else
    {
        motor->current_position--;
    }

    /* Update step counter */
    motor->step_counter++;

    /* Update speed if needed (simplified acceleration) */
    motor->speed_update_counter++;
    if (motor->speed_update_counter >= motor->speed_update_interval)
    {
        motor->speed_update_counter = 0;

        /* Simple acceleration: increase speed until max */
        if (motor->current_speed_hz < motor->profile_config.max_speed_hz)
        {
            motor->current_speed_hz += 100;  /* Simplified acceleration */
            if (motor->current_speed_hz > motor->profile_config.max_speed_hz)
            {
                motor->current_speed_hz = motor->profile_config.max_speed_hz;
            }

            /* Update timer frequency */
            set_motor_speed(axis, motor->current_speed_hz);
        }
    }

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Set motor speed
  * @param  axis: Motor axis
  * @param  speed_hz: Speed in Hz
  * @retval StepperCtrl_Status_t Status code
  */
static StepperCtrl_Status_t set_motor_speed(Stepper_Axis_t axis, uint32_t speed_hz)
{
    return StepperTimer_SetFrequency(axis, speed_hz);
}

/**
  * @brief  Set motor direction
  * @param  axis: Motor axis
  * @param  direction: Direction (0=forward, 1=reverse)
  * @retval StepperCtrl_Status_t Status code
  */
static StepperCtrl_Status_t set_motor_direction(Stepper_Axis_t axis, uint8_t direction)
{
    /* This function needs to be implemented based on your hardware */
    /* For now, it's a placeholder */

    (void)axis;
    (void)direction;

    return STEPPER_CTRL_OK;
}

/**
  * @brief  Handle controller error
  * @param  error_msg: Error message
  * @retval None
  */
static void handle_controller_error(const char* error_msg)
{
    (void)error_msg;  /* In production, could log error */
    g_stepper_ctrl.error_count++;
}

/**
  * @brief  Calculate acceleration time
  * @param  start_speed: Starting speed in Hz
  * @param  end_speed: Ending speed in Hz
  * @param  acceleration: Acceleration in Hz/s
  * @retval Time in seconds
  */
static uint32_t calculate_acceleration_time(uint32_t start_speed, uint32_t end_speed, uint32_t acceleration)
{
    if (acceleration == 0)
    {
        return 0;
    }

    uint32_t speed_change = (end_speed > start_speed) ? (end_speed - start_speed) : (start_speed - end_speed);
    return (speed_change * 1000) / acceleration;  /* Convert to milliseconds */
}

/**
  * @brief  Calculate acceleration distance
  * @param  start_speed: Starting speed in Hz
  * @param  end_speed: Ending speed in Hz
  * @param  acceleration: Acceleration in Hz/s
  * @retval Distance in steps
  */
static uint32_t calculate_acceleration_distance(uint32_t start_speed, uint32_t end_speed, uint32_t acceleration)
{
    if (acceleration == 0)
    {
        return 0;
    }

    /* distance = (end_speed² - start_speed²) / (2 * acceleration) */
    uint32_t start_speed_sq = start_speed * start_speed;
    uint32_t end_speed_sq = end_speed * end_speed;

    if (end_speed_sq <= start_speed_sq)
    {
        return 0;
    }

    return (end_speed_sq - start_speed_sq) / (2 * acceleration);
}