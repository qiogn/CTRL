/**
  ******************************************************************************
  * @file    stepper_timer.c
  * @brief   Hardware timer PWM driver implementation for stepper motor control
  *          Uses TIM3 to generate precise step pulses with frequency control
  ******************************************************************************
  */

#include "stepper_timer.h"
#include "stepper_config.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/* Timer clock configuration */
#define TIMER_CLOCK_FREQ_HZ     TIMER_CLOCK_FREQ_HZ  /* From config */
#define TIMER_PRESCALER         TIMER_PRESCALER      /* From config */
#define TIMER_COUNTER_MODE      TIM_COUNTERMODE_UP

/* PWM mode configuration */
#define PWM_MODE                TIM_OCMODE_PWM1
#define PWM_POLARITY            TIM_OCPOLARITY_HIGH
#define PWM_IDLE_STATE          TIM_OCIDLESTATE_RESET

/* Private variables ---------------------------------------------------------*/

static StepperTimer_Handle_t g_stepper_timer = {
    .htim = NULL,
    .config = {
        .timer_frequency_hz = TIMER_CLOCK_FREQ_HZ,
        .min_pulse_frequency_hz = STEPPER_TIMER_MIN_FREQ_HZ,
        .max_pulse_frequency_hz = STEPPER_TIMER_MAX_FREQ_HZ,
        .default_frequency_hz = STEPPER_TIMER_DEFAULT_FREQ_HZ,
        .pulse_duty_cycle = STEPPER_TIMER_PULSE_DUTY_CYCLE
    },
    .is_initialized = 0,
    .is_running = 0,
    .current_freq_hz = STEPPER_TIMER_DEFAULT_FREQ_HZ
};

static TIM_HandleTypeDef g_htim3;

/* Error tracking */
static volatile uint32_t g_error_count = 0;

/* Private function prototypes -----------------------------------------------*/
static StepperTimer_Status_t configure_timer_base(void);
static StepperTimer_Status_t configure_pwm_channel(uint32_t channel);
static uint32_t calculate_arr_value(uint32_t frequency_hz);
static StepperTimer_Status_t validate_frequency(uint32_t frequency_hz);
static void handle_timer_error(const char* error_msg);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the stepper timer hardware
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_Init(void)
{
    StepperTimer_Status_t status = STEPPER_TIMER_OK;

    /* Check if already initialized */
    if (g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_OK;
    }

    /* Enable timer clock */
    STEPPER_TIMER_CLK_ENABLE();

    /* Configure timer handle */
    g_htim3.Instance = STEPPER_TIMER_INSTANCE;
    g_htim3.Init.Prescaler = TIMER_PRESCALER;
    g_htim3.Init.CounterMode = TIMER_COUNTER_MODE;
    g_htim3.Init.Period = calculate_arr_value(g_stepper_timer.config.default_frequency_hz);
    g_htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_htim3.Init.RepetitionCounter = 0;
    g_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    /* Initialize timer */
    if (HAL_TIM_PWM_Init(&g_htim3) != HAL_OK)
    {
        handle_timer_error("Timer initialization failed");
        return STEPPER_TIMER_ERROR;
    }

    /* Configure PWM channels for both motors */
    if (configure_pwm_channel(STEPPER_TIMER_CHANNEL_1) != STEPPER_TIMER_OK ||
        configure_pwm_channel(STEPPER_TIMER_CHANNEL_2) != STEPPER_TIMER_OK)
    {
        handle_timer_error("PWM channel configuration failed");
        return STEPPER_TIMER_ERROR;
    }

    /* Configure timer base */
    if (configure_timer_base() != STEPPER_TIMER_OK)
    {
        handle_timer_error("Timer base configuration failed");
        return STEPPER_TIMER_ERROR;
    }

    /* Store handle and update status */
    g_stepper_timer.htim = &g_htim3;
    g_stepper_timer.is_initialized = 1;
    g_stepper_timer.is_running = 0;
    g_stepper_timer.current_freq_hz = g_stepper_timer.config.default_frequency_hz;

    return status;
}

/**
  * @brief  Deinitialize the stepper timer hardware
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_DeInit(void)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    /* Stop timer if running */
    if (g_stepper_timer.is_running)
    {
        StepperTimer_StopBoth();
    }

    /* Deinitialize timer */
    if (HAL_TIM_PWM_DeInit(&g_htim3) != HAL_OK)
    {
        handle_timer_error("Timer deinitialization failed");
        return STEPPER_TIMER_ERROR;
    }

    /* Reset handle and status */
    memset(&g_stepper_timer, 0, sizeof(g_stepper_timer));

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Get current timer configuration
  * @param  config: Pointer to configuration structure
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_GetConfig(StepperTimer_Config_t* config)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    if (config == NULL)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    memcpy(config, &g_stepper_timer.config, sizeof(StepperTimer_Config_t));

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Set timer configuration
  * @param  config: Pointer to new configuration
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_SetConfig(const StepperTimer_Config_t* config)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    if (config == NULL)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    /* Validate configuration parameters */
    if (config->min_pulse_frequency_hz > config->max_pulse_frequency_hz ||
        config->default_frequency_hz < config->min_pulse_frequency_hz ||
        config->default_frequency_hz > config->max_pulse_frequency_hz ||
        config->pulse_duty_cycle == 0 || config->pulse_duty_cycle > 100)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    /* Update configuration */
    memcpy(&g_stepper_timer.config, config, sizeof(StepperTimer_Config_t));

    /* If timer is running, update frequency */
    if (g_stepper_timer.is_running)
    {
        uint32_t current_freq = g_stepper_timer.current_freq_hz;

        /* Clamp current frequency to new limits */
        if (current_freq < config->min_pulse_frequency_hz)
        {
            current_freq = config->min_pulse_frequency_hz;
        }
        else if (current_freq > config->max_pulse_frequency_hz)
        {
            current_freq = config->max_pulse_frequency_hz;
        }

        StepperTimer_SetFrequencyBoth(current_freq, current_freq);
    }

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Set pulse frequency for specific axis
  * @param  axis: Motor axis
  * @param  frequency_hz: Desired frequency in Hz
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_SetFrequency(Stepper_Axis_t axis, uint32_t frequency_hz)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    /* Validate frequency */
    StepperTimer_Status_t status = validate_frequency(frequency_hz);
    if (status != STEPPER_TIMER_OK)
    {
        return status;
    }

    /* Calculate new ARR value */
    uint32_t arr_value = calculate_arr_value(frequency_hz);

    /* Update timer period */
    __HAL_TIM_SET_AUTORELOAD(g_stepper_timer.htim, arr_value);

    /* Update duty cycle based on new period */
    uint32_t ccr_value = (arr_value * g_stepper_timer.config.pulse_duty_cycle) / 100;

    if (axis == STEPPER_AXIS_1 || axis == STEPPER_AXIS_BOTH)
    {
        __HAL_TIM_SET_COMPARE(g_stepper_timer.htim, STEPPER_TIMER_CHANNEL_1, ccr_value);
    }

    if (axis == STEPPER_AXIS_2 || axis == STEPPER_AXIS_BOTH)
    {
        __HAL_TIM_SET_COMPARE(g_stepper_timer.htim, STEPPER_TIMER_CHANNEL_2, ccr_value);
    }

    /* Update current frequency */
    if (axis == STEPPER_AXIS_BOTH)
    {
        g_stepper_timer.current_freq_hz = frequency_hz;
    }

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Get current pulse frequency for specific axis
  * @param  axis: Motor axis
  * @param  frequency_hz: Pointer to store frequency
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_GetFrequency(Stepper_Axis_t axis, uint32_t* frequency_hz)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    if (frequency_hz == NULL)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    *frequency_hz = g_stepper_timer.current_freq_hz;

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Set different frequencies for both motors
  * @param  freq_motor1_hz: Frequency for motor 1
  * @param  freq_motor2_hz: Frequency for motor 2
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_SetFrequencyBoth(uint32_t freq_motor1_hz, uint32_t freq_motor2_hz)
{
    /* For simplicity, we use the higher frequency for both motors */
    /* In a more advanced implementation, we could use separate timers */
    uint32_t higher_freq = (freq_motor1_hz > freq_motor2_hz) ? freq_motor1_hz : freq_motor2_hz;

    return StepperTimer_SetFrequency(STEPPER_AXIS_BOTH, higher_freq);
}

/**
  * @brief  Start pulse generation for specific axis
  * @param  axis: Motor axis
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_Start(Stepper_Axis_t axis)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    HAL_StatusTypeDef hal_status = HAL_OK;

    if (axis == STEPPER_AXIS_1 || axis == STEPPER_AXIS_BOTH)
    {
        hal_status = HAL_TIM_PWM_Start(g_stepper_timer.htim, STEPPER_TIMER_CHANNEL_1);
        if (hal_status != HAL_OK)
        {
            handle_timer_error("Failed to start motor 1 PWM");
            return STEPPER_TIMER_ERROR;
        }
    }

    if (axis == STEPPER_AXIS_2 || axis == STEPPER_AXIS_BOTH)
    {
        hal_status = HAL_TIM_PWM_Start(g_stepper_timer.htim, STEPPER_TIMER_CHANNEL_2);
        if (hal_status != HAL_OK)
        {
            handle_timer_error("Failed to start motor 2 PWM");
            /* Try to stop motor 1 if it was started */
            if (axis == STEPPER_AXIS_BOTH)
            {
                HAL_TIM_PWM_Stop(g_stepper_timer.htim, STEPPER_TIMER_CHANNEL_1);
            }
            return STEPPER_TIMER_ERROR;
        }
    }

    if (axis == STEPPER_AXIS_BOTH)
    {
        g_stepper_timer.is_running = 1;
    }

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Stop pulse generation for specific axis
  * @param  axis: Motor axis
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_Stop(Stepper_Axis_t axis)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    if (axis == STEPPER_AXIS_1 || axis == STEPPER_AXIS_BOTH)
    {
        HAL_TIM_PWM_Stop(g_stepper_timer.htim, STEPPER_TIMER_CHANNEL_1);
    }

    if (axis == STEPPER_AXIS_2 || axis == STEPPER_AXIS_BOTH)
    {
        HAL_TIM_PWM_Stop(g_stepper_timer.htim, STEPPER_TIMER_CHANNEL_2);
    }

    if (axis == STEPPER_AXIS_BOTH)
    {
        g_stepper_timer.is_running = 0;
    }

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Start pulse generation for both motors
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_StartBoth(void)
{
    return StepperTimer_Start(STEPPER_AXIS_BOTH);
}

/**
  * @brief  Stop pulse generation for both motors
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_StopBoth(void)
{
    return StepperTimer_Stop(STEPPER_AXIS_BOTH);
}

/**
  * @brief  Get timer status for specific axis
  * @param  axis: Motor axis
  * @param  is_running: Pointer to store running status
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_GetStatus(Stepper_Axis_t axis, uint8_t* is_running)
{
    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    if (is_running == NULL)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    *is_running = g_stepper_timer.is_running;

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Check if timer is initialized
  * @param  is_initialized: Pointer to store initialization status
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_IsInitialized(uint8_t* is_initialized)
{
    if (is_initialized == NULL)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    *is_initialized = g_stepper_timer.is_initialized;

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Get error count
  * @param  error_count: Pointer to store error count
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_GetErrorCount(uint32_t* error_count)
{
    if (error_count == NULL)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    *error_count = g_error_count;

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Generate a single pulse on specific axis
  * @param  axis: Motor axis
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_GenerateSinglePulse(Stepper_Axis_t axis)
{
    /* This is a simplified implementation */
    /* In a real implementation, we would use one-pulse mode */

    if (!g_stepper_timer.is_initialized)
    {
        return STEPPER_TIMER_NOT_INITIALIZED;
    }

    /* Start PWM */
    StepperTimer_Start(axis);

    /* Wait for one period */
    uint32_t period_us = 1000000 / g_stepper_timer.current_freq_hz;
    HAL_Delay(period_us / 1000);  /* Convert to milliseconds */

    /* Stop PWM */
    StepperTimer_Stop(axis);

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Set pulse count for specific axis
  * @param  axis: Motor axis
  * @param  pulse_count: Number of pulses to generate
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_SetPulseCount(Stepper_Axis_t axis, uint32_t pulse_count)
{
    /* This function is a placeholder for future implementation */
    /* In a real implementation, we would use timer repetition counter */

    (void)axis;
    (void)pulse_count;

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Get remaining pulses for specific axis
  * @param  axis: Motor axis
  * @param  remaining_pulses: Pointer to store remaining pulses
  * @retval StepperTimer_Status_t Status code
  */
StepperTimer_Status_t StepperTimer_GetRemainingPulses(Stepper_Axis_t axis, uint32_t* remaining_pulses)
{
    /* This function is a placeholder for future implementation */

    (void)axis;

    if (remaining_pulses == NULL)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    *remaining_pulses = 0;

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Timer interrupt handler
  * @retval None
  */
void StepperTimer_TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_htim3);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure timer base settings
  * @retval StepperTimer_Status_t Status code
  */
static StepperTimer_Status_t configure_timer_base(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* Configure clock source */
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&g_htim3, &sClockSourceConfig) != HAL_OK)
    {
        return STEPPER_TIMER_ERROR;
    }

    /* Configure master mode */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_htim3, &sMasterConfig) != HAL_OK)
    {
        return STEPPER_TIMER_ERROR;
    }

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Configure PWM channel
  * @param  channel: Timer channel
  * @retval StepperTimer_Status_t Status code
  */
static StepperTimer_Status_t configure_pwm_channel(uint32_t channel)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = PWM_MODE;
    sConfigOC.Pulse = 0;  /* Will be set when frequency is configured */
    sConfigOC.OCPolarity = PWM_POLARITY;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = PWM_IDLE_STATE;

    if (HAL_TIM_PWM_ConfigChannel(&g_htim3, &sConfigOC, channel) != HAL_OK)
    {
        return STEPPER_TIMER_ERROR;
    }

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Calculate ARR value for given frequency
  * @param  frequency_hz: Desired frequency in Hz
  * @retval ARR register value
  */
static uint32_t calculate_arr_value(uint32_t frequency_hz)
{
    /* Timer clock after prescaler = 1MHz (72MHz / (71+1)) */
    /* ARR = (Timer clock / frequency) - 1 */
    uint32_t timer_clock_hz = TIMER_CLOCK_FREQ_HZ / (TIMER_PRESCALER + 1);
    uint32_t arr_value = (timer_clock_hz / frequency_hz) - 1;

    /* Ensure minimum ARR value */
    if (arr_value < 1)
    {
        arr_value = 1;
    }

    /* Ensure maximum ARR value (16-bit timer) */
    if (arr_value > 0xFFFF)
    {
        arr_value = 0xFFFF;
    }

    return arr_value;
}

/**
  * @brief  Validate frequency parameter
  * @param  frequency_hz: Frequency to validate
  * @retval StepperTimer_Status_t Status code
  */
static StepperTimer_Status_t validate_frequency(uint32_t frequency_hz)
{
    if (frequency_hz < g_stepper_timer.config.min_pulse_frequency_hz ||
        frequency_hz > g_stepper_timer.config.max_pulse_frequency_hz)
    {
        return STEPPER_TIMER_INVALID_PARAM;
    }

    return STEPPER_TIMER_OK;
}

/**
  * @brief  Handle timer error
  * @param  error_msg: Error message
  * @retval None
  */
static void handle_timer_error(const char* error_msg)
{
    (void)error_msg;  /* In production, could log error */
    g_error_count++;
}