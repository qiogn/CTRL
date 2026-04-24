/**
  ******************************************************************************
  * @file    dual_stepper.c
  * @brief   Dual stepper motor driver using TIM3 hardware PWM
  *          Non-blocking: MoveAxes returns immediately
  ******************************************************************************
  */

#include "dual_stepper.h"
#include "stepper_config.h"
#include "stepper_timer.h"

/* Last direction state per motor (to skip settling when unchanged) */
static uint8_t g_motor1_last_dir;
static uint8_t g_motor2_last_dir;

static volatile uint8_t g_hold_enabled = 0U;

/* Direction inversion per motor (matching old behaviour) */
#define MOTOR1_DIR_INVERT 1U
#define MOTOR2_DIR_INVERT 0U

/* DWT-based microsecond delay (Cortex-M3) */
static void DWT_DelayUs(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

static void set_motor_enable(GPIO_TypeDef *port, uint16_t pin, uint8_t enable)
{
    HAL_GPIO_WritePin(port, pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void set_motor_dir(GPIO_TypeDef *port, uint16_t pin, uint8_t invert,
                          uint8_t positive, uint8_t *last_dir, uint8_t first_time)
{
    uint8_t level = positive;
    if (invert) level = !level;

    /* Only wait settling time when direction actually changes */
    if (!first_time && (*last_dir == level)) {
        return;
    }
    *last_dir = level;

    HAL_GPIO_WritePin(port, pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    DWT_DelayUs(DIR_SETTLING_TIME_US);
}

void DualStepper_Init(void)
{
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Motor 1 (PA0=DIR, PA1=EN, PA6=STEP via TIM3) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = Dir1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Dir1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EN1_Pin;
    HAL_GPIO_Init(EN1_GPIO_Port, &GPIO_InitStruct);

    /* Motor 2 (PB0=DIR, PB10=EN, PB1=STEP via TIM3) */
    GPIO_InitStruct.Pin = Dir2_Pin;
    HAL_GPIO_Init(Dir2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EN2_Pin;
    HAL_GPIO_Init(EN2_GPIO_Port, &GPIO_InitStruct);

    /* Set initial direction levels */
    HAL_GPIO_WritePin(Dir1_GPIO_Port, Dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Dir2_GPIO_Port, Dir2_Pin, GPIO_PIN_RESET);
    g_motor1_last_dir = 0;
    g_motor2_last_dir = 0;

    /* Disable motors */
    set_motor_enable(EN1_GPIO_Port, EN1_Pin, 0U);
    set_motor_enable(EN2_GPIO_Port, EN2_Pin, 0U);

    /* Initialise TIM3 hardware PWM */
    StepperTimer_Init();

    g_hold_enabled = 0U;
}

void DualStepper_SetHoldEnabled(uint8_t enabled)
{
    g_hold_enabled = enabled ? 1U : 0U;
    if (enabled) {
        set_motor_enable(EN1_GPIO_Port, EN1_Pin, 1U);
        set_motor_enable(EN2_GPIO_Port, EN2_Pin, 1U);
    }
}

uint8_t DualStepper_IsMoving(void)
{
    return StepperTimer_IsBusy(STEPPER_CH_BOTH);
}

void DualStepper_MoveAxes(int16_t motor1_steps, int16_t motor2_steps, uint16_t pulse_width_us)
{
    /* Clamp pulse width */
    if (pulse_width_us < MIN_PULSE_WIDTH_US) pulse_width_us = MIN_PULSE_WIDTH_US;
    if (pulse_width_us > MAX_PULSE_WIDTH_US) pulse_width_us = MAX_PULSE_WIDTH_US;

    uint32_t s1_total = (motor1_steps >= 0) ? (uint32_t)motor1_steps : (uint32_t)(-motor1_steps);
    uint32_t s2_total = (motor2_steps >= 0) ? (uint32_t)motor2_steps : (uint32_t)(-motor2_steps);

    /* No movement needed */
    if (s1_total == 0 && s2_total == 0) return;

    /* Set direction pins (settling delay only when direction changes) */
    set_motor_dir(Dir1_GPIO_Port, Dir1_Pin, MOTOR1_DIR_INVERT,
                  (uint8_t)(motor1_steps >= 0), &g_motor1_last_dir, 0);
    set_motor_dir(Dir2_GPIO_Port, Dir2_Pin, MOTOR2_DIR_INVERT,
                  (uint8_t)(motor2_steps >= 0), &g_motor2_last_dir, 0);

    /* Enable motors if not in hold mode */
    if (!g_hold_enabled) {
        set_motor_enable(EN1_GPIO_Port, EN1_Pin, s1_total > 0);
        set_motor_enable(EN2_GPIO_Port, EN2_Pin, s2_total > 0);
    } else {
        set_motor_enable(EN1_GPIO_Port, EN1_Pin, 1U);
        set_motor_enable(EN2_GPIO_Port, EN2_Pin, 1U);
    }

    /* Convert pulse width to frequency: freq = 1000000 / (2 * pulse_width_us) */
    /* Each pulse cycle = high + low = pulse_width_us, so freq = 1000000 / pulse_width_us */
    uint32_t freq_hz = 1000000UL / pulse_width_us;
    if (freq_hz < MIN_SPEED_HZ) freq_hz = MIN_SPEED_HZ;

    /* Start hardware PWM — non-blocking, returns immediately */
    if (s1_total > 0) {
        StepperTimer_Start(STEPPER_CH_MOTOR1, freq_hz, s1_total);
    }
    if (s2_total > 0) {
        StepperTimer_Start(STEPPER_CH_MOTOR2, freq_hz, s2_total);
    }
}

/* Legacy / compatibility functions */
void DualStepper_SetTrackingCommand(int16_t motor1_cmd, int16_t motor2_cmd)
{
    DualStepper_MoveAxes(motor1_cmd, motor2_cmd, 1000);
}

void DualStepper_RunByVision(int16_t dx, int16_t dy)
{
    DualStepper_MoveAxes(dx, dy, 1000);
}

void DualStepper_TIM2_IRQHandler(void)
{
    /* Not used — TIM2 is kept inactive */
}
