/**
  ******************************************************************************
  * @file    stepper_timer.c
  * @brief   TIM3 dual-channel PWM driver for stepper motors
  *          CH1(PA6) -> Motor1 STEP, CH4(PB1) -> Motor2 STEP
  ******************************************************************************
  */

#include "stepper_timer.h"
#include "stepper_config.h"
#include "main.h"

/* Per-channel state */
typedef struct {
    uint32_t channel;       /* TIM_CHANNEL_x */
    uint32_t remaining;     /* pulses left to generate */
    uint32_t gpio_port_bsrr_set;   /* BSRR set value for STEP pin */
    uint32_t gpio_port_bsrr_reset; /* BSRR reset value for STEP pin */
    uint8_t  busy;
} ChState_t;

static TIM_HandleTypeDef g_htim3;
static ChState_t g_ch[2];

void StepperTimer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable DWT cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Channel state init */
    g_ch[STEPPER_CH_MOTOR1].channel = STEPPER_TIMER_CHANNEL_1;
    g_ch[STEPPER_CH_MOTOR1].gpio_port_bsrr_set  = (uint32_t)Step1_Pin;
    g_ch[STEPPER_CH_MOTOR1].gpio_port_bsrr_reset = (uint32_t)Step1_Pin << 16;
    g_ch[STEPPER_CH_MOTOR1].remaining = 0;
    g_ch[STEPPER_CH_MOTOR1].busy = 0;

    g_ch[STEPPER_CH_MOTOR2].channel = STEPPER_TIMER_CHANNEL_2;
    g_ch[STEPPER_CH_MOTOR2].gpio_port_bsrr_set  = (uint32_t)Step2_Pin;
    g_ch[STEPPER_CH_MOTOR2].gpio_port_bsrr_reset = (uint32_t)Step2_Pin << 16;
    g_ch[STEPPER_CH_MOTOR2].remaining = 0;
    g_ch[STEPPER_CH_MOTOR2].busy = 0;

    /* Enable clocks */
    STEPPER_TIMER_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA6 (TIM3_CH1) — alternate function push-pull */
    GPIO_InitStruct.Pin = Step1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Step1_GPIO_Port, &GPIO_InitStruct);

    /* PB1 (TIM3_CH4) — alternate function push-pull */
    GPIO_InitStruct.Pin = Step2_Pin;
    HAL_GPIO_Init(Step2_GPIO_Port, &GPIO_InitStruct);

    /* TIM3 base config */
    g_htim3.Instance = STEPPER_TIMER_INSTANCE;
    g_htim3.Init.Prescaler = TIMER_PRESCALER;
    g_htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_htim3.Init.Period = 999;  /* 1MHz / 1000 = 1kHz default */
    g_htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&g_htim3);

    /* Configure CH1 and CH4, 50% duty cycle */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;  /* 50% duty */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&g_htim3, &sConfigOC, STEPPER_TIMER_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&g_htim3, &sConfigOC, STEPPER_TIMER_CHANNEL_2);

    /* Interrupt priority — below UART2(0) */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void StepperTimer_Start(uint8_t channel, uint32_t freq_hz, uint32_t steps)
{
    /* If no steps requested, do nothing — ensure channel is not busy */
    if (steps == 0) {
        if (channel == STEPPER_CH_MOTOR1 || channel == STEPPER_CH_BOTH) {
            g_ch[STEPPER_CH_MOTOR1].busy = 0;
            g_ch[STEPPER_CH_MOTOR1].remaining = 0;
        }
        if (channel == STEPPER_CH_MOTOR2 || channel == STEPPER_CH_BOTH) {
            g_ch[STEPPER_CH_MOTOR2].busy = 0;
            g_ch[STEPPER_CH_MOTOR2].remaining = 0;
        }
        return;
    }

    if (freq_hz == 0) freq_hz = 1;

    /* 1MHz timer counter clock: ARR = 1000000/freq - 1 */
    uint32_t arr = (1000000U / freq_hz);
    if (arr > 1) arr -= 1; else arr = 1;
    if (arr > 65535U) arr = 65535U;
    uint32_t ccr = arr / 2;  /* 50% duty */
    if (ccr < 1) ccr = 1;

    __HAL_TIM_SET_AUTORELOAD(&g_htim3, arr);
    __HAL_TIM_SET_COMPARE(&g_htim3, STEPPER_TIMER_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&g_htim3, STEPPER_TIMER_CHANNEL_2, ccr);

    if (channel == STEPPER_CH_MOTOR1 || channel == STEPPER_CH_BOTH) {
        g_ch[STEPPER_CH_MOTOR1].remaining = steps;
        g_ch[STEPPER_CH_MOTOR1].busy = 1;
        HAL_TIM_PWM_Start(&g_htim3, STEPPER_TIMER_CHANNEL_1);
        __HAL_TIM_ENABLE_IT(&g_htim3, TIM_IT_UPDATE);
    }
    if (channel == STEPPER_CH_MOTOR2 || channel == STEPPER_CH_BOTH) {
        g_ch[STEPPER_CH_MOTOR2].remaining = steps;
        g_ch[STEPPER_CH_MOTOR2].busy = 1;
        HAL_TIM_PWM_Start(&g_htim3, STEPPER_TIMER_CHANNEL_2);
        __HAL_TIM_ENABLE_IT(&g_htim3, TIM_IT_UPDATE);
    }
}

void StepperTimer_Stop(uint8_t channel)
{
    if (channel == STEPPER_CH_MOTOR1 || channel == STEPPER_CH_BOTH) {
        HAL_TIM_PWM_Stop(&g_htim3, STEPPER_TIMER_CHANNEL_1);
        g_ch[STEPPER_CH_MOTOR1].remaining = 0;
        g_ch[STEPPER_CH_MOTOR1].busy = 0;
    }
    if (channel == STEPPER_CH_MOTOR2 || channel == STEPPER_CH_BOTH) {
        HAL_TIM_PWM_Stop(&g_htim3, STEPPER_TIMER_CHANNEL_2);
        g_ch[STEPPER_CH_MOTOR2].remaining = 0;
        g_ch[STEPPER_CH_MOTOR2].busy = 0;
    }
    if (!g_ch[STEPPER_CH_MOTOR1].busy && !g_ch[STEPPER_CH_MOTOR2].busy) {
        __HAL_TIM_DISABLE_IT(&g_htim3, TIM_IT_UPDATE);
    }
}

uint8_t StepperTimer_IsBusy(uint8_t channel)
{
    if (channel == STEPPER_CH_MOTOR1) return g_ch[STEPPER_CH_MOTOR1].busy;
    if (channel == STEPPER_CH_MOTOR2) return g_ch[STEPPER_CH_MOTOR2].busy;
    return g_ch[STEPPER_CH_MOTOR1].busy || g_ch[STEPPER_CH_MOTOR2].busy;
}

void StepperTimer_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_htim3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    /* Decrement remaining pulses for each busy channel, stop when done */
    for (uint8_t i = 0; i < 2; i++) {
        if (g_ch[i].busy) {
            if (g_ch[i].remaining == 0) {
                /* Already done — stop PWM and clear busy immediately */
                g_ch[i].busy = 0;
                HAL_TIM_PWM_Stop(&g_htim3, g_ch[i].channel);
            } else {
                g_ch[i].remaining--;
                if (g_ch[i].remaining == 0) {
                    g_ch[i].busy = 0;
                    HAL_TIM_PWM_Stop(&g_htim3, g_ch[i].channel);
                }
            }
        }
    }
    if (!g_ch[STEPPER_CH_MOTOR1].busy && !g_ch[STEPPER_CH_MOTOR2].busy) {
        __HAL_TIM_DISABLE_IT(&g_htim3, TIM_IT_UPDATE);
    }
}
