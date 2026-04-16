#include "dual_stepper.h"

/* 简化版双步进电机驱动 */
/* 只保留最基本的功能：初始化、直接移动 */

#define STEPPER_ENABLE_LEVEL GPIO_PIN_RESET
#define STEPPER_DISABLE_LEVEL GPIO_PIN_SET

#define MOTOR1_DIR_INVERT 1U
#define MOTOR2_DIR_INVERT 0U
#define MOTOR1_SWAP_STEP_DIR 0U
#define MOTOR2_SWAP_STEP_DIR 0U

#define DEFAULT_PULSE_WIDTH_US 950U
#define MIN_PULSE_WIDTH_US 20U
#define MAX_PULSE_WIDTH_US 10000U
#define MOTOR_DIR_SETTLE_US 1300U

static volatile uint8_t g_dual_stepper_hold_enabled = 0U;

static void DualStepper_ConfigOutput(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState initial_level)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(port, pin, initial_level);

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static uint16_t DualStepper_ClampPulseWidth(uint16_t pulse_width_us)
{
    if (pulse_width_us < MIN_PULSE_WIDTH_US)
    {
        return MIN_PULSE_WIDTH_US;
    }

    if (pulse_width_us > MAX_PULSE_WIDTH_US)
    {
        return MAX_PULSE_WIDTH_US;
    }

    return pulse_width_us;
}

/* 简化的延时函数，使用循环计数 */
static void DualStepper_DelayUs(uint16_t microseconds)
{
    /* 简单延时，基于系统时钟 */
    volatile uint32_t count = microseconds * 72; /* 72MHz时钟下的大致值 */
    while (count--) {
        __NOP();
    }
}

static void DualStepper_WritePinFast(GPIO_TypeDef *port, uint16_t pin, uint8_t level_high)
{
    if (level_high != 0U)
    {
        port->BSRR = pin;
    }
    else
    {
        port->BRR = pin;
    }
}

static void DualStepper_SetMotorEnable(GPIO_TypeDef *port, uint16_t pin, uint8_t enable)
{
    DualStepper_WritePinFast(port, pin, (enable != 0U) ? ((STEPPER_ENABLE_LEVEL == GPIO_PIN_SET) ? 1U : 0U)
                                                     : ((STEPPER_DISABLE_LEVEL == GPIO_PIN_SET) ? 1U : 0U));
}

static GPIO_PinState DualStepper_DirectionLevel(uint8_t positive_direction, uint8_t invert)
{
    uint8_t level = positive_direction;

    if (invert != 0U)
    {
        level = (uint8_t)!level;
    }

    return (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static void DualStepper_ResolvePins(GPIO_TypeDef *step_port,
                                    uint16_t step_pin,
                                    GPIO_TypeDef *dir_port,
                                    uint16_t dir_pin,
                                    uint8_t swap_step_dir,
                                    GPIO_TypeDef **run_step_port,
                                    uint16_t *run_step_pin,
                                    GPIO_TypeDef **run_dir_port,
                                    uint16_t *run_dir_pin)
{
    *run_step_port = step_port;
    *run_step_pin = step_pin;
    *run_dir_port = dir_port;
    *run_dir_pin = dir_pin;

    if (swap_step_dir != 0U)
    {
        *run_step_port = dir_port;
        *run_step_pin = dir_pin;
        *run_dir_port = step_port;
        *run_dir_pin = step_pin;
    }
}

/* 初始化函数 - 简化版 */
void DualStepper_Init(void)
{
    /* 禁用定时器中断，我们不使用复杂的中断逻辑 */
    /* 配置TIM2中断优先级（如果启用） */
    /* 设置优先级为7，低于TIM3(6)和UART2(0)，确保不会干扰关键中断 */
    HAL_NVIC_SetPriority(TIM2_IRQn, 7, 0);
    /* 注意：这里不启用TIM2中断，因为我们的实现不使用定时器中断 */
    /* 如果其他地方启用了TIM2中断，至少优先级已正确配置 */

    /* 只需设置GPIO初始状态 */

    /* 禁用两个电机 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    DualStepper_ConfigOutput(EN1_GPIO_Port, EN1_Pin, STEPPER_DISABLE_LEVEL);
    DualStepper_ConfigOutput(Step1_GPIO_Port, Step1_Pin, GPIO_PIN_RESET);
    DualStepper_ConfigOutput(Dir1_GPIO_Port, Dir1_Pin, GPIO_PIN_RESET);
    DualStepper_ConfigOutput(EN2_GPIO_Port, EN2_Pin, STEPPER_DISABLE_LEVEL);
    DualStepper_ConfigOutput(Step2_GPIO_Port, Step2_Pin, GPIO_PIN_RESET);
    DualStepper_ConfigOutput(Dir2_GPIO_Port, Dir2_Pin, GPIO_PIN_RESET);

    DualStepper_SetMotorEnable(EN1_GPIO_Port, EN1_Pin, 0U);
    DualStepper_SetMotorEnable(EN2_GPIO_Port, EN2_Pin, 0U);

    /* 设置步进和方向引脚为低 */
    HAL_GPIO_WritePin(Step1_GPIO_Port, Step1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Dir1_GPIO_Port, Dir1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Step2_GPIO_Port, Step2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Dir2_GPIO_Port, Dir2_Pin, GPIO_PIN_RESET);

    g_dual_stepper_hold_enabled = 1U;
}

/* 保持使能设置 - 简化版 */
void DualStepper_SetHoldEnabled(uint8_t enabled)
{
    g_dual_stepper_hold_enabled = (enabled != 0U) ? 1U : 0U;
}

/* 跟踪命令 - 简化版：直接调用MoveAxes函数 */
void DualStepper_SetTrackingCommand(int16_t motor1_cmd, int16_t motor2_cmd)
{
    /* 简单实现：直接移动，命令值作为步数 */
    DualStepper_MoveAxes(motor1_cmd, motor2_cmd, DEFAULT_PULSE_WIDTH_US);
}

/* 定时器中断处理 - 空实现 */
void DualStepper_TIM2_IRQHandler(void)
{
    /* 不处理定时器中断 */
}

/* 视觉控制移动 - 简化版 */
void DualStepper_RunByVision(int16_t dx, int16_t dy)
{
    /* 简单实现：直接移动 */
    DualStepper_MoveAxes(dx, dy, DEFAULT_PULSE_WIDTH_US);
}

/* 主移动函数 - 保留原实现，但简化依赖 */
void DualStepper_MoveAxes(int16_t motor1_steps, int16_t motor2_steps, uint16_t pulse_width_us)
{
    uint32_t motor1_total = (motor1_steps >= 0) ? (uint32_t)motor1_steps : (uint32_t)(-motor1_steps);
    uint32_t motor2_total = (motor2_steps >= 0) ? (uint32_t)motor2_steps : (uint32_t)(-motor2_steps);
    uint32_t total_cycles = (motor1_total > motor2_total) ? motor1_total : motor2_total;
    uint32_t accumulator1 = 0U;
    uint32_t accumulator2 = 0U;
    uint32_t cycle = 0U;
    GPIO_PinState motor1_dir;
    GPIO_PinState motor2_dir;
    GPIO_TypeDef *motor1_step_port = NULL;
    GPIO_TypeDef *motor1_dir_port = NULL;
    GPIO_TypeDef *motor2_step_port = NULL;
    GPIO_TypeDef *motor2_dir_port = NULL;
    uint16_t motor1_step_pin = 0U;
    uint16_t motor1_dir_pin = 0U;
    uint16_t motor2_step_pin = 0U;
    uint16_t motor2_dir_pin = 0U;
    pulse_width_us = DualStepper_ClampPulseWidth(pulse_width_us);
    uint16_t half_pulse = (uint16_t)(pulse_width_us / 2U);
    if (half_pulse == 0U)
    {
        half_pulse = 1U;
    }

    motor1_dir = DualStepper_DirectionLevel((motor1_steps >= 0) ? 1U : 0U, MOTOR1_DIR_INVERT);
    motor2_dir = DualStepper_DirectionLevel((motor2_steps >= 0) ? 1U : 0U, MOTOR2_DIR_INVERT);

    DualStepper_ResolvePins(Step1_GPIO_Port, Step1_Pin,
                            Dir1_GPIO_Port, Dir1_Pin,
                            MOTOR1_SWAP_STEP_DIR,
                            &motor1_step_port, &motor1_step_pin,
                            &motor1_dir_port, &motor1_dir_pin);

    DualStepper_ResolvePins(Step2_GPIO_Port, Step2_Pin,
                            Dir2_GPIO_Port, Dir2_Pin,
                            MOTOR2_SWAP_STEP_DIR,
                            &motor2_step_port, &motor2_step_pin,
                            &motor2_dir_port, &motor2_dir_pin);

    DualStepper_WritePinFast(motor1_dir_port, motor1_dir_pin, (motor1_dir == GPIO_PIN_SET) ? 1U : 0U);
    DualStepper_WritePinFast(motor2_dir_port, motor2_dir_pin, (motor2_dir == GPIO_PIN_SET) ? 1U : 0U);
    DualStepper_DelayUs(MOTOR_DIR_SETTLE_US);

    if (g_dual_stepper_hold_enabled != 0U)
    {
        DualStepper_SetMotorEnable(EN1_GPIO_Port, EN1_Pin, 1U);
        DualStepper_SetMotorEnable(EN2_GPIO_Port, EN2_Pin, 1U);
    }
    else
    {
        DualStepper_SetMotorEnable(EN1_GPIO_Port, EN1_Pin, (motor1_total > 0U) ? 1U : 0U);
        DualStepper_SetMotorEnable(EN2_GPIO_Port, EN2_Pin, (motor2_total > 0U) ? 1U : 0U);
    }

    if (total_cycles == 0U)
    {
        return;
    }

    for (cycle = 0U; cycle < total_cycles; ++cycle)
    {
        uint8_t pulse_motor1 = 0U;
        uint8_t pulse_motor2 = 0U;

        accumulator1 += motor1_total;
        accumulator2 += motor2_total;

        if (accumulator1 >= total_cycles)
        {
            accumulator1 -= total_cycles;
            pulse_motor1 = 1U;
            DualStepper_WritePinFast(motor1_step_port, motor1_step_pin, 1U);
        }

        if (accumulator2 >= total_cycles)
        {
            accumulator2 -= total_cycles;
            pulse_motor2 = 1U;
            DualStepper_WritePinFast(motor2_step_port, motor2_step_pin, 1U);
        }

        DualStepper_DelayUs(half_pulse);

        if (pulse_motor1 != 0U)
        {
            DualStepper_WritePinFast(motor1_step_port, motor1_step_pin, 0U);
        }

        if (pulse_motor2 != 0U)
        {
            DualStepper_WritePinFast(motor2_step_port, motor2_step_pin, 0U);
        }

        DualStepper_DelayUs(half_pulse);
    }

    if (g_dual_stepper_hold_enabled == 0U)
    {
        DualStepper_SetMotorEnable(EN1_GPIO_Port, EN1_Pin, 0U);
        DualStepper_SetMotorEnable(EN2_GPIO_Port, EN2_Pin, 0U);
    }
}
