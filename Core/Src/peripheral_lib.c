/**
  ******************************************************************************
  * @file    peripheral_lib.c
  * @brief   Peripheral configuration library implementation
  ******************************************************************************
  */

#include "peripheral_lib.h"
#include "gpio.h"
#include "usart.h"
#include "stepper_timer.h"
#include "stepper_controller.h"
#include "stepper_config.h"

/* Stepper motor control levels - same as dual_stepper.c */
#define STEPPER_ENABLE_LEVEL GPIO_PIN_RESET
#define STEPPER_DISABLE_LEVEL GPIO_PIN_SET

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef* g_uart_handle = NULL;

/* Private function prototypes -----------------------------------------------*/
static void configure_gpio_for_function(uint16_t pin, GPIO_TypeDef* port, GPIO_Function_t function);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Configure GPIO pin for specific function
  * @param  pin: GPIO pin number
  * @param  port: GPIO port
  * @param  function: Desired GPIO function
  * @retval None
  */
void peripheral_gpio_config(uint16_t pin, GPIO_TypeDef* port, GPIO_Function_t function)
{
    /* Enable clock for the GPIO port */
    if (port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }

    configure_gpio_for_function(pin, port, function);
}

/**
  * @brief  Write value to GPIO pin
  * @param  pin: GPIO pin number
  * @param  port: GPIO port
  * @param  value: 0 for low, non-zero for high
  * @retval None
  */
void peripheral_gpio_write(uint16_t pin, GPIO_TypeDef* port, uint8_t value)
{
    HAL_GPIO_WritePin(port, pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  Read value from GPIO pin
  * @param  pin: GPIO pin number
  * @param  port: GPIO port
  * @retval 0 for low, 1 for high
  */
uint8_t peripheral_gpio_read(uint16_t pin, GPIO_TypeDef* port)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0;
}

/**
  * @brief  Initialize UART peripheral
  * @param  uart_port: UART port identifier
  * @param  baudrate: Baud rate in bits per second
  * @retval None
  */
void peripheral_uart_init(UART_Port_t uart_port, uint32_t baudrate)
{
    /* Currently only supports UART2 (PA2/PA3) */
    if (uart_port == UART_PORT_2) {
        /* Use existing MX_USART2_UART_Init function */
        MX_USART2_UART_Init();

        /* Update baudrate if different from default */
        if (baudrate != 115200 && baudrate > 0) {
            huart2.Init.BaudRate = baudrate;
            if (HAL_UART_Init(&huart2) != HAL_OK) {
                Error_Handler();
            }
        }

        g_uart_handle = &huart2;
    }
    /* Add support for other UARTs here */
}

/**
  * @brief  Send data via UART
  * @param  data: Pointer to data buffer
  * @param  length: Number of bytes to send
  * @retval None
  */
void peripheral_uart_send(uint8_t* data, uint16_t length)
{
    if (g_uart_handle != NULL && length > 0) {
        HAL_UART_Transmit(g_uart_handle, data, length, HAL_MAX_DELAY);
    }
}

/**
  * @brief  Receive data via UART
  * @param  buffer: Pointer to receive buffer
  * @param  max_length: Maximum number of bytes to receive
  * @param  timeout: Timeout in milliseconds
  * @retval Number of bytes received
  */
uint16_t peripheral_uart_receive(uint8_t* buffer, uint16_t max_length, uint32_t timeout)
{
    if (g_uart_handle != NULL && max_length > 0) {
        uint16_t received = 0;
        HAL_StatusTypeDef status;

        status = HAL_UART_Receive(g_uart_handle, buffer, max_length, timeout);
        if (status == HAL_OK) {
            received = max_length;
        }

        return received;
    }

    return 0;
}

/**
  * @brief  Get UART handle for direct HAL operations
  * @param  uart_port: UART port identifier
  * @retval Pointer to UART_HandleTypeDef, or NULL if not initialized
  */
UART_HandleTypeDef* peripheral_get_uart_handle(UART_Port_t uart_port)
{
    /* Currently only supports UART2 */
    if (uart_port == UART_PORT_2) {
        return g_uart_handle;
    }
    return NULL;
}

/**
  * @brief  Initialize stepper motor
  * @param  axis: Motor axis identifier
  * @retval None
  */
void peripheral_motor_init(Motor_Axis_t axis)
{
    /* Initialize stepper controller */
    StepperCtrl_Init();

    /* Configure default profile */
    MotionProfile_Config_t profile_config = {
        .max_speed_hz = DEFAULT_SPEED_HZ,
        .start_speed_hz = MIN_SPEED_HZ,
        .acceleration_hz_s = DEFAULT_ACCELERATION_HZ_S,
        .deceleration_hz_s = DEFAULT_DECELERATION_HZ_S,
        .profile_type = PROFILE_TRAPEZOIDAL
    };

    Stepper_Axis_t stepper_axis = (axis == MOTOR_AXIS_1) ? STEPPER_AXIS_1 : STEPPER_AXIS_2;
    StepperCtrl_ConfigureProfile(stepper_axis, &profile_config);

    /* Disable motor initially */
    peripheral_motor_enable(axis, 0);
}

/**
  * @brief  Move stepper motor by specified number of steps (legacy API)
  * @param  axis: Motor axis identifier
  * @param  steps: Number of steps (positive for forward, negative for reverse)
  * @param  pulse_width_us: Pulse width in microseconds (ignored, using timer control)
  * @retval None
  */
void peripheral_motor_move(Motor_Axis_t axis, int16_t steps, uint16_t pulse_width_us)
{
    /* Convert to new API - use default frequency based on pulse width */
    uint32_t frequency_hz = 1000000U / pulse_width_us;  /* Convert us to Hz */

    /* Limit frequency to valid range */
    if (frequency_hz < MIN_SPEED_HZ) {
        frequency_hz = MIN_SPEED_HZ;
    } else if (frequency_hz > MAX_SPEED_HZ) {
        frequency_hz = MAX_SPEED_HZ;
    }

    peripheral_motor_move_smooth(axis, (int32_t)steps, frequency_hz);
}

/**
  * @brief  Move stepper motor with smooth acceleration (new API)
  * @param  axis: Motor axis identifier
  * @param  steps: Number of steps (positive for forward, negative for reverse)
  * @param  speed_hz: Speed in Hz
  * @retval None
  */
void peripheral_motor_move_smooth(Motor_Axis_t axis, int32_t steps, uint32_t speed_hz)
{
    Stepper_Axis_t stepper_axis = (axis == MOTOR_AXIS_1) ? STEPPER_AXIS_1 : STEPPER_AXIS_2;

    /* Set speed */
    StepperCtrl_SetSpeed(stepper_axis, speed_hz);

    /* Enable motor */
    peripheral_motor_enable(axis, 1);

    /* Move relative steps */
    StepperCtrl_MoveRelative(stepper_axis, steps);
}

/**
  * @brief  Enable or disable stepper motor
  * @param  axis: Motor axis identifier
  * @param  enable: 1 to enable, 0 to disable
  * @retval None
  */
void peripheral_motor_enable(Motor_Axis_t axis, uint8_t enable)
{
    /* Enable/disable specific motor axis */
    if (axis == MOTOR_AXIS_1) {
        /* Motor 1: EN1 pin (PB0) */
        HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, enable ? STEPPER_ENABLE_LEVEL : STEPPER_DISABLE_LEVEL);
    } else if (axis == MOTOR_AXIS_2) {
        /* Motor 2: EN2 pin (PA8) */
        HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, enable ? STEPPER_ENABLE_LEVEL : STEPPER_DISABLE_LEVEL);
    }
}

/**
  * @brief  Set motor speed
  * @param  axis: Motor axis identifier
  * @param  speed_hz: Speed in Hz
  * @retval None
  */
void peripheral_motor_set_speed(Motor_Axis_t axis, uint32_t speed_hz)
{
    Stepper_Axis_t stepper_axis = (axis == MOTOR_AXIS_1) ? STEPPER_AXIS_1 : STEPPER_AXIS_2;

    /* Limit speed to valid range */
    if (speed_hz < STEPPER_MIN_FREQUENCY_HZ) {
        speed_hz = STEPPER_MIN_FREQUENCY_HZ;
    } else if (speed_hz > STEPPER_MAX_FREQUENCY_HZ) {
        speed_hz = STEPPER_MAX_FREQUENCY_HZ;
    }

    StepperCtrl_SetSpeed(stepper_axis, speed_hz);
}

/**
  * @brief  Get motor speed
  * @param  axis: Motor axis identifier
  * @retval Current speed in Hz
  */
uint32_t peripheral_motor_get_speed(Motor_Axis_t axis)
{
    Stepper_Axis_t stepper_axis = (axis == MOTOR_AXIS_1) ? STEPPER_AXIS_1 : STEPPER_AXIS_2;
    uint32_t speed_hz = 0;

    StepperCtrl_GetSpeed(stepper_axis, &speed_hz);

    return speed_hz;
}

/**
  * @brief  Stop motor movement
  * @param  axis: Motor axis identifier
  * @retval None
  */
void peripheral_motor_stop(Motor_Axis_t axis)
{
    Stepper_Axis_t stepper_axis = (axis == MOTOR_AXIS_1) ? STEPPER_AXIS_1 : STEPPER_AXIS_2;
    StepperCtrl_Stop(stepper_axis);
}

/**
  * @brief  Stop all motors
  * @retval None
  */
void peripheral_motor_stop_all(void)
{
    StepperCtrl_StopAll();
}

/**
  * @brief  Initialize laser control
  * @retval None
  */
void peripheral_laser_init(void)
{
    /* Laser pin is already configured in MX_GPIO_Init() */
    peripheral_laser_off();  /* Start with laser off */
}

/**
  * @brief  Turn laser on
  * @retval None
  */
void peripheral_laser_on(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Turn laser off
  * @retval None
  */
void peripheral_laser_off(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Initialize all peripherals
  * @retval None
  */
void peripheral_system_init(void)
{
    /* Initialize GPIOs */
    MX_GPIO_Init();

    /* Initialize UART */
    peripheral_uart_init(UART_PORT_2, DEFAULT_UART_BAUDRATE);

    /* Initialize motors */
    peripheral_motor_init(MOTOR_AXIS_1);
    peripheral_motor_init(MOTOR_AXIS_2);

    /* Initialize laser */
    peripheral_laser_init();

    /* Turn off motors initially */
    peripheral_motor_enable(MOTOR_AXIS_1, 0);
    peripheral_motor_enable(MOTOR_AXIS_2, 0);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure GPIO pin based on desired function
  * @param  pin: GPIO pin number
  * @param  port: GPIO port
  * @param  function: Desired GPIO function
  * @retval None
  */
static void configure_gpio_for_function(uint16_t pin, GPIO_TypeDef* port, GPIO_Function_t function)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    switch (function) {
        case GPIO_FUNC_GPIO:
            /* General purpose output */
            GPIO_InitStruct.Pin = pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(port, &GPIO_InitStruct);
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
            break;

        case GPIO_FUNC_UART_TX:
            /* UART transmit - alternate function push-pull */
            GPIO_InitStruct.Pin = pin;
            GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(port, &GPIO_InitStruct);
            break;

        case GPIO_FUNC_UART_RX:
            /* UART receive - input */
            GPIO_InitStruct.Pin = pin;
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(port, &GPIO_InitStruct);
            break;

        case GPIO_FUNC_STEPPER_STEP:
        case GPIO_FUNC_STEPPER_DIR:
        case GPIO_FUNC_STEPPER_EN:
            /* Stepper motor signals - push-pull output */
            GPIO_InitStruct.Pin = pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(port, &GPIO_InitStruct);
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
            break;

        case GPIO_FUNC_LASER:
            /* Laser control - push-pull output */
            GPIO_InitStruct.Pin = pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(port, &GPIO_InitStruct);
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);  /* Start with laser off */
            break;

        case GPIO_FUNC_LED:
            /* LED - open-drain output */
            GPIO_InitStruct.Pin = pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(port, &GPIO_InitStruct);
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);  /* Start with LED off */
            break;

        default:
            /* Default to GPIO output */
            GPIO_InitStruct.Pin = pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(port, &GPIO_InitStruct);
            break;
    }
}