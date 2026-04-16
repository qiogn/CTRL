/**
  ******************************************************************************
  * @file    peripheral_lib.h
  * @brief   Peripheral configuration library for STM32
  *          Provides K210-style abstraction for GPIO and peripheral configuration
  ******************************************************************************
  */

#ifndef __PERIPHERAL_LIB_H
#define __PERIPHERAL_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  GPIO function enumeration (similar to K210 FPIOA functions)
  */
typedef enum {
    GPIO_FUNC_GPIO = 0,      /* General purpose input/output */
    GPIO_FUNC_UART_TX,       /* UART transmit */
    GPIO_FUNC_UART_RX,       /* UART receive */
    GPIO_FUNC_STEPPER_STEP,  /* Stepper motor step signal */
    GPIO_FUNC_STEPPER_DIR,   /* Stepper motor direction signal */
    GPIO_FUNC_STEPPER_EN,    /* Stepper motor enable signal */
    GPIO_FUNC_LASER,         /* Laser control signal */
    GPIO_FUNC_LED,           /* LED indicator */
} GPIO_Function_t;

/**
  * @brief  UART port enumeration
  */
typedef enum {
    UART_PORT_1 = 0,
    UART_PORT_2,
    UART_PORT_3,
} UART_Port_t;

/**
  * @brief  Motor axis enumeration
  */
typedef enum {
    MOTOR_AXIS_1 = 0,    /* Upper motor (vertical axis) */
    MOTOR_AXIS_2,        /* Lower motor (horizontal axis) */
} Motor_Axis_t;

/* Exported constants --------------------------------------------------------*/

/* Default configuration values */
#define DEFAULT_UART_BAUDRATE  115200
#define DEFAULT_PULSE_WIDTH_US 950

/* Exported functions --------------------------------------------------------*/

/* GPIO configuration functions */
void peripheral_gpio_config(uint16_t pin, GPIO_TypeDef* port, GPIO_Function_t function);
void peripheral_gpio_write(uint16_t pin, GPIO_TypeDef* port, uint8_t value);
uint8_t peripheral_gpio_read(uint16_t pin, GPIO_TypeDef* port);

/* UART configuration functions */
void peripheral_uart_init(UART_Port_t uart_port, uint32_t baudrate);
void peripheral_uart_send(uint8_t* data, uint16_t length);
uint16_t peripheral_uart_receive(uint8_t* buffer, uint16_t max_length, uint32_t timeout);
UART_HandleTypeDef* peripheral_get_uart_handle(UART_Port_t uart_port);

/* Motor control functions */
void peripheral_motor_init(Motor_Axis_t axis);
void peripheral_motor_move(Motor_Axis_t axis, int16_t steps, uint16_t pulse_width_us);
void peripheral_motor_enable(Motor_Axis_t axis, uint8_t enable);

/* Laser control functions */
void peripheral_laser_init(void);
void peripheral_laser_on(void);
void peripheral_laser_off(void);

/* System initialization */
void peripheral_system_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPHERAL_LIB_H */