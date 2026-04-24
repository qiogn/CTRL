/**
  ******************************************************************************
  * @file    stepper_test.c
  * @brief   Test functions for stepper motor driver
  ******************************************************************************
  */

#include "stepper_test.h"
#include "stepper_timer.h"
#include "stepper_controller.h"
#include "peripheral_lib.h"
#include "stepper_config.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static uint8_t g_test_running = 0;
static uint32_t g_test_step_count = 0;

/* Private function prototypes -----------------------------------------------*/
static void test_delay_ms(uint32_t ms);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Run basic stepper motor test
  * @retval None
  */
void StepperTest_RunBasicTest(void)
{
    printf("Starting stepper motor basic test...\r\n");

    /* Initialize stepper controller */
    if (StepperCtrl_Init() != STEPPER_CTRL_OK)
    {
        printf("Error: Failed to initialize stepper controller\r\n");
        return;
    }

    printf("Stepper controller initialized successfully\r\n");

    /* Test motor 1 */
    printf("Testing Motor 1 (Vertical Axis)...\r\n");
    peripheral_motor_enable(MOTOR_AXIS_1, 1);

    /* Move forward 100 steps */
    printf("Moving forward 100 steps...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, 100);

    /* Wait for movement to complete */
    uint8_t is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Movement completed\r\n");
    test_delay_ms(500);

    /* Move backward 100 steps */
    printf("Moving backward 100 steps...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, -100);

    /* Wait for movement to complete */
    is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Movement completed\r\n");
    peripheral_motor_enable(MOTOR_AXIS_1, 0);
    test_delay_ms(500);

    /* Test motor 2 */
    printf("Testing Motor 2 (Horizontal Axis)...\r\n");
    peripheral_motor_enable(MOTOR_AXIS_2, 1);

    /* Move forward 100 steps */
    printf("Moving forward 100 steps...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_2, 100);

    /* Wait for movement to complete */
    is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_2, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Movement completed\r\n");
    test_delay_ms(500);

    /* Move backward 100 steps */
    printf("Moving backward 100 steps...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_2, -100);

    /* Wait for movement to complete */
    is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_2, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Movement completed\r\n");
    peripheral_motor_enable(MOTOR_AXIS_2, 0);

    printf("Basic test completed successfully\r\n");
}

/**
  * @brief  Run speed test
  * @retval None
  */
void StepperTest_RunSpeedTest(void)
{
    printf("Starting speed test...\r\n");

    /* Initialize stepper controller */
    if (StepperCtrl_Init() != STEPPER_CTRL_OK)
    {
        printf("Error: Failed to initialize stepper controller\r\n");
        return;
    }

    peripheral_motor_enable(MOTOR_AXIS_1, 1);

    /* Test different speeds */
    uint32_t test_speeds[] = {100, 500, 1000, 2000, 5000};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        printf("Testing speed: %lu Hz\r\n", test_speeds[i]);

        /* Set speed */
        StepperCtrl_SetSpeed(STEPPER_AXIS_1, test_speeds[i]);

        /* Move 200 steps */
        StepperCtrl_MoveRelative(STEPPER_AXIS_1, 200);

        /* Wait for movement to complete */
        uint8_t is_moving = 1;
        while (is_moving)
        {
            StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
            StepperCtrl_Update();
            test_delay_ms(10);
        }

        test_delay_ms(300);

        /* Move back */
        StepperCtrl_MoveRelative(STEPPER_AXIS_1, -200);

        /* Wait for movement to complete */
        is_moving = 1;
        while (is_moving)
        {
            StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
            StepperCtrl_Update();
            test_delay_ms(10);
        }

        test_delay_ms(300);
    }

    peripheral_motor_enable(MOTOR_AXIS_1, 0);
    printf("Speed test completed\r\n");
}

/**
  * @brief  Run acceleration test
  * @retval None
  */
void StepperTest_RunAccelerationTest(void)
{
    printf("Starting acceleration test...\r\n");

    /* Initialize stepper controller */
    if (StepperCtrl_Init() != STEPPER_CTRL_OK)
    {
        printf("Error: Failed to initialize stepper controller\r\n");
        return;
    }

    /* Configure trapezoidal profile */
    MotionProfile_Config_t profile_config = {
        .max_speed_hz = 5000,
        .start_speed_hz = 100,
        .acceleration_hz_s = 5000,
        .deceleration_hz_s = 5000,
        .profile_type = PROFILE_TRAPEZOIDAL
    };

    StepperCtrl_ConfigureProfile(STEPPER_AXIS_1, &profile_config);

    peripheral_motor_enable(MOTOR_AXIS_1, 1);

    /* Move with acceleration */
    printf("Moving 1000 steps with acceleration...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, 1000);

    /* Monitor movement */
    uint8_t is_moving = 1;
    uint32_t last_speed = 0;

    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);

        /* Get current speed */
        uint32_t current_speed = 0;
        StepperCtrl_GetSpeed(STEPPER_AXIS_1, &current_speed);

        if (current_speed != last_speed)
        {
            printf("Current speed: %lu Hz\r\n", current_speed);
            last_speed = current_speed;
        }

        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Movement completed\r\n");
    test_delay_ms(500);

    /* Move back */
    printf("Moving back 1000 steps with deceleration...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, -1000);

    /* Wait for movement to complete */
    is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Movement completed\r\n");
    peripheral_motor_enable(MOTOR_AXIS_1, 0);

    printf("Acceleration test completed\r\n");
}

/**
  * @brief  Run position accuracy test
  * @retval None
  */
void StepperTest_RunPositionTest(void)
{
    printf("Starting position accuracy test...\r\n");

    /* Initialize stepper controller */
    if (StepperCtrl_Init() != STEPPER_CTRL_OK)
    {
        printf("Error: Failed to initialize stepper controller\r\n");
        return;
    }

    /* Reset position */
    StepperCtrl_ResetPosition(STEPPER_AXIS_1);

    peripheral_motor_enable(MOTOR_AXIS_1, 1);

    /* Test position accuracy */
    int32_t test_positions[] = {100, 200, 300, 400, 500};
    uint8_t num_positions = sizeof(test_positions) / sizeof(test_positions[0]);

    for (uint8_t i = 0; i < num_positions; i++)
    {
        printf("Moving to position: %ld\r\n", test_positions[i]);

        /* Move to position */
        StepperCtrl_MoveToPosition(STEPPER_AXIS_1, test_positions[i]);

        /* Wait for movement to complete */
        uint8_t is_moving = 1;
        while (is_moving)
        {
            StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
            StepperCtrl_Update();
            test_delay_ms(10);
        }

        /* Get actual position */
        int32_t actual_position = 0;
        StepperCtrl_GetPosition(STEPPER_AXIS_1, &actual_position);

        printf("Target: %ld, Actual: %ld, Error: %ld\r\n",
               test_positions[i], actual_position,
               test_positions[i] - actual_position);

        test_delay_ms(300);
    }

    /* Return to zero */
    printf("Returning to zero...\r\n");
    StepperCtrl_MoveToPosition(STEPPER_AXIS_1, 0);

    uint8_t is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Position test completed\r\n");
    peripheral_motor_enable(MOTOR_AXIS_1, 0);
}

/**
  * @brief  Run emergency stop test
  * @retval None
  */
void StepperTest_RunEmergencyStopTest(void)
{
    printf("Starting emergency stop test...\r\n");
    printf("WARNING: Motor will stop abruptly!\r\n");

    /* Initialize stepper controller */
    if (StepperCtrl_Init() != STEPPER_CTRL_OK)
    {
        printf("Error: Failed to initialize stepper controller\r\n");
        return;
    }

    peripheral_motor_enable(MOTOR_AXIS_1, 1);

    /* Start movement */
    printf("Starting movement...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, 1000);

    /* Wait a bit */
    test_delay_ms(100);

    /* Emergency stop */
    printf("Emergency stop!\r\n");
    StepperCtrl_EmergencyStop();

    printf("Motor stopped\r\n");
    test_delay_ms(1000);

    /* Try to move again */
    printf("Testing if motor can move again...\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, 100);

    uint8_t is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Emergency stop test completed\r\n");
    peripheral_motor_enable(MOTOR_AXIS_1, 0);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Run direction control test
  * @retval None
  */
void StepperTest_RunDirectionTest(void)
{
    printf("Starting direction control test...\r\n");

    /* Initialize stepper controller */
    if (StepperCtrl_Init() != STEPPER_CTRL_OK)
    {
        printf("Error: Failed to initialize stepper controller\r\n");
        return;
    }

    printf("Testing Motor 1 direction control...\r\n");
    peripheral_motor_enable(MOTOR_AXIS_1, 1);

    /* Test forward direction */
    printf("Setting direction: FORWARD\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, 10);  /* Small movement to test direction */

    /* Wait for movement to complete */
    uint8_t is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Forward movement completed\r\n");
    test_delay_ms(500);

    /* Test reverse direction */
    printf("Setting direction: REVERSE\r\n");
    StepperCtrl_MoveRelative(STEPPER_AXIS_1, -10);  /* Small movement to test direction */

    /* Wait for movement to complete */
    is_moving = 1;
    while (is_moving)
    {
        StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
        StepperCtrl_Update();
        test_delay_ms(10);
    }

    printf("Reverse movement completed\r\n");
    test_delay_ms(500);

    /* Test multiple direction changes */
    printf("Testing multiple direction changes...\r\n");
    for (int i = 0; i < 5; i++)
    {
        printf("Cycle %d: ", i + 1);

        /* Forward */
        printf("F");
        StepperCtrl_MoveRelative(STEPPER_AXIS_1, 5);
        is_moving = 1;
        while (is_moving)
        {
            StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
            StepperCtrl_Update();
            test_delay_ms(1);
        }

        /* Reverse */
        printf("R");
        StepperCtrl_MoveRelative(STEPPER_AXIS_1, -5);
        is_moving = 1;
        while (is_moving)
        {
            StepperCtrl_IsMoving(STEPPER_AXIS_1, &is_moving);
            StepperCtrl_Update();
            test_delay_ms(1);
        }

        printf(" ");
    }
    printf("\r\n");

    printf("Direction control test completed\r\n");
    peripheral_motor_enable(MOTOR_AXIS_1, 0);
}

/**
  * @brief  Simple delay function
  * @param  ms: Delay in milliseconds
  * @retval None
  */
static void test_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}