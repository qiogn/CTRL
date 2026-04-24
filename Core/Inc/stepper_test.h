/**
  ******************************************************************************
  * @file    stepper_test.h
  * @brief   Test functions for stepper motor driver
  ******************************************************************************
  */

#ifndef __STEPPER_TEST_H
#define __STEPPER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Run basic stepper motor test
  * @retval None
  */
void StepperTest_RunBasicTest(void);

/**
  * @brief  Run speed test
  * @retval None
  */
void StepperTest_RunSpeedTest(void);

/**
  * @brief  Run acceleration test
  * @retval None
  */
void StepperTest_RunAccelerationTest(void);

/**
  * @brief  Run position accuracy test
  * @retval None
  */
void StepperTest_RunPositionTest(void);

/**
  * @brief  Run emergency stop test
  * @retval None
  */
void StepperTest_RunEmergencyStopTest(void);

/**
  * @brief  Run direction control test
  * @retval None
  */
void StepperTest_RunDirectionTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_TEST_H */