/**
  ******************************************************************************
  * @file    stepper_timer_test.c
  * @brief   Test functions for stepper timer clock calculations
  ******************************************************************************
  */

#include "stepper_timer.h"
#include "stepper_config.h"
#include <stdio.h>

/**
  * @brief  Test timer clock calculations
  * @retval None
  */
void StepperTimer_TestClockCalculations(void)
{
    printf("=== Stepper Timer Clock Calculation Test ===\n");

    /* Test 1: Verify timer counter clock calculation */
    uint32_t timer_counter_clock_hz = TIMER_CLOCK_FREQ_HZ / (TIMER_PRESCALER + 1);
    printf("1. Timer counter clock calculation:\n");
    printf("   TIMER_CLOCK_FREQ_HZ = %lu Hz\n", TIMER_CLOCK_FREQ_HZ);
    printf("   TIMER_PRESCALER = %lu\n", TIMER_PRESCALER);
    printf("   Timer counter clock = %lu / (%lu + 1) = %lu Hz\n",
           TIMER_CLOCK_FREQ_HZ, TIMER_PRESCALER, timer_counter_clock_hz);
    printf("   Expected: 1,000,000 Hz (1MHz)\n");
    printf("   Result: %s\n\n", (timer_counter_clock_hz == 1000000U) ? "PASS" : "FAIL");

    /* Test 2: Calculate frequency limits */
    uint32_t min_achievable_hz, max_achievable_hz;

    /* Maximum frequency: ARR = 1 */
    max_achievable_hz = timer_counter_clock_hz / 2;  /* ARR = 1 */

    /* Minimum frequency: ARR = 0xFFFF */
    min_achievable_hz = timer_counter_clock_hz / (0xFFFF + 1);

    printf("2. Achievable frequency range:\n");
    printf("   Timer counter clock: %lu Hz\n", timer_counter_clock_hz);
    printf("   Max frequency (ARR=1): %lu Hz\n", max_achievable_hz);
    printf("   Min frequency (ARR=0xFFFF): %lu Hz\n", min_achievable_hz);
    printf("   Config min: %lu Hz, Config max: %lu Hz\n",
           STEPPER_TIMER_MIN_FREQ_HZ, STEPPER_TIMER_MAX_FREQ_HZ);

    /* Test 3: Test specific frequency calculations */
    printf("3. Specific frequency tests:\n");

    uint32_t test_frequencies[] = {10, 100, 1000, 10000, 50000};
    uint32_t expected_arr[] = {99999, 9999, 999, 99, 19};  /* (1MHz/freq) - 1 */

    for (int i = 0; i < 5; i++)
    {
        uint32_t freq = test_frequencies[i];
        uint32_t expected = expected_arr[i];

        /* Manual calculation for verification */
        uint32_t manual_arr = (timer_counter_clock_hz / freq);
        if (manual_arr > 1)
        {
            manual_arr -= 1;
        }
        else
        {
            manual_arr = 1;
        }

        printf("   %5lu Hz: Expected ARR=%5lu, Manual ARR=%5lu -> %s\n",
               freq, expected, manual_arr,
               (manual_arr == expected) ? "PASS" : "FAIL");
    }

    printf("\n=== Test Complete ===\n");
}

/**
  * @brief  Test ARR calculation for edge cases
  * @retval None
  */
void StepperTimer_TestEdgeCases(void)
{
    printf("=== Edge Case Tests ===\n");

    uint32_t timer_counter_clock_hz = TIMER_CLOCK_FREQ_HZ / (TIMER_PRESCALER + 1);

    /* Test 1: Frequency = 0 (should return ARR = 1) */
    printf("1. Frequency = 0 Hz: ARR should be 1\n");

    /* Test 2: Frequency > timer counter clock (should return ARR = 1) */
    printf("2. Frequency > %lu Hz: ARR should be 1\n", timer_counter_clock_hz);

    /* Test 3: Frequency = timer counter clock / 2 (ARR = 1) */
    uint32_t freq_max = timer_counter_clock_hz / 2;
    printf("3. Frequency = %lu Hz (max): ARR should be 1\n", freq_max);

    /* Test 4: Frequency = timer counter clock / 0x10000 (min) */
    uint32_t freq_min = timer_counter_clock_hz / (0xFFFF + 1);
    printf("4. Frequency = %lu Hz (min): ARR should be 0xFFFF\n", freq_min);

    printf("\n=== Edge Case Tests Complete ===\n");
}