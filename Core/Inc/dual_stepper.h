#ifndef __DUAL_STEPPER_H
#define __DUAL_STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void DualStepper_Init(void);
void DualStepper_SetHoldEnabled(uint8_t enabled);
void DualStepper_SetTrackingCommand(int16_t motor1_cmd, int16_t motor2_cmd);
void DualStepper_TIM2_IRQHandler(void);
void DualStepper_RunByVision(int16_t dx, int16_t dy);
void DualStepper_MoveAxes(int16_t motor1_steps, int16_t motor2_steps, uint16_t pulse_width_us);

#ifdef __cplusplus
}
#endif

#endif /* __DUAL_STEPPER_H */
