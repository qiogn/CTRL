#ifndef __UART_VISION_H
#define __UART_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
    int16_t dx;
    int16_t dy;
    uint8_t updated;
} VisionData_t;

extern VisionData_t vision_data;

void Vision_UART_Init(UART_HandleTypeDef *huart);
void Vision_UART_Process(UART_HandleTypeDef *huart);
void Vision_UART_Recover(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __UART_VISION_H */