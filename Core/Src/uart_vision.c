#include "uart_vision.h"

VisionData_t vision_data = {0, 0, 0};

void Vision_UART_Init(UART_HandleTypeDef *huart)
{
    (void)huart;
    vision_data.dx = 0;
    vision_data.dy = 0;
    vision_data.updated = 0;
}

void Vision_UART_Process(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void Vision_UART_Recover(UART_HandleTypeDef *huart)
{
    (void)huart;
}