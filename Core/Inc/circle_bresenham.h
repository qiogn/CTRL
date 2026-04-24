/**
  ******************************************************************************
  * @file    circle_bresenham.h
  * @brief   Bresenham circle algorithm for dual-axis stepper motor control
  *          Integer-only arc interpolation, generates continuous XY step pulses
  ******************************************************************************
  */

#ifndef __CIRCLE_BRESENHAM_H
#define __CIRCLE_BRESENHAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Test parameters — adjust before hardware testing */
#define CIRCLE_RADIUS_STEPS       50U    /* Circle radius in motor steps       */
#define CIRCLE_COUNT              3U     /* Number of complete circles          */
#define CIRCLE_PULSE_WIDTH_US     500U   /* Step pulse width → ~2kHz frequency  */

/* Algorithm state */
typedef struct {
    int16_t cx;          /* Circle center X (motor steps)       */
    int16_t cy;          /* Circle center Y (motor steps)       */
    int16_t radius;      /* Circle radius in steps              */
    int16_t cur_x;       /* Current Bresenham X offset (0..R)   */
    int16_t cur_y;       /* Current Bresenham Y offset (0..R)   */
    int16_t decision;    /* Bresenham decision parameter        */
    uint8_t  octant;     /* Current quadrant index (0..3), reused from octant naming */
    uint32_t step_total; /* Total steps emitted so far          */
    uint32_t step_per_circle; /* Steps in one full circle       */
    uint8_t  circles_done; /* Number of completed circles       */
    uint8_t  finished;   /* 1 = all circles completed           */
} CircleState_t;

/* API */
void CircleBresenham_Init(CircleState_t *s, int16_t cx, int16_t cy, uint16_t radius, uint8_t count);
void CircleBresenham_Iterate(CircleState_t *s, int16_t *dx, int16_t *dy);

#ifdef __cplusplus
}
#endif

#endif /* __CIRCLE_BRESENHAM_H */
