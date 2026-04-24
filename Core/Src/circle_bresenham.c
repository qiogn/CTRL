/**
  ******************************************************************************
  * @file    circle_bresenham.c
  * @brief   Bresenham circle for dual-axis stepper motors
  *
  *  Uses standard first-octant Bresenham (x always +1, y may -1).
  *  Maps the canonical step to motor coordinates for each of 8 octants.
  *
  *  Starting from (0, R), moves clockwise around the circle.
  *  Each Iterate() produces one motor step (dx, dy) where |dx|<=1, |dy|<=1.
  ******************************************************************************
  */

#include "circle_bresenham.h"

/* Per-octant major/minor motor step directions.
 *
 *  Motor step = major_dir + (minor_dir when decision >= 0)
 *
 *  Octant  Angular range    |ΔX| vs |ΔY|  Major dir  Minor dir
 *  0       (0,R)→(R/√2,R/√2)  |ΔX|>|ΔY|   (+1, 0)    (0,-1)
 *  1       (R/√2,R/√2)→(R,0)  |ΔY|>|ΔX|   (0,-1)    (+1, 0)
 *  2       (R,0)→(R/√2,-R/√2) |ΔY|>|ΔX|   (0,-1)   (-1, 0)
 *  3       (R/√2,-R/√2)→(0,-R)|ΔX|>|ΔY|  (-1, 0)    (0,-1)
 *  4       (0,-R)→(-R/√2,-R/√2)|ΔX|>|ΔY| (-1, 0)    (0,+1)
 *  5       (-R/√2,-R/√2)→(-R,0)|ΔY|>|ΔX| (0,+1)   (-1, 0)
 *  6       (-R,0)→(-R/√2,R/√2)|ΔY|>|ΔX|  (0,+1)   (+1, 0)
 *  7       (-R/√2,R/√2)→(0,R) |ΔX|>|ΔY|  (+1, 0)    (0,+1)
 */
static const int8_t MAJ_DX[8] = { 1,  0,  0, -1, -1,  0,  0,  1};
static const int8_t MAJ_DY[8] = { 0, -1, -1,  0,  0,  1,  1,  0};
static const int8_t MIN_DX[8] = { 0,  1, -1,  0,  0, -1,  1,  0};
static const int8_t MIN_DY[8] = {-1,  0,  0, -1,  1,  0,  0,  1};

void CircleBresenham_Init(CircleState_t *s, int16_t cx, int16_t cy, uint16_t radius, uint8_t count)
{
    s->cx = cx;
    s->cy = cy;
    s->radius = (int16_t)radius;
    /* Standard first-octant coordinates: start at (0, R) */
    s->cur_x = 0;
    s->cur_y = (int16_t)radius;
    s->decision = 3 - 2 * (int16_t)radius;
    s->octant = 0;              /* current octant 0..7 */
    s->step_total = 0;
    s->circles_done = 0;
    s->finished = 0;
    s->step_per_circle = 8U * (uint32_t)radius;
}

void CircleBresenham_Iterate(CircleState_t *s, int16_t *dx, int16_t *dy)
{
    if (s->finished) { *dx = 0; *dy = 0; return; }

    uint8_t o = s->octant;

    /* --- Standard first-octant Bresenham step --- */
    s->cur_x++;  /* major axis (X) always advances */

    if (s->decision >= 0) {
        /* Minor axis (Y) also steps */
        s->decision += 4 * (s->cur_x - s->cur_y) + 10;
        s->cur_y--;
        /* Motor step = major + minor direction */
        *dx = (int16_t)(MAJ_DX[o] + MIN_DX[o]);
        *dy = (int16_t)(MAJ_DY[o] + MIN_DY[o]);
    } else {
        /* Minor axis does NOT step */
        s->decision += 4 * s->cur_x + 6;
        /* Motor step = major direction only */
        *dx = (int16_t)MAJ_DX[o];
        *dy = (int16_t)MAJ_DY[o];
    }

    /* --- Check octant boundary (crossed 45° line) --- */
    if (s->cur_x > s->cur_y) {
        /* Reset Bresenham state for next octant */
        s->decision = 3 - 2 * (int16_t)s->radius;
        s->cur_x = 0;
        s->cur_y = (int16_t)s->radius;
        s->octant++;
        if (s->octant >= 8) {
            s->octant = 0;
            s->circles_done++;
            if (s->circles_done >= CIRCLE_COUNT)
                s->finished = 1;
        }
    }

    s->step_total++;
}
