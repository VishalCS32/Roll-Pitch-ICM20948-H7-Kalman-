/* kalman.h */
#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "icm20948.h"
#include "filters.h"
#include <stdint.h>

/* Kalman Filter Configuration */
#define KALMAN_Q_ANGLE 0.02f      // Increased for faster angle updates
#define KALMAN_Q_BIAS 0.003f      // Increased for rapid bias correction
#define KALMAN_R_MEASURE 0.025f    // Decreased to trust gyro more
#define SAMPLE_RATE_HZ 550.0f

typedef struct {
    float angle;
    float bias;
    float P[2][2];
} kalman_state_t;

typedef struct {
    kalman_state_t roll;
    kalman_state_t pitch;
} kalman_filter_t;

void kalman_init(kalman_filter_t *filter);
void kalman_update(kalman_filter_t *filter, filtered_axises *gyro, filtered_axises *accel, float *roll, float *pitch);

#endif /* __KALMAN_H__ */
