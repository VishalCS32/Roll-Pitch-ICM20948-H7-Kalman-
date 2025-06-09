/* filters.h */
#ifndef __FILTERS_H__
#define __FILTERS_H__

#include "icm20948.h"
#include <stdint.h>

#define SAMPLE_RATE_HZ 550.0f
#define PT1_CUTOFF_HZ_MIN 8.0f
#define PT1_CUTOFF_HZ_MAX 50.0f
#define PT2_CUTOFF_HZ_MIN 50.0f
#define PT2_CUTOFF_HZ_MAX 100.0f
#define CALIBRATION_SAMPLES 500

typedef struct {
    uint8_t is_calibrated;
    float gyro_bias[3];
    float accel_bias[3];
    float gyro_pt1_state[3];
    float gyro_pt2_state[3];
} filter_state_t;

typedef axises filtered_axises;

void filters_init(filter_state_t *state);
void filters_calibrate_gyro(filter_state_t *state);
void filters_calibrate_accel(filter_state_t *state);
void filters_apply_gyro(filtered_axises *gyro_data, filter_state_t *state);
void filters_apply_accel(filtered_axises *accel_data, filter_state_t *state);

#endif /* __FILTERS_H__ */
