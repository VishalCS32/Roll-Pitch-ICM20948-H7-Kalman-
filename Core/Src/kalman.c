/* kalman.c */
#include "kalman.h"
#include "filters.h"
#include <math.h>
#include <string.h>

static void kalman_update_single(kalman_state_t *state, float gyro_rate, float accel_angle, float dt, float r_measure);
static float calculate_roll(filtered_axises *accel);
static float calculate_pitch(filtered_axises *accel);

void kalman_init(kalman_filter_t *filter)
{
    // Initialize roll filter
    filter->roll.angle = 0.0f;
    filter->roll.bias = 0.0f;
    filter->roll.P[0][0] = 0.0f;
    filter->roll.P[0][1] = 0.0f;
    filter->roll.P[1][0] = 0.0f;
    filter->roll.P[1][1] = 0.0f;

    // Initialize pitch filter
    filter->pitch.angle = 0.0f;
    filter->pitch.bias = 0.0f;
    filter->pitch.P[0][0] = 0.0f;
    filter->pitch.P[0][1] = 0.0f;
    filter->pitch.P[1][0] = 0.0f;
    filter->pitch.P[1][1] = 0.0f;
}

void kalman_update(kalman_filter_t *filter, filtered_axises *gyro, filtered_axises *accel, float *roll, float *pitch)
{
    static uint32_t stationary_count = 0;
    static float gyro_bias_sum[2] = {0.0f, 0.0f}; // For roll (gyro->y) and pitch (gyro->x)
    float dt = 1.0f / SAMPLE_RATE_HZ;
    float accel_roll = calculate_roll(accel);
    float accel_pitch = calculate_pitch(accel);
    float accel_magnitude = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float r_measure = KALMAN_R_MEASURE;

    // Adjust r_measure dynamically based on accelerometer magnitude
    if (fabsf(accel_magnitude - 1.0f) > 0.02f) {
        r_measure *= (1.0f + 1.0f * fabsf(accel_magnitude - 1.0f)); // Softer scaling
    }

    // Detect stationary condition for bias and angle correction
    float gyro_magnitude = sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);
    if (gyro_magnitude < 0.15f && fabsf(accel_magnitude - 1.0f) < 0.015f) {
        stationary_count++;
        gyro_bias_sum[0] += gyro->y; // Roll uses gyro->y
        gyro_bias_sum[1] += -gyro->x; // Pitch uses -gyro->x
        if (stationary_count >= 20) { // ~0.036s at 550 Hz
            // Update bias with EMA
            float alpha = 0.3f; // Aggressive bias correction
            filter->roll.bias = alpha * (gyro_bias_sum[0] / stationary_count) + (1.0f - alpha) * filter->roll.bias;
            filter->pitch.bias = alpha * (gyro_bias_sum[1] / stationary_count) + (1.0f - alpha) * filter->pitch.bias;
            // Reset angles toward accelerometer values
            filter->roll.angle = 0.7f * filter->roll.angle + 0.3f * accel_roll;
            filter->pitch.angle = 0.7f * filter->pitch.angle + 0.3f * accel_pitch;
            stationary_count = 0;
            gyro_bias_sum[0] = 0.0f;
            gyro_bias_sum[1] = 0.0f;
        }
    } else {
        stationary_count = 0;
        gyro_bias_sum[0] = 0.0f;
        gyro_bias_sum[1] = 0.0f;
    }

    kalman_update_single(&filter->roll, gyro->y, accel_roll, dt, r_measure);
    kalman_update_single(&filter->pitch, -gyro->x, accel_pitch, dt, r_measure);
    *roll = filter->roll.angle;
    *pitch = filter->pitch.angle;
}

static void kalman_update_single(kalman_state_t *state, float gyro_rate, float accel_angle, float dt, float r_measure)
{
    // Predict step
    float angle_pred = state->angle + dt * (gyro_rate - state->bias);
    state->P[0][0] += dt * (dt * state->P[1][1] - state->P[0][1] - state->P[1][0] + KALMAN_Q_ANGLE);
    state->P[0][1] -= dt * state->P[1][1];
    state->P[1][0] -= dt * state->P[1][1];
    state->P[1][1] += KALMAN_Q_BIAS * dt;

    // Update step
    float innovation = accel_angle - angle_pred;
    float S = state->P[0][0] + r_measure;
    float K[2];
    K[0] = state->P[0][0] / S;
    K[1] = state->P[1][0] / S;

    // Update state
    state->angle = angle_pred + K[0] * innovation;
    state->bias += K[1] * innovation;

    // Update covariance matrix
    float P00_temp = state->P[0][0];
    float P01_temp = state->P[0][1];
    state->P[0][0] -= K[0] * P00_temp;
    state->P[0][1] -= K[0] * P01_temp;
    state->P[1][0] -= K[1] * P00_temp;
    state->P[1][1] -= K[1] * P01_temp;
}

static float calculate_roll(filtered_axises *accel)
{
    return atan2f(accel->y, sqrtf(accel->z * accel->z + accel->x * accel->x)) * 180.0f / M_PI;
}

static float calculate_pitch(filtered_axises *accel)
{
    return atan2f(-accel->x, accel->z) * 180.0f / M_PI;
}
