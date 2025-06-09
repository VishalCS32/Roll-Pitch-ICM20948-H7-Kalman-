/* filters.c */
#include "filters.h"
#include "icm20948.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart6;
extern char uart_buf[200];

static float apply_pt1_filter(float input, float *state, float cutoff_hz);
static float apply_pt2_filter(float input, float *state, float cutoff_hz);

void filters_init(filter_state_t *state)
{
    memset(state, 0, sizeof(filter_state_t));
    state->is_calibrated = 0;
}

void filters_calibrate_gyro(filter_state_t *state)
{
    axises temp;
    float sum[3] = {0};
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        icm20948_gyro_read_dps(&temp);
        sum[0] += temp.x;
        sum[1] += temp.y;
        sum[2] += temp.z;
        HAL_Delay(2);
    }
    state->gyro_bias[0] = sum[0] / CALIBRATION_SAMPLES;
    state->gyro_bias[1] = sum[1] / CALIBRATION_SAMPLES;
    state->gyro_bias[2] = sum[2] / CALIBRATION_SAMPLES;
    snprintf(uart_buf, sizeof(uart_buf), "Gyro Bias Sum: X=%.2f, Y=%.2f, Z=%.2f\r\n",
             sum[0], sum[1], sum[2]);
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);
    state->is_calibrated |= 0x01;
}

void filters_calibrate_accel(filter_state_t *state)
{
    axises temp;
    float sum[3] = {0};
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        icm20948_accel_read_g(&temp);
        sum[0] += temp.x;
        sum[1] += temp.y;
        sum[2] += (temp.z - 1.0f);
        HAL_Delay(2);
    }
    state->accel_bias[0] = sum[0] / CALIBRATION_SAMPLES;
    state->accel_bias[1] = sum[1] / CALIBRATION_SAMPLES;
    state->accel_bias[2] = sum[2] / CALIBRATION_SAMPLES;
    state->is_calibrated |= 0x02;
}

void filters_apply_gyro(filtered_axises *gyro_data, filter_state_t *state)
{
    static uint8_t print_counter = 0;
    const uint8_t print_interval = 70;
    axises raw_data;

    print_counter++;
    if (print_counter >= print_interval) {
        snprintf(uart_buf, sizeof(uart_buf), "\r\n--- Gyro Data ---\r\n");
        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);

        icm20948_gyro_read(&raw_data);
        snprintf(uart_buf, sizeof(uart_buf), "Raw Gyro ADC | X: %6d | Y: %6d | Z: %6d\r\n",
                 (int)raw_data.x, (int)raw_data.y, (int)raw_data.z);
        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);

        icm20948_gyro_read_dps(&raw_data);
        if (isnan(raw_data.x) || isnan(raw_data.y) || isnan(raw_data.z)) {
            snprintf(uart_buf, sizeof(uart_buf), "Invalid Raw Gyro DPS | X: %6.2f | Y: %6.2f | Z: %6.2f\r\n",
                     raw_data.x, raw_data.y, raw_data.z);
            HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);
            gyro_data->x = gyro_data->y = gyro_data->z = 0;
            print_counter = 0;
            return;
        }
        snprintf(uart_buf, sizeof(uart_buf), "Raw Gyro DPS | X: %6.2f | Y: %6.2f | Z: %6.2f\r\n",
                 raw_data.x, raw_data.y, raw_data.z);
        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);

        if (state->is_calibrated & 0x01) {
            raw_data.x -= state->gyro_bias[0];
            raw_data.y -= state->gyro_bias[1];
            raw_data.z -= state->gyro_bias[2];
            snprintf(uart_buf, sizeof(uart_buf), "Bias-Corrected | X: %6.2f | Y: %6.2f | Z: %6.2f\r\n",
                     raw_data.x, raw_data.y, raw_data.z);
            HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);
        }

        float mag = sqrtf(raw_data.x * raw_data.x + raw_data.y * raw_data.y + raw_data.z * raw_data.z);
        float pt1_cutoff_hz = PT1_CUTOFF_HZ_MIN + (PT1_CUTOFF_HZ_MAX - PT1_CUTOFF_HZ_MIN) * (mag / 600.0f);
        pt1_cutoff_hz = fmaxf(PT1_CUTOFF_HZ_MIN, fminf(PT1_CUTOFF_HZ_MAX, pt1_cutoff_hz));
        gyro_data->x = apply_pt1_filter(raw_data.x, &state->gyro_pt1_state[0], pt1_cutoff_hz);
        gyro_data->y = apply_pt1_filter(raw_data.y, &state->gyro_pt1_state[1], pt1_cutoff_hz);
        gyro_data->z = apply_pt1_filter(raw_data.z, &state->gyro_pt1_state[2], pt1_cutoff_hz);
        snprintf(uart_buf, sizeof(uart_buf), "PT1 Output   | X: %6.2f | Y: %6.2f | Z: %6.2f | Cutoff: %.1f Hz\r\n",
                 gyro_data->x, gyro_data->y, gyro_data->z, pt1_cutoff_hz);
        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);

        float pt2_cutoff_hz = PT2_CUTOFF_HZ_MIN + (PT2_CUTOFF_HZ_MAX - PT2_CUTOFF_HZ_MIN) * (mag / 600.0f);
        pt2_cutoff_hz = fmaxf(PT2_CUTOFF_HZ_MIN, fminf(PT2_CUTOFF_HZ_MAX, pt2_cutoff_hz));
        gyro_data->x = apply_pt2_filter(gyro_data->x, &state->gyro_pt2_state[0], pt2_cutoff_hz);
        gyro_data->y = apply_pt2_filter(gyro_data->y, &state->gyro_pt2_state[1], pt2_cutoff_hz);
        gyro_data->z = apply_pt2_filter(gyro_data->z, &state->gyro_pt2_state[2], pt2_cutoff_hz);
        snprintf(uart_buf, sizeof(uart_buf), "PT2 Output   | X: %6.2f | Y: %6.2f | Z: %6.2f | Cutoff: %.1f Hz\r\n",
                 gyro_data->x, gyro_data->y, gyro_data->z, pt2_cutoff_hz);
        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);

        print_counter = 0;
    } else {
        icm20948_gyro_read_dps(&raw_data);
        if (isnan(raw_data.x) || isnan(raw_data.y) || isnan(raw_data.z)) {
            gyro_data->x = gyro_data->y = gyro_data->z = 0;
            return;
        }
        if (state->is_calibrated & 0x01) {
            raw_data.x -= state->gyro_bias[0];
            raw_data.y -= state->gyro_bias[1];
            raw_data.z -= state->gyro_bias[2];
        }
        float mag = sqrtf(raw_data.x * raw_data.x + raw_data.y * raw_data.y + raw_data.z * raw_data.z);
        float pt1_cutoff_hz = PT1_CUTOFF_HZ_MIN + (PT1_CUTOFF_HZ_MAX - PT1_CUTOFF_HZ_MIN) * (mag / 2000.0f);
        pt1_cutoff_hz = fmaxf(PT1_CUTOFF_HZ_MIN, fminf(PT1_CUTOFF_HZ_MAX, pt1_cutoff_hz));
        gyro_data->x = apply_pt1_filter(raw_data.x, &state->gyro_pt1_state[0], pt1_cutoff_hz);
        gyro_data->y = apply_pt1_filter(raw_data.y, &state->gyro_pt1_state[1], pt1_cutoff_hz);
        gyro_data->z = apply_pt1_filter(raw_data.z, &state->gyro_pt1_state[2], pt1_cutoff_hz);
        float pt2_cutoff_hz = PT2_CUTOFF_HZ_MIN + (PT2_CUTOFF_HZ_MAX - PT2_CUTOFF_HZ_MIN) * (mag / 2000.0f);
        pt2_cutoff_hz = fmaxf(PT2_CUTOFF_HZ_MIN, fminf(PT2_CUTOFF_HZ_MAX, pt2_cutoff_hz));
        gyro_data->x = apply_pt2_filter(gyro_data->x, &state->gyro_pt2_state[0], pt2_cutoff_hz);
        gyro_data->y = apply_pt2_filter(gyro_data->y, &state->gyro_pt2_state[1], pt2_cutoff_hz);
        gyro_data->z = apply_pt2_filter(gyro_data->z, &state->gyro_pt2_state[2], pt2_cutoff_hz);
    }
}

void filters_apply_accel(filtered_axises *accel_data, filter_state_t *state)
{
    axises raw_data;
    icm20948_accel_read_g(&raw_data);
    if (state->is_calibrated & 0x02) {
        accel_data->x = raw_data.x - state->accel_bias[0];
        accel_data->y = raw_data.y - state->accel_bias[1];
        accel_data->z = raw_data.z - state->accel_bias[2];
    } else {
        accel_data->x = raw_data.x;
        accel_data->y = raw_data.y;
        accel_data->z = raw_data.z;
    }
}

static float apply_pt1_filter(float input, float *state, float cutoff_hz)
{
    float RC = 1.0f / (2.0f * M_PI * cutoff_hz);
    float dt = 1.0f / SAMPLE_RATE_HZ;
    float alpha = dt / (RC + dt);
    *state = *state + alpha * (input - *state);
    return *state;
}

static float apply_pt2_filter(float input, float *state, float cutoff_hz)
{
    float RC = 1.0f / (2.0f * M_PI * cutoff_hz);
    float dt = 1.0f / SAMPLE_RATE_HZ;
    float alpha = dt / (RC + dt);
    *state = *state + alpha * (input - *state);
    return *state;
}
