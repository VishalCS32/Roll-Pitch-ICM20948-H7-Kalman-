/*
 * icm20948.c
 *
 *  Created on: Jun 8, 2025
 *      Author: vishal
 */

#include "icm20948.h"
#include <string.h> // For memcpy
#include <stdio.h>  // For snprintf

//char uart_buf[200]; // Shared UART buffer

static float gyro_scale_factor = 16.4f;
static float accel_scale_factor = 2048.0f;
extern UART_HandleTypeDef huart6;

// Non-blocking DMA variables
static volatile uint8_t dma_data_ready = 0; // Flag for read completion
static volatile uint8_t dma_write_complete = 0; // Flag for write completion
static uint8_t dma_rx_buffer[7] = {0}; // Temporary buffer for DMA read (max 6 bytes + 1 dummy)
static uint8_t dma_reg_val[6] = {0}; // Static buffer for read results

/* Static Functions */
static void cs_high();
static void cs_low();

static void select_user_bank(userbank ub);

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg);
static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);
static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);
static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len);

static uint8_t read_single_ak09916_reg(uint8_t reg);
static void write_single_ak09916_reg(uint8_t reg, uint8_t val);
static uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len);

/* Main Functions */

void icm20948_init()
{
    while (!icm20948_who_am_i());
    icm20948_device_reset();
    icm20948_wakeup();
    icm20948_clock_source(1);
    icm20948_odr_align_enable();
    icm20948_spi_slave_enable();
    icm20948_gyro_low_pass_filter(0); // Minimal filtering
    icm20948_accel_low_pass_filter(0);
    icm20948_gyro_sample_rate_divider(1); // 562.5 Hz
    icm20948_accel_sample_rate_divider(1);
    icm20948_gyro_calibration();
    icm20948_accel_calibration();
    icm20948_gyro_full_scale_select(_2000dps);
    icm20948_accel_full_scale_select(_16g);
    snprintf(uart_buf, sizeof(uart_buf), "Gyro Scale: %.2f, Accel Scale: %.2f\r\n",
             gyro_scale_factor, accel_scale_factor);
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);
}

//void icm20948_init()
//{
//    while (!icm20948_who_am_i());
//
//    icm20948_device_reset();
//    icm20948_wakeup();
//
//    icm20948_clock_source(1);
//    icm20948_odr_align_enable();
//
//    icm20948_spi_slave_enable();
//
//    icm20948_gyro_low_pass_filter(0);
//    icm20948_accel_low_pass_filter(0);
//
//    icm20948_gyro_sample_rate_divider(1); // 1125/(1+1) = 562.5Hz
//    icm20948_accel_sample_rate_divider(1); // 1125/(1+1) = 562.5Hz
//
//    icm20948_gyro_calibration();
//    icm20948_accel_calibration();
//
//    icm20948_gyro_full_scale_select(_2000dps);
//    icm20948_accel_full_scale_select(_16g);
//}

void ak09916_init()
{
    icm20948_i2c_master_reset();
    icm20948_i2c_master_enable();
    icm20948_i2c_master_clk_frq(7);

    while (!ak09916_who_am_i());

    ak09916_soft_reset();
    ak09916_operation_mode_setting(continuous_measurement_100hz);
}

void icm20948_gyro_read(axises* data)
{
    uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);

    // Wait for DMA data to be ready
    uint32_t timeout = HAL_GetTick() + 3; // 3ms timeout
    while (!dma_data_ready) {
        if (HAL_GetTick() > timeout) {
            data->x = data->y = data->z = 0; // Timeout error
            return;
        }
    }

    if (temp != NULL) {
        data->x = (int16_t)(temp[0] << 8 | temp[1]);
        data->y = (int16_t)(temp[2] << 8 | temp[3]);
        data->z = (int16_t)(temp[4] << 8 | temp[5]);
    } else {
        data->x = data->y = data->z = 0; // Error handling
    }
}

void icm20948_accel_read(axises* data)
{
    uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

    // Wait for DMA data to be ready
    uint32_t timeout = HAL_GetTick() + 3; // 3ms timeout
    while (!dma_data_ready) {
        if (HAL_GetTick() > timeout) {
            data->x = data->y = data->z = 0; // Timeout error
            return;
        }
    }

    if (temp != NULL) {
        data->x = (int16_t)(temp[0] << 8 | temp[1]);
        data->y = (int16_t)(temp[2] << 8 | temp[3]);
        data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor;
    } else {
        data->x = data->y = data->z = 0; // Error handling
    }
}

bool ak09916_mag_read(axises* data)
{
    uint8_t* temp;
    uint8_t drdy, hofl; // data ready, overflow

    drdy = read_single_ak09916_reg(MAG_ST1) & 0x01;
    if (!drdy) return false;

    temp = read_multiple_ak09916_reg(MAG_HXL, 6);

    // Wait for DMA data to be ready
    uint32_t timeout = HAL_GetTick() + 100; // 100ms timeout
    while (!dma_data_ready) {
        if (HAL_GetTick() > timeout) {
            return false; // Timeout error
        }
    }

    if (temp == NULL) return false;

    hofl = read_single_ak09916_reg(MAG_ST2) & 0x08;
    if (hofl) return false;

    data->x = (int16_t)(temp[1] << 8 | temp[0]);
    data->y = (int16_t)(temp[3] << 8 | temp[2]);
    data->z = (int16_t)(temp[5] << 8 | temp[4]);

    return true;
}

void icm20948_gyro_read_dps(axises* data)
{
    icm20948_gyro_read(data);

    data->x /= gyro_scale_factor;
    data->y /= gyro_scale_factor;
    data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises* data)
{
    icm20948_accel_read(data);

    data->x /= accel_scale_factor;
    data->y /= accel_scale_factor;
    data->z /= accel_scale_factor;
}

bool ak09916_mag_read_uT(axises* data)
{
    axises temp;
    bool new_data = ak09916_mag_read(&temp);
    if (!new_data) return false;

    data->x = (float)(temp.x * 0.15);
    data->y = (float)(temp.y * 0.15);
    data->z = (float)(temp.z * 0.15);

    return true;
}

/* Sub Functions */
bool icm20948_who_am_i()
{
    uint8_t icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);
    return (icm20948_id == ICM20948_ID);
}

bool ak09916_who_am_i()
{
    uint8_t ak09916_id = read_single_ak09916_reg(MAG_WIA2);
    return (ak09916_id == AK09916_ID);
}

void icm20948_device_reset()
{
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
    HAL_Delay(100);
}

void ak09916_soft_reset()
{
    write_single_ak09916_reg(MAG_CNTL3, 0x01);
    HAL_Delay(100);
}

void icm20948_wakeup()
{
    uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
    new_val &= 0xBF;
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
    HAL_Delay(100);
}

void icm20948_sleep()
{
    uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
    new_val |= 0x40;
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
    HAL_Delay(100);
}

void icm20948_spi_slave_enable()
{
    uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
    new_val |= 0x10;
    write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_reset()
{
    uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
    new_val |= 0x02;
    write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
    uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
    new_val |= 0x20;
    write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
    HAL_Delay(100);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
    uint8_t new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
    new_val |= config;
    write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, new_val);
}

void icm20948_clock_source(uint8_t source)
{
    uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
    new_val |= source;
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable()
{
    write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
    uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
    new_val |= config << 3;
    write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
    uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
    new_val |= config << 3;
    write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val); // Note: Potential bug
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
    write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
    uint8_t divider_1 = (uint8_t)(divider >> 8);
    uint8_t divider_2 = (uint8_t)(0x0F & divider);
    write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
    write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

void ak09916_operation_mode_setting(operation_mode mode)
{
    write_single_ak09916_reg(MAG_CNTL2, mode);
    HAL_Delay(100);
}

void icm20948_gyro_calibration()
{
    axises temp;
    int32_t gyro_bias[3] = {0};
    uint8_t gyro_offset[6] = {0};

    for (int i = 0; i < 100; i++)
    {
        icm20948_gyro_read(&temp);
        gyro_bias[0] += temp.x;
        gyro_bias[1] += temp.y;
        gyro_bias[2] += temp.z;
    }

    gyro_bias[0] /= 100;
    gyro_bias[1] /= 100;
    gyro_bias[2] /= 100;

    gyro_offset[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
    gyro_offset[1] = (-gyro_bias[0] / 4) & 0xFF;
    gyro_offset[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset[3] = (-gyro_bias[1] / 4) & 0xFF;
    gyro_offset[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset[5] = (-gyro_bias[2] / 4) & 0xFF;

    write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);

    // Wait for write completion
    uint32_t timeout = HAL_GetTick() + 100;
    while (!dma_write_complete) {
        if (HAL_GetTick() > timeout) {
            return; // Timeout error
        }
    }
}

void icm20948_accel_calibration()
{
    axises temp;
    int32_t accel_bias[3] = {0};
    int32_t accel_bias_reg[3] = {0};
    uint8_t accel_offset[6] = {0};

    for (int i = 0; i < 100; i++)
    {
        icm20948_accel_read(&temp);
        accel_bias[0] += temp.x;
        accel_bias[1] += temp.y;
        accel_bias[2] += temp.z;
    }

    accel_bias[0] /= 100;
    accel_bias[1] /= 100;
    accel_bias[2] /= 100;

    uint8_t mask_bit[3] = {0, 0, 0};

    uint8_t* temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
    uint32_t timeout = HAL_GetTick() + 100;
    while (!dma_data_ready) {
        if (HAL_GetTick() > timeout) {
            return; // Timeout error
        }
    }
    if (temp2 != NULL) {
        accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
        mask_bit[0] = temp2[1] & 0x01;
    }

    uint8_t* temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
    timeout = HAL_GetTick() + 100;
    while (!dma_data_ready) {
        if (HAL_GetTick() > timeout) {
            return; // Timeout error
        }
    }
    if (temp3 != NULL) {
        accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
        mask_bit[1] = temp3[1] & 0x01;
    }

    uint8_t* temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
    timeout = HAL_GetTick() + 100;
    while (!dma_data_ready) {
        if (HAL_GetTick() > timeout) {
            return; // Timeout error
        }
    }
    if (temp4 != NULL) {
        accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
        mask_bit[2] = temp4[1] & 0x01;
    }

    accel_bias_reg[0] -= (accel_bias[0] / 8);
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    accel_offset[1] = (accel_bias_reg[0]) & 0xFE;
    accel_offset[1] = accel_offset[1] | mask_bit[0];

    accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    accel_offset[3] = (accel_bias_reg[1]) & 0xFE;
    accel_offset[3] = accel_offset[3] | mask_bit[1];

    accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    accel_offset[5] = (accel_bias_reg[2]) & 0xFE;
    accel_offset[5] = accel_offset[5] | mask_bit[2];

    write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
    timeout = HAL_GetTick() + 100;
    while (!dma_write_complete) {
        if (HAL_GetTick() > timeout) {
            return; // Timeout error
        }
    }
    write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
    timeout = HAL_GetTick() + 100;
    while (!dma_write_complete) {
        if (HAL_GetTick() > timeout) {
            return; // Timeout error
        }
    }
    write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
    timeout = HAL_GetTick() + 100;
    while (!dma_write_complete) {
        if (HAL_GetTick() > timeout) {
            return; // Timeout error
        }
    }
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
    uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);

    switch (full_scale)
    {
        case _250dps:
            new_val |= 0x00;
            gyro_scale_factor = 131.0;
            break;
        case _500dps:
            new_val |= 0x02;
            gyro_scale_factor = 65.5;
            break;
        case _1000dps:
            new_val |= 0x04;
            gyro_scale_factor = 32.8;
            break;
        case _2000dps:
            new_val |= 0x06;
            gyro_scale_factor = 16.4;
            break;
    }

    write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
    uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);

    switch (full_scale)
    {
        case _2g:
            new_val |= 0x00;
            accel_scale_factor = 16384;
            break;
        case _4g:
            new_val |= 0x02;
            accel_scale_factor = 8192;
            break;
        case _8g:
            new_val |= 0x04;
            accel_scale_factor = 4096;
            break;
        case _16g:
            new_val |= 0x06;
            accel_scale_factor = 2048;
            break;
    }

    write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}

/* Static Functions */
static void cs_high()
{
    HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, RESET);
}

static void select_user_bank(userbank ub)
{
    uint8_t write_reg[2];
    write_reg[0] = WRITE | REG_BANK_SEL;
    write_reg[1] = ub;

    cs_low();
    HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
    cs_high();
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
    uint8_t read_reg = READ | reg;
    uint8_t reg_val;
    select_user_bank(ub);

    cs_low();
    HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
    HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 1000); // Fixed typo: ®_val to reg_val
    cs_high();

    return reg_val;
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
    uint8_t write_reg[2];
    write_reg[0] = WRITE | reg;
    write_reg[1] = val;

    select_user_bank(ub);

    cs_low();
    HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
    cs_high();
}

static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
    uint8_t read_reg = READ | reg;
    HAL_StatusTypeDef status;

    if (len > 6) {
        return NULL; // Buffer overflow protection
    }

    // Reset data ready flag
    dma_data_ready = 0;

    select_user_bank(ub);
    cs_low();

    // Start non-blocking DMA transfer
    status = HAL_SPI_TransmitReceive_DMA(ICM20948_SPI, &read_reg, dma_rx_buffer, len + 1);
    if (status != HAL_OK) {
        cs_high();
        return NULL; // Error handling
    }

    return dma_reg_val; // Data will be available in callback
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
    uint8_t write_buffer[7];
    HAL_StatusTypeDef status;

    if (len > 6) {
        return; // Buffer overflow protection
    }

    // Reset write complete flag
    dma_write_complete = 0;

    write_buffer[0] = WRITE | reg;
    memcpy(&write_buffer[1], val, len);

    select_user_bank(ub);
    cs_low();

    status = HAL_SPI_Transmit_DMA(ICM20948_SPI, write_buffer, len + 1);
    if (status != HAL_OK) {
        cs_high();
        return; // Error handling
    }
}

static uint8_t read_single_ak09916_reg(uint8_t reg)
{
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);

    HAL_Delay(1);
    return read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static void write_single_ak09916_reg(uint8_t reg, uint8_t val)
{
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO, val);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

static uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len)
{
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

    HAL_Delay(1);
    return read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}

/* DMA Callback Functions */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == ICM20948_SPI) {
        char uart_buf[50]; // Local buffer for UART
        cs_high(); // End SPI transaction
        memcpy(dma_reg_val, &dma_rx_buffer[1], 6); // Copy data, skip dummy byte
        dma_data_ready = 1; // Signal data is ready
//        snprintf(uart_buf, sizeof(uart_buf), "SPI2 DMA Read Complete\r\n");
//        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == ICM20948_SPI) {
        char uart_buf[50]; // Local buffer for UART
        cs_high(); // End SPI transaction
        dma_write_complete = 1; // Signal write is complete
//        snprintf(uart_buf, sizeof(uart_buf), "SPI2 DMA Write Complete\r\n");
//        HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, strlen(uart_buf), 100);
    }
}
