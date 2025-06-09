/*
 * globals.h
 *
 *  Created on: Jun 8, 2025
 *      Author: vishal
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include "icm20948.h"

extern axises my_gyro;
extern axises my_accel;
extern axises my_mag;
extern char uart_buf[200];
extern float gyro_bias[3];
extern uint8_t uart_print_counter;

#endif /* GLOBALS_H */
