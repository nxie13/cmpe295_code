/*
 * water_level_sensor.h
 *
 *  Created on: Jul 2, 2020
 *      Author: Nuoya Xie
 */

#ifndef WATER_LEVEL_SENSOR_H_
#define WATER_LEVEL_SENSOR_H_

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "comm.h"

#define NO_TOUCH       0xFE
#define THRESHOLD      100
#define ATTINY1_HIGH_ADDR   0x78
#define ATTINY2_LOW_ADDR   0x77

void getHigh12SectionValue(uint8_t* temp_buffer, int buffer_size);
void getLow8SectionValue(uint8_t* temp_buffer, int buffer_size);
bool check_water_sensor_reading(uint8_t * data_buf);
int obtain_water_level_mm(void);


#endif /* WATER_LEVEL_SENSOR_H_ */
