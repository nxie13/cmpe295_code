/*
 * port_and_clock.h
 *
 *  Created on: Apr 3, 2020
 *      Author: Matt
 */

#ifndef PORT_AND_CLOCK_H_
#define PORT_AND_CLOCK_H_

//#define SAMPLING_PERIOD 300 // sampling period for sensors in seconds. Need to be multiple of 300
#define SAMPLING_PERIOD 32 // for debug purpose. sampling period for sensors in seconds. Need to be multiple of 4

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

void port_init(void);
void clock_init(void);
void timer_init(void);
void adc_init(void);

volatile unsigned long current_count;
unsigned long count_period;

#endif /* PORT_AND_CLOCK_H_ */
