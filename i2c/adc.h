/*
 * adc.h
 *
 *  Created on: May 3, 2020
 *      Author: Matt
 */

#ifndef ADC_H_
#define ADC_H_

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

void adc_init(void);
void trigger_adc(void);

#endif /* ADC_H_ */
