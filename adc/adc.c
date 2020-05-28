/*
 * adc.c
 *
 *  Created on: May 3, 2020
 *      Author: Nuoya Xie
 */


#include "adc.h"

void adc_init(void)
{
    //disable ADC for configuration
    ADC10CTL0 &= ~ENC; //reset ENC bit (bit 1)

    //CTL0 configuration
    uint16_t adc_CTL0_value = 0x0;
    adc_CTL0_value |= BIT3; //IE: interrupt enable
    ADC10CTL0 = adc_CTL0_value;

    //CTL1 configuration
    uint16_t adc_CTL1_value = 0x0;
    adc_CTL1_value |= BITE; //input pin is P1.4 corresponding ADC Input: A4
    adc_CTL1_value |= BIT3; //SSEL: clock source select: bit3 = ACLK
    ADC10CTL1 = adc_CTL1_value;

    //ADC10AE0 Analog input enable
    ADC10AE0 |= BIT4; //Enable analog input on P1.4
}

void trigger_adc(void)
{
    ADC10CTL0 |= ADC10ON | ENC; //ADC on and ADC enable conversion
    ADC10CTL0 |= ADC10SC; //ADC start conversion
}

