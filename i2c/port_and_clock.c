/*
 * port_and_clock.c
 *
 *  Created on: Apr 3, 2020
 *      Author: Nuoya Xie
 */

#include "port_and_clock.h"

void port_init(void)
{
    //All pins will be configured to output driving low to avoid unnecessary power draw
    P1DIR = 0xFF;
    P1OUT = 0x0;
    P2DIR = 0xFF;
    P2OUT = 0x0;
    P3DIR = 0xFF;
    P3OUT = 0x0;

    //P2.5 is used as the pin to wakeup Xbee
    P2SEL &= ~BIT5;
    P2SEL2 &= ~BIT5; //GPIO function
    P2DIR &= ~BIT5; //input by default, but switch to output to pull sleep pin down when waking Xbee up

    //P2.2 is used as the pin to receive ready signal from Xbee
    P2SEL &= ~BIT2;
    P2SEL2 &= ~BIT2; //GPIO function
    P2DIR &= ~BIT2; //input

    //P2.0 is debug LED
    P2SEL &= ~BIT0;
    P2SEL2 &= ~BIT0; //GPIO function
    P2DIR |= BIT0; //output

    P1DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4;
    P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);
}

void clock_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog timer

    //clock source: DCOCLK, 1MHz
    //clock signal: SMCLK, divider = 1

    //load DCO calibration data from TLV
    if (CALBC1_16MHZ == 0xFF)                  // If calibration constant erased
    {
        while (1)
            ;                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                          // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO calibration
    DCOCTL = CALDCO_1MHZ;
    //BCSCTL2 = DIVS_3;                     //divider = 8

    //clock source:VLOCLK, 12kHz
    //clock signal: ACLK, divider = 8, 12k / 8 = 1500Hz
    BCSCTL1 |= DIVA_3;
    BCSCTL3 = LFXT1S_2;
}

void timer_init(void)
{
    /*
     //TACTL configuration:
     //clock source select: SMCLK
     //clock divider: 8. 1MHz / 8 = 15625Hz
     //count mode: up mode
     //interrupt enable
     uint16_t timer_control_bits = TASSEL_2 | ID_3 | MC_1 | TAIE;
     TA1CTL = timer_control_bits;

     //this will make the frequency of interrupt 0.25Hz (every 4 seconds) (slowest possible)
     TA1CCR0 = 62500;

     //initialize variable to further divide up time
     current_count = 0;

     //calculate count period value to further slow down sensor sampling frequency
     count_period = SAMPLING_PERIOD / 4;*/

    //TACTL configuration:
    //clock source select: ACLK
    //clock divider: 8. 1500 / 8 = 187.5Hz
    //count mode: up mode
    //interrupt enable
    uint16_t timer_control_bits = TASSEL_1 | ID_3 | MC_1 | TAIE;
    TA1CTL = timer_control_bits;

    //this will make the frequency of interrupt every 300 seconds or 5 minutes (slowest whole number possible)
    //TA1CCR0 = 56250;
    TA1CCR0 = 750; //for debug purpose

    //initialize variable to further divide up time
    current_count = 0;

    //calculate count period value to further slow down sensor sampling frequency
    //count_period = SAMPLING_PERIOD / 300;
    count_period = SAMPLING_PERIOD / 4; //for debug purpose
}

