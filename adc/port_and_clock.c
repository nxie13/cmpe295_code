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

    //P2.2 is used as the pin to enable sensor power
    P2SEL &= ~BIT2;
    P2SEL2 &= ~BIT2; //GPIO function
    P2DIR |= BIT2; //output

    //P2.0 is debug LED
    P2SEL &= ~BIT0;
    P2SEL2 &= ~BIT0; //GPIO function
    P2DIR |= BIT0; //output

    P1DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4;
    P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);

    //PIN config: P2.6, P2.7 xstal input/output
    P2DIR &= ~BIT6; //Xin
    P2DIR |= BIT7; //Xout
    P2SEL |= BIT6 | BIT7;
    P2SEL2 &= ~(BIT6 | BIT7);
}

void clock_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog timer

    //clock source: ACLK for peripheral and MCLK for CPU, divider = 1,
    //clock signal: LFXT1, (32.768KHz)

    BCSCTL1 &= ~(DIVA1 | DIVA0 | RSEL0 | RSEL1 | RSEL2 | RSEL3 | XTS); // Set lowest frequency range; /1 for Aclk divider
    BCSCTL2 |= SELM0 | SELM1; //select LXT1CLK for MCLK
    BCSCTL3 &= ~(LFXT1S0 | LFXT1S1 | XCAP0); //32KHz crystal on LFXT1
    BCSCTL3 |= XCAP1; //effective capacitance: ~10pF
}

void timer_init(void)
{
    //TACTL configuration:
    //clock source select: ACLK
    //clock divider: /8. 32.768KHz / 8 = 4096Hz
    //count mode: up mode
    //interrupt enable
    uint16_t timer_control_bits = TASSEL_1 | ID_3 | MC_1 | TAIE;
    TA1CTL = timer_control_bits;

    //Max divider: 65535 (2^16 - 1)
    //this will make the frequency of interrupt 0.0625Hz (every 16 seconds) (slowest possible)
    TA1CCR0 = 65535; //65535

    //initialize variable to further divide up time
    current_count = 0;

    //calculate count period value to further slow down sensor sampling frequency
    count_period = SAMPLING_PERIOD / 16;
}

