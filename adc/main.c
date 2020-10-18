/*
 * Main code (ADC) for CMPE295 Master Project
 * By Chong Hang Cheong and Nuoya Xie
 *
 *To get lowest power consumption possible:
 * 1. Configure all GPIO as output 0
 * 2. When sleeping, the MCU stays in LPM3, lowest LPM possible with clock active
 *
 * This version of code should be used for sensors read with ADC. Another version
 * is for sensors read with I2C
 *
 * UART Pins: P1.1(RXD) P1.2(TXD)
 * GPIO pin to tell Xbee to wake up: P2.5
 * GPIO pin to give power to sensor: P2.2
 * Debug LED pin: P2.0
 * ADC pin: P1.4 (A4)
 *
 *
 * Timer used: TA1.1 on P2.1
 *
 *Sequence for MCU and XBee Communication
 *1. Timer wakes up MCU from sleep mode
 *2. MCU wakes up Xbee through pin P2.5
 *3. MCU gives sensor power and wait a little bit for sensor to get ready
 *4. MCU obtains sensor data
 *5. MCU disables sensor power
 *6. MCU sends data through UART to Xbee
 *7. MCU waits a set amount of time for Xbee to finish transmission.
 *8. MCU puts Xbee to sleep through pin P2.5 and goes to sleep mode itself
 *
 * UART Byte sequence:
 * UART communication speed is 1200 - to minimize TX errors
 * To send data over UART, first byte will be identification - which sensor data is sent. The choices are:
 * 0x01 - Temperature
 * 0x02 - humidity
 * 0x03 - sunlight
 * 0x04 - UV
 * 0x05 - IR
 * 0x06 - TDS
 * 0x07 - water level sensor
 * 0x08 - soil moisture
 *
 * The following data in teh format of characters will be sent on the bus
 *
 */

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "comm.h"
#include "port_and_clock.h"
#include "adc.h"

/**
 * main.c
 */

void sensor_output_uint_to_char(Data_Type sensor_type, uint16_t sensor_value, char arr[]);

static volatile uint16_t ADC_value = 0;
void main(void)
{
    clock_init();
    port_init();
    adc_init();
    UART_init();
    timer_init();
    __enable_interrupt(); //Enable interrupts

    P1DIR |= BIT6;

    P2IFG &= ~BIT5; //clear interrupt

    while (1)
    {
        P2OUT |= BIT2; //give power to sensor
        __delay_cycles(32000); //wait for sensor to ready
        P2OUT &= ~BIT5; //output low to pull sleep pin down to wake up xbee
        __delay_cycles(1000); //wait for xbee to ready

        trigger_adc();
        while (ADC10CTL1 & ADC10BUSY);
        P2OUT &= ~BIT2; //stop giving power to sensor

        char char_buf[16] = {0};
        sensor_output_uint_to_char(TDS, ADC_value, char_buf);
        int buf_size = strlen(char_buf) + 1; //include the '\0' character

        send_to_UART(char_buf, buf_size); //send to UART

        ADC_value = 0; //reset variable

        __delay_cycles(1000); //wait for xbee
        P2OUT |= BIT5; //output high to sleep pin

        LPM3; //go to low power mode
    }
}

//******************************************************************************
// ADC10 interrupt service routine
//******************************************************************************
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    ADC_value = ADC10MEM; //obtain ADC value from MEM buffer
    LPM3_EXIT;

#ifdef DEBUG_MODE
    //The code segment below is just for debugging purpose
    //No float conversion will be done in MSP430
    float adc_voltage = ((float) adc_value) * 3.3 / 1024.0;
    if (adc_voltage > 2)
    P1OUT |= BIT6;//Turn on on-board led
    else
    P1OUT &= ~BIT6;//Turn off on-board led
#endif
}

//******************************************************************************
// I2C/UART Interrupt For Received and Transmitted Data
//******************************************************************************
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    //P2OUT |= BIT0;
    if (IFG2 & UCB0RXIFG)                 // Receive Data Interrupt
    {
        //Must read from UCB0RXBUF
        uint8_t rx_val = UCB0RXBUF;

        if (RXByteCtr)
        {
            ReceiveBuffer[ReceiveIndex++] = rx_val;
            RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
            UCB0CTL1 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
            IE2 &= ~UCB0RXIE;
            MasterMode = IDLE_MODE;
        }
    }
    else if (IFG2 & UCB0TXIFG)            // Transmit Data Interrupt
    {
        switch (MasterMode)
        {
        case TX_REG_CMD_MODE:
            UCB0TXBUF = TransmitRegAddr;
            if (RXByteCtr)
                MasterMode = SWITCH_TO_RX_MODE; // Need to start receiving now
            else
                MasterMode = TX_DATA_MODE; // Continue to transmission with the data in Transmit Buffer
            break;

        case SWITCH_TO_RX_MODE:
            IE2 |= UCB0RXIE;              // Enable RX interrupt
            IE2 &= ~UCB0TXIE;             // Disable TX interrupt
            UCB0CTL1 &= ~UCTR;            // Switch to receiver
            MasterMode = RX_DATA_MODE;    // State state is to receive data
            UCB0CTL1 |= UCTXSTT;          // Send repeated start
            if (RXByteCtr == 1)
            {
                //Must send stop since this is the N-1 byte
                while ((UCB0CTL1 & UCTXSTT))
                    ;
                UCB0CTL1 |= UCTXSTP;      // Send stop condition
            }
            break;

        case TX_DATA_MODE:
            if (TXByteCtr)
            {
                UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                TXByteCtr--;
            }
            else
            {
                //Done with transmission
                UCB0CTL1 |= UCTXSTP;     // Send stop condition
                MasterMode = IDLE_MODE;
                IE2 &= ~UCB0TXIE;                    // disable TX interrupt
            }
            break;

        default:
            __no_operation();
            break;
        }
    }
    if ((IFG2 & UCA0TXIFG)) //UART byte TXIFG (IE2 & UCA0TXIE)
    {
        //P2OUT |= BIT0;
        if (UART_byte_count == 0)
        {
            UART_byte_count++;
        }
        else if (UART_byte_count < UART_total_byte_count - 1)
        {
            UCA0TXBUF = TransmitBuffer[UART_byte_count];
            UART_byte_count++;
        }
        else if (UART_byte_count == UART_total_byte_count - 1) //last byte
        {
            UCA0TXBUF = TransmitBuffer[UART_byte_count];
            UART_byte_count++;
            IFG2 &= ~UCA0TXIFG;
            IE2 &= ~UCA0TXIE;
        }
        else
            IFG2 &= ~UCA0TXIFG;
    }
    //P2OUT &= ~BIT0;
}

//******************************************************************************
// I2C/UART Interrupt For Start, Restart, Nack, Stop
//******************************************************************************
#pragma vector = USCIAB0RX_VECTOR
__interrupt
void USCIAB0RX_ISR(void)
{
    if (UCB0STAT & UCNACKIFG)
    {
        UCB0STAT &= ~UCNACKIFG;             // Clear NACK Flags
    }
    if (UCB0STAT & UCSTPIFG)                        //Stop or NACK Interrupt
    {
        UCB0STAT &= ~(UCSTTIFG + UCSTPIFG + UCNACKIFG); //Clear START/STOP/NACK Flags

    }
    if (UCB0STAT & UCSTTIFG)
    {
        UCB0STAT &= ~(UCSTTIFG);                    //Clear START Flags
    }
}

//******************************************************************************
// TIMERA1 TAIFG Interrupt for periodically waking up system
//******************************************************************************
#pragma vector=TIMER1_A1_VECTOR
__interrupt
void TIMER_ISR(void)
{
    current_count++;
    if (current_count == count_period)
    {
        LPM3_EXIT; //exit LPM3
        current_count = 0; //reset count to 0
    }
    TA1CTL &= ~TAIFG; //clear interrupt
    P2OUT &= ~BIT5; //reset trigger pin
}

//this function turns unsigned int to character for xbee transmission
void sensor_output_uint_to_char(Data_Type sensor_type, uint16_t sensor_value, char arr[])
{
    switch (sensor_type)
    {
    case TDS:
        arr[0] = 'T';
        arr[1] = 'D';
        arr[2] = 'S';
        arr[3] = ':';
        break;
    case H20LEVEL:
        arr[0] = 'L';
        arr[1] = 'V';
        arr[2] = 'L';
        arr[3] = ':';
        break;
    case SOILMOISTURE:
        arr[0] = 'M';
        arr[1] = 'O';
        arr[2] = 'I';
        arr[3] = ':';
        break;
}
    //sensor value: no more than 10 chars, including '\0'
    unsigned int sensor_value_int = (unsigned int)sensor_value;
    //unsigned int sensor_value_int = 123;
    char temp_buffer[10];
    itoa(sensor_value_int, temp_buffer);
    memcpy(&arr[4], temp_buffer, strlen(temp_buffer));
}

//itoa:  convert n to characters in s
 void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  //record sign
         n = -n;          //make n positive
     i = 0;
     do {       //generate digits in reverse order
         s[i++] = n % 10 + '0';   //get next digit
     } while ((n /= 10) > 0);     //delete it
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }

  void reverse(char s[])
  {
      int i, j;
      char c;

      for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
          c = s[i];
          s[i] = s[j];
          s[j] = c;
      }
  }
