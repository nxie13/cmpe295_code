/*
 * Main code (I2C) for CMPE295 Master Project
 * By Chong Hang Cheong and Nuoya Xie
 *
 *To get lowest power consumption possible:
 * 1. Configure all GPIO as output 0
 * 2. When sleeping, the MCU stays in LPM0, which consumes ~35uA
 *
 * This version of code should be used for sensors read with I2C. Another version
 * is for sensors read with adc
 *
 * UART Pins: P1.1(RXD) P1.2(TXD)
 * I2C pin: P1.6(SCL) P1.7(SDA)
 * UART Pins: P1.1(RXD) P1.2(TXD)
 * GPIO pin to tell Xbee to wake up: P2.5
 * GPIO pin to receive from Xbee when RF transmission is complete: P2.2
 * Debug LED pin: P2.0
 *
 * Timer used: TA1.1 on P2.1
 *
 *Sequence for MCU and XBee Communication
 *1. Timer wakes up MCU from sleep mode
 *2. MCU wakes up Xbee through pin P2.5
 *3. MCU obtains sensor data
 *4. MCU waits for Xbee to pull P2.2 to high, indicating Xbee is ready for UART transmission
 *4. After obtaining the ready signal, MCU sends data through UART to Xbee
 *5. MCU waits for signal from Xbee on port P2.2, indicating that Xbee is done sending RF
 *6. After ready signal is received from Xbee, MCU puts Xbee to sleep through pin P2.5 and goes to sleep mode itself
 *
 * UART Byte sequence:
 * To send data over UART, first byte will the the total number of bytes sent during transmission, not including this byte
 * second byte will be identification - which sensor data is sent. The choices are:
 * 0x01 - Temperature
 * 0x02 - humidity
 * 0x03 - sunlight
 * 0x04 = UV
 * 0x05 = IR
 *
 * two subsequent bytes will be actual data. first byte is MSB, and second byte is LSB
 *
 * This format of identification byte - data MSB byte - data LSB byte will keep going until all sensor data are sent
 *
 * TODO: implement error LED for any possible bad situations (i2c setting not correct, etc.)
 */

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "port_and_clock.h"
#include "comm.h"

/**
 * main.c
 */
void main(void)
{
    clock_init();
    port_init();
    UART_init();
    timer_init();
    __enable_interrupt(); //Enable interrupts
    __delay_cycles(COMM_WAIT_TIME);

    while (1)
    {
        // you should see 0x1 - 0xAA - 0xAA followed by 0x2 - 0xBB - 0xBB

        //P2OUT |= BIT0; //LED ON
        P2DIR |= BIT5; //switch to output to wake up Xbee
        P2OUT &= ~BIT5; //output low to pull sleep pin down

        uint8_t data_buf[16] = { 0 };
        data_buf[0] = 0xAA;
        data_buf[1] = 0xBB;

        //while (!(P2IN & BIT2)); //wait for Xbee to Signal ready on pin P2.2

        //Xbee is now ready, send data over UART
        send_to_UART(data_buf, 2);

        // while (P2IN & BIT2); //wait for Xbee to Signal ready on pin P2.2
        P2DIR &= ~BIT5; //switch back to input again

        //P2OUT &= ~BIT0; //LED OFF
        LPM0; //go to low power mode
    }
}

//******************************************************************************
// GPIO Interrupt - Only used when GPIO wakes up MCU
//******************************************************************************
/*#pragma vector=PORT2_VECTOR
 __interrupt void Port_2(void)
 {
 //If interrupt signal comes from P2.5
 if (P2IFG & BIT5)
 {
 //P2OUT ^= BIT0; //turn on LED

 P2IFG &= ~BIT5; //Clear interrupt
 LPM4_EXIT;
 }
 }*/

//******************************************************************************
// I2C/UART Interrupt For Received and Transmitted Data
//******************************************************************************
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    P2OUT |= BIT0;
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
                MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
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
                IE2 &= ~UCB0TXIE;                       // disable TX interrupt
            }
            break;

        default:
            __no_operation();
            break;
        }
    }
    if ((IFG2 & UCA0TXIFG)) //UART byte TXIFG (IE2 & UCA0TXIE)
    {
        if (UART_byte_count == 0)
        {
            UART_byte_count++;
        }
        else if (UART_byte_count < UART_total_byte_count - 1)
        {
            UCA0TXBUF = TransmitBuffer[UART_byte_count];
            UART_byte_count++;
        }
        else if (UART_byte_count == UART_total_byte_count - 1)//last byte
        {
            UCA0TXBUF = TransmitBuffer[UART_byte_count];
            UART_byte_count++;
            IFG2 &= ~UCA0TXIFG;
            IE2 &= ~UCA0TXIE;
        }
        else
            IFG2 &= ~UCA0TXIFG;
    }
    P2OUT &= ~BIT0;
}

//******************************************************************************
// I2C/UART Interrupt For Start, Restart, Nack, Stop
//******************************************************************************
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
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
__interrupt void TIMER_ISR(void)
{
    if (current_count == count_period)
    {
        LPM0_EXIT; //exit LPM4
        current_count = 0; //reset count to 0
    }
    current_count++; //increment count
    TA1CTL &= ~TAIFG; //clear interrupt
    P2OUT &= ~BIT5; //reset trigger pin
}

