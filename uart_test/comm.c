/*
 * comm.c
 *
 *  Created on: Mar 30, 2020
 *      Author: Nuoya Xie
 *      Reference: MSP430G2xx3 Demo - USCI_B0, I2C Master multiple byte TX/RX
 */

#include <comm.h>

void I2C_send_msg(uint8_t addr, uint8_t cmd, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_CMD_MODE;
    TransmitRegAddr = cmd;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = addr;
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    IE2 &= ~UCB0RXIE;                       // Disable RX interrupt
    IE2 |= UCB0TXIE;                        // Enable TX interrupt

    //bit 1: UCTXSTT - transmit START condition in master mode
    //bit 4: UCTR - Transmitter/receiver. 1 = transmitter
    UCB0CTL1 |= BIT1 | BIT4; //Start transmission
    while (UCB0STAT & UCBBUSY)
        ;
}

uint16_t I2C_receive_msg(uint8_t addr, uint8_t cmd, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_CMD_MODE;
    TransmitRegAddr = cmd;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = addr;
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    IE2 &= ~UCB0RXIE;       // Disable RX interrupt
    IE2 |= UCB0TXIE;       // Enable TX interrupt

    //bit 1: UCTXSTT - transmit START condition in master mode
    //bit 4: UCTR - Transmitter/receiver. 1 = transmitter
    UCB0CTL1 |= BIT1 | BIT4;    //Start transmission
    while (UCB0STAT & UCBBUSY)
        ;
    return ReceiveBuffer[0] << 8 | ReceiveBuffer[1]; //[0] MSB, [1] LSB
}

void I2C_send_byte(uint8_t addr, uint8_t cmd, uint8_t data_to_send)
{
    uint8_t reg_data[2] = { data_to_send, 0 };
    I2C_send_msg(addr, cmd, reg_data, 1);
}

void send_to_UART_per_sensor(Data_Type which_data, uint16_t data_to_send)
{
    //send data (16-bits) to xbee module
    //UART transmission starts by writing lowers 8 bits of data into TXBUF
    switch (which_data)
    {
    case VIS:
        TransmitBuffer[0] = 0x1;
        break;
    case UV:
        TransmitBuffer[0] = 0x2;
        break;
    case IR:
        TransmitBuffer[0] = 0x3;
        break;
    case TEMP:
        TransmitBuffer[0] = 0x4;
        break;
    case HUM:
        TransmitBuffer[0] = 0x5;
        break;
    default: //Length
        TransmitBuffer[0] = data_to_send;
        UART_total_byte_count = 1;
    }

    if (which_data) //not length byte
    {
        TransmitBuffer[1] = data_to_send >> 8; //MSB
        TransmitBuffer[2] = data_to_send & 0xff; //LSB
        UART_total_byte_count = 3;
    }

    UART_byte_count = 0;
    UCA0TXBUF = TransmitBuffer[0];
    IE2 |= UCA0TXIE; //Enable Interrupt - TXIFG
    while (UCA0STAT & UCBUSY)
           ;
}

void send_to_UART(uint8_t *data_to_send, uint8_t bytes_to_send)
{
    //send data (16-bits) to xbee module
    //UART transmission starts by writing lowers 8 bits of data into TXBUF
    CopyArray(data_to_send, TransmitBuffer, bytes_to_send);
    UART_total_byte_count = bytes_to_send;
    UART_byte_count = 0;
    UCA0TXBUF = TransmitBuffer[0];
    IE2 |= UCA0TXIE; //Enable Interrupt - TXIFG
    while (UCA0STAT & UCBUSY)
               ;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

void clear_buffer(uint8_t *buffer, int buffer_size)
{
    int i;
    for (i = 0; i < buffer_size; i++)
    {
        buffer[i] = 0;
    }
}

void UART_init(void)
{
    //Configuration for xbee: 8-bit, no parity bit, 1 stop bit, lsb first
    UCA0CTL1 |= BIT0; //Set UCSWRST bit

    //CTL0 configuration
    UCA0CTL0 = 0x0;

    //CTL1 configuration
    uint16_t uart_CTL1_value = 0x0;
    uart_CTL1_value = BIT7 | BIT0; //bit7 set and bit6 reset = SMCLK
    UCA0CTL1 = uart_CTL1_value;

    //baud rate configuration (check with xbee ->9600bps)
    /* N = f(BRCLK)/Baud rate, f(BRCLK) = 125KHz, baud rate is 9600bps. N = 13.02
     * low freq. mode, UCBRx = INT(N) = 13, UCBRSx = 0x1, UCOS16 = 0
     */
    UCA0BR0 = 13;
    UCA0BR1 = 0;
    UCA0MCTL = 0x1 << 1;

    //Port Configuration TX and RX
    P1SEL |= BIT1 | BIT2; //TX: P1.2. RX: P1.1
    P1SEL2 |= BIT1 | BIT2;

    //clear UCSWRST bit
    UCA0CTL1 &= ~BIT0; //Reset UCSWRST bit
    IFG2 &= ~UCA0TXIFG; //clear interrupt
    IE2 &= ~UCA0TXIE; //disable interrupt
    UART_byte_count = 0; //init buffer count variable
}

//I2C initialization
void I2C_init(void)
{
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    UCB0BR0 = 4;                            // fSCL = SMCLK/4 = 31250Hz (max)
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0I2CIE |= UCNACKIE | UCSTPIE;        // Enable nack, stop IFG

    //Configure ports as SCL and SDA
    P1SEL |= BIT6 | BIT7; //1.6: SCL 1.7: SDA
    P1SEL2 |= BIT6 | BIT7;

    //initialize variables
    MasterMode = IDLE_MODE;
    clear_buffer(ReceiveBuffer, MAX_BUFFER_SIZE);
    clear_buffer(TransmitBuffer, MAX_BUFFER_SIZE);
    RXByteCtr = 0;
    ReceiveIndex = 0;

    TXByteCtr = 0;
    TransmitIndex = 0;

    /* The Register Address/Command to use*/
    TransmitRegAddr = 0;
}

