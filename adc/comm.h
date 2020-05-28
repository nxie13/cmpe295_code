/*
 * comm.h
 *
 *  Created on: Mar 30, 2020
 *      Author: Matt
 */

#ifndef COMM_H_
#define COMM_H_

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_BUFFER_SIZE 32
#define COMM_WAIT_TIME 5000

//I2C State Machine Struct
typedef enum I2C_ModeEnum{
    IDLE_MODE,
    TX_REG_CMD_MODE,
    RX_REG_CMD_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
} I2C_Mode;

typedef enum Data_TypeEnum{
    LEN = 0,
    TEMP = 1,
    HUM = 2,
    VIS = 3,
    UV = 4,
    IR = 5,
    TDS = 6
} Data_Type;

volatile I2C_Mode MasterMode;

volatile uint8_t ReceiveBuffer[MAX_BUFFER_SIZE];
volatile uint8_t RXByteCtr;
volatile uint8_t ReceiveIndex;
volatile uint8_t TransmitBuffer[MAX_BUFFER_SIZE];
volatile uint8_t TXByteCtr;
volatile uint8_t TransmitIndex;
volatile uint8_t UART_byte_count;
volatile uint8_t UART_total_byte_count;

/* The Register Address/Command to use*/
volatile uint8_t TransmitRegAddr;

void I2C_init(void);
void UART_init(void);

//i2c functions
void I2C_send_msg(uint8_t addr, uint8_t cmd, uint8_t *reg_data, uint8_t count);
uint16_t I2C_receive_msg(uint8_t addr, uint8_t cmd, uint8_t count);
void I2C_send_byte(uint8_t addr, uint8_t cmd, uint8_t data_to_send);

void send_to_UART_per_sensor(Data_Type which_data, uint16_t data_to_send);
void send_to_UART(uint8_t * data_to_send, uint8_t bytes_to_send);

//utility functions
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);
void clear_buffer(uint8_t * buffer, int buffer_size);


#endif /* COMM_H_ */
