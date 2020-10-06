/*
 * temp_sensor.c
 *
 *  Created on: Mar 30, 2020
 *      Author: Nuoya Xie
 */

#include "temp_sensor.h"

uint16_t get_temperature(void)
{
    uint16_t bytes_obtained = I2C_receive_msg(SLAVE_ADDRESS,
                                              TRIGGER_TEMP_MEASURE_HOLD, 2);
    if (~(bytes_obtained & BIT1)) //grabbing status reg for temperature
        return bytes_obtained;
    else
        return 0xFFFF; //Error
}
uint16_t get_humidity(void)
{
    uint16_t bytes_obtained = I2C_receive_msg(SLAVE_ADDRESS,
                                              TRIGGER_HUMD_MEASURE_HOLD, 2);
    if (bytes_obtained & BIT1) //grabbing status reg for humidity
        return bytes_obtained;
    else
        return 0xFFFF; //Error
}

void temp_hum_soft_reset(void)
{
    //perform soft reset (rebooting sensor system without switching power off)
    uint8_t temp = 0;
    I2C_send_msg(SLAVE_ADDRESS, SOFT_RESET, temp, 0);
}

