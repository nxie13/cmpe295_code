/*
 * water_level_sensor.c
 *
 *  Created on: Jul 2, 2020
 *      Author: Nuoya
 */

#include "water_level_sensor.h"

void getHigh12SectionValue(uint8_t *temp_buffer, int buffer_size)
{
    memset(temp_buffer, 0, buffer_size);
    I2C_receive_msg_no_cmd(ATTINY1_HIGH_ADDR, buffer_size, temp_buffer);
}

void getLow8SectionValue(uint8_t *temp_buffer, int buffer_size)
{
    memset(temp_buffer, 0, buffer_size);
    I2C_receive_msg_no_cmd(ATTINY2_LOW_ADDR, buffer_size, temp_buffer);
}

bool check_water_sensor_reading(uint8_t *data_buf)
{
    const int sensorvalue_min = 250;
    const int sensorvalue_max = 255;
    int count = 0;

    //checking values
    int i;
    for (i = 0; i < 20; i++)
    {
        if (data_buf[i] >= sensorvalue_min && data_buf[i] <= sensorvalue_max)
        {
            count++;
        }
    }

    if (count != 20)
    {
        return false;
    }
    return true;
}

//returns the height of water in mm
int obtain_water_level_mm(void)
{
    int result = 0;
    uint8_t data_buf_low_8[8] = { 0 };
    uint8_t data_buf_high_12[12] = { 0 };
    uint8_t data_buf_total[20] = { 0 };

    //get 20 bytes of data from the water level sensor
    getHigh12SectionValue(data_buf_high_12, 12);
    getLow8SectionValue(data_buf_low_8, 8);

    memcpy(data_buf_total, data_buf_low_8, 8);
    memcpy(&data_buf_total[0] + 8, data_buf_high_12, 12);

    //count the number of markers that detected water
    int i;
    for (i = 0; i < 20; i++)
    {
        if (data_buf_total[i] > THRESHOLD)
        {
            result+= 5; //each of the marker detected is 5mm
        }
    }
    return result;
}

