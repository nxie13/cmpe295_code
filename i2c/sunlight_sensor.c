/*
 * sunlight_sensor.c
 *
 *  Created on: Mar 30, 2020
 *      Author: Nuoya Xie
 */

#include "sunlight_sensor.h"

void configure_sunlight_sensor(void)
{
    //configure and enable UV reading
    //These registers must be set specific numbers
    I2C_send_byte(SI114X_ADDR, SI114X_UCOEFF0, 0x29);

    I2C_send_byte(SI114X_ADDR, SI114X_UCOEFF1, 0x89);

    I2C_send_byte(SI114X_ADDR, SI114X_UCOEFF2, 0x02);

    I2C_send_byte(SI114X_ADDR, SI114X_UCOEFF3, 0x00);

    //write to pararmeter register to enable UV and VIS
    write_sunlight_param_data(
            SI114X_CHLIST,
            SI114X_CHLIST_ENUV | SI114X_CHLIST_ENALSVIS | SI114X_CHLIST_ENALSIR
                    | SI114X_CHLIST_ENPS1);

    //set LED1 CURRENT(22.4mA)(It is a normal value for many LED
    write_sunlight_param_data(SI114X_PS1_ADCMUX, SI114X_ADCMUX_LARGE_IR);
    I2C_send_byte(SI114X_ADDR, SI114X_PS_LED21, SI114X_LED_CURRENT_22MA);

    write_sunlight_param_data(SI114X_PSLED12_SELECT,
                              SI114X_PSLED12_SELECT_PS1_LED1);

    //PS ADC SETTING
    write_sunlight_param_data(SI114X_PS_ADC_GAIN, SI114X_ADC_GAIN_DIV1);
    write_sunlight_param_data(SI114X_PS_ADC_COUNTER,
                              SI114X_ADC_COUNTER_511ADCCLK);
    write_sunlight_param_data(
            SI114X_PS_ADC_MISC,
            SI114X_ADC_MISC_HIGHRANGE | SI114X_ADC_MISC_ADC_RAWADC);

    //set VIS ADC Setting - set to under direct sunlight
    write_sunlight_param_data(SI114X_ALS_VIS_ADC_GAIN,
    SI114X_ADC_GAIN_DIV1);
    write_sunlight_param_data(SI114X_ALS_VIS_ADC_COUNTER,
    SI114X_ADC_COUNTER_511ADCCLK);
    write_sunlight_param_data(SI114X_ALS_VIS_ADC_MISC,
    SI114X_ADC_MISC_HIGHRANGE);

    //Set IR ADC setting - set to under direct sunlight
    write_sunlight_param_data(SI114X_ALS_IR_ADC_GAIN, SI114X_ADC_GAIN_DIV1);
    write_sunlight_param_data(SI114X_ALS_IR_ADC_COUNTER,
    SI114X_ADC_COUNTER_511ADCCLK);
    write_sunlight_param_data(SI114X_ALS_IR_ADC_MISC,
    SI114X_ADC_MISC_HIGHRANGE);

    //interrupt enable
    I2C_send_byte(SI114X_ADDR, SI114X_INT_CFG, SI114X_INT_CFG_INTOE);
    I2C_send_byte(SI114X_ADDR, SI114X_IRQ_ENABLE, SI114X_IRQEN_ALS);

    //Auto Run
    /*I2C_send_byte(SI114X_ADDR, SI114X_MEAS_RATE0, 0xFF);
     I2C_send_byte(SI114X_ADDR, SI114X_COMMAND, SI114X_PSALS_AUTO);*/

    //Manual mode
    I2C_send_byte(SI114X_ADDR, SI114X_MEAS_RATE0, 0x0);
}

uint8_t write_sunlight_param_data(uint8_t reg, uint8_t value)
{
    I2C_send_byte(SI114X_ADDR, SI114X_WR, value);
    I2C_send_byte(SI114X_ADDR, SI114X_COMMAND, reg | SI114X_SET);
    I2C_receive_msg(SI114X_ADDR, SI114X_RD, 1);
    //read back to see if it is right
    return ReceiveBuffer[0];
}

void reset_sunlight_sensor(void)
{
    //These registes will need to be cleared
    I2C_send_byte(SI114X_ADDR, SI114X_MEAS_RATE0, 0x0);
    I2C_send_byte(SI114X_ADDR, SI114X_MEAS_RATE1, 0x0);
    I2C_send_byte(SI114X_ADDR, SI114X_IRQ_ENABLE, 0x0);
    I2C_send_byte(SI114X_ADDR, SI114X_IRQ_MODE1, 0x0);
    I2C_send_byte(SI114X_ADDR, SI114X_IRQ_MODE2, 0x0);
    I2C_send_byte(SI114X_ADDR, SI114X_INT_CFG, 0x0);
    I2C_send_byte(SI114X_ADDR, SI114X_IRQ_STATUS, 0xFF);
    I2C_send_byte(SI114X_ADDR, SI114X_COMMAND, SI114X_RESET);
    I2C_send_byte(SI114X_ADDR, SI114X_HW_KEY, 0x17);
}

uint16_t read_sunlight_VIS(void)
{
    I2C_send_byte(SI114X_ADDR, SI114X_COMMAND, SI114X_PSALS_FORCE);
    return I2C_receive_msg(SI114X_ADDR, SI114X_ALS_VIS_DATA0, 2); //read VIS_DATA
}

uint16_t read_sunlight_IR(void)
{
    I2C_send_byte(SI114X_ADDR, SI114X_COMMAND, SI114X_PSALS_FORCE);
    return I2C_receive_msg(SI114X_ADDR, SI114X_ALS_IR_DATA0, 2); //read IR_DATA
}

uint16_t read_sunlight_UV(void)
{
    I2C_send_byte(SI114X_ADDR, SI114X_COMMAND, SI114X_PSALS_FORCE);
    return I2C_receive_msg(SI114X_ADDR, SI114X_AUX_DATA0_UVINDEX0, 2); //read UV_DATA
}
