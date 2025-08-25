/**
 * @file ntc.h
 * @author omar megahed & mohamed elsafoury 
 * @brief 
 * @version 0.1
 * @date 2022-06-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef NTC_H_
#define NTC_H_

#include "../../MCAL/ADC/MCAL_R5F11BBC_ADC.h"



typedef struct 
{
    adc_channel_t adc_channel;
    uint8_t id ; 
    float nominal_resistance ; 
    float beta              ;
    float series_resistance;
    float ref_volt;
}ntc_t;



typedef enum 
{
    NTC_NORMAL = 0x00 , 
    NTC_OPEN_CIRCUIT = 0x01 , 
    NTC_SHORT_CIRCUIT = 0X02,
    NTC_NOT_FOUND= 0x03
}ntc_state_t ; 

/**
 * @brief 
 * 
 * @param ntc 
 * @return int8_t  1  "ntc is added successfully" or 0 "ntc is not added" 
 * 
 */
uint8_t ntc_init(ntc_t* ntc);

/**
 * @brief 
 * 
 * @param ntc_id 
 * @param temperature 
 * @return ntc_state_t cam be NTC_NORMAL , NTC_OPEN_CIRCUIT or NTC_SHORT_CIRCUIT 
 */
 ntc_state_t ntc_get_temperature_celsius(uint8_t ntc_id , float* temperature);

/**
 * @brief update the ntcs readings 
 * 
 */
void ntc_update(void);


#endif 
