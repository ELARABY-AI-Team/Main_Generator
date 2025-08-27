/**
 * @file ntc.c
 * @author mohamed elsafoury & omar megahed
 * @brief 
 * @version 0.1
 * @date 2022-06-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ntc.h" 
#include "ntc_config.h"

#include <math.h>




typedef struct 
{
   ntc_t ntc_param ; 
   float temperature_celsius;
   ntc_state_t state;
}ntc_internal_t;




static ntc_internal_t ntc_arr[NTC_MAX_NO]; 
static uint8_t internal_index = 0 ;

uint8_t ntc_init(ntc_t* ntc)
{
    if(internal_index < NTC_MAX_NO)
    {
        ntc_arr[internal_index].ntc_param .adc_channel = ntc->adc_channel;
        ntc_arr[internal_index].ntc_param .beta = ntc->beta;
        ntc_arr[internal_index].ntc_param .id = ntc->id;
        ntc_arr[internal_index].ntc_param .nominal_resistance = ntc->nominal_resistance;
        ntc_arr[internal_index].ntc_param .ref_volt = ntc->ref_volt;
        ntc_arr[internal_index].ntc_param .series_resistance = ntc->series_resistance;
        ntc_arr[internal_index].temperature_celsius = 0.0 ;
        ADC_Init(ntc_arr[internal_index].ntc_param .adc_channel, select_mode) ;
        ADC_Enable();
        
    }
    else
    {
        return 0 ; 
    }

    internal_index ++ ;
    return 1;
}

ntc_state_t ntc_get_temperature_celsius(uint8_t ntc_id , float* temperature)
{
   
    // //adc start conversion
    // adc_startconv(ntc_arr[index].ntc_param.adc_channel);
    // //get adc voltage ; 
    // //get resistance;
    // //calculate temperature;

    //determine the thermistor index 
    uint8_t index = 0 ; 
    
    for (index = 0 ; index < internal_index ; index++)
    {
        if(ntc_arr[index].ntc_param.id == ntc_id)
        {
           break;
        }
        else if (index == internal_index-1 && ntc_arr[index].ntc_param.id != ntc_id)
        {
            *temperature = NTC_NA_TEMP ;
            return NTC_NOT_FOUND; 
        }
    }

   
    *temperature =  ntc_arr[index].temperature_celsius ;
    
    return  (ntc_arr[index].state);

}



/**
 * @brief update the ntcs readings 
 * 
 */
void ntc_update(void)
{
    //loop all ntcs
    uint8_t index = 0 ;
    float vth = 0.0 ;
    float rth = 0.0 ;
    float temp = 0.0 ;
//    uint8_t timeOut = 0 ; 
    
    for (index = 0 ; index < internal_index; index++)
    {      
        vth = ADC_Get();
        vth = vth *(0.00322265625) ;
              
#if     (NTC_CONNECTION == NTC_PULL_DN)
        if( (vth >= (NTC_OC_VOLTAGE - NTC_OC_VOLTAGE_OFFSET)) )
        {
            ntc_arr[index].state = NTC_OPEN_CIRCUIT ;
            ntc_arr[index].temperature_celsius = NTC_OC_TEMP;
        }
        else if ((vth <= (NTC_OC_VOLTAGE - NTC_SC_VOLTAGE_OFFSET)))
        {
            ntc_arr[index].state = NTC_SHORT_CIRCUIT;
            ntc_arr[index].temperature_celsius = NTC_SC_TEMP ;
        }
        else
        {
			
            ntc_arr[index].state = NTC_NORMAL;
			
            rth = (vth/(NTC_REF_VOLTAGE-vth))*(ntc_arr[index].ntc_param.series_resistance);
            temp = NTC_NOMINAL_TEMP_KELVIN * (ntc_arr[index].ntc_param.beta);
            temp = temp / ((ntc_arr[index].ntc_param.beta)+((NTC_NOMINAL_TEMP_KELVIN)*(log(rth / (ntc_arr[index].ntc_param.nominal_resistance)))));
            temp = temp - 273.15;
            ntc_arr[index].temperature_celsius = temp;
		
		
        }     
        

#elif   (NTC_CONNECTION == NTC_PULL_UP)  
        /*IN CASE OF OPEN CIRCUIT CONDITION THE VTH WILL BE ZERO*/
        /*CALCULATING RTH INCLUDES DIVISION BY VTH SO IT MUST BE MONITORED*/
        if( (vth <= (NTC_OC_VOLTAGE+NTC_OC_VOLTAGE_OFFSET)) )
        {
            ntc_arr[index].state = NTC_OPEN_CIRCUIT ;
            ntc_arr[index].temperature_celsius = NTC_OC_TEMP;
           
            
        }
        else if ((vth >= (NTC_SC_VOLTAGE - NTC_SC_VOLTAGE_OFFSET)))
        {
            ntc_arr[index].state = NTC_SHORT_CIRCUIT;
            ntc_arr[index].temperature_celsius = NTC_SC_TEMP ;
            
        }
        else
        {
            ntc_arr[index].state = NTC_NORMAL;
			
				 rth = ((NTC_REF_VOLTAGE/vth)-1)*(ntc_arr[index].ntc_param.series_resistance);
				 //calculate temperature
				 temp = NTC_NOMINAL_TEMP_KELVIN * (ntc_arr[index].ntc_param.beta);
				 temp = temp / ((ntc_arr[index].ntc_param.beta)+((NTC_NOMINAL_TEMP_KELVIN)*(log(rth / (ntc_arr[index].ntc_param.nominal_resistance)))));
				 temp = temp - 273.15;
				 ntc_arr[index].temperature_celsius = temp;
				
			
			
            
        }     
#endif

    }    
    
}





