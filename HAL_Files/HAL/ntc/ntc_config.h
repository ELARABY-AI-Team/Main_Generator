/**
 * @file ntc_config.h
 * @author omar megahed & mohamed elsafoury
 * @brief 
 * @version 0.1
 * @date 2022-06-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef NTC_CONFIG_H_
#define NTC_CONFIG_H_


#define NTC_PULL_UP 0
#define NTC_PULL_DN 1



#define NTC_UPDATE_MS  500  
#define NTC_MAX_NO   1
#define NTC_CONV_TIMEOUT 1000
#define NTC_REF_VOLTAGE  3.3
#define NTC_NOMINAL_TEMP_KELVIN  (278.15F)


#define NTC_OC_TEMP       -30.0
#define NTC_SC_TEMP       -40.0
#define NTC_NA_TEMP       -50.0

#define NTC_CONNECTION      NTC_PULL_UP


#if (NTC_CONNECTION == NTC_PULL_UP)

#define NTC_OC_VOLTAGE                  0 /*VOLTS*/
#define NTC_OC_VOLTAGE_OFFSET           0.25 /*VOLTS*/
#define NTC_SC_VOLTAGE                  NTC_REF_VOLTAGE  /*VOLTS*/
#define NTC_SC_VOLTAGE_OFFSET           0.25 /*VOLTS*/

#else

#define NTC_OC_VOLTAGE                  NTC_REF_VOLTAGE /*VOLTS*/
#define NTC_OC_VOLTAGE_OFFSET           0.25 /*VOLTS*/
#define NTC_SC_VOLTAGE                  0  /*VOLTS*/
#define NTC_SC_VOLTAGE_OFFSET           0.25 /*VOLTS*/

#endif






#endif
