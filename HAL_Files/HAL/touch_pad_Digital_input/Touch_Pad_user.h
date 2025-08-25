/*
 * Touch_Pad_user.h
 *
 *  Created on: Dec 3, 2023
 *      Author: AHENDA01
 */

#ifndef TOUCH_PAD_USER_H_
#define TOUCH_PAD_USER_H_

#include "../../MCAL/GPIO/MCAL_R5F11BBC_GPIO.h"
/*******************This Part is Constant Prohipettied to change on it ***********/

typedef struct
{
	tport port;
	tpin pin;
	// callback is removed for now
}touch_pad_config_t;
/*******************end Constant Part **************************/

/**
* Defines the switches IDs.
* @note: Must match the Touch_Pad configuartion index in switch_config[]
*/
typedef enum{
	POWER_TOUCH = 0,
	NORMAL_TOUCH,
	COLD_TOUCH,
	ICE_TOUCH ,
    MAX_touchs,
}touch_pad_id_t;




/**
 * Specifies The Long press functionality usage
 * value should be: 0 if disabled
 * 					1 if enabled
 */
#define SWITCH_LONG_PRESS_USAGE        0

#if SWITCH_LONG_PRESS_USAGE == 1
#define SWITCH_LONG_PRESS_TIMING_MILLIS   1600
#endif

extern const touch_pad_config_t touch_config[]  ;
//todo: add callback usage

/*********************** End of File **********************************/

#endif /* TOUCH_PAD_USER_H_ */
