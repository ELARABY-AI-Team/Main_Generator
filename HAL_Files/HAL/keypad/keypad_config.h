/******************************************************************
* @Title    : keypad
* @Filename : keypad_config.h
* @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
* @Origin Date : 26/08/2023
* @Version  : 1.0.0
* @Compiler : CC-RL
* @Target   : R5F11BBC
* @brief    : This file implements device driver for key component.
*******************************************************************/
#ifndef KEYPAD_CONFIG_H_
#define KEYPAD_CONFIG_H_

#include "../../core/GPIO/GPIO.h"

#define KEYPAD_COL_MAX_NO 2
#define KEYPAD_ROW_MAX_NO 4

#define KEYPAD_COL_ON_LEVEL    (LOW)
#define KEYPAD_COL_OFF_LEVEL  	(HIGH)


#define KEYPAD_ROW_ON_LEVEL  	(HIGH)
#define KEYPAD_ROW_OFF_LEVEL  	(LOW)

#define KEYPAD_ALL_RELEASED  0x00

typedef enum
{
    /*COL 0*/
	KEYPAD_00 = 0 ,     // col : 0 , row : 0
    KEYPAD_01 = 1 ,     // col : 0 , row : 1
    KEYPAD_02 = 2 ,     // col : 0 , row : 2
    KEYPAD_03 = 3 ,     // col : 0 , row : 3
    /*COL 1*/
    KEYPAD_10 = 10 ,     // col : 1 , row : 0
    KEYPAD_11 = 11 ,     // col : 1 , row : 1
    KEYPAD_12 = 12 ,     // col : 1 , row : 2
    KEYPAD_13 = 13      // col : 1 , row : 3
}keypad_id_t;


#endif /* KEYPAD_CONFIG_H_ */
