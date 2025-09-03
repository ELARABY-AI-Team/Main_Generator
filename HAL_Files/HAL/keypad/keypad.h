/******************************************************************
* @Title    : keypad
* @Filename : keypad.h
* @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
* @Origin Date : 26/08/2023
* @Version  : 1.0.0
* @Compiler : CC-RL
* @Target   : R5F11BBC
* @brief    : This file implements device driver for key component.
*******************************************************************/
#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "../../core/GPIO/GPIO.h"
#include "keypad_config.h"


typedef struct
{
	gpio_port_t col_port_arr[KEYPAD_COL_MAX_NO];
	gpio_port_t row_port_arr[KEYPAD_ROW_MAX_NO];
    gpio_pin_t col_pin_arr[KEYPAD_COL_MAX_NO];
    gpio_pin_t row_pin_arr[KEYPAD_ROW_MAX_NO];
}keypad_t;


typedef enum {
    KEYPAD_SW_PREPRESSED ,
    KEYPAD_SW_PRESSED    ,
    KEYPAD_SW_PRERELEASED,
    KEYPAD_SW_RELEASED   ,
}keypad_sw_state_t;

void keypad_init(keypad_t* keypad);

keypad_sw_state_t keypad_get_switch_state(keypad_id_t switch_id);

void keypad_update(void);


#endif /* KEYPAD_H_ */
