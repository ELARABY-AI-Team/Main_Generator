/*
 * Touch_Pad.h
 *
 *  Created on: Dec 3, 2023
 *      Author: AHENDA01
 */

#ifndef TOUCH_PAD_H_
#define TOUCH_PAD_H_


#include "../../MCAL/CLOCK/MCAL_General_Config.h"
#include "../../MCAL/CLOCK/MCAL_R5F11BBC_MAIN.h"
#include "Touch_Pad_user.h"
/**Definitions ===============================================================================*/
#define SHORT_PRESS (4)
#define SHORT_RELEASE (10)
/**Selections ================================================================================*/

/******************************************************************
* @Title    : switch.h
* @Filename : switch.h
* @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
* @Origin Date : 12/06/2023
* @Version  : 1.0.0
* @Compiler : CC-RL
* @Target   : R5F11BBC
* @brief    : This file implements device driver for switch component.
*******************************************************************/


/******************
 *    Typedefs
*******************/
/**
 * Defines the switch state.
 */
typedef enum {
    TOUCH_PREPRESSED ,
    TOUCH_PRESSED    ,
    TOUCH_PRERELEASED,
    TOUCH_RELEASED   ,
#if TOUCH_LONG_PRESS_USAGE == 1
    TOUCH_LONG_PRESSED,
#endif
}touch_pad_state_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void TOUCH_PAD_Init(void);
touch_pad_state_t TOUCH_PAD_get_state(touch_pad_id_t touch_id);
void TOUCH_PAD_update(void);



#endif /* TOUCH_PAD_H_ */
