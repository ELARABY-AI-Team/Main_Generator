/*
 * TOUCH_Pad.c
 *
 *  Created on: Dec 3, 2023
 *      Author: AHENDA01
 */

#include "Touch_Pad.h"
#include "Touch_Pad_user.h"

/******************************************************************
* @Title    : switch
* @Filename : switch.c
* @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
* @Origin Date : 12/06/2023
* @Version  : 1.0.0
* @Compiler : CC-RL
* @Target   : R5F11BBC
* @brief    : This file implements device driver for switch component.
*******************************************************************/

/*********************
 *      DEFINES
 *********************/


    #define TOUCH_PRESSED_STATE         0
    #define TOUCH_RELEASED_STATE        1

uint8_t activecounter[MAX_touchs]  = {0} ;
/**********************
 *   STATIC VARIABLES
 **********************/
//extern const touch_pad_config_t touch_config[]  ;

static volatile touch_pad_state_t touch_current_state[MAX_touchs] = {TOUCH_RELEASED};
#if TOUCH_LONG_PRESS_USAGE == 1
static volatile uint8 count[MAX_touchs] = {0};
#endif
/**********************
 *   STATIC PROTOTYPES
 **********************/


/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * initializes all the switches.
 *
 * @return void
 */
void TOUCH_PAD_Init(void)
{
    uint8_t i;
	//filling the array of structs
	for (i = 0; i < MAX_touchs; i++)
	{
			GPIO_Input_Init(touch_config[i].port, touch_config[i].pin,normal_usage ) ;

	}
}


/**
 * Getter function for the a certain switch state.
 * @param sw_id: the ID of the switch
 * @return switch_state_t
 *
 */
touch_pad_state_t TOUCH_PAD_get_state(touch_pad_id_t  touch_id)
{
	return touch_current_state[touch_id];
}


/**********************
 *   STATIC FUNCTIONS
 **********************/
/**
* Periodic Task updates the states of all the switches
**/
void TOUCH_PAD_update(void)
{
	//the for loop is here
    uint8_t i;
	for (i = 0; i < MAX_touchs; i++)
	{
		uint8_t  gpio_state = GPIO_Value_Get(touch_config[i].port, touch_config[i].pin);

		switch (touch_current_state[i])
		{
		/******************** PRESSED/PREPRESSED STATE ****************************************/
			case TOUCH_PREPRESSED:
				if(gpio_state == TOUCH_PRESSED_STATE){
					activecounter[i]++ ;
				}else{
					activecounter[i] = 0 ;
					touch_current_state[i] = TOUCH_PRERELEASED ;
				}
				if(activecounter[i] > SHORT_PRESS){
					touch_current_state[i] = TOUCH_PRESSED ;
				}else{}



				break;
			case TOUCH_PRESSED:

				/*if (gpio_state == TOUCH_RELEASED_STATE)
				{
					touch_current_state[i] = TOUCH_PRERELEASED;
				}
				else*/ if (gpio_state == TOUCH_PRESSED_STATE)
				{
					//touch_current_state[i] = TOUCH_PREPRESSED;

				}
				else
				{
					activecounter[i] = 0 ;
					touch_current_state[i] = TOUCH_PRERELEASED ;
					//error

				}

			break;
			/********************PRERELEASED STATE**************************/
			case TOUCH_PRERELEASED:
				if (gpio_state == TOUCH_RELEASED_STATE)
								{
									touch_current_state[i] =TOUCH_RELEASED;
								}else{}break ;
			case TOUCH_RELEASED:
				activecounter[i] = 0 ;

				if (gpio_state == TOUCH_PRESSED_STATE)
				{
					touch_current_state[i] = TOUCH_PREPRESSED;
				}
				/*else if (gpio_state == TOUCH_RELEASED_STATE)
				{
					touch_current_state[i] = TOUCH_RELEASED;
				}*/
				else
				{
					//error

				}
			break;
			default:
			break;

		}
	}
}

