/******************************************************************
 * @Title    : keypad
 * @Filename : keypad.c
 * @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
 * @Origin Date : 26/08/2023
 * @Version  : 1.0.0
 * @Compiler : CC-RL
 * @Target   : R5F11BBC
 * @brief    : This file implements device driver for key component.
 *******************************************************************/

#include "stdint.h"
#include "../core/GPIO/GPIO.h"
#include "keypad.h"
#include "keypad_config.h"

keypad_t keypad_container;
static keypad_sw_state_t keypad_sw_current_state[KEYPAD_COL_MAX_NO][KEYPAD_ROW_MAX_NO] = {KEYPAD_SW_RELEASED};

void keypad_init(keypad_t* keypad)
{
	uint8_t index = 0 ;
	/*col handling */
	for(index = 0 ; index < KEYPAD_COL_MAX_NO ; index++)
	{
		keypad_container.col_port_arr[index] = keypad->col_port_arr[index];
		keypad_container.col_pin_arr[index]  = keypad->col_pin_arr[index];
		GPIO_Pin_Output_Init(keypad_container.col_port_arr[index],keypad_container.col_pin_arr[index]);
		//gpio_set_direction(keypad_container.col_port_arr[index],keypad_container.col_pin_arr[index],OUTPUT);
		GPIO_Set_Pin_Value(keypad_container.col_port_arr[index],keypad_container.col_pin_arr[index],KEYPAD_COL_OFF_LEVEL);
		//gpio_set_level(keypad_container.col_port_arr[index],keypad_container.col_pin_arr[index],COL_OFF_LEVEL);
	}

	/*row handling*/
	for(index = 0 ; index < KEYPAD_ROW_MAX_NO; index++)
	{
		keypad_container.row_port_arr[index] = keypad->row_port_arr[index];
		keypad_container.row_pin_arr[index]  = keypad->row_pin_arr[index];
		GPIO_Pin_Input_Init(keypad_container.row_port_arr[index],keypad_container.row_pin_arr[index]);
	}
	//GPIO_Pin_Input_Pull_Up_Init(keypad_container.row_port_arr[3],keypad_container.row_pin_arr[3]) ;
	//GPIO_Pin_Input_Pull_Up_Init(PORT1,PIN3) ;
	//GPIO_Pin_Output_Init(PORT1,PIN3);
	//GPIO_Set_Pin_Value(PORT1,PIN3 ,LOW);
}

void keypad_update(void)
{
	uint8_t Local_u8ColumnIdx , Local_u8RowIdx ;

	for(Local_u8ColumnIdx = 0; Local_u8ColumnIdx < KEYPAD_COL_MAX_NO; Local_u8ColumnIdx++)
	{
		// Activate current column
		GPIO_Set_Pin_Value(keypad_container.col_port_arr[Local_u8ColumnIdx],keypad_container.col_pin_arr[Local_u8ColumnIdx], KEYPAD_COL_ON_LEVEL);
		for(Local_u8RowIdx = 0; Local_u8RowIdx < KEYPAD_ROW_MAX_NO; Local_u8RowIdx++)
		{
			// Read the current row
			pin_level_t pin_value = GPIO_Get_Pin_Value(keypad_container.row_port_arr[Local_u8RowIdx], keypad_container.row_pin_arr[Local_u8RowIdx]);

			// Check if switch is pressed
			switch(keypad_sw_current_state[Local_u8ColumnIdx][Local_u8RowIdx])
			{

			/******************** PRESSED/PREPRESSED STATE ****************************************/
			case KEYPAD_SW_PREPRESSED:
			case KEYPAD_SW_PRESSED:

				if (pin_value == KEYPAD_ROW_OFF_LEVEL)
				{
					keypad_sw_current_state[Local_u8ColumnIdx][Local_u8RowIdx] = KEYPAD_SW_PRERELEASED;
				}
				else if (pin_value == KEYPAD_ROW_ON_LEVEL)
				{
					keypad_sw_current_state[Local_u8ColumnIdx][Local_u8RowIdx] = KEYPAD_SW_PRESSED;
				}
				else
				{
					//error

				}

				break;
				/********************PRERELEASED STATE**************************/
			case KEYPAD_SW_PRERELEASED:
			case KEYPAD_SW_RELEASED:
				if (pin_value == KEYPAD_ROW_ON_LEVEL)
				{
					keypad_sw_current_state[Local_u8ColumnIdx][Local_u8RowIdx] = KEYPAD_SW_PREPRESSED;
				}
				else if (pin_value == KEYPAD_ROW_OFF_LEVEL)
				{
					keypad_sw_current_state[Local_u8ColumnIdx][Local_u8RowIdx] = KEYPAD_SW_RELEASED;
				}
				else
				{
					//error

				}
				break;
			default:
				break;

			}
		}
		// Deactivate the current column
		GPIO_Set_Pin_Value(keypad_container.col_port_arr[Local_u8ColumnIdx],keypad_container.col_pin_arr[Local_u8ColumnIdx], KEYPAD_COL_OFF_LEVEL);
	}
}

keypad_sw_state_t keypad_get_switch_state(keypad_id_t switch_id)
{
	//get col and row
	uint8_t Local_u8RowId , Local_u8ColumnId ;

	Local_u8RowId = switch_id%10 ;
	Local_u8ColumnId = switch_id/10 ;

	return keypad_sw_current_state[Local_u8ColumnId][Local_u8RowId];
}


