/*
 * IR.c
 *
 *  Created on: Dec 18, 2023
 *      Author: AHENDA01
 */

#include "IR.h"
#include "../../MCAL/GPIO/MCAL_R5F11BBC_GPIO.h"
volatile  t_IR_RX_DATA  IR_Rx_state  = IR_Rx_RECIEVE_DATA ;
void IR_Init(void){
	GPIO_Output_Init(IR_Tx_PORT, IR_Tx_PIN, 1 ,normal_usage) ;
	GPIO_Input_Init ( IR_Rx_PORT, IR_Rx_PIN ,normal_usage  ) ;

}


void IR_Tx_Enable(void){

	GPIO_Value_Set(IR_Tx_PORT, IR_Tx_PIN, 0) ;

}

void IR_Tx_Disable(void){
	GPIO_Value_Set(IR_Tx_PORT, IR_Tx_PIN, 1) ;
	IR_Rx_state = IR_Tx_STOPED ;
}

t_IR_RX_DATA IR_Rx_get_read(void){

	if(GPIO_Value_Get(IR_Tx_PORT, IR_Tx_PIN)){

		IR_Rx_state = IR_Tx_STOPED ;
	}else {
		IR_Rx_state = GPIO_Value_Get(IR_Rx_PORT, IR_Rx_PIN) ;
	}
 return IR_Rx_state ;

}

void IR_update(void){

	if(IR_Rx_get_read() == IR_Rx_RECIEVE_DATA)
	{
		IR_Rx_state = IR_Rx_RECIEVE_DATA ;
	}else if(IR_Rx_get_read() == IR_Rx_NO_DATA){
		IR_Rx_state = IR_Rx_NO_DATA ;
	}else if (IR_Rx_get_read() == IR_Tx_STOPED){
		IR_Rx_state = IR_Tx_STOPED  ;
	}else{}
}
