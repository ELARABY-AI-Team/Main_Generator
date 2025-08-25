/*
 * IR.h
 *
 *  Created on: Dec 18, 2023
 *      Author: AHENDA01
 */

#ifndef HAL_IR_IR_H_
#define HAL_IR_IR_H_

#include "IR_user.h"


typedef enum {
	IR_Rx_RECIEVE_DATA= 0   ,
	IR_Rx_NO_DATA ,
	IR_Tx_STOPED
}t_IR_RX_DATA ;
//volatile  t_IR_RX_DATA  IR_Rx_state  = IR_Tx_STOPED ;

void IR_Init(void) ;
void IR_Tx_Enable(void) ;
void IR_Tx_Disable(void);
t_IR_RX_DATA IR_Rx_get_read(void) ;
void IR_update(void);
#endif /* HAL_IR_IR_H_ */
