/*
 * Stepper_motor.c
 *
 *  Created on: Dec 4, 2023
 *      Author: AHENDA01
 */

#include "Stepper_motor.h"


STEPPER_MOTOR_TERMINALS_t  STEPPER_MOTOR[STEPPER_MOTOR_MAX];

void  STEPPER_MOTOR_Init  (STEPPER_MOTOR_TERMINALS_t * MOTOR)
{
	uint8_t index =  0 ;
	uint8_t index_motor = 0 ;
	for(index_motor = 0 ;index_motor<STEPPER_MOTOR_MAX ; index_motor++ ){
	for(index = 0 ; index < MAX_NUMBER_OF_TERMINALS ; index++){
		STEPPER_MOTOR[index_motor].MOTOR_COIL_PORT[index] = MOTOR->MOTOR_COIL_PORT[index]  ;
		STEPPER_MOTOR[index_motor].MOTOR_COIL_PIN[index] = MOTOR->MOTOR_COIL_PIN[index]  ;
		GPIO_Output_Init(STEPPER_MOTOR[index_motor].MOTOR_COIL_PORT[index], STEPPER_MOTOR[index_motor].MOTOR_COIL_PIN[index], 0 , normal_usage);//A1

	}
	}
}
void  STEPPER_MOTOR_set   (t_Direction direction , uint16_t num_of_steps )
{
	uint8_t  index = 0 ;
	static uint8_t num = 0 ;
    for(index = 0  ; index < STEPPER_MOTOR_MAX ; index++){
    	STEPPER_MOTOR[index].STATE.DIRECTION = direction  ;
    	STEPPER_MOTOR[index].STATE.NUM_STEPS = num_of_steps ;
    }
	switch(direction){
	case LEFT_DIRECTION :
		switch(num){
		case 0 :
			GPIO_Value_Set(Port_5, Pin_1 , 1);
			GPIO_Value_Set(Port_5, Pin_0 , 0);
			GPIO_Value_Set(Port_3, Pin_0 , 1);
			GPIO_Value_Set(Port_7, Pin_0 , 0);
			num = 1;break ;
		case 1 :
			GPIO_Value_Set(Port_5, Pin_1 , 0);
			GPIO_Value_Set(Port_5, Pin_0 , 1);
			GPIO_Value_Set(Port_3, Pin_0 , 1);
			GPIO_Value_Set(Port_7, Pin_0 , 0);
			num= 2;break ;
		case 2 :
			GPIO_Value_Set(Port_5, Pin_1 , 0);
			GPIO_Value_Set(Port_5, Pin_0 , 1);
			GPIO_Value_Set(Port_3, Pin_0 , 0);
			GPIO_Value_Set(Port_7, Pin_0 , 1);
			num = 3;break ;
		case 3 :
			GPIO_Value_Set(Port_5, Pin_1 , 1);
			GPIO_Value_Set(Port_5, Pin_0 , 0);
			GPIO_Value_Set(Port_3, Pin_0 , 0);
			GPIO_Value_Set(Port_7, Pin_0 , 1);
			num = 0;
			for(index = 0  ; index < STEPPER_MOTOR_MAX ; index ++){
				if(STEPPER_MOTOR[index].STATE.DIRECTION == LEFT_DIRECTION){
					STEPPER_MOTOR[index].STATE.Actual_steps++ ;
				}else{}
			}
			break ;

		}break;
		case RIGHT_DIRECTION :
			switch(num){
			case 0 :
				GPIO_Value_Set(Port_5, Pin_1 , 1);
				GPIO_Value_Set(Port_5, Pin_0 , 0);
				GPIO_Value_Set(Port_3, Pin_0 , 0);
				GPIO_Value_Set(Port_7, Pin_0 , 1);
				num = 1;break ;
			case 1 :
				GPIO_Value_Set(Port_5, Pin_1 , 0);
				GPIO_Value_Set(Port_5, Pin_0 , 1);
				GPIO_Value_Set(Port_3, Pin_0 , 0);
				GPIO_Value_Set(Port_7, Pin_0 , 1);
				num= 2;break ;
			case 2 :
				GPIO_Value_Set(Port_5, Pin_1 , 0);
				GPIO_Value_Set(Port_5, Pin_0 , 1);
				GPIO_Value_Set(Port_3, Pin_0 , 1);
				GPIO_Value_Set(Port_7, Pin_0 , 0);
				num = 3;break ;
			case 3 :
				GPIO_Value_Set(Port_5, Pin_1 , 1);
				GPIO_Value_Set(Port_5, Pin_0 , 0);
				GPIO_Value_Set(Port_3, Pin_0 , 1);
				GPIO_Value_Set(Port_7, Pin_0 , 0);
				num = 0;
				for(index = 0  ; index < STEPPER_MOTOR_MAX ; index ++){
					if(STEPPER_MOTOR[index].STATE.DIRECTION == RIGHT_DIRECTION){
						STEPPER_MOTOR[index].STATE.Actual_steps++ ;
					}else{}
				}
				break ;

			}break;
	}

}
void  STEPPER_MOTOR_update(void )
{
	uint8_t index =  0 ;
	for(index = 0 ; index < STEPPER_MOTOR_MAX  ; index ++){
		if(STEPPER_MOTOR[index].STATE.Actual_steps < STEPPER_MOTOR[index].STATE.NUM_STEPS){
			STEPPER_MOTOR_set(STEPPER_MOTOR[index].STATE.DIRECTION,STEPPER_MOTOR[index].STATE.NUM_STEPS ) ;
		}else {

		}

		if(STEPPER_MOTOR[index].STATE.Actual_steps == STEPPER_MOTOR[index].STATE.NUM_STEPS){
					STEPPER_MOTOR[index].STATE.Actual_steps = 0 ;
					STEPPER_MOTOR[index].STATE.NUM_STEPS = 0 ;
				}else {

				}
	}
}
