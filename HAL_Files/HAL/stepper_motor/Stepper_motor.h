/*
 * Stepper_motor.h
 *
 *  Created on: Dec 4, 2023
 *      Author: AHENDA01
 */

#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#include "../../MCAL/CLOCK/MCAL_General_Config.h"
#include "../../MCAL/CLOCK/MCAL_R5F11BBC_MAIN.h"
#include "Stepper_motor_user.h"


typedef enum{
	LEFT_DIRECTION ,
	RIGHT_DIRECTION
}t_Direction  ;


typedef struct {
	t_Direction DIRECTION ;
	uint16_t NUM_STEPS  ;
	uint16_t Actual_steps ;
}Stepper_motor_t ;
typedef struct
{
	tport MOTOR_COIL_PORT[MAX_NUMBER_OF_TERMINALS];
    tpin MOTOR_COIL_PIN[MAX_NUMBER_OF_TERMINALS];
    Stepper_motor_t  STATE  ;

}STEPPER_MOTOR_TERMINALS_t;



void  STEPPER_MOTOR_Init  (STEPPER_MOTOR_TERMINALS_t * MOTOR) ;
void  STEPPER_MOTOR_set   (t_Direction direction , uint16_t num_of_steps ) ;
void  STEPPER_MOTOR_update(void );



#endif /* STEPPER_MOTOR_H_ */
