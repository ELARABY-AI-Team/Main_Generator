/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM module on STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* Include required header files */
#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"

/*
 * Note: The t_pwm_channel enum definition is required by the module
 * but not included here as per user instruction. It is assumed
 * to be defined elsewhere (e.g., STM32F401RC_MAIN.h) and potentially
 * typedef'd to TRD_Channel_t.
 */

/* ==================== FUNCTION DECLARATIONS ==================== */

/* Initializes a specific PWM channel */
void PWM_Init(TRD_Channel_t TRD_Channel);

/* Sets the frequency and duty cycle for a specific PWM channel */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/* Starts the PWM generation on a specific channel */
void PWM_Start(TRD_Channel_t TRD_Channel);

/* Stops the PWM generation on a specific channel */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/* Powers off all active PWM peripherals/channels */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */