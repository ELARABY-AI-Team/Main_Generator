/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM functionality
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
/* #include "STM32F401RC_GPIO.h" */ /* Included per template, but GPIO specifics are handled in a dedicated driver */


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * Defines the total number of channels supported by Timers that support PWM functionality
 * as identified from the provided RM0368 reference manual.
 * Timers TIM12, TIM13, TIM14 are not available in this device variant per RM0368 section 14.1.
 */
typedef enum
{
    TIM1_CH1 = 0, /* PDF Reference */
    TIM1_CH2,     /* PDF Reference */
    TIM1_CH3,     /* PDF Reference */
    TIM1_CH4,     /* PDF Reference */

    TIM2_CH1,     /* PDF Reference */
    TIM2_CH2,     /* PDF Reference */
    TIM2_CH3,     /* PDF Reference */
    TIM2_CH4,     /* PDF Reference */

    TIM3_CH1,     /* PDF Reference */
    TIM3_CH2,     /* PDF Reference */
    TIM3_CH3,     /* PDF Reference */
    TIM3_CH4,     /* PDF Reference */

    TIM4_CH1,     /* PDF Reference */
    TIM4_CH2,     /* PDF Reference */
    TIM4_CH3,     /* PDF Reference */
    TIM4_CH4,     /* PDF Reference */

    TIM5_CH1,     /* PDF Reference */
    TIM5_CH2,     /* PDF Reference */
    TIM5_CH3,     /* PDF Reference */
    TIM5_CH4,     /* PDF Reference */

    TIM9_CH1,     /* PDF Reference */
    TIM9_CH2,     /* PDF Reference */

    TIM10_CH1,    /* PDF Reference */

    TIM11_CH1,    /* PDF Reference */

    TRD_TOTAL_PWM_CHANNELS /* Sentinel value for total count */
} TRD_Channel_t;


 /* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel); /* Initializes the specified PWM channel. */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty); /* Sets the frequency and duty cycle for the specified PWM channel. */
void PWM_Start(TRD_Channel_t TRD_Channel); /* Starts the PWM generation on the specified channel. */
void PWM_Stop(TRD_Channel_t TRD_Channel); /* Stops the PWM generation on the specified channel. */
void PWM_PowerOff(void); /* Powers off all PWM peripherals. */


#endif /* STM32F401RC_PWM_H_ */