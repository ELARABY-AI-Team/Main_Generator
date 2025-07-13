/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : PWM functionality header file
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_MAIN.h"

/*
 * TRD_Channel_t - Defines the channels of timers capable of PWM generation
 * on STM32F401RC, based on RM0368 Reference Manual.
 * All identified PWM-capable timer channels are included.
 * No timers are reserved as the MCU contains multiple PWM-capable timers.
 */
typedef enum TRD_Channel_t
{
    TRD_PWM_TIM1_CH1, /* PDF Reference */
    TRD_PWM_TIM1_CH2, /* PDF Reference */
    TRD_PWM_TIM1_CH3, /* PDF Reference */
    TRD_PWM_TIM1_CH4, /* PDF Reference */
    TRD_PWM_TIM2_CH1, /* PDF Reference */
    TRD_PWM_TIM2_CH2, /* PDF Reference */
    TRD_PWM_TIM2_CH3, /* PDF Reference */
    TRD_PWM_TIM2_CH4, /* PDF Reference */
    TRD_PWM_TIM3_CH1, /* PDF Reference */
    TRD_PWM_TIM3_CH2, /* PDF Reference */
    TRD_PWM_TIM3_CH3, /* PDF Reference */
    TRD_PWM_TIM3_CH4, /* PDF Reference */
    TRD_PWM_TIM4_CH1, /* PDF Reference */
    TRD_PWM_TIM4_CH2, /* PDF Reference */
    TRD_PWM_TIM4_CH3, /* PDF Reference */
    TRD_PWM_TIM4_CH4, /* PDF Reference */
    TRD_PWM_TIM5_CH1, /* PDF Reference */
    TRD_PWM_TIM5_CH2, /* PDF Reference */
    TRD_PWM_TIM5_CH3, /* PDF Reference */
    TRD_PWM_TIM5_CH4, /* PDF Reference */
    TRD_PWM_TIM9_CH1, /* PDF Reference */
    TRD_PWM_TIM9_CH2, /* PDF Reference */
    TRD_PWM_TIM10_CH1, /* PDF Reference */
    TRD_PWM_TIM11_CH1, /* PDF Reference */
    TRD_PWM_CHANNEL_COUNT /* Total number of PWM channels */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel); /* Initializes the specified PWM channel for output */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty); /* Sets the frequency and duty cycle for the specified PWM channel */
void PWM_Start(TRD_Channel_t TRD_Channel); /* Starts PWM generation on the specified channel */
void PWM_Stop(TRD_Channel_t TRD_Channel); /* Stops PWM generation on the specified channel */
void PWM_PowerOff(void); /* Powers off all active PWM timers/channels */

#endif /* STM32F401RC_GPIO_H_ */