/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : This header file provides declarations for the PWM (Pulse Width Modulation) module for the STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* Assumed based on MCU conventions */
#include "STM32F401RC_GPIO.h" /* Assumed based on MCU conventions */

/*
 * TRD_Channel_t: Identifies PWM-capable timer channels on STM32F401RC.
 * Note: Timers TIM10 and TIM11 are reserved from this enum for other uses.
 */
typedef enum
{
    TRD_PWM_TIM1_CH1,  /* PDF Reference: TIM1 supports 4 PWM channels */
    TRD_PWM_TIM1_CH2,  /* PDF Reference: TIM1 supports 4 PWM channels */
    TRD_PWM_TIM1_CH3,  /* PDF Reference: TIM1 supports 4 PWM channels */
    TRD_PWM_TIM1_CH4,  /* PDF Reference: TIM1 supports 4 PWM channels */
    TRD_PWM_TIM2_CH1,  /* PDF Reference: TIM2 supports 4 PWM channels */
    TRD_PWM_TIM2_CH2,  /* PDF Reference: TIM2 supports 4 PWM channels */
    TRD_PWM_TIM2_CH3,  /* PDF Reference: TIM2 supports 4 PWM channels */
    TRD_PWM_TIM2_CH4,  /* PDF Reference: TIM2 supports 4 PWM channels */
    TRD_PWM_TIM3_CH1,  /* PDF Reference: TIM3 supports 4 PWM channels */
    TRD_PWM_TIM3_CH2,  /* PDF Reference: TIM3 supports 4 PWM channels */
    TRD_PWM_TIM3_CH3,  /* PDF Reference: TIM3 supports 4 PWM channels */
    TRD_PWM_TIM3_CH4,  /* PDF Reference: TIM3 supports 4 PWM channels */
    TRD_PWM_TIM4_CH1,  /* PDF Reference: TIM4 supports 4 PWM channels */
    TRD_PWM_TIM4_CH2,  /* PDF Reference: TIM4 supports 4 PWM channels */
    TRD_PWM_TIM4_CH3,  /* PDF Reference: TIM4 supports 4 PWM channels */
    TRD_PWM_TIM4_CH4,  /* PDF Reference: TIM4 supports 4 PWM channels */
    TRD_PWM_TIM5_CH1,  /* PDF Reference: TIM5 supports 4 PWM channels */
    TRD_PWM_TIM5_CH2,  /* PDF Reference: TIM5 supports 4 PWM channels */
    TRD_PWM_TIM5_CH3,  /* PDF Reference: TIM5 supports 4 PWM channels */
    TRD_PWM_TIM5_CH4,  /* PDF Reference: TIM5 supports 4 PWM channels */
    TRD_PWM_TIM9_CH1,  /* PDF Reference: TIM9 supports 2 PWM channels */
    TRD_PWM_TIM9_CH2,  /* PDF Reference: TIM9 supports 2 PWM channels */
    TRD_PWM_TOTAL_CHANNELS /* Assumed based on MCU conventions */
} TRD_Channel_t;

/* Reserved Timers (PWM-capable but not included in TRD_Channel_t enum for other uses): */
/* TIM10: 1 PWM channel (CH1). /* PDF Reference */ */
/* TIM11: 1 PWM channel (CH1). /* PDF Reference */ */

/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */