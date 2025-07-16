/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM peripheral configuration and control for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* PDF Reference */
#include "STM32F401RC_GPIO.h" /* Assumed based on file structure requirement */


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * This enumeration identifies the total number of channels supported by
 * Timers that support PWM functionality on the STM32F401RC microcontroller.
 *
 * All PWM-capable timers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
 * are identified in the provided reference manual (RM0368).
 *
 * As per requirements, if no dedicated non-PWM timers exist for system use
 * (e.g., OS ticks, delays), at least 2 timers (with all their channels) should
 * be excluded from this enumeration and marked as reserved.
 *
 * For this implementation, TIM10 and TIM11 (both single-channel timers)
 * are designated as reserved for potential system-level uses.
 */
typedef enum TRD_Channel_t
{
    TRD_CHANNEL_TIM1_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM1_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM1_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM1_CH4,  /* PDF Reference */
    TRD_CHANNEL_TIM2_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM2_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM2_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM2_CH4,  /* PDF Reference */
    TRD_CHANNEL_TIM3_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM3_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM3_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM3_CH4,  /* PDF Reference */
    TRD_CHANNEL_TIM4_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM4_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM4_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM4_CH4,  /* PDF Reference */
    TRD_CHANNEL_TIM5_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM5_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM5_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM5_CH4,  /* PDF Reference */
    TRD_CHANNEL_TIM9_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM9_CH2   /* PDF Reference */
} TRD_Channel_t;

/*
 * Reserved Timers for potential system use (e.g., OS ticks, delays), not included in TRD_Channel_t:
 * TIM10_CH1: /* PDF Reference */
 * TIM11_CH1: /* PDF Reference */
 */


/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */