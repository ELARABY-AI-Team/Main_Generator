/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : Header file for PWM peripheral driver.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/*
* IMPORTANT NOTE: The initial request asked for a "GPIO.h" file, but the provided template,
* file structure, and specific requirements (PWM channel enum, PWM specific functions)
* indicate that a "PWM.h" header file is intended. This file is generated as "PWM.h"
* based on the content of the provided requirements.
*/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* PDF Reference */
#include "STM32F401RC_GPIO.h" /* PDF Reference */


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * TRD_Channel_t: Enumerates all PWM-capable channels for STM32F401RC.
 * Based on RM0368 Reference Manual, all Timers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
 * support PWM functionality on their respective channels.
 *
 * For system-level or operating system (OS) delays, it is common practice to reserve
 * certain timers to prevent conflicts with general-purpose PWM usage.
 * TIM10 (single channel) and TIM11 (single channel) are designated for such purposes
 * (e.g., OS ticks, non-PWM delays) and are therefore not included in this enumeration.
 * This reservation fulfills the requirement to reserve at least 2 timers.
 */
typedef enum TRD_Channel_t
{
    TRD_TIM1_CH1 = 0U,  /* PDF Reference */
    TRD_TIM1_CH2,       /* PDF Reference */
    TRD_TIM1_CH3,       /* PDF Reference */
    TRD_TIM1_CH4,       /* PDF Reference */
    TRD_TIM2_CH1,       /* PDF Reference */
    TRD_TIM2_CH2,       /* PDF Reference */
    TRD_TIM2_CH3,       /* PDF Reference */
    TRD_TIM2_CH4,       /* PDF Reference */
    TRD_TIM3_CH1,       /* PDF Reference */
    TRD_TIM3_CH2,       /* PDF Reference */
    TRD_TIM3_CH3,       /* PDF Reference */
    TRD_TIM3_CH4,       /* PDF Reference */
    TRD_TIM4_CH1,       /* PDF Reference */
    TRD_TIM4_CH2,       /* PDF Reference */
    TRD_TIM4_CH3,       /* PDF Reference */
    TRD_TIM4_CH4,       /* PDF Reference */
    TRD_TIM5_CH1,       /* PDF Reference */
    TRD_TIM5_CH2,       /* PDF Reference */
    TRD_TIM5_CH3,       /* PDF Reference */
    TRD_TIM5_CH4,       /* PDF Reference */
    TRD_TIM9_CH1,       /* PDF Reference */
    TRD_TIM9_CH2,       /* PDF Reference */
    TRD_CHANNEL_COUNT   /* Sentinel value for total number of PWM channels */
} TRD_Channel_t;

/* Reserved Timers for System/OS use (not included in TRD_Channel_t enum): */
/* TIM10_CH1 - Reserved for system/OS use. (PDF Reference) */
/* TIM11_CH1 - Reserved for system/OS use. (PDF Reference) */


/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */