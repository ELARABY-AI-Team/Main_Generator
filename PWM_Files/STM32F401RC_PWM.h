/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_ /* File guard corrected to match the requested guard */
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"

/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * TRD_Channel_t identifies PWM-capable timer channels for STM32F401RC based on RM0368 reference.
 * The following timers and channels are confirmed to support PWM generation based on the provided document sections:
 * TIM1: Channels 1, 2, 3, 4
 * TIM2, TIM3, TIM4, TIM5: Channels 1, 2, 3, 4
 * TIM9: Channels 1, 2
 * TIM10, TIM11: Channel 1
 * Based on the provided RM0368 excerpts, all available timers (TIM1, 2, 3, 4, 5, 9, 10, 11) for this device variant support PWM.
 * Therefore, all PWM-capable channels mentioned are included in this enumeration, and no timers are reserved as comments as per the document analysis.
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1 = 0u,    /* PDF Reference */
    TIM1_CH2,         /* PDF Reference */
    TIM1_CH3,         /* PDF Reference */
    TIM1_CH4,         /* PDF Reference */
    TIM2_CH1,         /* PDF Reference */
    TIM2_CH2,         /* PDF Reference */
    TIM2_CH3,         /* PDF Reference */
    TIM2_CH4,         /* PDF Reference */
    TIM3_CH1,         /* PDF Reference */
    TIM3_CH2,         /* PDF Reference */
    TIM3_CH3,         /* PDF Reference */
    TIM3_CH4,         /* PDF Reference */
    TIM4_CH1,         /* PDF Reference */
    TIM4_CH2,         /* PDF Reference */
    TIM4_CH3,         /* PDF Reference */
    TIM4_CH4,         /* PDF Reference */
    TIM5_CH1,         /* PDF Reference */
    TIM5_CH2,         /* PDF Reference */
    TIM5_CH3,         /* PDF Reference */
    TIM5_CH4,         /* PDF Reference */
    TIM9_CH1,         /* PDF Reference */
    TIM9_CH2,         /* PDF Reference */
    TIM10_CH1,        /* PDF Reference */
    TIM11_CH1,        /* PDF Reference */
    NUM_TRD_CHANNELS /* Total number of PWM channels */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */ /* File guard corrected to match the #ifndef */