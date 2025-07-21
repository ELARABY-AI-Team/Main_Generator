/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for the PWM driver.
*                  Note: Although the prompt initially requested a "GPIO.h" file, the detailed requirements and provided
*                  file structure, enum definition, and function declarations are specifically for a PWM driver.
*                  This file has been generated as "STM32F401RC_PWM.h" based on these detailed PWM-centric instructions.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* For tbyte, tlong, etc. */
#include "STM32F401RC_GPIO.h" /* For GPIO related configurations, if needed by PWM functions */


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * typedef enum TRD_Channel_t:
 * Identifies the total number of channels supported by Timers that support PWM functionality.
 *
 * The STM32F401RC microcontroller (as per RM0368 Rev 5 reference manual) features:
 * - TIM1 (Advanced-control timer) with 4 PWM-capable channels. /* PDF Reference */
 * - TIM2, TIM3, TIM4, TIM5 (General-purpose timers) each with 4 PWM-capable channels. /* PDF Reference */
 * - TIM9 (General-purpose timer) with 2 PWM-capable channels. /* PDF Reference */
 * - TIM10, TIM11 (General-purpose timers) each with 1 PWM-capable channel. /* PDF Reference */
 *
 * Timers TIM8, TIM12, TIM13, and TIM14 are not available in the STM32F401xB/C and STM32F401xD/E
 * variants of the device. Since all available PWM-capable timers are included in the enum,
 * no explicit reservation of channels by non-inclusion was necessary based on the provided documentation. /* PDF Reference */
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1 = 0U,  /* PDF Reference */
    TIM1_CH2,       /* PDF Reference */
    TIM1_CH3,       /* PDF Reference */
    TIM1_CH4,       /* PDF Reference */
    TIM2_CH1,       /* PDF Reference */
    TIM2_CH2,       /* PDF Reference */
    TIM2_CH3,       /* PDF Reference */
    TIM2_CH4,       /* PDF Reference */
    TIM3_CH1,       /* PDF Reference */
    TIM3_CH2,       /* PDF Reference */
    TIM3_CH3,       /* PDF Reference */
    TIM3_CH4,       /* PDF Reference */
    TIM4_CH1,       /* PDF Reference */
    TIM4_CH2,       /* PDF Reference */
    TIM4_CH3,       /* PDF Reference */
    TIM4_CH4,       /* PDF Reference */
    TIM5_CH1,       /* PDF Reference */
    TIM5_CH2,       /* PDF Reference */
    TIM5_CH3,       /* PDF Reference */
    TIM5_CH4,       /* PDF Reference */
    TIM9_CH1,       /* PDF Reference */
    TIM9_CH2,       /* PDF Reference */
    TIM10_CH1,      /* PDF Reference */
    TIM11_CH1,      /* PDF Reference */
    TRD_CHANNEL_MAX /* Sentinel value indicating total number of PWM channels */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */
/*
 * Declarations for PWM control functions.
 * These functions provide an interface for initializing, configuring,
 * starting, and stopping PWM channels according to application requirements.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */