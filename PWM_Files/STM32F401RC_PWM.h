/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : This file contains declarations for PWM driver functions for the STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"


/**
 * @brief This enumeration identifies the total number of channels supported by
 *        Timers that support PWM functionality on STM32F401RC.
 *        As per design requirements, at least two PWM-capable timers are
 *        reserved for potential system/OS use or other dedicated functionalities,
 *        and are therefore not included in this enumeration.
 *        This reservation strategy is applied because the STM32F401RC does not
 *        feature dedicated non-PWM basic timers (e.g., TIM6, TIM7) for such purposes
 *        as implied by the provided RM0368 Rev 5 document.
 */
typedef enum TRD_Channel_t
{
    /* TIM1 PWM Channels (Advanced-control timer) */
    TRD_CHANNEL_TIM1_CH1 = 0U, /* PDF Reference - RM0368 Rev 5, Section 12.2 */
    TRD_CHANNEL_TIM1_CH2,      /* PDF Reference - RM0368 Rev 5, Section 12.2 */
    TRD_CHANNEL_TIM1_CH3,      /* PDF Reference - RM0368 Rev 5, Section 12.2 */
    TRD_CHANNEL_TIM1_CH4,      /* PDF Reference - RM0368 Rev 5, Section 12.2 */

    /* TIM4 PWM Channels (General-purpose timer) */
    TRD_CHANNEL_TIM4_CH1,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */
    TRD_CHANNEL_TIM4_CH2,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */
    TRD_CHANNEL_TIM4_CH3,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */
    TRD_CHANNEL_TIM4_CH4,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */

    /* TIM5 PWM Channels (General-purpose timer) */
    TRD_CHANNEL_TIM5_CH1,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */
    TRD_CHANNEL_TIM5_CH2,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */
    TRD_CHANNEL_TIM5_CH3,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */
    TRD_CHANNEL_TIM5_CH4,      /* PDF Reference - RM0368 Rev 5, Section 13.2 */

    /* TIM9 PWM Channels (General-purpose timer) */
    TRD_CHANNEL_TIM9_CH1,      /* PDF Reference - RM0368 Rev 5, Section 14.2.1 */
    TRD_CHANNEL_TIM9_CH2,      /* PDF Reference - RM0368 Rev 5, Section 14.2.1 */

    /* TIM10 PWM Channel (General-purpose timer) */
    TRD_CHANNEL_TIM10_CH1,     /* PDF Reference - RM0368 Rev 5, Section 14.2.2 */

    /* TIM11 PWM Channel (General-purpose timer) */
    TRD_CHANNEL_TIM11_CH1,     /* PDF Reference - RM0368 Rev 5, Section 14.2.2 */

    TRD_CHANNEL_MAX_COUNT      /* Denotes the total count of available PWM channels for this driver */
} TRD_Channel_t;

/*
 * The following timers are available on STM32F401RC and are PWM-capable,
 * but are intentionally reserved for other system functionalities (e.g., OS ticks,
 * system delays, or other non-PWM specific timer operations) as per design decision,
 * and are thus not included in the TRD_Channel_t enumeration:
 *
 * - TIM2: All 4 channels (CH1, CH2, CH3, CH4) are PWM capable. /* PDF Reference - RM0368 Rev 5, Section 13.2 */
 * - TIM3: All 4 channels (CH1, CH2, CH3, CH4) are PWM capable. /* PDF Reference - RM0368 Rev 5, Section 13.2 */
 *
 * Timers TIM8, TIM12, TIM13, and TIM14 are explicitly stated as not available
 * in STM32F401xB/C and STM32F401xD/E variants in the provided RM0368 Rev 5 document.
 * Timers TIM6 and TIM7 are typically basic timers in other STM32 families but are
 * not documented as available for STM32F401RC in RM0368 Rev 5.
 */


/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */