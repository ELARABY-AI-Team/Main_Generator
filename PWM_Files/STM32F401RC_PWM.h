/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : PWM driver header for STM32F401RC microcontroller
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
#include "STM32F401RC_GPIO.h"


/*
 * PWM CHANNEL ENUM
 *
 * Definition of PWM channels available on the STM32F401RC, based strictly on RM0368 sections 12, 13, and 14.
 * The microcontroller contains the following timers with PWM capability according to the provided PDF:
 * TIM1: CH1, CH2, CH3, CH4 (Section 12.2)
 * TIM2: CH1, CH2, CH3, CH4 (Section 13.2)
 * TIM3: CH1, CH2, CH3, CH4 (Section 13.2)
 * TIM4: CH1, CH2, CH3, CH4 (Section 13.2)
 * TIM5: CH1, CH2, CH3, CH4 (Section 13.2)
 * TIM9: CH1, CH2 (Section 14.2.1)
 * TIM10: CH1 (Section 14.2.2)
 * TIM11: CH1 (Section 14.2.2)
 *
 * According to requirements, and based on the absence of explicitly defined non-PWM timers for OS/delay within the provided PDF content,
 * at least 2 timer instances are reserved and excluded from this enumeration, including all their channels.
 * TIM4 and TIM5 are arbitrarily chosen for this reservation as per this requirement.
 * Reserved Timers (not included in the enum): TIM4 (CH1-CH4), TIM5 (CH1-CH4)
 */
typedef enum TRD_Channel_t
{
    TIM1_CHANNEL_1, /* PDF Reference */
    TIM1_CHANNEL_2, /* PDF Reference */
    TIM1_CHANNEL_3, /* PDF Reference */
    TIM1_CHANNEL_4, /* PDF Reference */
    TIM2_CHANNEL_1, /* PDF Reference */
    TIM2_CHANNEL_2, /* PDF Reference */
    TIM2_CHANNEL_3, /* PDF Reference */
    TIM2_CHANNEL_4, /* PDF Reference */
    TIM3_CHANNEL_1, /* PDF Reference */
    TIM3_CHANNEL_2, /* PDF Reference */
    TIM3_CHANNEL_3, /* PDF Reference */
    TIM3_CHANNEL_4, /* PDF Reference */
    TIM9_CHANNEL_1, /* PDF Reference */
    TIM9_CHANNEL_2, /* PDF Reference */
    TIM10_CHANNEL_1, /* PDF Reference */
    TIM11_CHANNEL_1, /* PDF Reference */
    TRD_TOTAL_PWM_CHANNELS /* Sentinel value */
} TRD_Channel_t;


/*
 * FUNCTION DECLARATIONS
 */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */