/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : Header file for the PWM driver.
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
 * This enum identifies the available PWM channels on the STM32F401RC microcontroller.
 * Based on the provided RM0368 Rev 5 reference manual sections detailing timers:
 * - Section 12: TIM1 (Advanced-control timer) supports PWM and has up to 4 independent channels.
 * - Section 13: TIM2 to TIM5 (General-purpose timers) support PWM and have up to 4 independent channels each.
 * - Section 14: TIM9, TIM10, TIM11 (General-purpose timers) support PWM. TIM9 has up to 2 channels, TIM10 and TIM11 have 1 channel each.
 * No specific timers or channels are excluded from this list as the provided documentation indicates their PWM capability.
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1, /* PDF Reference */
    TIM1_CH2, /* PDF Reference */
    TIM1_CH3, /* PDF Reference */
    TIM1_CH4, /* PDF Reference */
    TIM2_CH1, /* PDF Reference */
    TIM2_CH2, /* PDF Reference */
    TIM2_CH3, /* PDF Reference */
    TIM2_CH4, /* PDF Reference */
    TIM3_CH1, /* PDF Reference */
    TIM3_CH2, /* PDF Reference */
    TIM3_CH3, /* PDF Reference */
    TIM3_CH4, /* PDF Reference */
    TIM4_CH1, /* PDF Reference */
    TIM4_CH2, /* PDF Reference */
    TIM4_CH3, /* PDF Reference */
    TIM4_CH4, /* PDF Reference */
    TIM5_CH1, /* PDF Reference */
    TIM5_CH2, /* PDF Reference */
    TIM5_CH3, /* PDF Reference */
    TIM5_CH4, /* PDF Reference */
    TIM9_CH1, /* PDF Reference */
    TIM9_CH2, /* PDF Reference */
    TIM10_CH1, /* PDF Reference */
    TIM11_CH1, /* PDF Reference */
    TRD_TOTAL_CHANNELS /* Sentinel value for the total number of defined channels */
} TRD_Channel_t;

/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a specific PWM channel.
 * @param TRD_Channel: The PWM channel to initialize (e.g., TIM1_CH1).
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired output frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0 to 100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts PWM generation on a specific channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops PWM generation on a specific channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all PWM timers and associated peripherals.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */