/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : PWM (Pulse Width Modulation) driver header file for STM32F401RC microcontroller.
* Author         : Technology Innovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"

/**
 * @brief This enum identifies the total number of PWM-capable channels supported by the
 *        STM32F401RC microcontroller's general-purpose and advanced-control timers.
 *        Channels for TIM10 and TIM11 are intentionally excluded from this enumeration
 *        and reserved for potential future use, such as system timers or other non-PWM
 *        specific functionalities. This reservation ensures flexibility and avoids
 *        accidental allocation for PWM tasks.
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1,  // PDF Reference: Advanced-control timer TIM1, Channel 1
    TIM1_CH2,  // PDF Reference: Advanced-control timer TIM1, Channel 2
    TIM1_CH3,  // PDF Reference: Advanced-control timer TIM1, Channel 3
    TIM1_CH4,  // PDF Reference: Advanced-control timer TIM1, Channel 4
    TIM2_CH1,  // PDF Reference: General-purpose timer TIM2, Channel 1
    TIM2_CH2,  // PDF Reference: General-purpose timer TIM2, Channel 2
    TIM2_CH3,  // PDF Reference: General-purpose timer TIM2, Channel 3
    TIM2_CH4,  // PDF Reference: General-purpose timer TIM2, Channel 4
    TIM3_CH1,  // PDF Reference: General-purpose timer TIM3, Channel 1
    TIM3_CH2,  // PDF Reference: General-purpose timer TIM3, Channel 2
    TIM3_CH3,  // PDF Reference: General-purpose timer TIM3, Channel 3
    TIM3_CH4,  // PDF Reference: General-purpose timer TIM3, Channel 4
    TIM4_CH1,  // PDF Reference: General-purpose timer TIM4, Channel 1
    TIM4_CH2,  // PDF Reference: General-purpose timer TIM4, Channel 2
    TIM4_CH3,  // PDF Reference: General-purpose timer TIM4, Channel 3
    TIM4_CH4,  // PDF Reference: General-purpose timer TIM4, Channel 4
    TIM5_CH1,  // PDF Reference: General-purpose timer TIM5, Channel 1
    TIM5_CH2,  // PDF Reference: General-purpose timer TIM5, Channel 2
    TIM5_CH3,  // PDF Reference: General-purpose timer TIM5, Channel 3
    TIM5_CH4,  // PDF Reference: General-purpose timer TIM5, Channel 4
    TIM9_CH1,  // PDF Reference: General-purpose timer TIM9, Channel 1
    TIM9_CH2,  // PDF Reference: General-purpose timer TIM9, Channel 2
    TRD_CHANNEL_COUNT // Total number of available PWM channels in this enumeration
} TRD_Channel_t;

// The following timers and their channels are reserved and not included in TRD_Channel_t:
// TIM10: General-purpose timer, 1 channel (TIM10_CH1)
// TIM11: General-purpose timer, 1 channel (TIM11_CH1)

/**
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM output for the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM output for the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all active PWM timers and channels.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */