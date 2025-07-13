/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for STM32F401RC PWM functionality
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

// Enum identifying PWM-capable channels across supported timers.
// This enum lists all timer channels on STM32F401RC devices that support PWM output functionality.
// Based on the STM32F401RC Reference Manual (RM0368).
// Timers TIM6 and TIM7 are Basic timers without PWM output capability.
// Timers TIM8, TIM12, TIM13, TIM14 are not available on STM32F401RC devices (RM0368).
typedef enum TRD_Channel_t
{
    // TIM1 Channels (Advanced-control timer, supports PWM on all channels)
    TIM1_CH1, /* PDF Reference */ // RM0368, Ch 12, p. 244
    TIM1_CH2, /* PDF Reference */ // RM0368, Ch 12, p. 244
    TIM1_CH3, /* PDF Reference */ // RM0368, Ch 12, p. 244
    TIM1_CH4, /* PDF Reference */ // RM0368, Ch 12, p. 244
    // TIM2 Channels (General-purpose timer, supports PWM on all channels)
    TIM2_CH1, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM2_CH2, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM2_CH3, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM2_CH4, /* PDF Reference */ // RM0368, Ch 13, p. 316
    // TIM3 Channels (General-purpose timer, supports PWM on all channels)
    TIM3_CH1, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM3_CH2, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM3_CH3, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM3_CH4, /* PDF Reference */ // RM0368, Ch 13, p. 316
    // TIM4 Channels (General-purpose timer, supports PWM on all channels)
    TIM4_CH1, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM4_CH2, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM4_CH3, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM4_CH4, /* PDF Reference */ // RM0368, Ch 13, p. 316
    // TIM5 Channels (General-purpose timer, supports PWM on all channels)
    TIM5_CH1, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM5_CH2, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM5_CH3, /* PDF Reference */ // RM0368, Ch 13, p. 316
    TIM5_CH4, /* PDF Reference */ // RM0368, Ch 13, p. 316
    // TIM9 Channels (General-purpose timer, supports PWM on both channels)
    TIM9_CH1, /* PDF Reference */ // RM0368, Ch 14, p. 376
    TIM9_CH2, /* PDF Reference */ // RM0368, Ch 14, p. 376
    // TIM10 Channels (General-purpose timer, supports PWM on channel 1)
    TIM10_CH1, /* PDF Reference */ // RM0368, Ch 14, p. 377
    // TIM11 Channels (General-purpose timer, supports PWM on channel 1)
    TIM11_CH1, /* PDF Reference */ // RM0368, Ch 14, p. 377

    TRD_TOTAL_PWM_CHANNELS // Sentinel value for the total count of PWM channels listed
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes the specified PWM channel for operation.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired output frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all initialized PWM peripherals.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */