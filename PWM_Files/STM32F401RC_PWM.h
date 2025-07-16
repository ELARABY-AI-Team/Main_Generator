/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM peripheral declarations.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/*
 * @brief This header file implements PWM-related declarations for STM32F401RC.
 *        It follows the provided template structure, which explicitly defines content
 *        for 'STM32F401RC_PWM.h' despite a conflicting instruction for 'GPIO.h'
 *        in the overall prompt.
 */

#include "STM32F401RC_MAIN.h" /* PDF Reference */
#include "STM32F401RC_GPIO.h" /* PDF Reference */


/*
 * @brief Identifies the total number of channels supported by Timers that support PWM functionality.
 *        Two general-purpose timers (TIM10 and TIM11) are intentionally excluded from this
 *        enumeration to reserve them for potential other purposes (e.g., OS ticks, delays),
 *        as per design requirements.
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
    TIM9_CH2  /* PDF Reference */
    /* TIM10 and TIM11 are reserved for other purposes (e.g., OS ticks, delays) as per requirements. */
    /* TIM10 has 1 PWM-capable channel (CH1). */
    /* TIM11 has 1 PWM-capable channel (CH1). */
} TRD_Channel_t;


/*
 * @brief Initializes a specified PWM channel with default parameters.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief Sets the frequency and duty cycle for a specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief Starts the PWM generation on a specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief Stops the PWM generation on a specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief Powers off all active PWM channels and related timer peripherals.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */