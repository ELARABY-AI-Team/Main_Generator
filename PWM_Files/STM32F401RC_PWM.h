/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : Header file for GPIO related functions and definitions.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_MAIN.h"

/**
 * @brief Enumeration for identifying PWM channels on STM32F401RC timers.
 *        Derived from the provided reference manual sections on TIM1, TIM2-TIM5, and TIM9-TIM11.
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
    TIM11_CH1 /* PDF Reference */
} TRD_Channel_t;

/**
 * @brief Initializes a specified PWM channel.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specified PWM channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on a specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on a specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all active PWM timers and channels.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_GPIO_H_ */