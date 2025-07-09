/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for STM32F401RC PWM driver.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/
#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* Include necessary headers */
#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h" /* Assuming this header might be needed for pin configuration details elsewhere, as per prompt */

/*
 * PWM Channel Enumeration
 *
 * This enumeration defines all PWM-capable timer channels available on the STM32F401RC.
 * Based on STM32F401xC/xE Reference Manual (RM0368) and Datasheet (DS9765).
 *
 * Note: Timers TIM10 and TIM11 are reserved and not included in this enumeration
 * to allow for potential dedicated usage (e.g., OS ticks, delays) as per requirements.
 * All other PWM-capable timers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9) and their channels
 * are included.
 */
typedef enum t_pwm_channel
{
    /* TIM1 Channels (Advanced Control Timer) */
    PWM_CHANNEL_TIM1_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM1_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM1_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM1_CH4, /* PDF Reference */

    /* TIM2 Channels (General Purpose Timer) */
    PWM_CHANNEL_TIM2_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM2_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM2_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM2_CH4, /* PDF Reference */

    /* TIM3 Channels (General Purpose Timer) */
    PWM_CHANNEL_TIM3_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM3_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM3_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM3_CH4, /* PDF Reference */

    /* TIM4 Channels (General Purpose Timer) */
    PWM_CHANNEL_TIM4_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM4_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM4_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM4_CH4, /* PDF Reference */

    /* TIM5 Channels (General Purpose Timer) */
    PWM_CHANNEL_TIM5_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM5_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM5_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM5_CH4, /* PDF Reference */

    /* TIM9 Channels (General Purpose Timer) */
    PWM_CHANNEL_TIM9_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM9_CH2, /* PDF Reference */

    /* Total number of defined PWM channels */
    PWM_CHANNEL_COUNT
} t_pwm_channel;

/*
 * Function Declarations
 *
 * Declarations for the Public API of the PWM driver.
 */

/**
 * @brief Initializes a specific PWM channel.
 * @param TRD_Channel: The PWM channel to initialize (type expected from STM32F401RC_MAIN.h).
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 * @param TRD_Channel: The PWM channel to configure (type expected from STM32F401RC_MAIN.h).
 * @param frequency: The desired PWM frequency in Hz (type expected from STM32F401RC_MAIN.h).
 * @param duty: The desired PWM duty cycle in percentage (0-100) (type expected from STM32F401RC_MAIN.h).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on a specific channel.
 * @param TRD_Channel: The PWM channel to start (type expected from STM32F401RC_MAIN.h).
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on a specific channel.
 * @param TRD_Channel: The PWM channel to stop (type expected from STM32F401RC_MAIN.h).
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Turns off power to all PWM-related peripherals (Timers).
 * This function would typically disable clocks and potentially reset timer registers.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */