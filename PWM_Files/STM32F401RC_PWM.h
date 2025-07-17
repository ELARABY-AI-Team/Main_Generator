/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM peripheral declarations.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-17
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* For standard type definitions (tbyte, tword, tlong) */
#include "STM32F401RC_GPIO.h" /* Required for GPIO alternate function configuration */

/*
 * TRD_Channel_t: Enumerates all PWM-capable channels on the STM32F401RC microcontroller.
 * This enumeration identifies channels of Timers that support PWM functionality,
 * based on the STM32F401xC/xE Reference Manual (RM0368) and Datasheet (DS9656).
 *
 * Note: TIM6 and TIM7 are basic timers without PWM capability and are typically
 * reserved for OS ticks or delay functions; therefore, their channels are not
 * included in this PWM-specific enumeration.
 */
typedef enum TRD_Channel_t
{
    /* TIM1 - Advanced-control timer with up to 4 PWM channels */
    TIM1_CH1,  /* PDF Reference */
    TIM1_CH2,  /* PDF Reference */
    TIM1_CH3,  /* PDF Reference */
    TIM1_CH4,  /* PDF Reference */

    /* TIM2 - General-purpose timer with up to 4 PWM channels */
    TIM2_CH1,  /* PDF Reference */
    TIM2_CH2,  /* PDF Reference */
    TIM2_CH3,  /* PDF Reference */
    TIM2_CH4,  /* PDF Reference */

    /* TIM3 - General-purpose timer with up to 4 PWM channels */
    TIM3_CH1,  /* PDF Reference */
    TIM3_CH2,  /* PDF Reference */
    TIM3_CH3,  /* PDF Reference */
    TIM3_CH4,  /* PDF Reference */

    /* TIM4 - General-purpose timer with up to 4 PWM channels */
    TIM4_CH1,  /* PDF Reference */
    TIM4_CH2,  /* PDF Reference */
    TIM4_CH3,  /* PDF Reference */
    TIM4_CH4,  /* PDF Reference */

    /* TIM5 - General-purpose timer with up to 4 PWM channels */
    TIM5_CH1,  /* PDF Reference */
    TIM5_CH2,  /* PDF Reference */
    TIM5_CH3,  /* PDF Reference */
    TIM5_CH4,  /* PDF Reference */

    /* TIM9 - General-purpose timer with up to 2 PWM channels */
    TIM9_CH1,  /* PDF Reference */
    TIM9_CH2,  /* PDF Reference */

    /* TIM10 - General-purpose timer with 1 PWM channel */
    TIM10_CH1, /* PDF Reference */

    /* TIM11 - General-purpose timer with 1 PWM channel */
    TIM11_CH1, /* PDF Reference */

    TRD_CHANNEL_COUNT /* Total number of PWM channels defined */
} TRD_Channel_t;

/*
 * @brief Initializes a specified PWM channel.
 * @param TRD_Channel: The PWM channel to initialize.
 * @return void
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief Sets the frequency and duty cycle for a specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired frequency in Hz.
 * @param duty: The desired duty cycle percentage (0-100).
 * @return void
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief Starts PWM generation on a specified channel.
 * @param TRD_Channel: The PWM channel to start.
 * @return void
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief Stops PWM generation on a specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 * @return void
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief Powers off all active PWM peripherals and channels.
 * @return void
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */