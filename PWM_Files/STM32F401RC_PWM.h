/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM functionality on STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* Include necessary microcontroller-specific and general header files. */
#include "STM32F401RC_MAIN.h" /* For standard type definitions like tbyte, tword, tlong. */
#include "STM32F401RC_GPIO.h" /* For GPIO related configurations, potentially used by PWM alternate functions. */


/**
 * @brief Defines the available PWM channels on the STM32F401RC microcontroller.
 *        This enumeration lists all timer channels that support PWM functionality
 *        as identified from the provided RM0368 reference manual sections 12, 13, and 14.
 * @note  Timers TIM6 and TIM7 are basic timers not capable of PWM (RM0368 Section 15).
 *        The SysTick timer is a dedicated core timer and does not support PWM.
 *        Therefore, all PWM-capable timers are included in this enumeration without reservation.
 */
typedef enum TRD_Channel_t
{
    /* Advanced-control timer (TIM1) - Supports 4 independent PWM channels. */
    TRD_TIM1_CH1,  /* PDF Reference */
    TRD_TIM1_CH2,  /* PDF Reference */
    TRD_TIM1_CH3,  /* PDF Reference */
    TRD_TIM1_CH4,  /* PDF Reference */

    /* General-purpose timers (TIM2 to TIM5) - Each supports 4 independent PWM channels. */
    TRD_TIM2_CH1,  /* PDF Reference */
    TRD_TIM2_CH2,  /* PDF Reference */
    TRD_TIM2_CH3,  /* PDF Reference */
    TRD_TIM2_CH4,  /* PDF Reference */
    TRD_TIM3_CH1,  /* PDF Reference */
    TRD_TIM3_CH2,  /* PDF Reference */
    TRD_TIM3_CH3,  /* PDF Reference */
    TRD_TIM3_CH4,  /* PDF Reference */
    TRD_TIM4_CH1,  /* PDF Reference */
    TRD_TIM4_CH2,  /* PDF Reference */
    TRD_TIM4_CH3,  /* PDF Reference */
    TRD_TIM4_CH4,  /* PDF Reference */
    TRD_TIM5_CH1,  /* PDF Reference */
    TRD_TIM5_CH2,  /* PDF Reference */
    TRD_TIM5_CH3,  /* PDF Reference */
    TRD_TIM5_CH4,  /* PDF Reference */

    /* General-purpose timers (TIM9 to TIM11) - Support various numbers of independent PWM channels. */
    TRD_TIM9_CH1,  /* PDF Reference */
    TRD_TIM9_CH2,  /* PDF Reference */
    TRD_TIM10_CH1, /* PDF Reference */
    TRD_TIM11_CH1  /* PDF Reference */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a specific PWM channel with default settings.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts PWM generation on a specific channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops PWM generation on a specific channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all configured PWM channels and their associated timers.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */