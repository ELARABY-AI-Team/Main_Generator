/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM functionality on STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
* 
* Note: This header file implements the requirements for PWM functions.
* The requested filename was GPIO.h, but the content and function names clearly indicate a PWM header.
* The include guard and endif directive are adjusted to reflect PWM.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * typedef enum TRD_Channel_t:
 * Defines the PWM-capable channels available on the STM32F401RC timers.
 * Based on the provided RM0368 reference manual excerpts for TIM1, TIM2-5, and TIM9-11.
 * All channels confirmed to support PWM generation are included.
 * No timers are reserved as PWM-capable timers exist and the requirement is to include all in this case.
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1,  /* PDF Reference */
    TIM1_CH2,  /* PDF Reference */
    TIM1_CH3,  /* PDF Reference */
    TIM1_CH4,  /* PDF Reference */
    TIM2_CH1,  /* PDF Reference */
    TIM2_CH2,  /* PDF Reference */
    TIM2_CH3,  /* PDF Reference */
    TIM2_CH4,  /* PDF Reference */
    TIM3_CH1,  /* PDF Reference */
    TIM3_CH2,  /* PDF Reference */
    TIM3_CH3,  /* PDF Reference */
    TIM3_CH4,  /* PDF Reference */
    TIM4_CH1,  /* PDF Reference */
    TIM4_CH2,  /* PDF Reference */
    TIM4_CH3,  /* PDF Reference */
    TIM4_CH4,  /* PDF Reference */
    TIM5_CH1,  /* PDF Reference */
    TIM5_CH2,  /* PDF Reference */
    TIM5_CH3,  /* PDF Reference */
    TIM5_CH4,  /* PDF Reference */
    TIM9_CH1,  /* PDF Reference */
    TIM9_CH2,  /* PDF Reference */
    TIM10_CH1, /* PDF Reference */
    TIM11_CH1  /* PDF Reference */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */
/*
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired output frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief Starts the PWM output on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief Stops the PWM output on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief Powers off all configured PWM timers/channels.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */