/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : This header file provides declarations for PWM module functionalities
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* For project-specific data types (tbyte, tlong, etc.) */
#include "STM32F401RC_GPIO.h" /* For GPIO configuration which is required for alternate function modes */

/*
 * ==================== PWM CHANNEL ENUM ====================
 *
 * This enumeration identifies the total number of PWM-capable channels
 * supported by timers on the STM32F401RC microcontroller, based on the RM0368 Reference Manual.
 *
 * Note on Timer Reservation:
 * According to RM0368, all general-purpose and advanced-control timers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
 * explicitly support PWM functionality. As per the requirements, if no dedicated non-PWM timers
 * exist for OS or delay use, at least two PWM-capable timers (with all their channels) must be
 * reserved by not including them in this enumeration, but still mentioned as comments.
 *
 * Therefore, TIM10 and TIM11, both single-channel timers, have been chosen for this reservation.
 * They are designated for potential dedicated OS or delay usage and are intentionally excluded
 * from the TRD_Channel_t enumeration below.
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
    TRD_CHANNEL_COUNT /* Total number of PWM channels defined in this enumeration */
} TRD_Channel_t;

/*
 * Reserved PWM-capable Timers (not included in TRD_Channel_t for system use, as per requirements):
 * - TIM10_CH1: Single channel, PWM-capable. /* PDF Reference */
 * - TIM11_CH1: Single channel, PWM-capable. /* PDF Reference */
 */

/* ==================== FUNCTION DECLARATIONS ==================== */
/**
 * @brief Initializes a specified PWM channel.
 * @param TRD_Channel: The PWM channel to initialize (e.g., TIM1_CH1).
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on a specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on a specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all active PWM modules.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */