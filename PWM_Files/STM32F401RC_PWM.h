/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM functionality for STM32F401RC
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


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * Definition of PWM channels available on STM32F401RC Timers.
 * Identified timers with PWM capability based on provided RM0368 reference manual sections 12, 13, and 14.
 * TIM1: Advanced-control timer with up to 4 independent channels for PWM generation (Section 12).
 * TIM2-5: General-purpose timers with up to 4 independent channels for PWM generation (Section 13).
 * TIM9: General-purpose timer with up to 2 independent channels for PWM generation (Section 14).
 * TIM10-11: General-purpose timers with 1 independent channel for PWM generation (Section 14).
 * TIM8, TIM12, TIM13, TIM14 are stated as not available in STM32F401xB/C/xD/E variants in the provided PDF.
 * This enumeration lists all identified PWM-capable channels from the available timers.
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1,  // PDF Reference
    TIM1_CH2,  // PDF Reference
    TIM1_CH3,  // PDF Reference
    TIM1_CH4,  // PDF Reference
    TIM2_CH1,  // PDF Reference
    TIM2_CH2,  // PDF Reference
    TIM2_CH3,  // PDF Reference
    TIM2_CH4,  // PDF Reference
    TIM3_CH1,  // PDF Reference
    TIM3_CH2,  // PDF Reference
    TIM3_CH3,  // PDF Reference
    TIM3_CH4,  // PDF Reference
    TIM4_CH1,  // PDF Reference
    TIM4_CH2,  // PDF Reference
    TIM4_CH3,  // PDF Reference
    TIM4_CH4,  // PDF Reference
    TIM5_CH1,  // PDF Reference
    TIM5_CH2,  // PDF Reference
    TIM5_CH3,  // PDF Reference
    TIM5_CH4,  // PDF Reference
    TIM9_CH1,  // PDF Reference
    TIM9_CH2,  // PDF Reference
    TIM10_CH1, // PDF Reference
    TIM11_CH1, // PDF Reference
    TRD_Channel_Count // Total number of PWM channels identified
} TRD_Channel_t;


 /* ==================== FUNCTION DECLARATIONS ==================== */

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
 * @brief Starts the specified PWM channel output.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the specified PWM channel output.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all configured PWM timers and channels.
 */
void PWM_PowerOff(void);


#endif /* STM32F401RC_PWM_H_ */