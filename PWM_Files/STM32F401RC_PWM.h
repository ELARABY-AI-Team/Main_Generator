/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM peripheral functionality.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-22
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* MISRA-C:2012 Rule 21.1 violation: #include directives for standard libraries. */
/* Justification: Standard library includes are necessary for common data types (if not custom defined in MAIN.h) or utilities. */

/* MISRA-C:2012 Rule 20.2 violation: Re-use of C standard library names. */
/* Justification: The use of standard header <stdint.h> is a common practice for fixed-width integer types.
                  Assuming tbyte, tword, tlong are custom types from STM32F401RC_MAIN.h as per requirements. */

#include "STM32F401RC_MAIN.h" /* PDF Reference */
#include "STM32F401RC_GPIO.h" /* PDF Reference */


/*
 * Timer reservation logic:
 * The provided documentation (RM0368) describes all available General-purpose (TIM2-TIM5, TIM9-TIM11)
 * and Advanced-control (TIM1) timers as being capable of PWM generation.
 * As no dedicated non-PWM timers for OS or delay use are explicitly mentioned in the provided context,
 * two timers (TIM10 and TIM11) are reserved from the available PWM-capable timers to fulfill the
 * requirement of reserving at least two timers with all their channels.
 */

/**
 * @brief  Enumeration to identify the total number of PWM channels supported by the MCU.
 *         Each entry indicates the timer and channel used for PWM.
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
    /*
     * TIM10 and TIM11 are available on STM32F401RC and are PWM-capable (TIM10_CH1, TIM11_CH1).
     * However, as per requirements, these two timers are reserved and not included in this enumeration
     * for potential dedicated non-PWM use (e.g., OS ticks or delays).
     * No dedicated non-PWM timers were explicitly identified in the provided documentation,
     * so two general-purpose timers are chosen for this reservation.
     */
    NUMBER_OF_PWM_CHANNELS /* Enum entry to represent the total count of defined PWM channels. */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief  Initializes a specified PWM channel.
 * @param  TRD_Channel: The PWM channel to initialize.
 * @retval None
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief  Sets the frequency and duty cycle for a specified PWM channel.
 * @param  TRD_Channel: The PWM channel to configure.
 * @param  frequency: The desired frequency in Hz.
 * @param  duty: The desired duty cycle in percentage (0-100).
 * @retval None
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief  Starts PWM generation on a specified channel.
 * @param  TRD_Channel: The PWM channel to start.
 * @retval None
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief  Stops PWM generation on a specified channel.
 * @param  TRD_Channel: The PWM channel to stop.
 * @retval None
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief  Powers off all PWM peripherals (timers used for PWM).
 * @param  None
 * @retval None
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */