/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM peripheral declarations.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* Required for standard type definitions (e.g., tbyte, tlong) */
#include "STM32F401RC_GPIO.h" /* Required for GPIO configuration if PWM output pins need to be configured */


/*
 * typedef enum TRD_Channel_t:
 * Identify the total number of channels supported by Timers that support PWM functionality.
 * For each PWM-capable channel, enum entries are generated following STM32F401RC naming conventions.
 * Each entry includes a single-line comment indicating if confirmed from official documentation or inferred.
 */
typedef enum TRD_Channel_t
{
    /* TIM1 PWM Channels */
    TRD_TIM1_CH1, /* PDF Reference: TIM1 has up to 4 independent channels for PWM generation (RM0368, page 244) */
    TRD_TIM1_CH2, /* PDF Reference: TIM1 has up to 4 independent channels for PWM generation (RM0368, page 244) */
    TRD_TIM1_CH3, /* PDF Reference: TIM1 has up to 4 independent channels for PWM generation (RM0368, page 244) */
    TRD_TIM1_CH4, /* PDF Reference: TIM1 has up to 4 independent channels for PWM generation (RM0368, page 244) */

    /* TIM2 PWM Channels */
    TRD_TIM2_CH1, /* PDF Reference: TIM2 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM2_CH2, /* PDF Reference: TIM2 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM2_CH3, /* PDF Reference: TIM2 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM2_CH4, /* PDF Reference: TIM2 has up to 4 independent channels for PWM generation (RM0368, page 317) */

    /* TIM3 PWM Channels */
    TRD_TIM3_CH1, /* PDF Reference: TIM3 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM3_CH2, /* PDF Reference: TIM3 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM3_CH3, /* PDF Reference: TIM3 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM3_CH4, /* PDF Reference: TIM3 has up to 4 independent channels for PWM generation (RM0368, page 317) */

    /* TIM4 PWM Channels */
    TRD_TIM4_CH1, /* PDF Reference: TIM4 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM4_CH2, /* PDF Reference: TIM4 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM4_CH3, /* PDF Reference: TIM4 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM4_CH4, /* PDF Reference: TIM4 has up to 4 independent channels for PWM generation (RM0368, page 317) */

    /* TIM5 PWM Channels */
    TRD_TIM5_CH1, /* PDF Reference: TIM5 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM5_CH2, /* PDF Reference: TIM5 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM5_CH3, /* PDF Reference: TIM5 has up to 4 independent channels for PWM generation (RM0368, page 317) */
    TRD_TIM5_CH4, /* PDF Reference: TIM5 has up to 4 independent channels for PWM generation (RM0368, page 317) */

    /* TIM9 PWM Channels */
    TRD_TIM9_CH1, /* PDF Reference: TIM9 has up to 2 independent channels for PWM generation (RM0368, page 376) */
    TRD_TIM9_CH2, /* PDF Reference: TIM9 has up to 2 independent channels for PWM generation (RM0368, page 376) */

    /* TIM10 PWM Channel */
    TRD_TIM10_CH1, /* PDF Reference: TIM10 has 1 independent channel for PWM generation (RM0368, page 378) */

    /* TIM11 PWM Channel */
    TRD_TIM11_CH1, /* PDF Reference: TIM11 has 1 independent channel for PWM generation (RM0368, page 378) */

    TRD_TOTAL_PWM_CHANNELS /* Total count of PWM channels supported by the MCU. */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel The PWM channel to initialize.
 * @return None
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 * @return None
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 * @return None
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 * @return None
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all PWM peripherals.
 * @param None
 * @return None
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */