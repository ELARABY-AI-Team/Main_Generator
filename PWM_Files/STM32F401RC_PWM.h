/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : PWM Channel Definitions and Function Declarations
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* Includes tbyte, tword, tlong types */
#include "STM32F401RC_GPIO.h" /* Assumed based on MCU conventions for pin configuration */

/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * Identification of PWM-capable timer channels for STM32F401RC.
 * Identified channels are based on the provided reference manual text sections:
 * Section 12 (TIM1), Section 13 (TIM2-5), Section 14 (TIM9-11).
 * Note: All PWM-capable channels found in these sections are included per requirements.
 */
typedef enum TRD_Channel_t
{
    /* TIM1 Channels (Advanced-control timer) */
    TRD_TIM1_CH1 = 0,  /* PDF Reference (Section 12) */
    TRD_TIM1_CH2,      /* PDF Reference (Section 12) */
    TRD_TIM1_CH3,      /* PDF Reference (Section 12) */
    TRD_TIM1_CH4,      /* PDF Reference (Section 12) */

    /* TIM2 Channels (General-purpose timer, 32-bit) */
    TRD_TIM2_CH1,      /* PDF Reference (Section 13) */
    TRD_TIM2_CH2,      /* PDF Reference (Section 13) */
    TRD_TIM2_CH3,      /* PDF Reference (Section 13) */
    TRD_TIM2_CH4,      /* PDF Reference (Section 13) */

    /* TIM3 Channels (General-purpose timer, 16-bit) */
    TRD_TIM3_CH1,      /* PDF Reference (Section 13) */
    TRD_TIM3_CH2,      /* PDF Reference (Section 13) */
    TRD_TIM3_CH3,      /* PDF Reference (Section 13) */
    TRD_TIM3_CH4,      /* PDF Reference (Section 13) */

    /* TIM4 Channels (General-purpose timer, 16-bit) */
    TRD_TIM4_CH1,      /* PDF Reference (Section 13) */
    TRD_TIM4_CH2,      /* PDF Reference (Section 13) */
    TRD_TIM4_CH3,      /* PDF Reference (Section 13) */
    TRD_TIM4_CH4,      /* PDF Reference (Section 13) */

    /* TIM5 Channels (General-purpose timer, 32-bit) */
    TRD_TIM5_CH1,      /* PDF Reference (Section 13) */
    TRD_TIM5_CH2,      /* PDF Reference (Section 13) */
    TRD_TIM5_CH3,      /* PDF Reference (Section 13) */
    TRD_TIM5_CH4,      /* PDF Reference (Section 13) */

    /* TIM9 Channels (General-purpose timer, 16-bit) */
    TRD_TIM9_CH1,      /* PDF Reference (Section 14) */
    TRD_TIM9_CH2,      /* PDF Reference (Section 14) */

    /* TIM10 Channels (General-purpose timer, 16-bit) */
    TRD_TIM10_CH1,     /* PDF Reference (Section 14) */

    /* TIM11 Channels (General-purpose timer, 16-bit) */
    TRD_TIM11_CH1,     /* PDF Reference (Section 14) */

    TRD_TOTAL_CHANNELS /* Total count of available PWM channels */

} TRD_Channel_t;

/* ==================== FUNCTION DECLARATIONS ==================== */
/*
 * @brief  Initializes a specific PWM channel.
 * @param  TRD_Channel: The PWM channel to initialize.
 * @return None
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief  Sets the frequency and duty cycle for a specific PWM channel.
 * @param  TRD_Channel: The PWM channel to configure.
 * @param  frequency: The desired frequency in Hz.
 * @param  duty: The desired duty cycle in percentage (0-100).
 * @return None
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief  Starts the PWM output on a specific channel.
 * @param  TRD_Channel: The PWM channel to start.
 * @return None
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief  Stops the PWM output on a specific channel.
 * @param  TRD_Channel: The PWM channel to stop.
 * @return None
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief  Powers off all initialized PWM timers and channels.
 * @return None
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */