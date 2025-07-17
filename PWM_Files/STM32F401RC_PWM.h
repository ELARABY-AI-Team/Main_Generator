/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for the Pulse Width Modulation (PWM) module on the STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-17
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* Includes */
#include "STM32F401RC_MAIN.h" /* Required for standard type definitions (tbyte, tlong, etc.) */
#include "STM32F401RC_GPIO.h" /* Required for GPIO configuration related to PWM channels */


/* ==================== PWM CHANNEL ENUM ==================== */
/**
 * @brief  Enumeration of all PWM-capable timer channels available on the STM32F401RC microcontroller.
 *         This enumeration identifies all channels from general-purpose and advanced-control timers
 *         that can be configured to generate Pulse Width Modulation signals.
 *
 * @note   This list is derived from the STM32F401RC Reference Manual (RM0368) and Datasheet (DS10629).
 *         Timers TIM6 and TIM7 are basic timers and do not provide PWM output functionality;
 *         they are typically used for time-base generation or as triggers for other peripherals.
 */
typedef enum TRD_Channel_t
{
    /* TIM1 Advanced-control timer channels (4 channels) */
    TRD_PWM_CHANNEL_TIM1_CH1,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */
    TRD_PWM_CHANNEL_TIM1_CH2,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */
    TRD_PWM_CHANNEL_TIM1_CH3,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */
    TRD_PWM_CHANNEL_TIM1_CH4,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */

    /* TIM2 General-purpose timer channels (4 channels) */
    TRD_PWM_CHANNEL_TIM2_CH1,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */
    TRD_PWM_CHANNEL_TIM2_CH2,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */
    TRD_PWM_CHANNEL_TIM2_CH3,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */
    TRD_PWM_CHANNEL_TIM2_CH4,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */

    /* TIM3 General-purpose timer channels (4 channels) */
    TRD_PWM_CHANNEL_TIM3_CH1,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */
    TRD_PWM_CHANNEL_TIM3_CH2,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */
    TRD_PWM_CHANNEL_TIM3_CH3,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */
    TRD_PWM_CHANNEL_TIM3_CH4,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */

    /* TIM4 General-purpose timer channels (4 channels) */
    TRD_PWM_CHANNEL_TIM4_CH1,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */
    TRD_PWM_CHANNEL_TIM4_CH2,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */
    TRD_PWM_CHANNEL_TIM4_CH3,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */
    TRD_PWM_CHANNEL_TIM4_CH4,  /* PDF Reference: RM0368 Rev 7, Section 14.3.1 */

    /* TIM5 General-purpose timer channels (4 channels) */
    TRD_PWM_CHANNEL_TIM5_CH1,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */
    TRD_PWM_CHANNEL_TIM5_CH2,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */
    TRD_PWM_CHANNEL_TIM5_CH3,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */
    TRD_PWM_CHANNEL_TIM5_CH4,  /* PDF Reference: RM0368 Rev 7, Section 13.3.1 */

    /* TIM8 Advanced-control timer channels (4 channels) */
    TRD_PWM_CHANNEL_TIM8_CH1,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */
    TRD_PWM_CHANNEL_TIM8_CH2,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */
    TRD_PWM_CHANNEL_TIM8_CH3,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */
    TRD_PWM_CHANNEL_TIM8_CH4,  /* PDF Reference: RM0368 Rev 7, Section 12.3.1 */

    /* TIM9 General-purpose timer channels (2 channels) */
    TRD_PWM_CHANNEL_TIM9_CH1,  /* PDF Reference: RM0368 Rev 7, Section 15.3.1 */
    TRD_PWM_CHANNEL_TIM9_CH2,  /* PDF Reference: RM0368 Rev 7, Section 15.3.1 */

    /* TIM10 General-purpose timer channel (1 channel) */
    TRD_PWM_CHANNEL_TIM10_CH1, /* PDF Reference: RM0368 Rev 7, Section 15.3.1 */

    /* TIM11 General-purpose timer channel (1 channel) */
    TRD_PWM_CHANNEL_TIM11_CH1, /* PDF Reference: RM0368 Rev 7, Section 15.3.1 */

    /* Total count of PWM channels defined in this enumeration for bounds checking */
    TRD_PWM_CHANNEL_COUNT      /* This entry must always be the last */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief  Initializes a specific PWM channel with default settings.
 * @param  TRD_Channel: The PWM channel to be initialized.
 * @return None.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief  Sets the frequency and duty cycle for a specific PWM channel.
 * @param  TRD_Channel: The PWM channel to configure.
 * @param  frequency: The desired PWM signal frequency in Hz (Hertz).
 * @param  duty: The desired PWM signal duty cycle in percentage (0-100).
 * @return None.
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief  Starts the PWM generation on a specific channel.
 * @param  TRD_Channel: The PWM channel to enable and start outputting a signal.
 * @return None.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief  Stops the PWM generation on a specific channel.
 * @param  TRD_Channel: The PWM channel to disable and stop outputting a signal.
 * @return None.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief  Powers off all PWM modules and related peripherals.
 *         This function ensures all PWM timer clocks are disabled and associated
 *         GPIO pins are reset to a safe default state to conserve power.
 * @param  None.
 * @return None.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */