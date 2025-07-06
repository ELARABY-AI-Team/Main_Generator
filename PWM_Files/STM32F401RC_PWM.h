/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : STM32F410RC PWM Driver Header
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F410RC_PWM_H_
#define STM32F410RC_PWM_H_

#include "STM32F410RC_MAIN.h" /* Provides tbyte, tword, tlong, etc. */
#include "STM32F410RC_GPIO.h" /* Required for GPIO configuration (Alternate Functions) */


/* ==================== PWM CHANNEL ENUM (DO NOT MODIFY) ==================== */
/*
 * - typedef enum t_pwm_channel:
 *   - Identify the total number of channels supported by Timer 5 and please mention timer number as a comment
 *   - Use official datasheets or reference manuals of STM32F410RC
 *   - For each channel found for the 5 (e.g., CH1, CH2, ...), generate enum entries
 *   - Each enum entry name must follow the naming conventions typically used for STM32F410RC,
 *     and clearly indicate the timer and channel (e.g., TIMx_CHx or similar per MCU standard)
 *   - Each entry must include a single-line comment:
 *       - /* PDF Reference */ if confirmed from official documentation
 *       - /* Assumed – please verify */ if inferred from context
 *   - The enum must be named `t_pwm_channel` and follow production-grade C conventions.
 */
/* Supported Timer for PWM is TIM5 */
typedef enum
{
    TIM5_CH1 = 0U, /* PDF Reference */
    TIM5_CH2 = 1U, /* PDF Reference */
    TIM5_CH3 = 2U, /* PDF Reference */
    TIM5_CH4 = 3U, /* PDF Reference */
    TIM5_NUM_CHANNELS = 4U /* Total number of TIM5 channels */
} t_pwm_channel;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes the specified PWM channel.
 *
 * @param TRD_Channel The PWM channel to initialize (e.g., TIM5_CH1).
 */
void PWM_Init(t_pwm_channel TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 *
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency   The desired frequency in Hz.
 * @param duty        The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(t_pwm_channel TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on the specified channel.
 *
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(t_pwm_channel TRD_Channel);

/**
 * @brief Stops the PWM generation on the specified channel.
 *
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel TRD_Channel);

/**
 * @brief Powers off all initialized PWM channels and associated timers/GPIO.
 *
 */
void PWM_PowerOff(void);

#endif /* STM32F410RC_PWM_H_ */