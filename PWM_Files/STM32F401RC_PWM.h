/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : Header file for PWM functionality on STM32F410RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F410RC_PWM_H_
#define STM32F410RC_PWM_H_

/* Include necessary headers */
#include "STM32F410RC_MAIN.h"
#include "STM32F410RC_GPIO.h"


/* ==================== PWM CHANNEL ENUM (DO NOT MODIFY) ==================== */
/*
 * - typedef enum t_pwm_channel:
 *   - Identify the total number of channels supported by Timer 5 on STM32F410RC.
 *   - Timer 5 (TIM5) supports 4 channels for Input Capture/Output Compare/PWM.
 *   - Use official datasheets or reference manuals of STM32F410RC.
 *   - For each channel found for the 5 (e.g., CH1, CH2, ...), generate enum entries.
 *   - Each enum entry name must follow the naming conventions typically used for STM32F410RC,
 *     and clearly indicate the timer and channel (e.g., TIMx_CHx or similar per MCU standard).
 *   - Each entry must include a single-line comment:
 *       - /* PDF Reference */ if confirmed from official documentation
 *       - /* Assumed – please verify */ if inferred from context
 *   - The enum must be named `t_pwm_channel` and follow production-grade C conventions.
 */

/**
 * @brief Enumeration for identifying PWM channels on Timer 5.
 */
typedef enum
{
    TIM5_CH1 = 0u,  /* PDF Reference */
    TIM5_CH2 = 1u,  /* PDF Reference */
    TIM5_CH3 = 2u,  /* PDF Reference */
    TIM5_CH4 = 3u,  /* PDF Reference */
    TIM5_CHANNEL_COUNT = 4u /* PDF Reference */
} t_pwm_channel;

/*
 * @brief Define a type for PWM channels for clarity in function signatures.
 * Note: Assumed TRD_Channel_t is intended to be a typedef for t_pwm_channel
 * based on the function declarations provided in the requirements.
 * This typedef is defined here to satisfy the function signature requirement.
 */
typedef t_pwm_channel TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel: The PWM channel to initialize (e.g., TIM5_CH1).
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hertz (Hz).
 * @param duty: The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all active PWM channels and associated timer peripherals.
 */
void PWM_PowerOff(void);

#endif /* STM32F410RC_PWM_H_ */