/***********************************************************************************************************************
* File Name      : STM32F410RC_GPIO.h
* Description    : PWM header file for STM32F410RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/*
* NOTE TO USER:
* but contains definitions and declarations specifically for PWM functionality
* using Timer 5, as detailed in the requirements.
*
* The #ifndef and #endif guards use different symbols
* (#ifndef STM32F410RC_PWM_H_ and #endif /* STM32F410RC_GPIO_H_ */ )
* as strictly specified in the provided template structure.
* While unconventional, this structure is followed precisely.
*/

#ifndef STM32F410RC_PWM_H_
#define STM32F410RC_PWM_H_

#include "STM32F410RC_MAIN.h"
/*
 * The requirement structure included "#include "STM32F410RC_GPIO.h"".
 * As this is the STM32F410RC_GPIO.h file itself,
 * including itself is omitted to prevent circular dependencies.
 */


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * Identify the total number of channels supported by Timer 5 (4 Channels)
 * Referencing STM32F410RC Reference Manual (RM0383)
 */
typedef enum t_pwm_channel
{
    TIM5_CH1 = 0, /* PDF Reference - RM0383 Section 17 */
    TIM5_CH2, /* PDF Reference - RM0383 Section 17 */
    TIM5_CH3, /* PDF Reference - RM0383 Section 17 */
    TIM5_CH4 /* PDF Reference - RM0383 Section 17 */
} t_pwm_channel;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a specific PWM channel on Timer 5.
 * @param TRD_Channel: The PWM channel to initialize (e.g., TIM5_CH1).
 */
void PWM_Init(t_pwm_channel TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz.
 * @param duty: The desired duty cycle in a value from 0 to 255 (assuming tbyte range).
 */
void PWM_Set_Freq(t_pwm_channel TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM output on a specific channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(t_pwm_channel TRD_Channel);

/**
 * @brief Stops the PWM output on a specific channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel TRD_Channel);

/**
 * @brief Powers off the Timer 5 peripheral used for PWM.
 */
void PWM_PowerOff(void);

#endif /* STM32F410RC_GPIO_H_ */