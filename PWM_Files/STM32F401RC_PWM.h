/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for the PWM module on STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* Provides project-specific types like tbyte, tlong */
#include "STM32F401RC_GPIO.h" /* Dependency on GPIO configuration */


/*
 * @brief Enumeration of PWM channels supported by Timer 5 (TIM5) on STM32F401RC.
 * Based on STM32F401xC/xE Reference Manual (RM0368).
 */
typedef enum
{
    TIM5_CH1, /* PDF Reference */
    TIM5_CH2, /* PDF Reference */
    TIM5_CH3, /* PDF Reference */
    TIM5_CH4, /* PDF Reference */
    NUM_PWM_CHANNELS /* Total number of channels supported by TIM5 */
} t_pwm_channel;

/*
 * @brief Defines the type used for referring to a PWM channel.
 * Assumed to be defined in STM32F401RC_MAIN.h and compatible with t_pwm_channel.
 * For example: typedef t_pwm_channel TRD_Channel_t;
 */
/* Note: TRD_Channel_t is assumed to be defined in STM32F401RC_MAIN.h */


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz.
 * @param duty: The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief Starts the PWM generation for the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief Stops the PWM generation for the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief Powers off all PWM channels or the entire PWM timer module.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */

/***********************************************************************************************************************
* END OF FILE
***********************************************************************************************************************/