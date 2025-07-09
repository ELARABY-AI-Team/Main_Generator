/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : PWM driver header file for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* Includes */
#include "STM32F401RC_MAIN.h" /* Required for tbyte, tlong, etc. */
#include "STM32F401RC_GPIO.h" /* Required for GPIO related definitions if needed internally by implementation */


/*
 * This enum lists all PWM-capable timer channels available on STM32F401RC,
 * with the exception of TIM10 and TIM11, which are reserved for other
 * system uses (e.g., OS ticks, delays) within this project's framework
 * and are explicitly excluded from this PWM driver's control.
 */
typedef enum
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

    /*
     * Reserved Timers (Excluded from this enum):
     * TIM10 (1 channel)
     * TIM11 (1 channel)
     * TIM14 (Basic Timer, no PWM)
     */

    NUM_PWM_CHANNELS /* Total number of channels included in this enum */
} t_pwm_channel;


/* Function Declarations */
/*
 * @brief Initializes the specified PWM channel.
 * @param channel: The PWM channel to initialize (e.g., TIM1_CH1).
 */
void PWM_Init(t_pwm_channel channel);

/*
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param channel: The PWM channel to configure.
 * @param frequency: The desired frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(t_pwm_channel channel, tlong frequency, tbyte duty);

/*
 * @brief Starts the PWM generation on the specified channel.
 * @param channel: The PWM channel to start.
 */
void PWM_Start(t_pwm_channel channel);

/*
 * @brief Stops the PWM generation on the specified channel.
 * @param channel: The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel channel);

/*
 * @brief Powers off all active PWM timers/channels managed by this driver.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */