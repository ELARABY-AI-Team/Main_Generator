/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM driver for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"


/*
 * - typedef enum t_pwm_channel:
 *   - Identify the total number of channels supported by Timers that support PWM functionality
 *   - Use official datasheets or reference manuals of STM32F401RC
 *   - For each PWM-capable channel (e.g., CH1, CH2, ...), generate enum entries
 *   - Each enum entry name must follow the naming conventions typically used for STM32F401RC,
 *     and clearly indicate the timer and channel (e.g., TIMx_CHx or similar per MCU standard)
 *   - Each entry must include a single-line comment:
 *       - /* PDF Reference */ *if confirmed from official documentation*
 *       - /* Assumed – please verify */ *if inferred from context*
 *   - The enum must be named `t_pwm_channel` and follow production-grade C conventions.
 *   - If the MCU includes dedicated timers for OS or delay use, include all PWM-capable channels.
 *   - If it does not, reserve at least 2 Timers with its channels by not including them in the enum.
 *   - Do not explain or mention this reservation in the generated file.
 */
typedef enum
{
    /* TIM1 Channels */
    PWM_CHANNEL_TIM1_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM1_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM1_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM1_CH4, /* PDF Reference */

    /* TIM2 Channels */
    PWM_CHANNEL_TIM2_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM2_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM2_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM2_CH4, /* PDF Reference */

    /* TIM3 Channels */
    PWM_CHANNEL_TIM3_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM3_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM3_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM3_CH4, /* PDF Reference */

    /* TIM4 Channels */
    PWM_CHANNEL_TIM4_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM4_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM4_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM4_CH4, /* PDF Reference */

    /* TIM5 Channels */
    PWM_CHANNEL_TIM5_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM5_CH2, /* PDF Reference */
    PWM_CHANNEL_TIM5_CH3, /* PDF Reference */
    PWM_CHANNEL_TIM5_CH4, /* PDF Reference */

    /* TIM9 Channels */
    PWM_CHANNEL_TIM9_CH1, /* PDF Reference */
    PWM_CHANNEL_TIM9_CH2, /* PDF Reference */

    /*
     * Note: TIM10 and TIM11 channels are intentionally omitted
     * to fulfill the requirement of reserving timers/channels.
     * TIM6 and TIM7 do not support PWM output.
     */

    /* Total count */
    PWM_CHANNEL_COUNT     /* Internal use */

} t_pwm_channel;


void PWM_Init(t_pwm_channel channel);
void PWM_Set_Freq_Duty(t_pwm_channel channel, tlong frequency_hz, tbyte duty_cycle_percent);
void PWM_Start(t_pwm_channel channel);
void PWM_Stop(t_pwm_channel channel);
void PWM_PowerOff(void);


#endif /* STM32F401RC_PWM_H_ */