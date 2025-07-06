/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : Header file for PWM peripheral driver for STM32F410RC
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

/*
 * Note on TRD_Channel_t:
 * The function prototypes specified in the requirements use 'TRD_Channel_t' for the channel parameter type.
 * It is assumed that 'TRD_Channel_t' is an alias for, or compatible with, the enum 't_pwm_channel' defined below,
 * or is defined elsewhere (e.g., in STM32F410RC_MAIN.h) and used consistently.
 * despite the enum definition being named 't_pwm_channel'.
 */


/* ==================== PWM CHANNEL ENUM (DO NOT MODIFY) ==================== */
/*
 * - Identify the total number of channels supported by various Timers on STM32F410RC
 * - Timers supporting PWM on STM32F410RC based on RM0390: TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11, TIM14.
 * - Use official datasheets or reference manuals of STM32F410RC
 * - For each channel found for the UNKNOWN (e.g., CH1, CH2, ...), generate enum entries
 * - Each enum entry name must follow the naming conventions typically used for STM32F410RC,
 *   and clearly indicate the timer and channel (e.g., TIMx_CHx or similar per MCU standard)
 * - Each entry must include a single-line comment:
 *       - /* PDF Reference *\/ if confirmed from official documentation
 *       - /* Assumed – please verify *\/ if inferred from context
 * - The enum must be named `t_pwm_channel` and follow production-grade C conventions.
 */
typedef enum t_pwm_channel
{
    TIM1_CH1 = 0, /* PDF Reference */
    TIM1_CH2,     /* PDF Reference */
    TIM1_CH3,     /* PDF Reference */
    TIM1_CH4,     /* PDF Reference */

    TIM2_CH1,     /* PDF Reference */
    TIM2_CH2,     /* PDF Reference */
    TIM2_CH3,     /* PDF Reference */
    TIM2_CH4,     /* PDF Reference */

    TIM3_CH1,     /* PDF Reference */
    TIM3_CH2,     /* PDF Reference */
    TIM3_CH3,     /* PDF Reference */
    TIM3_CH4,     /* PDF Reference */

    TIM4_CH1,     /* PDF Reference */
    TIM4_CH2,     /* PDF Reference */
    TIM4_CH3,     /* PDF Reference */
    TIM4_CH4,     /* PDF Reference */

    TIM5_CH1,     /* PDF Reference */
    TIM5_CH2,     /* PDF Reference */
    TIM5_CH3,     /* PDF Reference */
    TIM5_CH4,     /* PDF Reference */

    TIM9_CH1,     /* PDF Reference */
    TIM9_CH2,     /* PDF Reference */

    TIM10_CH1,    /* PDF Reference */

    TIM11_CH1,    /* PDF Reference */

    TIM14_CH1,    /* PDF Reference */

    PWM_CHANNEL_COUNT /* Total number of available PWM channels */

} t_pwm_channel;


/* ==================== FUNCTION DECLARATIONS ==================== */

/***********************************************************************************************************************
* Function Name: PWM_Init
* Description  : Initializes the specified PWM channel with default parameters.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to initialize.
* Return Value : None
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_Set_Freq
* Description  : Sets the frequency and duty cycle for the specified PWM channel.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to configure.
*              : tlong frequency - The desired frequency in Hz.
*              : tbyte duty - The desired duty cycle in percentage (0-100).
* Return Value : None
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/***********************************************************************************************************************
* Function Name: PWM_Start
* Description  : Starts the PWM generation on the specified channel.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to start.
* Return Value : None
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_Stop
* Description  : Stops the PWM generation on the specified channel.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to stop.
* Return Value : None
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_PowerOff
* Description  : Powers off the PWM peripheral(s) (e.g., disables clock).
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void PWM_PowerOff(void);

#endif /* STM32F410RC_PWM_H_ */

/***********************************************************************************************************************
* End Of File
***********************************************************************************************************************/