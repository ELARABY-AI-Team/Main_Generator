/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM peripheral driver on STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * TRD_Channel_t: Identifies PWM channels available on STM32F401RC Timers.
 * This enum lists channels confirmed to support PWM functionality
 * based on the provided reference manual (RM0368 Rev 5).
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1,       /* PDF Reference */
    TIM1_CH2,       /* PDF Reference */
    TIM1_CH3,       /* PDF Reference */
    TIM1_CH4,       /* PDF Reference */
    TIM2_CH1,       /* PDF Reference */
    TIM2_CH2,       /* PDF Reference */
    TIM2_CH3,       /* PDF Reference */
    TIM2_CH4,       /* PDF Reference */
    TIM3_CH1,       /* PDF Reference */
    TIM3_CH2,       /* PDF Reference */
    TIM3_CH3,       /* PDF Reference */
    TIM3_CH4,       /* PDF Reference */
    TIM4_CH1,       /* PDF Reference */
    TIM4_CH2,       /* PDF Reference */
    TIM4_CH3,       /* PDF Reference */
    TIM4_CH4,       /* PDF Reference */
    TIM5_CH1,       /* PDF Reference */
    TIM5_CH2,       /* PDF Reference */
    TIM5_CH3,       /* PDF Reference */
    TIM5_CH4,       /* PDF Reference */
    TIM9_CH1,       /* PDF Reference */
    TIM9_CH2,       /* PDF Reference */
    TIM10_CH1,      /* PDF Reference */
    TIM11_CH1,      /* PDF Reference */
    _TRD_TOTAL_CHANNELS /* Total number of PWM channels */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/***********************************************************************************************************************
* Function Name: PWM_Init
* Description  : Initializes a specific timer channel for PWM output.
* Arguments    : TRD_Channel - The PWM channel to initialize (e.g., TIM1_CH1).
* Return Value : None.
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_Set_Freq
* Description  : Sets the frequency and duty cycle for a specific PWM channel.
* Arguments    : TRD_Channel - The PWM channel.
*              : frequency   - The desired frequency in Hz.
*              : duty        - The desired duty cycle in percentage (0-100).
* Return Value : None.
* Note         : Actual frequency and duty cycle may vary based on timer clock and resolution.
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/***********************************************************************************************************************
* Function Name: PWM_Start
* Description  : Starts the PWM generation on a specific channel.
* Arguments    : TRD_Channel - The PWM channel to start.
* Return Value : None.
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_Stop
* Description  : Stops the PWM generation on a specific channel.
* Arguments    : TRD_Channel - The PWM channel to stop.
* Return Value : None.
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_PowerOff
* Description  : Powers off/disables all PWM outputs.
* Arguments    : None.
* Return Value : None.
***********************************************************************************************************************/
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */