/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for the PWM driver
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-10
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"

/*
 * TRD_Channel_t: Enum to identify available PWM channels.
 * Based on the provided RM0368 reference manual sections describing TIM1, TIM2-5, and TIM9-11.
 * Each entry corresponds to a specific Timer and Channel capable of PWM output.
 *
 * Note: Timers TIM10 and TIM11 (with their respective channels) are intentionally
 * excluded from this enum definition as per requirements, but listed below as a comment.
 * These timers are reserved for other potential uses such as time-base or calibration,
 * separate from the application's main PWM requirements.
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1,  /* PDF Reference */
    TIM1_CH2,  /* PDF Reference */
    TIM1_CH3,  /* PDF Reference */
    TIM1_CH4,  /* PDF Reference */
    TIM2_CH1,  /* PDF Reference */
    TIM2_CH2,  /* PDF Reference */
    TIM2_CH3,  /* PDF Reference */
    TIM2_CH4,  /* PDF Reference */
    TIM3_CH1,  /* PDF Reference */
    TIM3_CH2,  /* PDF Reference */
    TIM3_CH3,  /* PDF Reference */
    TIM3_CH4,  /* PDF Reference */
    TIM4_CH1,  /* PDF Reference */
    TIM4_CH2,  /* PDF Reference */
    TIM4_CH3,  /* PDF Reference */
    TIM4_CH4,  /* PDF Reference */
    TIM5_CH1,  /* PDF Reference */
    TIM5_CH2,  /* PDF Reference */
    TIM5_CH3,  /* PDF Reference */
    TIM5_CH4,  /* PDF Reference */
    TIM9_CH1,  /* PDF Reference */
    TIM9_CH2,  /* PDF Reference */
    TRD_Channel_Count /* Total number of defined PWM channels */
} TRD_Channel_t;

/*
 * Reserved Timers (not included in TRD_Channel_t enum):
 * TIM10_CH1  /* PDF Reference * /
 * TIM11_CH1  /* PDF Reference * /
 */


/* ==================== FUNCTION DECLARATIONS ==================== */

/***********************************************************************************************************************
* Function Name: PWM_Init
* Description  : Initializes a specific PWM channel.
* Arguments    : TRD_Channel - The PWM channel to initialize.
* Return Value : None.
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_Set_Freq
* Description  : Sets the frequency and duty cycle for a specific PWM channel.
* Arguments    : TRD_Channel - The PWM channel.
*              : frequency   - The desired PWM frequency in Hz.
*              : duty        - The desired PWM duty cycle in percentage (0-100).
* Return Value : None.
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
* Description  : Powers off all PWM-related timer peripherals.
* Arguments    : None.
* Return Value : None.
***********************************************************************************************************************/
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */
/***********************************************************************************************************************
* End Of File
***********************************************************************************************************************/