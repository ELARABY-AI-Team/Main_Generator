/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : PWM driver header file for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* PDF Reference */
#include "STM32F401RC_GPIO.h" /* PDF Reference */

/*
 * The following enumeration lists all Timer Channels on the STM32F401RC
 * capable of generating PWM signals, based on the provided RM0368 Reference Manual sections.
 * PWM capabilities were identified for TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, and TIM11.
 * TIM12, TIM13, and TIM14 are mentioned as not available in STM32F401xB/C/xD/E.
 * PWM capability for each timer channel is based on descriptions mentioning
 * "PWM generation" and the presence of Output Compare Mode (OCxM) bits
 * in the Capture/Compare Mode Registers (TIMx_CCMRx).
 */
typedef enum TRD_Channel_t
{
    /* TIM1: Advanced-control timer with up to 4 PWM channels (RM0368 Sec 12) */
    TRD_TIM1_CH1, /* PDF Reference */
    TRD_TIM1_CH2, /* PDF Reference */
    TRD_TIM1_CH3, /* PDF Reference */
    TRD_TIM1_CH4, /* PDF Reference */

    /* TIM2: General-purpose timer with up to 4 PWM channels (RM0368 Sec 13) */
    TRD_TIM2_CH1, /* PDF Reference */
    TRD_TIM2_CH2, /* PDF Reference */
    TRD_TIM2_CH3, /* PDF Reference */
    TRD_TIM2_CH4, /* PDF Reference */

    /* TIM3: General-purpose timer with up to 4 PWM channels (RM0368 Sec 13) */
    TRD_TIM3_CH1, /* PDF Reference */
    TRD_TIM3_CH2, /* PDF Reference */
    TRD_TIM3_CH3, /* PDF Reference */
    TRD_TIM3_CH4, /* PDF Reference */

    /* TIM4: General-purpose timer with up to 4 PWM channels (RM0368 Sec 13) */
    TRD_TIM4_CH1, /* PDF Reference */
    TRD_TIM4_CH2, /* PDF Reference */
    TRD_TIM4_CH3, /* PDF Reference */
    TRD_TIM4_CH4, /* PDF Reference */

    /* TIM5: General-purpose timer with up to 4 PWM channels (RM0368 Sec 13) */
    TRD_TIM5_CH1, /* PDF Reference */
    TRD_TIM5_CH2, /* PDF Reference */
    TRD_TIM5_CH3, /* PDF Reference */
    TRD_TIM5_CH4, /* PDF Reference */

    /* TIM9: General-purpose timer with up to 2 PWM channels (RM0368 Sec 14) */
    TRD_TIM9_CH1, /* PDF Reference */
    TRD_TIM9_CH2, /* PDF Reference */

    /* TIM10: General-purpose timer with 1 PWM channel (RM0368 Sec 14) */
    TRD_TIM10_CH1, /* PDF Reference */

    /* TIM11: General-purpose timer with 1 PWM channel (RM0368 Sec 14) */
    TRD_TIM11_CH1, /* PDF Reference */

    /* Total number of PWM channels */
    TRD_TOTAL_PWM_CHANNELS /* PDF Reference */

} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/***********************************************************************************************************************
* Function Name  : PWM_Init
* Description    : Initializes a specific PWM channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel to initialize.
* Return Value   : None
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name  : PWM_Set_Freq
* Description    : Sets the frequency and duty cycle for a specific PWM channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel.
*                : tlong frequency             - The desired frequency in Hz.
*                : tbyte duty                  - The desired duty cycle in percentage (0-100).
* Return Value   : None
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/***********************************************************************************************************************
* Function Name  : PWM_Start
* Description    : Starts the PWM generation on a specific channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel to start.
* Return Value   : None
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name  : PWM_Stop
* Description    : Stops the PWM generation on a specific channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel to stop.
* Return Value   : None
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name  : PWM_PowerOff
* Description    : Powers off all active PWM peripherals.
* Arguments      : None
* Return Value   : None
***********************************************************************************************************************/
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */