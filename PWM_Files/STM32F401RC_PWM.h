/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : Production-ready header for STM32F401RC GPIO PWM functionality
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_MAIN.h"
/* #include "STM32F401RC_GPIO.h" is typically included if GPIO peripheral structs/macros are needed, */
/* but this file focuses on the PWM interface using a channel enum. */
/* If GPIO definitions were required for pin configuration within the PWM functions, */
/* the relevant header would be included here. As only the channel enum and function */
/* declarations are requested, GPIO header is not strictly needed for this file's purpose. */

/*
 * typedef enum TRD_Channel_t:
 * Identifies all PWM-capable timer channels on STM32F401RC based on RM0368 sections 12, 13, 14.
 * All timers described in the provided RM0368 sections (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
 * support PWM functionality on one or more channels. Therefore, no timers are reserved in this enumeration.
 */
typedef enum TRD_Channel_t
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
    TIM10_CH1, /* PDF Reference */
    TIM11_CH1, /* PDF Reference */
    TRD_CHANNEL_COUNT /* Total number of PWM channels */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/***********************************************************************************************************************
* Function Name  : PWM_Init
* Description    : Initializes a specified PWM channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel to initialize.
* Return Value   : None
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name  : PWM_Set_Freq
* Description    : Sets the frequency and duty cycle for a specified PWM channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel.
*                  tlong frequency - The desired frequency in Hz.
*                  tbyte duty - The desired duty cycle in percent (0-100).
* Return Value   : None
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/***********************************************************************************************************************
* Function Name  : PWM_Start
* Description    : Starts the PWM generation on a specified channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel to start.
* Return Value   : None
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name  : PWM_Stop
* Description    : Stops the PWM generation on a specified channel.
* Arguments      : TRD_Channel_t TRD_Channel - The PWM channel to stop.
* Return Value   : None
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name  : PWM_PowerOff
* Description    : Powers off or disables all configured PWM peripherals.
* Arguments      : None
* Return Value   : None
***********************************************************************************************************************/
void PWM_PowerOff(void);

#endif /* STM32F401RC_GPIO_H_ */