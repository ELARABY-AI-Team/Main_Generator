/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM peripheral declarations and channel definitions.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* MISRA-C:2012 Rule 20.10, 21.1, 21.2: Standard library headers are allowed as they are necessary for common types. */
/* MISRA-C:2012 Rule 20.5: No redefinition of external identifiers will occur. */
#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * TRD_Channel_t: Defines all PWM-capable channels available on the STM32F401RC microcontroller.
 * This enumeration identifies specific timer and channel combinations that can generate PWM signals.
 * All channels listed below are confirmed as PWM-capable from the STM32F401RC Reference Manual (RM0368).
 *
 * Timers like TIM6 and TIM7 are not included in this enumeration as they are basic timers primarily
 * used for delays or DAC triggering and do not support PWM output functionality.
 */
typedef enum
{
    TRD_TIM1_CH1,  /* PDF Reference */
    TRD_TIM1_CH2,  /* PDF Reference */
    TRD_TIM1_CH3,  /* PDF Reference */
    TRD_TIM1_CH4,  /* PDF Reference */

    TRD_TIM2_CH1,  /* PDF Reference */
    TRD_TIM2_CH2,  /* PDF Reference */
    TRD_TIM2_CH3,  /* PDF Reference */
    TRD_TIM2_CH4,  /* PDF Reference */

    TRD_TIM3_CH1,  /* PDF Reference */
    TRD_TIM3_CH2,  /* PDF Reference */
    TRD_TIM3_CH3,  /* PDF Reference */
    TRD_TIM3_CH4,  /* PDF Reference */

    TRD_TIM4_CH1,  /* PDF Reference */
    TRD_TIM4_CH2,  /* PDF Reference */
    TRD_TIM4_CH3,  /* PDF Reference */
    TRD_TIM4_CH4,  /* PDF Reference */

    TRD_TIM5_CH1,  /* PDF Reference */
    TRD_TIM5_CH2,  /* PDF Reference */
    TRD_TIM5_CH3,  /* PDF Reference */
    TRD_TIM5_CH4,  /* PDF Reference */

    TRD_TIM9_CH1,  /* PDF Reference */
    TRD_TIM9_CH2,  /* PDF Reference */

    TRD_TIM10_CH1, /* PDF Reference */

    TRD_TIM11_CH1, /* PDF Reference */

    TRD_PWM_CHANNEL_COUNT, /* Total count of available PWM channels */
    TRD_CHANNEL_UNKNOWN    /* Placeholder for uninitialized or invalid channel */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */
/*
 * @brief   Initializes the specified PWM channel.
 * @param   TRD_Channel: The PWM channel to initialize (e.g., TRD_TIM1_CH1).
 * @return  None.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief   Sets the frequency and duty cycle for a specific PWM channel.
 * @param   TRD_Channel: The PWM channel for which to set parameters.
 * @param   frequency: The desired PWM frequency in Hz.
 * @param   duty: The desired PWM duty cycle in percentage (0-100).
 * @return  None.
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief   Starts PWM signal generation on the specified channel.
 * @param   TRD_Channel: The PWM channel to start.
 * @return  None.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief   Stops PWM signal generation on the specified channel.
 * @param   TRD_Channel: The PWM channel to stop.
 * @return  None.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief   Powers off all active PWM peripherals and channels.
 * @return  None.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */