/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for STM32F401RC PWM functionality.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"

/*
 * TRD_Channel_t: Identifies available PWM-capable timer channels.
 * Based on STM32F401RC Reference Manual/Datasheet.
 * Timers TIM9 and TIM10 are reserved for potential OS or delay functions
 * and their channels are therefore commented out below.
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

    // TIM9_CH1, /* Reserved for OS/Delay */
    // TIM9_CH2, /* Reserved for OS/Delay */

    // TIM10_CH1, /* Reserved for OS/Delay */

    TIM11_CH1, /* PDF Reference */

    TRD_TOTAL_CHANNELS /* Total number of available PWM channels */

} TRD_Channel_t;

/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * Initialize a specific PWM channel.
 *
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * Set the frequency and duty cycle for a specific PWM channel.
 *
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * Start PWM generation on a specific channel.
 *
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * Stop PWM generation on a specific channel.
 *
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * Power off all initialized PWM peripherals.
 * This function disables the clocks to the relevant timers.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */