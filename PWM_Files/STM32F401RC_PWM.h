/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : PWM driver header
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

/*
 * TRD_Channel_t: Identifies PWM channels available on the device.
 * This enum lists PWM-capable channels from TIM1, TIM2, TIM3, TIM4, TIM5, and TIM9 as found in the reference manual.
 * TIM10 (1 channel) and TIM11 (1 channel) are also PWM-capable timers as per the reference manual but are intentionally
 * not included in this enum to fulfill the requirement to reserve at least 2 timers (with all their channels).
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

    TRD_TOTAL_PWM_CHANNELS /* Delimiter for total count */

} TRD_Channel_t;

/*
 * Reserved Timers (not included in TRD_Channel_t as per requirement):
 * TIM10 (1 channel): TIM10_CH1 (PWM capable)
 * TIM11 (1 channel): TIM11_CH1 (PWM capable)
 */

/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a specific PWM channel.
 *
 * This function configures the necessary timer and GPIO for the specified PWM channel.
 * It sets up the timer's clock source, prescaler, auto-reload value, and output compare mode
 * for PWM generation. The specific parameters like frequency and duty cycle are set
 * using the PWM_Set_Freq function after initialization.
 *
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 *
 * This function calculates and sets the appropriate timer register values (ARR, CCRx)
 * to achieve the desired frequency and duty cycle for the specified PWM channel.
 * The timer must be initialized before calling this function.
 *
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM output on a specific channel.
 *
 * This function enables the counter and the output compare channel, allowing the
 * PWM signal to be generated on the corresponding GPIO pin.
 *
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM output on a specific channel.
 *
 * This function disables the output compare channel, stopping the PWM signal.
 * The timer counter may continue running depending on configuration.
 *
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all active PWM timers and associated outputs.
 *
 * This function disables the clock to the timers used for PWM and ensures all
 * PWM outputs are forced to a safe inactive state.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */