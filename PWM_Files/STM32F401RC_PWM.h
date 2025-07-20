/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : Header file for PWM peripheral management on STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/*
 * @brief Note: The request initially specified "GPIO.h" as the target file name,
 *              however, the provided file structure, content requirements (PWM channel enum, PWM functions),
 *              and included comments clearly indicate that the intent was to generate a "PWM.h" header file.
 *              This file is generated as "PWM.h" to align with the detailed requirements for PWM functionality.
 */

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* Include necessary dependencies */
#include "STM32F401RC_MAIN.h" /* For custom types like tbyte, tlong */
#include "STM32F401RC_GPIO.h" /* For GPIO pin configuration, as PWM outputs are GPIO pins */


/* ==================== PWM CHANNEL ENUM ==================== */
/*
 * @brief TRD_Channel_t: Enumerates all PWM-capable timer channels available on the STM32F401RC.
 *        This enumeration is based on the STM32F401RC Reference Manual (RM0368) and Datasheet (DS10619).
 *        The STM32F401RC microcontroller includes dedicated non-PWM timers such as TIM6, TIM7 (basic timers),
 *        and SysTick (system timer) which are typically used for OS tasks, delays, or internal triggers.
 *        As the MCU has these dedicated non-PWM timers, all identified PWM-capable timers and their channels
 *        are included in this enum as per the requirements. No PWM-capable timers are reserved from the enum.
 */
typedef enum TRD_Channel_t
{
    TRD_CHANNEL_TIM1_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM1_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM1_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM1_CH4,  /* PDF Reference */

    TRD_CHANNEL_TIM2_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM2_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM2_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM2_CH4,  /* PDF Reference */

    TRD_CHANNEL_TIM3_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM3_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM3_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM3_CH4,  /* PDF Reference */

    TRD_CHANNEL_TIM4_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM4_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM4_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM4_CH4,  /* PDF Reference */

    TRD_CHANNEL_TIM5_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM5_CH2,  /* PDF Reference */
    TRD_CHANNEL_TIM5_CH3,  /* PDF Reference */
    TRD_CHANNEL_TIM5_CH4,  /* PDF Reference */

    TRD_CHANNEL_TIM9_CH1,  /* PDF Reference */
    TRD_CHANNEL_TIM9_CH2,  /* PDF Reference */

    TRD_CHANNEL_TIM10_CH1, /* PDF Reference */

    TRD_CHANNEL_TIM11_CH1, /* PDF Reference */

    TRD_CHANNEL_COUNT      /* Total number of PWM channels defined in this enumeration */
} TRD_Channel_t;

/*
 * @brief Non-PWM capable timers and their channels which are not included in TRD_Channel_t enum:
 *        TIM6: Basic timer with no I/O channels. Primarily used for DAC trigger and update event generation.
 *        TIM7: Basic timer with no I/O channels. Primarily used for DAC trigger and update event generation.
 *        SysTick: System Timer, dedicated for operating system time-keeping and delay functions.
 */


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a specified PWM channel with default settings.
 * @param TRD_Channel: The specific PWM channel to be initialized.
 * @pre   The corresponding timer's clock must be enabled prior to calling this function.
 * @pre   The GPIO pin associated with this PWM channel must be configured as Alternate Function output.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief Sets the desired frequency and duty cycle for a specified PWM channel.
 * @param TRD_Channel: The specific PWM channel to configure.
 * @param frequency:   The desired frequency in Hertz (Hz).
 * @param duty:        The desired duty cycle as a percentage (0-100%).
 * @pre   The PWM channel must have been previously initialized by calling PWM_Init().
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief Starts the PWM signal generation on a specified channel.
 * @param TRD_Channel: The specific PWM channel to start.
 * @pre   The PWM channel must be initialized and configured for operation.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief Stops the PWM signal generation on a specified channel.
 * @param TRD_Channel: The specific PWM channel to stop.
 * @pre   The PWM channel must be actively generating a signal.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief Powers off all PWM peripherals managed by this module.
 *        This function disables clocks and resets peripheral registers for all timers
 *        that might have been actively used for PWM generation, ensuring low power consumption.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */