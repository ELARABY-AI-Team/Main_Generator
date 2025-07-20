/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : This header file provides declarations for Pulse Width Modulation (PWM) functionalities.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-20
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef ATMEGA32_PWM_H_
#define ATMEGA32_PWM_H_

#include "ATMEGA32_MAIN.h" /* For standard type definitions like tbyte, tlong, etc. */
#include "ATMEGA32_GPIO.h" /* For GPIO pin configurations related to PWM outputs. */

/**
 * @brief This enumeration identifies the total number of PWM channels supported by the ATMEGA32 microcontroller.
 *        The channels are derived from the Timers that support PWM functionality.
 *        All available PWM-capable channels on the ATMEGA32's general-purpose Timers (Timer0, Timer1, Timer2)
 *        are included as per the requirement to list all PWM-capable timers and channels.
 *        The ATMEGA32 does not possess dedicated non-PWM timers that would need to be reserved from this list.
 */
typedef enum TRD_Channel_t
{
    PWM_CHANNEL_TIM0_OC0,   /* PDF Reference: Timer/Counter0 Output Compare Match A (OC0) */
    PWM_CHANNEL_TIM1_OC1A,  /* PDF Reference: Timer/Counter1 Output Compare Match A (OC1A) */
    PWM_CHANNEL_TIM1_OC1B,  /* PDF Reference: Timer/Counter1 Output Compare Match B (OC1B) */
    PWM_CHANNEL_TIM2_OC2,   /* PDF Reference: Timer/Counter2 Output Compare Match A (OC2) */
    PWM_CHANNEL_COUNT       /* Total number of available PWM channels */
} TRD_Channel_t;

/**
 * @brief Initializes a specified PWM channel.
 *        This function configures the necessary Timer/Counter registers for PWM operation.
 * @param TRD_Channel The PWM channel to initialize.
 * @return void
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specified PWM channel.
 *        This function adjusts the Timer/Counter and Output Compare Registers.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM output frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 * @return void
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on a specified channel.
 * @param TRD_Channel The PWM channel to start.
 * @return void
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on a specified channel.
 * @param TRD_Channel The PWM channel to stop.
 * @return void
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all PWM-related hardware peripherals.
 *        This function disables all Timers used for PWM to minimize power consumption.
 * @param void
 * @return void
 */
void PWM_PowerOff(void);

#endif /* ATMEGA32_PWM_H_ */