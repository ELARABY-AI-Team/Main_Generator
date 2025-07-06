/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : Header file for PWM driver on STM32F410RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2024-07-30
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F410RC_PWM_H_
#define STM32F410RC_PWM_H_

/* Included files */
#include "STM32F410RC_MAIN.h"
#include "STM32F410RC_GPIO.h"


/* ==================== PWM CHANNEL ENUM (DO NOT MODIFY) ==================== */
/* Enumeration for identifying abstract PWM channels used by the system. */
/* These map to specific timer channels (TIMx_CHy) and GPIO pins in the implementation. */
typedef enum
{
    /* Example logical PWM channels. Number and mapping must be verified against application requirements. */
    PWM_CH1 = 0, /* Assumed – please verify */
    PWM_CH2,     /* Assumed – please verify */
    PWM_CH3,     /* Assumed – please verify */
    PWM_CH4,     /* Assumed – please verify */
    PWM_CH5,     /* Assumed – please verify */
    PWM_CH6,     /* Assumed – please verify */
    PWM_CH7,     /* Assumed – please verify */
    PWM_CH8,     /* Assumed – please verify */
    PWM_CHANNEL_COUNT /* Total number of defined PWM channels */
} TRD_Channel_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a specific PWM channel.
 * @param TRD_Channel The specific PWM channel to initialize (from TRD_Channel_t enum).
 *        Mapping to actual hardware timer channel (TIMx_CHy) and GPIO pin is handled in implementation.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 *        This function should only be called after PWM_Init for the specified channel.
 * @param TRD_Channel The specific PWM channel to configure (from TRD_Channel_t enum).
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief Starts the PWM output on a specific channel.
 *        Enables the timer and output compare channel for the specified PWM output.
 * @param TRD_Channel The specific PWM channel to start (from TRD_Channel_t enum).
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief Stops the PWM output on a specific channel.
 *        Disables the output compare channel for the specified PWM output.
 * @param TRD_Channel The specific PWM channel to stop (from TRD_Channel_t enum).
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief Powers off all configured PWM channels and related timers/peripherals.
 *        Resets the PWM driver state and peripheral clocks if necessary.
 */
void PWM_PowerOff(void);


#endif /* STM32F410RC_PWM_H_ */