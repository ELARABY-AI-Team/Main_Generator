/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/* Include required header files for base types and GPIO functionality. */
#include "STM32F401RC_MAIN.h" /* Assumed based on MCU conventions */
#include "STM32F401RC_GPIO.h" /* Assumed based on MCU conventions */


/*
 * TRD_Channel_t: Enum to identify PWM-capable channels.
 *
 * This enumeration identifies the total number of channels supported by Timers that
 * support PWM functionality on the STM32F401RC microcontroller, as derived from RM0368 Rev 5.
 *
 * Note on Timer Reservation:
 * The project requirements state that if the MCU does not include dedicated non-PWM timers
 * for OS or delay use, at least two timers (with all their channels) must be reserved
 * by not including them in this enumeration, but listing them as comments.
 * Based on the provided documentation (RM0368 Rev 5), all general-purpose and advanced-control timers
 * (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11) on the STM32F401RC support PWM functionality.
 * There are no dedicated non-PWM timers specified in the provided reference for this MCU.
 *
 * Therefore, to comply with the reservation requirement, TIM10 and TIM11 (which have a single PWM channel each)
 * are hereby reserved from this enumeration for potential alternative system-level functions
 * (e.g., OS ticks, delays, or other non-PWM specific features).
 * This reservation should be considered in the overall system design.
 */
typedef enum TRD_Channel_t
{
    /* TIM1 channels (Advanced-control timer, 16-bit, up to 4 channels) */
    TRD_TIM1_CH1,  /* PDF Reference */
    TRD_TIM1_CH2,  /* PDF Reference */
    TRD_TIM1_CH3,  /* PDF Reference */
    TRD_TIM1_CH4,  /* PDF Reference */

    /* TIM2 channels (General-purpose timer, 32-bit, up to 4 channels) */
    TRD_TIM2_CH1,  /* PDF Reference */
    TRD_TIM2_CH2,  /* PDF Reference */
    TRD_TIM2_CH3,  /* PDF Reference */
    TRD_TIM2_CH4,  /* PDF Reference */

    /* TIM3 channels (General-purpose timer, 16-bit, up to 4 channels) */
    TRD_TIM3_CH1,  /* PDF Reference */
    TRD_TIM3_CH2,  /* PDF Reference */
    TRD_TIM3_CH3,  /* PDF Reference */
    TRD_TIM3_CH4,  /* PDF Reference */

    /* TIM4 channels (General-purpose timer, 16-bit, up to 4 channels) */
    TRD_TIM4_CH1,  /* PDF Reference */
    TRD_TIM4_CH2,  /* PDF Reference */
    TRD_TIM4_CH3,  /* PDF Reference */
    TRD_TIM4_CH4,  /* PDF Reference */

    /* TIM5 channels (General-purpose timer, 32-bit, up to 4 channels) */
    TRD_TIM5_CH1,  /* PDF Reference */
    TRD_TIM5_CH2,  /* PDF Reference */
    TRD_TIM5_CH3,  /* PDF Reference */
    TRD_TIM5_CH4,  /* PDF Reference */

    /* TIM9 channels (General-purpose timer, 16-bit, up to 2 channels) */
    TRD_TIM9_CH1,  /* PDF Reference */
    TRD_TIM9_CH2,  /* PDF Reference */

    TRD_CHANNEL_COUNT /* Assumed based on MCU conventions */
} TRD_Channel_t;

/*
 * Reserved Timers (not included in TRD_Channel_t for general PWM use):
 *
 * - TIM10 (General-purpose timer, 16-bit, 1 PWM channel: CH1)  /* PDF Reference */
 *   Reason for reservation: Reserved for dedicated system-level functions (e.g., OS ticks, delays)
 *   or future non-PWM specific features, as per the requirement for reserving at least 2 timers.
 *
 * - TIM11 (General-purpose timer, 16-bit, 1 PWM channel: CH1)  /* PDF Reference */
 *   Reason for reservation: Reserved for dedicated system-level functions (e.g., OS ticks, delays)
 *   or future non-PWM specific features, as per the requirement for reserving at least 2 timers.
 *
 * Non-available Timers on STM32F401xB/C/D/E:
 * - TIM8 /* PDF Reference */
 * - TIM12 /* PDF Reference */
 * - TIM13 /* PDF Reference */
 * - TIM14 /* PDF Reference */
 */


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel: The PWM channel to initialize.
 * @return void
 */
void PWM_Init(TRD_Channel_t TRD_Channel); /* Assumed based on MCU conventions */

/*
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz (tlong type expected from STM32F401RC_MAIN.h).
 * @param duty: The desired PWM duty cycle in percentage (tbyte type expected from STM32F401RC_MAIN.h).
 * @return void
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty); /* Assumed based on MCU conventions */

/*
 * @brief Starts the specified PWM channel.
 * @param TRD_Channel: The PWM channel to start.
 * @return void
 */
void PWM_Start(TRD_Channel_t TRD_Channel); /* Assumed based on MCU conventions */

/*
 * @brief Stops the specified PWM channel.
 * @param TRD_Channel: The PWM channel to stop.
 * @return void
 */
void PWM_Stop(TRD_Channel_t TRD_Channel); /* Assumed based on MCU conventions */

/*
 * @brief Powers off all active PWM modules.
 * @param void
 * @return void
 */
void PWM_PowerOff(void); /* Assumed based on MCU conventions */

#endif /* STM32F401RC_PWM_H_ */