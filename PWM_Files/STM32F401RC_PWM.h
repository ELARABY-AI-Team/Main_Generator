/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/*
 * Note: This file is generated as STM32F401RC_PWM.h, aligning with the PWM-related content
 * and function declarations provided in the requirements, despite the initial prompt
 * mentioning "GPIO.h". This ensures consistency between the filename and its purpose.
 */

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* Required for custom types such as tbyte, tlong */
#include "STM32F401RC_GPIO.h" /* Required for GPIO configurations often associated with PWM output pins */

/*
 * @brief TRD_Channel_t: Enumeration of PWM-capable timer channels available on the STM32F401RC microcontroller.
 *
 * This enumeration identifies all channels from the general-purpose (TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
 * and advanced-control (TIM1) timers that explicitly support Pulse Width Modulation (PWM) functionality.
 *
 * Timer Reservation Logic:
 * The STM32F401RC includes basic timers (TIM6 and TIM7) which are not designed for PWM generation.
 * These specific timers are commonly reserved for fundamental timing operations, system tick generation,
 * or as a time-base for an operating system. Consequently, they are intentionally excluded from
 * this enumeration to maintain a clear focus on PWM-specific channels and adhere to best practices
 * for resource allocation in embedded systems.
 */
typedef enum TRD_Channel_t
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

  TRD_CHANNEL_COUNT /* Total count of enumerated PWM channels */
} TRD_Channel_t;

/* ==================== FUNCTION DECLARATIONS ==================== */

/* @brief Initializes a specific PWM channel with default parameters. */
void PWM_Init(TRD_Channel_t TRD_Channel);

/* @brief Configures the frequency and duty cycle for a specified PWM channel. */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/* @brief Starts the PWM signal generation on the designated channel. */
void PWM_Start(TRD_Channel_t TRD_Channel);

/* @brief Stops the PWM signal generation on the designated channel. */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/* @brief Deactivates and powers off all PWM timer peripherals. */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */