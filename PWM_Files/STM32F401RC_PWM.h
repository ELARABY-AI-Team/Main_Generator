/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : Header file for PWM functionality on STM32F410RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F410RC_PWM_H_
#define STM32F410RC_PWM_H_

#include "STM32F410RC_MAIN.h"
#include "STM32F410RC_GPIO.h"

/* ==================== PWM CHANNEL ENUM ==================== */

/*
 * @brief Enumeration for identifying specific PWM channels across available timers.
 *        Lists channels for TIM1, TIM2, TIM3, TIM4, TIM9, TIM10, and TIM11.
 *        Based on STM32F410RC reference manual (RM0395).
 */
typedef enum
{
  PWM_CHANNEL_TIM1_CH1,  /* PDF Reference */
  PWM_CHANNEL_TIM1_CH2,  /* PDF Reference */
  PWM_CHANNEL_TIM1_CH3,  /* PDF Reference */
  PWM_CHANNEL_TIM1_CH4,  /* PDF Reference */
  PWM_CHANNEL_TIM2_CH1,  /* PDF Reference */
  PWM_CHANNEL_TIM2_CH2,  /* PDF Reference */
  PWM_CHANNEL_TIM2_CH3,  /* PDF Reference */
  PWM_CHANNEL_TIM2_CH4,  /* PDF Reference */
  PWM_CHANNEL_TIM3_CH1,  /* PDF Reference */
  PWM_CHANNEL_TIM3_CH2,  /* PDF Reference */
  PWM_CHANNEL_TIM3_CH3,  /* PDF Reference */
  PWM_CHANNEL_TIM3_CH4,  /* PDF Reference */
  PWM_CHANNEL_TIM4_CH1,  /* PDF Reference */
  PWM_CHANNEL_TIM4_CH2,  /* PDF Reference */
  PWM_CHANNEL_TIM4_CH3,  /* PDF Reference */
  PWM_CHANNEL_TIM4_CH4,  /* PDF Reference */
  PWM_CHANNEL_TIM9_CH1,  /* PDF Reference */
  PWM_CHANNEL_TIM9_CH2,  /* PDF Reference */
  PWM_CHANNEL_TIM10_CH1, /* PDF Reference */
  PWM_CHANNEL_TIM11_CH1, /* PDF Reference */
  PWM_CHANNEL_COUNT      /* Total number of PWM channels available */
} t_pwm_channel;

/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a specific PWM channel.
 * @param TRD_Channel: The specific PWM channel to initialize (type assumed from STM32F410RC_MAIN.h or related).
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/*
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 * @param TRD_Channel: The specific PWM channel.
 * @param frequency: The desired frequency in Hz (type assumed from STM32F410RC_MAIN.h).
 * @param duty: The desired duty cycle in percent (0-100) (type assumed from STM32F410RC_MAIN.h).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/*
 * @brief Starts PWM generation on a specific channel.
 * @param TRD_Channel: The specific PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/*
 * @brief Stops PWM generation on a specific channel.
 * @param TRD_Channel: The specific PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/*
 * @brief Powers off all configured PWM peripherals/channels.
 */
void PWM_PowerOff(void);

#endif /* STM32F410RC_PWM_H_ */