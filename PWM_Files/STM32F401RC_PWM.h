/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : Header file for the PWM driver on STM32F410RC
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

/* ==================== PWM CHANNEL ENUM (DO NOT MODIFY) ==================== */
/*
 * - typedef enum t_pwm_channel:
 *   - Identify the total number of channels supported by Timer 5 and please mention timer number as a comment
 *   - Use official datasheets or reference manuals of STM32F410RC
 *   - For each channel found for the 5 (e.g., CH1, CH2, ...), generate enum entries
 *   - Each enum entry name must follow the naming conventions typically used for STM32F410RC,
 *     and clearly indicate the timer and channel (e.g., TIMx_CHx or similar per MCU standard)
 *   - Each entry must include a single-line comment:
 *       - /* PDF Reference */ if confirmed from official documentation
 *       - /* Assumed – please verify */ if inferred from context
 *   - The enum must be named `t_pwm_channel` and follow production-grade C conventions.
 */

/* Timer 5 on STM32F410RC supports 4 channels */
typedef enum t_pwm_channel
{
  TIM5_CH1 = 0U, /* PDF Reference */
  TIM5_CH2 = 1U, /* PDF Reference */
  TIM5_CH3 = 2U, /* PDF Reference */
  TIM5_CH4 = 3U  /* PDF Reference */
} t_pwm_channel;


/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F410RC_PWM_H_ */