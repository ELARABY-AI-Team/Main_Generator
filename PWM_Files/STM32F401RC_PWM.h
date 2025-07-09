/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
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
 * ==================== PWM CHANNEL ENUM (DO NOT MODIFY) ====================
 */
/*
 * Define enum for PWM channels.
 * Based on RM0368 for STM32F401xB/C and STM32F401xD/E, the following timers support PWM:
 * TIM1 (Advanced-control), TIM2-5 (General-purpose), TIM9-11 (General-purpose).
 * Channels supported by each timer are identified from the PDF sections 12.2, 13.2, 14.2.1, 14.2.2.
 * TIM1: CH1, CH2, CH3, CH4
 * TIM2-5: CH1, CH2, CH3, CH4
 * TIM9: CH1, CH2
 * TIM10-11: CH1
 * Total PWM capable channels are 24.
 * As per requirements, reserving TIM10 and TIM11 channels from the active enum list
 * and listing them as comments to fulfill the "reserve at least 2 timers" instruction
 * in a context where all listed timers support PWM.
 */
typedef enum
{
    /* TIM1 PWM Channels */
    TIM1_CH1, /* PDF Reference */
    TIM1_CH2, /* PDF Reference */
    TIM1_CH3, /* PDF Reference */
    TIM1_CH4, /* PDF Reference */
    /* TIM2 PWM Channels */
    TIM2_CH1, /* PDF Reference */
    TIM2_CH2, /* PDF Reference */
    TIM2_CH3, /* PDF Reference */
    TIM2_CH4, /* PDF Reference */
    /* TIM3 PWM Channels */
    TIM3_CH1, /* PDF Reference */
    TIM3_CH2, /* PDF Reference */
    TIM3_CH3, /* PDF Reference */
    TIM3_CH4, /* PDF Reference */
    /* TIM4 PWM Channels */
    TIM4_CH1, /* PDF Reference */
    TIM4_CH2, /* PDF Reference */
    TIM4_CH3, /* PDF Reference */
    TIM4_CH4, /* PDF Reference */
    /* TIM5 PWM Channels */
    TIM5_CH1, /* PDF Reference */
    TIM5_CH2, /* PDF Reference */
    TIM5_CH3, /* PDF Reference */
    TIM5_CH4, /* PDF Reference */
    /* TIM9 PWM Channels */
    TIM9_CH1, /* PDF Reference */
    TIM9_CH2, /* PDF Reference */

    /* Reserved Channels (TIM10, TIM11) */
    /* TIM10_CH1, PDF Reference */
    /* TIM11_CH1, PDF Reference */

    TRD_MAX_PWM_CHANNELS /* Sentinel value for total number of *active* channels */
} TRD_Channel_t;

/*
 * ==================== FUNCTION DECLARATIONS ====================
 */

/**
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel : The PWM channel to initialize.
 * @return None
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel : The PWM channel to configure.
 * @param frequency   : The desired PWM frequency in Hz.
 * @param duty        : The desired PWM duty cycle in percentage (0-100).
 * @return None
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts PWM generation on the specified channel.
 * @param TRD_Channel : The PWM channel to start.
 * @return None
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops PWM generation on the specified channel.
 * @param TRD_Channel : The PWM channel to stop.
 * @return None
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all PWM peripherals.
 *        Note: Actual power gating depends on MCU architecture and clock configuration.
 * @return None
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_GPIO_H_ */