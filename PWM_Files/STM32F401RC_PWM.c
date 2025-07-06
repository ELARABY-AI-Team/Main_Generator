/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : Header file for PWM functionality on STM32F410RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef PWM_H
#define PWM_H

/* Includes --------------------------------------------------------------------*/
// Using standard STM32 HAL header and stdint for fixed-width types
#include "stm32f4xx_hal.h" // Include the appropriate STM32 HAL header
#include <stdint.h>       // For standard integer types (uint8_t, uint32_t)


/* Type Definitions ------------------------------------------------------------*/

/**
  * @brief  Enumeration for identifying TRD PWM channels.
  *         **NOTE: MODIFY THIS ENUMERATION TO MATCH YOUR ACTUAL HARDWARE PWM CHANNEL CONFIGURATION.**
  *         e.g., Map these enum values to specific Timer instances and Channels (TIMx_CHy).
  */
typedef enum
{
    TRD_CHANNEL_1 = 0, // Example: Could represent TIM2_CH1
    TRD_CHANNEL_2,     // Example: Could represent TIM3_CH2
    TRD_CHANNEL_3,     // Example: Could represent TIM4_CH1
    TRD_CHANNEL_4,     // Example: Could represent TIM5_CH2
    // Add other channels as needed based on your design
    TRD_CHANNEL_COUNT  // Helper value: Total number of defined channels
} TRD_Channel_t;


/**
  * @brief  Alias for a 32-bit unsigned integer type.
  */
typedef uint32_t tlong; // Assuming tlong is a 32-bit unsigned integer

/**
  * @brief  Alias for an 8-bit unsigned integer type.
  */
typedef uint8_t  tbyte; // Assuming tbyte is an 8-bit unsigned integer


/**Static Variables ====================================================================*/
// Static variables are typically defined in the .c file, not declared in the header.


/**Functions ===========================================================================*/


/* ====================  Implement the following functions ==================== */

/**
  * @brief Initializes a specific PWM channel (e.g., configures timer, GPIO pin).
  * @param TRD_Channel: The channel to initialize (e.g., TRD_CHANNEL_1).
  * @retval None
  */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
  * @brief Sets the frequency and duty cycle for a specific PWM channel.
  *        Note: Setting frequency might affect other channels on the same timer.
  * @param TRD_Channel: The channel to configure.
  * @param frequency: The desired PWM frequency in Hz.
  * @param duty: The desired duty cycle in percentage (0-100).
  * @retval None
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
  * @brief Starts PWM generation on a specific channel.
  * @param TRD_Channel: The channel to start.
  * @retval None
  */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
  * @brief Stops PWM generation on a specific channel.
  * @param TRD_Channel: The channel to stop.
  * @retval None
  */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
  * @brief Powers off all configured PWM timers/channels.
  *        This function should de-initialize timers and associated GPIOs.
  * @retval None
  */
void PWM_PowerOff(void);


#endif /* PWM_H */