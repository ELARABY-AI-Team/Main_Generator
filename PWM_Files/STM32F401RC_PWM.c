/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : STM32F410RC PWM Driver Header File
* Author         : Technology Innovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F410RC_PWM_H
#define STM32F410RC_PWM_H

/***********************************************************************************************************************
* Includes
***********************************************************************************************************************/
#include <stdint.h> // Required for standard integer types like uint8_t, uint32_t

// Note: Depending on the implementation, you might need to include the main STM32 device header,
// e.g., #include "stm32f4xx.h" or #include "stm32f4xx_hal.h" in the corresponding .c file.
// It is generally NOT required in the header file unless specific peripheral structures or enums
// defined there are directly used in the function signatures defined here.

/***********************************************************************************************************************
* Type Definitions
***********************************************************************************************************************/

/**
 * @brief Represents a specific PWM channel.
 *        These values must map to the actual hardware timer and channel
 *        configurations used for PWM on the STM32F410RC.
 *        Modify this enumeration based on the specific timer instances and
 *        channels configured for PWM output in your project.
 */
typedef enum
{
    // Example Channels (replace with actual used channels)
    TRD_CHANNEL_TIMER1_CH1 = 0, /**< Example: Maps to TIM1 Channel 1 */
    TRD_CHANNEL_TIMER1_CH2,     /**< Example: Maps to TIM1 Channel 2 */
    TRD_CHANNEL_TIMER3_CH1,     /**< Example: Maps to TIM3 Channel 1 */
    TRD_CHANNEL_TIMER3_CH2,     /**< Example: Maps to TIM3 Channel 2 */
    // Add all other required PWM channels here...

    TRD_CHANNEL_COUNT           /**< Total number of defined PWM channels */
} TRD_Channel_t;

/**
 * @brief Represents a long integer type, typically used for frequency values (e.g., in Hz).
 */
typedef uint32_t tlong;

/**
 * @brief Represents a byte type, typically used for duty cycle values (e.g., 0-100%).
 */
typedef uint8_t tbyte;

/**Static Variables ====================================================================*/

// Note: Static variable *definitions* typically belong in the .c file.
// Static variable *declarations* (using extern) would go here if they needed
// to be accessed from other files, but this is generally discouraged for module-internal statics.

/**Functions ===========================================================================*/

/* ====================  Implement the following functions ==================== */

/**
 * @brief Initializes a specific PWM channel.
 *
 * @param TRD_Channel The PWM channel to initialize.
 *        This function should configure the associated timer, GPIO pin,
 *        and basic PWM parameters for the specified channel.
 *        It does not start the PWM output.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 *
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency   The desired PWM frequency in Hz.
 * @param duty        The desired PWM duty cycle, typically as a percentage (0-100).
 *                    The actual range and interpretation might depend on the
 *                    implementation (e.g., 0-100, 0-255, or timer period related).
 *        This function recalculates the timer period and compare values based on
 *        the provided frequency and duty cycle.
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM output on a specific channel.
 *
 * @param TRD_Channel The PWM channel to start.
 *        This typically enables the timer channel's output and the timer itself.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM output on a specific channel.
 *
 * @param TRD_Channel The PWM channel to stop.
 *        This typically disables the timer channel's output. The timer might
 *        continue running or be stopped depending on the implementation.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off or de-initializes all PWM channels managed by this driver.
 *
 *        This function should disable the relevant timers, disable the GPIO pins
 *        used for PWM, and potentially gate the peripheral clocks to conserve power.
 *        After calling this, all PWM outputs should be inactive.
 */
void PWM_PowerOff(void);

#endif /* STM32F410RC_PWM_H */