/***********************************************************************************************************************
* File Name      : STM32F410RC_PWM.h
* Description    : Header file for the STM32F410RC PWM driver.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F410RC_PWM_H
#define STM32F410RC_PWM_H

// Note: The request included `#include "STM32F410RC_PWM.h"` which is incorrect
// for a header file (a header includes its dependencies, not itself).
// Replaced with the necessary standard include for defined types.
#include <stdint.h> // Required for uint32_t (used by tlong) and uint8_t (used by tbyte)

// It's generally better practice to use stdint.h types directly where possible.
typedef uint32_t tlong; // Alias for a 32-bit unsigned integer (suitable for frequency)
typedef uint8_t  tbyte; // Alias for an 8-bit unsigned integer (suitable for duty cycle 0-100)

/**
 * @brief Enum defining the supported TRD (let user decide what TRD means)
 *        PWM channels.
 *        This enum *MUST* be populated based on the actual hardware
 *        implementation, mapping abstract channels to specific STM32
 *        Timer/Channel/Pin configurations used for PWM.
 *
 *        Example:
 *        typedef enum
 *        {
 *            TRD_CHANNEL_MOTOR_1 = 0, // e.g., mapped to TIM3_CH1 on PA6
 *            TRD_CHANNEL_LED_BRIGHTNESS, // e.g., mapped to TIM4_CH2 on PB7
 *            // Add more channels as needed...
 *            TRD_CHANNEL_COUNT // Total number of defined channels
 *        } TRD_Channel_t;
 */
typedef enum
{
    // TODO: Define your specific TRD_Channel_t values here, mapping them
    //       to the actual PWM channels you use on the STM32F410RC.
    //       Example placeholder:
    TRD_CHANNEL_EXAMPLE_1 = 0,
    TRD_CHANNEL_EXAMPLE_2,

    TRD_CHANNEL_LAST_ENUM, // Helper for counting defined channels
    TRD_CHANNEL_INVALID = -1 // Value to indicate an invalid channel
} TRD_Channel_t;


/**Static Variables ====================================================================*/
// Static variables for the PWM driver implementation should be defined
// in the corresponding .c file, not in the header.
// This section is included to strictly match the requested file structure.


/**Functions ===========================================================================*/

/* ====================  Implement the following functions ==================== */

/**
 * @brief Initializes the specified TRD PWM channel.
 *        This function should configure the necessary Timer, GPIO, and
 *        NVIC settings for the given channel.
 * @param[in] TRD_Channel: The TRD channel to initialize. Must be a valid
 *                         value from the TRD_Channel_t enum (excluding TRD_CHANNEL_INVALID).
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified TRD PWM channel.
 *        The channel must have been initialized first using PWM_Init().
 * @param[in] TRD_Channel: The TRD channel to configure. Must be a valid
 *                         value from the TRD_Channel_t enum (excluding TRD_CHANNEL_INVALID).
 * @param[in] frequency: The desired PWM frequency in Hertz (Hz).
 * @param[in] duty: The desired PWM duty cycle in percentage (0-100).
 *                  A value of 0 means always low, 100 means always high.
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on the specified TRD channel.
 *        The channel must have been initialized and configured.
 * @param[in] TRD_Channel: The TRD channel to start. Must be a valid
 *                         value from the TRD_Channel_t enum (excluding TRD_CHANNEL_INVALID).
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on the specified TRD channel.
 * @param[in] TRD_Channel: The TRD channel to stop. Must be a valid
 *                         value from the TRD_Channel_t enum (excluding TRD_CHANNEL_INVALID).
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all initialized PWM channels and related peripherals.
 *        This function should disable timers, GPIOs, and potentially
 *        reset associated clocks if appropriate.
 */
void PWM_PowerOff(void);

#endif /* STM32F410RC_PWM_H */