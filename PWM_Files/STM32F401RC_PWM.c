#ifndef PWM_H_
#define PWM_H_

/***********************************************************************************************************************
* File Name      : PWM.h
* Description    : Header file for PWM functionality on STM32F410RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**
 * @defgroup PWM PWM Driver
 * @brief Production-ready driver for PWM functionality on STM32F410RC.
 *
 * This file provides the function declarations for controlling PWM outputs
 * using the timers on the STM32F410RC microcontroller.
 *
 * @note This header assumes that `STM32F410RC_PWM.h` contains necessary
 *       definitions, likely related to peripheral registers or configuration macros.
 * @note The custom types `TRD_Channel_t`, `tlong`, and `tbyte` are assumed
 *       to be defined elsewhere in the project (e.g., in a common `types.h`
 *       or `common.h` file). Placeholders are provided here for completeness,
 *       but you should ensure they match your project's definitions.
 */

/**
 * @file PWM.h
 * @brief Header file for the PWM driver on STM32F410RC.
 */



/* Assume the following types are defined in a common header file */
/* Example definitions (adjust to match your project's actual types): */
typedef enum
{
    // Define your specific PWM channels here (e.g., TIMx_CHy)
    TRD_CHANNEL_PWM_1, // Example Channel 1
    TRD_CHANNEL_PWM_2, // Example Channel 2
    // Add more channels as needed for your application
    TRD_CHANNEL_COUNT  // Total number of defined channels
} TRD_Channel_t;       /**< Enumeration for identifying PWM channels. */

typedef unsigned long tlong; /**< Type for frequency and potentially period values. */
typedef unsigned char tbyte; /**< Type for duty cycle (e.g., 0-100 or 0-255). */


/**Static Variables ====================================================================*/
// Note: Static variables are typically defined in the .c file and are local to that file.


/**Functions ===========================================================================*/

/* ====================  Implement the following functions ==================== */

/**
 * @brief Initializes the specified PWM channel.
 *
 * Configures the necessary timer, GPIO pin alternate function, and basic
 * initial settings (like prescaler, period) for the given PWM channel
 * in anticipation of starting PWM generation. The channel is typically stopped
 * and set to a default state (e.g., 0% duty cycle) after initialization.
 *
 * @param TRD_Channel The enumeration value representing the PWM channel to initialize.
 * @return None.
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for a specified PWM channel.
 *
 * This function calculates and sets the appropriate timer prescaler,
 * auto-reload register (ARR), and capture/compare register (CCR) values
 * based on the desired PWM frequency and duty cycle.
 *
 * @param TRD_Channel The enumeration value representing the PWM channel to configure.
 * @param frequency   The desired PWM frequency in Hertz (Hz).
 * @param duty        The desired PWM duty cycle. The range and interpretation
 *                    (e.g., 0-100 for percentage, 0-255, 0-ARR+1) must match
 *                    the implementation in the corresponding .c file.
 *                    A common interpretation is 0-100 representing 0% to 100%.
 * @return None.
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on the specified channel.
 *
 * Enables the timer output compare mode and the corresponding output pin
 * for the specific channel, causing the PWM signal to be generated.
 * The timer counter might also be started or ensured to be running if not already.
 *
 * @param TRD_Channel The enumeration value representing the PWM channel to start.
 * @return None.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on the specified channel.
 *
 * Disables the output compare mode or the output pin for the specific channel,
 * halting the PWM signal generation on that pin. The timer itself might continue
 * running depending on implementation details and whether it's shared.
 *
 * @param TRD_Channel The enumeration value representing the PWM channel to stop.
 * @return None.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief Powers off all active PWM peripherals used by this driver.
 *
 * This function disables the clocks for all timers and potentially GPIO ports
 * that are used for PWM generation, putting the hardware into a low-power state
 * relative to PWM functionality. It might also reset relevant registers.
 *
 * @param None.
 * @return None.
 */
void PWM_PowerOff(void);


#endif /* PWM_H_ */