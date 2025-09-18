/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel application.
 *
 * This file provides user-editable parameters and feature flags for the
 * LEDToggel middleware. Hardware-specific details like pin assignments
 * are explicitly excluded and handled at the HAL/MCAL layer.
 */

#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

//==================================================================================================
// Includes
//==================================================================================================
#include <stdint.h> // For standard integer types if needed for user parameters

//==================================================================================================
// User-Editable Parameters
//==================================================================================================

/**
 * @brief Defines the initial state of the LED upon application startup.
 *
 * According to the flowchart, the LED is initially set "ON".
 * For an active-low LED (as implied by the flowchart "0=ON, 1=OFF"),
 * this value should be 0 (LOW).
 *
 * @note This value represents the *logical* state (ON/OFF), not the raw pin level.
 *       The conversion to physical pin level is handled internally by the driver.
 *       0: LED ON
 *       1: LED OFF
 */
#define LEDTOGGEL_INITIAL_STATE_VALUE (0U)

// Add other user-configurable parameters here, e.g.,
// #define LEDTOGGEL_DEBUG_MODE_ENABLED (0) // 1 to enable debug prints, 0 to disable

#endif // LEDTOGGEL_USER_H
//==================================================================================================
// End of File
//==================================================================================================