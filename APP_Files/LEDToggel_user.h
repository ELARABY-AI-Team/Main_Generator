#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel middleware application.
 *
 * This file contains user-editable constants, feature flags, and parameters
 * that can be adjusted without modifying the core middleware logic.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h" // Required for t_port and t_pin types

//==================================================================================================
// User Configuration Parameters
//==================================================================================================

/**
 * @brief Defines the GPIO port for the LED.
 *        Refer to MCAL.h for available t_port enumerations.
 *        Example: Port_2
 */
#define LEDTOGGEL_LED_PORT    Port_2

/**
 * @brief Defines the GPIO pin for the LED within the specified port.
 *        Refer to MCAL.h for available t_pin enumerations.
 *        Example: Pin_5
 */
#define LEDTOGGEL_LED_PIN     Pin_5

/**
 * @brief Defines the toggle period for the LED in milliseconds.
 *        This value determines how frequently the LED state will be changed.
 *        Example: 500 (for 500 milliseconds)
 */
#define LEDTOGGEL_TOGGLE_PERIOD_MS 500

#endif // LEDTOGGEL_USER_H
