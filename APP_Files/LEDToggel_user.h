#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h" // For t_port, t_pin types

/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel middleware.
 *
 * This file contains user-editable parameters, feature flags, and hardware
 * pin assignments that can be adjusted without modifying the core middleware logic.
 */

//==================================================================================================
// LED Configuration
//==================================================================================================

/**
 * @def LED_TOGGLE_PORT
 * @brief Specifies the GPIO port for the LED.
 *        Refer to MCAL.h for available t_port enumerations.
 *        Flowchart indicates Port 7.
 */
#define LED_TOGGLE_PORT   port_7

/**
 * @def LED_TOGGLE_PIN
 * @brief Specifies the GPIO pin for the LED.
 *        Refer to MCAL.h for available t_pin enumerations.
 *        Flowchart indicates Pin 4.
 */
#define LED_TOGGLE_PIN    pin_4

/**
 * @def LED_ON_VALUE
 * @brief Defines the GPIO value that turns the LED ON.
 *        Flowchart indicates 0=ON, 1=OFF.
 */
#define LED_ON_VALUE      0

/**
 * @def LED_OFF_VALUE
 * @brief Defines the GPIO value that turns the LED OFF.
 *        Flowchart indicates 0=ON, 1=OFF.
 */
#define LED_OFF_VALUE     1

/**
 * @def LED_TOGGLE_PERIOD_MS
 * @brief The period in milliseconds at which the LED toggles.
 *        Flowchart indicates 500ms.
 */
#define LED_TOGGLE_PERIOD_MS 500

#endif // LEDTOGGEL_USER_H