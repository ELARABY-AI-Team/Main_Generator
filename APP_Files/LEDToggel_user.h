#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel application.
 *
 * This file contains user-editable constants and feature flags for the
 * LEDToggel application. End-users can adjust behavior here without
 * modifying the core middleware logic.
 *
 * ⚠ Note: Hardware-specific pin configurations or register values are
 *         intentionally excluded from this file, as they belong to the
 *         Hardware Abstraction Layer (HAL) or Microcontroller Abstraction Layer (MCAL).
 */

/*==============================================================================================
 * Configuration Macros
 *=============================================================================================*/

/**
 * @brief The period in milliseconds at which the LED toggling task will run.
 *        A value of 500 means the LED state will change every 500ms.
 */
#define LEDTOGGEL_TOGGLE_PERIOD_MS (500)

/**
 * @brief Defines the logical state for the LED to be considered "ON".
 *        0 for active-low LED, 1 for active-high LED.
 */
#define LED_ACTIVE_STATE (0)

/**
 * @brief Defines the logical state for the LED to be considered "OFF".
 *        1 for active-low LED, 0 for active-high LED.
 */
#define LED_INACTIVE_STATE (1)

/**
 * @brief The GPIO port to which the LED is connected.
 *        This defines the physical port, e.g., 'port_4'.
 *        (Note: While technically hardware-related, this is kept here for
 *        user-facing configuration of *which* LED, abstracting the physical pin).
 *        Choosing Port_4 as an example, assuming it's available.
 */
#define LEDTOGGEL_LED_PORT (port_4)

/**
 * @brief The GPIO pin to which the LED is connected.
 *        This defines the physical pin number within the selected port, e.g., 'pin_0'.
 *        (Note: Same justification as LEDTOGGEL_LED_PORT).
 */
#define LEDTOGGEL_LED_PIN (pin_0)


#endif /* LEDTOGGEL_USER_H */
