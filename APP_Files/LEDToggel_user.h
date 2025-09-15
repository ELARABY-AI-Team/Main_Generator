#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel application.
 *
 * This file contains user-editable constants and parameters to customize
 * the behavior of the LEDToggel middleware without modifying core logic.
 * Hardware-specific details like pin assignments are intentionally excluded,
 * as they are expected to be handled by an underlying HAL layer or system configuration.
 */

/*==============================================================================================
 * User Configuration Macros
 *============================================================================================*/

/**
 * @brief Defines the LED toggle period in milliseconds.
 *
 * This value determines how frequently the LED state is inverted.
 * For example, a value of 500U will cause the LED to toggle every 500 milliseconds,
 * resulting in a 1Hz blink rate (500ms ON, 500ms OFF).
 */
#define LEDTOGGEL_TOGGLE_PERIOD_MS    (500U) // Toggle every 500 milliseconds

#endif /* LEDTOGGEL_USER_H */
