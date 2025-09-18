/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel application.
 *
 * This file contains user-editable parameters, feature flags, and constants
 * that can be modified without changing the core application logic.
 */

#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

//==================================================================================================
// Includes (Minimal includes for types if necessary, MCAL.h handles standard types)
//==================================================================================================
#include "MCAL.h" // Provides t_port, t_pin types

//==================================================================================================
// User Configuration Macros
//==================================================================================================

/**
 * @brief Defines the GPIO port where the LED is connected.
 *        As per the "Application.png" flowchart, the LED is on Port 7.
 */
#define LED_TOGGLE_PORT         port_7

/**
 * @brief Defines the GPIO pin where the LED is connected within the specified port.
 *        As per the "Application.png" flowchart, the LED is on Pin 4.
 */
#define LED_TOGGLE_PIN          pin_4

/**
 * @brief Defines the initial delay (in scheduler ticks) before the LED toggling task starts.
 *        As per the "Application.png" flowchart, the delay is 0ms (starts immediately).
 */
#define LED_TOGGLE_TASK_DELAY   ((tword)0)

/**
 * @brief Defines the period (in scheduler ticks) for the LED toggling task.
 *        As per the "Application.png" flowchart, the period is 500ms.
 */
#define LED_TOGGLE_TASK_PERIOD  ((tword)500)

/**
 * @brief Defines the base tick time for the scheduler in milliseconds.
 *        This value determines the resolution of `LED_TOGGLE_TASK_PERIOD`.
 *        For example, a 1ms base tick means a 500ms period will be 500 scheduler ticks.
 *        This must match the `Tick_Time_ms` used in `tt_init`.
 */
#define SCHEDULER_BASE_TICK_MS  ((uint16_t)1) // Assuming a 1ms base tick for the scheduler

#endif // LEDTOGGEL_USER_H