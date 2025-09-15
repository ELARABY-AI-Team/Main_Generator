#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel middleware.
 *
 * This file contains user-editable constants, feature flags, and parameters
 * that allow customization of the LEDToggel middleware's behavior
 * without modifying the core application logic.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h" // Required for t_port, t_pin, t_sys_volt, t_tick_time enums

//==================================================================================================
// User Configuration Macros
//==================================================================================================

/**
 * @brief Defines the GPIO port for the LED to be toggled.
 *        Refer to MCAL.h for available `t_port` enum values.
 */
#define LED_TOGGLE_PORT   (port_2)

/**
 * @brief Defines the GPIO pin for the LED to be toggled.
 *        Refer to MCAL.h for available `t_pin` enum values.
 */
#define LED_TOGGLE_PIN    (pin_0)

/**
 * @brief Defines the logic level for the LED to be ON.
 *        (e.g., 1 for active-high, 0 for active-low).
 */
#define LED_ON_LEVEL      (1U)

/**
 * @brief Defines the logic level for the LED to be OFF.
 *        (e.g., 0 for active-high, 1 for active-low).
 */
#define LED_OFF_LEVEL     (0U)

/**
 * @brief Defines the system voltage level for MCU configuration.
 *        Refer to MCAL.h for available `t_sys_volt` enum values.
 *        (e.g., sys_volt_3v, sys_volt_5v).
 */
#define SYS_VOLT_LEVEL    (sys_volt_5v) // Example: 5V system voltage

/**
 * @brief Defines the period for the LED toggle task in milliseconds.
 *        The LED will change state every `LED_TOGGLE_PERIOD_MS` milliseconds.
 */
#define LED_TOGGLE_PERIOD_MS (500) // Toggle every 500 ms (blink rate 1 Hz)

/**
 * @brief Defines the tick time interval for the Time Triggered OS in milliseconds.
 *        Refer to MCAL.h for available `t_tick_time` enum values.
 *        Choose a value that evenly divides `LED_TOGGLE_PERIOD_MS`.
 */
#define SCHEDULER_TICK_MS (10) // Scheduler tick every 10 ms

/**
 * @brief Converts `SCHEDULER_TICK_MS` to the corresponding `t_tick_time` enum.
 *        Adjust this mapping if `SCHEDULER_TICK_MS` is changed.
 */
#if (SCHEDULER_TICK_MS == 1)
#define SCHEDULER_TICK_MS_ENUM    (tick_time_1ms)
#elif (SCHEDULER_TICK_MS == 10)
#define SCHEDULER_TICK_MS_ENUM    (tick_time_10ms)
#elif (SCHEDULER_TICK_MS == 100)
#define SCHEDULER_TICK_MS_ENUM    (tick_time_100ms)
#elif (SCHEDULER_TICK_MS == 1000)
#define SCHEDULER_TICK_MS_ENUM    (tick_time_1s)
#else
#error "SCHEDULER_TICK_MS is not a supported value (1, 10, 100, 1000)"
#endif

/**
 * @brief Calculates the number of scheduler ticks for the LED toggle period.
 */
#define LED_TOGGLE_PERIOD_TICKS (LED_TOGGLE_PERIOD_MS / SCHEDULER_TICK_MS)

#endif // LEDTOGGEL_USER_H
