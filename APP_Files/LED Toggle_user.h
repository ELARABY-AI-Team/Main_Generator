#ifndef LED_TOGGLE_USER_H
#define LED_TOGGLE_USER_H

/**
 * @file LED Toggle_user.h
 * @brief User configuration header for the LED Toggle middleware application.
 *
 * This file provides configurable parameters for the LED Toggle application,
 * allowing end-users to adjust behavior (e.g., LED pin, toggle period)
 * without modifying the core middleware code.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h" // Required for t_port, t_pin, t_sys_volt, t_tick_time definitions

//==================================================================================================
// User-Configurable Parameters for LED Toggle Application
//==================================================================================================

/**
 * @brief Defines the GPIO port connected to the LED.
 *        Refer to `t_port` in MCAL.h for available ports.
 */
#define LED_TOGGLE_LED_PORT     port_1

/**
 * @brief Defines the GPIO pin connected to the LED.
 *        Refer to `t_pin` in MCAL.h for available pins (0-7).
 */
#define LED_TOGGLE_LED_PIN      pin_0

/**
 * @brief Defines the logic level for the LED to be considered ON.
 *        - `1` for active-high LED (LED lights up when pin is HIGH).
 *        - `0` for active-low LED (LED lights up when pin is LOW).
 */
#define LED_TOGGLE_ON_STATE     1

/**
 * @brief Defines the logic level for the LED to be considered OFF.
 *        - `0` for active-high LED (LED is off when pin is LOW).
 *        - `1` for active-low LED (LED is off when pin is HIGH).
 */
#define LED_TOGGLE_OFF_STATE    0

/**
 * @brief Defines the initial state of the LED when the application starts.
 *        Should be either `LED_TOGGLE_ON_STATE` or `LED_TOGGLE_OFF_STATE`.
 */
#define LED_TOGGLE_INITIAL_STATE LED_TOGGLE_OFF_STATE

/**
 * @brief Defines the period (in milliseconds) at which the LED will toggle.
 *        For example, `500` means the LED will toggle every 500ms (flash rate of 1Hz).
 */
#define LED_TOGGLE_TASK_PERIOD_MS 500 // Toggle every 500ms (i.e., 1Hz flash rate)

//==================================================================================================
// User-Configurable Parameters for System-Wide MCAL/Scheduler Setup
//==================================================================================================

/**
 * @brief Defines the system voltage level for MCU configuration.
 *        Refer to `t_sys_volt` in MCAL.h for available options.
 *        (e.g., `sys_volt_3v`, `sys_volt_5v`).
 */
#define APP_SYSTEM_VOLTAGE      sys_volt_5v

/**
 * @brief Defines the base tick time interval for the Time Triggered (TT) scheduler.
 *        This influences the granularity of task scheduling.
 *        Refer to `t_tick_time` in MCAL.h for available options.
 *        (e.g., `tick_time_1ms`, `tick_time_10ms`).
 * @note The `LED_TOGGLE_TASK_PERIOD_MS` should be a multiple of this tick time.
 *       If `APP_SCHEDULER_TICK_TIME` is `tick_time_1ms`, then 1 unit of `period`
 *       in `TT_Add_task` corresponds to 1 millisecond.
 */
#define APP_SCHEDULER_TICK_TIME   tick_time_1ms

#endif // LED_TOGGLE_USER_H
