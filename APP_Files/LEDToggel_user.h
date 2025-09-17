#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

#include "MCAL.h" // Required for t_port and t_pin types

/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel middleware application.
 *
 * This file contains user-editable constants and parameters to customize the
 * behavior of the LEDToggel application without modifying its core logic.
 *
 * @note In a strictly layered architecture, specific hardware pin assignments (like LED_PORT, LED_PIN)
 *       would typically reside in a dedicated HAL (Hardware Abstraction Layer) configuration file
 *       (e.g., `HAL_LED_Config.h`). For this project, as no such HAL layer for generic LEDs is provided,
 *       these definitions are placed here to enable direct interaction with the MCAL GPIO functions.
 */

//==================================================================================================
// User Configuration Parameters
//==================================================================================================

/**
 * @brief Defines the port for the LED to be toggled.
 *        Example: `port_1` corresponds to GPIO Port 1.
 */
#define LED_PORT                    port_1

/**
 * @brief Defines the pin number within the selected port for the LED.
 *        Example: `pin_0` corresponds to Pin 0.
 */
#define LED_PIN                     pin_0

/**
 * @brief Defines the initial state of the LED at application startup.
 *        Use `1` for ON, `0` for OFF. (Matches MCAL GPIO_Value_Set logic)
 */
#define LED_INITIAL_STATE           1

/**
 * @brief Defines the logic level for the LED to be considered ON.
 *        Use `1` for active-high LED, `0` for active-low LED.
 */
#define LED_ON_VALUE                1

/**
 * @brief Defines the logic level for the LED to be considered OFF.
 *        Use `0` for active-high LED, `1` for active-low LED.
 */
#define LED_OFF_VALUE               0

/**
 * @brief Defines the period (in milliseconds) at which the LED will toggle.
 *        This value determines the frequency of the `LedToggel_RunTask` execution.
 */
#define LED_TOGGLE_PERIOD_MS        500 // Toggle every 500 milliseconds (0.5 seconds)

/**
 * @brief Defines the system voltage for MCU configuration.
 *        Used by `MCU_Config_Init()`.
 *        Options: `sys_volt_3v`, `sys_volt_5v`.
 */
#define LED_SYS_VOLTAGE             sys_volt_5v

/**
 * @brief Defines the base tick time for the Time-Triggered Scheduler.
 *        This should be a value from the `t_Tick_Time` enum in `Interval_Timer.h`.
 *        A 1ms tick provides good granularity for many applications.
 */
#define TT_BASE_TICK_TIME           Tick_1_ms

#endif // LEDTOGGEL_USER_H
