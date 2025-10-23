#ifndef TOGGLELED_USER_H
#define TOGGLELED_USER_H

#include "MCAL.h" // Required for t_port and t_pin definitions

/**
 * @file Toggleled_user.h
 * @brief User configuration header for the Toggleled application.
 *
 * This file contains user-editable configurations, constants, and
 * feature flags for the Toggleled application. No hardware-specific
 * logic should be placed here, only high-level parameters.
 */

//==================================================================================================
// LED Configuration
//==================================================================================================

/**
 * @brief Defines the GPIO port connected to the LED.
 *        Refer to MCAL.h for available port enumerations (e.g., port_7).
 */
#define TOGGLELED_LED_PORT     (port_7)

/**
 * @brief Defines the GPIO pin connected to the LED.
 *        Refer to MCAL.h for available pin enumerations (e.g., pin_4).
 */
#define TOGGLELED_LED_PIN      (pin_4)

/**
 * @brief Defines the logic level that turns the LED ON.
 *        (0 for active-low, 1 for active-high).
 *        Flowchart specifies: "0=ON, 1=OFF"
 */
#define TOGGLELED_LED_ON_VALUE  ((tbyte)0)

/**
 * @brief Defines the logic level that turns the LED OFF.
 *        (1 for active-low, 0 for active-high).
 *        Flowchart specifies: "0=ON, 1=OFF"
 */
#define TOGGLELED_LED_OFF_VALUE ((tbyte)1)

//==================================================================================================
// Scheduler Configuration
//==================================================================================================

/**
 * @brief Defines the duration of one scheduler tick in milliseconds.
 *        This value is used for internal calculations and to initialize the scheduler.
 *        The actual timer configuration should align with this value.
 *        Note: The numerical value `1` is used here. `Interval_Timer_Init` will use
 *        the corresponding `t_Tick_Time` enum (e.g., `Tick_1_ms`).
 */
#define APP_SCHEDULER_TICK_DURATION_MS ((uint16_t)1)

/**
 * @brief Defines the period (in milliseconds) for the LED toggle task.
 *        Flowchart specifies: "period=500ms"
 */
#define TOGGLELED_TASK_PERIOD_MS ((tword)500)

#endif // TOGGLELED_USER_H
