#ifndef LEDTOGGLE_USER_H
#define LEDTOGGLE_USER_H

#include "MCAL.h" // For t_port, t_pin, tbyte, t_tick_time types

/**
 * @file LEDToggle_user.h
 * @brief User configuration header for the LEDToggle application.
 *
 * This file allows end-users to customize the behavior of the LEDToggle
 * application without modifying its core logic.
 */

/*==============================================================================================
*                                    User Configurations
==============================================================================================*/

/**
 * @brief Specifies the GPIO Port for the LED.
 *        Refer to MCAL.h for available t_port values (e.g., port_1, port_2).
 * @note This should be a port available for general-purpose I/O.
 */
#define LED_TOGGLE_PORT         port_1

/**
 * @brief Specifies the GPIO Pin for the LED within the chosen port.
 *        Refer to MCAL.h for available t_pin values (e.g., pin_0, pin_1).
 * @note This should be a pin available for general-purpose I/O.
 */
#define LED_TOGGLE_PIN          pin_7

/**
 * @brief Defines the value for the LED being ON.
 *        (0 for active-low LED, 1 for active-high LED).
 */
#define LED_ON_VALUE            (1U)

/**
 * @brief Defines the value for the LED being OFF.
 *        (0 for active-high LED, 1 for active-low LED).
 */
#define LED_OFF_VALUE           (0U)

/**
 * @brief Sets the initial state of the LED when the application starts.
 *        Use LED_ON_VALUE or LED_OFF_VALUE.
 */
#define LED_INITIAL_STATE       LED_ON_VALUE

/**
 * @brief Defines the period (in milliseconds) at which the LED state will toggle.
 *        Example: 500 for a 500ms toggle period (1Hz overall blink rate).
 */
#define LED_TOGGLE_PERIOD_MS    (500)

/**
 * @brief Configures the base tick time for the scheduler.
 *        This influences the granularity of task scheduling.
 *        Refer to MCAL.h's t_tick_time enum.
 */
#define LED_SCHEDULER_TICK_TIME tick_time_1ms

/**
 * @brief Defines the numeric value in milliseconds for the chosen scheduler tick time.
 *        Must match the LED_SCHEDULER_TICK_TIME definition.
 *        Example: If LED_SCHEDULER_TICK_TIME is tick_time_1ms, then TICK_PERIOD_MS is 1.
 */
#define TICK_PERIOD_MS          (1)

#endif /* LEDTOGGLE_USER_H */
