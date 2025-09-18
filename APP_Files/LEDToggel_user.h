/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel middleware application.
 *
 * This file allows end-users to customize constants and parameters
 * without modifying the core application logic.
 */

#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

/*==============================================================================================*/
/* Includes */
/*==============================================================================================*/
#include "MCAL.h" // For t_port, t_pin, t_sys_volt definitions

/*==============================================================================================*/
/* User Configuration Macros */
/*==============================================================================================*/

/**
 * @brief Defines the GPIO Port where the LED is connected.
 *        (e.g., port_0, port_1, ..., port_15 as defined in MCAL.h)
 */
#define LED_GPIO_PORT   port_7

/**
 * @brief Defines the GPIO Pin where the LED is connected.
 *        (e.g., pin_0, pin_1, ..., pin_7 as defined in MCAL.h)
 */
#define LED_GPIO_PIN    pin_4

/**
 * @brief Defines the active level of the LED.
 *        - Set to 0 for Active Low (LED turns ON when pin is LOW).
 *        - Set to 1 for Active High (LED turns ON when pin is HIGH).
 *        Flowchart specifies "Active Low0".
 */
#define LED_ACTIVE_LEVEL 0

/**
 * @brief Defines the period for toggling the LED in milliseconds.
 *        Flowchart specifies 500ms.
 */
#define LED_TOGGLE_PERIOD_MS 500

/**
 * @brief Defines the tick period of the time-triggered scheduler in milliseconds.
 *        A smaller tick provides higher resolution but more overhead.
 *        This value affects the granularity of task scheduling.
 *        Example: 10ms means tasks run on multiples of 10ms.
 */
#define SCHEDULER_TICK_MS 10

/**
 * @brief Defines the system voltage for MCU configuration.
 *        (e.g., sys_volt_3v, sys_volt_5v as defined in MCAL.h)
 */
#define MCU_SYSTEM_VOLTAGE sys_volt_5v

#endif /* LEDTOGGEL_USER_H */
