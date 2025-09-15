#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel application.
 *
 * This file provides configurable parameters for the LEDToggel middleware
 * application. End-users can adjust these values to modify behavior without
 * changing the core application logic.
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "MCAL.h" /* For t_port, t_pin, t_sys_volt, t_tick_time types */

/*==================================================================================================
*                                     USER CONFIGURATION
==================================================================================================*/

/*
 * @brief System voltage selection for MCU configuration.
 *        Choose between sys_volt_3v or sys_volt_5v.
 */
#define LEDTOGGEL_SYS_VOLTAGE             (sys_volt_5v)

/*
 * @brief Port for the LED to be toggled.
 *        Possible values: port_0, port_1, ..., port_15 as defined in MCAL.h.
 */
#define LEDTOGGEL_STATUS_PORT             (port_1)

/*
 * @brief Pin for the LED to be toggled.
 *        Possible values: pin_0, pin_1, ..., pin_7 as defined in MCAL.h.
 */
#define LEDTOGGEL_STATUS_PIN              (pin_0)

/*
 * @brief Defines if the LED is active high (1) or active low (0).
 *        Set to 1 if HIGH means ON, 0 if LOW means ON.
 */
#define LEDTOGGEL_LED_ACTIVE_HIGH         (1)

/*
 * @brief Default initial state for the LED after initialization.
 *        Use LEDTOGGEL_STATE_ON or LEDTOGGEL_STATE_OFF.
 *        (Note: These are logical states, actual hardware value depends on LEDTOGGEL_LED_ACTIVE_HIGH).
 */
#if (LEDTOGGEL_LED_ACTIVE_HIGH == 1)
#define LEDTOGGEL_INITIAL_STATE_DEFAULT   (0) /* 0 for OFF (active high) */
#else
#define LEDTOGGEL_INITIAL_STATE_DEFAULT   (1) /* 1 for OFF (active low) */
#endif

/*
 * @brief Toggle period for the LED in milliseconds.
 *        This defines how often the LED state will change.
 */
#define LEDTOGGEL_TOGGLE_PERIOD_MS        (500) /* e.g., 500ms for a 1Hz blink rate */

/*
 * @brief The tick time interval for the Time Triggered OS scheduler.
 *        Choose one of the t_tick_time enum values:
 *        tick_time_1ms, tick_time_10ms, tick_time_100ms, tick_time_1s.
 */
#define LEDTOGGEL_SCHEDULER_TICK_TIME     (tick_time_10ms)

#endif /* LEDTOGGEL_USER_H */
