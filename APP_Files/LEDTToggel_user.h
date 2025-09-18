#ifndef LED_TTOGGEL_USER_H
#define LED_TTOGGEL_USER_H

/**
 * @file LEDTToggel_user.h
 * @brief User configuration header for the LEDTToggel middleware application.
 *
 * This file allows end-users to customize the LEDTToggel application's behavior
 * without modifying the core source code.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h"       // For MCAL types like t_port, t_pin, t_sys_volt
#include "Interval_Timer.h" // For t_Tick_Time and t_clk_source

//==================================================================================================
// User Configuration Macros
//==================================================================================================

/**
 * @brief GPIO Port where the LED is connected.
 *        Refer to MCAL.h for available port enumerations (e.g., port_7).
 */
#define LED_GPIO_PORT              port_7

/**
 * @brief GPIO Pin where the LED is connected within the specified port.
 *        Refer to MCAL.h for available pin enumerations (e.g., pin_4).
 */
#define LED_GPIO_PIN               pin_4

/**
 * @brief Defines the active state of the LED.
 *        Set to 1 if the LED is active LOW (pin LOW turns LED ON).
 *        Set to 0 if the LED is active HIGH (pin HIGH turns LED ON).
 */
#define LED_ACTIVE_LOW             1

/**
 * @brief Period in milliseconds at which the LED_Toggle_Task should run.
 */
#define LED_TOGGLE_PERIOD_MS       500

/**
 * @brief System voltage level for MCU configuration.
 *        Refer to MCAL.h for available voltage enumerations (e.g., sys_volt_3v).
 */
#define MCU_SYSTEM_VOLTAGE         sys_volt_3v

/**
 * @brief The base tick time for the scheduler in milliseconds.
 *        This defines the resolution of the time-triggered scheduler.
 */
#define SCHEDULER_TICK_TIME_MS     1

/**
 * @brief Clock source for the Interval Timer which drives the scheduler.
 *        Refer to Interval_Timer.h for available clock sources (e.g., LOW_SPEED_INTERNAL_CLOCK).
 */
#define INTERVAL_TIMER_CLOCK_SOURCE LOW_SPEED_INTERNAL_CLOCK

/**
 * @brief Enumerated tick time for the Interval Timer, corresponding to SCHEDULER_TICK_TIME_MS.
 *        Must match one of the t_Tick_Time enumerations in Interval_Timer.h.
 *        If SCHEDULER_TICK_TIME_MS is 1, use Tick_1_ms.
 */
#if (SCHEDULER_TICK_TIME_MS == 1)
    #define LED_SCHEDULER_TICK_TIME_ENUM Tick_1_ms
#elif (SCHEDULER_TICK_TIME_MS == 0) // Example for other ticks
    #define LED_SCHEDULER_TICK_TIME_ENUM Tick_0_133_ms
#else
    #error "SCHEDULER_TICK_TIME_MS value does not have a direct t_Tick_Time enum mapping or is not supported."
    // Add more mappings here if the Interval_Timer.h enum supports other precise MS values
#endif

#endif // LED_TTOGGEL_USER_H
