#ifndef LEDTOGGEL_STM_USER_H
#define LEDTOGGEL_STM_USER_H

// --- Application Configuration Parameters ---

/**
 * @brief Default system voltage for MCU configuration.
 *        Refer to MCAL's t_sys_volt enum for available options.
 *        Example: sys_volt_3v (for 3V operation on STM32F401RC).
 *        Users can adjust this if their system voltage requires a different setting.
 */
#define APP_MCU_SYSTEM_VOLTAGE      sys_volt_3v

/**
 * @brief The period in milliseconds at which the LED toggle task will execute.
 *        This defines how often the LED state changes.
 *        Value is in milliseconds.
 */
#define LED_TOGGLE_PERIOD_MS        (500U) // Toggle every 500 ms (0.5 second)

/**
 * @brief The base tick time for the time-triggered scheduler in milliseconds.
 *        All task periods and delays are calculated based on this tick rate.
 *        Lower values provide higher resolution but might increase overhead.
 */
#define SCHEDULER_TICK_RATE_MS      (10U) // Scheduler tick every 10 ms

#endif // LEDTOGGEL_STM_USER_H
