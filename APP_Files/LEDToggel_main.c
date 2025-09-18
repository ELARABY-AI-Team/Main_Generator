/**
 * @file LEDToggel_main.c
 * @brief Application entry point for the LEDToggel project.
 *
 * This file contains the main function, which initializes the application,
 * sets up the time-triggered task scheduler, and enters the main processing loop.
 */

/*==============================================================================================*/
/* Includes */
/*==============================================================================================*/
#include "LEDToggel.h"        // Application middleware header
#include "LEDToggel_user.h"   // User configuration for the application
#include "tt_scheduler.h"     // Time-triggered scheduler API
#include "MCAL.h"             // Microcontroller Abstraction Layer for WDT_Reset

/*==============================================================================================*/
/* Main Application Entry Point */
/*==============================================================================================*/

/**
 * @brief Main function - The entry point of the application.
 *
 * This function initializes all necessary modules, configures the scheduler,
 * and enters an infinite loop for task dispatching and watchdog refreshing.
 */
int main(void)
{
    // 1. Call App_Init() at startup.
    // This initializes the MCU and the LED GPIO as per the flowchart.
    LEDToggel_Init();

    // Calculate the task period in terms of scheduler ticks.
    // The scheduler expects periods in multiples of SCHEDULER_TICK_MS.
    const tword period_in_ticks = LED_TOGGLE_PERIOD_MS / SCHEDULER_TICK_MS;

    // Initialize the Time-Triggered Scheduler.
    // NOTE: For Renesas R5F11BBC, the provided tt_scheduler.c
    // MUST be modified to use Interval_Timer_Init and Interval_Timer_ISR_Callback
    // instead of SysTick_Config, as detailed in the preamble.
    tt_init(SCHEDULER_TICK_MS);

    // 3. Task scheduler setup using tt_add_task().
    // Add the LEDToggel_Task to the scheduler.
    // delay = 0 means the task will run on the very first tick.
    // period = period_in_ticks means it will run every 'period_in_ticks' scheduler ticks.
    tt_add_task(LEDToggel_Task, 0, period_in_ticks);

    // 4. Call tt_start() ensuring it runs on the Interval Timer driver.
    // NOTE: This call relies on tt_start() internally activating the Interval Timer
    // and its ISR, as specified in the preamble.
    tt_start();

    // 5. Infinite loop with WDT_Reset() and tt_dispatch_task().
    // This is the heart of the time-triggered operating system.
    while (1)
    {
        WDT_Reset();         // Periodically reset the Watchdog Timer to prevent MCU reset.
        tt_dispatch_task();  // Dispatch any tasks that are due to run.
    }
}