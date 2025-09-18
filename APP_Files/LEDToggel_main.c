//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"
#include "MCAL.h"           // For MCU_Config, WDT, Global Interrupts
#include "tt_scheduler.h"   // For scheduler functions
#include "Interval_Timer.h" // For t_Tick_Time enum

/**
 * @file LEDToggel_main.c
 * @brief Application entry point for the LEDToggel project.
 *
 * This file contains the main function, responsible for system initialization,
 * scheduler setup, and the main application loop.
 */

//==================================================================================================
// Main Application Entry Point
//==================================================================================================

/**
 * @brief Main function of the LEDToggel application.
 *
 * This function serves as the entry point for the microcontroller application.
 * It initializes the system, configures the scheduler, and enters an infinite
 * loop to dispatch tasks and manage the watchdog timer.
 *
 * @return int Returns 0, though typically never exits in embedded systems.
 */
int main(void)
{
    // 1. Call to App_Init() at startup
    // This performs initial MCU setup and LED GPIO configuration.
    LEDToggel_Init();

    // Configure and start the Time-Triggered Scheduler.
    // The requirement is to get SCHEDULER_TICK_MS from Interval_Timer.h.
    // We choose Tick_1_ms from the t_Tick_Time enum, implying a 1ms scheduler tick.
    tt_init(Tick_1_ms); // Initialize scheduler with a 1ms tick period

    // 2. Enable Global Interrupt
    Global_interrupt_Enable();

    // 3. Task scheduler setup using tt_add_task()
    // Add the LEDToggel_Task to the scheduler.
    // - delay = 0: The task will run immediately on the first scheduler tick.
    // - period = LED_TOGGLE_PERIOD_MS / 1: The task will run every 500ms
    //   (since LED_TOGGLE_PERIOD_MS is 500 and the tick is 1ms).
    tt_add_task(LEDToggel_Task, 0, LED_TOGGLE_PERIOD_MS / 1);

    // 4. Call to tt_start() ensuring it runs on the Interval Timer driver
    // This function enables the configured Interval Timer to start generating
    // interrupts and triggering the scheduler's `TT_ISR`.
    tt_start();

    // 5. Infinite loop with WDT_Reset() and tt_dispatch_task()
    while (1)
    {
        WDT_Reset();         // Periodically reset the Watchdog Timer to prevent system resets.
        tt_dispatch_task();  // Dispatch ready tasks that have been flagged by the scheduler ISR.
    }

    return 0; // This line should ideally not be reached in an embedded application.
}