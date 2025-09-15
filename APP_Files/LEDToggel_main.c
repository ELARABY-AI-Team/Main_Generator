/**
 * @file LEDToggel_main.c
 * @brief Application entry point for the LEDToggel project.
 *
 * This file contains the `main()` function, which is the starting point
 * of the embedded application. It handles system initialization, sets up
 * the time-triggered scheduler, and dispatches tasks in an infinite loop.
 */

/*==============================================================================================
 * Includes
 *============================================================================================*/
#include "LEDToggel.h"      // For App_Init() and App_Toggle_LED()
#include "tt_scheduler.h"   // For scheduler functions like tt_init(), tt_add_task(), tt_start(), tt_dispatch_task()
#include "MCAL.h"           // For MCAL functions like WDT_Reset() and MCAL types like t_tick_time

/*==============================================================================================
 * Main Application Entry Point
 *============================================================================================*/

/**
 * @brief Main function of the LEDToggel application.
 *
 * This is the application entry point. It performs the following steps:
 * 1. Initializes the application-specific components using `App_Init()`.
 * 2. Initializes the Time-Triggered scheduler with a specified tick rate.
 * 3. Adds the `App_Toggle_LED()` task to the scheduler with a defined period.
 * 4. Starts the scheduler.
 * 5. Enters an infinite loop to continuously reset the Watchdog Timer and
 *    dispatch scheduled tasks.
 *
 * @return int This function should ideally never return.
 */
int main(void)
{
    // 1. Call App_Init() at startup
    // This function initializes the microcontroller and sets up the LED's initial state.
    App_Init();

    // 2. Task scheduler setup using tt_add_task()
    // Initialize the Time-Triggered scheduler.
    // We choose a 1ms tick time for fine-grained scheduling.
    // Note: The `tt_scheduler.h/c` uses `t_Tick_Time` for its `tt_init` parameter,
    // which is assumed to be compatible with `t_tick_time` from `MCAL.h`.
    tt_init(tick_time_1ms);

    // Add the App_Toggle_LED task to the scheduler.
    // The task will be executed periodically. The period and initial delay are set
    // by `LEDTOGGEL_TOGGLE_PERIOD_MS`, defined in LEDToggel_user.h.
    // Since tt_init is configured for a 1ms tick, the period and delay values
    // directly correspond to milliseconds.
    tt_add_task(App_Toggle_LED, LEDTOGGEL_TOGGLE_PERIOD_MS, LEDTOGGEL_TOGGLE_PERIOD_MS);

    // 3. Call to tt_start()
    // This function enables the timer interrupts, starting the scheduler.
    tt_start();

    // 4. Infinite loop with WDT_Reset() and tt_dispatch_task()
    // The main loop continuously keeps the Watchdog Timer happy and dispatches
    // any tasks that are due to run.
    while (1)
    {
        WDT_Reset();         // Keep the Watchdog Timer (WDT) from resetting the MCU
        tt_dispatch_task();  // Execute any tasks that are ready to run
    }

    return 0; // This line should theoretically never be reached in an embedded application's main loop
}