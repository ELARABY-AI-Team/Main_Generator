/**
 * @file LEDTToggel_main.c
 * @brief Application entry point for the LEDTToggel project.
 *
 * This file contains the main function which serves as the application's
 * entry point. It handles initial setup, scheduler configuration, and
 * the main application loop.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDTToggel.h"
#include "LEDTToggel_user.h"
#include "MCAL.h"
#include "tt_scheduler.h"
#include "Interval_Timer.h"

//==================================================================================================
// Main Function
//==================================================================================================

/**
 * @brief Main function - The entry point of the application.
 *
 * This function initializes the system, configures and starts the time-triggered
 * scheduler, and then enters an infinite loop for continuous operation.
 */
int main(void)
{
    // 1. Call to App_Init() at startup
    App_Init();

    // Disable global interrupts during critical scheduler setup
    Global_interrupt_Disable();

    // The provided tt_scheduler.c's tt_init() hardcodes SysTick_Config().
    // To ensure the scheduler runs on the Interval Timer driver as required,
    // we will directly configure the Interval Timer to call the scheduler's TT_ISR.
    // This effectively replaces the timer setup portion of tt_init().

    // Optionally, clear existing tasks in the scheduler's task list (mimicking tt_init's effect)
    // for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    // {
    //    tt_delete_task(i);
    // }
    // Note: tt_add_task automatically finds the next available slot, so this manual
    // clearing might not be strictly necessary if tasks are added correctly, but good for robustness.
    // For this simple application, we rely on tt_add_task finding an empty slot.

    // Initialize Interval Timer to provide the scheduler's tick (e.g., 1ms tick)
    Interval_Timer_Init(INTERVAL_TIMER_CLOCK_SOURCE, LED_SCHEDULER_TICK_TIME_ENUM);

    // Register the time-triggered scheduler's ISR as the Interval Timer's callback
    Interval_Timer_ISR_Callback(TT_ISR);

    // Enable global interrupts after setup
    Global_interrupt_Enable();

    // 2. Task scheduler setup using tt_add_task()
    // Add the LED toggle task with a 0ms delay (run immediately on first tick)
    // and a period defined in LEDTToggel_user.h.
    tt_add_task(LED_Toggle_Task, 0, LED_TOGGLE_PERIOD_TICKS);

    // 3. Call to tt_start() ensuring it runs on the Interval Timer driver.
    // The tt_start() function in the provided tt_scheduler.c is empty.
    // The actual timer start is handled by Interval_Timer_Enable().
    Interval_Timer_Enable();

    // 4. Infinite loop with WDT_Reset() and tt_dispatch_task()
    while (1)
    {
        WDT_Reset();         // Keep the Watchdog Timer happy
        tt_dispatch_task();  // Dispatch pending tasks
    }

    return 0; // Should not be reached in an embedded system infinite loop
}

//==================================================================================================
// End of File
//==================================================================================================