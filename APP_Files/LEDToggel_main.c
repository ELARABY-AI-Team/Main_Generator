#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h"           // For WDT_Reset()
#include "tt_scheduler.h"   // For tt_init, tt_start, tt_add_task, tt_dispatch_task
#include "Interval_Timer.h" // For t_Tick_Time enum (implicitly used by tt_scheduler via tt_scheduler.h)

/**
 * @file LEDToggel_main.c
 * @brief Application entry point for the LEDToggel project.
 *
 * This file contains the `main()` function, which serves as the
 * entry point of the embedded application. It initializes the system,
 * sets up the time-triggered scheduler, and enters an infinite loop
 * for continuous operation and task dispatching.
 */

//==================================================================================================
// Main Application Entry Point
//==================================================================================================

/**
 * @brief Main function of the LEDToggel application.
 *
 * This function initializes the entire system, configures the
 * time-triggered scheduler, adds the LED toggling task, and
 * then continuously dispatches tasks in an infinite loop.
 *
 * @return int This function should ideally never return.
 */
int main(void)
{
    // 1. Call App_Init() for system and middleware initialization
    App_Init();

    // 2. Initialize the Time-Triggered Scheduler with the defined base tick time
    // The scheduler will internally configure the Interval Timer and its ISR.
    tt_init(TT_BASE_TICK_TIME);

    // Calculate the task period in terms of scheduler ticks.
    // Assuming TT_BASE_TICK_TIME corresponds to 1ms (Tick_1_ms) based on Interval_Timer.h enum.
    // If TT_BASE_TICK_TIME represents a different value, this conversion needs adjustment.
    const tword TASK_PERIOD_TICKS = LED_TOGGLE_PERIOD_MS; // Assuming 1 tick = 1 ms

    // 3. Add the LedToggel_RunTask to the scheduler
    // The task will run repeatedly after an initial delay equal to its period.
    // Setting initial delay to 0 means it runs on the first available tick after start, then periodically.
    // For a toggle, setting initial delay to period ensures first run is after a full cycle.
    tt_add_task(LedToggel_RunTask, TASK_PERIOD_TICKS, TASK_PERIOD_TICKS);

    // 4. Start the Time-Triggered Scheduler
    tt_start();

    // 5. Enter the infinite loop for continuous operation
    while (1)
    {
        WDT_Reset();         // Periodically reset the Watchdog Timer
        tt_dispatch_task();  // Dispatch any tasks that are due to run
    }

    // This point should never be reached in a typical embedded application
    return 0;
}