#include "MCAL.h"
#include "7segment.h"
#include "7segment_user.h"
#include "Interval_Timer.h" // Needed for Interval_Timer_Enable()
#include "HAL_SSD.h"        // For SSD_Update()
#include "Touch_Pad.h"      // For TOUCH_PAD_update()
#include "tt_scheduler.h"   // For tt_add_task() and tt_dispatch_task()

/**
 * @file 7segment_main.c
 * @brief Main application entry point for the 7segment project.
 *
 * This file contains the primary execution flow, including system initialization,
 * scheduler setup, and the infinite main loop.
 */

//==================================================================================================
// Main Application Entry Point
//==================================================================================================

/**
 * @brief Main entry point of the application.
 *
 * This function initializes the system, configures and starts the
 * time-triggered scheduler, and enters an infinite loop for task dispatching.
 */
void main(void)
{
    // 1. Call to App_Init() at startup
    App_Init();

    // 2. Enable Global Interrupt
    Global_interrupt_Enable();

    // The tt_init() function from tt_scheduler.c uses SysTick.
    // Per requirements, Interval_Timer.c/h is to be used for the scheduler tick.
    // App_Init() already sets up Interval_Timer_Init and registers TT_ISR.
    // Therefore, tt_init() from tt_scheduler.c is NOT called here to avoid conflicting SysTick setup.

    // 3. Scheduler setup via tt_add_task()
    // Add tasks as per the flowchart:
    //   - TOUCH_PAD_update (delay=0, period=18ms)
    //   - SSD_Update (delay=2ms, period=5ms)
    //   - APP_MainTask (using APP_MAIN_TASK_PERIOD_MS from user.h)

    tt_add_task(TOUCH_PAD_update, 0, 18 / SCHEDULER_TICK_VALUE_MS); // Period in ticks
    tt_add_task(SSD_Update, 2 / SCHEDULER_TICK_VALUE_MS, 5 / SCHEDULER_TICK_VALUE_MS); // Period in ticks
    tt_add_task(App_MainTask, 0, APP_MAIN_TASK_PERIOD_MS / SCHEDULER_TICK_VALUE_MS); // Period in ticks

    // 4. Scheduler start via tt_start() (using Interval Timer)
    // tt_start() in tt_scheduler.c is mostly a no-op if SysTick is already enabled.
    // The actual hardware timer (Interval Timer) is started separately.
    tt_start(); // Call tt_start() for compliance, though its effect is minimal without SysTick.

    // Enable the Interval Timer (hardware timer) to start generating ticks for the scheduler.
    // This effectively "starts the timer" as depicted in the main flowchart.
    Interval_Timer_Enable();

    // 5. Infinite loop with WDT_Reset() and tt_dispatch_task()
    while (1)
    {
        WDT_Reset();         // Reset Watchdog Timer to prevent system reset
        tt_dispatch_task();  // Dispatch pending tasks from the scheduler
    }
}