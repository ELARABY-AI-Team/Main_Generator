/**
 * @file sevensegment_main.c
 * @brief Application entry point for the Seven Segment middleware project.
 *
 * This file contains the main function, which initializes the application,
 * sets up the time-triggered scheduler, and enters the main operating loop.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "sevensegment.h"      // Seven Segment middleware application header
#include "sevensegment_user.h" // User configurations
#include "MCAL.h"              // Microcontroller Abstraction Layer for WDT_Reset and Global_interrupt_Enable
#include "Interval_Timer.h"    // Required for scheduler's timer source
#include "HAL_SSD.h"           // Required for SSD_Update task
#include "Touch_Pad.h"         // Required for TOUCH_PAD_update task

//==================================================================================================
// Main Application Entry Point
//==================================================================================================

/**
 * @brief Main function of the application.
 *
 * This function is the entry point of the embedded application. It initializes
 * the system, sets up the time-triggered scheduler, and dispatches tasks
 * in an infinite loop.
 */
void main(void)
{
    // 1. Call application initialization function
    App_Init();

    // 2. Enable Global Interrupts (explicitly mentioned in rules, also done in App_Init)
    Global_interrupt_Enable();

    // 3. Initialize the Interval Timer for the scheduler's tick
    // The flowchart indicates 1ms time trigger.
    // Use SUBSYSTEM_EXTERNAL_CLOCK or LOW_SPEED_INTERNAL_CLOCK.
    // Given 32MHz clock init, an external clock (32768Hz) for precise timing is typical.
    // The `SCHEDULER_TICK_MS` (1ms) is derived from `Tick_1_ms` enum from `Interval_Timer.h`.
    Interval_Timer_Init(SUBSYSTEM_EXTERNAL_CLOCK, SCHEDULER_TICK_MS);

    // 4. Register the scheduler's ISR callback with the Interval Timer
    Interval_Timer_ISR_Callback(TT_ISR);

    // 5. Add tasks to the time-triggered scheduler using tt_add_task()
    // Tasks are added with their respective delays and periods (in milliseconds).

    // Add TOUCH_PAD_update task: delay=0ms, period=18ms
    tt_add_task(TOUCH_PAD_update, 0, 18);

    // Add SSD_Update task: delay=2ms, period=5ms
    tt_add_task(SSD_Update, 2, 5);

    // Add APP_MainTask (application logic): delay and period from user config
    tt_add_task(APP_MainTask, APP_MAIN_TASK_DELAY_MS, APP_MAIN_TASK_PERIOD_MS);

    // 6. Start the Interval Timer to begin generating scheduler ticks
    // This corresponds to "Start Timer" in the main flowchart.
    Interval_Timer_Enable();

    // Call tt_start() - as per rules, even if its implementation in tt_scheduler.c is empty.
    tt_start();

    // 7. Enter infinite loop for task dispatching and watchdog reset
    while (1)
    {
        WDT_Reset();            // Reset Watchdog Timer to prevent MCU reset
        tt_dispatch_task();     // Dispatch pending tasks
    }
}