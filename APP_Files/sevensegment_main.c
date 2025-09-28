/**
 * @file sevensegment_main.c
 * @brief Application entry point for the sevensegment project.
 *
 * This file contains the main function which initializes the application,
 * sets up the time-triggered scheduler, and enters the infinite dispatch loop.
 * It adheres strictly to the defined requirements for the main function structure.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "sevensegment.h"       // For App_Init, App_MainTask, SSD_Update_Task, TOUCH_PAD_Update_Task
#include "sevensegment_user.h"  // For application-specific configurations like task periods
#include "MCAL.h"               // For MCAL APIs like Global_interrupt_Enable and WDT_Reset
#include "Interval_Timer.h"     // For the hardware timer driver (Interval Timer)
#include "tt_scheduler.h"       // For the global time-triggered task scheduler

//==================================================================================================
// Main Application Entry Point
//==================================================================================================

/**
 * @brief Main function of the sevensegment application.
 *
 * This function serves as the application's entry point. It performs the
 * following critical steps as per the requirements:
 * 1. Calls `App_Init()` to initialize the middleware and hardware.
 * 2. Enables global interrupts.
 * 3. Configures the Interval Timer to provide the scheduler's tick and links it to `TT_ISR`.
 * 4. Adds all application tasks to the time-triggered scheduler.
 * 5. Calls `tt_start()` and enables the Interval Timer to begin task scheduling.
 * 6. Enters an infinite loop, continuously resetting the watchdog timer and
 *    dispatching pending tasks.
 */
void main(void)
{
    // 1. Call to App_Init() at startup
    App_Init();

    // 2. Enable Global Interrupt
    Global_interrupt_Enable();

    // Configure the Interval Timer as the scheduler's hardware tick source.
    // The flowchart specifies "Init Time Trigger (1 ms)".
    // We select the LOW_SPEED_INTERNAL_CLOCK for the Interval Timer and specify a 1ms tick.
    // Note: The specific clock source might need adjustment based on system-level clock tree.
    Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK, Tick_1_ms);

    // Link the Interval Timer's Interrupt Service Routine (ISR) to the scheduler's
    // global tick handler (TT_ISR). This makes the Interval Timer drive the scheduler.
    Interval_Timer_ISR_Callback(TT_ISR);

    // 3. Task scheduler setup using tt_add_task()
    // Add the TOUCH_PAD_update task with a delay of 0ms and a period of 18ms.
    tt_add_task(TOUCH_PAD_Update_Task, 0, SEVENSEGMENT_TOUCHPAD_UPDATE_PERIOD_MS);

    // Add the SSD_Update task with an initial delay of 2ms and a period of 5ms.
    tt_add_task(SSD_Update_Task, 2, SEVENSEGMENT_SSD_UPDATE_PERIOD_MS);

    // Add the main application logic task (App_MainTask)
    // with an initial delay of 1ms and a period defined in sevensegment_user.h.
    tt_add_task(App_MainTask, 1, SEVENSEGMENT_MAIN_TASK_PERIOD_MS);

    // 4. Call to tt_start() ensuring it runs on the Interval Timer driver.
    // The provided `tt_scheduler.c`'s `tt_start()` function is a no-operation
    // (NOP). The actual hardware timer enablement is done here via the
    // `Interval_Timer_Enable()` call to ensure the scheduler receives its ticks.
    Interval_Timer_Enable();
    tt_start(); // Call as per requirement, but relies on Interval_Timer_Enable() for timing.

    // 5. Infinite loop with WDT_Reset() and tt_dispatch_task()
    // This loop continuously runs, serving as the main system loop.
    while (1)
    {
        WDT_Reset();         // Periodically reset the Watchdog Timer to prevent system resets.
        tt_dispatch_task();  // Check for and execute any tasks that are due to run.
    }
}