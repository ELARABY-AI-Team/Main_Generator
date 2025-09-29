/**
 * @file sevensegment_main.c
 * @brief Main application entry point for the sevensegment project.
 *
 * This file contains the primary `main()` function, which initializes
 * the application, sets up the time-triggered scheduler, and enters
 * the infinite task dispatch loop.
 */

//=============================================================================
// Includes
//=============================================================================
#include "sevensegment.h"         // Main application middleware header
#include "sevensegment_user.h"    // User configuration header
#include "tt_scheduler.h"         // Time-Triggered Scheduler API
#include "Interval_Timer.h"       // Interval Timer API (for enabling/disabling)
#include "MCAL.h"                 // Microcontroller Abstraction Layer (for WDT_Reset, Global_interrupt_Enable)
#include "HAL_SSD.h"              // Seven Segment Display HAL (for SSD_Update task)
#include "Touch_Pad.h"            // Touch Pad HAL (for TOUCH_PAD_update task)

//=============================================================================
// Main Application Entry Point
//=============================================================================

/**
 * @brief Main function of the sevensegment application.
 *
 * This function serves as the application's entry point, orchestrating
 * the initialization of the system and scheduling of periodic tasks
 * using the time-triggered scheduler. It then enters an infinite loop
 * to dispatch tasks and reset the watchdog timer.
 */
void main(void)
{
    // 1. Call to App_Init() at startup.
    // This initializes all hardware and software components for the application.
    App_Init();

    // 2. Enable Global Interrupt.
    // This is crucial for the time-triggered scheduler's timer ISR to operate.
    Global_interrupt_Enable();

    // 3. Task scheduler setup using tt_add_task().
    // Tasks are added with their respective delays and periods (in scheduler ticks).

    // Add TOUCH_PAD_update task: checks for touch button states.
    tt_add_task(TOUCH_PAD_update, TOUCH_PAD_UPDATE_DELAY_TICKS, TOUCH_PAD_UPDATE_PERIOD_TICKS);

    // Add SSD_Update task: refreshes the seven-segment display segments.
    tt_add_task(SSD_Update, SSD_UPDATE_DELAY_TICKS, SSD_UPDATE_PERIOD_TICKS);

    // Add App_MainTask: contains the core application logic (reading touch, updating value).
    tt_add_task(App_MainTask, APP_MAIN_TASK_DELAY_TICKS, APP_MAIN_TASK_PERIOD_TICKS);

    // 4. Call to tt_start() ensuring it runs on the Interval Timer driver.
    // The tt_scheduler's internal tt_start() is often a placeholder if SysTick is used.
    // For the Interval Timer, we explicitly enable it here to begin generating ticks.
    Interval_Timer_Enable();

    // 5. Infinite loop with WDT_Reset() and tt_dispatch_task().
    // This loop continuously dispatches scheduled tasks and prevents the
    // watchdog timer from resetting the microcontroller.
    while (1)
    {
        WDT_Reset();          // Reset the Watchdog Timer to prevent system reset.
        tt_dispatch_task();   // Dispatch any tasks that are due to run.
    }
}

//=============================================================================
// End of File
//=============================================================================