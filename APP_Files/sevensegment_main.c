#include "sevensegment.h"
#include "sevensegment_user.h"
#include "tt_scheduler.h"
#include "Interval_Timer.h"

/**
 * @file sevensegment_main.c
 * @brief Main application entry point for the sevensegment project.
 *
 * This file contains the `main()` function, which initializes the application,
 * configures and starts the time-triggered scheduler using the Interval Timer,
 * and enters the main infinite loop for task dispatching and watchdog resetting.
 */

//==============================================================================
// Application Entry Point
//==============================================================================

/**
 * @brief Main function of the application.
 *
 * This is the entry point for the microcontroller program. It performs
 * system initialization, enables global interrupts, sets up the time-triggered
 * scheduler, and then enters an infinite loop to dispatch tasks and reset the watchdog.
 *
 * @note This function adheres strictly to the specified requirements for `main()`.
 */
void main(void)
{
    // 1. Call to App_Init() at startup
    App_Init();

    // 2. Enable Global Interrupt
    Global_interrupt_Enable();

    // 3. Interval Timer setup for the scheduler
    // Initialize the Interval Timer to provide a 1ms tick for the scheduler.
    // SCHEDULER_TICK_TIME_MS is defined in sevensegment_user.h as a t_Tick_Time enum value.
    Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK, SCHEDULER_TICK_TIME_MS);

    // Register the Time-Triggered Scheduler's ISR as the callback for the Interval Timer.
    Interval_Timer_ISR_Callback(TT_ISR);

    // 4. Task scheduler setup using tt_add_task()
    // Add TOUCH_PAD_update task
    tt_add_task(TOUCH_PAD_update, 
                0 / SCHEDULER_TICK_MS_VALUE, // Delay 0ms
                18 / SCHEDULER_TICK_MS_VALUE); // Period 18ms

    // Add SSD_Update task
    tt_add_task(SSD_Update, 
                2 / SCHEDULER_TICK_MS_VALUE, // Delay 2ms
                5 / SCHEDULER_TICK_MS_VALUE); // Period 5ms

    // Add APP_MainTask (application logic)
    tt_add_task(APP_MainTask, 
                DELAY_APP_MAINTASK_TICKS, 
                PERIOD_APP_MAINTASK_TICKS);

    // 5. Call to tt_start() ensuring it runs on the Interval Timer driver.
    // This is achieved by enabling the configured Interval Timer.
    Interval_Timer_Enable();

    // 6. Infinite loop with WDT_Reset() and tt_dispatch_task()
    while (1)
    {
        WDT_Reset();         // Reset the Watchdog Timer to prevent system reset
        tt_dispatch_task();  // Dispatch any tasks that are due
    }
}