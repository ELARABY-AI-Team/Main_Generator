#include "MCAL.h"
#include "Interval_Timer.h" // For t_clk_source, t_Tick_Time
#include "tt_scheduler.h"
#include "7segtemp.h"
#include "7segtemp_user.h" // For SCHEDULER_TICK_MS and button IDs

// HAL update tasks
#include "Touch_Pad.h"
#include "HAL_SSD.h"

/**
 * @file 7segtemp_main.c
 * @brief Application entry point for the 7segtemp project.
 *
 * This file contains the main function which initializes the application,
 * sets up the time-triggered scheduler, and enters the main infinite loop.
 */

// Global variable to define the clock source for the Interval Timer
// Assuming LOW_SPEED_INTERNAL_CLOCK (15kHz) is suitable for 1ms ticks.
#define INTERVAL_TIMER_CLK_SOURCE LOW_SPEED_INTERNAL_CLOCK

// Redefine tt_init and tt_start to integrate with Interval_Timer
// This redefinition is necessary because the provided tt_scheduler.c uses SysTick (ARM Cortex M)
// instead of the specified Interval Timer for RENESAS_R5F11BBC.
#undef tt_init
#undef tt_start

/**
 * @brief Initializes the Time-Triggered Scheduler using the Interval Timer.
 * @param Tick_Time_ms The desired tick interval for the scheduler in milliseconds.
 */
void tt_init(uint16_t Tick_Time_ms)
{
    // Initialize task control blocks
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        tt_delete_task(i);
    }

    // Initialize the Renesas Interval Timer as the scheduler's time base
    // We use Tick_1_ms from Interval_Timer.h which corresponds to 1ms.
    Interval_Timer_Init(INTERVAL_TIMER_CLK_SOURCE, Tick_1_ms);

    // Register the scheduler's ISR with the Interval Timer's callback mechanism
    Interval_Timer_ISR_Callback(TT_ISR);
}

/**
 * @brief Starts the Time-Triggered Scheduler.
 *
 * This function enables the underlying hardware timer that drives the scheduler.
 */
void tt_start(void)
{
    // Enable the Renesas Interval Timer
    Interval_Timer_Enable();
}


/**
 * @brief Main entry point of the application.
 *
 * This function performs the following actions:
 * 1. Calls `App_Init()` for middleware initialization.
 * 2. Enables global interrupts.
 * 3. Sets up scheduled tasks using `tt_add_task()`.
 * 4. Starts the time-triggered scheduler using `tt_start()`.
 * 5. Enters an infinite loop, continuously resetting the Watchdog Timer
 *    and dispatching scheduled tasks.
 */
void main(void)
{
    // 1. Call App_Init() for application middleware initialization
    App_Init();

    // 2. Enable Global Interrupt (as per flowchart/requirements)
    Global_interrupt_Enable();

    // 3. Task scheduler setup using tt_add_task()
    // Add TOUCH_PAD_update task: delay=0ms, period=18ms
    tt_add_task(TOUCH_PAD_update, 0, 18);

    // Add SSD_Update task: delay=2ms, period=5ms
    tt_add_task(SSD_Update, 2, 5);

    // Add APP_MainTask (Logic) task: delay=0ms, period=20ms (example period for logic)
    // The flowchart implies APP_MainTask runs continuously. A periodic task is suitable.
    // Adjust period based on responsiveness requirements for button presses.
    tt_add_task(APP_MainTask, 0, 20);

    // Initialize the time-triggered scheduler with the defined tick period
    tt_init(SCHEDULER_TICK_MS);

    // 4. Call to tt_start() ensuring it runs on the Interval Timer driver
    tt_start();

    // 5. Infinite loop for task dispatching and watchdog reset
    while (1)
    {
        WDT_Reset();          // Reset Watchdog Timer to prevent system reset
        tt_dispatch_task();   // Dispatch any tasks that are due to run
    }
}