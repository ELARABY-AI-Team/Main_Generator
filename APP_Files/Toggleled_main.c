#include "MCAL.h"           // For MCU_Config_Init, WDT_Reset, Global_interrupt_Enable
#include "Toggleled.h"      // For App_Init, LED_Toggle_Task
#include "Toggleled_user.h" // For application-specific configurations
#include "Interval_Timer.h" // For hardware timer setup
#include "tt_scheduler.h"   // For time-triggered task scheduler

/**
 * @file Toggleled_main.c
 * @brief Main application entry point for the Toggleled project.
 *
 * This file contains the `main()` function, which initializes the MCU,
 * sets up the scheduler, adds tasks, and enters the main application loop.
 */

//==================================================================================================
// Main Application Entry Point
//==================================================================================================

/**
 * @brief Main function - the entry point of the application.
 *
 * This function initializes the entire system, sets up the time-triggered
 * scheduler, and then enters an infinite loop to dispatch tasks and
 * reset the watchdog timer.
 * @return void (This function is not expected to return)
 */
void main(void)
{
    // 1. Call application-specific initialization.
    // This includes MCU_Config, GPIO_Init for LED, and initial LED_SetOn.
    App_Init();

    // 2. Enable Global Interrupts after all critical initialization is complete.
    Global_interrupt_Enable();

    // 3. Initialize the hardware Interval Timer for the scheduler tick.
    // The `Interval_Timer_Init` function takes a clock source and the tick duration in ms.
    Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK, APP_SCHEDULER_TICK_DURATION_MS);

    // 4. Register the scheduler's interrupt service routine (ISR) as the callback for the Interval Timer.
    Interval_Timer_ISR_Callback(&TT_ISR);

    // 5. Initialize the Time-Triggered Scheduler.
    // This clears the task array and performs any initial scheduler setup.
    // Note: The `tt_init` in `tt_scheduler.c` calls `SysTick_Config` which is Cortex-M specific.
    // For RENESAS_R5F11BBC, the actual tick source is `Interval_Timer`. This call
    // is included as per the requirement to reuse `tt_scheduler.c` as is.
    tt_init(APP_SCHEDULER_TICK_DURATION_MS);

    // 6. Add the LED_Toggle_Task to the scheduler.
    // - Task: &LED_Toggle_Task
    // - Delay: 0 (run at the first available tick)
    // - Period: TOGGLELED_TASK_PERIOD_MS divided by APP_SCHEDULER_TICK_DURATION_MS
    tt_add_task(&LED_Toggle_Task, 0, TOGGLELED_TASK_PERIOD_MS / APP_SCHEDULER_TICK_DURATION_MS);

    // 7. Start the hardware Interval Timer.
    // This will begin generating the periodic interrupts (ticks) for the scheduler.
    Interval_Timer_Enable();

    // 8. Start the Time-Triggered Scheduler.
    // In the provided `tt_scheduler.c`, this function is empty.
    tt_start();

    // 9. Enter the main infinite loop.
    // Inside this loop, the Watchdog Timer is reset, and scheduler tasks are dispatched.
    while (1)
    {
        WDT_Reset();         // Reset the Watchdog Timer to prevent system resets.
        tt_dispatch_task();  // Dispatch any tasks that are due to run.
    }
}

//==================================================================================================
// End of File
//==================================================================================================