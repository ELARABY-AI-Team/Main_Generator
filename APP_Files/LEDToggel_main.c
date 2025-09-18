#include "LEDToggel.h"
#include "MCAL.h"
#include "tt_scheduler.h"
#include "Interval_Timer.h"

/*==============================================================================================
 * Main Application Entry Point
 *=============================================================================================*/

/**
 * @brief Main entry point of the LEDToggel application.
 *
 * This function initializes the application, sets up the time-triggered scheduler
 * using the Interval Timer driver, adds the LED toggling task, and enters
 * an infinite loop to dispatch tasks and reset the watchdog timer.
 */
void main(void)
{
    // 1. Initialize the LEDToggel application components.
    App_Init();

    // 2. Setup and enable the Time-Triggered Scheduler using the Interval Timer driver.
    // Disable global interrupts before configuring critical components like timers.
    Global_interrupt_Disable();

    // Initialize the Interval Timer.
    // For a 500ms toggle period, a 1ms tick time for the scheduler is appropriate.
    // Use LOW_SPEED_INTERNAL_CLOCK as a standard clock source for the Interval Timer.
    Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK, Tick_1_ms);

    // Register the Time-Triggered Scheduler's ISR (TT_ISR) as the callback for the Interval Timer.
    Interval_Timer_ISR_Callback(&TT_ISR);

    // Enable the Interval Timer to start generating periodic interrupts.
    Interval_Timer_Enable();

    // Re-enable global interrupts after configuration.
    Global_interrupt_Enable();

    // 3. Add the LED_Toggle_Task to the scheduler.
    // The task will run immediately (delay=0) and repeat every LEDTOGGEL_TOGGLE_PERIOD_MS milliseconds.
    tt_add_task(&LED_Toggle_Task, 0, LEDTOGGEL_TOGGLE_PERIOD_MS);

    // 4. Enter the main infinite loop.
    // This loop continuously resets the watchdog timer and dispatches scheduled tasks.
    while (1)
    {
        WDT_Reset();         // Keep the Watchdog Timer happy to prevent MCU resets.
        tt_dispatch_task();  // Execute any tasks that are due to run.
    }
}