#include "LEDToggel-stm.h"
#include "LEDToggel-stm_user.h"
#include "MCAL.h"             // For MCU_Config_Init, WDT_Reset, Go_to_sleep_mode
#include "tt_scheduler.h"     // For time-triggered scheduler functions
#include "Interval_Timer.h"   // Included by tt_scheduler, no direct use here

/**
 * @brief Main entry point of the application.
 *        This function orchestrates the initialization of the application,
 *        sets up the time-triggered scheduler, and enters an infinite loop
 *        to continuously dispatch tasks and maintain system health.
 */
int main(void)
{
    // 1. Initialize the application middleware.
    //    This sets up the MCU, configures the LED GPIO, and sets its initial state.
    App_Init();

    // 2. Initialize the time-triggered scheduler.
    //    The scheduler uses the Interval Timer driver internally, configured with
    //    the desired tick rate from LEDToggel-stm_user.h.
    tt_init(SCHEDULER_TICK_RATE_MS);

    // 3. Add the LED Toggle task to the scheduler.
    //    The task will be executed periodically. Its delay and period are calculated
    //    based on the desired LED_TOGGLE_PERIOD_MS and the SCHEDULER_TICK_RATE_MS.
    //    Setting initial delay equal to period ensures the first execution happens
    //    after one full period and then continues periodically.
    tt_add_task(App_LED_Toggle_Task,
                LED_TOGGLE_PERIOD_MS / SCHEDULER_TICK_RATE_MS,  // Initial delay in ticks
                LED_TOGGLE_PERIOD_MS / SCHEDULER_TICK_RATE_MS); // Period in ticks

    // 4. Start the time-triggered scheduler.
    //    This enables the underlying Interval Timer interrupts, beginning the
    //    periodic execution of scheduled tasks.
    tt_start();

    // 5. Enter the main application loop.
    //    This loop continuously checks for and dispatches tasks, and resets the
    //    watchdog timer to prevent system resets.
    while (1)
    {
        // Reset the Watchdog Timer. This is crucial for system stability
        // and must be called regularly within the main loop.
        WDT_Reset();

        // Dispatch tasks that are marked as 'run_me' by the scheduler's ISR.
        // This function executes pending tasks.
        tt_dispatch_task();

        // Optionally, put the MCU into a low-power sleep mode until the next
        // timer interrupt. This can save power in idle periods.
        // Go_to_sleep_mode(); // Uncomment this line if power saving is desired.
    }
}