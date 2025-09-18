/**
 * @file LEDToggel_main.c
 * @brief Application entry point for the LEDToggel project.
 *
 * This file contains the main function, which initializes the application,
 * sets up the time-triggered scheduler, and enters the main processing loop.
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "LEDToggel.h"        /* For App_Init() prototype */
#include "LEDToggel_user.h"   /* For LEDTOGGLE_PERIOD_MS */
#include "tt_scheduler.h"     /* For scheduler functions */
#include "Interval_Timer.h"   /* For the scheduler's underlying timer driver */
#include "MCAL.h"             /* For WDT_Reset() */

/*==================================================================================================
*                                        MAIN FUNCTION
==================================================================================================*/

/**
 * @brief Main function - the application entry point.
 *
 * This function initializes the microcontroller, the LED toggling middleware,
 * sets up and starts the time-triggered scheduler, and then enters an
 * infinite loop to dispatch tasks and reset the watchdog timer.
 *
 * @return int This function should ideally never return.
 */
int main(void)
{
    // 1. Call to App_Init() at startup.
    // This initializes the microcontroller (MCU_Config_Init), configures the LED GPIO,
    // and adds the LED toggling task to the scheduler.
    App_Init();

    // 2. Scheduler setup and initialization.
    // Initialize the time-triggered scheduler.
    // The scheduler will use the Interval Timer driver.
    // The Tick_Time_ms is passed as a value which the Interval Timer will interpret
    // using its default calculation for non-enum values.
    tt_init(LEDTOGGLE_PERIOD_MS); // Note: Scheduler tick rate typically dictates the minimum task period.
                                 // Here, the scheduler tick is set to the task period for simplicity,
                                 // meaning the task runs immediately when due. If multiple tasks with
                                 // smaller periods are present, the tick rate should be smaller (e.g., 1ms).
                                 // For this single task, a tick of 500ms works fine.

    // 3. Call to tt_start() ensuring it runs on the Interval Timer driver.
    // This enables the Interval Timer, which will periodically call the scheduler's ISR (TT_ISR).
    tt_start();

    // 4. Infinite loop with WDT_Reset() and tt_dispatch_task().
    while (1)
    {
        WDT_Reset();        // Reset the Watchdog Timer to prevent MCU reset.
        tt_dispatch_task(); // Dispatch any ready tasks from the scheduler.
        Go_to_sleep_mode(); // Optional: Enter low-power mode if no tasks are ready.
                            // The tt_dispatch_task function already includes __WFI().
    }

    // Should theoretically not be reached.
    return 0;
}

/*==================================================================================================
*                                        END OF FILE
==================================================================================================*/