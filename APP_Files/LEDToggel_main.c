/**
 * @file LEDToggel_main.c
 * @brief Main application entry point for the LEDToggel project.
 *
 * This file contains the main function, system initialization, scheduler setup,
 * and the infinite loop for the RENESAS_R5F11BBC microcontroller.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"        // Our middleware application header
#include "MCAL.h"             // Microcontroller Abstraction Layer for WDT and global interrupts
#include "tt_scheduler.h"     // Time-triggered scheduler API
#include "Interval_Timer.h.h" // Hardware-specific timer driver for the scheduler

//==================================================================================================
// Main Function
//==================================================================================================

/**
 * @brief Main entry point of the application.
 *
 * This function initializes the microcontroller, sets up the LED toggling
 * application, configures and starts the time-triggered scheduler using the
 * Interval Timer, and then enters an infinite loop for task dispatching.
 *
 * @return int Standard main function return (should not be reached in embedded systems).
 */
int main(void)
{
    // 1. Disable global interrupts before initial configuration to prevent unintended behavior.
    Global_interrupt_Disable();

    // 2. Call App_Init() at startup for middleware initialization.
    App_Init();

    // 3. Configure and initialize the Interval Timer for the time-triggered scheduler.
    //    Using LOW_SPEED_INTERNAL_CLOCK (15kHz) and a 1ms tick time as defined in Interval_Timer.c.
    Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK, Tick_1_ms);

    // 4. Register the scheduler's ISR (TT_ISR) as the callback for the Interval Timer.
    Interval_Timer_ISR_Callback(TT_ISR);

    // 5. Setup the LED_Toggle_Task using the time-triggered scheduler.
    //    According to the flowchart, the task has a delay of 0 and a period of 500ms.
    //    Since our tick is 1ms, the period is 500 ticks.
    tt_add_task(LED_Toggle_Task, 0, 500); // delay=0, period=500 ticks (500ms)

    // 6. Enable the Interval Timer to start generating scheduler ticks.
    Interval_Timer_Enable();

    // 7. Enable global interrupts now that the system is configured.
    Global_interrupt_Enable();

    // 8. Enter the infinite loop for continuous operation.
    while (1)
    {
        // 8.1. Reset the Watchdog Timer (WDT_Reset()) to prevent system resets.
        WDT_Reset();

        // 8.2. Dispatch scheduled tasks (tt_dispatch_task()).
        //      This function checks for pending tasks and executes them.
        //      It also includes a sleep mechanism (`tt_sleep()`) to save power if no tasks are ready.
        tt_dispatch_task();
    }

    // This line should not be reached in typical embedded applications.
    return 0;
}

//==================================================================================================
// End of File
//==================================================================================================