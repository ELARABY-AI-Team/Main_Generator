/**
 * @file LEDToggel_main.c
 * @brief Application entry point for the LEDToggel project.
 *
 * This file contains the main function, which initializes the application,
 * sets up the time-triggered scheduler, and enters the main infinite loop.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"        // Contains `App_Init` and application-specific definitions
#include "LEDToggel_user.h"   // Contains user-configurable parameters like `SCHEDULER_BASE_TICK_MS`
#include "tt_scheduler.h"     // Provides the Time-Triggered Scheduler API (tt_init, tt_add_task, tt_start, tt_dispatch_task)
#include "Interval_Timer.h"   // Provides the Interval Timer API (for scheduler's underlying timer)
#include "MCAL.h"             // Provides MCAL functions (WDT_Reset, Global_interrupt_Enable/Disable, Go_to_sleep_mode)

//==================================================================================================
// Note on Scheduler Integration
//==================================================================================================
/**
 * @note The provided `tt_scheduler.c` and `tt_scheduler.h` files are designed for
 *       a generic platform and refer to `SysTick_Config` and `HAL_RCC_GetHCLKFreq`.
 *       For full integration with the `RENESAS_R5F11BBC` microcontroller and the
 *       `Interval_Timer` driver as required, the `tt_scheduler.c` file MUST be
 *       modified as follows (these changes are assumed to be performed manually
 *       in the `tt_scheduler.c` file by the user):
 *
 *       1. **Remove SysTick-related code:** Remove calls to `SysTick_Config` and
 *          references to `HAL_RCC_GetHCLKFreq` in `tt_init`.
 *       2. **Integrate Interval_Timer in `tt_init`:**
 *          Replace the SysTick initialization with:
 *          `Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK, Tick_1_ms);`
 *          (Assuming `SCHEDULER_BASE_TICK_MS` is 1ms, mapped to `Tick_1_ms` enum).
 *       3. **Register `TT_ISR` as Interval Timer Callback:**
 *          Add: `Interval_Timer_ISR_Callback(TT_ISR);`
 *       4. **Integrate Interval_Timer in `tt_start`:**
 *          Replace empty `tt_start` with: `Interval_Timer_Enable();`
 *       5. **Adapt `tt_sleep`:**
 *          Change `__WFI();` to `Go_to_sleep_mode();` or `__asm("HALT");`
 *       6. **Update `tt_scheduler.h`:**
 *          Change `#include "main.h"` to `#include <stdint.h>` or `#include "MCAL.h"`.
 *          (And potentially `#include "Interval_Timer.h"` in `tt_scheduler.c`).
 *
 *       This `LEDToggel_main.c` assumes these modifications have been made to
 *       `tt_scheduler.c/h` for proper functionality.
 */

//==================================================================================================
// main function
//==================================================================================================

/**
 * @brief Main entry point of the application.
 *
 * This function initializes the microcontroller and the LEDToggel application,
 * then starts the time-triggered scheduler and enters an infinite loop
 * for task dispatching and watchdog refreshing.
 */
int main(void)
{
    // Disable global interrupts during critical initialization phases
    Global_interrupt_Disable();

    // 1. Initialize the LEDToggel application components.
    // This includes MCU configuration, LED GPIO setup, initial LED state,
    // and registering the LED toggling task with the scheduler.
    App_Init();
    
    // 2. Initialize the Time-Triggered Scheduler.
    // This call is assumed to internally configure and register with the Interval Timer,
    // based on the `SCHEDULER_BASE_TICK_MS` defined in `LEDToggel_user.h`.
    tt_init(SCHEDULER_BASE_TICK_MS);

    // Enable global interrupts after all essential initializations are complete
    Global_interrupt_Enable();

    // 3. Start the Time-Triggered Scheduler.
    // This call is assumed to enable the underlying Interval Timer, allowing
    // periodic interrupts to drive the scheduler's task dispatching.
    tt_start();

    // The main application loop. It runs continuously, dispatching tasks
    // and refreshing the watchdog timer.
    while (1)
    {
        // 4. Reset the Watchdog Timer (WDT) regularly.
        // This is crucial to prevent the WDT from timing out and resetting the MCU.
        WDT_Reset();

        // 5. Dispatch scheduled tasks.
        // This function checks if any tasks are due to run and executes them.
        // It may also put the MCU into a low-power sleep mode if no tasks are pending.
        tt_dispatch_task();
    }
}