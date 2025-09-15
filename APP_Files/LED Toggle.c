/**
 * @file LED Toggle.c
 * @brief Main source file for the LED Toggle middleware application.
 *
 * This file contains the initialization, middleware logic, and application
 * entry points for the LED Toggle feature, based on the provided flowchart
 * and MCAL/Scheduler interfaces.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LED Toggle.h"      // Header for LED Toggle middleware API
#include "LED Toggle_user.h" // User configuration for LED Toggle
#include "MCAL.h"            // Microcontroller Abstraction Layer (MCAL) APIs

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LED Toggle application.
 *
 * This function sets up the LED GPIO pin as an output and sets its initial state.
 * It directly implements the "APP_Init Process" steps related to GPIO initialization.
 */
void LED_Toggle_Init(void)
{
    // Ensure the Watchdog Timer is reset during initialization.
    WDT_Reset();

    // 1. GPIO_Init: Configure the LED pin as an output.
    // The `value` parameter for `GPIO_Output_Init` sets the initial state of the LED.
    GPIO_Output_Init(LED_TOGGLE_LED_PORT, LED_TOGGLE_LED_PIN, LED_TOGGLE_INITIAL_STATE);

    // After initialization, the LED should be set to its initial state.
    // The `GPIO_Output_Init` call already takes care of this.
}

/**
 * @brief Starts the LED Toggle application.
 *
 * This function registers the `LED_Toggle_Task` with the Time Triggered (TT) scheduler
 * to be executed periodically. The period and initial delay are defined in
 * `LED Toggle_user.h`.
 */
void LED_Toggle_Start(void)
{
    // Ensure the Watchdog Timer is reset.
    WDT_Reset();

    // 1. Add the LED_Toggle_Task to the scheduler.
    // The `period` and `delay` parameters are expected in units of the scheduler's
    // configured tick time. If `APP_SCHEDULER_TICK_TIME` is `tick_time_1ms`, then
    // `LED_TOGGLE_TASK_PERIOD_MS` directly corresponds to the number of ticks.
    TT_Add_task(LED_Toggle_Task, (tword)LED_TOGGLE_TASK_PERIOD_MS, (tword)LED_TOGGLE_TASK_PERIOD_MS);
}

/**
 * @brief The periodic task function to toggle the LED.
 *
 * This function implements the "Led Toggle Process" flowchart.
 * It reads the current hardware state of the LED pin and sets it to the opposite state,
 * effectively toggling the LED. This function is intended to be called by the scheduler.
 */
void LED_Toggle_Task(void)
{
    // Ensure the Watchdog Timer is reset to prevent timeout during task execution.
    // WDT_Reset() is typically called in the main loop, but can be added here
    // if tasks are long-running or critical. For short tasks, main loop call is sufficient.
    // WDT_Reset(); 

    // 1. "LED State?": Read the current hardware state of the LED pin.
    tbyte current_hw_state = GPIO_Value_Get(LED_TOGGLE_LED_PORT, LED_TOGGLE_LED_PIN);

    // 2. "ON" / "OFF" decision and action.
    if (current_hw_state == LED_TOGGLE_ON_STATE)
    {
        // "Set Led off": If the LED is currently ON, turn it OFF.
        GPIO_Value_Set(LED_TOGGLE_LED_PORT, LED_TOGGLE_LED_PIN, LED_TOGGLE_OFF_STATE);
    }
    else // current_hw_state == LED_TOGGLE_OFF_STATE or any other unexpected state
    {
        // "Set Led on": If the LED is currently OFF, turn it ON.
        GPIO_Value_Set(LED_TOGGLE_LED_PORT, LED_TOGGLE_LED_PIN, LED_TOGGLE_ON_STATE);
    }
}
