#include "LEDToggle.h"
#include "LEDToggle_user.h"
#include "MCAL.h" // For MCAL GPIO and TT functions

/**
 * @file LEDToggle.c
 * @brief Main source file for the LEDToggle middleware application.
 *
 * This file contains the initialization, middleware logic, and application
 * entry points for the LEDToggle project. It utilizes MCAL's GPIO and
 * Time-Triggered scheduler functionalities.
 */

/*==============================================================================================
*                                    Private Variables
==============================================================================================*/
// No private variables needed for this simple application.

/*==============================================================================================
*                                    Function Implementations
==============================================================================================*/

/**
 * @brief Initializes the LEDToggle application.
 *
 * This function performs the following steps:
 * 1. Initializes the specified LED GPIO pin as an output and sets its initial state.
 * 2. Initializes the MCAL's Time-Triggered scheduler with the defined tick time.
 * 3. Adds the `LEDToggle_ToggleTask` to the scheduler to run periodically.
 *
 * Corresponds to the "APP_Init Process" flowchart:
 * - MCU_Config (handled in main.c)
 * - GPIO_Init (performed here)
 * - LED_Seton (performed here)
 */
void LEDToggle_Init(void)
{
    // 1. Initialize the LED GPIO pin as output and set its initial state.
    // GPIO_Output_Init also sets the initial value.
    GPIO_Output_Init(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_INITIAL_STATE);

    // 2. Initialize the MCAL's Time-Triggered scheduler.
    // The scheduler tick time is configured by the user.
    TT_Init(LED_SCHEDULER_TICK_TIME);

    // 3. Add the LEDToggle_ToggleTask to the scheduler.
    // Calculate period and delay in ticks.
    const tword task_period_ticks = LED_TOGGLE_PERIOD_MS / TICK_PERIOD_MS;
    TT_Add_task(&LEDToggle_ToggleTask, task_period_ticks, task_period_ticks);
}

/**
 * @brief Starts the LEDToggle application.
 *
 * This function simply calls the MCAL's TT_Start function to enable
 * the scheduler, allowing the registered tasks to begin execution.
 */
void LEDToggle_Start(void)
{
    // Start the Time-Triggered scheduler.
    TT_Start();
}

/**
 * @brief The periodic task to toggle the LED state.
 *
 * This function is registered with the Time-Triggered scheduler and is called
 * every `LED_TOGGLE_PERIOD_MS`. It reads the current state of the LED and
 * then sets it to the opposite state.
 *
 * Corresponds to the "Leg Toggle Process" flowchart.
 */
void LEDToggle_ToggleTask(void)
{
    tbyte current_led_state;

    // 1. Read the current LED state.
    current_led_state = GPIO_Value_Get(LED_TOGGLE_PORT, LED_TOGGLE_PIN);

    // 2. Toggle the LED state based on its current value.
    if (current_led_state == LED_ON_VALUE)
    {
        // If LED is ON, set it OFF.
        GPIO_Value_Set(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_OFF_VALUE);
    }
    else
    {
        // If LED is OFF, set it ON.
        GPIO_Value_Set(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_ON_VALUE);
    }
}

/*==============================================================================================
*                                    Interrupt Service Routines
==============================================================================================*/
// Note: The actual timer ISR (e.g., TAU0 Channel 7 for MCAL's TT_ISR) should be
// defined in the MCAL layer or startup code, which then calls MCAL's TT_ISR.
// No direct ISR definition here.
