/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the initialization routines, middleware logic, and
 * application entry points for the LED toggling functionality.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h"         // For GPIO functions (GPIO_Output_Init, GPIO_Value_Set, GPIO_Value_Get)
#include "tt_scheduler.h" // For tt_add_task

//==================================================================================================
// Static Variables
//==================================================================================================

/**
 * @brief Static variable to store the current state of the LED.
 *        Initialized to the OFF state as per the flowchart's initial GPIO_Output_Init setting
 *        and the LED_SetOn call in App_Init.
 */
static tbyte s_led_current_state = LED_OFF_VALUE;

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs necessary hardware initialization for the LED and
 * registers the LED toggling task with the time-triggered scheduler.
 */
void App_Init(void)
{
    // 1. MCU_Config
    // As per flowchart: Call MCU_Config to initialize basic MCU settings.
    // Assuming a typical system voltage of 5V for initialization.
    MCU_Config_Init(sys_volt_5v);

    // 2. GPIO_Init (Active Low0) Port 7, Pin 4
    // Configure the LED pin (Port 7, Pin 4) as an output.
    // "Active Low0" implies that a logical 0 turns the LED ON and a logical 1 turns it OFF.
    // Initialize the pin to the OFF state (value 1) before setting it ON.
    GPIO_Output_Init(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_OFF_VALUE);

    // 3. LED_SetOn (0=ON, 1=OFF)
    // Set the LED to the ON state (value 0) initially as per the flowchart.
    LED_Set_State(LED_ON_VALUE);

    // 4. Add Task: LED_Toggle (delay=0, period=500ms)
    // Register the LEDToggel_Task with the time-triggered scheduler.
    // It will start immediately (delay=0) and repeat every 500 scheduler ticks.
    tt_add_task(LEDToggel_Task, LED_TOGGLE_TASK_DELAY, LED_TOGGLE_TASK_PERIOD);
}

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function is called periodically by the scheduler (every LED_TOGGLE_TASK_PERIOD ticks)
 * to invert the current state of the LED.
 */
void LEDToggel_Task(void)
{
    // Keep the watchdog timer reset to prevent system resets during task execution.
    WDT_Reset();

    // 1. Check LED State? (As per flowchart)
    // Get the current logical state of the LED.
    if (LED_Get_State() == LED_ON_VALUE)
    {
        // If the LED is currently ON (logical 0), set it to OFF (logical 1).
        LED_Set_State(LED_OFF_VALUE);
    }
    else // The LED is currently OFF (logical 1)
    {
        // Set it to ON (logical 0).
        LED_Set_State(LED_ON_VALUE);
    }
}

/**
 * @brief Sets the state of the LED.
 * @param state The desired state of the LED. Use `LED_ON_VALUE` for ON
 *              or `LED_OFF_VALUE` for OFF, corresponding to active-low logic.
 */
void LED_Set_State(tbyte state)
{
    // Keep the watchdog timer reset.
    WDT_Reset();

    // Update the hardware pin to reflect the desired LED state.
    GPIO_Value_Set(LED_TOGGLE_PORT, LED_TOGGLE_PIN, state);

    // Update the internal software representation of the LED's state.
    s_led_current_state = state;
}

/**
 * @brief Gets the current state of the LED.
 * @return The current state of the LED (`LED_ON_VALUE` or `LED_OFF_VALUE`).
 *         This value reflects the logical state based on active-low definition.
 */
tbyte LED_Get_State(void)
{
    // Keep the watchdog timer reset.
    WDT_Reset();

    // Return the internally tracked state. This is typically faster and
    // more consistent than reading the pin directly if the state is well-managed.
    return s_led_current_state;
}
