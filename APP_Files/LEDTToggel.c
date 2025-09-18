/**
 * @file LEDTToggel.c
 * @brief Main source file for the LEDTToggel middleware application.
 *
 * This file contains the initialization, middleware logic, and application
 * entry points for the LEDTToggel project. It integrates with MCAL and
 * the provided time-triggered scheduler.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDTToggel.h"
#include "LEDTToggel_user.h"
#include "MCAL.h"

//==================================================================================================
// Private Variables
//==================================================================================================
// No specific private variables needed for this simple LED toggler.

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LEDTToggel application middleware.
 *
 * This function performs necessary hardware initialization for the LED and
 * prepares the application for operation, following the APP_Init Process flowchart.
 */
void App_Init(void)
{
    // Step 1: MCU_Config (Initialize MCU with specified voltage)
    MCU_Config_Init(MCU_SYSTEM_VOLTAGE);

    // Step 2: GPIO_Init (Active Low0) Port 7, Pin 4
    // Initialize the LED pin as an output. Set initial value to OFF state.
    // As per flowchart, "Active Low0" means LED is ON when pin is LOW.
    // Initial state set to OFF (pin HIGH) as per common practice before turning ON.
    GPIO_Output_Init(LED_GPIO_PORT, LED_GPIO_PIN, LED_OFF_STATE);

    // Step 3: LED_SetOn (0=ON, 1=OFF)
    // Turn the LED ON initially as required by the flowchart.
    LED_SetOn();

    // The "Add Task" step is handled in LEDTToggel_main.c after App_Init.
}

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This task is executed periodically by the scheduler to change the LED's
 * ON/OFF state, following the LED Toggle Task flowchart.
 */
void LED_Toggle_Task(void)
{
    // The GPIO_Value_Tog function simplifies the logic of reading the state
    // and then setting the opposite state, correctly handling active-low/high
    // based on the underlying MCAL implementation.
    GPIO_Value_Tog(LED_GPIO_PORT, LED_GPIO_PIN);
}

/**
 * @brief Sets the LED to the ON state.
 *
 * This function controls the GPIO pin to turn the LED ON based on
 * the configured active level (LED_ON_STATE).
 */
void LED_SetOn(void)
{
    GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, LED_ON_STATE);
}

/**
 * @brief Sets the LED to the OFF state.
 *
 * This function controls the GPIO pin to turn the LED OFF based on
 * the configured active level (LED_OFF_STATE).
 */
void LED_SetOff(void)
{
    GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, LED_OFF_STATE);
}

//==================================================================================================
// End of File
//==================================================================================================
