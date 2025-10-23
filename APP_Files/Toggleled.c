#include "Toggleled.h"
#include "Toggleled_user.h"
#include "MCAL.h"       // Include MCAL for GPIO operations
#include "tt_scheduler.h" // Include for scheduler task management

/**
 * @file Toggleled.c
 * @brief Middleware application logic for the Toggleled project.
 *
 * This file implements the core behavior of the Toggleled application,
 * integrating with the HAL and scheduler layers as required.
 */

//==================================================================================================
// Global Variables (if any, typically static to this file)
//==================================================================================================

// No global variables needed for this simple LED toggle application.

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the Toggleled application and its peripherals.
 *
 * This function performs necessary hardware and software initializations
 * required for the Toggleled application to operate correctly, as described
 * in the "APP_Init Process" flowchart.
 */
void App_Init(void)
{
    // 1. Initialize MCU configuration (system clock, WDT, LVR, etc.)
    // Assuming sys_volt_3v as a common default.
    MCU_Config_Init(sys_volt_3v);

    // 2. Initialize the LED GPIO pin as output and set its initial state.
    // The flowchart indicates "GPIO_Init (High=1)" and then "LED_SetOn (0=ON, 1=OFF)".
    // This means initially setting the pin to HIGH (1), and then explicitly setting it to ON (0).
    // So, initialize to OFF_VALUE (1), then immediately set to ON_VALUE (0).
    GPIO_Output_Init(TOGGLELED_LED_PORT, TOGGLELED_LED_PIN, TOGGLELED_LED_OFF_VALUE);

    // 3. Set the LED to its initial ON state as per flowchart "LED_SetOn (0=ON, 1=OFF)".
    GPIO_Value_Set(TOGGLELED_LED_PORT, TOGGLELED_LED_PIN, TOGGLELED_LED_ON_VALUE);
}

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function checks the current state of the LED and toggles it
 * between ON and OFF, as described in the "LED Toggle Task" flowchart.
 * It uses MCAL GPIO functions to interact with the LED.
 */
void LED_Toggle_Task(void)
{
    // The flowchart explicitly shows checking the LED state and then setting it.
    // However, the MCAL provides `GPIO_Value_Tog` which directly toggles the pin,
    // achieving the same result more concisely and efficiently.
    GPIO_Value_Tog(TOGGLELED_LED_PORT, TOGGLELED_LED_PIN);
}

//==================================================================================================
// End of File
//==================================================================================================
