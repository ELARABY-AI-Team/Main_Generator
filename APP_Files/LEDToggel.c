/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the implementation of the LEDToggel middleware,
 * including initialization routines and the periodic LED toggling logic.
 * It integrates with the MCAL and scheduler layers.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h" // Required for GPIO functions

//==================================================================================================
// Global Variables (if any)
//==================================================================================================
// No specific global variables needed for this simple toggle logic using GPIO_Value_Tog.

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LEDToggel middleware.
 *
 * This function performs necessary initializations for the LED blinking
 * application, including configuring the LED GPIO pin as an output
 * and setting its initial state to ON, as per the APP_Init Process flowchart.
 */
void LEDToggel_Init(void)
{
    // Initialize the LED pin as an output and set its initial state.
    // As per the flowchart: "GPIO_Init" and then "LED_Seton".
    // Assuming LED is active high, so 1 for ON.
    GPIO_Output_Init(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_ON_LEVEL);
}

/**
 * @brief Task function for toggling the LED state.
 *
 * This function implements the core LED toggling logic as described in the
 * Leg Toggle Process flowchart. It checks the current LED state and
 * switches it to the opposite state. This function is intended to be
 * called periodically by a scheduler.
 */
void LEDToggel_ToggleTask(void)
{
    // The "Leg Toggle Process" flowchart shows:
    // 1. Check "LED State?" (ON or OFF)
    // 2. If ON, "Set Led off"
    // 3. If OFF, "Set Led on"
    // The MCAL provides a direct `GPIO_Value_Tog` function which simplifies this logic.
    GPIO_Value_Tog(LED_TOGGLE_PORT, LED_TOGGLE_PIN);
}

//==================================================================================================
// End of File
//==================================================================================================
