#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h" // For MCAL APIs: MCU_Config_Init, GPIO_Output_Init, GPIO_Value_Set, GPIO_Value_Get, WDT_Reset

/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the implementation of the LEDToggel application's
 * initialization logic and its time-triggered task. It directly interacts
 * with the MCAL layer for GPIO control.
 */

//==================================================================================================
// Private Variables
//==================================================================================================

/**
 * @brief Static variable to store the current state of the LED.
 *        Used to track and toggle the LED without reading the pin state every time.
 */
static tbyte currentLedState = LED_OFF_VALUE;

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs all necessary initializations for the LEDToggel
 * middleware, including MCU configuration, GPIO setup for the LED,
 * and setting the initial LED state according to the APP_Init Process flowchart.
 */
void App_Init(void)
{
    WDT_Reset(); // Keep the Watchdog Timer happy during initialization

    // 1. Configure the Microcontroller Unit (MCU_Config)
    MCU_Config_Init(LED_SYS_VOLTAGE);

    // 2. Initialize the LED pin as a GPIO output (GPIO_Init)
    // The initial value is set by LED_INITIAL_STATE from LEDToggel_user.h
    GPIO_Output_Init(LED_PORT, LED_PIN, LED_INITIAL_STATE);

    // Update the internal state variable
    currentLedState = LED_INITIAL_STATE;

    // 3. Set the LED to the ON state initially (LED_Seton) as per flowchart.
    // If LED_INITIAL_STATE is already ON, this is redundant but follows the flowchart.
    GPIO_Value_Set(LED_PORT, LED_PIN, LED_ON_VALUE);
    currentLedState = LED_ON_VALUE; // Ensure internal state reflects the actual state

    WDT_Reset(); // Final WDT reset after initialization
}

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function implements the "Leg Toggle Process" from the flowchart.
 * It reads the current state of the LED (or uses an internal state variable for efficiency)
 * and toggles it (ON to OFF, OFF to ON). This task is intended to be scheduled
 * by the Time-Triggered Scheduler.
 */
void LedToggel_RunTask(void)
{
    WDT_Reset(); // Keep the Watchdog Timer happy during task execution

    // Read the current state of the LED (or use internal state for optimization)
    // For simplicity and to directly reflect the flowchart's "LED State?" decision,
    // we'll read the actual pin state here.
    tbyte led_status = GPIO_Value_Get(LED_PORT, LED_PIN);

    if (led_status == LED_ON_VALUE)
    {
        // LED is currently ON, so turn it OFF
        GPIO_Value_Set(LED_PORT, LED_PIN, LED_OFF_VALUE);
        currentLedState = LED_OFF_VALUE;
    }
    else
    {
        // LED is currently OFF, so turn it ON
        GPIO_Value_Set(LED_PORT, LED_PIN, LED_ON_VALUE);
        currentLedState = LED_ON_VALUE;
    }

    // No need for an additional WDT_Reset() here, as GPIO_Value_Set already calls it.
}
