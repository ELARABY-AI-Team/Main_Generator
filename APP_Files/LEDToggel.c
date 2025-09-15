/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the implementation of initialization routines,
 * middleware logic, and application entry points as per the project requirements.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"         // Middleware public interface
#include "LEDToggel_user.h"    // User configuration settings
#include "MCAL.h"              // Microcontroller Abstraction Layer functions (GPIO, MCU_Config)

//==================================================================================================
// Private Variables
//==================================================================================================

/**
 * @brief Static variable to track the current state of the LED.
 *        'true' indicates the LED is ON, 'false' indicates it's OFF.
 */
static bool s_led_is_on = false;

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs the initial setup as described in the APP_Init Process
 * flowchart, including MCU configuration, GPIO initialization for the LED,
 * and setting the LED to an initial ON state.
 */
void APP_Init(void)
{
    // Step 1: MCU_Config (from flowchart)
    // Initialize the MCU with a specified system voltage.
    // Assuming a 5V system voltage for general purpose.
    MCU_Config_Init(sys_volt_5v);

    // Step 2: GPIO_Init and LED_Seton (from flowchart)
    // Initialize the LED pin as an output and set its initial state to ON.
    // The GPIO_Output_Init function handles both direction setting and initial value.
    GPIO_Output_Init(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, 1);
    s_led_is_on = true; // Update the internal state tracker
}

/**
 * @brief Turns the configured LED ON.
 *
 * This function sets the GPIO pin associated with the LED to a high state,
 * illuminating the LED.
 */
void led_on(void)
{
    // Set the LED GPIO pin to logic high (ON)
    GPIO_Value_Set(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, 1);
    s_led_is_on = true; // Update the internal state tracker
}

/**
 * @brief Turns the configured LED OFF.
 *
 * This function sets the GPIO pin associated with the LED to a low state,
 * turning off the LED.
 */
void led_off(void)
{
    // Set the LED GPIO pin to logic low (OFF)
    GPIO_Value_Set(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, 0);
    s_led_is_on = false; // Update the internal state tracker
}

/**
 * @brief Toggles the state of the configured LED.
 *
 * This function implements the "Leg Toggle Process" flowchart. It reads the
 * current state of the LED and switches it to the opposite state (ON to OFF,
 * or OFF to ON).
 */
void LEDTOGGEL_ToggleLed(void)
{
    // Check the current LED state using the internal tracker.
    // Relying on the tracker (`s_led_is_on`) is generally preferred for performance
    // over reading the actual pin state (`GPIO_Value_Get`) in time-critical tasks,
    // assuming the tracker is kept consistent by `led_on` and `led_off`.
    if (s_led_is_on) // If LED State is ON
    {
        led_off();   // Set Led off
    }
    else             // If LED State is OFF
    {
        led_on();    // Set Led on
    }
}

//==================================================================================================
// End of File
//==================================================================================================
