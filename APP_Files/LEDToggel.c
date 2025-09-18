/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the implementation of the LEDToggel application,
 * including initialization, LED control logic, and integration with the
 * underlying MCAL layer.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h" // Include MCAL for GPIO functions and MCU configuration

//==================================================================================================
// Global Variables
//==================================================================================================

/**
 * @brief Stores the current state of the LED.
 *        This variable is used to keep track of the LED's logical state
 *        (ON or OFF) for toggling purposes.
 */
static tbyte current_led_state = LED_OFF_VALUE; // Initialize LED to OFF (HIGH value on active-low LED)

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LEDToggel middleware application.
 *
 * This function performs necessary MCU configurations, sets up the LED GPIO,
 * and initializes the LED to its starting state as per the application logic.
 *
 * @note Follows the "APP_Init Process" flowchart from the provided documentation.
 */
void LEDToggel_Init(void)
{
    // Step 1: MCU_Config
    // Initialize MCU configuration (e.g., system voltage, WDT, LVR).
    // Using a default 3V system voltage.
    MCU_Config_Init(sys_volt_3v);

    // Step 2: GPIO_Init (High=1) Port 7, Pin 4
    // Initialize the LED pin as an output with an initial value of High (1),
    // which corresponds to LED_OFF_VALUE for an active-low LED as per flowchart.
    GPIO_Output_Init(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_OFF_VALUE);
    current_led_state = LED_OFF_VALUE; // Explicitly update internal state to match hardware

    // Step 3: LED_SetOn (0=ON, 1=OFF)
    // Turn the LED ON by setting the pin value to LED_ON_VALUE (0).
    GPIO_Value_Set(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_ON_VALUE);
    current_led_state = LED_ON_VALUE; // Update internal state to ON
}

/**
 * @brief Time-triggered task for toggling the LED.
 *
 * This function checks the current state of the LED and toggles it
 * (from ON to OFF, or OFF to ON). Designed to be run by a scheduler.
 *
 * @note Follows the "LED Toggle Task (Time Triggered)" flowchart from the
 *       provided documentation.
 */
void LEDToggel_Task(void)
{
    // Check LED State
    if (current_led_state == LED_ON_VALUE)
    {
        // If LED is currently ON (pin is set to LED_ON_VALUE), set it OFF.
        GPIO_Value_Set(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_OFF_VALUE);
        current_led_state = LED_OFF_VALUE;
    }
    else // current_led_state == LED_OFF_VALUE
    {
        // If LED is currently OFF (pin is set to LED_OFF_VALUE), set it ON.
        GPIO_Value_Set(LED_TOGGLE_PORT, LED_TOGGLE_PIN, LED_ON_VALUE);
        current_led_state = LED_ON_VALUE;
    }
}