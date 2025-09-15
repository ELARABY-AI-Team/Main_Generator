/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the initialization routines, middleware logic,
 * and application-specific entry points (excluding main()) for the
 * LEDToggel project on RENESAS_R5F11BBC.
 */

/*==============================================================================================
 * Includes
 *============================================================================================*/
#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h" // Provides MCAL API functions like GPIO_Output_Init, GPIO_Value_Set, GPIO_Value_Get, MCU_Config_Init

/*==============================================================================================
 * Local Defines
 *============================================================================================*/

// Define the LED GPIO port and pin.
// As per project requirements, hardware-specific configurations like pin assignments
// are expected to be defined in a lower-level HAL or system configuration.
// Since a dedicated HAL_LED.h was not provided, and LEDToggel_user.h is for
// user-editable logical parameters, these are defined here as an application's
// specific hardware interface assumption.
// Assuming Port_1, Pin_0 for the LED based on typical board configurations.
#define LED_GPIO_PORT           (port_1)
#define LED_GPIO_PIN            (pin_0)

/*==============================================================================================
 * Private Variables
 *============================================================================================*/

/**
 * @brief Static variable to track the current logical state of the LED.
 *        0 indicates OFF, 1 indicates ON.
 */
static tbyte current_led_state = 0; // Initialize to OFF

/*==============================================================================================
 * Function Implementations
 *============================================================================================*/

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs necessary MCU configuration, initializes the LED GPIO,
 * and sets the initial state of the LED as per the 'APP_Init Process' flowchart.
 */
void App_Init(void)
{
    // Ensure Watchdog Timer is reset periodically, especially during critical initialization.
    WDT_Reset();

    // 1. MCU_Config (from 'APP_Init Process' flowchart)
    // Initialize MCU with a default system voltage (e.g., 3V).
    // This call handles global MCU settings, WDT initialization, LVR, and disabling unused peripherals.
    MCU_Config_Init(sys_volt_3v);

    WDT_Reset(); // Reset WDT after MCU config

    // 2. GPIO_Init (from 'APP_Init Process' flowchart)
    // Initialize the designated LED pin as a digital output.
    // The LED is initially set to OFF (value 0) before its direction is fully established.
    GPIO_Output_Init(LED_GPIO_PORT, LED_GPIO_PIN, 0);

    WDT_Reset(); // Reset WDT after GPIO init

    // 3. LED_Seton (from 'APP_Init Process' flowchart)
    // Set the LED to its initial ON state as required by the flowchart.
    GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, 1);
    current_led_state = 1; // Update the internal state to ON

    WDT_Reset(); // Final WDT reset for init completion
}

/**
 * @brief Toggles the state of the LED.
 *
 * This function reads the current state of the LED and inverts it,
 * as per the 'Leg Toggle Process' flowchart. This function is intended
 * to be called periodically by a scheduler to achieve a blinking effect.
 */
void App_Toggle_LED(void)
{
    // The current state is tracked by 'current_led_state'.
    // We can directly toggle the GPIO and update this state variable.
    // Reading `GPIO_Value_Get` could be used to retrieve actual pin state,
    // but relying on `current_led_state` for this simple toggle is efficient.

    // If LED State is ON, set Led off. Else (if OFF), set Led on.
    if (current_led_state == 1) // Current LED State is ON
    {
        GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, 0); // Set LED OFF
        current_led_state = 0; // Update internal state
    }
    else // Current LED State is OFF
    {
        GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, 1); // Set LED ON
        current_led_state = 1; // Update internal state
    }

    // WDT_Reset is implicitly handled by the main loop, but can be
    // added in task functions if the task duration is long.
}
