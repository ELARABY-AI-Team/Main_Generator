#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h"
#include "tt_scheduler.h" // Required for tt_add_task, if used here, otherwise in main.c

/*==============================================================================================
 * Local Defines & Constants
 *=============================================================================================*/
// No local defines needed, using macros from LEDToggel_user.h

/*==============================================================================================
 * Static Variables
 *=============================================================================================*/
// No static variables needed, LED state can be read directly from GPIO.

/*==============================================================================================
 * Function Implementations
 *=============================================================================================*/

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs necessary hardware initialization for the LED,
 * such as configuring its GPIO pin, and sets its initial state.
 * It also prepares the LED for scheduling.
 */
void App_Init(void)
{
    // 1. MCU_Config: Initialize the microcontroller configuration.
    // As per flowchart, call MCU_Config. Using 5V system voltage as a reasonable default.
    // The MCU_Config_Init function also handles WDT initialization and LVR setup.
    MCU_Config_Init(sys_volt_5v);

    // 2. GPIO_Init (Active Low 0): Configure the LED pin as an output.
    // The LED is active low (0=ON, 1=OFF). Initialize it to the OFF state first.
    GPIO_Output_Init(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, LED_INACTIVE_STATE);

    // 3. LED_SetOn (0=ON, 1=OFF): Set the LED to its initial ON state.
    // For an active-low LED, setting it ON means setting the pin to 0.
    GPIO_Value_Set(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, LED_ACTIVE_STATE);

    // WDT_Reset is called within MCAL functions, but it's good practice
    // to periodically call it in application logic if there are long tasks.
    WDT_Reset();
}

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function is designed to be called periodically by the time-triggered scheduler.
 * It reads the current state of the LED and switches it to the opposite state.
 */
void LED_Toggle_Task(void)
{
    // Read the current state of the LED pin
    tbyte current_led_state = GPIO_Value_Get(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN);

    // Check LED State and toggle it
    if (current_led_state == LED_ACTIVE_STATE)
    {
        // LED is currently ON, set it OFF
        GPIO_Value_Set(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, LED_INACTIVE_STATE);
    }
    else // current_led_state == LED_INACTIVE_STATE
    {
        // LED is currently OFF, set it ON
        GPIO_Value_Set(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, LED_ACTIVE_STATE);
    }

    // WDT_Reset is called within MCAL functions, but it's good practice
    // to periodically call it in application logic if there are long tasks.
    WDT_Reset();
}
