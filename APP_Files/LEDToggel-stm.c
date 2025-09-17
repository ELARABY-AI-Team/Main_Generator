#include "LEDToggel-stm.h"
#include "LEDToggel-stm_user.h"
#include "MCAL.h" // Include MCAL for peripheral access

// --- Hardware-specific LED Pin Definition (STM32F401RC) ---
// Using GPIOA Pin 5, which is often an on-board LED (e.g., LED2 on Nucleo-F401RE).
// These definitions are specific to the hardware and are placed here,
// not in _user.h, to separate core logic from hardware specifics.
#define APP_LED_PORT    port_a
#define APP_LED_PIN     pin_5

/**
 * @brief Initializes the LED Toggle application middleware.
 *        This function sets up the MCU configuration and prepares the
 *        LED GPIO pin according to the "APP_Init Process" flowchart.
 */
void App_Init(void)
{
    // 1. Call MCU_Config to set up core microcontroller features.
    //    Uses the user-defined system voltage from LEDToggel-stm_user.h.
    MCU_Config_Init(APP_MCU_SYSTEM_VOLTAGE);

    // 2. Initialize the GPIO pin connected to the LED as an output.
    //    The "LED_Seton" step in the flowchart implies the LED should be ON
    //    after the initialization process. Hence, the initial value is set to 1 (HIGH).
    //    The `GPIO_Output_Init` function handles both direction and initial value setting,
    //    and includes internal verification loops as per MCAL requirements.
    GPIO_Output_Init(APP_LED_PORT, APP_LED_PIN, 1);
}

/**
 * @brief Time-triggered task to toggle the LED state.
 *        This function is designed to be executed periodically by a scheduler.
 *        It implements the logic described in the "Leg Toggle Process" flowchart.
 */
void App_LED_Toggle_Task(void)
{
    // Read the current state of the LED pin.
    tbyte current_led_state = GPIO_Value_Get(APP_LED_PORT, APP_LED_PIN);

    // Based on the "LED State?" decision in the flowchart:
    if (current_led_state == 1) // If the LED is currently ON
    {
        // Set the LED pin to LOW to turn it OFF.
        GPIO_Value_Set(APP_LED_PORT, APP_LED_PIN, 0);
    }
    else // If the LED is currently OFF (current_led_state == 0)
    {
        // Set the LED pin to HIGH to turn it ON.
        GPIO_Value_Set(APP_LED_PORT, APP_LED_PIN, 1);
    }
}
