/**
 * @file LEDToggel.c
 * @brief Middleware application code for LED toggling functionality.
 *
 * This file contains the initialization routines and the time-triggered task
 * for toggling an LED on the RENESAS_R5F11BBC microcontroller.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h" // For MCAL GPIO and MCU configuration APIs

//==================================================================================================
// Local Defines and Macros
//==================================================================================================

/**
 * @brief Specifies the GPIO port for the LED.
 * @note This is a hardware detail and should ideally be abstracted by HAL or
 *       defined in a hardware-specific configuration, not here.
 */
#define LEDTOGGEL_LED_PORT     port_7

/**
 * @brief Specifies the GPIO pin for the LED.
 * @note This is a hardware detail and should ideally be abstracted by HAL or
 *       defined in a hardware-specific configuration, not here.
 */
#define LEDTOGGEL_LED_PIN      pin_4

//==================================================================================================
// Global Variables (if any)
//==================================================================================================

// No global variables are strictly needed for this simple LED toggling logic.

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application middleware.
 *
 * This function performs the necessary MCU configurations and initializes the
 * LED GPIO pin as an output, setting its initial state as specified by the flowchart.
 * It follows the "APP_Init Process" flowchart.
 */
void App_Init(void)
{
    // --- Start ---

    // 1. MCU_Config
    // Initialize MCU with a default system voltage (e.g., 5V).
    // The specific voltage chosen might depend on the board's power supply.
    MCU_Config_Init(sys_volt_5v);

    // 2. GPIO_Init (Active Low0) -> Port 7, Pin 4
    // Initialize Port 7, Pin 4 as an output pin.
    // "Active Low0" means that a logic LOW (0) turns the LED ON.
    // The flowchart also specifies "LED_SetOn (0=ON, 1=OFF)", which means
    // the initial state should be ON (logic 0).
    GPIO_Output_Init(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN, LEDTOGGEL_INITIAL_STATE_VALUE);

    // --- End (Scheduler task addition will be in main) ---
}

/**
 * @brief Time-triggered task for toggling the LED.
 *
 * This function is called periodically by the scheduler. It reads the current
 * state of the LED pin and toggles it to the opposite state.
 * It follows the "LED Toggle Task (Time Triggered)" flowchart.
 */
void LED_Toggle_Task(void)
{
    // --- Start ---

    // Toggles the LED pin. For an active-low LED, this will turn it ON if OFF,
    // and OFF if ON, which matches the flowchart logic:
    // If ON (pin is LOW), toggle makes it HIGH (OFF).
    // If OFF (pin is HIGH), toggle makes it LOW (ON).
    GPIO_Value_Tog(LEDTOGGEL_LED_PORT, LEDTOGGEL_LED_PIN);

    // --- End ---
}

//==================================================================================================
// End of File
//==================================================================================================