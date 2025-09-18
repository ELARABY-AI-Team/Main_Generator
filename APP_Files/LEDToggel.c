/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the initialization logic, middleware components,
 * and application task definitions for toggling an LED based on a
 * time-triggered schedule.
 */

/*==============================================================================================*/
/* Includes */
/*==============================================================================================*/
#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h" // For GPIO and MCU configuration
#include "tt_scheduler.h" // For adding tasks to the scheduler

/*==============================================================================================*/
/* Global Variables */
/*==============================================================================================*/

// No global variables needed for this simple LED toggling logic.

/*==============================================================================================*/
/* Function Definitions */
/*==============================================================================================*/

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs the necessary setup for the LED toggling application,
 * including MCU general configuration, GPIO initialization for the LED,
 * and setting the initial state of the LED.
 *
 * It follows the "APP_Init Process" flowchart.
 */
void LEDToggel_Init(void)
{
    // 1. MCU_Config
    // Initialize the MCU with the defined system voltage.
    MCU_Config_Init(MCU_SYSTEM_VOLTAGE);

    // 2. GPIO_Init (Active Low0) Port 7, Pin 4
    // Configure the LED pin as an output with the specified initial state.
    // The LED is active low, so 0=ON, 1=OFF.
    GPIO_Output_Init(LED_GPIO_PORT, LED_GPIO_PIN, LED_ON);

    // 3. LED_SetOn (0=ON, 1=OFF)
    // The previous step already sets the initial state to ON.
    // Explicitly setting it again to adhere to flowchart's sequence if initial value in GPIO_Output_Init
    // was not guaranteed to be LED_ON.
    GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, LED_ON);
}

/**
 * @brief Time-triggered task to toggle the LED.
 *
 * This function is executed periodically by the scheduler. It reads the
 * current state of the LED and toggles it.
 *
 * It follows the "LED Toggle Task" flowchart.
 */
void LEDToggel_Task(void)
{
    // Read current LED state
    // tbyte current_state = GPIO_Value_Get(LED_GPIO_PORT, LED_GPIO_PIN);

    // Toggle the LED based on its current state.
    // MCAL provides a direct toggle function, which is more efficient.
    GPIO_Value_Tog(LED_GPIO_PORT, LED_GPIO_PIN);

    // Alternative explicit logic as per flowchart:
    /*
    if (current_state == LED_ON)
    {
        // LED is currently ON, set it OFF
        GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, LED_OFF);
    }
    else // current_state == LED_OFF
    {
        // LED is currently OFF, set it ON
        GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, LED_ON);
    }
    */
}

/**
 * @brief Sets the state of the LED.
 * @param state The desired state (LED_ON or LED_OFF).
 */
void LEDToggel_SetState(tbyte state)
{
    GPIO_Value_Set(LED_GPIO_PORT, LED_GPIO_PIN, state);
}

/**
 * @brief Gets the current state of the LED.
 * @return The current state (LED_ON or LED_OFF).
 */
tbyte LEDToggel_GetState(void)
{
    return GPIO_Value_Get(LED_GPIO_PORT, LED_GPIO_PIN);
}
