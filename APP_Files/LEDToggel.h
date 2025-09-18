/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file defines the public interface, data types, and macros for the
 * LEDToggel middleware, intended for use on the RENESAS_R5F11BBC microcontroller.
 */

#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel_user.h" // User configurable parameters
#include "MCAL.h"           // For MCAL data types like tbyte, tport, tpin

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application middleware.
 *
 * This function performs the necessary MCU configurations and initializes the
 * LED GPIO pin as an output, setting its initial state as specified.
 */
void App_Init(void);

/**
 * @brief Time-triggered task for toggling the LED.
 *
 * This function is designed to be called periodically by a scheduler to
 * toggle the state of the LED.
 */
void LED_Toggle_Task(void);

#endif // LEDTOGGEL_H
//==================================================================================================
// End of File
//==================================================================================================