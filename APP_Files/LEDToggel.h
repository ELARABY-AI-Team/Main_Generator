#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the LEDToggel middleware, conforming to the APP_API.json specification.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h"              // Include MCAL for basic types and GPIO operations
#include "LEDToggel_user.h"    // User-configurable parameters for LEDToggel

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs the initial setup as described in the APP_Init Process
 * flowchart, including MCU configuration, GPIO initialization for the LED,
 * and setting the LED to an initial ON state.
 *
 * @param None
 * @return None
 */
void APP_Init(void);

/**
 * @brief Turns the configured LED ON.
 *
 * This function sets the GPIO pin associated with the LED to a high state,
 * illuminating the LED.
 *
 * @param None
 * @return None
 */
void led_on(void);

/**
 * @brief Turns the configured LED OFF.
 *
 * This function sets the GPIO pin associated with the LED to a low state,
 * turning off the LED.
 *
 * @param None
 * @return None
 */
void led_off(void);

/**
 * @brief Toggles the state of the configured LED.
 *
 * This function implements the "Leg Toggle Process" flowchart. It reads the
 * current state of the LED and switches it to the opposite state (ON to OFF,
 * or OFF to ON).
 *
 * @param None
 * @return None
 */
void LEDTOGGEL_ToggleLed(void);

#endif // LEDTOGGEL_H
