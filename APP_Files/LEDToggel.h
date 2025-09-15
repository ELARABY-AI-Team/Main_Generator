#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains the function prototypes, macros, and global structures
 * for the LEDToggel middleware, enabling LED blinking functionality.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h"           // Microcontroller Abstraction Layer definitions
#include "LEDToggel_user.h" // User configuration for LEDToggel

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LEDToggel middleware.
 *
 * This function performs necessary initializations for the LED blinking
 * application, including configuring the LED GPIO pin as an output
 * and setting its initial state. It follows the APP_Init Process flowchart.
 */
void LEDToggel_Init(void);

/**
 * @brief Task function for toggling the LED state.
 *
 * This function implements the core LED toggling logic as described in the
 * Leg Toggle Process flowchart. It checks the current LED state and
 * switches it to the opposite state. This function is intended to be
 * called periodically by a scheduler.
 */
void LEDToggel_ToggleTask(void);

#endif // LEDTOGGEL_H
