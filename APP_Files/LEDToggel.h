#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

//==================================================================================================
// Includes
//==================================================================================================
#include "LEDToggel_user.h" // Include user configurations

/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file declares the public interface for the LEDToggel application,
 * including initialization and the main task function.
 */

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LEDToggel middleware application.
 *
 * This function performs necessary MCU configurations, sets up the LED GPIO,
 * and initializes the LED to its starting state as per the application logic.
 */
void LEDToggel_Init(void);

/**
 * @brief Time-triggered task for toggling the LED.
 *
 * This function checks the current state of the LED and toggles it
 * (from ON to OFF, or OFF to ON). Designed to be run by a scheduler.
 */
void LEDToggel_Task(void);

#endif // LEDTOGGEL_H