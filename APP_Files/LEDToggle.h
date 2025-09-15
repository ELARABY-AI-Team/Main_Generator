#ifndef LEDTOGGLE_H
#define LEDTOGGLE_H

#include "LEDToggle_user.h" // For user-defined configurations

/**
 * @file LEDToggle.h
 * @brief Header file for the LEDToggle middleware application.
 *
 * This file contains function prototypes and definitions for the LEDToggle
 * application, which initializes a GPIO pin as an LED and periodically toggles its state.
 */

/*==============================================================================================
*                                    Function Prototypes
==============================================================================================*/

/**
 * @brief Initializes the LEDToggle application.
 *
 * This function initializes the LED GPIO pin, sets its initial state,
 * and configures the Time-Triggered scheduler to run the LED toggle task.
 */
void LEDToggle_Init(void);

/**
 * @brief Starts the LEDToggle application.
 *
 * This function initiates the Time-Triggered scheduler, allowing the LED
 * toggle task to begin execution.
 */
void LEDToggle_Start(void);

/**
 * @brief The periodic task to toggle the LED state.
 *
 * This function is intended to be called by the scheduler. It reads the
 * current state of the LED and toggles it (ON to OFF, or OFF to ON).
 */
void LEDToggle_ToggleTask(void);

#endif /* LEDTOGGLE_H */
