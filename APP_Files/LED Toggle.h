#ifndef LED_TOGGLE_H
#define LED_TOGGLE_H

/**
 * @file LED Toggle.h
 * @brief Header file for the LED Toggle middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the LED Toggle middleware application, adhering to clean and
 * modular coding guidelines.
 */

//==================================================================================================
// Includes
//==================================================================================================
// No direct MCAL or user includes here to keep the interface clean.
// They are typically included in the corresponding .c file.

//==================================================================================================
// Public Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LED Toggle application.
 *
 * This function sets up the necessary hardware (e.g., LED GPIO pin)
 * and prepares the application for operation.
 */
void LED_Toggle_Init(void);

/**
 * @brief Starts the LED Toggle application.
 *
 * This function registers the LED toggling task with the system scheduler,
 * enabling periodic execution of the LED logic.
 */
void LED_Toggle_Start(void);

/**
 * @brief The periodic task function to toggle the LED.
 *
 * This function contains the core logic for checking the LED's state
 * and switching it on or off. It is designed to be called by the scheduler
 * at regular intervals.
 */
void LED_Toggle_Task(void);

#endif // LED_TOGGLE_H
