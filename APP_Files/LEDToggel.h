/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the LEDToggel application, facilitating clean and modular code.
 */

#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

/*==============================================================================================*/
/* Includes */
/*==============================================================================================*/
#include "LEDToggel_user.h" // User configuration for LEDToggel
#include "MCAL.h"           // For general types like tbyte

/*==============================================================================================*/
/* Macros */
/*==============================================================================================*/

/**
 * @brief Defines the logical ON state for the LED based on its active level.
 *        If LED_ACTIVE_LEVEL is 0 (active low), LED_ON is 0.
 *        If LED_ACTIVE_LEVEL is 1 (active high), LED_ON is 1.
 */
#define LED_ON  LED_ACTIVE_LEVEL

/**
 * @brief Defines the logical OFF state for the LED based on its active level.
 *        If LED_ACTIVE_LEVEL is 0 (active low), LED_OFF is 1.
 *        If LED_ACTIVE_LEVEL is 1 (active high), LED_OFF is 0.
 */
#define LED_OFF (1 - LED_ACTIVE_LEVEL)

/*==============================================================================================*/
/* Function Prototypes */
/*==============================================================================================*/

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs the necessary setup for the LED toggling application,
 * including MCU general configuration and GPIO initialization for the LED.
 */
void LEDToggel_Init(void);

/**
 * @brief Time-triggered task to toggle the LED.
 *
 * This function is designed to be called periodically by a scheduler to
 * change the state of the LED.
 */
void LEDToggel_Task(void);

/**
 * @brief Sets the state of the LED (ON or OFF).
 * @param state The desired logical state (LED_ON or LED_OFF).
 */
void LEDToggel_SetState(tbyte state);

/**
 * @brief Gets the current logical state of the LED.
 * @return The current logical state (LED_ON or LED_OFF).
 */
tbyte LEDToggel_GetState(void);

#endif /* LEDTOGGEL_H */
