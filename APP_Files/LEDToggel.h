#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains function prototypes, macros, and global data structures
 * for the LEDToggel application.
 */

/*==============================================================================================
 * Includes
 *============================================================================================*/
#include "MCAL.h"           // For MCAL data types (t_port, t_pin, tbyte) and potentially other MCAL definitions
#include "LEDToggel_user.h" // For user configurations specific to LEDToggel

/*==============================================================================================
 * Function Prototypes
 *============================================================================================*/

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs necessary MCU configuration, initializes the LED GPIO,
 * and sets the initial state of the LED as per the 'APP_Init Process' flowchart.
 */
void App_Init(void);

/**
 * @brief Toggles the state of the LED.
 *
 * This function reads the current state of the LED and inverts it,
 * as per the 'Leg Toggle Process' flowchart. This function is intended
 * to be called periodically by a scheduler to achieve a blinking effect.
 */
void App_Toggle_LED(void);

#endif /* LEDTOGGEL_H */
