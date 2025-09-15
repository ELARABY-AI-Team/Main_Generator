#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains the function prototypes and global definitions
 * for the LEDToggel application, which controls an LED.
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "LEDToggel_user.h" /* For user configurable parameters */

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs the necessary MCU configurations,
 * initializes the GPIO for the LED, and sets the initial LED state.
 * It follows the APP_Init Process flowchart.
 */
void App_Init(void);

/**
 * @brief Toggles the state of the LED.
 *
 * This function reads the current state of the LED and
 * switches it to the opposite state (ON to OFF, or OFF to ON).
 * It follows the Leg Toggle Process flowchart.
 */
void App_ToggleLed(void);

#endif /* LEDTOGGEL_H */
