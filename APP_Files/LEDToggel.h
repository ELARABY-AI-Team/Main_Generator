#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the LEDToggel application, which is responsible for toggling an LED.
 */

/*==============================================================================================
 * Includes
 *=============================================================================================*/
#include "LEDToggel_user.h" /* User-configurable parameters for LEDToggel */
#include "MCAL.h"           /* Microcontroller Abstraction Layer definitions (e.g., t_port, t_pin, tbyte) */

/*==============================================================================================
 * Function Prototypes
 *=============================================================================================*/

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs necessary hardware initialization for the LED,
 * such as configuring its GPIO pin, and sets its initial state.
 * It also prepares the LED for scheduling.
 */
void App_Init(void);

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function is designed to be called periodically by the time-triggered scheduler.
 * It reads the current state of the LED and switches it to the opposite state.
 */
void LED_Toggle_Task(void);

#endif /* LEDTOGGEL_H */
