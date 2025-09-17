#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the LEDToggel application, which controls an LED based on a time-triggered schedule.
 */

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs all necessary initializations for the LEDToggel
 * middleware, including MCU configuration, GPIO setup for the LED,
 * and setting the initial LED state. It follows the APP_Init Process flowchart.
 */
void App_Init(void);

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function implements the "Leg Toggle Process" from the flowchart.
 * It reads the current state of the LED and toggles it (ON to OFF, OFF to ON).
 * This task is intended to be scheduled by the Time-Triggered Scheduler.
 */
void LedToggel_RunTask(void);

#endif // LEDTOGGEL_H
