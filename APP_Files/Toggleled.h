#ifndef TOGGLELED_H
#define TOGGLELED_H

#include "Toggleled_user.h"
#include "MCAL.h" // For tbyte, t_port, t_pin types

/**
 * @file Toggleled.h
 * @brief Header file for the Toggleled middleware application.
 *
 * This file contains function prototypes, macros, and global
 * structure definitions for the Toggleled application, abstracting
 * the core logic from hardware specifics.
 */

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the Toggleled application and its peripherals.
 *
 * This function performs necessary hardware and software initializations
 * required for the Toggleled application to operate correctly.
 */
void App_Init(void);

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function checks the current state of the LED and toggles it
 * between ON and OFF. It is designed to be called periodically by the scheduler.
 */
void LED_Toggle_Task(void);

#endif // TOGGLELED_H
