/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file defines the API for the LEDToggel application, including
 * initialization and the time-triggered task.
 */

#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h"            // Provides MCAL types (tbyte, t_port, t_pin) and GPIO APIs
#include "LEDToggel_user.h"  // User-configurable parameters for LEDToggel

//==================================================================================================
// Macros
//==================================================================================================

/**
 * @brief Defines the ON state for the LED.
 *        Based on "Active Low0" from the flowchart, a logical 0 means LED is ON.
 */
#define LED_ON_VALUE    ((tbyte)0)

/**
 * @brief Defines the OFF state for the LED.
 *        Based on "Active Low0" from the flowchart, a logical 1 means LED is OFF.
 */
#define LED_OFF_VALUE   ((tbyte)1)

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs necessary hardware initialization for the LED and
 * registers the LED toggling task with the time-triggered scheduler.
 */
void App_Init(void);

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This function is designed to be called periodically by the scheduler
 * to invert the current state of the LED.
 */
void LEDToggel_Task(void);

/**
 * @brief Sets the state of the LED.
 * @param state The desired state of the LED (use `LED_ON_VALUE` or `LED_OFF_VALUE`).
 */
void LED_Set_State(tbyte state);

/**
 * @brief Gets the current state of the LED.
 * @return The current state of the LED (`LED_ON_VALUE` or `LED_OFF_VALUE`).
 */
tbyte LED_Get_State(void);

#endif // LEDTOGGEL_H