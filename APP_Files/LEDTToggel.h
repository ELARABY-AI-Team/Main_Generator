#ifndef LED_TTOGGEL_H
#define LED_TTOGGEL_H

/**
 * @file LEDTToggel.h
 * @brief Header file for the LEDTToggel middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the LEDTToggel middleware, which controls an LED using a time-triggered scheduler.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h"
#include "LEDTToggel_user.h"
#include "tt_scheduler.h"

//==================================================================================================
// Defines & Macros
//==================================================================================================

/**
 * @brief Defines the logical ON state for the LED based on its active level.
 *        If LED_ACTIVE_LOW is 1, ON is 0. If 0, ON is 1.
 */
#if (LED_ACTIVE_LOW == 1)
    #define LED_ON_STATE  ((tbyte)0)
#else
    #define LED_ON_STATE  ((tbyte)1)
#endif

/**
 * @brief Defines the logical OFF state for the LED based on its active level.
 *        If LED_ACTIVE_LOW is 1, OFF is 1. If 0, OFF is 0.
 */
#if (LED_ACTIVE_LOW == 1)
    #define LED_OFF_STATE ((tbyte)1)
#else
    #define LED_OFF_STATE ((tbyte)0)
#endif

// Calculate the number of scheduler ticks for the LED toggle period
#define LED_TOGGLE_PERIOD_TICKS (LED_TOGGLE_PERIOD_MS / SCHEDULER_TICK_TIME_MS)

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the LEDTToggel application middleware.
 *
 * This function performs necessary hardware initialization for the LED and
 * prepares the application for operation.
 */
void App_Init(void);

/**
 * @brief Time-triggered task to toggle the LED state.
 *
 * This task is executed periodically by the scheduler to change the LED's
 * ON/OFF state.
 */
void LED_Toggle_Task(void);

/**
 * @brief Sets the LED to the ON state.
 *
 * This function controls the GPIO pin to turn the LED ON based on
 * the configured active level.
 */
void LED_SetOn(void);

/**
 * @brief Sets the LED to the OFF state.
 *
 * This function controls the GPIO pin to turn the LED OFF based on
 * the configured active level.
 */
void LED_SetOff(void);

#endif // LED_TTOGGEL_H
