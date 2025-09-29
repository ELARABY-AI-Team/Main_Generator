#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H

/**
 * @file sevensegment.h
 * @brief Header file for the Seven Segment middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the seven segment display control application.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include <stdint.h>
#include "sevensegment_user.h" // User configuration for the application
#include "MCAL.h"              // Microcontroller Abstraction Layer
#include "HAL_SSD.h"           // Seven Segment Display HAL
#include "Touch_Pad.h"         // Touch Pad HAL
#include "tt_scheduler.h"      // Time-Triggered Scheduler

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the Seven Segment application middleware.
 *
 * This function performs all necessary initializations for the application,
 * including MCU clock, SSD, and touch pads.
 */
void App_Init(void);

/**
 * @brief Main logic task for the Seven Segment application.
 *
 * This task continuously checks for touchpad inputs to increment or decrement
 * the displayed value on the seven segment display. It updates the display
 * only if the value has changed.
 */
void APP_MainTask(void);

#endif // SEVENSEGMENT_H
