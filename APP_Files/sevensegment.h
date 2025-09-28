#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H

/**
 * @file sevensegment.h
 * @brief Header file for the sevensegment middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the sevensegment display application, integrating with MCAL, HAL,
 * and the time-triggered scheduler.
 */

//==============================================================================
// Includes
//==============================================================================
#include "MCAL.h"                 // Microcontroller Abstraction Layer
#include "HAL_SSD.h"              // Seven Segment Display HAL
#include "Touch_Pad.h"            // Touch Pad HAL
#include "tt_scheduler.h"         // Time-Triggered Scheduler
#include "sevensegment_user.h"    // User configurable parameters

//==============================================================================
// Public Function Prototypes
//==============================================================================

/**
 * @brief Initializes the sevensegment application.
 *
 * This function performs all necessary initializations for the sevensegment
 * application, including MCAL, HAL components, and setting up initial display values.
 */
void App_Init(void);

/**
 * @brief Main application logic task for the sevensegment display.
 *
 * This task handles reading touch pad inputs (PLUS/MINUS), updating the
 * internal sevensegment value, and refreshing the display if the value changes.
 * This function is designed to be called periodically by the time-triggered scheduler.
 */
void APP_MainTask(void);

#endif // SEVENSEGMENT_H
