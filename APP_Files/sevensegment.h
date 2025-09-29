/**
 * @file sevensegment.h
 * @brief Header file for the sevensegment middleware application.
 *
 * This file defines function prototypes, macros, and global structures
 * for the sevensegment application.
 */

#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H

//=============================================================================
// Includes
//=============================================================================
#include "MCAL.h"                // Microcontroller Abstraction Layer API
#include "sevensegment_user.h"   // User configurable parameters
#include "HAL_SSD.h"             // Seven Segment Display HAL
#include "Touch_Pad.h"           // Touch Pad HAL

//=============================================================================
// Function Prototypes
//=============================================================================

/**
 * @brief Initializes the sevensegment application middleware.
 *
 * This function performs all necessary hardware and software initializations
 * for the application, including MCAL, HAL components, and the time-triggered
 * scheduler's timer.
 */
void App_Init(void);

/**
 * @brief The main application logic task for sevensegment display control.
 *
 * This task checks for touch pad inputs (PLUS/MINUS) to adjust the displayed
 * value on the seven-segment display and updates it if there is a change.
 * This function runs periodically via the time-triggered scheduler.
 */
void App_MainTask(void);

#endif // SEVENSEGMENT_H