#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H

/**
 * @file sevensegment.h
 * @brief Header file for the Seven Segment Display middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the sevensegment application middleware. It includes necessary MCAL
 * and HAL headers for the microcontroller and peripherals used.
 */

//==================================================================================================
// Standard & MCAL Includes
//==================================================================================================
#include "MCAL.h" // Microcontroller Abstraction Layer definitions (e.g., tbyte)

//==================================================================================================
// External HAL & Scheduler Includes
//==================================================================================================
// Assuming HAL headers are located relative to the project's include paths
#include "HAL_SSD.h"      // Seven Segment Display Hardware Abstraction Layer
#include "Touch_Pad.h"    // Touch Pad Hardware Abstraction Layer
#include "tt_scheduler.h" // Time-Triggered Scheduler API

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the sevensegment application middleware.
 *
 * This function performs all necessary initializations for the sevensegment
 * application, including MCU configuration, peripheral initializations (SSD, Touch Pad),
 * and the time-triggered scheduler setup. It should be called once at system startup.
 */
void App_Init(void);

/**
 * @brief Main application logic task for the sevensegment display.
 *
 * This task checks for touch pad inputs (PLUS/MINUS), updates a temporary
 * value, and if different from the current display value, updates the
 * seven segment display accordingly. This function is intended to be
 * scheduled periodically by the time-triggered scheduler.
 */
void App_MainTask(void);

/**
 * @brief Task wrapper for updating the Seven Segment Display.
 *
 * This function calls the underlying HAL_SSD_Update to refresh the display
 * multiplexing. It is intended to be scheduled by the time-triggered scheduler
 * to ensure continuous display update.
 */
void SSD_Update_Task(void);

/**
 * @brief Task wrapper for updating the Touch Pad states.
 *
 * This function calls the underlying HAL_TOUCH_PAD_update to poll touch pad inputs
 * and update their internal states. It is intended to be scheduled by the
 * time-triggered scheduler for periodic input scanning.
 */
void TOUCH_PAD_Update_Task(void);

#endif // SEVENSEGMENT_H
