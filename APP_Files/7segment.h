#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file 7segment.h
 * @brief Header file for the 7segment application middleware.
 *
 * This file declares function prototypes, macros, and global structures
 * for the 7segment application layer.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "MCAL.h"
#include "HAL_SSD.h"        // For 7-segment display HAL
#include "Touch_Pad.h"      // For touch pad HAL
#include "tt_scheduler.h"   // For scheduler integration
#include "7segment_user.h"  // For application-specific user configurations

//==================================================================================================
// Global Variables
//==================================================================================================

/**
 * @brief Stores the current value displayed on the 7-segment display.
 */
extern tword SSD_Value;

/**
 * @brief Stores a temporary value for calculation before updating SSD_Value.
 */
extern tword TempValue;

//==================================================================================================
// Function Prototypes
//==================================================================================================

/**
 * @brief Initializes the 7segment application middleware.
 *
 * This function performs all necessary initializations for the 7segment
 * application, including hardware abstraction layers (HAL) and the
 * time-triggered scheduler.
 */
void App_Init(void);

/**
 * @brief Main task logic for the 7segment application.
 *
 * This function implements the core behavior of the 7segment application,
 * checking touch pad inputs, updating the display value, and refreshing
 * the 7-segment display. This task is scheduled by the time-triggered scheduler.
 */
void App_MainTask(void);

#ifdef __cplusplus
}
#endif

#endif // SEVENSEGMENT_H
