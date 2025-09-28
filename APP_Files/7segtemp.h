#ifndef SEGTEMP_H
#define SEGTEMP_H

#include "MCAL.h"
#include "7segtemp_user.h"

/**
 * @file 7segtemp.h
 * @brief Header file for the 7-segment temperature display middleware.
 *
 * This file contains the function prototypes, macros, and global structures
 * for the 7segtemp application, integrating with the underlying HAL and MCAL.
 */

/* Exported function prototypes */

/**
 * @brief Initializes the 7segtemp application middleware.
 *
 * This function performs all necessary initializations for the 7segtemp
 * application, including clock setup, display driver initialization,
 * and time-triggered scheduler setup.
 */
void App_Init(void);

/**
 * @brief Main application logic task for 7segtemp.
 *
 * This task is responsible for checking touch pad inputs (PLUS/MINUS),
 * updating the internal temperature value, and refreshing the 7-segment display.
 */
void APP_MainTask(void);

#endif // SEGTEMP_H
