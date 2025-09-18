/**
 * @file LEDToggel.h
 * @brief Header file for the LEDToggel middleware application.
 *
 * This file contains function prototypes, macros, and global structures
 * for the LEDToggel application.
 */

#ifndef LEDTOGGEL_H
#define LEDTOGGEL_H

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "LEDToggel_user.h" /* User-editable configuration */
#include "MCAL.h"           /* For general MCAL data types like tbyte, tword */

/*==================================================================================================
*                                        MACROS AND DEFINES
==================================================================================================*/
/* No specific macros or structures defined here as they are either in user.h or local to .c file */

/*==================================================================================================
*                                        FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief Initializes the LEDToggel application middleware.
 *
 * This function performs all necessary startup configurations for the LED
 * toggling functionality, including MCU setup, LED GPIO initialization,
 * and registering the LED toggling task with the scheduler.
 */
void App_Init(void);

#endif /* LEDTOGGEL_H */
