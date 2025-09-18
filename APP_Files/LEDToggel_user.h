/**
 * @file LEDToggel_user.h
 * @brief User configuration header for the LEDToggel middleware application.
 *
 * This file allows end-users to customize the behavior of the LEDToggel
 * application without modifying the core middleware code.
 *
 * ⚠ IMPORTANT: Remove anything related to hardware like pin configuration
 *              or hardware parameter values. Those belong to HAL layer or
 *              directly in the .c file if no specific HAL exists.
 */

#ifndef LEDTOGGEL_USER_H
#define LEDTOGGEL_USER_H

/*==================================================================================================
*                                        MACROS AND DEFINES
==================================================================================================*/

/**
 * @brief Defines the period for the LED toggling task in milliseconds.
 *        The LED will change its state every this many milliseconds.
 *        (e.g., 500 for 500ms, resulting in 1Hz total toggle frequency).
 *        Flowchart specifies 500ms.
 */
#define LEDTOGGLE_PERIOD_MS       (500)

/**
 * @brief Configures the initial state of the LED at application startup.
 *        Set to 1 for the LED to be ON initially, 0 for OFF.
 */
#define LEDTOGGLE_INITIAL_STATE_ON (0) /* 0: Start OFF, 1: Start ON */

/**
 * @brief Configures whether the LED is active low or active high.
 *        Set to 1 if the LED turns ON when the pin is LOW.
 *        Set to 0 if the LED turns ON when the pin is HIGH.
 *        Flowchart specifies "Active Low0", implying 0 for ON.
 */
#define LEDTOGGLE_ACTIVE_LOW      (1) /* 1: Active Low, 0: Active High */

/*==================================================================================================
*                                        END OF FILE
==================================================================================================*/

#endif /* LEDTOGGEL_USER_H */
