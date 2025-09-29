#ifndef SEVENSEGMENT_USER_H
#define SEVENSEGMENT_USER_H

/**
 * @file sevensegment_user.h
 * @brief User configuration header for the Seven Segment middleware application.
 *
 * This file contains user-editable constants, feature flags, and parameters
 * that can be adjusted without modifying the core middleware code.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "Touch_Pad_user.h" // For Touch Pad IDs
#include "Interval_Timer.h" // To get SCHEDULER_TICK_MS from t_Tick_Time enum

//==================================================================================================
// User Configuration Macros
//==================================================================================================

/**
 * @brief Defines the microcontroller's system voltage for MCAL configuration.
 *        Choose between `sys_volt_3v` or `sys_volt_5v`.
 */
#define SEVENSEGMENT_SYSTEM_VOLTAGE     sys_volt_5v

/**
 * @brief Defines the initial value to be displayed on the Seven Segment Display.
 */
#define SEVENSEGMENT_INITIAL_VALUE      (25)

/**
 * @brief Defines the minimum allowed value for the Seven Segment Display.
 */
#define SEVENSEGMENT_MIN_VALUE          (0)

/**
 * @brief Defines the maximum allowed value for the Seven Segment Display.
 */
#define SEVENSEGMENT_MAX_VALUE          (99) // Assuming a 2-digit display based on SSD_MAX=2

/**
 * @brief Defines the Touch Pad ID for incrementing the displayed value.
 *        Refer to `Touch_Pad_user.h` for available IDs.
 */
#define SEVENSEGMENT_PLUS_TOUCH_PAD_ID  NORMAL_TOUCH

/**
 * @brief Defines the Touch Pad ID for decrementing the displayed value.
 *        Refer to `Touch_Pad_user.h` for available IDs.
 */
#define SEVENSEGMENT_MINUS_TOUCH_PAD_ID COLD_TOUCH

/**
 * @brief Defines the scheduler tick time in milliseconds.
 *        This value is derived from `Interval_Timer.h` to ensure consistency.
 *        `Tick_1_ms` from `t_Tick_Time` enum corresponds to 1 millisecond.
 */
#define SCHEDULER_TICK_MS               ((uint16_t)Tick_1_ms)

/**
 * @brief Period for the `APP_MainTask` in scheduler ticks (milliseconds).
 */
#define APP_MAIN_TASK_PERIOD_MS         (50) // Run every 50ms

/**
 * @brief Delay for the `APP_MainTask` in scheduler ticks (milliseconds).
 */
#define APP_MAIN_TASK_DELAY_MS          (5)  // Start after 5ms

#endif // SEVENSEGMENT_USER_H
