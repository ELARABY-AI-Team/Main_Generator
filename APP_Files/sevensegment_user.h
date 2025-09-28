#ifndef SEVENSEGMENT_USER_H
#define SEVENSEGMENT_USER_H

/**
 * @file sevensegment_user.h
 * @brief User configuration header for the sevensegment middleware application.
 *
 * This file contains user-editable constants, feature flags, and parameters
 * that can be adjusted without modifying the core middleware logic.
 */

//==============================================================================
// Includes
//==============================================================================
#include "MCAL.h"
#include "Interval_Timer.h" // Required to get t_Tick_Time typedef

//==============================================================================
// Configuration Defines
//==============================================================================

/**
 * @brief Defines the scheduler tick time in milliseconds.
 *
 * This value must be derived from `t_Tick_Time` enum in `Interval_Timer.h`.
 * For 1ms tick, `Tick_1_ms` is the appropriate enum value.
 */
#define SCHEDULER_TICK_TIME_MS Tick_1_ms
#define SCHEDULER_TICK_MS_VALUE 1U // Numerical value in milliseconds for calculations

/**
 * @brief Default initial value for the sevensegment display.
 */
#define SEVENSEGMENT_DEFAULT_VALUE 0

/**
 * @brief Maximum displayable value on the sevensegment display.
 *
 * This depends on the number of SSD digits configured in HAL_SSD_Config.h (SSD_MAX).
 * If SSD_MAX is 2, the max value is 99.
 */
#define SEVENSEGMENT_MAX_VALUE 99

/**
 * @brief Minimum displayable value on the sevensegment display.
 */
#define SEVENSEGMENT_MIN_VALUE 0

/**
 * @brief Touch pad ID for the "PLUS" button.
 *
 * This maps to an entry in the `touch_config` array in `Touch_Pad_user.c`.
 * Using NORMAL_TOUCH for PLUS based on `Touch_Pad_user.h` enumeration.
 */
#define PLUS_TOUCH_ID NORMAL_TOUCH

/**
 * @brief Touch pad ID for the "MINUS" button.
 *
 * This maps to an entry in the `touch_config` array in `Touch_Pad_user.c`.
 * Using COLD_TOUCH for MINUS based on `Touch_Pad_user.h` enumeration.
 */
#define MINUS_TOUCH_ID COLD_TOUCH

/**
 * @brief Period for the `APP_MainTask` in milliseconds.
 *
 * This determines how often the main application logic (reading buttons,
 * updating display value) is executed by the scheduler.
 */
#define APP_MAINTASK_PERIOD_MS 20U

/**
 * @brief Delay before the first execution of `APP_MainTask` in milliseconds.
 */
#define APP_MAINTASK_DELAY_MS 0U

//==============================================================================
// Derived Constants (Do Not Modify)
//==============================================================================

/**
 * @brief Converts `APP_MAINTASK_PERIOD_MS` to scheduler ticks.
 */
#define PERIOD_APP_MAINTASK_TICKS (APP_MAINTASK_PERIOD_MS / SCHEDULER_TICK_MS_VALUE)

/**
 * @brief Converts `APP_MAINTASK_DELAY_MS` to scheduler ticks.
 */
#define DELAY_APP_MAINTASK_TICKS (APP_MAINTASK_DELAY_MS / SCHEDULER_TICK_MS_VALUE)

#endif // SEVENSEGMENT_USER_H
