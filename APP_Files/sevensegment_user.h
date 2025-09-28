#ifndef SEVENSEGMENT_USER_H
#define SEVENSEGMENT_USER_H

/**
 * @file sevensegment_user.h
 * @brief User configuration header for the sevensegment application.
 *
 * This file contains user-editable constants, feature flags, and
 * parameters that can be adjusted without modifying the core middleware code.
 * It provides a central place for application-specific configurations.
 */

//==================================================================================================
// Required Includes for Type Definitions
//==================================================================================================
#include "MCAL.h"           // For MCAL types like tbyte if directly used by definitions
#include "Touch_Pad_user.h" // For touch_pad_id_t enum to map touch pads

//==================================================================================================
// Scheduler Configuration
//==================================================================================================
/**
 * @brief Defines the scheduler tick time in milliseconds.
 *        This value must correspond to one of the t_Tick_Time enum values
 *        defined in Interval_Timer.h. For example, '1' corresponds to 'Tick_1_ms'.
 *        This is used to configure the underlying hardware timer.
 */
#define SCHEDULER_TICK_MS                       1

//==================================================================================================
// Task Periods (in milliseconds)
//==================================================================================================
/**
 * @brief Period for the Touch Pad update task in milliseconds.
 *        This value is derived directly from the flowchart requirement (18ms).
 */
#define SEVENSEGMENT_TOUCHPAD_UPDATE_PERIOD_MS  18

/**
 * @brief Period for the Seven Segment Display update task in milliseconds.
 *        This value is derived directly from the flowchart requirement (5ms).
 */
#define SEVENSEGMENT_SSD_UPDATE_PERIOD_MS       5

/**
 * @brief Period for the main application logic task in milliseconds.
 *        This period is chosen to be a reasonable interval for the main logic,
 *        and often a multiple of other task periods for better synchronization.
 */
#define SEVENSEGMENT_MAIN_TASK_PERIOD_MS        50

//==================================================================================================
// Seven Segment Display Configuration
//==================================================================================================
/**
 * @brief Maximum value that can be displayed on the seven segment.
 *        Assumes a 2-digit display, as indicated by SSD_MAX=2 in HAL_SSD_Config.h,
 *        allowing values from 0 to 99.
 */
#define SEVENSEGMENT_MAX_VALUE                  99

/**
 * @brief Minimum value that can be displayed on the seven segment.
 */
#define SEVENSEGMENT_MIN_VALUE                  0

//==================================================================================================
// Touch Pad Mapping
//==================================================================================================
/**
 * @brief Defines the Touch Pad ID used for incrementing the display value.
 *        This maps to an enum value from `touch_pad_id_t` defined in `Touch_Pad_user.h`.
 *        Example: `NORMAL_TOUCH` for a general 'plus' button.
 */
#define SEVENSEGMENT_PLUS_TOUCH_ID              NORMAL_TOUCH

/**
 * @brief Defines the Touch Pad ID used for decrementing the display value.
 *        This maps to an enum value from `touch_pad_id_t` defined in `Touch_Pad_user.h`.
 *        Example: `COLD_TOUCH` for a general 'minus' button.
 */
#define SEVENSEGMENT_MINUS_TOUCH_ID             COLD_TOUCH

#endif // SEVENSEGMENT_USER_H
