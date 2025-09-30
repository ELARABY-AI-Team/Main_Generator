#ifndef SEVENSEGMENT_USER_H
#define SEVENSEGMENT_USER_H

/**
 * @file 7segment_user.h
 * @brief User configuration for the 7segment application middleware.
 *
 * This file contains high-level user-editable constants and feature flags.
 * No hardware-specific code should be present here.
 */

//==================================================================================================
// Configuration Defines
//==================================================================================================

/**
 * @brief The initial value to display on the 7-segment display at startup.
 */
#define APP_INITIAL_SSD_VALUE       (0)

/**
 * @brief The maximum value that can be displayed on the 7-segment display.
 * @note This should be aligned with the number of SSD digits configured in HAL_SSD_Config.h (SSD_MAX).
 *       If SSD_MAX is 2, then max value is 99.
 */
#define APP_SSD_MAX_DISPLAY_VALUE   (99)

/**
 * @brief The minimum value that can be displayed on the 7-segment display.
 */
#define APP_SSD_MIN_DISPLAY_VALUE   (0)

/**
 * @brief The ID of the touch pad used for incrementing the display value.
 * @note Mapped to specific IDs from Touch_Pad_user.h.
 */
#define APP_TOUCH_PLUS_ID           (NORMAL_TOUCH) // Assuming NORMAL_TOUCH acts as '+'

/**
 * @brief The ID of the touch pad used for decrementing the display value.
 * @note Mapped to specific IDs from Touch_Pad_user.h.
 */
#define APP_TOUCH_MINUS_ID          (COLD_TOUCH)   // Assuming COLD_TOUCH acts as '-'

/**
 * @brief The period in milliseconds for the main application logic task (APP_MainTask).
 */
#define APP_MAIN_TASK_PERIOD_MS     (20)

/**
 * @brief The desired tick time for the scheduler in milliseconds.
 * @note This value should correspond to one of the t_Tick_Time enum values in Interval_Timer.h.
 *       Here, Tick_1_ms is used, so the value is 1.
 */
#define SCHEDULER_TICK_VALUE_MS     (1)

#endif // SEVENSEGMENT_USER_H
