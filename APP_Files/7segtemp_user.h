#ifndef SEGTEMP_USER_H
#define SEGTEMP_USER_H

/**
 * @file 7segtemp_user.h
 * @brief User configuration header for the 7-segment temperature display middleware.
 *
 * This file contains user-editable constants, feature flags, and parameters
 * that can be adjusted without modifying the core middleware logic.
 */

/* --- Scheduler Configuration --- */
/**
 * @brief Defines the tick time for the time-triggered scheduler in milliseconds.
 *
 * This value determines the granularity of the scheduler tasks.
 * Must match a valid `t_Tick_Time` enum value from `Interval_Timer.h`
 * or result in a suitable counter value for the Interval Timer.
 * For `Interval_Timer_Init`, if this is 1, use `Tick_1_ms`.
 */
#define SCHEDULER_TICK_MS       (1)

/* --- Application Parameters --- */
/**
 * @brief Initial temperature value displayed on the 7-segment display.
 */
#define INITIAL_TEMP_VALUE      (25)

/**
 * @brief Minimum allowable temperature value.
 */
#define MIN_TEMP_VALUE          (0)

/**
 * @brief Maximum allowable temperature value.
 */
#define MAX_TEMP_VALUE          (99)

/* --- Touch Pad Configuration --- */
/**
 * @brief ID for the "PLUS" button (increment temperature).
 *
 * This macro maps to an ID defined in `Touch_Pad_user.h` (e.g., `touch_pad_id_t` enum).
 * Make sure `Touch_Pad_user.h` contains the physical pin mapping for this ID.
 * Example mapping:
 * - POWER_TOUCH = 0
 * - NORMAL_TOUCH = 1 -> Used for PLUS
 * - COLD_TOUCH = 2 -> Used for MINUS
 * - ICE_TOUCH = 3
 */
#define TEMP_PLUS_BUTTON_ID     NORMAL_TOUCH

/**
 * @brief ID for the "MINUS" button (decrement temperature).
 *
 * This macro maps to an ID defined in `Touch_Pad_user.h` (e.g., `touch_pad_id_t` enum).
 * Make sure `Touch_Pad_user.h` contains the physical pin mapping for this ID.
 */
#define TEMP_MINUS_BUTTON_ID    COLD_TOUCH

/* --- Display Configuration --- */
/**
 * @brief The number of digits supported by the 7-segment display hardware.
 *
 * Must match the `SSD_MAX` configuration in `HAL_SSD_Config.h`.
 */
#define DISPLAY_NUM_DIGITS      (2) // Based on HAL_SSD_Config.h SSD_MAX


#endif // SEGTEMP_USER_H
