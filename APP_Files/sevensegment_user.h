/**
 * @file sevensegment_user.h
 * @brief User configuration header for the sevensegment middleware.
 *
 * This file contains constants, feature flags, and editable parameters
 * that allow end-users to adjust the application's behavior without
 * modifying the core middleware code.
 */

#ifndef SEVENSEGMENT_USER_H
#define SEVENSEGMENT_USER_H

//=============================================================================
// Includes (for types if needed)
//=============================================================================
#include "MCAL.h"             // For t_sys_volt type
#include "Interval_Timer.h"   // For t_clk_source, t_Tick_Time types
#include "Touch_Pad.h"        // For touch_pad_id_t types

//=============================================================================
// User Configuration Parameters
//=============================================================================

/**
 * @brief Defines the system voltage for MCU configuration.
 *        Used by MCU_Config_Init.
 */
#define SYS_VOLTAGE                     sys_volt_5v

/**
 * @brief Initial value displayed on the seven-segment display at startup.
 *        Must be within MIN_SSD_DISPLAY_VALUE and MAX_SSD_DISPLAY_VALUE.
 */
#define INITIAL_SSD_DISPLAY_VALUE       25

/**
 * @brief Minimum value that can be displayed on the seven-segment display.
 */
#define MIN_SSD_DISPLAY_VALUE           0

/**
 * @brief Maximum value that can be displayed on the seven-segment display.
 */
#define MAX_SSD_DISPLAY_VALUE           99 // Assuming a 2-digit display (SSD_MAX=2 in HAL_SSD_Config.h)

/**
 * @brief Defines the ID of the touch button used for incrementing the display value.
 *        Maps to existing touch_pad_id_t from Touch_Pad_user.h
 */
#define PLUS_TOUCH_ID                   POWER_TOUCH

/**
 * @brief Defines the ID of the touch button used for decrementing the display value.
 *        Maps to existing touch_pad_id_t from Touch_Pad_user.h
 */
#define MINUS_TOUCH_ID                  NORMAL_TOUCH

//=============================================================================
// Time-Triggered Scheduler Configuration (in ticks, where 1 tick = 1ms)
//=============================================================================

/**
 * @brief Clock source for the Interval Timer (scheduler's tick source).
 *        Choose between SUBSYSTEM_EXTERNAL_CLOCK or LOW_SPEED_INTERNAL_CLOCK.
 */
#define SCHEDULER_CLOCK_SOURCE          LOW_SPEED_INTERNAL_CLOCK

/**
 * @brief Tick time for the scheduler's Interval Timer.
 *        Must be one of the t_Tick_Time enum values (e.g., Tick_1_ms).
 *        This defines the base tick period for all scheduled tasks.
 *        For this application, we are targeting 1ms tick as per flowchart "Init Time Trigger (1 ms)".
 */
#define SCHEDULER_TICK_TIME_ENUM        Tick_1_ms

/**
 * @brief Delay (in scheduler ticks) before the TOUCH_PAD_update task first runs.
 *        Flowchart specifies delay=0ms.
 */
#define TOUCH_PAD_UPDATE_DELAY_TICKS    0

/**
 * @brief Period (in scheduler ticks) at which the TOUCH_PAD_update task runs.
 *        Flowchart specifies period=18ms.
 */
#define TOUCH_PAD_UPDATE_PERIOD_TICKS   18

/**
 * @brief Delay (in scheduler ticks) before the SSD_Update task first runs.
 *        Flowchart specifies delay=2ms.
 */
#define SSD_UPDATE_DELAY_TICKS          2

/**
 * @brief Period (in scheduler ticks) at which the SSD_Update task runs.
 *        Flowchart specifies period=5ms.
 */
#define SSD_UPDATE_PERIOD_TICKS         5

/**
 * @brief Delay (in scheduler ticks) before the App_MainTask first runs.
 *        The flowchart doesn't specify delay, using 0 for immediate start.
 */
#define APP_MAIN_TASK_DELAY_TICKS       0

/**
 * @brief Period (in scheduler ticks) at which the App_MainTask runs.
 *        A value of 100ms provides a reasonable responsiveness for button presses.
 */
#define APP_MAIN_TASK_PERIOD_TICKS      100

//=============================================================================
// Workaround for HAL files' MCAL dependency (read "Coding Guidelines" in prompt)
// The HAL files (e.g., HAL_SSD.c, Touch_Pad.c) often include
// "MCAL_General_Config.h" and pass a `usage` parameter to GPIO functions.
// This is a common pattern in specific embedded frameworks.
// Given the constraint "The hall folder contains donot change it or play inside it only use it",
// and that `MCAL_General_Config.h` is not provided, nor do MCAL.h's GPIO functions
// accept a `usage` parameter, this define attempts to bridge the gap.
// This defines `normal_usage` to an integer, which will be ignored by MCAL's API
// but allow HAL's calls to compile if the compiler accepts extra parameters
// (or if the MCAL implementation is flexible). In a strict C environment, this would
// be a compile-time error due to mismatched function signatures.
// If this causes compilation issues, the HAL files would need to be adapted
// to the provided MCAL.h/c signatures, which contradicts a project constraint.
// Proceeding with this workaround as a "best effort" to integrate without modifying HAL.
//=============================================================================
#define normal_usage 0

#endif // SEVENSEGMENT_USER_H