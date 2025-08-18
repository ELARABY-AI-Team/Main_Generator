#ifndef TT_H
#define TT_H

#include <stdbool.h> // For bool type

// --- Data Types from TT_Rules.json -> TT_general_rules -> data_types ---
typedef unsigned int        t_tick_time; // unsigned integer representing milliseconds
typedef unsigned char       tbyte;       // unsigned 8-bit integer for task indexes
typedef unsigned short      tword;       // unsigned 16-bit integer for time values

// --- Constants ---
#define TT_NUMBER_TASKS             10       // Maximum number of tasks in the scheduler
#define TT_ERROR_ADD_TASK_FAILED    TT_NUMBER_TASKS // Return value for TT_Add_task on failure

// --- API Functions (from TT_API.json) ---

/**
 * @brief Initializes the Timing & Task (TT) module.
 *        Configures the system timer for the specified tick time.
 *        Clears all existing tasks.
 * @param tick_time_ms The desired system tick time in milliseconds (1ms to 1000ms).
 */
void TT_Init(t_tick_time tick_time_ms);

/**
 * @brief Starts the Timing & Task (TT) scheduler.
 *        Enables the configured system timer to begin generating interrupts.
 */
void TT_Start(void);

/**
 * @brief Dispatches ready tasks and manages the system idle state.
 *        This function should be called repeatedly in the main loop.
 */
void TT_Dispatch_task(void);

/**
 * @brief Timer Interrupt Service Routine (ISR) handler for the TT module.
 *        Updates task timing and readiness flags.
 *        This function must be called from the appropriate timer interrupt handler.
 */
void TT_ISR(void);

/**
 * @brief Adds a new task to the scheduler.
 * @param task Pointer to the task function to be executed.
 * @param period The period (in ticks) at which the task should repeat.
 *               Set to 0 for a one-shot task.
 * @param delay The initial delay (in ticks) before the task first runs.
 * @return The index of the added task, or TT_ERROR_ADD_TASK_FAILED if the task list is full.
 */
tbyte TT_Add_task(void (* task)(void), const tword period, const tword delay);

/**
 * @brief Deletes a task from the scheduler.
 *        Clears all information for the specified task index, effectively removing it.
 * @param task_index The index of the task to be deleted.
 */
void TT_Delete_task(const tbyte task_index);

#endif // TT_H