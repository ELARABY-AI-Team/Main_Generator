#ifndef TT_H
#define TT_H

#include <stdint.h> // global_includes

// General requirements for Time Triggered scheduler implementation
// All tasks must be non-blocking
// Maximum task execution time must be less than tick time
// System must have idle task for power saving
// Critical sections must protect shared resources
// Task priorities are handled by execution order in dispatch

/**
 * @brief Represents time in milliseconds.
 */
typedef uint32_t t_tick_time;

/**
 * @brief Represents an unsigned 8-bit integer, used for task indexes.
 */
typedef uint8_t tbyte;

/**
 * @brief Represents an unsigned 16-bit integer, used for time values (period, delay).
 */
typedef uint16_t tword;

// Configuration for the Timing & Task (TT) module
#define TT_NUMBER_TASKS         (10)      ///< Maximum number of tasks the scheduler can manage
#define TT_ERROR                TT_NUMBER_TASKS ///< Error return value for TT_Add_task

// Internal task structure
typedef struct {
    void (*task_func)(void);    ///< Pointer to the task function
    tword delay;                ///< Initial delay before the task first runs (in ticks)
    tword period;               ///< Period for recurring tasks (in ticks); 0 for one-shot tasks
    uint8_t run_me;             ///< Flag: 1 when the task is due to run
} TT_Task_t;

// External declaration for the task buffer, which is defined in TT.c
extern TT_Task_t TT_Task_Buffer[TT_NUMBER_TASKS];

// API functions for the Timing & Task (TT) module

/**
 * @brief Initializes the Timer with the given tick time (Tick_Time_ms).
 * This defines the basic time unit for the scheduler.
 * It loops through all available task slots (TT_NUMBER_TASKS) and deletes them.
 *
 * @param tick_time_ms The desired tick time in milliseconds (1ms to 1000ms).
 */
void TT_Init(t_tick_time tick_time_ms);

/**
 * @brief Enables the timer, starting scheduler tick interrupts for task scheduling and dispatching.
 * Must be called after TT_Init. Should only be called once.
 */
void TT_Start(void);

/**
 * @brief Scans the task list, runs ready tasks, cleans up one-shot tasks,
 * and puts the system into idle mode until the next tick.
 * Should be called from the main loop.
 */
void TT_Dispatch_task(void);

/**
 * @brief Updates the timing of all scheduled tasks every tick.
 * Decreases delay counters, marks tasks as ready when delay reaches zero,
 * and reschedules periodic tasks.
 * Must be called from the timer interrupt handler.
 */
void TT_ISR(void);

/**
 * @brief Registers a new task with a function pointer, initial delay, and period.
 *
 * @param task Function pointer to the task to be added.
 * @param period Period for the task in ticks. 0 for a one-shot task.
 * @param delay Initial delay before the task first runs in ticks.
 * @return The index of the added task, or TT_NUMBER_TASKS if no free slot.
 */
tbyte TT_Add_task(void (* task) (void), const tword period, const tword delay);

/**
 * @brief Clears all information for the specified task index,
 * effectively removing it from the scheduler.
 *
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index);

#endif // TT_H