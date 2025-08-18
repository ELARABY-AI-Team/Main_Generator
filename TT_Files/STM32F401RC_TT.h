#ifndef TT_H
#define TT_H

#include <stdint.h> // For standard integer types
#include <stdbool.h> // For boolean type

// --- Global Includes (from Rules.json) ---
// No specific global includes defined in Rules.json beyond standard types.

// --- MCU Name ---
// STM32F401RC

// --- Definitions ---
#define TT_NUMBER_TASKS 10      // Maximum number of tasks the scheduler can manage
#define TT_MAX_DELAY    UINT16_MAX // Maximum delay value for a task (from tword type)

// --- Data Types (from Rules.json) ---
typedef unsigned int t_tick_time; // unsigned integer representing milliseconds
typedef uint8_t tbyte;           // unsigned 8-bit integer for task indexes
typedef uint16_t tword;          // unsigned 16-bit integer for time values

// --- Task Control Block Structure ---
typedef struct
{
    void (*pTask)(void); // Pointer to the task function
    tword Delay;         // Delay (in ticks) until the task is next due
    tword Period;        // Period (in ticks) for periodic tasks. 0 for one-shot tasks.
    bool RunMe;          // Flag to indicate that the task is ready to run
} sTask;

// --- External Declarations ---
// The task list array is managed internally by the TT module, but its existence
// is acknowledged here if it were to be directly accessed (which is not recommended).
// extern sTask TT_Tasks[TT_NUMBER_TASKS]; // Not strictly needed for public API, but good for understanding

// --- API Function Prototypes (from TT_API.json) ---

/**
 * @brief Initializes the Timing & Task (TT) module and configures the system tick timer.
 *        This function must be called before any other TT API function.
 * @param tick_time The desired tick time in milliseconds. Must be between 1ms and 1000ms.
 *
 * @details This function performs the following:
 *          - Initializes all task slots as empty.
 *          - Configures a hardware timer (e.g., TIM2 for STM32F401RC) to generate
 *            an interrupt at the specified tick_time frequency.
 *          - Ensures the system clock is enabled for the chosen timer.
 */
void TT_Init(t_tick_time tick_time_ms);

/**
 * @brief Starts the Timing & Task (TT) scheduler.
 *        This function enables the timer interrupts, allowing the scheduler to begin
 *        tracking time and dispatching tasks.
 *        Must be called after TT_Init().
 */
void TT_Start(void);

/**
 * @brief Dispatches ready tasks from the scheduler's task list.
 *        This function should be called repeatedly from the main loop.
 *        It executes tasks whose 'RunMe' flag is set, cleans up one-shot tasks,
 *        and puts the system into a low-power idle mode until the next tick.
 */
void TT_Dispatch_task(void);

/**
 * @brief Interrupt Service Routine (ISR) for the TT module.
 *        This function must be called from the configured hardware timer's interrupt handler.
 *        It decrements task delay counters and sets the 'RunMe' flag for tasks that are due.
 *        Execution time must be minimized, and no blocking functions should be called.
 */
void TT_ISR(void);

/**
 * @brief Adds a new task to the scheduler's task list.
 * @param task Pointer to the task function (must be non-blocking).
 * @param period The period (in ticks) at which the task should run. Set to 0 for a one-shot task.
 * @param delay The initial delay (in ticks) before the task first runs.
 * @return The index of the added task in the scheduler's list, or TT_NUMBER_TASKS
 *         if the task list is full or inputs are invalid.
 */
tbyte TT_Add_task(void (* task) (void), const tword period, const tword delay);

/**
 * @brief Deletes a task from the scheduler's task list.
 * @param task_index The index of the task to be deleted.
 *
 * @details This function clears all information for the specified task index,
 *          effectively removing it from the scheduler. It validates the task index
 *          to prevent invalid memory access and ensures other tasks are not affected.
 */
void TT_Delete_task(const tbyte task_index);

#endif // TT_H