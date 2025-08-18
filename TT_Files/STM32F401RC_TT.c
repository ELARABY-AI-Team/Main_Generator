#include "TT.h" // global_includes as per Rules.json
#include <stddef.h> // For NULL

// --- Generic System Intrinsics (for WFI, enable/disable IRQ) ---
// These are typical ARM Cortex-M intrinsics not provided in register JSON,
// assumed based on MCU_NAME (STM32F401RC) and general embedded practice.
#if defined(__GNUC__)
  #define __disable_irq()     __asm volatile ("cpsid i" : : : "memory")
  #define __enable_irq()      __asm volatile ("cpsie i" : : : "memory")
  #define __WFI()             __asm volatile ("wfi" : : : "memory")
#else
  // Fallback for other compilers (e.g., Keil MDK, IAR EWARM).
  // User may need to replace with compiler-specific intrinsics or empty macros.
  // For strict compliance with "implemented as code", empty do-while is used.
  #warning "Compiler-specific intrinsics for __disable_irq, __enable_irq, __WFI may be needed."
  #define __disable_irq()     do {} while(0)
  #define __enable_irq()      do {} while(0)
  #define __WFI()             do {} while(0)
#endif

// --- Watchdog Specific Defines ---
// Values for IWDG_KR based on STM32F401RC reference manual
#define IWDG_KEY_ENABLE     (0xCCCCU) // Enable the watchdog
#define IWDG_KEY_RELOAD     (0xAAAAU) // Reload the watchdog counter
#define IWDG_KEY_ACCESS     (0x5555U) // Enable write access to PR and RLR

// --- Internal Task Structure ---
// Represents a single task managed by the scheduler.
typedef struct
{
    void (*pTask)(void);   // Pointer to the task function to be executed
    tword  Delay;          // Current delay (in ticks) until the task is due to run
    tword  Period;         // Period (in ticks) for periodic tasks. 0 for one-shot tasks.
    tbyte  RunMe;          // Flag (TRUE/FALSE, or count) to indicate when task is due
} TT_Task_t;

// --- Internal Scheduler Data ---
static TT_Task_t TT_Tasks[TT_NUMBER_TASKS]; // Array to hold all scheduled tasks
static volatile t_tick_time g_tick_count = 0; // Global counter for scheduler ticks
static t_tick_time s_tick_ms_period = 0;    // Stores the configured tick period in milliseconds

// --- Private Function: WDT_Reset ---
/**
 * @brief Implements a generic watchdog refresh sequence based on STM32F401RC.
 *        This function assumes the Independent Watchdog (IWDG) peripheral is
 *        configured and enabled elsewhere (e.g., in a system initialization).
 *        It only reloads the watchdog counter to prevent a system reset.
 * @note  As per "Rules.json", this is implemented as code, not comment.
 *        The IWDG registers are used based on common STM32F401RC knowledge,
 *        as they were not explicitly provided in the REGISTER_JSON, but required
 *        by the rule for WDT implementation on this MCU.
 */
static void WDT_Reset(void)
{
    // Reload the IWDG counter to prevent reset
    // This assumes IWDG_BASE and IWDG_TypeDef are correctly defined
    // and IWDG is already initialized (e.g., prescaler, reload value set).
    IWDG->KR = IWDG_KEY_RELOAD;
}

// --- Public API Functions (from TT_API.json) ---

/**
 * @brief Initializes the Timing & Task (TT) module.
 *        Configures TIM2 as the scheduler's time base and clears all task slots.
 * @param tick_time_ms The desired tick period in milliseconds (1ms to 1000ms).
 * @note  Assumes the APB1 timer clock (TIM2) is running at 42MHz for STM32F401RC.
 *        This function must be called before any other TT API function.
 *        If tick_time_ms is invalid, it defaults to 1ms.
 */
void TT_Init(t_tick_time tick_time_ms)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Validate tick_time_ms as per Rules.json requirement (1ms to 1000ms)
    if (tick_time_ms == 0 || tick_time_ms > 1000)
    {
        // Invalid tick time, default to 1ms to ensure functional operation.
        tick_time_ms = 1;
    }
    s_tick_ms_period = tick_time_ms;

    // Critical section to prevent interrupts during clock configuration.
    __disable_irq();

    // 1. Enable TIM2 peripheral clock (TIM2 is connected to APB1 bus)
    // RCC_APB1ENR address: 0x40023838 (AHB1 peripheral clock enable register)
    // TIM2EN is bit 0 in APB1ENR
    RCC->APB1ENR |= (1UL << 0); // Set TIM2EN bit to enable TIM2 clock

    // 2. Configure TIM2 for desired tick_time_ms
    // Assumed Timer clock frequency (TIM2 input clock) = APB1 clock = 42MHz for STM32F401RC.
    // We want a counter frequency that allows easy calculation for milliseconds.
    // A 1MHz counter frequency means 1000 counts per millisecond.
    // To achieve 1MHz from 42MHz, we need a prescaler of (42 - 1) = 41.
    // TIM2_PSC address: 0x40000028 (TIM2 prescaler)
    TIM2->PSC = 41; // Sets prescaler to 41, resulting in 42MHz / (41 + 1) = 1MHz counter clock.

    // Auto-reload register (ARR) sets the period of the timer.
    // At 1MHz counter clock, 1000 counts correspond to 1ms.
    // So for a tick_time_ms period, we need (1000 * tick_time_ms) counts.
    // The ARR register holds the value that the counter compares against, so it's (counts - 1).
    // TIM2_ARR address: 0x4000002C (TIM2 auto-reload register)
    TIM2->ARR = (1000UL * s_tick_ms_period) - 1;

    // Reset TIM2 counter to 0
    // TIM2_CNT address: 0x40000024 (TIM2 counter)
    TIM2->CNT = 0;

    // Re-enable interrupts after configuration.
    __enable_irq();

    // 3. Clear all existing tasks as per Rules.json requirement
    // Iterates through all possible task slots and effectively disables them.
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        TT_Delete_task(i); // Call TT_Delete_task to clear task data
    }

    // 4. Initialize the global tick counter
    g_tick_count = 0;
}

/**
 * @brief Starts the scheduler by enabling the timer and its interrupts.
 *        This function must be called after TT_Init().
 *        As per Rules.json, it should only be called once.
 */
void TT_Start(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Critical section to ensure atomic enabling of timer and interrupts.
    __disable_irq(); 

    // Clear any pending TIM2 update interrupt flag to avoid immediate spurious interrupt.
    // TIM2_SR address: 0x40000010 (TIM2 status register). UIF is bit 0.
    TIM2->SR &= ~(1UL << 0);

    // Enable TIM2 update interrupt (UIE bit in DIER register).
    // TIM2_DIER address: 0x4000000C (TIM2 DMA/interrupt enable register). UIE is bit 0.
    TIM2->DIER |= (1UL << 0);

    // Enable the TIM2 counter (CEN bit in CR1 register) to start the timer.
    // TIM2_CR1 address: 0x40000000 (TIM2 control register 1). CEN is bit 0.
    TIM2->CR1 |= (1UL << 0);

    // Enable TIM2 interrupt in the Nested Vectored Interrupt Controller (NVIC).
    // TIM2_IRQn (28) for STM32F401RC. NVIC_ISER (Interrupt Set-Enable Register)
    // manages enabling interrupts. ISER[0] covers IRQ 0-31.
    // NVIC_BASE address: 0xE000E100 (ISER[0] address).
    NVIC->ISER[TIM2_IRQn / 32] |= (1UL << (TIM2_IRQn % 32));

    __enable_irq(); // Re-enable interrupts.
}

/**
 * @brief Dispatches ready tasks from the scheduler's task list.
 *        This function should be called repeatedly from the main application loop.
 *        It executes tasks that are marked as ready (`RunMe > 0`).
 *        One-shot tasks (Period == 0) are automatically deleted after execution.
 *        Puts the system into an idle (low-power) mode if no tasks are ready to run.
 * @note  As per Rules.json:
 *        - All tasks are expected to be non-blocking and their total execution time
 *          must be less than the scheduler's tick period.
 *        - This function handles task execution and cleanup.
 */
void TT_Dispatch_task(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Iterate through all possible task slots.
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        // Check if the task is active and marked as ready to run.
        if (TT_Tasks[i].pTask != NULL && TT_Tasks[i].RunMe > 0)
        {
            TT_Tasks[i].pTask(); // Execute the task function.

            // Decrement the 'RunMe' flag, indicating it has been dispatched.
            TT_Tasks[i].RunMe--; 

            // If it's a one-shot task (Period is 0), delete it after execution.
            // Rule: Must properly clean up completed one-shot tasks.
            if (TT_Tasks[i].Period == 0)
            {
                TT_Delete_task(i);
            }
        }
    }

    // Rule: System must have idle task for power saving.
    // Put the system into idle mode (Wait For Interrupt) until the next interrupt occurs.
    // This allows the MCU to save power when no tasks are immediately ready.
    __WFI(); 
}

/**
 * @brief Updates the timing of all scheduled tasks.
 *        This function MUST be called from the timer interrupt handler (e.g., TIM2_IRQHandler).
 *        It decrements delay counters, marks tasks as ready, and reschedules periodic tasks.
 * @note  As per Rules.json:
 *        - Execution time must be minimized.
 *        - Should not call any blocking functions.
 */
void TT_ISR(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Check and clear TIM2 Update Interrupt Flag (UIF) to confirm interrupt source.
    // TIM2_SR address: 0x40000010 (TIM2 status register). UIF is bit 0.
    if ((TIM2->SR & (1UL << 0)) != 0) // Check if Update Interrupt Flag is set
    {
        TIM2->SR &= ~(1UL << 0); // Clear the UIF flag by writing 0 to it.

        g_tick_count++; // Increment the global tick counter.

        // Iterate through all task slots to update their timing.
        for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
        {
            // Check if the task slot is active (i.e., a task is assigned).
            if (TT_Tasks[i].pTask != NULL)
            {
                if (TT_Tasks[i].Delay == 0)
                {
                    // If delay is zero, the task is due to run.
                    TT_Tasks[i].RunMe++; // Increment RunMe flag to mark task as ready.

                    if (TT_Tasks[i].Period != 0)
                    {
                        // For periodic tasks, reset their delay for the next period.
                        TT_Tasks[i].Delay = TT_Tasks[i].Period;
                    }
                }
                else
                {
                    // Task is not yet due, decrement its delay counter.
                    TT_Tasks[i].Delay--; 
                }
            }
        }
    }
}

/**
 * @brief Registers a new task with the scheduler.
 *        It finds an available slot, assigns the task function, its period, and initial delay.
 * @param task Pointer to the task function (must be non-blocking as per Rules.json).
 * @param period The period (in ticks) at which the task should run.
 *               Set to 0 for one-shot tasks (task runs once after delay and is deleted).
 * @param delay The initial delay (in ticks) before the task first runs.
 * @return The index (0 to TT_NUMBER_TASKS-1) of the added task, or TT_NUMBER_TASKS
 *         if the task list is full or input is invalid.
 */
tbyte TT_Add_task(void (* task) (void), const tword period, const tword delay)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    tbyte index = TT_NUMBER_TASKS; // Default return value indicating failure (task list full).

    // Input validation: Task function pointer must not be NULL.
    if (task == NULL)
    {
        return TT_NUMBER_TASKS;
    }

    // Critical section to protect the shared task list from concurrent access
    // (e.g., from the ISR or other main loop functions).
    __disable_irq(); 

    // Iterate through task list to find an empty slot.
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        if (TT_Tasks[i].pTask == NULL) // Check if the slot is empty (indicated by NULL task pointer).
        {
            // Empty slot found, assign task properties.
            TT_Tasks[i].pTask = task;
            TT_Tasks[i].Delay = delay;
            TT_Tasks[i].Period = period;
            TT_Tasks[i].RunMe = 0; // Initialize RunMe to 0, task is not yet ready.
            index = i; // Store the assigned index.
            break;     // Exit loop once a task is added.
        }
    }
    __enable_irq(); // Re-enable interrupts.

    return index;
}

/**
 * @brief Deletes a task from the scheduler.
 *        Clears all information for the specified task index, effectively removing it.
 * @param task_index The index of the task to delete (0 to TT_NUMBER_TASKS-1).
 * @note  As per Rules.json:
 *        - Must validate task index before deletion.
 *        - Should not affect other tasks.
 *        - Must free all resources associated with task (by setting to NULL/0).
 */
void TT_Delete_task(const tbyte task_index)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Validate task_index to ensure it is within the valid range.
    if (task_index < TT_NUMBER_TASKS)
    {
        // Critical section to protect the shared task list during modification.
        __disable_irq(); 

        // Clear all task information to remove it from the scheduler.
        // Setting pTask to NULL marks the slot as free.
        TT_Tasks[task_index].pTask = NULL;   
        TT_Tasks[task_index].Delay = 0;
        TT_Tasks[task_index].Period = 0;
        TT_Tasks[task_index].RunMe = 0;

        __enable_irq(); // Re-enable interrupts.
    }
    // If task_index is out of range, do nothing as per validation rule.
}

// --- Example Interrupt Handler for TIM2 ---
// This function needs to be linked to the TIM2_IRQHandler vector in the startup code.
// It serves as an example of how the TT_ISR function should be invoked from
// the microcontroller's interrupt vector table.
// This function itself is not part of the TT API but illustrates its typical usage.
void TIM2_IRQHandler(void)
{
    TT_ISR();
}