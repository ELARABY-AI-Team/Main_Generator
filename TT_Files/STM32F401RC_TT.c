/**
 * @file TT.c
 * @brief Implementation file for the Timing & Task (TT) module.
 *
 * This module implements a cooperative time-triggered scheduler for the STM32F401RC.
 * It utilizes TIM2 for generating periodic ticks.
 *
 * MCU: STM32F401RC
 */

#include "TT.h" // Global includes
#include <stdint.h> // Included for clarity, also in TT.h
#include <stddef.h> // Included for NULL, also in TT.h

// Intrinsics for disabling/enabling interrupts, for critical sections
// These are standard CMSIS functions for ARM Cortex-M, not specific registers.
extern void __disable_irq(void);
extern void __enable_irq(void);
extern void __WFI(void); // Wait For Interrupt instruction for idle mode

/* --- Internal Data Structures --- */

/**
 * @brief Structure to hold information for each scheduled task.
 */
typedef struct {
    void (*pTask)(void); /**< Pointer to the task function. */
    tword delay_count;   /**< Current delay counter before the task runs. */
    tword period;        /**< Period for periodic tasks; 0 for one-shot tasks. */
    tbyte run_me;        /**< Flag indicating if the task is ready to run. */
} tt_task_t;

/* --- Internal Global Variables --- */

/**
 * @brief Array of task structures, representing the scheduler's task list.
 */
static tt_task_t s_Tasks[TT_NUMBER_TASKS];

/**
 * @brief Stores the configured scheduler tick time in milliseconds.
 */
static t_tick_time s_tick_ms = 0;

/* --- Watchdog Refresh Placeholder --- */

/**
 * @brief Placeholder for Watchdog Refresh.
 *
 * No IWDG_KR (Independent Watchdog Key Register) or other WDT registers are
 * provided in REGISTER_JSON. A typical STM32 watchdog refresh involves writing
 * 0xAAAA to the IWDG_KR register (e.g., at address 0x40003004).
 * As per instructions: "If missing, implement a generic watchdog refresh sequence based on STM32F401RC".
 * However, also "Do NOT invent registers". Therefore, this function serves as a
 * placeholder for where the refresh *would* occur, acknowledging the lack of
 * specific register definitions in the provided input.
 */
static inline void WDT_Refresh(void) {
    // Placeholder for Watchdog Refresh
    // If IWDG_KR were available (e.g., #define IWDG_KR_ADDR 0x40003004), it would be:
    // *(volatile uint32_t*)IWDG_KR_ADDR = 0xAAAA;
    __asm__("nop"); // A no-op to signify some action, though not an actual WDT refresh.
}

/* --- NVIC Enable Placeholder --- */

/**
 * @brief Placeholder for NVIC (Nested Vectored Interrupt Controller) Enable for TIM2.
 *
 * No NVIC_ISER (Interrupt Set-Enable Register) or other NVIC registers are
 * provided in REGISTER_JSON. A typical STM32 interrupt enable involves setting
 * the corresponding bit in NVIC_ISERx and configuring priority in NVIC_IPRx.
 * For STM32F401RC, TIM2_IRQn is 28, and it's enabled by setting bit 28 in NVIC_ISER0.
 * As per instruction: "Do NOT invent registers". Therefore, this function serves
 * as a placeholder for where the NVIC configuration *would* occur.
 */
static inline void NVIC_Enable_TIM2_IRQ(void) {
    // Placeholder for enabling TIM2 interrupt in NVIC.
    // If NVIC_ISER0 were available (e.g., #define NVIC_ISER0_ADDR 0xE000E100), it would be:
    // *(volatile uint32_t*)NVIC_ISER0_ADDR |= (1U << 28); // TIM2_IRQn is 28 for STM32F401RC
    // A more complete implementation would also set interrupt priority using NVIC_IPRx.
    __asm__("nop"); // A no-op to signify some action.
}

/* --- API Function Implementations (from TT_API.json) --- */

/**
 * @brief Initializes the Timing & Task (TT) module.
 *
 * @param tick_time_ms The desired period of the scheduler tick in milliseconds.
 *
 * @note This function must be called once before any other TT API function.
 *       It configures TIM2 to generate periodic ticks and clears all existing tasks.
 *       Assumes HCLK is 84MHz and APB1 timer clock is also 84MHz (due to multiplier).
 */
void TT_Init(t_tick_time tick_time_ms) {
    // Rule: API_implementation_sequence: WDT_Reset() must be the first thing.
    WDT_Refresh();

    // Rule: TT_Init requirements: Tick time must be between 1ms and 1000ms.
    if (tick_time_ms < 1 || tick_time_ms > 1000) {
        // Handle error: Invalid tick_time_ms. For this example, we'll default to 1ms.
        tick_time_ms = 1;
    }
    s_tick_ms = tick_time_ms;

    // 1. Enable TIM2 clock (TIM2 is on APB1 bus)
    // Register: RCC_APB1ENR (APB1 peripheral clock enable register)
    // Bit 0: TIM2EN
    RCC_APB1ENR |= (1U << 0); // Enable clock for TIM2

    // 2. Configure TIM2 for desired tick_time_ms
    // Assuming TIM2 clock is 84 MHz (HCLK for STM32F401RC, with APB1 Timer clock multiplier)
    // To get 1us (1MHz) base tick, Prescaler (PSC) should be (84,000,000 / 1,000,000) - 1 = 83
    // Then, Auto-reload Register (ARR) for 'tick_time_ms' will be (tick_time_ms * 1000) - 1
    TIM2_PSC = 83; // Prescaler: Timer clock / (83+1) = 84MHz / 84 = 1MHz (1us per count)
    TIM2_ARR = (s_tick_ms * 1000U) - 1U; // Auto-reload: For 's_tick_ms' milliseconds

    // 3. Clear the counter and generate an update event to load the prescaler/ARR values immediately
    // Register: TIM2_CNT (TIM2 counter)
    // Register: TIM2_EGR (TIM2 event generation register)
    // Bit 0: UG (Update Generation)
    TIM2_CNT = 0;       // Reset counter
    TIM2_EGR |= (1U << 0); // Generate an update event

    // 4. Enable TIM2 Update Interrupt
    // Register: TIM2_DIER (TIM2 DMA/Interrupt enable register)
    // Bit 0: UIE (Update Interrupt Enable)
    TIM2_DIER |= (1U << 0); // Enable Update Interrupt

    // Rule: TT_Init requirements: Clears all existing tasks during initialization.
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        TT_Delete_task(i); // Call TT_Delete_task to clear each slot
    }
}

/**
 * @brief Starts the Timing & Task (TT) scheduler.
 *
 * @note This function must be called after TT_Init.
 *       It should only be called once.
 *       Enables TIM2 and its update interrupt in NVIC.
 */
void TT_Start(void) {
    // Rule: API_implementation_sequence: WDT_Reset() must be the first thing.
    WDT_Refresh();

    // 1. Enable TIM2 Counter
    // Register: TIM2_CR1 (TIM2 control register 1)
    // Bit 0: CEN (Counter Enable)
    TIM2_CR1 |= (1U << 0); // Enable TIM2 counter

    // 2. Enable TIM2 Interrupt in NVIC
    // Rule: Must enable timer interrupts with correct priority.
    // Placeholder since NVIC registers are not provided in REGISTER_JSON.
    NVIC_Enable_TIM2_IRQ();
}

/**
 * @brief Dispatches ready tasks for execution.
 *
 * This function iterates through the task list and executes any task
 * that is marked as ready to run (`run_me` flag set).
 *
 * @note This function should be called from the main loop (`while(1)`).
 *       It ensures tasks run within one tick period and cleans up one-shot tasks.
 *       Includes `__WFI()` for idle mode power saving.
 */
void TT_Dispatch_task(void) {
    // Rule: API_implementation_sequence: WDT_Reset() must be the first thing.
    WDT_Refresh();

    tbyte i;
    for (i = 0; i < TT_NUMBER_TASKS; i++) {
        // Critical section to protect shared 'run_me' flag
        __disable_irq(); // Rule: Critical sections must protect shared resources
        if (s_Tasks[i].run_me > 0) {
            s_Tasks[i].run_me--; // Decrement run_me flag
            __enable_irq();

            if (s_Tasks[i].pTask != NULL) {
                // Rule: All tasks must be non-blocking. Max task execution time < tick time.
                s_Tasks[i].pTask(); // Execute the task
            }

            // Rule: Must properly clean up completed one-shot tasks.
            // If the task is one-shot (period == 0) and has just run, delete it.
            if (s_Tasks[i].period == 0) {
                TT_Delete_task(i);
            }
        } else {
            __enable_irq();
        }
    }

    // Rule: System must have idle task for power saving.
    // Puts the CPU into sleep mode until the next interrupt (e.g., timer tick).
    __WFI(); // Wait for Interrupt to conserve power
}

/**
 * @brief Timer Interrupt Service Routine for the TT module.
 *
 * This function is intended to be called by the TIM2 Update Interrupt Handler.
 * It updates the timing of all scheduled tasks.
 *
 * @note Must be called from the timer interrupt handler.
 *       Execution time must be minimized.
 *       Should not call any blocking functions.
 */
void TT_ISR(void) {
    // Rule: API_implementation_sequence: WDT_Reset() must be the first thing.
    WDT_Refresh();

    // Clear TIM2 Update Interrupt Flag
    // Register: TIM2_SR (TIM2 status register)
    // Bit 0: UIF (Update Interrupt Flag)
    if ((TIM2_SR & (1U << 0)) != 0) { // Check if Update Interrupt Flag is set
        TIM2_SR &= ~(1U << 0); // Clear the flag by writing 0 to it (W1C behavior)

        tbyte i;
        for (i = 0; i < TT_NUMBER_TASKS; i++) {
            // Check if task is active (pTask is not NULL)
            if (s_Tasks[i].pTask != NULL) {
                // Decrement delay counter
                if (s_Tasks[i].delay_count > 0) {
                    s_Tasks[i].delay_count--;
                }

                // If delay counter reaches zero, mark task as ready
                if (s_Tasks[i].delay_count == 0) {
                    s_Tasks[i].run_me++; // Mark as ready to run

                    // Reschedule periodic tasks
                    // Rule: Reschedules periodic tasks.
                    if (s_Tasks[i].period > 0) {
                        s_Tasks[i].delay_count = s_Tasks[i].period;
                    }
                }
            }
        }
    }
}

/**
 * @brief Adds a new task to the scheduler.
 *
 * @param task Pointer to the task function to be executed.
 * @param period The period of the task in scheduler ticks (0 for one-shot task).
 * @param delay The initial delay before the task first runs, in scheduler ticks.
 *
 * @return The index of the added task (0 to TT_NUMBER_TASKS-1), or TT_NUMBER_TASKS
 *         if the task list is full.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    // Rule: API_implementation_sequence: WDT_Reset() must be the first thing.
    WDT_Refresh();

    tbyte index = TT_NUMBER_TASKS; // Default to error value

    // Rule: Task function must be non-blocking (programmer responsibility).
    // Rule: Period must be multiple of tick time (period is in tick units).
    // Rule: Delay must be less than maximum allowed value.
    if (delay > TT_MAX_DELAY) {
        return index; // Return error if delay is out of bounds
    }

    __disable_irq(); // Critical section

    // Find an empty slot in the task list
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        if (s_Tasks[i].pTask == NULL) {
            s_Tasks[i].pTask = task;
            s_Tasks[i].delay_count = delay;
            s_Tasks[i].period = period;
            s_Tasks[i].run_me = 0; // Task is not immediately ready to run
            index = i; // Store the assigned index
            break;     // Found a slot, exit loop
        }
    }

    __enable_irq();

    return index;
}

/**
 * @brief Deletes a task from the scheduler.
 *
 * @param task_index The index of the task to be deleted.
 *
 * @note This function validates the task index before deletion and
 *       does not affect other tasks.
 */
void TT_Delete_task(const tbyte task_index) {
    // Rule: API_implementation_sequence: WDT_Reset() must be the first thing.
    WDT_Refresh();

    // Rule: Must validate task index before deletion.
    if (task_index < TT_NUMBER_TASKS) {
        __disable_irq(); // Critical section

        // Rule: Should not affect other tasks.
        // Rule: Must free all resources associated with task (by clearing struct members).
        s_Tasks[task_index].pTask = NULL;
        s_Tasks[task_index].delay_count = 0;
        s_Tasks[task_index].period = 0;
        s_Tasks[task_index].run_me = 0;

        __enable_irq();
    }
    // If task_index is invalid, do nothing.
}