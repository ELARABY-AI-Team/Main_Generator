/**
 * @file TT.c
 * @brief Implementation file for the Timing & Task (TT) module.
 *
 * This module provides a simple time-triggered scheduler for the STM32F401RC MCU,
 * managing tasks based on a configurable timer tick.
 */

#include "TT.h" // Module header file

// --- Internal Data Structures and Global Variables ---

/**
 * @brief Enum for task states.
 */
typedef enum {
    TT_TASK_STATE_IDLE = 0,   /**< Task is inactive or waiting for its turn. */
    TT_TASK_STATE_READY,      /**< Task is ready to be executed. */
    TT_TASK_STATE_DELETED     /**< Task slot is empty/available. */
} TT_TaskState_t;

/**
 * @brief Structure to hold individual task information.
 */
typedef struct {
    void (*task_func)(void); /**< Pointer to the task function. */
    tword period;           /**< Task period in ticks. 0 for one-shot. */
    tword delay;            /**< Remaining delay until next execution in ticks. */
    TT_TaskState_t state;   /**< Current state of the task. */
    tbyte task_id;          /**< Unique identifier for the task (its index in the array). */
} TT_Task_t;

static TT_Task_t s_tt_tasks[TT_NUMBER_TASKS]; /**< Array of task control blocks. */
static volatile t_tick_time s_scheduler_tick_ms = 0; /**< Stores the configured tick time in ms. */
static volatile t_tick_time s_current_system_ticks = 0; /**< Global counter for scheduler ticks. */

// --- Internal Helper Functions ---

/**
 * @brief Placeholder function for Watchdog Timer (WDT) refresh.
 *
 * This function should contain the necessary register writes to refresh the
 * MCU's watchdog timer, preventing a system reset. As per the instructions,
 * specific WDT registers were not provided in the JSON input, so this is an
 * empty placeholder to satisfy the requirement that `WDT_Refresh()` is called
 * in every API function.
 *
 * For STM32F401RC, this would typically involve writing to the IWDG_KR (Key Register)
 * with the reload value (0xAAAA).
 * Example if IWDG_KR was defined:
 * #define IWDG_KR ((volatile uint32_t *)0x40003004U) // Placeholder address, not from JSON
 * #define IWDG_KEY_RELOAD (0xAAAAU) // Reload key
 * REG_WRITE(IWDG_KR, 0, IWDG_KEY_RELOAD); // Or equivalent direct write
 */
static inline void WDT_Refresh(void) {
    // WDT refresh code would go here.
    // As no specific WDT registers were provided in the input JSON,
    // this function is intentionally left empty but serves as a required call.
}

/**
 * @brief Placeholder for entering a low-power idle mode.
 *
 * This function represents the system entering a low-power state
 * until the next interrupt (e.g., timer tick).
 *
 * On ARM Cortex-M microcontrollers, this is typically achieved using the
 * `__WFI()` (Wait For Interrupt) instruction.
 * As per instructions, direct use of non-provided APIs is restricted.
 * This is a conceptual representation.
 */
static inline void TT_EnterIdleMode(void) {
    // __WFI(); // Wait for Interrupt to save power until the next tick
    // As __WFI() is not a register or an API provided, this is commented out
    // but indicates the intended behavior as per "System must have idle task for power saving" rule.
}

/**
 * @brief Placeholder for entering a critical section.
 *
 * This function disables interrupts to protect shared resources.
 * As per instructions, specific registers for interrupt control were not provided.
 *
 * On ARM Cortex-M microcontrollers, this is typically achieved using the
 * `__disable_irq()` intrinsic.
 */
static inline void TT_ENTER_CRITICAL(void) {
    // __disable_irq(); // Disable global interrupts
}

/**
 * @brief Placeholder for exiting a critical section.
 *
 * This function re-enables interrupts.
 * As per instructions, specific registers for interrupt control were not provided.
 *
 * On ARM Cortex-M microcontrollers, this is typically achieved using the
 * `__enable_irq()` intrinsic.
 */
static inline void TT_EXIT_CRITICAL(void) {
    // __enable_irq(); // Enable global interrupts
}


// --- API Function Implementations (from TT_API.json) ---

void TT_Init(t_tick_time tick_time_ms) {
    WDT_Refresh(); // API_implementation_sequence: WDT_Reset() first

    // TT_Init requirements:
    // 1. Must be called before any other TT API function (implicit by design).
    // 2. Tick time must be between 1ms and 1000ms.
    if (tick_time_ms < TT_MIN_TICK_TIME_MS || tick_time_ms > TT_MAX_TICK_TIME_MS) {
        // Handle error: invalid tick_time_ms. For this exercise, we'll
        // either clamp it or use a default, or simply return/assert.
        // Let's default to 1ms if outside range for robustness.
        s_scheduler_tick_ms = TT_MIN_TICK_TIME_MS;
    } else {
        s_scheduler_tick_ms = tick_time_ms;
    }

    s_current_system_ticks = 0;

    // 3. Clears all existing tasks during initialization.
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        // Calling TT_Delete_task to clear all slots.
        // Note: TT_Delete_task itself calls WDT_Refresh().
        // For initialization, direct internal reset is fine to avoid recursive WDT_Refresh calls.
        s_tt_tasks[i].task_func = NULL;
        s_tt_tasks[i].period = 0;
        s_tt_tasks[i].delay = 0;
        s_tt_tasks[i].state = TT_TASK_STATE_DELETED;
        s_tt_tasks[i].task_id = i;
    }

    // Configure TIM3 for periodic interrupts
    // The STM32F401RC has TIM3 on APB1. Max APB1 frequency is 42MHz.
    // If APB1 prescaler is > 1, the timer clock is 2 * PCLK1.
    // Assuming PCLK1 = 42MHz, then TIM3_CLK = 84MHz.
    const uint32_t TIM3_CLK_FREQ_HZ = 84000000U; // 84 MHz

    // 1. Enable TIM3 clock (on APB1 bus)
    REG_SET_BIT(RCC_BASE_ADDR, RCC_APB1ENR_OFFSET, RCC_APB1ENR_TIM3EN_POS);

    // 2. Disable TIM3 before configuration
    REG_CLEAR_BIT(TIM3_BASE_ADDR, TIM3_CR1_OFFSET, TIM3_CR1_CEN_POS);

    // 3. Set prescaler and auto-reload register for desired tick_time_ms
    // To achieve a 1ms tick:
    // PSC = (TIM3_CLK_FREQ_HZ / 1,000,000) - 1 = (84MHz / 1MHz) - 1 = 83.
    // This makes the timer counter increment at 1 MHz (1 tick = 1 microsecond).
    // ARR = (desired_tick_time_ms * 1000 microseconds/ms) - 1.
    // Example: for 1ms tick, ARR = (1 * 1000) - 1 = 999.
    // Example: for 10ms tick, ARR = (10 * 1000) - 1 = 9999.
    uint32_t prescaler = (TIM3_CLK_FREQ_HZ / 1000000U) - 1U;
    uint32_t auto_reload = (s_scheduler_tick_ms * 1000U) - 1U;

    REG_WRITE(TIM3_BASE_ADDR, TIM3_PSC_OFFSET, prescaler);
    REG_WRITE(TIM3_BASE_ADDR, TIM3_ARR_OFFSET, auto_reload);

    // 4. Generate an update event to apply prescaler and ARR values immediately
    REG_SET_BIT(TIM3_BASE_ADDR, TIM3_EGR_OFFSET, TIM3_EGR_UG_POS);

    // 5. Clear update interrupt flag
    REG_CLEAR_BIT(TIM3_BASE_ADDR, TIM3_SR_OFFSET, TIM3_SR_UIF_POS);

    // 6. Enable update interrupt
    REG_SET_BIT(TIM3_BASE_ADDR, TIM3_DIER_OFFSET, TIM3_DIER_UIE_POS);

    // Note on NVIC: Interrupt controller setup (NVIC_EnableIRQ, NVIC_SetPriority)
    // is typically handled by CMSIS functions. Since no NVIC registers were provided
    // in the JSON, this part is omitted from direct register access,
    // but would be necessary in a real application.
    // For example:
    // NVIC_EnableIRQ(TIM3_IRQn);
    // NVIC_SetPriority(TIM3_IRQn, 0); // Highest priority
}

void TT_Start(void) {
    WDT_Refresh(); // API_implementation_sequence: WDT_Reset() first

    // TT_Start requirements:
    // 1. Must be called after TT_Init (implicit).
    // 2. Should only be called once (implicit by design, repeated calls harmless but unnecessary).
    // 3. Must enable timer interrupts with correct priority (partially handled in TT_Init comment).

    // Enable TIM3 counter
    REG_SET_BIT(TIM3_BASE_ADDR, TIM3_CR1_OFFSET, TIM3_CR1_CEN_POS);
}

void TT_Dispatch_task(void) {
    WDT_Refresh(); // API_implementation_sequence: WDT_Reset() first

    // TT_Dispatch_task requirements:
    // 1. Should be called from main loop.
    // 2. Must handle task execution within one tick period.
    // 3. Must properly clean up completed one-shot tasks.

    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        TT_ENTER_CRITICAL(); // Protect task state modification
        if (s_tt_tasks[i].state == TT_TASK_STATE_READY) {
            s_tt_tasks[i].state = TT_TASK_STATE_IDLE; // Mark as dispatched/running
            TT_EXIT_CRITICAL();

            // Execute the task
            if (s_tt_tasks[i].task_func != NULL) {
                s_tt_tasks[i].task_func();
            }

            // Cleanup one-shot tasks
            if (s_tt_tasks[i].period == 0) {
                TT_Delete_task(i); // This will call WDT_Refresh internally
            }
        } else {
            TT_EXIT_CRITICAL(); // Exit if not entering critical action
        }
    }

    // System must have idle task for power saving
    // After dispatching all ready tasks, enter low-power mode until next interrupt.
    TT_EnterIdleMode();
}

void TT_ISR(void) {
    WDT_Refresh(); // API_implementation_sequence: WDT_Reset() first

    // TT_ISR requirements:
    // 1. Must be called from the timer interrupt handler.
    // 2. Execution time must be minimized.
    // 3. Should not call any blocking functions.

    // Check if TIM3 Update Interrupt Flag is set and clear it
    if (REG_CHECK_BIT(TIM3_BASE_ADDR, TIM3_SR_OFFSET, TIM3_SR_UIF_POS)) {
        REG_CLEAR_BIT(TIM3_BASE_ADDR, TIM3_SR_OFFSET, TIM3_SR_UIF_POS);

        s_current_system_ticks++; // Increment global tick counter

        // Update the timing of all scheduled tasks
        for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
            if (s_tt_tasks[i].state != TT_TASK_STATE_DELETED) {
                if (s_tt_tasks[i].delay > 0) {
                    s_tt_tasks[i].delay--;
                }

                if (s_tt_tasks[i].delay == 0) {
                    s_tt_tasks[i].state = TT_TASK_STATE_READY;

                    // Reschedule periodic tasks
                    if (s_tt_tasks[i].period > 0) {
                        s_tt_tasks[i].delay = s_tt_tasks[i].period;
                    }
                    // One-shot tasks (period == 0) will remain with delay = 0
                    // and will be deleted by TT_Dispatch_task after execution.
                }
            }
        }
    }
}

tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Refresh(); // API_implementation_sequence: WDT_Reset() first

    // TT_Add_task requirements:
    // 1. Task function must be non-blocking (implicit, caller's responsibility).
    // 2. Period must be multiple of tick time (implicit as period/delay are in ticks).
    // 3. Delay must be less than maximum allowed value (tword handles max 65535 ticks).

    if (task == NULL) {
        return TT_INVALID_TASK_ID; // Cannot add a NULL task
    }

    tbyte new_task_index = TT_INVALID_TASK_ID;

    TT_ENTER_CRITICAL(); // Protect access to shared task array
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        if (s_tt_tasks[i].state == TT_TASK_STATE_DELETED) {
            s_tt_tasks[i].task_func = task;
            s_tt_tasks[i].period = period;
            s_tt_tasks[i].delay = delay;
            s_tt_tasks[i].state = TT_TASK_STATE_IDLE; // Initially idle, will be ready when delay expires
            s_tt_tasks[i].task_id = i; // Assign its own index as ID
            new_task_index = i;
            break; // Found an empty slot, exit loop
        }
    }
    TT_EXIT_CRITICAL();

    return new_task_index;
}

void TT_Delete_task(const tbyte task_index) {
    WDT_Refresh(); // API_implementation_sequence: WDT_Reset() first

    // TT_Delete_task requirements:
    // 1. Must validate task index before deletion.
    if (task_index >= TT_NUMBER_TASKS) {
        return; // Invalid task index
    }

    TT_ENTER_CRITICAL(); // Protect task state modification
    // 2. Should not affect other tasks (handled by modifying only the specified index).
    // 3. Must free all resources associated with task (conceptually, reset its slot).
    s_tt_tasks[task_index].task_func = NULL;
    s_tt_tasks[task_index].period = 0;
    s_tt_tasks[task_index].delay = 0;
    s_tt_tasks[task_index].state = TT_TASK_STATE_DELETED;
    TT_EXIT_CRITICAL();
}

// --- STM32 Specific Interrupt Handler Integration ---
// This assumes the user will map this function to the actual TIM3 interrupt.
// Example for TIM3_IRQHandler (needs to be defined in startup_stm32f401xc.s or similar):
// extern void TT_ISR(void); // Declare TT_ISR to be visible
// void TIM3_IRQHandler(void) {
//     TT_ISR();
// }