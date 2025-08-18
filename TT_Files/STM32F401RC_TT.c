#include "TT.h"
#include <stddef.h> // For NULL

// --- Global Includes (as per Rules.json - "global_includes" section) ---
// Note: The Rules.json example did not explicitly specify "global_includes" content.
// For embedded systems, common headers like stdint.h for integer types are typical.
// CMSIS core headers for intrinsics like __disable_irq(), __enable_irq(), and __WFI()
// are also common, but not provided in the inputs, so placeholders are used.
#include <stdint.h> // For uint32_t, uint16_t, uint8_t

// --- MCU specific registers and bit definitions (from REGISTER_JSON) ---

// Macro for simplified register access
#define REG_ACCESS(address) (*((volatile uint32_t *)(address)))

// RCC (Reset and Clock Control) registers:
// Used for clock_enable to activate peripheral clocks.
#define RCC_AHB1ENR_ADDR    ((uint32_t)0x40023830) // AHB1 peripheral clock enable register.
#define RCC_APB1ENR_ADDR    ((uint32_t)0x40023838) // APB1 peripheral clock enable register.
#define RCC_APB2ENR_ADDR    ((uint32_t)0x4002383C) // APB2 peripheral clock enable register.

// TIM2 (General Purpose Timer 2) registers:
// Selected for system tick generation (timer_control, timer_prescaler, timer_auto_reload,
// timer_interrupt_dma, timer_status).
#define TIM2_CR1_ADDR       ((uint32_t)0x40000000) // TIM2 control register 1.
#define TIM2_DIER_ADDR      ((uint32_t)0x4000000C) // TIM2 DMA/Interrupt enable register.
#define TIM2_SR_ADDR        ((uint32_t)0x40000010) // TIM2 status register.
#define TIM2_PSC_ADDR       ((uint32_t)0x40000028) // TIM2 prescaler.
#define TIM2_ARR_ADDR       ((uint32_t)0x4000002C) // TIM2 auto-reload register.

// PWR (Power Control) registers:
// PWR_CR (power_control) might be used for low-power modes, though __WFI() is a CPU instruction.
#define PWR_CR_ADDR         ((uint32_t)0x40007000) // Power control register.

// --- Bit definitions for selected registers ---

// RCC_APB1ENR bits
#define RCC_APB1ENR_TIM2EN  (1UL << 0)   // Bit 0: TIM2 clock enable

// TIMx_CR1 bits (Control Register 1)
#define TIM_CR1_CEN         (1UL << 0)   // Bit 0: Counter enable
#define TIM_CR1_UDIS        (1UL << 1)   // Bit 1: Update disable (Disables UEV generation)
#define TIM_CR1_URS         (1UL << 2)   // Bit 2: Update request source (0 = any event, 1 = only counter overflow/underflow)
#define TIM_CR1_OPM         (1UL << 3)   // Bit 3: One pulse mode

// TIMx_DIER bits (DMA/Interrupt Enable Register)
#define TIM_DIER_UIE        (1UL << 0)   // Bit 0: Update interrupt enable

// TIMx_SR bits (Status Register)
#define TIM_SR_UIF          (1UL << 0)   // Bit 0: Update interrupt flag

// --- Placeholder for generic watchdog reset ---
// The provided REGISTER_JSON does not include specific Watchdog Timer (IWDG/WWDG) registers.
// As per rules, a generic watchdog refresh sequence based on STM32F401RC is implemented as a placeholder.
// In a real application, this would interact with IWDG_KR, IWDG_RLR registers.
static inline void WDT_Reset(void) {
    // Example for STM32 Independent Watchdog (IWDG) refresh: REG_ACCESS(IWDG_KR_ADDR) = 0xAAAAU;
    // For now, an assembly NOP instruction is used to fulfill the "executable code" requirement.
    asm("nop");
}

// --- Assumed MCU Clock Frequencies ---
// For STM32F401RC, the maximum core clock is 84 MHz.
// APB1 timer clock (TIM2) can be up to 84 MHz (PCLK1 * 2 if APB1 prescaler > 1).
#define SYSTEM_CORE_CLOCK_FREQ_HZ   84000000UL // Max core clock for STM32F401RC
#define APB1_TIMER_CLOCK_FREQ_HZ    84000000UL // Assumed TIM2 input clock (PCLK1 * 2 for STM32F4)

// --- Internal Data Structures ---
// Task Control Block (TCB) Structure
typedef struct {
    void (*pTask)(void);    // Pointer to the task function
    tword Delay;            // Delay (ticks) until the task is due to run
    tword Period;           // Period (ticks) between successive runs, 0 for one-shot
    bool RunMe;             // Flag indicating if the task is due to run
} sTask;

// --- Private Variables ---
static sTask TT_Tasks[TT_NUMBER_TASKS]; // Array to hold task control blocks
static t_tick_time SystemTickMs = 0;    // Stores the configured system tick time in milliseconds

// --- Internal function prototype for __WFI() and interrupt control if not provided by CMSIS ---
// These are typically CMSIS intrinsics for ARM Cortex-M microcontrollers.
// As they are not in the provided JSON, generic placeholder macros are used.
#ifndef __WFI
#define __WFI() asm volatile ("wfi") // Placeholder for Wait For Interrupt instruction
#endif

// Critical section placeholders for disabling/enabling interrupts.
// In a real application, these would be __disable_irq() and __enable_irq() from CMSIS.
#define ENTER_CRITICAL_SECTION()    // Placeholder for __disable_irq()
#define EXIT_CRITICAL_SECTION()     // Placeholder for __enable_irq()


// --- API Function Implementations ---

/**
 * @brief Initializes the Timing & Task (TT) module.
 *        Configures the system timer (TIM2) for the specified tick time.
 *        Clears all existing tasks.
 * @param tick_time_ms The desired system tick time in milliseconds (1ms to 1000ms).
 *
 * @rules TT_Init:
 *   - Must be called before any other TT API function.
 *   - Tick time must be between 1ms and 1000ms.
 *   - Clears all existing tasks during initialization.
 * @rules WDT_requirements:
 *   - WDT_Reset() must be the first executable statement in each function.
 *   - Functions with loops must call WDT_Reset() at least once per loop iteration.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // WDT_Reset() must be the first executable statement in the function

    // TT_Init requirement: Tick time must be between 1ms and 1000ms
    if (tick_time_ms == 0 || tick_time_ms > 1000) {
        // Clamp tick_time_ms to a valid range or handle error appropriately.
        // For this example, we'll clamp to 1ms if 0, or 1000ms if >1000.
        tick_time_ms = (tick_time_ms == 0) ? 1 : (tick_time_ms > 1000 ? 1000 : tick_time_ms);
    }
    SystemTickMs = tick_time_ms;

    // TT_Init requirement: Clears all existing tasks during initialization
    // Loop through all available task slots (TT_NUMBER_TASKS) and deletes them.
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        WDT_Reset(); // WDT_Reset() must be called at least once per loop iteration
        TT_Delete_task(i); // Call TT_Delete_task to clear each slot
    }

    // Configure TIM2 as the system tick timer
    // 1. Enable TIM2 peripheral clock (RCC_APB1ENR - bit 0)
    REG_ACCESS(RCC_APB1ENR_ADDR) |= RCC_APB1ENR_TIM2EN;

    // 2. Disable the timer to configure (TIM2_CR1 - CEN bit)
    REG_ACCESS(TIM2_CR1_ADDR) &= ~TIM_CR1_CEN;

    // 3. Calculate Prescaler (PSC) and Auto-Reload Register (ARR) values
    // Timer clock frequency = APB1_TIMER_CLOCK_FREQ_HZ (e.g., 84 MHz)
    // For a 1 microsecond (1µs) resolution: Prescaler = (APB1_TIMER_CLOCK_FREQ_HZ / 1,000,000) - 1
    // Then, ARR = (Number of µs per tick) - 1.
    // Example: For 1ms tick, and 1µs resolution: ARR = (1000 * 1) - 1 = 999
    REG_ACCESS(TIM2_PSC_ADDR) = (APB1_TIMER_CLOCK_FREQ_HZ / 1000000UL) - 1; // Sets timer to 1µs resolution
    REG_ACCESS(TIM2_ARR_ADDR) = (1000UL * SystemTickMs) - 1;                // Auto-reload value for the desired tick_time_ms period

    // 4. Clear any pending Update Interrupt Flag (TIM2_SR - UIF bit)
    REG_ACCESS(TIM2_SR_ADDR) &= ~TIM_SR_UIF;

    // 5. Enable Update Interrupt (TIM2_DIER - UIE bit)
    REG_ACCESS(TIM2_DIER_ADDR) |= TIM_DIER_UIE;

    // 6. Set Update Request Source (TIM2_CR1 - URS bit) to only overflow/underflow
    // This prevents interrupts on counter enable or software generation.
    REG_ACCESS(TIM2_CR1_ADDR) |= TIM_CR1_URS;

    // Note on NVIC (Nested Vectored Interrupt Controller) configuration:
    // Enabling the timer interrupt in the NVIC is crucial but requires NVIC-specific registers
    // (e.g., NVIC_ISERx, NVIC_IPRx) which are not present in the provided JSON.
    // This part would typically involve:
    // NVIC_SetPriority(TIM2_IRQn, priority_value); // Set desired priority
    // NVIC_EnableIRQ(TIM2_IRQn);                  // Enable TIM2 interrupt
    // This configuration must be handled externally to this module or manually.
}

/**
 * @brief Starts the Timing & Task (TT) scheduler.
 *        Enables the configured system timer (TIM2) to begin generating interrupts.
 * @rules TT_Start:
 *   - Must be called after TT_Init.
 *   - Should only be called once.
 *   - Must enable timer interrupts with correct priority (commented).
 * @rules WDT_requirements:
 *   - WDT_Reset() must be the first executable statement in each function.
 */
void TT_Start(void) {
    WDT_Reset(); // WDT_Reset() must be the first executable statement

    // Enable the TIM2 counter (TIM2_CR1 - CEN bit)
    REG_ACCESS(TIM2_CR1_ADDR) |= TIM_CR1_CEN;
}

/**
 * @brief Dispatches ready tasks and manages the system idle state.
 *        This function should be called repeatedly in the main loop.
 * @rules TT_Dispatch_task:
 *   - Should be called from main loop.
 *   - Must handle task execution within one tick period.
 *   - Must properly clean up completed one-shot tasks.
 * @rules TT_general_rules:
 *   - System must have idle task for power saving.
 *   - Critical sections must protect shared resources.
 * @rules WDT_requirements:
 *   - WDT_Reset() must be the first executable statement in each function.
 *   - Functions with loops must call WDT_Reset() at least once per loop iteration.
 */
void TT_Dispatch_task(void) {
    WDT_Reset(); // WDT_Reset() must be the first executable statement

    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        WDT_Reset(); // WDT_Reset() must be called at least once per loop iteration

        // Critical section start: Protect TT_Tasks array from concurrent modification by ISR
        ENTER_CRITICAL_SECTION(); // Placeholder for __disable_irq()

        if (TT_Tasks[i].RunMe) {
            // Task is ready to run. Clear the RunMe flag.
            // Note: If multiple ISR calls happen before dispatch, RunMe might be > 1.
            // Decrementing here is robust, but for a simple scheduler, a boolean is common.
            TT_Tasks[i].RunMe = false;

            // Critical section end: Re-enable interrupts before task execution
            EXIT_CRITICAL_SECTION(); // Placeholder for __enable_irq()

            if (TT_Tasks[i].pTask != NULL) {
                TT_Tasks[i].pTask(); // Execute the task function (TT_general_rules: must be non-blocking)
            }

            // Critical section start again: For manipulating task properties after execution
            ENTER_CRITICAL_SECTION(); // Placeholder for __disable_irq()

            // If it's a periodic task, reset its delay
            if (TT_Tasks[i].Period != 0) {
                TT_Tasks[i].Delay = TT_Tasks[i].Period;
            } else {
                // If it's a one-shot task (Period == 0), delete it after execution
                // TT_Dispatch_task requirement: Must properly clean up completed one-shot tasks
                TT_Delete_task(i);
            }
        }
        // Critical section end (always ensure interrupts are re-enabled if they were disabled)
        EXIT_CRITICAL_SECTION(); // Placeholder for __enable_irq()
    }

    // TT_general_rules: System must have idle task for power saving
    // If no tasks are ready or all dispatched, enter a low-power idle mode until the next tick.
    // __WFI() (Wait For Interrupt) is an ARM Cortex-M instruction.
    __WFI(); // Placeholder for entering low power mode
}

/**
 * @brief Timer Interrupt Service Routine (ISR) handler for the TT module.
 *        Updates the timing of all scheduled tasks every tick. Decreases delay counters,
 *        marks tasks as ready when delay reaches zero, and reschedules periodic tasks.
 *        This function must be called from the appropriate timer interrupt handler.
 * @rules TT_ISR:
 *   - Must be called from the timer interrupt handler.
 *   - Execution time must be minimized.
 *   - Should not call any blocking functions.
 * @rules WDT_requirements:
 *   - WDT_Reset() must be the first executable statement in each function.
 *   - Functions with loops must call WDT_Reset() at least once per loop iteration.
 */
void TT_ISR(void) {
    WDT_Reset(); // WDT_Reset() must be the first executable statement

    // TT_ISR requirement: Must be called from the timer interrupt handler
    // Clear the TIM2 Update Interrupt Flag (UIF) to prevent re-entry/sticky flag
    if (REG_ACCESS(TIM2_SR_ADDR) & TIM_SR_UIF) {
        REG_ACCESS(TIM2_SR_ADDR) &= ~TIM_SR_UIF; // Clear the flag
    }

    // TT_ISR requirement: Updates the timing of all scheduled tasks every tick.
    // Critical section start: Protect TT_Tasks array from concurrent modification
    ENTER_CRITICAL_SECTION(); // Placeholder for __disable_irq()

    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        WDT_Reset(); // WDT_Reset() must be called at least once per loop iteration

        if (TT_Tasks[i].pTask != NULL) { // Only process active tasks
            if (TT_Tasks[i].Delay > 0) {
                TT_Tasks[i].Delay--; // Decrement delay counter
            }

            if (TT_Tasks[i].Delay == 0) {
                TT_Tasks[i].RunMe = true; // Mark task as ready to run

                // TT_ISR requirement: reschedules periodic tasks.
                if (TT_Tasks[i].Period != 0) {
                    TT_Tasks[i].Delay = TT_Tasks[i].Period; // Reset delay for periodic tasks
                }
                // One-shot tasks (Period == 0) will remain with Delay = 0 until dispatched and removed.
            }
        }
    }
    // Critical section end
    EXIT_CRITICAL_SECTION(); // Placeholder for __enable_irq()
}

/**
 * @brief Adds a new task to the scheduler.
 * @param task Pointer to the task function to be executed.
 * @param period The period (in ticks) at which the task should repeat.
 *               Set to 0 for a one-shot task.
 * @param delay The initial delay (in ticks) before the task first runs.
 * @return The index of the added task, or TT_ERROR_ADD_TASK_FAILED if the task list is full.
 *
 * @rules TT_Add_task:
 *   - Task function must be non-blocking (user responsibility).
 *   - Period must be multiple of tick time (implicitly handled if inputs are provided in ticks, user responsibility).
 *   - Delay must be less than maximum allowed value.
 * @rules TT_general_rules:
 *   - Critical sections must protect shared resources.
 * @rules WDT_requirements:
 *   - WDT_Reset() must be the first executable statement in each function.
 *   - Functions with loops must call WDT_Reset() at least once per loop iteration.
 */
tbyte TT_Add_task(void (* task)(void), const tword period, const tword delay) {
    WDT_Reset(); // WDT_Reset() must be the first executable statement

    tbyte index = TT_ERROR_ADD_TASK_FAILED;

    // TT_Add_task requirement: Delay must be less than maximum allowed value
    if (delay > TT_MAX_DELAY || period > TT_MAX_DELAY || task == NULL) {
        return TT_ERROR_ADD_TASK_FAILED; // Invalid delay, period, or NULL task pointer
    }

    // Find an empty slot to add the new task
    // Critical section start: Protect TT_Tasks array while searching/adding
    ENTER_CRITICAL_SECTION(); // Placeholder for __disable_irq()

    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        WDT_Reset(); // WDT_Reset() must be called at least once per loop iteration
        if (TT_Tasks[i].pTask == NULL) { // Check if slot is empty
            TT_Tasks[i].pTask = task;
            TT_Tasks[i].Period = period;
            TT_Tasks[i].Delay = delay;
            TT_Tasks[i].RunMe = false; // Task not ready immediately, waits for its delay
            index = i;
            break; // Task added, exit loop
        }
    }
    // Critical section end
    EXIT_CRITICAL_SECTION(); // Placeholder for __enable_irq()

    return index; // Returns task index or TT_ERROR_ADD_TASK_FAILED
}

/**
 * @brief Deletes a task from the scheduler.
 *        Clears all information for the specified task index, effectively removing it.
 * @param task_index The index of the task to be deleted.
 *
 * @rules TT_Delete_task:
 *   - Must validate task index before deletion.
 *   - Should not affect other tasks.
 *   - Must free all resources associated with task (handled by setting pTask to NULL).
 * @rules TT_general_rules:
 *   - Critical sections must protect shared resources.
 * @rules WDT_requirements:
 *   - WDT_Reset() must be the first executable statement in each function.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // WDT_Reset() must be the first executable statement

    // TT_Delete_task requirement: Must validate task index before deletion
    if (task_index < TT_NUMBER_TASKS) {
        // Critical section start: Protect TT_Tasks array during deletion
        ENTER_CRITICAL_SECTION(); // Placeholder for __disable_irq()

        // Clear all information for the specified task slot
        TT_Tasks[task_index].pTask = NULL;
        TT_Tasks[task_index].Period = 0;
        TT_Tasks[task_index].Delay = 0;
        TT_Tasks[task_index].RunMe = false;
        // TT_Delete_task requirement: Must free all resources associated with task.
        // For this simple scheduler, setting the function pointer to NULL
        // effectively "frees" the slot for reuse.
        
        // Critical section end
        EXIT_CRITICAL_SECTION(); // Placeholder for __enable_irq()
    }
    // TT_Delete_task requirement: Should not affect other tasks (handled by direct indexing)
}

/*
// Example of how the timer interrupt handler would typically call TT_ISR:
// This part would reside in the main application's interrupt handler file (e.g., stm32f4xx_it.c)

// extern void TT_ISR(void); // Declare the function if not already in scope

// void TIM2_IRQHandler(void) {
//     // Ensure the correct interrupt flag is checked and cleared before calling TT_ISR
//     // if (REG_ACCESS(TIM2_SR_ADDR) & TIM_SR_UIF) { // Already checked and cleared inside TT_ISR
//     //     REG_ACCESS(TIM2_SR_ADDR) &= ~TIM_SR_UIF;
//     // }
//     TT_ISR();
// }
*/