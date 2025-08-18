#include "TT.h"
#include <stddef.h> // For NULL

// Define register addresses and access macros based on the provided JSON.
// Using volatile uint32_t* for direct memory-mapped register access.

// RCC Registers
#define RCC_APB1ENR_ADDR    0x40023838U
#define RCC_APB1ENR         (*((volatile uint32_t *)RCC_APB1ENR_ADDR))

// TIM2 Registers (chosen for 32-bit counter, suitable for scheduling)
#define TIM2_CR1_ADDR       0x40000000U
#define TIM2_CR1            (*((volatile uint32_t *)TIM2_CR1_ADDR))
#define TIM2_DIER_ADDR      0x4000000CU
#define TIM2_DIER           (*((volatile uint32_t *)TIM2_DIER_ADDR))
#define TIM2_SR_ADDR        0x40000010U
#define TIM2_SR             (*((volatile uint32_t *)TIM2_SR_ADDR))
#define TIM2_EGR_ADDR       0x40000014U
#define TIM2_EGR            (*((volatile uint32_t *)TIM2_EGR_ADDR))
#define TIM2_CNT_ADDR       0x40000024U
#define TIM2_CNT            (*((volatile uint32_t *)TIM2_CNT_ADDR))
#define TIM2_PSC_ADDR       0x40000028U
#define TIM2_PSC            (*((volatile uint32_t *)TIM2_PSC_ADDR))
#define TIM2_ARR_ADDR       0x4000002CU
#define TIM2_ARR            (*((volatile uint32_t *)TIM2_ARR_ADDR))

// Bit definitions for TIM2 registers
#define TIM_CR1_CEN_Pos         (0U)
#define TIM_CR1_CEN_Msk         (0x1U << TIM_CR1_CEN_Pos)
#define TIM_CR1_CEN             TIM_CR1_CEN_Msk

#define TIM_DIER_UIE_Pos        (0U)
#define TIM_DIER_UIE_Msk        (0x1U << TIM_DIER_UIE_Pos)
#define TIM_DIER_UIE            TIM_DIER_UIE_Msk

#define TIM_SR_UIF_Pos          (0U)
#define TIM_SR_UIF_Msk          (0x1U << TIM_SR_UIF_Pos)
#define TIM_SR_UIF              TIM_SR_UIF_Msk

#define RCC_APB1ENR_TIM2EN_Pos  (0U)
#define RCC_APB1ENR_TIM2EN_Msk  (0x1U << RCC_APB1ENR_TIM2EN_Pos)
#define RCC_APB1ENR_TIM2EN      RCC_APB1ENR_TIM2EN_Msk

// IWDG (Independent Watchdog) Registers - Not present in provided JSON,
// but required by the rule "implement a generic watchdog refresh sequence based on STM32F401RC"
// when WDT-related registers are missing from the input.
#define IWDG_BASE               0x40003000U
#define IWDG_KR_ADDR            (IWDG_BASE + 0x00U) // Key Register
#define IWDG_PR_ADDR            (IWDG_BASE + 0x04U) // Prescaler Register
#define IWDG_RLR_ADDR           (IWDG_BASE + 0x08U) // Reload Register

#define IWDG_KR                 (*((volatile uint32_t *)IWDG_KR_ADDR))
#define IWDG_PR                 (*((volatile uint32_t *)IWDG_PR_ADDR))
#define IWDG_RLR                (*((volatile uint32_t *)IWDG_RLR_ADDR))

// IWDG Key Register values
#define IWDG_KEY_ENABLE         0xCCCCU // Enable IWDG
#define IWDG_KEY_REFRESH        0xAAAAU // Reload IWDG counter with RLR value
#define IWDG_KEY_ACCESS         0x5555U // Enable write access to PR and RLR registers

// IWDG Prescaler Register values (examples)
#define IWDG_PR_DIV256          0x6U    // Prescaler /256

// IWDG Reload Register values
#define IWDG_RLR_MAX            0xFFFU  // Max reload value

// Global variables
TT_Task_t TT_Task_Buffer[TT_NUMBER_TASKS];
static t_tick_time TT_Current_Tick_Time_ms = 0; // Stores the configured tick time in ms

// Forward declaration for the WDT_Reset helper function
static void WDT_Reset(void);

/**
 * @brief Generic watchdog refresh sequence based on STM32F401RC.
 * This function will be called at the beginning of every public API function.
 * Note: The IWDG registers were not provided in the input JSON. Their definitions
 * are derived from standard STM32F401RC documentation to fulfill the requirement
 * of implementing a "generic watchdog refresh sequence based on STM32F401RC"
 * when WDT registers are missing from the input.
 */
static void WDT_Reset(void) {
    // Write access to PR and RLR registers is enabled by writing 0x5555 in IWDG_KR
    IWDG_KR = IWDG_KEY_ACCESS;
    // Set prescaler to /256 (for example, adjust as needed for desired timeout)
    IWDG_PR = IWDG_PR_DIV256;
    // Set reload value to maximum (for example, adjust as needed for desired timeout)
    IWDG_RLR = IWDG_RLR_MAX;
    // Reload the watchdog counter with the RLR value by writing 0xAAAA in IWDG_KR
    IWDG_KR = IWDG_KEY_REFRESH;
}

/**
 * @brief Initializes the Timer with the given tick time (Tick_Time_ms).
 * This defines the basic time unit for the scheduler.
 * It loops through all available task slots (TT_NUMBER_TASKS) and deletes them.
 *
 * @param tick_time_ms The desired tick time in milliseconds (1ms to 1000ms).
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // API_implementation_sequence

    // Validate tick_time_ms
    if (tick_time_ms < 1 || tick_time_ms > 1000) {
        // Handle error, e.g., set to a default value or return.
        // For this implementation, we'll default to 1ms if out of range.
        tick_time_ms = 1;
    }
    TT_Current_Tick_Time_ms = tick_time_ms;

    // Clear all existing tasks during initialization
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        // No need for WDT_Reset here as it's handled by TT_Delete_task
        TT_Delete_task(i);
    }

    // Configure TIM2 for desired tick_time_ms
    // Assume APB1 Timer Clock (PCLK1) is 42 MHz for STM32F401RC
    // The timer clock frequency is PCLK1, unless APB1 prescaler is /1, then it's 2*PCLK1.
    // Assuming 42MHz for simplicity.
    const uint32_t timer_clock_freq = 42000000U; // Hz

    // For a 1ms tick: PSC = (42MHz / 1MHz) - 1 = 41. ARR = (1ms * 1000) - 1 = 999.
    // Counter runs at 1 tick per microsecond. ARR specifies 1ms.
    // Calculation: (timer_clock_freq / (PSC + 1)) = target_freq_Hz
    // target_freq_Hz = 1 / (tick_time_ms / 1000) = 1000 / tick_time_ms
    // (PSC + 1) = timer_clock_freq * tick_time_ms / 1000
    // PSC = (timer_clock_freq * tick_time_ms / 1000) - 1
    // ARR = (number of target ticks for 1 ms) - 1 = (1000) - 1 = 999.
    // If PSC makes it count in milliseconds: PSC = (timer_clock_freq / 1000) - 1
    // Then ARR = tick_time_ms - 1

    // Strategy: Make the counter tick at 1MHz (1 tick = 1 microsecond)
    uint32_t prescaler_value = (timer_clock_freq / 1000000U) - 1; // Counter ticks every 1 us
    uint32_t auto_reload_value = (TT_Current_Tick_Time_ms * 1000U) - 1; // Number of 1us ticks for TT_Current_Tick_Time_ms

    // Enable TIM2 clock in RCC
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Disable TIM2 before configuration
    TIM2_CR1 &= ~TIM_CR1_CEN;

    // Set prescaler and auto-reload values
    TIM2_PSC = prescaler_value;
    TIM2_ARR = auto_reload_value;

    // Clear counter
    TIM2_CNT = 0;

    // Generate an update event to apply new PSC and ARR values
    TIM2_EGR = 0x1U; // UG bit (Update Generation)

    // Enable TIM2 Update Interrupt
    TIM2_DIER |= TIM_DIER_UIE;

    // Clear pending TIM2 Update Interrupt Flag
    TIM2_SR = ~TIM_SR_UIF; // Write 0 to clear UIF (some architectures write 1, STM32 is write 0 for some bits, write 1 to clear for others. SR bits are cleared by writing 0 to relevant bit for TIMx_SR)
    // Correct way to clear UIF: Write 0 to the bit in the register, or 1 to the corresponding bit in the Flag Clear Register (FCR) if it exists.
    // For STM32 TIMx_SR, write 0 to clear (or 1 depending on the bit type, UIF is 'RC_W0' in many cases meaning Read-Clear by writing 0).
    // Let's assume writing 0 to the bit works, or more safely, use explicit bit clear.
    // To clear UIF flag:
    TIM2_SR = (uint32_t)~TIM_SR_UIF; // Clear UIF by writing 0 to the bit.

    // Note: Enabling the TIM2 interrupt in the NVIC (Nested Vectored Interrupt Controller)
    // is typically done using CMSIS functions (e.g., `NVIC_EnableIRQ(TIM2_IRQn);`)
    // or direct register access to the NVIC ISER registers. These are not part of the
    // provided 'register_json' and thus cannot be implemented directly within this constraint.
    // This step must be handled externally.
}

/**
 * @brief Enables the timer, starting scheduler tick interrupts for task scheduling and dispatching.
 * Must be called after TT_Init. Should only be called once.
 */
void TT_Start(void) {
    WDT_Reset(); // API_implementation_sequence

    // Enable TIM2 counter
    TIM2_CR1 |= TIM_CR1_CEN;

    // Note: Enabling global interrupts for TIM2 (e.g., via NVIC) is
    // an external dependency not covered by the provided register JSON.
    // This function assumes the necessary NVIC configuration has been done.
}

/**
 * @brief Scans the task list, runs ready tasks, cleans up one-shot tasks,
 * and puts the system into idle mode until the next tick.
 * Should be called from the main loop.
 */
void TT_Dispatch_task(void) {
    WDT_Reset(); // API_implementation_sequence

    uint8_t tasks_run_this_cycle = 0;

    // Loop through all tasks
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        // Check if task is due to run
        // Note: Critical section needed here to protect TT_Task_Buffer
        // from concurrent access by TT_ISR. As per instructions, __disable_irq()
        // and similar intrinsics are not from provided input, so they are omitted.
        // An external mechanism for critical sections (e.g., specific CPU register manipulation
        // not in provided JSON, or compiler intrinsics) would be required.
        if (TT_Task_Buffer[i].run_me > 0) {
            TT_Task_Buffer[i].task_func(); // Execute the task
            TT_Task_Buffer[i].run_me--;    // Decrement run_me counter (for multiple executions per tick, if applicable, though typically it's 0 or 1)

            // If it's a one-shot task and it has just run
            if (TT_Task_Buffer[i].period == 0) {
                TT_Delete_task(i); // Clean up completed one-shot tasks
            }
            tasks_run_this_cycle++;
        }
    }

    // Enter idle mode for power saving if no tasks were run this cycle
    // and if the system is not actively busy.
    // __WFI() is a common instruction for entering sleep mode until an interrupt occurs.
    // It's a CPU instruction, not a peripheral register, and thus its use does not
    // violate the "do not invent registers" rule.
    if (tasks_run_this_cycle == 0) {
        // __WFI() (Wait For Interrupt) is a common Cortex-M instruction
        // for low-power sleep mode. Assumes CMSIS is available or
        // equivalent assembly/intrinsic.
        // For strict adherence to "use only inputs below", this should ideally
        // be tied to a PWR register if available for entering a specific mode.
        // Since PWR_CR is described as "Power control register.", it *could*
        // be used to set low-power modes before __WFI().
        // However, a generic __WFI() is standard practice for idle.
        // Setting low-power bits in PWR_CR is beyond the scope of a generic __WFI.
        // So, we just use __WFI().
        asm volatile ("wfi");
    }
}

/**
 * @brief Updates the timing of all scheduled tasks every tick.
 * Decreases delay counters, marks tasks as ready when delay reaches zero,
 * and reschedules periodic tasks.
 * Must be called from the timer interrupt handler (e.g., TIM2_IRQHandler).
 */
void TT_ISR(void) {
    WDT_Reset(); // API_implementation_sequence

    // Check if the update interrupt flag is set and clear it
    if (TIM2_SR & TIM_SR_UIF) {
        TIM2_SR = (uint32_t)~TIM_SR_UIF; // Clear UIF by writing 0 to the bit

        // Note: Critical section needed here to protect TT_Task_Buffer
        // from concurrent access by TT_Dispatch_task. As per instructions, __disable_irq()
        // and similar intrinsics are not from provided input, so they are omitted.
        // An external mechanism for critical sections would be required.

        for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
            if (TT_Task_Buffer[i].task_func != NULL) { // Check if slot is active
                if (TT_Task_Buffer[i].delay == 0) {
                    TT_Task_Buffer[i].run_me++; // Mark task as ready to run

                    if (TT_Task_Buffer[i].period != 0) {
                        // For periodic tasks, reload delay
                        TT_Task_Buffer[i].delay = TT_Task_Buffer[i].period;
                    }
                    // For one-shot tasks (period == 0), delay remains 0 after first run
                    // and will be deleted by TT_Dispatch_task.
                } else {
                    TT_Task_Buffer[i].delay--; // Decrement delay for next tick
                }
            }
        }
    }
}

/**
 * @brief Registers a new task with a function pointer, initial delay, and period.
 *
 * @param task Function pointer to the task to be added.
 * @param period Period for the task in ticks. 0 for a one-shot task.
 * @param delay Initial delay before the task first runs in ticks.
 * @return The index of the added task, or TT_NUMBER_TASKS if no free slot.
 */
tbyte TT_Add_task(void (* task) (void), const tword period, const tword delay) {
    WDT_Reset(); // API_implementation_sequence

    // Task function must be non-blocking - this is a design rule for the user, not checked here.
    // Period must be multiple of tick time - implicitly handled by using 'ticks'.
    //   If 'period' refers to milliseconds, then it should be checked against TT_Current_Tick_Time_ms.
    //   Assuming 'period' and 'delay' are already in units of 'ticks'.
    // Delay must be less than maximum allowed value (tword max value is 65535, which is implicit).

    // Note: Critical section needed here to protect TT_Task_Buffer.
    // As per instructions, __disable_irq() and similar intrinsics are not from provided input,
    // so they are omitted. An external mechanism for critical sections would be required.

    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++) {
        if (TT_Task_Buffer[i].task_func == NULL) { // Found an empty slot
            TT_Task_Buffer[i].task_func = task;
            TT_Task_Buffer[i].period = period;
            TT_Task_Buffer[i].delay = delay;
            TT_Task_Buffer[i].run_me = 0; // Initialize run_me to 0, will be set by ISR

            // If delay is 0, set run_me immediately so it runs on the first dispatch
            // (or next tick's ISR will set it)
            if (delay == 0) {
                TT_Task_Buffer[i].run_me = 1;
            }
            return i;
        }
    }
    return TT_ERROR; // No free slot
}

/**
 * @brief Clears all information for the specified task index,
 * effectively removing it from the scheduler.
 *
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // API_implementation_sequence

    // Validate task index
    if (task_index < TT_NUMBER_TASKS) {
        // Note: Critical section needed here to protect TT_Task_Buffer.
        // As per instructions, __disable_irq() and similar intrinsics are not from provided input,
        // so they are omitted. An external mechanism for critical sections would be required.

        // Clear all resources associated with task (by setting function pointer to NULL)
        TT_Task_Buffer[task_index].task_func = NULL;
        TT_Task_Buffer[task_index].delay = 0;
        TT_Task_Buffer[task_index].period = 0;
        TT_Task_Buffer[task_index].run_me = 0;
        // This ensures the slot is marked as free and will not affect other tasks.
    }
}

// STM32F401RC specific interrupt handler for TIM2
// This function needs to be linked to the TIM2 interrupt vector.
// Typically defined as void TIM2_IRQHandler(void) in startup files.
// Assuming this is the entry point for the TIM2 update interrupt.
void TIM2_IRQHandler(void) __attribute__((interrupt)); // For bare-metal context
void TIM2_IRQHandler(void) {
    TT_ISR(); // Call the generic scheduler ISR
}