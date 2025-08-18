/**
 * @file TT.h
 * @brief Header file for the Timing & Task (TT) module.
 *
 * This module provides a simple time-triggered scheduler for the STM32F401RC MCU.
 * It uses a timer to generate periodic ticks, dispatching registered tasks based on their
 * configured periods and delays.
 */

#ifndef TT_H
#define TT_H

// Global includes as per Rules.json
#include <stdint.h>  // For standard integer types (uint8_t, uint16_t, uint32_t)
#include <stdbool.h> // For boolean type (bool)
#include <stddef.h>  // For NULL

// --- Data Types as per TT_general_rules in Rules.json ---
typedef unsigned int t_tick_time; /**< Unsigned integer representing milliseconds. */
typedef uint8_t tbyte;           /**< Unsigned 8-bit integer for task indexes. */
typedef uint16_t tword;          /**< Unsigned 16-bit integer for time values. */

// --- Constants and Macros ---
#define TT_NUMBER_TASKS (10U) /**< Maximum number of tasks the scheduler can manage. */
#define TT_INVALID_TASK_ID TT_NUMBER_TASKS /**< Return value for TT_Add_task when no slot is available. */
#define TT_MIN_TICK_TIME_MS (1U)    /**< Minimum allowed tick time in milliseconds. */
#define TT_MAX_TICK_TIME_MS (1000U) /**< Maximum allowed tick time in milliseconds. */

// Define register base addresses and offsets as per REGISTER_JSON
// Peripheral: RCC (Reset and Clock Control)
#define RCC_BASE_ADDR            ((volatile uint32_t *)0x40023800U)
#define RCC_AHB1ENR_OFFSET       (0x30U) // RCC_AHB1ENR address: 0x40023830
#define RCC_APB1ENR_OFFSET       (0x38U) // RCC_APB1ENR address: 0x40023838
#define RCC_APB2ENR_OFFSET       (0x3C) // RCC_APB2ENR address: 0x4002383C

// Peripheral: TIM3
#define TIM3_BASE_ADDR           ((volatile uint32_t *)0x40000400U)
#define TIM3_CR1_OFFSET          (0x00U) // TIM3_CR1 address: 0x40000400
#define TIM3_DIER_OFFSET         (0x0CU) // TIM3_DIER address: 0x4000040C
#define TIM3_SR_OFFSET           (0x10U) // TIM3_SR address: 0x40000410
#define TIM3_EGR_OFFSET          (0x14U) // TIM3_EGR address: 0x40000414
#define TIM3_CNT_OFFSET          (0x24U) // TIM3_CNT address: 0x40000424
#define TIM3_PSC_OFFSET          (0x28U) // TIM3_PSC address: 0x40000428
#define TIM3_ARR_OFFSET          (0x2CU) // TIM3_ARR address: 0x4000042C

// Normalized Register Access Macros
// These macros provide a generic way to read/write/modify peripheral registers
// while adhering to the "no direct peripheral access" rule (e.g., TIM1->CR1).
// The addresses are derived from the JSON and cast to volatile uint32_t pointers.
#define REG_ADDR(base, offset)    ((volatile uint32_t *)((uintptr_t)(base) + (offset)))
#define REG_READ(base, offset)    (*(REG_ADDR(base, offset)))
#define REG_WRITE(base, offset, val) (*(REG_ADDR(base, offset)) = (val))
#define REG_SET_BIT(base, offset, bit) (REG_WRITE(base, offset, REG_READ(base, offset) | (1U << (bit))))
#define REG_CLEAR_BIT(base, offset, bit) (REG_WRITE(base, offset, REG_READ(base, offset) & ~(1U << (bit))))
#define REG_CHECK_BIT(base, offset, bit) ((REG_READ(base, offset) & (1U << (bit))) != 0U)

// Bits for RCC_APB1ENR
#define RCC_APB1ENR_TIM3EN_POS (1U) // TIM3 clock enable bit

// Bits for TIM3_CR1 (Control Register 1)
#define TIM3_CR1_CEN_POS (0U) // Counter enable

// Bits for TIM3_DIER (DMA/Interrupt Enable Register)
#define TIM3_DIER_UIE_POS (0U) // Update interrupt enable

// Bits for TIM3_SR (Status Register)
#define TIM3_SR_UIF_POS (0U) // Update interrupt flag

// Bits for TIM3_EGR (Event Generation Register)
#define TIM3_EGR_UG_POS (0U) // Update generation event

// --- API Function Declarations (from TT_API.json) ---

/**
 * @brief Initializes the Timing & Task (TT) module.
 *
 * This function must be called before any other TT API function. It configures the
 * underlying hardware timer (TIM3) to generate periodic ticks at the specified
 * interval and clears all existing task slots.
 *
 * @param tick_time_ms The desired tick time in milliseconds. Must be between 1ms and 1000ms.
 */
void TT_Init(t_tick_time tick_time_ms);

/**
 * @brief Starts the TT scheduler by enabling the configured timer.
 *
 * This function should be called once after TT_Init() to begin task scheduling
 * and dispatching.
 */
void TT_Start(void);

/**
 * @brief Dispatches ready tasks from the scheduler's task list.
 *
 * This function should be called repeatedly from the main application loop.
 * It identifies tasks that are due for execution, runs them, and manages
 * the lifecycle of one-shot tasks. It also puts the system into an idle mode
 * (e.g., WFI) until the next tick.
 */
void TT_Dispatch_task(void);

/**
 * @brief Timing & Task module's Interrupt Service Routine (ISR) handler.
 *
 * This function must be called from the appropriate timer interrupt handler
 * (e.g., TIM3_IRQHandler). It updates the timing for all scheduled tasks,
 * decreasing their delay counters and marking them as ready when their delay
 * reaches zero. Periodic tasks are automatically re-scheduled.
 */
void TT_ISR(void);

/**
 * @brief Registers a new task with the scheduler.
 *
 * This function adds a new task to the scheduler's task list. The task will
 * be executed after an initial delay and then periodically at the specified period.
 *
 * @param task A function pointer to the task to be registered. The task function
 *             must be non-blocking.
 * @param period The period of the task in scheduler ticks. If 0, the task is
 *               considered a one-shot task and will be deleted after its first execution.
 * @param delay The initial delay before the task's first execution, in scheduler ticks.
 * @return The index of the added task, or TT_INVALID_TASK_ID if no free slot is available.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);

/**
 * @brief Deletes a task from the scheduler.
 *
 * This function removes a task from the scheduler's active list, preventing it
 * from being dispatched further.
 *
 * @param task_index The index of the task to be deleted, as returned by TT_Add_task().
 */
void TT_Delete_task(const tbyte task_index);

#endif // TT_H