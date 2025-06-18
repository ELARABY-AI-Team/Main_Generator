/**
 * @file main.h
 * @brief Main header file for STM32F407 embedded project.
 * @author Embedded C Engineer
 * @device STM32F407
 * @creation date 2025-06-17
 * @copyright Copyright (c) 2025, All Rights Reserved.
 */

#ifndef STM32F407_MAIN_H_
#define STM32F407_MAIN_H_

/* --- Standard C Includes ------------------------------------------------ */
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <iso646.h>
#include <limits.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --- MCU Specific Includes ------------------------------------------------ */
// Include the appropriate device header for STM32F407.
// This header provides access to peripheral registers and core functions (CMSIS).
#include "stm32f4xx.h"

/* --- Useful Typedefs ------------------------------------------------------ */
/** @brief Unsigned 8-bit type */
typedef uint8_t tbyte;
/** @brief Unsigned 16-bit type */
typedef uint16_t tword;
/** @brief Unsigned 32-bit type */
typedef uint32_t tlong;

/* --- Core Macros ---------------------------------------------------------- */
/** @brief Sets a specific bit in a register */
#define SET_BIT(reg, bit)     ((reg) |= (1UL << (bit)))
/** @brief Clears a specific bit in a register */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1UL << (bit)))
/** @brief Gets the value of a specific bit in a register */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1UL)
/** @brief Toggles a specific bit in a register */
#define TOG_BIT(reg, bit)     ((reg) ^= (1UL << (bit)))

/** @brief Disable Global Interrupts (using CMSIS intrinsic) */
#define DI                    __disable_irq
/** @brief Enable Global Interrupts (using CMSIS intrinsic) */
#define EI                    __enable_irq

/** @brief Execute No Operation (using CMSIS intrinsic) */
#define NOP                   __NOP

/** @brief Enter Wait For Interrupt state (low power mode, using CMSIS intrinsic) */
#define HALT                  __WFI

/* --- Safeguard Macros ----------------------------------------------------- */

/**
 * @brief Configures all GPIO pins to a safe default state (Input, No Pull-up/down).
 * This function disables peripheral clocks to GPIOs first (for safety/reset)
 * then configures mode, speed, pull-up/down, output type, and alternate functions.
 *
 * This should be called early in initialization.
 */
#define GPIO_SAFEGUARD_Init()                                               \
    do {                                                                    \
        /* Ensure GPIO clocks are enabled before access */                  \
        /* Clearing AHB1ENR later in Registers_SAFEGUARD_Init will disable them again */ \
        /* but enabling here allows configuring the registers */            \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN_Pos);                      \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOIEN_Pos);                      \
                                                                            \
        /* Configure all GPIO ports to Input mode, No Pull-up/down, etc. */ \
        /* Writing 0x00000000 to these registers achieves this safe state */ \
        GPIOA->MODER = 0x00000000;                                          \
        GPIOA->OSPEEDR = 0x00000000;                                        \
        GPIOA->PUPDR = 0x00000000;                                          \
        GPIOA->OTYPER = 0x00000000;                                         \
        GPIOA->AFR[0] = 0x00000000;                                         \
        GPIOA->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOB->MODER = 0x00000000;                                          \
        GPIOB->OSPEEDR = 0x00000000;                                        \
        GPIOB->PUPDR = 0x00000000;                                          \
        GPIOB->OTYPER = 0x00000000;                                         \
        GPIOB->AFR[0] = 0x00000000;                                         \
        GPIOB->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOC->MODER = 0x00000000;                                          \
        GPIOC->OSPEEDR = 0x00000000;                                        \
        GPIOC->PUPDR = 0x00000000;                                          \
        GPIOC->OTYPER = 0x00000000;                                         \
        GPIOC->AFR[0] = 0x00000000;                                         \
        GPIOC->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOD->MODER = 0x00000000;                                          \
        GPIOD->OSPEEDR = 0x00000000;                                        \
        GPIOD->PUPDR = 0x00000000;                                          \
        GPIOD->OTYPER = 0x00000000;                                         \
        GPIOD->AFR[0] = 0x00000000;                                         \
        GPIOD->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOE->MODER = 0x00000000;                                          \
        GPIOE->OSPEEDR = 0x00000000;                                        \
        GPIOE->PUPDR = 0x00000000;                                          \
        GPIOE->OTYPER = 0x00000000;                                         \
        GPIOE->AFR[0] = 0x00000000;                                         \
        GPIOE->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOF->MODER = 0x00000000;                                          \
        GPIOF->OSPEEDR = 0x00000000;                                        \
        GPIOF->PUPDR = 0x00000000;                                          \
        GPIOF->OTYPER = 0x00000000;                                         \
        GPIOF->AFR[0] = 0x00000000;                                         \
        GPIOF->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOG->MODER = 0x00000000;                                          \
        GPIOG->OSPEEDR = 0x00000000;                                        \
        GPIOG->PUPDR = 0x00000000;                                          \
        GPIOG->OTYPER = 0x00000000;                                         \
        GPIOG->AFR[0] = 0x00000000;                                         \
        GPIOG->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOH->MODER = 0x00000000;                                          \
        GPIOH->OSPEEDR = 0x00000000;                                        \
        GPIOH->PUPDR = 0x00000000;                                          \
        GPIOH->OTYPER = 0x00000000;                                         \
        GPIOH->AFR[0] = 0x00000000;                                         \
        GPIOH->AFR[1] = 0x00000000;                                         \
                                                                            \
        GPIOI->MODER = 0x00000000;                                          \
        GPIOI->OSPEEDR = 0x00000000;                                        \
        GPIOI->PUPDR = 0x00000000;                                          \
        GPIOI->OTYPER = 0x00000000;                                         \
        GPIOI->AFR[0] = 0x00000000;                                         \
        GPIOI->AFR[1] = 0x00000000;                                         \
                                                                            \
        /* Disable clocks to GPIOs again if not explicitly needed */        \
        /* This saves power and complements Registers_SAFEGUARD_Init */     \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN_Pos);                      \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOIEN_Pos);                      \
                                                                            \
    } while(0)

/**
 * @brief Disables clocks to most peripherals in RCC to ensure a minimal and safe state.
 * This effectively turns off most peripherals by stopping their clock signals.
 * Flash, SRAM, and the System Configuration Controller (SYSCFG) clocks are
 * typically enabled by default after reset or are essential, and are left running.
 * AHB1/AHB2/AHB3/APB1/APB2 peripheral clocks are disabled by zeroing the enable registers.
 *
 * This should be called early in initialization.
 */
#define Registers_SAFEGUARD_Init()                                          \
    do {                                                                    \
        /* Disable all AHB1 peripheral clocks EXCEPT essential ones like Flash/SRAM */ \
        /* By default, clearing the register is safest */                   \
        RCC->AHB1ENR = 0x00000000;                                          \
                                                                            \
        /* Disable all AHB2 peripheral clocks */                            \
        RCC->AHB2ENR = 0x00000000;                                          \
                                                                            \
        /* Disable all AHB3 peripheral clocks */                            \
        RCC->AHB3ENR = 0x00000000;                                          \
                                                                            \
        /* Disable all APB1 peripheral clocks */                            \
        RCC->APB1ENR = 0x00000000;                                          \
                                                                            \
        /* Disable all APB2 peripheral clocks */                            \
        /* Note: SYSCFG clock (bit 14) is often needed for external interrupts */ \
        /* Clearing the whole register is safer per the requirement to disable *unused* */ \
        /* peripherals. Enable SYSCFG later if needed. */                   \
        RCC->APB2ENR = 0x00000000;                                          \
                                                                            \
        /* Optional: Wait for peripheral clocks to stop? Usually not necessary */ \
        /* as they stop within a few clock cycles. */                       \
                                                                            \
    } while(0)


/* --- Function Prototypes -------------------------------------------------- */
// Add main application function prototypes here if needed.
// Example:
// void SystemClock_Config(void);
// void MX_GPIO_Init(void);

#endif /* MAIN_H */
