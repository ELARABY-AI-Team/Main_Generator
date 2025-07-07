#ifndef MAIN_H_
#define MAIN_H_

/**********************************************************************************
 * @file      : main.h
 * @brief     : Minimal and common includes, typedefs, and macros for STM32F401RC
 * @author    : Technology Inovation Software Team
 * @device    : STM32F401RC
 * @creation  : 2025-07-07
 * @standard  : MISRA C
 * @copyright : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 **********************************************************************************/

/*
 * MISRA C:2012 Deviations:
 * - Rule 11.4 (Required): Conversions from a pointer to a non-pointer type or
 *   vice versa. This is necessary for direct register access in embedded systems.
 *   The device header (stm32f4xx.h) handles this by defining peripheral base
 *   addresses and register structures.
 * - Rule 20.8 (Required): The use of the #include directive shall be followed
 *   by a <standard header name> or "quasi-standard header name". The prompt
 *   requires specific standard headers and device-specific include.
 * - Rule 20.10 (Required): The #include directive shall be followed by either
 *   a <filename> or "filename" sequence. Standard headers use <>, device-specific
 *   and project headers use "".
 * - Rule 21.6 (Required): The Standard Library functions abort, exit, getenv,
 *   these specific functions should be avoided in application code.
 * - Rule 21.7 (Required): The Standard Library functions bsearch, calloc,
 *   div, ldiv, llabs, qsort, rand, realloc, and srand shall not be used.
 *   Similar to 21.6, specific functions from included headers should be avoided.
 * - Rule 21.8 (Required): The Standard Library functions controlling the
 *   localization problem shall not be used. While <locale.h> is not included
 *   here, this rule is relevant if adding locale-specific headers.
 * - Rule 21.10 (Required): The Standard Library header <stdio.h> shall not be used
 *   re-implemented or avoided in bare-metal embedded systems.
 * - Rule 21.11 (Required): The Standard Library header <stdlib.h> shall not be used
 *   in production code. Similar to <stdio.h>, resource management functions
 *   are typically handled differently.
 *
 * Justification: The nature of low-level embedded programming necessitates direct
 * hardware interaction via register access, which inherently requires pointer
 * conversions. The specific list of standard headers was explicitly requested.
 * Adherence to other MISRA rules, especially regarding code structure, side
 * effects, and type safety (where possible without violating required hardware
 * access patterns), should be maintained in the application code.
 */

/* Include Standard C headers */
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

/* Include MCU-specific header */
/* This header defines peripheral register structures and base addresses */
#include "stm32f4xx.h"

/* Include CMSIS core definitions for intrinsics (__NOP, __enable_irq, etc.) */
/* Typically included transitively by stm32f4xx.h, but explicitly mentioning for clarity */
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#include "core_cm4.h"


/**
 * @brief Type definitions for common data sizes
 */
typedef uint8_t  tbyte; /**< 8-bit unsigned integer */
typedef uint16_t tword; /**< 16-bit unsigned integer */
typedef uint32_t tlong; /**< 32-bit unsigned integer */


/**
 * @brief Bit manipulation macros
 */
#define SET_BIT(reg, bit)   ((reg) |= (1UL << (bit)))         /**< Set specified bit in a register */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1UL << (bit)))        /**< Clear specified bit in a register */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1UL)          /**< Get the value of a specified bit */
#define TOG_BIT(reg, bit)   ((reg) ^= (1UL << (bit)))         /**< Toggle specified bit in a register */


/**
 * @brief Core processor control macros (CMSIS intrinsics)
 *        These macros use CMSIS intrinsic functions provided by core_cm4.h
 */
#define DI()                  __disable_irq()           /**< Disable global interrupts using PRIMASK */
#define EI()                  __enable_irq()            /**< Enable global interrupts using PRIMASK */
#define Global_Int_Disable()  __disable_irq()           /**< Disable global interrupts (alias) */
#define Global_Int_Enable()   __enable_irq()            /**< Enable global interrupts (alias) */
#define HALT()                __WFI()                   /**< Wait For Interrupt low-power mode */
#define NOP()                 __NOP()                   /**< No operation (single cycle) */


/**
 * @brief SAFEGUARD MACRO: Initialize all GPIO ports to a safe, known state.
 *        Configures all pins on all ports (A, B, C, D, E, H) as:
 *        - Output Data Register (ODR): 0 (low output)
 *        - Mode Register (MODER): 00b (Input mode)
 *        - Pull-up/Pull-down Register (PUPDR): 00b (No pull-up, no pull-down)
 *        - Output Type Register (OTYPER): 0b (Push-pull - default after reset)
 *        - Output Speed Register (OSPEEDR): 00b (Low speed - default after reset)
 *        - Alternate Function Register (AFR): 0000b (AF0 - System function)
 *        Note: This macro assumes the clock for the respective GPIO ports
 *              is already enabled in the RCC (Reset and Clock Control)
 *              peripheral before being called.
 */
#define GPIO_SAFEGUARD_Init() do {                              \
    /* Configure GPIOA */                                       \
    GPIOA->ODR      = 0x00000000UL;                             \
    GPIOA->MODER    = 0x00000000UL; /* 00b: Input */            \
    GPIOA->PUPDR    = 0x00000000UL; /* 00b: No pull-up/down */  \
    GPIOA->OTYPER   = 0x00000000UL; /* 0b: Push-pull */         \
    GPIOA->OSPEEDR  = 0x00000000UL; /* 00b: Low speed */        \
    GPIOA->AFR[0]   = 0x00000000UL; /* 0000b: AF0 (System) */   \
    GPIOA->AFR[1]   = 0x00000000UL;                             \
                                                                \
    /* Configure GPIOB */                                       \
    GPIOB->ODR      = 0x00000000UL;                             \
    GPIOB->MODER    = 0x00000000UL;                             \
    GPIOB->PUPDR    = 0x00000000UL;                             \
    GPIOB->OTYPER   = 0x00000000UL;                             \
    GPIOB->OSPEEDR  = 0x00000000UL;                             \
    GPIOB->AFR[0]   = 0x00000000UL;                             \
    GPIOB->AFR[1]   = 0x00000000UL;                             \
                                                                \
    /* Configure GPIOC */                                       \
    GPIOC->ODR      = 0x00000000UL;                             \
    GPIOC->MODER    = 0x00000000UL;                             \
    GPIOC->PUPDR    = 0x00000000UL;                             \
    GPIOC->OTYPER   = 0x00000000UL;                             \
    GPIOC->OSPEEDR  = 0x00000000UL;                             \
    GPIOC->AFR[0]   = 0x00000000UL;                             \
    GPIOC->AFR[1]   = 0x00000000UL;                             \
                                                                \
    /* Configure GPIOD */                                       \
    GPIOD->ODR      = 0x00000000UL;                             \
    GPIOD->MODER    = 0x00000000UL;                             \
    GPIOD->PUPDR    = 0x00000000UL;                             \
    GPIOD->OTYPER   = 0x00000000UL;                             \
    GPIOD->OSPEEDR  = 0x00000000UL;                             \
    GPIOD->AFR[0]   = 0x00000000UL;                             \
    GPIOD->AFR[1]   = 0x00000000UL;                             \
                                                                \
    /* Configure GPIOE */                                       \
    GPIOE->ODR      = 0x00000000UL;                             \
    GPIOE->MODER    = 0x00000000UL;                             \
    GPIOE->PUPDR    = 0x00000000UL;                             \
    GPIOE->OTYPER   = 0x00000000UL;                             \
    GPIOE->OSPEEDR  = 0x00000000UL;                             \
    GPIOE->AFR[0]   = 0x00000000UL;                             \
    GPIOE->AFR[1]   = 0x00000000UL;                             \
                                                                \
    /* Configure GPIOH (Check datasheet for actual pins on H, typically only a few) */ \
    GPIOH->ODR      = 0x00000000UL;                             \
    GPIOH->MODER    = 0x00000000UL;                             \
    GPIOH->PUPDR    = 0x00000000UL;                             \
    GPIOH->OTYPER   = 0x00000000UL;                             \
    GPIOH->OSPEEDR  = 0x00000000UL;                             \
    GPIOH->AFR[0]   = 0x00000000UL;                             \
    GPIOH->AFR[1]   = 0x00000000UL;                             \
} while(0)


/**
 * @brief SAFEGUARD MACRO: Disable common peripherals and configure critical settings.
 *        - Disables global interrupts.
 *        - Disables all Timers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11) by resetting CR1 and DIER.
 *        - Disables Window Watchdog (WWDG) by resetting its CR. (IWDG cannot be disabled once started except by reset).
 *        - Disables ADC1 by resetting its CR1, CR2, and SR.
 *        - Disables all UART/USARTs (USART1, USART2, USART6, UART4, UART5) by resetting their CR1, CR2, and CR3.
 *        - Disables all I2Cs (I2C1, I2C2, I2C3) by resetting their CR1 and CR2.
 *        - Disables all SPIs (SPI1, SPI2, SPI3, SPI4) by resetting their CR1 and CR2.
 *        - Sets all GPIO Alternate Function registers to AF0 (System), ensuring pins are not
 *          connected to peripherals via AF mapping. (Also done in GPIO_SAFEGUARD_Init,
 *          but repeated here for robustness in a general register safeguard).
 *        Note: This macro attempts to reset peripheral registers to a disabled state.
 *              It assumes the necessary peripheral clocks might already be enabled
 *              or handles cases where they are enabled. For a true reset state,
 *              disabling peripheral clocks in RCC would also be necessary, but
 *              that's typically handled by higher-level initialization.
 */
#define Registers_SAFEGUARD_Init() do {                         \
    /* Disable global interrupts */                             \
    __disable_irq();                                            \
                                                                \
    /* Disable Timers by resetting Control Register 1 (CR1) */  \
    TIM1->CR1 = 0x0000UL; TIM1->DIER = 0x0000UL;                \
    TIM2->CR1 = 0x0000UL; TIM2->DIER = 0x0000UL;                \
    TIM3->CR1 = 0x0000UL; TIM3->DIER = 0x0000UL;                \
    TIM4->CR1 = 0x0000UL; TIM4->DIER = 0x0000UL;                \
    TIM5->CR1 = 0x0000UL; TIM5->DIER = 0x0000UL;                \
    TIM9->CR1 = 0x0000UL; TIM9->DIER = 0x0000UL;                \
    TIM10->CR1 = 0x0000UL; TIM10->DIER = 0x0000UL;              \
    TIM11->CR1 = 0x0000UL; TIM11->DIER = 0x0000UL;              \
                                                                \
    /* Disable Window Watchdog (WWDG). IWDG cannot be disabled once started. */ \
    WWDG->CR = 0x00UL;                                          \
                                                                \
    /* Disable ADC1 */                                          \
    ADC1->CR2 = 0x00000000UL;                                   \
    ADC1->CR1 = 0x00000000UL;                                   \
    ADC1->SR = 0x00000000UL;                                    \
                                                                \
    /* Disable UART/USARTs by resetting their Control Registers */ \
    USART1->CR1 = 0x00000000UL; USART1->CR2 = 0x0000UL; USART1->CR3 = 0x0000UL; \
    USART2->CR1 = 0x00000000UL; USART2->CR2 = 0x0000UL; USART2->CR3 = 0x0000UL; \
    USART6->CR1 = 0x00000000UL; USART6->CR2 = 0x0000UL; USART6->CR3 = 0x0000UL; \
    UART4->CR1 = 0x00000000UL; UART4->CR2 = 0x0000UL; UART4->CR3 = 0x0000UL;   \
    UART5->CR1 = 0x00000000UL; UART5->CR2 = 0x0000UL; UART5->CR3 = 0x0000UL;   \
                                                                \
    /* Disable I2Cs by resetting their Control Registers */     \
    I2C1->CR1 = 0x0000UL; I2C1->CR2 = 0x0000UL;                 \
    I2C2->CR1 = 0x0000UL; I2C2->CR2 = 0x0000UL;                 \
    I2C3->CR1 = 0x0000UL; I2C3->CR2 = 0x0000UL;                 \
                                                                \
    /* Disable SPIs by resetting their Control Registers */     \
    SPI1->CR1 = 0x0000UL; SPI1->CR2 = 0x0000UL;                 \
    SPI2->CR1 = 0x0000UL; SPI2->CR2 = 0x0000UL;                 \
    SPI3->CR1 = 0x0000UL; SPI3->CR2 = 0x0000UL;                 \
    SPI4->CR1 = 0x0000UL; SPI4->CR2 = 0x0000UL;                 \
                                                                \
    /* Configure GPIO Alternate Functions to default (AF0 - System) */ \
    /* This ensures pins are not connected to peripherals via AF mapping. */ \
    GPIOA->AFR[0] = 0x00000000UL; GPIOA->AFR[1] = 0x00000000UL; \
    GPIOB->AFR[0] = 0x00000000UL; GPIOB->AFR[1] = 0x00000000UL; \
    GPIOC->AFR[0] = 0x00000000UL; GPIOC->AFR[1] = 0x00000000UL; \
    GPIOD->AFR[0] = 0x00000000UL; GPIOD->AFR[1] = 0x00000000UL; \
    GPIOE->AFR[0] = 0x00000000UL; GPIOE->AFR[1] = 0x00000000UL; \
    GPIOH->AFR[0] = 0x00000000UL; GPIOH->AFR[1] = 0x00000000UL; \
                                                                \
} while(0)


#endif /* MAIN_H_ */