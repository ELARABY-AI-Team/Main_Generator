/**
 * @file main.h
 * @brief A minimal header file containing commonly used elements for STM32F407 embedded projects.
 * @author Inovation Center (Software Group)
 * @device STM32F407xx
 * @date 2025-06-19
 * @standard MISRA C compliance considerations
 * @copyright Copyright (c) 2025, Inovation Center (Software Group). All rights reserved.
 */

#ifndef STM32F407_MAIN_H_
#define STM32F407_MAIN_H_

/*==================================================================================================
*                                        INCLUDE FILES
* 1. Include standard C headers required
* 2. Include specific MCU headers
==================================================================================================*/

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

/* Include the appropriate STM32F4 device header file */
#include "stm32f4xx.h"

/*==================================================================================================
*                                          TYPEDEFS
==================================================================================================*/

/** @typedef tbyte
 *  @brief Standard 8-bit unsigned integer type.
 */
typedef uint8_t tbyte;

/** @typedef tword
 *  @brief Standard 16-bit unsigned integer type.
 */
typedef uint16_t tword;

/** @typedef tlong
 *  @brief Standard 32-bit unsigned integer type.
 */
typedef uint32_t tlong;


/*==================================================================================================
*                                           MACROS
==================================================================================================*/

/** @name Bit Manipulation Macros
 *  @{
 */
#define SET_BIT(reg, bit)     ((reg) |= (1U << (bit)))         /**< Sets the specified bit in a register. */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1U << (bit)))        /**< Clears the specified bit in a register. */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1U)          /**< Gets the value of the specified bit in a register. */
#define TOG_BIT(reg, bit)     ((reg) ^= (1U << (bit)))         /**< Toggles the specified bit in a register. */
/** @} */

/** @name System and Interrupt Macros
 *  @{
 */
#define Global_Int_Disable()  __disable_irq()                  /**< Disables global interrupts (using CMSIS intrinsic). */
#define Global_Int_Enable()   __enable_irq()                   /**< Enables global interrupts (using CMSIS intrinsic). */
#define HALT()                __WFI()                          /**< Enters Wait For Interrupt mode (using CMSIS intrinsic). */
#define NOP()                 __NOP()                          /**< Executes a No Operation instruction (using CMSIS intrinsic). */
/** @} */


/*==================================================================================================
*                                       SAFEGUARD MACROS
* These macros implement initial configuration to put the system in a safe, low-power state.
* They configure peripherals to a default, disabled state using actual MCU registers.
==================================================================================================*/

/**
 * @brief Configures all GPIO ports to a safe, default state (low output, input mode, no pull-up/down, EXTI disabled).
 *
 * This macro iterates through common GPIO ports (A to I) and sets their state as follows:
 * - Sets the Output Data Register (ODR) to 0 (all output pins low).
 * - Sets the Mode Register (MODER) to 0 (all pins configured as general purpose input).
 * - Sets the Pull-up/Pull-down Register (PUPDR) to 0 (all pull-up/down resistors disabled).
 * - Disables all External Interrupt (EXTI) lines and clears pending interrupts.
 * Clock for GPIO ports is enabled first to allow register access.
 * Using do-while(0) for MISRA compliance.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Enable clock for GPIO ports A to I */ \
        /* Ensure clocks are enabled before accessing GPIO registers */ \
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | \
                         RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | \
                         RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN); \
        /* Dummy read to ensure clock is stable */ \
        (void)RCC->AHB1ENR; \
        \
        /* Set Output Data Register (ODR) to 0 for all ports */ \
        GPIOA->ODR = 0x00000000; \
        GPIOB->ODR = 0x00000000; \
        GPIOC->ODR = 0x00000000; \
        GPIOD->ODR = 0x00000000; \
        GPIOE->ODR = 0x00000000; \
        GPIOF->ODR = 0x00000000; \
        GPIOG->ODR = 0x00000000; \
        GPIOH->ODR = 0x00000000; \
        GPIOI->ODR = 0x00000000; \
        \
        /* Set Mode Register (MODER) to 0 (Input mode) for all pins */ \
        GPIOA->MODER = 0x00000000; \
        GPIOB->MODER = 0x00000000; \
        GPIOC->MODER = 0x00000000; \
        GPIOD->MODER = 0x00000000; \
        GPIOE->MODER = 0x00000000; \
        GPIOF->MODER = 0x00000000; \
        GPIOG->MODER = 0x00000000; \
        GPIOH->MODER = 0x00000000; \
        GPIOI->MODER = 0x00000000; \
        \
        /* Set Pull-up/Pull-down Register (PUPDR) to 0 (No pull-up/down) */ \
        GPIOA->PUPDR = 0x00000000; \
        GPIOB->PUPDR = 0x00000000; \
        GPIOC->PUPDR = 0x00000000; \
        GPIOD->PUPDR = 0x00000000; \
        GPIOE->PUPDR = 0x00000000; \
        GPIOF->PUPDR = 0x00000000; \
        GPIOG->PUPDR = 0x00000000; \
        GPIOH->PUPDR = 0x00000000; \
        GPIOI->PUPDR = 0x00000000; \
        \
        /* Disable EXTI lines and clear pending interrupts (related to wake-up sources) */ \
        EXTI->IMR = 0x00000000;     /* Interrupt Mask Register */ \
        EXTI->EMR = 0x00000000;     /* Event Mask Register */ \
        EXTI->RTSR = 0x00000000;    /* Rising Trigger Selection Register */ \
        EXTI->FTSR = 0x00000000;    /* Falling Trigger Selection Register */ \
        EXTI->PR = 0xFFFFFFFF;      /* Pending Register - clear pending bits by writing 1 */ \
    } while(0)

/**
 * @brief Configures various peripheral registers to a safe, disabled state.
 *
 * This macro disables global interrupts, peripheral clocks for common modules (Timers,
 * Watchdog, SPI, USART/UART, I2C, ADC), resets these peripherals, and ensures
 * GPIO pins are configured as inputs (not special functions).
 * Note: Independent Watchdog (IWDG) cannot be disabled once enabled by option bytes.
 * Using do-while(0) for MISRA compliance.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable global interrupts */ \
        Global_Int_Disable(); \
        \
        /* Disable Peripheral Clocks (APBxENR registers) */ \
        /* This stops operation of most peripherals */ \
        /* Clear all bits in APB1ENR to disable TIM2-7, TIM12-14, WWDG, SPI2/3, USART2-5, I2C1-3, UART7/8, CAN1/2, PWR, DAC */ \
        RCC->APB1ENR = 0x00000000; \
        /* Clear all bits in APB2ENR to disable TIM1, TIM8-11, USART1/6, ADC1-3, SPI1/4, SYSCFG, SDIO */ \
        RCC->APB2ENR = 0x00000000; \
        /* Clear specific bits in AHB1ENR for complex peripherals like DMA/CRC, keeping GPIO clocks on if needed */ \
        /* GPIO clocks (bits 0-8) might be needed depending on calling context, assume GPIO_SAFEGUARD_Init enables them */ \
        /* Let's surgically clear DMA1, DMA2, CRC clocks */ \
        RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_CRCEN); \
        /* Clear all bits in AHB2ENR to disable DCMI, OTGFSEN */ \
        RCC->AHB2ENR = 0x00000000; \
        /* Clear all bits in AHB3ENR to disable FMC, QSPI (if present), OTGHSEN */ \
        RCC->AHB3ENR = 0x00000000; \
        \
        /* Disable Window Watchdog (WWDG) - Note: IWDG cannot be stopped if enabled by option bytes */ \
        /* If WWDG was enabled, this will stop it */ \
        WWDG->CR &= ~WWDG_CR_WDGA; \
        \
        /* Reset Peripherals (APBxRSTR registers) */ \
        /* Assert and then de-assert reset for APB1 peripherals to force default register states */ \
        RCC->APB1RSTR = 0xFFFFFFFF; \
        NOP(); /* Small delay */ \
        RCC->APB1RSTR = 0x00000000; \
        /* Assert and then de-assert reset for APB2 peripherals */ \
        RCC->APB2RSTR = 0xFFFFFFFF; \
        NOP(); /* Small delay */ \
        RCC->APB2RSTR = 0x00000000; \
        /* Note: AHB peripherals typically reset differently or don't have a bus-wide RSTR */ \
        \
        /* Ensure GPIOs are configured as general purpose Input (not special function) */ \
        /* Enable clock for GPIO ports A to I (in case they were disabled) */ \
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | \
                         RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | \
                         RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN); \
        (void)RCC->AHB1ENR; /* Dummy read */ \
        /* Set Mode Register (MODER) to 0 (Input mode) for all pins */ \
        GPIOA->MODER = 0x00000000; \
        GPIOB->MODER = 0x00000000; \
        GPIOC->MODER = 0x00000000; \
        GPIOD->MODER = 0x00000000; \
        GPIOE->MODER = 0x00000000; \
        GPIOF->MODER = 0x00000000; \
        GPIOG->MODER = 0x00000000; \
        GPIOH->MODER = 0x00000000; \
        GPIOI->MODER = 0x00000000; \
        /* PUPDR/ODR are typically handled by GPIO_SAFEGUARD_Init if needed, focus is on MODER here */ \
    } while(0)


/*==================================================================================================
*                                         GLOBAL CONSTANTS
==================================================================================================*/
/* Define any widely used constants here if necessary */


/*==================================================================================================
*                                         GLOBAL DATA
==================================================================================================*/
/* Declare any global variables with 'extern' keyword here if necessary */


/*==================================================================================================
*                                       FUNCTION PROTOTYPES
==================================================================================================*/
/* Declare any function prototypes accessible from other files here */


#endif /* MAIN_H_ */