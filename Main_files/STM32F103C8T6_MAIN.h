/**
 * @file    main.h
 * @brief   General header file for the STM32F103C8T6 microcontroller.
 *          Includes common standard types, useful macros, and initialization
 *          safeguard functions to put the system into a safe state.
 *
 * @author  Inovation Center (Software Group)
 * @device  STM32F103C8T6
 * @date    2025-06-19
 * @standard MISRA
 *
 * @copyright Copyright (c) 2025 Inovation Center. All rights reserved.
 */

#ifndef STM32F103C8T6_MAIN_H_
#define STM32F103C8T6_MAIN_H_

/* --- Standard Includes --------------------------------------------------- */
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

/* --- MCU Specific Includes ----------------------------------------------- */
/*
 * Note: Ensure your project environment is configured to find this header.
 *       This header provides definitions for peripheral registers etc.
 */
#include "stm32f1xx.h"


/* --- Type Definitions ---------------------------------------------------- */
typedef uint8_t  tbyte; /**< Type definition for 8-bit unsigned integer. */
typedef uint16_t tword; /**< Type definition for 16-bit unsigned integer. */
typedef uint32_t tlong; /**< Type definition for 32-bit unsigned integer. */


/* --- Core Macros --------------------------------------------------------- */

/**
 * @brief Sets the specified bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-31).
 */
#define SET_BIT(reg, bit)   ((reg) |= (1UL << (bit)))

/**
 * @brief Clears the specified bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-31).
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1UL << (bit)))

/**
 * @brief Gets the value of the specified bit in a register.
 * @param reg The register to read from.
 * @param bit The bit position (0-31).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1UL)

/**
 * @brief Toggles the specified bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-31).
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1UL << (bit)))

/* --- Global Interrupt Control Macros (CMSIS) ----------------------------- */

/**
 * @brief Disable global interrupts.
 */
#define Global_Int_Disable()    __disable_irq()

/**
 * @brief Enable global interrupts.
 */
#define Global_Int_Enable()     __enable_irq()

/* --- System Macros (CMSIS) ----------------------------------------------- */

/**
 * @brief Execute No Operation.
 */
#define NOP()                   __NOP()

/**
 * @brief Wait For Interrupt. Puts the core to sleep until an interrupt occurs.
 */
#define HALT()                  __WFI()


/* --- Safeguard Initialization Macros ------------------------------------- */

/**
 * @brief Configures all GPIO pins on available ports (A, B, C, D, E)
 *        into a safe, default state: Input Floating, ODR=0.
 *        This disables outputs, pull-ups/downs, analog, and alternate functions
 *        at the GPIO level.
 *        Note: Port E might not be fully available on all packages (e.g., C8T6).
 *        Accesses registers directly based on STM32F1xx definitions.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Ensure GPIO clocks are enabled before configuring registers */ \
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_GPIOAEN_Pos); \
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_GPIOBEN_Pos); \
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_GPIOCEN_Pos); \
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_GPIODEN_Pos); \
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_GPIOEEN_Pos); \
         \
        /* Configure all pins on ports A, B, C, D, E as Input Floating (MODE=00, CNF=01) */ \
        /* This is 0x4 per pin (4 bits), resulting in 0x44444444 for CRL/CRH */ \
         \
        /* Port A */ \
        GPIOA->ODR = 0x0000; /* Set output data register to 0 (safe default) */ \
        GPIOA->CRL = 0x44444444; \
        GPIOA->CRH = 0x44444444; \
         \
        /* Port B */ \
        GPIOB->ODR = 0x0000; /* Set output data register to 0 (safe default) */ \
        GPIOB->CRL = 0x44444444; \
        GPIOB->CRH = 0x44444444; \
         \
        /* Port C */ \
        GPIOC->ODR = 0x0000; /* Set output data register to 0 (safe default) */ \
        GPIOC->CRL = 0x44444444; \
        GPIOC->CRH = 0x44444444; \
         \
        /* Port D */ \
        GPIOD->ODR = 0x0000; /* Set output data register to 0 (safe default) */ \
        GPIOD->CRL = 0x44444444; /* Note: Many PD pins are JTAG/SWD */ \
        GPIOD->CRH = 0x44444444; \
         \
        /* Port E */ \
        GPIOE->ODR = 0x0000; /* Set output data register to 0 (safe default) */ \
        GPIOE->CRL = 0x44444444; \
        GPIOE->CRH = 0x44444444; \
         \
        /* Disable peripheral clocks for GPIOs (optional, but good practice after config) */ \
        /* Note: Leaving clocks on might be needed if GPIOs are used immediately after. */ \
        /* Keeping clocks enabled allows registers to be accessed. Disable only if power saving is critical before any peripheral init. */ \
         \
    } while(0)

/**
 * @brief Disables common peripherals and puts them into a safe state.
 *        Disables global interrupts, Timers, WWDG, ADC, USART, I2C, SPI.
 *        Note: IWDG cannot be disabled once enabled. This macro does NOT
 *        interact with IWDG in a way that could enable it.
 *        Accesses registers directly based on STM32F1xx definitions.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable global interrupts */ \
        Global_Int_Disable(); \
         \
        /* Disable peripheral clocks (recommended before disabling peripherals) */ \
        /* This step is crucial for power saving and ensuring stable state. */ \
        RCC->APB1ENR = 0x00000000; /* Disable all APB1 peripherals (Timers, WWDG, SPI2/3, USART2/3, I2C1/2, CAN1/2, BKP, PWR, DAC) */ \
        RCC->APB2ENR = 0x00000000; /* Disable all APB2 peripherals (AFIO, IOPA-IOE, ADC1/2, TIM1, SPI1, USART1) */ \
        /* Leave AHB enabled to allow access to Flash, SRAM, RCC */ \
        /* Note: GPIO clocks were enabled briefly in GPIO_SAFEGUARD_Init. They are now disabled here. */ \
         \
        /* Explicitly disable peripherals by clearing enable bits (redundant if clocks are off, but defensive) */ \
        /* This step assumes clocks *could* be turned back on later without a system reset */ \
         \
        /* Timers (TIM1-TIM4 for F103C8T6) */ \
        /* Accesses might fault if clock is off, but writing 0 is harmless if it doesn't land */ \
        TIM1->CR1 &= ~TIM_CR1_CEN; \
        TIM2->CR1 &= ~TIM_CR1_CEN; \
        TIM3->CR1 &= ~TIM_CR1_CEN; \
        TIM4->CR1 &= ~TIM_CR1_CEN; \
         \
        /* Watchdog Timer (WWDG) - IWDG cannot be disabled */ \
        WWDG->CR &= ~WWDG_CR_WDGA; /* Clear enable bit if clock was on */ \
         \
        /* ADC (ADC1, ADC2 for F103C8T6) */ \
        ADC1->CR2 &= ~ADC_CR2_ADON; /* Power down ADC */ \
        ADC2->CR2 &= ~ADC_CR2_ADON; \
         \
        /* USART (USART1-USART3 for F103C8T6) */ \
        USART1->CR1 &= ~USART_CR1_UE; /* Disable USART */ \
        USART2->CR1 &= ~USART_CR1_UE; \
        USART3->CR1 &= ~USART_CR1_UE; \
         \
        /* I2C (I2C1, I2C2 for F103C8T6) */ \
        I2C1->CR1 &= ~I2C_CR1_PE; /* Disable I2C */ \
        I2C2->CR1 &= ~I2C_CR1_PE; \
         \
        /* SPI (SPI1, SPI2 for F103C8T6) */ \
        SPI1->CR1 &= ~SPI_CR1_SPE; /* Disable SPI */ \
        SPI2->CR1 &= ~SPI_CR1_SPE; \
         \
        /* Input Capture Unit (ICU) and Pulse Width Modulation (PWM) */ \
        /* These are modes of Timers. Disabling Timers (CEN=0) above stops ICU/PWM operation. */ \
         \
        /* Configure GPIOS as I/O, not special function registers */ \
        /* This is handled by GPIO_SAFEGUARD_Init(). The current state is Input Floating. */ \
        /* If you needed a different default (e.g., Input Pull-up/down), you would modify GPIO_SAFEGUARD_Init */ \
        /* or call it again with different parameters/logic. This safeguard assumes Input Floating is the desired safe default. */ \
         \
    } while(0)

#endif /* MAIN_H */