#ifndef MAIN_H
#define MAIN_H

/*
 * File Name: main.h
 * Description: Main project header file for Device name.
 * Author: Technology Inovation Software Team
 * Device name: [Please replace with the actual microcontroller name]
 * Creation date: 2025-07-07
 * Standard: MISRA C
 * Copyright: ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

/*
 * Include directives
 *
 * IMPORTANT:
 * Replace <[MCU specific header].h> with the actual header file for your microcontroller.
 * This header typically defines peripheral register structures and base addresses.
 * Examples: <stm32f4xx.h>, <avr/io.h>, <xc.h> etc.
 */
#include <[MCU specific header].h>

/*
 * Note: Some of these headers may not be strictly necessary for a minimal
 * embedded project but are included here as per the requirements.
 */
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>      /* MISRA C:2012 Rule 20.1 is advisory regarding <float.h> */
#include <inttypes.h>   /* MISRA C:2012 Rule 20.1 is advisory regarding <inttypes.h> */
#include <iso646.h>     /* MISRA C:2012 Rule 20.1 is advisory regarding <iso646.h> */
#include <limits.h>
#include <math.h>       /* MISRA C:2012 Rule 20.1 is advisory regarding <math.h> */
#include <setjmp.h>     /* MISRA C:2012 Rule 20.1 is advisory regarding <setjmp.h> */
#include <stdarg.h>     /* MISRA C:2012 Rule 20.1 is advisory regarding <stdarg.h> */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>      /* MISRA C:2012 Rule 20.13 is advisory regarding file operations */
#include <stdlib.h>     /* MISRA C:2012 Rule 20.1 is advisory regarding <stdlib.h> */
#include <string.h>

/*
 * Useful typedefs
 */
typedef uint8_t tbyte;  /* 8-bit unsigned integer */
typedef uint16_t tword; /* 16-bit unsigned integer */
typedef uint32_t tlong; /* 32-bit unsigned integer */

/*
 * Core macros for bit manipulation
 * Using 1U for unsigned literals as per MISRA C recommendations.
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))    /* Set a specific bit in a register */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))   /* Clear a specific bit in a register */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)     /* Get the value of a specific bit */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))    /* Toggle a specific bit in a register */

/*
 * Core macros for Global Interrupt Control
 * Note: These typically require architecture-specific intrinsics or register access.
 * For ARM Cortex-M, they might map to __disable_irq() and __enable_irq()
 * which are often provided by CMSIS (needs #include <cmsis_core.h> or similar
 * included by the MCU-specific header).
 */
#define Global_Int_Disable()    __disable_irq() /* Disable global interrupts - Example for ARM Cortex-M */
#define Global_Int_Enable()     __enable_irq()  /* Enable global interrupts - Example for ARM Cortex-M */
/* If not using ARM Cortex-M, replace with equivalent compiler intrinsic or register manipulation */


/*
 * SAFEGUARD MACROS
 *
 * IMPORTANT:
 * These macros provide example implementations based on STM32F4xx register names
 * and peripheral types. YOU MUST REPLACE THE REGISTER ACCESS CODE within the
 * do-while(0) blocks with the actual register names, bitfields, and values
 * for YOUR SPECIFIC MICROCONTROLLER.
 *
 * These macros are intended to place the system in a known, safe, low-power-ish
 * state before main application logic takes over.
 *
 * Using do-while(0) block for safe macro usage in all contexts.
 */

/**
 * @brief Configures all GPIO pins to a safe default state.
 *
 * This example implementation for STM32F4xx sets:
 * - All GPIOx->ODR (Output Data Register) to 0.
 * - All GPIOx->MODER (Mode Register) to Input mode (0b00).
 * - All GPIOx->PUPDR (Pull-up/Pull-down Register) to No pull-up/down (0b00).
 * - Attempts to clear EXTI pending flags and disable EXTI interrupts (basic "woke up" reset).
 *
 * Note: This assumes the GPIO peripheral clocks are already enabled
 * (e.g., via RCC->AHBxENR) which is typically done in system_stm32f4xx.c
 * or system initialization code before main().
 * Replace GPIOx with the actual GPIO port names for your MCU (e.g., PORTA, PTB, etc.)
 * and adjust register names (e.g., DDRx, PORTx, PUEAx, etc.) according to your datasheet.
 */
#define GPIO_SAFEGUARD_Init() do { \
    /* Example implementation for STM32F4xx (replace with your MCU's registers) */ \
    /* Note: Ensure GPIO clocks are enabled *before* calling this. */ \
    \
    /* Set Output Data Register (ODR) to 0 for all available GPIO ports */ \
    /* Assuming GPIOA, GPIOB, ..., GPIOI exist and are pointers to structs */ \
    GPIOA->ODR = 0x00000000U; \
    GPIOB->ODR = 0x00000000U; \
    GPIOC->ODR = 0x00000000U; \
    GPIOD->ODR = 0x00000000U; \
    GPIOE->ODR = 0x00000000U; \
    GPIOF->ODR = 0x00000000U; \
    GPIOG->ODR = 0x00000000U; \
    GPIOH->ODR = 0x00000000U; \
    GPIOI->ODR = 0x00000000U; \
    /* Add or remove GPIO ports based on your specific STM32F4 model or your MCU */ \
    \
    /* Configure Mode Register (MODER) for Input mode (0b00) for all pins */ \
    /* This also ensures they are not in Alternate Function mode (ADC, UART, etc.) */ \
    GPIOA->MODER = 0x00000000U; \
    GPIOB->MODER = 0x00000000U; \
    GPIOC->MODER = 0x00000000U; \
    GPIOD->MODER = 0x00000000U; \
    GPIOE->MODER = 0x00000000U; \
    GPIOF->MODER = 0x00000000U; \
    GPIOG->MODER = 0x00000000U; \
    GPIOH->MODER = 0x00000000U; \
    GPIOI->MODER = 0x00000000U; \
    /* Add or remove GPIO ports based on your specific STM32F4 model or your MCU */ \
    \
    /* Configure Pull-up/Pull-down Register (PUPDR) for No Pull-up/Pull-down (0b00) for all pins */ \
    GPIOA->PUPDR = 0x00000000U; \
    GPIOB->PUPDR = 0x00000000U; \
    GPIOC->PUPDR = 0x00000000U; \
    GPIOD->PUPDR = 0x00000000U; \
    GPIOE->PUPDR = 0x00000000U; \
    GPIOF->PUPDR = 0x00000000U; \
    GPIOG->PUPDR = 0x00000000U; \
    GPIOH->PUPDR = 0x00000000U; \
    GPIOI->PUPDR = 0x00000000U; \
    /* Add or remove GPIO ports based on your specific STM32F4 model or your MCU */ \
    \
    /* Disable External Interrupts (EXTI) - basic "woke up" disable example */ \
    /* Clear pending interrupts and disable interrupt masks */ \
    EXTI->PR = 0xFFFFFFFFU;  /* Clear all pending EXTI flags */ \
    EXTI->IMR = 0x00000000U; /* Disable all EXTI interrupt requests */ \
    /* If your MCU uses a different mechanism for external interrupts or wakeup, replace this. */ \
    \
    /* End of STM32F4xx Example Implementation */ \
} while(0)

/**
 * @brief Disables various peripherals to achieve a known initial state.
 *
 * This example implementation for STM32F4xx attempts to disable:
 * - Global Interrupts.
 * - All Timer peripherals (TIM1-TIM14 by clearing CR1).
 * - ADC peripherals (by clearing ADON bit).
 * - UART/USART peripherals (by clearing UE bit).
 * - I2C peripherals (by clearing PE bit).
 * - SPI peripherals (by clearing SPE bit).
 *
 * Note:
 * - This may not fully power down peripherals (clock enables might need to be cleared in RCC).
 * - Watchdog Timers (WDT) are often designed NOT to be disabled by software once active,
 *   or require specific key sequences. Attempting to disable might not work or could
 *   cause a reset if not handled correctly. WDT disabling is omitted here due to this risk.
 * - Replace peripheral instances (TIMx, ADCx, USARTx, I2Cx, SPIx) and
 *   register names (CR1, CR2, UE, PE, SPE, etc.) with those for your specific MCU.
 */
#define Registers_SAFEGUARD_Init() do { \
    /* Example implementation for STM32F4xx (replace with your MCU's registers) */ \
    \
    /* Disable global interrupts */ \
    Global_Int_Disable(); /* Uses macro defined above (example for ARM Cortex-M) */ \
    \
    /* Disable all Timers (clear CEN bit in CR1) */ \
    /* Assuming TIM1-TIM14 exist and are pointers to structs */ \
    TIM1->CR1 = 0x0000U; TIM2->CR1 = 0x0000U; TIM3->CR1 = 0x0000U; TIM4->CR1 = 0x0000U; \
    TIM5->CR1 = 0x0000U; TIM6->CR1 = 0x0000U; TIM7->CR1 = 0x0000U; TIM8->CR1 = 0x0000U; \
    TIM9->CR1 = 0x0000U; TIM10->CR1 = 0x0000U; TIM11->CR1 = 0x0000U; TIM12->CR1 = 0x0000U; \
    TIM13->CR1 = 0x0000U; TIM14->CR1 = 0x0000U; \
    /* Add or remove Timers based on your specific STM32F4 model or your MCU */ \
    /* Disabling Timers also disables typical PWM and Input Capture functions which are modes of timers. */ \
    \
    /* Disable ADC peripherals (clear ADON bit in CR2) */ \
    /* Assuming ADC1, ADC2, ADC3 exist and are pointers to structs */ \
    ADC1->CR2 &= ~ADC_CR2_ADON; \
    ADC2->CR2 &= ~ADC_CR2_ADON; \
    ADC3->CR2 &= ~ADC_CR2_ADON; \
    /* Add or remove ADCs based on your specific STM32F4 model or your MCU */ \
    \
    /* Disable UART/USART peripherals (clear UE bit in CR1) */ \
    /* Assuming USART1-USART6 and UART7-UART8 exist and are pointers to structs */ \
    USART1->CR1 &= ~USART_CR1_UE; USART2->CR1 &= ~USART_CR1_UE; USART3->CR1 &= ~USART_CR1_UE; \
    UART4->CR1 &= ~UART_CR1_UE;   UART5->CR1 &= ~UART_CR1_UE;   USART6->CR1 &= ~USART_CR1_UE; \
    UART7->CR1 &= ~UART_CR1_UE;   UART8->CR1 &= ~UART_CR1_UE; \
    /* Add or remove UART/USARTs based on your specific STM32F4 model or your MCU */ \
    \
    /* Disable I2C peripherals (clear PE bit in CR1) */ \
    /* Assuming I2C1-I2C3 exist and are pointers to structs */ \
    I2C1->CR1 &= ~I2C_CR1_PE; \
    I2C2->CR1 &= ~I2C_CR1_PE; \
    I2C3->CR1 &= ~I2C_CR1_PE; \
    /* Add or remove I2Cs based on your specific STM32F4 model or your MCU */ \
    \
    /* Disable SPI peripherals (clear SPE bit in CR1) */ \
    /* Assuming SPI1-SPI3 exist and are pointers to structs */ \
    SPI1->CR1 &= ~SPI_CR1_SPE; \
    SPI2->CR1 &= ~SPI_CR1_SPE; \
    SPI3->CR1 &= ~SPI_CR1_SPE; \
    /* Add or remove SPIs based on your specific STM32F4 model or your MCU */ \
    \
    /* Watchdog Timer (WDT) is typically NOT disabled here as it might be enabled */ \
    /* by fuses/option bytes and disabling it might not be possible or safe. */ \
    /* If you need to disable WDT, consult your MCU datasheet carefully for the procedure. */ \
    \
    /* Configure all GPIOS as input/output pins (I/O) not special function registers */ \
    /* This is handled by setting GPIO->MODER to Input mode (0x00) in GPIO_SAFEGUARD_Init(), */\
    /* which prevents them from operating in Alternate Function modes (ADC, UART, etc.). */ \
    \
    /* End of STM32F4xx Example Implementation */ \
} while(0)


#endif /* MAIN_H */