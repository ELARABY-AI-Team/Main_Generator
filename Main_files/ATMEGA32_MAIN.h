/**
 * @file main.h
 * @brief Main system header file for STM32F401RC microcontroller.
 *        Includes common definitions, types, core macros, and safeguard functions.
 * @author Technology Inovation Software Team
 * @device STM32F401RC
 * @date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/* Includes */

/* Device specific header - Includes CMSIS definitions for STM32F4xx series */
#include "stm32f4xx.h"

/* Standard C Includes */
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

/* Type Definitions */
typedef uint8_t  tbyte;  /**< 8-bit unsigned integer type */
typedef uint16_t tword;  /**< 16-bit unsigned integer type */
typedef uint32_t tlong;  /**< 32-bit unsigned integer type */

/* Core Macros */

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-31).
 */
#define SET_BIT(reg, bit)     ((reg) |= (1UL << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-31).
 */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1UL << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-31).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1UL)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-31).
 */
#define TOG_BIT(reg, bit)     ((reg) ^= (1UL << (bit)))

/**
 * @brief Disable global interrupts (Cortex-M intrinsic).
 */
#define Global_Int_Disable()  __disable_irq()

/**
 * @brief Enable global interrupts (Cortex-M intrinsic).
 */
#define Global_Int_Enable()   __enable_irq()

/**
 * @brief Alias for Global_Int_Disable().
 */
#define DI()                  Global_Int_Disable()

/**
 * @brief Alias for Global_Int_Enable().
 */
#define EI()                  Global_Int_Enable()

/**
 * @brief Execute Wait For Interrupt instruction (Cortex-M intrinsic).
 */
#define HALT()                __WFI()

/**
 * @brief Execute No Operation instruction (Cortex-M intrinsic).
 */
#define NOP()                 __NOP()

/* SAFEGUARD MACROS - FULLY IMPLEMENTED */

/**
 * @brief Safeguard initialization for GPIOs.
 *        Configures all GPIO pins to input mode with no pull-up/down,
 *        clears output data register, resets alternate functions,
 *        and disables EXTI interrupts from GPIO sources.
 *        This sets a known, safe state for all GPIOs upon startup.
 */
#define GPIO_SAFEGUARD_Init() \
do { \
    /* Enable clocks for all potential GPIO ports (A-H) to ensure access */ \
    /* STM32F401RC has GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH */ \
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | \
                     RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | \
                     RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN); \
    \
    /* Configure all GPIO pins as Input (MODER=00), No Pull-up/down (PUPDR=00), */ \
    /* Push-pull Output Type (OTYPER=0), Low Speed (OSPEEDR=00), */ \
    /* and System Function (AFRL/AFRH=0000). Clear ODR (Output Data Register). */ \
    /* Note: Clearing registers sets default values (0) which correspond to Input mode, */ \
    /* no pull-up/down, push-pull, low speed, and system function for MODER, PUPDR, OTYPER, OSPEEDR, AFR. */ \
    GPIOA->ODR = 0x00000000UL; GPIOA->MODER = 0x00000000UL; GPIOA->OTYPER = 0x00000000UL; GPIOA->OSPEEDR = 0x00000000UL; GPIOA->PUPDR = 0x00000000UL; GPIOA->AFR[0] = 0x00000000UL; GPIOA->AFR[1] = 0x00000000UL; \
    GPIOB->ODR = 0x00000000UL; GPIOB->MODER = 0x00000000UL; GPIOB->OTYPER = 0x00000000UL; GPIOB->OSPEEDR = 0x00000000UL; GPIOB->PUPDR = 0x00000000UL; GPIOB->AFR[0] = 0x00000000UL; GPIOB->AFR[1] = 0x00000000UL; \
    GPIOC->ODR = 0x00000000UL; GPIOC->MODER = 0x00000000UL; GPIOC->OTYPER = 0x00000000UL; GPIOC->OSPEEDR = 0x00000000UL; GPIOC->PUPDR = 0x00000000UL; GPIOC->AFR[0] = 0x00000000UL; GPIOC->AFR[1] = 0x00000000UL; \
    GPIOD->ODR = 0x00000000UL; GPIOD->MODER = 0x00000000UL; GPIOD->OTYPER = 0x00000000UL; GPIOD->OSPEEDR = 0x00000000UL; GPIOD->PUPDR = 0x00000000UL; GPIOD->AFR[0] = 0x00000000UL; GPIOD->AFR[1] = 0x00000000UL; \
    GPIOE->ODR = 0x00000000UL; GPIOE->MODER = 0x00000000UL; GPIOE->OTYPER = 0x00000000UL; GPIOE->OSPEEDR = 0x00000000UL; GPIOE->PUPDR = 0x00000000UL; GPIOE->AFR[0] = 0x00000000UL; GPIOE->AFR[1] = 0x00000000UL; \
    GPIOH->ODR = 0x00000000UL; GPIOH->MODER = 0x00000000UL; GPIOH->OTYPER = 0x00000000UL; GPIOH->OSPEEDR = 0x00000000UL; GPIOH->PUPDR = 0x00000000UL; GPIOH->AFR[0] = 0x00000000UL; GPIOH->AFR[1] = 0x00000000UL; \
    \
    /* Disable all External Interrupts (EXTI) sources tied to GPIOs */ \
    /* Clear pending requests for all lines */ \
    EXTI->PR = 0xFFFFFFFFUL; \
    /* Disable interrupt and event requests for all lines */ \
    EXTI->IMR = 0x00000000UL; \
    EXTI->EMR = 0x00000000UL; \
    /* Disable rising and falling triggers for all lines */ \
    EXTI->RTSR = 0x00000000UL; \
    EXTI->FTSR = 0x00000000UL; \
    \
    /* Disconnect SYSCFG from EXTI lines (resets to Port A linkage) */ \
    SYSCFG->EXTICR[0] = 0x00000000UL; \
    SYSCFG->EXTICR[1] = 0x00000000UL; \
    SYSCFG->EXTICR[2] = 0x00000000UL; \
    SYSCFG->EXTICR[3] = 0x00000000UL; \
} while(0) /* MISRA Rule 19.4, 19.7 */

/**
 * @brief Safeguard initialization for common peripherals.
 *        Disables global interrupts, disables common timers, watchdog, ADC,
 *        UART, I2C, and SPI modules. Configures GPIOs back to basic I/O (Input mode).
 *        This sets a known, safe state for major peripherals upon startup.
 *        Note: IWDG cannot be disabled by software if enabled.
 */
#define Registers_SAFEGUARD_Init() \
do { \
    /* Disable global interrupts */ \
    Global_Int_Disable(); \
    \
    /* Enable clocks for all relevant peripherals to allow register access */ \
    /* Note: This is necessary to write to the peripheral registers. */ \
    /* The peripherals are then disabled by clearing their enable bits or control registers. */ \
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | \
                     RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | \
                     RCC_APB1ENR_WWDGEN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_SPI3EN | \
                     RCC_APB1ENR_USART2EN | RCC_APB1ENR_I2C1EN | RCC_APB1ENR_I2C2EN | \
                     RCC_APB1ENR_I2C3EN | RCC_APB1ENR_PWREN); /* PWR clock for voltage regulator/low power settings */ \
    \
    RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN | RCC_APB2ENR_USART1EN | \
                     RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN | \
                     RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN); \
    \
    /* Disable all Timers (PWM, ICU modes are timer features and disabled by clearing control registers) */ \
    TIM1->CR1 = 0x00000000UL; TIM1->CCER = 0x00000000UL; \
    TIM2->CR1 = 0x00000000UL; TIM2->CCER = 0x00000000UL; \
    TIM3->CR1 = 0x00000000UL; TIM3->CCER = 0x00000000UL; \
    TIM4->CR1 = 0x00000000UL; TIM4->CCER = 0x00000000UL; \
    TIM5->CR1 = 0x00000000UL; TIM5->CCER = 0x00000000UL; \
    TIM6->CR1 = 0x00000000UL; /* Basic timer has no CCER */ \
    TIM7->CR1 = 0x00000000UL; /* Basic timer has no CCER */ \
    TIM8->CR1 = 0x00000000UL; TIM8->CCER = 0x00000000UL; \
    TIM9->CR1 = 0x00000000UL; TIM9->CCER = 0x00000000UL; \
    TIM10->CR1 = 0x00000000UL; TIM10->CCER = 0x00000000UL; \
    TIM11->CR1 = 0x00000000UL; TIM11->CCER = 0x00000000UL; \
    \
    /* Disable Watchdog Timer (WWDG only, IWDG is hardware enabled and cannot be disabled by software if active) */ \
    WWDG->CR &= ~WWDG_CR_WDGA; /* Clear WDGA bit to disable WWDG */ \
    \
    /* Disable ADC */ \
    ADC1->CR2 &= ~ADC_CR2_ADON; /* Ensure ADC is off */ \
    ADC1->CR1 = 0x00000000UL; \
    ADC1->CR2 = 0x00000000UL; \
    ADC1->CR3 = 0x00000000UL; \
    ADC1->SQR1 = 0x00000000UL; ADC1->SQR2 = 0x00000000UL; ADC1->SQR3 = 0x00000000UL; \
    ADC1->SMPR1 = 0x00000000UL; ADC1->SMPR2 = 0x00000000UL; \
    /* Clearing DR is not applicable as it's read-only */ \
    ADC->CCR = 0x00000000UL; /* Clear common control registers */ \
    \
    /* Disable UARTs (USARTs) */ \
    USART1->CR1 &= ~USART_CR1_UE; /* Disable UART */ \
    USART1->CR1 = 0x00000000UL; USART1->CR2 = 0x00000000UL; USART1->CR3 = 0x00000000UL; \
    USART2->CR1 &= ~USART_CR1_UE; /* Disable UART */ \
    USART2->CR1 = 0x00000000UL; USART2->CR2 = 0x00000000UL; USART2->CR3 = 0x00000000UL; \
    \
    /* Disable I2Cs */ \
    I2C1->CR1 &= ~I2C_CR1_PE; /* Disable I2C */ \
    I2C1->CR1 = 0x00000000UL; I2C1->CR2 = 0x00000000UL; \
    I2C2->CR1 &= ~I2C_CR1_PE; /* Disable I2C */ \
    I2C2->CR1 = 0x00000000UL; I2C2->CR2 = 0x00000000UL; \
    I2C3->CR1 &= ~I2C_CR1_PE; /* Disable I2C */ \
    I2C3->CR1 = 0x00000000UL; I2C3->CR2 = 0x00000000UL; \
    \
    /* Disable SPIs */ \
    SPI1->CR1 &= ~SPI_CR1_SPE; /* Disable SPI */ \
    SPI1->CR1 = 0x00000000UL; SPI1->CR2 = 0x00000000UL; \
    SPI2->CR1 &= ~SPI_CR1_SPE; /* Disable SPI */ \
    SPI2->CR1 = 0x00000000UL; SPI2->CR2 = 0x00000000UL; \
    SPI3->CR1 &= ~SPI_CR1_SPE; /* Disable SPI */ \
    SPI3->CR1 = 0x00000000UL; SPI3->CR2 = 0x00000000UL; \
    \
    /* Configure all GPIOS as standard I/O (Input Mode) and reset Alternate Functions */ \
    /* This ensures GPIOs are not left configured for special functions */ \
     GPIOA->MODER = 0x00000000UL; GPIOA->AFR[0] = 0x00000000UL; GPIOA->AFR[1] = 0x00000000UL; \
     GPIOB->MODER = 0x00000000UL; GPIOB->AFR[0] = 0x00000000UL; GPIOB->AFR[1] = 0x00000000UL; \
     GPIOC->MODER = 0x00000000UL; GPIOC->AFR[0] = 0x00000000UL; GPIOC->AFR[1] = 0x00000000UL; \
     GPIOD->MODER = 0x00000000UL; GPIOD->AFR[0] = 0x00000000UL; GPIOD->AFR[1] = 0x00000000UL; \
     GPIOE->MODER = 0x00000000UL; GPIOE->AFR[0] = 0x00000000UL; GPIOE->AFR[1] = 0x00000000UL; \
     GPIOH->MODER = 0x00000000UL; GPIOH->AFR[0] = 0x00000000UL; GPIOH->AFR[1] = 0x00000000UL; \
    /* Other GPIO settings (ODR, OTYPER, OSPEEDR, PUPDR) are handled by GPIO_SAFEGUARD_Init */ \
    /* This part specifically addresses the requirement to reset AF and mode to basic I/O */ \
    \
} while(0) /* MISRA Rule 19.4, 19.7 */

#endif /* MAIN_H_ */