/**
 * @file main.h
 * @brief Main project header file including core definitions and safeguards.
 * @author Technology Inovation Software Team
 * @device STM32F407ZG (Example MCU - implementation uses registers for this device)
 * @creation date: 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/*============================================================================*/
/*                           MCU Specific Include                             */
/*============================================================================*/

/*
 * NOTE: The implementation of SAFEGUARD macros uses registers specific to the
 * STM32F407ZG microcontroller. If using a different MCU, replace the include
 * below and update the SAFEGUARD macro implementations accordingly.
 */
#include "stm32f4xx.h"


/*============================================================================*/
/*                            Standard C Includes                             */
/*============================================================================*/

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


/*============================================================================*/
/*                                Useful Typedefs                             */
/*============================================================================*/

/** @brief 8-bit unsigned integer type */
typedef uint8_t tbyte;

/** @brief 16-bit unsigned integer type */
typedef uint16_t tword;

/** @brief 32-bit unsigned integer type */
typedef uint32_t tlong;


/*============================================================================*/
/*                                Core Macros                                 */
/*============================================================================*/

/** @brief Sets a specific bit in a register */
#define SET_BIT(reg, bit)       ((reg) |= (1UL << (bit)))

/** @brief Clears a specific bit in a register */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1UL << (bit)))

/** @brief Toggles a specific bit in a register */
#define TOG_BIT(reg, bit)       ((reg) ^= (1UL << (bit)))

/** @brief Gets the value of a specific bit in a register */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1UL)

/** @brief Disables global interrupts */
#define Global_Int_Disable()    (__disable_irq())

/** @brief Enables global interrupts */
#define Global_Int_Enable()     (__enable_irq())

/** @brief Enters a low-power wait-for-interrupt state */
#define HALT()                  (__WFI())

/** @brief Executes a no-operation instruction */
#define NOP()                   (__NOP())


/*============================================================================*/
/*                              SAFEGUARD MACROS                              */
/*============================================================================*/

/**
 * @brief Configures all GPIO pins to a safe, default state (Input, no pull-up/down, output low).
 *
 * This macro iterates through common GPIO ports (A-I on STM32F407ZG) and
 * configures their registers to a known safe state:
 * 1. Set Output Data Register (ODR) to 0 (ensuring outputs are low if briefly in output mode).
 * 2. Set Pull-up/Pull-down Register (PUPDR) to 0 (disables pull-ups/downs).
 * 3. Set Output Speed Register (OSPEEDR) to 0 (lowest speed).
 * 4. Set Output Type Register (OTYPER) to 0 (push-pull, though mode will be input).
 * 5. Set Alternate Function Registers (AFR[0], AFR[1]) to 0 (selects AF0).
 * 6. Set Mode Register (MODER) to 0 (configures all pins as input).
 *
 * NOTE: GPIO clocks must be enabled in the RCC registers for this macro to
 * have effect. This safeguard typically runs early in system startup AFTER
 * necessary clocks are enabled but BEFORE application-specific GPIO init.
 * The implementation uses registers specific to STM32F4xx.
 */
#define GPIO_SAFEGUARD_Init() do {                                      \
    /* Ensure clocks are enabled for relevant GPIO ports */             \
    /* This safeguard assumes clocking is handled elsewhere (e.g. SystemInit) */ \
                                                                        \
    /* Process GPIO Port A */                                           \
    GPIOA->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOA->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOA->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOA->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOA->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOA->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOA->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port B */                                           \
    GPIOB->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOB->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOB->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOB->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOB->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOB->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOB->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port C */                                           \
    GPIOC->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOC->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOC->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOC->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOC->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOC->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOC->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port D */                                           \
    GPIOD->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOD->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOD->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOD->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOD->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOD->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOD->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port E */                                           \
    GPIOE->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOE->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOE->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOE->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOE->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOE->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOE->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port F */                                           \
    GPIOF->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOF->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOF->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOF->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOF->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOF->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOF->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port G */                                           \
    GPIOG->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOG->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOG->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOG->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOG->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOG->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOG->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port H */                                           \
    GPIOH->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOH->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOH->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOH->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOH->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOH->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOH->MODER = 0x00000000;/* Set all pins to Input mode */          \
                                                                        \
    /* Process GPIO Port I */                                           \
    GPIOI->ODR = 0x0000;      /* Set output data to 0 */                \
    GPIOI->PUPDR = 0x00000000;/* Disable pull-ups/downs */              \
    GPIOI->OSPEEDR = 0x00000000;/* Lowest speed */                      \
    GPIOI->OTYPER = 0x0000;   /* Push-pull output type */               \
    GPIOI->AFR[0] = 0x00000000;/* AF0 selected */                        \
    GPIOI->AFR[1] = 0x00000000;/* AF0 selected */                        \
    GPIOI->MODER = 0x00000000;/* Set all pins to Input mode */          \
} while(0)

/**
 * @brief Disables common peripherals and sets them to a safe state.
 *
 * This macro attempts to disable various peripherals commonly found on an MCU:
 * 1. Disables global interrupts.
 * 2. Disables Timers (TIM1-TIM14 on STM32F407ZG).
 * 3. Disables Window Watchdog Timer (WWDG). Independent Watchdog (IWDG)
 *    cannot be disabled once started from software.
 * 4. Disables ADC instances (ADC1-ADC3).
 * 5. Disables UART/USART instances (USART1-USART6, UART7, UART8).
 * 6. Disables I2C instances (I2C1-I2C3).
 * 7. Disables SPI instances (SPI1-SPI6).
 * 8. Resets GPIO Alternate Function registers (though primarily done by GPIO_SAFEGUARD_Init).
 *
 * NOTE: Peripheral clocks must be enabled in the RCC registers for the
 * register writes within this macro to have effect. This safeguard typically
 * runs early in system startup. The implementation uses registers specific
 * to STM32F4xx. Setting peripheral control registers to 0 is a common way
 * to disable and reset them to a default state.
 */
#define Registers_SAFEGUARD_Init() do {                                 \
    /* Disable global interrupts */                                     \
    __disable_irq();                                                    \
                                                                        \
    /* Ensure clocks are enabled for relevant peripherals */            \
    /* This safeguard assumes clocking is handled elsewhere (e.g. SystemInit) */ \
                                                                        \
    /* Disable Timers (common instances on STM32F4) */                  \
    TIM1->CR1 = 0x0000; TIM1->CCER = 0x0000; /* Stop and reset config */\
    TIM2->CR1 = 0x0000; TIM2->CCER = 0x0000;                            \
    TIM3->CR1 = 0x0000; TIM3->CCER = 0x0000;                            \
    TIM4->CR1 = 0x0000; TIM4->CCER = 0x0000;                            \
    TIM5->CR1 = 0x0000; TIM5->CCER = 0x0000;                            \
    TIM6->CR1 = 0x0000; /* Basic timer */                               \
    TIM7->CR1 = 0x0000; /* Basic timer */                               \
    TIM8->CR1 = 0x0000; TIM8->CCER = 0x0000;                            \
    TIM9->CR1 = 0x0000; TIM9->CCER = 0x0000;                            \
    TIM10->CR1 = 0x0000; TIM10->CCER = 0x0000;                          \
    TIM11->CR1 = 0x0000; TIM11->CCER = 0x0000;                          \
    TIM12->CR1 = 0x0000; TIM12->CCER = 0x0000;                          \
    TIM13->CR1 = 0x0000; TIM13->CCER = 0x0000;                          \
    TIM14->CR1 = 0x0000; TIM14->CCER = 0x0000;                          \
                                                                        \
    /* Disable Watchdog (Window Watchdog can be disabled) */            \
    WWDG->CR = 0x0000;   /* Disable WWDG_CR_WWDGEN */                   \
    /* Note: IWDG cannot be stopped by software register write if started */\
                                                                        \
    /* Disable ADC (common instances) */                                \
    ADC1->CR2 &= ~ADC_CR2_ADON; /* Clear ADON bit */                    \
    ADC1->CR1 = 0x0000; ADC1->CR2 = 0x0000;                             \
    ADC2->CR2 &= ~ADC_CR2_ADON;                                         \
    ADC2->CR1 = 0x0000; ADC2->CR2 = 0x0000;                             \
    ADC3->CR2 &= ~ADC_CR2_ADON;                                         \
    ADC3->CR1 = 0x0000; ADC3->CR2 = 0x0000;                             \
    /* Also reset common ADC registers if they exist and need reset state */ \
    ADC->CCR = 0x0000; /* Common ADC registers */                       \
                                                                        \
    /* Disable UART/USART (common instances) */                         \
    USART1->CR1 &= ~USART_CR1_UE; /* Clear UE bit */                    \
    USART1->CR1 = 0x0000; USART1->CR2 = 0x0000; USART1->CR3 = 0x0000;   \
    USART2->CR1 &= ~USART_CR1_UE;                                       \
    USART2->CR1 = 0x0000; USART2->CR2 = 0x0000; USART2->CR3 = 0x0000;   \
    USART3->CR1 &= ~USART_CR1_UE;                                       \
    USART3->CR1 = 0x0000; USART3->CR2 = 0x0000; USART3->CR3 = 0x0000;   \
    UART4->CR1 &= ~USART_CR1_UE; /* UART has different instance name but similar registers */ \
    UART4->CR1 = 0x0000; UART4->CR2 = 0x0000; UART4->CR3 = 0x0000;   \
    UART5->CR1 &= ~USART_CR1_UE;                                       \
    UART5->CR1 = 0x0000; UART5->CR2 = 0x0000; UART5->CR3 = 0x0000;   \
    USART6->CR1 &= ~USART_CR1_UE;                                       \
    USART6->CR1 = 0x0000; USART6->CR2 = 0x0000; USART6->CR3 = 0x0000;   \
    /* Check for UART7, UART8 if they exist and are clocked */          \
    /* UART7->CR1 &= ~USART_CR1_UE; ... */                              \
    /* UART8->CR1 &= ~USART_CR1_UE; ... */                              \
                                                                        \
    /* Disable I2C (common instances) */                                \
    I2C1->CR1 &= ~I2C_CR1_PE; /* Clear PE bit */                        \
    I2C1->CR1 = 0x0000; I2C1->CR2 = 0x0000;                             \
    I2C2->CR1 &= ~I2C_CR1_PE;                                           \
    I2C2->CR1 = 0x0000; I2C2->CR2 = 0x0000;                             \
    I2C3->CR1 &= ~I2C_CR1_PE;                                           \
    I2C3->CR1 = 0x0000; I2C3->CR2 = 0x0000;                             \
                                                                        \
    /* Disable SPI (common instances) */                                \
    SPI1->CR1 &= ~SPI_CR1_SPE; /* Clear SPE bit */                      \
    SPI1->CR1 = 0x0000; SPI1->CR2 = 0x0000;                             \
    SPI2->CR1 &= ~SPI_CR1_SPE;                                          \
    SPI2->CR1 = 0x0000; SPI2->CR2 = 0x0000;                             \
    SPI3->CR1 &= ~SPI_CR1_SPE;                                          \
    SPI3->CR1 = 0x0000; SPI3->CR2 = 0x0000;                             \
    /* Check for SPI4, SPI5, SPI6 if they exist and are clocked */      \
    /* SPI4->CR1 &= ~SPI_CR1_SPE; ... */                                \
    /* SPI5->CR1 &= ~SPI_CR1_SPE; ... */                                \
    /* SPI6->CR1 &= ~SPI_CR1_SPE; ... */                                \
                                                                        \
    /* Configure all GPIOs as I/O (input/output) by resetting AF */    \
    /* This is largely redundant if GPIO_SAFEGUARD_Init() is called first, */ \
    /* but included as per requirement description. */                 \
    GPIOA->AFR[0] = 0x00000000; GPIOA->AFR[1] = 0x00000000;             \
    GPIOB->AFR[0] = 0x00000000; GPIOB->AFR[1] = 0x00000000;             \
    GPIOC->AFR[0] = 0x00000000; GPIOC->AFR[1] = 0x00000000;             \
    GPIOD->AFR[0] = 0x00000000; GPIOD->AFR[1] = 0x00000000;             \
    GPIOE->AFR[0] = 0x00000000; GPIOE->AFR[1] = 0x00000000;             \
    GPIOF->AFR[0] = 0x00000000; GPIOF->AFR[1] = 0x00000000;             \
    GPIOG->AFR[0] = 0x00000000; GPIOG->AFR[1] = 0x00000000;             \
    GPIOH->AFR[0] = 0x00000000; GPIOH->AFR[1] = 0x00000000;             \
    GPIOI->AFR[0] = 0x00000000; GPIOI->AFR[1] = 0x00000000;             \
    /* Note: This sets them to AF0, which is often GPIO, but MODER determines the mode */\
    /* and should be handled by GPIO_SAFEGUARD_Init for setting inputs. */\
} while(0)


#endif /* MAIN_H_ */