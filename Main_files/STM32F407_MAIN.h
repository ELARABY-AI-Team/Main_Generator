/**
 * @file main.h
 * @brief Main header file providing common definitions and safeguards for STM32F407 projects.
 * @author Inovation Center (Software Group)
 * @device STM32F407xx
 * @creation_date 2025-06-19
 * @standard MISRA C
 * @copyright (c) 2025 Inovation Center (Software Group). All rights reserved.
 */

#ifndef STM32F407_MAIN_H_
#define STM32F407_MAIN_H_

/* Include Guards */

/* Standard C Headers */
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

/* MCU Specific Includes */
/*
 * This header file is typically provided by STMicroelectronics as part of the
 * STM32CubeF4 package or legacy Standard Peripheral Library. It defines
 * peripheral register structures, bit fields, and base addresses.
 * It also includes CMSIS headers like core_cm4.h
 */
#include "stm32f4xx.h"


/*============================================================================*/
/*                         USEFUL TYPEDEFS                                    */
/*============================================================================*/

/**
 * @brief 8-bit unsigned integer type.
 */
typedef uint8_t  tbyte;

/**
 * @brief 16-bit unsigned integer type.
 */
typedef uint16_t tword;

/**
 * @brief 32-bit unsigned integer type.
 */
typedef uint32_t tlong;


/*============================================================================*/
/*                         CORE MACROS                                        */
/*============================================================================*/

/**
 * @brief Set a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-31) to set.
 */
#define SET_BIT(reg, bit)   ((reg) |= (1UL << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-31) to clear.
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1UL << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit number (0-31) to get.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1UL)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-31) to toggle.
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1UL << (bit)))


/*============================================================================*/
/*                      MCU CONTROL MACROS (STM32F407)                        */
/*============================================================================*/

/* These typically map to CMSIS functions for Cortex-M processors */

/**
 * @brief Enable Global Interrupts.
 * This is a CMSIS function.
 */
#define Global_Int_Enable() __enable_irq()

/**
 * @brief Disable Global Interrupts.
 * This is a CMSIS function.
 */
#define Global_Int_Disable() __disable_irq()

/**
 * @brief Insert a No Operation instruction.
 * This is a CMSIS function.
 */
#define NOP()               __NOP()

/**
 * @brief Wait For Interrupt. Enters sleep mode until an interrupt occurs.
 * This is a CMSIS function.
 */
#define HALT()              __WFI()


/*============================================================================*/
/*                        SAFEGUARD MACROS                                    */
/*============================================================================*/

/**
 * @brief Configure all GPIO pins to a safe initial state (Input, No Pull, Output Data 0).
 * This macro assumes GPIO clocks need to be enabled to write to registers.
 * It targets GPIO ports A through I, common on STM32F407 devices.
 * It fulfills: Data=0, Mode=Input, Disable Pull-up/down, Disable Wakeup (by mode/pull config).
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Enable clocks for all GPIO ports (A-I) to ensure register access is valid */ \
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | \
                         RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | \
                         RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN); \
        /* Ensure clocks are stable (optional, can add __DSB() if strict timing needed) */ \
        (void)RCC->AHB1ENR; /* Read back to ensure flag is set before proceeding (RM recommendation) */ \
        \
        /* Set all pins on ports A-I to General Purpose Input mode (MODER = 00b) */ \
        /* This is the reset state and disables internal pull-up/down and alternate functions */ \
        GPIOA->MODER = 0x00000000UL; \
        GPIOB->MODER = 0x00000000UL; \
        GPIOC->MODER = 0x00000000UL; \
        GPIOD->MODER = 0x00000000UL; \
        GPIOE->MODER = 0x00000000UL; \
        GPIOF->MODER = 0x00000000UL; \
        GPIOG->MODER = 0x00000000UL; \
        GPIOH->MODER = 0x00000000UL; \
        GPIOI->MODER = 0x00000000UL; \
        \
        /* Disable internal pull-up and pull-down resistors (PUPDR = 00b) */ \
        /* This is the reset state */ \
        GPIOA->PUPDR = 0x00000000UL; \
        GPIOB->PUPDR = 0x00000000UL; \
        GPIOC->PUPDR = 0x00000000UL; \
        GPIOD->PUPDR = 0x00000000UL; \
        GPIOE->PUPDR = 0x00000000UL; \
        GPIOF->PUPDR = 0x00000000UL; \
        GPIOG->PUPDR = 0x00000000UL; \
        GPIOH->PUPDR = 0x00000000UL; \
        GPIOI->PUPDR = 0x00000000UL; \
        \
        /* Set output data register to 0. While in input mode, ODR value is internally connected */ \
        /* to the output buffer but doesn't affect the pin state unless switched to output. */ \
        /* Setting it to 0 aligns with the "Data=0" requirement and is harmless. */ \
        GPIOA->ODR = 0x00000000UL; \
        GPIOB->ODR = 0x00000000UL; \
        GPIOC->ODR = 0x00000000UL; \
        GPIOD->ODR = 0x00000000UL; \
        GPIOE->ODR = 0x00000000UL; \
        GPIOF->ODR = 0x00000000UL; \
        GPIOG->ODR = 0x00000000UL; \
        GPIOH->ODR = 0x00000000UL; \
        GPIOI->ODR = 0x00000000UL; \
        \
        /* Clear Output Type, Output Speed, and Alternate Function Registers */ \
        /* (MODER=00b makes these irrelevant, but clearing ensures reset state) */ \
        GPIOA->OTYPER = 0x00000000UL; GPIOB->OTYPER = 0x00000000UL; GPIOC->OTYPER = 0x00000000UL; \
        GPIOD->OTYPER = 0x00000000UL; GPIOE->OTYPER = 0x00000000UL; GPIOF->OTYPER = 0x00000000UL; \
        GPIOG->OTYPER = 0x00000000UL; GPIOH->OTYPER = 0x00000000UL; GPIOI->OTYPER = 0x00000000UL; \
        \
        GPIOA->OSPEEDR = 0x00000000UL; GPIOB->OSPEEDR = 0x00000000UL; GPIOC->OSPEEDR = 0x00000000UL; \
        GPIOD->OSPEEDR = 0x00000000UL; GPIOE->OSPEEDR = 0x00000000UL; GPIOF->OSPEEDR = 0x00000000UL; \
        GPIOG->OSPEEDR = 0x00000000UL; GPIOH->OSPEEDR = 0x00000000UL; GPIOI->OSPEEDR = 0x00000000UL; \
        \
        GPIOA->AFR[0] = 0x00000000UL; GPIOA->AFR[1] = 0x00000000UL; \
        GPIOB->AFR[0] = 0x00000000UL; GPIOB->AFR[1] = 0x00000000UL; \
        GPIOC->AFR[0] = 0x00000000UL; GPIOC->AFR[1] = 0x00000000UL; \
        GPIOD->AFR[0] = 0x00000000UL; GPIOD->AFR[1] = 0x00000000UL; \
        GPIOE->AFR[0] = 0x00000000UL; GPIOE->AFR[1] = 0x00000000UL; \
        GPIOF->AFR[0] = 0x00000000UL; GPIOF->AFR[1] = 0x00000000UL; \
        GPIOG->AFR[0] = 0x00000000UL; GPIOG->AFR[1] = 0x00000000UL; \
        GPIOH->AFR[0] = 0x00000000UL; GPIOH->AFR[1] = 0x00000000UL; \
        GPIOI->AFR[0] = 0x00000000UL; GPIOI->AFR[1] = 0x00000000UL; \
        \
        /* Note on "wakeup registers": STM32 GPIOs don't have specific wakeup registers. */ \
        /* Wakeup from low power modes is typically handled via EXTI lines (configured in SYSCFG) */ \
        /* or specific peripheral wakeups. Disabling pull-ups and setting to input mode */ \
        /* reduces potential for unintended wakeups via floating pins. EXTI configuration */ \
        /* is not part of this basic GPIO safeguard. */ \
        \
    } while(0)

/**
 * @brief Disable common peripherals by clearing their clock enable bits and reset GPIOs.
 * This provides a known state for most peripherals before system initialization.
 * It fulfills: Disable Global IRQ, Disable Timers, PWM (part of timers), WDT (prevent enable),
 * ICU (part of timers), ADC, UART, I2C, SPI. Also sets GPIOs to I/O (Input).
 * Note: Some peripherals like WWDG/IWDG cannot be disabled once enabled without a reset.
 * This macro clears their *enable* bits to prevent accidental activation.
 * It is assumed essential clocks like FLASH, SRAM, SYSCFG might remain enabled by startup code.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        __disable_irq(); \
        \
        /* Disable peripheral clocks by clearing their enable bits in RCC */ \
        /* This is the most effective way to disable peripherals. */ \
        \
        /* AHB1 Peripherals (DMA, CRC) */ \
        RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_CRCEN); \
        \
        /* AHB2 Peripherals (DCMI, CRYP, HASH, RNG) */ \
        RCC->AHB2ENR &= ~(RCC_AHB2ENR_DCMIEN | RCC_AHB2ENR_CRYPEN | RCC_AHB2ENR_HASHEN | RCC_AHB2ENR_RNGEN); \
        \
        /* AHB3 Peripherals (FMC) */ \
        RCC->AHB3ENR &= ~(RCC_AHB3ENR_FMCEN); \
        \
        /* APB1 Peripherals (TIM2-7, 12-14, WWDG, SPI2-3, USART2-3, UART4-5, I2C1-3, CAN1-2, PWR, DAC, UART7-8) */ \
        /* Leave PWR enabled as it's often needed for voltage scaling etc. */ \
        RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN | \
                         RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_TIM13EN | RCC_APB1ENR_TIM14EN | \
                         RCC_APB1ENR_WWDGEN | RCC_APB1ENR_SPI2EN | RCC_APB1ENR_SPI3EN | RCC_APB1ENR_USART2EN | \
                         RCC_APB1ENR_USART3EN | RCC_APB1ENR_UART4EN | RCC_APB1ENR_UART5EN | RCC_APB1ENR_I2C1EN | \
                         RCC_APB1ENR_I2C2EN | RCC_APB1ENR_I2C3EN | RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN | \
                         RCC_APB1ENR_DACEN | RCC_APB1ENR_UART7EN | RCC_APB1ENR_UART8EN); \
        \
        /* APB2 Peripherals (TIM1, 8-11, USART1, 6, ADC1-3, SDIO, SPI1, SYSCFG) */ \
        /* Leave SYSCFG enabled as it's needed for EXTI configuration */ \
        RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_USART6EN | \
                         RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN | RCC_APB2ENR_SDIOEN | \
                         RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN); \
        \
        /* Note on WDT (IWDG/WWDG): They cannot be disabled by clearing the clock enable bit */ \
        /* if they were already enabled. This step only prevents their *initial* activation. */ \
        /* A system reset is typically required to disable an active WDT. */ \
        \
        /* Configure all GPIOs as General Purpose Input (I/O, not special function) */ \
        /* This is redundant if GPIO_SAFEGUARD_Init was called, but included to meet */ \
        /* the requirement for this specific macro as well, ensuring pins are I/O. */ \
        /* Ensure GPIO clocks are enabled to write to registers */ \
        RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | \
                         RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | \
                         RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN); \
        (void)RCC->AHB1ENR; /* Read back */ \
        \
        GPIOA->MODER = 0x00000000UL; GPIOB->MODER = 0x00000000UL; GPIOC->MODER = 0x00000000UL; \
        GPIOD->MODER = 0x00000000UL; GPIOE->MODER = 0x00000000UL; GPIOF->MODER = 0x00000000UL; \
        GPIOG->MODER = 0x00000000UL; GPIOH->MODER = 0x00000000UL; GPIOI->MODER = 0x00000000UL; \
        \
        GPIOA->PUPDR = 0x00000000UL; GPIOB->PUPDR = 0x00000000UL; GPIOC->PUPDR = 0x00000000UL; \
        GPIOD->PUPDR = 0x00000000UL; GPIOE->PUPDR = 0x00000000UL; GPIOF->PUPDR = 0x00000000UL; \
        GPIOG->PUPDR = 0x00000000UL; GPIOH->PUPDR = 0x00000000UL; GPIOI->PUPDR = 0x00000000UL; \
        \
        GPIOA->AFR[0] = 0x00000000UL; GPIOA->AFR[1] = 0x00000000UL; \
        GPIOB->AFR[0] = 0x00000000UL; GPIOB->AFR[1] = 0x00000000UL; \
        GPIOC->AFR[0] = 0x00000000UL; GPIOC->AFR[1] = 0x00000000UL; \
        GPIOD->AFR[0] = 0x00000000UL; GPIOD->AFR[1] = 0x00000000UL; \
        GPIOE->AFR[0] = 0x00000000UL; GPIOE->AFR[1] = 0x00000000UL; \
        GPIOF->AFR[0] = 0x00000000UL; GPIOF->AFR[1] = 0x00000000UL; \
        GPIOG->AFR[0] = 0x00000000UL; GPIOG->AFR[1] = 0x00000000UL; \
        GPIOH->AFR[0] = 0x00000000UL; GPIOH->AFR[1] = 0x00000000UL; \
        GPIOI->AFR[0] = 0x00000000UL; GPIOI->AFR[1] = 0x00000000UL; \
        \
        GPIOA->ODR = 0x00000000UL; GPIOB->ODR = 0x00000000UL; GPIOC->ODR = 0x00000000UL; \
        GPIOD->ODR = 0x00000000UL; GPIOE->ODR = 0x00000000UL; GPIOF->ODR = 0x00000000UL; \
        GPIOG->ODR = 0x00000000UL; GPIOH->ODR = 0x00000000UL; GPIOI->ODR = 0x00000000UL; \
        \
        /* Note: While disabling clocks is generally sufficient, explicitly zeroing */ \
        /* common peripheral control registers (e.g., TIMx->CR1=0, USARTx->CR1=0, etc.) */ \
        /* would require temporarily re-enabling clocks for each peripheral type */ \
        /* before writing, then disabling again. This adds significant complexity and */ \
        /* is usually not necessary for a basic safeguard where the intent is to stop clocks. */ \
        /* The current approach relies on disabling the clocks which powers down the peripherals. */ \
        \
    } while(0)


/*============================================================================*/
/*                         END OF FILE                                        */
/*============================================================================*/

#endif /* MAIN_H */