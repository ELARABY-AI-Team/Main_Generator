/**
 * @file main.h
 * @brief Main header file for STM32F401RC embedded projects.
 *        Provides core definitions, typedefs, and essential macros.
 * @author AI
 * @device STM32F401RC
 * @creation_date 2025-07-01
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef STM32F401RC_MAIN_H_
#define STM32F401RC_MAIN_H_

/* --- Standard C Includes --- */
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

/* --- MCU Specific Includes --- */
/*
 * Includes the CMSIS device header for STM32F401RC.
 * This header defines peripheral registers and bit definitions.
 */
#include "stm32f401xc.h"

/* --- Useful Typedefs --- */
/** @brief 8-bit unsigned integer type */
typedef uint8_t tbyte;

/** @brief 16-bit unsigned integer type */
typedef uint16_t tword;

/** @brief 32-bit unsigned integer type */
typedef uint32_t tlong;

/* --- Core Macros --- */

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-31) to set.
 */
#define SET_BIT(reg, bit)     ((reg) |= (1UL << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-31) to clear.
 */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1UL << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit position (0-31) to get.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1UL)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-31) to toggle.
 */
#define TOG_BIT(reg, bit)     ((reg) ^= (1UL << (bit)))

/* --- Global Interrupt Control Macros --- */
/*
 * These macros typically rely on CMSIS intrinsic functions defined in
 * core_cmFunc.h. This header is usually included by the device-specific header
 * (stm32f401xc.h) or by the build system's standard includes, so it is not
 * explicitly included here as per requirements.
 */

/**
 * @brief Disables global interrupts.
 *        Relies on __disable_irq() intrinsic.
 */
#define Global_Int_Disable()  __disable_irq()

/**
 * @brief Enables global interrupts.
 *        Relies on __enable_irq() intrinsic.
 */
#define Global_Int_Enable()   __enable_irq()

/* --- Utility Macros --- */
/*
 * These macros may rely on CMSIS intrinsic functions or simple constructs.
 * __NOP() is typically defined in core_cmInstr.h, included similarly to
 * interrupt intrinsics. A simple infinite loop is used for HALT.
 */

/**
 * @brief Executes a No Operation instruction.
 *        Relies on __NOP() intrinsic.
 */
#define NOP()                 __NOP()

/**
 * @brief Halts the program execution indefinitely.
 */
#define HALT()                for(;;){}

/* --- SAFEGUARD MACROS --- */
/*
 * These macros are intended to put the microcontroller into a known, safe state
 * by configuring GPIOs and disabling most peripherals.
 */

/**
 * @brief Configures all general-purpose I/Os (GPIOA to GPIOE) to a safe state.
 *        Sets output data to low, configures pins as inputs with no pull-up/down,
 *        low speed, push-pull type, and disables alternate functions.
 *        Requires GPIO clocks to be enabled *before* calling this macro.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Array of GPIO base pointers for easier iteration (assuming A-E available) */ \
        GPIO_TypeDef *gpio_ports[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE}; \
        uint32_t num_ports = sizeof(gpio_ports) / sizeof(gpio_ports[0]); \
        uint32_t i; \
        \
        /* Ensure GPIO clocks are enabled before configuration */ \
        /* This is often done elsewhere (e.g. in Registers_SAFEGUARD_Init) */ \
        /* but explicitly enabling them here makes this macro self-contained for GPIOs */ \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos); \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos); \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos); \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos); \
        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN_Pos); \
        \
        /* Iterate through enabled GPIO ports (A-E) */ \
        for (i = 0; i < num_ports; i++) \
        { \
            /* Set Output Data Register to 0 */ \
            /* This is done first so if pins were outputs, they go low before changing mode */ \
            gpio_ports[i]->ODR = 0x0000; \
            \
            /* Set Mode Register to Input (0b00) for all pins */ \
            gpio_ports[i]->MODER = 0x00000000; \
            \
            /* Set Output Type Register to Push-Pull (0b0) for all pins */ \
            /* This is the default reset value but set explicitly */ \
            gpio_ports[i]->OTYPER = 0x0000; \
            \
            /* Set Output Speed Register to Low Speed (0b00) for all pins */ \
            /* This is the default reset value but set explicitly */ \
            gpio_ports[i]->OSPEEDR = 0x00000000; \
            \
            /* Set Pull-up/Pull-down Register to No Pull-up/Pull-down (0b00) for all pins */ \
            gpio_ports[i]->PUPDR = 0x00000000; \
            \
            /* Set Alternate Function Registers to AF0 (0b0000) for all pins */ \
            /* AF0 is typically System Function or default GPIO */ \
            gpio_ports[i]->AFR[0] = 0x00000000; /* AFRL */ \
            gpio_ports[i]->AFR[1] = 0x00000000; /* AFRH */ \
        } \
        \
        /* Note: Disabling WAKEUP pins is usually handled at EXTI/PWR level, not per GPIO register. */ \
        /* Disabling EXTI interrupts/events would be part of Registers_SAFEGUARD_Init. */ \
        \
    } while(0)

/**
 * @brief Disables most peripherals and configures system registers to a safe state.
 *        Disables global interrupts, peripheral clocks, and peripheral enable bits.
 *        Calls GPIO_SAFEGUARD_Init to configure pins as general I/O.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable global interrupts */ \
        Global_Int_Disable(); \
        \
        /* Disable clocks for most peripherals in RCC AHB1ENR */ \
        /* Keep essential clocks like FLASH, SRAM, RCC enabled */ \
        /* Keep GPIO clocks enabled as needed for GPIO_SAFEGUARD_Init() */ \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN_Pos);   /* Disable DMA1 clock */ \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_Pos);   /* Disable DMA2 clock */ \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN_Pos);    /* Disable CRC clock */ \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_BKPSRAMEN_Pos);/* Disable BKPSRAM clock */ \
        CLR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_SDIOEN_Pos);   /* Disable SDIO clock */ \
        \
        /* Disable clocks for most peripherals in RCC APB1ENR */ \
        /* Keep PWR enabled as it handles voltage scaling, etc. (assumed essential) */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN_Pos);  /* Disable TIM2 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN_Pos);  /* Disable TIM3 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN_Pos);  /* Disable TIM4 clock */ \
        CLR_BIT(RCC_APB1ENR_TIM5EN_Pos);  /* Disable TIM5 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN_Pos);  /* Disable WWDG clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN_Pos);  /* Disable SPI2 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN_Pos);  /* Disable SPI3 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN_Pos);/* Disable USART2 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN_Pos);  /* Disable I2C1 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN_Pos);  /* Disable I2C2 clock */ \
        CLR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN_Pos);  /* Disable I2C3 clock */ \
        /* Note: UART4, UART5, I2C4, etc. not present on STM32F401RC */ \
        \
        /* Disable clocks for most peripherals in RCC APB2ENR */ \
        /* Keep SYSCFG enabled as it's needed for EXTI configuration, which might be needed later (assumed) */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN_Pos);  /* Disable TIM1 clock */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN_Pos);/* Disable USART1 clock */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN_Pos);/* Disable USART6 clock */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN_Pos);  /* Disable ADC1 clock */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Pos);  /* Disable SPI1 clock */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN_Pos);  /* Disable TIM9 clock */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN_Pos); /* Disable TIM10 clock */ \
        CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN_Pos); /* Disable TIM11 clock */ \
        /* Note: TIM8, SDIO, SPI4, SPI5, SPI6, etc. not present on STM32F401RC */ \
        \
        /* Explicitly disable peripheral enable bits where applicable */ \
        /* This is an extra safety measure in case clocks were not disabled */ \
        /* Disable Timers (TIM1-TIM5, TIM9-TIM11) */ \
        TIM1->CR1 &= ~TIM_CR1_CEN; \
        TIM2->CR1 &= ~TIM_CR1_CEN; \
        TIM3->CR1 &= ~TIM_CR1_CEN; \
        TIM4->CR1 &= ~TIM_CR1_CEN; \
        TIM5->CR1 &= ~TIM_CR1_CEN; \
        TIM9->CR1 &= ~TIM_CR1_CEN; \
        TIM10->CR1 &= ~TIM_CR1_CEN; \
        TIM11->CR1 &= ~TIM_CR1_CEN; \
        /* Note: PWM, Input Capture (ICU) are timer modes, disabled by disabling the timer */ \
        \
        /* Disable Watchdog Timer (WDT) - WWDG can be stopped, IWDG often cannot if started */ \
        WWDG->CR = 0x00; /* Attempt to stop WWDG */ \
        /* Note: Independent Watchdog (IWDG) is clock-independent and may not be disableable once enabled by hardware or option bytes. */ \
        \
        /* Disable Analog to Digital Converter (ADC) */ \
        ADC1->CR2 &= ~ADC_CR2_ADON; \
        \
        /* Disable UART/USART */ \
        USART1->CR1 &= ~USART_CR1_UE; \
        USART2->CR1 &= ~USART_CR1_UE; \
        USART6->CR1 &= ~USART_CR1_UE; \
        \
        /* Disable I2C */ \
        I2C1->CR1 &= ~I2C_CR1_PE; \
        I2C2->CR1 &= ~I2C_CR1_PE; \
        I2C3->CR1 &= ~I2C_CR1_PE; \
        \
        /* Disable SPI communication */ \
        SPI1->CR1 &= ~SPI_CR1_SPE; \
        SPI2->CR1 &= ~SPI_CR1_SPE; \
        SPI3->CR1 &= ~SPI_CR1_SPE; \
        \
        /* Configure all GPIOs as general input/output pins */ \
        /* This macro resets GPIO configurations to a safe input state */ \
        GPIO_SAFEGUARD_Init(); \
        \
        /* Note: EXTI, DMA streams, CRC, SDIO, etc. registers could also be reset/cleared if needed, */ \
        /* but disabling their clocks usually suffices for a basic safeguard. */ \
        \
    } while(0)

#endif /* MAIN_H */