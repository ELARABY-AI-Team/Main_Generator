/**
 * @file main.h
 * @brief A minimal header file for STM32F103C8T6 embedded projects.
 * @author Embedded C Engineer
 * @device STM32F103C8T6
 * @date 2025-06-19
 * @copyright Copyright (c) 2025, [Your Company/Name]. All rights reserved.
 *
 * This header file provides basic includes, typedefs, and core macros
 * commonly used in embedded C development for the specified microcontroller.
 * It also includes safeguard initialization macros to set a safe default
 * state for GPIOs and disable unused peripherals early in the boot process.
 */

#ifndef STM32F103C8T6_MAIN_H_
#define STM32F103C8T6_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 * Standard Library Includes
 *----------------------------------------------------------------------------*/
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

/*------------------------------------------------------------------------------
 * MCU-Specific Includes
 *----------------------------------------------------------------------------*/
// Include the appropriate header file for the STM32F103 series.
// This typically pulls in CMSIS core and peripheral definitions.
#include "stm32f10x.h"

/*------------------------------------------------------------------------------
 * Useful Typedefs
 *----------------------------------------------------------------------------*/
typedef uint8_t  tbyte; /**< 8-bit unsigned integer */
typedef uint16_t tword; /**< 16-bit unsigned integer */
typedef uint32_t tlong; /**< 32-bit unsigned integer */

/*------------------------------------------------------------------------------
 * Core Macros
 *----------------------------------------------------------------------------*/

/**
 * @brief Set a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-31) to set.
 */
#define SET_BIT(reg, bit)     ((reg) |= (1U << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-31) to clear.
 */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1U << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit number (0-31) to get.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-31) to toggle.
 */
#define TOG_BIT(reg, bit)     ((reg) ^= (1U << (bit)))

/**
 * @brief Insert a value into a bit field of a register.
 * @param reg The register to modify.
 * @param field_mask The mask for the bit field.
 * @param field_shift The bit position of the least significant bit of the field.
 * @param val The value to insert (will be masked by the field width).
 */
#define INSERT_FIELD(reg, field_mask, field_shift, val) \
    ((reg) = ((reg) & ~(field_mask)) | (((val) << (field_shift)) & (field_mask)))

/**
 * @brief Disable global interrupts (using CMSIS intrinsic).
 */
#define Global_Int_Disable    __disable_irq()

/**
 * @brief Enable global interrupts (using CMSIS intrinsic).
 */
#define Global_Int_Enable     __enable_irq()

/**
 * @brief Execute No Operation (NOP) instruction (using CMSIS intrinsic).
 */
#define NOP()                 __NOP()

/**
 * @brief Enter Wait for Interrupt (WFI) low-power state (using CMSIS intrinsic).
 */
#define HALT()                __WFI()


/*------------------------------------------------------------------------------
 * SAFEGUARD MACROS
 *
 * These macros provide a basic safe default state for the system.
 * They are typically called early in the boot sequence before main application
 * logic starts.
 *----------------------------------------------------------------------------*/

/**
 * @brief Configure GPIO pins to a safe default state (Analog Input).
 *
 * This macro configures all available GPIO pins on ports A, B, and C
 * to Analog Input mode by clearing their corresponding CRL and CRH
 * registers. This ensures no pins are left floating in inappropriate
 * configurations or driving signals unintentionally before the application
 * explicitly configures them.
 */
#define GPIO_SAFEGUARD_Init() do { \
    /* Ensure GPIO clocks are enabled *before* configuring */ \
    /* (This is typically done by default startup code, but explicit is safer) */ \
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN_Pos); /* Enable GPIOA clock */ \
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN_Pos); /* Enable GPIOB clock */ \
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN_Pos); /* Enable GPIOC clock */ \
    (void)RCC->APB2ENR; /* Read back to ensure clocks are stable (optional barrier) */ \
    \
    /* Configure GPIOA pins to Analog Input (MODE=00, CNF=00) */ \
    GPIOA->CRL = 0x00000000; \
    GPIOA->CRH = 0x00000000; \
    \
    /* Configure GPIOB pins to Analog Input (MODE=00, CNF=00) */ \
    GPIOB->CRL = 0x00000000; \
    GPIOB->CRH = 0x00000000; \
    \
    /* Configure GPIOC pins (C13-C15 on C8T6) to Analog Input (MODE=00, CNF=00) */ \
    /* Note: GPIOC CRL is mostly unused on this package */ \
    GPIOC->CRL = 0x00000000; \
    GPIOC->CRH = 0x00000000; \
    \
    /* Explicitly disable JTAG/SWD if not needed to free up pins (Optional, uncomment if needed) */ \
    /* This requires AFIO clock enabled */ \
    /* SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN_Pos); */ /* Enable AFIO clock */ \
    /* AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_Disable; */ /* Disable SWD/JTAG */ \
    /* CLR_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN_Pos); */ /* Disable AFIO clock if no longer needed */ \
} while(0)

/**
 * @brief Disable clocks for common unused peripherals.
 *
 * This macro clears the enable bits for many peripherals in the
 * RCC APB1ENR, APB2ENR, and AHBENR registers that are typically
 * not needed for minimal application startup. This helps reduce
 * power consumption and prevents unexpected behavior from uninitialized
 * peripherals. Essential clocks like Flash (FLITF), SRAM, and GPIOs
 * (often needed for basic board setup) are typically left enabled
 * (or enabled by `GPIO_SAFEGUARD_Init` if needed).
 */
#define Registers_SAFEGUARD_Init() do { \
    /* Disable clocks for common unused peripherals on APB1 bus */ \
    /* (TIM2-7, WWDG, SPI2/3, USART2/3, I2C1/2, CAN, BKP, PWR, DAC) */ \
    RCC->APB1ENR = 0x00000000; \
    \
    /* Disable clocks for common unused peripherals on APB2 bus */ \
    /* (ADC1/2, TIM1/8, SPI1, USART1). */ \
    /* NOTE: GPIOA/B/C and AFIO clocks are typically enabled by default */ \
    /* or by GPIO_SAFEGUARD_Init. We explicitly clear only the non-GPIO/AFIO bits. */ \
    /* AFIOEN=0, IOPAEN=2, IOPBEN=3, IOPCEN=4, IOPDEN=5, IOPEEN=6, IOPFEN=7, IOPGEN=8 */ \
    /* ADC1EN=9, ADC2EN=10, TIM1EN=11, SPI1EN=12, USART1EN=14, TIM8EN=13 */ \
    RCC->APB2ENR &= ~((1U << RCC_APB2ENR_ADC1EN_Pos)  | (1U << RCC_APB2ENR_ADC2EN_Pos)  | \
                      (1U << RCC_APB2ENR_TIM1EN_Pos)  | (1U << RCC_APB2ENR_TIM8EN_Pos)  | \
                      (1U << RCC_APB2ENR_SPI1EN_Pos)  | (1U << RCC_APB2ENR_USART1EN_Pos)  ); \
    \
    /* Disable clocks for common unused peripherals on AHB bus */ \
    /* (DMA1, DMA2, CRC). */ \
    /* NOTE: FLITF (Flash) and SRAM clocks are essential and typically left enabled. */ \
    /* DMA1EN=0, DMA2EN=1, SRAMEN=2, FLITFEN=4, CRCEN=6, FSMCEN=8, SDIOEN=10 */ \
    RCC->AHBENR &= ~((1U << RCC_AHBENR_DMA1EN_Pos) | (1U << RCC_AHBENR_DMA2EN_Pos) | \
                      (1U << RCC_AHBENR_CRCEN_Pos) ); \
\
} while(0)

/*------------------------------------------------------------------------------
 * Function Prototypes (optional - add application-specific prototypes here)
 *----------------------------------------------------------------------------*/
// Example:
// void SystemClock_Config(void);
// void MX_GPIO_Init(void);

/*------------------------------------------------------------------------------
 * Configuration Constants (optional - add system config constants here)
 *----------------------------------------------------------------------------*/
// Example:
// #define SYS_CLOCK_FREQ_MHZ  72
// #define TICK_RATE_HZ        1000

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */