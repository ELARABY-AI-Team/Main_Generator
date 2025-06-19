/**
 * @file main.h
 * @brief Main header file for HOLTEK HT46R24 microcontroller applications.
 *        Includes essential standard libraries, core types, macros,
 *        and system safeguarding initialization functions.
 * @author AI
 * @device HOLTEK_HT46R24
 * @creation date 2025-06-19
 * @standard MISRA
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef HOLTEK_HT46R24_MAIN_H_
#define HOLTEK_HT46R24_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/*                           Standard Includes                                */
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
/*                             MCU Specific Includes                          */
/*============================================================================*/
/*
 * Include the specific header file provided by the MCU vendor (Holtek).
 * This file contains the definitions for peripheral registers, bits, etc.
 */
#include "ht46r24.h" /* Example: Replace with the actual path/name if different */


/*============================================================================*/
/*                                 Typedefs                                   */
/*============================================================================*/
typedef uint8_t  tbyte; ///< 8-bit unsigned integer
typedef uint16_t tword; ///< 16-bit unsigned integer
typedef uint32_t tlong; ///< 32-bit unsigned integer


/*============================================================================*/
/*                               Core Macros                                  */
/*============================================================================*/
#define SET_BIT(reg, bit)   ((reg) |= (1UL << (bit)))   ///< Set a specific bit in a register
#define CLR_BIT(reg, bit)   ((reg) &= ~(1UL << (bit)))  ///< Clear a specific bit in a register
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1UL)    ///< Get the value of a specific bit in a register
#define TOG_BIT(reg, bit)   ((reg) ^= (1UL << (bit)))   ///< Toggle a specific bit in a register

/*============================================================================*/
/*                             Embedded Macros                                */
/*============================================================================*/

/**
 * @brief Disable Global Interrupts
 *
 * For HT46R24, this clears the EMI (Enable Master Interrupt) bit in INTC0.
 */
#define DI() \
    do { CLR_BIT(INTC0, 7); } while(0)

/**
 * @brief Enable Global Interrupts
 *
 * For HT46R24, this sets the EMI (Enable Master Interrupt) bit in INTC0.
 */
#define EI() \
    do { SET_BIT(INTC0, 7); } while(0)

/**
 * @brief Execute a No-Operation (NOP) instruction
 *
 * This provides a small delay and is often used for timing or debugging.
 */
#define NOP() \
    do { __asm__("nop"); } while(0) /* Use intrinsic if available, otherwise inline assembly */

/**
 * @brief Enter Halt (sleep) mode
 *
 * This puts the microcontroller into a low-power state.
 */
#define HALT() \
    do { __asm__("halt"); } while(0) /* Use intrinsic if available, otherwise inline assembly */

/*============================================================================*/
/*                            SAFEGUARD MACROS                                */
/*============================================================================*/

/**
 * @brief Safeguard Initialization for GPIOs.
 *
 * Configures all available GPIO pins to a safe, known state:
 * - Data output state driven low (0).
 * - Direction configured as input.
 * - Internal pull-up resistors disabled.
 * - Wake-up function disabled.
 *
 * Note: Assumes the existence of ports PA, PB, PC, and their corresponding
 *       control registers (PACP/PBDP/PCDP for direction, PAPH/PBPH/PCPH for pull-up,
 *       PAWKUP/PBWKUP/PCWKUP for wake-up). Adjust or add ports based on the
 *       specific HT46R24 package used if necessary.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set Data to 0 (safe output state if accidentally configured as output) */ \
        PA = 0x00; \
        PB = 0x00; \
        PC = 0x00; \
        \
        /* Configure all pins as Inputs */ \
        /* PACP/PBDP/PCDP controls direction. 1=Output, 0=Input. Set all bits to 0. */ \
        PACP = 0x00; /* PA Direction */ \
        PBDP = 0x00; /* PB Direction */ \
        PCDP = 0x00; /* PC Direction */ \
        \
        /* Disable Pull-up Resistors */ \
        /* PAPH/PBPH/PCPH controls pull-up. 1=Enable, 0=Disable. Set all bits to 0. */ \
        PAPH = 0x00; /* PA Pull-up */ \
        PBPH = 0x00; /* PB Pull-up */ \
        PCPH = 0x00; /* PC Pull-up */ \
        \
        /* Disable Wake-up Function */ \
        /* PAWKUP/PBWKUP/PCWKUP controls wake-up. 1=Enable, 0=Disable. Set all bits to 0. */ \
        PAWKUP = 0x00; /* PA Wake-up */ \
        PBWKUP = 0x00; /* PB Wake-up */ \
        PCWKUP = 0x00; /* PC Wake-up */ \
    } while(0)


/**
 * @brief Safeguard Initialization for MCU Peripherals.
 *
 * Disables commonly used peripherals to ensure a clean state before
 * application-specific initialization.
 * - Disables global interrupts.
 * - Disables Timers (TM0, TM1, TC0, TC1).
 * - Disables PWM (assuming tied to Timers like TC0/TC1).
 * - Disables ADC.
 * - Disables UART communication.
 * - Disables I2C communication.
 * - Disables SPI communication.
 *
 * Note on Watchdog Timer (WDT): The HT46R24 WDT is typically always enabled
 * after reset/configuration options and cannot be disabled via software at runtime
 * in the same way other peripherals can. A specific sequence is needed to reset it
 * periodically, not disable it. This safeguard does NOT attempt to disable the WDT
 * if runtime disabling is not possible for this specific MCU.
 * Note on Input Capture Unit (ICU): HT46R24 timers (TC0/TC1) provide capture
 * functionality. Disabling the timers effectively disables this.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts (Clear EMI bit in INTC0) */ \
        CLR_BIT(INTC0, 7); \
        \
        /* Disable Timers (Time Base & Timer/Counter) */ \
        /* Clear control registers to stop timers */ \
        TM0C0 = 0x00; \
        TM1C0 = 0x00; \
        TC0C0 = 0x00; /* Also disables TC0 PWM if used */ \
        TC1C0 = 0x00; /* Also disables TC1 PWM if used */ \
        \
        /* Disable ADC (Clear ADEN bit in ADCC) */ \
        CLR_BIT(ADCC, 7); \
        \
        /* Disable UART */ \
        /* Clear control registers to disable UART functions */ \
        UARTC0 = 0x00; \
        UARTC1 = 0x00; \
        \
        /* Disable I2C */ \
        /* Clear control register to disable I2C */ \
        I2CC = 0x00; \
        \
        /* Disable SPI */ \
        /* Clear control register to disable SPI */ \
        SPICR = 0x00; \
        \
        /* Note: WDT runtime disable is not typically possible on HT46R24. */ \
        /* Note: ICU functionality is tied to timers; disabling timers handles this. */ \
        \
        /* Ensure pins configured as I/O (this is the default when peripherals are disabled */ \
        /* and GPIO direction is set appropriately, which is done in GPIO_SAFEGUARD_Init). */ \
        /* No extra step needed here beyond disabling peripherals. */ \
        \
    } while(0)


#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */