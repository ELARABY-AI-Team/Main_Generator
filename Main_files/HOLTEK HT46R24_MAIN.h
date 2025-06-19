/**
 * @file main.h
 * @brief Main header file for the HT46R24 microcontroller project.
 *        Includes common definitions, types, and essential initialisation safeguards.
 * @author AI
 * @device HOLTEK HT46R24
 * @creation date: 2025-06-19
 * @standard: MISRA
 * @copyright: ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef HOLTEK HT46R24_MAIN_H_
#define HOLTEK HT46R24_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *                      Standard Libraries Inclusion                           *
 *******************************************************************************/

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

/*******************************************************************************
 *                       MCU-Specific Libraries Inclusion                      *
 *******************************************************************************/

// Include the Holtek HT46R24 specific header file provided by the vendor
// This header typically defines the special function registers (SFRs).
#include "ht46r24.h" // Assuming this is the correct vendor header name

/*******************************************************************************
 *                                  Typedefs                                   *
 *******************************************************************************/

/**
 * @brief Unsigned 8-bit integer type.
 */
typedef uint8_t tbyte;

/**
 * @brief Unsigned 16-bit integer type.
 */
typedef uint16_t tword;

/**
 * @brief Unsigned 32-bit integer type.
 */
typedef uint32_t tlong;

/*******************************************************************************
 *                                Core Macros                                  *
 *******************************************************************************/

/**
 * @brief Sets a specific bit in a register.
 * @param[in,out] reg The register to modify.
 * @param[in] bit The bit number (0-indexed) to set.
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param[in,out] reg The register to modify.
 * @param[in] bit The bit number (0-indexed) to clear.
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param[in] reg The register to read from.
 * @param[in] bit The bit number (0-indexed) to get.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param[in,out] reg The register to modify.
 * @param[in] bit The bit number (0-indexed) to toggle.
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))

/*******************************************************************************
 *                         MCU Specific Utility Macros                         *
 *******************************************************************************/

/**
 * @brief Disables global interrupts.
 *        Implemented by clearing the Global Interrupt Master Enable (GMIE) bit in INTC.
 */
#define Global_Int_Disable()    do { INTCbits.GMIE = 0U; } while(0)

/**
 * @brief Enables global interrupts.
 *        Implemented by setting the Global Interrupt Master Enable (GMIE) bit in INTC.
 */
#define Global_Int_Enable()     do { INTCbits.GMIE = 1U; } while(0)

/**
 * @brief Executes a No Operation (NOP) instruction.
 */
#define NOP()                   do { __asm__("nop"); } while(0)

/**
 * @brief Executes a HALT instruction to enter sleep mode.
 */
#define HALT()                  do { __asm__("halt"); } while(0)

/*******************************************************************************
 *                            SAFEGUARD MACROS                               *
 *******************************************************************************/

/**
 * @brief Configures all GPIO ports to a safe default state.
 *        Sets data low (0), configures direction as input, disables pull-ups,
 *        and disables wake-up functions.
 *        This is a critical step at the start of main() or system initialisation.
 */
#define GPIO_SAFEGUARD_Init()   \
    do {                        \
        /* Set data/value low for all ports */ \
        PORTA = 0x00U;          \
        PORTB = 0x00U;          \
        PORTC = 0x00U;          \
                                \
        /* Configure direction as input for all ports (TRIS = 1 means input) */ \
        TRISA = 0xFFU;          \
        TRISB = 0xFFU;          \
        TRISC = 0xFFU;          \
                                \
        /* Disable pull-up resistors (PULC controls Port C pull-ups) */ \
        PULC = 0x00U;           \
                                \
        /* Disable wake-up functions on I/O pins (WAKE register) */ \
        WAKE = 0x00U;           \
    } while(0)

/**
 * @brief Disables core peripheral registers to ensure a known, safe state.
 *        Disables global interrupts, timers, PWM, WDT, ADC, UART, SPI,
 *        and configures GPIOs away from special functions.
 *        This is a critical step at the start of main() or system initialisation.
 */
#define Registers_SAFEGUARD_Init()  \
    do {                            \
        /* Disable global interrupts */ \
        INTCbits.GMIE = 0U;         \
                                    \
        /* Disable Timers (Timer 0 and Timer 1 control registers) */ \
        TM0C = 0x00U;               \
        TM1C = 0x00U;               \
                                    \
        /* Disable PWM (PWM 0 and PWM 1 control registers) */ \
        PWM0C = 0x00U;              \
        PWM1C = 0x00U;              \
                                    \
        /* Disable Watchdog Timer (Clear WDTEN bit in WDTC) */ \
        WDTCbits.WDTEN = 0U;        \
                                    \
        /* Disable ADC (ADC control registers) */ \
        ADCR0 = 0x00U;              \
        ADCR1 = 0x00U;              \
                                    \
        /* Disable UART (UART control registers) */ \
        UARTCR0 = 0x00U;            \
        UARTCR1 = 0x00U;            \
                                    \
        /* Disable SPI communication (SPI control registers) */ \
        SPICR0 = 0x00U;             \
        SPICR1 = 0x00U;             \
                                    \
        /* Configure GPIOs as standard I/O, not special functions */ \
        /* Clear Peripheral Source Mapping registers (PSMFC) */ \
        PSMFC0 = 0x00U;             \
        PSMFC1 = 0x00U;             \
        PSMFC2 = 0x00U;             \
                                    \
        /* Explicitly set TRIS registers to input (0xFF) after clearing SFs for safety */ \
        TRISA = 0xFFU;              \
        TRISB = 0xFFU;              \
        TRISC = 0xFFU;              \
                                    \
    } while(0)


#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */