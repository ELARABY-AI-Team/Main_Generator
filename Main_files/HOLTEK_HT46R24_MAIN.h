/**
 * @file main.h
 * @brief Main project header file with common includes, typedefs, and macros.
 * @author AI
 * @device HOLTEK_HT46R24
 * @creation date: 2025-06-24
 * @standard: MISRA C
 * @copyright: ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef HOLTEK_HT46R24_MAIN_H_
#define HOLTEK_HT46R24_MAIN_H_

/* --- Necessary Standard Includes --- */
/*
 * Note: This list includes all headers explicitly requested by the user.
 * Some headers may not be strictly necessary for a minimal project
 * depending on the features used, but are included as requested.
 */
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

/* --- MCU Specific Include --- */
/*
 * Include the header file provided by the microcontroller vendor/toolchain.
 * This file defines the Special Function Registers (SFRs) and memory map.
 * The exact name may vary depending on the specific compiler and package.
 * Example: For HOLTEK_HT46R24, it's typically named 'ht46r24.h'.
 */
#include <ht46r24.h> // Assume this header is available and correct

/* --- Useful Typedefs --- */
/**
 * @brief 8-bit unsigned integer type.
 */
typedef uint8_t tbyte;

/**
 * @brief 16-bit unsigned integer type.
 */
typedef uint16_t tword;

/**
 * @brief 32-bit unsigned integer type.
 */
typedef uint32_t tlong;

/* --- Core Macros --- */

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register to modify (volatile).
 * @param bit The bit number (0-indexed).
 */
#define SET_BIT(reg, bit)   ((reg) |= (1u << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register to modify (volatile).
 * @param bit The bit number (0-indexed).
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1u << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register to read (volatile).
 * @param bit The bit number (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1u)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register to modify (volatile).
 * @param bit The bit number (0-indexed).
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1u << (bit)))

/* --- MCU Specific Core Macros --- */
/*
 * Macros for common MCU operations based on HOLTEK_HT46R24 architecture.
 * Assuming EMI is bit 7 of INTC0 register.
 * Assuming NOP and HALT are compiler intrinsics.
 */

/**
 * @brief Disables global interrupts.
 * Corresponds to clearing the EMI bit in INTC0.
 */
#define Global_Int_Disable()    (INTC0bits.EMI = 0u) // Assuming bitfield access

/**
 * @brief Enables global interrupts.
 * Corresponds to setting the EMI bit in INTC0.
 */
#define Global_Int_Enable()     (INTC0bits.EMI = 1u) // Assuming bitfield access

/**
 * @brief Executes a No-Operation instruction.
 * Uses the compiler intrinsic __nop().
 */
#define NOP()                   (__nop())

/**
 * @brief Puts the MCU into idle/halt mode, waiting for interrupt.
 * Uses the compiler intrinsic __halt().
 */
#define HALT()                  (__halt())

/* --- SAFEGUARD MACROS --- */
/*
 * These macros are intended to set the MCU to a known, safe state
 * immediately after reset or during initialization.
 * Register names and bit definitions are based on typical HOLTEK_HT46R24
 * architecture as found in datasheets and common usage.
 * ASSUMPTION: All relevant SFRs (PORTA, PORTB, PADS, PBDS, PAPS, PBPS, PWKE,
 * INTC0, TMR0C, TMR1C, WDTC, WDTR, ADCC0, UACR0, I2CC0, SPICR) and their
 * bitfields (.EMI, .TON, .WDTEN, .ADE, .UARTE, .I2CEN, .SPIEN) are correctly
 * defined in the included <ht46r24.h> header.
 */

/**
 * @brief Configures all GPIO ports to a safe, initial state.
 * Sets output data to 0, direction to input, disables pull-high resistors
 * and port wake-up functions for all available ports (A and B assumed).
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set output data to 0 for all ports (PORTA, PORTB) */ \
        PORTA = 0x00u; \
        PORTB = 0x00u; \
        \
        /* Set direction to input (PADS, PBDS = 1 for input) */ \
        /* Assuming all bits in PADS/PBDS control port pins */ \
        PADS = 0xFFu; \
        PBDS = 0xFFu; \
        \
        /* Disable pull-high resistors (PAPS, PBPS = 1 for no pull-up) */ \
        /* Assuming all bits in PAPS/PBPS control pull-ups */ \
        PAPS = 0xFFu; \
        PBPS = 0xFFu; \
        \
        /* Disable port wake-up function (clear relevant bits in PWKE) */ \
        /* Assuming PWKE register controls wake-up for ports A/B or specific pins */ \
        PWKE = 0x00u; \
    } while(0)

/**
 * @brief Disables most MCU peripherals and puts registers into a safe state.
 * Disables global interrupts, timers, PWM, watchdog, ADC, UART, I2C, SPI.
 * Ensures GPIO pins are configured as general purpose I/O (input by default
 * after peripheral disablement and GPIO_SAFEGUARD_Init).
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable global interrupt (Clear EMI bit in INTC0) */ \
        INTC0bits.EMI = 0u; \
        \
        /* Disable Timers (TMR0, TMR1) - Clear TON bits or similar enable bits */ \
        /* Assuming TMRx control registers have TON bits (e.g., bit 0) */ \
        TMR0Cbits.TON = 0u; \
        TMR1Cbits.TON = 0u; \
        \
        /* Disable PWM - Typically part of timers. Disabling timers usually stops PWM. */ \
        /* No separate PWM enable register assumed unless explicitly found in datasheet */ \
        \
        /* Disable Watchdog Timer (WDT) - Requires specific sequence for HT46R24 */ \
        /* Typical Holtek WDT disable sequence: Write keys to WDTR, then clear WDTEN bit in WDTC */ \
        WDTR = 0x5Au; \
        WDTR = 0xA5u; \
        WDTCbits.WDTEN = 0u; /* Assuming WDTEN bit exists in WDTC */ \
        \
        /* Disable ICU (Input Capture Unit) - Often part of timers. */ \
        /* No separate ICU enable register assumed unless explicitly found in datasheet */ \
        \
        /* Disable ADC (Analog-to-Digital Converter) - ADCC0 register, ADE bit (e.g., bit 7) */ \
        ADCC0bits.ADE = 0u; /* Assuming ADE bit exists in ADCC0 */ \
        \
        /* Disable UART - UACR0 register, UARTE bit (e.g., bit 0) */ \
        UACR0bits.UARTE = 0u; /* Assuming UARTE bit exists in UACR0 */ \
        \
        /* Disable I2C - I2CC0 register, I2CEN bit (e.g., bit 7) */ \
        I2CC0bits.I2CEN = 0u; /* Assuming I2CEN bit exists in I2CC0 */ \
        \
        /* Disable SPI communication - SPICR register, SPIEN bit (e.g., bit 7) */ \
        SPICRbits.SPIEN = 0u; /* Assuming SPIEN bit exists in SPICR */ \
        \
        /* Configure all GPIOS as I/O (not special function registers) */ \
        /* This is implicitly handled by disabling peripherals (which frees up pins) */ \
        /* and setting PxDS=1 (input mode) in GPIO_SAFEGUARD_Init. No extra steps needed here. */ \
    } while(0)

#endif /* MAIN_H */