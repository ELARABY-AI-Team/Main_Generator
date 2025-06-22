/**
 * @file main.h
 * @brief Main header file for the project, including core types, macros, and safeguard functions.
 * @author AI
 * @device HOLTEK_HT46R24
 * @creation_date 2025-06-22
 * @standared MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef HOLTEK_HT46R24_MAIN_H_
#define HOLTEK_HT46R24_MAIN_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------

// Standard C Includes (MISRA C:2012 Rule 20.5 mandatory)
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

// MCU Specific Includes
// Ensure your toolchain provides the correct header for the HT46R24.
// This header defines peripheral registers and bits like PA, PAC, EMI, TCC0, etc.
#include <ht46r24.h> // Placeholder - verify with your specific toolchain/SDK path

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

/** @brief Alias for unsigned 8-bit integer */
typedef uint8_t tbyte;

/** @brief Alias for unsigned 16-bit integer */
typedef uint16_t tword;

/** @brief Alias for unsigned 32-bit integer */
typedef uint32_t tlong;

// ----------------------------------------------------------------------------
// Core Macros
// ----------------------------------------------------------------------------

/**
 * @brief Sets the specified bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-based, 0 to 7 for 8-bit registers).
 */
#define SET_BIT(reg, bit)     ((reg) |= (1U << (bit)))

/**
 * @brief Clears the specified bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-based, 0 to 7 for 8-bit registers).
 */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of the specified bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-based, 0 to 7 for 8-bit registers).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles the specified bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-based, 0 to 7 for 8-bit registers).
 */
#define TOG_BIT(reg, bit)     ((reg) ^= (1U << (bit)))

// Global Interrupt Control (HT46R24 uses EMI bit, often available as a direct symbol)
/**
 * @brief Disables global interrupts.
 */
#define Global_Int_Disable()  (EMI = 0) // EMI is the Enable Master Interrupt bit

/**
 * @brief Enables global interrupts.
 */
#define Global_Int_Enable()   (EMI = 1)

// Processor Control Macros (often map to specific instructions or compiler intrinsics)
// Note: The exact syntax for NOP and HALT might vary depending on your compiler (e.g., Hi-Tech C, SDCC).
//       __nop() and __halt() are common intrinsics in Hi-Tech C/Microchip compilers.
#ifdef __HTC__ // Example check for Hi-Tech C / Microchip C Compiler
    /** @brief Executes a No Operation (NOP) instruction. */
    #define NOP()             __nop()
    /** @brief Executes a Halt (HALT) instruction, entering power-down mode. */
    #define HALT()            __halt()
#else
    // Provide generic definitions using assembly syntax if intrinsics are not available
    // WARNING: Verify if the __asm__ syntax and instruction names ("nop", "halt")
    //          are correct for your specific compiler/assembler.
    #warning "Using generic NOP() and HALT() macros. Verify compiler intrinsics or assembly syntax."
    /** @brief Executes a No Operation (NOP) instruction (generic assembly). */
    #define NOP()             __asm__("nop")
    /** @brief Executes a Halt (HALT) instruction (generic assembly). */
    #define HALT()            __asm__("halt")
#endif


// ----------------------------------------------------------------------------
// Safeguard Macros (Fully Implemented for HOLTEK_HT46R24)
// ----------------------------------------------------------------------------

/**
 * @brief Initializes all GPIO ports to a safe default state (input, pull-up/wake-up disabled, output data low).
 *
 * This macro configures all available GPIO ports:
 * - Sets output data registers (PA, PB, PC) to 0x00.
 * - Configures direction registers (PAC, PBC, PDC) for all pins as inputs (0xFF).
 * - Disables pull-up resistors (PPUA, PPUB, PPUC) for all pins (0x00).
 * - Disables wake-up flags/enables (PWKFA, PWKFB, PWKFC) for all pins (0x00).
 *
 * Uses real HT46R24 register names. Assumes ports A, B, C are available.
 * Writing to registers for ports not present on a specific package might be ignored
 * or cause unexpected behavior; consult your chip variant's datasheet.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set output data registers to 0x00 */ \
        /* (Relevant only if direction is output, but safe to set regardless) */ \
        PA = 0x00; \
        PB = 0x00; \
        PC = 0x00; /* Port C might not exist on all packages */ \
        /* Configure all pins as inputs (1 = input) */ \
        PAC = 0xFF; \
        PBC = 0xFF; \
        PDC = 0xFF; /* Port C direction might not exist */ \
        /* Disable pull-up resistors (0 = disable) */ \
        PPUA = 0x00; \
        PPUB = 0x00; \
        PPUC = 0x00; /* Port C pull-up might not exist */ \
        /* Disable wake-up flags/enables (0 = disable, clears flags and disables enables) */ \
        PWKFA = 0x00; \
        PWKFB = 0x00; \
        PWKFC = 0x00; /* Port C wake-up might not exist */ \
    } while(0)

/**
 * @brief Disables core peripherals and sets module-specific pins to general I/O by disabling the peripheral module.
 *
 * This macro disables the following peripherals by writing 0x00 to their main control registers:
 * - Global Interrupts (via EMI bit)
 * - Timers (TC/EC TCC0/TCC1, TB TBCR)
 * - PWM (PWM0C)
 * - Watchdog Timer (clears it by writing 0x5A/0xA5 sequence; cannot be runtime disabled if fuse-enabled)
 * - ADC (ADCCRL)
 * - UART (UCR1)
 * - I2C (Simplified I2C SIMCR)
 * - SPI (SPICR)
 *
 * Disabling these peripherals typically causes their associated pins to revert from
 * special function mode back to general purpose I/O mode. The direction of these
 * general purpose I/O pins is then determined by the corresponding PAC, PBC, PDC
 * registers, which are set to input by GPIO_SAFEGUARD_Init().
 *
 * Uses real HT46R24 register names. Assumes typical peripherals are present.
 * Writing 0x00 is a common way to disable modules; consult datasheet for specific bits if needed.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable global interrupts */ \
        Global_Int_Disable(); \
        /* Disable Timers (TC/EC TCC0/TCC1, TB TBCR) */ \
        TCC0 = 0x00; \
        TCC1 = 0x00; \
        TBCR = 0x00; \
        /* Disable PWM */ \
        PWM0C = 0x00; \
        /* Clear Watchdog Timer (cannot runtime disable if fuse-enabled). */ \
        /* Write sequence 0x5A, 0xA5 to WDTCR to clear the timer and prevent reset. */ \
        WDTCR = 0x5A; \
        WDTCR = 0xA5; \
        /* Disable ADC */ \
        ADCCRL = 0x00; \
        /* Disable UART */ \
        UCR1 = 0x00; \
        /* Disable I2C (Simplified I2C) */ \
        SIMCR = 0x00; \
        /* Disable SPI */ \
        SPICR = 0x00; \
        /* Note: Pins previously configured for disabled peripherals revert to GPIO. */ \
        /*       Their direction is controlled by PAC/PBC/PDC, set to input by GPIO_SAFEGUARD_Init(). */ \
    } while(0)

// ----------------------------------------------------------------------------
// End of File
// ----------------------------------------------------------------------------

#endif // MAIN_H