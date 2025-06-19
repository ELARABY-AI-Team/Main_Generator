/**
 * @file main.h
 * @brief Main include file for project setup and common definitions.
 * @author AI
 * @device RENESAS_R5F11BBC (RL78/G1F)
 * @creation date 2025-06-19
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef RENESAS_R5F11BBC_MAIN_H_
#define RENESAS_R5F11BBC_MAIN_H_

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

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

/* MCU Specific Includes */
/* Include the Renesas provided header for peripheral register definitions */
#include "iodefine.h"
/* Include intrinsic functions header for __DI, __EI, __nop */
#include "renesas.h"

/*
 ******************************************************************************
 * Type Definitions
 ******************************************************************************
 */

typedef uint8_t  tbyte; /**< 8-bit unsigned integer type */
typedef uint16_t tword; /**< 16-bit unsigned integer type */
typedef uint32_t tlong; /**< 32-bit unsigned integer type */

/*
 ******************************************************************************
 * Core Macros
 ******************************************************************************
 */

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-indexed) to set.
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-indexed) to clear.
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the state of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit position (0-indexed) to get.
 * @return The state of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-indexed) to toggle.
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))

/*
 ******************************************************************************
 * MCU Specific Utility Macros
 ******************************************************************************
 */

/**
 * @brief Disables global interrupts.
 *        Uses Renesas intrinsic function.
 */
#define Global_Int_Disable()    __DI()

/**
 * @brief Enables global interrupts.
 *        Uses Renesas intrinsic function.
 */
#define Global_Int_Enable()     __EI()

/**
 * @brief Inserts a No Operation instruction.
 *        Uses Renesas intrinsic function.
 */
#define NOP()                   __nop()

/**
 * @brief Halts CPU execution until an interrupt occurs.
 *        Uses Renesas intrinsic function.
 */
#define HALT()                  __halt()


/*
 ******************************************************************************
 * Safeguard Initialization Macros (Production Ready)
 *
 * These macros configure essential peripherals to a known, safe state after reset.
 * Designed for use early in the startup code before specific peripheral drivers
 * are initialized.
 ******************************************************************************
 */

/**
 * @brief Configures all available GPIO pins to a safe default state (Input, Low Data, No Pull-up/Wake-up).
 *        This sets the pin direction to input, sets the output latch low (though ignored in input mode),
 *        and disables pull-up resistors and wake-up functions.
 *        Applies to Port P0 through P15 and other relevant ports like PIO.
 *        Assumes typical RL78/G1F port structure as defined in iodefine.h.
 *        Note: Not all ports P0-P15 might be available on all specific R5F11BBC package variants.
 *              The iodefine.h file defines which port registers exist.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set Port Data Registers (Output Latch) to Low (0x00) */ \
        /* Though ignored when pin is configured as input, setting to 0 is safe */ \
        P0  = 0x00U; \
        P1  = 0x00U; \
        P2  = 0x00U; \
        P3  = 0x00U; \
        P4  = 0x00U; \
        P5  = 0x00U; \
        P6  = 0x00U; \
        P7  = 0x00U; \
        P8  = 0x00U; \
        P9  = 0x00U; \
        P10 = 0x00U; \
        P11 = 0x00U; \
        P12 = 0x00U; \
        P13 = 0x00U; \
        P14 = 0x00U; \
        P15 = 0x00U; \
        /* PIO register might exist for specific pins, set to 0 */ \
        /* Example: PIO = 0x00U; (if PIO register is defined) */ \
\
        /* Set Port Mode Registers to Input (0xFF for 8-bit ports) */ \
        /* 1: Input, 0: Output for PM registers */ \
        PM0  = 0xFFU; \
        PM1  = 0xFFU; \
        PM2  = 0xFFU; \
        PM3  = 0xFFU; \
        PM4  = 0xFFU; \
        PM5  = 0xFFU; \
        PM6  = 0xFFU; \
        PM7  = 0xFFU; \
        PM8  = 0xFFU; \
        PM9  = 0xFFU; \
        PM10 = 0xFFU; \
        PM11 = 0xFFU; \
        PM12 = 0xFFU; \
        PM13 = 0xFFU; \
        PM14 = 0xFFU; \
        PM15 = 0xFFU; \
        /* PMCIO register might exist for specific pins, set to 0xFF */ \
        /* Example: PMCIO = 0xFFU; (if PMCIO register is defined) */ \
\
        /* Disable Pull-Up Resistors (0x00) */ \
        /* 1: Pull-up enabled, 0: Pull-up disabled for PU registers */ \
        PU0  = 0x00U; \
        PU1  = 0x00U; \
        PU2  = 0x00U; \
        PU3  = 0x00U; \
        PU4  = 0x00U; \
        PU5  = 0x00U; \
        PU6  = 0x00U; \
        PU7  = 0x00U; \
        PU8  = 0x00U; \
        PU9  = 0x00U; \
        PU10 = 0x00U; \
        PU11 = 0x00U; \
        PU12 = 0x00U; \
        PU13 = 0x00U; \
        PU14 = 0x00U; \
        PU15 = 0x00U; \
\
        /* Disable Wake-up Registers (0x00) */ \
        /* 1: Wake-up enabled, 0: Wake-up disabled for W registers */ \
        W0  = 0x00U; \
        W1  = 0x00U; \
        W2  = 0x00U; \
        W3  = 0x00U; \
        W4  = 0x00U; \
        W5  = 0x00U; \
        W6  = 0x00U; \
        W7  = 0x00U; \
        W8  = 0x00U; \
        W9  = 0x00U; \
        W10 = 0x00U; \
        W11 = 0x00U; \
        W12 = 0x00U; \
        W13 = 0x00U; \
        W14 = 0x00U; \
        W15 = 0x00U; \
\
    } while(0)

/**
 * @brief Disables common peripherals and configures pins for general I/O.
 *        This macro disables global interrupts, stops and disables timer/serial modules,
 *        disables ADC, I2C, SPI, and ensures GPIOs are configured as digital I/O.
 *        Note on WDT: On RL78, the Watchdog Timer is typically configured by option bytes and
 *        cannot be disabled via software registers after boot if enabled. This macro does
 *        not attempt to disable the WDT, but rather focuses on peripherals that *can*
 *        be controlled via registers post-boot. If WDT is enabled by option bytes,
 *        the application is responsible for servicing it.
 *        This assumes typical RL78/G1F peripherals controlled by PER0, TAU0, SAU0, IICA0, ADM0, etc.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        __DI(); \
\
        /* Disable common Peripheral Modules via PER0 register */ \
        /* This disables module clocks/operation for listed peripherals */ \
        /* Bit 7: DTCEN, Bit 6: ELCLEN, Bit 5: ADCEN, Bit 4: TMKAEN, Bit 3: SAU0EN, Bit 2: SAU1EN, Bit 1: TAU0EN, Bit 0: TAU1EN */ \
        /* Setting PER0 to 0x00 disables all these modules */ \
        PER0 = 0x00U; \
\
        /* Stop Timer Array Unit 0 (TAU0) channels */ \
        /* TT0 bits stop corresponding channels (0-7) */ \
        TT0 = 0xFFU; /* Stop all 8 channels of TAU0 if implemented */ \
        /* Note: If more channels/units (TAU1) exist, they would need similar stopping */ \
\
        /* Stop Serial Array Unit 0 (SAU0) channels */ \
        /* ST0 bits stop corresponding channels (0-7) used for UART, SPI, CSI */ \
        ST0 = 0xFFU; /* Stop all 8 channels of SAU0 if implemented */ \
        /* Note: If more channels/units (SAU1) exist, they would need similar stopping */ \
\
        /* Disable I2C module 0 (IICA0) operation */ \
        /* ICENA0 bit (Bit 7) in IICA0CTL0 register */ \
        IICA0CTL0 = 0x00U; /* Clear the control register, disabling I2C */ \
\
        /* Disable ADC module operation */ \
        /* ADCE bit (Bit 7) in ADM0 register */ \
        ADM0 = 0x00U; /* Clear the mode register, disabling ADC */ \
\
        /* Watchdog Timer (WDT) Note: */ \
        /* On RL78, WDT is typically controlled by option bytes and cannot be */ \
        /* disabled via software registers after boot if enabled. This macro */ \
        /* does not interact with the WDT configuration (WDTE register) as */
        /* attempting to write certain values after the WDT is running can */
        /* be complex and may not achieve true "disabling". The application */
        /* should be aware of WDT state from option bytes and service it if needed. */
\
        /* Ensure GPIOs are configured as digital I/O (inputs by default) */ \
        /* This step is also in GPIO_SAFEGUARD_Init, repeated here for robustness */ \
        /* as disabling peripherals can sometimes affect pin state. */ \
        PM0  = 0xFFU; PM1  = 0xFFU; PM2  = 0xFFU; PM3  = 0xFFU; \
        PM4  = 0xFFU; PM5  = 0xFFU; PM6  = 0xFFU; PM7  = 0xFFU; \
        PM8  = 0xFFU; PM9  = 0xFFU; PM10 = 0xFFU; PM11 = 0xFFU; \
        PM12 = 0xFFU; PM13 = 0xFFU; PM14 = 0xFFU; PM15 = 0xFFU; \
        /* PMCIO = 0xFFU; (if PMCIO exists) */ \
\
        /* All other peripheral registers are typically in their reset state (disabled) */ \
        /* unless enabled by option bytes or prior code execution. The steps above */ \
        /* cover the most common configurable peripherals that might be active. */ \
\
    } while(0)


/*
 ******************************************************************************
 * Global Variables (if any - none required for this minimal header)
 ******************************************************************************
 */

/*
 ******************************************************************************
 * Function Prototypes (if any - none required for this minimal header)
 ******************************************************************************
 */


#endif /* MAIN_H_ */