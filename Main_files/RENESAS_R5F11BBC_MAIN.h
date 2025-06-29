/**
 * @file main.h
 * @brief Minimal and essential header file for RENESAS_R5F11BBC embedded projects.
 * @author AI
 * @device RENESAS_R5F11BBC
 * @creation_date 2025-06-29
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef RENESAS_R5F11BBC_MAIN_H_
#define RENESAS_R5F11BBC_MAIN_H_

/* --- Includes --- */

/* Standard C Libraries (as requested) */
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

/* MCU-Specific Includes */
/*
 * Assumption: This is the primary header file defining peripheral registers
 * for the RENESAS R5F11BBC microcontroller. Please verify the exact path and name
 * for your specific toolchain and device package.
 */
#include "ior5f11bb.h" /* Please change this if your MCU header name is different */

/* --- Typedefs --- */

/** @brief 8-bit unsigned integer */
typedef uint8_t tbyte;

/** @brief 16-bit unsigned integer */
typedef uint16_t tword;

/** @brief 32-bit unsigned integer */
typedef uint32_t tlong;

/* --- Core Macros --- */

/**
 * @brief Set a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/*
 * Note: Macros for global interrupt control (DI, EI) and system control (NOP, HALT)
 * are often compiler intrinsics or require specific headers not covered by the
 * constraints (e.g., headers defining __nop(), __disable_interrupt()).
 * Standard Renesas RL78 uses DI/EI instructions. Implementation as simple macros
 * without proper intrinsic headers or inline assembly is not portable or safe.
 * Therefore, these are not defined here per the constraints.
 * Global interrupts are typically controlled via __disable_interrupt() and __enable_interrupt().
 */

/* --- Safeguard Macros --- */

/**
 * @brief Safeguard macro to initialize all GPIO ports to a safe, known state.
 * Configures all ports to output 0 (data low), then sets them as inputs,
 * disables pull-up resistors, and disables wake-up/special functions on pins.
 *
 * Note: Assumes the existence of Px, PMx, PUPx, and PMCx registers for ports 0-15.
 *       Please verify available ports and register names for R5F11BBC.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set all port output data registers to 0 */ \
        P0 = 0x00U; /* Port 0 data low */ \
        P1 = 0x00U; /* Port 1 data low */ \
        P2 = 0x00U; /* Port 2 data low */ \
        P3 = 0x00U; /* Port 3 data low */ \
        P4 = 0x00U; /* Port 4 data low */ \
        P5 = 0x00U; /* Port 5 data low */ \
        P6 = 0x00U; /* Port 6 data low */ \
        P7 = 0x00U; /* Port 7 data low */ \
        P8 = 0x00U; /* Port 8 data low */ \
        P9 = 0x00U; /* Port 9 data low */ \
        P10 = 0x00U; /* Port 10 data low */ \
        P11 = 0x00U; /* Port 11 data low */ \
        P12 = 0x00U; /* Port 12 data low */ \
        P13 = 0x00U; /* Port 13 data low */ \
        P14 = 0x00U; /* Port 14 data low */ \
        P15 = 0x00U; /* Port 15 data low */ /* Assumption: Ports P0-P15 exist. Please change if your MCU has fewer. */ \
\
        /* Set all port mode registers to 0xFF (configure all pins as inputs) */ \
        PM0 = 0xFFU; /* Port 0 input mode */ \
        PM1 = 0xFFU; /* Port 1 input mode */ \
        PM2 = 0xFFU; /* Port 2 input mode */ \
        PM3 = 0xFFU; /* Port 3 input mode */ \
        PM4 = 0xFFU; /* Port 4 input mode */ \
        PM5 = 0xFFU; /* Port 5 input mode */ \
        PM6 = 0xFFU; /* Port 6 input mode */ \
        PM7 = 0xFFU; /* Port 7 input mode */ \
        PM8 = 0xFFU; /* Port 8 input mode */ \
        PM9 = 0xFFU; /* Port 9 input mode */ \
        PM10 = 0xFFU; /* Port 10 input mode */ \
        PM11 = 0xFFU; /* Port 11 input mode */ \
        PM12 = 0xFFU; /* Port 12 input mode */ \
        PM13 = 0xFFU; /* Port 13 input mode */ \
        PM14 = 0xFFU; /* Port 14 input mode */ \
        PM15 = 0xFFU; /* Port 15 input mode */ /* Assumption: Ports P0-P15 exist. Please change if your MCU has fewer. */ \
\
        /* Disable internal pull-up resistors for all ports */ \
        PUP0 = 0x00U; /* Port 0 pull-up disable */ \
        PUP1 = 0x00U; /* Port 1 pull-up disable */ \
        PUP2 = 0x00U; /* Port 2 pull-up disable */ \
        PUP3 = 0x00U; /* Port 3 pull-up disable */ \
        PUP4 = 0x00U; /* Port 4 pull-up disable */ \
        PUP5 = 0x00U; /* Port 5 pull-up disable */ \
        PUP6 = 0x00U; /* Port 6 pull-up disable */ \
        PUP7 = 0x00U; /* Port 7 pull-up disable */ \
        PUP8 = 0x00U; /* Port 8 pull-up disable */ \
        PUP9 = 0x00U; /* Port 9 pull-up disable */ \
        PUP10 = 0x00U; /* Port 10 pull-up disable */ \
        PUP11 = 0x00U; /* Port 11 pull-up disable */ \
        PUP12 = 0x00U; /* Port 12 pull-up disable */ \
        PUP13 = 0x00U; /* Port 13 pull-up disable */ \
        PUP14 = 0x00U; /* Port 14 pull-up disable */ \
        PUP15 = 0x00U; /* Port 15 pull-up disable */ /* Assumption: Ports P0-P15 exist. Please change if your MCU has fewer. */ \
\
        /* Disable pin special functions, including wake-up function, for all ports */ \
        /* Setting PMCx to 0 configures the pin as standard I/O (controlled by PMx/Px). */ \
        PMC0 = 0x00U; /* Port 0 no special function */ \
        PMC1 = 0x00U; /* Port 1 no special function */ \
        PMC2 = 0x00U; /* Port 2 no special function */ \
        PMC3 = 0x00U; /* Port 3 no special function */ \
        PMC4 = 0x00U; /* Port 4 no special function */ \
        PMC5 = 0x00U; /* Port 5 no special function */ \
        PMC6 = 0x00U; /* Port 6 no special function */ \
        PMC7 = 0x00U; /* Port 7 no special function */ \
        PMC8 = 0x00U; /* Port 8 no special function */ \
        PMC9 = 0x00U; /* Port 9 no special function */ \
        PMC10 = 0x00U; /* Port 10 no special function */ \
        PMC11 = 0x00U; /* Port 11 no special function */ \
        PMC12 = 0x00U; /* Port 12 no special function */ \
        PMC13 = 0x00U; /* Port 13 no special function */ \
        PMC14 = 0x00U; /* Port 14 no special function */ \
        PMC15 = 0x00U; /* Port 15 no special function */ /* Assumption: Ports P0-P15 exist. Please change if your MCU has fewer. */ \
    } while(0)

/**
 * @brief Safeguard macro to disable common peripherals and interrupts.
 * Disables global interrupts, timers, PWM, WDT, ICU, ADC, UART, I2C, SPI,
 * and ensures peripheral pins are configured as standard I/O.
 *
 * Note: Assumes common RL78 register names and peripheral instances (e.g., TAU0, ADC, UART0, IIC0, SPI0).
 *       Please verify available peripherals and register names for R5F11BBC.
 *       Disabling WDT requires a specific sequence and depends on its initial state.
 *       The method shown for WDT attempts to stop it, but might not work if
 *       configured for continuous operation without window.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        /* Standard Renesas way is often __disable_interrupt() intrinsic (DI instruction). */ \
        /* If PSW is directly accessible via header, clear the IE bit. */ \
        /* Assuming PSW bitfield access is available via included header. */ \
        PSWbits.IE = 0U; /* Disable global interrupts (Assumption: PSWbits.IE exists) */ /* Please change if bitfield access is different or use __disable_interrupt() if available/allowed */ \
\
        /* Disable Timers (Timer Array Units - TAU) and PWM (often part of TAU) */ \
        /* Stop all TAU channels (assuming TAU0, channels 0-7) */ \
        TAU0.TAT0 = 0U; /* Stop TAU0 channel 0 */ \
        TAU0.TAT1 = 0U; /* Stop TAU0 channel 1 */ \
        TAU0.TAT2 = 0U; /* Stop TAU0 channel 2 */ \
        TAU0.TAT3 = 0U; /* Stop TAU0 channel 3 */ \
        TAU0.TAT4 = 0U; /* Stop TAU0 channel 4 */ \
        TAU0.TAT5 = 0U; /* Stop TAU0 channel 5 */ \
        TAU0.TAT6 = 0U; /* Stop TAU0 channel 6 */ \
        TAU0.TAT7 = 0U; /* Stop TAU0 channel 7 */ /* Assumption: TAU0 exists with channels 0-7. Please change if different. */ \
        /* Mask all TAU interrupts */ \
        TMMK00 = 1U; /* TAU0 Channel 0 Interrupt Mask */ \
        TMMK01 = 1U; /* TAU0 Channel 1 Interrupt Mask */ \
        TMMK02 = 1U; /* TAU0 Channel 2 Interrupt Mask */ \
        TMMK03 = 1U; /* TAU0 Channel 3 Interrupt Mask */ \
        TMMK04 = 1U; /* TAU0 Channel 4 Interrupt Mask */ \
        TMMK05 = 1U; /* TAU0 Channel 5 Interrupt Mask */ \
        TMMK06 = 1U; /* TAU0 Channel 6 Interrupt Mask */ \
        TMMK07 = 1U; /* TAU0 Channel 7 Interrupt Mask */ /* Assumption: TMMK00-TMMK07 exist. Please change if different. */ \
\
        /* Disable Watchdog Timer (WDT) */ \
        /* WDT disable requires a specific sequence: write 0x80, then 0x00 to WDTC within 4 clocks. */ \
        /* Note: If WDT is configured for continuous mode without window via hardware option bytes, it cannot be disabled. */ \
        WDTC = 0x80U; /* WDT unlock command (Assumption: WDTC register exists) */ /* Please change if register name is different */ \
        WDTC = 0x00U; /* WDT stop command (within 4 clocks of previous write) */ /* Please change if register name is different */ \
\
        /* Disable Input Capture Unit (ICU) - often part of TAU, covered by TMMK masks */ \
        /* No separate ICU specific registers typically needed if integrated with TAU/timers. */ \
\
        /* Disable Analog to Digital Converter (ADC) */ \
        ADM0bits.ADCS = 0U; /* Disable ADC conversion (Assumption: ADM0bits.ADCS exists) */ /* Please change if register/bit name is different */ \
        ADMK = 1U; /* Mask ADC interrupt (Assumption: ADMK exists) */ /* Please change if register name is different */ \
\
        /* Disable UART (Serial Interface - CSI/UART/IIC) */ \
        /* Assuming UART0 exists. Disable TX and RX, mask interrupts. */ \
        STMK0 = 1U; /* Mask UART0 Send Interrupt (Assumption: STMK0 exists) */ /* Please change if register name is different */ \
        SRMK0 = 1U; /* Mask UART0 Receive Interrupt (Assumption: SRMK0 exists) */ /* Please change if register name is different */ \
        SSERMK0 = 1U; /* Mask UART0 Error Interrupt (Assumption: SSERMK0 exists) */ /* Please change if register name is different */ \
        /* Disable transmit/receive operations - depends on specific SImn/UARmn registers. */ \
        /* Clearing SCRmn enables might be complex without knowing mode. Masking interrupts is safer. */ \
        /* If SCRmn.TXE and SCRmn.RXE exist, clear them: */ \
        SCR0bits.TXE = 0U; /* Disable UART0 TX (Assumption: SCR0bits.TXE exists) */ /* Please change if different */ \
        SCR0bits.RXE = 0U; /* Disable UART0 RX (Assumption: SCR0bits.RXE exists) */ /* Please change if different */ \
\
        /* Disable I2C (Serial Interface - IIC) */ \
        /* Assuming IIC0 exists. Mask interrupts. */ \
        IICRMK0 = 1U; /* Mask IIC0 Receive Interrupt (Assumption: IICRMK0 exists) */ /* Please change if different */ \
        IICBMK0 = 1U; /* Mask IIC0 Bus Interrupt (Assumption: IICBMK0 exists) */ /* Please change if different */ \
        IICWMK0 = 1U; /* Mask IIC0 Wait Interrupt (Assumption: IICWMK0 exists) */ /* Please change if different */ \
        /* Disable IIC operation - depends on IICCTLmn registers. Masking interrupts is safer. */ \
        /* If IICCTLmn.SPIE (Stop Interrupt Enable) or similar exists, clear it: */ \
        /* Example: Clear IICCTL00 bits to reset state */ \
        IICCTL00 = 0x00U; /* Reset IIC0 control register 0 (Assumption: IICCTL00 exists) */ /* Please change if different */ \
        IICCTL01 = 0x00U; /* Reset IIC0 control register 1 (Assumption: IICCTL01 exists) */ /* Please change if different */ \
\
        /* Disable SPI (Serial Interface - CSI) */ \
        /* Assuming SPI0 exists. Mask interrupts. */ \
        SPIMK00 = 1U; /* Mask SPI0 Receive Interrupt (Assumption: SPIMK00 exists) */ /* Please change if different */ \
        SPIMK01 = 1U; /* Mask SPI0 Transmit Interrupt (Assumption: SPIMK01 exists) */ /* Please change if different */ \
        /* Disable SPI operation - depends on SPICMn/SPICTMn registers. Masking interrupts is safer. */ \
        /* Example: Clear control registers to reset state */ \
        SPICM00 = 0x00U; /* Reset SPI0 control register 0 (Assumption: SPICM00 exists) */ /* Please change if different */ \
        SPICTM00 = 0x00U; /* Reset SPI0 control register 1 (Assumption: SPICTM00 exists) */ /* Please change if different */ \
\
        /* Configure all GPIOs as standard Input/Output pins (not special functions) */ \
        /* This is achieved by clearing the Port Mode Control (PMC) registers. */ \
        /* Note: This step is redundant if GPIO_SAFEGUARD_Init() was called just before, */ \
        /* as GPIO_SAFEGUARD_Init() already clears all PMCx registers. Included here */ \
        /* for robustness in case this macro is called standalone. */ \
        PMC0 = 0x00U; /* Port 0 no special function */ \
        PMC1 = 0x00U; /* Port 1 no special function */ \
        PMC2 = 0x00U; /* Port 2 no special function */ \
        PMC3 = 0x00U; /* Port 3 no special function */ \
        PMC4 = 0x00U; /* Port 4 no special function */ \
        PMC5 = 0x00U; /* Port 5 no special function */ \
        PMC6 = 0x00U; /* Port 6 no special function */ \
        PMC7 = 0x00U; /* Port 7 no special function */ \
        PMC8 = 0x00U; /* Port 8 no special function */ \
        PMC9 = 0x00U; /* Port 9 no special function */ \
        PMC10 = 0x00U; /* Port 10 no special function */ \
        PMC11 = 0x00U; /* Port 11 no special function */ \
        PMC12 = 0x00U; /* Port 12 no special function */ \
        PMC13 = 0x00U; /* Port 13 no special function */ \
        PMC14 = 0x00U; /* Port 14 no special function */ \
        PMC15 = 0x00U; /* Port 15 no special function */ /* Assumption: Ports P0-P15 exist. Please change if your MCU has fewer. */ \
    } while(0)

#endif /* MAIN_H_ */

/* End of file */