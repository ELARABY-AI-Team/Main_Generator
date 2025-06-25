/**
 * @file main.h
 * @brief Minimal and essential header file for embedded projects.
 * @author AI
 * @device RENESAS R5F11BBC (RL78/G14)
 * @creation_date 2025-06-25
 * @standared MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef RENESAS_R5F11BBC_MAIN_H_
#define RENESAS_R5F11BBC_MAIN_H_

/* --- Standard Includes --- */
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
 * Note: This header defines the specific registers for the R5F11BBC.
 * It must be provided by the Renesas toolchain (e.g., CubeSuite+, e2 studio).
 * Do not include headers for intrinsic functions (e.g., <renesas/intrinsics.h>) as requested.
 */
#include "iodefine.h" // Renesas Standard I/O Register Definition Header

/* --- Useful Typedefs --- */
typedef uint8_t  tbyte; /**< 8-bit unsigned integer */
typedef uint16_t tword; /**< 16-bit unsigned integer */
typedef uint32_t tlong; /**< 32-bit unsigned integer */

/* --- Core Macros --- */

/**
 * @brief Set a specific bit in a register.
 * @param[in,out] reg The register variable.
 * @param[in] bit The bit position (0-indexed).
 */
#define SET_BIT(reg, bit)   do { (reg) |= (1U << (bit)); } while(0)

/**
 * @brief Clear a specific bit in a register.
 * @param[in,out] reg The register variable.
 * @param[in] bit The bit position (0-indexed).
 */
#define CLR_BIT(reg, bit)   do { (reg) &= ~(1U << (bit)); } while(0)

/**
 * @brief Get the value of a specific bit in a register.
 * @param[in] reg The register variable.
 * @param[in] bit The bit position (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param[in,out] reg The register variable.
 * @param[in] bit The bit position (0-indexed).
 */
#define TOG_BIT(reg, bit)   do { (reg) ^= (1U << (bit)); } while(0)

/* --- Common Embedded Macros --- */
/*
 * Note: Global interrupt control (DI/EI) and instruction macros (HALT/NOP)
 * often rely on compiler-specific intrinsics or inline assembly.
 * Although <renesas/intrinsics.h> is not included as requested,
 * the compiler typically provides built-in support for DI(), EI(), HALT(), NOP().
 * We define the macros assuming this compiler support.
 */

/**
 * @brief Disable global interrupts.
 */
#define DI()    do { __asm("di"); } while(0)

/**
 * @brief Enable global interrupts.
 */
#define EI()    do { __asm("ei"); } while(0)

/**
 * @brief Execute the HALT instruction (enter power-saving mode).
 */
#define HALT()  do { __asm("halt"); } while(0)

/**
 * @brief Execute the NOP instruction (no operation).
 */
#define NOP()   do { __asm("nop"); } while(0)


/* --- SAFEGUARD MACROS --- */

/**
 * @brief Safely initialize all GPIO pins to a default input state with pull-ups disabled.
 *        Sets data=0, configures as input, disables pull-ups, disables special functions.
 *        Uses actual registers for RENESAS RL78/G14 (R5F11BBC).
 */
#define GPIO_SAFEGUARD_Init() do {                                            \
    /* Disable global interrupts during safeguard */                          \
    DI();                                                                     \
                                                                              \
    /* Set all Port Data registers to 0 (safe low state) */                   \
    P0  = 0x00; P1  = 0x00; P2  = 0x00; P3  = 0x00; P4  = 0x00;                \
    P5  = 0x00; P6  = 0x00; P7  = 0x00; P8  = 0x00; P9  = 0x00;                \
    P10 = 0x00; P11 = 0x00; P12 = 0x00; P13 = 0x00; P14 = 0x00;               \
                                                                              \
    /* Configure all Port Mode registers to 0xFF (all pins as input) */       \
    PM0  = 0xFF; PM1  = 0xFF; PM2  = 0xFF; PM3  = 0xFF; PM4  = 0xFF;            \
    PM5  = 0xFF; PM6  = 0xFF; PM7  = 0xFF; PM8  = 0xFF; PM9  = 0xFF;            \
    PM10 = 0xFF; PM11 = 0xFF; PM12 = 0xFF; PM13 = 0xFF; PM14 = 0xFF;           \
                                                                              \
    /* Configure all Pull-up Option registers to 0x00 (pull-ups disabled) */  \
    PU0  = 0x00; PU1  = 0x00; PU2  = 0x00; PU3  = 0x00; PU4  = 0x00;            \
    PU5  = 0x00; PU6  = 0x00; PU7  = 0x00; PU8  = 0x00; PU9  = 0x00;            \
    PU10 = 0x00; PU11 = 0x00; PU12 = 0x00; PU13 = 0x00; PU14 = 0x00;           \
                                                                              \
    /* Configure all Port Function Control registers to 0x00 (standard I/O) */\
    PFC0  = 0x00; PFC1  = 0x00; PFC2  = 0x00; PFC3  = 0x00; PFC4  = 0x00;       \
    PFC5  = 0x00; PFC6  = 0x00; PFC7  = 0x00; PFC8  = 0x00; PFC9  = 0x00;       \
    PFC10 = 0x00; PFC11 = 0x00; PFC12 = 0x00; PFC13 = 0x00; PFC14 = 0x00;      \
                                                                              \
    /* Configure all Port Mode Control registers to 0x00 (standard I/O) */    \
    PMC0  = 0x00; PMC1  = 0x00; PMC2  = 0x00; PMC3  = 0x00; PMC4  = 0x00;       \
    PMC5  = 0x00; PMC6  = 0x00; PMC7  = 0x00; PMC8  = 0x00; PMC9  = 0x00;       \
    PMC10 = 0x00; PMC11 = 0x00; PMC12 = 0x00; PMC13 = 0x00; PMC14 = 0x00;      \
                                                                              \
    /* Clear Port Input Flag registers (clear any pending wake-up flags) */   \
    PIF0 = 0x00; PIF1 = 0x00; PIF2 = 0x00; PIF3 = 0x00; PIF4 = 0x00;            \
    PIF5 = 0x00; PIF6 = 0x00; PIF7 = 0x00;                                    \
                                                                              \
    /* Individual interrupt enables related to ports (disabling handled below)*/\
                                                                              \
    /* Re-enable global interrupts if desired after safeguard */              \
    /* EI(); */ /* Optional: enable interrupts again after safeguard */        \
} while(0)


/**
 * @brief Safely initialize and disable core peripherals.
 *        Disables global interrupts, timers, PWM, WDT, ICU, ADC, UART, I2C, SPI.
 *        Also ensures GPIOs are configured as standard I/O.
 *        Uses actual registers for RENESAS RL78/G14 (R5F11BBC).
 */
#define Registers_SAFEGUARD_Init() do {                                       \
    /* Disable global interrupts first for a stable state */                  \
    DI();                                                                     \
                                                                              \
    /* Disable clocks/supply to most peripherals using PER0 */                \
    /* This is a strong disable. Specific control registers reset next. */    \
    /* PER0 &= ~(_PER0_TAU0EN | _PER0_TAU1EN | _PER0_SAU0EN | _PER0_SAU1EN | */\
    /*         _PER0_IICA0EN | _PER0_ADCEN   | _PER0_RTCEN   | _PER0_ITEN); */\
    /* Setting PER0 to 0 disables all peripherals controlled by this reg */   \
    PER0 = 0x00U;                                                             \
                                                                              \
    /* Disable/Stop Timers (TAU0, TAU1) more specifically */                  \
    /* Stop all channels */                                                   \
    TT0 = 0xFFFFU;                                                            \
    TT1 = 0xFFFFU;                                                            \
    /* Clear Timer Interrupt Flags (TIFL0, TIFH0, TIFL1, TIFH1) */            \
    TIFL0 = 0x00U; TIFH0 = 0x00U; TIFL1 = 0x00U; TIFH1 = 0x00U;                \
    /* Disable Timer Interrupt Enables (TIEL0, TIEH0, TIEL1, TIEH1) */        \
    TIEL0 = 0x00U; TIEH0 = 0x00U; TIEL1 = 0x00U; TIEH1 = 0x00U;                \
    /* Reset Timer Mode Registers (TMR, TCR, etc.) if needed - PER0 handles clock */\
                                                                              \
    /* PWM is typically part of TAU, disabled by disabling timers */          \
                                                                              \
    /* Disable Watchdog Timer (WDT) - If enabled in option bytes, it may */   \
    /* require specific sequence or cannot be fully disabled in software. */  \
    /* Attempting a software disable if possible (WDTE register) */           \
    /* Note: WDTE address is 0xFF6B for RL78. __at ensures direct access. */  \
    __near volatile uint8_t * const p_WDTE = (__near volatile uint8_t * const)0xFF6BU; \
    *p_WDTE = 0x00U; /* Clear WDTE bit 7 (WDTEN) if available */              \
    /* Also reset WDT Mode register */                                        \
    WDTM = 0x00U;                                                             \
                                                                              \
    /* Disable External/Input Capture Unit Interrupts (ICU) */                \
    /* Clear all External Interrupt Flags (IF0 - IF7) */                      \
    IF0 &= ~(_IF0_INTIF0 | _IF0_INTIF1 | _IF0_INTIF2 | _IF0_INTIF3 |         \
             _IF0_INTIF4 | _IF0_INTIF5 | _IF0_INTIF6 | _IF0_INTIF7);          \
    /* Disable all External Interrupt Enables (IE0 - IE7) */                  \
    IE0 &= ~(_IE0_INTIE0 | _IE0_INTIE1 | _IE0_INTIE2 | _IE0_INTIE3 |         \
             _IE0_INTIE4 | _IE0_INTIE5 | _IE0_INTIE6 | _IE0_INTIE7);          \
    /* Port Input Flag registers (PIF) already cleared in GPIO safeguard */   \
                                                                              \
    /* Disable Analog to Digital Converter (ADC) */                           \
    ADCE = 0U; /* Disable AD conversion enable */                             \
    ADCS = 0U; /* Stop conversion */                                          \
    /* Clear ADC Interrupt Flag (ADIF) */                                     \
    IFC0 &= ~_IFC0_ADIF;                                                      \
    /* Disable ADC Interrupt Enable (ADIEn) */                                \
    IEC0 &= ~_IEC0_ADIEN;                                                     \
    /* Reset ADC mode registers */                                            \
    ADM0 = 0x00U; ADM1 = 0x00U; ADMK = 0x80U; /* ADCE bit is bit 7 of ADMK */  \
                                                                              \
    /* Disable Serial Array Unit (SAU) - used for UART, SPI, I2C */          \
    /* Stop all SAU channels (SAU0 and SAU1) */                               \
    ST0 = 0xFFFFU;                                                            \
    ST1 = 0xFFFFU;                                                            \
    /* Clear SAU Interrupt Flags (STIF, SIF, SRIF, SQSFI) and Enables */      \
    /* (Specific registers depend on channels, general clear if possible) */  \
    IF0 &= ~(_IF0_STIF0 | _IF0_SRIF0 | _IF0_SSIF0 | _IF0_STIF1 |             \
             _IF0_SRIF1 | _IF0_SSIF1 ); /* Clear relevant flags */            \
    IE0 &= ~(_IE0_STIE0 | _IE0_SRIE0 | _IE0_SSIE0 | _IE0_STIE1 |             \
             _IE0_SRIE1 | _IE0_SSIE1 ); /* Disable relevant enables */        \
    /* Reset SAU mode/control registers (SMR, SCR, SDR) if needed */          \
    /* (PER0 handles clock supply, stopping channels is primary disable) */   \
                                                                              \
    /* Disable IICA (I2C) - If used independently of SAU */                   \
    IICEN00 = 0U; /* Disable IICA unit */                                     \
    /* Clear IICA Interrupt Flags and Enables if needed */                    \
    IFC0 &= ~_IFC0_IICIF0; IEC0 &= ~_IEC0_IICIE0; /* Clear relevant flags/enables */\
    IICCTL00 = 0x00U; IICFSA00 = 0x00U; /* Reset control registers */         \
                                                                              \
    /* Ensure GPIOs are configured as standard I/O, not special functions */ \
    /* This is mostly handled by clearing PMC/PFC and setting PM in */        \
    /* GPIO_SAFEGUARD_Init. Ensure PM is set to input (safer default) */     \
    PM0  = 0xFF; PM1  = 0xFF; PM2  = 0xFF; PM3  = 0xFF; PM4  = 0xFF;            \
    PM5  = 0xFF; PM6  = 0xFF; PM7  = 0xFF; PM8  = 0xFF; PM9  = 0xFF;            \
    PM10 = 0xFF; PM11 = 0xFF; PM12 = 0xFF; PM13 = 0xFF; PM14 = 0xFF;           \
                                                                              \
    /* Re-enable global interrupts if desired after safeguard */              \
    /* EI(); */ /* Optional: enable interrupts again after safeguard */        \
                                                                              \
} while(0)

#endif /* MAIN_H */