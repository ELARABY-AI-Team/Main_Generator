/**
 * @file main.h
 * @brief Main header file for microcontroller configuration and common utilities.
 * @author AI
 * @device RENESAS R5F11BBC
 * @creation date 2025-06-19
 * @standared MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef RENESAS R5F11BBC_MAIN_H_
#define RENESAS R5F11BBC_MAIN_H_

/* *******************************************************************************************************************
 * Includes
 ******************************************************************************************************************* */

/* Standard C Libraries */
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
/* This header provides access to the device-specific registers and definitions */
#include <ior5f11bbc.h>
#include <intrinsics.h> /* Provides access to intrinsic functions like DI, EI, HALT, NOP */

/* *******************************************************************************************************************
 * Type Definitions
 ******************************************************************************************************************* */

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

/* *******************************************************************************************************************
 * Core Macros
 ******************************************************************************************************************* */

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number to set (0-indexed).
 */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number to clear (0-indexed).
 */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit number to get (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number to toggle (0-indexed).
 */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/**
 * @brief Disable Global Interrupts.
 * Uses the intrinsic DI() function.
 */
#define Global_Int_Disable()    DI()

/**
 * @brief Enable Global Interrupts.
 * Uses the intrinsic EI() function.
 */
#define Global_Int_Enable()     EI()

/**
 * @brief Put the CPU into HALT mode (Low-power mode).
 * Uses the intrinsic HALT() function.
 */
#define HALT()                  HALT()

/**
 * @brief Execute a No Operation instruction.
 * Uses the intrinsic NOP() function.
 */
#define NOP()                   NOP()

/* *******************************************************************************************************************
 * Safeguard Macros
 ******************************************************************************************************************* */

/**
 * @brief Configures all GPIO pins to a safe, known state (digital input, pull-up disabled, output low).
 * This function iterates through all possible port registers and sets them to safe default values
 * for the RENESAS R5F11BBC (RL78/G1B) microcontroller.
 * The number of available pins per port varies, but writing to the registers for non-existent bits
 * is typically ignored or read back as 0, which is safe for this purpose.
 */
#define GPIO_SAFEGUARD_Init()   \
    do                          \
    {                           \
        /* Configure all Port Data Registers (Px) to low */ \
        P0 = 0x00; P1 = 0x00; P2 = 0x00; P3 = 0x00; \
        P4 = 0x00; P5 = 0x00; P6 = 0x00; P7 = 0x00; \
        P8 = 0x00; P9 = 0x00; P10 = 0x00; P11 = 0x00; \
        P12 = 0x00; P13 = 0x00; P14 = 0x00; P15 = 0x00; \
                            \
        /* Configure all Port Mode Registers (PMx) to input (1) */ \
        PM0 = 0xFF; PM1 = 0xFF; PM2 = 0xFF; PM3 = 0xFF; \
        PM4 = 0xFF; PM5 = 0xFF; PM6 = 0xFF; PM7 = 0xFF; \
        PM8 = 0xFF; PM9 = 0xFF; PM10 = 0xFF; PM11 = 0xFF; \
        PM12 = 0xFF; PM13 = 0xFF; PM14 = 0xFF; PM15 = 0xFF; \
                            \
        /* Configure all Pull-Up Option Registers (PUx) to disabled (0) */ \
        PU0 = 0x00; PU1 = 0x00; PU2 = 0x00; PU3 = 0x00; \
        PU4 = 0x00; PU5 = 0x00; PU6 = 0x00; PU7 = 0x00; \
        PU8 = 0x00; PU9 = 0x00; PU10 = 0x00; PU11 = 0x00; \
        PU12 = 0x00; PU13 = 0x00; PU14 = 0x00; PU15 = 0x00; \
                            \
        /* Configure all Port Mode Control Registers (PMCTx) to digital I/O (0) */ \
        /* This ensures pins are not configured for analog or special functions initially */ \
        PMCT0 = 0x00; PMCT1 = 0x00; PMCT2 = 0x00; PMCT3 = 0x00; \
        PMCT4 = 0x00; PMCT5 = 0x00; PMCT6 = 0x00; PMCT7 = 0x00; \
        PMCT8 = 0x00; PMCT9 = 0x00; PMCT10 = 0x00; PMCT11 = 0x00; \
        PMCT12 = 0x00; PMCT13 = 0x00; PMCT14 = 0x00; PMCT15 = 0x00; \
                            \
        /* Add configuration for Port Input Mode Register (PIM) if needed for open-drain inputs */ \
        /* Setting to 0 ensures standard CMOS inputs. */ \
        PIM0 = 0x00; PIM1 = 0x00; PIM2 = 0x00; PIM3 = 0x00; \
        PIM4 = 0x00; PIM5 = 0x00; PIM6 = 0x00; PIM7 = 0x00; \
        PIM8 = 0x00; PIM9 = 0x00; PIM10 = 0x00; PIM11 = 0x00; \
        PIM12 = 0x00; PIM13 = 0x00; PIM14 = 0x00; PIM15 = 0x00; \
                            \
        /* Add configuration for Port Output Mode Register (POM) if needed for open-drain outputs */ \
        /* Setting to 0 ensures standard CMOS outputs (relevant if any are later configured as output) */ \
        POM0 = 0x00; POM1 = 0x00; POM2 = 0x00; POM3 = 0x00; \
        POM4 = 0x00; POM5 = 0x00; POM6 = 0x00; POM7 = 0x00; \
        POM8 = 0x00; POM9 = 0x00; POM10 = 0x00; POM11 = 0x00; \
        POM12 = 0x00; POM13 = 0x00; POM14 = 0x00; POM15 = 0x00; \
                            \
        /* No dedicated "wake up" registers per port to disable directly in this manner, */ \
        /* wake-up is typically configured via interrupt/event settings. */ \
        /* Input mode (PMx = 1) naturally puts pins in a safe state regarding accidental wake-up. */ \
                            \
    } while(0)

/**
 * @brief Disables or resets common peripheral registers to a safe, known state.
 * This macro targets key peripherals of the RENESAS R5F11BBC (RL78/G1B)
 * to ensure they are not active upon initial boot or before explicit configuration.
 */
#define Registers_SAFEGUARD_Init()  \
    do                              \
    {                               \
        /* Disable Global Interrupts */ \
        Global_Int_Disable();       \
                                    \
        /* Disable Timers (TAU0, TAU1) */ \
        /* Stop all channels in TAU0 and TAU1 */ \
        TT0 = 0xFFFFU;              \
        TT1 = 0xFFFFU;              \
        /* Disable TAU0 and TAU1 clock supply */ \
        CLR_BIT(PER0, 0); /* TAU0EN = 0 */ \
        CLR_BIT(PER0, 1); /* TAU1EN = 0 */ \
                                    \
        /* Disable Interval Timer (IT) */ \
        ITMC = 0x0000U;             \
        CLR_BIT(PER1, 4); /* ITEN = 0 */ \
                                    \
        /* Disable Real-time Timer (RT) */ \
        RTCC0 = 0x00U;              \
        RTCC1 = 0x00U;              \
        CLR_BIT(PER1, 5); /* RTEN = 0 */ \
                                    \
        /* Disable Watchdog Timer (WDT) */ \
        /* Note: WDT disable via WDTE=0 might be prevented by option bytes. */ \
        /* If option bytes force WDT active, it must be serviced instead. */ \
        /* This line assumes it is permissible to disable it. */ \
        WDTE = 0x00U;               \
        /* Clear WDT reset flag if set */ \
        RWDTFLG = 0U;               \
                                    \
        /* Disable Analog-to-Digital Converter (ADC) */ \
        ADCE = 0U;                  \
        CLR_BIT(PER0, 5); /* ADCEN = 0 */ \
                                    \
        /* Disable Serial Communication Interfaces (CSI/UART/SPI) - SAU0, SAU1 */ \
        /* Stop all channels in SAU0 and SAU1 */ \
        ST0 = 0xFFFFU;              \
        ST1 = 0xFFFFU;              \
        /* Disable SAU0 and SAU1 clock supply */ \
        CLR_BIT(PER0, 2); /* SAU0EN = 0 */ \
        CLR_BIT(PER0, 3); /* SAU1EN = 0 */ \
                                    \
        /* Disable I2C Interface (IIC0) */ \
        IICE0 = 0U;                 \
        CLR_BIT(PER0, 4); /* IIC0EN = 0 */ \
                                    \
        /* Configure all GPIOs as digital I/O, overriding special functions */ \
        /* This is already covered in GPIO_SAFEGUARD_Init by setting PMCTx to 0. */ \
        /* Ensure PMCTx are 0 for all ports */ \
        PMCT0 = 0x00; PMCT1 = 0x00; PMCT2 = 0x00; PMCT3 = 0x00; \
        PMCT4 = 0x00; PMCT5 = 0x00; PMCT6 = 0x00; PMCT7 = 0x00; \
        PMCT8 = 0x00; PMCT9 = 0x00; PMCT10 = 0x00; PMCT11 = 0x00; \
        PMCT12 = 0x00; PMCT13 = 0x00; PMCT14 = 0x00; PMCT15 = 0x00; \
                                    \
        /* Add other peripherals specific to R5F11BBC if necessary */ \
        /* For example, LCD controller, Data Transfer Controller (DTC), etc. */ \
        /* Disable DTC */ \
        DTCEN = 0U; /* Disable DTC activation */ \
        CLR_BIT(PER0, 7); /* DTCEN (in PER0) = 0 */ \
                                    \
        /* Disable CRC calculator */ \
        CRC0EN = 0U; \
        CLR_BIT(PER0, 6); /* CRC0EN (in PER0) = 0 */ \
                                    \
        /* Disable LCD Controller if present and enabled by option bytes */ \
        /* LCKCR = 0x00U; */ /* Stop clock supply - Check manual if applicable and safest state */ \
                                    \
    } while(0)

/* *******************************************************************************************************************
 * Function Prototypes (Optional - Add as needed for global functions)
 ******************************************************************************************************************* */

/* Example: */
/* void System_Init(void); */

#endif /* MAIN_H */