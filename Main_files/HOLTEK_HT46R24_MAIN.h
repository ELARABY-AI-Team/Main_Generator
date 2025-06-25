/**
 * @file main.h
 * @brief Main project header file for HOLTEK HT46R24 microcontroller.
 * @author AI
 * @device HOLTEK_HT46R24
 * @creation date 2025-06-25
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef HOLTEK_HT46R24_MAIN_H_
#define HOLTEK_HT46R24_MAIN_H_

/* Include MCU-specific header */
/* Note: The exact name and path of the MCU header may vary depending on the Holtek toolchain (HT-IDE3000). */
/* This assumes a standard header file provided by the toolchain defines registers and peripherals. */
#include <ht46r24.h> /* Example header file name for HT46R24 */

/* Include Standard C Headers */
/* Note: Availability and full functionality of all standard headers may be limited on a constrained 8-bit MCU environment. */
/* Including as requested, but use judgement in actual application code. */
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

/* Useful typedefs */
typedef uint8_t  tbyte;  /**< Typedef for 8-bit unsigned integer */
typedef uint16_t tword;  /**< Typedef for 16-bit unsigned integer */
typedef uint32_t tlong;  /**< Typedef for 32-bit unsigned integer */

/* Core Macros */

/**
 * @brief Sets a specific bit in a register.
 * @param[in,out] reg The register to modify.
 * @param[in] bit The bit position (0-based) to set.
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param[in,out] reg The register to modify.
 * @param[in] bit The bit position (0-based) to clear.
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param[in] reg The register to read.
 * @param[in] bit The bit position (0-based) to read.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param[in,out] reg The register to modify.
 * @param[in] bit The bit position (0-based) to toggle.
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))

/* Global Interrupt Control Macros */
/* Assumes INTC register and _gie bit are defined in ht46r24.h */
/**
 * @brief Enables global interrupts.
 */
#define Global_Int_Enable()     SET_BIT(_intc, _gie)

/**
 * @brief Disables global interrupts.
 */
#define Global_Int_Disable()    CLR_BIT(_intc, _gie)

/* Intrinsic/Assembly Macros */
/* Note: These may vary based on the compiler/toolchain. _nop() and _halt() are common intrinsics. */
/**
 * @brief Executes a No Operation instruction.
 */
#define NOP()                   _nop()

/**
 * @brief Halts the microcontroller execution.
 */
#define HALT()                  _halt()


/* SAFEGUARD MACROS */

/**
 * @brief Initializes all GPIO ports to a safe default state (output low, input direction, pull-ups/wake-ups disabled).
 * @note Assumes existence of registers _pa, _paca, _pua, _wka for Port A etc., as per HT46R24 architecture.
 *       Adjust port registers based on the specific chip variation and package if needed.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Configure Data/Output: Set all output pins to Low */ \
        _pa = 0x00; \
        _pb = 0x00; \
        _pc = 0x00; \
        /* Add other ports if they exist: _pd = 0x00; _pe = 0x00; ... */ \
 \
        /* Configure Direction/Control: Set all pins to Input */ \
        /* 1 in PACx register bit means input, 0 means output */ \
        _paca = 0xFF; \
        _pacb = 0xFF; \
        _pacc = 0xFF; \
        /* Add other ports if they exist: _pacd = 0xFF; _pace = 0xFF; ... */ \
 \
        /* Disable Pull-Up Resistors */ \
        /* 1 in PUx register bit means pull-up enabled, 0 means disabled */ \
        _pua = 0x00; \
        _pub = 0x00; \
        _puc = 0x00; \
        /* Add other ports if they exist: _pud = 0x00; _pue = 0x00; ... */ \
 \
        /* Disable Wake-Up Functions */ \
        /* 1 in WKx register bit means wake-up enabled, 0 means disabled */ \
        _wka = 0x00; \
        _wkb = 0x00; \
        _wkc = 0x00; \
        /* Add other ports if they exist: _wkd = 0x00; _wke = 0x00; ... */ \
    } while(0)

/**
 * @brief Disables various microcontroller peripherals and configures pins for digital I/O.
 * @note This macro attempts to disable common peripherals by clearing their control registers.
 *       Specific register names are based on common HT46R24 naming conventions (_intc, _ptm0c, _tm0c, etc.).
 *       The Watchdog Timer (WDT) on HT46R24 is typically configured and enabled by Option Bytes (fuses) and cannot be disabled in software if fuse-enabled.
 *       This macro does NOT disable the WDT if fuse-enabled; it only attempts to reset interrupt flags if applicable.
 *       A comprehensive safeguard might require disabling ALL special function enables per pin, which is extensive.
 *       This macro focuses on main peripheral module disablement.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        CLR_BIT(_intc, _gie); \
 \
        /* Disable Timers (PTM, TM) */ \
        /* Clear Timer Control Registers */ \
        _ptm0c = 0x00; /* Programmable Timer Module 0 Control */ \
        _tm0c = 0x00;  /* Timer Module 0 Control (if exists) */ \
        _tm1c = 0x00;  /* Timer Module 1 Control (if exists) */ \
        /* Add other Timer Modules if they exist */ \
 \
        /* Disable PWM (often controlled by Timer settings) */ \
        /* Disabling PTM/TM usually disables associated PWM outputs */ \
 \
        /* Disable Watchdog Timer (Note: WDT is fuse-controlled. Cannot disable in software if fuse-enabled) */ \
        /* Clear any potential WDT interrupt flags or related software controls if they existed, */ \
        /* but direct disabling is typically not possible via registers if fuse-enabled. */ \
        /* If WDT is enabled by fuse, the application must periodically service it. */ \
 \
        /* Disable Input Capture Unit (ICU - often part of Timers) */ \
        /* Disabling Timers generally covers this. Clear INTEN for external pin interrupts. */ \
        _inten = 0x00; /* External Interrupt Enable Register */ \
 \
        /* Disable Analog to Digital Converter (ADC) */ \
        _ader = 0x00;  /* ADC Enable Register */ \
        _adcr = 0x00;  /* ADC Control Register */ \
        _adsr = 0x00;  /* ADC Status Register (clear flags) */ \
        /* Disable Analog function on pins */ \
        _aerc = 0x00; /* Analog Function Enable Register for Port A */ \
        _berc = 0x00; /* Analog Function Enable Register for Port B */ \
        _cerc = 0x00; /* Analog Function Enable Register for Port C */ \
        /* Add other ports if they exist: _derc = 0x00; _eerc = 0x00; ... */ \
 \
        /* Disable UART */ \
        _usbcr = 0x00; /* USART Control Register */ \
 \
        /* Disable I2C */ \
        _i2cc1 = 0x00; /* I2C Control Register 1 */ \
 \
        /* Disable SPI communication */ \
        _spicr = 0x00; /* SPI Control Register */ \
 \
        /* Configure all GPIOS as Input (safer default) - Redundant if GPIO_SAFEGUARD_Init is called first, */ \
        /* but included here for robustness of this macro. */ \
        _paca = 0xFF; \
        _pacb = 0xFF; \
        _pacc = 0xFF; \
        /* Add other ports if they exist: _pacd = 0xFF; _pace = 0xFF; ... */ \
 \
    } while(0)


#endif /* MAIN_H */