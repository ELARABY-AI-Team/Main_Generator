/**
 ******************************************************************************
 * @file    main.h
 * @brief   Main header file for ATmega328P embedded projects.
 *          Provides common includes, typedefs, macros, and safeguard initializations.
 * @author  [Your Name or Organization]
 * @device  ATmega328P
 * @date    2025-06-18
 * @copyright Copyright (c) 2025 [Your Company/Organization Name]
 ******************************************************************************
 */

#ifndef ATMEGA328P_MAIN_H_
#define ATMEGA328P_MAIN_H_

/* Include standard C libraries */
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

/* Include MCU-specific headers */
#include <avr/io.h>         /* AVR specific I/O registers and bits */
#include <avr/interrupt.h>  /* Interrupt handling macros and functions (sei, cli) */

/*----------------------------------------------------------------------------*/
/*                                Typedefs                                    */
/*----------------------------------------------------------------------------*/

/**
 * @brief 8-bit unsigned integer type.
 */
typedef uint8_t  tbyte;

/**
 * @brief 16-bit unsigned integer type.
 */
typedef uint16_t tword;

/**
 * @brief 32-bit unsigned integer type.
 */
typedef uint32_t tlong;

/*----------------------------------------------------------------------------*/
/*                                 Macros                                     */
/*----------------------------------------------------------------------------*/

/**
 * @brief Set a specific bit in a register.
 * @param reg: The register.
 * @param bit: The bit position (0-7).
 */
#define SET_BIT(reg, bit)     ((reg) |= (1 << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg: The register.
 * @param bit: The bit position (0-7).
 */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1 << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param reg: The register.
 * @param bit: The bit position (0-7).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg: The register.
 * @param bit: The bit position (0-7).
 */
#define TOG_BIT(reg, bit)     ((reg) ^= (1 << (bit)))

/**
 * @brief Enable global interrupts.
 */
#define Global_Int_Enable()   sei() /* Set Global Interrupt Flag */

/**
 * @brief Disable global interrupts.
 */
#define Global_Int_Disable()  cli() /* Clear Global Interrupt Flag */

/**
 * @brief Execute a No Operation instruction.
 */
#define NOP()                 _NOP() /* Defined in <avr/io.h> */

/**
 * @brief Halt execution (enter infinite loop).
 *        Used typically for error states or end of program.
 */
#define HALT()                while(1) { /* Infinite loop */ }

/*----------------------------------------------------------------------------*/
/*                             Safeguard Macros                               */
/*----------------------------------------------------------------------------*/

/**
 * @brief Initialize all GPIO pins to a safe default state.
 *        Configures all pins as inputs with pull-ups disabled.
 *        Uses AVR ATmega328P registers DDRx and PORTx.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Configure Port B pins (PB0-PB7) as inputs */ \
        DDRB = 0x00; \
        /* Disable internal pull-ups for Port B pins */ \
        PORTB = 0x00; \
        \
        /* Configure Port C pins (PC0-PC6) as inputs */ \
        DDRC = 0x00; \
        /* Disable internal pull-ups for Port C pins */ \
        PORTC = 0x00; \
        \
        /* Configure Port D pins (PD0-PD7) as inputs */ \
        DDRD = 0x00; \
        /* Disable internal pull-ups for Port D pins */ \
        PORTD = 0x00; \
    } while(0)

/**
 * @brief Disable unused peripherals and configure registers to a safe state.
 *        Disables clocks to most peripherals via PRR, disables ADC, Analog Comparator,
 *        and attempts to disable the Watchdog Timer.
 *        Uses AVR ATmega328P registers PRR, ACSR, ADCSRA, WDTCSR.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Power Reduction Register (PRR) */ \
        /* Setting a bit disables the clock to the corresponding peripheral. */ \
        /* Disable clocks for ADC, USART0, SPI, TWI, Timer2, Timer0, Timer1 */ \
        PRR = (1 << PRADC) | (1 << PRUSART0) | (1 << PRSPI) | (1 << PRTWI) | \
              (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRTIM1); \
        \
        /* Analog Comparator Status Register (ACSR) */ \
        /* Set ACD bit to disable the Analog Comparator */ \
        ACSR |= (1 << ACD); \
        \
        /* ADC Control and Status Register A (ADCSRA) */ \
        /* Clear ADEN bit to disable the ADC */ \
        ADCSRA &= ~(1 << ADEN); \
        \
        /* Watchdog Timer Control Register (WDTCSR) */ \
        /* Perform the sequence to disable the WDT. */ \
        /* Note: This requires the WDT Lock bit not to be set in fuses. */ \
        /* 1. Write WDCE and WDE simultaneously (within 4 clock cycles). */ \
        WDTCSR |= (1 << WDCE) | (1 << WDE); \
        /* 2. Within the next 4 clock cycles, write WDE=0 (and WDCE=0). */ \
        /*    Setting WDTCSR to 0 clears all bits, including WDE. */ \
        WDTCSR = 0x00; \
        \
        /* Note: Brown-out Detection (BOD) is typically controlled by fuses and BORF flag. */ \
        /*       Disabling it during runtime requires careful consideration and is not */ \
        /*       included in this minimal safeguard unless explicitly required. */ \
        /* Note: Clock Prescaler (CLKPR) is often configured early, but isn't strictly */ \
        /*       a 'safeguard' against unexpected peripheral behavior. */ \
        \
    } while(0)


#endif /* _MAIN_H_ */