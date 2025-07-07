/**
 * @file main.h
 * @brief Minimal and commonly used definitions for ATMEGA32 embedded projects.
 * @author Technology Inovation Software Team
 * @device ATMEGA32
 * @creation date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef _MAIN_H_
#define _MAIN_H_

/*============================================================================*/
/*                             Include Directives                             */
/*============================================================================*/

/* MCU-specific include */
#include <avr/io.h>         /* Provides ATMEGA32 register definitions */
#include <avr/interrupt.h>  /* Provides cli() and sei() intrinsics */
#include <avr/builtins.h>   /* Provides _NOP() intrinsic */

/* Standard C includes (as specified) */
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
/*                                Useful Typedefs                             */
/*============================================================================*/

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

/*============================================================================*/
/*                                 Core Macros                                */
/*============================================================================*/

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-indexed).
 */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-indexed).
 */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit number (0-indexed).
 */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/**
 * @brief Disable global interrupts.
 */
#define DI()                    cli()

/**
 * @brief Enable global interrupts.
 */
#define EI()                    sei()

/**
 * @brief Disable global interrupts (alias for DI).
 */
#define Global_Int_Disable()    DI()

/**
 * @brief Enable global interrupts (alias for EI).
 */
#define Global_Int_Enable()     EI()

/**
 * @brief Execute a No Operation instruction.
 */
#define NOP()                   _NOP()

/**
 * @brief Halt the processor (enter an infinite loop).
 * Note: For entering sleep modes, refer to avr/sleep.h.
 * This macro implements a simple halt by looping indefinitely.
 */
#define HALT()                  while(1)

/*============================================================================*/
/*                              Safeguard Macros                              */
/*============================================================================*/

/**
 * @brief Safeguard initialization for all GPIO ports.
 * Sets all pins to output low (0) then configures them as inputs
 * and disables internal pull-up resistors.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set all PORT registers to 0x00 (Output low or disable pull-ups) */ \
        PORTA = 0x00U; \
        PORTB = 0x00U; \
        PORTC = 0x00U; \
        PORTD = 0x00U; \
        \
        /* Set all DDR registers to 0x00 (Configure as inputs) */ \
        DDRA = 0x00U; \
        DDRB = 0x00U; \
        DDRC = 0x00U; \
        DDRD = 0x00U; \
    } while(0)

/**
 * @brief Safeguard initialization for various peripheral registers.
 * Disables key peripherals and global interrupts.
 * Note: Configuring pins back to pure I/O after peripheral disable
 * happens inherently when peripheral enable bits are cleared.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        DI(); \
        \
        /* Disable Timers/Counters (T/C0, T/C1, T/C2) and their related PWM */ \
        TCCR0 = 0x00U; /* T/C0 Control Register */ \
        TCCR1A = 0x00U; /* T/C1 Control Register A */ \
        TCCR1B = 0x00U; /* T/C1 Control Register B (stops clock, disables Input Capture) */ \
        TCCR2 = 0x00U; /* T/C2 Control Register */ \
        \
        /* Disable Watchdog Timer (WDT) */ \
        /* Note: WDT disable requires a specific timed sequence (WDTOE and WDE bits) */ \
        WDTCR |= (1U << WDTOE) | (1U << WDE); \
        WDTCR = 0x00U; \
        \
        /* Disable Analog to Digital Converter (ADC) */ \
        ADCSRA = 0x00U; /* Disable ADC Enable bit (ADEN) and others */ \
        \
        /* Disable USART (UART) */ \
        UCSRA = 0x00U; \
        UCSRB = 0x00U; /* Disable Receiver (RXEN) and Transmitter (TXEN) */ \
        UCSRC = 0x00U; /* Reset frame format, mode, etc. */ \
        \
        /* Disable TWI (Two Wire Interface - I2C) */ \
        TWCR = 0x00U; /* Disable TWI Enable bit (TWEN) and others */ \
        \
        /* Disable SPI (Serial Peripheral Interface) */ \
        SPCR = 0x00U; /* Disable SPI Enable bit (SPE) and others */ \
        \
        /* Input Capture Unit (ICU) is part of Timer1, disabled by TCCR1B = 0x00U. */ \
        /* GPIOs revert to being controlled by DDR/PORT registers when peripherals are disabled. */ \
        /* Specific GPIO state (e.g., all inputs) is handled by GPIO_SAFEGUARD_Init. */ \
        \
    } while(0)

/*============================================================================*/
/*                           End of Safeguard Macros                          */
/*============================================================================*/

#endif /* _MAIN_H_ */