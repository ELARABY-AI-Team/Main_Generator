/**
 * @file main.h
 * @brief Main header file for ATMEGA32 microcontroller projects.
 *
 * This file provides core definitions, macros, and safeguard functions
 * commonly used in embedded C applications for the ATMEGA32 device,
 * following the MISRA C standard.
 *
 * Author: Technology Inovation Software Team
 * Device name ATMEGA32
 * Creation date: 2025-07-07
 * Standard: MISRA C
 * Copyright: ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/* ----------------------------------------------------------------------------
 * Include Directives
 * --------------------------------------------------------------------------*/

/* MCU-specific include */
#include <avr/io.h>

/* Standard C headers (as specified) */
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


/* ----------------------------------------------------------------------------
 * Useful Typedefs
 * --------------------------------------------------------------------------*/

/** @brief 8-bit unsigned integer type */
typedef uint8_t  tbyte;

/** @brief 16-bit unsigned integer type */
typedef uint16_t tword;

/** @brief 32-bit unsigned integer type */
typedef uint32_t tlong;


/* ----------------------------------------------------------------------------
 * Core Macros
 * --------------------------------------------------------------------------*/

/** @brief Sets a specific bit in a register. */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))

/** @brief Clears a specific bit in a register. */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))

/** @brief Gets the value of a specific bit in a register. */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/** @brief Toggles a specific bit in a register. */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))

/* Interrupt and Control Macros (AVR-specific) */

/** @brief Disables global interrupts. */
#define DI()                cli() // Use AVR-libc built-in function

/** @brief Enables global interrupts. */
#define EI()                sei() // Use AVR-libc built-in function

/** @brief Executes a No Operation instruction. */
#define NOP()               __asm__ __volatile__("nop")

/** @brief Halts program execution in an infinite loop. */
#define HALT()              do { while(1); } while(0)


/* ----------------------------------------------------------------------------
 * Safeguard Functions
 *
 * These functions are implemented using static inline to allow inclusion
 * directly in the header file while avoiding multiple definition errors
 * and enabling potential inlining by the compiler.
 * --------------------------------------------------------------------------*/

/**
 * @brief Initializes all GPIO ports to a safe, known state.
 *
 * Configures all port pins as input with internal pull-ups disabled and
 * sets the output value latch to low. This is a common safeguard during
 * initialization to prevent unintended output states or excessive current
 * draw before proper pin configuration.
 */
static inline void GPIO_SAFEGUARD_Init(void)
{
    /* Set Data Direction Registers to 0: Configure all pins as input */
    DDRA = 0x00U;
    DDRB = 0x00U;
    DDRC = 0x00U;
    DDRD = 0x00U;

    /*
     * Set Port Output Registers to 0:
     * When configured as input (DDRx=0), writing 0 to PORTx disables
     * the internal pull-up resistor.
     * When configured as output (DDRx=1, which is not the case here),
     * writing 0 to PORTx sets the output low.
     * This step ensures pull-ups are disabled and output latches are low.
     */
    PORTA = 0x00U;
    PORTB = 0x00U;
    PORTC = 0x00U;
    PORTD = 0x00U;

    /*
     * ATmega32 does not have dedicated per-pin wake-up registers or
     * a global pull-up disable mechanism separate from the PORTx/DDRx control.
     * Disabling pull-ups via PORTx=0 when DDRx=0 is handled above.
     * Wake-up sources (like external interrupts) are disabled in Registers_SAFEGUARD_Init.
     */
}

/**
 * @brief Disables most peripherals and puts registers into a safe, known state.
 *
 * This function aims to turn off commonly used peripherals (Timers, PWM, WDT,
 * ADC, UART, I2C, SPI), disable global interrupts, clear pending interrupt
 * flags, and configure relevant control registers to default or disabled states.
 * It also ensures GPIO pins are configured for general I/O use by disabling
 * peripheral control over pins.
 */
static inline void Registers_SAFEGUARD_Init(void)
{
    /* Disable global interrupts */
    cli(); /* Clear Global Interrupt Enable bit in SREG */

    /* Disable Watchdog Timer (requires a specific timed sequence if enabled) */
    /* This sequence ensures disabling even if the WDT was active. */
    SET_BIT(WDTCR, WDCE); /* Set WDCE (Watchdog Change Enable) */
    SET_BIT(WDTCR, WDE);  /* Set WDE (Watchdog Enable) - WDCE must be set first */
    /* Within 4 cycles, write the desired WDTCR value with WDE=0 */
    WDTCR = 0x00U; /* Clear WDT settings and disable WDE */

    /* Disable Timers/Counters and clear their control and value registers */
    TCCR0 = 0x00U; TCNT0 = 0x00U; OCR0 = 0x00U;
    TCCR1A = 0x00U; TCCR1B = 0x00U; TCNT1H = 0x00U; TCNT1L = 0x00U;
    OCR1AH = 0x00U; OCR1AL = 0x00U; OCR1BH = 0x00U; OCR1BL = 0x00U;
    ICR1H = 0x00U; ICR1L = 0x00U;
    TCCR2 = 0x00U; ASSR = 0x00U; TCNT2 = 0x00U; OCR2 = 0x00U;

    /* Disable Analog-to-Digital Converter (ADC) */
    ADCSRA = 0x00U; /* Disable ADC (ADEN=0) and clear status/control bits */
    ADMUX = 0x00U;  /* Set ADC input to ADC0 and clear other selection bits */
    /* Clear ADC Auto Trigger Source selection in SFIOR */
    SFIOR &= ~((1U<<ADTS2)|(1U<<ADTS1)|(1U<<ADTS0));

    /* Disable Universal Synchronous/Asynchronous Receiver/Transmitter (USART) */
    UCSRB = 0x00U; /* Disable Transmitter (TXEN=0) and Receiver (RXEN=0) */
    UCSRA = 0x00U; /* Clear flags (RXC, TXC, UDRE, FE, DOR, PE, U2X, MPCM) */
    /* UCSRC mode bits are not cleared here; disabling Tx/Rx is sufficient to stop operation. */
    UBRRH = 0x00U; /* Reset Baud Rate High byte */
    UBRRL = 0x00U; /* Reset Baud Rate Low byte */

    /* Disable Serial Peripheral Interface (SPI) */
    SPCR = 0x00U; /* Disable SPI (SPE=0) and clear settings */
    SPSR = 0x00U; /* Clear SPI Status Register (WCOL, SPIF, SPI2X) */

    /* Disable Two Wire Interface (TWI / I2C) */
    TWCR = 0x00U; /* Disable TWI (TWEN=0) and clear control bits */

    /*
     * Configure GPIOs as general I/O:
     * This is achieved implicitly by disabling the peripheral control over the pins
     * (e.g., disabling ADC means the PA pins function as normal GPIO).
     * The GPIO_SAFEGUARD_Init function should be called separately to set the
     * initial direction (input) and state (low/pull-up disabled) of the pins.
     */

    /* Clear Pending Interrupt Flags (writing 1 to an interrupt flag bit clears it) */
    TIFR = 0xFFU; /* Clear Timer/Counter Interrupt Flags (TOV0, OCF0, TOV1, OCF1A, OCF1B, ICF1, TOV2, OCF2) */
    EIFR = (1U<<INTF1) | (1U<<INTF0); /* Clear External Interrupt Flags INTF1, INTF0 */
    GIFR = (1U<<INTF2); /* Clear External Interrupt Flag INTF2 */

    /* Disable External Interrupts and Pin Change Interrupts */
    GICR = 0x00U; /* Disable External Interrupt Request 1 (INT1) and INT0 */
    /* MCUCSR controls ISC2 for INT2 and SFIOR has PUD bit (Pull-up Disable) */
    MCUCSR &= ~((1U<<ISC2)); /* Clear ISC2 bit (default level triggered for INT2) */
                             /* SFIOR PUD bit is global pull-up disable, handled by PORTx settings */
    MCUCR = 0x00U; /* Disable edge/level triggering for INT0 and INT1 */

    /* Note: Other peripherals like Analog Comparator, EEPROM, etc.,
     * could also be explicitly reset if needed, but the list above covers
     * the most common ones requested.
     */
}


#endif /* MAIN_H_ */