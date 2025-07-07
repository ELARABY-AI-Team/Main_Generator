/**
 ******************************************************************************
 * @file    main.h
 * @brief   A minimal header file for ATMEGA32 projects.
 * @author  Technology Inovation Software Team
 * @device  ATMEGA32
 * @creation date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 ******************************************************************************
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 ******************************************************************************
 * Includes
 ******************************************************************************
 */

/* MCU Specific Include */
#include <avr/io.h>         /* ATmega32 specific registers and bits */
#include <avr/interrupt.h>  /* For cli() and sei() */
#include <avr/wdt.h>        /* For wdt_disable() - standard library approach */


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

/*
 ******************************************************************************
 * Useful Typedefs
 ******************************************************************************
 */

/** @typedef tbyte
 *  @brief 8-bit unsigned integer type */
typedef uint8_t tbyte;

/** @typedef tword
 *  @brief 16-bit unsigned integer type */
typedef uint16_t tword;

/** @typedef tlong
 *  @brief 32-bit unsigned integer type */
typedef uint32_t tlong;

/*
 ******************************************************************************
 * Core Macros
 ******************************************************************************
 */

/** @def SET_BIT
 *  @brief Sets a specific bit in a register.
 *  @param reg The register to modify.
 *  @param bit The bit number to set (0-7). */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/** @def CLR_BIT
 *  @brief Clears a specific bit in a register.
 *  @param reg The register to modify.
 *  @param bit The bit number to clear (0-7). */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/** @def GET_BIT
 *  @brief Gets the value of a specific bit in a register.
 *  @param reg The register to read from.
 *  @param bit The bit number to get (0-7).
 *  @return The value of the bit (0 or 1). */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/** @def TOG_BIT
 *  @brief Toggles a specific bit in a register.
 *  @param reg The register to modify.
 *  @param bit The bit number to toggle (0-7). */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/** @def DI
 *  @brief Disables global interrupts. (Alias for Global_Int_Disable) */
#define DI()                    cli()

/** @def EI
 *  @brief Enables global interrupts. (Alias for Global_Int_Enable) */
#define EI()                    sei()

/** @def Global_Int_Disable
 *  @brief Disables global interrupts by clearing the I-bit in SREG. */
#define Global_Int_Disable()    cli()

/** @def Global_Int_Enable
 *  @brief Enables global interrupts by setting the I-bit in SREG. */
#define Global_Int_Enable()     sei()

/** @def HALT
 *  @brief Halts program execution in an infinite loop. */
#define HALT()                  for(;;);

/** @def NOP
 *  @brief Executes a No Operation instruction. */
#define NOP()                   __builtin_avr_nop()


/*
 ******************************************************************************
 * SAFEGUARD MACROS - Fully Implemented
 ******************************************************************************
 */

/**
 * @def GPIO_SAFEGUARD_Init
 * @brief Configures all GPIO ports to a safe, known state (inputs, no pull-ups, value 0).
 *        Disables external/pin change interrupts.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set all PORT pins to 0 (no output high, no pull-up on input) */ \
        PORTA = 0x00; \
        PORTB = 0x00; \
        PORTC = 0x00; \
        PORTD = 0x00; \
        \
        /* Configure all DDR pins as input (0) */ \
        DDRA = 0x00; \
        DDRB = 0x00; \
        DDRC = 0x00; \
        DDRD = 0x00; \
        \
        /* Pull-up resistors are disabled by writing 0 to PORTx when DDRx is 0 (input) */ \
        \
        /* Disable External Interrupts (INT0, INT1) */ \
        GICR &= ~((1U << INT1) | (1U << INT0)); \
        /* Disable External Interrupt 2 */ \
        MCUCSR &= ~(1U << ISC2); \
        /* Clear external interrupt trigger modes for INT0, INT1 */ \
        MCUCR &= ~((1U << ISC11) | (1U << ISC10) | (1U << ISC01) | (1U << ISC00)); \
        \
        /* Clear External Interrupt Flags (INTF0, INTF1, INTF2) by writing a logical one */ \
        GIFR |= ((1U << INTF1) | (1U << INTF0) | (1U << INTF2)); \
        \
    } while(0)

/**
 * @def Registers_SAFEGUARD_Init
 * @brief Disables core peripherals and sets them to a default/off state.
 *        Assumes GPIOs were configured by GPIO_SAFEGUARD_Init().
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable global interrupts */ \
        cli(); \
        \
        /* Disable Watchdog Timer (WDT) */ \
        /* This sequence requires precise timing. Using wdt_disable() from avr/wdt.h is safer */ \
        /* Alternative direct register method (timing critical): */ \
        /* WDTCR |= (1U << WDTOE) | (1U << WDE); */ \
        /* WDTCR = 0x00; */ \
        wdt_disable(); /* Standard and safe way */ \
        \
        /* Disable all Timers (Timer0, Timer1, Timer2) */ \
        /* Stop clocks (CS bits = 0), clear modes, clear COM bits (disables PWM) */ \
        TCCR0 = 0x00; \
        TCCR1A = 0x00; \
        TCCR1B = 0x00; \
        TCCR2 = 0x00; \
        /* Disable all Timer Interrupts */ \
        TIMSK = 0x00; \
        /* Clear all Timer Flags (by writing a logical one) */ \
        TIFR = 0xFF; \
        \
        /* Disable Input Capture Unit (ICU) - part of Timer1, disabled by TCCR1B=0 and TIMSK=0 */ \
        \
        /* Disable Analog to Digital Converter (ADC) */ \
        ADCSRA &= ~(1U << ADEN); /* Disable ADC Enable */ \
        ADCSRA = 0x00;           /* Clear all bits (ADEN, ADIE, ADIF, prescaler) */ \
        ADMUX = 0x00;            /* Clear reference, channel, and ADLAR */ \
        \
        /* Disable Universal Synchronous/Asynchronous Receiver/Transmitter (USART) */ \
        UCSRB &= ~((1U << RXEN) | (1U << TXEN)); /* Disable Receiver and Transmitter */ \
        UCSRB = 0x00; /* Clear remaining bits (Interrupts, UCSZn2, RXB8, TXB8) */ \
        UCSRA = 0x00; /* Clear all flags and control bits */ \
        /* Clear UCSRC - requires setting URSEL bit (MSB) first on ATmega32 */ \
        UCSRC = (1U << URSEL); /* Set URSEL, clear others (UMSEL, UPM, USBS, UCSZ, UCPOL) */ \
        /* Set Baud Rate to 0 (or default power-on reset value) */ \
        UBRRL = 0x00; \
        UBRRH = 0x00; \
        \
        /* Disable Two Wire Interface (TWI/I2C) */ \
        TWCR &= ~(1U << TWEN); /* Disable TWI Enable */ \
        TWCR = 0x00;           /* Clear all bits (Interrupts, Enable, Flags) */ \
        TWSR = 0x00;           /* Clear Status and Prescaler */ \
        TWBR = 0x00;           /* Clear Baud Rate Register */ \
        \
        /* Disable Serial Peripheral Interface (SPI) */ \
        SPCR &= ~(1U << SPE); /* Disable SPI Enable */ \
        SPCR = 0x00;          /* Clear all bits (SPE, SPIE, DORD, MSTR, CPOL, CPHA, SPR) */ \
        /* Clear SPI Flags (by writing a logical one if interrupt enabled, otherwise just read/write SPDR) */ \
        /* SPSR &= ~(1U << SPIF); No, write 1 to clear */ \
        /* Reading SPDR is part of flag clear sequence, but not needed if SPI is off */ \
        /* Let's ensure no lingering flags. SPSR bit SPIF clears on interrupt service or SW sequence */ \
        /* With SPIE=0, writing 1 to SPIF bit in SPSR has no effect. The flag would be cleared by reading SPDR */ \
        /* Let's just ensure the control register is reset */ \
        SPSR = 0x00;          /* Clear all bits (SPIF, WCOL, SPI2X) - writing 1 to SPIF should clear it if SPIE is enabled, but SPIE is now 0 */ \
        \
        /*
         * Configuring all GPIOs as I/O (not special function registers) is
         * implicitly handled by disabling the peripherals that use those pins
         * as special functions (ADC, UART, TWI, SPI) and setting DDRx/PORTx
         * to their default (input/no pull-up) state in GPIO_SAFEGUARD_Init.
         */ \
    } while(0)

/*
 ******************************************************************************
 * End of file
 ******************************************************************************
 */

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */