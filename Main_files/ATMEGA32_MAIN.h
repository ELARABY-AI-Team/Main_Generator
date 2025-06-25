/**
 ******************************************************************************
 * @file    main.h
 * @brief   Minimal and common header file for ATmega32 embedded projects.
 * @author  AI
 * @device  ATmega32
 * @date    2025-06-25
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 ******************************************************************************
 */

#ifndef ATMEGA32_MAIN_H_
#define ATMEGA32_MAIN_H_

/* Standard Library Includes */
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
#include <stdbool.h> // For boolean types
#include <stddef.h>
#include <stdint.h>  // For fixed-width integer types
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* MCU Specific Includes */
#include <avr/io.h>         // Standard AVR I/O definitions for registers and bits
#include <avr/interrupt.h>  // For sei() and cli() - standard atomic interrupt control

/* Useful Typedefs */
typedef uint8_t tbyte;  ///< 8-bit unsigned integer
typedef uint16_t tword; ///< 16-bit unsigned integer
typedef uint32_t tlong; ///< 32-bit unsigned integer

/* Core Macros for Bit Manipulation */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))     ///< Set a specific bit in a register
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))    ///< Clear a specific bit in a register
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)      ///< Get the value of a specific bit in a register
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))     ///< Toggle a specific bit in a register

/* Microcontroller Specific Core Macros */
// Using sei() and cli() from <avr/interrupt.h> for atomic global interrupt control
// as direct SREG register manipulation is not atomic for disabling interrupts.
#define Global_Int_Enable() sei()   ///< Enable global interrupts atomically
#define Global_Int_Disable() cli()  ///< Disable global interrupts atomically

#define HALT() while(1)             ///< Halt execution (infinite loop)
#define NOP() asm volatile("nop")   ///< No operation instruction

/* SAFEGUARD MACROS */

/**
 * @brief Configures all GPIO ports to a safe default state (output low, then input, pull-ups disabled).
 *
 * This macro iterates through all ports (PORTA, PORTB, PORTC, PORTD)
 * and configures them as follows:
 * 1. Set data register to 0x00 (output low state if configured as output).
 * 2. Set data direction register to 0x00 (configure as inputs).
 *    Setting PORTx to 0x00 before setting DDRx to 0x00 ensures that when the pin
 *    becomes an input, the internal pull-up resistor is disabled.
 * 3. Globally disable all internal pull-up resistors using the PUD bit in SFIOR.
 * 4. Disable external interrupt enable bits (INT0, INT1, INT2) to prevent wake-up from these sources.
 *    Assumes no other GPIO-specific wake-up registers exist beyond interrupt enables.
 */
#define GPIO_SAFEGUARD_Init() do { \
    /* Configure all ports as output low */ \
    PORTA = 0x00; \
    PORTB = 0x00; \
    PORTC = 0x00; \
    PORTD = 0x00; \
    \
    /* Configure all ports as input */ \
    DDRA = 0x00; \
    DDRB = 0x00; \
    DDRC = 0x00; \
    DDRD = 0x00; \
    \
    /* Disable all internal pull-ups globally */ \
    SET_BIT(SFIOR, PUD); \
    \
    /* Disable external interrupt wake-up sources */ \
    /* INT0, INT1, INT2 enable bits are in GICR (page 65) */ \
    GICR &= ~((1 << INT0) | (1 << INT1) | (1 << INT2)); \
    \
    /* Assume no other GPIO-specific wake-up registers exist beyond interrupt enables */ \
} while(0)

/**
 * @brief Disables and resets key microcontroller peripherals to a safe state.
 *
 * This macro disables the following peripherals:
 * - Global interrupts
 * - Timer/Counter0, Timer/Counter1, Timer/Counter2 (stops clocks, disables interrupts, clears flags)
 * - Watchdog Timer (using required timed sequence)
 * - Input Capture Unit (by stopping Timer1 clock and disconnecting Analog Comparator)
 * - Analog to Digital Converter (ADC) and Analog Comparator (AC)
 * - USART (UART)
 * - TWI (I2C)
 * - SPI
 *
 * Peripheral pins are effectively returned to standard I/O function (as inputs per GPIO_SAFEGUARD_Init).
 */
#define Registers_SAFEGUARD_Init() do { \
    /* Disable global interrupts atomically */ \
    Global_Int_Disable(); /* Defined as cli() */ \
    \
    /* Disable Timer/Counter0 (8-bit) */ \
    TCCR0 = 0x00; /* Stop clock (CS0x=000), disconnect OC0 pin (COM0x=00), Normal mode (WGM0x=00) */ \
    TIMSK &= ~((1 << OCIE0) | (1 << TOIE0)); /* Disable Compare Match and Overflow interrupts */ \
    TIFR = (1 << OCF0) | (1 << TOV0); /* Clear pending Compare Match and Overflow flags by writing 1 (page 81) */ \
    \
    /* Disable Timer/Counter1 (16-bit) */ \
    TCCR1B = 0x00; /* Stop clock (CS1x=000), disable Input Capture Noise Canceler (ICNC1=0), falling edge trigger (ICES1=0) */ \
    TCCR1A = 0x00; /* Disconnect OC1A/OC1B pins (COM1Ax=00, COM1Bx=00), Normal mode (WGM1x=00) */ \
    TIMSK &= ~((1 << TICIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1)); /* Disable Input Capture, Compare Match A/B, Overflow interrupts */ \
    TIFR = (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B) | (1 << TOV1); /* Clear pending flags by writing 1 (page 111) */ \
    \
    /* Disable Timer/Counter2 (8-bit Asynchronous) */ \
    TCCR2 = 0x00; /* Stop clock (CS2x=000), disconnect OC2 pin (COM2x=00), Normal mode (WGM2x=00) */ \
    TIMSK &= ~((1 << OCIE2) | (1 << TOIE2)); /* Disable Compare Match and Overflow interrupts */ \
    TIFR = (1 << OCF2) | (1 << TOV2); /* Clear pending flags by writing 1 (page 128) */ \
    ASSR = 0x00; /* Ensure Timer2 is synchronous (AS2=0), clear update busy flags (page 126) */ \
    \
    /* Disable Watchdog Timer */ \
    /* This requires a specific timed sequence as described in the manual (page 40) */ \
    /* The sequence is safe because interrupts are already disabled */ \
    WDTCR = (1 << WDTOE) | (1 << WDE); /* Write WDTOE and WDE to 1 to enable timed sequence */ \
    WDTCR = (0 << WDE);                /* Write WDE to 0 within the next four clock cycles to disable WDT */ \
    \
    /* Disable Input Capture Unit (part of Timer1) */ \
    /* Functionality is primarily disabled by stopping Timer1 clock (TCCR1B=0) and disabling its interrupt (TIMSK &= ~TICIE1) */ \
    ACSR &= ~(1 << ACIC); /* Disconnect Analog Comparator from Timer1 Input Capture (page 197) */ \
    /* Note: No explicit register bit found in the manual to fully disconnect the ICP1 pin itself from the capture logic without using a specific WGM mode. */ \
    \
    /* Disable ADC and Analog Comparator */ \
    ADCSRA = 0x00; /* Disable ADC (ADEN=0), disable interrupts (ADIE=0), disable auto trigger (ADATE=0), stop conversion (ADSC=0) (page 214) */ \
    ADCSRA = (1 << ADIF); /* Clear pending ADC flag by writing 1 (page 214) */ \
    ADMUX = 0x00; /* Set reference to AREF, Internal Vref off (REFS1:0=00), select ADC0 (MUX4:0=00000), right adjust result (ADLAR=0) (page 212) */ \
    SFIOR &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0)); /* Disable ADC auto trigger source selection (page 216) */ \
    ACSR |= (1 << ACD); /* Disable Analog Comparator (page 197) */ \
    ACSR &= ~(1 << ACBG); /* Disable Analog Comparator Bandgap Select (page 197) - Note: BOD fuse can still keep bandgap on */ \
    SFIOR &= ~(1 << ACME); /* Ensure ADC multiplexer isn't used for comparator (page 196) */ \
    \
    /* Disable USART (UART) */ \
    UCSRB = 0x00; /* Disable RX (RXEN=0), TX (TXEN=0), and all USART interrupts (RXCIE, TXCIE, UDRIE, UCSZ2) (page 159) */ \
    /* UCSRC shares address with UBRRH, must write URSEL (bit 7) to access UCSRC (page 160) */ \
    UCSRC = (1 << URSEL); /* Select UCSRC, set defaults (Async, No parity, 1 stop bit, 5 data bits - UCSZ1:0=00 when URSEL=1) (page 160-161) */ \
    UBRRH = 0x00; /* Set baud rate register to lowest possible (0) - safest default (page 162) */ \
    UBRRL = 0x00; /* Set baud rate register to lowest possible (0) (page 162) */ \
    UCSRA = (1 << RXC) | (1 << TXC) | (1 << UDRE); /* Clear pending USART flags by writing 1 (page 158) */ \
    UCSRA &= ~((1 << U2X) | (1 << MPCM)); /* Disable Double Speed (U2X=0), disable Multi-processor mode (MPCM=0) (page 159) */ \
    \
    /* Disable TWI (I2C) */ \
    /* Clear TWINT flag by writing 1 is needed to enable writes to other bits */ \
    TWCR = (1 << TWINT); /* Clear TWINT first (page 175) */ \
    TWCR = (0 << TWEN) | (0 << TWIE) | (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO); /* Disable TWI (TWEN=0), interrupts (TWIE=0), ACK (TWEA=0), START (TWSTA=0), STOP (TWSTO=0) (page 175-176) */ \
    TWAR = 0x00; /* Clear slave address (TWAx=0), disable General Call recognition (TWGCE=0) (page 177) */ \
    TWBR = 0x00; /* Set baud rate register to 0 (page 175) */ \
    TWSR = 0x00; /* Set prescaler to 0 (TWPSx=00) (page 176) */ \
    \
    /* Disable SPI */ \
    SPCR = 0x00; /* Disable SPI (SPE=0), disable interrupts (SPIE=0), set defaults (MSB first, SCK low idle, sample leading edge, lowest speed div) (page 134) */ \
    SPSR = (1 << SPIF) | (1 << WCOL); /* Clear SPIF and WCOL flags by writing 1 (page 136) */ \
    /* Pin direction and pull-ups should be handled by GPIO_SAFEGUARD_Init() */ \
    \
    /* GPIOs are configured as inputs (by GPIO_SAFEGUARD_Init) and peripherals are disabled, */ \
    /* so pins default to standard input function where special function is disabled. */ \
    \
} while(0)


#endif /* MAIN_H_ */