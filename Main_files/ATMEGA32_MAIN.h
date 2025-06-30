/**
 * @file main.h
 * @brief Core header file for ATMEGA32 embedded projects.
 *        Includes essential standard libraries, typedefs, core macros,
 *        and system safeguard initialization routines for ATMEGA32.
 * @author AI
 * @device ATMEGA32
 * @creation date 2025-06-30
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef ATMEGA32_MAIN_H_
#define ATMEGA32_MAIN_H_

/*
 *******************************************************************************
 *                              MCU SPECIFIC INCLUDES                          *
 *******************************************************************************
 */

/**
 * @brief Include the ATMEGA32 device header file.
 *        This file typically contains register definitions and bit names.
 */
#include <avr/io.h> // Includes definitions for ATMEGA32 registers (PORTx, DDRx, TCCRx, etc.)

/*
 *******************************************************************************
 *                               STANDARD C INCLUDES                           *
 *******************************************************************************
 */

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
 *******************************************************************************
 *                                USEFUL TYPEDEFS                              *
 *******************************************************************************
 */

/**
 * @brief Typedef for an 8-bit unsigned integer.
 */
typedef uint8_t tbyte;

/**
 * @brief Typedef for a 16-bit unsigned integer.
 */
typedef uint16_t tword;

/**
 * @brief Typedef for a 32-bit unsigned integer.
 */
typedef uint32_t tlong;

/*
 *******************************************************************************
 *                                  CORE MACROS                                *
 *******************************************************************************
 */

/**
 * @brief Macro to set a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7) to set.
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))

/**
 * @brief Macro to clear a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7) to clear.
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))

/**
 * @brief Macro to get the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit number (0-7) to get.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/**
 * @brief Macro to toggle a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7) to toggle.
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))

/**
 * @brief Macro to disable global interrupts.
 *        Manually clears the Global Interrupt Enable (I) bit in SREG.
 */
#define Global_Int_Disable() \
    do {                     \
        /* The I bit is bit 7 of the SREG register */ \
        SREG &= ~(1U << 7);  \
    } while(0)

/**
 * @brief Macro to enable global interrupts.
 *        Manually sets the Global Interrupt Enable (I) bit in SREG.
 */
#define Global_Int_Enable() \
    do {                    \
        /* The I bit is bit 7 of the SREG register */ \
        SREG |= (1U << 7);  \
    } while(0)

/**
 * @brief Executes a No Operation (NOP) instruction.
 */
#define NOP() \
    do {      \
        /* Use inline assembly for a direct NOP instruction */ \
        asm volatile("nop"); \
    } while(0)

/**
 * @brief Puts the microcontroller into a low-power sleep mode (Power-Down).
 *        Requires sleep mode to be configured and sleep enable bit to be set.
 *        This macro configures Power-Down mode and executes the sleep instruction.
 *        Awakens on external interrupt, TWI address match, or Watchdog Timer interrupt.
 */
#define HALT() \
    do {       \
        /* Configure Power-Down sleep mode (SM1=1, SM0=0) */ \
        /* Note: SM0 and SM1 bits are in MCUCR */ \
        MCUCR = (MCUCR & ~((1U << SM0) | (1U << SM1))) | (1U << SM1); \
        /* Enable sleep mode (SE bit in MCUCR) */ \
        MCUCR |= (1U << SE); \
        /* Execute sleep instruction */ \
        /* Note: Interrupts should generally be enabled BEFORE calling HALT() */ \
        asm volatile("sleep"); \
        /* Sleep Enable is automatically cleared upon waking up */ \
    } while(0)


/*
 *******************************************************************************
 *                               SAFEGUARD MACROS                              *
 *******************************************************************************
 */

/**
 * @brief Initializes all GPIO ports to a safe state.
 *        Sets all pins to output low (0), then configures them as inputs,
 *        disables internal pull-up resistors, and disables associated wake-up sources
 *        (like external interrupts).
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set initial output value to 0 for all ports (before setting direction) */ \
        /* This also ensures pull-ups are disabled when pins are configured as inputs */ \
        PORTA = 0x00; \
        PORTB = 0x00; \
        PORTC = 0x00; \
        PORTD = 0x00; \
        \
        /* Configure all pins as inputs */ \
        DDRA = 0x00; \
        DDRB = 0x00; \
        DDRC = 0x00; \
        DDRD = 0x00; \
        \
        /* Disable external pin change interrupts (INT0, INT1, INT2) */ \
        /* GICR: General Interrupt Control Register */ \
        /* INT0 is bit 6, INT1 is bit 7, INT2 is bit 5 in GICR */ \
        GICR &= ~((1U << INT1) | (1U << INT0) | (1U << INT2)); \
        \
        /* Set external interrupt sense control to a safe, non-triggering state (e.g., low level for INT0/INT1) */ \
        /* MCUCR: Controls ISC00/ISC01 for INT0 and ISC10/ISC11 for INT1 */ \
        /* ISCxx = 00 sets low level trigger */ \
        MCUCR &= ~((1U << ISC11) | (1U << ISC10) | (1U << ISC01) | (1U << ISC00)); \
        /* MCUCSR: Controls ISC2 for INT2 */ \
        /* ISC2 = 0 sets falling edge trigger (default after reset, but ensures state) */ \
        MCUCSR &= ~(1U << ISC2); \
    } while(0)

/**
 * @brief Initializes essential microcontroller registers to a safe state.
 *        Disables global interrupts, timers, PWM, watchdog timer, input capture,
 *        ADC, UART, I2C, and SPI peripherals. Ensures peripherals are not active.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        /* The 'I' bit is bit 7 of SREG. Manually clear it as <avr/interrupt.h> is excluded. */ \
        SREG &= ~(1U << 7); \
        \
        /* Disable Timers */ \
        /* Timer/Counter 0 */ \
        TCCR0 = 0x00; /* Stop clock source (CS0x=000) and set waveform mode (WGM0x=00) */ \
        TCNT0 = 0x00; /* Reset counter value */ \
        OCR0  = 0x00; /* Reset output compare register */ \
        /* Disable Timer0 interrupts (Overflow, Output Compare Match) */ \
        TIMSK &= ~((1U << TOIE0) | (1U << OCIE0)); \
        \
        /* Timer/Counter 1 (16-bit) */ \
        TCCR1A = 0x00; /* Stop clock source (WGM1x=00, COM1xx=00 for output compare) */ \
        TCCR1B = 0x00; /* Stop clock source (WGM1x=00, CS1x=000) */ \
        TCNT1H = 0x00; /* Reset counter value (High Byte) */ \
        TCNT1L = 0x00; /* Reset counter value (Low Byte) */ \
        OCR1AH = 0x00; /* Reset Output Compare A (High Byte) */ \
        OCR1AL = 0x00; /* Reset Output Compare A (Low Byte) */ \
        OCR1BH = 0x00; /* Reset Output Compare B (High Byte) */ \
        OCR1BL = 0x00; /* Reset Output Compare B (Low Byte) */ \
        ICR1H  = 0x00; /* Reset Input Capture Register (High Byte) */ \
        ICR1L  = 0x00; /* Reset Input Capture Register (Low Byte) */ \
        /* Disable Timer1 interrupts (Overflow, OCMA, OCMB, Input Capture) */ \
        TIMSK &= ~((1U << TOIE1) | (1U << OCIE1A) | (1U << OCIE1B) | (1U << TICIE1)); \
        \
        /* Timer/Counter 2 (8-bit Async) */ \
        TCCR2 = 0x00; /* Stop clock source (CS2x=000) and set waveform mode (WGM2x=00) */ \
        TCNT2 = 0x00; /* Reset counter value */ \
        OCR2  = 0x00; /* Reset output compare register */ \
        ASSR  = 0x00; /* Ensure async mode is off (default is synchronous) and stop clock source */ \
        /* Disable Timer2 interrupts (Overflow, Output Compare Match) */ \
        TIMSK &= ~((1U << TOIE2) | (1U << OCIE2)); \
        \
        /* Disable PWM (handled by disabling Timers and setting COM bits to 0) */ \
        \
        /* Disable Watchdog Timer */ \
        /* This requires a specific timed sequence. The sequence must complete within 4 cycles. */ \
        /* Set WDCE (WDT Change Enable) and WDE (WDT Enable) */ \
        WDTCR |= (1U << WDCE) | (1U << WDE); \
        /* Within 4 cycles, clear WDE and WDCE and the prescaler bits (WDPx) */ \
        WDTCR = 0x00; \
        \
        /* Disable Input Capture Unit (Handled by disabling Timer1) */ \
        \
        /* Disable Analog to Digital Converter (ADC) */ \
        ADCSRA &= ~(1U << ADEN); /* Disable ADC Enable */ \
        ADCSRA &= ~(1U << ADIE); /* Disable ADC Interrupt Enable */ \
        ADMUX = 0x00; /* Set to default (ADC0, AREF reference) */ \
        /* SFIOR: Special Function IO Register, ADTS bits control Auto Trigger Source */ \
        SFIOR &= ~((1U << ADTS2) | (1U << ADTS1) | (1U << ADTS0)); /* Disable ADC Auto Trigger */ \
        \
        /* Disable UART (USART) */ \
        UCSRB &= ~((1U << RXEN) | (1U << TXEN)); /* Disable Receiver and Transmitter */ \
        UCSRB &= ~((1U << RXCIE) | (1U << TXCIE) | (1U << UDRIE)); /* Disable interrupts */ \
        /* UCSRC: UCSR Control Register C. Set to default Async, 8-bit, 1 stop, No parity. Requires URSEL bit set for access on ATMEGA32. */ \
        UCSRC = (1U << URSEL) | 0x06; /* URSEL=1, UCSZ1=1, UCSZ0=1 (8-bit data) */ \
        UBRRH = 0x00; /* Reset Baud Rate Register High */ \
        UBRRL = 0x00; /* Reset Baud Rate Register Low */ \
        UCSRA = 0x00; /* Reset status register (clear flags like RXC, TXC, etc.) */ \
        \
        /* Disable I2C (TWI) */ \
        TWCR &= ~(1U << TWEN); /* Disable TWI Enable */ \
        TWCR &= ~(1U << TWIE); /* Disable TWI Interrupt Enable */ \
        TWSR = 0x00; /* Reset TWI Status Register */ \
        TWAR = 0x00; /* Reset TWI (Slave) Address Register */ \
        TWBR = 0x00; /* Reset TWI Bit Rate Register */ \
        \
        /* Disable SPI Communication */ \
        SPCR &= ~(1U << SPE); /* Disable SPI Enable */ \
        SPCR &= ~(1U << SPIE); /* Disable SPI Interrupt Enable */ \
        SPCR = 0x00; /* Reset control register (MSTR=0, CPOL=0, CPHA=0, SPRx=0) */ \
        SPSR = 0x00; /* Reset status register (clear flags like SPIF, WCOL) */ \
        \
        /* Configure all GPIOS as generic I/O (not special function registers) */ \
        /* This is achieved by disabling the peripherals above. */ \
        /* GPIO_SAFEGUARD_Init() already configured them as inputs. */ \
        /* This step here implicitly reinforces that peripheral functions on pins are off. */ \
        /* No additional register writes needed here as GPIO_SAFEGUARD_Init covers DDR/PORT. */ \
    } while(0)

#endif /* MAIN_H */