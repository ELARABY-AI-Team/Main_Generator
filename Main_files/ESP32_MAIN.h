/*
 * File: main.h
 * Description: Main header file for ATMEGA32 projects. Includes core definitions and safeguards.
 * Author: Technology Inovation Software Team
 * Device name: ATMEGA32
 * Creation date: 2025-07-07
 * Standard: MISRA C
 * Copyright: ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/* Include Directives */

/* MCU-specific include for ATMEGA32 registers */
#include <avr/io.h>

/* Standard C Libraries (as specified) */
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

/* Useful Typedefs */
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/* Core Macros */

/**
 * @brief Set a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7).
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7).
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit number (0-7).
 * @return The value of the bit (0 or non-zero).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7).
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))

/* Global Interrupt Control */
/* Provided by <avr/io.h> via toolchain builtins */
#define DI()    cli() /* Disable Global Interrupts */
#define EI()    sei() /* Enable Global Interrupts */

/* Alternative names for clarity */
#define Global_Int_Disable()    cli() /* Disable Global Interrupts */
#define Global_Int_Enable()     sei() /* Enable Global Interrupts */

/* CPU Control */
/* NOP is a standard AVR instruction */
#define NOP()   _NOP() /* No Operation */

/* HALT - No direct instruction for HALT in ATMEGA32.
 * A software halt is typically an infinite loop.
 * Not providing a generic HALT macro here as it's application specific.
 */

/* SAFEGUARD MACROS */

/**
 * @brief Safely initializes all GPIO ports to a known input state.
 * Configures all ports as input with pull-up resistors disabled.
 * Disables Pin Change Interrupts.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Configure all port pins as inputs */ \
        DDRA = 0x00U; \
        DDRB = 0x00U; \
        DDRC = 0x00U; \
        DDRD = 0x00U; \
        \
        /* Disable pull-up resistors (when configured as input) */ \
        PORTA = 0x00U; \
        PORTB = 0x00U; \
        PORTC = 0x00U; \
        PORTD = 0x00U; \
        \
        /* Disable Pin Change Interrupts (PCINT) */ \
        CLR_BIT(GICR, PCIE0); \
        CLR_BIT(GICR, PCIE1); \
        CLR_BIT(GICR, PCIE2); \
        PCMSK0 = 0x00U; /* Clear PCINT0-7 mask */ \
        PCMSK1 = 0x00U; /* Clear PCINT8-15 mask */ \
        PCMSK2 = 0x00U; /* Clear PCINT16-23 mask */ \
        SET_BIT(GIFR, PCIF0); /* Clear PCINT Flag 0 by writing 1 */ \
        SET_BIT(GIFR, PCIF1); /* Clear PCINT Flag 1 by writing 1 */ \
        SET_BIT(GIFR, PCIF2); /* Clear PCINT Flag 2 by writing 1 */ \
        \
        /* Disable External Interrupts (INT0, INT1, INT2) */ \
        CLR_BIT(GICR, INT0); \
        CLR_BIT(GICR, INT1); \
        CLR_BIT(GICR, INT2); \
        MCUCR = 0x00U;  /* Default INT0/INT1 sensing to low level */ \
        MCUCSR = 0x00U; /* Default INT2 sensing to falling edge */ \
        SET_BIT(GIFR, INTF0); /* Clear INT Flag 0 by writing 1 */ \
        SET_BIT(GIFR, INTF1); /* Clear INT Flag 1 by writing 1 */ \
        SET_BIT(GIFR, INTF2); /* Clear INT Flag 2 by writing 1 */ \
    } while(0)

/**
 * @brief Safely initializes and disables common MCU peripherals.
 * Disables Global Interrupts, Timers, PWM, WDT, ICU, ADC, UART, I2C, SPI.
 * Ensures peripheral pins are released for general I/O use (state set by GPIO_SAFEGUARD_Init).
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        cli(); \
        \
        /* Disable Timers and associated functions (PWM, ICU) */ \
        TCCR0 = 0x00U; /* Stop Timer0 */ \
        TCNT0 = 0x00U; /* Reset Timer0 count */ \
        OCR0 = 0x00U;  /* Clear Timer0 Output Compare */ \
        \
        TCCR1A = 0x00U; /* Stop Timer1 - Control A */ \
        TCCR1B = 0x00U; /* Stop Timer1 - Control B (Includes ICU edge select) */ \
        TCNT1H = 0x00U; /* Reset Timer1 count High */ \
        TCNT1L = 0x00U; /* Reset Timer1 count Low */ \
        OCR1AH = 0x00U; /* Clear Timer1 Output Compare A High */ \
        OCR1AL = 0x00U; /* Clear Timer1 Output Compare A Low */ \
        OCR1BH = 0x00U; /* Clear Timer1 Output Compare B High */ \
        OCR1BL = 0x00U; /* Clear Timer1 Output Compare B Low */ \
        ICR1H = 0x00U;  /* Clear Timer1 Input Capture High */ \
        ICR1L = 0x00U;  /* Clear Timer1 Input Capture Low */ \
        \
        TCCR2 = 0x00U; /* Stop Timer2 */ \
        TCNT2 = 0x00U; /* Reset Timer2 count */ \
        OCR2 = 0x00U;  /* Clear Timer2 Output Compare */ \
        \
        TIMSK = 0x00U; /* Disable Timer/Counter Interrupt Mask */ \
        ETIMSK = 0x00U;/* Disable Extended Timer/Counter Interrupt Mask */ \
        SET_BIT(TIFR, TOV0);  /* Clear Timer0 Overflow Flag */ \
        SET_BIT(TIFR, OCF0);  /* Clear Timer0 Compare Match Flag */ \
        SET_BIT(TIFR, TOV1);  /* Clear Timer1 Overflow Flag */ \
        SET_BIT(TIFR, OCF1B); /* Clear Timer1 Compare Match B Flag */ \
        SET_BIT(TIFR, OCF1A); /* Clear Timer1 Compare Match A Flag */ \
        SET_BIT(TIFR, ICF1);  /* Clear Timer1 Input Capture Flag */ \
        SET_BIT(TIFR, TOV2);  /* Clear Timer2 Overflow Flag */ \
        SET_BIT(TIFR, OCF2);  /* Clear Timer2 Compare Match Flag */ \
        \
        /* Disable Watchdog Timer (Requires specific sequence) */ \
        /* Enable WDT Change Enable and WDT Enable */ \
        /* Write WDE to 0 within 4 clock cycles after setting WDCE */ \
        /* This sequence is defined in ATMEGA32 datasheet for disabling WDT */ \
        WDTCR = (1U << WDCE) | (1U << WDE); \
        WDTCR = 0x00U; \
        \
        /* Disable Analog to Digital Converter (ADC) */ \
        ADCSRA = 0x00U; /* Disable ADC, disable interrupts, stop conversions */ \
        ADMUX = 0x00U;  /* Default MUX and ADLAR settings */ \
        /* Disable ADC Auto Trigger Source in SFIOR */ \
        SFIOR &= ~((1U << ADTS2) | (1U << ADTS1) | (1U << ADTS0)); \
        SET_BIT(ADCSRA, ADIF); /* Clear ADC Interrupt Flag by writing 1 */ \
        \
        /* Disable UART (USART) */ \
        UCSRA = 0x00U; /* Clear Status/Control A */ \
        UCSRB = 0x00U; /* Disable Tx/Rx, disable interrupts - Control B */ \
        UCSRC = 0x00U; /* Clear Control C (Mode, Parity, Stop bits, Data size) */ \
        UBRRH = 0x00U; /* Clear Baud Rate High */ \
        UBRRL = 0x00U; /* Clear Baud Rate Low */ \
        SET_BIT(UCSRA, RXC); /* Clear Receive Complete Flag */ \
        SET_BIT(UCSRA, TXC); /* Clear Transmit Complete Flag */ \
        \
        /* Disable I2C (TWI) */ \
        TWCR = 0x00U; /* Disable TWI, disable interrupts, clear flags */ \
        TWBR = 0x00U; /* Clear Baud Rate Register */ \
        TWAR = 0x00U; /* Clear Slave Address Register */ \
        TWSR = 0x00U; /* Clear Status Register (also clears Prescaler bits) */ \
        \
        /* Disable SPI communication */ \
        SPCR = 0x00U; /* Disable SPI, disable interrupts */ \
        SPSR = 0x00U; /* Clear Status Register (WCOL, SPIF flags) */ \
        \
        /* Note: Disabling peripheral control registers returns pin control
         * to the DDRx/PORTx registers, which should be configured by
         * GPIO_SAFEGUARD_Init or subsequent application code.
         */ \
    } while(0)

#endif /* MAIN_H_ */