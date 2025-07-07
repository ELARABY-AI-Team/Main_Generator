/**
 * @file main.h
 * @brief Main system header file for ATMEGA32 project.
 * @author Technology Inovation Software Team
 * @device ATMEGA32
 * @creation_date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/* Include Device Specific Header File */
#include <avr/io.h>

/* Include Standard C Libraries */
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <iso646.h> /* Provides alternative spellings for operators */
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

/* Include AVR-Libc Specific Libraries */
#include <avr/interrupt.h> /* For sei(), cli() */
#include <avr/wdt.h>       /* For wdt_disable() */
#include <util/delay.h>    /* Potentially useful, though not strictly required by prompt */
#include <avr/sleep.h>     /* Potentially useful for HALT, but HALT is implemented as a spin */

/* Useful Type Definitions */
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/* Core Macros */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit))) /**< Set a specific bit in a register */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit))) /**< Clear a specific bit in a register */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)  /**< Get the value of a specific bit in a register */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit))) /**< Toggle a specific bit in a register */

/* Global Interrupt Control Macros */
#define EI()                sei() /**< Enable Global Interrupts (uses AVR-Libc intrinsic) */
#define DI()                cli() /**< Disable Global Interrupts (uses AVR-Libc intrinsic) */
#define Global_Int_Enable() sei() /**< Enable Global Interrupts (uses AVR-Libc intrinsic) */
#define Global_Int_Disable() cli()/**< Disable Global Interrupts (uses AVR-Libc intrinsic) */

/* System Control Macros */
#define NOP()               __asm__ __volatile__ ("nop") /**< No Operation (uses inline assembly) */
#define HALT()              do {} while(1)              /**< Software Halt (infinite loop) */

/* SAFEGUARD MACROS - Initialize system to a known, safe state */

/**
 * @brief Initializes all General Purpose Input/Output (GPIO) pins to a safe state.
 *
 * Configures all data registers (PORTx) to 0 (low output or pull-up disabled)
 * and all data direction registers (DDRx) to 0 (input). This disables pull-ups
 * and configures all pins as inputs, preventing accidental outputs or unintended
 * consumption/driving of external circuitry upon startup or reset.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set initial output/pull-up state low for all ports */ \
        PORTA = 0x00; \
        PORTB = 0x00; \
        PORTC = 0x00; \
        PORTD = 0x00; \
        \
        /* Configure all pins as input for all ports */ \
        DDRA = 0x00; \
        DDRB = 0x00; \
        DDRC = 0x00; \
        DDRD = 0x00; \
         \
        /* Pull-up resistors are disabled by setting PORTx=0 when DDRx=0 */ \
        /* Wake-up sources tied to GPIO (like external/pin change interrupts) */ \
        /* are typically disabled by disabling global interrupts and specific */ \
        /* interrupt enables, which is handled in Registers_SAFEGUARD_Init(). */ \
    } while(0)

/**
 * @brief Initializes crucial microcontroller registers to a known, safe state.
 *
 * Disables global interrupts, watchdog timer, all timers, PWM, ADC, UART, SPI,
 * and TWI modules. This macro aims to put the core peripherals into a default,
 * inactive state before specific initialization routines are executed.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* 1. Disable Global Interrupts */ \
        cli(); /* Uses AVR-Libc intrinsic */ \
        \
        /* 2. Disable Watchdog Timer */ \
        wdt_disable(); /* Uses AVR-Libc function from <avr/wdt.h> */ \
        \
        /* 3. Disable Timers/Counters and PWM */ \
        TCCR0 = 0x00;   /* Timer/Counter 0 Control Register */ \
        TCCR1A = 0x00;  /* Timer/Counter 1 Control Register A */ \
        TCCR1B = 0x00;  /* Timer/Counter 1 Control Register B */ \
        TCCR2 = 0x00;   /* Timer/Counter 2 Control Register */ \
        TCNT0 = 0x00;   /* Timer/Counter 0 Value */ \
        TCNT1H = 0x00;  /* Timer/Counter 1 Value High */ \
        TCNT1L = 0x00;  /* Timer/Counter 1 Value Low */ \
        TCNT2 = 0x00;   /* Timer/Counter 2 Value */ \
        OCR0 = 0x00;    /* Output Compare Register 0 */ \
        OCR1A = 0x0000; /* Output Compare Register 1 A */ \
        OCR1B = 0x0000; /* Output Compare Register 1 B */ \
        ICR1 = 0x0000;  /* Input Capture Register 1 */ \
        OCR2 = 0x00;    /* Output Compare Register 2 */ \
        \
        /* Clear Timer Interrupt Flags (write 1 to clear) */ \
        TIFR = (1<<OCF2) | (1<<TOV2) | (1<<ICF1) | (1<<OCF1A) | (1<<OCF1B) | (1<<TOV1) | (1<<OCF0) | (1<<TOV0); \
        /* Disable Timer Interrupt Enables */ \
        TIMSK = 0x00; \
        \
        /* 4. Disable ADC (Analog to Digital Converter) */ \
        ADCSRA = 0x00; /* Disables ADC, clears flags, disables interrupts */ \
        ADMUX = 0x00;  /* Selects ADC channel 0, AREF reference */ \
        SFIOR = 0x00;  /* Clear bits like PSR10, PSR2 (Prescaler Reset) */ \
        \
        /* 5. Disable UART (USART) Communication */ \
        UCSRB = 0x00; /* Disable Transmitter, Receiver, and associated Interrupts */ \
        UCSRA = 0x00; /* Clear flags like RXC, TXC, FE, DOR, PE */ \
        /* Set UCSRC - must set URSEL bit (bit 7) to write this register */ \
        /* Setting to (1<<URSEL) puts it in a basic Async, 8N1 config, but disabled via UCSRB */ \
        UCSRC = (1<<URSEL); \
        UBRRH = 0x00; /* Clear Baud Rate High */ \
        UBRRL = 0x00; /* Clear Baud Rate Low */ \
        \
        /* 6. Disable SPI Communication */ \
        SPCR = 0x00; /* Disable SPI, interrupts, clear mode/clock settings */ \
        SPSR = (1<<SPIF) | (1<<WCOL); /* Clear flags */ \
        /* SPDR data register is not typically explicitly cleared on disable */ \
        \
        /* 7. Disable I2C (TWI) Communication */ \
        TWCR = 0x00; /* Disable TWI, interrupts, clear enable bit and other control bits */ \
        TWSR &= ~((1<<TWPS1) | (1<<TWPS0)); /* Clear Prescaler bits (TWPS) to 0 */ \
        TWDR = 0xFF; /* Data Register - dummy write or known state */ \
        TWAR = 0x00; /* Slave Address Register */ \
        TWAMR = 0x00; /* Slave Address Mask Register */ \
        /* Clear TWI Interrupt Flag (write 1 to clear) */ \
        TWCR = (1<<TWINT); \
         /* Disable TWI Interrupt Enable - redundant as TWCR=0x00 was done, but explicit */ \
        TWCR &= ~(1<<TWIE); \
        \
        /* 8. Ensure GPIO pins are generic I/O */ \
        /* This is primarily achieved by disabling the peripherals above, */ \
        /* which releases the pins for general I/O usage according to DDR/PORT. */ \
        /* The GPIO_SAFEGUARD_Init() macro sets DDR/PORT to a safe input/low state. */ \
         \
    } while(0)

#endif /* MAIN_H_ */