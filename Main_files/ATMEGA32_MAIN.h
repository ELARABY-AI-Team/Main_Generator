/*
 * File: main.h
 * Description: Minimal header file for ATmega32 embedded projects.
 * Author: Technology Inovation Software Team
 * Device name: ATMEGA32
 * Creation date: 2025-07-07
 * Standard: MISRA C
 * Copyright: ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/* Include Directives */

/* MCU-specific include for ATmega32 registers and bits */
#include <avr/io.h>
/* Include for AVR-GCC intrinsics like cli(), sei(), _delay_nop() and __builtin_avr_wdt_disable() */
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay_basic.h> /* For _delay_nop() */

/* Standard C headers */
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
typedef uint8_t tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/* Core macros */

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register.
 * @param bit The bit position (0-7).
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register.
 * @param bit The bit position (0-7).
 */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register.
 * @param bit The bit position (0-7).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register.
 * @param bit The bit position (0-7).
 */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))

/* Global Interrupts and other common operations (using AVR-GCC intrinsics) */
/**
 * @brief Disable global interrupts.
 */
#define DI() cli()

/**
 * @brief Enable global interrupts.
 */
#define EI() sei()

/**
 * @brief Disable global interrupts. Alias for DI().
 */
#define Global_Int_Disable() cli()

/**
 * @brief Enable global interrupts. Alias for EI().
 */
#define Global_Int_Enable() sei()

/**
 * @brief Execute a No Operation (NOP) instruction.
 */
#define NOP() _delay_nop()

/**
 * @brief Halt execution in an infinite loop.
 */
#define HALT() for(;;)

/* SAFEGUARD MACROS */

/**
 * @brief Safeguard function to initialize all GPIO ports to a safe state.
 * Configures all pins as inputs, sets output latches to 0,
 * disables pull-up resistors and external interrupt wake-up sources.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set Data/Value/Logic to 0 for all ports */ \
        PORTA = 0x00; \
        PORTB = 0x00; \
        PORTC = 0x00; \
        PORTD = 0x00; \
        /* Configure all ports as Input */ \
        DDRA = 0x00; \
        DDRB = 0x00; \
        DDRC = 0x00; \
        DDRD = 0x00; \
        /* Disable pull-up resistors (already done by setting PORTx to 0 when DDRx is input) */ \
        /* Disable external interrupt wake-up triggers (INT0, INT1, INT2) and clear flags */ \
        GICR = 0x00;    /* Disable INT0, INT1, INT2 enables */ \
        MCUCR = 0x00;   /* Disable edge/level triggers for INT0/INT1 (ISC00, ISC01, ISC10, ISC11) */ \
        MCUCSR = 0x00;  /* Disable edge trigger for INT2 (ISC2) */ \
        GIFR = 0xFF;    /* Clear any pending external interrupt flags (INTF0, INTF1, INTF2) */ \
    } while(0)

/**
 * @brief Safeguard function to initialize common microcontroller registers to a safe state.
 * Disables global interrupts, timers, PWM, watchdog timer, ADC, UART, I2C, SPI.
 * Also calls GPIO_SAFEGUARD_Init.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupt */ \
        cli(); \
        \
        /* Disable Watchdog Timer (Using compiler intrinsic for safety) */ \
        __builtin_avr_wdt_disable(); \
        \
        /* Disable Timers/Counters */ \
        TCCR0 = 0x00; /* Timer/Counter 0 Control (Stop clock source) */ \
        TCCR1A = 0x00; /* Timer/Counter 1 Control A */ \
        TCCR1B = 0x00; /* Timer/Counter 1 Control B (Stop clock source) */ \
        TCCR2 = 0x00; /* Timer/Counter 2 Control (Stop clock source) */ \
        \
        /* Disable Timer/Counter Interrupts and clear flags */ \
        TIMSK = 0x00; /* Timer/Counter Interrupt Mask Register */ \
        TIFR = 0xFF;  /* Timer/Counter Interrupt Flag Register (Clear pending) */ \
        \
        /* Disable Analog to Digital Converter (ADC) */ \
        ADCSRA &= ~(1<<ADEN); /* Clear ADC Enable bit */ \
        \
        /* Disable Universal Synchronous/Asynchronous Receiver/Transmitter (USART) */ \
        UCSRB &= ~((1<<RXEN) | (1<<TXEN)); /* Disable Receiver and Transmitter Enables */ \
        UCSRA = 0x00; /* Clear status flags like FE, DOR, UPE, RXC, TXC */ \
        \
        /* Disable Two Wire Interface (TWI / I2C) */ \
        TWCR &= ~(1<<TWEN); /* Clear TWI Enable bit */ \
        \
        /* Disable Serial Peripheral Interface (SPI) Communication */ \
        SPCR &= ~(1<<SPE); /* Clear SPI Enable bit */ \
        SPSR = 0x00; /* Clear status flags like SPIF, WCOL */ \
        \
        /* Note: Disabling peripherals returns control of their pins to standard GPIO mode. */ \
        /* Ensure all GPIOs are in a safe, known input state with pull-ups off. */ \
        GPIO_SAFEGUARD_Init(); \
        \
    } while(0)


#endif /* MAIN_H_ */