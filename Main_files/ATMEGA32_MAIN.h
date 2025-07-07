/**
 * @file main.h
 * @brief Main project header file with common includes, typedefs, and macros.
 * @author Technology Inovation Software Team
 * @date 2025-07-07
 * @device ATMEGA32
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H
#define MAIN_H

/*******************************************************************************
 *                              Include Directives                             *
 *******************************************************************************/

/* MCU-specific include (for ATmega32) */
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

/*******************************************************************************
 *                                Useful Typedefs                              *
 *******************************************************************************/

/** @typedef tbyte
 * @brief Alias for uint8_t, representing an 8-bit unsigned integer (byte).
 */
typedef uint8_t tbyte;

/** @typedef tword
 * @brief Alias for uint16_t, representing a 16-bit unsigned integer (word).
 */
typedef uint16_t tword;

/** @typedef tlong
 * @brief Alias for uint32_t, representing a 32-bit unsigned integer (long word).
 */
typedef uint32_t tlong;

/*******************************************************************************
 *                                Core Macros                                  *
 *******************************************************************************/

/**
 * @brief Set a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-based) to set.
 */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-based) to clear.
 */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit position (0-based) to get.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit position (0-based) to toggle.
 */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/**
 * @brief Disable global interrupts.
 *        Wrapper for the AVR-GCC specific cli() function.
 */
#define DI()                    cli()

/**
 * @brief Enable global interrupts.
 *        Wrapper for the AVR-GCC specific sei() function.
 */
#define EI()                    sei()

/**
 * @brief Disable global interrupts (alternative name).
 */
#define Global_Int_Disable()    cli()

/**
 * @brief Enable global interrupts (alternative name).
 */
#define Global_Int_Enable()     sei()

/**
 * @brief Halt the program execution in an infinite loop.
 */
#define HALT()                  do { while(1); } while(0)

/**
 * @brief Execute a No Operation instruction.
 *        Wrapper for the AVR-GCC specific _NOP() intrinsic.
 */
#define NOP()                   _NOP()


/*******************************************************************************
 *                              Safeguard Macros                               *
 *******************************************************************************/

/**
 * @brief Safeguard initialization for GPIOs.
 *        Configures all ports to a known safe state:
 *        - Output value 0 for all pins.
 *        - Configured as inputs.
 *        - Pull-up resistors disabled.
 *        - No special function override from GPIO registers (handled by disabling peripherals).
 *        Note: Wake-up source configuration is often peripheral-specific (e.g., external interrupts, PCINTs)
 *        and is typically handled in the Registers_SAFEGUARD_Init or specific peripheral setup.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set output value to 0 for all ports (also disables pull-ups when configured as input) */ \
        PORTA = 0U; \
        PORTB = 0U; \
        PORTC = 0U; \
        PORTD = 0U; \
        \
        /* Configure all ports as inputs */ \
        DDRA = 0U; \
        DDRB = 0U; \
        DDRC = 0U; \
        DDRD = 0U; \
    } while(0)

/**
 * @brief Safeguard initialization for core peripheral registers.
 *        Disables commonly used peripherals and ensures control registers are in a default state.
 *        - Disables global interrupts.
 *        - Disables Timers (Timer0, Timer1, Timer2).
 *        - Disables PWM functionality (part of Timer configuration).
 *        - Disables Watchdog Timer (requires a specific sequence).
 *        - Disables Input Capture Unit (part of Timer1).
 *        - Disables Analog-to-Digital Converter (ADC).
 *        - Disables UART (USART).
 *        - Disables I2C (TWI).
 *        - Disables SPI communication.
 *        - Ensures GPIO pins are configured for basic I/O (done by disabling peripherals and GPIO_SAFEGUARD_Init).
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable global interrupts */ \
        cli(); \
        \
        /* Disable Timers (by stopping the clock source) and PWM */ \
        TCCR0 = 0U;     /* Timer/Counter 0 Control Register */ \
        TCCR1A = 0U;    /* Timer/Counter 1 Control Register A */ \
        TCCR1B = 0U;    /* Timer/Counter 1 Control Register B (contains clock select) */ \
        TCCR2 = 0U;     /* Timer/Counter 2 Control Register */ \
        \
        /* Disable Watchdog Timer (requires specific sequence) */ \
        /* Note: The ATmega32 datasheet specifies a sequence involving WDCE and WDE. */ \
        MCUCSR |= (1U << JTD); /* Disable JTAG for alternative pin functions if needed (optional but common in init) */ \
        MCUCSR |= (1U << JTD); /* Repeat to ensure lock */ \
        \
        WDTCR |= (1U << WDCE) | (1U << WDE); /* Set WDCE (Watchdog Change Enable) and WDE (Watchdog Enable) */ \
        /* Within 4 cycles, write the new value (0) to WDTCR to disable */ \
        WDTCR = 0U; \
        \
        /* Disable Input Capture Unit (part of Timer1 - already handled by TCCR1B=0) */ \
        \
        /* Disable Analog-to-Digital Converter (ADC) */ \
        ADCSRA = 0U;    /* ADC Control and Status Register A */ \
        ADMUX = 0U;     /* ADC Multiplexer Selection Register */ \
        \
        /* Disable UART (USART) */ \
        UCSR0B = 0U;    /* USART Control and Status Register B (contains RXEN, TXEN) */ \
        UCSR0C = 0U;    /* USART Control and Status Register C */ \
        UBRR0H = 0U;    /* USART Baud Rate Register High */ \
        UBRR0L = 0U;    /* USART Baud Rate Register Low */ \
        \
        /* Disable I2C (TWI) */ \
        TWCR = 0U;      /* TWI Control Register (contains TWEN) */ \
        TWDR = 0U;      /* TWI Data Register */ \
        TWAR = 0U;      /* TWI (Slave) Address Register */ \
        TWSR = 0U;      /* TWI Status Register */ \
        TWBR = 0U;      /* TWI Bit Rate Register */ \
        \
        /* Disable SPI communication */ \
        SPCR = 0U;      /* SPI Control Register (contains SPE) */ \
        SPSR = 0U;      /* SPI Status Register */ \
        SPDR = 0U;      /* SPI Data Register */ \
        \
        /* Additional safeguards for other peripherals could be added here if needed */ \
        ACSR = 0U;      /* Analog Comparator Control and Status Register */ \
        SFIOR = 0U;     /* Special Function IO Register (contains PUD, ACME, etc.) */ \
        ASSR = 0U;      /* Asynchronous Status Register (for Timer2 async operation) */ \
        EECR = 0U;      /* EEPROM Control Register */ \
    } while(0)


/*******************************************************************************
 *                          Function Prototypes (Optional)                     *
 *******************************************************************************/

/* Add any commonly used function prototypes here if applicable to this header */


#endif /* MAIN_H */