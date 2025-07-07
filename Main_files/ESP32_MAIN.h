/**
 * @file main.h
 * @brief Minimal and clean header file for ATMEGA32 embedded projects.
 * @author Technology Inovation Software Team
 * @device ATMEGA32
 * @date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/*----------------------------------------------------------------------------
 *                            Include Directives
 *----------------------------------------------------------------------------*/

/* MCU-specific include */
#include <avr/io.h>
/* Include standard C headers */
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

/* Standard AVR Interrupts header (Often included by avr/io.h, but explicit is clearer) */
#include <avr/interrupt.h>
/* Standard AVR Watchdog header (For wdt_disable if not using direct register) */
/* #include <avr/wdt.h> // Not including as direct register access requested for WDT disable */


/*----------------------------------------------------------------------------
 *                               Useful Typedefs
 *----------------------------------------------------------------------------*/

/** @brief Typedef for an 8-bit unsigned integer */
typedef uint8_t tbyte;

/** @brief Typedef for a 16-bit unsigned integer */
typedef uint16_t tword;

/** @brief Typedef for a 32-bit unsigned integer */
typedef uint32_t tlong;

/*----------------------------------------------------------------------------
 *                                 Core Macros
 *----------------------------------------------------------------------------*/

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number to set (0-indexed).
 */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number to clear (0-indexed).
 */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit number to get (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number to toggle (0-indexed).
 */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/**
 * @brief Disable Global Interrupts.
 *        Corresponds to the cli() instruction.
 */
#define Global_Int_Disable()    cli()

/**
 * @brief Enable Global Interrupts.
 *        Corresponds to the sei() instruction.
 */
#define Global_Int_Enable()     sei()

/**
 * @brief Execute a No Operation (NOP) instruction.
 *        Forces the CPU to wait one cycle.
 */
#define NOP()                   __asm__ __volatile__ ("nop")

/**
 * @brief Software "halt" - Enters an infinite loop.
 *        Note: This is not a hardware halt but a common way to stop execution.
 */
#define HALT()                  do { while(1); } while(0)


/*----------------------------------------------------------------------------
 *                               SAFEGUARD MACROS
 *----------------------------------------------------------------------------*/

/**
 * @brief Configures all GPIO ports to a safe, initial state.
 *        Sets data registers to 0 and direction registers to input.
 *        Disables internal pull-ups by setting PORT bits to 0 when DDR bits are 0.
 *        Does not address specific per-pin wake-up settings as these are often peripheral-dependent.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set output values to 0 */ \
        PORTA = 0U; \
        PORTB = 0U; \
        PORTC = 0U; \
        PORTD = 0U; \
        \
        /* Set direction to input (DDRx = 0) */ \
        DDRA = 0U; \
        DDRB = 0U; \
        DDRC = 0U; \
        DDRD = 0U; \
        \
        /* Internal pull-ups are disabled when DDRx is 0 and PORTx is 0 */ \
    } while(0)

/**
 * @brief Disables common peripheral registers and sets a safe system state.
 *        Includes: Global Interrupts, Timers, Watchdog, ADC, USART, TWI (I2C), SPI.
 *        Also attempts to configure GPIO pins back to general I/O where applicable
 *        by disabling peripherals (this is the primary method on AVR).
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        cli(); \
        \
        /* Disable Timers (Timer/Counter0, 1, 2) */ \
        TCCR0 = 0x00U; TCNT0 = 0x00U; OCR0 = 0x00U; \
        TCCR1A = 0x00U; TCCR1B = 0x00U; TCCR1C = 0x00U; TCNT1H = 0x00U; TCNT1L = 0x00U; OCR1AH = 0x00U; OCR1AL = 0x00U; OCR1BH = 0x00U; OCR1BL = 0x00U; ICR1H = 0x00U; ICR1L = 0x00U; \
        TCCR2 = 0x00U; TCNT2 = 0x00U; OCR2 = 0x00U; \
        TIMSK = 0x00U; /* Timer/Counter Interrupt Mask Register */ \
        ETIMSK = 0x00U; /* Extended Timer/Counter Interrupt Mask Register (on ATmega32) */ \
        /* Clear Timer Interrupt Flags by writing 1 (if any are pending) */ \
        TIFR = (1U << TOV0) | (1U << OCF0) | (1U << TOV1) | (1U << OCF1A) | (1U << OCF1B) | (1U << ICF1) | (1U << TOV2) | (1U << OCF2); \
        ETIFR = 0x00U; /* No flags in ETIFR on ATmega32 */ \
        \
        /* Disable Watchdog Timer (requires timed sequence) */ \
        /* WARNING: This sequence is timing-critical and should occur within 4 clock cycles */ \
        /* of writing WDCE. Placing this in a macro might be sensitive to compiler optimization. */ \
        WDTCR |= (1U << WDCE) | (1U << WDE); /* Set WDCE (Watchdog Change Enable) and WDE (Watchdog Enable) */ \
        WDTCR = 0x00U; /* Clear WDE and prescaler bits */ \
        \
        /* Disable ADC */ \
        ADCSRA = 0x00U; /* Disable ADC (ADEN=0) and clear flags/settings */ \
        ADMUX = 0x00U; /* Reset MUX settings */ \
        ACSR = (1U << ACD); /* Disable Analog Comparator (ACD=1) */ \
        /* Clear ADC Interrupt Flag (if any) */ \
        SET_BIT(ADCSRA, ADIF); \
        \
        /* Disable USART (UART) */ \
        UCSRB = 0x00U; /* Disable TX/RX (RXEN=0, TXEN=0) */ \
        UCSRA = 0x00U; /* Clear flags/settings */ \
        /* UCSRC register requires URSEL bit set for access on ATmega32. */ \
        /* Writing 0 with URSEL=1 is a safe reset. */ \
        UBRRH = 0x00U; UBRRL = 0x00U; \
        SET_BIT(UCSRC, URSEL); UCSRC = (1U << URSEL); /* Set URSEL, then write safe default (UMSEL=0, UPM=0, USBS=0, UCSZ=0) */ \
        \
        /* Disable TWI (I2C) */ \
        TWCR = 0x00U; /* Disable TWI (TWEN=0) and clear flags */ \
        TWSR = 0x00U; /* Clear Status bits and Prescaler */ \
        TWAR = 0x00U; /* Clear Slave Address Register */ \
        TWBR = 0x00U; /* Clear Bit Rate Register */ \
        \
        /* Disable SPI */ \
        SPCR = 0x00U; /* Disable SPI (SPE=0) */ \
        SPSR = 0x00U; /* Clear flags */ \
        /* SPDR is data register, no need to clear */ \
        \
        /* Configure GPIOs as I/O (not special functions): */ \
        /* Disabling the peripherals above effectively returns their pins */ \
        /* to default GPIO functionality, which is controlled by the DDR/PORT registers. */ \
        /* The GPIO_SAFEGUARD_Init() macro should be called after this */ \
        /* or ensure it is called before main execution for a defined pin state. */ \
        \
    } while(0)


/*----------------------------------------------------------------------------
 *                           End of File Definition
 *----------------------------------------------------------------------------*/

#endif /* MAIN_H_ */