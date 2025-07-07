/**
 * @file main.h
 * @brief Main header file for ATmega32 microcontroller projects.
 *        Includes essential standard libraries, core types, and utility macros.
 *        Provides safeguard macros for initial system state configuration.
 * @author Technology Inovation Software Team
 * @device ATMEGA32
 * @creation date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/* --- Includes --- */

/**
 * @brief Include device specific header file.
 *        This typically contains definitions for peripherals and registers.
 */
#include <avr/io.h>

/*
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

/* --- Useful Typedefs --- */

/**
 * @brief Standard integer types with clearer names.
 */
typedef uint8_t  tbyte; /**< 8-bit unsigned integer */
typedef uint16_t tword; /**< 16-bit unsigned integer */
typedef uint32_t tlong; /**< 32-bit unsigned integer */


/* --- Core Macros --- */

/**
 * @brief Set a specific bit in a register.
 * @param reg The register to modify (must be a modifiable lvalue).
 * @param bit The bit number (0-7).
 */
#define SET_BIT(reg, bit)    ((reg) |= (1U << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg The register to modify (must be a modifiable lvalue).
 * @param bit The bit number (0-7).
 */
#define CLR_BIT(reg, bit)    ((reg) &= ~(1U << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param reg The register to read.
 * @param bit The bit number (0-7).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)    (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg The register to modify (must be a modifiable lvalue).
 * @param bit The bit number (0-7).
 */
#define TOG_BIT(reg, bit)    ((reg) ^= (1U << (bit)))

/**
 * @brief Disable global interrupts.
 *        Maps to the AVR 'cli' instruction.
 */
#define DI()    asm volatile("cli"::)

/**
 * @brief Enable global interrupts.
 *        Maps to the AVR 'sei' instruction.
 */
#define EI()    asm volatile("sei"::)

/**
 * @brief Disable global interrupts (alias for DI).
 */
#define Global_Int_Disable() DI()

/**
 * @brief Enable global interrupts (alias for EI).
 */
#define Global_Int_Enable()  EI()

/**
 * @brief Execute a No Operation (NOP) instruction.
 *        Maps to the AVR 'nop' instruction.
 */
#define NOP()    asm volatile("nop"::)

/**
 * @brief Halt execution indefinitely.
 *        Typically used at the end of main or in error states.
 *        Causes the program to loop forever.
 */
#define HALT()   do {} while(1)


/* --- SAFEGUARD MACROS --- */

/**
 * @brief Macro to initialize all GPIO ports to a safe, known state.
 *        Sets data/output registers to 0, configures all pins as inputs,
 *        disables internal pull-up resistors, and disables external/pin change interrupts.
 *        Uses actual ATmega32 registers (PORTx, DDRx, GICR, MCUCR, MCUCSR).
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set all Port Data Registers to 0 (Output Low or Pull-up Disabled) */ \
        PORTA = 0x00; \
        PORTB = 0x00; \
        PORTC = 0x00; \
        PORTD = 0x00; \
        \
        /* Configure all Data Direction Registers to Input (0) */ \
        DDRA = 0x00; \
        DDRB = 0x00; \
        DDRC = 0x00; \
        DDRD = 0x00; \
        \
        /* Disabling Pull-ups: Setting PORTx bits to 0 when DDRx is 0 disables internal pull-ups. */ \
        /* This is already done by the PORTx = 0x00 steps above. */ \
        \
        /* Disable External Interrupts (INT0, INT1, INT2) */ \
        GICR &= ~((1U << INT0) | (1U << INT1) | (1U << INT2)); \
        \
        /* Configure External Interrupt trigger to a safer default (low level for INT0/INT1, falling edge for INT2) */ \
        /* This prevents spurious interrupts if GICR is later re-enabled without reconfiguring MCUCR/MCUCSR */ \
        MCUCR = 0x00; /* Sets ISC0[1:0] and ISC1[1:0] to 00b (Low Level Trigger) */ \
        MCUCSR &= ~(1U << ISC2); /* Sets ISC2 to 0 (Falling Edge Trigger) */ \
        \
        /* Note: Pin Change Interrupts are not standard on ATmega32 in the same way as newer AVRs (no PCICR/PCMSK). */ \
        /* External Interrupts INT0/INT1/INT2 are handled above. */ \
        \
    } while(0)

/**
 * @brief Macro to initialize various peripheral registers to a safe, known (typically disabled) state.
 *        Disables global interrupts, timers, PWM, WDT, ICU, ADC, UART, I2C, SPI.
 *        Also ensures GPIOs are configured as standard I/O, not special functions, by disabling peripherals.
 *        Uses actual ATmega32 registers (SREG, TCCRnx, TIMSK, TIFR, WDTCR, ADCSRA, UCSRn, TWCR, SPCR etc.).
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        asm volatile("cli"::); \
        \
        /* Disable and Reset Timers (Timer0, Timer1, Timer2) */ \
        TCCR0 = 0x00; /* Stop Timer0 (no clock source) */ \
        TCNT0 = 0x00; /* Clear Timer0 count */ \
        OCR0 = 0x00; /* Clear Timer0 Output Compare Register */ \
        \
        TCCR1A = 0x00; /* Stop Timer1 (no clock source, WGM bits 0/1 to Normal mode) */ \
        TCCR1B = 0x00; /* Stop Timer1 (no clock source, WGM bits 2/3 to Normal mode) */ \
        TCNT1H = 0x00; /* Clear Timer1 count (High Byte) */ \
        TCNT1L = 0x00; /* Clear Timer1 count (Low Byte) */ \
        OCR1A = 0x00; /* Clear Timer1 Output Compare Register A */ \
        OCR1B = 0x00; /* Clear Timer1 Output Compare Register B */ \
        ICR1H = 0x00; /* Clear Timer1 Input Capture Register (High Byte) */ \
        ICR1L = 0x00; /* Clear Timer1 Input Capture Register (Low Byte) */ \
        \
        TCCR2 = 0x00; /* Stop Timer2 (no clock source) */ \
        TCNT2 = 0x00; /* Clear Timer2 count */ \
        OCR2 = 0x00; /* Clear Timer2 Output Compare Register */ \
        \
        TIMSK = 0x00; /* Disable all Timer Interrupts (TOIE0, OCIE0, TOIE1, OCIE1A, OCIE1B, TICIE1, TOIE2, OCIE2) */ \
        TIFR = 0xFF;  /* Clear all pending Timer Interrupt Flags by writing 1 to them */ \
        \
        /* Disable PWM: Handled by clearing TCCRn registers and OCRn registers above. */ \
        \
        /* Disable Watchdog Timer (Requires specific timed sequence - per datasheet) */ \
        /* NOTE: If the WDT is already configured for System Reset mode, this sequence */ \
        /* must be executed correctly and quickly after reset, BEFORE a WDR (Watchdog Reset). */ \
        /* If WDT is locked (fuse), this sequence will not work. */ \
        WDTCR |= (1U << WDCE) | (1U << WDE); /* Set WDCE (Watchdog Change Enable) and WDE (Watchdog Enable) */ \
        /* Within 4 clock cycles of setting WDCE, write 0 to WDE and prescaler bits */ \
        WDTCR = 0x00; /* Clear WDE and prescalers (WDPS[2:0]) */ \
        \
        /* Disable Input Capture Unit (ICU): Handled by clearing TCCR1B (ICES1, ICNC1) and TIMSK (TICIE1) above. */ \
        \
        /* Disable Analog to Digital Converter (ADC) */ \
        ADCSRA &= ~(1U << ADEN); /* Clear ADEN bit (ADC Enable) */ \
        ADCSRA &= ~(1U << ADIE); /* Clear ADIE bit (ADC Interrupt Enable) */ \
        ADCSRA |= (1U << ADIF);  /* Clear pending ADIF flag by writing 1 */ \
        ADMUX = 0x00; /* Reset ADMUX (Select ADC0, External AREF) - safe default */ \
        ACSR &= ~(1U << ACIE); /* Disable Analog Comparator Interrupt */ \
        ACSR |= (1U << ACI);   /* Clear pending Analog Comparator Interrupt flag */ \
        \
        /* Disable UART (USART) */ \
        UCSRB = 0x00; /* Disable Receiver (RXEN), Transmitter (TXEN), and all associated interrupts (RXCIE, TXCIE, UDRIE) */ \
        UCSRA = 0x00; /* Clear status flags (MPCM, U2X, PE, DOR, FE, TXC, RXC) - UDRE is read-only/flag */ \
        /* Note: UCSRC settings are often left at default or ignored if UCSRB prevents operation. Clearing UBRR is optional but safe. */ \
        UBRRH = 0x00; /* Clear Baud Rate Registers */ \
        UBRRL = 0x00; \
        \
        /* Disable I2C (Two Wire Interface - TWI) */ \
        TWCR = 0x00; /* Disable TWI Enable (TWEN), Interrupt Enable (TWIE), and clear flags (TWINT, TWEA, TWWC, TWSTO, TWSTA) */ \
        TWSR = 0x00; /* Reset Status Register (Clear prescalers, leave status code) */ \
        TWBR = 0x00; /* Clear Bit Rate Register */ \
        \
        /* Disable SPI */ \
        SPCR = 0x00; /* Disable SPI Enable (SPE), Interrupt Enable (SPIE), reset other settings (DORD, MSTR, CPOL, CPHA, SPR) */ \
        SPSR |= (1U << SPIF); /* Clear pending SPI Interrupt Flag by writing 1 */ \
        /* Note: SPI2X in SPSR is a control bit but often reset by SPCR=0. */ \
        \
        /* Configure GPIOs as I/O, not special functions: This is primarily achieved */ \
        /* by the GPIO_SAFEGUARD_Init() macro (setting DDR to input) and by disabling */ \
        /* the peripheral modules above, which relinquishes control of the pins. */ \
        \
    } while(0)


#endif /* MAIN_H_ */