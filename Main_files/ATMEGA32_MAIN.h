/**
 ******************************************************************************
 * @file    main.h
 * @brief   Main header file for ATmega32 project. Includes common definitions
 *          and safeguard functions.
 * @author  Technology Inovation Software Team
 * @device  ATMEGA32
 * @date    2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 ******************************************************************************
 */

#ifndef _MAIN_H_
#define _MAIN_H_

/* Include Guard Note: Using a simple unique identifier is MISRA C compliant. */

/*----------------------------------------------------------------------------
 *                           MCU Specific Include
 *----------------------------------------------------------------------------*/
/* Include the appropriate header file for the ATmega32 microcontroller */
#include <avr/io.h>

/*----------------------------------------------------------------------------
 *                           Standard C Includes
 *----------------------------------------------------------------------------*/
/*
 * Including standard C headers. Note that some headers might be considered
 * less critical or potentially problematic in extremely resource-constrained
 * or strict MISRA environments (e.g., <setjmp.h>, <stdlib.h> without careful
 */
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <iso646.h>  /* Provides alternative tokens (e.g., 'and' for '&&') */
#include <limits.h>
#include <math.h>
#include <setjmp.h>  /* For non-local jumps, typically avoided in embedded */
#include <stdarg.h>  /* For variable argument lists */
#include <stdbool.h> /* For boolean type (_Bool) and macros (true, false) */
#include <stddef.h>  /* For size_t, ptrdiff_t, NULL */
#include <stdint.h>  /* For fixed-width integer types */
#include <stdio.h>   /* For input/output functions (e.g., printf for debugging) */
#include <stdlib.h>  /* For general utilities (e.g., malloc, atoi), use with care */
#include <string.h>  /* For string manipulation functions */

/*----------------------------------------------------------------------------
 *                      Additional AVR-Libc Specific Includes
 *----------------------------------------------------------------------------*/
/* Needed for standard AVR global interrupt functions (cli(), sei()) */
#include <avr/interrupt.h>
/* Needed for standard AVR watchdog timer functions (wdt_disable()) */
#include <avr/wdt.h>
/* Needed for standard AVR NOP intrinsic (_NOP()) */
#include <avr/builtins.h>


/*----------------------------------------------------------------------------
 *                              Useful Typedefs
 *----------------------------------------------------------------------------*/
/* Define standard fixed-width integer types with embedded-specific names */
typedef uint8_t  tbyte;  /**< Alias for 8-bit unsigned integer */
typedef uint16_t tword;  /**< Alias for 16-bit unsigned integer */
typedef uint32_t tlong;  /**< Alias for 32-bit unsigned integer */


/*----------------------------------------------------------------------------
 *                               Core Macros
 *----------------------------------------------------------------------------*/
/*
 * Generic bit manipulation macros.
 * Ensure bit argument is within the range of the register type.
 * Using 1U for unsigned literal as per MISRA C guidelines.
 */
#define SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))    /**< Set a specific bit in a register */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))   /**< Clear a specific bit in a register */
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1U)     /**< Get the value of a specific bit in a register (returns 0 or 1) */
#define TOG_BIT(reg, bit)   ((reg) ^= (1U << (bit)))    /**< Toggle a specific bit in a register */

/*
 * Global Interrupt Control Macros/Functions.
 * Uses AVR-Libc provided intrinsic functions.
 */
#define DI()                cli()                       /**< Disable Global Interrupts (uses AVR-Libc cli()) */
#define EI()                sei()                       /**< Enable Global Interrupts (uses AVR-Libc sei()) */
#define Global_Int_Disable() cli()                      /**< Alias for cli() */
#define Global_Int_Enable()  sei()                      /**< Alias for sei() */

/*
 * CPU Control Macros/Functions.
 */
#define NOP()               _NOP()                      /**< Execute a No Operation instruction (uses AVR-Libc _NOP() intrinsic) */
/*
 * HALT() macro. Implemented as an infinite loop, which effectively halts CPU
 * execution in a known state. For low-power sleep modes, refer to specific
 * power management functions and setup (e.g., sleep_mode()).
 */
#define HALT()              while(1) {}                 /**< Halt CPU execution in an infinite loop */


/*----------------------------------------------------------------------------
 *                             SAFEGUARD MACROS
 *----------------------------------------------------------------------------*/
/*
 * IMPORTANT: These macros contain full implementations directly, as required
 * by the prompt. This is not standard practice for complex initializations,
 * which are usually placed in functions in a .c file. The do...while(0)
 * structure is used to allow safe usage in if/else statements.
 */

/**
 * @brief Safeguard macro to initialize all GPIO ports to a safe, known state.
 *
 * Configures all data/output registers to 0, sets all data direction
 * registers to input, and disables pull-up resistors and wake-up features
 * related to GPIO pins by setting PORTx and DDRx to 0x00.
 * Assumes default power-on state for other GPIO-related registers
 * (like external interrupt control registers which are handled
 * by Registers_SAFEGUARD_Init).
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Set all port outputs/pull-ups to 0 */ \
        PORTA = 0x00; \
        PORTB = 0x00; \
        PORTC = 0x00; \
        PORTD = 0x00; \
        \
        /* Set all port directions to input */ \
        DDRA = 0x00; \
        DDRB = 0x00; \
        DDRC = 0x00; \
        DDRD = 0x00; \
        \
        /* On ATmega32, setting DDRx to 0 and PORTx to 0 disables pull-ups. */ \
        /* Specific GPIO wake-up registers (like external interrupt masks) */ \
        /* are handled by Registers_SAFEGUARD_Init(). */ \
    } while(0)

/**
 * @brief Safeguard macro to disable most common peripherals and interrupts.
 *
 * Disables global interrupts, timers, PWM (via timers), watchdog timer,
 * input capture unit (via Timer 1), ADC, UART, I2C, and SPI.
 * Also clears related interrupt flags and disables interrupt enables.
 * This helps ensure peripherals do not interfere with general I/O
 * functionality (set by GPIO_SAFEGUARD_Init) or cause unexpected behavior
 * upon startup.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable Global Interrupts */ \
        cli(); \
        \
        /* Disable Timers (0, 1, 2) */ \
        TCCR0 = 0x00; \
        TCCR1A = 0x00; \
        TCCR1B = 0x00; \
        TCCR2 = 0x00; \
        /* Clear Timer Interrupt Flags by writing 1s */ \
        TIFR = (1<<TOV0) | (1<<OCF0) | (1<<TOV1) | (1<<OCF1A) | (1<<OCF1B) | (1<<ICF1) | (1<<TOV2) | (1<<OCF2); /* Clear all flags by writing 1 to each bit */ \
        /* Disable Timer Interrupt Enables */ \
        TIMSK = 0x00; \
        \
        /* Disable PWM (Implicitly done by disabling Timers) */ \
        /* Optionally clear Output Compare Registers (duty cycles) */ \
        OCR0 = 0x00; \
        OCR1A = 0x00; OCR1B = 0x00; \
        OCR2 = 0x00; \
        \
        /* Disable Watchdog Timer */ \
        /* Note: This requires a specific timed sequence. Using AVR-Libc function is safer. */ \
        /* Include <avr/wdt.h> is required for wdt_disable() */ \
        wdt_disable(); \
        \
        /* Disable Input Capture Unit (Implicitly done by disabling Timer 1) */ \
        /* ICU shares TCCR1B register and ICF1 flag with Timer 1 */ \
        \
        /* Disable ADC */ \
        ADCSRA = 0x00; /* Clears ADEN (Enable), ADSC (Start Conv), ADATE (Auto Trigger), ADIE (Interrupt Enable) */ \
        /* Clear pending ADC Interrupt Flag (ADIF) by writing 1 */ \
        ADCSRA |= (1<<ADIF); /* Write 1 to ADIF bit */ \
        ADMUX = 0x00; /* Reset ADMUX to default (AREF, ADC0) */ \
        SFIOR &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); /* Disable ADC Auto Trigger Source (set to Free Running Mode implicit in ADATE=0) */ \
        \
        /* Disable UART (USART) */ \
        UCSRB = 0x00; /* Clears RXEN, TXEN, RXCIE, TXCIE, UDRIE, TXB8, RXB8 */ \
        /* Clear pending UART Interrupt Flags (RXC, TXC, FE, DOR, PE) by writing 1s */ \
        UCSRA = (1<<RXC) | (1<<TXC) | (1<<FE) | (1<<DOR) | (1<<PE); \
        /* UCSRC has default mode (Async, 8N1, 1 Stop) after reset, no need to clear unless specific modes were set */ \
        \
        /* Disable I2C (TWI) */ \
        TWCR = 0x00; /* Clears TWEN (Enable), TWIE (Interrupt Enable), TWEA, TWSTO, TWSTA, TWWC, TWINT */ \
        /* Clear pending TWI Interrupt Flag (TWINT) by writing 1 */ \
        TWCR = (1<<TWINT); /* Set TWINT bit and write to TWCR to clear flag */ \
        /* TWBR and TWAR contain clock speed and address, not enabling/disabling */ \
        \
        /* Disable SPI */ \
        SPCR = 0x00; /* Clears SPIE (Interrupt Enable), SPE (Enable), DORD, MSTR, CPOL, CPHA */ \
        /* Clear pending SPI Interrupt Flag (SPIF) by writing 1 */ \
        SPSR |= (1<<SPIF); /* Write 1 to SPIF bit */ \
        /* SPDR holds data, not enabling/disabling */ \
        \
        /* Disable External Interrupts (INT0, INT1, INT2) */ \
        GICR = 0x00; /* Clears INT1, INT0, INT2 enable bits */ \
        GIFR = (1<<INTF1) | (1<<INTF0) | (1<<INTF2); /* Clear External Interrupt Flags by writing 1s */ \
        MCUCR = 0x00; /* Reset External Interrupt Control Register (configures trigger edge/level) */ \
        MCUCSR &= ~(1<<ISC2); /* Reset ISC2 bit for INT2 control */ \
        \
        /* Ensure pins used by disabled peripherals revert to generic I/O */ \
        /* (Handled by GPIO_SAFEGUARD_Init setting DDRx and PORTx) */ \
        \
    } while(0)


#endif /* _MAIN_H_ */