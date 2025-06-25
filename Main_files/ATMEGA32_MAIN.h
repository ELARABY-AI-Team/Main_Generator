/**
 * @file main.h
 * @brief A minimal header file for ATMEGA32 embedded projects.
 *        Includes essential standard libraries, common typedefs,
 *        core macros, and safeguard functions for initial system state.
 *
 * @author AI
 * @device ATMEGA32
 * @creation date 2025-06-25
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 */

#ifndef ATMEGA32_MAIN_H_
#define ATMEGA32_MAIN_H_

/* ==================================================================== */
/* Standard Includes                                                    */
/* ==================================================================== */
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

/* ==================================================================== */
/* MCU Specific Includes                                                */
/* ==================================================================== */
/* Include the device specific header file for register definitions */
#include <avr/io.h> /* PDF Reference */

/* ==================================================================== */
/* Typedefs                                                             */
/* ==================================================================== */
/** @brief 8-bit unsigned integer type */
typedef uint8_t tbyte;

/** @brief 16-bit unsigned integer type */
typedef uint16_t tword;

/** @brief 32-bit unsigned integer type */
typedef uint32_t tlong;

/* ==================================================================== */
/* Core Macros                                                          */
/* ==================================================================== */
/**
 * @brief Sets a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7).
 */
#define SET_BIT(reg, bit) ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7).
 */
#define CLR_BIT(reg, bit) ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register to read.
 * @param bit The bit number (0-7).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit) (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-7).
 */
#define TOG_BIT(reg, bit) ((reg) ^= (1U << (bit)))

/**
 * @brief Disables global interrupts.
 * Uses the 'cli' instruction.
 */
#define Global_Int_Disable() do { asm volatile ("cli" ::: "memory"); } while(0) /* PDF Reference (CLI instruction), Assumed compiler built-in/instruction */

/**
 * @brief Enables global interrupts.
 * Uses the 'sei' instruction.
 */
#define Global_Int_Enable() do { asm volatile ("sei" ::: "memory"); } while(0) /* PDF Reference (SEI instruction), Assumed compiler built-in/instruction */

/**
 * @brief Executes a No Operation instruction.
 * Uses the 'nop' instruction.
 */
#define NOP() do { asm volatile ("nop" ::: "memory"); } while(0) /* PDF Reference (NOP instruction), Assumed compiler built-in/instruction */

/**
 * @brief Puts the MCU into the selected sleep mode.
 * Requires SE bit in MCUCR to be set. Uses the 'sleep' instruction.
 */
#define SLEEP() do { asm volatile ("sleep" ::: "memory"); } while(0) /* PDF Reference (SLEEP instruction), Assumed compiler built-in/instruction */

/* ==================================================================== */
/* Safeguard Macros                                                     */
/* ==================================================================== */
/**
 * @brief Configures all GPIO ports to a safe, low-power input state.
 * Sets all pins to output 0, then configures them as inputs with pull-ups disabled.
 * This macro should typically be called after disabling peripherals.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        PORTA = 0x00; /* Set initial output data to 0 for all Port A pins */ /* PDF Reference */ \
        PORTB = 0x00; /* Set initial output data to 0 for all Port B pins */ /* PDF Reference */ \
        PORTC = 0x00; /* Set initial output data to 0 for all Port C pins */ /* PDF Reference */ \
        PORTD = 0x00; /* Set initial output data to 0 for all Port D pins */ /* PDF Reference */ \
        \
        DDRA = 0x00;  /* Configure all Port A pins as inputs */ /* PDF Reference */ \
        DDRB = 0x00;  /* Configure all Port B pins as inputs */ /* PDF Reference */ \
        DDRC = 0x00;  /* Configure all Port C pins as inputs */ /* PDF Reference */ \
        DDRD = 0x00;  /* Configure all Port D pins as inputs */ /* PDF Reference */ \
        \
        /* With DDRx = 0, PORTx = 0 disables pull-ups per pin. */ \
        /* Globally disable pull-ups as an extra safeguard. */ \
        SFIOR |= (1U << PUD); /* Disable global pull-ups */ /* PDF Reference */ \
        /* Note: PUD bit might need to be cleared later if pull-ups are desired. */ \
    } while(0)

/**
 * @brief Disables major peripherals and configures a safe system state.
 * Disables global interrupts, WDT, Timers, PWM, ICU, ADC, UART, TWI, SPI.
 * Then calls GPIO_SAFEGUARD_Init to configure pins as general I/O.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* 1. Disable global interrupts */ \
        Global_Int_Disable(); /* PDF Reference (CLI instruction) */ \
        \
        /* 2. Disable Watchdog Timer (WDT) */ \
        /* Requires a specific timed sequence */ \
        WDTCR = (1U << WDTOE) | (1U << WDE); /* Write logical one to WDTOE and WDE */ /* PDF Reference */ \
        WDTCR = 0x00; /* Turn off WDT (Write logical zero to WDE within 4 cycles) */ /* PDF Reference */ \
        /* Clear WDT Reset Flag (WDRF) by writing 0 to MCUCSR if needed, but POR also clears it. */ \
        \
        /* 3. Disable Timers (Timer0, Timer1, Timer2) clocks */ \
        TCCR0 = 0x00; /* Stop Timer/Counter0 (CS0 bits = 000) */ /* PDF Reference */ \
        TCCR1A = 0x00; /* Disconnect OC1A/OC1B */ /* PDF Reference */ \
        TCCR1B = 0x00; /* Stop Timer/Counter1 (CS1 bits = 000), disable Noise Canceler, Input Capture Edge Select */ /* PDF Reference */ \
        TCCR2 = 0x00; /* Stop Timer/Counter2 (CS2 bits = 000) */ /* PDF Reference */ \
        ASSR = 0x00; /* Ensure Timer2 is synchronous (AS2 = 0) */ /* PDF Reference */ \
        \
        /* 4. Disable Timer/Counter interrupts */ \
        TIMSK = 0x00; /* Disable all Timer/Counter interrupts */ /* PDF Reference */ \
        \
        /* 5. Clear Timer/Counter interrupt flags (write 1 to clear) */ \
        TIFR |= (1U << TOV0) | (1U << OCF0) | (1U << TOV1) | (1U << OCF1A) | (1U << OCF1B) | (1U << ICF1) | (1U << TOV2) | (1U << OCF2); /* PDF Reference */ \
        \
        /* 6. Disable Analog Comparator */ \
        ACSR = (1U << ACD); /* Power off Analog Comparator */ /* PDF Reference */ \
        ACSR &= ~((1U << ACIE) | (1U << ACIC) | (1U << ACIS1) | (1U << ACIS0)); /* Disable AC interrupt, Input Capture trigger, mode select */ /* PDF Reference */ \
        SFIOR &= ~(1U << ACME); /* Disable Analog Comparator Multiplexer */ /* PDF Reference */ \
        ACSR |= (1U << ACI); /* Clear Analog Comparator Interrupt Flag (write 1 to clear) */ /* PDF Reference */ \
        \
        /* 7. Disable ADC */ \
        ADCSRA = 0x00; /* Disable ADC (ADEN=0), stop conversions (ADSC=0), disable Auto Trigger (ADATE=0) */ /* PDF Reference */ \
        ADCSRA &= ~(1U << ADIE); /* Disable ADC Interrupt */ /* PDF Reference */ \
        ADCSRA |= (1U << ADIF); /* Clear ADC Interrupt Flag (write 1 to clear) */ /* PDF Reference */ \
        ADMUX = 0x00; /* Reset ADC Mux and Reference Selection */ /* PDF Reference */ \
        \
        /* 8. Disable USART (UART) */ \
        UCSRB = 0x00; /* Disable Receiver (RXEN=0), Transmitter (TXEN=0), disable interrupts (RXCIE, TXCIE, UDRIE) */ /* PDF Reference */ \
        /* Flush Receive buffer */ \
        while (UCSRA & (1U << RXC)) { /* While unread data exists */ /* PDF Reference */ \
            volatile uint8_t dummy_read = UDR; /* Read UDR to clear RXC */ /* PDF Reference */ \
            (void)dummy_read; /* Prevent unused variable warning */ \
        } \
        UCSRA |= (1U << TXC); /* Clear Transmit Complete Flag (write 1 to clear) */ /* PDF Reference */ \
        \
        /* 9. Disable TWI (I2C) */ \
        TWCR = 0x00; /* Disable TWI (TWEN=0) */ /* PDF Reference */ \
        TWCR &= ~(1U << TWIE); /* Disable TWI Interrupt */ /* PDF Reference */ \
        TWCR |= (1U << TWINT); /* Clear TWI Interrupt Flag (write 1 to clear) */ /* PDF Reference */ \
        \
        /* 10. Disable SPI communication */ \
        SPCR = 0x00; /* Disable SPI (SPE=0), reset config */ /* PDF Reference */ \
        SPCR &= ~(1U << SPIE); /* Disable SPI Interrupt */ /* PDF Reference */ \
        /* SPIF and WCOL flags are cleared by reading SPSR then SPDR. */ \
        /* Disabling the peripheral and interrupt is sufficient for safeguard. */ \
        \
        /* 11. Ensure JTAG interface is disabled (using soft-disable feature) */ \
        /* Requires a timed sequence to change JTD bit */ \
        /* This step is needed to make JTAG pins (PC2-PC5) available as GPIOs */ \
        MCUCSR = (1U << JTD); /* First write 1 to JTD */ /* PDF Reference */ \
        MCUCSR = (1U << JTD); /* Second write 1 to JTD within 4 cycles */ /* PDF Reference */ \
        \
        /* 12. Configure all GPIOs as inputs with pull-ups disabled */ \
        GPIO_SAFEGUARD_Init(); /* Configures PORTx, DDRx, SFIOR |= (1<<PUD) */ \
        \
        /* Other safeguards like disabling BOD (BODEN Fuse) or OCD (OCDEN/JTAGEN Fuses) */ \
        /* require Fuse programming and are outside the scope of this runtime safeguard macro. */ \
        /* Clear WDRF flag in MCUCSR (indicates WDT reset occurred) */ \
        /* Writing 0 to MCUCSR clears JTRF and WDRF, but not BORF/EXTRF/PORF (which are reset causes) */ \
        /* Only clear WDRF if desired to indicate no WDT reset happened *after* safeguard */ \
        /* MCUCSR &= ~(1U << WDRF); *//* Clear WDRF Flag by writing 0 */ /* PDF Reference */ \
        \
    } while(0)

/* ==================================================================== */
/* Notes:                                                               */
/* - The MISRA C standard imposes strict rules. Ensure that any code    */
/*   using this header file also adheres to these rules, especially     */
/*   regarding type casting, side effects in expressions, and volatile  */
/*   accesses.                                                          */
/* - Direct register access via `<avr/io.h>` is inherently non-MISRA   */
/*   compliant in some contexts (e.g., writing to registers that might  */
/*   be accessed by ISRs without proper atomicity). The safeguard macros*/
/*   disable interrupts globally before configuration to mitigate this   */
/*   during initialization. Application code must handle atomicity as needed.*/
/* - The provided PDF content describes the ATMEGA32's peripherals and */
/*   registers. The safeguard implementations are based directly on this */
/*   information.                                                       */
/* ==================================================================== */

#endif /* MAIN_H_ */