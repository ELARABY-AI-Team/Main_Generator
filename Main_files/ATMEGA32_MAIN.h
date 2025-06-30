/**********************************************************************
 * @file     main.h
 * @brief    Main header file with common definitions and macros
 * @author   AI
 * @device   ATMEGA32
 * @date     2025-06-30
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDDED SYSTEM GROUP
 **********************************************************************/

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
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/* Core macros */
#define SET_BIT(reg, bit)     ((reg) |= (1 << (bit)))
#define CLR_BIT(reg, bit)     ((reg) &= ~(1 << (bit)))
#define GET_BIT(reg, bit)     ((reg) & (1 << (bit)))
#define TOG_BIT(reg, bit)     ((reg) ^= (1 << (bit)))

#if defined(__GNUC__)
    #define DI   __asm__ volatile("cli" ::)
    #define EI   __asm__ volatile("sei" ::)
    #define HALT __asm__ volatile("halt" ::)
    #define NOP  __asm__ volatile("nop" ::)
#else
    #error "Compiler not supported"
#endif

/* SAFEGUARD MACROS */
#define GPIO_SAFEGUARD_Init() do { \
    /* Set all GPIO ports to 0 */ \
    PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; \
    /* Set all GPIO directions as input */ \
    DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; \
    /* Disable pull-up resistors for all ports */ \
    PUEA = 0; PUEB = 0; PUEC = 0; PUED = 0; \
} while(0)

#define Registers_SAFEGUARD_Init() do { \
    /* Disable global interrupts */ \
    cli(); \
    /* Disable all timers (Timer/Counter Control Register) */ \
    TIMSK &= ~(1 << TOIE3 | 1 << OCIE3B | 1 << OCIE3A | 1 << TOIE2 | 1 << OCIE2B | 1 << OCIE2A | 1 << TOIE1 | 1 << OCIE1B | 1 << OCIE1A); \
    TCCR3B = 0; TCCR2B = 0; TCCR1B = 0; \
    /* Disable PWM on all channels */ \
    TCCR0 &= ~(1 << WGM00 | 1 << WGM01); \
    TCCR1A &= ~(1 << WGM10 | 1 << WGM11); \
    TCCR2 &= ~(1 << WGM20 | 1 << WGM21); \
    TCCR3A &= ~(1 << WGM30 | 1 << WGM31); \
    /* Disable watchdog timer */ \
    WDTCR |= (1 << WDCE); \
    WDTCR = 0; \
    /* Disable ADC */ \
    ADCSRA &= ~(1 << ADEN); \
    /* Configure all GPIO as general purpose I/O pins */ \
    DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; \
} while(0)