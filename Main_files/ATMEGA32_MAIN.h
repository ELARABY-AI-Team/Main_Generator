/*********************************************************************
* File name: main.h
* Description: Core definitions and macros for ATMEGA32 embedded projects
* Author: AI
* Device name: ATmega32
* Creation date: 2025-06-30
* Standard: MISRA C
* Copyright: ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDDED SYSTEM GROUP
*********************************************************************/

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
typedef uint8_t  tbyte;  /* Unsigned byte type */
typedef uint16_t tword;  /* Unsigned word type (2 bytes) */
typedef uint32_t tlong;  /* Unsigned long type (4 bytes) */

/* Core macros */
#define SET_BIT(reg, bit)   do { reg |= (1 << bit); } while(0)
#define CLR_BIT(reg, bit)   do { reg &= ~(1 << bit); } while(0)
#define GET_BIT(reg, bit)   ((reg >> bit) & 1)
#define TOG_BIT(reg, bit)   do { reg ^= (1 << bit); } while(0)

/* ATmega32 specific macros */
#define DI        __disable_interrupt() /* Disable global interrupts */
#define EI        __enable_interrupt()  /* Enable global interrupts */
#define HALT      __asm__ volatile("sleep") /* Halt processor */
#define NOP       __asm__ volatile("nop")   /* No operation */

/* SAFEGUARD MACROS */
#define GPIO_SAFEGUARD_Init() do { \
    DDRA = 0x00; /* Set all Port A pins as input */ \
    DDRB = 0x00; /* Set all Port B pins as input */ \
    DDRC = 0x00; /* Set all Port C pins as input */ \
    PORTA = 0x00; /* Set all Port A pins to 0 */ \
    PORTB = 0x00; /* Set all Port B pins to 0 */ \
    PORTC = 0x00; /* Set all Port C pins to 0 */ \
} while(0)

#define Registers_SAFEGUARD_Init() do { \
    TIMSK |= (1 << TOIE1); /* Assuming Timer/Counter1 Overflow Interrupt Enable */ \
    TCCR1A = 0x00;       /* Reset Timer/Counter1 Control Register A */ \
    TCCR1B = 0x00;       /* Reset Timer/Counter1 Control Register B */ \
    WDTCR = (1 << WDCE) | (1 << WDE); /* Watchdog timer reset */ \
    ADCSRA = 0x00;      /* Disable ADC */ \
} while(0)

/* Note: This is a minimal implementation. Additional peripherals */
/*       should be added based on specific project requirements. */

#endif /* MAIN_H */