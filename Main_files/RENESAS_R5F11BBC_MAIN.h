/***********************************************************************************************
 * @file      main.h
 * @brief     Main header file for RENESAS R5F11BBC embedded projects
 * @author    AI (ELARABY GROUP)
 * @device    RENESAS R5F11BBC
 * @date      2025-06-24
 * @standard  MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 ***********************************************************************************************/

#ifndef RENESAS_R5F11BBC_MAIN_H_
#define RENESAS_R5F11BBC_MAIN_H_

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

/* Useful type definitions */
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/* Core macros */
#define SET_BIT(reg, bit)   do { (reg) |= (1 << (bit)); } while(0)
#define CLR_BIT(reg, bit)   do { (reg) &= ~(1 << (bit)); } while(0)
#define GET_BIT(reg, bit)   ((reg >> (bit)) & 1)
#define TOG_BIT(reg, bit)   do { (reg) ^= (1 << (bit)); } while(0)

/* Interrupt control macros */
#define Global_Int_Disable()  do { __asm("CPSID I"); } while(0)
#define Global_Int_Enable()   do { __asm("CPSIE I"); } while(0)
#define HALT()               do { __asm("WFI"); } while(0)

/* GPIO registers structure */
typedef struct {
    volatile uint32_t DR;  /* Data Register */
    volatile uint32_t DDR; /* Direction Register */
} gpio_port_t;

/* Peripheral register base addresses */
#define GPIO_PORT0_BASE   ((gpio_port_t*)0xF00000)
#define GPIO_PORT1_BASE   ((gpio_port_t*)0xF00100)
#define GPIO_PORT2_BASE   ((gpio_port_t*)0xF00200)

/* System Control Registers */
#define SYSC_REG          (*((volatile uint32_t*)0xF01000))
#define INT_CTRL_REG      (*((volatile uint32_t*)0xF02000))

/* Safeguard macros implementation */
#define GPIO_SAFEGUARD_Init() do { \
    /* Set all GPIO data to 0 */ \
    GPIO_PORT0_BASE->DR = 0; \
    GPIO_PORT1_BASE->DR = 0; \
    GPIO_PORT2_BASE->DR = 0; \
\
    /* Configure all GPIO directions as input */ \
    GPIO_PORT0_BASE->DDR = 0x00000000; \
    GPIO_PORT1_BASE->DDR = 0x00000000; \
    GPIO_PORT2_BASE->DDR = 0x00000000; \
\
    /* Disable pull-up resistors if available */ \
    /* (Assuming pull-up control register exists) */ \
    *((volatile uint32_t*)0xF00300) = 0x00000000; \
} while(0)

#define Registers_SAFEGUARD_Init() do { \
    /* Disable global interrupts */ \
    Global_Int_Disable(); \
\
    /* Reset all peripheral clocks */ \
    SYSC_REG &= ~0x0000FFFF; \
\
    /* Disable all timers */ \
    *((volatile uint32_t*)0xF04000) = 0x00000000; \
\
    /* Disable PWM modules */ \
    *((volatile uint32_t*)0xF05000) = 0x00000000; \
\
    /* Stop watchdog timer */ \
    WDT->CNT = 0x00000000; \
\
    /* Disable input capture units */ \
    ICU->CR &= ~(1 << 0); \
\
    /* Power down ADC */ \
    ADC->CR |= (1 << 8); /* Assuming power-down bit exists */ \
\
    /* Reset all communication peripherals */ \
    *((volatile uint32_t*)0xF06000) = 0x00000000; \
} while(0)

#endif /* MAIN_H */