/**
 * @file main.h
 * @brief Main project header file including core definitions and safeguards.
 * @author Embedded C Engineer
 * @device NXP LPC1768
 * @creation date 2025-06-18
 * @copyright Copyright (c) 2025
 */

#ifndef NXP LPC1768_MAIN_H_
#define NXP LPC1768_MAIN_H_

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

/* MCU specific headers */
#include "LPC17xx.h"             /* NXP LPC17xx devices header */

/* Useful typedefs */
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/* Core Macros */

/** @brief Set a specific bit in a register. */
#define SET_BIT(reg, bit)       ((reg) |= (1UL << (bit)))

/** @brief Clear a specific bit in a register. */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1UL << (bit)))

/** @brief Get the value of a specific bit in a register. */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1UL)

/** @brief Toggle a specific bit in a register. */
#define TOG_BIT(reg, bit)       ((reg) ^= (1UL << (bit)))

/** @brief Disable global interrupts. */
#define DI()                    __disable_irq()

/** @brief Enable global interrupts. */
#define EI()                    __enable_irq()

/** @brief Execute No Operation instruction. */
#define NOP()                   __nop()

/** @brief Wait For Interrupt instruction (low power mode). */
#define HALT()                  __wfi()


/* SAFEGUARD MACROS */

/**
 * @brief Configures all GPIO pins to a safe default state (Input).
 * This is done by setting FIODIRx registers to 0, PINSELx to 0 (GPIO function),
 * and PINMODEx to 0 (Pull-up enabled).
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Configure all pins as GPIO function */ \
        LPC_PINCON->PINSEL0 = 0x00000000; \
        LPC_PINCON->PINSEL1 = 0x00000000; \
        LPC_PINCON->PINSEL2 = 0x00000000; \
        LPC_PINCON->PINSEL3 = 0x00000000; \
        LPC_PINCON->PINSEL4 = 0x00000000; \
        LPC_PINCON->PINSEL5 = 0x00000000; /* For P2.10-P2.13 */ \
        LPC_PINCON->PINSEL6 = 0x00000000; /* For P3 */ \
        LPC_PINCON->PINSEL7 = 0x00000000; /* For P3 */ \
        LPC_PINCON->PINSEL8 = 0x00000000; /* For P4 */ \
        LPC_PINCON->PINSEL9 = 0x00000000; /* For P4.28-P4.31 */ \
        \
        /* Configure all pins with internal pull-up resistor enabled (Mode 00) */ \
        LPC_PINCON->PINMODE0 = 0x00000000; \
        LPC_PINCON->PINMODE1 = 0x00000000; \
        LPC_PINMODE_OD0 = 0x00000000; /* Ensure Open-Drain is disabled for P0 */ \
        LPC_PINCON->PINMODE2 = 0x00000000; \
        LPC_PINCON->PINMODE3 = 0x00000000; \
        LPC_PINMODE_OD1 = 0x00000000; /* Ensure Open-Drain is disabled for P1 */ \
        LPC_PINCON->PINMODE4 = 0x00000000; \
        LPC_PINCON->PINMODE5 = 0x00000000; \
        LPC_PINMODE_OD2 = 0x00000000; /* Ensure Open-Drain is disabled for P2 */ \
        LPC_PINCON->PINMODE6 = 0x00000000; \
        LPC_PINCON->PINMODE7 = 0x00000000; \
        LPC_PINMODE_OD3 = 0x00000000; /* Ensure Open-Drain is disabled for P3 */ \
        LPC_PINCON->PINMODE8 = 0x00000000; \
        LPC_PINCON->PINMODE9 = 0x00000000; \
        LPC_PINMODE_OD4 = 0x00000000; /* Ensure Open-Drain is disabled for P4 */ \
        \
        /* Set all GPIOs as input */ \
        LPC_GPIO0->FIODIR = 0x00000000; \
        LPC_GPIO1->FIODIR = 0x00000000; \
        LPC_GPIO2->FIODIR = 0x00000000; \
        LPC_GPIO3->FIODIR = 0x00000000; \
        LPC_GPIO4->FIODIR = 0x00000000; \
    } while(0)

/**
 * @brief Disables power to most on-chip peripherals by clearing PCONP register.
 * This is a standard safety measure to reduce power consumption and ensure
 * peripherals start in a known state.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        /* Disable power/clock to most peripherals */ \
        LPC_SC->PCONP = 0x00000000; \
        \
        /* Note: GPIO power is typically always on or enabled by default. */ \
        /* System Control Block, SysTick, NVIC, Flash Controller are not in PCONP. */ \
        \
    } while(0)


#endif /* MAIN_H */