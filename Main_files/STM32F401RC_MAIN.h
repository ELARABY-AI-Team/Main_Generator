/******************************************************************************************
 * @file     main.h
 * @brief    Main header file for STM32F401RC embedded projects
 * @author   AI (ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDDED SYSTEM GROUP)
 * @device   STM32F401RC
 * @date     2025-06-30
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP
 *****************************************************************************************/

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

/* Include MCU-specific headers */
#include "stm32f4xx.h"

/* Useful typedefs */
typedef uint8_t  tbyte;    /* Byte type                   */
typedef uint16_t tword;    /* Word (16-bit) type          */
typedef uint32_t tlong;    /* Long word (32-bit) type     */

/* Core macros */
#define SET_BIT(reg, bit)   ((reg) |= (1 << (bit)))        /* Set specified bit in register */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1 << (bit)))       /* Clear specified bit in register */
#define GET_BIT(reg, bit)   ((reg >> (bit)) & 1)           /* Get value of specified bit */
#define TOG_BIT(reg, bit)   ((reg) ^= (1 << (bit)))        /* Toggle specified bit in register */

/* Interrupt control macros */
#define DI()                __disable_irq()               /* Disable global interrupts */
#define EI()                __enable_irq()                /* Enable global interrupts  */
#define HALT()              __WFI()                       /* Halt CPU execution       */
#define NOP()               __NOP()                       /* No operation instruction */

/* SAFEGUARD MACROS */
#define GPIO_SAFEGUARD_Init() do { \
    /* Configure all GPIO ports as input with no pull-up resistors */ \
    GPIOA->MODER = 0x00000000; /* All pins input mode */ \
    GPIOB->MODER = 0x00000000; \
    GPIOC->MODER = 0x00000000; \
    GPIOD->MODER = 0x00000000; \
    GPIOE->MODER = 0x00000000; \
    GPIOF->MODER = 0x00000000; \
    GPIOG->MODER = 0x00000000; \
    GPIOH->MODER = 0x00000000; \
    GPIOI->MODER = 0x00000000; \
    GPIOJ->MODER = 0x00000000; \
    GPIOK->MODER = 0x00000000; \
    /* Set all GPIO ports to 0 */ \
    GPIOA->ODR = 0x00000000; \
    GPIOB->ODR = 0x00000000; \
    GPIOC->ODR = 0x00000000; \
    GPIOD->ODR = 0x00000000; \
    GPIOE->ODR = 0x00000000; \
    GPIOF->ODR = 0x00000000; \
    GPIOG->ODR = 0x00000000; \
    GPIOH->ODR = 0x00000000; \
    GPIOI->ODR = 0x00000000; \
    GPIOJ->ODR = 0x00000000; \
    GPIOK->ODR = 0x00000000; \
} while(0)

#define Registers_SAFEGUARD_Init() do { \
    /* Disable global interrupts */ \
    DI(); \
    /* Reset all peripherals (using RCC_APB2ENR and RCC_APB1ENR registers) */ \
    RCC->APB2ENR = 0x00000000; /* Disable all APB2 peripherals */ \
    RCC->APB1ENR = 0x00000000; /* Disable all APB1 peripherals */ \
    /* Set all GPIOs as general purpose I/O pins */ \
    /* (Done in GPIO_SAFEGUARD_Init() above) */ \
} while(0)