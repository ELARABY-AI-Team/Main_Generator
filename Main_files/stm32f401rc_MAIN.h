/*************************************************************************
* File name: main.h
* Description: Core header file containing essential definitions and macros
              for embedded projects using STM32F401RC microcontroller.
* Author: AI
* Device name: STM32F401RC
* Creation date: 2025-06-30
* Standard: MISRA C
* Copyright: ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDDED SYSTEM GROUP
*************************************************************************/

#ifndef STM32F401RC_MAIN_H_
#define STM32F401RC_MAIN_H_

/* Include necessary standard headers */
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
typedef uint8_t  tbyte;  /* Unsigned byte type */
typedef uint16_t tword;  /* Unsigned word type (16-bit) */
typedef uint32_t tlong;  /* Unsigned long type (32-bit) */

/* Core macros */
#define SET_BIT(reg, bit)   ((reg) |= (1 << (bit))) /* Set specific bit in register */
#define CLR_BIT(reg, bit)   ((reg) &= ~(1 << (bit))) /* Clear specific bit in register */
#define GET_BIT(reg, bit)   ((reg >> (bit)) & 1)     /* Get value of specific bit */
#define TOG_BIT(reg, bit)   ((reg) ^= (1 << (bit))) /* Toggle specific bit */

/* Interrupt control macros */
#define DI    __disable_irq() /* Disable global interrupts */
#define EI    __enable_irq()  /* Enable global interrupts */
#define HALT  while(1);      /* Halt the system */
#define NOP   do { } while(0) /* No operation */

/* Safeguard initialization macros */
#define GPIO_SAFEGUARD_Init() do { \
    /* Configure all GPIO ports to reset state */ \
    /* Set all ports' output data to 0 */ \
    GPIOA->ODR = 0; \
    GPIOB->ODR = 0; \
    GPIOC->ODR = 0; \
    GPIOD->ODR = 0; \
    GPIOE->ODR = 0; \
    GPIOF->ODR = 0; \
    GPIOG->ODR = 0; \
    GPIOH->ODR = 0; \
    /* Set all ports to input mode */ \
    GPIOA->MODER = 0x00000000; \
    GPIOB->MODER = 0x00000000; \
    GPIOC->MODER = 0x00000000; \
    GPIOD->MODER = 0x00000000; \
    GPIOE->MODER = 0x00000000; \
    GPIOF->MODER = 0x00000000; \
    GPIOG->MODER = 0x00000000; \
    GPIOH->ODR = 0x00000000; \
    /* Disable pull-up/pull-down resistors */ \
    GPIOA->PUPDR = 0x00000000; \
    GPIOB->PUPDR = 0x00000000; \
    GPIOC->PUPDR = 0x00000000; \
    GPIOD->PUPDR = 0x00000000; \
    GPIOE->PUPDR = 0x00000000; \
    GPIOF->PUPDR = 0x00000000; \
    GPIOG->PUPDR = 0x00000000; \
    } while(0)

#define Registers_SAFEGUARD_Init() do { \
    /* Disable global interrupts */ \
    DI; \
    /* Disable all timers */ \
    TIM1->CR1 = 0; \
    TIM2->CR1 = 0; \
    TIM3->CR1 = 0; \
    TIM4->CR1 = 0; \
    /* Disable PWM */ \
    TIM1->CCER &= ~TIM_CCER_CC1E; \
    TIM2->CCER &= ~TIM_CCER_CC1E; \
    TIM3->CCER &= ~TIM_CCER_CC1E; \
    TIM4->CCER &= ~TIM_CCER_CC1E; \
    /* Disable watchdog timer */ \
    IWDG->KR = 0x5555; \
    IWDG->PR = 0xFF; \
    IWDG->RLR = 0xFFFFFFFF; \
    IWDG->SR = 0x00000000; \
    /* Disable Input Capture Unit (ICU) */ \
    TIM1->CCMR1 &= ~TIM_CCMR1_CC1S; \
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S; \
    TIM3->CCMR1 &= ~TIM_CCMR1_CC1S; \
    TIM4->CCMR1 &= ~TIM_CCMR1_CC1S; \
    /* Disable ADC */ \
    ADC1->CR2 = 0x00000000; \
    ADC2->CR2 = 0x00000000; \
    ADC3->CR2 = 0x00000000; \
    /* Disable UART */ \
    USART1->CR1 &= ~USART_CR1_UE; \
    USART2->CR1 &= ~USART_CR1_UE; \
    USART3->CR1 &= ~USART_CR1_UE; \
    /* Disable I2C */ \
    I2C1->CR1 &= ~I2C_CR1_PE; \
    I2C2->CR1 &= ~I2C_CR1_PE; \
    I2C3->CR1 &= ~I2C_CR1_PE; \
    /* Disable SPI communication */ \
    SPI1->CR1 &= ~SPI_CR1_SPE; \
    SPI2->CR1 &= ~SPI_CR1_SPE; \
    SPIn->CR1 &= ~SPI_CR1_SPE; \
    /* Configure all GPIOs as general purpose I/O pins */ \
    GPIO_SAFEGUARD_Init(); \
    } while(0)

#endif /* MAIN_H */