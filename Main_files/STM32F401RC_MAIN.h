/*****************************************************************************************
 * @file     main.h
 * @brief    Main header file for STM32F401RC embedded projects
 * @author   Technology Innovation Software Team 
 * @device   STM32F401RC
 * @date     2025-07-21
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 ****************************************************************************************/

#include "stm32f4xx.h"

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

typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

#define SET_BIT(reg, bit)   ((reg) |= (1 << (bit)))
#define CLR_BIT(reg, bit)   ((reg) &= ~(1 << (bit)))
#define GET_BIT(reg, bit)   (((reg) >> (bit)) & 1)
#define TOG_BIT(reg, bit)   ((reg) ^= (1 << (bit)))

#define DI()                __disable_irq()
#define EI()                __enable_irq()

/* GPIO SAFEGUARD INIT */
#define GPIO_SAFEGUARD_Init() do { \
    /* Set all GPIO ports to 0 */ \
    GPIOA->ODR = 0; \
    GPIOB->ODR = 0; \
    GPIOC->ODR = 0; \
    GPIOD->ODR = 0; \
    GPIOE->ODR = 0; \
    GPIOF->ODR = 0; \
    GPIOG->ODR = 0; \
    GPIOH->ODR = 0; \
\
    /* Set all GPIO modes to input */ \
    GPIOA->MODER &= ~0x00000000; \
    GPIOB->MODER &= ~0x00000000; \
    GPIOC->MODER &= ~0x00000000; \
    GPIOD->MODER &= ~0x00000000; \
    GPIOE->MODER &= ~0x00000000; \
    GPIOF->MODER &= ~0x00000000; \
    GPIOG->MODER &= ~0x00000000; \
    GPIOH->MODER &= ~0x00000000; \
\
    /* Disable pull-up/pull-down resistors */ \
    GPIOA->PUPDR &= ~0x00000000; \
    GPIOB->PUPDR &= ~0x00000000; \
    GPIOC->PUPDR &= ~0x00000000; \
    GPIOD->PUPDR &= ~0x00000000; \
    GPIOE->PUPDR &= ~0x00000000; \
    GPIOF->PUPDR &= ~0x00000000; \
    GPIOG->PUPDR &= ~0x00000000; \
    GPIOH->PUPDR &= ~0x00000000; \
} while (0)

/* REGISTERS SAFEGUARD INIT */
#define Registers_SAFEGUARD_Init() do { \
    /* Disable global interrupts */ \
    __disable_irq(); \
\
    /* Reset all timers */ \
    TIM1->DIER = 0; \
    TIM2->DIER = 0; \
    TIM3->DIER = 0; \
    TIM4->DIER = 0; \
    TIM5->DIER = 0; \
    TIM6->DIER = 0; \
    TIM7->DIER = 0; \
\
    /* Disable watchdog timers */ \
    WWDG->CFD = 0x00000000; \
    IWDG->KR = 0x5555; \
    IWDG->PR = 0x0000; \
    IWDG->RLR = 0x0000; \
\
    /* Disable ADC */ \
    ADC1->CR2 = 0x00000000; \
    ADC2->CR2 = 0x00000000; \
    ADC3->CR2 = 0x00000000; \
\
    /* Disable UART */ \
    USART1->CR1 &= ~(USART_CR1_UE | USART_CR1_TE); \
    USART2->CR1 &= ~(USART_CR1_UE | USART_CR1_TE); \
    USART3->CR1 &= ~(USART_CR1_UE | USART_CR1_TE); \
\
    /* Disable I2C */ \
    I2C1->CR1 = 0x00000000; \
    I2C2->CR1 = 0x00000000; \
    I2C3->CR1 = 0x00000000; \
\
    /* Disable SPI communication */ \
    SPI1->CR1 &= ~SPI_CR1_SPEIE; \
    SPI2->CR1 &= ~SPI_CR1_SPEIE; \
    SPI3->CR1 &= ~SPI_CR1_SPEIE; \
} while (0)