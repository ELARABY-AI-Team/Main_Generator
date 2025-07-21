/*********************************************************************
* File Name: main.h
* Description: Core header file for STM32F401RC embedded projects
* Author: Technology Innovation Software Team
* Device Name: STM32F401RC
* Creation Date: 2025-07-21
* Standard: MISRA C
* Copyright: ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
*********************************************************************/

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

#define SET_BIT(reg, bit)     ((reg) |= (1 << (bit)))
#define CLR_BIT(reg, bit)     ((reg) &= ~(1 << (bit)))
#define GET_BIT(reg, bit)     ((reg >> (bit)) & 0x01)
#define TOG_BIT(reg, bit)     ((reg) ^= (1 << (bit)))

#define DI()                 __disable_irq()
#define EI()                 __enable_irq()

void GPIO_SAFEGUARD_Init(void);
void Registers_SAFEGUARD_Init(void);

#define GPIO_SAFEGUARD_Init() do { \
    GPIOA->ODR = 0x00; \
    GPIOB->ODR = 0x00; \
    GPIOC->ODR = 0x00; \
    GPIOD->ODR = 0x00; \
    GPIOE->ODR = 0x00; \
    GPIOF->ODR = 0x00; \
    GPIOG->ODR = 0x00; \
    \
    GPIOA->MODER &= 0x00; \
    GPIOB->MODER &= 0x00; \
    GPIOC->MODER &= 0x00; \
    GPIOD->MODER &= 0x00; \
    GPIOE->MODER &= 0x00; \
    GPIOF->MODER &= 0x00; \
    GPIOG->MODER &= 0x00; \
    \
    GPIOA->PUPDR &= 0x00; \
    GPIOB->PUPDR &= 0x00; \
    GPIOC->PUPDR &= 0x00; \
    GPIOD->PUPDR &= 0x00; \
    GPIOE->PUPDR &= 0x00; \
    GPIOF->PUPDR &= 0x00; \
    GPIOG->PUPDR &= 0x00; \
    } while(0)

#define Registers_SAFEGUARD_Init() do { \
    __disable_irq(); \
    \
    TIM1->CR1 = 0x00; \
    TIM2->CR1 = 0x00; \
    TIM3->CR1 = 0x00; \
    TIM4->CR1 = 0x00; \
    TIM5->CR1 = 0x00; \
    TIM6->CR1 = 0x00; \
    TIM7->CR1 = 0x00; \
    TIM8->CR1 = 0x00; \
    \
    IWDG->KR = 0x00; \
    LPTIM1->CFGR &= ~(LPTIM_CFGR_TEN); \
    \
    ADC1->CR2 = 0x00; \
    ADC2->CR2 = 0x00; \
    ADC3->CR2 = 0x00; \
    \
    USART1->CR1 = 0x00; \
    USART2->CR1 = 0x00; \
    USART3->CR1 = 0x00; \
    UART4->CR1 = 0x00; \
    UART5->CR1 = 0x00; \
    \
    I2C1->CR = 0x00; \
    I2C2->CR = 0x00; \
    I2C3->CR = 0x00; \
    \
    SPI1->CR1 = 0x00; \
    SPI2->CR1 = 0x00; \
    SPI3->CR1 = 0x00; \
    } while(0)