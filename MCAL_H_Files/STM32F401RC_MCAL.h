#ifndef MCAL_H
#define MCAL_H

// core_includes: Required include statements as per Rules.json
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// data_type_definitions: Type definitions as per Rules.json
#define Unit_8 uint8_t
#define unit_16 uint16_t
#define unit_32 uint32_t

typedef Unit_8 tbyte;
typedef unit_16 tword;
typedef unit_32 tlong;

// MCU Register Definitions: Direct memory-mapped access using volatile pointers.
// These definitions allow accessing hardware registers by their physical addresses.

// FLASH Registers
#define FLASH_ACR               (*((volatile uint32_t *)0x40023C00)) // Flash access control register.
#define FLASH_KEYR              (*((volatile uint32_t *)0x40023C04)) // Flash key register. Used to unlock the Flash control register.
#define FLASH_OPTKEYR           (*((volatile uint32_t *)0x40023C08)) // Flash option key register. Used to unlock the Flash option control register.
#define FLASH_SR                (*((volatile uint32_t *)0x40023C0C)) // Flash status register.
#define FLASH_CR                (*((volatile uint32_t *)0x40023C10)) // Flash control register.
#define FLASH_OPTCR             (*((volatile uint32_t *)0x40023C14)) // Flash option control register.

// RCC Registers
#define RCC_CR                  (*((volatile uint32_t *)0x40023800)) // RCC clock control register.
#define RCC_PLLCFGR             (*((volatile uint32_t *)0x40023804)) // RCC PLL configuration register.
#define RCC_CFGR                (*((volatile uint32_t *)0x40023808)) // RCC clock configuration register.
#define RCC_CIR                 (*((volatile uint32_t *)0x4002380C)) // RCC clock interrupt register.
#define RCC_AHB1RSTR            (*((volatile uint32_t *)0x40023810)) // RCC AHB1 peripheral reset register.
#define RCC_AHB2RSTR            (*((volatile uint32_t *)0x40023814)) // RCC AHB2 peripheral reset register.
#define RCC_APB1RSTR            (*((volatile uint32_t *)0x40023818)) // RCC APB1 peripheral reset register.
#define RCC_APB2RSTR            (*((volatile uint32_t *)0x4002381C)) // RCC APB2 peripheral reset register.
#define RCC_AHB1ENR             (*((volatile uint32_t *)0x40023830)) // RCC AHB1 peripheral clock enable register.
#define RCC_AHB2ENR             (*((volatile uint32_t *)0x40023834)) // RCC AHB2 peripheral clock enable register.
#define RCC_APB1ENR             (*((volatile uint32_t *)0x40023838)) // RCC APB1 peripheral clock enable register.
#define RCC_APB2ENR             (*((volatile uint32_t *)0x4002383C)) // RCC APB2 peripheral clock enable register.
#define RCC_AHB1LPENR           (*((volatile uint32_t *)0x40023840)) // RCC AHB1 peripheral clock enable in low power mode register.
#define RCC_AHB2LPENR           (*((volatile uint32_t *)0x40023844)) // RCC AHB2 peripheral clock enable in low power mode register.
#define RCC_APB1LPENR           (*((volatile uint32_t *)0x40023848)) // RCC APB1 peripheral clock enable in low power mode register.
#define RCC_APB2LPENR           (*((volatile uint32_t *)0x4002384C)) // RCC APB2 peripheral clock enabled in low power mode register.
#define RCC_BDCR                (*((volatile uint32_t *)0x40023850)) // RCC Backup domain control register.
#define RCC_CSR                 (*((volatile uint32_t *)0x40023854)) // RCC clock control & status register.
#define RCC_SSCGR               (*((volatile uint32_t *)0x40023858)) // RCC spread spectrum clock generation register.
#define RCC_PLLI2SCFGR          (*((volatile uint32_t *)0x4002385C)) // RCC PLLI2S configuration register.
#define RCC_DCKCFGR             (*((volatile uint32_t *)0x40023864)) // RCC Dedicated Clocks Configuration Register.

// SYSCFG Registers
#define SYSCFG_MEMRMP           (*((volatile uint32_t *)0x40013800)) // SYSCFG memory remap register.
#define SYSCFG_PMC              (*((volatile uint32_t *)0x40013804)) // SYSCFG peripheral mode configuration register.
#define SYSCFG_EXTICR1          (*((volatile uint32_t *)0x40013808)) // SYSCFG external interrupt configuration register 1 (EXTI0-EXTI3 source selection).
#define SYSCFG_EXTICR2          (*((volatile uint32_t *)0x4001380C)) // SYSCFG external interrupt configuration register 2 (EXTI4-EXTI7 source selection).
#define SYSCFG_EXTICR3          (*((volatile uint32_t *)0x40013810)) // SYSCFG external interrupt configuration register 3 (EXTI8-EXTI11 source selection).
#define SYSCFG_EXTICR4          (*((volatile uint32_t *)0x40013814)) // SYSCFG external interrupt configuration register 4 (EXTI12-EXTI15 source selection).
#define SYSCFG_CMPCR            (*((volatile uint32_t *)0x40013820)) // Compensation cell control register.

// GPIO Registers (Port A)
#define GPIOA_MODER             (*((volatile uint32_t *)0x40020000)) // GPIO port mode register for Port A.
#define GPIOA_OTYPER            (*((volatile uint32_t *)0x40020004)) // GPIO port output type register for Port A.
#define GPIOA_OSPEEDR           (*((volatile uint32_t *)0x40020008)) // GPIO port output speed register for Port A.
#define GPIOA_PUPDR             (*((volatile uint32_t *)0x4002000C)) // GPIO port pull-up/pull-down register for Port A.
#define GPIOA_IDR               (*((volatile uint32_t *)0x40020010)) // GPIO port input data register for Port A.
#define GPIOA_ODR               (*((volatile uint32_t *)0x40020014)) // GPIO port output data register for Port A.
#define GPIOA_BSRR              (*((volatile uint32_t *)0x40020018)) // GPIO port bit set/reset register for Port A.
#define GPIOA_LCKR              (*((volatile uint32_t *)0x4002001C)) // GPIO port configuration lock register for Port A.
#define GPIOA_AFRL              (*((volatile uint32_t *)0x40020020)) // GPIO alternate function low register for Port A.
#define GPIOA_AFRH              (*((volatile uint32_t *)0x40020024)) // GPIO alternate function high register for Port A.

// GPIO Registers (Port B)
#define GPIOB_MODER             (*((volatile uint32_t *)0x40020400)) // GPIO port mode register for Port B.
#define GPIOB_OTYPER            (*((volatile uint32_t *)0x40020404)) // GPIO port output type register for Port B.
#define GPIOB_OSPEEDR           (*((volatile uint32_t *)0x40020408)) // GPIO port output speed register for Port B.
#define GPIOB_PUPDR             (*((volatile uint32_t *)0x4002040C)) // GPIO port pull-up/pull-down register for Port B.
#define GPIOB_IDR               (*((volatile uint32_t *)0x40020410)) // GPIO port input data register for Port B.
#define GPIOB_ODR               (*((volatile uint32_t *)0x40020414)) // GPIO port output data register for Port B.
#define GPIOB_BSRR              (*((volatile uint32_t *)0x40020418)) // GPIO port bit set/reset register for Port B.
#define GPIOB_LCKR              (*((volatile uint32_t *)0x4002041C)) // GPIO port configuration lock register for Port B.
#define GPIOB_AFRL              (*((volatile uint32_t *)0x40020420)) // GPIO alternate function low register for Port B.
#define GPIOB_AFRH              (*((volatile uint32_t *)0x40020424)) // GPIO alternate function high register for Port B.

// GPIO Registers (Port C)
#define GPIOC_MODER             (*((volatile uint32_t *)0x40020800)) // GPIO port mode register for Port C.
#define GPIOC_OTYPER            (*((volatile uint32_t *)0x40020804)) // GPIO port output type register for Port C.
#define GPIOC_OSPEEDR           (*((volatile uint32_t *)0x40020808)) // GPIO port output speed register for Port C.
#define GPIOC_PUPDR             (*((volatile uint32_t *)0x4002080C)) // GPIO port pull-up/pull-down register for Port C.
#define GPIOC_IDR               (*((volatile uint32_t *)0x40020810)) // GPIO port input data register for Port C.
#define GPIOC_ODR               (*((volatile uint32_t *)0x40020814)) // GPIO port output data register for Port C.
#define GPIOC_BSRR              (*((volatile uint32_t *)0x40020818)) // GPIO port bit set/reset register for Port C.
#define GPIOC_LCKR              (*((volatile uint32_t *)0x4002081C)) // GPIO port configuration lock register for Port C.
#define GPIOC_AFRL              (*((volatile uint32_t *)0x40020820)) // GPIO alternate function low register for Port C.
#define GPIOC_AFRH              (*((volatile uint32_t *)0x40020824)) // GPIO alternate function high register for Port C.

// GPIO Registers (Port D)
#define GPIOD_MODER             (*((volatile uint32_t *)0x40020C00)) // GPIO port mode register for Port D.
#define GPIOD_OTYPER            (*((volatile uint32_t *)0x40020C04)) // GPIO port output type register for Port D.
#define GPIOD_OSPEEDR           (*((volatile uint32_t *)0x40020C08)) // GPIO port output speed register for Port D.
#define GPIOD_PUPDR             (*((volatile uint32_t *)0x40020C0C)) // GPIO port pull-up/pull-down register for Port D.
#define GPIOD_IDR               (*((volatile uint32_t *)0x40020C10)) // GPIO port input data register for Port D.
#define GPIOD_ODR               (*((volatile uint32_t *)0x40020C14)) // GPIO port output data register for Port D.
#define GPIOD_BSRR              (*((volatile uint32_t *)0x40020C18)) // GPIO port bit set/reset register for Port D.
#define GPIOD_LCKR              (*((volatile uint32_t *)0x40020C1C)) // GPIO port configuration lock register for Port D.
#define GPIOD_AFRL              (*((volatile uint32_t *)0x40020C20)) // GPIO alternate function low register for Port D.
#define GPIOD_AFRH              (*((volatile uint32_t *)0x40020C24)) // GPIO alternate function high register for Port D.

// GPIO Registers (Port E)
#define GPIOE_MODER             (*((volatile uint32_t *)0x40021000)) // GPIO port mode register for Port E.
#define GPIOE_OTYPER            (*((volatile uint32_t *)0x40021004)) // GPIO port output type register for Port E.
#define GPIOE_OSPEEDR           (*((volatile uint32_t *)0x40021008)) // GPIO port output speed register for Port E.
#define GPIOE_PUPDR             (*((volatile uint32_t *)0x4002100C)) // GPIO port pull-up/pull-down register for Port E.
#define GPIOE_IDR               (*((volatile uint32_t *)0x40021010)) // GPIO port input data register for Port E.
#define GPIOE_ODR               (*((volatile uint32_t *)0x40021014)) // GPIO port output data register for Port E.
#define GPIOE_BSRR              (*((volatile uint32_t *)0x40021018)) // GPIO port bit set/reset register for Port E.
#define GPIOE_LCKR              (*((volatile uint32_t *)0x4002101C)) // GPIO port configuration lock register for Port E.
#define GPIOE_AFRL              (*((volatile uint32_t *)0x40021020)) // GPIO alternate function low register for Port E.
#define GPIOE_AFRH              (*((volatile uint32_t *)0x40021024)) // GPIO alternate function high register for Port E.

// GPIO Registers (Port H)
#define GPIOH_MODER             (*((volatile uint32_t *)0x40021C00)) // GPIO port mode register for Port H.
#define GPIOH_OTYPER            (*((volatile uint32_t *)0x40021C04)) // GPIO port output type register for Port H.
#define GPIOH_OSPEEDR           (*((volatile uint32_t *)0x40021C08)) // GPIO port output speed register for Port H.
#define GPIOH_PUPDR             (*((volatile uint32_t *)0x40021C0C)) // GPIO port pull-up/pull-down register for Port H.
#define GPIOH_IDR               (*((volatile uint32_t *)0x40021C10)) // GPIO port input data register for Port H.
#define GPIOH_ODR               (*((volatile uint32_t *)0x40021C14)) // GPIO port output data register for Port H.
#define GPIOH_BSRR              (*((volatile uint32_t *)0x40021C18)) // GPIO port bit set/reset register for Port H.
#define GPIOH_LCKR              (*((volatile uint32_t *)0x40021C1C)) // GPIO port configuration lock register for Port H.
#define GPIOH_AFRL              (*((volatile uint32_t *)0x40021C20)) // GPIO alternate function low register for Port H.
#define GPIOH_AFRH              (*((volatile uint32_t *)0x40021C24)) // GPIO alternate function high register for Port H.

// EXTI Registers
#define EXTI_IMR                (*((volatile uint32_t *)0x40013C00)) // Interrupt mask register.
#define EXTI_EMR                (*((volatile uint32_t *)0x40013C04)) // Event mask register.
#define EXTI_RTSR               (*((volatile uint32_t *)0x40013C08)) // Rising trigger selection register.
#define EXTI_FTSR               (*((volatile uint32_t *)0x40013C0C)) // Falling trigger selection register.
#define EXTI_SWIER              (*((volatile uint32_t *)0x40013C10)) // Software interrupt event register.
#define EXTI_PR                 (*((volatile uint32_t *)0x40013C14)) // Pending register.

// ADC Registers
#define ADC1_SR                 (*((volatile uint32_t *)0x40012000)) // ADC status register.
#define ADC1_CR1                (*((volatile uint32_t *)0x40012004)) // ADC control register 1.
#define ADC1_CR2                (*((volatile uint32_t *)0x40012008)) // ADC control register 2.
#define ADC1_SMPR1              (*((volatile uint32_t *)0x4001200C)) // ADC sample time register 1.
#define ADC1_SMPR2              (*((volatile uint32_t *)0x40012010)) // ADC sample time register 2.
#define ADC1_JOFR1              (*((volatile uint32_t *)0x40012014)) // ADC injected channel data offset register 1.
#define ADC1_JOFR2              (*((volatile uint32_t *)0x40012018)) // ADC injected channel data offset register 2.
#define ADC1_JOFR3              (*((volatile uint32_t *)0x4001201C)) // ADC injected channel data offset register 3.
#define ADC1_JOFR4              (*((volatile uint32_t *)0x40012020)) // ADC injected channel data offset register 4.
#define ADC1_HTR                (*((volatile uint32_t *)0x40012024)) // ADC watchdog higher threshold register.
#define ADC1_LTR                (*((volatile uint32_t *)0x40012028)) // ADC watchdog lower threshold register.
#define ADC1_SQR1               (*((volatile uint32_t *)0x4001202C)) // ADC regular sequence register 1.
#define ADC1_SQR2               (*((volatile uint32_t *)0x40012030)) // ADC regular sequence register 2.
#define ADC1_SQR3               (*((volatile uint32_t *)0x40012034)) // ADC regular sequence register 3.
#define ADC1_JSQR               (*((volatile uint32_t *)0x40012038)) // ADC injected sequence register.
#define ADC1_JDR1               (*((volatile uint32_t *)0x4001203C)) // ADC injected data register 1.
#define ADC1_JDR2               (*((volatile uint32_t *)0x40012040)) // ADC injected data register 2.
#define ADC1_JDR3               (*((volatile uint32_t *)0x40012044)) // ADC injected data register 3.
#define ADC1_JDR4               (*((volatile uint32_t *)0x40012048)) // ADC injected data register 4.
#define ADC1_DR                 (*((volatile uint32_t *)0x4001204C)) // ADC regular data register.
#define ADC_CCR                 (*((volatile uint32_t *)0x40012304)) // ADC common control register.

// TIM1 Registers
#define TIM1_CR1                (*((volatile uint32_t *)0x40010000)) // TIM1 control register 1.
#define TIM1_CR2                (*((volatile uint32_t *)0x40010004)) // TIM1 control register 2.
#define TIM1_SMCR               (*((volatile uint32_t *)0x40010008)) // TIM1 slave mode control register.
#define TIM1_DIER               (*((volatile uint32_t *)0x4001000C)) // TIM1 DMA/interrupt enable register.
#define TIM1_SR                 (*((volatile uint32_t *)0x40010010)) // TIM1 status register.
#define TIM1_EGR                (*((volatile uint32_t *)0x40010014)) // TIM1 event generation register.
#define TIM1_CCMR1              (*((volatile uint32_t *)0x40010018)) // TIM1 capture/compare mode register 1.
#define TIM1_CCMR2              (*((volatile uint32_t *)0x4001001C)) // TIM1 capture/compare mode register 2.
#define TIM1_CCER               (*((volatile uint32_t *)0x40010020)) // TIM1 capture/compare enable register.
#define TIM1_CNT                (*((volatile uint32_t *)0x40010024)) // TIM1 counter register.
#define TIM1_PSC                (*((volatile uint32_t *)0x40010028)) // TIM1 prescaler register.
#define TIM1_ARR                (*((volatile uint32_t *)0x4001002C)) // TIM1 auto-reload register.
#define TIM1_RCR                (*((volatile uint32_t *)0x40010030)) // TIM1 repetition counter register.
#define TIM1_CCR1               (*((volatile uint32_t *)0x40010034)) // TIM1 capture/compare register 1.
#define TIM1_CCR2               (*((volatile uint32_t *)0x40010038)) // TIM1 capture/compare register 2.
#define TIM1_CCR3               (*((volatile uint32_t *)0x4001003C)) // TIM1 capture/compare register 3.
#define TIM1_CCR4               (*((volatile uint32_t *)0x40010040)) // TIM1 capture/compare register 4.
#define TIM1_BDTR               (*((volatile uint32_t *)0x40010044)) // TIM1 break and dead-time register.
#define TIM1_DCR                (*((volatile uint32_t *)0x40010048)) // TIM1 DMA control register.
#define TIM1_DMAR               (*((volatile uint32_t *)0x4001004C)) // TIM1 DMA address for full transfer.

// TIM2 Registers
#define TIM2_CR1                (*((volatile uint32_t *)0x40000000)) // TIM2 control register 1.
#define TIM2_CR2                (*((volatile uint32_t *)0x40000004)) // TIM2 control register 2.
#define TIM2_SMCR               (*((volatile uint32_t *)0x40000008)) // TIM2 slave mode control register.
#define TIM2_DIER               (*((volatile uint32_t *)0x4000000C)) // TIM2 DMA/Interrupt enable register.
#define TIM2_SR                 (*((volatile uint32_t *)0x40000010)) // TIM2 status register.
#define TIM2_EGR                (*((volatile uint32_t *)0x40000014)) // TIM2 event generation register.
#define TIM2_CCMR1              (*((volatile uint32_t *)0x40000018)) // TIM2 capture/compare mode register 1.
#define TIM2_CCMR2              (*((volatile uint32_t *)0x4000001C)) // TIM2 capture/compare mode register 2.
#define TIM2_CCER               (*((volatile uint32_t *)0x40000020)) // TIM2 capture/compare enable register.
#define TIM2_CNT                (*((volatile uint32_t *)0x40000024)) // TIM2 counter register.
#define TIM2_PSC                (*((volatile uint32_t *)0x40000028)) // TIM2 prescaler register.
#define TIM2_ARR                (*((volatile uint32_t *)0x4000002C)) // TIM2 auto-reload register.
#define TIM2_CCR1               (*((volatile uint32_t *)0x40000034)) // TIM2 capture/compare register 1.
#define TIM2_CCR2               (*((volatile uint32_t *)0x40000038)) // TIM2 capture/compare register 2.
#define TIM2_CCR3               (*((volatile uint32_t *)0x4000003C)) // TIM2 capture/compare register 3.
#define TIM2_CCR4               (*((volatile uint32_t *)0x40000040)) // TIM2 capture/compare register 4.
#define TIM2_DCR                (*((volatile uint32_t *)0x40000048)) // TIM2 DMA control register.
#define TIM2_DMAR               (*((volatile uint32_t *)0x4000004C)) // TIM2 DMA address for full transfer.
#define TIM2_OR                 (*((volatile uint32_t *)0x40000050)) // TIM2 option register.

// TIM3 Registers
#define TIM3_CR1                (*((volatile uint32_t *)0x40000400)) // TIM3 control register 1.
#define TIM3_CR2                (*((volatile uint32_t *)0x40000404)) // TIM3 control register 2.
#define TIM3_SMCR               (*((volatile uint32_t *)0x40000408)) // TIM3 slave mode control register.
#define TIM3_DIER               (*((volatile uint32_t *)0x4000040C)) // TIM3 DMA/Interrupt enable register.
#define TIM3_SR                 (*((volatile uint32_t *)0x40000410)) // TIM3 status register.
#define TIM3_EGR                (*((volatile uint32_t *)0x40000414)) // TIM3 event generation register.
#define TIM3_CCMR1              (*((volatile uint32_t *)0x40000418)) // TIM3 capture/compare mode register 1.
#define TIM3_CCMR2              (*((volatile uint32_t *)0x4000041C)) // TIM3 capture/compare mode register 2.
#define TIM3_CCER               (*((volatile uint32_t *)0x40000420)) // TIM3 capture/compare enable register.
#define TIM3_CNT                (*((volatile uint32_t *)0x40000424)) // TIM3 counter register.
#define TIM3_PSC                (*((volatile uint32_t *)0x40000428)) // TIM3 prescaler register.
#define TIM3_ARR                (*((volatile uint32_t *)0x4000042C)) // TIM3 auto-reload register.
#define TIM3_CCR1               (*((volatile uint32_t *)0x40000434)) // TIM3 capture/compare register 1.
#define TIM3_CCR2               (*((volatile uint32_t *)0x40000438)) // TIM3 capture/compare register 2.
#define TIM3_CCR3               (*((volatile uint32_t *)0x4000043C)) // TIM3 capture/compare register 3.
#define TIM3_CCR4               (*((volatile uint32_t *)0x40000440)) // TIM3 capture/compare register 4.
#define TIM3_DCR                (*((volatile uint32_t *)0x40000448)) // TIM3 DMA control register.
#define TIM3_DMAR               (*((volatile uint32_t *)0x4000044C)) // TIM3 DMA address for full transfer.

// TIM4 Registers
#define TIM4_CR1                (*((volatile uint32_t *)0x40000800)) // TIM4 control register 1.
#define TIM4_CR2                (*((volatile uint32_t *)0x40000804)) // TIM4 control register 2.
#define TIM4_SMCR               (*((volatile uint32_t *)0x40000808)) // TIM4 slave mode control register.
#define TIM4_DIER               (*((volatile uint32_t *)0x4000080C)) // TIM4 DMA/Interrupt enable register.
#define TIM4_SR                 (*((volatile uint32_t *)0x40000810)) // TIM4 status register.
#define TIM4_EGR                (*((volatile uint32_t *)0x40000814)) // TIM4 event generation register.
#define TIM4_CCMR1              (*((volatile uint32_t *)0x40000818)) // TIM4 capture/compare mode register 1.
#define TIM4_CCMR2              (*((volatile uint32_t *)0x4000081C)) // TIM4 capture/compare mode register 2.
#define TIM4_CCER               (*((volatile uint32_t *)0x40000820)) // TIM4 capture/compare enable register.
#define TIM4_CNT                (*((volatile uint32_t *)0x40000824)) // TIM4 counter register.
#define TIM4_PSC                (*((volatile uint32_t *)0x40000828)) // TIM4 prescaler register.
#define TIM4_ARR                (*((volatile uint32_t *)0x4000082C)) // TIM4 auto-reload register.
#define TIM4_CCR1               (*((volatile uint32_t *)0x40000834)) // TIM4 capture/compare register 1.
#define TIM4_CCR2               (*((volatile uint32_t *)0x40000838)) // TIM4 capture/compare register 2.
#define TIM4_CCR3               (*((volatile uint32_t *)0x4000083C)) // TIM4 capture/compare register 3.
#define TIM4_CCR4               (*((volatile uint32_t *)0x40000840)) // TIM4 capture/compare register 4.
#define TIM4_DCR                (*((volatile uint32_t *)0x40000848)) // TIM4 DMA control register.
#define TIM4_DMAR               (*((volatile uint32_t *)0x4000084C)) // TIM4 DMA address for full transfer.

// TIM5 Registers
#define TIM5_CR1                (*((volatile uint32_t *)0x40000C00)) // TIM5 control register 1.
#define TIM5_CR2                (*((volatile uint32_t *)0x40000C04)) // TIM5 control register 2.
#define TIM5_SMCR               (*((volatile uint32_t *)0x40000C08)) // TIM5 slave mode control register.
#define TIM5_DIER               (*((volatile uint32_t *)0x40000C0C)) // TIM5 DMA/Interrupt enable register.
#define TIM5_SR                 (*((volatile uint32_t *)0x40000C10)) // TIM5 status register.
#define TIM5_EGR                (*((volatile uint32_t *)0x40000C14)) // TIM5 event generation register.
#define TIM5_CCMR1              (*((volatile uint32_t *)0x40000C18)) // TIM5 capture/compare mode register 1.
#define TIM5_CCMR2              (*((volatile uint32_t *)0x40000C1C)) // TIM5 capture/compare mode register 2.
#define TIM5_CCER               (*((volatile uint32_t *)0x40000C20)) // TIM5 capture/compare enable register.
#define TIM5_CNT                (*((volatile uint32_t *)0x40000C24)) // TIM5 counter register.
#define TIM5_PSC                (*((volatile uint32_t *)0x40000C28)) // TIM5 prescaler register.
#define TIM5_ARR                (*((volatile uint32_t *)0x40000C2C)) // TIM5 auto-reload register.
#define TIM5_CCR1               (*((volatile uint32_t *)0x40000C34)) // TIM5 capture/compare register 1.
#define TIM5_CCR2               (*((volatile uint32_t *)0x40000C38)) // TIM5 capture/compare register 2.
#define TIM5_CCR3               (*((volatile uint32_t *)0x40000C3C)) // TIM5 capture/compare register 3.
#define TIM5_CCR4               (*((volatile uint32_t *)0x40000C40)) // TIM5 capture/compare register 4.
#define TIM5_DCR                (*((volatile uint32_t *)0x40000C48)) // TIM5 DMA control register.
#define TIM5_DMAR               (*((volatile uint32_t *)0x40000C4C)) // TIM5 DMA address for full transfer.
#define TIM5_OR                 (*((volatile uint32_t *)0x40000C54)) // TIM5 option register.

// TIM9 Registers
#define TIM9_CR1                (*((volatile uint32_t *)0x40014000)) // TIM9 control register 1.
#define TIM9_SMCR               (*((volatile uint32_t *)0x40014008)) // TIM9 slave mode control register.
#define TIM9_DIER               (*((volatile uint32_t *)0x4001400C)) // TIM9 Interrupt enable register.
#define TIM9_SR                 (*((volatile uint32_t *)0x40014010)) // TIM9 status register.
#define TIM9_EGR                (*((volatile uint32_t *)0x40014014)) // TIM9 event generation register.
#define TIM9_CCMR1              (*((volatile uint32_t *)0x40014018)) // TIM9 capture/compare mode register 1.
#define TIM9_CCER               (*((volatile uint32_t *)0x40014020)) // TIM9 capture/compare enable register.
#define TIM9_CNT                (*((volatile uint32_t *)0x40014024)) // TIM9 counter register.
#define TIM9_PSC                (*((volatile uint32_t *)0x40014028)) // TIM9 prescaler register.
#define TIM9_ARR                (*((volatile uint32_t *)0x4001402C)) // TIM9 auto-reload register.
#define TIM9_CCR1               (*((volatile uint32_t *)0x40014034)) // TIM9 capture/compare register 1.
#define TIM9_CCR2               (*((volatile uint32_t *)0x40014038)) // TIM9 capture/compare register 2.

// TIM10 Registers
#define TIM10_CR1               (*((volatile uint32_t *)0x40014400)) // TIM10 control register 1.
#define TIM10_DIER              (*((volatile uint32_t *)0x4001440C)) // TIM10 Interrupt enable register.
#define TIM10_SR                (*((volatile uint32_t *)0x40014410)) // TIM10 status register.
#define TIM10_EGR               (*((volatile uint32_t *)0x40014414)) // TIM10 event generation register.
#define TIM10_CCMR1             (*((volatile uint32_t *)0x40014418)) // TIM10 capture/compare mode register 1.
#define TIM10_CCER              (*((volatile uint32_t *)0x40014420)) // TIM10 capture/compare enable register.
#define TIM10_CNT               (*((volatile uint32_t *)0x40014424)) // TIM10 counter register.
#define TIM10_PSC               (*((volatile uint32_t *)0x40014428)) // TIM10 prescaler register.
#define TIM10_ARR               (*((volatile uint32_t *)0x4001442C)) // TIM10 auto-reload register.
#define TIM10_CCR1              (*((volatile uint32_t *)0x40014434)) // TIM10 capture/compare register 1.

// TIM11 Registers
#define TIM11_CR1               (*((volatile uint32_t *)0x40014800)) // TIM11 control register 1.
#define TIM11_DIER              (*((volatile uint32_t *)0x4001480C)) // TIM11 Interrupt enable register.
#define TIM11_SR                (*((volatile uint32_t *)0x40014810)) // TIM11 status register.
#define TIM11_EGR               (*((volatile uint32_t *)0x40014814)) // TIM11 event generation register.
#define TIM11_CCMR1             (*((volatile uint32_t *)0x40014818)) // TIM11 capture/compare mode register 1.
#define TIM11_CCER              (*((volatile uint32_t *)0x40014820)) // TIM11 capture/compare enable register.
#define TIM11_CNT               (*((volatile uint32_t *)0x40014824)) // TIM11 counter register.
#define TIM11_PSC               (*((volatile uint32_t *)0x40014828)) // TIM11 prescaler register.
#define TIM11_ARR               (*((volatile uint32_t *)0x4001482C)) // TIM11 auto-reload register.
#define TIM11_CCR1              (*((volatile uint32_t *)0x40014834)) // TIM11 capture/compare register 1.

// USART Registers
#define USART1_SR               (*((volatile uint32_t *)0x40011000)) // USART1 Status register.
#define USART1_DR               (*((volatile uint32_t *)0x40011004)) // USART1 Data register.
#define USART1_BRR              (*((volatile uint32_t *)0x40011008)) // USART1 Baud rate register.
#define USART1_CR1              (*((volatile uint32_t *)0x4001100C)) // USART1 Control register 1.
#define USART1_CR2              (*((volatile uint32_t *)0x40011010)) // USART1 Control register 2.
#define USART1_CR3              (*((volatile uint32_t *)0x40011014)) // USART1 Control register 3.
#define USART1_GTPR             (*((volatile uint32_t *)0x40011018)) // USART1 Guard time and prescaler register.

#define USART2_SR               (*((volatile uint32_t *)0x40004400)) // USART2 Status register.
#define USART2_DR               (*((volatile uint32_t *)0x40004404)) // USART2 Data register.
#define USART2_BRR              (*((volatile uint32_t *)0x40004408)) // USART2 Baud rate register.
#define USART2_CR1              (*((volatile uint32_t *)0x4000440C)) // USART2 Control register 1.
#define USART2_CR2              (*((volatile uint32_t *)0x40004410)) // USART2 Control register 2.
#define USART2_CR3              (*((volatile uint32_t *)0x40004414)) // USART2 Control register 3.
#define USART2_GTPR             (*((volatile uint32_t *)0x40004418)) // USART2 Guard time and prescaler register.

#define USART6_SR               (*((volatile uint32_t *)0x40011400)) // USART6 Status register.
#define USART6_DR               (*((volatile uint32_t *)0x40011404)) // USART6 Data register.
#define USART6_BRR              (*((volatile uint32_t *)0x40011408)) // USART6 Baud rate register.
#define USART6_CR1              (*((volatile uint32_t *)0x4001140C)) // USART6 Control register 1.
#define USART6_CR2              (*((volatile uint32_t *)0x40011410)) // USART6 Control register 2.
#define USART6_CR3              (*((volatile uint32_t *)0x40011414)) // USART6 Control register 3.
#define USART6_GTPR             (*((volatile uint32_t *)0x40011418)) // USART6 Guard time and prescaler register.

// I2C Registers
#define I2C1_CR1                (*((volatile uint32_t *)0x40005400)) // I2C1 Control register 1.
#define I2C1_CR2                (*((volatile uint32_t *)0x40005404)) // I2C1 Control register 2.
#define I2C1_OAR1               (*((volatile uint32_t *)0x40005408)) // I2C1 Own address register 1.
#define I2C1_OAR2               (*((volatile uint32_t *)0x4000540C)) // I2C1 Own address register 2.
#define I2C1_DR                 (*((volatile uint32_t *)0x40005410)) // I2C1 Data register.
#define I2C1_SR1                (*((volatile uint32_t *)0x40005414)) // I2C1 Status register 1.
#define I2C1_SR2                (*((volatile uint32_t *)0x40005418)) // I2C1 Status register 2.
#define I2C1_CCR                (*((volatile uint32_t *)0x4000541C)) // I2C1 Clock control register.
#define I2C1_TRISE              (*((volatile uint32_t *)0x40005420)) // I2C1 TRISE register.
#define I2C1_FLTR               (*((volatile uint32_t *)0x40005424)) // I2C1 Filter register.

#define I2C2_CR1                (*((volatile uint32_t *)0x40005800)) // I2C2 Control register 1.
#define I2C2_CR2                (*((volatile uint32_t *)0x40005804)) // I2C2 Control register 2.
#define I2C2_OAR1               (*((volatile uint32_t *)0x40005808)) // I2C2 Own address register 1.
#define I2C2_OAR2               (*((volatile uint32_t *)0x4000580C)) // I2C2 Own address register 2.
#define I2C2_DR                 (*((volatile uint32_t *)0x40005810)) // I2C2 Data register.
#define I2C2_SR1                (*((volatile uint32_t *)0x40005814)) // I2C2 Status register 1.
#define I2C2_SR2                (*((volatile uint32_t *)0x40005818)) // I2C2 Status register 2.
#define I2C2_CCR                (*((volatile uint32_t *)0x4000581C)) // I2C2 Clock control register.
#define I2C2_TRISE              (*((volatile uint32_t *)0x40005820)) // I2C2 TRISE register.
#define I2C2_FLTR               (*((volatile uint32_t *)0x40005824)) // I2C2 Filter register.

#define I2C3_CR1                (*((volatile uint32_t *)0x40005C00)) // I2C3 Control register 1.
#define I2C3_CR2                (*((volatile uint32_t *)0x40005C04)) // I2C3 Control register 2.
#define I2C3_OAR1               (*((volatile uint32_t *)0x40005C08)) // I2C3 Own address register 1.
#define I2C3_OAR2               (*((volatile uint32_t *)0x40005C0C)) // I2C3 Own address register 2.
#define I2C3_DR                 (*((volatile uint32_t *)0x40005C10)) // I2C3 Data register.
#define I2C3_SR1                (*((volatile uint32_t *)0x40005C14)) // I2C3 Status register 1.
#define I2C3_SR2                (*((volatile uint32_t *)0x40005C18)) // I2C3 Status register 2.
#define I2C3_CCR                (*((volatile uint32_t *)0x40005C1C)) // I2C3 Clock control register.
#define I2C3_TRISE              (*((volatile uint32_t *)0x40005C20)) // I2C3 TRISE register.
#define I2C3_FLTR               (*((volatile uint32_t *)0x40005C24)) // I2C3 Filter register.

// SPI Registers
#define SPI1_CR1                (*((volatile uint32_t *)0x40013000)) // SPI1 Control register 1.
#define SPI1_CR2                (*((volatile uint32_t *)0x40013004)) // SPI1 Control register 2.
#define SPI1_SR                 (*((volatile uint32_t *)0x40013008)) // SPI1 Status register.
#define SPI1_DR                 (*((volatile uint32_t *)0x4001300C)) // SPI1 Data register.
#define SPI1_CRCPR              (*((volatile uint32_t *)0x40013010)) // SPI1 CRC polynomial register.
#define SPI1_RXCRCR             (*((volatile uint32_t *)0x40013014)) // SPI1 Rx CRC register.
#define SPI1_TXCRCR             (*((volatile uint32_t *)0x40013018)) // SPI1 Tx CRC register.
#define SPI1_I2SCFGR            (*((volatile uint32_t *)0x4001301C)) // SPI1 I2S configuration register.
#define SPI1_I2SPR              (*((volatile uint32_t *)0x40013020)) // SPI1 I2S prescaler register.

#define SPI2_CR1                (*((volatile uint32_t *)0x40003800)) // SPI2 Control register 1.
#define SPI2_CR2                (*((volatile uint32_t *)0x40003804)) // SPI2 Control register 2.
#define SPI2_SR                 (*((volatile uint32_t *)0x40003808)) // SPI2 Status register.
#define SPI2_DR                 (*((volatile uint32_t *)0x4000380C)) // SPI2 Data register.
#define SPI2_CRCPR              (*((volatile uint32_t *)0x40003810)) // SPI2 CRC polynomial register.
#define SPI2_RXCRCR             (*((volatile uint32_t *)0x40003814)) // SPI2 Rx CRC register.
#define SPI2_TXCRCR             (*((volatile uint32_t *)0x40003818)) // SPI2 Tx CRC register.
#define SPI2_I2SCFGR            (*((volatile uint32_t *)0x4000381C)) // SPI2 I2S configuration register.
#define SPI2_I2SPR              (*((volatile uint32_t *)0x40003820)) // SPI2 I2S prescaler register.

#define SPI3_CR1                (*((volatile uint32_t *)0x40003C00)) // SPI3 Control register 1.
#define SPI3_CR2                (*((volatile uint32_t *)0x40003C04)) // SPI3 Control register 2.
#define SPI3_SR                 (*((volatile uint32_t *)0x40003C08)) // SPI3 Status register.
#define SPI3_DR                 (*((volatile uint32_t *)0x40003C0C)) // SPI3 Data register.
#define SPI3_CRCPR              (*((volatile uint32_t *)0x40003C10)) // SPI3 CRC polynomial register.
#define SPI3_RXCRCR             (*((volatile uint32_t *)0x40003C14)) // SPI3 Rx CRC register.
#define SPI3_TXCRCR             (*((volatile uint32_t *)0x40003C18)) // SPI3 Tx CRC register.
#define SPI3_I2SCFGR            (*((volatile uint32_t *)0x40003C1C)) // SPI3 I2S configuration register.
#define SPI3_I2SPR              (*((volatile uint32_t *)0x40003C20)) // SPI3 I2S prescaler register.


// Typedefs for MCAL Layer based on API.json and Rules.json
// Enums follow LOWERCASE_WITH_UNDERSCORE style and are derived from register_json or common MCU practice.

// MCU CONFIG
typedef enum
{
  SYS_VOLT_3V,
  SYS_VOLT_5V
} t_sys_volt;

// LVD (Low Voltage Detection)
typedef enum
{
  LVD_THRESHOLD_0_5V,
  LVD_THRESHOLD_1V,
  LVD_THRESHOLD_1_5V,
  LVD_THRESHOLD_2V,
  LVD_THRESHOLD_2_5V,
  LVD_THRESHOLD_3V,
  LVD_THRESHOLD_3_5V,
  LVD_THRESHOLD_4V,
  LVD_THRESHOLD_4_5V,
  LVD_THRESHOLD_5V
} t_lvd_thrthresholdLevel;

// UART
typedef enum
{
  UART_CHANNEL_1, // Corresponds to USART1
  UART_CHANNEL_2, // Corresponds to USART2
  UART_CHANNEL_6  // Corresponds to USART6
} t_uart_channel;

typedef enum
{
  UART_BAUD_RATE_2400,
  UART_BAUD_RATE_4800,
  UART_BAUD_RATE_9600,
  UART_BAUD_RATE_19200,
  UART_BAUD_RATE_38400,
  UART_BAUD_RATE_57600,
  UART_BAUD_RATE_115200,
  UART_BAUD_RATE_230400,
  UART_BAUD_RATE_460800,
  UART_BAUD_RATE_921600
} t_uart_baud_rate;

typedef enum
{
  UART_DATA_LENGTH_8_BITS,
  UART_DATA_LENGTH_9_BITS
} t_uart_data_length;

typedef enum
{
  UART_STOP_BIT_1,
  UART_STOP_BIT_0_5,
  UART_STOP_BIT_2,
  UART_STOP_BIT_1_5
} t_uart_stop_bit;

typedef enum
{
  UART_PARITY_NONE,
  UART_PARITY_EVEN,
  UART_PARITY_ODD
} t_uart_parity;

// I2C
typedef enum
{
  I2C_CHANNEL_1, // Corresponds to I2C1
  I2C_CHANNEL_2, // Corresponds to I2C2
  I2C_CHANNEL_3  // Corresponds to I2C3
} t_i2c_channel;

typedef enum
{
  I2C_CLK_SPEED_STANDARD, // 100 kHz
  I2C_CLK_SPEED_FAST      // 400 kHz (fast mode is required by rules.json)
} t_i2c_clk_speed;

typedef enum
{
  I2C_DEVICE_ADDRESS_7BIT,
  I2C_DEVICE_ADDRESS_10BIT
} t_i2c_device_address;

typedef enum
{
  I2C_ACK_DISABLE,
  I2C_ACK_ENABLE
} t_i2c_ack;

typedef enum
{
  I2C_DATA_LENGTH_8_BITS // I2C data transfers are typically byte-oriented
} t_i2c_datalength;

// SPI (CSI)
typedef enum
{
  SPI_CHANNEL_1, // Corresponds to SPI1
  SPI_CHANNEL_2, // Corresponds to SPI2
  SPI_CHANNEL_3  // Corresponds to SPI3
} t_spi_channel;

typedef enum
{
  SPI_MODE_MASTER,
  SPI_MODE_SLAVE
} t_spi_mode;

typedef enum
{
  SPI_CPOL_LOW,  // Clock polarity low
  SPI_CPOL_HIGH  // Clock polarity high
} t_spi_cpol;

typedef enum
{
  SPI_CPHA_1ST_EDGE, // Clock phase 1st edge
  SPI_CPHA_2ND_EDGE  // Clock phase 2nd edge
} t_spi_cpha;

typedef enum
{
  SPI_DFF_8_BIT, // Data frame format 8-bit
  SPI_DFF_16_BIT // Data frame format 16-bit
} t_spi_dff;

typedef enum
{
  SPI_BIT_ORDER_MSB_FIRST, // Most significant bit first
  SPI_BIT_ORDER_LSB_FIRST  // Least significant bit first
} t_spi_bit_order;

// External Interrupt
typedef enum
{
  EXTERNAL_INT_CHANNEL_0,
  EXTERNAL_INT_CHANNEL_1,
  EXTERNAL_INT_CHANNEL_2,
  EXTERNAL_INT_CHANNEL_3,
  EXTERNAL_INT_CHANNEL_4,
  EXTERNAL_INT_CHANNEL_5,
  EXTERNAL_INT_CHANNEL_6,
  EXTERNAL_INT_CHANNEL_7,
  EXTERNAL_INT_CHANNEL_8,
  EXTERNAL_INT_CHANNEL_9,
  EXTERNAL_INT_CHANNEL_10,
  EXTERNAL_INT_CHANNEL_11,
  EXTERNAL_INT_CHANNEL_12,
  EXTERNAL_INT_CHANNEL_13,
  EXTERNAL_INT_CHANNEL_14,
  EXTERNAL_INT_CHANNEL_15
} t_external_int_channel;

typedef enum
{
  EXTERNAL_INT_EDGE_RISING,
  EXTERNAL_INT_EDGE_FALLING,
  EXTERNAL_INT_EDGE_RISING_FALLING
} t_external_int_edge;

// GPIO
typedef enum
{
  PORT_A, // Corresponds to GPIOA
  PORT_B, // Corresponds to GPIOB
  PORT_C, // Corresponds to GPIOC
  PORT_D, // Corresponds to GPIOD
  PORT_E, // Corresponds to GPIOE
  PORT_H  // Corresponds to GPIOH
} t_port;

typedef enum
{
  PIN_0,
  PIN_1,
  PIN_2,
  PIN_3,
  PIN_4,
  PIN_5,
  PIN_6,
  PIN_7,
  PIN_8,
  PIN_9,
  PIN_10,
  PIN_11,
  PIN_12,
  PIN_13,
  PIN_14,
  PIN_15
} t_pin;

typedef enum
{
  GPIO_DIRECTION_INPUT,
  GPIO_DIRECTION_OUTPUT
} t_direction;

// PWM
typedef enum
{
  PWM_CHANNEL_TIM1_CH1,  // PA8, PE9
  PWM_CHANNEL_TIM1_CH2,  // PA9, PE11
  PWM_CHANNEL_TIM1_CH3,  // PA10, PE13
  PWM_CHANNEL_TIM1_CH4,  // PA11, PE14

  PWM_CHANNEL_TIM2_CH1,  // PA0, PA5, PA15, PB3
  PWM_CHANNEL_TIM2_CH2,  // PA1, PB3, PB10
  PWM_CHANNEL_TIM2_CH3,  // PA2, PB10
  PWM_CHANNEL_TIM2_CH4,  // PA3, PB11

  PWM_CHANNEL_TIM3_CH1,  // PA6, PB4, PC6
  PWM_CHANNEL_TIM3_CH2,  // PA7, PB5, PC7
  PWM_CHANNEL_TIM3_CH3,  // PB0, PC8
  PWM_CHANNEL_TIM3_CH4,  // PB1, PC9

  PWM_CHANNEL_TIM4_CH1,  // PB6
  PWM_CHANNEL_TIM4_CH2,  // PB7
  PWM_CHANNEL_TIM4_CH3,  // PB8
  PWM_CHANNEL_TIM4_CH4,  // PB9

  PWM_CHANNEL_TIM5_CH1,  // PA0
  PWM_CHANNEL_TIM5_CH2,  // PA1
  PWM_CHANNEL_TIM5_CH3,  // PA2
  PWM_CHANNEL_TIM5_CH4,  // PA3

  PWM_CHANNEL_TIM9_CH1,  // PA2, PE5
  PWM_CHANNEL_TIM9_CH2,  // PA3, PE6

  PWM_CHANNEL_TIM10_CH1, // PB8, PA6

  PWM_CHANNEL_TIM11_CH1  // PB9, PA7
} t_pwm_channel;

// ICU (Input Capture Unit)
typedef enum
{
  ICU_CHANNEL_TIM1_CH1,  // PA8, PE9
  ICU_CHANNEL_TIM1_CH2,  // PA9, PE11
  ICU_CHANNEL_TIM1_CH3,  // PA10, PE13
  ICU_CHANNEL_TIM1_CH4,  // PA11, PE14

  ICU_CHANNEL_TIM2_CH1,  // PA0, PA5, PA15, PB3
  ICU_CHANNEL_TIM2_CH2,  // PA1, PB3, PB10
  ICU_CHANNEL_TIM2_CH3,  // PA2, PB10
  ICU_CHANNEL_TIM2_CH4,  // PA3, PB11

  ICU_CHANNEL_TIM3_CH1,  // PA6, PB4, PC6
  ICU_CHANNEL_TIM3_CH2,  // PA7, PB5, PC7
  ICU_CHANNEL_TIM3_CH3,  // PB0, PC8
  ICU_CHANNEL_TIM3_CH4,  // PB1, PC9

  ICU_CHANNEL_TIM4_CH1,  // PB6
  ICU_CHANNEL_TIM4_CH2,  // PB7
  ICU_CHANNEL_TIM4_CH3,  // PB8
  ICU_CHANNEL_TIM4_CH4,  // PB9

  ICU_CHANNEL_TIM5_CH1,  // PA0
  ICU_CHANNEL_TIM5_CH2,  // PA1
  ICU_CHANNEL_TIM5_CH3,  // PA2
  ICU_CHANNEL_TIM5_CH4,  // PA3

  ICU_CHANNEL_TIM9_CH1,  // PA2, PE5
  ICU_CHANNEL_TIM9_CH2,  // PA3, PE6

  ICU_CHANNEL_TIM10_CH1, // PB8, PA6

  ICU_CHANNEL_TIM11_CH1  // PB9, PA7
} t_icu_channel;

typedef enum
{
  ICU_PRESCALLER_DIV1,
  ICU_PRESCALLER_DIV2,
  ICU_PRESCALLER_DIV4,
  ICU_PRESCALLER_DIV8,
  ICU_PRESCALLER_DIV16,
  ICU_PRESCALLER_DIV32,
  ICU_PRESCALLER_DIV64,
  ICU_PRESCALLER_DIV128,
  ICU_PRESCALLER_DIV256,
  ICU_PRESCALLER_DIV512,
  ICU_PRESCALLER_DIV1024
} t_icu_prescaller;

typedef enum
{
  ICU_EDGE_RISING,
  ICU_EDGE_FALLING,
  ICU_EDGE_BOTH // Rising and falling edges
} t_icu_edge;

// Timer
typedef enum
{
  TIMER_CHANNEL_1,  // Corresponds to TIM1
  TIMER_CHANNEL_2,  // Corresponds to TIM2
  TIMER_CHANNEL_3,  // Corresponds to TIM3
  TIMER_CHANNEL_4,  // Corresponds to TIM4
  TIMER_CHANNEL_5,  // Corresponds to TIM5
  TIMER_CHANNEL_9,  // Corresponds to TIM9
  TIMER_CHANNEL_10, // Corresponds to TIM10
  TIMER_CHANNEL_11  // Corresponds to TIM11
} t_timer_channel;

// ADC (Analog to Digital Converter)
typedef enum
{
  ADC_CHANNEL_0,  // PA0
  ADC_CHANNEL_1,  // PA1
  ADC_CHANNEL_2,  // PA2
  ADC_CHANNEL_3,  // PA3
  ADC_CHANNEL_4,  // PA4
  ADC_CHANNEL_5,  // PA5
  ADC_CHANNEL_6,  // PA6
  ADC_CHANNEL_7,  // PA7
  ADC_CHANNEL_8,  // PB0
  ADC_CHANNEL_9,  // PB1
  ADC_CHANNEL_10, // PC0
  ADC_CHANNEL_11, // PC1
  ADC_CHANNEL_12, // PC2
  ADC_CHANNEL_13, // PC3
  ADC_CHANNEL_14, // PC4
  ADC_CHANNEL_15, // PC5
  // Internal channels (STM32F401RC specific)
  ADC_CHANNEL_16, // Temperature Sensor
  ADC_CHANNEL_17, // Vrefint
  ADC_CHANNEL_18  // Vbat
} t_adc_channel;

typedef enum
{
  ADC_MODE_POLLING,
  ADC_MODE_INTERRUPT
} t_adc_mode_t;

// TT (Time Triggered OS)
typedef enum
{
  TICK_TIME_1MS,
  TICK_TIME_5MS,
  TICK_TIME_10MS,
  TICK_TIME_100MS,
  TICK_TIME_1S
} t_tick_time;

// I2S (Inter-IC Sound)
typedef enum
{
  I2S_CHANNEL_1, // Corresponds to SPI1 I2S mode
  I2S_CHANNEL_2, // Corresponds to SPI2 I2S mode
  I2S_CHANNEL_3  // Corresponds to SPI3 I2S mode
} t_i2s_channel;

// Placeholder enums for I2S as their specific bitfield values are not provided
// and would typically be MCU-specific or defined in a lower-level driver.
typedef enum {
    I2S_MODE_SLAVE_TX,
    I2S_MODE_SLAVE_RX,
    I2S_MODE_MASTER_TX,
    I2S_MODE_MASTER_RX
} I2S_Mode_t;

typedef enum {
    I2S_STANDARD_PHILIPS,
    I2S_STANDARD_MSB,
    I2S_STANDARD_LSB,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

typedef enum {
    I2S_DATAFORMAT_16B,
    I2S_DATAFORMAT_16B_EXTENDED,
    I2S_DATAFORMAT_24B,
    I2S_DATAFORMAT_32B
} I2S_DataFormat_t;

typedef enum {
    I2S_CHANNELMODE_STEREO,
    I2S_CHANNELMODE_MONO
} I2S_ChannelMode_t;

// WiFi Driver
typedef enum
{
  TX_MODE_WIFI_A,
  TX_MODE_WIFI_B,
  TX_MODE_WIFI_G,
  TX_MODE_WIFI_N
} t_tx_mode;

// End of typedefs


// MCAL API Function Prototypes (from API.json)

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // Defined here; also part of WDT module.
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
void LVD_Enable(void);
void LVD_Disable(void);

// UART
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

// I2C
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

// SPI (CSI)
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);

// GPIO
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);

// PWM
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);

// ICU
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
float ICU_GetFrequency(t_icu_channel icu_channel); // Return type assumed as float (not specified in API.json)
void ICU_setCallback(void (*callback)(void));

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(t_adc_channel adc_channel);
void ADC_Disable(t_adc_channel adc_channel);
tword ADC_Get_POLLING(t_adc_channel adc_channel);
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel);

// Internal_EEPROM
// Internal_EEPROM not supported on STM32F401RC (no dedicated internal EEPROM, Flash emulation typically used which is not covered by provided register_json).
// void Internal_EEPROM_Init(void);
// void Internal_EEPROM_Set(tbyte address, tbyte data);
// tbyte Internal_EEPROM_Get(tbyte address);

// TT
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// MCAL_OUTPUT_BUZZER not supported on STM32F401RC


// WDT
void WDT_Init(void);
// WDT_Reset prototype is already listed under MCU CONFIG to avoid duplication.

// DAC
// DAC not supported on STM32F401RC (no dedicated DAC registers in provided register_json).
// typedef enum { DAC_CHANNEL_1, DAC_CHANNEL_2 } dac_channel_t;
// void DAC_Init(dac_channel_t channel);
// void DAC_Enable(dac_channel_t channel);
// void DAC_Disable(dac_channel_t channel);
// void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue);

// I2S
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// MQTT Protocol
// MQTT Protocol not supported on this MCU (no direct register support in provided register_json for a full MQTT stack).
// void MQTT_Init(void);
// void MQTT_Subscribe_Topic(void *handler_args, esp_event_base_t base, tslong event_id, void *event_data);
// void MQTT_Publish_Message(const tsbyte *message);
// void AZURE_IoT_Hub_Client_Init(void);
// void AZURE_Connection_Enable(void);

// HTTP Protocol
// HTTP Protocol not supported on this MCU (no direct register support in provided register_json for a full HTTP stack).
// void HTTP_Get_Device_ID(char *device_id, size_t size);
// void HTTP_Server_Init(void);
// void HTTP_Server_Start(void);
// void HTTP_Server_Stop(void);
// void HTTP_Reset_SSID_PASSWORD(void);
// esp_err_t HTTP_Config_Handler(httpd_req_t *req);

// WiFi Driver
// WiFi Driver not supported on this MCU (no direct register support in provided register_json for a Wi-Fi module).
// void WiFi_Init(void);
// void WiFi_Connect(const tsbyte *ssid, const tsbyte *password);
// void WiFi_Enable(void);
// void WiFi_Disable(void);
// void WiFi_SetTxMode(t_tx_mode mode);
// tsword WiFi_Check_Connection(void);
// int WiFi_Check_Internet(void);

// DTC_driver
// DTC_driver not supported on this MCU (no direct register support in provided register_json for this specific DTC abstraction, though DMA exists).
// void DTC_Init();
// void DTC_EnableSource(uint8_t source_id, uint8_t channel);
// void DTC_DisableSource(uint8_t source_id);
// void DTC_Start(void);
// void DTC_Stop(void);
// void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int);

#endif /* MCAL_H */
