/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the API prototypes and type definitions for the MCAL
 * layer, providing an abstraction over the STM32F401RC microcontroller's
 * peripherals.
 */

#ifndef MCAL_H_
#define MCAL_H_

/*
 * =================================================================================================
 *                                        Core Includes
 * =================================================================================================
 */
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * =================================================================================================
 *                                     Data Type Definitions
 * =================================================================================================
 */
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;
typedef int16_t  tsword; // For API.json types not explicitly in rules.json
typedef int32_t  tslong; // For API.json types not explicitly in rules.json
typedef size_t   t_size_t; // For standard library sizes

/*
 * =================================================================================================
 *                                       Register Addresses
 * =================================================================================================
 */
// FLASH Registers
#define FLASH_ACR_ADDRESS       ((volatile uint32_t *)0x40023C00)
#define FLASH_KEYR_ADDRESS      ((volatile uint32_t *)0x40023C04)
#define FLASH_OPTKEYR_ADDRESS   ((volatile uint32_t *)0x40023C08)
#define FLASH_SR_ADDRESS        ((volatile uint32_t *)0x40023C0C)
#define FLASH_CR_ADDRESS        ((volatile uint32_t *)0x40023C10)
#define FLASH_OPTCR_ADDRESS     ((volatile uint32_t *)0x40023C14)

// RCC Registers
#define RCC_CR_ADDRESS          ((volatile uint32_t *)0x40023800)
#define RCC_PLLCFGR_ADDRESS     ((volatile uint32_t *)0x40023804)
#define RCC_CFGR_ADDRESS        ((volatile uint32_t *)0x40023808)
#define RCC_CIR_ADDRESS         ((volatile uint32_t *)0x4002380C)
#define RCC_AHB1RSTR_ADDRESS    ((volatile uint32_t *)0x40023810)
#define RCC_AHB2RSTR_ADDRESS    ((volatile uint32_t *)0x40023814)
#define RCC_APB1RSTR_ADDRESS    ((volatile uint32_t *)0x40023818)
#define RCC_APB2RSTR_ADDRESS    ((volatile uint32_t *)0x4002381C)
#define RCC_AHB1ENR_ADDRESS     ((volatile uint32_t *)0x40023830)
#define RCC_AHB2ENR_ADDRESS     ((volatile uint32_t *)0x40023834)
#define RCC_APB1ENR_ADDRESS     ((volatile uint32_t *)0x40023838)
#define RCC_APB2ENR_ADDRESS     ((volatile uint32_t *)0x4002383C)
#define RCC_AHB1LPENR_ADDRESS   ((volatile uint32_t *)0x40023840)
#define RCC_AHB2LPENR_ADDRESS   ((volatile uint32_t *)0x40023844)
#define RCC_APB1LPENR_ADDRESS   ((volatile uint32_t *)0x40023848)
#define RCC_APB2LPENR_ADDRESS   ((volatile uint32_t *)0x4002384C)
#define RCC_BDCR_ADDRESS        ((volatile uint32_t *)0x40023850)
#define RCC_CSR_ADDRESS         ((volatile uint32_t *)0x40023854)
#define RCC_SSCGR_ADDRESS       ((volatile uint32_t *)0x40023858)
#define RCC_PLLI2SCFGR_ADDRESS  ((volatile uint32_t *)0x4002385C)
#define RCC_DCKCFGR_ADDRESS     ((volatile uint32_t *)0x40023864)

// SYSCFG Registers
#define SYSCFG_MEMRMP_ADDRESS   ((volatile uint32_t *)0x40013800)
#define SYSCFG_PMC_ADDRESS      ((volatile uint32_t *)0x40013804)
#define SYSCFG_EXTICR1_ADDRESS  ((volatile uint32_t *)0x40013808)
#define SYSCFG_EXTICR2_ADDRESS  ((volatile uint32_t *)0x4001380C)
#define SYSCFG_EXTICR3_ADDRESS  ((volatile uint32_t *)0x40013810)
#define SYSCFG_EXTICR4_ADDRESS  ((volatile uint32_t *)0x40013814)
#define SYSCFG_CMPCR_ADDRESS    ((volatile uint32_t *)0x40013820)

// GPIO Registers
#define GPIOA_MODER_ADDRESS     ((volatile uint32_t *)0x40020000)
#define GPIOA_OTYPER_ADDRESS    ((volatile uint32_t *)0x40020004)
#define GPIOA_OSPEEDR_ADDRESS   ((volatile uint32_t *)0x40020008)
#define GPIOA_PUPDR_ADDRESS     ((volatile uint32_t *)0x4002000C)
#define GPIOA_IDR_ADDRESS       ((volatile uint32_t *)0x40020010)
#define GPIOA_ODR_ADDRESS       ((volatile uint32_t *)0x40020014)
#define GPIOA_BSRR_ADDRESS      ((volatile uint32_t *)0x40020018)
#define GPIOA_LCKR_ADDRESS      ((volatile uint32_t *)0x4002001C)
#define GPIOA_AFRL_ADDRESS      ((volatile uint32_t *)0x40020020)
#define GPIOA_AFRH_ADDRESS      ((volatile uint32_t *)0x40020024)

#define GPIOB_MODER_ADDRESS     ((volatile uint32_t *)0x40020400)
#define GPIOB_OTYPER_ADDRESS    ((volatile uint32_t *)0x40020404)
#define GPIOB_OSPEEDR_ADDRESS   ((volatile uint32_t *)0x40020408)
#define GPIOB_PUPDR_ADDRESS     ((volatile uint32_t *)0x4002040C)
#define GPIOB_IDR_ADDRESS       ((volatile uint32_t *)0x40020410)
#define GPIOB_ODR_ADDRESS       ((volatile uint32_t *)0x40020414)
#define GPIOB_BSRR_ADDRESS      ((volatile uint32_t *)0x40020418)
#define GPIOB_LCKR_ADDRESS      ((volatile uint32_t *)0x4002041C)
#define GPIOB_AFRL_ADDRESS      ((volatile uint32_t *)0x40020420)
#define GPIOB_AFRH_ADDRESS      ((volatile uint32_t *)0x40020424)

#define GPIOC_MODER_ADDRESS     ((volatile uint32_t *)0x40020800)
#define GPIOC_OTYPER_ADDRESS    ((volatile uint32_t *)0x40020804)
#define GPIOC_OSPEEDR_ADDRESS   ((volatile uint32_t *)0x40020808)
#define GPIOC_PUPDR_ADDRESS     ((volatile uint32_t *)0x4002080C)
#define GPIOC_IDR_ADDRESS       ((volatile uint32_t *)0x40020810)
#define GPIOC_ODR_ADDRESS       ((volatile uint32_t *)0x40020814)
#define GPIOC_BSRR_ADDRESS      ((volatile uint32_t *)0x40020818)
#define GPIOC_LCKR_ADDRESS      ((volatile uint32_t *)0x4002081C)
#define GPIOC_AFRL_ADDRESS      ((volatile uint32_t *)0x40020820)
#define GPIOC_AFRH_ADDRESS      ((volatile uint32_t *)0x40020824)

#define GPIOD_MODER_ADDRESS     ((volatile uint32_t *)0x40020C00)
#define GPIOD_OTYPER_ADDRESS    ((volatile uint32_t *)0x40020C04)
#define GPIOD_OSPEEDR_ADDRESS   ((volatile uint32_t *)0x40020C08)
#define GPIOD_PUPDR_ADDRESS     ((volatile uint32_t *)0x40020C0C)
#define GPIOD_IDR_ADDRESS       ((volatile uint32_t *)0x40020C10)
#define GPIOD_ODR_ADDRESS       ((volatile uint32_t *)0x40020C14)
#define GPIOD_BSRR_ADDRESS      ((volatile uint32_t *)0x40020C18)
#define GPIOD_LCKR_ADDRESS      ((volatile uint32_t *)0x40020C1C)
#define GPIOD_AFRL_ADDRESS      ((volatile uint32_t *)0x40020C20)
#define GPIOD_AFRH_ADDRESS      ((volatile uint32_t *)0x40020C24)

#define GPIOE_MODER_ADDRESS     ((volatile uint32_t *)0x40021000)
#define GPIOE_OTYPER_ADDRESS    ((volatile uint32_t *)0x40021004)
#define GPIOE_OSPEEDR_ADDRESS   ((volatile uint32_t *)0x40021008)
#define GPIOE_PUPDR_ADDRESS     ((volatile uint32_t *)0x4002100C)
#define GPIOE_IDR_ADDRESS       ((volatile uint32_t *)0x40021010)
#define GPIOE_ODR_ADDRESS       ((volatile uint32_t *)0x40021014)
#define GPIOE_BSRR_ADDRESS      ((volatile uint32_t *)0x40021018)
#define GPIOE_LCKR_ADDRESS      ((volatile uint32_t *)0x4002101C)
#define GPIOE_AFRL_ADDRESS      ((volatile uint32_t *)0x40021020)
#define GPIOE_AFRH_ADDRESS      ((volatile uint32_t *)0x40021024)

#define GPIOH_MODER_ADDRESS     ((volatile uint32_t *)0x40021C00)
#define GPIOH_OTYPER_ADDRESS    ((volatile uint32_t *)0x40021C04)
#define GPIOH_OSPEEDR_ADDRESS   ((volatile uint32_t *)0x40021C08)
#define GPIOH_PUPDR_ADDRESS     ((volatile uint32_t *)0x40021C0C)
#define GPIOH_IDR_ADDRESS       ((volatile uint32_t *)0x40021C10)
#define GPIOH_ODR_ADDRESS       ((volatile uint32_t *)0x40021C14)
#define GPIOH_BSRR_ADDRESS      ((volatile uint32_t *)0x40021C18)
#define GPIOH_LCKR_ADDRESS      ((volatile uint32_t *)0x40021C1C)
#define GPIOH_AFRL_ADDRESS      ((volatile uint32_t *)0x40021C20)
#define GPIOH_AFRH_ADDRESS      ((volatile uint32_t *)0x40021C24)

// EXTI Registers
#define EXTI_IMR_ADDRESS        ((volatile uint32_t *)0x40013C00)
#define EXTI_EMR_ADDRESS        ((volatile uint32_t *)0x40013C04)
#define EXTI_RTSR_ADDRESS       ((volatile uint32_t *)0x40013C08)
#define EXTI_FTSR_ADDRESS       ((volatile uint32_t *)0x40013C0C)
#define EXTI_SWIER_ADDRESS      ((volatile uint32_t *)0x40013C10)
#define EXTI_PR_ADDRESS         ((volatile uint32_t *)0x40013C14)

// ADC Registers
#define ADC1_SR_ADDRESS         ((volatile uint32_t *)0x40012000)
#define ADC1_CR1_ADDRESS        ((volatile uint32_t *)0x40012004)
#define ADC1_CR2_ADDRESS        ((volatile uint32_t *)0x40012008)
#define ADC1_SMPR1_ADDRESS      ((volatile uint32_t *)0x4001200C)
#define ADC1_SMPR2_ADDRESS      ((volatile uint32_t *)0x40012010)
#define ADC1_JOFR1_ADDRESS      ((volatile uint32_t *)0x40012014)
#define ADC1_JOFR2_ADDRESS      ((volatile uint32_t *)0x40012018)
#define ADC1_JOFR3_ADDRESS      ((volatile uint32_t *)0x4001201C)
#define ADC1_JOFR4_ADDRESS      ((volatile uint32_t *)0x40012020)
#define ADC1_HTR_ADDRESS        ((volatile uint32_t *)0x40012024)
#define ADC1_LTR_ADDRESS        ((volatile uint32_t *)0x40012028)
#define ADC1_SQR1_ADDRESS       ((volatile uint32_t *)0x4001202C)
#define ADC1_SQR2_ADDRESS       ((volatile uint32_t *)0x40012030)
#define ADC1_SQR3_ADDRESS       ((volatile uint32_t *)0x40012034)
#define ADC1_JSQR_ADDRESS       ((volatile uint32_t *)0x40012038)
#define ADC1_JDR1_ADDRESS       ((volatile uint32_t *)0x4001203C)
#define ADC1_JDR2_ADDRESS       ((volatile uint32_t *)0x40012040)
#define ADC1_JDR3_ADDRESS       ((volatile uint32_t *)0x40012044)
#define ADC1_JDR4_ADDRESS       ((volatile uint32_t *)0x40012048)
#define ADC1_DR_ADDRESS         ((volatile uint32_t *)0x4001204C)
#define ADC_CCR_ADDRESS         ((volatile uint32_t *)0x40012304)

// TIM1 Registers
#define TIM1_CR1_ADDRESS        ((volatile uint32_t *)0x40010000)
#define TIM1_CR2_ADDRESS        ((volatile uint32_t *)0x40010004)
#define TIM1_SMCR_ADDRESS       ((volatile uint32_t *)0x40010008)
#define TIM1_DIER_ADDRESS       ((volatile uint32_t *)0x4001000C)
#define TIM1_SR_ADDRESS         ((volatile uint32_t *)0x40010010)
#define TIM1_EGR_ADDRESS        ((volatile uint32_t *)0x40010014)
#define TIM1_CCMR1_ADDRESS      ((volatile uint32_t *)0x40010018)
#define TIM1_CCMR2_ADDRESS      ((volatile uint32_t *)0x4001001C)
#define TIM1_CCER_ADDRESS       ((volatile uint32_t *)0x40010020)
#define TIM1_CNT_ADDRESS        ((volatile uint32_t *)0x40010024)
#define TIM1_PSC_ADDRESS        ((volatile uint32_t *)0x40010028)
#define TIM1_ARR_ADDRESS        ((volatile uint32_t *)0x4001002C)
#define TIM1_RCR_ADDRESS        ((volatile uint32_t *)0x40010030)
#define TIM1_CCR1_ADDRESS       ((volatile uint32_t *)0x40010034)
#define TIM1_CCR2_ADDRESS       ((volatile uint32_t *)0x40010038)
#define TIM1_CCR3_ADDRESS       ((volatile uint32_t *)0x4001003C)
#define TIM1_CCR4_ADDRESS       ((volatile uint32_t *)0x40010040)
#define TIM1_BDTR_ADDRESS       ((volatile uint32_t *)0x40010044)
#define TIM1_DCR_ADDRESS        ((volatile uint32_t *)0x40010048)
#define TIM1_DMAR_ADDRESS       ((volatile uint32_t *)0x4001004C)

// TIM2 Registers
#define TIM2_CR1_ADDRESS        ((volatile uint32_t *)0x40000000)
#define TIM2_CR2_ADDRESS        ((volatile uint32_t *)0x40000004)
#define TIM2_SMCR_ADDRESS       ((volatile uint32_t *)0x40000008)
#define TIM2_DIER_ADDRESS       ((volatile uint32_t *)0x4000000C)
#define TIM2_SR_ADDRESS         ((volatile uint32_t *)0x40000010)
#define TIM2_EGR_ADDRESS        ((volatile uint32_t *)0x40000014)
#define TIM2_CCMR1_ADDRESS      ((volatile uint32_t *)0x40000018)
#define TIM2_CCMR2_ADDRESS      ((volatile uint32_t *)0x4000001C)
#define TIM2_CCER_ADDRESS       ((volatile uint32_t *)0x40000020)
#define TIM2_CNT_ADDRESS        ((volatile uint32_t *)0x40000024)
#define TIM2_PSC_ADDRESS        ((volatile uint32_t *)0x40000028)
#define TIM2_ARR_ADDRESS        ((volatile uint32_t *)0x4000002C)
#define TIM2_CCR1_ADDRESS       ((volatile uint32_t *)0x40000034)
#define TIM2_CCR2_ADDRESS       ((volatile uint32_t *)0x40000038)
#define TIM2_CCR3_ADDRESS       ((volatile uint32_t *)0x4000003C)
#define TIM2_CCR4_ADDRESS       ((volatile uint32_t *)0x40000040)
#define TIM2_DCR_ADDRESS        ((volatile uint32_t *)0x40000048)
#define TIM2_DMAR_ADDRESS       ((volatile uint32_t *)0x4000004C)
#define TIM2_OR_ADDRESS         ((volatile uint32_t *)0x40000050)

// TIM3 Registers
#define TIM3_CR1_ADDRESS        ((volatile uint32_t *)0x40000400)
#define TIM3_CR2_ADDRESS        ((volatile uint32_t *)0x40000404)
#define TIM3_SMCR_ADDRESS       ((volatile uint32_t *)0x40000408)
#define TIM3_DIER_ADDRESS       ((volatile uint32_t *)0x4000040C)
#define TIM3_SR_ADDRESS         ((volatile uint32_t *)0x40000410)
#define TIM3_EGR_ADDRESS        ((volatile uint32_t *)0x40000414)
#define TIM3_CCMR1_ADDRESS      ((volatile uint32_t *)0x40000418)
#define TIM3_CCMR2_ADDRESS      ((volatile uint32_t *)0x4000041C)
#define TIM3_CCER_ADDRESS       ((volatile uint32_t *)0x40000420)
#define TIM3_CNT_ADDRESS        ((volatile uint32_t *)0x40000424)
#define TIM3_PSC_ADDRESS        ((volatile uint32_t *)0x40000428)
#define TIM3_ARR_ADDRESS        ((volatile uint32_t *)0x4000042C)
#define TIM3_CCR1_ADDRESS       ((volatile uint32_t *)0x40000434)
#define TIM3_CCR2_ADDRESS       ((volatile uint32_t *)0x40000438)
#define TIM3_CCR3_ADDRESS       ((volatile uint32_t *)0x4000043C)
#define TIM3_CCR4_ADDRESS       ((volatile uint32_t *)0x40000440)
#define TIM3_DCR_ADDRESS        ((volatile uint32_t *)0x40000448)
#define TIM3_DMAR_ADDRESS       ((volatile uint32_t *)0x4000044C)

// TIM4 Registers
#define TIM4_CR1_ADDRESS        ((volatile uint32_t *)0x40000800)
#define TIM4_CR2_ADDRESS        ((volatile uint32_t *)0x40000804)
#define TIM4_SMCR_ADDRESS       ((volatile uint32_t *)0x40000808)
#define TIM4_DIER_ADDRESS       ((volatile uint32_t *)0x4000080C)
#define TIM4_SR_ADDRESS         ((volatile uint32_t *)0x40000810)
#define TIM4_EGR_ADDRESS        ((volatile uint32_t *)0x40000814)
#define TIM4_CCMR1_ADDRESS      ((volatile uint32_t *)0x40000818)
#define TIM4_CCMR2_ADDRESS      ((volatile uint32_t *)0x4000081C)
#define TIM4_CCER_ADDRESS       ((volatile uint32_t *)0x40000820)
#define TIM4_CNT_ADDRESS        ((volatile uint32_t *)0x40000824)
#define TIM4_PSC_ADDRESS        ((volatile uint32_t *)0x40000828)
#define TIM4_ARR_ADDRESS        ((volatile uint32_t *)0x4000082C)
#define TIM4_CCR1_ADDRESS       ((volatile uint32_t *)0x40000834)
#define TIM4_CCR2_ADDRESS       ((volatile uint32_t *)0x40000838)
#define TIM4_CCR3_ADDRESS       ((volatile uint32_t *)0x4000083C)
#define TIM4_CCR4_ADDRESS       ((volatile uint32_t *)0x40000840)
#define TIM4_DCR_ADDRESS        ((volatile uint32_t *)0x40000848)
#define TIM4_DMAR_ADDRESS       ((volatile uint32_t *)0x4000084C)

// TIM5 Registers
#define TIM5_CR1_ADDRESS        ((volatile uint32_t *)0x40000C00)
#define TIM5_CR2_ADDRESS        ((volatile uint32_t *)0x40000C04)
#define TIM5_SMCR_ADDRESS       ((volatile uint32_t *)0x40000C08)
#define TIM5_DIER_ADDRESS       ((volatile uint32_t *)0x40000C0C)
#define TIM5_SR_ADDRESS         ((volatile uint32_t *)0x40000C10)
#define TIM5_EGR_ADDRESS        ((volatile uint32_t *)0x40000C14)
#define TIM5_CCMR1_ADDRESS      ((volatile uint32_t *)0x40000C18)
#define TIM5_CCMR2_ADDRESS      ((volatile uint32_t *)0x40000C1C)
#define TIM5_CCER_ADDRESS       ((volatile uint32_t *)0x40000C20)
#define TIM5_CNT_ADDRESS        ((volatile uint32_t *)0x40000C24)
#define TIM5_PSC_ADDRESS        ((volatile uint32_t *)0x40000C28)
#define TIM5_ARR_ADDRESS        ((volatile uint32_t *)0x40000C2C)
#define TIM5_CCR1_ADDRESS       ((volatile uint32_t *)0x40000C34)
#define TIM5_CCR2_ADDRESS       ((volatile uint32_t *)0x40000C38)
#define TIM5_CCR3_ADDRESS       ((volatile uint32_t *)0x40000C3C)
#define TIM5_CCR4_ADDRESS       ((volatile uint32_t *)0x40000C40)
#define TIM5_DCR_ADDRESS        ((volatile uint32_t *)0x40000C48)
#define TIM5_DMAR_ADDRESS       ((volatile uint32_t *)0x40000C4C)
#define TIM5_OR_ADDRESS         ((volatile uint32_t *)0x40000C54)

// TIM9 Registers
#define TIM9_CR1_ADDRESS        ((volatile uint32_t *)0x40014000)
#define TIM9_SMCR_ADDRESS       ((volatile uint32_t *)0x40014008)
#define TIM9_DIER_ADDRESS       ((volatile uint32_t *)0x4001400C)
#define TIM9_SR_ADDRESS         ((volatile uint32_t *)0x40014010)
#define TIM9_EGR_ADDRESS        ((volatile uint32_t *)0x40014014)
#define TIM9_CCMR1_ADDRESS      ((volatile uint32_t *)0x40014018)
#define TIM9_CCER_ADDRESS       ((volatile uint32_t *)0x40014020)
#define TIM9_CNT_ADDRESS        ((volatile uint32_t *)0x40014024)
#define TIM9_PSC_ADDRESS        ((volatile uint32_t *)0x40014028)
#define TIM9_ARR_ADDRESS        ((volatile uint32_t *)0x4001402C)
#define TIM9_CCR1_ADDRESS       ((volatile uint32_t *)0x40014034)
#define TIM9_CCR2_ADDRESS       ((volatile uint32_t *)0x40014038)

// TIM10 Registers
#define TIM10_CR1_ADDRESS       ((volatile uint32_t *)0x40014400)
#define TIM10_DIER_ADDRESS      ((volatile uint32_t *)0x4001440C)
#define TIM10_SR_ADDRESS        ((volatile uint32_t *)0x40014410)
#define TIM10_EGR_ADDRESS       ((volatile uint32_t *)0x40014414)
#define TIM10_CCMR1_ADDRESS     ((volatile uint32_t *)0x40014418)
#define TIM10_CCER_ADDRESS      ((volatile uint32_t *)0x40014420)
#define TIM10_CNT_ADDRESS       ((volatile uint32_t *)0x40014424)
#define TIM10_PSC_ADDRESS       ((volatile uint32_t *)0x40014428)
#define TIM10_ARR_ADDRESS       ((volatile uint32_t *)0x4001442C)
#define TIM10_CCR1_ADDRESS      ((volatile uint32_t *)0x40014434)

// TIM11 Registers
#define TIM11_CR1_ADDRESS       ((volatile uint32_t *)0x40014800)
#define TIM11_DIER_ADDRESS      ((volatile uint32_t *)0x4001480C)
#define TIM11_SR_ADDRESS        ((volatile uint32_t *)0x40014810)
#define TIM11_EGR_ADDRESS       ((volatile uint32_t *)0x40014814)
#define TIM11_CCMR1_ADDRESS     ((volatile uint32_t *)0x40014818)
#define TIM11_CCER_ADDRESS      ((volatile uint32_t *)0x40014820)
#define TIM11_CNT_ADDRESS       ((volatile uint32_t *)0x40014824)
#define TIM11_PSC_ADDRESS       ((volatile uint32_t *)0x40014828)
#define TIM11_ARR_ADDRESS       ((volatile uint32_t *)0x4001482C)
#define TIM11_CCR1_ADDRESS      ((volatile uint32_t *)0x40014834)

// USART Registers
#define USART1_SR_ADDRESS       ((volatile uint32_t *)0x40011000)
#define USART1_DR_ADDRESS       ((volatile uint32_t *)0x40011004)
#define USART1_BRR_ADDRESS      ((volatile uint32_t *)0x40011008)
#define USART1_CR1_ADDRESS      ((volatile uint32_t *)0x4001100C)
#define USART1_CR2_ADDRESS      ((volatile uint32_t *)0x40011010)
#define USART1_CR3_ADDRESS      ((volatile uint32_t *)0x40011014)
#define USART1_GTPR_ADDRESS     ((volatile uint32_t *)0x40011018)

#define USART2_SR_ADDRESS       ((volatile uint32_t *)0x40004400)
#define USART2_DR_ADDRESS       ((volatile uint32_t *)0x40004404)
#define USART2_BRR_ADDRESS      ((volatile uint32_t *)0x40004408)
#define USART2_CR1_ADDRESS      ((volatile uint32_t *)0x4000440C)
#define USART2_CR2_ADDRESS      ((volatile uint32_t *)0x40004410)
#define USART2_CR3_ADDRESS      ((volatile uint32_t *)0x40004414)
#define USART2_GTPR_ADDRESS     ((volatile uint32_t *)0x40004418)

#define USART6_SR_ADDRESS       ((volatile uint32_t *)0x40011400)
#define USART6_DR_ADDRESS       ((volatile uint32_t *)0x40011404)
#define USART6_BRR_ADDRESS      ((volatile uint32_t *)0x40011408)
#define USART6_CR1_ADDRESS      ((volatile uint32_t *)0x4001140C)
#define USART6_CR2_ADDRESS      ((volatile uint32_t *)0x40011410)
#define USART6_CR3_ADDRESS      ((volatile uint32_t *)0x40011414)
#define USART6_GTPR_ADDRESS     ((volatile uint32_t *)0x40011418)

// I2C Registers
#define I2C1_CR1_ADDRESS        ((volatile uint32_t *)0x40005400)
#define I2C1_CR2_ADDRESS        ((volatile uint32_t *)0x40005404)
#define I2C1_OAR1_ADDRESS       ((volatile uint32_t *)0x40005408)
#define I2C1_OAR2_ADDRESS       ((volatile uint32_t *)0x4000540C)
#define I2C1_DR_ADDRESS         ((volatile uint32_t *)0x40005410)
#define I2C1_SR1_ADDRESS        ((volatile uint32_t *)0x40005414)
#define I2C1_SR2_ADDRESS        ((volatile uint32_t *)0x40005418)
#define I2C1_CCR_ADDRESS        ((volatile uint32_t *)0x4000541C)
#define I2C1_TRISE_ADDRESS      ((volatile uint32_t *)0x40005420)
#define I2C1_FLTR_ADDRESS       ((volatile uint32_t *)0x40005424)

#define I2C2_CR1_ADDRESS        ((volatile uint32_t *)0x40005800)
#define I2C2_CR2_ADDRESS        ((volatile uint32_t *)0x40005804)
#define I2C2_OAR1_ADDRESS       ((volatile uint32_t *)0x40005808)
#define I2C2_OAR2_ADDRESS       ((volatile uint32_t *)0x4000580C)
#define I2C2_DR_ADDRESS         ((volatile uint32_t *)0x40005810)
#define I2C2_SR1_ADDRESS        ((volatile uint32_t *)0x40005814)
#define I2C2_SR2_ADDRESS        ((volatile uint32_t *)0x40005818)
#define I2C2_CCR_ADDRESS        ((volatile uint32_t *)0x4000581C)
#define I2C2_TRISE_ADDRESS      ((volatile uint32_t *)0x40005820)
#define I2C2_FLTR_ADDRESS       ((volatile uint32_t *)0x40005824)

#define I2C3_CR1_ADDRESS        ((volatile uint32_t *)0x40005C00)
#define I2C3_CR2_ADDRESS        ((volatile uint32_t *)0x40005C04)
#define I2C3_OAR1_ADDRESS       ((volatile uint32_t *)0x40005C08)
#define I2C3_OAR2_ADDRESS       ((volatile uint32_t *)0x40005C0C)
#define I2C3_DR_ADDRESS         ((volatile uint32_t *)0x40005C10)
#define I2C3_SR1_ADDRESS        ((volatile uint32_t *)0x40005C14)
#define I2C3_SR2_ADDRESS        ((volatile uint32_t *)0x40005C18)
#define I2C3_CCR_ADDRESS        ((volatile uint32_t *)0x40005C1C)
#define I2C3_TRISE_ADDRESS      ((volatile uint32_t *)0x40005C20)
#define I2C3_FLTR_ADDRESS       ((volatile uint32_t *)0x40005C24)

// SPI Registers
#define SPI1_CR1_ADDRESS        ((volatile uint32_t *)0x40013000)
#define SPI1_CR2_ADDRESS        ((volatile uint32_t *)0x40013004)
#define SPI1_SR_ADDRESS         ((volatile uint32_t *)0x40013008)
#define SPI1_DR_ADDRESS         ((volatile uint32_t *)0x4001300C)
#define SPI1_CRCPR_ADDRESS      ((volatile uint32_t *)0x40013010)
#define SPI1_RXCRCR_ADDRESS     ((volatile uint32_t *)0x40013014)
#define SPI1_TXCRCR_ADDRESS     ((volatile uint32_t *)0x40013018)
#define SPI1_I2SCFGR_ADDRESS    ((volatile uint32_t *)0x4001301C)
#define SPI1_I2SPR_ADDRESS      ((volatile uint32_t *)0x40013020)

#define SPI2_CR1_ADDRESS        ((volatile uint32_t *)0x40003800)
#define SPI2_CR2_ADDRESS        ((volatile uint32_t *)0x40003804)
#define SPI2_SR_ADDRESS         ((volatile uint32_t *)0x40003808)
#define SPI2_DR_ADDRESS         ((volatile uint32_t *)0x4000380C)
#define SPI2_CRCPR_ADDRESS      ((volatile uint32_t *)0x40003810)
#define SPI2_RXCRCR_ADDRESS     ((volatile uint32_t *)0x40003814)
#define SPI2_TXCRCR_ADDRESS     ((volatile uint32_t *)0x40003818)
#define SPI2_I2SCFGR_ADDRESS    ((volatile uint32_t *)0x4000381C)
#define SPI2_I2SPR_ADDRESS      ((volatile uint32_t *)0x40003820)

#define SPI3_CR1_ADDRESS        ((volatile uint32_t *)0x40003C00)
#define SPI3_CR2_ADDRESS        ((volatile uint32_t *)0x40003C04)
#define SPI3_SR_ADDRESS         ((volatile uint32_t *)0x40003C08)
#define SPI3_DR_ADDRESS         ((volatile uint32_t *)0x40003C0C)
#define SPI3_CRCPR_ADDRESS      ((volatile uint32_t *)0x40003C10)
#define SPI3_RXCRCR_ADDRESS     ((volatile uint32_t *)0x40003C14)
#define SPI3_TXCRCR_ADDRESS     ((volatile uint32_t *)0x40003C18)
#define SPI3_I2SCFGR_ADDRESS    ((volatile uint32_t *)0x40003C1C)
#define SPI3_I2SPR_ADDRESS      ((volatile uint32_t *)0x40003C20)

/*
 * =================================================================================================
 *                                       Peripheral-Specific Enums
 * =================================================================================================
 */

// MCU Config
typedef enum
{
    VOLT_3V = 0,
    VOLT_5V
} t_sys_volt;

// LVD (Low Voltage Detection)
typedef enum
{
    LVD_THRESHOLD_0_5V = 0, // Placeholder, specific values might vary by MCU
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1_5V,
    LVD_THRESHOLD_2V,
    LVD_THRESHOLD_2_5V,
    LVD_THRESHOLD_3V,
    LVD_THRESHOLD_3_5V,
    LVD_THRESHOLD_4V,
    LVD_THRESHOLD_4_5V,
    LVD_THRESHOLD_5V,
    // Add other thresholds as per MCU datasheet
    LVD_THRESHOLD_DEFAULT_3V_SYSTEM = LVD_THRESHOLD_2V,   // Rule: 2V for 3V system
    LVD_THRESHOLD_DEFAULT_5V_SYSTEM = LVD_THRESHOLD_3_5V  // Rule: 3.5V for 5V system
} t_lvd_thrthresholdLevel;

// UART
typedef enum
{
    UART_CHANNEL_1 = 0,
    UART_CHANNEL_2,
    UART_CHANNEL_6
} t_uart_channel;

typedef enum
{
    UART_BAUD_9600,
    UART_BAUD_115200,
    // Add other common baud rates
} t_uart_baud_rate;

typedef enum
{
    UART_DATA_8BIT,
    UART_DATA_9BIT
} t_uart_data_length;

typedef enum
{
    UART_STOP_1BIT,
    UART_STOP_0_5BIT,
    UART_STOP_2BIT,
    UART_STOP_1_5BIT
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
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum
{
    I2C_CLK_SPEED_STANDARD = 100000, // 100 kHz
    I2C_CLK_SPEED_FAST     = 400000  // 400 kHz
} t_i2c_clk_speed; // Rule: Always use fast mode, will select 400kHz

typedef tbyte t_i2c_device_address; // 7-bit or 10-bit address
typedef enum
{
    I2C_ACK_DISABLE,
    I2C_ACK_ENABLE
} t_i2c_ack;

typedef enum
{
    I2C_DATALENGTH_8BIT,
    I2C_DATALENGTH_16BIT // Not standard for I2C data register, but keeping for API compatibility
} t_i2c_datalength;

// SPI
typedef enum
{
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

typedef enum
{
    SPI_MODE_MASTER,
    SPI_MODE_SLAVE
} t_spi_mode;

typedef enum
{
    SPI_CPOL_LOW,  // Clock Polarity: CPOL=0
    SPI_CPOL_HIGH  // Clock Polarity: CPOL=1
} t_spi_cpol;

typedef enum
{
    SPI_CPHA_1EDGE, // Clock Phase: CPHA=0 (first edge)
    SPI_CPHA_2EDGE  // Clock Phase: CPHA=1 (second edge)
} t_spi_cpha;

typedef enum
{
    SPI_DFF_8BIT,
    SPI_DFF_16BIT
} t_spi_dff;

typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt
typedef enum
{
    EXT_INT_CHANNEL_0 = 0,
    EXT_INT_CHANNEL_1,
    EXT_INT_CHANNEL_2,
    EXT_INT_CHANNEL_3,
    EXT_INT_CHANNEL_4,
    EXT_INT_CHANNEL_5,
    EXT_INT_CHANNEL_6,
    EXT_INT_CHANNEL_7,
    EXT_INT_CHANNEL_8,
    EXT_INT_CHANNEL_9,
    EXT_INT_CHANNEL_10,
    EXT_INT_CHANNEL_11,
    EXT_INT_CHANNEL_12,
    EXT_INT_CHANNEL_13,
    EXT_INT_CHANNEL_14,
    EXT_INT_CHANNEL_15
} t_external_int_channel;

typedef enum
{
    EXT_INT_EDGE_RISING,
    EXT_INT_EDGE_FALLING,
    EXT_INT_EDGE_BOTH
} t_external_int_edge;

// GPIO
typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_H, // STM32F401RC only has GPIOA-E, H
    NUM_GPIO_PORTS
} t_port;

typedef enum
{
    GPIO_PIN_0 = 0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15
} t_pin;

typedef enum
{
    GPIO_DIR_INPUT = 0,
    GPIO_DIR_OUTPUT
} t_direction;

// PWM
typedef enum
{
    PWM_CHANNEL_TIM1_CH1 = 0,  // PA8, PE9
    PWM_CHANNEL_TIM1_CH2,      // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,      // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,      // PA11, PE14
    PWM_CHANNEL_TIM2_CH1,      // PA0, PA5, PA15, PB3
    PWM_CHANNEL_TIM2_CH2,      // PA1, PB3, PB10
    PWM_CHANNEL_TIM2_CH3,      // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,      // PA3, PB11
    PWM_CHANNEL_TIM3_CH1,      // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,      // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,      // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,      // PB1, PC9
    PWM_CHANNEL_TIM4_CH1,      // PB6
    PWM_CHANNEL_TIM4_CH2,      // PB7
    PWM_CHANNEL_TIM4_CH3,      // PB8
    PWM_CHANNEL_TIM4_CH4,      // PB9
    PWM_CHANNEL_TIM5_CH1,      // PA0
    PWM_CHANNEL_TIM5_CH2,      // PA1
    PWM_CHANNEL_TIM5_CH3,      // PA2
    PWM_CHANNEL_TIM5_CH4,      // PA3
    PWM_CHANNEL_TIM9_CH1,      // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,      // PA3, PE6
    PWM_CHANNEL_TIM10_CH1,     // PB8, PA6
    PWM_CHANNEL_TIM11_CH1      // PB9, PA7
} t_pwm_channel;

// ICU
typedef enum
{
    ICU_CHANNEL_TIM1_CH1 = 0,  // PA8, PE9
    ICU_CHANNEL_TIM1_CH2,      // PA9, PE11
    ICU_CHANNEL_TIM1_CH3,      // PA10, PE13
    ICU_CHANNEL_TIM1_CH4,      // PA11, PE14
    ICU_CHANNEL_TIM2_CH1,      // PA0, PA5, PA15, PB3
    ICU_CHANNEL_TIM2_CH2,      // PA1, PB3, PB10
    ICU_CHANNEL_TIM2_CH3,      // PA2, PB10
    ICU_CHANNEL_TIM2_CH4,      // PA3, PB11
    ICU_CHANNEL_TIM3_CH1,      // PA6, PB4, PC6
    ICU_CHANNEL_TIM3_CH2,      // PA7, PB5, PC7
    ICU_CHANNEL_TIM3_CH3,      // PB0, PC8
    ICU_CHANNEL_TIM3_CH4,      // PB1, PC9
    ICU_CHANNEL_TIM4_CH1,      // PB6
    ICU_CHANNEL_TIM4_CH2,      // PB7
    ICU_CHANNEL_TIM4_CH3,      // PB8
    ICU_CHANNEL_TIM4_CH4,      // PB9
    ICU_CHANNEL_TIM5_CH1,      // PA0
    ICU_CHANNEL_TIM5_CH2,      // PA1
    ICU_CHANNEL_TIM5_CH3,      // PA2
    ICU_CHANNEL_TIM5_CH4,      // PA3
    ICU_CHANNEL_TIM9_CH1,      // PA2, PE5
    ICU_CHANNEL_TIM9_CH2,      // PA3, PE6
    ICU_CHANNEL_TIM10_CH1,     // PB8, PA6
    ICU_CHANNEL_TIM11_CH1      // PB9, PA7
} t_icu_channel;

typedef enum
{
    ICU_PRESCALER_DIV1,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8,
    // Add other prescaler values
} t_icu_prescaller;

typedef enum
{
    ICU_EDGE_RISING,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer
typedef enum
{
    TIMER_CHANNEL_1 = 0,  // TIM1
    TIMER_CHANNEL_2,      // TIM2
    TIMER_CHANNEL_3,      // TIM3
    TIMER_CHANNEL_4,      // TIM4
    TIMER_CHANNEL_5,      // TIM5
    TIMER_CHANNEL_9,      // TIM9
    TIMER_CHANNEL_10,     // TIM10
    TIMER_CHANNEL_11      // TIM11
} t_timer_channel;

// ADC
typedef enum
{
    ADC_CHANNEL_0 = 0,  // PA0
    ADC_CHANNEL_1,      // PA1
    ADC_CHANNEL_2,      // PA2
    ADC_CHANNEL_3,      // PA3
    ADC_CHANNEL_4,      // PA4
    ADC_CHANNEL_5,      // PA5
    ADC_CHANNEL_6,      // PA6
    ADC_CHANNEL_7,      // PA7
    ADC_CHANNEL_8,      // PB0
    ADC_CHANNEL_9,      // PB1
    ADC_CHANNEL_10,     // PC0
    ADC_CHANNEL_11,     // PC1
    ADC_CHANNEL_12,     // PC2
    ADC_CHANNEL_13,     // PC3
    ADC_CHANNEL_14,     // PC4
    ADC_CHANNEL_15,     // PC5
    // Add internal channels if needed (Temperature Sensor, VrefInt, Vbat)
} t_adc_channel;

typedef enum
{
    ADC_MODE_SINGLE_CONVERSION,
    ADC_MODE_CONTINUOUS_CONVERSION,
    ADC_MODE_SCAN_CONVERSION
} t_adc_mode_t;

// TT (Time-Triggered OS)
typedef tword t_tick_time; // in ms

// MCAL_OUTPUT_BUZZER
// Buzzer numbers can be mapped to GPIO pins or PWM channels
// For simplicity, assuming a generic single buzzer which can be GPIO controlled.
// If multiple buzzers, would need a mapping array for their pins.

// DAC (Digital to Analog Converter)
// DAC is not supported on this MCU based on available registers.
// Placeholder for API compatibility, but functions will be empty.
typedef enum
{
    DAC_CHANNEL_1 = 0,
    DAC_CHANNEL_2
} dac_channel_t;

// I2S (Inter-IC Sound)
typedef enum
{
    I2S_CHANNEL_1 = 0, // Maps to SPI1 I2S
    I2S_CHANNEL_2,     // Maps to SPI2 I2S
    I2S_CHANNEL_3      // Maps to SPI3 I2S
} t_i2s_channel;

typedef enum
{
    I2S_MODE_SLAVE_TX,
    I2S_MODE_SLAVE_RX,
    I2S_MODE_MASTER_TX,
    I2S_MODE_MASTER_RX
} I2S_Mode_t;

typedef enum
{
    I2S_STANDARD_PHILIPS,
    I2S_STANDARD_MSB,
    I2S_STANDARD_LSB,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

typedef enum
{
    I2S_DATAFORMAT_16B,
    I2S_DATAFORMAT_16B_EXTENDED,
    I2S_DATAFORMAT_24B,
    I2S_DATAFORMAT_32B
} I2S_DataFormat_t;

typedef enum
{
    I2S_CHMODE_STEREO,
    I2S_CHMODE_MONO
} I2S_ChannelMode_t;

// WDT (Watchdog Timer) - Not in API.json, but `WDT_Reset` is in MCU_Config
// Using simple defines for WDT_Init based on common STM32 IWDG setup
// (See MCAL.c for inferred IWDG registers)

// WiFi Driver
// WiFi Driver is not supported on this MCU (STM32F401RC) based on available registers.
// Placeholder for API compatibility, but functions will be empty.
typedef enum
{
    TX_MODE_LOW_POWER,
    TX_MODE_HIGH_PERFORMANCE
} t_tx_mode;

/*
 * =================================================================================================
 *                                       API Prototypes
 * =================================================================================================
 */

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // Implemented as a generic watchdog refresh.
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
// LVD registers are not provided in register_json, typically found in PWR peripheral (e.g., PWR_CR).
// Implementing with dummy operations and comments.
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel); // lvd_thresholdLevel parameter unused in dummy impl.
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
tlong ICU_GetFrequency(t_icu_channel icu_channel);
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

// Internal_EEPROM (Emulated using Flash memory on STM32F401RC)
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time-Triggered OS)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void); // Assumes this is called from a timer interrupt
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// MCAL_OUTPUT_BUZZER (Implemented using GPIO)
void BUZZER_OUTPUT_Init(tbyte buzzer_number); // buzzer_number currently ignored, assumes one generic buzzer
void BUZZER_OUTPUT_Start(tbyte NUMBER_BUZZER); // NUMBER_BUZZER currently ignored
void BUZZER_OUTPUT_Stop(tbyte NUMBER_BUZZER);  // NUMBER_BUZZER currently ignored

// WDT (Watchdog Timer) - WDT_Reset is already covered under MCU_Config
void WDT_Init(void);
// void WDT_Reset(void); // Already declared above.

// DAC not supported on this MCU based on available registers.
// void DAC_Init(dac_channel_t channel);
// void DAC_Enable(dac_channel_t channel);
// void DAC_Disable(dac_channel_t channel);
// void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue);

// I2S
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, t_size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, t_size_t length);

// MQTT Protocol not supported on this MCU based on available registers/typical architecture.
// void MQTT_Init(void);
// void MQTT_Subscribe_Topic(void *handler_args, esp_event_base_t base, tslong event_id, void *event_data);
// void MQTT_Publish_Message(const tsbyte *message);
// void AZURE_IoT_Hub_Client_Init(void);
// void AZURE_Connection_Enable(void);

// HTTP Protocol not supported on this MCU based on available registers/typical architecture.
// void HTTP_Get_Device_ID(char *device_id, size_t size);
// void HTTP_Server_Init(void);
// void HTTP_Server_Start(void);
// void HTTP_Server_Stop(void);
// void HTTP_Reset_SSID_PASSWORD(void);
// esp_err_t HTTP_Config_Handler(httpd_req_t *req);

// WiFi Driver not supported on this MCU based on available registers/typical architecture.
// void WiFi_Init(void);
// void WiFi_Connect(const tsbyte *ssid, const tsbyte *password);
// void WiFi_Enable(void);
// void WiFi_Disable(void);
// void WiFi_SetTxMode(t_tx_mode mode);
// tsword WiFi_Check_Connection(void);
// int WiFi_Check_Internet(void);

// DTC_driver not supported on this MCU based on available registers (DMA is present but no DTC specific).
// void DTC_Init();
// void DTC_EnableSource(uint8_t source_id, uint8_t channel);
// void DTC_DisableSource(uint8_t source_id);
// void DTC_Start(void);
// void DTC_Stop(void);
// void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int);

#endif /* MCAL_H_ */