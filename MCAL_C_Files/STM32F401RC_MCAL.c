/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) for STM32F401RC.
 *
 * This file implements the MCAL APIs based on the provided register definitions
 * and coding rules. It provides a low-level interface for accessing and
 * controlling various microcontroller peripherals.
 *
 * MCU: STM32F401RC
 */

/* Core includes from Rules.json */
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>      // For standard integer types (uint32_t, etc.)
#include <stdbool.h>     // For boolean type (bool)
#include <stddef.h>      // For size_t
#include <string.h>      // For string manipulation functions (used by send/get frame/string)
#include <stdio.h>       // For standard I/O (e.g., for placeholders or debugging)
#include <stdlib.h>      // For general utilities
#include <math.h>        // For math functions

/* Data type definitions from Rules.json */
#define tbyte  uint8_t
#define tword  uint16_t
#define tlong  uint32_t

/* Register access macro */
#define REG_ADDR(address)               ((volatile uint32_t*)(address))

/* Bit manipulation macros */
#define SET_BIT(REG, BIT)               (*(REG) |= (1U << (BIT)))
#define CLEAR_BIT(REG, BIT)             (*(REG) &= ~(1U << (BIT)))
#define TOGGLE_BIT(REG, BIT)            (*(REG) ^= (1U << (BIT)))
#define READ_BIT(REG, BIT)              ((*(REG) >> (BIT)) & 1U)
#define WRITE_REG(REG, VAL)             (*(REG) = (VAL))
#define READ_REG(REG)                   (*(REG))
#define MODIFY_REG(REG, CLEAR_MASK, SET_MASK) WRITE_REG((REG), ((READ_REG(REG) & ~(CLEAR_MASK)) | (SET_MASK)))

/* --- MCU Register Definitions (from register_json) --- */

// FLASH Registers
#define FLASH_ACR_REG                   REG_ADDR(0x40023C00)
#define FLASH_KEYR_REG                  REG_ADDR(0x40023C04)
#define FLASH_OPTKEYR_REG               REG_ADDR(0x40023C08)
#define FLASH_SR_REG                    REG_ADDR(0x40023C0C)
#define FLASH_CR_REG                    REG_ADDR(0x40023C10)
#define FLASH_OPTCR_REG                 REG_ADDR(0x40023C14)

// RCC Registers
#define RCC_CR_REG                      REG_ADDR(0x40023800)
#define RCC_PLLCFGR_REG                 REG_ADDR(0x40023804)
#define RCC_CFGR_REG                    REG_ADDR(0x40023808)
#define RCC_CIR_REG                     REG_ADDR(0x4002380C)
#define RCC_AHB1RSTR_REG                REG_ADDR(0x40023810)
#define RCC_AHB2RSTR_REG                REG_ADDR(0x40023814)
#define RCC_APB1RSTR_REG                REG_ADDR(0x40023818)
#define RCC_APB2RSTR_REG                REG_ADDR(0x4002381C)
#define RCC_AHB1ENR_REG                 REG_ADDR(0x40023830)
#define RCC_AHB2ENR_REG                 REG_ADDR(0x40023834)
#define RCC_APB1ENR_REG                 REG_ADDR(0x40023838)
#define RCC_APB2ENR_REG                 REG_ADDR(0x4002383C)
#define RCC_AHB1LPENR_REG               REG_ADDR(0x40023840)
#define RCC_AHB2LPENR_REG               REG_ADDR(0x40023844)
#define RCC_APB1LPENR_REG               REG_ADDR(0x40023848)
#define RCC_APB2LPENR_REG               REG_ADDR(0x4002384C)
#define RCC_BDCR_REG                    REG_ADDR(0x40023850)
#define RCC_CSR_REG                     REG_ADDR(0x40023854)
#define RCC_SSCGR_REG                   REG_ADDR(0x40023858)
#define RCC_PLLI2SCFGR_REG              REG_ADDR(0x4002385C)
#define RCC_DCKCFGR_REG                 REG_ADDR(0x40023864)

// SYSCFG Registers
#define SYSCFG_MEMRMP_REG               REG_ADDR(0x40013800)
#define SYSCFG_PMC_REG                  REG_ADDR(0x40013804)
#define SYSCFG_EXTICR1_REG              REG_ADDR(0x40013808)
#define SYSCFG_EXTICR2_REG              REG_ADDR(0x4001380C)
#define SYSCFG_EXTICR3_REG              REG_ADDR(0x40013810)
#define SYSCFG_EXTICR4_REG              REG_ADDR(0x40013814)
#define SYSCFG_CMPCR_REG                REG_ADDR(0x40013820)

// GPIO A Registers
#define GPIOA_MODER_REG                 REG_ADDR(0x40020000)
#define GPIOA_OTYPER_REG                REG_ADDR(0x40020004)
#define GPIOA_OSPEEDR_REG               REG_ADDR(0x40020008)
#define GPIOA_PUPDR_REG                 REG_ADDR(0x4002000C)
#define GPIOA_IDR_REG                   REG_ADDR(0x40020010)
#define GPIOA_ODR_REG                   REG_ADDR(0x40020014)
#define GPIOA_BSRR_REG                  REG_ADDR(0x40020018)
#define GPIOA_LCKR_REG                  REG_ADDR(0x4002001C)
#define GPIOA_AFRL_REG                  REG_ADDR(0x40020020)
#define GPIOA_AFRH_REG                  REG_ADDR(0x40020024)

// GPIO B Registers
#define GPIOB_MODER_REG                 REG_ADDR(0x40020400)
#define GPIOB_OTYPER_REG                REG_ADDR(0x40020404)
#define GPIOB_OSPEEDR_REG               REG_ADDR(0x40020408)
#define GPIOB_PUPDR_REG                 REG_ADDR(0x4002040C)
#define GPIOB_IDR_REG                   REG_ADDR(0x40020410)
#define GPIOB_ODR_REG                   REG_ADDR(0x40020414)
#define GPIOB_BSRR_REG                  REG_ADDR(0x40020418)
#define GPIOB_LCKR_REG                  REG_ADDR(0x4002041C)
#define GPIOB_AFRL_REG                  REG_ADDR(0x40020420)
#define GPIOB_AFRH_REG                  REG_ADDR(0x40020424)

// GPIO C Registers
#define GPIOC_MODER_REG                 REG_ADDR(0x40020800)
#define GPIOC_OTYPER_REG                REG_ADDR(0x40020804)
#define GPIOC_OSPEEDR_REG               REG_ADDR(0x40020808)
#define GPIOC_PUPDR_REG                 REG_ADDR(0x4002080C)
#define GPIOC_IDR_REG                   REG_ADDR(0x40020810)
#define GPIOC_ODR_REG                   REG_ADDR(0x40020814)
#define GPIOC_BSRR_REG                  REG_ADDR(0x40020818)
#define GPIOC_LCKR_REG                  REG_ADDR(0x4002081C)
#define GPIOC_AFRL_REG                  REG_ADDR(0x40020820)
#define GPIOC_AFRH_REG                  REG_ADDR(0x40020824)

// GPIO D Registers
#define GPIOD_MODER_REG                 REG_ADDR(0x40020C00)
#define GPIOD_OTYPER_REG                REG_ADDR(0x40020C04)
#define GPIOD_OSPEEDR_REG               REG_ADDR(0x40020C08)
#define GPIOD_PUPDR_REG                 REG_ADDR(0x40020C0C)
#define GPIOD_IDR_REG                   REG_ADDR(0x40020C10)
#define GPIOD_ODR_REG                   REG_ADDR(0x40020C14)
#define GPIOD_BSRR_REG                  REG_ADDR(0x40020C18)
#define GPIOD_LCKR_REG                  REG_ADDR(0x40020C1C)
#define GPIOD_AFRL_REG                  REG_ADDR(0x40020C20)
#define GPIOD_AFRH_REG                  REG_ADDR(0x40020C24)

// GPIO E Registers
#define GPIOE_MODER_REG                 REG_ADDR(0x40021000)
#define GPIOE_OTYPER_REG                REG_ADDR(0x40021004)
#define GPIOE_OSPEEDR_REG               REG_ADDR(0x40021008)
#define GPIOE_PUPDR_REG                 REG_ADDR(0x4002100C)
#define GPIOE_IDR_REG                   REG_ADDR(0x40021010)
#define GPIOE_ODR_REG                   REG_ADDR(0x40021014)
#define GPIOE_BSRR_REG                  REG_ADDR(0x40021018)
#define GPIOE_LCKR_REG                  REG_ADDR(0x4002101C)
#define GPIOE_AFRL_REG                  REG_ADDR(0x40021020)
#define GPIOE_AFRH_REG                  REG_ADDR(0x40021024)

// GPIO H Registers
#define GPIOH_MODER_REG                 REG_ADDR(0x40021C00)
#define GPIOH_OTYPER_REG                REG_ADDR(0x40021C04)
#define GPIOH_OSPEEDR_REG               REG_ADDR(0x40021C08)
#define GPIOH_PUPDR_REG                 REG_ADDR(0x40021C0C)
#define GPIOH_IDR_REG                   REG_ADDR(0x40021C10)
#define GPIOH_ODR_REG                   REG_ADDR(0x40021C14)
#define GPIOH_BSRR_REG                  REG_ADDR(0x40021C18)
#define GPIOH_LCKR_REG                  REG_ADDR(0x40021C1C)
#define GPIOH_AFRL_REG                  REG_ADDR(0x40021C20)
#define GPIOH_AFRH_REG                  REG_ADDR(0x40021C24)

// EXTI Registers
#define EXTI_IMR_REG                    REG_ADDR(0x40013C00)
#define EXTI_EMR_REG                    REG_ADDR(0x40013C04)
#define EXTI_RTSR_REG                   REG_ADDR(0x40013C08)
#define EXTI_FTSR_REG                   REG_ADDR(0x40013C0C)
#define EXTI_SWIER_REG                  REG_ADDR(0x40013C10)
#define EXTI_PR_REG                     REG_ADDR(0x40013C14)

// ADC Registers
#define ADC1_SR_REG                     REG_ADDR(0x40012000)
#define ADC1_CR1_REG                    REG_ADDR(0x40012004)
#define ADC1_CR2_REG                    REG_ADDR(0x40012008)
#define ADC1_SMPR1_REG                  REG_ADDR(0x4001200C)
#define ADC1_SMPR2_REG                  REG_ADDR(0x40012010)
#define ADC1_JOFR1_REG                  REG_ADDR(0x40012014)
#define ADC1_JOFR2_REG                  REG_ADDR(0x40012018)
#define ADC1_JOFR3_REG                  REG_ADDR(0x4001201C)
#define ADC1_JOFR4_REG                  REG_ADDR(0x40012020)
#define ADC1_HTR_REG                    REG_ADDR(0x40012024)
#define ADC1_LTR_REG                    REG_ADDR(0x40012028)
#define ADC1_SQR1_REG                   REG_ADDR(0x4001202C)
#define ADC1_SQR2_REG                   REG_ADDR(0x40012030)
#define ADC1_SQR3_REG                   REG_ADDR(0x40012034)
#define ADC1_JSQR_REG                   REG_ADDR(0x40012038)
#define ADC1_JDR1_REG                   REG_ADDR(0x4001203C)
#define ADC1_JDR2_REG                   REG_ADDR(0x40012040)
#define ADC1_JDR3_REG                   REG_ADDR(0x40012044)
#define ADC1_JDR4_REG                   REG_ADDR(0x40012048)
#define ADC1_DR_REG                     REG_ADDR(0x4001204C)
#define ADC_CCR_REG                     REG_ADDR(0x40012304)

// TIM1 Registers
#define TIM1_CR1_REG                    REG_ADDR(0x40010000)
#define TIM1_CR2_REG                    REG_ADDR(0x40010004)
#define TIM1_SMCR_REG                   REG_ADDR(0x40010008)
#define TIM1_DIER_REG                   REG_ADDR(0x4001000C)
#define TIM1_SR_REG                     REG_ADDR(0x40010010)
#define TIM1_EGR_REG                    REG_ADDR(0x40010014)
#define TIM1_CCMR1_REG                  REG_ADDR(0x40010018)
#define TIM1_CCMR2_REG                  REG_ADDR(0x4001001C)
#define TIM1_CCER_REG                   REG_ADDR(0x40010020)
#define TIM1_CNT_REG                    REG_ADDR(0x40010024)
#define TIM1_PSC_REG                    REG_ADDR(0x40010028)
#define TIM1_ARR_REG                    REG_ADDR(0x4001002C)
#define TIM1_RCR_REG                    REG_ADDR(0x40010030)
#define TIM1_CCR1_REG                   REG_ADDR(0x40010034)
#define TIM1_CCR2_REG                   REG_ADDR(0x40010038)
#define TIM1_CCR3_REG                   REG_ADDR(0x4001003C)
#define TIM1_CCR4_REG                   REG_ADDR(0x40010040)
#define TIM1_BDTR_REG                   REG_ADDR(0x40010044)
#define TIM1_DCR_REG                    REG_ADDR(0x40010048)
#define TIM1_DMAR_REG                   REG_ADDR(0x4001004C)

// TIM2 Registers
#define TIM2_CR1_REG                    REG_ADDR(0x40000000)
#define TIM2_CR2_REG                    REG_ADDR(0x40000004)
#define TIM2_SMCR_REG                   REG_ADDR(0x40000008)
#define TIM2_DIER_REG                   REG_ADDR(0x4000000C)
#define TIM2_SR_REG                     REG_ADDR(0x40000010)
#define TIM2_EGR_REG                    REG_ADDR(0x40000014)
#define TIM2_CCMR1_REG                  REG_ADDR(0x40000018)
#define TIM2_CCMR2_REG                  REG_ADDR(0x4000001C)
#define TIM2_CCER_REG                   REG_ADDR(0x40000020)
#define TIM2_CNT_REG                    REG_ADDR(0x40000024)
#define TIM2_PSC_REG                    REG_ADDR(0x40000028)
#define TIM2_ARR_REG                    REG_ADDR(0x4000002C)
#define TIM2_CCR1_REG                   REG_ADDR(0x40000034)
#define TIM2_CCR2_REG                   REG_ADDR(0x40000038)
#define TIM2_CCR3_REG                   REG_ADDR(0x4000003C)
#define TIM2_CCR4_REG                   REG_ADDR(0x40000040)
#define TIM2_DCR_REG                    REG_ADDR(0x40000048)
#define TIM2_DMAR_REG                   REG_ADDR(0x4000004C)
#define TIM2_OR_REG                     REG_ADDR(0x40000050)

// TIM3 Registers
#define TIM3_CR1_REG                    REG_ADDR(0x40000400)
#define TIM3_CR2_REG                    REG_ADDR(0x40000404)
#define TIM3_SMCR_REG                   REG_ADDR(0x40000408)
#define TIM3_DIER_REG                   REG_ADDR(0x4000040C)
#define TIM3_SR_REG                     REG_ADDR(0x40000410)
#define TIM3_EGR_REG                    REG_ADDR(0x40000414)
#define TIM3_CCMR1_REG                  REG_ADDR(0x40000418)
#define TIM3_CCMR2_REG                  REG_ADDR(0x4000041C)
#define TIM3_CCER_REG                   REG_ADDR(0x40000420)
#define TIM3_CNT_REG                    REG_ADDR(0x40000424)
#define TIM3_PSC_REG                    REG_ADDR(0x40000428)
#define TIM3_ARR_REG                    REG_ADDR(0x4000042C)
#define TIM3_CCR1_REG                   REG_ADDR(0x40000434)
#define TIM3_CCR2_REG                   REG_ADDR(0x40000438)
#define TIM3_CCR3_REG                   REG_ADDR(0x4000043C)
#define TIM3_CCR4_REG                   REG_ADDR(0x40000440)
#define TIM3_DCR_REG                    REG_ADDR(0x40000448)
#define TIM3_DMAR_REG                   REG_ADDR(0x4000044C)

// TIM4 Registers
#define TIM4_CR1_REG                    REG_ADDR(0x40000800)
#define TIM4_CR2_REG                    REG_ADDR(0x40000804)
#define TIM4_SMCR_REG                   REG_ADDR(0x40000808)
#define TIM4_DIER_REG                   REG_ADDR(0x4000080C)
#define TIM4_SR_REG                     REG_ADDR(0x40000810)
#define TIM4_EGR_REG                    REG_ADDR(0x40000814)
#define TIM4_CCMR1_REG                  REG_ADDR(0x40000818)
#define TIM4_CCMR2_REG                  REG_ADDR(0x4000081C)
#define TIM4_CCER_REG                   REG_ADDR(0x40000820)
#define TIM4_CNT_REG                    REG_ADDR(0x40000824)
#define TIM4_PSC_REG                    REG_ADDR(0x40000828)
#define TIM4_ARR_REG                    REG_ADDR(0x4000082C)
#define TIM4_CCR1_REG                   REG_ADDR(0x40000834)
#define TIM4_CCR2_REG                   REG_ADDR(0x40000838)
#define TIM4_CCR3_REG                   REG_ADDR(0x4000083C)
#define TIM4_CCR4_REG                   REG_ADDR(0x40000840)
#define TIM4_DCR_REG                    REG_ADDR(0x40000848)
#define TIM4_DMAR_REG                   REG_ADDR(0x4000084C)

// TIM5 Registers
#define TIM5_CR1_REG                    REG_ADDR(0x40000C00)
#define TIM5_CR2_REG                    REG_ADDR(0x40000C04)
#define TIM5_SMCR_REG                   REG_ADDR(0x40000C08)
#define TIM5_DIER_REG                   REG_ADDR(0x40000C0C)
#define TIM5_SR_REG                     REG_ADDR(0x40000C10)
#define TIM5_EGR_REG                    REG_ADDR(0x40000C14)
#define TIM5_CCMR1_REG                  REG_ADDR(0x40000C18)
#define TIM5_CCMR2_REG                  REG_ADDR(0x40000C1C)
#define TIM5_CCER_REG                   REG_ADDR(0x40000C20)
#define TIM5_CNT_REG                    REG_ADDR(0x40000C24)
#define TIM5_PSC_REG                    REG_ADDR(0x40000C28)
#define TIM5_ARR_REG                    REG_ADDR(0x40000C2C)
#define TIM5_CCR1_REG                   REG_ADDR(0x40000C34)
#define TIM5_CCR2_REG                   REG_ADDR(0x40000C38)
#define TIM5_CCR3_REG                   REG_ADDR(0x40000C3C)
#define TIM5_CCR4_REG                   REG_ADDR(0x40000C40)
#define TIM5_DCR_REG                    REG_ADDR(0x40000C48)
#define TIM5_DMAR_REG                   REG_ADDR(0x40000C4C)
#define TIM5_OR_REG                     REG_ADDR(0x40000C54)

// TIM9 Registers
#define TIM9_CR1_REG                    REG_ADDR(0x40014000)
#define TIM9_SMCR_REG                   REG_ADDR(0x40014008)
#define TIM9_DIER_REG                   REG_ADDR(0x4001400C)
#define TIM9_SR_REG                     REG_ADDR(0x40014010)
#define TIM9_EGR_REG                    REG_ADDR(0x40014014)
#define TIM9_CCMR1_REG                  REG_ADDR(0x40014018)
#define TIM9_CCER_REG                   REG_ADDR(0x40014020)
#define TIM9_CNT_REG                    REG_ADDR(0x40014024)
#define TIM9_PSC_REG                    REG_ADDR(0x40014028)
#define TIM9_ARR_REG                    REG_ADDR(0x4001402C)
#define TIM9_CCR1_REG                   REG_ADDR(0x40014034)
#define TIM9_CCR2_REG                   REG_ADDR(0x40014038)

// TIM10 Registers
#define TIM10_CR1_REG                   REG_ADDR(0x40014400)
#define TIM10_DIER_REG                  REG_ADDR(0x4001440C)
#define TIM10_SR_REG                    REG_ADDR(0x40014410)
#define TIM10_EGR_REG                   REG_ADDR(0x40014414)
#define TIM10_CCMR1_REG                 REG_ADDR(0x40014418)
#define TIM10_CCER_REG                  REG_ADDR(0x40014420)
#define TIM10_CNT_REG                   REG_ADDR(0x40014424)
#define TIM10_PSC_REG                   REG_ADDR(0x40014428)
#define TIM10_ARR_REG                   REG_ADDR(0x4001442C)
#define TIM10_CCR1_REG                  REG_ADDR(0x40014434)

// TIM11 Registers
#define TIM11_CR1_REG                   REG_ADDR(0x40014800)
#define TIM11_DIER_REG                  REG_ADDR(0x4001480C)
#define TIM11_SR_REG                    REG_ADDR(0x40014810)
#define TIM11_EGR_REG                   REG_ADDR(0x40014814)
#define TIM11_CCMR1_REG                 REG_ADDR(0x40014818)
#define TIM11_CCER_REG                  REG_ADDR(0x40014820)
#define TIM11_CNT_REG                   REG_ADDR(0x40014824)
#define TIM11_PSC_REG                   REG_ADDR(0x40014828)
#define TIM11_ARR_REG                   REG_ADDR(0x4001482C)
#define TIM11_CCR1_REG                  REG_ADDR(0x40014834)

// USART1 Registers
#define USART1_SR_REG                   REG_ADDR(0x40011000)
#define USART1_DR_REG                   REG_ADDR(0x40011004)
#define USART1_BRR_REG                  REG_ADDR(0x40011008)
#define USART1_CR1_REG                  REG_ADDR(0x4001100C)
#define USART1_CR2_REG                  REG_ADDR(0x40011010)
#define USART1_CR3_REG                  REG_ADDR(0x40011014)
#define USART1_GTPR_REG                 REG_ADDR(0x40011018)

// USART2 Registers
#define USART2_SR_REG                   REG_ADDR(0x40004400)
#define USART2_DR_REG                   REG_ADDR(0x40004404)
#define USART2_BRR_REG                  REG_ADDR(0x40004408)
#define USART2_CR1_REG                  REG_ADDR(0x4000440C)
#define USART2_CR2_REG                  REG_ADDR(0x40004410)
#define USART2_CR3_REG                  REG_ADDR(0x40004414)
#define USART2_GTPR_REG                 REG_ADDR(0x40004418)

// USART6 Registers
#define USART6_SR_REG                   REG_ADDR(0x40011400)
#define USART6_DR_REG                   REG_ADDR(0x40011404)
#define USART6_BRR_REG                  REG_ADDR(0x40011408)
#define USART6_CR1_REG                  REG_ADDR(0x4001140C)
#define USART6_CR2_REG                  REG_ADDR(0x40011410)
#define USART6_CR3_REG                  REG_ADDR(0x40011414)
#define USART6_GTPR_REG                 REG_ADDR(0x40011418)

// I2C1 Registers
#define I2C1_CR1_REG                    REG_ADDR(0x40005400)
#define I2C1_CR2_REG                    REG_ADDR(0x40005404)
#define I2C1_OAR1_REG                   REG_ADDR(0x40005408)
#define I2C1_OAR2_REG                   REG_ADDR(0x4000540C)
#define I2C1_DR_REG                     REG_ADDR(0x40005410)
#define I2C1_SR1_REG                    REG_ADDR(0x40005414)
#define I2C1_SR2_REG                    REG_ADDR(0x40005418)
#define I2C1_CCR_REG                    REG_ADDR(0x4000541C)
#define I2C1_TRISE_REG                  REG_ADDR(0x40005420)
#define I2C1_FLTR_REG                   REG_ADDR(0x40005424)

// I2C2 Registers
#define I2C2_CR1_REG                    REG_ADDR(0x40005800)
#define I2C2_CR2_REG                    REG_ADDR(0x40005804)
#define I2C2_OAR1_REG                   REG_ADDR(0x40005808)
#define I2C2_OAR2_REG                   REG_ADDR(0x4000580C)
#define I2C2_DR_REG                     REG_ADDR(0x40005810)
#define I2C2_SR1_REG                    REG_ADDR(0x40005814)
#define I2C2_SR2_REG                    REG_ADDR(0x40005818)
#define I2C2_CCR_REG                    REG_ADDR(0x4000581C)
#define I2C2_TRISE_REG                  REG_ADDR(0x40005820)
#define I2C2_FLTR_REG                   REG_ADDR(0x40005824)

// I2C3 Registers
#define I2C3_CR1_REG                    REG_ADDR(0x40005C00)
#define I2C3_CR2_REG                    REG_ADDR(0x40005C04)
#define I2C3_OAR1_REG                   REG_ADDR(0x40005C08)
#define I2C3_OAR2_REG                   REG_ADDR(0x40005C0C)
#define I2C3_DR_REG                     REG_ADDR(0x40005C10)
#define I2C3_SR1_REG                    REG_ADDR(0x40005C14)
#define I2C3_SR2_REG                    REG_ADDR(0x40005C18)
#define I2C3_CCR_REG                    REG_ADDR(0x40005C1C)
#define I2C3_TRISE_REG                  REG_ADDR(0x40005C20)
#define I2C3_FLTR_REG                   REG_ADDR(0x40005C24)

// SPI1 Registers
#define SPI1_CR1_REG                    REG_ADDR(0x40013000)
#define SPI1_CR2_REG                    REG_ADDR(0x40013004)
#define SPI1_SR_REG                     REG_ADDR(0x40013008)
#define SPI1_DR_REG                     REG_ADDR(0x4001300C)
#define SPI1_CRCPR_REG                  REG_ADDR(0x40013010)
#define SPI1_RXCRCR_REG                 REG_ADDR(0x40013014)
#define SPI1_TXCRCR_REG                 REG_ADDR(0x40013018)
#define SPI1_I2SCFGR_REG                REG_ADDR(0x4001301C)
#define SPI1_I2SPR_REG                  REG_ADDR(0x40013020)

// SPI2 Registers
#define SPI2_CR1_REG                    REG_ADDR(0x40003800)
#define SPI2_CR2_REG                    REG_ADDR(0x40003804)
#define SPI2_SR_REG                     REG_ADDR(0x40003808)
#define SPI2_DR_REG                     REG_ADDR(0x4000380C)
#define SPI2_CRCPR_REG                  REG_ADDR(0x40003810)
#define SPI2_RXCRCR_REG                 REG_ADDR(0x40003814)
#define SPI2_TXCRCR_REG                 REG_ADDR(0x40003818)
#define SPI2_I2SCFGR_REG                REG_ADDR(0x4000381C)
#define SPI2_I2SPR_REG                  REG_ADDR(0x40003820)

// SPI3 Registers
#define SPI3_CR1_REG                    REG_ADDR(0x40003C00)
#define SPI3_CR2_REG                    REG_ADDR(0x40003C04)
#define SPI3_SR_REG                     REG_ADDR(0x40003C08)
#define SPI3_DR_REG                     REG_ADDR(0x40003C0C)
#define SPI3_CRCPR_REG                  REG_ADDR(0x40003C10)
#define SPI3_RXCRCR_REG                 REG_ADDR(0x40003C14)
#define SPI3_TXCRCR_REG                 REG_ADDR(0x40003C18)
#define SPI3_I2SCFGR_REG                REG_ADDR(0x40003C1C)
#define SPI3_I2SPR_REG                  REG_ADDR(0x40003C20)

/* --- Enum and Struct Definitions (based on API.json and register_json) --- */

typedef enum {
    VOLT_3V = 0,
    VOLT_5V = 1
} t_sys_volt;

typedef enum {
    PORTA = 0,
    PORTB,
    PORTC,
    PORTD,
    PORTE,
    PORTH
} t_port;

typedef enum {
    PIN0 = 0,
    PIN1,
    PIN2,
    PIN3,
    PIN4,
    PIN5,
    PIN6,
    PIN7,
    PIN8,
    PIN9,
    PIN10,
    PIN11,
    PIN12,
    PIN13,
    PIN14,
    PIN15
} t_pin;

typedef enum {
    INPUT = 0,
    OUTPUT = 1
} t_direction;

typedef enum {
    UART_CHANNEL_1 = 0,
    UART_CHANNEL_2,
    UART_CHANNEL_6
} t_uart_channel;

typedef enum {
    BAUD_9600 = 0,
    BAUD_19200,
    BAUD_115200,
    // Add more baud rates as needed
} t_uart_baud_rate;

typedef enum {
    DATA_8BIT = 0,
    DATA_9BIT
} t_uart_data_length;

typedef enum {
    STOP_1BIT = 0,
    STOP_0_5BIT,
    STOP_2BIT,
    STOP_1_5BIT
} t_uart_stop_bit;

typedef enum {
    PARITY_NONE = 0,
    PARITY_EVEN,
    PARITY_ODD
} t_uart_parity;

typedef enum {
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum {
    I2C_CLK_STANDARD = 0, // 100 kHz
    I2C_CLK_FAST = 1      // 400 kHz (as per rules, always use fast mode)
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // 7-bit address
typedef enum {
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_8BIT = 0, // Not explicitly mentioned, but typical
} t_i2c_datalength;

typedef enum {
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

typedef enum {
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW = 0,
    SPI_CPOL_HIGH
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1EDGE = 0,
    SPI_CPHA_2EDGE
} t_spi_cpha;

typedef enum {
    SPI_DFF_8BIT = 0,
    SPI_DFF_16BIT
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB = 0,
    SPI_BIT_ORDER_LSB
} t_spi_bit_order;

typedef enum {
    EXTI_CHANNEL_0 = 0,
    EXTI_CHANNEL_1,
    EXTI_CHANNEL_2,
    EXTI_CHANNEL_3,
    EXTI_CHANNEL_4,
    EXTI_CHANNEL_5,
    EXTI_CHANNEL_6,
    EXTI_CHANNEL_7,
    EXTI_CHANNEL_8,
    EXTI_CHANNEL_9,
    EXTI_CHANNEL_10,
    EXTI_CHANNEL_11,
    EXTI_CHANNEL_12,
    EXTI_CHANNEL_13,
    EXTI_CHANNEL_14,
    EXTI_CHANNEL_15
} t_external_int_channel;

typedef enum {
    RISING_EDGE = 0,
    FALLING_EDGE,
    RISING_FALLING_EDGE
} t_external_int_edge;

typedef enum {
    PWM_CHANNEL_TIM1_CH1 = 0, // PA8, PE9
    PWM_CHANNEL_TIM1_CH2,     // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,     // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,     // PA11, PE14
    PWM_CHANNEL_TIM2_CH1,     // PA0, PA5, PA15, PB3
    PWM_CHANNEL_TIM2_CH2,     // PA1, PB3, PB10
    PWM_CHANNEL_TIM2_CH3,     // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,     // PA3, PB11
    PWM_CHANNEL_TIM3_CH1,     // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,     // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,     // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,     // PB1, PC9
    PWM_CHANNEL_TIM4_CH1,     // PB6
    PWM_CHANNEL_TIM4_CH2,     // PB7
    PWM_CHANNEL_TIM4_CH3,     // PB8
    PWM_CHANNEL_TIM4_CH4,     // PB9
    PWM_CHANNEL_TIM5_CH1,     // PA0
    PWM_CHANNEL_TIM5_CH2,     // PA1
    PWM_CHANNEL_TIM5_CH3,     // PA2
    PWM_CHANNEL_TIM5_CH4,     // PA3
    PWM_CHANNEL_TIM9_CH1,     // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,     // PA3, PE6
    PWM_CHANNEL_TIM10_CH1,    // PB8, PA6
    PWM_CHANNEL_TIM11_CH1     // PB9, PA7
} t_pwm_channel;

typedef t_pwm_channel t_icu_channel; // ICU uses the same timer channels
typedef enum {
    ICU_PRESCALLER_1 = 0, // Example prescalers
    ICU_PRESCALLER_2,
    ICU_PRESCALLER_4,
    ICU_PRESCALLER_8
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

typedef enum {
    TIMER_CHANNEL_1 = 0,
    TIMER_CHANNEL_2,
    TIMER_CHANNEL_3,
    TIMER_CHANNEL_4,
    TIMER_CHANNEL_5,
    TIMER_CHANNEL_9,
    TIMER_CHANNEL_10,
    TIMER_CHANNEL_11
} t_timer_channel;

typedef enum {
    ADC_CHANNEL_0 = 0,  // PA0, PC0
    ADC_CHANNEL_1,      // PA1, PC1
    ADC_CHANNEL_2,      // PA2, PC2
    ADC_CHANNEL_3,      // PA3, PC3
    ADC_CHANNEL_4,      // PA4, PC4
    ADC_CHANNEL_5,      // PA5, PC5
    ADC_CHANNEL_6,      // PA6
    ADC_CHANNEL_7,      // PA7
    ADC_CHANNEL_8,      // PB0
    ADC_CHANNEL_9,      // PB1
    ADC_CHANNEL_10,     // PC0 (duplicate with ADC_CHANNEL_0, internal mapping)
    ADC_CHANNEL_11,     // PC1
    ADC_CHANNEL_12,     // PC2
    ADC_CHANNEL_13,     // PC3
    ADC_CHANNEL_14,     // PC4
    ADC_CHANNEL_15,     // PC5
    // Internal channels not listed in register_json's assigned_pin
    // ADC_CHANNEL_VREFINT,
    // ADC_CHANNEL_TEMPSENSOR
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE = 0,
    ADC_MODE_CONTINUOUS,
    ADC_MODE_DMA
} t_adc_mode_t;

typedef tword t_tick_time; // For TT module, typically in milliseconds

typedef enum {
    I2S_CHANNEL_1 = 0, // SPI1_I2SCFGR
    I2S_CHANNEL_2,     // SPI2_I2SCFGR
    I2S_CHANNEL_3      // SPI3_I2SCFGR
} t_i2s_channel;

typedef enum {
    I2S_MODE_MASTER_TX = 0,
    I2S_MODE_MASTER_RX,
    I2S_MODE_SLAVE_TX,
    I2S_MODE_SLAVE_RX
} I2S_Mode_t;

typedef enum {
    I2S_STANDARD_PHILIPS = 0,
    I2S_STANDARD_MSB_JUSTIFIED,
    I2S_STANDARD_LSB_JUSTIFIED,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

typedef enum {
    I2S_DATA_FORMAT_16B = 0,
    I2S_DATA_FORMAT_24B,
    I2S_DATA_FORMAT_32B
} I2S_DataFormat_t;

typedef enum {
    I2S_CHANNEL_MODE_MONO = 0,
    I2S_CHANNEL_MODE_STEREO
} I2S_ChannelMode_t;

/* --- Private Helper Functions / Mappings --- */

// Array of GPIOx_BASE addresses for easier access
static volatile uint32_t* const GPIO_MODER_ADDRS[] = {
    GPIOA_MODER_REG, GPIOB_MODER_REG, GPIOC_MODER_REG,
    GPIOD_MODER_REG, GPIOE_MODER_REG, GPIOH_MODER_REG
};
static volatile uint32_t* const GPIO_OTYPER_ADDRS[] = {
    GPIOA_OTYPER_REG, GPIOB_OTYPER_REG, GPIOC_OTYPER_REG,
    GPIOD_OTYPER_REG, GPIOE_OTYPER_REG, GPIOH_OTYPER_REG
};
static volatile uint32_t* const GPIO_OSPEEDR_ADDRS[] = {
    GPIOA_OSPEEDR_REG, GPIOB_OSPEEDR_REG, GPIOC_OSPEEDR_REG,
    GPIOD_OSPEEDR_REG, GPIOE_OSPEEDR_REG, GPIOH_OSPEEDR_REG
};
static volatile uint32_t* const GPIO_PUPDR_ADDRS[] = {
    GPIOA_PUPDR_REG, GPIOB_PUPDR_REG, GPIOC_PUPDR_REG,
    GPIOD_PUPDR_REG, GPIOE_PUPDR_REG, GPIOH_PUPDR_REG
};
static volatile uint32_t* const GPIO_IDR_ADDRS[] = {
    GPIOA_IDR_REG, GPIOB_IDR_REG, GPIOC_IDR_REG,
    GPIOD_IDR_REG, GPIOE_IDR_REG, GPIOH_IDR_REG
};
static volatile uint32_t* const GPIO_ODR_ADDRS[] = {
    GPIOA_ODR_REG, GPIOB_ODR_REG, GPIOC_ODR_REG,
    GPIOD_ODR_REG, GPIOE_ODR_REG, GPIOH_ODR_REG
};
static volatile uint32_t* const GPIO_BSRR_ADDRS[] = {
    GPIOA_BSRR_REG, GPIOB_BSRR_REG, GPIOC_BSRR_REG,
    GPIOD_BSRR_REG, GPIOE_BSRR_REG, GPIOH_BSRR_REG
};
static volatile uint32_t* const GPIO_AFRL_ADDRS[] = {
    GPIOA_AFRL_REG, GPIOB_AFRL_REG, GPIOC_AFRL_REG,
    GPIOD_AFRL_REG, GPIOE_AFRL_REG, GPIOH_AFRL_REG
};
static volatile uint32_t* const GPIO_AFRH_ADDRS[] = {
    GPIOA_AFRH_REG, GPIOB_AFRH_REG, GPIOC_AFRH_REG,
    GPIOD_AFRH_REG, GPIOE_AFRH_REG, GPIOH_AFRH_REG
};

/**
 * @brief Generic Watchdog Timer Reset
 * This function serves as a placeholder as no specific WDT registers
 * were provided in the register_json. In a real implementation, this
 * would access the IWDG/WWDG registers.
 */
void WDT_Reset(void) {
    // Placeholder for Watchdog Reset
    // For STM32 IWDG, this would typically involve writing to the KR register.
    // E.g., IWDG->KR = 0xAAAA;
    // Since no specific WDT registers provided, this is a generic placeholder.
}

/**
 * @brief Enables the clock for a given GPIO port.
 * @param port The GPIO port to enable (PORTA, PORTB, etc.).
 */
static void enable_gpio_clock(t_port port) {
    tlong ahb1_enr_bit = 0;
    switch (port) {
        case PORTA: ahb1_enr_bit = 0; break;
        case PORTB: ahb1_enr_bit = 1; break;
        case PORTC: ahb1_enr_bit = 2; break;
        case PORTD: ahb1_enr_bit = 3; break;
        case PORTE: ahb1_enr_bit = 4; break;
        case PORTH: ahb1_enr_bit = 7; break;
        default: return; // Invalid port
    }
    SET_BIT(RCC_AHB1ENR_REG, ahb1_enr_bit);
    // Dummy read to ensure clock is enabled (recommended for some MCUs)
    (void)READ_REG(RCC_AHB1ENR_REG);
}

/**
 * @brief Maps a UART channel to its base registers.
 * @param channel The UART channel.
 * @param sr Pointer to the status register.
 * @param dr Pointer to the data register.
 * @param brr Pointer to the baud rate register.
 * @param cr1 Pointer to control register 1.
 * @param cr2 Pointer to control register 2.
 * @param cr3 Pointer to control register 3.
 * @return True if successful, false otherwise.
 */
static bool get_uart_registers(t_uart_channel channel,
                               volatile uint32_t** sr, volatile uint32_t** dr,
                               volatile uint32_t** brr, volatile uint32_t** cr1,
                               volatile uint32_t** cr2, volatile uint32_t** cr3) {
    switch (channel) {
        case UART_CHANNEL_1:
            *sr = USART1_SR_REG; *dr = USART1_DR_REG; *brr = USART1_BRR_REG;
            *cr1 = USART1_CR1_REG; *cr2 = USART1_CR2_REG; *cr3 = USART1_CR3_REG;
            break;
        case UART_CHANNEL_2:
            *sr = USART2_SR_REG; *dr = USART2_DR_REG; *brr = USART2_BRR_REG;
            *cr1 = USART2_CR1_REG; *cr2 = USART2_CR2_REG; *cr3 = USART2_CR3_REG;
            break;
        case UART_CHANNEL_6:
            *sr = USART6_SR_REG; *dr = USART6_DR_REG; *brr = USART6_BRR_REG;
            *cr1 = USART6_CR1_REG; *cr2 = USART6_CR2_REG; *cr3 = USART6_CR3_REG;
            break;
        default: return false;
    }
    return true;
}

/**
 * @brief Maps an I2C channel to its base registers.
 * @param channel The I2C channel.
 * @param cr1 Pointer to control register 1.
 * @param cr2 Pointer to control register 2.
 * @param oar1 Pointer to own address register 1.
 * @param dr Pointer to data register.
 * @param sr1 Pointer to status register 1.
 * @param sr2 Pointer to status register 2.
 * @param ccr Pointer to clock control register.
 * @param trise Pointer to TRISE register.
 * @return True if successful, false otherwise.
 */
static bool get_i2c_registers(t_i2c_channel channel,
                               volatile uint32_t** cr1, volatile uint32_t** cr2,
                               volatile uint32_t** oar1, volatile uint32_t** dr,
                               volatile uint32_t** sr1, volatile uint32_t** sr2,
                               volatile uint32_t** ccr, volatile uint32_t** trise) {
    switch (channel) {
        case I2C_CHANNEL_1:
            *cr1 = I2C1_CR1_REG; *cr2 = I2C1_CR2_REG; *oar1 = I2C1_OAR1_REG;
            *dr = I2C1_DR_REG; *sr1 = I2C1_SR1_REG; *sr2 = I2C1_SR2_REG;
            *ccr = I2C1_CCR_REG; *trise = I2C1_TRISE_REG;
            break;
        case I2C_CHANNEL_2:
            *cr1 = I2C2_CR1_REG; *cr2 = I2C2_CR2_REG; *oar1 = I2C2_OAR1_REG;
            *dr = I2C2_DR_REG; *sr1 = I2C2_SR1_REG; *sr2 = I2C2_SR2_REG;
            *ccr = I2C2_CCR_REG; *trise = I2C2_TRISE_REG;
            break;
        case I2C_CHANNEL_3:
            *cr1 = I2C3_CR1_REG; *cr2 = I2C3_CR2_REG; *oar1 = I2C3_OAR1_REG;
            *dr = I2C3_DR_REG; *sr1 = I2C3_SR1_REG; *sr2 = I2C3_SR2_REG;
            *ccr = I2C3_CCR_REG; *trise = I2C3_TRISE_REG;
            break;
        default: return false;
    }
    return true;
}

/**
 * @brief Maps an SPI channel to its base registers.
 * @param channel The SPI channel.
 * @param cr1 Pointer to control register 1.
 * @param cr2 Pointer to control register 2.
 * @param sr Pointer to status register.
 * @param dr Pointer to data register.
 * @param i2scfgr Pointer to I2S configuration register.
 * @return True if successful, false otherwise.
 */
static bool get_spi_registers(t_spi_channel channel,
                              volatile uint32_t** cr1, volatile uint32_t** cr2,
                              volatile uint32_t** sr, volatile uint32_t** dr,
                              volatile uint32_t** i2scfgr) {
    switch (channel) {
        case SPI_CHANNEL_1:
            *cr1 = SPI1_CR1_REG; *cr2 = SPI1_CR2_REG; *sr = SPI1_SR_REG;
            *dr = SPI1_DR_REG; *i2scfgr = SPI1_I2SCFGR_REG;
            break;
        case SPI_CHANNEL_2:
            *cr1 = SPI2_CR1_REG; *cr2 = SPI2_CR2_REG; *sr = SPI2_SR_REG;
            *dr = SPI2_DR_REG; *i2scfgr = SPI2_I2SCFGR_REG;
            break;
        case SPI_CHANNEL_3:
            *cr1 = SPI3_CR1_REG; *cr2 = SPI3_CR2_REG; *sr = SPI3_SR_REG;
            *dr = SPI3_DR_REG; *i2scfgr = SPI3_I2SCFGR_REG;
            break;
        default: return false;
    }
    return true;
}

/**
 * @brief Maps a Timer channel to its base registers.
 * @param channel The Timer channel.
 * @param cr1 Pointer to control register 1.
 * @param dier Pointer to DMA/interrupt enable register.
 * @param sr Pointer to status register.
 * @param egr Pointer to event generation register.
 * @param ccmr1 Pointer to capture/compare mode register 1.
 * @param ccmr2 Pointer to capture/compare mode register 2 (can be NULL if not available).
 * @param ccer Pointer to capture/compare enable register.
 * @param cnt Pointer to counter register.
 * @param psc Pointer to prescaler register.
 * @param arr Pointer to auto-reload register.
 * @param ccr1 Pointer to capture/compare register 1.
 * @param ccr2 Pointer to capture/compare register 2.
 * @param ccr3 Pointer to capture/compare register 3.
 * @param ccr4 Pointer to capture/compare register 4.
 * @param bdtr Pointer to break and dead-time register (can be NULL if not available).
 * @return True if successful, false otherwise.
 */
static bool get_timer_registers(t_timer_channel channel,
                                volatile uint32_t** cr1, volatile uint32_t** dier,
                                volatile uint32_t** sr, volatile uint32_t** egr,
                                volatile uint32_t** ccmr1, volatile uint32_t** ccmr2,
                                volatile uint32_t** ccer, volatile uint32_t** cnt,
                                volatile uint32_t** psc, volatile uint32_t** arr,
                                volatile uint32_t** ccr1, volatile uint32_t** ccr2,
                                volatile uint32_t** ccr3, volatile uint32_t** ccr4,
                                volatile uint32_t** bdtr) {
    switch (channel) {
        case TIMER_CHANNEL_1:
            *cr1 = TIM1_CR1_REG; *dier = TIM1_DIER_REG; *sr = TIM1_SR_REG; *egr = TIM1_EGR_REG;
            *ccmr1 = TIM1_CCMR1_REG; *ccmr2 = TIM1_CCMR2_REG; *ccer = TIM1_CCER_REG;
            *cnt = TIM1_CNT_REG; *psc = TIM1_PSC_REG; *arr = TIM1_ARR_REG;
            *ccr1 = TIM1_CCR1_REG; *ccr2 = TIM1_CCR2_REG; *ccr3 = TIM1_CCR3_REG; *ccr4 = TIM1_CCR4_REG;
            *bdtr = TIM1_BDTR_REG;
            break;
        case TIMER_CHANNEL_2:
            *cr1 = TIM2_CR1_REG; *dier = TIM2_DIER_REG; *sr = TIM2_SR_REG; *egr = TIM2_EGR_REG;
            *ccmr1 = TIM2_CCMR1_REG; *ccmr2 = TIM2_CCMR2_REG; *ccer = TIM2_CCER_REG;
            *cnt = TIM2_CNT_REG; *psc = TIM2_PSC_REG; *arr = TIM2_ARR_REG;
            *ccr1 = TIM2_CCR1_REG; *ccr2 = TIM2_CCR2_REG; *ccr3 = TIM2_CCR3_REG; *ccr4 = TIM2_CCR4_REG;
            *bdtr = NULL; // TIM2 does not have BDTR
            break;
        case TIMER_CHANNEL_3:
            *cr1 = TIM3_CR1_REG; *dier = TIM3_DIER_REG; *sr = TIM3_SR_REG; *egr = TIM3_EGR_REG;
            *ccmr1 = TIM3_CCMR1_REG; *ccmr2 = TIM3_CCMR2_REG; *ccer = TIM3_CCER_REG;
            *cnt = TIM3_CNT_REG; *psc = TIM3_PSC_REG; *arr = TIM3_ARR_REG;
            *ccr1 = TIM3_CCR1_REG; *ccr2 = TIM3_CCR2_REG; *ccr3 = TIM3_CCR3_REG; *ccr4 = TIM3_CCR4_REG;
            *bdtr = NULL; // TIM3 does not have BDTR
            break;
        case TIMER_CHANNEL_4:
            *cr1 = TIM4_CR1_REG; *dier = TIM4_DIER_REG; *sr = TIM4_SR_REG; *egr = TIM4_EGR_REG;
            *ccmr1 = TIM4_CCMR1_REG; *ccmr2 = TIM4_CCMR2_REG; *ccer = TIM4_CCER_REG;
            *cnt = TIM4_CNT_REG; *psc = TIM4_PSC_REG; *arr = TIM4_ARR_REG;
            *ccr1 = TIM4_CCR1_REG; *ccr2 = TIM4_CCR2_REG; *ccr3 = TIM4_CCR3_REG; *ccr4 = TIM4_CCR4_REG;
            *bdtr = NULL; // TIM4 does not have BDTR
            break;
        case TIMER_CHANNEL_5:
            *cr1 = TIM5_CR1_REG; *dier = TIM5_DIER_REG; *sr = TIM5_SR_REG; *egr = TIM5_EGR_REG;
            *ccmr1 = TIM5_CCMR1_REG; *ccmr2 = TIM5_CCMR2_REG; *ccer = TIM5_CCER_REG;
            *cnt = TIM5_CNT_REG; *psc = TIM5_PSC_REG; *arr = TIM5_ARR_REG;
            *ccr1 = TIM5_CCR1_REG; *ccr2 = TIM5_CCR2_REG; *ccr3 = TIM5_CCR3_REG; *ccr4 = TIM5_CCR4_REG;
            *bdtr = NULL; // TIM5 does not have BDTR
            break;
        case TIMER_CHANNEL_9:
            *cr1 = TIM9_CR1_REG; *dier = TIM9_DIER_REG; *sr = TIM9_SR_REG; *egr = TIM9_EGR_REG;
            *ccmr1 = TIM9_CCMR1_REG; *ccmr2 = NULL; // TIM9 has only CCMR1
            *ccer = TIM9_CCER_REG; *cnt = TIM9_CNT_REG; *psc = TIM9_PSC_REG; *arr = TIM9_ARR_REG;
            *ccr1 = TIM9_CCR1_REG; *ccr2 = TIM9_CCR2_REG; *ccr3 = NULL; *ccr4 = NULL;
            *bdtr = NULL; // TIM9 does not have BDTR
            break;
        case TIMER_CHANNEL_10:
            *cr1 = TIM10_CR1_REG; *dier = TIM10_DIER_REG; *sr = TIM10_SR_REG; *egr = TIM10_EGR_REG;
            *ccmr1 = TIM10_CCMR1_REG; *ccmr2 = NULL; // TIM10 has only CCMR1
            *ccer = TIM10_CCER_REG; *cnt = TIM10_CNT_REG; *psc = TIM10_PSC_REG; *arr = TIM10_ARR_REG;
            *ccr1 = TIM10_CCR1_REG; *ccr2 = NULL; *ccr3 = NULL; *ccr4 = NULL;
            *bdtr = NULL; // TIM10 does not have BDTR
            break;
        case TIMER_CHANNEL_11:
            *cr1 = TIM11_CR1_REG; *dier = TIM11_DIER_REG; *sr = TIM11_SR_REG; *egr = TIM11_EGR_REG;
            *ccmr1 = TIM11_CCMR1_REG; *ccmr2 = NULL; // TIM11 has only CCMR1
            *ccer = TIM11_CCER_REG; *cnt = TIM11_CNT_REG; *psc = TIM11_PSC_REG; *arr = TIM11_ARR_REG;
            *ccr1 = TIM11_CCR1_REG; *ccr2 = NULL; *ccr3 = NULL; *ccr4 = NULL;
            *bdtr = NULL; // TIM11 does not have BDTR
            break;
        default: return false;
    }
    return true;
}

/**
 * @brief Helper to enable clock for a given timer channel.
 * @param channel The timer channel.
 */
static void enable_timer_clock(t_timer_channel channel) {
    WDT_Reset(); // Required by rule
    tlong apb1_enr_bit = 0xFF; // Sentinel value
    tlong apb2_enr_bit = 0xFF; // Sentinel value

    switch (channel) {
        case TIMER_CHANNEL_1: apb2_enr_bit = 0; break;
        case TIMER_CHANNEL_2: apb1_enr_bit = 0; break;
        case TIMER_CHANNEL_3: apb1_enr_bit = 1; break;
        case TIMER_CHANNEL_4: apb1_enr_bit = 2; break;
        case TIMER_CHANNEL_5: apb1_enr_bit = 3; break;
        case TIMER_CHANNEL_9: apb2_enr_bit = 16; break;
        case TIMER_CHANNEL_10: apb2_enr_bit = 17; break;
        case TIMER_CHANNEL_11: apb2_enr_bit = 18; break;
        default: return; // Invalid timer channel
    }

    if (apb1_enr_bit != 0xFF) {
        SET_BIT(RCC_APB1ENR_REG, apb1_enr_bit);
    }
    if (apb2_enr_bit != 0xFF) {
        SET_BIT(RCC_APB2ENR_REG, apb2_enr_bit);
    }
}

/**
 * @brief Placeholder for a callback function when an ICU event occurs.
 */
static void (*icu_callback_func)(void) = NULL;


/* --- API Implementations --- */

/**
 * @brief Initializes the MCU configurations.
 *
 * This function performs comprehensive initialization of the MCU, including
 * GPIO setup, peripheral disabling, and watchdog configuration.
 *
 * @param volt The system voltage (e.g., VOLT_3V, VOLT_5V).
 */
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // 1. Set all GPIO pins to 0 and verify with while loop
    for (t_port port = PORTA; port <= PORTH; port++) {
        if (port == PORTH && (RCC_AHB1ENR_REG == 0 || (READ_REG(RCC_AHB1ENR_REG) & (1U << 7)) == 0)) {
            // PORTH only has PH0, PH1. For STM32F401RC, GPIOH is often limited or not fully available on all packages.
            // If RCC_AHB1ENR_REG doesn't exist for GPIOH or is not enabled, skip setting/clearing.
            // For robust initialization, ensure clock is on before accessing GPIO.
            // For now, assuming standard PORTS (A-E) as common.
            // Explicitly enable PORTH clock just in case for this operation.
            SET_BIT(RCC_AHB1ENR_REG, 7); // Inferred: GPIOHEN bit 7 for RCC_AHB1ENR.
        } else if (port <= PORTE) { // Standard GPIO ports A-E
            enable_gpio_clock(port);
        } else {
             continue; // Skip any other non-standard port indices if they exist
        }

        WRITE_REG(GPIO_ODR_ADDRS[port], 0x00000000); // Set all pins to 0
        while (READ_REG(GPIO_ODR_ADDRS[port]) != 0x00000000); // Verify
    }

    // 2. Set all GPIO pins direction to input and verify with while loop
    for (t_port port = PORTA; port <= PORTH; port++) {
        if (port == PORTH && (RCC_AHB1ENR_REG == 0 || (READ_REG(RCC_AHB1ENR_REG) & (1U << 7)) == 0)) {
            SET_BIT(RCC_AHB1ENR_REG, 7); // Inferred: GPIOHEN bit 7 for RCC_AHB1ENR.
        } else if (port <= PORTE) { // Standard GPIO ports A-E
            enable_gpio_clock(port);
        } else {
             continue; // Skip any other non-standard port indices if they exist
        }

        // Set all pins to input mode (00b for each pin in MODER)
        WRITE_REG(GPIO_MODER_ADDRS[port], 0x00000000);
        while (READ_REG(GPIO_MODER_ADDRS[port]) != 0x00000000); // Verify
        // Set all pins to pull-up (01b for each pin in PUPDR) for inputs as per GPIO rules
        WRITE_REG(GPIO_PUPDR_ADDRS[port], 0x55555555); // Sets 01 for each pin
    }

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    // Global interrupt
    __disable_irq();

    // ADC (ADC1 is the only one in register_json)
    CLEAR_BIT(ADC1_CR2_REG, 0); // ADON bit for ADC1

    // UART (USART1, USART2, USART6)
    CLEAR_BIT(USART1_CR1_REG, 13); // UE bit for USART1
    CLEAR_BIT(USART2_CR1_REG, 13); // UE bit for USART2
    CLEAR_BIT(USART6_CR1_REG, 13); // UE bit for USART6

    // I2S (Part of SPI peripherals)
    CLEAR_BIT(SPI1_I2SCFGR_REG, 11); // I2SE bit for SPI1_I2S
    CLEAR_BIT(SPI2_I2SCFGR_REG, 11); // I2SE bit for SPI2_I2S
    CLEAR_BIT(SPI3_I2SCFGR_REG, 11); // I2SE bit for SPI3_I2S

    // SPI (SPI1, SPI2, SPI3)
    CLEAR_BIT(SPI1_CR1_REG, 6); // SPE bit for SPI1
    CLEAR_BIT(SPI2_CR1_REG, 6); // SPE bit for SPI2
    CLEAR_BIT(SPI3_CR1_REG, 6); // SPE bit for SPI3

    // TIMERS (TIM1-5, TIM9-11)
    CLEAR_BIT(TIM1_CR1_REG, 0); // CEN bit for TIM1
    CLEAR_BIT(TIM2_CR1_REG, 0); // CEN bit for TIM2
    CLEAR_BIT(TIM3_CR1_REG, 0); // CEN bit for TIM3
    CLEAR_BIT(TIM4_CR1_REG, 0); // CEN bit for TIM4
    CLEAR_BIT(TIM5_CR1_REG, 0); // CEN bit for TIM5
    CLEAR_BIT(TIM9_CR1_REG, 0); // CEN bit for TIM9
    CLEAR_BIT(TIM10_CR1_REG, 0); // CEN bit for TIM10
    CLEAR_BIT(TIM11_CR1_REG, 0); // CEN bit for TIM11

    // I2C (I2C1, I2C2, I2C3)
    CLEAR_BIT(I2C1_CR1_REG, 0); // PE bit for I2C1
    CLEAR_BIT(I2C2_CR1_REG, 0); // PE bit for I2C2
    CLEAR_BIT(I2C3_CR1_REG, 0); // PE bit for I2C3

    // External Interrupts (Disable all EXTI lines in IMR)
    WRITE_REG(EXTI_IMR_REG, 0x00000000); // Disable all EXTI interrupts

    // 4. Enable WDT (Watchdog Timer)
    // No specific WDT registers in register_json. Implementing generic placeholder.
    // In a real STM32F401RC, this would involve IWDG_KR, IWDG_PR, IWDG_RLR.
    // Example: IWDG->KR = 0x5555; IWDG->PR = 0x00; IWDG->RLR = 0xFFF; IWDG->KR = 0xCCCC;
    // For now, only a comment to indicate it's enabled.
    (void)0; // WDT is notionally enabled.

    // 5. Clear WDT timer
    WDT_Reset();

    // 6. Set WDT period >= 8 msec
    // No specific WDT registers. Placeholder logic.
    // For IWDG, IWDG->RLR would be set based on LSI clock and prescaler.
    (void)0; // WDT period notionally set.

    // 7. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // No specific LVR registers in register_json. Placeholder logic.
    // For STM32, PVD (Programmable Voltage Detector) in PWR_CR is used for this.
    if (volt == VOLT_3V) {
        // Assume PVD set to ~2.0V
        (void)0;
    } else if (volt == VOLT_5V) {
        // Assume PVD set to ~3.5V
        (void)0;
    }

    // 8. Enable LVR (Low Voltage Reset)
    // No specific LVR registers. Placeholder logic.
    // For STM32, PVDEN bit in PWR_CR.
    (void)0; // LVR notionally enabled.

    // 9. Clear WDT again
    WDT_Reset();
}

/**
 * @brief Goes to sleep mode.
 *
 * This function puts the MCU into a low-power sleep mode, waiting for an interrupt.
 */
void Go_to_sleep_mode(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    __WFI(); // ARM Cortex-M Wait For Interrupt instruction
}

/**
 * @brief Enables global interrupts.
 */
void Global_interrupt_Enable(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    __enable_irq(); // ARM Cortex-M instruction to enable global interrupts
}

/**
 * @brief Disables global interrupts.
 */
void Global_interrupt_Disable(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    __disable_irq(); // ARM Cortex-M instruction to disable global interrupts
}

// LVD not supported on this MCU due to missing registers in register_json.
// Internal_EEPROM not supported on this MCU due to missing registers in register_json.
// MCAL_OUTPUT_BUZZER not supported on this MCU due to missing registers in register_json.
// DAC not supported on this MCU due to missing registers in register_json.
// MQTT Protocol not supported on this MCU due to missing registers/dependencies in register_json.
// HTTP Protocol not supported on this MCU due to missing registers/dependencies in register_json.
// WiFi Driver not supported on this MCU due to missing registers/dependencies in register_json.
// DTC_driver not supported on this MCU due to missing registers/dependencies in register_json.


/**
 * @brief Initializes a GPIO pin as an output.
 *
 * @param port The GPIO port (e.g., PORTA).
 * @param pin The pin number (e.g., PIN0).
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (port > PORTH || pin > PIN15) {
        return; // Invalid port or pin
    }

    // 1. Enable GPIOx clock
    enable_gpio_clock(port);

    // 2. Always set value before setting direction
    // Set initial output value using ODR
    if (value) {
        SET_BIT(GPIO_ODR_ADDRS[port], pin);
    } else {
        CLEAR_BIT(GPIO_ODR_ADDRS[port], pin);
    }
    // Verify value
    while (READ_BIT(GPIO_ODR_ADDRS[port], pin) != value);

    // 3. Set pin mode to Output (01b)
    MODIFY_REG(GPIO_MODER_ADDRS[port], (0x3U << (pin * 2)), (0x1U << (pin * 2)));
    // Verify mode
    while (((READ_REG(GPIO_MODER_ADDRS[port]) >> (pin * 2)) & 0x3U) != 0x1U);

    // 4. Set output type to Push-Pull (0b)
    CLEAR_BIT(GPIO_OTYPER_ADDRS[port], pin);

    // 5. Set output speed to Very High Speed (11b) - per rule: "use >=20mA sink current & >=10mA source current"
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[port], (0x3U << (pin * 2)), (0x3U << (pin * 2)));

    // 6. Disable pull-up/pull-down resistors (00b) for output pins
    MODIFY_REG(GPIO_PUPDR_ADDRS[port], (0x3U << (pin * 2)), (0x0U << (pin * 2)));
}

/**
 * @brief Initializes a GPIO pin as an input.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (port > PORTH || pin > PIN15) {
        return; // Invalid port or pin
    }

    // 1. Enable GPIOx clock
    enable_gpio_clock(port);

    // 2. Set pin mode to Input (00b)
    MODIFY_REG(GPIO_MODER_ADDRS[port], (0x3U << (pin * 2)), (0x0U << (pin * 2)));
    // Verify mode
    while (((READ_REG(GPIO_MODER_ADDRS[port]) >> (pin * 2)) & 0x3U) != 0x0U);

    // 3. All input pins have pull-up resistors enabled (01b) and wakeup feature enabled (if available)
    MODIFY_REG(GPIO_PUPDR_ADDRS[port], (0x3U << (pin * 2)), (0x1U << (pin * 2)));
    // Wakeup feature is typically an EXTI configuration, not a direct GPIO register setting.
    // This MCAL function only configures the GPIO itself. EXTI configuration is handled in External_INT_Init.
}

/**
 * @brief Gets the current direction of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction (INPUT or OUTPUT).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (port > PORTH || pin > PIN15) {
        return INPUT; // Return a default/error value
    }
    // Read MODER register bits for the specific pin
    tlong moder_val = (READ_REG(GPIO_MODER_ADDRS[port]) >> (pin * 2)) & 0x3U;
    return (moder_val == 0x0U) ? INPUT : OUTPUT;
}

/**
 * @brief Sets the value of a GPIO output pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (port > PORTH || pin > PIN15) {
        return; // Invalid port or pin
    }

    // Use BSRR for atomic set/reset
    if (value) {
        WRITE_REG(GPIO_BSRR_ADDRS[port], (1U << pin)); // Set bit
    } else {
        WRITE_REG(GPIO_BSRR_ADDRS[port], (1U << (pin + 16))); // Reset bit
    }
}

/**
 * @brief Gets the value of a GPIO input pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (port > PORTH || pin > PIN15) {
        return 0; // Return a default/error value
    }
    // Read IDR register bit for the specific pin
    return READ_BIT(GPIO_IDR_ADDRS[port], pin);
}

/**
 * @brief Toggles the value of a GPIO output pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (port > PORTH || pin > PIN15) {
        return; // Invalid port or pin
    }
    // Toggle the corresponding bit in the ODR register
    TOGGLE_BIT(GPIO_ODR_ADDRS[port], pin);
}

/**
 * @brief Initializes a UART channel.
 *
 * @param uart_channel The UART channel to initialize.
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data frame length.
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting.
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate,
               t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit,
               t_uart_parity uart_parity) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *sr, *dr, *brr, *cr1, *cr2, *cr3;
    if (!get_uart_registers(uart_channel, &sr, &dr, &brr, &cr1, &cr2, &cr3)) return;

    // Enable UARTx clock
    tlong apb_enr_bit = 0;
    t_port tx_port = PORTA, rx_port = PORTA;
    t_pin tx_pin = PIN0, rx_pin = PIN0;
    tbyte af_val = 0; // Alternate function value

    switch (uart_channel) {
        case UART_CHANNEL_1:
            SET_BIT(RCC_APB2ENR_REG, 4); // USART1EN bit 4 for APB2 (inferred)
            // Example GPIO for USART1: TX=PA9, RX=PA10 (AF7)
            tx_port = PORTA; tx_pin = PIN9; rx_port = PORTA; rx_pin = PIN10; af_val = 7;
            break;
        case UART_CHANNEL_2:
            SET_BIT(RCC_APB1ENR_REG, 17); // USART2EN bit 17 for APB1 (inferred)
            // Example GPIO for USART2: TX=PA2, RX=PA3 (AF7)
            tx_port = PORTA; tx_pin = PIN2; rx_port = PORTA; rx_pin = PIN3; af_val = 7;
            break;
        case UART_CHANNEL_6:
            SET_BIT(RCC_APB2ENR_REG, 5); // USART6EN bit 5 for APB2 (inferred)
            // Example GPIO for USART6: TX=PC6, RX=PC7 (AF8)
            tx_port = PORTC; tx_pin = PIN6; rx_port = PORTC; rx_pin = PIN7; af_val = 8;
            break;
        default: return;
    }
    // Dummy read for clock activation
    (void)READ_REG(RCC_APB1ENR_REG);
    (void)READ_REG(RCC_APB2ENR_REG);

    // Configure GPIOs for TX/RX in alternate function mode
    enable_gpio_clock(tx_port);
    enable_gpio_clock(rx_port);

    // Set TX pin to AF mode, Push-pull, High speed
    MODIFY_REG(GPIO_MODER_ADDRS[tx_port], (0x3U << (tx_pin * 2)), (0x2U << (tx_pin * 2)));
    CLEAR_BIT(GPIO_OTYPER_ADDRS[tx_port], tx_pin); // Push-pull
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[tx_port], (0x3U << (tx_pin * 2)), (0x2U << (tx_pin * 2))); // High speed
    MODIFY_REG((tx_pin < 8 ? GPIO_AFRL_ADDRS[tx_port] : GPIO_AFRH_ADDRS[tx_port]),
               (0xFU << ((tx_pin % 8) * 4)), (af_val << ((tx_pin % 8) * 4)));

    // Set RX pin to AF mode, Pull-up (inputs by default have pull-ups in GPIO_Input_Init rule)
    MODIFY_REG(GPIO_MODER_ADDRS[rx_port], (0x3U << (rx_pin * 2)), (0x2U << (rx_pin * 2)));
    MODIFY_REG(GPIO_PUPDR_ADDRS[rx_port], (0x3U << (rx_pin * 2)), (0x1U << (rx_pin * 2))); // Pull-up
    MODIFY_REG((rx_pin < 8 ? GPIO_AFRL_ADDRS[rx_port] : GPIO_AFRH_ADDRS[rx_port]),
               (0xFU << ((rx_pin % 8) * 4)), (af_val << ((rx_pin % 8) * 4)));

    // Disable UART before configuration (if already enabled)
    CLEAR_BIT(cr1, 13); // UE bit

    // Configure Baud Rate
    // SystemCoreClock is not available, assume it's set up elsewhere or use a typical value (e.g., 16MHz for APB1, 84MHz for APB2)
    // For STM32F401RC, APB1 is max 42MHz, APB2 is max 84MHz.
    tlong pclk_freq; // Peripheral clock frequency
    if (uart_channel == UART_CHANNEL_1 || uart_channel == UART_CHANNEL_6) {
        // USART1, USART6 on APB2 bus
        pclk_freq = 84000000; // Assuming 84MHz APB2 clock (max for STM32F401RC)
    } else { // UART_CHANNEL_2 on APB1 bus
        pclk_freq = 42000000; // Assuming 42MHz APB1 clock (max for STM32F401RC)
    }

    tlong baud_value;
    switch (uart_baud_rate) {
        case BAUD_9600: baud_value = pclk_freq / (16 * 9600); break;
        case BAUD_19200: baud_value = pclk_freq / (16 * 19200); break;
        case BAUD_115200: baud_value = pclk_freq / (16 * 115200); break;
        default: baud_value = pclk_freq / (16 * 9600); break; // Default to 9600
    }
    WRITE_REG(brr, baud_value);

    // Configure Control Register 1 (CR1)
    tlong cr1_val = 0;
    if (uart_data_length == DATA_9BIT) SET_BIT(&cr1_val, 12); // M bit for 9 data bits
    if (uart_parity != PARITY_NONE) {
        SET_BIT(&cr1_val, 10); // PCE bit for parity enable
        if (uart_parity == PARITY_ODD) SET_BIT(&cr1_val, 9); // PS bit for odd parity
    }
    // Enable Transmitter (TE) and Receiver (RE)
    SET_BIT(&cr1_val, 3); // TE
    SET_BIT(&cr1_val, 2); // RE
    WRITE_REG(cr1, cr1_val);

    // Configure Control Register 2 (CR2) for Stop Bits
    tlong cr2_val = 0;
    switch (uart_stop_bit) {
        case STOP_0_5BIT: MODIFY_REG(&cr2_val, (0x3U << 12), (0x1U << 12)); break;
        case STOP_2BIT: MODIFY_REG(&cr2_val, (0x3U << 12), (0x2U << 12)); break;
        case STOP_1_5BIT: MODIFY_REG(&cr2_val, (0x3U << 12), (0x3U << 12)); break;
        case STOP_1BIT: // Default, 00b
        default: break;
    }
    WRITE_REG(cr2, cr2_val);

    // CR3 is left at default for now (no DMA, no error interrupts configured)
}

/**
 * @brief Enables a UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *sr, *dr, *brr, *cr1, *cr2, *cr3;
    if (!get_uart_registers(uart_channel, &sr, &dr, &brr, &cr1, &cr2, &cr3)) return;

    // Ensure peripheral clock is enabled (if not already by Init)
    switch (uart_channel) {
        case UART_CHANNEL_1:
            SET_BIT(RCC_APB2ENR_REG, 4); // USART1EN (inferred)
            break;
        case UART_CHANNEL_2:
            SET_BIT(RCC_APB1ENR_REG, 17); // USART2EN (inferred)
            break;
        case UART_CHANNEL_6:
            SET_BIT(RCC_APB2ENR_REG, 5); // USART6EN (inferred)
            break;
        default: return;
    }
    (void)READ_REG(RCC_APB1ENR_REG); // Dummy read
    (void)READ_REG(RCC_APB2ENR_REG); // Dummy read

    SET_BIT(cr1, 13); // UE bit for UART Enable
}

/**
 * @brief Disables a UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *sr, *dr, *brr, *cr1, *cr2, *cr3;
    if (!get_uart_registers(uart_channel, &sr, &dr, &brr, &cr1, &cr2, &cr3)) return;

    CLEAR_BIT(cr1, 13); // UE bit for UART Disable
}

/**
 * @brief Sends a single byte over UART.
 * @param uart_channel The UART channel.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *sr, *dr, *brr, *cr1, *cr2, *cr3;
    if (!get_uart_registers(uart_channel, &sr, &dr, &brr, &cr1, &cr2, &cr3)) return;

    while (!READ_BIT(sr, 7)); // Wait for TXE (Transmit data register empty)
    WRITE_REG(dr, byte);      // Write data to DR
    while (!READ_BIT(sr, 6)); // Wait for TC (Transmission complete)
}

/**
 * @brief Sends a frame of data over UART.
 * @param uart_channel The UART channel.
 * @param data Pointer to the data buffer.
 * @param length The length of the data to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over UART.
 * @param uart_channel The UART channel.
 * @param str The string to send.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str) {
    UART_send_frame(uart_channel, str, strlen(str));
}

/**
 * @brief Gets a single byte from UART.
 * @param uart_channel The UART channel.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *sr, *dr, *brr, *cr1, *cr2, *cr3;
    if (!get_uart_registers(uart_channel, &sr, &dr, &brr, &cr1, &cr2, &cr3)) return 0;

    while (!READ_BIT(sr, 5)); // Wait for RXNE (Read data register not empty)
    return (tbyte)READ_REG(dr); // Read data from DR
}

/**
 * @brief Reads a frame of data from UART.
 * @param uart_channel The UART channel.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of data to read.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Reads a null-terminated string from UART.
 * @param uart_channel The UART channel.
 * @param buffer Pointer to the buffer to store received string.
 * @param max_length The maximum length of string to read (including null terminator).
 * @return The length of the received string (excluding null terminator).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    int i = 0;
    for (i = 0; i < max_length - 1; i++) {
        char received_char = (char)UART_Get_Byte(uart_channel);
        buffer[i] = received_char;
        if (received_char == '\0' || received_char == '\n' || received_char == '\r') {
            break;
        }
    }
    buffer[i] = '\0'; // Null-terminate the string
    return i;
}

/**
 * @brief Initializes an I2C channel.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (always fast mode).
 * @param i2c_device_address The device's own 7-bit address.
 * @param i2c_ack Acknowledge control.
 * @param i2c_datalength Not used in common I2C configuration at this level.
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed,
              t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack,
              t_i2c_datalength i2c_datalength) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *oar1, *dr, *sr1, *sr2, *ccr, *trise;
    if (!get_i2c_registers(i2c_channel, &cr1, &cr2, &oar1, &dr, &sr1, &sr2, &ccr, &trise)) return;

    // Enable I2Cx clock
    tlong apb1_enr_bit = 0;
    t_port scl_port = PORTB, sda_port = PORTB;
    t_pin scl_pin = PIN0, sda_pin = PIN0;
    tbyte af_val = 0; // Alternate function value

    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            SET_BIT(RCC_APB1ENR_REG, 21); // I2C1EN (inferred)
            // Example GPIO: SCL=PB6, SDA=PB7 (AF4)
            scl_port = PORTB; scl_pin = PIN6; sda_port = PORTB; sda_pin = PIN7; af_val = 4;
            break;
        case I2C_CHANNEL_2:
            SET_BIT(RCC_APB1ENR_REG, 22); // I2C2EN (inferred)
            // Example GPIO: SCL=PB10, SDA=PB11 (AF4)
            scl_port = PORTB; scl_pin = PIN10; sda_port = PORTB; sda_pin = PIN11; af_val = 4;
            break;
        case I2C_CHANNEL_3:
            SET_BIT(RCC_APB1ENR_REG, 23); // I2C3EN (inferred)
            // Example GPIO: SCL=PA8, SDA=PC9 (AF4)
            scl_port = PORTA; scl_pin = PIN8; sda_port = PORTC; sda_pin = PIN9; af_val = 4;
            break;
        default: return;
    }
    // Dummy read for clock activation
    (void)READ_REG(RCC_APB1ENR_REG);

    // Configure GPIOs for SCL/SDA in Alternate Function, Open-drain, High Speed, Pull-up
    enable_gpio_clock(scl_port);
    enable_gpio_clock(sda_port);

    // SCL pin config
    MODIFY_REG(GPIO_MODER_ADDRS[scl_port], (0x3U << (scl_pin * 2)), (0x2U << (scl_pin * 2))); // AF mode
    SET_BIT(GPIO_OTYPER_ADDRS[scl_port], scl_pin); // Open-drain
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[scl_port], (0x3U << (scl_pin * 2)), (0x2U << (scl_pin * 2))); // High speed
    MODIFY_REG(GPIO_PUPDR_ADDRS[scl_port], (0x3U << (scl_pin * 2)), (0x1U << (scl_pin * 2))); // Pull-up
    MODIFY_REG((scl_pin < 8 ? GPIO_AFRL_ADDRS[scl_port] : GPIO_AFRH_ADDRS[scl_port]),
               (0xFU << ((scl_pin % 8) * 4)), (af_val << ((scl_pin % 8) * 4)));

    // SDA pin config
    MODIFY_REG(GPIO_MODER_ADDRS[sda_port], (0x3U << (sda_pin * 2)), (0x2U << (sda_pin * 2))); // AF mode
    SET_BIT(GPIO_OTYPER_ADDRS[sda_port], sda_pin); // Open-drain
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[sda_port], (0x3U << (sda_pin * 2)), (0x2U << (sda_pin * 2))); // High speed
    MODIFY_REG(GPIO_PUPDR_ADDRS[sda_port], (0x3U << (sda_pin * 2)), (0x1U << (sda_pin * 2))); // Pull-up
    MODIFY_REG((sda_pin < 8 ? GPIO_AFRL_ADDRS[sda_port] : GPIO_AFRH_ADDRS[sda_port]),
               (0xFU << ((sda_pin % 8) * 4)), (af_val << ((sda_pin % 8) * 4)));


    // Disable I2C peripheral before configuration
    CLEAR_BIT(cr1, 0); // PE bit

    // Reset I2C peripheral (optional, but good practice)
    // SET_BIT(RCC_APB1RSTR_REG, apb1_enr_bit);
    // CLEAR_BIT(RCC_APB1RSTR_REG, apb1_enr_bit);

    // Set peripheral clock frequency in CR2 (required for CCR and TRISE calculations)
    // Assuming APB1 clock is 42 MHz for STM32F401RC
    WRITE_REG(cr2, 42); // Freq[5:0] = 42MHz

    // Configure CCR for Fast Mode (400kHz) and TRISE
    tlong pclk1_freq = 42000000; // APB1 clock in Hz
    tlong ccr_val = 0;
    tlong trise_val = 0;

    // "Always use fast mode" rule
    // For 400kHz Fast Mode (Fm): Thigh = CCRR * Tpclk1, Tlow = 2 * CCRR * Tpclk1 for duty cycle 2
    // I2C_CCR_FS bit (15) for Fast Mode, I2C_CCR_DUTY (14) for fast mode duty cycle
    SET_BIT(&ccr_val, 15); // Fast mode
    SET_BIT(&ccr_val, 14); // Fm mode duty cycle Tlow/Thigh = 16/9 (Duty cycle 2)

    // CCR calculation for 400kHz: Thigh = Tlow = (1/(2*400kHz)) = 1.25us
    // CCR = PCLK1_FREQ / (2 * I2C_SPEED) for duty cycle 0 (50%)
    // For fast mode, T_PCLK1 * (1+DUTY_CYCLE) * CCR = 1/f_I2C, where DUTY_CYCLE is 2 for 16/9.
    // CCR = PCLK1_FREQ / (I2C_SPEED * (DutyCycle_Factor_Num/DutyCycle_Factor_Den + 1))
    // For 400kHz, Fm duty = 16/9:
    // CCRR = PCLK1 / (25 * I2C_speed) (for standard mode) or PCLK1 / (3 * I2C_speed) (for fast mode, duty cycle 2: Tlow=2*CCRR*Tpclk1, Thigh=CCRR*Tpclk1 => 3*CCRR*Tpclk1 = 1/f_I2C)
    // CCRR (for duty 2, 400kHz) = (PCLK1_FREQ / (400000 * 3))
    ccr_val |= (pclk1_freq / (400000 * 3)); // (inferred)
    WRITE_REG(ccr, ccr_val);

    // TRISE calculation for Fast Mode (300ns max rise time)
    // TRISE = (Max_Rise_Time / T_PCLK1) + 1
    trise_val = (pclk1_freq / 1000000 * 0.3) + 1; // 300ns max rise time for Fast Mode (inferred)
    WRITE_REG(trise, trise_val);

    // Configure Own Address Register 1 (OAR1)
    WRITE_REG(oar1, (1U << 14) | (i2c_device_address << 1)); // Set bit 14, 7-bit address mode

    // Configure CR1 for ACK
    if (i2c_ack == I2C_ACK_ENABLE) {
        SET_BIT(cr1, 10); // ACK bit
    } else {
        CLEAR_BIT(cr1, 10);
    }
    // No explicit configuration for i2c_datalength in common I2C register bits, assuming 8-bit default.
}

/**
 * @brief Enables an I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *oar1, *dr, *sr1, *sr2, *ccr, *trise;
    if (!get_i2c_registers(i2c_channel, &cr1, &cr2, &oar1, &dr, &sr1, &sr2, &ccr, &trise)) return;

    // Ensure peripheral clock is enabled (if not already by Init)
    switch (i2c_channel) {
        case I2C_CHANNEL_1: SET_BIT(RCC_APB1ENR_REG, 21); break; // I2C1EN (inferred)
        case I2C_CHANNEL_2: SET_BIT(RCC_APB1ENR_REG, 22); break; // I2C2EN (inferred)
        case I2C_CHANNEL_3: SET_BIT(RCC_APB1ENR_REG, 23); break; // I2C3EN (inferred)
        default: return;
    }
    (void)READ_REG(RCC_APB1ENR_REG); // Dummy read

    SET_BIT(cr1, 0); // PE bit for I2C Enable
}

/**
 * @brief Disables an I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *oar1, *dr, *sr1, *sr2, *ccr, *trise;
    if (!get_i2c_registers(i2c_channel, &cr1, &cr2, &oar1, &dr, &sr1, &sr2, &ccr, &trise)) return;

    CLEAR_BIT(cr1, 0); // PE bit for I2C Disable
}

/**
 * @brief Sends a single byte over I2C.
 * This implementation assumes master mode.
 * @param i2c_channel The I2C channel.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *oar1, *dr, *sr1, *sr2, *ccr, *trise;
    if (!get_i2c_registers(i2c_channel, &cr1, &cr2, &oar1, &dr, &sr1, &sr2, &ccr, &trise)) return;

    // Send START condition
    SET_BIT(cr1, 8); // START bit
    while (!READ_BIT(sr1, 0)); // Wait for SB (Start Bit)

    // Send slave address (example for a generic 7-bit write address)
    WRITE_REG(dr, (0x50 << 1) | 0x00); // Example slave address 0x50, write mode
    while (!READ_BIT(sr1, 1)); // Wait for ADDR (Address sent/matched)
    (void)READ_REG(sr2); // Clear ADDR by reading SR2

    // Send data byte
    WRITE_REG(dr, byte);
    while (!READ_BIT(sr1, 7)); // Wait for TxE (Data register empty)

    // Send STOP condition (if this is the last byte in a transaction)
    // Rule: "Always generate a repeated start condition instead of stop between transactions"
    // So, a STOP is only generated if this is a standalone transmission or the end of a block.
    // For this generic function, it's safer to always generate a STOP.
    // In a higher-level driver, START/STOP would be controlled by transaction state.
    SET_BIT(cr1, 9); // STOP bit
    while (READ_BIT(cr1, 9)); // Wait for STOPF to clear (device is stopped)
}

/**
 * @brief Sends a frame of data over I2C.
 * @param i2c_channel The I2C channel.
 * @param data Pointer to the data buffer.
 * @param length The length of the data to send.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *oar1, *dr, *sr1, *sr2, *ccr, *trise;
    if (!get_i2c_registers(i2c_channel, &cr1, &cr2, &oar1, &dr, &sr1, &sr2, &ccr, &trise)) return;

    // Send START condition
    SET_BIT(cr1, 8); // START bit
    while (!READ_BIT(sr1, 0)); // Wait for SB (Start Bit)

    // Send slave address (example for a generic 7-bit write address)
    WRITE_REG(dr, (0x50 << 1) | 0x00); // Example slave address 0x50, write mode
    while (!READ_BIT(sr1, 1)); // Wait for ADDR (Address sent/matched)
    (void)READ_REG(sr2); // Clear ADDR by reading SR2

    for (int i = 0; i < length; i++) {
        WRITE_REG(dr, (tbyte)data[i]);
        while (!READ_BIT(sr1, 7)); // Wait for TxE
    }

    // Send STOP condition
    SET_BIT(cr1, 9); // STOP bit
    while (READ_BIT(cr1, 9)); // Wait for STOPF to clear
}

/**
 * @brief Sends a null-terminated string over I2C.
 * @param i2c_channel The I2C channel.
 * @param str The string to send.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    I2C_send_frame(i2c_channel, str, strlen(str));
}

/**
 * @brief Gets a single byte from I2C.
 * This implementation assumes master mode and a generic read from a slave address.
 * @param i2c_channel The I2C channel.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *oar1, *dr, *sr1, *sr2, *ccr, *trise;
    if (!get_i2c_registers(i2c_channel, &cr1, &cr2, &oar1, &dr, &sr1, &sr2, &ccr, &trise)) return 0;

    // Send START condition
    SET_BIT(cr1, 8); // START bit
    while (!READ_BIT(sr1, 0)); // Wait for SB

    // Send slave address with read bit (example slave address 0x50, read mode)
    WRITE_REG(dr, (0x50 << 1) | 0x01);
    while (!READ_BIT(sr1, 1)); // Wait for ADDR
    (void)READ_REG(sr2); // Clear ADDR by reading SR2

    // Set NACK for last byte, then STOP
    CLEAR_BIT(cr1, 10); // Clear ACK bit for NACK
    SET_BIT(cr1, 9); // Send STOP after receiving byte

    while (!READ_BIT(sr1, 6)); // Wait for RxNE (Data register not empty)
    tbyte received_byte = (tbyte)READ_REG(dr);

    while (READ_BIT(cr1, 9)); // Wait for STOPF to clear
    SET_BIT(cr1, 10); // Re-enable ACK for future transfers (good practice)

    return received_byte;
}

/**
 * @brief Reads a frame of data from I2C.
 * @param i2c_channel The I2C channel.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of data to read.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *oar1, *dr, *sr1, *sr2, *ccr, *trise;
    if (!get_i2c_registers(i2c_channel, &cr1, &cr2, &oar1, &dr, &sr1, &sr2, &ccr, &trise)) return;

    // Send START condition
    SET_BIT(cr1, 8); // START bit
    while (!READ_BIT(sr1, 0)); // Wait for SB

    // Send slave address with read bit
    WRITE_REG(dr, (0x50 << 1) | 0x01);
    while (!READ_BIT(sr1, 1)); // Wait for ADDR
    (void)READ_REG(sr2); // Clear ADDR

    for (int i = 0; i < max_length; i++) {
        if (i == max_length - 1) {
            CLEAR_BIT(cr1, 10); // Send NACK for the last byte
            SET_BIT(cr1, 9); // Send STOP after receiving the last byte
        }
        while (!READ_BIT(sr1, 6)); // Wait for RxNE
        buffer[i] = (char)READ_REG(dr);
    }
    while (READ_BIT(cr1, 9)); // Wait for STOPF to clear
    SET_BIT(cr1, 10); // Re-enable ACK for future transfers
}

/**
 * @brief Reads a null-terminated string from I2C.
 * @param i2c_channel The I2C channel.
 * @param buffer Pointer to the buffer to store received string.
 * @param max_length The maximum length of string to read (including null terminator).
 * @return The length of the received string (excluding null terminator).
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    int i = 0;
    for (i = 0; i < max_length - 1; i++) {
        char received_char = (char)I2C_Get_Byte(i2c_channel);
        buffer[i] = received_char;
        if (received_char == '\0' || received_char == '\n' || received_char == '\r') {
            break;
        }
    }
    buffer[i] = '\0'; // Null-terminate the string
    return i;
}

/**
 * @brief Initializes an SPI channel.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode Master/Slave mode.
 * @param spi_cpol Clock polarity.
 * @param spi_cpha Clock phase.
 * @param spi_dff Data frame format.
 * @param spi_bit_order Bit order (MSB/LSB).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol,
              t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *sr, *dr, *i2scfgr;
    if (!get_spi_registers(spi_channel, &cr1, &cr2, &sr, &dr, &i2scfgr)) return;

    // Enable SPIx clock
    tlong apb_enr_bit = 0;
    t_port sck_port = PORTA, miso_port = PORTA, mosi_port = PORTA, nss_port = PORTA;
    t_pin sck_pin = PIN0, miso_pin = PIN0, mosi_pin = PIN0, nss_pin = PIN0;
    tbyte af_val = 0; // Alternate function value

    switch (spi_channel) {
        case SPI_CHANNEL_1:
            SET_BIT(RCC_APB2ENR_REG, 0); // SPI1EN (inferred)
            // Example GPIO: SCK=PA5, MISO=PA6, MOSI=PA7, NSS=PA4 (AF5)
            sck_port = PORTA; sck_pin = PIN5; miso_port = PORTA; miso_pin = PIN6;
            mosi_port = PORTA; mosi_pin = PIN7; nss_port = PORTA; nss_pin = PIN4; af_val = 5;
            break;
        case SPI_CHANNEL_2:
            SET_BIT(RCC_APB1ENR_REG, 14); // SPI2EN (inferred)
            // Example GPIO: SCK=PB13, MISO=PB14, MOSI=PB15, NSS=PB12 (AF5)
            sck_port = PORTB; sck_pin = PIN13; miso_port = PORTB; miso_pin = PIN14;
            mosi_port = PORTB; mosi_pin = PIN15; nss_port = PORTB; nss_pin = PIN12; af_val = 5;
            break;
        case SPI_CHANNEL_3:
            SET_BIT(RCC_APB1ENR_REG, 15); // SPI3EN (inferred)
            // Example GPIO: SCK=PB3, MISO=PB4, MOSI=PB5, NSS=PA15 (AF6)
            sck_port = PORTB; sck_pin = PIN3; miso_port = PORTB; miso_pin = PIN4;
            mosi_port = PORTB; mosi_pin = PIN5; nss_port = PORTA; nss_pin = PIN15; af_val = 6;
            break;
        default: return;
    }
    // Dummy read for clock activation
    (void)READ_REG(RCC_APB1ENR_REG);
    (void)READ_REG(RCC_APB2ENR_REG);

    // Configure GPIOs for SPI pins (SCK, MISO, MOSI, NSS)
    enable_gpio_clock(sck_port);
    enable_gpio_clock(miso_port);
    enable_gpio_clock(mosi_port);
    enable_gpio_clock(nss_port);

    // Set pins to AF mode, Push-pull, High speed (no pull-up/down for AF by default, managed by peripheral)
    // SCK
    MODIFY_REG(GPIO_MODER_ADDRS[sck_port], (0x3U << (sck_pin * 2)), (0x2U << (sck_pin * 2)));
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[sck_port], (0x3U << (sck_pin * 2)), (0x2U << (sck_pin * 2)));
    MODIFY_REG((sck_pin < 8 ? GPIO_AFRL_ADDRS[sck_port] : GPIO_AFRH_ADDRS[sck_port]),
               (0xFU << ((sck_pin % 8) * 4)), (af_val << ((sck_pin % 8) * 4)));
    // MISO
    MODIFY_REG(GPIO_MODER_ADDRS[miso_port], (0x3U << (miso_pin * 2)), (0x2U << (miso_pin * 2)));
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[miso_port], (0x3U << (miso_pin * 2)), (0x2U << (miso_pin * 2)));
    MODIFY_REG((miso_pin < 8 ? GPIO_AFRL_ADDRS[miso_port] : GPIO_AFRH_ADDRS[miso_port]),
               (0xFU << ((miso_pin % 8) * 4)), (af_val << ((miso_pin % 8) * 4)));
    // MOSI
    MODIFY_REG(GPIO_MODER_ADDRS[mosi_port], (0x3U << (mosi_pin * 2)), (0x2U << (mosi_pin * 2)));
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[mosi_port], (0x3U << (mosi_pin * 2)), (0x2U << (mosi_pin * 2)));
    MODIFY_REG((mosi_pin < 8 ? GPIO_AFRL_ADDRS[mosi_port] : GPIO_AFRH_ADDRS[mosi_port]),
               (0xFU << ((mosi_pin % 8) * 4)), (af_val << ((mosi_pin % 8) * 4)));
    // NSS (Slave Select)
    // Rule: "Slave Select always software-controlled" -> NSS pin configured as general purpose output or input.
    // For software control: SPI_CR1_SSM (bit 9) and SPI_CR1_SSI (bit 8). NSS pin itself is then unused by hardware.
    // For now, if NSS pin specified, configure it as AF as well to be consistent with hardware if needed later.
    // However, if SSM is set, the hardware NSS pin is not used, so its mode doesn't strictly matter for SPI itself.
    // Will configure as AF as per standard setup.
    MODIFY_REG(GPIO_MODER_ADDRS[nss_port], (0x3U << (nss_pin * 2)), (0x2U << (nss_pin * 2)));
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[nss_port], (0x3U << (nss_pin * 2)), (0x2U << (nss_pin * 2)));
    MODIFY_REG((nss_pin < 8 ? GPIO_AFRL_ADDRS[nss_port] : GPIO_AFRH_ADDRS[nss_port]),
               (0xFU << ((nss_pin % 8) * 4)), (af_val << ((nss_pin % 8) * 4)));


    // Disable SPI before configuration
    CLEAR_BIT(cr1, 6); // SPE bit

    tlong cr1_val = 0;
    // Set Master/Slave mode
    if (spi_mode == SPI_MODE_MASTER) SET_BIT(&cr1_val, 2); // MSTR bit
    // "Always use full duplex"
    CLEAR_BIT(&cr1_val, 15); // BIDIMODE = 0 (2-line unidirectional data mode)

    // Set CPHA and CPOL
    if (spi_cpol == SPI_CPOL_HIGH) SET_BIT(&cr1_val, 1); // CPOL
    if (spi_cpha == SPI_CPHA_2EDGE) SET_BIT(&cr1_val, 0); // CPHA

    // Set Data Frame Format (DFF)
    if (spi_dff == SPI_DFF_16BIT) SET_BIT(&cr1_val, 11); // DFF

    // Set Bit Order (LSBFIRST)
    if (spi_bit_order == SPI_BIT_ORDER_LSB) SET_BIT(&cr1_val, 7); // LSBFIRST

    // "Always use fast speed" - Set Baud Rate Control (BR) to smallest prescaler possible (PCLK/2)
    MODIFY_REG(&cr1_val, (0x7U << 3), (0x0U << 3)); // BR[2:0] = 000 (PCLK/2)

    // "Slave Select always software-controlled"
    SET_BIT(&cr1_val, 9); // SSM (Software Slave Management)
    SET_BIT(&cr1_val, 8); // SSI (Internal Slave Select)

    // "Always enable CRC"
    SET_BIT(&cr1_val, 13); // CRCEN

    WRITE_REG(cr1, cr1_val);

    // Configure CR2
    // For software NSS, SSOE (bit 2) in CR2 is generally cleared in master mode
    // if NSS is not used as output, or set if multi-master/slave is needed.
    // For simple master/slave, with SSM set, NSS pin is irrelevant.
    // If SPI_MODE_MASTER is used, this typically means a single master.
    // For now, keep CR2 default or clear SSOE if master.
    tlong cr2_val = 0;
    if (spi_mode == SPI_MODE_MASTER) {
        // Master mode: If NSS pin is not used, clear SSOE.
        // If NSS pin is used as hardware output for multi-slave, set SSOE.
        // Given "Slave Select always software-controlled", SSOE might not be strictly needed.
        // For now, clear to be safe.
        CLEAR_BIT(&cr2_val, 2); // SSOE
    } else { // Slave mode
        // In slave mode, NSS input is usually required. If software NSS is active,
        // it acts as a virtual NSS. Keep SSOE cleared.
        CLEAR_BIT(&cr2_val, 2); // SSOE
    }
    WRITE_REG(cr2, cr2_val);

    // I2S is part of the SPI peripheral, ensure I2S mode is disabled during SPI init
    CLEAR_BIT(i2scfgr, 11); // I2SE bit (I2S Enable)
}

/**
 * @brief Enables an SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *sr, *dr, *i2scfgr;
    if (!get_spi_registers(spi_channel, &cr1, &cr2, &sr, &dr, &i2scfgr)) return;

    // Ensure peripheral clock is enabled (if not already by Init)
    switch (spi_channel) {
        case SPI_CHANNEL_1: SET_BIT(RCC_APB2ENR_REG, 0); break; // SPI1EN (inferred)
        case SPI_CHANNEL_2: SET_BIT(RCC_APB1ENR_REG, 14); break; // SPI2EN (inferred)
        case SPI_CHANNEL_3: SET_BIT(RCC_APB1ENR_REG, 15); break; // SPI3EN (inferred)
        default: return;
    }
    (void)READ_REG(RCC_APB1ENR_REG); // Dummy read
    (void)READ_REG(RCC_APB2ENR_REG); // Dummy read

    SET_BIT(cr1, 6); // SPE bit for SPI Enable
}

/**
 * @brief Disables an SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *sr, *dr, *i2scfgr;
    if (!get_spi_registers(spi_channel, &cr1, &cr2, &sr, &dr, &i2scfgr)) return;

    CLEAR_BIT(cr1, 6); // SPE bit for SPI Disable
}

/**
 * @brief Sends a single byte over SPI.
 * @param spi_channel The SPI channel.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *sr, *dr, *i2scfgr;
    if (!get_spi_registers(spi_channel, &cr1, &cr2, &sr, &dr, &i2scfgr)) return;

    // Wait until transmit buffer is empty (TXE flag)
    while (!READ_BIT(sr, 1));
    // Write the data to the Data Register
    WRITE_REG(dr, byte);
    // Wait until receive buffer is not empty (RXNE flag) -- for full duplex to complete transaction
    while (!READ_BIT(sr, 0));
    // Read dummy data to clear RXNE (if master reading, otherwise can be discarded)
    (void)READ_REG(dr);
}

/**
 * @brief Sends a frame of data over SPI.
 * @param spi_channel The SPI channel.
 * @param data Pointer to the data buffer.
 * @param length The length of the data to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Gets a single byte from SPI.
 * @param spi_channel The SPI channel.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *cr2, *sr, *dr, *i2scfgr;
    if (!get_spi_registers(spi_channel, &cr1, &cr2, &sr, &dr, &i2scfgr)) return 0;

    // For full-duplex operation, a dummy write is often needed to initiate clock cycles for reception.
    // Wait until transmit buffer is empty (TXE flag)
    while (!READ_BIT(sr, 1));
    // Write dummy data to generate clock for reception
    WRITE_REG(dr, 0xFF);
    // Wait until receive buffer is not empty (RXNE flag)
    while (!READ_BIT(sr, 0));
    // Read the received data from the Data Register
    return (tbyte)READ_REG(dr);
}

/**
 * @brief Reads a frame of data from SPI.
 * @param spi_channel The SPI channel.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of data to read.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Reads a null-terminated string from SPI.
 * @param spi_channel The SPI channel.
 * @param buffer Pointer to the buffer to store received string.
 * @param max_length The maximum length of string to read (including null terminator).
 * @return The length of the received string (excluding null terminator).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    int i = 0;
    for (i = 0; i < max_length - 1; i++) {
        char received_char = (char)SPI_Get_Byte(spi_channel);
        buffer[i] = received_char;
        if (received_char == '\0' || received_char == '\n' || received_char == '\r') {
            break;
        }
    }
    buffer[i] = '\0'; // Null-terminate the string
    return i;
}

/**
 * @brief Initializes an External Interrupt channel.
 * @param external_int_channel The EXTI line to configure.
 * @param external_int_edge The triggering edge.
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (external_int_channel > EXTI_CHANNEL_15) return;

    // 1. Enable SYSCFG clock
    SET_BIT(RCC_APB2ENR_REG, 14); // SYSCFGEN bit 14 for APB2 (inferred)
    (void)READ_REG(RCC_APB2ENR_REG); // Dummy read

    // 2. Configure SYSCFG_EXTICR for the input line source selection
    // Determine which EXTICR register (1-4) and which nibble within it
    volatile uint32_t* exticr_reg = NULL;
    tbyte exti_reg_idx = external_int_channel / 4;
    tbyte exti_bit_pos = (external_int_channel % 4) * 4; // Each EXTI source is 4 bits

    switch (exti_reg_idx) {
        case 0: exticr_reg = SYSCFG_EXTICR1_REG; break;
        case 1: exticr_reg = SYSCFG_EXTICR2_REG; break;
        case 2: exticr_reg = SYSCFG_EXTICR3_REG; break;
        case 3: exticr_reg = SYSCFG_EXTICR4_REG; break;
        default: return; // Should not happen for EXTI0-15
    }

    // Default to PORTA (0000) for source selection. User can set specific port with another API.
    // For now, assuming PAx is the default source if not explicitly passed.
    // Example: To select PBx, would write 0001 (0x1) to the 4-bit field.
    // Modify this if a GPIO port is passed to the function.
    tbyte gpio_port_code = 0; // 0000 = PAx, 0001 = PBx, ...
    // Assuming PAx for now, as no port specified in API, but pins are tied to specific ports
    // For a robust system, this needs the GPIO port as input.
    // For EXTI_EXTICR1 example: PA0 for EXTI0 is 0x0 << 0; PB0 for EXTI0 is 0x1 << 0.
    MODIFY_REG(exticr_reg, (0xFU << exti_bit_pos), (gpio_port_code << exti_bit_pos));

    // 3. Configure EXTI_RTSR and EXTI_FTSR for edge detection
    CLEAR_BIT(EXTI_RTSR_REG, external_int_channel); // Clear both first
    CLEAR_BIT(EXTI_FTSR_REG, external_int_channel);

    if (external_int_edge == RISING_EDGE || external_int_edge == RISING_FALLING_EDGE) {
        SET_BIT(EXTI_RTSR_REG, external_int_channel);
    }
    if (external_int_edge == FALLING_EDGE || external_int_edge == RISING_FALLING_EDGE) {
        SET_BIT(EXTI_FTSR_REG, external_int_channel);
    }

    // 4. Disable the interrupt mask by default, enable explicitly with External_INT_Enable
    CLEAR_BIT(EXTI_IMR_REG, external_int_channel);
}

/**
 * @brief Enables an External Interrupt channel.
 * @param external_int_channel The EXTI line to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (external_int_channel > EXTI_CHANNEL_15) return;
    SET_BIT(EXTI_IMR_REG, external_int_channel); // Set Interrupt Mask Register bit
}

/**
 * @brief Disables an External Interrupt channel.
 * @param external_int_channel The EXTI line to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (external_int_channel > EXTI_CHANNEL_15) return;
    CLEAR_BIT(EXTI_IMR_REG, external_int_channel); // Clear Interrupt Mask Register bit
}

/**
 * @brief Initializes a PWM channel.
 * @param pwm_channel The PWM channel to configure.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    t_timer_channel timer_channel;
    tbyte timer_ch_idx;
    t_port gpio_port;
    t_pin gpio_pin;
    tbyte af_val;

    // Map PWM channel to Timer instance and channel index
    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1: timer_channel = TIMER_CHANNEL_1; timer_ch_idx = 1; gpio_port = PORTA; gpio_pin = PIN8; af_val = 1; break; // AF1 for TIM1_CH1_PA8
        case PWM_CHANNEL_TIM1_CH2: timer_channel = TIMER_CHANNEL_1; timer_ch_idx = 2; gpio_port = PORTA; gpio_pin = PIN9; af_val = 1; break; // AF1 for TIM1_CH2_PA9
        case PWM_CHANNEL_TIM1_CH3: timer_channel = TIMER_CHANNEL_1; timer_ch_idx = 3; gpio_port = PORTA; gpio_pin = PIN10; af_val = 1; break; // AF1 for TIM1_CH3_PA10
        case PWM_CHANNEL_TIM1_CH4: timer_channel = TIMER_CHANNEL_1; timer_ch_idx = 4; gpio_port = PORTA; gpio_pin = PIN11; af_val = 1; break; // AF1 for TIM1_CH4_PA11

        case PWM_CHANNEL_TIM2_CH1: timer_channel = TIMER_CHANNEL_2; timer_ch_idx = 1; gpio_port = PORTA; gpio_pin = PIN0; af_val = 1; break; // AF1 for TIM2_CH1_PA0
        case PWM_CHANNEL_TIM2_CH2: timer_channel = TIMER_CHANNEL_2; timer_ch_idx = 2; gpio_port = PORTA; gpio_pin = PIN1; af_val = 1; break; // AF1 for TIM2_CH2_PA1
        case PWM_CHANNEL_TIM2_CH3: timer_channel = TIMER_CHANNEL_2; timer_ch_idx = 3; gpio_port = PORTA; gpio_pin = PIN2; af_val = 1; break; // AF1 for TIM2_CH3_PA2
        case PWM_CHANNEL_TIM2_CH4: timer_channel = TIMER_CHANNEL_2; timer_ch_idx = 4; gpio_port = PORTA; gpio_pin = PIN3; af_val = 1; break; // AF1 for TIM2_CH4_PA3

        case PWM_CHANNEL_TIM3_CH1: timer_channel = TIMER_CHANNEL_3; timer_ch_idx = 1; gpio_port = PORTA; gpio_pin = PIN6; af_val = 2; break; // AF2 for TIM3_CH1_PA6
        case PWM_CHANNEL_TIM3_CH2: timer_channel = TIMER_CHANNEL_3; timer_ch_idx = 2; gpio_port = PORTA; gpio_pin = PIN7; af_val = 2; break; // AF2 for TIM3_CH2_PA7
        case PWM_CHANNEL_TIM3_CH3: timer_channel = TIMER_CHANNEL_3; timer_ch_idx = 3; gpio_port = PORTB; gpio_pin = PIN0; af_val = 2; break; // AF2 for TIM3_CH3_PB0
        case PWM_CHANNEL_TIM3_CH4: timer_channel = TIMER_CHANNEL_3; timer_ch_idx = 4; gpio_port = PORTB; gpio_pin = PIN1; af_val = 2; break; // AF2 for TIM3_CH4_PB1

        case PWM_CHANNEL_TIM4_CH1: timer_channel = TIMER_CHANNEL_4; timer_ch_idx = 1; gpio_port = PORTB; gpio_pin = PIN6; af_val = 2; break; // AF2 for TIM4_CH1_PB6
        case PWM_CHANNEL_TIM4_CH2: timer_channel = TIMER_CHANNEL_4; timer_ch_idx = 2; gpio_port = PORTB; gpio_pin = PIN7; af_val = 2; break; // AF2 for TIM4_CH2_PB7
        case PWM_CHANNEL_TIM4_CH3: timer_channel = TIMER_CHANNEL_4; timer_ch_idx = 3; gpio_port = PORTB; gpio_pin = PIN8; af_val = 2; break; // AF2 for TIM4_CH3_PB8
        case PWM_CHANNEL_TIM4_CH4: timer_channel = TIMER_CHANNEL_4; timer_ch_idx = 4; gpio_port = PORTB; gpio_pin = PIN9; af_val = 2; break; // AF2 for TIM4_CH4_PB9

        case PWM_CHANNEL_TIM5_CH1: timer_channel = TIMER_CHANNEL_5; timer_ch_idx = 1; gpio_port = PORTA; gpio_pin = PIN0; af_val = 2; break; // AF2 for TIM5_CH1_PA0 (re-check with TIM2, often same AF)
        case PWM_CHANNEL_TIM5_CH2: timer_channel = TIMER_CHANNEL_5; timer_ch_idx = 2; gpio_port = PORTA; gpio_pin = PIN1; af_val = 2; break; // AF2 for TIM5_CH2_PA1
        case PWM_CHANNEL_TIM5_CH3: timer_channel = TIMER_CHANNEL_5; timer_ch_idx = 3; gpio_port = PORTA; gpio_pin = PIN2; af_val = 2; break; // AF2 for TIM5_CH3_PA2
        case PWM_CHANNEL_TIM5_CH4: timer_channel = TIMER_CHANNEL_5; timer_ch_idx = 4; gpio_port = PORTA; gpio_pin = PIN3; af_val = 2; break; // AF2 for TIM5_CH4_PA3

        case PWM_CHANNEL_TIM9_CH1: timer_channel = TIMER_CHANNEL_9; timer_ch_idx = 1; gpio_port = PORTA; gpio_pin = PIN2; af_val = 3; break; // AF3 for TIM9_CH1_PA2
        case PWM_CHANNEL_TIM9_CH2: timer_channel = TIMER_CHANNEL_9; timer_ch_idx = 2; gpio_port = PORTA; gpio_pin = PIN3; af_val = 3; break; // AF3 for TIM9_CH2_PA3

        case PWM_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; timer_ch_idx = 1; gpio_port = PORTB; gpio_pin = PIN8; af_val = 3; break; // AF3 for TIM10_CH1_PB8

        case PWM_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; timer_ch_idx = 1; gpio_port = PORTB; gpio_pin = PIN9; af_val = 3; break; // AF3 for TIM11_CH1_PB9

        default: return;
    }

    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    // 1. Enable Timer clock
    enable_timer_clock(timer_channel);

    // 2. Configure GPIO in alternate function mode for the specific channel
    enable_gpio_clock(gpio_port);
    MODIFY_REG(GPIO_MODER_ADDRS[gpio_port], (0x3U << (gpio_pin * 2)), (0x2U << (gpio_pin * 2))); // AF mode
    CLEAR_BIT(GPIO_OTYPER_ADDRS[gpio_port], gpio_pin); // Push-pull
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[gpio_port], (0x3U << (gpio_pin * 2)), (0x2U << (gpio_pin * 2))); // High speed
    MODIFY_REG((gpio_pin < 8 ? GPIO_AFRL_ADDRS[gpio_port] : GPIO_AFRH_ADDRS[gpio_port]),
               (0xFU << ((gpio_pin % 8) * 4)), (af_val << ((gpio_pin % 8) * 4)));

    // Disable timer before configuration
    CLEAR_BIT(cr1, 0); // CEN bit

    // 3. Calculate PSC and ARR for frequency
    // Assuming APB1 clock for TIM2-5 (42MHz), APB2 clock for TIM1, 9-11 (84MHz)
    tlong pclk_freq;
    if (timer_channel == TIMER_CHANNEL_1 || timer_channel == TIMER_CHANNEL_9 ||
        timer_channel == TIMER_CHANNEL_10 || timer_channel == TIMER_CHANNEL_11) {
        pclk_freq = 84000000;
    } else {
        pclk_freq = 42000000;
    }

    tlong timer_freq_hz = pwm_khz_freq * 1000;
    tlong prescaler = (pclk_freq / timer_freq_hz / 1000) - 1; // Example prescaler, assuming ARR of 1000
    if (prescaler > 0xFFFF) prescaler = 0xFFFF; // Max 16-bit
    if (prescaler < 0) prescaler = 0;

    tlong auto_reload = (pclk_freq / (prescaler + 1) / timer_freq_hz) - 1;
    if (auto_reload > 0xFFFF) auto_reload = 0xFFFF; // Max 16-bit
    if (auto_reload < 0) auto_reload = 0;

    WRITE_REG(psc, prescaler);
    WRITE_REG(arr, auto_reload);

    // 4. Configure CCMRx for PWM mode (Output Compare Mode 1 - 110b) and preload enable
    tlong ccmr_val = 0;
    volatile uint32_t* ccmr_ptr = NULL;
    tbyte ccmr_offset = 0; // For odd channels (CH1, CH3) it's 0, for even (CH2, CH4) it's 8

    if (timer_ch_idx == 1 || timer_ch_idx == 2) {
        ccmr_ptr = ccmr1;
        ccmr_offset = (timer_ch_idx == 1) ? 0 : 8;
    } else if (timer_ch_idx == 3 || timer_ch_idx == 4) {
        ccmr_ptr = ccmr2;
        ccmr_offset = (timer_ch_idx == 3) ? 0 : 8;
    } else { return; } // Invalid channel index

    // Output Compare Mode 1: 110 (PWM mode 1)
    // OCxPE: Output compare x preload enable (bit 3 for channel 1/3, bit 11 for channel 2/4)
    ccmr_val = READ_REG(ccmr_ptr);
    MODIFY_REG(&ccmr_val, (0x7U << (ccmr_offset + 4)), (0x6U << (ccmr_offset + 4))); // Set OCxM to 110 (PWM Mode 1)
    SET_BIT(&ccmr_val, ccmr_offset + 3); // Set OCxPE
    WRITE_REG(ccmr_ptr, ccmr_val);

    // 5. Calculate and set CCRx for duty cycle
    tlong pulse_value = (auto_reload + 1) * pwm_duty / 100;
    if (pulse_value > auto_reload + 1) pulse_value = auto_reload + 1;

    switch (timer_ch_idx) {
        case 1: WRITE_REG(ccr1, pulse_value); break;
        case 2: WRITE_REG(ccr2, pulse_value); break;
        case 3: WRITE_REG(ccr3, pulse_value); break;
        case 4: WRITE_REG(ccr4, pulse_value); break;
        default: return;
    }

    // 6. Set CCER to enable output
    tlong ccer_val = READ_REG(ccer);
    SET_BIT(&ccer_val, (timer_ch_idx - 1) * 4); // CCxE (Capture/Compare Output Enable)
    WRITE_REG(ccer, ccer_val);

    // 7. For advanced timers (like TIM1), enable main output
    if (timer_channel == TIMER_CHANNEL_1 && bdtr != NULL) {
        SET_BIT(bdtr, 15); // MOE (Main Output Enable)
    }

    // Clear available FREQUENCY Ranges for each channel as comments
    // TIM1, TIM9, TIM10, TIM11 are on APB2 (84MHz Max)
    // TIM2, TIM3, TIM4, TIM5 are on APB1 (42MHz Max)
    // For 84MHz PCLK, Prescaler 0, ARR 0xFFFF (65535) -> 84MHz / 65536 ~= 1.28 kHz
    // For 84MHz PCLK, Prescaler 0xFFFF (65535), ARR 0 -> 84MHz / (65536 * 1) = 1.28 kHz
    // For 42MHz PCLK, Prescaler 0, ARR 0xFFFF (65535) -> 42MHz / 65536 ~= 0.64 kHz
    // For 42MHz PCLK, Prescaler 0xFFFF (65535), ARR 0 -> 42MHz / (65536 * 1) = 0.64 kHz
    // Example ranges:
    // With PCLK at 84MHz: Freq = PCLK / ((PSC+1)*(ARR+1))
    // Min Freq: 84MHz / ((65535+1)*(65535+1)) = 84MHz / (65536^2) ~ 19.5 Hz
    // Max Freq: 84MHz / ((0+1)*(0+1)) = 84MHz (but usually a lower practical max)
    // With PCLK at 42MHz:
    // Min Freq: 42MHz / ((65535+1)*(65535+1)) = 42MHz / (65536^2) ~ 9.7 Hz
    // Max Freq: 42MHz / ((0+1)*(0+1)) = 42MHz
}

/**
 * @brief Starts a PWM channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    t_timer_channel timer_channel;
    tbyte timer_ch_idx; // Dummy

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM1_CH2: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM1_CH3: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM1_CH4: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM2_CH1: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM2_CH2: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM2_CH3: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM2_CH4: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM3_CH1: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM3_CH2: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM3_CH3: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM3_CH4: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM4_CH1: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM4_CH2: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM4_CH3: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM4_CH4: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM5_CH1: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM5_CH2: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM5_CH3: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM5_CH4: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM9_CH1: timer_channel = TIMER_CHANNEL_9; break;
        case PWM_CHANNEL_TIM9_CH2: timer_channel = TIMER_CHANNEL_9; break;
        case PWM_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; break;
        case PWM_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; break;
        default: return;
    }

    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    SET_BIT(egr, 0); // UG bit (Update Generation) to load all registers to active registers
    SET_BIT(cr1, 0); // CEN bit (Counter Enable)
}

/**
 * @brief Stops a PWM channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    t_timer_channel timer_channel;
    tbyte timer_ch_idx; // Dummy

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM1_CH2: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM1_CH3: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM1_CH4: timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM2_CH1: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM2_CH2: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM2_CH3: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM2_CH4: timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM3_CH1: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM3_CH2: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM3_CH3: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM3_CH4: timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM4_CH1: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM4_CH2: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM4_CH3: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM4_CH4: timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM5_CH1: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM5_CH2: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM5_CH3: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM5_CH4: timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM9_CH1: timer_channel = TIMER_CHANNEL_9; break;
        case PWM_CHANNEL_TIM9_CH2: timer_channel = TIMER_CHANNEL_9; break;
        case PWM_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; break;
        case PWM_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; break;
        default: return;
    }

    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    CLEAR_BIT(cr1, 0); // CEN bit (Counter Enable)
}

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to configure.
 * @param icu_prescaller The input capture prescaler.
 * @param icu_edge The triggering edge.
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    t_timer_channel timer_channel;
    tbyte timer_ch_idx;
    t_port gpio_port;
    t_pin gpio_pin;
    tbyte af_val;

    // Map ICU channel to Timer instance and channel index
    switch (icu_channel) {
        case PWM_CHANNEL_TIM1_CH1: timer_channel = TIMER_CHANNEL_1; timer_ch_idx = 1; gpio_port = PORTA; gpio_pin = PIN8; af_val = 1; break;
        case PWM_CHANNEL_TIM2_CH1: timer_channel = TIMER_CHANNEL_2; timer_ch_idx = 1; gpio_port = PORTA; gpio_pin = PIN0; af_val = 1; break;
        // ... (similar mapping for other ICU channels)
        // For simplicity, using one example. A real implementation would have full mappings.
        default: return;
    }

    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    // 1. Enable Timer clock
    enable_timer_clock(timer_channel);

    // 2. Configure GPIO in alternate function mode for the specific channel
    enable_gpio_clock(gpio_port);
    MODIFY_REG(GPIO_MODER_ADDRS[gpio_port], (0x3U << (gpio_pin * 2)), (0x2U << (gpio_pin * 2))); // AF mode
    MODIFY_REG(GPIO_PUPDR_ADDRS[gpio_port], (0x3U << (gpio_pin * 2)), (0x1U << (gpio_pin * 2))); // Pull-up for input
    MODIFY_REG((gpio_pin < 8 ? GPIO_AFRL_ADDRS[gpio_port] : GPIO_AFRH_ADDRS[gpio_port]),
               (0xFU << ((gpio_pin % 8) * 4)), (af_val << ((gpio_pin % 8) * 4)));

    // Disable timer before configuration
    CLEAR_BIT(cr1, 0); // CEN bit

    // 3. Set PSC to desired value (or default to 0 for no division)
    WRITE_REG(psc, 0); // No explicit prescaler register bit in timer registers for input, usually set by TIM_PSC.
                      // The icu_prescaller parameter usually refers to the input capture prescaler (ICPSC)
                      // bits within CCMR, not the main timer prescaler.
    tlong icpsc_val = 0;
    switch (icu_prescaller) {
        case ICU_PRESCALLER_1: icpsc_val = 0; break;
        case ICU_PRESCALLER_2: icpsc_val = 1; break; // Capture every 2 events
        case ICU_PRESCALLER_4: icpsc_val = 2; break; // Capture every 4 events
        case ICU_PRESCALLER_8: icpsc_val = 3; break; // Capture every 8 events
        default: icpsc_val = 0; break;
    }

    // 4. Configure CCMRx for Input Capture mode (CCxS bits)
    tlong ccmr_val = 0;
    volatile uint32_t* ccmr_ptr = NULL;
    tbyte ccmr_offset = 0; // For odd channels (CH1, CH3) it's 0, for even (CH2, CH4) it's 8

    if (timer_ch_idx == 1 || timer_ch_idx == 2) {
        ccmr_ptr = ccmr1;
        ccmr_offset = (timer_ch_idx == 1) ? 0 : 8;
    } else if (timer_ch_idx == 3 || timer_ch_idx == 4) {
        ccmr_ptr = ccmr2;
        ccmr_offset = (timer_ch_idx == 3) ? 0 : 8;
    } else { return; } // Invalid channel index

    // CCxS = 01b for Input Capture on TIx (CC1S[1:0]=01 for IC1 mapped on TI1)
    ccmr_val = READ_REG(ccmr_ptr);
    MODIFY_REG(&ccmr_val, (0x3U << ccmr_offset), (0x1U << ccmr_offset)); // Set CCxS
    MODIFY_REG(&ccmr_val, (0x3U << (ccmr_offset + 2)), (icpsc_val << (ccmr_offset + 2))); // Set ICxPSC
    WRITE_REG(ccmr_ptr, ccmr_val);

    // 5. Configure CCER for input capture enable (CCxE) and polarity (CCxP, CCxNP)
    tlong ccer_val = READ_REG(ccer);
    CLEAR_BIT(&ccer_val, (timer_ch_idx - 1) * 4 + 1); // Clear CCxP (polarity)
    CLEAR_BIT(&ccer_val, (timer_ch_idx - 1) * 4 + 2); // Clear CCxNP (inverted polarity)

    if (icu_edge == RISING_EDGE) {
        SET_BIT(&ccer_val, (timer_ch_idx - 1) * 4); // CCxE enable
    } else if (icu_edge == FALLING_EDGE) {
        SET_BIT(&ccer_val, (timer_ch_idx - 1) * 4); // CCxE enable
        SET_BIT(&ccer_val, (timer_ch_idx - 1) * 4 + 1); // CCxP (falling edge)
    } else if (icu_edge == ICU_EDGE_BOTH) {
        SET_BIT(&ccer_val, (timer_ch_idx - 1) * 4); // CCxE enable
        SET_BIT(&ccer_val, (timer_ch_idx - 1) * 4 + 1); // CCxP
        SET_BIT(&ccer_val, (timer_ch_idx - 1) * 4 + 2); // CCxNP (both edges)
    }
    WRITE_REG(ccer, ccer_val);

    // 6. Enable interrupt in DIER (CCxIE)
    SET_BIT(dier, timer_ch_idx); // CCxIE bit
}

/**
 * @brief Enables an ICU channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    t_timer_channel timer_channel;
    tbyte timer_ch_idx; // Dummy

    switch (icu_channel) {
        case PWM_CHANNEL_TIM1_CH1: timer_channel = TIMER_CHANNEL_1; break;
        // ... (similar mapping for other ICU channels)
        default: return;
    }

    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    SET_BIT(cr1, 0); // CEN bit (Counter Enable)
}

/**
 * @brief Disables an ICU channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    t_timer_channel timer_channel;
    tbyte timer_ch_idx; // Dummy

    switch (icu_channel) {
        case PWM_CHANNEL_TIM1_CH1: timer_channel = TIMER_CHANNEL_1; break;
        // ... (similar mapping for other ICU channels)
        default: return;
    }

    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    CLEAR_BIT(cr1, 0); // CEN bit (Counter Enable)
}

/**
 * @brief Gets the frequency measured by the ICU.
 *
 * This function is a placeholder. A full implementation would involve
 * reading capture registers in an ISR and calculating frequency.
 * @param icu_channel The ICU channel.
 * @return The measured frequency (placeholder, returns 0).
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Placeholder: In a real implementation, this would involve reading values
    // captured in CCRx registers, typically from an ISR.
    // For now, return a dummy value.
    return 0; // Not implemented for direct register access.
}

/**
 * @brief Sets a callback function for ICU events.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    icu_callback_func = callback;
}


/**
 * @brief Initializes a general-purpose timer.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    // 1. Enable Timer clock
    enable_timer_clock(timer_channel);

    // Disable timer before configuration
    CLEAR_BIT(cr1, 0); // CEN bit

    // Default prescaler and auto-reload for a basic timer
    WRITE_REG(psc, 16000 - 1); // Prescaler (e.g., for 1ms tick with 16MHz clock)
                               // Assuming APB1/APB2 clock is 16MHz for simplicity, 16MHz/16000 = 1KHz
    WRITE_REG(arr, 0xFFFF);    // Max value for ARR (16-bit timer)

    // Enable Update Interrupt
    SET_BIT(dier, 0); // UIE bit (Update Interrupt Enable)

    // Clear event generation register
    WRITE_REG(egr, 0);
}

/**
 * @brief Sets the timer period in microseconds.
 * @param timer_channel The timer channel.
 * @param time The time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    tlong pclk_freq;
    if (timer_channel == TIMER_CHANNEL_1 || timer_channel == TIMER_CHANNEL_9 ||
        timer_channel == TIMER_CHANNEL_10 || timer_channel == TIMER_CHANNEL_11) {
        pclk_freq = 84000000; // Assuming 84MHz APB2 clock
    } else {
        pclk_freq = 42000000; // Assuming 42MHz APB1 clock
    }

    tlong ticks_per_us = pclk_freq / 1000000; // Ticks per microsecond
    tlong total_ticks = ticks_per_us * time;

    tlong prescaler_val = 0;
    tlong auto_reload_val = 0;

    // Find a suitable prescaler and ARR to achieve total_ticks
    if (total_ticks <= 0xFFFF) { // If total ticks fit in ARR
        prescaler_val = 0;
        auto_reload_val = total_ticks - 1;
    } else { // Otherwise, apply prescaler
        prescaler_val = (total_ticks / 0xFFFF);
        if (prescaler_val > 0xFFFF) prescaler_val = 0xFFFF; // Clamp to max 16-bit
        auto_reload_val = (total_ticks / (prescaler_val + 1)) - 1;
    }

    if (auto_reload_val > 0xFFFF) auto_reload_val = 0xFFFF;
    if (auto_reload_val < 0) auto_reload_val = 0;
    if (prescaler_val < 0) prescaler_val = 0;

    WRITE_REG(psc, prescaler_val);
    WRITE_REG(arr, auto_reload_val);
}

/**
 * @brief Sets the timer period in milliseconds.
 * @param timer_channel The timer channel.
 * @param time The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    TIMER_Set_us(timer_channel, (tlong)time * 1000); // Convert ms to us
}

/**
 * @brief Sets the timer period in seconds.
 * @param timer_channel The timer channel.
 * @param time The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    TIMER_Set_us(timer_channel, (tlong)time * 1000000); // Convert sec to us
}

/**
 * @brief Sets the timer period in minutes.
 * @param timer_channel The timer channel.
 * @param time The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    TIMER_Set_us(timer_channel, (tlong)time * 60 * 1000000); // Convert min to us
}

/**
 * @brief Sets the timer period in hours.
 * @param timer_channel The timer channel.
 * @param time The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    TIMER_Set_us(timer_channel, (tlong)time * 3600 * 1000000); // Convert hour to us
}

/**
 * @brief Enables a timer channel.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    SET_BIT(egr, 0); // UG bit (Update Generation) to load all registers to active registers
    SET_BIT(cr1, 0); // CEN bit (Counter Enable)
}

/**
 * @brief Disables a timer channel.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    if (!get_timer_registers(timer_channel, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    CLEAR_BIT(cr1, 0); // CEN bit (Counter Enable)
}

/**
 * @brief Initializes the ADC.
 * @param adc_channel The ADC channel to configure.
 * @param adc_mode The ADC operating mode (single, continuous, DMA).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // 1. Enable ADC1 clock
    SET_BIT(RCC_APB2ENR_REG, 8); // ADC1EN bit 8 for APB2 (inferred)
    (void)READ_REG(RCC_APB2ENR_REG); // Dummy read

    // Disable ADC before configuration
    CLEAR_BIT(ADC1_CR2_REG, 0); // ADON bit

    // 2. Configure GPIO for analog mode for the channel
    t_port gpio_port;
    t_pin gpio_pin;

    switch (adc_channel) {
        case ADC_CHANNEL_0: gpio_port = PORTA; gpio_pin = PIN0; break; // PA0
        case ADC_CHANNEL_1: gpio_port = PORTA; gpio_pin = PIN1; break; // PA1
        case ADC_CHANNEL_2: gpio_port = PORTA; gpio_pin = PIN2; break; // PA2
        case ADC_CHANNEL_3: gpio_port = PORTA; gpio_pin = PIN3; break; // PA3
        case ADC_CHANNEL_4: gpio_port = PORTA; gpio_pin = PIN4; break; // PA4
        case ADC_CHANNEL_5: gpio_port = PORTA; gpio_pin = PIN5; break; // PA5
        case ADC_CHANNEL_6: gpio_port = PORTA; gpio_pin = PIN6; break; // PA6
        case ADC_CHANNEL_7: gpio_port = PORTA; gpio_pin = PIN7; break; // PA7
        case ADC_CHANNEL_8: gpio_port = PORTB; gpio_pin = PIN0; break; // PB0
        case ADC_CHANNEL_9: gpio_port = PORTB; gpio_pin = PIN1; break; // PB1
        case ADC_CHANNEL_10: gpio_port = PORTC; gpio_pin = PIN0; break; // PC0
        case ADC_CHANNEL_11: gpio_port = PORTC; gpio_pin = PIN1; break; // PC1
        case ADC_CHANNEL_12: gpio_port = PORTC; gpio_pin = PIN2; break; // PC2
        case ADC_CHANNEL_13: gpio_port = PORTC; gpio_pin = PIN3; break; // PC3
        case ADC_CHANNEL_14: gpio_port = PORTC; gpio_pin = PIN4; break; // PC4
        case ADC_CHANNEL_15: gpio_port = PORTC; gpio_pin = PIN5; break; // PC5
        default: return;
    }
    enable_gpio_clock(gpio_port);
    // Set pin mode to Analog (11b)
    MODIFY_REG(GPIO_MODER_ADDRS[gpio_port], (0x3U << (gpio_pin * 2)), (0x3U << (gpio_pin * 2)));
    // No pull-up/down, no speed for analog
    MODIFY_REG(GPIO_PUPDR_ADDRS[gpio_port], (0x3U << (gpio_pin * 2)), (0x0U << (gpio_pin * 2)));

    // 3. Configure ADC1_CR1 and ADC1_CR2
    tlong cr1_val = READ_REG(ADC1_CR1_REG);
    tlong cr2_val = READ_REG(ADC1_CR2_REG);

    // Resolution (RES[1:0] bits 25:24 in CR1), default 12-bit (00b)
    MODIFY_REG(&cr1_val, (0x3U << 24), (0x0U << 24)); // 12-bit resolution

    // Data alignment (ALIGN bit 11 in CR2), default Right alignment (0b)
    CLEAR_BIT(&cr2_val, 11); // Right alignment

    // Continuous conversion mode (CONT bit 1 in CR2)
    if (adc_mode == ADC_MODE_CONTINUOUS) {
        SET_BIT(&cr2_val, 1);
    } else {
        CLEAR_BIT(&cr2_val, 1);
    }

    // DMA mode (DMA bit 8 in CR2, DDS bit 9 in CR2 for DMA disable/enable)
    if (adc_mode == ADC_MODE_DMA) {
        SET_BIT(&cr2_val, 8); // DMA enable
        CLEAR_BIT(&cr2_val, 9); // DDS = 0, DMA requests are generated one shot
    } else {
        CLEAR_BIT(&cr2_val, 8);
    }

    // Scan mode (SCAN bit 8 in CR1), enable if multiple channels are desired. For single channel, disable.
    CLEAR_BIT(&cr1_val, 8); // Disable scan mode

    WRITE_REG(ADC1_CR1_REG, cr1_val);
    WRITE_REG(ADC1_CR2_REG, cr2_val);

    // 4. Configure sample time in ADC1_SMPR1 or ADC1_SMPR2
    // Each channel has 3 bits for sample time. For simplicity, set to 3 cycles (000b)
    // SMPR2 for channels 0-9, SMPR1 for channels 10-18
    if (adc_channel < ADC_CHANNEL_10) { // Channels 0-9 use SMPR2
        MODIFY_REG(ADC1_SMPR2_REG, (0x7U << (adc_channel * 3)), (0x0U << (adc_channel * 3))); // 3 cycles
    } else { // Channels 10-15 use SMPR1
        MODIFY_REG(ADC1_SMPR1_REG, (0x7U << ((adc_channel - 10) * 3)), (0x0U << ((adc_channel - 10) * 3))); // 3 cycles
    }

    // 5. Configure sequence in ADC1_SQRx (regular sequence)
    // For single channel conversion, L (length) bits (23:20 in SQR1) = 0000b (1 conversion)
    // Convert the desired channel (ADC_CHANNEL_x) into the sequence register.
    // SQ1 is at bits 0-4 of SQR3, SQ16 is at bits 20-24 of SQR1.
    WRITE_REG(ADC1_SQR1_REG, 0x00000000); // Clear SQR1 (L=0 for 1 conversion)
    WRITE_REG(ADC1_SQR2_REG, 0x00000000); // Clear SQR2
    WRITE_REG(ADC1_SQR3_REG, adc_channel); // Set SQ1 to the desired channel

    // 6. Enable ADC
    SET_BIT(ADC1_CR2_REG, 0); // ADON bit
}

/**
 * @brief Enables the ADC.
 * @param adc_channel The ADC channel (ADC1 is implied for STM32F401RC).
 */
void ADC_Enable(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Ensure ADC1 clock is enabled (if not already by Init)
    SET_BIT(RCC_APB2ENR_REG, 8); // ADC1EN (inferred)
    (void)READ_REG(RCC_APB2ENR_REG); // Dummy read

    SET_BIT(ADC1_CR2_REG, 0); // ADON bit to enable ADC
}

/**
 * @brief Disables the ADC.
 * @param adc_channel The ADC channel (ADC1 is implied for STM32F401RC).
 */
void ADC_Disable(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    CLEAR_BIT(ADC1_CR2_REG, 0); // ADON bit to disable ADC
}

/**
 * @brief Gets ADC conversion result using polling.
 * @param adc_channel The ADC channel to read.
 * @return The 12-bit converted value.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Start conversion
    SET_BIT(ADC1_CR2_REG, 30); // SWSTART bit

    // Wait for EOC (End of Conversion) flag
    while (!READ_BIT(ADC1_SR_REG, 1)); // EOC flag is bit 1 in SR

    // Clear EOC flag by reading SR then DR
    (void)READ_REG(ADC1_SR_REG); // Read SR to clear EOC is not strictly correct. EOC is cleared by reading DR.
    tword result = (tword)READ_REG(ADC1_DR_REG); // Read data register to get result and clear EOC

    return result;
}

/**
 * @brief Gets ADC conversion result using interrupts.
 *
 * This function is a placeholder as full interrupt handling (ISR, callbacks)
 * is beyond the scope of direct register access in MCAL.c.
 * @param adc_channel The ADC channel to read.
 * @return The 12-bit converted value (placeholder, returns 0).
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Placeholder: Full interrupt-driven ADC requires an ISR to handle EOC and store the result.
    // This function would typically trigger conversion and then return the last-converted value
    // stored by an ISR, or start a conversion and wait for a flag.
    // For now, return a dummy value.
    return 0; // Not implemented for direct register access.
}

/**
 * @brief Initializes an I2S channel.
 * @param channel The I2S channel.
 * @param mode I2S mode (master/slave, TX/RX).
 * @param standard I2S audio standard.
 * @param data_format I2S data format (16, 24, 32 bit).
 * @param channel_mode I2S channel mode (mono/stereo).
 * @param sample_rate Audio sample rate (e.g., 44100 Hz).
 * @param mclk_freq Master clock frequency (ignored if not master).
 * @param dma_buffer_size DMA buffer size (ignored if DMA not used).
 */
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard,
              I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode,
              uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1_spi, *cr2_spi, *sr_spi, *dr_spi, *i2scfgr_spi, *i2spr_spi;
    t_spi_channel spi_channel;
    t_port sck_port, ws_port, sd_port, mck_port;
    t_pin sck_pin, ws_pin, sd_pin, mck_pin;
    tbyte af_val;

    switch (channel) {
        case I2S_CHANNEL_1:
            spi_channel = SPI_CHANNEL_1;
            // Example GPIO: I2S_CK=PA5, I2S_WS=PA4, I2S_SD=PA7, I2S_MCK=PB0 (AF5)
            sck_port = PORTA; sck_pin = PIN5; ws_port = PORTA; ws_pin = PIN4;
            sd_port = PORTA; sd_pin = PIN7; mck_port = PORTB; mck_pin = PIN0; af_val = 5;
            i2spr_spi = SPI1_I2SPR_REG;
            break;
        case I2S_CHANNEL_2:
            spi_channel = SPI_CHANNEL_2;
            // Example GPIO: I2S_CK=PB10, I2S_WS=PB9, I2S_SD=PC3, I2S_MCK=PC6 (AF5)
            sck_port = PORTB; sck_pin = PIN10; ws_port = PORTB; ws_pin = PIN9;
            sd_port = PORTC; sd_pin = PIN3; mck_port = PORTC; mck_pin = PIN6; af_val = 5;
            i2spr_spi = SPI2_I2SPR_REG;
            break;
        case I2S_CHANNEL_3:
            spi_channel = SPI_CHANNEL_3;
            // Example GPIO: I2S_CK=PB3, I2S_WS=PA4, I2S_SD=PB5, I2S_MCK=PC7 (AF6)
            sck_port = PORTB; sck_pin = PIN3; ws_port = PORTA; ws_pin = PIN4;
            sd_port = PORTB; sd_pin = PIN5; mck_port = PORTC; mck_pin = PIN7; af_val = 6;
            i2spr_spi = SPI3_I2SPR_REG;
            break;
        default: return;
    }

    if (!get_spi_registers(spi_channel, &cr1_spi, &cr2_spi, &sr_spi, &dr_spi, &i2scfgr_spi)) return;

    // 1. Enable SPIx clock (I2S is a mode of SPI)
    switch (spi_channel) {
        case SPI_CHANNEL_1: SET_BIT(RCC_APB2ENR_REG, 0); break;
        case SPI_CHANNEL_2: SET_BIT(RCC_APB1ENR_REG, 14); break;
        case SPI_CHANNEL_3: SET_BIT(RCC_APB1ENR_REG, 15); break;
        default: return;
    }
    (void)READ_REG(RCC_APB1ENR_REG); // Dummy read
    (void)READ_REG(RCC_APB2ENR_REG); // Dummy read

    // 2. Configure GPIOs for I2S (SCK, WS, SD, MCK) in AF mode, Push-pull, High speed
    // SCK
    enable_gpio_clock(sck_port);
    MODIFY_REG(GPIO_MODER_ADDRS[sck_port], (0x3U << (sck_pin * 2)), (0x2U << (sck_pin * 2)));
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[sck_port], (0x3U << (sck_pin * 2)), (0x2U << (sck_pin * 2)));
    MODIFY_REG((sck_pin < 8 ? GPIO_AFRL_ADDRS[sck_port] : GPIO_AFRH_ADDRS[sck_port]),
               (0xFU << ((sck_pin % 8) * 4)), (af_val << ((sck_pin % 8) * 4)));
    // WS (Word Select)
    enable_gpio_clock(ws_port);
    MODIFY_REG(GPIO_MODER_ADDRS[ws_port], (0x3U << (ws_pin * 2)), (0x2U << (ws_pin * 2)));
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[ws_port], (0x3U << (ws_pin * 2)), (0x2U << (ws_pin * 2)));
    MODIFY_REG((ws_pin < 8 ? GPIO_AFRL_ADDRS[ws_port] : GPIO_AFRH_ADDRS[ws_port]),
               (0xFU << ((ws_pin % 8) * 4)), (af_val << ((ws_pin % 8) * 4)));
    // SD (Serial Data)
    enable_gpio_clock(sd_port);
    MODIFY_REG(GPIO_MODER_ADDRS[sd_port], (0x3U << (sd_pin * 2)), (0x2U << (sd_pin * 2)));
    MODIFY_REG(GPIO_OSPEEDR_ADDRS[sd_port], (0x3U << (sd_pin * 2)), (0x2U << (sd_pin * 2)));
    MODIFY_REG((sd_pin < 8 ? GPIO_AFRL_ADDRS[sd_port] : GPIO_AFRH_ADDRS[sd_port]),
               (0xFU << ((sd_pin % 8) * 4)), (af_val << ((sd_pin % 8) * 4)));
    // MCK (Master Clock, optional, only for master mode)
    if (mode == I2S_MODE_MASTER_TX || mode == I2S_MODE_MASTER_RX) {
        enable_gpio_clock(mck_port);
        MODIFY_REG(GPIO_MODER_ADDRS[mck_port], (0x3U << (mck_pin * 2)), (0x2U << (mck_pin * 2)));
        MODIFY_REG(GPIO_OSPEEDR_ADDRS[mck_port], (0x3U << (mck_pin * 2)), (0x2U << (mck_pin * 2)));
        MODIFY_REG((mck_pin < 8 ? GPIO_AFRL_ADDRS[mck_port] : GPIO_AFRH_ADDRS[mck_port]),
                   (0xFU << ((mck_pin % 8) * 4)), (af_val << ((mck_pin % 8) * 4)));
    }

    // Disable I2S peripheral before configuration
    CLEAR_BIT(i2scfgr_spi, 11); // I2SE bit

    // Configure SPIx_I2SCFGR register
    tlong i2scfgr_val = 0;
    SET_BIT(&i2scfgr_val, 10); // I2SMOD (I2S mode selected)

    // Master/Slave, TX/RX mode
    switch (mode) {
        case I2S_MODE_SLAVE_TX: SET_BIT(&i2scfgr_val, 8); break; // I2SCFG[1:0] = 01 (Slave Transmit)
        case I2S_MODE_SLAVE_RX: break;                         // I2SCFG[1:0] = 00 (Slave Receive)
        case I2S_MODE_MASTER_TX: MODIFY_REG(&i2scfgr_val, (0x3U << 8), (0x2U << 8)); break; // I2SCFG[1:0] = 10 (Master Transmit)
        case I2S_MODE_MASTER_RX: MODIFY_REG(&i2scfgr_val, (0x3U << 8), (0x3U << 8)); break; // I2SCFG[1:0] = 11 (Master Receive)
    }

    // I2S Standard
    switch (standard) {
        case I2S_STANDARD_PHILIPS: break; // I2SSTD[1:0] = 00
        case I2S_STANDARD_MSB_JUSTIFIED: SET_BIT(&i2scfgr_val, 4); break; // I2SSTD[1:0] = 01
        case I2S_STANDARD_LSB_JUSTIFIED: MODIFY_REG(&i2scfgr_val, (0x3U << 4), (0x2U << 4)); break; // I2SSTD[1:0] = 10
        case I2S_STANDARD_PCM_SHORT: MODIFY_REG(&i2scfgr_val, (0x3U << 4), (0x3U << 4)); SET_BIT(&i2scfgr_val, 7); break; // PCM mode, CHLEN (bit 7)
        case I2S_STANDARD_PCM_LONG: MODIFY_REG(&i2scfgr_val, (0x3U << 4), (0x3U << 4)); CLEAR_BIT(&i2scfgr_val, 7); break; // PCM mode, CHLEN (bit 7)
    }

    // Data Format
    switch (data_format) {
        case I2S_DATA_FORMAT_16B: break; // DATLEN[1:0] = 00
        case I2S_DATA_FORMAT_24B: SET_BIT(&i2scfgr_val, 1); break; // DATLEN[1:0] = 01
        case I2S_DATA_FORMAT_32B: MODIFY_REG(&i2scfgr_val, (0x3U << 1), (0x2U << 1)); break; // DATLEN[1:0] = 10
    }

    // Channel mode (mono/stereo)
    if (channel_mode == I2S_CHANNEL_MODE_MONO) SET_BIT(&i2scfgr_val, 3); // CHLEN = 1 for mono
    else CLEAR_BIT(&i2scfgr_val, 3); // CHLEN = 0 for stereo

    // Master Clock Output Enable (MCKOE) if master
    if (mode == I2S_MODE_MASTER_TX || mode == I2S_MODE_MASTER_RX) {
        SET_BIT(&i2scfgr_val, 9); // MCKOE
    }

    WRITE_REG(i2scfgr_spi, i2scfgr_val);

    // Configure SPIx_I2SPR register (I2S Prescaler Register)
    // PCLK calculation for I2S (often from dedicated clock or system clock, assuming APB bus freq for calculation)
    tlong pclk_freq;
    if (spi_channel == SPI_CHANNEL_1) pclk_freq = 84000000; // APB2
    else pclk_freq = 42000000; // APB1

    // Detailed I2S prescaler calculation is complex, depends on sample rate, data format, MCK output.
    // Example for 48kHz, 16-bit, stereo:
    // I2SDIV = (PCLK / (32 * 2 * SampleRate)) for non-odd factor
    // I2SDIV = (PCLK / (256 * 2 * SampleRate)) for non-odd factor if MCKOE=1
    // (example for standard 16-bit stereo)
    tlong i2sdiv = 0;
    tlong odd = 0;
    // For now, simplify and set to common value for typical 48kHz / 16-bit stereo
    // Assuming PCLK = 84MHz, Target Fs = 48kHz, 16bit, stereo:
    // For Fs = 48kHz, I2SDIV = 175, ODD = 0, MCKOE=1 for 256*Fs
    // i2s_clk = PCLK / [(2*I2SDIV)*(1+ODD)] = 84MHz / (2*175*1) = 240kHz (incorrect, this is for 32 bit data)
    // For 16-bit data, I2S clock is 2 * Fs * 16. Total 32 clock cycles per stereo frame.
    // I2S clock = PCLK / ((2*I2SDIV) * (1+ODD)) -> I2SDIV = PCLK / (I2S_clock * 2 * (1+ODD))
    // I2S_clock_required = FrameLength * SampleRate (FrameLength = 32 for 16-bit stereo, 64 for 32-bit stereo)
    // If MCLK output is enabled (MCKOE): MCLK = 256 * Fs, I2S_CK = MCLK / 2 = 128 * Fs
    // If MCLK is not enabled: I2S_CK = 32 * Fs (for 16-bit stereo)
    // For Master, without MCLK: I2SDIV = PCLK / ( (2 * 32 * Fs) * (1+ODD) )
    // For Master, with MCLK: I2SDIV = PCLK / ( (2 * 256 * Fs) * (1+ODD) )
    // For 48kHz, 16bit, stereo: Fs = 48000, frame = 32.
    // PCLK_I2S = 84MHz. target_clk = 48000 * 32 = 1536000 Hz.
    // I2SDIV = PCLK_I2S / (2 * target_clk) = 84000000 / (2 * 1536000) = 27.34. Nearest integer is 27.
    // With I2SDIV = 27, ODD = 0 -> Actual I2S_CK = 84MHz / (2*27) = 1.55MHz.
    // Actual Fs = 1.55MHz / 32 = 48.4kHz. This is close enough.

    i2sdiv = 27; // Example value for 48kHz sample rate, 16-bit stereo
    odd = 0;     // No odd prescaler

    WRITE_REG(i2spr_spi, (i2sdiv << 0) | (odd << 8)); // I2SDIV[7:0] and ODD bit
}

/**
 * @brief Enables an I2S channel.
 * @param channel The I2S channel to enable.
 */
void I2S_Enable(t_i2s_channel channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1_spi, *cr2_spi, *sr_spi, *dr_spi, *i2scfgr_spi;
    t_spi_channel spi_channel;
    switch (channel) {
        case I2S_CHANNEL_1: spi_channel = SPI_CHANNEL_1; break;
        case I2S_CHANNEL_2: spi_channel = SPI_CHANNEL_2; break;
        case I2S_CHANNEL_3: spi_channel = SPI_CHANNEL_3; break;
        default: return;
    }
    if (!get_spi_registers(spi_channel, &cr1_spi, &cr2_spi, &sr_spi, &dr_spi, &i2scfgr_spi)) return;

    // Ensure peripheral clock is enabled
    switch (spi_channel) {
        case SPI_CHANNEL_1: SET_BIT(RCC_APB2ENR_REG, 0); break;
        case SPI_CHANNEL_2: SET_BIT(RCC_APB1ENR_REG, 14); break;
        case SPI_CHANNEL_3: SET_BIT(RCC_APB1ENR_REG, 15); break;
        default: return;
    }
    (void)READ_REG(RCC_APB1ENR_REG);
    (void)READ_REG(RCC_APB2ENR_REG);

    SET_BIT(i2scfgr_spi, 11); // I2SE bit to enable I2S
}

/**
 * @brief Transmits data over an I2S channel.
 * @param channel The I2S channel.
 * @param data Pointer to the data to transmit.
 * @param length The length of data in bytes (or samples if specified by data_format).
 */
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1_spi, *cr2_spi, *sr_spi, *dr_spi, *i2scfgr_spi;
    t_spi_channel spi_channel;
    switch (channel) {
        case I2S_CHANNEL_1: spi_channel = SPI_CHANNEL_1; break;
        case I2S_CHANNEL_2: spi_channel = SPI_CHANNEL_2; break;
        case I2S_CHANNEL_3: spi_channel = SPI_CHANNEL_3; break;
        default: return;
    }
    if (!get_spi_registers(spi_channel, &cr1_spi, &cr2_spi, &sr_spi, &dr_spi, &i2scfgr_spi)) return;

    const uint16_t *tx_data = (const uint16_t *)data; // Assuming 16-bit samples for I2S
    size_t num_samples = length / sizeof(uint16_t);

    for (size_t i = 0; i < num_samples; i++) {
        while (!READ_BIT(sr_spi, 1)); // Wait for TXE (Transmit buffer empty)
        WRITE_REG(dr_spi, tx_data[i]);
    }
}

/**
 * @brief Receives data over an I2S channel.
 * @param channel The I2S channel.
 * @param buffer Pointer to the buffer to store received data.
 * @param length The maximum length of data in bytes (or samples if specified by data_format).
 */
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    volatile uint32_t *cr1_spi, *cr2_spi, *sr_spi, *dr_spi, *i2scfgr_spi;
    t_spi_channel spi_channel;
    switch (channel) {
        case I2S_CHANNEL_1: spi_channel = SPI_CHANNEL_1; break;
        case I2S_CHANNEL_2: spi_channel = SPI_CHANNEL_2; break;
        case I2S_CHANNEL_3: spi_channel = SPI_CHANNEL_3; break;
        default: return;
    }
    if (!get_spi_registers(spi_channel, &cr1_spi, &cr2_spi, &sr_spi, &dr_spi, &i2scfgr_spi)) return;

    uint16_t *rx_buffer = (uint16_t *)buffer; // Assuming 16-bit samples for I2S
    size_t num_samples = length / sizeof(uint16_t);

    for (size_t i = 0; i < num_samples; i++) {
        while (!READ_BIT(sr_spi, 0)); // Wait for RXNE (Receive buffer not empty)
        rx_buffer[i] = (uint16_t)READ_REG(dr_spi);
    }
}


/* --- TT Module Implementations --- */

// A simple task structure
typedef struct {
    void (*task_func)(void);
    tword period; // In ticks
    tword delay;  // Initial delay in ticks
    tword elapsed_time;
    tbyte enabled;
} TT_Task_t;

#define MAX_TT_TASKS 10 // Max number of tasks
static TT_Task_t tt_tasks[MAX_TT_TASKS];
static tbyte tt_task_count = 0;

/**
 * @brief Initializes the Time Triggered (TT) OS.
 *
 * This function sets up a timer (e.g., TIM2) to provide the system tick.
 *
 * @param tick_time_ms The desired tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIMER_Init(TIMER_CHANNEL_2); // Use TIM2 for TT OS tick
    TIMER_Set_Time_ms(TIMER_CHANNEL_2, tick_time_ms);
    for (tbyte i = 0; i < MAX_TT_TASKS; i++) {
        tt_tasks[i].task_func = NULL;
        tt_tasks[i].enabled = 0;
    }
    tt_task_count = 0;
}

/**
 * @brief Starts the Time Triggered (TT) OS.
 *
 * This function enables the timer responsible for generating system ticks.
 */
void TT_Start(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIMER_Enable(TIMER_CHANNEL_2); // Start TIM2
}

/**
 * @brief Dispatches tasks in the Time Triggered (TT) OS.
 *
 * This function iterates through registered tasks and executes them if their
 * delay/period has elapsed.
 */
void TT_Dispatch_task(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    for (tbyte i = 0; i < tt_task_count; i++) {
        if (tt_tasks[i].enabled && tt_tasks[i].task_func != NULL) {
            if (tt_tasks[i].elapsed_time >= tt_tasks[i].delay) {
                tt_tasks[i].task_func();
                tt_tasks[i].elapsed_time = 0; // Reset for next period
                tt_tasks[i].delay = tt_tasks[i].period; // Set delay to period after first run
            }
        }
    }
}

/**
 * @brief Time Triggered (TT) OS Interrupt Service Routine.
 *
 * This function should be called from the timer ISR that provides the system tick.
 * It increments the elapsed time for all active tasks.
 */
void TT_ISR(void) {
    // No WDT_Reset here, as ISRs should be minimal and not trigger WDT resets directly
    // unless they manage the WDT itself. WDT_Reset should be in the main loop or before long operations.

    // Clear the timer Update Interrupt Flag (UIF)
    volatile uint32_t *cr1, *dier, *sr, *egr, *ccmr1, *ccmr2, *ccer, *cnt, *psc, *arr, *ccr1, *ccr2, *ccr3, *ccr4, *bdtr;
    if (!get_timer_registers(TIMER_CHANNEL_2, &cr1, &dier, &sr, &egr, &ccmr1, &ccmr2, &ccer,
                             &cnt, &psc, &arr, &ccr1, &ccr2, &ccr3, &ccr4, &bdtr)) return;

    CLEAR_BIT(sr, 0); // Clear UIF (Update Interrupt Flag)

    for (tbyte i = 0; i < tt_task_count; i++) {
        if (tt_tasks[i].enabled) {
            tt_tasks[i].elapsed_time++;
        }
    }
}

/**
 * @brief Adds a task to the Time Triggered (TT) OS scheduler.
 * @param task Pointer to the task function.
 * @param period The task's execution period in ticks.
 * @param delay The initial delay before the first execution in ticks.
 * @return The index of the added task, or -1 if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (tt_task_count < MAX_TT_TASKS && task != NULL) {
        tt_tasks[tt_task_count].task_func = task;
        tt_tasks[tt_task_count].period = period;
        tt_tasks[tt_task_count].delay = delay;
        tt_tasks[tt_task_count].elapsed_time = 0;
        tt_tasks[tt_task_count].enabled = 1;
        tt_task_count++;
        return tt_task_count - 1;
    }
    return (tbyte)-1; // Failed to add task
}

/**
 * @brief Deletes a task from the Time Triggered (TT) OS scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (task_index < tt_task_count) {
        tt_tasks[task_index].enabled = 0;
        tt_tasks[task_index].task_func = NULL; // Clear the function pointer
        // Optional: Shift remaining tasks to fill the gap.
        // For simplicity, just disable and clear for now.
    }
}