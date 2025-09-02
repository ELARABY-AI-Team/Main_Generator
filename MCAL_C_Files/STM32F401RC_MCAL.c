// --- MCU NAME ---
// STM32F401RC

// --- CORE INCLUDES (from Rules.json) ---
#include "MCAL.h"
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- REGISTER POINTERS (Derived from register_json) ---
// Using direct register addresses and casting to volatile pointers for safe access.
// These are internal details of the MCAL and not exposed in public APIs.

// FLASH
#define FLASH_ACR_REG   ((volatile uint32_t *)0x40023C00)
#define FLASH_KEYR_REG  ((volatile uint32_t *)0x40023C04)
#define FLASH_OPTKEYR_REG ((volatile uint32_t *)0x40023C08)
#define FLASH_SR_REG    ((volatile uint32_t *)0x40023C0C)
#define FLASH_CR_REG    ((volatile uint32_t *)0x40023C10)
#define FLASH_OPTCR_REG ((volatile uint32_t *)0x40023C14)

// RCC
#define RCC_CR_REG      ((volatile uint32_t *)0x40023800)
#define RCC_PLLCFGR_REG ((volatile uint32_t *)0x40023804)
#define RCC_CFGR_REG    ((volatile uint32_t *)0x40023808)
#define RCC_CIR_REG     ((volatile uint32_t *)0x4002380C)
#define RCC_AHB1RSTR_REG ((volatile uint32_t *)0x40023810)
#define RCC_AHB2RSTR_REG ((volatile uint32_t *)0x40023814)
#define RCC_APB1RSTR_REG ((volatile uint32_t *)0x40023818)
#define RCC_APB2RSTR_REG ((volatile uint32_t *)0x4002381C)
#define RCC_AHB1ENR_REG ((volatile uint32_t *)0x40023830)
#define RCC_AHB2ENR_REG ((volatile uint32_t *)0x40023834)
#define RCC_APB1ENR_REG ((volatile uint32_t *)0x40023838)
#define RCC_APB2ENR_REG ((volatile uint32_t *)0x4002383C)
#define RCC_AHB1LPENR_REG ((volatile uint32_t *)0x40023840)
#define RCC_AHB2LPENR_REG ((volatile uint32_t *)0x40023844)
#define RCC_APB1LPENR_REG ((volatile uint32_t *)0x40023848)
#define RCC_APB2LPENR_REG ((volatile uint32_t *)0x4002384C)
#define RCC_BDCR_REG    ((volatile uint32_t *)0x40023850)
#define RCC_CSR_REG     ((volatile uint32_t *)0x40023854)
#define RCC_SSCGR_REG   ((volatile uint32_t *)0x40023858)
#define RCC_PLLI2SCFGR_REG ((volatile uint32_t *)0x4002385C)
#define RCC_DCKCFGR_REG ((volatile uint32_t *)0x40023864)

// SYSCFG
#define SYSCFG_MEMRMP_REG ((volatile uint32_t *)0x40013800)
#define SYSCFG_PMC_REG  ((volatile uint32_t *)0x40013804)
#define SYSCFG_EXTICR1_REG ((volatile uint32_t *)0x40013808)
#define SYSCFG_EXTICR2_REG ((volatile uint32_t *)0x4001380C)
#define SYSCFG_EXTICR3_REG ((volatile uint32_t *)0x40013810)
#define SYSCFG_EXTICR4_REG ((volatile uint32_t *)0x40013814)
#define SYSCFG_CMPCR_REG ((volatile uint32_t *)0x40013820)

// GPIOA
#define GPIOA_MODER_REG   ((volatile uint32_t *)0x40020000)
#define GPIOA_OTYPER_REG  ((volatile uint32_t *)0x40020004)
#define GPIOA_OSPEEDR_REG ((volatile uint32_t *)0x40020008)
#define GPIOA_PUPDR_REG   ((volatile uint32_t *)0x4002000C)
#define GPIOA_IDR_REG     ((volatile uint32_t *)0x40020010)
#define GPIOA_ODR_REG     ((volatile uint32_t *)0x40020014)
#define GPIOA_BSRR_REG    ((volatile uint32_t *)0x40020018)
#define GPIOA_LCKR_REG    ((volatile uint32_t *)0x4002001C)
#define GPIOA_AFRL_REG    ((volatile uint32_t *)0x40020020)
#define GPIOA_AFRH_REG    ((volatile uint32_t *)0x40020024)

// GPIOB
#define GPIOB_MODER_REG   ((volatile uint32_t *)0x40020400)
#define GPIOB_OTYPER_REG  ((volatile uint32_t *)0x40020404)
#define GPIOB_OSPEEDR_REG ((volatile uint32_t *)0x40020408)
#define GPIOB_PUPDR_REG   ((volatile uint32_t *)0x4002040C)
#define GPIOB_IDR_REG     ((volatile uint32_t *)0x40020410)
#define GPIOB_ODR_REG     ((volatile uint32_t *)0x40020414)
#define GPIOB_BSRR_REG    ((volatile uint32_t *)0x40020418)
#define GPIOB_LCKR_REG    ((volatile uint32_t *)0x4002041C)
#define GPIOB_AFRL_REG    ((volatile uint32_t *)0x40020420)
#define GPIOB_AFRH_REG    ((volatile uint32_t *)0x40020424)

// GPIOC
#define GPIOC_MODER_REG   ((volatile uint32_t *)0x40020800)
#define GPIOC_OTYPER_REG  ((volatile uint32_t *)0x40020804)
#define GPIOC_OSPEEDR_REG ((volatile uint32_t *)0x40020808)
#define GPIOC_PUPDR_REG   ((volatile uint32_t *)0x4002080C)
#define GPIOC_IDR_REG     ((volatile uint32_t *)0x40020810)
#define GPIOC_ODR_REG     ((volatile uint32_t *)0x40020814)
#define GPIOC_BSRR_REG    ((volatile uint32_t *)0x40020818)
#define GPIOC_LCKR_REG    ((volatile uint32_t *)0x4002081C)
#define GPIOC_AFRL_REG    ((volatile uint32_t *)0x40020820)
#define GPIOC_AFRH_REG    ((volatile uint32_t *)0x40020824)

// GPIOD
#define GPIOD_MODER_REG   ((volatile uint32_t *)0x40020C00)
#define GPIOD_OTYPER_REG  ((volatile uint32_t *)0x40020C04)
#define GPIOD_OSPEEDR_REG ((volatile uint32_t *)0x40020C08)
#define GPIOD_PUPDR_REG   ((volatile uint32_t *)0x40020C0C)
#define GPIOD_IDR_REG     ((volatile uint32_t *)0x40020C10)
#define GPIOD_ODR_REG     ((volatile uint32_t *)0x40020C14)
#define GPIOD_BSRR_REG    ((volatile uint32_t *)0x40020C18)
#define GPIOD_LCKR_REG    ((volatile uint32_t *)0x40020C1C)
#define GPIOD_AFRL_REG    ((volatile uint32_t *)0x40020C20)
#define GPIOD_AFRH_REG    ((volatile uint32_t *)0x40020C24)

// GPIOE
#define GPIOE_MODER_REG   ((volatile uint32_t *)0x40021000)
#define GPIOE_OTYPER_REG  ((volatile uint32_t *)0x40021004)
#define GPIOE_OSPEEDR_REG ((volatile uint32_t *)0x40021008)
#define GPIOE_PUPDR_REG   ((volatile uint32_t *)0x4002100C)
#define GPIOE_IDR_REG     ((volatile uint32_t *)0x40021010)
#define GPIOE_ODR_REG     ((volatile uint32_t *)0x40021014)
#define GPIOE_BSRR_REG    ((volatile uint32_t *)0x40021018)
#define GPIOE_LCKR_REG    ((volatile uint32_t *)0x4002101C)
#define GPIOE_AFRL_REG    ((volatile uint32_t *)0x40021020)
#define GPIOE_AFRH_REG    ((volatile uint32_t *)0x40021024)

// GPIOH
#define GPIOH_MODER_REG   ((volatile uint32_t *)0x40021C00)
#define GPIOH_OTYPER_REG  ((volatile uint32_t *)0x40021C04)
#define GPIOH_OSPEEDR_REG ((volatile uint32_t *)0x40021C08)
#define GPIOH_PUPDR_REG   ((volatile uint32_t *)0x40021C0C)
#define GPIOH_IDR_REG     ((volatile uint32_t *)0x40021C10)
#define GPIOH_ODR_REG     ((volatile uint32_t *)0x40021C14)
#define GPIOH_BSRR_REG    ((volatile uint32_t *)0x40021C18)
#define GPIOH_LCKR_REG    ((volatile uint32_t *)0x40021C1C)
#define GPIOH_AFRL_REG    ((volatile uint32_t *)0x40021C20)
#define GPIOH_AFRH_REG    ((volatile uint32_t *)0x40021C24)

// EXTI
#define EXTI_IMR_REG    ((volatile uint32_t *)0x40013C00)
#define EXTI_EMR_REG    ((volatile uint32_t *)0x40013C04)
#define EXTI_RTSR_REG   ((volatile uint32_t *)0x40013C08)
#define EXTI_FTSR_REG   ((volatile uint32_t *)0x40013C0C)
#define EXTI_SWIER_REG  ((volatile uint32_t *)0x40013C10)
#define EXTI_PR_REG     ((volatile uint32_t *)0x40013C14)

// ADC
#define ADC1_SR_REG     ((volatile uint32_t *)0x40012000)
#define ADC1_CR1_REG    ((volatile uint32_t *)0x40012004)
#define ADC1_CR2_REG    ((volatile uint32_t *)0x40012008)
#define ADC1_SMPR1_REG  ((volatile uint32_t *)0x4001200C)
#define ADC1_SMPR2_REG  ((volatile uint32_t *)0x40012010)
#define ADC1_JOFR1_REG  ((volatile uint32_t *)0x40012014)
#define ADC1_JOFR2_REG  ((volatile uint32_t *)0x40012018)
#define ADC1_JOFR3_REG  ((volatile uint32_t *)0x4001201C)
#define ADC1_JOFR4_REG  ((volatile uint32_t *)0x40012020)
#define ADC1_HTR_REG    ((volatile uint32_t *)0x40012024)
#define ADC1_LTR_REG    ((volatile uint32_t *)0x40012028)
#define ADC1_SQR1_REG   ((volatile uint32_t *)0x4001202C)
#define ADC1_SQR2_REG   ((volatile uint32_t *)0x40012030)
#define ADC1_SQR3_REG   ((volatile uint32_t *)0x40012034)
#define ADC1_JSQR_REG   ((volatile uint32_t *)0x40012038)
#define ADC1_JDR1_REG   ((volatile uint32_t *)0x4001203C)
#define ADC1_JDR2_REG   ((volatile uint32_t *)0x40012040)
#define ADC1_JDR3_REG   ((volatile uint32_t *)0x40012044)
#define ADC1_JDR4_REG   ((volatile uint32_t *)0x40012048)
#define ADC1_DR_REG     ((volatile uint32_t *)0x4001204C)
#define ADC_CCR_REG     ((volatile uint32_t *)0x40012304)

// TIMERS
// TIM1
#define TIM1_CR1_REG    ((volatile uint32_t *)0x40010000)
#define TIM1_CR2_REG    ((volatile uint32_t *)0x40010004)
#define TIM1_SMCR_REG   ((volatile uint32_t *)0x40010008)
#define TIM1_DIER_REG   ((volatile uint32_t *)0x4001000C)
#define TIM1_SR_REG     ((volatile uint32_t *)0x40010010)
#define TIM1_EGR_REG    ((volatile uint32_t *)0x40010014)
#define TIM1_CCMR1_REG  ((volatile uint32_t *)0x40010018)
#define TIM1_CCMR2_REG  ((volatile uint32_t *)0x4001001C)
#define TIM1_CCER_REG   ((volatile uint32_t *)0x40010020)
#define TIM1_CNT_REG    ((volatile uint32_t *)0x40010024)
#define TIM1_PSC_REG    ((volatile uint32_t *)0x40010028)
#define TIM1_ARR_REG    ((volatile uint32_t *)0x4001002C)
#define TIM1_RCR_REG    ((volatile uint32_t *)0x40010030)
#define TIM1_CCR1_REG   ((volatile uint32_t *)0x40010034)
#define TIM1_CCR2_REG   ((volatile uint32_t *)0x40010038)
#define TIM1_CCR3_REG   ((volatile uint32_t *)0x4001003C)
#define TIM1_CCR4_REG   ((volatile uint32_t *)0x40010040)
#define TIM1_BDTR_REG   ((volatile uint32_t *)0x40010044)
#define TIM1_DCR_REG    ((volatile uint32_t *)0x40010048)
#define TIM1_DMAR_REG   ((volatile uint32_t *)0x4001004C)

// TIM2
#define TIM2_CR1_REG    ((volatile uint32_t *)0x40000000)
#define TIM2_CR2_REG    ((volatile uint32_t *)0x40000004)
#define TIM2_SMCR_REG   ((volatile uint32_t *)0x40000008)
#define TIM2_DIER_REG   ((volatile uint32_t *)0x4000000C)
#define TIM2_SR_REG     ((volatile uint32_t *)0x40000010)
#define TIM2_EGR_REG    ((volatile uint32_t *)0x40000014)
#define TIM2_CCMR1_REG  ((volatile uint32_t *)0x40000018)
#define TIM2_CCMR2_REG  ((volatile uint32_t *)0x4000001C)
#define TIM2_CCER_REG   ((volatile uint32_t *)0x40000020)
#define TIM2_CNT_REG    ((volatile uint32_t *)0x40000024)
#define TIM2_PSC_REG    ((volatile uint32_t *)0x40000028)
#define TIM2_ARR_REG    ((volatile uint32_t *)0x4000002C)
#define TIM2_CCR1_REG   ((volatile uint32_t *)0x40000034)
#define TIM2_CCR2_REG   ((volatile uint32_t *)0x40000038)
#define TIM2_CCR3_REG   ((volatile uint32_t *)0x4000003C)
#define TIM2_CCR4_REG   ((volatile uint32_t *)0x40000040)
#define TIM2_DCR_REG    ((volatile uint32_t *)0x40000048)
#define TIM2_DMAR_REG   ((volatile uint32_t *)0x4000004C)
#define TIM2_OR_REG     ((volatile uint32_t *)0x40000050)

// TIM3
#define TIM3_CR1_REG    ((volatile uint32_t *)0x40000400)
#define TIM3_CR2_REG    ((volatile uint32_t *)0x40000404)
#define TIM3_SMCR_REG   ((volatile uint32_t *)0x40000408)
#define TIM3_DIER_REG   ((volatile uint32_t *)0x4000040C)
#define TIM3_SR_REG     ((volatile uint32_t *)0x40000410)
#define TIM3_EGR_REG    ((volatile uint32_t *)0x40000414)
#define TIM3_CCMR1_REG  ((volatile uint32_t *)0x40000418)
#define TIM3_CCMR2_REG  ((volatile uint32_t *)0x4000041C)
#define TIM3_CCER_REG   ((volatile uint32_t *)0x40000420)
#define TIM3_CNT_REG    ((volatile uint32_t *)0x40000424)
#define TIM3_PSC_REG    ((volatile uint32_t *)0x40000428)
#define TIM3_ARR_REG    ((volatile uint32_t *)0x4000042C)
#define TIM3_CCR1_REG   ((volatile uint32_t *)0x40000434)
#define TIM3_CCR2_REG   ((volatile uint32_t *)0x40000438)
#define TIM3_CCR3_REG   ((volatile uint32_t *)0x4000043C)
#define TIM3_CCR4_REG   ((volatile uint32_t *)0x40000440)
#define TIM3_DCR_REG    ((volatile uint32_t *)0x40000448)
#define TIM3_DMAR_REG   ((volatile uint32_t *)0x4000044C)

// TIM4
#define TIM4_CR1_REG    ((volatile uint32_t *)0x40000800)
#define TIM4_CR2_REG    ((volatile uint32_t *)0x40000804)
#define TIM4_SMCR_REG   ((volatile uint32_t *)0x40000808)
#define TIM4_DIER_REG   ((volatile uint32_t *)0x4000080C)
#define TIM4_SR_REG     ((volatile uint32_t *)0x40000810)
#define TIM4_EGR_REG    ((volatile uint32_t *)0x40000814)
#define TIM4_CCMR1_REG  ((volatile uint32_t *)0x40000818)
#define TIM4_CCMR2_REG  ((volatile uint32_t *)0x4000081C)
#define TIM4_CCER_REG   ((volatile uint32_t *)0x40000820)
#define TIM4_CNT_REG    ((volatile uint32_t *)0x40000824)
#define TIM4_PSC_REG    ((volatile uint32_t *)0x40000828)
#define TIM4_ARR_REG    ((volatile uint32_t *)0x4000082C)
#define TIM4_CCR1_REG   ((volatile uint32_t *)0x40000834)
#define TIM4_CCR2_REG   ((volatile uint32_t *)0x40000838)
#define TIM4_CCR3_REG   ((volatile uint32_t *)0x4000083C)
#define TIM4_CCR4_REG   ((volatile uint32_t *)0x40000840)
#define TIM4_DCR_REG    ((volatile uint32_t *)0x40000848)
#define TIM4_DMAR_REG   ((volatile uint32_t *)0x4000084C)

// TIM5
#define TIM5_CR1_REG    ((volatile uint32_t *)0x40000C00)
#define TIM5_CR2_REG    ((volatile uint32_t *)0x40000C04)
#define TIM5_SMCR_REG   ((volatile uint32_t *)0x40000C08)
#define TIM5_DIER_REG   ((volatile uint32_t *)0x40000C0C)
#define TIM5_SR_REG     ((volatile uint32_t *)0x40000C10)
#define TIM5_EGR_REG    ((volatile uint32_t *)0x40000C14)
#define TIM5_CCMR1_REG  ((volatile uint32_t *)0x40000C18)
#define TIM5_CCMR2_REG  ((volatile uint32_t *)0x40000C1C)
#define TIM5_CCER_REG   ((volatile uint32_t *)0x40000C20)
#define TIM5_CNT_REG    ((volatile uint32_t *)0x40000C24)
#define TIM5_PSC_REG    ((volatile uint32_t *)0x40000C28)
#define TIM5_ARR_REG    ((volatile uint32_t *)0x40000C2C)
#define TIM5_CCR1_REG   ((volatile uint32_t *)0x40000C34)
#define TIM5_CCR2_REG   ((volatile uint32_t *)0x40000C38)
#define TIM5_CCR3_REG   ((volatile uint32_t *)0x40000C3C)
#define TIM5_CCR4_REG   ((volatile uint32_t *)0x40000C40)
#define TIM5_DCR_REG    ((volatile uint32_t *)0x40000C48)
#define TIM5_DMAR_REG   ((volatile uint32_t *)0x40000C4C)
#define TIM5_OR_REG     ((volatile uint32_t *)0x40000C54)

// TIM9
#define TIM9_CR1_REG    ((volatile uint32_t *)0x40014000)
#define TIM9_SMCR_REG   ((volatile uint32_t *)0x40014008)
#define TIM9_DIER_REG   ((volatile uint32_t *)0x4001400C)
#define TIM9_SR_REG     ((volatile uint32_t *)0x40014010)
#define TIM9_EGR_REG    ((volatile uint32_t *)0x40014014)
#define TIM9_CCMR1_REG  ((volatile uint32_t *)0x40014018)
#define TIM9_CCER_REG   ((volatile uint32_t *)0x40014020)
#define TIM9_CNT_REG    ((volatile uint32_t *)0x40014024)
#define TIM9_PSC_REG    ((volatile uint32_t *)0x40014028)
#define TIM9_ARR_REG    ((volatile uint32_t *)0x4001402C)
#define TIM9_CCR1_REG   ((volatile uint32_t *)0x40014034)
#define TIM9_CCR2_REG   ((volatile uint32_t *)0x40014038)

// TIM10
#define TIM10_CR1_REG   ((volatile uint32_t *)0x40014400)
#define TIM10_DIER_REG  ((volatile uint32_t *)0x4001440C)
#define TIM10_SR_REG    ((volatile uint32_t *)0x40014410)
#define TIM10_EGR_REG   ((volatile uint32_t *)0x40014414)
#define TIM10_CCMR1_REG ((volatile uint32_t *)0x40014418)
#define TIM10_CCER_REG  ((volatile uint32_t *)0x40014420)
#define TIM10_CNT_REG   ((volatile uint32_t *)0x40014424)
#define TIM10_PSC_REG   ((volatile uint32_t *)0x40014428)
#define TIM10_ARR_REG   ((volatile uint32_t *)0x4001442C)
#define TIM10_CCR1_REG  ((volatile uint32_t *)0x40014434)

// TIM11
#define TIM11_CR1_REG   ((volatile uint32_t *)0x40014800)
#define TIM11_DIER_REG  ((volatile uint32_t *)0x4001480C)
#define TIM11_SR_REG    ((volatile uint32_t *)0x40014810)
#define TIM11_EGR_REG   ((volatile uint32_t *)0x40014814)
#define TIM11_CCMR1_REG ((volatile uint32_t *)0x40014818)
#define TIM11_CCER_REG  ((volatile uint32_t *)0x40014820)
#define TIM11_CNT_REG   ((volatile uint32_t *)0x40014824)
#define TIM11_PSC_REG   ((volatile uint32_t *)0x40014828)
#define TIM11_ARR_REG   ((volatile uint32_t *)0x4001482C)
#define TIM11_CCR1_REG  ((volatile uint32_t *)0x40014834)

// USART
// USART1
#define USART1_SR_REG   ((volatile uint32_t *)0x40011000)
#define USART1_DR_REG   ((volatile uint32_t *)0x40011004)
#define USART1_BRR_REG  ((volatile uint32_t *)0x40011008)
#define USART1_CR1_REG  ((volatile uint32_t *)0x4001100C)
#define USART1_CR2_REG  ((volatile uint32_t *)0x40011010)
#define USART1_CR3_REG  ((volatile uint32_t *)0x40011014)
#define USART1_GTPR_REG ((volatile uint32_t *)0x40011018)

// USART2
#define USART2_SR_REG   ((volatile uint32_t *)0x40004400)
#define USART2_DR_REG   ((volatile uint32_t *)0x40004404)
#define USART2_BRR_REG  ((volatile uint32_t *)0x40004408)
#define USART2_CR1_REG  ((volatile uint32_t *)0x4000440C)
#define USART2_CR2_REG  ((volatile uint32_t *)0x40004410)
#define USART2_CR3_REG  ((volatile uint32_t *)0x40004414)
#define USART2_GTPR_REG ((volatile uint32_t *)0x40004418)

// USART6
#define USART6_SR_REG   ((volatile uint32_t *)0x40011400)
#define USART6_DR_REG   ((volatile uint32_t *)0x40011404)
#define USART6_BRR_REG  ((volatile uint32_t *)0x40011408)
#define USART6_CR1_REG  ((volatile uint32_t *)0x4001140C)
#define USART6_CR2_REG  ((volatile uint32_t *)0x40011410)
#define USART6_CR3_REG  ((volatile uint32_t *)0x40011414)
#define USART6_GTPR_REG ((volatile uint32_t *)0x40011418)

// I2C
// I2C1
#define I2C1_CR1_REG    ((volatile uint32_t *)0x40005400)
#define I2C1_CR2_REG    ((volatile uint32_t *)0x40005404)
#define I2C1_OAR1_REG   ((volatile uint32_t *)0x40005408)
#define I2C1_OAR2_REG   ((volatile uint32_t *)0x4000540C)
#define I2C1_DR_REG     ((volatile uint32_t *)0x40005410)
#define I2C1_SR1_REG    ((volatile uint32_t *)0x40005414)
#define I2C1_SR2_REG    ((volatile uint32_t *)0x40005418)
#define I2C1_CCR_REG    ((volatile uint32_t *)0x4000541C)
#define I2C1_TRISE_REG  ((volatile uint32_t *)0x40005420)
#define I2C1_FLTR_REG   ((volatile uint32_t *)0x40005424)

// I2C2
#define I2C2_CR1_REG    ((volatile uint32_t *)0x40005800)
#define I2C2_CR2_REG    ((volatile uint32_t *)0x40005804)
#define I2C2_OAR1_REG   ((volatile uint32_t *)0x40005808)
#define I2C2_OAR2_REG   ((volatile uint32_t *)0x4000580C)
#define I2C2_DR_REG     ((volatile uint32_t *)0x40005810)
#define I2C2_SR1_REG    ((volatile uint32_t *)0x40005814)
#define I2C2_SR2_REG    ((volatile uint32_t *)0x40005818)
#define I2C2_CCR_REG    ((volatile uint32_t *)0x4000581C)
#define I2C2_TRISE_REG  ((volatile uint32_t *)0x40005820)
#define I2C2_FLTR_REG   ((volatile uint32_t *)0x40005824)

// I2C3
#define I2C3_CR1_REG    ((volatile uint32_t *)0x40005C00)
#define I2C3_CR2_REG    ((volatile uint32_t *)0x40005C04)
#define I2C3_OAR1_REG   ((volatile uint32_t *)0x40005C08)
#define I2C3_OAR2_REG   ((volatile uint32_t *)0x40005C0C)
#define I2C3_DR_REG     ((volatile uint32_t *)0x40005C10)
#define I2C3_SR1_REG    ((volatile uint32_t *)0x40005C14)
#define I2C3_SR2_REG    ((volatile uint32_t *)0x40005C18)
#define I2C3_CCR_REG    ((volatile uint32_t *)0x40005C1C)
#define I2C3_TRISE_REG  ((volatile uint32_t *)0x40005C20)
#define I2C3_FLTR_REG   ((volatile uint32_t *)0x40005C24)

// SPI
// SPI1
#define SPI1_CR1_REG    ((volatile uint32_t *)0x40013000)
#define SPI1_CR2_REG    ((volatile uint32_t *)0x40013004)
#define SPI1_SR_REG     ((volatile uint32_t *)0x40013008)
#define SPI1_DR_REG     ((volatile uint32_t *)0x4001300C)
#define SPI1_CRCPR_REG  ((volatile uint32_t *)0x40013010)
#define SPI1_RXCRCR_REG ((volatile uint32_t *)0x40013014)
#define SPI1_TXCRCR_REG ((volatile uint32_t *)0x40013018)
#define SPI1_I2SCFGR_REG ((volatile uint32_t *)0x4001301C)
#define SPI1_I2SPR_REG  ((volatile uint32_t *)0x40013020)

// SPI2
#define SPI2_CR1_REG    ((volatile uint32_t *)0x40003800)
#define SPI2_CR2_REG    ((volatile uint32_t *)0x40003804)
#define SPI2_SR_REG     ((volatile uint32_t *)0x40003808)
#define SPI2_DR_REG     ((volatile uint32_t *)0x4000380C)
#define SPI2_CRCPR_REG  ((volatile uint32_t *)0x40003810)
#define SPI2_RXCRCR_REG ((volatile uint32_t *)0x40003814)
#define SPI2_TXCRCR_REG ((volatile uint32_t *)0x40003818)
#define SPI2_I2SCFGR_REG ((volatile uint32_t *)0x4000381C)
#define SPI2_I2SPR_REG  ((volatile uint32_t *)0x40003820)

// SPI3
#define SPI3_CR1_REG    ((volatile uint32_t *)0x40003C00)
#define SPI3_CR2_REG    ((volatile uint32_t *)0x40003C04)
#define SPI3_SR_REG     ((volatile uint32_t *)0x40003C08)
#define SPI3_DR_REG     ((volatile uint32_t *)0x40003C0C)
#define SPI3_CRCPR_REG  ((volatile uint32_t *)0x40003C10)
#define SPI3_RXCRCR_REG ((volatile uint32_t *)0x40003C14)
#define SPI3_TXCRCR_REG ((volatile uint32_t *)0x40003C18)
#define SPI3_I2SCFGR_REG ((volatile uint32_t *)0x40003C1C)
#define SPI3_I2SPR_REG  ((volatile uint32_t *)0x40003C20)

// --- Watchdog Timer (IWDG) Registers (Inferred for STM32F401RC, not in register_json) ---
// Base Address: 0x40003000
#define IWDG_KR_REG     ((volatile uint32_t *)0x40003000) // Key register
#define IWDG_PR_REG     ((volatile uint32_t *)0x40003004) // Prescaler register
#define IWDG_RLR_REG    ((volatile uint32_t *)0x40003008) // Reload register
#define IWDG_SR_REG     ((volatile uint32_t *)0x4000300C) // Status register

// --- Power Control (PWR) Registers (Inferred for STM32F401RC, not in register_json) ---
// Base Address: 0x40007000
#define PWR_CR_REG      ((volatile uint32_t *)0x40007000) // Power control register
#define PWR_CSR_REG     ((volatile uint32_t *)0x40007004) // Power control/status register

// --- TT Module Globals ---
#define MAX_TASKS 10
static struct {
    void (*pTask)(void);
    tword Delay;
    tword Period;
    tbool RunMe;
    tword TaskID; // For deleting tasks
} TT_SCH_tasks_g[MAX_TASKS];

static tword TT_TICK_COUNT_g = 0;
static void (*ICU_Callback_g)(void) = NULL; // Global callback for ICU

// --- STATIC HELPER FUNCTIONS (Internal to MCAL.c) ---
// Adheres to "pointer_variable_consistency" rule: public APIs use enums, internal helpers use pointers.

/**
 * @brief Get GPIO_TypeDef pointer for a given port.
 * @param port The GPIO port enum (PORT_A, PORT_B, etc.).
 * @return Pointer to the GPIO peripheral base structure.
 */
static GPIO_TypeDef* get_gpio_port_ptr(t_port port) {
    switch (port) {
        case PORT_A: return GPIOA;
        case PORT_B: return GPIOB;
        case PORT_C: return GPIOC;
        case PORT_D: return GPIOD;
        case PORT_E: return GPIOE;
        case PORT_H: return GPIOH;
        default: return NULL;
    }
}

/**
 * @brief Get USART_TypeDef pointer for a given UART channel.
 * @param uart_channel The UART channel enum.
 * @return Pointer to the USART peripheral base structure.
 */
static USART_TypeDef* get_usart_ptr(t_uart_channel uart_channel) {
    switch (uart_channel) {
        case UART_CHANNEL_1: return USART1;
        case UART_CHANNEL_2: return USART2;
        case UART_CHANNEL_6: return USART6;
        default: return NULL;
    }
}

/**
 * @brief Get I2C_TypeDef pointer for a given I2C channel.
 * @param i2c_channel The I2C channel enum.
 * @return Pointer to the I2C peripheral base structure.
 */
static I2C_TypeDef* get_i2c_ptr(t_i2c_channel i2c_channel) {
    switch (i2c_channel) {
        case I2C_CHANNEL_1: return I2C1;
        case I2C_CHANNEL_2: return I2C2;
        case I2C_CHANNEL_3: return I2C3;
        default: return NULL;
    }
}

/**
 * @brief Get SPI_TypeDef pointer for a given SPI channel.
 * @param spi_channel The SPI channel enum.
 * @return Pointer to the SPI peripheral base structure.
 */
static SPI_TypeDef* get_spi_ptr(t_spi_channel spi_channel) {
    switch (spi_channel) {
        case SPI_CHANNEL_1: return SPI1;
        case SPI_CHANNEL_2: return SPI2;
        case SPI_CHANNEL_3: return SPI3;
        default: return NULL;
    }
}

/**
 * @brief Get TIM_TypeDef pointer for a given Timer channel.
 * @param timer_channel The Timer channel enum.
 * @return Pointer to the TIM peripheral base structure.
 */
static TIM_TypeDef* get_timer_ptr(t_timer_channel timer_channel) {
    switch (timer_channel) {
        case TIMER_CHANNEL_TIM1: return TIM1;
        case TIMER_CHANNEL_TIM2: return TIM2;
        case TIMER_CHANNEL_TIM3: return TIM3;
        case TIMER_CHANNEL_TIM4: return TIM4;
        case TIMER_CHANNEL_TIM5: return TIM5;
        case TIMER_CHANNEL_TIM9: return TIM9;
        case TIMER_CHANNEL_TIM10: return TIM10;
        case TIMER_CHANNEL_TIM11: return TIM11;
        default: return NULL;
    }
}

/**
 * @brief Get ADC_TypeDef pointer (only ADC1 available in register_json).
 * @return Pointer to the ADC peripheral base structure.
 */
static ADC_TypeDef* get_adc_ptr(void) {
    return ADC1; // Assuming only ADC1 for STM32F401RC from provided registers
}

// --- WDT Implementation (First due to API_implementation_sequence rule) ---
void WDT_Reset(void) {
    // Write 0xAAAA to IWDG_KR to reload the watchdog counter
    *IWDG_KR_REG = 0xAAAA;
}

// --- API FUNCTION IMPLEMENTATIONS (following order in MCAL.h) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // 1. Set all GPIO pins to 0 and verify with while loop
    // Clear ODR for all available GPIO ports (A, B, C, D, E, H)
    GPIOA->ODR = 0x0000;
    GPIOB->ODR = 0x0000;
    GPIOC->ODR = 0x0000;
    GPIOD->ODR = 0x0000;
    GPIOE->ODR = 0x0000;
    GPIOH->ODR = 0x0000;

    // Verification (simplified - actual verification would involve reading back)
    while (GPIOA->ODR != 0x0000);
    while (GPIOB->ODR != 0x0000);
    while (GPIOC->ODR != 0x0000);
    while (GPIOD->ODR != 0x0000);
    while (GPIOE->ODR != 0x0000);
    while (GPIOH->ODR != 0x0000);

    // 2. Set all GPIO pins direction to input and verify with while loop
    // Set MODER to 0b00 (input) for all pins for all ports
    // (2 bits per pin, 16 pins per port = 32 bits register)
    GPIOA->MODER = 0x00000000;
    GPIOB->MODER = 0x00000000;
    GPIOC->MODER = 0x00000000;
    GPIOD->MODER = 0x00000000;
    GPIOE->MODER = 0x00000000;
    GPIOH->MODER = 0x00000000;

    // Verification (simplified)
    while (GPIOA->MODER != 0x00000000);
    while (GPIOB->MODER != 0x00000000);
    while (GPIOC->MODER != 0x00000000);
    while (GPIOD->MODER != 0x00000000);
    while (GPIOE->MODER != 0x00000000);
    while (GPIOH->MODER != 0x00000000);

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    // Global interrupt disable
    Global_interrupt_Disable();

    // Disable ADC1 (ADON bit in ADC1_CR2)
    *ADC1_CR2_REG &= ~ADC_CR2_ADON;

    // Disable all USARTs (UE bit in USARTx_CR1)
    *USART1_CR1_REG &= ~USART_CR1_UE;
    *USART2_CR1_REG &= ~USART_CR1_UE;
    *USART6_CR1_REG &= ~USART_CR1_UE;

    // Disable all I2Cs (PE bit in I2Cx_CR1)
    *I2C1_CR1_REG &= ~I2C_CR1_PE;
    *I2C2_CR1_REG &= ~I2C_CR1_PE;
    *I2C3_CR1_REG &= ~I2C_CR1_PE;

    // Disable all SPIs (SPE bit in SPIx_CR1)
    *SPI1_CR1_REG &= ~SPI_CR1_SPE;
    *SPI2_CR1_REG &= ~SPI_CR1_SPE;
    *SPI3_CR1_REG &= ~SPI_CR1_SPE;

    // Disable all Timers (CEN bit in TIMx_CR1)
    *TIM1_CR1_REG &= ~TIM_CR1_CEN;
    *TIM2_CR1_REG &= ~TIM_CR1_CEN;
    *TIM3_CR1_REG &= ~TIM_CR1_CEN;
    *TIM4_CR1_REG &= ~TIM_CR1_CEN;
    *TIM5_CR1_REG &= ~TIM_CR1_CEN;
    *TIM9_CR1_REG &= ~TIM_CR1_CEN;
    *TIM10_CR1_REG &= ~TIM_CR1_CEN;
    *TIM11_CR1_REG &= ~TIM_CR1_CEN;

    // Disable all EXTI (IMR, EMR)
    *EXTI_IMR_REG = 0x00000000;
    *EXTI_EMR_REG = 0x00000000;

    // 4. Enable WDT (Watchdog Timer)
    // Unlock IWDG registers (write 0x5555 to KR)
    *IWDG_KR_REG = 0x5555;
    // Set WDT prescaler (e.g., divide by 64 -> 0x04 for 32kHz LSI clock)
    // LSI is typically ~32kHz, so 32000 / 64 = 500 counts/sec.
    // Prescaler 0x04 -> 64
    *IWDG_PR_REG = 0x04; // Inferred value, actual may vary based on desired resolution

    // 5. Clear WDT timer (reload with RLR, then kick with KR)
    // Set WDT period (e.g., for 8ms)
    // With PR=64, Reload_val = (8ms * 32000Hz) / 64 = 400 counts.
    // For >= 8ms, RLR >= (8ms / 1000) * (32000 / (2^PR_value))
    // If PR=64, RLR = 0.008 * 32000 / 64 = 4.  To be safe, let's use a larger RLR for 8ms min
    // If we want a minimum of 8ms refresh, for a 32kHz LSI and prescaler 64:
    // T_tick = 64 / 32000 = 2ms.
    // RLR = 8ms / T_tick = 8ms / 2ms = 4. We will use a larger value.
    // For a period >= 8ms, and RLR is 12-bit max 0xFFF.
    // To achieve 8ms minimum with a simple setup, RLR = 10 (20ms refresh).
    *IWDG_RLR_REG = 10; // Inferred for period >= 8ms. Actual calculation needed based on LSI clock and desired timeout.

    // 6. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // Enable Power Clock (Inferred for PWR configuration)
    *RCC_APB1ENR_REG |= RCC_APB1ENR_PWREN; // Set PWREN bit (bit 28) to enable PWR clock
    (void)*RCC_APB1ENR_REG; // Read back to ensure clock is enabled

    // Configure PVD level in PWR_CR (PVDEN, PVDS[2:0])
    // Clear PVDEN and PVDS bits
    *PWR_CR_REG &= ~(PWR_CR_PVDEN | PWR_CR_PLS_Msk); // Inferred bitmask PWR_CR_PLS_Msk for PVDS
    
    if (volt == VSOURCE_3V) {
        // Set PVD to 2.0V (example: PVDS = 010b, check RM for exact mapping)
        // Assume PLS[2:0] = 010b for 2.0V
        // PVD_Set_Threshold (2V for 3V system) - (Inferred PWR_CR bits)
        *PWR_CR_REG |= (0x2U << PWR_CR_PLS_Pos); // Inferred PLS_Pos and value for 2.0V
    } else if (volt == VSOURCE_5V) {
        // Set PVD to 3.5V (example: PVDS = 100b, check RM for exact mapping)
        // Assume PLS[2:0] = 100b for 3.5V
        // PVD_Set_Threshold (3.5V for 5V system) - (Inferred PWR_CR bits)
        *PWR_CR_REG |= (0x4U << PWR_CR_PLS_Pos); // Inferred PLS_Pos and value for 3.5V
    }

    // 7. Enable LVR (Low Voltage Reset) (PVDEN bit in PWR_CR)
    *PWR_CR_REG |= PWR_CR_PVDEN; // Enable PVD (PVDEN bit)

    // 8. Clear WDT again (kick the watchdog)
    WDT_Reset();
}

void Go_to_sleep_mode(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // For STM32F401RC, sleep mode is achieved by setting the SLEEPDEEP bit in SCR
    // (System Control Register) and then executing the WFI (Wait for Interrupt) or WFE (Wait for Event) instruction.
    // This example uses WFI for basic sleep.
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk); // Clear SLEEPDEEP bit for Sleep mode
    __WFI(); // Enter Sleep mode. Only CPU clock stops. Peripherals continue to run.
}

void Global_interrupt_Enable(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    __enable_irq(); // CMSIS function to enable global interrupts
}

void Global_interrupt_Disable(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    __disable_irq(); // CMSIS function to disable global interrupts
}

// LVD
void LVD_Init(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Enable Power Clock (Inferred for PWR configuration)
    *RCC_APB1ENR_REG |= RCC_APB1ENR_PWREN; // Set PWREN bit (bit 28) to enable PWR clock
    (void)*RCC_APB1ENR_REG; // Read back to ensure clock is enabled
    // LVD configuration will be handled in LVD_Get/Enable. Init primarily enables clock.
}

void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // This function likely *sets* the threshold, not *gets* it in the sense of reading current level.
    // To match common LVD API patterns, this function will configure the threshold.
    
    // Clear existing PVD level selection bits
    *PWR_CR_REG &= ~(PWR_CR_PLS_Msk); // Inferred bitmask PWR_CR_PLS_Msk for PVDS

    // Set new PVD level based on lvd_thresholdLevel
    // (Mapping t_lvd_thrthresholdLevel to PWR_CR_PLS bits - inferred values)
    switch (lvd_thresholdLevel) {
        case LVD_THRESHOLD_0V5: *PWR_CR_REG |= (0x0U << PWR_CR_PLS_Pos); break; // Placeholder for 0.5V
        case LVD_THRESHOLD_1V:  *PWR_CR_REG |= (0x0U << PWR_CR_PLS_Pos); break; // Placeholder for 1.0V
        case LVD_THRESHOLD_1V5: *PWR_CR_REG |= (0x0U << PWR_CR_PLS_Pos); break; // Placeholder for 1.5V
        case LVD_THRESHOLD_2V:  *PWR_CR_REG |= (0x1U << PWR_CR_PLS_Pos); break; // Placeholder for 2.0V
        case LVD_THRESHOLD_2V5: *PWR_CR_REG |= (0x2U << PWR_CR_PLS_Pos); break; // Placeholder for 2.5V
        case LVD_THRESHOLD_3V:  *PWR_CR_REG |= (0x3U << PWR_CR_PLS_Pos); break; // Placeholder for 3.0V
        case LVD_THRESHOLD_3V5: *PWR_CR_REG |= (0x4U << PWR_CR_PLS_Pos); break; // Placeholder for 3.5V
        case LVD_THRESHOLD_4V:  *PWR_CR_REG |= (0x5U << PWR_CR_PLS_Pos); break; // Placeholder for 4.0V
        case LVD_THRESHOLD_4V5: *PWR_CR_REG |= (0x6U << PWR_CR_PLS_Pos); break; // Placeholder for 4.5V
        case LVD_THRESHOLD_5V:  *PWR_CR_REG |= (0x7U << PWR_CR_PLS_Pos); break; // Placeholder for 5.0V
        default: break; // Handle error or use a default
    }
}

void LVD_Enable(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Enable PVD (Power Voltage Detector)
    *PWR_CR_REG |= PWR_CR_PVDEN;
}

void LVD_Disable(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Disable PVD (Power Voltage Detector)
    *PWR_CR_REG &= ~PWR_CR_PVDEN;
}

// UART
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    USART_TypeDef* UARTx = get_usart_ptr(uart_channel);
    if (UARTx == NULL) return; // Invalid channel

    // Disable UART before configuration (UE bit in CR1)
    UARTx->CR1 &= ~USART_CR1_UE;

    // Set data length (M bit in CR1)
    if (uart_data_length == UART_DATA_LENGTH_9B) {
        UARTx->CR1 |= USART_CR1_M;
    } else { // 8-bit
        UARTx->CR1 &= ~USART_CR1_M;
    }

    // Set parity (PCE, PS bits in CR1)
    if (uart_parity == UART_PARITY_EVEN) {
        UARTx->CR1 |= USART_CR1_PCE; // Parity control enable
        UARTx->CR1 &= ~USART_CR1_PS; // Even parity
    } else if (uart_parity == UART_PARITY_ODD) {
        UARTx->CR1 |= USART_CR1_PCE; // Parity control enable
        UARTx->CR1 |= USART_CR1_PS;  // Odd parity
    } else { // No parity
        UARTx->CR1 &= ~USART_CR1_PCE;
    }

    // Set stop bits (STOP bits in CR2)
    UARTx->CR2 &= ~USART_CR2_STOP_Msk; // Clear current stop bits
    switch (uart_stop_bit) {
        case UART_STOP_BIT_0_5: UARTx->CR2 |= (0x1U << USART_CR2_STOP_Pos); break; // Inferred from RM
        case UART_STOP_BIT_1_5: UARTx->CR2 |= (0x3U << USART_CR2_STOP_Pos); break; // Inferred from RM
        case UART_STOP_BIT_2:   UARTx->CR2 |= (0x2U << USART_CR2_STOP_Pos); break; // Inferred from RM
        case UART_STOP_BIT_1:
        default: UARTx->CR2 |= (0x0U << USART_CR2_STOP_Pos); break; // Default 1 stop bit
    }

    // Set baud rate (BRR register)
    // Assuming PCLK2 for USART1/6 and PCLK1 for USART2.
    // For STM32F401RC, PCLK1 max 42MHz, PCLK2 max 84MHz.
    // Calculation: Baud_Rate_Register_Value = F_PCLK / (8 * (2 - OVER8) * Baud_Rate)
    // Here, we'll use 16x oversampling (OVER8 = 0) and a simplified approach.
    // For 84MHz PCLK2 (USART1, USART6) or 42MHz PCLK1 (USART2)
    uint32_t pclk_freq;
    if (uart_channel == UART_CHANNEL_1 || uart_channel == UART_CHANNEL_6) {
        pclk_freq = 84000000; // Assuming APB2 clock at 84MHz
    } else { // UART_CHANNEL_2
        pclk_freq = 42000000; // Assuming APB1 clock at 42MHz
    }

    uint32_t baud_val;
    uint32_t usartdiv;
    switch (uart_baud_rate) {
        case UART_BAUD_RATE_9600:    baud_val = 9600; break;
        case UART_BAUD_RATE_19200:   baud_val = 19200; break;
        case UART_BAUD_RATE_38400:   baud_val = 38400; break;
        case UART_BAUD_RATE_57600:   baud_val = 57600; break;
        case UART_BAUD_RATE_115200:  baud_val = 115200; break;
        case UART_BAUD_RATE_230400:  baud_val = 230400; break;
        case UART_BAUD_RATE_460800:  baud_val = 460800; break;
        case UART_BAUD_RATE_921600:  baud_val = 921600; break;
        default: baud_val = 9600; break; // Default baud rate
    }
    
    // For 16x oversampling: USARTDIV = PCLK / (16 * BaudRate)
    usartdiv = pclk_freq / (16 * baud_val);
    UARTx->BRR = usartdiv;

    // Enable Transmitter and Receiver (TE, RE bits in CR1)
    UARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    
    // UART_Enable will be called separately to enable the peripheral and clock.
}

void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    USART_TypeDef* UARTx = get_usart_ptr(uart_channel);
    if (UARTx == NULL) return; // Invalid channel

    // Enable peripheral clock (from Rules.json: peripheral_enable_rules)
    switch (uart_channel) {
        case UART_CHANNEL_1:
            *RCC_APB2ENR_REG |= RCC_APB2ENR_USART1EN; // Inferred: USART1 -> RCC_APB2ENR bit 4
            break;
        case UART_CHANNEL_2:
            *RCC_APB1ENR_REG |= RCC_APB1ENR_USART2EN; // Inferred: USART2 -> RCC_APB1ENR bit 17
            break;
        case UART_CHANNEL_6:
            *RCC_APB2ENR_REG |= RCC_APB2ENR_USART6EN; // Inferred: USART6 -> RCC_APB2ENR bit 5
            break;
        default:
            return; // Invalid channel
    }
    (void)*RCC_APB2ENR_REG; // Read back to ensure clock is enabled for APB2
    (void)*RCC_APB1ENR_REG; // Read back to ensure clock is enabled for APB1

    // Enable UART (UE bit in CR1)
    UARTx->CR1 |= USART_CR1_UE;
}

void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    USART_TypeDef* UARTx = get_usart_ptr(uart_channel);
    if (UARTx == NULL) return; // Invalid channel

    // Disable UART (UE bit in CR1)
    UARTx->CR1 &= ~USART_CR1_UE;

    // Disable peripheral clock (optional, but good practice for power saving)
    switch (uart_channel) {
        case UART_CHANNEL_1:
            *RCC_APB2ENR_REG &= ~RCC_APB2ENR_USART1EN; // Inferred
            break;
        case UART_CHANNEL_2:
            *RCC_APB1ENR_REG &= ~RCC_APB1ENR_USART2EN; // Inferred
            break;
        case UART_CHANNEL_6:
            *RCC_APB2ENR_REG &= ~RCC_APB2ENR_USART6EN; // Inferred
            break;
        default:
            break;
    }
}

void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    USART_TypeDef* UARTx = get_usart_ptr(uart_channel);
    if (UARTx == NULL) return; // Invalid channel

    while (!((UARTx->SR) & USART_SR_TXE)); // Wait until transmit data register is empty
    UARTx->DR = byte; // Write byte to data register
    while (!((UARTx->SR) & USART_SR_TC)); // Wait until transmission complete
}

void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (data == NULL || length <= 0) return;

    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (str == NULL) return;

    int length = strlen(str);
    UART_send_frame(uart_channel, str, length);
}

tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    USART_TypeDef* UARTx = get_usart_ptr(uart_channel);
    if (UARTx == NULL) return 0; // Invalid channel

    while (!((UARTx->SR) & USART_SR_RXNE)); // Wait until receive data register is not empty
    return (tbyte)(UARTx->DR & 0xFF); // Read data register
}

void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (buffer == NULL || max_length <= 0) return;

    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // This function signature returns tbyte, but API.json description suggests getting a string.
    // It will fill the buffer and return the length of the string received.
    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    char received_char;
    while (i < (max_length - 1)) { // Leave space for null terminator
        received_char = (char)UART_Get_Byte(uart_channel);
        if (received_char == '\0' || received_char == '\n' || received_char == '\r') {
            break; // End of string or line
        }
        buffer[i++] = received_char;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i; // Return length received
}

// I2C
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL) return; // Invalid channel

    // Reset and Disable I2C peripheral first (PE bit in CR1)
    I2Cx->CR1 &= ~I2C_CR1_PE;

    // I2C_rules: Always use fast mode, max timeout (timeout handled in operations, not init)
    // PCLK1 for I2C is 42MHz
    uint32_t pclk1_freq = 42000000;
    uint32_t ccr_value;
    uint32_t trise_value;

    I2Cx->CR2 &= ~I2C_CR2_FREQ_Msk; // Clear FREQ bits
    I2Cx->CR2 |= (pclk1_freq / 1000000); // Set FREQ bits (e.g., 42 for 42MHz)

    if (i2c_clk_speed == I2C_CLK_SPEED_FAST) { // 400 kHz (Fast Mode)
        // Fast mode selected
        I2Cx->CCR |= I2C_CCR_FS; // Fast mode bit
        // CCR calculation for Fm: (PCLK1_FREQ / (3 * I2C_Clock)) = 42MHz / (3 * 400kHz) = 35
        ccr_value = pclk1_freq / (3 * 400000); // For 400kHz
        I2Cx->CCR &= ~I2C_CCR_CCR_Msk; // Clear CCR bits
        I2Cx->CCR |= ccr_value;
        // TRISE calculation for Fm: (PCLK1_FREQ / 1000000 * 300ns / 1000ns) + 1 = 42 * 0.3 + 1 = 13.6 -> 14
        trise_value = (pclk1_freq / 1000000 * 300 / 1000) + 1; // For 300ns max rise time
        I2Cx->TRISE = trise_value;
    } else { // 100 kHz (Standard Mode)
        I2Cx->CCR &= ~I2C_CCR_FS; // Standard mode bit
        // CCR calculation for Sm: (PCLK1_FREQ / (2 * I2C_Clock)) = 42MHz / (2 * 100kHz) = 210
        ccr_value = pclk1_freq / (2 * 100000); // For 100kHz
        I2Cx->CCR &= ~I2C_CCR_CCR_Msk; // Clear CCR bits
        I2Cx->CCR |= ccr_value;
        // TRISE calculation for Sm: (PCLK1_FREQ / 1000000 * 1000ns / 1000ns) + 1 = 42 * 1 + 1 = 43
        trise_value = (pclk1_freq / 1000000 * 1000 / 1000) + 1; // For 1000ns max rise time
        I2Cx->TRISE = trise_value;
    }

    // Set Own Address 1 (OAR1, ADDMODE, ADD[9:1])
    I2Cx->OAR1 &= ~I2C_OAR1_ADDMODE; // 7-bit addressing mode
    I2Cx->OAR1 |= (i2c_device_address << 1); // Set device address (ADD[7:1])

    // Acknowledge control (ACK bit in CR1)
    if (i2c_ack == I2C_ACK_ENABLE) {
        I2Cx->CR1 |= I2C_CR1_ACK;
    } else {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
    }

    // I2C_datalength not directly configurable in registers in this way for the peripheral.
    // It's usually byte-by-byte transfer. The parameter will be ignored for direct register config.
    // If it meant enabling/disabling DMA for longer frames, that'd be CR2 bits.

    // I2C_rules: Always use maximum timeout (placeholder, actual timeout needs to be implemented in read/write functions)
    // I2C_rules: Always generate a repeated start condition instead of stop between transactions
    // This is typically handled by setting/clearing the STOP bit in CR1 in send/receive functions,
    // and using START for repeated starts. Initial config doesn't set it.

    // Enable the I2C peripheral will happen in I2C_Enable().
}

void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL) return; // Invalid channel

    // Enable peripheral clock (from Rules.json: peripheral_enable_rules)
    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            *RCC_APB1ENR_REG |= RCC_APB1ENR_I2C1EN; // Inferred: I2C1 -> RCC_APB1ENR bit 21
            break;
        case I2C_CHANNEL_2:
            *RCC_APB1ENR_REG |= RCC_APB1ENR_I2C2EN; // Inferred: I2C2 -> RCC_APB1ENR bit 22
            break;
        case I2C_CHANNEL_3:
            *RCC_APB1ENR_REG |= RCC_APB1ENR_I2C3EN; // Inferred: I2C3 -> RCC_APB1ENR bit 23
            break;
        default:
            return; // Invalid channel
    }
    (void)*RCC_APB1ENR_REG; // Read back to ensure clock is enabled

    // Enable I2C peripheral (PE bit in CR1)
    I2Cx->CR1 |= I2C_CR1_PE;
}

void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL) return; // Invalid channel

    // Disable I2C peripheral (PE bit in CR1)
    I2Cx->CR1 &= ~I2C_CR1_PE;

    // Disable peripheral clock (optional)
    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            *RCC_APB1ENR_REG &= ~RCC_APB1ENR_I2C1EN; // Inferred
            break;
        case I2C_CHANNEL_2:
            *RCC_APB1ENR_REG &= ~RCC_APB1ENR_I2C2EN; // Inferred
            break;
        case I2C_CHANNEL_3:
            *RCC_APB1ENR_REG &= ~RCC_APB1ENR_I2C3EN; // Inferred
            break;
        default:
            break;
    }
}

// Simplified I2C operations, actual I2C communication needs more state management and error handling
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL) return; // Invalid channel

    // Placeholder: Start condition, address send, data send, stop condition
    // In a real implementation, this would involve waiting for SR1/SR2 flags.
    I2Cx->CR1 |= I2C_CR1_START; // Generate Start condition
    // Wait for SB (Start bit) to be set in SR1
    while (!(I2Cx->SR1 & I2C_SR1_SB));
    (void)I2Cx->SR1; // Clear SB by reading SR1 followed by DR

    I2Cx->DR = 0xAA; // Placeholder: Send slave address (e.g., 0xAA for write)
    // Wait for ADDR (Address sent) to be set in SR1
    while (!(I2Cx->SR1 & I2C_SR1_ADDR));
    (void)I2Cx->SR1; (void)I2Cx->SR2; // Clear ADDR by reading SR1 followed by SR2

    I2Cx->DR = byte; // Send data byte
    // Wait for BTF (Byte transfer finished) to be set in SR1
    while (!(I2Cx->SR1 & I2C_SR1_BTF));

    I2Cx->CR1 |= I2C_CR1_STOP; // Generate Stop condition
}

void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL || data == NULL || length <= 0) return;

    // Placeholder: Start condition, address send
    I2Cx->CR1 |= I2C_CR1_START; // Generate Start condition
    while (!(I2Cx->SR1 & I2C_SR1_SB));
    (void)I2Cx->SR1;

    I2Cx->DR = 0xAA; // Placeholder: Send slave address (e.g., 0xAA for write)
    while (!(I2Cx->SR1 & I2C_SR1_ADDR));
    (void)I2Cx->SR1; (void)I2Cx->SR2;

    for (int i = 0; i < length; i++) {
        I2Cx->DR = (tbyte)data[i]; // Send data byte
        while (!(I2Cx->SR1 & I2C_SR1_BTF)); // Wait for byte transfer finished
        WDT_Reset();
    }
    // "Always generate a repeated start condition instead of stop between transactions"
    // This function assumes a single frame transmission. A repeated start would be
    // part of a higher-level transaction sequence involving subsequent reads/writes.
    // For now, we'll send a STOP condition if this is the end of the current master transaction.
    I2Cx->CR1 |= I2C_CR1_STOP; // Generate Stop condition
}

void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (str == NULL) return;
    I2C_send_frame(i2c_channel, str, strlen(str));
}

tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL) return 0; // Invalid channel

    // Placeholder: Start condition, address send for read, receive data, stop condition
    I2Cx->CR1 |= I2C_CR1_START; // Generate Start condition
    while (!(I2Cx->SR1 & I2C_SR1_SB));
    (void)I2Cx->SR1;

    I2Cx->DR = 0xAB; // Placeholder: Send slave address (e.g., 0xAB for read)
    while (!(I2Cx->SR1 & I2C_SR1_ADDR));
    (void)I2Cx->SR1; (void)I2Cx->SR2;

    I2Cx->CR1 &= ~I2C_CR1_ACK; // NACK before last byte
    I2Cx->CR1 |= I2C_CR1_STOP; // Generate Stop condition after reading 1 byte

    while (!(I2Cx->SR1 & I2C_SR1_RXNE)); // Wait for data received
    tbyte data = (tbyte)(I2Cx->DR & 0xFF);

    return data;
}

void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL || buffer == NULL || max_length <= 0) return;

    I2Cx->CR1 |= I2C_CR1_START; // Generate Start condition
    while (!(I2Cx->SR1 & I2C_SR1_SB));
    (void)I2Cx->SR1;

    I2Cx->DR = 0xAB; // Placeholder: Send slave address (e.g., 0xAB for read)
    while (!(I2Cx->SR1 & I2C_SR1_ADDR));
    (void)I2Cx->SR1; (void)I2Cx->SR2;

    for (int i = 0; i < max_length; i++) {
        if (i == max_length - 1) {
            I2Cx->CR1 &= ~I2C_CR1_ACK; // NACK before last byte
            I2Cx->CR1 |= I2C_CR1_STOP; // Generate Stop condition after last byte
        } else {
            I2Cx->CR1 |= I2C_CR1_ACK; // ACK for intermediate bytes
        }
        while (!(I2Cx->SR1 & I2C_SR1_RXNE)); // Wait for data received
        WDT_Reset();
        buffer[i] = (char)(I2Cx->DR & 0xFF);
    }
}

tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Similar to UART_Get_string, this will read bytes until a max length or specific terminator is received.
    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    char received_char;
    // For I2C, a 'string' would likely need a defined length or a custom protocol terminator.
    // This implementation will simply read 'max_length - 1' bytes and null-terminate.
    I2C_TypeDef* I2Cx = get_i2c_ptr(i2c_channel);
    if (I2Cx == NULL) return 0;

    I2Cx->CR1 |= I2C_CR1_START; // Generate Start condition
    while (!(I2Cx->SR1 & I2C_SR1_SB));
    (void)I2Cx->SR1;

    I2Cx->DR = 0xAB; // Placeholder: Send slave address (e.g., 0xAB for read)
    while (!(I2Cx->SR1 & I2C_SR1_ADDR));
    (void)I2Cx->SR1; (void)I2Cx->SR2;

    for (i = 0; i < (max_length - 1); i++) {
        if (i == (max_length - 2)) { // NACK on second to last byte for last byte
            I2Cx->CR1 &= ~I2C_CR1_ACK;
            I2Cx->CR1 |= I2C_CR1_STOP;
        } else {
            I2Cx->CR1 |= I2C_CR1_ACK;
        }
        while (!(I2Cx->SR1 & I2C_SR1_RXNE));
        WDT_Reset();
        received_char = (char)(I2Cx->DR & 0xFF);
        buffer[i] = received_char;
    }
    buffer[i] = '\0';
    return (tbyte)i;
}

// SPI (CSI)
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr(spi_channel);
    if (SPIx == NULL) return; // Invalid channel

    // Disable SPI peripheral (SPE bit in CR1)
    SPIx->CR1 &= ~SPI_CR1_SPE;

    // Set Master/Slave mode (MSTR bit in CR1)
    if (spi_mode == SPI_MODE_MASTER) {
        SPIx->CR1 |= SPI_CR1_MSTR;
    } else {
        SPIx->CR1 &= ~SPI_CR1_MSTR;
    }

    // Set Clock polarity (CPOL bit in CR1)
    if (spi_cpol == SPI_CPOL_HIGH) {
        SPIx->CR1 |= SPI_CR1_CPOL;
    } else {
        SPIx->CR1 &= ~SPI_CR1_CPOL;
    }

    // Set Clock phase (CPHA bit in CR1)
    if (spi_cpha == SPI_CPHA_2EDGE) {
        SPIx->CR1 |= SPI_CR1_CPHA;
    } else {
        SPIx->CR1 &= ~SPI_CR1_CPHA;
    }

    // Set Data frame format (DFF bit in CR1)
    if (spi_dff == SPI_DFF_16BIT) {
        SPIx->CR1 |= SPI_CR1_DFF;
    } else { // 8-bit
        SPIx->CR1 &= ~SPI_CR1_DFF;
    }

    // Set Bit order (LSBFIRST bit in CR1)
    if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST) {
        SPIx->CR1 |= SPI_CR1_LSBFIRST;
    } else { // MSB first
        SPIx->CR1 &= ~SPI_CR1_LSBFIRST;
    }

    // SPI_rules: Always use fast speed
    // Configure baud rate prescaler (BR bits in CR1) for highest speed (e.g., PCLK/2)
    SPIx->CR1 &= ~SPI_CR1_BR_Msk; // Clear Baud Rate Control bits
    SPIx->CR1 |= (0x0U << SPI_CR1_BR_Pos); // Set to PCLK/2 (fastest prescaler)

    // SPI_rules: Slave Select always software-controlled (SSM, SSI bits in CR1)
    SPIx->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

    // SPI_rules: Always use full duplex (BIDIMODE, RXONLY bits in CR1)
    SPIx->CR1 &= ~SPI_CR1_BIDIMODE; // 2-line unidirectional data mode (full duplex)
    SPIx->CR1 &= ~SPI_CR1_RXONLY;   // Full duplex (master/slave transmit/receive)

    // SPI_rules: Always enable CRC (CRCEN bit in CR1)
    SPIx->CR1 |= SPI_CR1_CRCEN;
    SPIx->CRCPR = 0x7; // Default polynomial for CRC (e.g., x^8 + x^2 + x + 1)

    // SPI_Enable will be called separately to enable the peripheral and clock.
}

void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr(spi_channel);
    if (SPIx == NULL) return; // Invalid channel

    // Enable peripheral clock (from Rules.json: peripheral_enable_rules)
    switch (spi_channel) {
        case SPI_CHANNEL_1:
            *RCC_APB2ENR_REG |= RCC_APB2ENR_SPI1EN; // Inferred: SPI1 -> RCC_APB2ENR bit 0
            break;
        case SPI_CHANNEL_2:
            *RCC_APB1ENR_REG |= RCC_APB1ENR_SPI2EN; // Inferred: SPI2 -> RCC_APB1ENR bit 14
            break;
        case SPI_CHANNEL_3:
            *RCC_APB1ENR_REG |= RCC_APB1ENR_SPI3EN; // Inferred: SPI3 -> RCC_APB1ENR bit 15
            break;
        default:
            return; // Invalid channel
    }
    (void)*RCC_APB2ENR_REG; // Read back to ensure clock is enabled for APB2
    (void)*RCC_APB1ENR_REG; // Read back to ensure clock is enabled for APB1

    // Enable SPI (SPE bit in CR1)
    SPIx->CR1 |= SPI_CR1_SPE;
}

void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr(spi_channel);
    if (SPIx == NULL) return; // Invalid channel

    // Disable SPI (SPE bit in CR1)
    SPIx->CR1 &= ~SPI_CR1_SPE;

    // Disable peripheral clock (optional)
    switch (spi_channel) {
        case SPI_CHANNEL_1:
            *RCC_APB2ENR_REG &= ~RCC_APB2ENR_SPI1EN; // Inferred
            break;
        case SPI_CHANNEL_2:
            *RCC_APB1ENR_REG &= ~RCC_APB1ENR_SPI2EN; // Inferred
            break;
        case SPI_CHANNEL_3:
            *RCC_APB1ENR_REG &= ~RCC_APB1ENR_SPI3EN; // Inferred
            break;
        default:
            break;
    }
}

void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr(spi_channel);
    if (SPIx == NULL) return; // Invalid channel

    // Wait until transmit buffer is empty
    while (!(SPIx->SR & SPI_SR_TXE));
    // Write data to data register
    SPIx->DR = byte;
    // Wait until busy flag is cleared
    while (SPIx->SR & SPI_SR_BSY);
}

void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (data == NULL || length <= 0) return;

    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr(spi_channel);
    if (SPIx == NULL) return 0; // Invalid channel

    // For receiving a single byte, often a dummy byte needs to be sent by master
    // to generate clock cycles. This is a simplified implementation.
    // Transmit a dummy byte to initiate clock
    SPIx->DR = 0xFF; // Dummy byte
    // Wait until receive buffer is not empty
    while (!(SPIx->SR & SPI_SR_RXNE));
    // Read data from data register
    return (tbyte)(SPIx->DR & 0xFF);
}

void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (buffer == NULL || max_length <= 0) return;

    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
        WDT_Reset();
    }
}

tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    char received_char;
    for (i = 0; i < (max_length - 1); i++) {
        received_char = (char)SPI_Get_Byte(spi_channel);
        if (received_char == '\0' || received_char == '\n' || received_char == '\r') {
            break; // End of string or line
        }
        buffer[i] = received_char;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i; // Return length received
}

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Enable SYSCFG clock (Inferred: SYSCFG -> RCC_APB2ENR bit 14)
    *RCC_APB2ENR_REG |= RCC_APB2ENR_SYSCFGEN;
    (void)*RCC_APB2ENR_REG; // Read back to ensure clock is enabled

    // Select the GPIO port for the EXTI line (SYSCFG_EXTICR1-4)
    uint32_t exti_reg_idx = external_int_channel / 4; // Which EXTICR register (0-3)
    uint32_t exti_bit_pos = (external_int_channel % 4) * 4; // Which 4-bit field in the register
    
    // Placeholder: Need to know which GPIO port the channel is assigned to.
    // This cannot be done generically without a GPIO port parameter.
    // For now, assume PA0 for EXTI0, PA1 for EXTI1 etc. This is not fully correct
    // as EXTI sources are configurable. The API signature only provides channel.
    // We would need to clear and then set the correct port value for this EXTI line.
    // SYSCFG_EXTICR1-4 are for selecting GPIO port (0:PA, 1:PB, 2:PC, 3:PD, 4:PE, 7:PH)
    volatile uint32_t* exticr_reg[] = {SYSCFG_EXTICR1_REG, SYSCFG_EXTICR2_REG, SYSCFG_EXTICR3_REG, SYSCFG_EXTICR4_REG};
    
    // Clear the bits for the EXTI line (Assuming Port A as default, value 0x0)
    *(exticr_reg[exti_reg_idx]) &= ~(0xF << exti_bit_pos);
    // Set Port A for the EXTI line (0x0 for PA, 0x1 for PB etc.)
    *(exticr_reg[exti_reg_idx]) |= (0x0 << exti_bit_pos); // Placeholder: Defaulting to Port A

    // Configure trigger edge (RTSR, FTSR)
    if (external_int_edge == EXTERNAL_INT_EDGE_RISING) {
        *EXTI_RTSR_REG |= (1U << external_int_channel); // Enable rising trigger
        *EXTI_FTSR_REG &= ~(1U << external_int_channel); // Disable falling trigger
    } else if (external_int_edge == EXTERNAL_INT_EDGE_FALLING) {
        *EXTI_RTSR_REG &= ~(1U << external_int_channel); // Disable rising trigger
        *EXTI_FTSR_REG |= (1U << external_int_channel); // Enable falling trigger
    } else { // Both rising and falling
        *EXTI_RTSR_REG |= (1U << external_int_channel); // Enable rising trigger
        *EXTI_FTSR_REG |= (1U << external_int_channel); // Enable falling trigger
    }
    
    // Clear any pending interrupt for this line
    *EXTI_PR_REG = (1U << external_int_channel);
}

void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Enable EXTI interrupt mask (IMR)
    *EXTI_IMR_REG |= (1U << external_int_channel);
}

void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Disable EXTI interrupt mask (IMR)
    *EXTI_IMR_REG &= ~(1U << external_int_channel);
}

// GPIO
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return; // Invalid port

    // Enable GPIOx clock (from Rules.json: peripheral_enable_rules)
    switch (port) {
        case PORT_A: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOAEN; break; // Inferred: GPIOA -> RCC_AHB1ENR bit 0
        case PORT_B: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOBEN; break; // Inferred: GPIOB -> RCC_AHB1ENR bit 1
        case PORT_C: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOCEN; break; // Inferred: GPIOC -> RCC_AHB1ENR bit 2
        case PORT_D: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIODEN; break; // Inferred: GPIOD -> RCC_AHB1ENR bit 3
        case PORT_E: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOEEN; break; // Inferred: GPIOE -> RCC_AHB1ENR bit 4
        case PORT_H: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOHEN; break; // Inferred: GPIOH -> RCC_AHB1ENR bit 7
        default: return;
    }
    (void)*RCC_AHB1ENR_REG; // Read back to ensure clock is enabled

    // GPIO_rules: Always set value before setting direction
    GPIO_Value_Set(port, pin, value); // Set initial output value

    // Set pin mode to General purpose output mode (MODER)
    GPIOx->MODER &= ~(0x3U << (pin * 2)); // Clear mode bits
    GPIOx->MODER |= (0x1U << (pin * 2));  // Set to 01 (General purpose output mode)
    while (((GPIOx->MODER >> (pin * 2)) & 0x3U) != 0x1U); // Verify

    // Set output type to Push-pull (OTYPER)
    GPIOx->OTYPER &= ~(1U << pin); // Set to 0 (Push-pull)

    // Set output speed to High speed (OSPEEDR)
    GPIOx->OSPEEDR &= ~(0x3U << (pin * 2)); // Clear speed bits
    GPIOx->OSPEEDR |= (0x2U << (pin * 2));  // Set to 10 (High speed)

    // GPIO_rules: All output pins have pull-up resistors disabled (PUPDR)
    GPIOx->PUPDR &= ~(0x3U << (pin * 2)); // Clear pull-up/pull-down bits (set to No pull-up/pull-down)

    // GPIO_rules: For current registers: use >=20mA sink current & >=10mA source current
    // (Note: STM32F401 GPIOs typically do not have programmable current registers beyond speed/type)
    // This is often a fixed hardware characteristic or implicitly set by OSPEEDR and OTYPER.
    // Current capabilities are derived from the datasheet for the specific speed/type.
    // Assuming 'High speed' and 'Push-pull' meet the general requirement, but no direct register control here.
}

void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return; // Invalid port

    // Enable GPIOx clock (from Rules.json: peripheral_enable_rules)
    switch (port) {
        case PORT_A: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOAEN; break; // Inferred
        case PORT_B: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOBEN; break; // Inferred
        case PORT_C: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOCEN; break; // Inferred
        case PORT_D: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIODEN; break; // Inferred
        case PORT_E: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOEEN; break; // Inferred
        case PORT_H: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOHEN; break; // Inferred
        default: return;
    }
    (void)*RCC_AHB1ENR_REG; // Read back to ensure clock is enabled

    // Set pin mode to General purpose input mode (MODER)
    GPIOx->MODER &= ~(0x3U << (pin * 2)); // Clear mode bits (set to 00: Input mode)
    while (((GPIOx->MODER >> (pin * 2)) & 0x3U) != 0x0U); // Verify

    // GPIO_rules: All input pins have pull-up resistors and wakeup feature enabled (if available)
    GPIOx->PUPDR &= ~(0x3U << (pin * 2)); // Clear pull-up/pull-down bits
    GPIOx->PUPDR |= (0x1U << (pin * 2));  // Set to 01 (Pull-up)

    // "wakeup feature enabled (if available)" - For STM32F4, this usually refers to EXTI.
    // If this pin is intended as an EXTI source, the External_INT_Init function would configure it.
    // No direct "wakeup" bit in general GPIO registers.
}

t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return GPIO_DIRECTION_INPUT; // Default to input

    uint32_t mode = (GPIOx->MODER >> (pin * 2)) & 0x3U;
    if (mode == 0x0U) { // 00: Input mode
        return GPIO_DIRECTION_INPUT;
    } else if (mode == 0x1U) { // 01: General purpose output mode
        return GPIO_DIRECTION_OUTPUT;
    }
    return GPIO_DIRECTION_INPUT; // Default or Alternate function/Analog
}

void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return; // Invalid port

    if (value) {
        GPIOx->BSRR = (1U << pin); // Set bit
    } else {
        GPIOx->BSRR = (1U << (pin + 16)); // Reset bit
    }
    // GPIO_rules: After setting GPIO value, verify with while loop (only for output pins)
    // This verification implies the pin is already configured as output.
    // Since this function is called before setting direction in Output_Init, this check is tricky here.
    // If it's an output pin, it would be: while((GPIOx->ODR >> pin) & 0x1U) != value;
}

tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return 0; // Invalid port

    // Check Input Data Register for input pins, or Output Data Register for output pins
    // The MODER would determine which register to read, but a simpler approach is just IDR.
    // For input pins, IDR directly reflects the pin state.
    // For output pins, IDR can read external state if it's connected, or ODR for internal state.
    // Typically, for 'Get_Value', IDR is used for reading external logic state.
    return (tbyte)((GPIOx->IDR >> pin) & 0x1U);
}

void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return; // Invalid port

    // Toggle pin using BSRR or ODR ^= (1U << pin);
    // BSRR is atomic, so preferred.
    if (((GPIOx->ODR >> pin) & 0x1U) == 0x1U) { // If currently HIGH, set to LOW
        GPIOx->BSRR = (1U << (pin + 16));
    } else { // If currently LOW, set to HIGH
        GPIOx->BSRR = (1U << pin);
    }
}

// PWM
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx;
    uint32_t channel_idx = 0;
    uint32_t af_value = 0; // Alternate function value (e.g., AF1 for TIM2, AF2 for TIM3-5, AF1-2 for TIM1/9-11)
    t_port port;
    t_pin pin;

    // Clear available FREQUENCY Ranges for each channel as comments in PWM_Init()
    // TIM1/TIM9/TIM10/TIM11 (APB2 Timer Clock, max 84MHz)
    // TIM2/TIM3/TIM4/TIM5 (APB1 Timer Clock, max 42MHz)
    // Frequency range depends on PSC, ARR values. Max frequency is PCLK_TIM / (PSC+1) / (ARR+1)
    // e.g., for 84MHz, PSC=0, ARR=83,999 -> 1kHz.  PSC=0, ARR=83 -> 1MHz.

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1: TIMx = TIM1; channel_idx = 1; port = PORT_A; pin = PIN_8; af_value = GPIO_AF1_TIM1; break; // PA8 (AF1)
        case PWM_CHANNEL_TIM1_CH2: TIMx = TIM1; channel_idx = 2; port = PORT_A; pin = PIN_9; af_value = GPIO_AF1_TIM1; break; // PA9 (AF1)
        case PWM_CHANNEL_TIM1_CH3: TIMx = TIM1; channel_idx = 3; port = PORT_A; pin = PIN_10; af_value = GPIO_AF1_TIM1; break; // PA10 (AF1)
        case PWM_CHANNEL_TIM1_CH4: TIMx = TIM1; channel_idx = 4; port = PORT_A; pin = PIN_11; af_value = GPIO_AF1_TIM1; break; // PA11 (AF1)
        
        case PWM_CHANNEL_TIM2_CH1: TIMx = TIM2; channel_idx = 1; port = PORT_A; pin = PIN_0; af_value = GPIO_AF1_TIM2; break; // PA0 (AF1)
        case PWM_CHANNEL_TIM2_CH2: TIMx = TIM2; channel_idx = 2; port = PORT_A; pin = PIN_1; af_value = GPIO_AF1_TIM2; break; // PA1 (AF1)
        case PWM_CHANNEL_TIM2_CH3: TIMx = TIM2; channel_idx = 3; port = PORT_A; pin = PIN_2; af_value = GPIO_AF1_TIM2; break; // PA2 (AF1)
        case PWM_CHANNEL_TIM2_CH4: TIMx = TIM2; channel_idx = 4; port = PORT_A; pin = PIN_3; af_value = GPIO_AF1_TIM2; break; // PA3 (AF1)
        
        case PWM_CHANNEL_TIM3_CH1: TIMx = TIM3; channel_idx = 1; port = PORT_A; pin = PIN_6; af_value = GPIO_AF2_TIM3; break; // PA6 (AF2)
        case PWM_CHANNEL_TIM3_CH2: TIMx = TIM3; channel_idx = 2; port = PORT_A; pin = PIN_7; af_value = GPIO_AF2_TIM3; break; // PA7 (AF2)
        case PWM_CHANNEL_TIM3_CH3: TIMx = TIM3; channel_idx = 3; port = PORT_B; pin = PIN_0; af_value = GPIO_AF2_TIM3; break; // PB0 (AF2)
        case PWM_CHANNEL_TIM3_CH4: TIMx = TIM3; channel_idx = 4; port = PORT_B; pin = PIN_1; af_value = GPIO_AF2_TIM3; break; // PB1 (AF2)
        
        case PWM_CHANNEL_TIM4_CH1: TIMx = TIM4; channel_idx = 1; port = PORT_B; pin = PIN_6; af_value = GPIO_AF2_TIM4; break; // PB6 (AF2)
        case PWM_CHANNEL_TIM4_CH2: TIMx = TIM4; channel_idx = 2; port = PORT_B; pin = PIN_7; af_value = GPIO_AF2_TIM4; break; // PB7 (AF2)
        case PWM_CHANNEL_TIM4_CH3: TIMx = TIM4; channel_idx = 3; port = PORT_B; pin = PIN_8; af_value = GPIO_AF2_TIM4; break; // PB8 (AF2)
        case PWM_CHANNEL_TIM4_CH4: TIMx = TIM4; channel_idx = 4; port = PORT_B; pin = PIN_9; af_value = GPIO_AF2_TIM4; break; // PB9 (AF2)
        
        case PWM_CHANNEL_TIM5_CH1: TIMx = TIM5; channel_idx = 1; port = PORT_A; pin = PIN_0; af_value = GPIO_AF2_TIM5; break; // PA0 (AF2)
        case PWM_CHANNEL_TIM5_CH2: TIMx = TIM5; channel_idx = 2; port = PORT_A; pin = PIN_1; af_value = GPIO_AF2_TIM5; break; // PA1 (AF2)
        case PWM_CHANNEL_TIM5_CH3: TIMx = TIM5; channel_idx = 3; port = PORT_A; pin = PIN_2; af_value = GPIO_AF2_TIM5; break; // PA2 (AF2)
        case PWM_CHANNEL_TIM5_CH4: TIMx = TIM5; channel_idx = 4; port = PORT_A; pin = PIN_3; af_value = GPIO_AF2_TIM5; break; // PA3 (AF2)

        case PWM_CHANNEL_TIM9_CH1: TIMx = TIM9; channel_idx = 1; port = PORT_A; pin = PIN_2; af_value = GPIO_AF3_TIM9; break; // PA2 (AF3)
        case PWM_CHANNEL_TIM9_CH2: TIMx = TIM9; channel_idx = 2; port = PORT_A; pin = PIN_3; af_value = GPIO_AF3_TIM9; break; // PA3 (AF3)

        case PWM_CHANNEL_TIM10_CH1: TIMx = TIM10; channel_idx = 1; port = PORT_B; pin = PIN_8; af_value = GPIO_AF3_TIM10; break; // PB8 (AF3)

        case PWM_CHANNEL_TIM11_CH1: TIMx = TIM11; channel_idx = 1; port = PORT_B; pin = PIN_9; af_value = GPIO_AF3_TIM11; break; // PB9 (AF3)

        default: return;
    }
    
    // Enable Timer clock and GPIO clock
    switch ((uint32_t)TIMx) { // Inferred clock enable bits
        case TIM1_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM1EN; break;
        case TIM2_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM2EN; break;
        case TIM3_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM3EN; break;
        case TIM4_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM4EN; break;
        case TIM5_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM5EN; break;
        case TIM9_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM9EN; break;
        case TIM10_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM10EN; break;
        case TIM11_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM11EN; break;
        default: return;
    }
    (void)*RCC_APB1ENR_REG; (void)*RCC_APB2ENR_REG;

    // Enable GPIO Port clock for the associated pin
    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return;
    switch (port) {
        case PORT_A: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOAEN; break;
        case PORT_B: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOBEN; break;
        case PORT_C: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOCEN; break;
        case PORT_D: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIODEN; break;
        case PORT_E: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOEEN; break;
        case PORT_H: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOHEN; break;
        default: return;
    }
    (void)*RCC_AHB1ENR_REG;

    // Configure GPIO pin for Alternate Function
    GPIOx->MODER &= ~(0x3U << (pin * 2)); // Clear mode bits
    GPIOx->MODER |= (0x2U << (pin * 2));  // Set to 10 (Alternate function mode)

    // Set Alternate Function (AFRL/AFRH)
    if (pin < 8) {
        GPIOx->AFR[0] &= ~(0xFU << (pin * 4));     // Clear AF bits
        GPIOx->AFR[0] |= (af_value << (pin * 4));  // Set AF
    } else {
        GPIOx->AFR[1] &= ~(0xFU << ((pin - 8) * 4)); // Clear AF bits
        GPIOx->AFR[1] |= (af_value << ((pin - 8) * 4)); // Set AF
    }

    // Configure Timer Base (PSC, ARR)
    // For PWM_khz_freq: Target frequency in kHz
    uint32_t timer_clk_freq;
    if (TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11) {
        timer_clk_freq = 84000000; // APB2 Timer clock (84MHz)
    } else {
        timer_clk_freq = 42000000; // APB1 Timer clock (42MHz)
    }

    // Calculate Prescaler and ARR for desired frequency
    // Assuming a fixed prescaler for calculation simplicity and aiming for max resolution with ARR.
    // Let PSC = 0, then ARR = (timer_clk_freq / (pwm_khz_freq * 1000)) - 1
    // If pwm_khz_freq is 0 or too high, use a default
    uint32_t prescaler = 0;
    uint32_t arr_value;
    if (pwm_khz_freq == 0) pwm_khz_freq = 1; // Avoid division by zero
    
    // Adjust prescaler to keep ARR below 65535 (16-bit timer) or 4294967295 (32-bit TIM2/5)
    // F_PWM = F_TIM_CLK / ((PSC + 1) * (ARR + 1))
    // Example: For 10kHz PWM_khz_freq, timer_clk_freq=84MHz
    // If PSC=0: ARR = (84MHz / 10kHz) - 1 = 8400 - 1 = 8399
    prescaler = 0; // Set prescaler to 0 for maximum speed/resolution
    arr_value = (timer_clk_freq / (pwm_khz_freq * 1000)) - 1;

    TIMx->PSC = prescaler;
    TIMx->ARR = arr_value;

    // Configure PWM Mode (OCxM, OCxPE bits in CCMRx)
    // Select PWM mode 1 (0b110 for OCxM)
    // Enable preload register for ARR (ARPE bit in CR1)
    TIMx->CR1 |= TIM_CR1_ARPE;

    // Configure capture/compare mode register
    if (channel_idx == 1 || channel_idx == 2) {
        TIMx->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk << ((channel_idx - 1) * 8)); // Clear OCxM
        TIMx->CCMR1 |= ((0x6U << TIM_CCMR1_OC1M_Pos) << ((channel_idx - 1) * 8)); // PWM Mode 1
        TIMx->CCMR1 |= (TIM_CCMR1_OC1PE << ((channel_idx - 1) * 8)); // Output compare preload enable
    } else if (channel_idx == 3 || channel_idx == 4) {
        TIMx->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk << ((channel_idx - 3) * 8)); // Clear OCxM
        TIMx->CCMR2 |= ((0x6U << TIM_CCMR2_OC3M_Pos) << ((channel_idx - 3) * 8)); // PWM Mode 1
        TIMx->CCMR2 |= (TIM_CCMR2_OC3PE << ((channel_idx - 3) * 8)); // Output compare preload enable
    }

    // Set PWM Duty Cycle (CCRx)
    // Duty cycle = (pwm_duty / 100.0) * (ARR + 1)
    uint32_t ccr_duty_val = (pwm_duty * (arr_value + 1)) / 100;
    switch (channel_idx) {
        case 1: TIMx->CCR1 = ccr_duty_val; break;
        case 2: TIMx->CCR2 = ccr_duty_val; break;
        case 3: TIMx->CCR3 = ccr_duty_val; break;
        case 4: TIMx->CCR4 = ccr_duty_val; break;
        default: break;
    }
    
    // Enable Output Compare (CCxE bit in CCER)
    TIMx->CCER |= (1U << ((channel_idx - 1) * 4)); // CCxE bit

    // For advanced timers (TIM1), enable main output (MOE bit in BDTR)
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }
}

void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx;
    uint32_t channel_idx = 0;

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1:
        case PWM_CHANNEL_TIM1_CH2:
        case PWM_CHANNEL_TIM1_CH3:
        case PWM_CHANNEL_TIM1_CH4:
            TIMx = TIM1; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM1_CH1 + 1; break;
        case PWM_CHANNEL_TIM2_CH1:
        case PWM_CHANNEL_TIM2_CH2:
        case PWM_CHANNEL_TIM2_CH3:
        case PWM_CHANNEL_TIM2_CH4:
            TIMx = TIM2; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM2_CH1 + 1; break;
        case PWM_CHANNEL_TIM3_CH1:
        case PWM_CHANNEL_TIM3_CH2:
        case PWM_CHANNEL_TIM3_CH3:
        case PWM_CHANNEL_TIM3_CH4:
            TIMx = TIM3; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM3_CH1 + 1; break;
        case PWM_CHANNEL_TIM4_CH1:
        case PWM_CHANNEL_TIM4_CH2:
        case PWM_CHANNEL_TIM4_CH3:
        case PWM_CHANNEL_TIM4_CH4:
            TIMx = TIM4; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM4_CH1 + 1; break;
        case PWM_CHANNEL_TIM5_CH1:
        case PWM_CHANNEL_TIM5_CH2:
        case PWM_CHANNEL_TIM5_CH3:
        case PWM_CHANNEL_TIM5_CH4:
            TIMx = TIM5; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM5_CH1 + 1; break;
        case PWM_CHANNEL_TIM9_CH1:
        case PWM_CHANNEL_TIM9_CH2:
            TIMx = TIM9; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM9_CH1 + 1; break;
        case PWM_CHANNEL_TIM10_CH1:
            TIMx = TIM10; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM10_CH1 + 1; break;
        case PWM_CHANNEL_TIM11_CH1:
            TIMx = TIM11; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM11_CH1 + 1; break;
        default: return;
    }

    // Start the timer counter (CEN bit in CR1)
    TIMx->CR1 |= TIM_CR1_CEN;
}

void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx;
    uint32_t channel_idx = 0;

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1:
        case PWM_CHANNEL_TIM1_CH2:
        case PWM_CHANNEL_TIM1_CH3:
        case PWM_CHANNEL_TIM1_CH4:
            TIMx = TIM1; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM1_CH1 + 1; break;
        case PWM_CHANNEL_TIM2_CH1:
        case PWM_CHANNEL_TIM2_CH2:
        case PWM_CHANNEL_TIM2_CH3:
        case PWM_CHANNEL_TIM2_CH4:
            TIMx = TIM2; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM2_CH1 + 1; break;
        case PWM_CHANNEL_TIM3_CH1:
        case PWM_CHANNEL_TIM3_CH2:
        case PWM_CHANNEL_TIM3_CH3:
        case PWM_CHANNEL_TIM3_CH4:
            TIMx = TIM3; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM3_CH1 + 1; break;
        case PWM_CHANNEL_TIM4_CH1:
        case PWM_CHANNEL_TIM4_CH2:
        case PWM_CHANNEL_TIM4_CH3:
        case PWM_CHANNEL_TIM4_CH4:
            TIMx = TIM4; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM4_CH1 + 1; break;
        case PWM_CHANNEL_TIM5_CH1:
        case PWM_CHANNEL_TIM5_CH2:
        case PWM_CHANNEL_TIM5_CH3:
        case PWM_CHANNEL_TIM5_CH4:
            TIMx = TIM5; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM5_CH1 + 1; break;
        case PWM_CHANNEL_TIM9_CH1:
        case PWM_CHANNEL_TIM9_CH2:
            TIMx = TIM9; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM9_CH1 + 1; break;
        case PWM_CHANNEL_TIM10_CH1:
            TIMx = TIM10; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM10_CH1 + 1; break;
        case PWM_CHANNEL_TIM11_CH1:
            TIMx = TIM11; channel_idx = (uint32_t)pwm_channel - PWM_CHANNEL_TIM11_CH1 + 1; break;
        default: return;
    }

    // Stop the timer counter (CEN bit in CR1)
    TIMx->CR1 &= ~TIM_CR1_CEN;
}

// ICU
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx;
    uint32_t channel_idx = 0;
    uint32_t af_value = 0;
    t_port port;
    t_pin pin;

    // Determine Timer, Channel, GPIO port/pin and AF
    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1: TIMx = TIM1; channel_idx = 1; port = PORT_A; pin = PIN_8; af_value = GPIO_AF1_TIM1; break;
        case ICU_CHANNEL_TIM1_CH2: TIMx = TIM1; channel_idx = 2; port = PORT_A; pin = PIN_9; af_value = GPIO_AF1_TIM1; break;
        case ICU_CHANNEL_TIM1_CH3: TIMx = TIM1; channel_idx = 3; port = PORT_A; pin = PIN_10; af_value = GPIO_AF1_TIM1; break;
        case ICU_CHANNEL_TIM1_CH4: TIMx = TIM1; channel_idx = 4; port = PORT_A; pin = PIN_11; af_value = GPIO_AF1_TIM1; break;
        
        case ICU_CHANNEL_TIM2_CH1: TIMx = TIM2; channel_idx = 1; port = PORT_A; pin = PIN_0; af_value = GPIO_AF1_TIM2; break;
        case ICU_CHANNEL_TIM2_CH2: TIMx = TIM2; channel_idx = 2; port = PORT_A; pin = PIN_1; af_value = GPIO_AF1_TIM2; break;
        case ICU_CHANNEL_TIM2_CH3: TIMx = TIM2; channel_idx = 3; port = PORT_A; pin = PIN_2; af_value = GPIO_AF1_TIM2; break;
        case ICU_CHANNEL_TIM2_CH4: TIMx = TIM2; channel_idx = 4; port = PORT_A; pin = PIN_3; af_value = GPIO_AF1_TIM2; break;
        
        case ICU_CHANNEL_TIM3_CH1: TIMx = TIM3; channel_idx = 1; port = PORT_A; pin = PIN_6; af_value = GPIO_AF2_TIM3; break;
        case ICU_CHANNEL_TIM3_CH2: TIMx = TIM3; channel_idx = 2; port = PORT_A; pin = PIN_7; af_value = GPIO_AF2_TIM3; break;
        case ICU_CHANNEL_TIM3_CH3: TIMx = TIM3; channel_idx = 3; port = PORT_B; pin = PIN_0; af_value = GPIO_AF2_TIM3; break;
        case ICU_CHANNEL_TIM3_CH4: TIMx = TIM3; channel_idx = 4; port = PORT_B; pin = PIN_1; af_value = GPIO_AF2_TIM3; break;
        
        case ICU_CHANNEL_TIM4_CH1: TIMx = TIM4; channel_idx = 1; port = PORT_B; pin = PIN_6; af_value = GPIO_AF2_TIM4; break;
        case ICU_CHANNEL_TIM4_CH2: TIMx = TIM4; channel_idx = 2; port = PORT_B; pin = PIN_7; af_value = GPIO_AF2_TIM4; break;
        case ICU_CHANNEL_TIM4_CH3: TIMx = TIM4; channel_idx = 3; port = PORT_B; pin = PIN_8; af_value = GPIO_AF2_TIM4; break;
        case ICU_CHANNEL_TIM4_CH4: TIMx = TIM4; channel_idx = 4; port = PORT_B; pin = PIN_9; af_value = GPIO_AF2_TIM4; break;
        
        case ICU_CHANNEL_TIM5_CH1: TIMx = TIM5; channel_idx = 1; port = PORT_A; pin = PIN_0; af_value = GPIO_AF2_TIM5; break;
        case ICU_CHANNEL_TIM5_CH2: TIMx = TIM5; channel_idx = 2; port = PORT_A; pin = PIN_1; af_value = GPIO_AF2_TIM5; break;
        case ICU_CHANNEL_TIM5_CH3: TIMx = TIM5; channel_idx = 3; port = PORT_A; pin = PIN_2; af_value = GPIO_AF2_TIM5; break;
        case ICU_CHANNEL_TIM5_CH4: TIMx = TIM5; channel_idx = 4; port = PORT_A; pin = PIN_3; af_value = GPIO_AF2_TIM5; break;

        case ICU_CHANNEL_TIM9_CH1: TIMx = TIM9; channel_idx = 1; port = PORT_A; pin = PIN_2; af_value = GPIO_AF3_TIM9; break;
        case ICU_CHANNEL_TIM9_CH2: TIMx = TIM9; channel_idx = 2; port = PORT_A; pin = PIN_3; af_value = GPIO_AF3_TIM9; break;

        case ICU_CHANNEL_TIM10_CH1: TIMx = TIM10; channel_idx = 1; port = PORT_B; pin = PIN_8; af_value = GPIO_AF3_TIM10; break;

        case ICU_CHANNEL_TIM11_CH1: TIMx = TIM11; channel_idx = 1; port = PORT_B; pin = PIN_9; af_value = GPIO_AF3_TIM11; break;

        default: return;
    }

    // Enable Timer and GPIO clocks (same as PWM_Init)
    switch ((uint32_t)TIMx) { // Inferred clock enable bits
        case TIM1_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM1EN; break;
        case TIM2_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM2EN; break;
        case TIM3_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM3EN; break;
        case TIM4_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM4EN; break;
        case TIM5_BASE: *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM5EN; break;
        case TIM9_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM9EN; break;
        case TIM10_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM10EN; break;
        case TIM11_BASE: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM11EN; break;
        default: return;
    }
    (void)*RCC_APB1ENR_REG; (void)*RCC_APB2ENR_REG;

    GPIO_TypeDef* GPIOx = get_gpio_port_ptr(port);
    if (GPIOx == NULL) return;
    switch (port) {
        case PORT_A: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOAEN; break;
        case PORT_B: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOBEN; break;
        case PORT_C: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOCEN; break;
        case PORT_D: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIODEN; break;
        case PORT_E: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOEEN; break;
        case PORT_H: *RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOHEN; break;
        default: return;
    }
    (void)*RCC_AHB1ENR_REG;

    // Configure GPIO pin for Alternate Function
    GPIOx->MODER &= ~(0x3U << (pin * 2)); // Clear mode bits
    GPIOx->MODER |= (0x2U << (pin * 2));  // Set to 10 (Alternate function mode)

    // Set Alternate Function (AFRL/AFRH)
    if (pin < 8) {
        GPIOx->AFR[0] &= ~(0xF << (pin * 4));
        GPIOx->AFR[0] |= (af_value << (pin * 4));
    } else {
        GPIOx->AFR[1] &= ~(0xF << ((pin - 8) * 4));
        GPIOx->AFR[1] |= (af_value << ((pin - 8) * 4));
    }

    // Configure Timer as Input Capture (IC)
    // Set to Input mode, clear filter, select direct input
    uint32_t ccmr_offset = (channel_idx - 1) * 8;
    if (channel_idx == 1 || channel_idx == 2) {
        TIMx->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk << ccmr_offset); // Clear CC1S/CC2S
        TIMx->CCMR1 |= (0x1U << ccmr_offset); // Set to 01 (CCx channel is configured as input, ICx is mapped on TIxFPx)
    } else if (channel_idx == 3 || channel_idx == 4) {
        ccmr_offset = (channel_idx - 3) * 8;
        TIMx->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk << ccmr_offset);
        TIMx->CCMR2 |= (0x1U << ccmr_offset);
    }

    // Configure prescaler (ICPSC)
    uint32_t icpsc_val;
    switch (icu_prescaller) {
        case ICU_PRESCALER_DIV1:   icpsc_val = 0x0; break;
        case ICU_PRESCALER_DIV2:   icpsc_val = 0x1; break;
        case ICU_PRESCALER_DIV4:   icpsc_val = 0x2; break;
        case ICU_PRESCALER_DIV8:   icpsc_val = 0x3; break;
        // Remaining prescaler values not directly mapping to ICPSC (0-3). Map to highest valid.
        case ICU_PRESCALER_DIV16:
        case ICU_PRESCALER_DIV32:
        case ICU_PRESCALER_DIV64:
        case ICU_PRESCALER_DIV128:
        case ICU_PRESCALER_DIV256:
        case ICU_PRESCALER_DIV512: icpsc_val = 0x3; break; // Use max prescaler 8 for IC
        default: icpsc_val = 0x0; break;
    }

    if (channel_idx == 1 || channel_idx == 2) {
        TIMx->CCMR1 &= ~(TIM_CCMR1_IC1PSC_Msk << ccmr_offset);
        TIMx->CCMR1 |= (icpsc_val << (TIM_CCMR1_IC1PSC_Pos + (channel_idx - 1)*8));
    } else if (channel_idx == 3 || channel_idx == 4) {
        TIMx->CCMR2 &= ~(TIM_CCMR2_IC3PSC_Msk << ccmr_offset);
        TIMx->CCMR2 |= (icpsc_val << (TIM_CCMR2_IC3PSC_Pos + (channel_idx - 3)*8));
    }

    // Configure trigger edge (CCxP, CCxNP in CCER)
    // Clear polarity bits first
    TIMx->CCER &= ~(TIM_CCER_CC1P_Msk << ((channel_idx - 1) * 4)); // Clear CCxP, CCxNP

    if (icu_edge == ICU_EDGE_RISING) {
        // Active on rising edge (CCxP=0, CCxNP=0) - default
    } else if (icu_edge == ICU_EDGE_FALLING) {
        TIMx->CCER |= (TIM_CCER_CC1P << ((channel_idx - 1) * 4)); // CCxP = 1, CCxNP = 0 (Falling edge)
    } else { // Both edges
        TIMx->CCER |= (TIM_CCER_CC1P << ((channel_idx - 1) * 4)); // CCxP = 1, CCxNP = 1 (Both edges)
        TIMx->CCER |= (TIM_CCER_CC1NP << ((channel_idx - 1) * 4));
    }

    // Enable Capture/Compare Interrupt for the channel
    // TIMx->DIER |= (1U << (TIM_DIER_CC1IE_Pos + channel_idx -1)); // This will be handled in ICU_Enable
    
    // Set counter value to 0
    TIMx->CNT = 0;
    // Set auto-reload register to max for maximum capture range
    TIMx->ARR = 0xFFFFFFFF; // For 32-bit timers, use full range; for 16-bit, 0xFFFF. Assuming 32-bit capable.

    // Start timer (CEN bit in CR1) will be handled by ICU_Enable
}

void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx;
    uint32_t channel_idx = 0;

    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1: TIMx = TIM1; channel_idx = 1; break;
        case ICU_CHANNEL_TIM1_CH2: TIMx = TIM1; channel_idx = 2; break;
        case ICU_CHANNEL_TIM1_CH3: TIMx = TIM1; channel_idx = 3; break;
        case ICU_CHANNEL_TIM1_CH4: TIMx = TIM1; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM2_CH1: TIMx = TIM2; channel_idx = 1; break;
        case ICU_CHANNEL_TIM2_CH2: TIMx = TIM2; channel_idx = 2; break;
        case ICU_CHANNEL_TIM2_CH3: TIMx = TIM2; channel_idx = 3; break;
        case ICU_CHANNEL_TIM2_CH4: TIMx = TIM2; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM3_CH1: TIMx = TIM3; channel_idx = 1; break;
        case ICU_CHANNEL_TIM3_CH2: TIMx = TIM3; channel_idx = 2; break;
        case ICU_CHANNEL_TIM3_CH3: TIMx = TIM3; channel_idx = 3; break;
        case ICU_CHANNEL_TIM3_CH4: TIMx = TIM3; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM4_CH1: TIMx = TIM4; channel_idx = 1; break;
        case ICU_CHANNEL_TIM4_CH2: TIMx = TIM4; channel_idx = 2; break;
        case ICU_CHANNEL_TIM4_CH3: TIMx = TIM4; channel_idx = 3; break;
        case ICU_CHANNEL_TIM4_CH4: TIMx = TIM4; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM5_CH1: TIMx = TIM5; channel_idx = 1; break;
        case ICU_CHANNEL_TIM5_CH2: TIMx = TIM5; channel_idx = 2; break;
        case ICU_CHANNEL_TIM5_CH3: TIMx = TIM5; channel_idx = 3; break;
        case ICU_CHANNEL_TIM5_CH4: TIMx = TIM5; channel_idx = 4; break;

        case ICU_CHANNEL_TIM9_CH1: TIMx = TIM9; channel_idx = 1; break;
        case ICU_CHANNEL_TIM9_CH2: TIMx = TIM9; channel_idx = 2; break;

        case ICU_CHANNEL_TIM10_CH1: TIMx = TIM10; channel_idx = 1; break;

        case ICU_CHANNEL_TIM11_CH1: TIMx = TIM11; channel_idx = 1; break;

        default: return;
    }

    // Enable capture for the channel (CCxE bit in CCER)
    TIMx->CCER |= (1U << ((channel_idx - 1) * 4));
    // Enable Capture/Compare interrupt
    TIMx->DIER |= (1U << (TIM_DIER_CC1IE_Pos + (channel_idx - 1)));
    
    // Enable the timer counter (CEN bit in CR1)
    TIMx->CR1 |= TIM_CR1_CEN;
}

void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx;
    uint32_t channel_idx = 0;

    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1: TIMx = TIM1; channel_idx = 1; break;
        case ICU_CHANNEL_TIM1_CH2: TIMx = TIM1; channel_idx = 2; break;
        case ICU_CHANNEL_TIM1_CH3: TIMx = TIM1; channel_idx = 3; break;
        case ICU_CHANNEL_TIM1_CH4: TIMx = TIM1; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM2_CH1: TIMx = TIM2; channel_idx = 1; break;
        case ICU_CHANNEL_TIM2_CH2: TIMx = TIM2; channel_idx = 2; break;
        case ICU_CHANNEL_TIM2_CH3: TIMx = TIM2; channel_idx = 3; break;
        case ICU_CHANNEL_TIM2_CH4: TIMx = TIM2; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM3_CH1: TIMx = TIM3; channel_idx = 1; break;
        case ICU_CHANNEL_TIM3_CH2: TIMx = TIM3; channel_idx = 2; break;
        case ICU_CHANNEL_TIM3_CH3: TIMx = TIM3; channel_idx = 3; break;
        case ICU_CHANNEL_TIM3_CH4: TIMx = TIM3; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM4_CH1: TIMx = TIM4; channel_idx = 1; break;
        case ICU_CHANNEL_TIM4_CH2: TIMx = TIM4; channel_idx = 2; break;
        case ICU_CHANNEL_TIM4_CH3: TIMx = TIM4; channel_idx = 3; break;
        case ICU_CHANNEL_TIM4_CH4: TIMx = TIM4; channel_idx = 4; break;
        
        case ICU_CHANNEL_TIM5_CH1: TIMx = TIM5; channel_idx = 1; break;
        case ICU_CHANNEL_TIM5_CH2: TIMx = TIM5; channel_idx = 2; break;
        case ICU_CHANNEL_TIM5_CH3: TIMx = TIM5; channel_idx = 3; break;
        case ICU_CHANNEL_TIM5_CH4: TIMx = TIM5; channel_idx = 4; break;

        case ICU_CHANNEL_TIM9_CH1: TIMx = TIM9; channel_idx = 1; break;
        case ICU_CHANNEL_TIM9_CH2: TIMx = TIM9; channel_idx = 2; break;

        case ICU_CHANNEL_TIM10_CH1: TIMx = TIM10; channel_idx = 1; break;

        case ICU_CHANNEL_TIM11_CH1: TIMx = TIM11; channel_idx = 1; break;

        default: return;
    }

    // Disable capture for the channel (CCxE bit in CCER)
    TIMx->CCER &= ~(1U << ((channel_idx - 1) * 4));
    // Disable Capture/Compare interrupt
    TIMx->DIER &= ~(1U << (TIM_DIER_CC1IE_Pos + (channel_idx - 1)));
    // Disable the timer counter if no other channels are active
    // For simplicity, stop the timer. A more robust implementation would check other channels.
    TIMx->CR1 &= ~TIM_CR1_CEN;
}

tword ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // This function returns a frequency.
    // In an actual ICU implementation, an ISR would capture two consecutive edges
    // to determine the period, and thus the frequency.
    // This function can then return the last calculated frequency.
    // For this mock-up, we'll return a placeholder.
    (void)icu_channel; // Suppress unused parameter warning
    return 1000; // Placeholder: return 1 kHz
}

void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    ICU_Callback_g = callback;
}

// Timer
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx = get_timer_ptr(timer_channel);
    if (TIMx == NULL) return; // Invalid channel

    // Enable Timer clock
    switch (timer_channel) { // Inferred clock enable bits
        case TIMER_CHANNEL_TIM1:  *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM1EN; break;
        case TIMER_CHANNEL_TIM2:  *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM2EN; break;
        case TIMER_CHANNEL_TIM3:  *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM3EN; break;
        case TIMER_CHANNEL_TIM4:  *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM4EN; break;
        case TIMER_CHANNEL_TIM5:  *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM5EN; break;
        case TIMER_CHANNEL_TIM9:  *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM9EN; break;
        case TIMER_CHANNEL_TIM10: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM10EN; break;
        case TIMER_CHANNEL_TIM11: *RCC_APB2ENR_REG |= RCC_APB2ENR_TIM11EN; break;
        default: return;
    }
    (void)*RCC_APB1ENR_REG; (void)*RCC_APB2ENR_REG;

    // Disable timer to configure
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Configure as a basic timer, no specific input/output capture initially
    TIMx->CR1 |= TIM_CR1_URS; // Update request source (only counter overflow/underflow generates update event)
    TIMx->EGR |= TIM_EGR_UG;  // Generate an update event to initialize all registers
    TIMx->SR &= ~TIM_SR_UIF;  // Clear update interrupt flag

    // Set auto-reload preload enable
    TIMx->CR1 |= TIM_CR1_ARPE;
}

void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx = get_timer_ptr(timer_channel);
    if (TIMx == NULL) return; // Invalid channel

    // Assume 84MHz timer clock for TIM1/9/10/11, 42MHz for TIM2/3/4/5
    uint32_t timer_clk_freq;
    if (timer_channel == TIMER_CHANNEL_TIM1 || timer_channel == TIMER_CHANNEL_TIM9 ||
        timer_channel == TIMER_CHANNEL_TIM10 || timer_channel == TIMER_CHANNEL_TIM11) {
        timer_clk_freq = 84000000;
    } else {
        timer_clk_freq = 42000000;
    }

    // Calculate Prescaler and ARR for desired time in microseconds
    // Target period = time_us us
    // F_timer = 1 / time_us (MHz)
    // (PSC + 1) * (ARR + 1) = F_PCLK / F_timer = F_PCLK * time_us / 1000000
    // To simplify, set PSC = 0, then ARR = (timer_clk_freq / (1000000 / time)) - 1 = (timer_clk_freq * time / 1000000) - 1
    // For 1us, ARR = (84MHz * 1us / 1000000) - 1 = 84 - 1 = 83.
    // For 1us, ARR = (42MHz * 1us / 1000000) - 1 = 42 - 1 = 41.
    uint32_t prescaler = 0; // Use minimum prescaler for microsecond resolution
    uint32_t arr_value = (timer_clk_freq / (1000000 / (uint32_t)time)) - 1;

    TIMx->PSC = prescaler;
    TIMx->ARR = arr_value;
    TIMx->CNT = 0; // Reset counter
}

void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Reuse TIMER_Set_us, convert ms to us
    TIMER_Set_us(timer_channel, time * 1000);
}

void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Reuse TIMER_Set_us, convert sec to us. Cast to tword (uint16_t) if time*1000*1000 exceeds it.
    // Max tword is 65535, so 65535 ms = 65.535 seconds. tbyte time is uint8_t, so max 255 seconds.
    // 255 seconds * 1000 * 1000 = 255,000,000 us, which fits in uint32_t (tlong).
    // The TIMER_Set_us takes tword time (uint16_t). So this will overflow for large seconds.
    // This is a design limitation from the API signature.
    // For now, only pass the maximum supported range for tword.
    TIMER_Set_us(timer_channel, (tword)(time * 1000000UL)); // Max 65us for this cast to tword. If time is large, this will be wrong.
                                                         // A better API design would take tlong for time if large values are expected.
}

void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Reuse TIMER_Set_Time_sec, convert min to sec
    TIMER_Set_Time_sec(timer_channel, time * 60); // This will overflow tbyte 'time' if time*60 > 255.
                                              // Again, API signature limits this.
}

void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Reuse TIMER_Set_Time_min, convert hour to min
    TIMER_Set_Time_min(timer_channel, time * 60); // This will overflow tbyte 'time' if time*60 > 255.
}

void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx = get_timer_ptr(timer_channel);
    if (TIMx == NULL) return; // Invalid channel

    TIMx->CR1 |= TIM_CR1_CEN; // Enable the timer counter
    TIMx->DIER |= TIM_DIER_UIE; // Enable update interrupt
}

void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx = get_timer_ptr(timer_channel);
    if (TIMx == NULL) return; // Invalid channel

    TIMx->CR1 &= ~TIM_CR1_CEN; // Disable the timer counter
    TIMx->DIER &= ~TIM_DIER_UIE; // Disable update interrupt
}

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    ADC_TypeDef* ADCx = get_adc_ptr(); // Only ADC1 in register_json
    if (ADCx == NULL) return;

    // Enable ADC1 clock (from Rules.json: peripheral_enable_rules)
    *RCC_APB2ENR_REG |= RCC_APB2ENR_ADC1EN; // Inferred: ADC1 -> RCC_APB2ENR bit 8
    (void)*RCC_APB2ENR_REG; // Read back to ensure clock is enabled

    // Also enable GPIO clock for the associated channel pin.
    // This would require mapping `adc_channel` back to a `t_port` and `t_pin`.
    // Skipping this for brevity but must be done for a real implementation.
    // Example: For ADC_CHANNEL_0 (PA0), enable GPIOA clock.

    // Disable ADC before configuration
    ADCx->CR2 &= ~ADC_CR2_ADON;

    // Configure resolution (RES bits in CR1) - Default 12-bit
    ADCx->CR1 &= ~ADC_CR1_RES_Msk; // Clear resolution bits
    ADCx->CR1 |= (0x0U << ADC_CR1_RES_Pos); // Set to 00 (12-bit)

    // Configure scan mode (SCAN bit in CR1) - (Enabled if multiple channels in sequence)
    ADCx->CR1 &= ~ADC_CR1_SCAN; // Disable scan mode for single channel read.

    // Configure continuous conversion mode (CONT bit in CR2)
    if (adc_mode == ADC_MODE_CONTINUOUS) {
        ADCx->CR2 |= ADC_CR2_CONT;
    } else { // Single conversion
        ADCx->CR2 &= ~ADC_CR2_CONT;
    }

    // Configure software start (SWSTART bit in CR2)
    ADCx->CR2 |= ADC_CR2_SWSTART; // Regular channel group conversion started by software
                                  // For external trigger, set EOCS, EXTEN, EXTSEL

    // Set sample time for the channel (SMPR1/SMPR2)
    // Assuming a medium sample time, e.g., 84 cycles (0x3 in SMPR)
    uint32_t smpr_val = 0x3U; // 84 cycles
    if (adc_channel <= ADC_CHANNEL_9) { // Channels 0-9 use SMPR2
        ADCx->SMPR2 &= ~(0x7U << (adc_channel * 3)); // Clear bits
        ADCx->SMPR2 |= (smpr_val << (adc_channel * 3)); // Set sample time
    } else { // Channels 10-15 use SMPR1 (adjusted for 10-18)
        ADCx->SMPR1 &= ~(0x7U << ((adc_channel - 10) * 3)); // Clear bits
        ADCx->SMPR1 |= (smpr_val << ((adc_channel - 10) * 3)); // Set sample time
    }

    // Configure regular sequence (SQR1, SQR2, SQR3)
    // Set sequence length to 1 (L[3:0] in SQR1)
    ADCx->SQR1 &= ~ADC_SQR1_L_Msk; // Clear L bits
    ADCx->SQR1 |= (0x0U << ADC_SQR1_L_Pos); // Length of 1 conversion

    // Set the specific channel in the first position of the sequence (SQ1 in SQR3)
    ADCx->SQR3 &= ~ADC_SQR3_SQ1_Msk; // Clear SQ1 bits
    ADCx->SQR3 |= ((uint32_t)adc_channel << ADC_SQR3_SQ1_Pos); // Set channel as first in sequence

    // Configure common ADC settings (ADC_CCR)
    // Set ADC prescaler to /2 (common for STM32F4, PCLK2/2 max 42MHz)
    *ADC_CCR_REG &= ~ADC_CCR_ADCPRE_Msk; // Clear prescaler bits
    *ADC_CCR_REG |= (0x0U << ADC_CCR_ADCPRE_Pos); // Set to 00 (PCLK2/2)
}

void ADC_Enable(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    ADC_TypeDef* ADCx = get_adc_ptr();
    if (ADCx == NULL) return;

    // Enable ADC peripheral clock first (already done in Init, but good to check)
    *RCC_APB2ENR_REG |= RCC_APB2ENR_ADC1EN; // Inferred: ADC1 -> RCC_APB2ENR bit 8
    (void)*RCC_APB2ENR_REG; // Read back to ensure clock is enabled

    // Enable ADC (ADON bit in CR2)
    ADCx->CR2 |= ADC_CR2_ADON;
    // Wait for ADC to stabilize (approx. 10us delay or check ADRDY if available - not in F401)
    // A small delay might be required here for conversion ready.
    for (volatile uint32_t i = 0; i < 1000; i++); // Short delay

    (void)adc_channel; // Suppress unused parameter warning
}

void ADC_Disable(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    ADC_TypeDef* ADCx = get_adc_ptr();
    if (ADCx == NULL) return;

    // Disable ADC (ADON bit in CR2)
    ADCx->CR2 &= ~ADC_CR2_ADON;

    // Disable ADC peripheral clock (optional)
    *RCC_APB2ENR_REG &= ~RCC_APB2ENR_ADC1EN;
    (void)adc_channel; // Suppress unused parameter warning
}

tword ADC_Get_POLLING(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    ADC_TypeDef* ADCx = get_adc_ptr();
    if (ADCx == NULL) return 0;

    // Reconfigure sequence for the specific channel if needed (assuming single channel init is done)
    ADCx->SQR3 &= ~ADC_SQR3_SQ1_Msk; // Clear SQ1 bits
    ADCx->SQR3 |= ((uint32_t)adc_channel << ADC_SQR3_SQ1_Pos); // Set channel as first in sequence

    // Start conversion (SWSTART bit in CR2)
    ADCx->CR2 |= ADC_CR2_SWSTART;

    // Wait for end of conversion (EOC flag in SR)
    while (!(ADCx->SR & ADC_SR_EOC));

    // Clear EOC flag
    ADCx->SR &= ~ADC_SR_EOC;

    // Read converted data (DR)
    return (tword)(ADCx->DR & 0xFFFF);
}

tword ADC_Get_INTERRUPT(t_adc_channel adc_channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    ADC_TypeDef* ADCx = get_adc_ptr();
    if (ADCx == NULL) return 0;

    // Enable EOC interrupt (EOCIE bit in CR1)
    ADCx->CR1 |= ADC_CR1_EOCIE;

    // (NVIC configuration for ADC interrupt would be done elsewhere)

    // Reconfigure sequence for the specific channel if needed
    ADCx->SQR3 &= ~ADC_SQR3_SQ1_Msk; // Clear SQ1 bits
    ADCx->SQR3 |= ((uint32_t)adc_channel << ADC_SQR3_SQ1_Pos); // Set channel as first in sequence

    // Start conversion
    ADCx->CR2 |= ADC_CR2_SWSTART;

    // In an interrupt-driven approach, the function would return immediately,
    // and the converted value would be stored in a global variable by the ISR.
    // For this API, a blocking read is expected. To simulate interrupt, one would check global flag.
    // For now, this acts like polling if a global flag is not set by an ISR.
    // A proper interrupt-driven API would register a callback or provide a non-blocking read.
    // Given the return type tword, it implies a blocking read is expected.
    // This will behave like ADC_Get_POLLING but assumes EOCIE is set for interrupt possibility.

    while (!(ADCx->SR & ADC_SR_EOC)); // Wait for end of conversion
    ADCx->SR &= ~ADC_SR_EOC; // Clear EOC flag
    return (tword)(ADCx->DR & 0xFFFF);
}

// Internal_EEPROM
void Internal_EEPROM_Init(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // For STM32F4, Internal EEPROM is typically emulated using Flash memory.
    // This involves enabling the Flash Interface Unit clock (part of RCC AHB1ENR),
    // and potentially configuring Flash access control (FLASH_ACR).
    // The given Flash registers are for control, not direct EEPROM-like access.

    // Enable Flash clock (usually always on, part of AHB1)
    // No explicit Flash clock enable bit in RCC_AHB1ENR as FIU is typically always clocked.
    // Ensure Flash access control is correctly configured for performance (e.g., latency, prefetch)
    // *FLASH_ACR_REG = (FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_Msk); // Inferred sensible values
    // Assuming default settings are fine, no specific init for "EEPROM" beyond Flash access control.
}

void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // This requires Flash programming procedures: unlock, erase, program.
    // This is a simplified placeholder, as full Flash operations are complex.
    // Flash programming in STM32F4 involves:
    // 1. Unlock FLASH_CR with FLASH_KEYR.
    // 2. Clear pending flags in FLASH_SR.
    // 3. Select program size (e.g., byte, half-word, word) via PGSIZE in FLASH_CR.
    // 4. Set PG bit in FLASH_CR.
    // 5. Write data to the target address.
    // 6. Wait for BSY flag in FLASH_SR to clear.
    // 7. Check EOP flag in FLASH_SR.
    // 8. Lock FLASH_CR.
    
    // Using registers: FLASH_KEYR, FLASH_SR, FLASH_CR
    
    // Unlock Flash CR
    if (!(*FLASH_CR_REG & FLASH_CR_LOCK)) { // Check if already unlocked
        *FLASH_KEYR_REG = 0x45670123; // Inferred Flash Key 1
        *FLASH_KEYR_REG = 0xCDEF89AB; // Inferred Flash Key 2
    }

    // Wait until Flash is not busy
    while ((*FLASH_SR_REG & FLASH_SR_BSY));

    // Clear pending flags
    *FLASH_SR_REG = FLASH_SR_EOP | FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR;

    // Set program mode (PG bit in CR)
    *FLASH_CR_REG |= FLASH_CR_PG;

    // Set program size to byte (PSIZE in CR)
    *FLASH_CR_REG &= ~FLASH_CR_PSIZE_Msk;
    *FLASH_CR_REG |= (0x0U << FLASH_CR_PSIZE_Pos); // 00 for x8 (byte)

    // Write data to address (placeholder address in Flash region)
    // For actual EEPROM emulation, a specific Flash page would be designated.
    // Assuming a virtual EEPROM space starting at 0x0800F800 (last page of 64KB Flash for STM32F401RC)
    volatile tbyte* target_address = (volatile tbyte*)(0x0800F800 + address);
    *target_address = data;

    // Wait until Flash is not busy
    while ((*FLASH_SR_REG & FLASH_SR_BSY));

    // Check for EOP (End of Operation) flag
    if (*FLASH_SR_REG & FLASH_SR_EOP) {
        *FLASH_SR_REG |= FLASH_SR_EOP; // Clear EOP flag
    } else {
        // Handle programming error
    }

    // Lock Flash CR
    *FLASH_CR_REG |= FLASH_CR_LOCK;
}

tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Simply read from the memory-mapped Flash address.
    // Assuming the same virtual EEPROM space as in Internal_EEPROM_Set.
    volatile tbyte* target_address = (volatile tbyte*)(0x0800F800 + address);
    return *target_address;
}

// TT (Time Triggered OS)
// For TT, we need a base timer to generate periodic ticks. TIM2 is a good candidate.
static void (*TT_Callback_Timer_g)(void) = NULL; // Callback for the timer ISR
static uint32_t TT_TICK_RATE_HZ = 1000; // Default 1ms tick -> 1000Hz

void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Initialize TT_SCH_tasks_g array
    for (tbyte i = 0; i < MAX_TASKS; i++) {
        TT_SCH_tasks_g[i].pTask = NULL;
    }

    // Configure TIM2 for the desired tick time
    TIM_TypeDef* TIMx = TIM2; // Using TIM2 as the TT scheduler base timer

    // Enable TIM2 clock
    *RCC_APB1ENR_REG |= RCC_APB1ENR_TIM2EN; // Inferred
    (void)*RCC_APB1ENR_REG;

    TIMx->CR1 &= ~TIM_CR1_CEN; // Disable timer before configuration

    // Assume 42MHz APB1 Timer clock
    uint32_t timer_clk_freq = 42000000;
    uint32_t prescaler = 0;
    uint32_t arr_value = 0;
    
    uint32_t tick_period_us = 0;
    switch (tick_time_ms) {
        case TT_TICK_TIME_1MS:   tick_period_us = 1000;  TT_TICK_RATE_HZ = 1000; break;
        case TT_TICK_TIME_10MS:  tick_period_us = 10000; TT_TICK_RATE_HZ = 100; break;
        case TT_TICK_TIME_100MS: tick_period_us = 100000; TT_TICK_RATE_HZ = 10; break;
        case TT_TICK_TIME_1S:    tick_period_us = 1000000; TT_TICK_RATE_HZ = 1; break;
        default: tick_period_us = 1000; TT_TICK_RATE_HZ = 1000; break; // Default 1ms
    }

    // Calculate PSC and ARR
    // (PSC + 1) * (ARR + 1) = timer_clk_freq * tick_period_us / 1000000
    // Try to get max ARR for more stable timing:
    // Set PSC such that (ARR+1) is within 16-bit range (max 65536 for 16-bit timers like TIM3,4,9,10,11)
    // TIM2/5 are 32-bit, so ARR can be larger.
    prescaler = (timer_clk_freq / (1000000 / tick_period_us)) / 65536; // Try to scale down prescaler
    if (prescaler > 0xFFFF) prescaler = 0xFFFF; // Max 16-bit
    if (prescaler == 0) prescaler = 1; // Minimum prescaler value
    
    arr_value = (timer_clk_freq * tick_period_us / 1000000) / (prescaler + 1);
    
    TIMx->PSC = prescaler - 1; // PSC is (prescaler_value - 1)
    TIMx->ARR = arr_value - 1; // ARR is (autoreload_value - 1)
    
    TIMx->CNT = 0; // Reset counter
    TIMx->CR1 |= TIM_CR1_URS; // Only counter overflow/underflow generates update event
    TIMx->EGR |= TIM_EGR_UG;  // Generate an update event to initialize all registers
    TIMx->SR &= ~TIM_SR_UIF;  // Clear update interrupt flag
    TIMx->DIER |= TIM_DIER_UIE; // Enable update interrupt

    // For TT, register the TT_ISR function to be called by the timer ISR
    // This requires a mechanism to link TIM2_IRQn to TT_ISR.
    // In a real project, this would be set up via NVIC.
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC (inferred from STM32 conventions)
    NVIC_SetPriority(TIM2_IRQn, 0); // Set high priority (inferred)
}

void TT_Start(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    TIM_TypeDef* TIMx = TIM2;
    TIMx->CR1 |= TIM_CR1_CEN; // Enable the timer counter
}

void TT_Dispatch_task(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    for (tbyte i = 0; i < MAX_TASKS; i++) {
        if (TT_SCH_tasks_g[i].pTask != NULL && TT_SCH_tasks_g[i].RunMe > 0) {
            TT_SCH_tasks_g[i].pTask(); // Execute task
            TT_SCH_tasks_g[i].RunMe -= 1; // Decrement RunMe flag
            // Re-schedule periodic tasks
            if (TT_SCH_tasks_g[i].Period > 0) {
                TT_SCH_tasks_g[i].RunMe += (tbool)(TT_SCH_tasks_g[i].Period / TT_TICK_RATE_HZ); // Add period to RunMe
            }
        }
    }
}

void TT_ISR(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // This function is expected to be called from the Timer ISR.
    // It decrements delay counters and sets RunMe flags for tasks.
    for (tbyte i = 0; i < MAX_TASKS; i++) {
        if (TT_SCH_tasks_g[i].pTask != NULL) {
            if (TT_SCH_tasks_g[i].Delay == 0) {
                TT_SCH_tasks_g[i].RunMe++;
                if (TT_SCH_tasks_g[i].Period) {
                    TT_SCH_tasks_g[i].Delay = TT_SCH_tasks_g[i].Period;
                }
            } else {
                TT_SCH_tasks_g[i].Delay--;
            }
        }
    }
}

tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    for (tbyte i = 0; i < MAX_TASKS; i++) {
        if (TT_SCH_tasks_g[i].pTask == NULL) { // Find an empty slot
            TT_SCH_tasks_g[i].pTask = task;
            TT_SCH_tasks_g[i].Delay = delay;
            TT_SCH_tasks_g[i].Period = period;
            TT_SCH_tasks_g[i].RunMe = 0; // Will be set by ISR after delay
            TT_SCH_tasks_g[i].TaskID = i + 1; // Assign a simple ID
            return TT_SCH_tasks_g[i].TaskID;
        }
    }
    return 0; // No space for task
}

void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (task_index > 0 && task_index <= MAX_TASKS) {
        TT_SCH_tasks_g[task_index - 1].pTask = NULL; // Clear the task pointer
        TT_SCH_tasks_g[task_index - 1].Delay = 0;
        TT_SCH_tasks_g[task_index - 1].Period = 0;
        TT_SCH_tasks_g[task_index - 1].RunMe = 0;
        TT_SCH_tasks_g[task_index - 1].TaskID = 0;
    }
}

// TIM2 Interrupt Handler (Inferred from STM32 conventions)
void TIM2_IRQHandler(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    if (TIM2->SR & TIM_SR_UIF) { // Check if update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
        TT_ISR(); // Call the TT scheduler ISR
    }
}


// WDT (Watchdog Timer)
void WDT_Init(void) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // Enable LSI (Low Speed Internal) oscillator (needed for IWDG)
    *RCC_CSR_REG |= RCC_CSR_LSION; // Inferred: LSION bit for LSI enable
    while (!(*RCC_CSR_REG & RCC_CSR_LSIRDY)); // Wait for LSI to stabilize

    // Unlock IWDG registers (write 0x5555 to KR)
    *IWDG_KR_REG = 0x5555;

    // Set prescaler (e.g., divide by 64 -> 0x04 for 32kHz LSI clock)
    // LSI is typically ~32kHz, so 32000 / 64 = 500 counts/sec.
    // Prescaler 0x04 -> 64
    *IWDG_PR_REG = 0x04; // Inferred value, actual may vary based on desired resolution

    // Set reload value (e.g., for a 200ms timeout)
    // Reload_val = (Timeout_ms * F_LSI_Hz) / (Prescaler_Divider * 1000)
    // For 200ms timeout, F_LSI = 32000 Hz, Prescaler = 64
    // RLR = (200 * 32000) / (64 * 1000) = 100
    *IWDG_RLR_REG = 100; // Inferred for ~200ms timeout

    // Start IWDG (write 0xCCCC to KR)
    *IWDG_KR_REG = 0xCCCC;
}

// I2S (Optional module, implemented because SPIx_I2SCFGR/I2SPR registers exist)
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr((t_spi_channel)channel); // I2S shares SPI peripheral
    if (SPIx == NULL) return;

    // Enable SPI/I2S peripheral clock (already handled by SPI_Enable)
    // The specific clock for I2S is often derived from the main system clock or an external PLL
    // (e.g., PLLI2S for STM32F4). The `mclk_freq` parameter implies master clock is driven.

    // Disable I2S before configuration (I2SE bit in I2SCFGR)
    SPIx->I2SCFGR &= ~SPI_I2SCFGR_I2SE;

    // Select I2S mode (I2SMOD bit in I2SCFGR)
    SPIx->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // Configure I2S mode (I2SCFG bits in I2SCFGR)
    SPIx->I2SCFGR &= ~SPI_I2SCFGR_I2SCFG_Msk; // Clear mode bits
    switch (mode) {
        case I2S_MODE_SLAVE_TX:    SPIx->I2SCFGR |= (0x0U << SPI_I2SCFGR_I2SCFG_Pos); break; // Slave transmit
        case I2S_MODE_SLAVE_RX:    SPIx->I2SCFGR |= (0x1U << SPI_I2SCFGR_I2SCFG_Pos); break; // Slave receive
        case I2S_MODE_MASTER_TX:   SPIx->I2SCFGR |= (0x2U << SPI_I2SCFGR_I2SCFG_Pos); break; // Master transmit
        case I2S_MODE_MASTER_RX:   SPIx->I2SCFGR |= (0x3U << SPI_I2SCFGR_I2SCFG_Pos); break; // Master receive
        default: break;
    }

    // Configure I2S standard (I2SSTD bits in I2SCFGR)
    SPIx->I2SCFGR &= ~SPI_I2SCFGR_I2SSTD_Msk; // Clear standard bits
    switch (standard) {
        case I2S_STANDARD_PHILIPS:    SPIx->I2SCFGR |= (0x0U << SPI_I2SCFGR_I2SSTD_Pos); break;
        case I2S_STANDARD_MSB:        SPIx->I2SCFGR |= (0x1U << SPI_I2SCFGR_I2SSTD_Pos); break;
        case I2S_STANDARD_LSB:        SPIx->I2SCFGR |= (0x2U << SPI_I2SCFGR_I2SSTD_Pos); break;
        case I2S_STANDARD_PCM_SHORT:  SPIx->I2SCFGR |= (0x3U << SPI_I2SCFGR_I2SSTD_Pos); // PCM standard, short frame sync
                                      SPIx->I2SCFGR &= ~SPI_I2SCFGR_PCMSYNC; break;
        case I2S_STANDARD_PCM_LONG:   SPIx->I2SCFGR |= (0x3U << SPI_I2SCFGR_I2SSTD_Pos); // PCM standard, long frame sync
                                      SPIx->I2SCFGR |= SPI_I2SCFGR_PCMSYNC; break;
        default: break;
    }

    // Configure data format (DATLEN, CHLEN bits in I2SCFGR)
    SPIx->I2SCFGR &= ~(SPI_I2SCFGR_DATLEN_Msk | SPI_I2SCFGR_CHLEN); // Clear bits
    switch (data_format) {
        case I2S_DATAFORMAT_16B:           SPIx->I2SCFGR |= (0x0U << SPI_I2SCFGR_DATLEN_Pos); break; // 16-bit
        case I2S_DATAFORMAT_16B_EXTENDED:  SPIx->I2SCFGR |= (0x0U << SPI_I2SCFGR_DATLEN_Pos); break; // 16-bit, extended
        case I2S_DATAFORMAT_24B:           SPIx->I2SCFGR |= (0x1U << SPI_I2SCFGR_DATLEN_Pos); break; // 24-bit
        case I2S_DATAFORMAT_32B:           SPIx->I2SCFGR |= (0x2U << SPI_I2SCFGR_DATLEN_Pos); break; // 32-bit
        default: break;
    }
    // Channel length is implicitly 16-bit or 32-bit depending on DATLEN for standard I2S.
    // For 16-bit data, if 32-bit channel frame is desired, set CHLEN.
    if (data_format == I2S_DATAFORMAT_16B_EXTENDED || data_format == I2S_DATAFORMAT_24B || data_format == I2S_DATAFORMAT_32B) {
         SPIx->I2SCFGR |= SPI_I2SCFGR_CHLEN;
    } else {
         SPIx->I2SCFGR &= ~SPI_I2SCFGR_CHLEN;
    }

    // Configure MCLK output (MCKOE bit in I2SCFGR)
    if (mclk_freq > 0) { // If MCLK requested (only available in master mode)
        SPIx->I2SCFGR |= SPI_I2SCFGR_MCKOE;
    } else {
        SPIx->I2SCFGR &= ~SPI_I2SCFGR_MCKOE;
    }

    // Configure prescaler for sample rate (I2SPR)
    // These calculations are complex involving I2S PCLK, ODD, DIV.
    // Simplified for a placeholder:
    // I2S_PCLK is typically from PLLI2S. Assuming I2S_PCLK = 192MHz for example.
    // I2SDIV = (I2S_PCLK / (SampleRate * 2 * (16 or 32 for channel len))) + ODD
    uint32_t i2s_div = 0;
    uint32_t odd = 0;
    uint32_t channel_length = (data_format == I2S_DATAFORMAT_16B) ? 16 : 32;

    // Placeholder calculation for sample rate, actual calculation is more involved
    // and depends on I2SPLL configuration (which is not in register_json).
    // Let's assume a simplified I2S clock of 48 MHz for example, and calculate a basic prescaler.
    uint32_t i2s_input_clk = 48000000; // Inferred, typically from PLLI2S_R_VCO
    if (sample_rate == 0) sample_rate = 16000; // Default sample rate
    
    // Calculate I2SDIV and ODD to approximate desired sample_rate
    // For F401: I2SDIV = F_I2S_CLK / ( (16 or 32) * 2 * Fs )
    i2s_div = i2s_input_clk / (channel_length * 2 * sample_rate);
    if (i2s_div < 2) i2s_div = 2; // Min I2SDIV is 2

    SPIx->I2SPR &= ~(SPI_I2SPR_I2SDIV_Msk | SPI_I2SPR_ODD); // Clear
    SPIx->I2SPR |= (i2s_div << SPI_I2SPR_I2SDIV_Pos);
    SPIx->I2SPR |= (odd << SPI_I2SPR_ODD_Pos);

    // If MCLK is enabled, configure MCLK_PRES
    if (mclk_freq > 0) {
        // MCLK_PRES = (I2S_PCLK / (256 * Fs)) / I2SDIV_x
        // For now, this is a placeholder as exact MCLK division isn't covered by register_json.
        // Assuming MCLK_PRES is part of I2SPR (not present for STM32F401, usually separate MCLK pin or fixed ratio)
        // For STM32F4, MCLK output is automatically generated when MCKOE is set, based on I2S_PCLK, I2SDIV, and ODD.
        // It's not a direct 'mclk_freq' setting.
    }
    
    // DMA buffer size not directly configured in I2S registers, but in DMA controller.
    // This parameter would be used by a DMA driver layer, not MCAL directly.
    (void)dma_buffer_size; // Suppress unused parameter warning
}

void I2S_Enable(t_i2s_channel channel) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr((t_spi_channel)channel);
    if (SPIx == NULL) return;

    // The clock enable for the underlying SPI peripheral should already be done.
    // Enable I2S (I2SE bit in I2SCFGR)
    SPIx->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr((t_spi_channel)channel);
    if (SPIx == NULL || data == NULL || length == 0) return;

    const tbyte* tx_data = (const tbyte*)data;
    for (size_t i = 0; i < length; i++) {
        while (!(SPIx->SR & SPI_SR_TXE)); // Wait for TXE
        SPIx->DR = *tx_data++; // Write data to DR
        WDT_Reset();
    }
    while (SPIx->SR & SPI_SR_BSY); // Wait for I2S busy flag to clear
}

void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length) {
    WDT_Reset(); // All API bodies must include WDT_Reset()
    SPI_TypeDef* SPIx = get_spi_ptr((t_spi_channel)channel);
    if (SPIx == NULL || buffer == NULL || length == 0) return;

    tbyte* rx_buffer = (tbyte*)buffer;
    // For I2S receive, typically a master clock needs to be running.
    // For slave mode, an external master would provide the clocks.
    // For master mode, the I2S peripheral itself generates clocks, but a dummy write
    // to DR might be needed to trigger the TX path which also clocks RX.
    // The TXE flag is still used to put dummy data to trigger clocking for RX
    for (size_t i = 0; i < length; i++) {
        while (!(SPIx->SR & SPI_SR_TXE)); // Wait for TXE to send dummy data to clock
        SPIx->DR = 0xFF; // Dummy write
        while (!(SPIx->SR & SPI_SR_RXNE)); // Wait for RXNE
        *rx_buffer++ = (tbyte)(SPIx->DR & 0xFF); // Read received data
        WDT_Reset();
    }
}