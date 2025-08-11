#ifndef MCAL_H
#define MCAL_H

// --- Core MCU header ---
#include "stm32f401xc.h"  // Core device header for STM32F401RC (inferred from MCU name)

// --- Standard C Library Includes ---
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// --- Data Type Definitions (from Rules.json) ---
#define Unit_8  tbyte
#define unit_16 tword
#define unit_32 tlong

typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// --- Register Address Definitions (from REGISTER_JSON) ---
// Flash Registers
#define FLASH_ACR               (*((volatile uint32_t *)0x40023C00))
#define FLASH_KEYR              (*((volatile uint32_t *)0x40023C04))
#define FLASH_OPTKEYR           (*((volatile uint32_t *)0x40023C08))
#define FLASH_SR                (*((volatile uint32_t *)0x40023C0C))
#define FLASH_CR                (*((volatile uint32_t *)0x40023C10))
#define FLASH_OPTCR             (*((volatile uint32_t *)0x40023C14))

// CRC Registers
#define CRC_DR                  (*((volatile uint32_t *)0x40023000))
#define CRC_IDR                 (*((volatile uint32_t *)0x40023004))
#define CRC_CR                  (*((volatile uint32_t *)0x40023008))

// PWR Registers
#define PWR_CR                  (*((volatile uint32_t *)0x40007000))
#define PWR_CSR                 (*((volatile uint32_t *)0x40007004))

// RCC Registers
#define RCC_CR                  (*((volatile uint32_t *)0x40023800))
#define RCC_PLLCFGR             (*((volatile uint32_t *)0x40023804))
#define RCC_CFGR                (*((volatile uint32_t *)0x40023808))
#define RCC_CIR                 (*((volatile uint32_t *)0x4002380C))
#define RCC_AHB1RSTR            (*((volatile uint32_t *)0x40023810))
#define RCC_AHB2RSTR            (*((volatile uint32_t *)0x40023814))
#define RCC_APB1RSTR            (*((volatile uint32_t *)0x40023818))
#define RCC_APB2RSTR            (*((volatile uint32_t *)0x4002381C))
#define RCC_AHB1ENR             (*((volatile uint32_t *)0x40023830))
#define RCC_AHB2ENR             (*((volatile uint32_t *)0x40023834))
#define RCC_APB1ENR             (*((volatile uint32_t *)0x40023838))
#define RCC_APB2ENR             (*((volatile uint32_t *)0x4002383C))
#define RCC_AHB1LPENR           (*((volatile uint32_t *)0x40023850))
#define RCC_AHB2LPENR           (*((volatile uint32_t *)0x40023854))
#define RCC_APB1LPENR           (*((volatile uint32_t *)0x40023858))
#define RCC_APB2LPENR           (*((volatile uint32_t *)0x4002385C))
#define RCC_BDCR                (*((volatile uint32_t *)0x40023870))
#define RCC_CSR                 (*((volatile uint32_t *)0x40023874))
#define RCC_SSCGR               (*((volatile uint32_t *)0x40023880))
#define RCC_PLLI2SCFGR          (*((volatile uint32_t *)0x40023884))
#define RCC_DCKCFGR             (*((volatile uint32_t *)0x4002388C))

// SYSCFG Registers
#define SYSCFG_MEMRMP           (*((volatile uint32_t *)0x40013800))
#define SYSCFG_PMC              (*((volatile uint32_t *)0x40013804))
#define SYSCFG_EXTICR1          (*((volatile uint32_t *)0x40013808))
#define SYSCFG_EXTICR2          (*((volatile uint32_t *)0x4001380C))
#define SYSCFG_EXTICR3          (*((volatile uint32_t *)0x40013810))
#define SYSCFG_EXTICR4          (*((volatile uint32_t *)0x40013814))
#define SYSCFG_CMPCR            (*((volatile uint32_t *)0x40013820))

// GPIO Registers (Base Addresses for Ports)
#define GPIOA_BASE              (0x40020000UL)
#define GPIOB_BASE              (0x40020400UL)
#define GPIOC_BASE              (0x40020800UL)
#define GPIOD_BASE              (0x40020C00UL)
#define GPIOE_BASE              (0x40021000UL)
#define GPIOH_BASE              (0x40021C00UL)

// GPIO Register Offsets (relative to base address)
#define GPIO_MODER_OFFSET       (0x00UL)
#define GPIO_OTYPER_OFFSET      (0x04UL)
#define GPIO_OSPEEDR_OFFSET     (0x08UL)
#define GPIO_PUPDR_OFFSET       (0x0CUL)
#define GPIO_IDR_OFFSET         (0x10UL)
#define GPIO_ODR_OFFSET         (0x14UL)
#define GPIO_BSRR_OFFSET        (0x18UL)
#define GPIO_LCKR_OFFSET        (0x1CUL)
#define GPIO_AFRL_OFFSET        (0x20UL)
#define GPIO_AFRH_OFFSET        (0x24UL)

// GPIO Register Macros (using base and offset)
#define GPIOA_MODER             (*((volatile uint32_t *)(GPIOA_BASE + GPIO_MODER_OFFSET)))
#define GPIOA_OTYPER            (*((volatile uint32_t *)(GPIOA_BASE + GPIO_OTYPER_OFFSET)))
#define GPIOA_OSPEEDR           (*((volatile uint32_t *)(GPIOA_BASE + GPIO_OSPEEDR_OFFSET)))
#define GPIOA_PUPDR             (*((volatile uint32_t *)(GPIOA_BASE + GPIO_PUPDR_OFFSET)))
#define GPIOA_IDR               (*((volatile uint32_t *)(GPIOA_BASE + GPIO_IDR_OFFSET)))
#define GPIOA_ODR               (*((volatile uint32_t *)(GPIOA_BASE + GPIO_ODR_OFFSET)))
#define GPIOA_BSRR              (*((volatile uint32_t *)(GPIOA_BASE + GPIO_BSRR_OFFSET)))
#define GPIOA_LCKR              (*((volatile uint32_t *)(GPIOA_BASE + GPIO_LCKR_OFFSET)))
#define GPIOA_AFRL              (*((volatile uint32_t *)(GPIOA_BASE + GPIO_AFRL_OFFSET)))
#define GPIOA_AFRH              (*((volatile uint32_t *)(GPIOA_BASE + GPIO_AFRH_OFFSET)))

#define GPIOB_MODER             (*((volatile uint32_t *)(GPIOB_BASE + GPIO_MODER_OFFSET)))
#define GPIOB_OTYPER            (*((volatile uint32_t *)(GPIOB_BASE + GPIO_OTYPER_OFFSET)))
#define GPIOB_OSPEEDR           (*((volatile uint32_t *)(GPIOB_BASE + GPIO_OSPEEDR_OFFSET)))
#define GPIOB_PUPDR             (*((volatile uint32_t *)(GPIOB_BASE + GPIO_PUPDR_OFFSET)))
#define GPIOB_IDR               (*((volatile uint32_t *)(GPIOB_BASE + GPIO_IDR_OFFSET)))
#define GPIOB_ODR               (*((volatile uint32_t *)(GPIOB_BASE + GPIO_ODR_OFFSET)))
#define GPIOB_BSRR              (*((volatile uint32_t *)(GPIOB_BASE + GPIO_BSRR_OFFSET)))
#define GPIOB_LCKR              (*((volatile uint32_t *)(GPIOB_BASE + GPIO_LCKR_OFFSET)))
#define GPIOB_AFRL              (*((volatile uint32_t *)(GPIOB_BASE + GPIO_AFRL_OFFSET)))
#define GPIOB_AFRH              (*((volatile uint32_t *)(GPIOB_BASE + GPIO_AFRH_OFFSET)))

#define GPIOC_MODER             (*((volatile uint32_t *)(GPIOC_BASE + GPIO_MODER_OFFSET)))
#define GPIOC_OTYPER            (*((volatile uint32_t *)(GPIOC_BASE + GPIO_OTYPER_OFFSET)))
#define GPIOC_OSPEEDR           (*((volatile uint32_t *)(GPIOC_BASE + GPIO_OSPEEDR_OFFSET)))
#define GPIOC_PUPDR             (*((volatile uint32_t *)(GPIOC_BASE + GPIO_PUPDR_OFFSET)))
#define GPIOC_IDR               (*((volatile uint32_t *)(GPIOC_BASE + GPIO_IDR_OFFSET)))
#define GPIOC_ODR               (*((volatile uint32_t *)(GPIOC_BASE + GPIO_ODR_OFFSET)))
#define GPIOC_BSRR              (*((volatile uint32_t *)(GPIOC_BASE + GPIO_BSRR_OFFSET)))
#define GPIOC_LCKR              (*((volatile uint32_t *)(GPIOC_BASE + GPIO_LCKR_OFFSET)))
#define GPIOC_AFRL              (*((volatile uint32_t *)(GPIOC_BASE + GPIO_AFRL_OFFSET)))
#define GPIOC_AFRH              (*((volatile uint32_t *)(GPIOC_BASE + GPIO_AFRH_OFFSET)))

#define GPIOD_MODER             (*((volatile uint32_t *)(GPIOD_BASE + GPIO_MODER_OFFSET)))
#define GPIOD_OTYPER            (*((volatile uint32_t *)(GPIOD_BASE + GPIO_OTYPER_OFFSET)))
#define GPIOD_OSPEEDR           (*((volatile uint32_t *)(GPIOD_BASE + GPIO_OSPEEDR_OFFSET)))
#define GPIOD_PUPDR             (*((volatile uint32_t *)(GPIOD_BASE + GPIO_PUPDR_OFFSET)))
#define GPIOD_IDR               (*((volatile uint32_t *)(GPIOD_BASE + GPIO_IDR_OFFSET)))
#define GPIOD_ODR               (*((volatile uint32_t *)(GPIOD_BASE + GPIO_ODR_OFFSET)))
#define GPIOD_BSRR              (*((volatile uint32_t *)(GPIOD_BASE + GPIO_BSRR_OFFSET)))
#define GPIOD_LCKR              (*((volatile uint32_t *)(GPIOD_BASE + GPIO_LCKR_OFFSET)))
#define GPIOD_AFRL              (*((volatile uint32_t *)(GPIOD_BASE + GPIO_AFRL_OFFSET)))
#define GPIOD_AFRH              (*((volatile uint32_t *)(GPIOD_BASE + GPIO_AFRH_OFFSET)))

#define GPIOE_MODER             (*((volatile uint32_t *)(GPIOE_BASE + GPIO_MODER_OFFSET)))
#define GPIOE_OTYPER            (*((volatile uint32_t *)(GPIOE_BASE + GPIO_OTYPER_OFFSET)))
#define GPIOE_OSPEEDR           (*((volatile uint32_t *)(GPIOE_BASE + GPIO_OSPEEDR_OFFSET)))
#define GPIOE_PUPDR             (*((volatile uint32_t *)(GPIOE_BASE + GPIO_PUPDR_OFFSET)))
#define GPIOE_IDR               (*((volatile uint32_t *)(GPIOE_BASE + GPIO_IDR_OFFSET)))
#define GPIOE_ODR               (*((volatile uint32_t *)(GPIOE_BASE + GPIO_ODR_OFFSET)))
#define GPIOE_BSRR              (*((volatile uint32_t *)(GPIOE_BASE + GPIO_BSRR_OFFSET)))
#define GPIOE_LCKR              (*((volatile uint32_t *)(GPIOE_BASE + GPIO_LCKR_OFFSET)))
#define GPIOE_AFRL              (*((volatile uint32_t *)(GPIOE_BASE + GPIO_AFRL_OFFSET)))
#define GPIOE_AFRH              (*((volatile uint32_t *)(GPIOE_BASE + GPIO_AFRH_OFFSET)))

#define GPIOH_MODER             (*((volatile uint32_t *)(GPIOH_BASE + GPIO_MODER_OFFSET)))
#define GPIOH_OTYPER            (*((volatile uint32_t *)(GPIOH_BASE + GPIO_OTYPER_OFFSET)))
#define GPIOH_OSPEEDR           (*((volatile uint32_t *)(GPIOH_BASE + GPIO_OSPEEDR_OFFSET)))
#define GPIOH_PUPDR             (*((volatile uint32_t *)(GPIOH_BASE + GPIO_PUPDR_OFFSET)))
#define GPIOH_IDR               (*((volatile uint32_t *)(GPIOH_BASE + GPIO_IDR_OFFSET)))
#define GPIOH_ODR               (*((volatile uint32_t *)(GPIOH_BASE + GPIO_ODR_OFFSET)))
#define GPIOH_BSRR              (*((volatile uint32_t *)(GPIOH_BASE + GPIO_BSRR_OFFSET)))
#define GPIOH_LCKR              (*((volatile uint32_t *)(GPIOH_BASE + GPIO_LCKR_OFFSET)))
#define GPIOH_AFRL              (*((volatile uint32_t *)(GPIOH_BASE + GPIO_AFRL_OFFSET)))
#define GPIOH_AFRH              (*((volatile uint32_t *)(GPIOH_BASE + GPIO_AFRH_OFFSET)))


// DMA Registers (DMA1 and DMA2)
#define DMA1_LISR               (*((volatile uint32_t *)0x40026000))
#define DMA1_HISR               (*((volatile uint32_t *)0x40026004))
#define DMA1_LIFCR              (*((volatile uint32_t *)0x40026008))
#define DMA1_HIFCR              (*((volatile uint32_t *)0x4002600C))
// DMA1 Stream 0-7 Registers
#define DMA1_S0CR               (*((volatile uint32_t *)0x40026010))
#define DMA1_S0NDTR             (*((volatile uint32_t *)0x40026014))
#define DMA1_S0PAR              (*((volatile uint32_t *)0x40026018))
#define DMA1_S0M0AR             (*((volatile uint32_t *)0x4002601C))
#define DMA1_S0M1AR             (*((volatile uint32_t *)0x40026020))
#define DMA1_S0FCR              (*((volatile uint32_t *)0x40026024))
#define DMA1_S1CR               (*((volatile uint32_t *)0x40026028))
#define DMA1_S1NDTR             (*((volatile uint32_t *)0x4002602C))
#define DMA1_S1PAR              (*((volatile uint32_t *)0x40026030))
#define DMA1_S1M0AR             (*((volatile uint32_t *)0x40026034))
#define DMA1_S1M1AR             (*((volatile uint32_t *)0x40026038))
#define DMA1_S1FCR              (*((volatile uint32_t *)0x4002603C))
#define DMA1_S2CR               (*((volatile uint32_t *)0x40026040))
#define DMA1_S2NDTR             (*((volatile uint32_t *)0x40026044))
#define DMA1_S2PAR              (*((volatile uint32_t *)0x40026048))
#define DMA1_S2M0AR             (*((volatile uint32_t *)0x4002604C))
#define DMA1_S2M1AR             (*((volatile uint32_t *)0x40026050))
#define DMA1_S2FCR              (*((volatile uint32_t *)0x40026054))
#define DMA1_S3CR               (*((volatile uint32_t *)0x40026058))
#define DMA1_S3NDTR             (*((volatile uint32_t *)0x4002605C))
#define DMA1_S3PAR              (*((volatile uint32_t *)0x40026060))
#define DMA1_S3M0AR             (*((volatile uint32_t *)0x40026064))
#define DMA1_S3M1AR             (*((volatile uint32_t *)0x40026068))
#define DMA1_S3FCR              (*((volatile uint32_t *)0x4002606C))
#define DMA1_S4CR               (*((volatile uint32_t *)0x40026070))
#define DMA1_S4NDTR             (*((volatile uint32_t *)0x40026074))
#define DMA1_S4PAR              (*((volatile uint32_t *)0x40026078))
#define DMA1_S4M0AR             (*((volatile uint32_t *)0x4002607C))
#define DMA1_S4M1AR             (*((volatile uint32_t *)0x40026080))
#define DMA1_S4FCR              (*((volatile uint32_t *)0x40026084))
#define DMA1_S5CR               (*((volatile uint32_t *)0x40026088))
#define DMA1_S5NDTR             (*((volatile uint32_t *)0x4002608C))
#define DMA1_S5PAR              (*((volatile uint32_t *)0x40026090))
#define DMA1_S5M0AR             (*((volatile uint32_t *)0x40026094))
#define DMA1_S5M1AR             (*((volatile uint32_t *)0x40026098))
#define DMA1_S5FCR              (*((volatile uint32_t *)0x4002609C))
#define DMA1_S6CR               (*((volatile uint32_t *)0x400260A0))
#define DMA1_S6NDTR             (*((volatile uint32_t *)0x400260A4))
#define DMA1_S6PAR              (*((volatile uint32_t *)0x400260A8))
#define DMA1_S6M0AR             (*((volatile uint32_t *)0x400260AC))
#define DMA1_S6M1AR             (*((volatile uint32_t *)0x400260B0))
#define DMA1_S6FCR              (*((volatile uint32_t *)0x400260B4))
#define DMA1_S7CR               (*((volatile uint32_t *)0x400260B8))
#define DMA1_S7NDTR             (*((volatile uint32_t *)0x400260BC))
#define DMA1_S7PAR              (*((volatile uint32_t *)0x400260C0))
#define DMA1_S7M0AR             (*((volatile uint32_t *)0x400260C4))
#define DMA1_S7M1AR             (*((volatile uint32_t *)0x400260C8))
#define DMA1_S7FCR              (*((volatile uint32_t *)0x400260CC))

#define DMA2_LISR               (*((volatile uint32_t *)0x40026400))
#define DMA2_HISR               (*((volatile uint32_t *)0x40026404))
#define DMA2_LIFCR              (*((volatile uint32_t *)0x40026408))
#define DMA2_HIFCR              (*((volatile uint32_t *)0x4002640C))
// DMA2 Stream 0-7 Registers
#define DMA2_S0CR               (*((volatile uint32_t *)0x40026410))
#define DMA2_S0NDTR             (*((volatile uint32_t *)0x40026414))
#define DMA2_S0PAR              (*((volatile uint32_t *)0x40026418))
#define DMA2_S0M0AR             (*((volatile uint32_t *)0x4002641C))
#define DMA2_S0M1AR             (*((volatile uint32_t *)0x40026420))
#define DMA2_S0FCR              (*((volatile uint32_t *)0x40026424))
#define DMA2_S1CR               (*((volatile uint32_t *)0x40026428))
#define DMA2_S1NDTR             (*((volatile uint32_t *)0x4002642C))
#define DMA2_S1PAR              (*((volatile uint32_t *)0x40026430))
#define DMA2_S1M0AR             (*((volatile uint32_t *)0x40026434))
#define DMA2_S1M1AR             (*((volatile uint32_t *)0x40026438))
#define DMA2_S1FCR              (*((volatile uint32_t *)0x4002643C))
#define DMA2_S2CR               (*((volatile uint32_t *)0x40026440))
#define DMA2_S2NDTR             (*((volatile uint32_t *)0x40026444))
#define DMA2_S2PAR              (*((volatile uint32_t *)0x40026448))
#define DMA2_S2M0AR             (*((volatile uint32_t *)0x4002644C))
#define DMA2_S2M1AR             (*((volatile uint32_t *)0x40026450))
#define DMA2_S2FCR              (*((volatile uint32_t *)0x40026454))
#define DMA2_S3CR               (*((volatile uint32_t *)0x40026458))
#define DMA2_S3NDTR             (*((volatile uint32_t *)0x4002645C))
#define DMA2_S3PAR              (*((volatile uint32_t *)0x40026460))
#define DMA2_S3M0AR             (*((volatile uint32_t *)0x40026464))
#define DMA2_S3M1AR             (*((volatile uint32_t *)0x40026468))
#define DMA2_S3FCR              (*((volatile uint32_t *)0x4002646C))
#define DMA2_S4CR               (*((volatile uint32_t *)0x40026470))
#define DMA2_S4NDTR             (*((volatile uint32_t *)0x40026474))
#define DMA2_S4PAR              (*((volatile uint32_t *)0x40026478))
#define DMA2_S4M0AR             (*((volatile uint32_t *)0x4002647C))
#define DMA2_S4M1AR             (*((volatile uint32_t *)0x40026480))
#define DMA2_S4FCR              (*((volatile uint32_t *)0x40026484))
#define DMA2_S5CR               (*((volatile uint32_t *)0x40026488))
#define DMA2_S5NDTR             (*((volatile uint32_t *)0x4002648C))
#define DMA2_S5PAR              (*((volatile uint32_t *)0x40026490))
#define DMA2_S5M0AR             (*((volatile uint32_t *)0x40026494))
#define DMA2_S5M1AR             (*((volatile uint32_t *)0x40026498))
#define DMA2_S5FCR              (*((volatile uint32_t *)0x4002649C))
#define DMA2_S6CR               (*((volatile uint32_t *)0x400264A0))
#define DMA2_S6NDTR             (*((volatile uint32_t *)0x400264A4))
#define DMA2_S6PAR              (*((volatile uint32_t *)0x400264A8))
#define DMA2_S6M0AR             (*((volatile uint32_t *)0x400264AC))
#define DMA2_S6M1AR             (*((volatile uint32_t *)0x400264B0))
#define DMA2_S6FCR              (*((volatile uint32_t *)0x400264B4))
#define DMA2_S7CR               (*((volatile uint32_t *)0x400264B8))
#define DMA2_S7NDTR             (*((volatile uint32_t *)0x400264BC))
#define DMA2_S7PAR              (*((volatile uint32_t *)0x400264C0))
#define DMA2_S7M0AR             (*((volatile uint32_t *)0x400264C4))
#define DMA2_S7M1AR             (*((volatile uint32_t *)0x400264C8))
#define DMA2_S7FCR              (*((volatile uint32_t *)0x400264CC))

// EXTI Registers
#define EXTI_IMR                (*((volatile uint32_t *)0x40013C00))
#define EXTI_EMR                (*((volatile uint32_t *)0x40013C04))
#define EXTI_RTSR               (*((volatile uint32_t *)0x40013C08))
#define EXTI_FTSR               (*((volatile uint32_t *)0x40013C0C))
#define EXTI_SWIER              (*((volatile uint32_t *)0x40013C10))
#define EXTI_PR                 (*((volatile uint32_t *)0x40013C14))

// ADC Registers
#define ADC_SR                  (*((volatile uint32_t *)0x40012000))
#define ADC_CR1                 (*((volatile uint32_t *)0x40012004))
#define ADC_CR2                 (*((volatile uint32_t *)0x40012008))
#define ADC_SMPR1               (*((volatile uint32_t *)0x4001200C))
#define ADC_SMPR2               (*((volatile uint32_t *)0x40012010))
#define ADC_JOFR1               (*((volatile uint32_t *)0x40012014))
#define ADC_JOFR2               (*((volatile uint32_t *)0x40012018))
#define ADC_JOFR3               (*((volatile uint32_t *)0x4001201C))
#define ADC_JOFR4               (*((volatile uint32_t *)0x40012020))
#define ADC_HTR                 (*((volatile uint32_t *)0x40012024))
#define ADC_LTR                 (*((volatile uint32_t *)0x40012028))
#define ADC_SQR1                (*((volatile uint32_t *)0x4001202C))
#define ADC_SQR2                (*((volatile uint32_t *)0x40012030))
#define ADC_SQR3                (*((volatile uint32_t *)0x40012034))
#define ADC_JSQR                (*((volatile uint32_t *)0x40012038))
#define ADC_JDR1                (*((volatile uint32_t *)0x4001203C))
#define ADC_JDR2                (*((volatile uint32_t *)0x40012040))
#define ADC_JDR3                (*((volatile uint32_t *)0x40012044))
#define ADC_JDR4                (*((volatile uint32_t *)0x40012048))
#define ADC_DR                  (*((volatile uint32_t *)0x4001204C))
#define ADC_CCR                 (*((volatile uint32_t *)0x40012300))

// Timer Registers (TIM1-TIM5, TIM9-TIM11)
// TIM1 Base: 0x40010000
#define TIM1_CR1                (*((volatile uint32_t *)0x40010000))
#define TIM1_CR2                (*((volatile uint32_t *)0x40010004))
#define TIM1_SMCR               (*((volatile uint32_t *)0x40010008))
#define TIM1_DIER               (*((volatile uint32_t *)0x4001000C))
#define TIM1_SR                 (*((volatile uint32_t *)0x40010010))
#define TIM1_EGR                (*((volatile uint32_t *)0x40010014))
#define TIM1_CCMR1              (*((volatile uint32_t *)0x40010018))
#define TIM1_CCMR2              (*((volatile uint32_t *)0x4001001C))
#define TIM1_CCER               (*((volatile uint32_t *)0x40010020))
#define TIM1_CNT                (*((volatile uint32_t *)0x40010024))
#define TIM1_PSC                (*((volatile uint32_t *)0x40010028))
#define TIM1_ARR                (*((volatile uint32_t *)0x4001002C))
#define TIM1_RCR                (*((volatile uint32_t *)0x40010030))
#define TIM1_CCR1               (*((volatile uint32_t *)0x40010034))
#define TIM1_CCR2               (*((volatile uint32_t *)0x40010038))
#define TIM1_CCR3               (*((volatile uint32_t *)0x4001003C))
#define TIM1_CCR4               (*((volatile uint32_t *)0x40010040))
#define TIM1_BDTR               (*((volatile uint32_t *)0x40010044))
#define TIM1_DCR                (*((volatile uint32_t *)0x40010048))
#define TIM1_DMAR               (*((volatile uint32_t *)0x4001004C))

// TIM2 Base: 0x40000000
#define TIM2_CR1                (*((volatile uint32_t *)0x40000000))
#define TIM2_CR2                (*((volatile uint32_t *)0x40000004))
#define TIM2_SMCR               (*((volatile uint32_t *)0x40000008))
#define TIM2_DIER               (*((volatile uint32_t *)0x4000000C))
#define TIM2_SR                 (*((volatile uint32_t *)0x40000010))
#define TIM2_EGR                (*((volatile uint32_t *)0x40000014))
#define TIM2_CCMR1              (*((volatile uint32_t *)0x40000018))
#define TIM2_CCMR2              (*((volatile uint32_t *)0x4000001C))
#define TIM2_CCER               (*((volatile uint32_t *)0x40000020))
#define TIM2_CNT                (*((volatile uint32_t *)0x40000024))
#define TIM2_PSC                (*((volatile uint32_t *)0x40000028))
#define TIM2_ARR                (*((volatile uint32_t *)0x4000002C))
#define TIM2_CCR1               (*((volatile uint32_t *)0x40000034))
#define TIM2_CCR2               (*((volatile uint32_t *)0x40000038))
#define TIM2_CCR3               (*((volatile uint32_t *)0x4000003C))
#define TIM2_CCR4               (*((volatile uint32_t *)0x40000040))
#define TIM2_DCR                (*((volatile uint32_t *)0x40000048))
#define TIM2_DMAR               (*((volatile uint32_t *)0x4000004C))
#define TIM2_OR                 (*((volatile uint32_t *)0x40000050))

// TIM3 Base: 0x40000400
#define TIM3_CR1                (*((volatile uint32_t *)0x40000400))
#define TIM3_CR2                (*((volatile uint32_t *)0x40000404))
#define TIM3_SMCR               (*((volatile uint32_t *)0x40000408))
#define TIM3_DIER               (*((volatile uint32_t *)0x4000040C))
#define TIM3_SR                 (*((volatile uint32_t *)0x40000410))
#define TIM3_EGR                (*((volatile uint32_t *)0x40000414))
#define TIM3_CCMR1              (*((volatile uint32_t *)0x40000418))
#define TIM3_CCMR2              (*((volatile uint32_t *)0x4000041C))
#define TIM3_CCER               (*((volatile uint32_t *)0x40000420))
#define TIM3_CNT                (*((volatile uint32_t *)0x40000424))
#define TIM3_PSC                (*((volatile uint32_t *)0x40000428))
#define TIM3_ARR                (*((volatile uint32_t *)0x4000042C))
#define TIM3_CCR1               (*((volatile uint32_t *)0x40000434))
#define TIM3_CCR2               (*((volatile uint32_t *)0x40000438))
#define TIM3_CCR3               (*((volatile uint32_t *)0x4000043C))
#define TIM3_CCR4               (*((volatile uint32_t *)0x40000440))
#define TIM3_DCR                (*((volatile uint32_t *)0x40000448))
#define TIM3_DMAR               (*((volatile uint32_t *)0x4000044C))

// TIM4 Base: 0x40000800
#define TIM4_CR1                (*((volatile uint32_t *)0x40000800))
#define TIM4_CR2                (*((volatile uint32_t *)0x40000804))
#define TIM4_SMCR               (*((volatile uint32_t *)0x40000808))
#define TIM4_DIER               (*((volatile uint32_t *)0x4000080C))
#define TIM4_SR                 (*((volatile uint32_t *)0x40000810))
#define TIM4_EGR                (*((volatile uint32_t *)0x40000814))
#define TIM4_CCMR1              (*((volatile uint32_t *)0x40000818))
#define TIM4_CCMR2              (*((volatile uint32_t *)0x4000081C))
#define TIM4_CCER               (*((volatile uint32_t *)0x40000820))
#define TIM4_CNT                (*((volatile uint32_t *)0x40000824))
#define TIM4_PSC                (*((volatile uint32_t *)0x40000828))
#define TIM4_ARR                (*((volatile uint32_t *)0x4000082C))
#define TIM4_CCR1               (*((volatile uint32_t *)0x40000834))
#define TIM4_CCR2               (*((volatile uint32_t *)0x40000838))
#define TIM4_CCR3               (*((volatile uint32_t *)0x4000083C))
#define TIM4_CCR4               (*((volatile uint32_t *)0x40000840))
#define TIM4_DCR                (*((volatile uint32_t *)0x40000848))
#define TIM4_DMAR               (*((volatile uint32_t *)0x4000084C))

// TIM5 Base: 0x40000C00
#define TIM5_CR1                (*((volatile uint32_t *)0x40000C00))
#define TIM5_CR2                (*((volatile uint32_t *)0x40000C04))
#define TIM5_SMCR               (*((volatile uint32_t *)0x40000C08))
#define TIM5_DIER               (*((volatile uint32_t *)0x40000C0C))
#define TIM5_SR                 (*((volatile uint32_t *)0x40000C10))
#define TIM5_EGR                (*((volatile uint32_t *)0x40000C14))
#define TIM5_CCMR1              (*((volatile uint32_t *)0x40000C18))
#define TIM5_CCMR2              (*((volatile uint32_t *)0x40000C1C))
#define TIM5_CCER               (*((volatile uint32_t *)0x40000C20))
#define TIM5_CNT                (*((volatile uint32_t *)0x40000C24))
#define TIM5_PSC                (*((volatile uint32_t *)0x40000C28))
#define TIM5_ARR                (*((volatile uint32_t *)0x40000C2C))
#define TIM5_CCR1               (*((volatile uint32_t *)0x40000C34))
#define TIM5_CCR2               (*((volatile uint32_t *)0x40000C38))
#define TIM5_CCR3               (*((volatile uint32_t *)0x40000C3C))
#define TIM5_CCR4               (*((volatile uint32_t *)0x40000C40))
#define TIM5_DCR                (*((volatile uint32_t *)0x40000C48))
#define TIM5_DMAR               (*((volatile uint32_t *)0x40000C4C))
#define TIM5_OR                 (*((volatile uint32_t *)0x40000C50))

// TIM9 Base: 0x40014000
#define TIM9_CR1                (*((volatile uint32_t *)0x40014000))
#define TIM9_SMCR               (*((volatile uint32_t *)0x40014008))
#define TIM9_DIER               (*((volatile uint32_t *)0x4001400C))
#define TIM9_SR                 (*((volatile uint32_t *)0x40014010))
#define TIM9_EGR                (*((volatile uint32_t *)0x40014014))
#define TIM9_CCMR1              (*((volatile uint32_t *)0x40014018))
#define TIM9_CCER               (*((volatile uint32_t *)0x40014020))
#define TIM9_CNT                (*((volatile uint32_t *)0x40014024))
#define TIM9_PSC                (*((volatile uint32_t *)0x40014028))
#define TIM9_ARR                (*((volatile uint32_t *)0x4001402C))
#define TIM9_CCR1               (*((volatile uint32_t *)0x40014034))
#define TIM9_CCR2               (*((volatile uint32_t *)0x40014038))

// TIM10 Base: 0x40014400
#define TIM10_CR1               (*((volatile uint32_t *)0x40014400))
#define TIM10_DIER              (*((volatile uint32_t *)0x4001440C))
#define TIM10_SR                (*((volatile uint32_t *)0x40014410))
#define TIM10_EGR               (*((volatile uint32_t *)0x40014414))
#define TIM10_CCMR1             (*((volatile uint32_t *)0x40014418))
#define TIM10_CCER              (*((volatile uint32_t *)0x40014420))
#define TIM10_CNT               (*((volatile uint32_t *)0x40014424))
#define TIM10_PSC               (*((volatile uint32_t *)0x40014428))
#define TIM10_ARR               (*((volatile uint32_t *)0x4001442C))
#define TIM10_CCR1              (*((volatile uint32_t *)0x40014434))

// TIM11 Base: 0x40014800
#define TIM11_CR1               (*((volatile uint32_t *)0x40014800))
#define TIM11_DIER              (*((volatile uint32_t *)0x4001480C))
#define TIM11_SR                (*((volatile uint32_t *)0x40014810))
#define TIM11_EGR               (*((volatile uint32_t *)0x40014814))
#define TIM11_CCMR1             (*((volatile uint32_t *)0x40014818))
#define TIM11_CCER              (*((volatile uint32_t *)0x40014820))
#define TIM11_CNT               (*((volatile uint32_t *)0x40014824))
#define TIM11_PSC               (*((volatile uint32_t *)0x40014828))
#define TIM11_ARR               (*((volatile uint32_t *)0x4001482C))
#define TIM11_CCR1              (*((volatile uint32_t *)0x40014834))

// USART Registers
// USART1 Base: 0x40011000
#define USART1_SR               (*((volatile uint32_t *)0x40011000))
#define USART1_DR               (*((volatile uint32_t *)0x40011004))
#define USART1_BRR              (*((volatile uint32_t *)0x40011008))
#define USART1_CR1              (*((volatile uint32_t *)0x4001100C))
#define USART1_CR2              (*((volatile uint32_t *)0x40011010))
#define USART1_CR3              (*((volatile uint32_t *)0x40011014))
#define USART1_GTPR             (*((volatile uint32_t *)0x40011018))

// USART2 Base: 0x40004400
#define USART2_SR               (*((volatile uint32_t *)0x40004400))
#define USART2_DR               (*((volatile uint32_t *)0x40004404))
#define USART2_BRR              (*((volatile uint32_t *)0x40004408))
#define USART2_CR1              (*((volatile uint32_t *)0x4000440C))
#define USART2_CR2              (*((volatile uint32_t *)0x40004410))
#define USART2_CR3              (*((volatile uint32_t *)0x40004414))
#define USART2_GTPR             (*((volatile uint32_t *)0x40004418))

// USART6 Base: 0x40011400
#define USART6_SR               (*((volatile uint32_t *)0x40011400))
#define USART6_DR               (*((volatile uint32_t *)0x40011404))
#define USART6_BRR              (*((volatile uint32_t *)0x40011408))
#define USART6_CR1              (*((volatile uint32_t *)0x4001140C))
#define USART6_CR2              (*((volatile uint32_t *)0x40011410))
#define USART6_CR3              (*((volatile uint32_t *)0x40011414))
#define USART6_GTPR             (*((volatile uint32_t *)0x40011418))

// I2C Registers
// I2C1 Base: 0x40005400
#define I2C1_CR1                (*((volatile uint32_t *)0x40005400))
#define I2C1_CR2                (*((volatile uint32_t *)0x40005404))
#define I2C1_OAR1               (*((volatile uint32_t *)0x40005408))
#define I2C1_OAR2               (*((volatile uint32_t *)0x4000540C))
#define I2C1_DR                 (*((volatile uint32_t *)0x40005410))
#define I2C1_SR1                (*((volatile uint32_t *)0x40005414))
#define I2C1_SR2                (*((volatile uint32_t *)0x40005418))
#define I2C1_CCR                (*((volatile uint32_t *)0x4000541C))
#define I2C1_TRISE              (*((volatile uint32_t *)0x40005420))
#define I2C1_FLTR               (*((volatile uint32_t *)0x40005424))

// I2C2 Base: 0x40005800
#define I2C2_CR1                (*((volatile uint32_t *)0x40005800))
#define I2C2_CR2                (*((volatile uint32_t *)0x40005804))
#define I2C2_OAR1               (*((volatile uint32_t *)0x40005808))
#define I2C2_OAR2               (*((volatile uint32_t *)0x4000580C))
#define I2C2_DR                 (*((volatile uint32_t *)0x40005810))
#define I2C2_SR1                (*((volatile uint32_t *)0x40005814))
#define I2C2_SR2                (*((volatile uint32_t *)0x40005818))
#define I2C2_CCR                (*((volatile uint32_t *)0x4000581C))
#define I2C2_TRISE              (*((volatile uint32_t *)0x40005820))
#define I2C2_FLTR               (*((volatile uint32_t *)0x40005824))

// I2C3 Base: 0x40005C00
#define I2C3_CR1                (*((volatile uint32_t *)0x40005C00))
#define I2C3_CR2                (*((volatile uint32_t *)0x40005C04))
#define I2C3_OAR1               (*((volatile uint32_t *)0x40005C08))
#define I2C3_OAR2               (*((volatile uint32_t *)0x40005C0C))
#define I2C3_DR                 (*((volatile uint32_t *)0x40005C10))
#define I2C3_SR1                (*((volatile uint32_t *)0x40005C14))
#define I2C3_SR2                (*((volatile uint32_t *)0x40005C18))
#define I2C3_CCR                (*((volatile uint32_t *)0x40005C1C))
#define I2C3_TRISE              (*((volatile uint32_t *)0x40005C20))
#define I2C3_FLTR               (*((volatile uint32_t *)0x40005C24))

// SPI Registers
// SPI1 Base: 0x40013000
#define SPI1_CR1                (*((volatile uint32_t *)0x40013000))
#define SPI1_CR2                (*((volatile uint32_t *)0x40013004))
#define SPI1_SR                 (*((volatile uint32_t *)0x40013008))
#define SPI1_DR                 (*((volatile uint32_t *)0x4001300C))
#define SPI1_CRCPR              (*((volatile uint32_t *)0x40013010))
#define SPI1_RXCRCR             (*((volatile uint32_t *)0x40013014))
#define SPI1_TXCRCR             (*((volatile uint32_t *)0x40013018))
#define SPI1_I2SCFGR            (*((volatile uint32_t *)0x4001301C))
#define SPI1_I2SPR              (*((volatile uint32_t *)0x40013020))

// SPI2 Base: 0x40003800
#define SPI2_CR1                (*((volatile uint32_t *)0x40003800))
#define SPI2_CR2                (*((volatile uint32_t *)0x40003804))
#define SPI2_SR                 (*((volatile uint32_t *)0x40003808))
#define SPI2_DR                 (*((volatile uint32_t *)0x4000380C))
#define SPI2_CRCPR              (*((volatile uint32_t *)0x40003810))
#define SPI2_RXCRCR             (*((volatile uint32_t *)0x40003814))
#define SPI2_TXCRCR             (*((volatile uint32_t *)0x40003818))
#define SPI2_I2SCFGR            (*((volatile uint32_t *)0x4000381C))
#define SPI2_I2SPR              (*((volatile uint32_t *)0x40003820))

// SPI3 Base: 0x40003C00
#define SPI3_CR1                (*((volatile uint32_t *)0x40003C00))
#define SPI3_CR2                (*((volatile uint32_t *)0x40003C04))
#define SPI3_SR                 (*((volatile uint32_t *)0x40003C08))
#define SPI3_DR                 (*((volatile uint32_t *)0x40003C0C))
#define SPI3_CRCPR              (*((volatile uint32_t *)0x40003C10))
#define SPI3_RXCRCR             (*((volatile uint32_t *)0x40003C14))
#define SPI3_TXCRCR             (*((volatile uint32_t *)0x40003C18))
#define SPI3_I2SCFGR            (*((volatile uint32_t *)0x40003C1C))
#define SPI3_I2SPR              (*((volatile uint32_t *)0x40003C20))


// --- Enumerations (from API.json and inferred MCU capabilities) ---

typedef enum
{
    Vsource_3V,
    Vsource_5V
} t_sys_volt;

typedef enum
{
    LVD_THRESHOLD_0_5V, // Placeholder, actual voltage levels depend on specific MCU PVD
    LVD_THRESHOLD_1V,   // Placeholder
    LVD_THRESHOLD_1_5V, // Placeholder
    LVD_THRESHOLD_2V,   // For 3V system: will map to PWR_CR PLS[2:0] for ~2.2V
    LVD_THRESHOLD_2_5V, // Placeholder
    LVD_THRESHOLD_3V,   // Placeholder
    LVD_THRESHOLD_3_5V, // For 5V system: will map to PWR_CR PLS[2:0] for ~3.5V
    LVD_THRESHOLD_4V,   // Placeholder
    LVD_THRESHOLD_4_5V, // Placeholder
    LVD_THRESHOLD_5V    // Placeholder
} t_lvd_thrthresholdLevel;

typedef enum
{
    LVD_CHANNEL_MAIN // LVD is typically a single unit, but API takes a channel
} t_lvd_channel;

typedef enum
{
    UART_CHANNEL_1, // USART1
    UART_CHANNEL_2, // USART2
    UART_CHANNEL_6  // USART6
} t_uart_channel;

typedef enum
{
    UART_BAUD_RATE_9600,
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_115200,
    // Add more standard baud rates as needed
} t_uart_baud_rate;

typedef enum
{
    UART_DATA_LENGTH_8_BITS,
    UART_DATA_LENGTH_9_BITS
} t_uart_data_length;

typedef enum
{
    UART_STOP_BIT_1,
    UART_STOP_BIT_0_5, // Not all MCUs support
    UART_STOP_BIT_2,
    UART_STOP_BIT_1_5  // Not all MCUs support
} t_uart_stop_bit;

typedef enum
{
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

typedef enum
{
    I2C_CHANNEL_1,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum
{
    I2C_CLK_SPEED_STANDARD, // 100 kHz
    I2C_CLK_SPEED_FAST      // 400 kHz
} t_i2c_clk_speed;

typedef enum
{
    I2C_DEVICE_ADDRESS_7_BIT_0x00, // Example, typically user-defined
    I2C_DEVICE_ADDRESS_7_BIT_0x01,
    // ...
    I2C_DEVICE_ADDRESS_10_BIT_0x00, // Example
    // Define specific common addresses if needed
} t_i2c_device_address;

typedef enum
{
    I2C_ACK_ENABLE,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum
{
    I2C_DATA_LENGTH_1_BYTE, // I2C transfers are byte-wise, this might refer to a concept not a hw register
    I2C_DATA_LENGTH_2_BYTE,
    I2C_DATA_LENGTH_4_BYTE
} t_i2c_datalength;

typedef enum
{
    SPI_CHANNEL_1,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

typedef enum
{
    SPI_MODE_SLAVE,
    SPI_MODE_MASTER
} t_spi_mode;

typedef enum
{
    SPI_CPOL_LOW,  // Clock Polarity Low
    SPI_CPOL_HIGH  // Clock Polarity High
} t_spi_cpol;

typedef enum
{
    SPI_CPHA_1_EDGE, // Clock Phase 1st Edge
    SPI_CPHA_2_EDGE  // Clock Phase 2nd Edge
} t_spi_cpha;

typedef enum
{
    SPI_DFF_8_BIT, // Data Frame Format 8-bit
    SPI_DFF_16_BIT // Data Frame Format 16-bit
} t_spi_dff;

typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST, // Most Significant Bit First
    SPI_BIT_ORDER_LSB_FIRST  // Least Significant Bit First
} t_spi_bit_order;

typedef enum
{
    EXT_INT_CHANNEL_0,
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
    EXT_INT_EDGE_RISING_FALLING
} t_external_int_edge;

typedef enum
{
    PORTA,
    PORTB,
    PORTC,
    PORTD,
    PORTE,
    PORTH
} t_port;

typedef enum
{
    PIN0,
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

typedef enum
{
    INPUT,
    OUTPUT
} t_direction;

typedef enum
{
    // TIM1 Channels
    PWM_CHANNEL_TIM1_CH1,  // PA8, PE9
    PWM_CHANNEL_TIM1_CH2,  // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,  // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,  // PA11, PE14
    // TIM2 Channels
    PWM_CHANNEL_TIM2_CH1,  // PA0, PA5, PA15
    PWM_CHANNEL_TIM2_CH2,  // PA1, PB3
    PWM_CHANNEL_TIM2_CH3,  // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,  // PA3, PB11
    // TIM3 Channels
    PWM_CHANNEL_TIM3_CH1,  // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,  // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,  // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,  // PB1, PC9
    // TIM4 Channels
    PWM_CHANNEL_TIM4_CH1,  // PB6, PD12
    PWM_CHANNEL_TIM4_CH2,  // PB7, PD13
    PWM_CHANNEL_TIM4_CH3,  // PB8, PD14
    PWM_CHANNEL_TIM4_CH4,  // PB9, PD15
    // TIM5 Channels
    PWM_CHANNEL_TIM5_CH1,  // PA0
    PWM_CHANNEL_TIM5_CH2,  // PA1
    PWM_CHANNEL_TIM5_CH3,  // PA2
    PWM_CHANNEL_TIM5_CH4,  // PA3
    // TIM9 Channels
    PWM_CHANNEL_TIM9_CH1,  // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,  // PA3, PE6
    // TIM10 Channels
    PWM_CHANNEL_TIM10_CH1, // PA6, PB8
    // TIM11 Channels
    PWM_CHANNEL_TIM11_CH1  // PA7, PB9
} t_pwm_channel;

typedef enum
{
    // ICU channels typically map to Timer Capture/Compare channels
    ICU_CHANNEL_TIM1_CH1,
    ICU_CHANNEL_TIM1_CH2,
    ICU_CHANNEL_TIM1_CH3,
    ICU_CHANNEL_TIM1_CH4,
    ICU_CHANNEL_TIM2_CH1,
    ICU_CHANNEL_TIM2_CH2,
    ICU_CHANNEL_TIM2_CH3,
    ICU_CHANNEL_TIM2_CH4,
    ICU_CHANNEL_TIM3_CH1,
    ICU_CHANNEL_TIM3_CH2,
    ICU_CHANNEL_TIM3_CH3,
    ICU_CHANNEL_TIM3_CH4,
    ICU_CHANNEL_TIM4_CH1,
    ICU_CHANNEL_TIM4_CH2,
    ICU_CHANNEL_TIM4_CH3,
    ICU_CHANNEL_TIM4_CH4,
    ICU_CHANNEL_TIM5_CH1,
    ICU_CHANNEL_TIM5_CH2,
    ICU_CHANNEL_TIM5_CH3,
    ICU_CHANNEL_TIM5_CH4,
    ICU_CHANNEL_TIM9_CH1,
    ICU_CHANNEL_TIM9_CH2,
    ICU_CHANNEL_TIM10_CH1,
    ICU_CHANNEL_TIM11_CH1
} t_icu_channel;

typedef enum
{
    ICU_PRESCALER_DIV1,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8
} t_icu_prescaller;

typedef enum
{
    ICU_EDGE_RISING,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

typedef enum
{
    TIMER_CHANNEL_TIM1,
    TIMER_CHANNEL_TIM2,
    TIMER_CHANNEL_TIM3,
    TIMER_CHANNEL_TIM4,
    TIMER_CHANNEL_TIM5,
    TIMER_CHANNEL_TIM9,
    TIMER_CHANNEL_TIM10,
    TIMER_CHANNEL_TIM11
} t_timer_channel;

typedef enum
{
    TICK_TIME_1MS,
    TICK_TIME_10MS,
    TICK_TIME_100MS,
    TICK_TIME_1S
} t_tick_time;

typedef enum
{
    ADC_CHANNEL_0,  // PA0, ADC_SMPR2 (channels 0-9)
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5,  // PA5
    ADC_CHANNEL_6,  // PA6
    ADC_CHANNEL_7,  // PA7
    ADC_CHANNEL_8,  // PB0
    ADC_CHANNEL_9,  // PB1
    ADC_CHANNEL_10, // PC0, ADC_SMPR1 (channels 10-18)
    ADC_CHANNEL_11, // PC1
    ADC_CHANNEL_12, // PC2
    ADC_CHANNEL_13, // PC3
    ADC_CHANNEL_14, // PC4
    ADC_CHANNEL_15  // PC5
} t_adc_channel;

typedef enum
{
    ADC_MODE_SINGLE_CONVERSION,
    ADC_MODE_CONTINUOUS_CONVERSION
} t_adc_mode_t;

// --- API Function Declarations (from API.json) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel); // This API is used for setting the threshold.
void LVD_Enable(void);
void LVD_Disable(void);
void LVD_ClearFlag(t_lvd_channel lvd_channel);

// UART
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_Update(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);
void UART_ClearFlag(t_uart_channel uart_channel);

// I2C
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_Update(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);
void I2C_ClearFlag(t_i2c_channel i2c_channel);

// SPI
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Update(void);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
void SPI_send_string(t_spi_channel spi_channel, const char *str);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);
void SPI_ClearFlag(t_spi_channel spi_channel);

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel);

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
void ICU_Updatefrequency(t_icu_channel icu_channel);
uint32_t ICU_GetFrequency(t_icu_channel icu_channel); // Returning uint32_t as frequency can be large
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel);

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel);

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void);

// Internal_EEPROM (Not supported by provided registers)
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time Triggered OS - requires a configured Timer)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void); // This would typically be a hardware ISR
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif // MCAL_H