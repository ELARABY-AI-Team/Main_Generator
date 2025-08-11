/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the declarations for the MCAL API functions,
 * register addresses, and type definitions for the STM32F401RC microcontroller.
 * All definitions and APIs are strictly based on the provided JSON inputs.
 */

#ifndef MCAL_H_
#define MCAL_H_

// Core MCU header file and standard C library includes
#include "stm32f401xc.h"  // Core device header (Inferred based on MCU name)
#include <stdint.h>       // For standard integer types (uint8_t, uint16_t, uint32_t)
#include <stdbool.h>      // For boolean type
#include <stddef.h>       // For size_t and NULL
#include <string.h>       // For string manipulation functions
// #include <stdio.h>     // Not strictly necessary for given APIs
// #include <stdlib.h>    // Not strictly necessary for given APIs
// #include <math.h>      // Not strictly necessary for given APIs

// --- Data Type Definitions ---
// As per "data_type_definitions" rule
#define Unit_8 uint8_t
#define unit_16 uint16_t
#define unit_32 uint32_t

typedef Unit_8 tbyte;
typedef unit_16 tword;
typedef unit_32 tlong;

// --- Register Definitions ---
// Using extern volatile uint32_t * for each register.
// These pointers will be initialized in MCAL.c
// Flash Registers
extern volatile uint32_t * const pFLASH_ACR;
extern volatile uint32_t * const pFLASH_KEYR;
extern volatile uint32_t * const pFLASH_OPTKEYR;
extern volatile uint32_t * const pFLASH_SR;
extern volatile uint32_t * const pFLASH_CR;
extern volatile uint32_t * const pFLASH_OPTCR;

// CRC Registers
extern volatile uint32_t * const pCRC_DR;
extern volatile uint32_t * const pCRC_IDR;
extern volatile uint32_t * const pCRC_CR;

// PWR Registers
extern volatile uint32_t * const pPWR_CR;
extern volatile uint32_t * const pPWR_CSR;

// RCC Registers
extern volatile uint32_t * const pRCC_CR;
extern volatile uint32_t * const pRCC_PLLCFGR;
extern volatile uint32_t * const pRCC_CFGR;
extern volatile uint32_t * const pRCC_CIR;
extern volatile uint32_t * const pRCC_AHB1RSTR;
extern volatile uint32_t * const pRCC_AHB2RSTR;
extern volatile uint32_t * const pRCC_APB1RSTR;
extern volatile uint32_t * const pRCC_APB2RSTR;
extern volatile uint32_t * const pRCC_AHB1ENR;
extern volatile uint32_t * const pRCC_AHB2ENR;
extern volatile uint32_t * const pRCC_APB1ENR;
extern volatile uint32_t * const pRCC_APB2ENR;
extern volatile uint32_t * const pRCC_AHB1LPENR;
extern volatile uint32_t * const pRCC_AHB2LPENR;
extern volatile uint32_t * const pRCC_APB1LPENR;
extern volatile uint32_t * const pRCC_APB2LPENR;
extern volatile uint32_t * const pRCC_BDCR;
extern volatile uint32_t * const pRCC_CSR;
extern volatile uint32_t * const pRCC_SSCGR;
extern volatile uint32_t * const pRCC_PLLI2SCFGR;
extern volatile uint32_t * const pRCC_DCKCFGR;

// SYSCFG Registers
extern volatile uint32_t * const pSYSCFG_MEMRMP;
extern volatile uint32_t * const pSYSCFG_PMC;
extern volatile uint32_t * const pSYSCFG_EXTICR1;
extern volatile uint32_t * const pSYSCFG_EXTICR2;
extern volatile uint32_t * const pSYSCFG_EXTICR3;
extern volatile uint32_t * const pSYSCFG_EXTICR4;
extern volatile uint32_t * const pSYSCFG_CMPCR;

// GPIO Registers
// Port A
extern volatile uint32_t * const pGPIOA_MODER;
extern volatile uint32_t * const pGPIOA_OTYPER;
extern volatile uint32_t * const pGPIOA_OSPEEDR;
extern volatile uint32_t * const pGPIOA_PUPDR;
extern volatile uint32_t * const pGPIOA_IDR;
extern volatile uint32_t * const pGPIOA_ODR;
extern volatile uint32_t * const pGPIOA_BSRR;
extern volatile uint32_t * const pGPIOA_LCKR;
extern volatile uint32_t * const pGPIOA_AFRL;
extern volatile uint32_t * const pGPIOA_AFRH;
// Port B
extern volatile uint32_t * const pGPIOB_MODER;
extern volatile uint32_t * const pGPIOB_OTYPER;
extern volatile uint32_t * const pGPIOB_OSPEEDR;
extern volatile uint32_t * const pGPIOB_PUPDR;
extern volatile uint32_t * const pGPIOB_IDR;
extern volatile uint32_t * const pGPIOB_ODR;
extern volatile uint32_t * const pGPIOB_BSRR;
extern volatile uint32_t * const pGPIOB_LCKR;
extern volatile uint32_t * const pGPIOB_AFRL;
extern volatile uint32_t * const pGPIOB_AFRH;
// Port C
extern volatile uint32_t * const pGPIOC_MODER;
extern volatile uint32_t * const pGPIOC_OTYPER;
extern volatile uint32_t * const pGPIOC_OSPEEDR;
extern volatile uint32_t * const pGPIOC_PUPDR;
extern volatile uint32_t * const pGPIOC_IDR;
extern volatile uint32_t * const pGPIOC_ODR;
extern volatile uint32_t * const pGPIOC_BSRR;
extern volatile uint32_t * const pGPIOC_LCKR;
extern volatile uint32_t * const pGPIOC_AFRL;
extern volatile uint32_t * const pGPIOC_AFRH;
// Port D
extern volatile uint32_t * const pGPIOD_MODER;
extern volatile uint32_t * const pGPIOD_OTYPER;
extern volatile uint32_t * const pGPIOD_OSPEEDR;
extern volatile uint32_t * const pGPIOD_PUPDR;
extern volatile uint32_t * const pGPIOD_IDR;
extern volatile uint32_t * const pGPIOD_ODR;
extern volatile uint32_t * const pGPIOD_BSRR;
extern volatile uint32_t * const pGPIOD_LCKR;
extern volatile uint32_t * const pGPIOD_AFRL;
extern volatile uint32_t * const pGPIOD_AFRH;
// Port E
extern volatile uint32_t * const pGPIOE_MODER;
extern volatile uint32_t * const pGPIOE_OTYPER;
extern volatile uint32_t * const pGPIOE_OSPEEDR;
extern volatile uint32_t * const pGPIOE_PUPDR;
extern volatile uint32_t * const pGPIOE_IDR;
extern volatile uint32_t * const pGPIOE_ODR;
extern volatile uint32_t * const pGPIOE_BSRR;
extern volatile uint32_t * const pGPIOE_LCKR;
extern volatile uint32_t * const pGPIOE_AFRL;
extern volatile uint32_t * const pGPIOE_AFRH;
// Port H
extern volatile uint32_t * const pGPIOH_MODER;
extern volatile uint32_t * const pGPIOH_OTYPER;
extern volatile uint32_t * const pGPIOH_OSPEEDR;
extern volatile uint32_t * const pGPIOH_PUPDR;
extern volatile uint32_t * const pGPIOH_IDR;
extern volatile uint32_t * const pGPIOH_ODR;
extern volatile uint32_t * const pGPIOH_BSRR;
extern volatile uint32_t * const pGPIOH_LCKR;
extern volatile uint32_t * const pGPIOH_AFRL;
extern volatile uint32_t * const pGPIOH_AFRH;

// DMA Registers
// DMA1
extern volatile uint32_t * const pDMA1_LISR;
extern volatile uint32_t * const pDMA1_HISR;
extern volatile uint32_t * const pDMA1_LIFCR;
extern volatile uint32_t * const pDMA1_HIFCR;
extern volatile uint32_t * const pDMA1_S0CR;
extern volatile uint32_t * const pDMA1_S0NDTR;
extern volatile uint32_t * const pDMA1_S0PAR;
extern volatile uint32_t * const pDMA1_S0M0AR;
extern volatile uint32_t * const pDMA1_S0M1AR;
extern volatile uint32_t * const pDMA1_S0FCR;
extern volatile uint32_t * const pDMA1_S1CR;
extern volatile uint32_t * const pDMA1_S1NDTR;
extern volatile uint32_t * const pDMA1_S1PAR;
extern volatile uint32_t * const pDMA1_S1M0AR;
extern volatile uint32_t * const pDMA1_S1M1AR;
extern volatile uint32_t * const pDMA1_S1FCR;
extern volatile uint32_t * const pDMA1_S2CR;
extern volatile uint32_t * const pDMA1_S2NDTR;
extern volatile uint32_t * const pDMA1_S2PAR;
extern volatile uint32_t * const pDMA1_S2M0AR;
extern volatile uint32_t * const pDMA1_S2M1AR;
extern volatile uint32_t * const pDMA1_S2FCR;
extern volatile uint32_t * const pDMA1_S3CR;
extern volatile uint32_t * const pDMA1_S3NDTR;
extern volatile uint32_t * const pDMA1_S3PAR;
extern volatile uint32_t * const pDMA1_S3M0AR;
extern volatile uint32_t * const pDMA1_S3M1AR;
extern volatile uint32_t * const pDMA1_S3FCR;
extern volatile uint32_t * const pDMA1_S4CR;
extern volatile uint32_t * const pDMA1_S4NDTR;
extern volatile uint32_t * const pDMA1_S4PAR;
extern volatile uint32_t * const pDMA1_S4M0AR;
extern volatile uint32_t * const pDMA1_S4M1AR;
extern volatile uint32_t * const pDMA1_S4FCR;
extern volatile uint32_t * const pDMA1_S5CR;
extern volatile uint32_t * const pDMA1_S5NDTR;
extern volatile uint32_t * const pDMA1_S5PAR;
extern volatile uint32_t * const pDMA1_S5M0AR;
extern volatile uint32_t * const pDMA1_S5M1AR;
extern volatile uint32_t * const pDMA1_S5FCR;
extern volatile uint32_t * const pDMA1_S6CR;
extern volatile uint32_t * const pDMA1_S6NDTR;
extern volatile uint32_t * const pDMA1_S6PAR;
extern volatile uint32_t * const pDMA1_S6M0AR;
extern volatile uint32_t * const pDMA1_S6M1AR;
extern volatile uint32_t * const pDMA1_S6FCR;
extern volatile uint32_t * const pDMA1_S7CR;
extern volatile uint32_t * const pDMA1_S7NDTR;
extern volatile uint32_t * const pDMA1_S7PAR;
extern volatile uint32_t * const pDMA1_S7M0AR;
extern volatile uint32_t * const pDMA1_S7M1AR;
extern volatile uint32_t * const pDMA1_S7FCR;
// DMA2
extern volatile uint32_t * const pDMA2_LISR;
extern volatile uint32_t * const pDMA2_HISR;
extern volatile uint32_t * const pDMA2_LIFCR;
extern volatile uint32_t * const pDMA2_HIFCR;
extern volatile uint32_t * const pDMA2_S0CR;
extern volatile uint32_t * const pDMA2_S0NDTR;
extern volatile uint32_t * const pDMA2_S0PAR;
extern volatile uint32_t * const pDMA2_S0M0AR;
extern volatile uint32_t * const pDMA2_S0M1AR;
extern volatile uint32_t * const pDMA2_S0FCR;
extern volatile uint32_t * const pDMA2_S1CR;
extern volatile uint32_t * const pDMA2_S1NDTR;
extern volatile uint32_t * const pDMA2_S1PAR;
extern volatile uint32_t * const pDMA2_S1M0AR;
extern volatile uint32_t * const pDMA2_S1M1AR;
extern volatile uint32_t * const pDMA2_S1FCR;
extern volatile uint32_t * const pDMA2_S2CR;
extern volatile uint32_t * const pDMA2_S2NDTR;
extern volatile uint32_t * const pDMA2_S2PAR;
extern volatile uint32_t * const pDMA2_S2M0AR;
extern volatile uint32_t * const pDMA2_S2M1AR;
extern volatile uint32_t * const pDMA2_S2FCR;
extern volatile uint32_t * const pDMA2_S3CR;
extern volatile uint32_t * const pDMA2_S3NDTR;
extern volatile uint32_t * const pDMA2_S3PAR;
extern volatile uint32_t * const pDMA2_S3M0AR;
extern volatile uint32_t * const pDMA2_S3M1AR;
extern volatile uint32_t * const pDMA2_S3FCR;
extern volatile uint32_t * const pDMA2_S4CR;
extern volatile uint32_t * const pDMA2_S4NDTR;
extern volatile uint32_t * const pDMA2_S4PAR;
extern volatile uint32_t * const pDMA2_S4M0AR;
extern volatile uint32_t * const pDMA2_S4M1AR;
extern volatile uint32_t * const pDMA2_S4FCR;
extern volatile uint32_t * const pDMA2_S5CR;
extern volatile uint32_t * const pDMA2_S5NDTR;
extern volatile uint32_t * const pDMA2_S5PAR;
extern volatile uint32_t * const pDMA2_S5M0AR;
extern volatile uint32_t * const pDMA2_S5M1AR;
extern volatile uint32_t * const pDMA2_S5FCR;
extern volatile uint32_t * const pDMA2_S6CR;
extern volatile uint32_t * const pDMA2_S6NDTR;
extern volatile uint32_t * const pDMA2_S6PAR;
extern volatile uint32_t * const pDMA2_S6M0AR;
extern volatile uint32_t * const pDMA2_S6M1AR;
extern volatile uint32_t * const pDMA2_S6FCR;
extern volatile uint32_t * const pDMA2_S7CR;
extern volatile uint32_t * const pDMA2_S7NDTR;
extern volatile uint32_t * const pDMA2_S7PAR;
extern volatile uint32_t * const pDMA2_S7M0AR;
extern volatile uint32_t * const pDMA2_S7M1AR;
extern volatile uint32_t * const pDMA2_S7FCR;

// EXTI Registers
extern volatile uint32_t * const pEXTI_IMR;
extern volatile uint32_t * const pEXTI_EMR;
extern volatile uint32_t * const pEXTI_RTSR;
extern volatile uint32_t * const pEXTI_FTSR;
extern volatile uint32_t * const pEXTI_SWIER;
extern volatile uint32_t * const pEXTI_PR;

// ADC Registers
extern volatile uint32_t * const pADC_SR;
extern volatile uint32_t * const pADC_CR1;
extern volatile uint32_t * const pADC_CR2;
extern volatile uint32_t * const pADC_SMPR1;
extern volatile uint32_t * const pADC_SMPR2;
extern volatile uint32_t * const pADC_JOFR1;
extern volatile uint32_t * const pADC_JOFR2;
extern volatile uint32_t * const pADC_JOFR3;
extern volatile uint32_t * const pADC_JOFR4;
extern volatile uint32_t * const pADC_HTR;
extern volatile uint32_t * const pADC_LTR;
extern volatile uint32_t * const pADC_SQR1;
extern volatile uint32_t * const pADC_SQR2;
extern volatile uint32_t * const pADC_SQR3;
extern volatile uint32_t * const pADC_JSQR;
extern volatile uint32_t * const pADC_JDR1;
extern volatile uint32_t * const pADC_JDR2;
extern volatile uint32_t * const pADC_JDR3;
extern volatile uint32_t * const pADC_JDR4;
extern volatile uint32_t * const pADC_DR;
extern volatile uint32_t * const pADC_CCR;

// TIM Registers
// TIM1
extern volatile uint32_t * const pTIM1_CR1;
extern volatile uint32_t * const pTIM1_CR2;
extern volatile uint32_t * const pTIM1_SMCR;
extern volatile uint32_t * const pTIM1_DIER;
extern volatile uint32_t * const pTIM1_SR;
extern volatile uint32_t * const pTIM1_EGR;
extern volatile uint32_t * const pTIM1_CCMR1;
extern volatile uint32_t * const pTIM1_CCMR2;
extern volatile uint32_t * const pTIM1_CCER;
extern volatile uint32_t * const pTIM1_CNT;
extern volatile uint32_t * const pTIM1_PSC;
extern volatile uint32_t * const pTIM1_ARR;
extern volatile uint32_t * const pTIM1_RCR;
extern volatile uint32_t * const pTIM1_CCR1;
extern volatile uint32_t * const pTIM1_CCR2;
extern volatile uint32_t * const pTIM1_CCR3;
extern volatile uint32_t * const pTIM1_CCR4;
extern volatile uint32_t * const pTIM1_BDTR;
extern volatile uint32_t * const pTIM1_DCR;
extern volatile uint32_t * const pTIM1_DMAR;
// TIM2
extern volatile uint32_t * const pTIM2_CR1;
extern volatile uint32_t * const pTIM2_CR2;
extern volatile uint32_t * const pTIM2_SMCR;
extern volatile uint32_t * const pTIM2_DIER;
extern volatile uint32_t * const pTIM2_SR;
extern volatile uint32_t * const pTIM2_EGR;
extern volatile uint32_t * const pTIM2_CCMR1;
extern volatile uint32_t * const pTIM2_CCMR2;
extern volatile uint32_t * const pTIM2_CCER;
extern volatile uint32_t * const pTIM2_CNT;
extern volatile uint32_t * const pTIM2_PSC;
extern volatile uint32_t * const pTIM2_ARR;
extern volatile uint32_t * const pTIM2_CCR1;
extern volatile uint32_t * const pTIM2_CCR2;
extern volatile uint32_t * const pTIM2_CCR3;
extern volatile uint32_t * const pTIM2_CCR4;
extern volatile uint32_t * const pTIM2_DCR;
extern volatile uint32_t * const pTIM2_DMAR;
extern volatile uint32_t * const pTIM2_OR;
// TIM3
extern volatile uint32_t * const pTIM3_CR1;
extern volatile uint32_t * const pTIM3_CR2;
extern volatile uint32_t * const pTIM3_SMCR;
extern volatile uint32_t * const pTIM3_DIER;
extern volatile uint32_t * const pTIM3_SR;
extern volatile uint32_t * const pTIM3_EGR;
extern volatile uint32_t * const pTIM3_CCMR1;
extern volatile uint32_t * const pTIM3_CCMR2;
extern volatile uint32_t * const pTIM3_CCER;
extern volatile uint32_t * const pTIM3_CNT;
extern volatile uint32_t * const pTIM3_PSC;
extern volatile uint32_t * const pTIM3_ARR;
extern volatile uint32_t * const pTIM3_CCR1;
extern volatile uint32_t * const pTIM3_CCR2;
extern volatile uint32_t * const pTIM3_CCR3;
extern volatile uint32_t * const pTIM3_CCR4;
extern volatile uint32_t * const pTIM3_DCR;
extern volatile uint32_t * const pTIM3_DMAR;
// TIM4
extern volatile uint32_t * const pTIM4_CR1;
extern volatile uint32_t * const pTIM4_CR2;
extern volatile uint32_t * const pTIM4_SMCR;
extern volatile uint32_t * const pTIM4_DIER;
extern volatile uint32_t * const pTIM4_SR;
extern volatile uint32_t * const pTIM4_EGR;
extern volatile uint32_t * const pTIM4_CCMR1;
extern volatile uint32_t * const pTIM4_CCMR2;
extern volatile uint32_t * const pTIM4_CCER;
extern volatile uint32_t * const pTIM4_CNT;
extern volatile uint32_t * const pTIM4_PSC;
extern volatile uint32_t * const pTIM4_ARR;
extern volatile uint32_t * const pTIM4_CCR1;
extern volatile uint32_t * const pTIM4_CCR2;
extern volatile uint32_t * const pTIM4_CCR3;
extern volatile uint32_t * const pTIM4_CCR4;
extern volatile uint32_t * const pTIM4_DCR;
extern volatile uint32_t * const pTIM4_DMAR;
// TIM5
extern volatile uint32_t * const pTIM5_CR1;
extern volatile uint32_t * const pTIM5_CR2;
extern volatile uint32_t * const pTIM5_SMCR;
extern volatile uint32_t * const pTIM5_DIER;
extern volatile uint32_t * const pTIM5_SR;
extern volatile uint32_t * const pTIM5_EGR;
extern volatile uint32_t * const pTIM5_CCMR1;
extern volatile uint32_t * const pTIM5_CCMR2;
extern volatile uint32_t * const pTIM5_CCER;
extern volatile uint32_t * const pTIM5_CNT;
extern volatile uint32_t * const pTIM5_PSC;
extern volatile uint32_t * const pTIM5_ARR;
extern volatile uint32_t * const pTIM5_CCR1;
extern volatile uint32_t * const pTIM5_CCR2;
extern volatile uint32_t * const pTIM5_CCR3;
extern volatile uint32_t * const pTIM5_CCR4;
extern volatile uint32_t * const pTIM5_DCR;
extern volatile uint32_t * const pTIM5_DMAR;
extern volatile uint32_t * const pTIM5_OR;
// TIM9
extern volatile uint32_t * const pTIM9_CR1;
extern volatile uint32_t * const pTIM9_SMCR;
extern volatile uint32_t * const pTIM9_DIER;
extern volatile uint32_t * const pTIM9_SR;
extern volatile uint32_t * const pTIM9_EGR;
extern volatile uint32_t * const pTIM9_CCMR1;
extern volatile uint32_t * const pTIM9_CCER;
extern volatile uint32_t * const pTIM9_CNT;
extern volatile uint32_t * const pTIM9_PSC;
extern volatile uint32_t * const pTIM9_ARR;
extern volatile uint32_t * const pTIM9_CCR1;
extern volatile uint32_t * const pTIM9_CCR2;
// TIM10
extern volatile uint32_t * const pTIM10_CR1;
extern volatile uint32_t * const pTIM10_DIER;
extern volatile uint32_t * const pTIM10_SR;
extern volatile uint32_t * const pTIM10_EGR;
extern volatile uint32_t * const pTIM10_CCMR1;
extern volatile uint32_t * const pTIM10_CCER;
extern volatile uint32_t * const pTIM10_CNT;
extern volatile uint32_t * const pTIM10_PSC;
extern volatile uint32_t * const pTIM10_ARR;
extern volatile uint32_t * const pTIM10_CCR1;
// TIM11
extern volatile uint32_t * const pTIM11_CR1;
extern volatile uint32_t * const pTIM11_DIER;
extern volatile uint32_t * const pTIM11_SR;
extern volatile uint32_t * const pTIM11_EGR;
extern volatile uint32_t * const pTIM11_CCMR1;
extern volatile uint32_t * const pTIM11_CCER;
extern volatile uint32_t * const pTIM11_CNT;
extern volatile uint32_t * const pTIM11_PSC;
extern volatile uint32_t * const pTIM11_ARR;
extern volatile uint32_t * const pTIM11_CCR1;

// USART Registers
// USART1
extern volatile uint32_t * const pUSART1_SR;
extern volatile uint32_t * const pUSART1_DR;
extern volatile uint32_t * const pUSART1_BRR;
extern volatile uint32_t * const pUSART1_CR1;
extern volatile uint32_t * const pUSART1_CR2;
extern volatile uint32_t * const pUSART1_CR3;
extern volatile uint32_t * const pUSART1_GTPR;
// USART2
extern volatile uint32_t * const pUSART2_SR;
extern volatile uint32_t * const pUSART2_DR;
extern volatile uint32_t * const pUSART2_BRR;
extern volatile uint32_t * const pUSART2_CR1;
extern volatile uint32_t * const pUSART2_CR2;
extern volatile uint32_t * const pUSART2_CR3;
extern volatile uint32_t * const pUSART2_GTPR;
// USART6
extern volatile uint32_t * const pUSART6_SR;
extern volatile uint32_t * const pUSART6_DR;
extern volatile uint32_t * const pUSART6_BRR;
extern volatile uint32_t * const pUSART6_CR1;
extern volatile uint32_t * const pUSART6_CR2;
extern volatile uint32_t * const pUSART6_CR3;
extern volatile uint32_t * const pUSART6_GTPR;

// I2C Registers
// I2C1
extern volatile uint32_t * const pI2C1_CR1;
extern volatile uint32_t * const pI2C1_CR2;
extern volatile uint32_t * const pI2C1_OAR1;
extern volatile uint32_t * const pI2C1_OAR2;
extern volatile uint32_t * const pI2C1_DR;
extern volatile uint32_t * const pI2C1_SR1;
extern volatile uint32_t * const pI2C1_SR2;
extern volatile uint32_t * const pI2C1_CCR;
extern volatile uint32_t * const pI2C1_TRISE;
extern volatile uint32_t * const pI2C1_FLTR;
// I2C2
extern volatile uint32_t * const pI2C2_CR1;
extern volatile uint32_t * const pI2C2_CR2;
extern volatile uint32_t * const pI2C2_OAR1;
extern volatile uint32_t * const pI2C2_OAR2;
extern volatile uint32_t * const pI2C2_DR;
extern volatile uint32_t * const pI2C2_SR1;
extern volatile uint32_t * const pI2C2_SR2;
extern volatile uint32_t * const pI2C2_CCR;
extern volatile uint32_t * const pI2C2_TRISE;
extern volatile uint32_t * const pI2C2_FLTR;
// I2C3
extern volatile uint32_t * const pI2C3_CR1;
extern volatile uint32_t * const pI2C3_CR2;
extern volatile uint32_t * const pI2C3_OAR1;
extern volatile uint32_t * const pI2C3_OAR2;
extern volatile uint32_t * const pI2C3_DR;
extern volatile uint32_t * const pI2C3_SR1;
extern volatile uint32_t * const pI2C3_SR2;
extern volatile uint32_t * const pI2C3_CCR;
extern volatile uint32_t * const pI2C3_TRISE;
extern volatile uint32_t * const pI2C3_FLTR;

// SPI Registers
// SPI1
extern volatile uint32_t * const pSPI1_CR1;
extern volatile uint32_t * const pSPI1_CR2;
extern volatile uint32_t * const pSPI1_SR;
extern volatile uint32_t * const pSPI1_DR;
extern volatile uint32_t * const pSPI1_CRCPR;
extern volatile uint32_t * const pSPI1_RXCRCR;
extern volatile uint32_t * const pSPI1_TXCRCR;
extern volatile uint32_t * const pSPI1_I2SCFGR;
extern volatile uint32_t * const pSPI1_I2SPR;
// SPI2
extern volatile uint32_t * const pSPI2_CR1;
extern volatile uint32_t * const pSPI2_CR2;
extern volatile uint32_t * const pSPI2_SR;
extern volatile uint32_t * const pSPI2_DR;
extern volatile uint32_t * const pSPI2_CRCPR;
extern volatile uint32_t * const pSPI2_RXCRCR;
extern volatile uint32_t * const pSPI2_TXCRCR;
extern volatile uint32_t * const pSPI2_I2SCFGR;
extern volatile uint32_t * const pSPI2_I2SPR;
// SPI3
extern volatile uint32_t * const pSPI3_CR1;
extern volatile uint32_t * const pSPI3_CR2;
extern volatile uint32_t * const pSPI3_SR;
extern volatile uint32_t * const pSPI3_DR;
extern volatile uint32_t * const pSPI3_CRCPR;
extern volatile uint32_t * const pSPI3_RXCRCR;
extern volatile uint32_t * const pSPI3_TXCRCR;
extern volatile uint32_t * const pSPI3_I2SCFGR;
extern volatile uint32_t * const pSPI3_I2SPR;

// --- Enums and Type Definitions for API Parameters ---

/**
 * @brief System Voltage for MCU_Config_Init.
 */
typedef enum
{
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

/**
 * @brief LVD Threshold Levels.
 * As per "LVD_requirements" rule.
 */
typedef enum
{
    LVD_VOLT_0_5V = 0, // Placeholder mapping, actual register bits will vary
    LVD_VOLT_1V,
    LVD_VOLT_1_5V,
    LVD_VOLT_2V,
    LVD_VOLT_2_5V,
    LVD_VOLT_3V,
    LVD_VOLT_3_5V,
    LVD_VOLT_4V,
    LVD_VOLT_4_5V,
    LVD_VOLT_5V
} t_lvd_thrthresholdLevel;

/**
 * @brief LVD Channel identifier.
 * STM32F401 typically has one PVD/LVD module.
 */
typedef enum
{
    LVD_CHANNEL_MAIN = 0
} t_lvd_channel;

/**
 * @brief UART Channel identifiers.
 */
typedef enum
{
    UART_CHANNEL_1 = 0, // Maps to USART1
    UART_CHANNEL_2,     // Maps to USART2
    UART_CHANNEL_6      // Maps to USART6
} t_uart_channel;

/**
 * @brief UART Baud Rates.
 * Common baud rates, specific values depend on clock.
 */
typedef enum
{
    UART_BAUD_RATE_9600 = 0,
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_38400,
    UART_BAUD_RATE_57600,
    UART_BAUD_RATE_115200
} t_uart_baud_rate;

/**
 * @brief UART Data Length.
 */
typedef enum
{
    UART_DATA_8BIT = 0,
    UART_DATA_9BIT
} t_uart_data_length;

/**
 * @brief UART Stop Bit configuration.
 */
typedef enum
{
    UART_STOP_BITS_1 = 0,
    UART_STOP_BITS_0_5,
    UART_STOP_BITS_2,
    UART_STOP_BITS_1_5
} t_uart_stop_bit;

/**
 * @brief UART Parity configuration.
 */
typedef enum
{
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

/**
 * @brief I2C Channel identifiers.
 */
typedef enum
{
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

/**
 * @brief I2C Clock Speed.
 * As per "I2C_rules", always use fast mode.
 */
typedef enum
{
    I2C_CLK_SPEED_STANDARD = 0, // 100 kHz
    I2C_CLK_SPEED_FAST          // 400 kHz (chosen as default per rule)
} t_i2c_clk_speed;

/**
 * @brief I2C Device Address.
 * Placeholder, actual address will be a numerical value.
 */
typedef uint16_t t_i2c_device_address;

/**
 * @brief I2C Acknowledge control.
 */
typedef enum
{
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

/**
 * @brief I2C Data Length.
 */
typedef enum
{
    I2C_DATA_LENGTH_8BIT = 0,
    I2C_DATA_LENGTH_16BIT // Though I2C is typically 8-bit, including as per generic parameter
} t_i2c_datalength;

/**
 * @brief SPI Channel identifiers.
 */
typedef enum
{
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

/**
 * @brief SPI Mode (Master/Slave).
 */
typedef enum
{
    SPI_MODE_SLAVE = 0,
    SPI_MODE_MASTER
} t_spi_mode;

/**
 * @brief SPI Clock Polarity (CPOL).
 */
typedef enum
{
    SPI_CPOL_LOW = 0,  // Clock low when idle
    SPI_CPOL_HIGH      // Clock high when idle
} t_spi_cpol;

/**
 * @brief SPI Clock Phase (CPHA).
 */
typedef enum
{
    SPI_CPHA_1EDGE = 0, // Data captured on first clock edge
    SPI_CPHA_2EDGE      // Data captured on second clock edge
} t_spi_cpha;

/**
 * @brief SPI Data Frame Format (DFF).
 */
typedef enum
{
    SPI_DFF_8BIT = 0,
    SPI_DFF_16BIT
} t_spi_dff;

/**
 * @brief SPI Bit Order (MSB/LSB first).
 */
typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

/**
 * @brief External Interrupt Channel (EXTI line).
 */
typedef enum
{
    EXTI_LINE_0 = 0,
    EXTI_LINE_1,
    EXTI_LINE_2,
    EXTI_LINE_3,
    EXTI_LINE_4,
    EXTI_LINE_5,
    EXTI_LINE_6,
    EXTI_LINE_7,
    EXTI_LINE_8,
    EXTI_LINE_9,
    EXTI_LINE_10,
    EXTI_LINE_11,
    EXTI_LINE_12,
    EXTI_LINE_13,
    EXTI_LINE_14,
    EXTI_LINE_15
} t_external_int_channel;

/**
 * @brief External Interrupt Edge Trigger.
 */
typedef enum
{
    EXTI_EDGE_RISING = 0,
    EXTI_EDGE_FALLING,
    EXTI_EDGE_BOTH // Rising and Falling
} t_external_int_edge;

/**
 * @brief GPIO Port identifiers.
 */
typedef enum
{
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    PORT_H
} t_port;

/**
 * @brief GPIO Pin identifiers (0-15).
 */
typedef enum
{
    PIN_0 = 0,
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

/**
 * @brief GPIO Direction.
 */
typedef enum
{
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;

/**
 * @brief Timer Channel for PWM/ICU.
 * Maps to specific Timer instance and channel, combining them for clarity.
 */
typedef enum
{
    // TIM1 Channels
    TIMER_CHANNEL_TIM1_CH1 = 0, // PA8, PE9
    TIMER_CHANNEL_TIM1_CH2,     // PA9, PE11
    TIMER_CHANNEL_TIM1_CH3,     // PA10, PE13
    TIMER_CHANNEL_TIM1_CH4,     // PA11, PE14
    // TIM2 Channels
    TIMER_CHANNEL_TIM2_CH1,     // PA0, PA5, PA15
    TIMER_CHANNEL_TIM2_CH2,     // PA1, PB3
    TIMER_CHANNEL_TIM2_CH3,     // PA2, PB10
    TIMER_CHANNEL_TIM2_CH4,     // PA3, PB11
    // TIM3 Channels
    TIMER_CHANNEL_TIM3_CH1,     // PA6, PB4, PC6
    TIMER_CHANNEL_TIM3_CH2,     // PA7, PB5, PC7
    TIMER_CHANNEL_TIM3_CH3,     // PB0, PC8
    TIMER_CHANNEL_TIM3_CH4,     // PB1, PC9
    // TIM4 Channels
    TIMER_CHANNEL_TIM4_CH1,     // PB6, PD12
    TIMER_CHANNEL_TIM4_CH2,     // PB7, PD13
    TIMER_CHANNEL_TIM4_CH3,     // PB8, PD14
    TIMER_CHANNEL_TIM4_CH4,     // PB9, PD15
    // TIM5 Channels
    TIMER_CHANNEL_TIM5_CH1,     // PA0
    TIMER_CHANNEL_TIM5_CH2,     // PA1
    TIMER_CHANNEL_TIM5_CH3,     // PA2
    TIMER_CHANNEL_TIM5_CH4,     // PA3
    // TIM9 Channels
    TIMER_CHANNEL_TIM9_CH1,     // PA2, PE5
    TIMER_CHANNEL_TIM9_CH2,     // PA3, PE6
    // TIM10 Channels
    TIMER_CHANNEL_TIM10_CH1,    // PA6, PB8
    // TIM11 Channels
    TIMER_CHANNEL_TIM11_CH1     // PA7, PB9
} t_timer_capture_compare_channel;

// Re-using t_timer_capture_compare_channel for PWM and ICU due to common underlying hardware
typedef t_timer_capture_compare_channel t_pwm_channel;
typedef t_timer_capture_compare_channel t_icu_channel;

/**
 * @brief ICU Prescaler values.
 * Specific values depend on timer capabilities.
 */
typedef enum
{
    ICU_PRESCALER_1 = 0,
    ICU_PRESCALER_2,
    ICU_PRESCALER_4,
    ICU_PRESCALER_8,
    ICU_PRESCALER_16,
    ICU_PRESCALER_32,
    ICU_PRESCALER_64,
    ICU_PRESCALER_128,
    ICU_PRESCALER_256
} t_icu_prescaller;

/**
 * @brief ICU Edge Trigger.
 */
typedef enum
{
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH_EDGES
} t_icu_edge;

/**
 * @brief Timer Channel identifiers for general purpose timers.
 */
typedef enum
{
    TIMER_CHANNEL_TIM1 = 0,
    TIMER_CHANNEL_TIM2,
    TIMER_CHANNEL_TIM3,
    TIMER_CHANNEL_TIM4,
    TIMER_CHANNEL_TIM5,
    TIMER_CHANNEL_TIM9,
    TIMER_CHANNEL_TIM10,
    TIMER_CHANNEL_TIM11
} t_timer_channel;

/**
 * @brief ADC Channel identifier.
 * STM32F401RC has a single ADC peripheral (ADC1).
 */
typedef enum
{
    ADC_CHANNEL_MAIN = 0 // Represents the single ADC peripheral
} t_adc_channel;

/**
 * @brief ADC Conversion Mode.
 */
typedef enum
{
    ADC_MODE_SINGLE_CONVERSION = 0,
    ADC_MODE_CONTINUOUS_CONVERSION
} t_adc_mode_t;

/**
 * @brief Time Triggered OS tick time in milliseconds.
 */
typedef enum
{
    TT_TICK_1MS = 0,
    TT_TICK_2MS,
    TT_TICK_5MS,
    TT_TICK_10MS,
    TT_TICK_20MS,
    TT_TICK_50MS,
    TT_TICK_100MS
} t_tick_time;


// --- MCAL API Function Prototypes (from API.json) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel); // Note: Original API name typo retained
void LVD_Enable(void);
void LVD_Disable(void);
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Note: Missing return type in API.json, assumed void

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
void UART_ClearFlag(t_uart_channel uart_channel); // Note: Missing return type in API.json, assumed void

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
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Note: Missing return type in API.json, assumed void

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
void SPI_ClearFlag(t_spi_channel spi_channel); // Note: Missing return type in API.json, assumed void

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Note: Missing return type in API.json, assumed void

// GPIO
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value); // Note: Original API name typo retains t_byte, used tbyte
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
tword ICU_GetFrequency(t_icu_channel icu_channel); // Note: Missing return type in API.json, assumed tword
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Note: Missing return type in API.json, assumed void

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Note: Missing return type in API.json, assumed void

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void); // Note: Missing parameters in API.json, assumed void
tword ADC_Get(void); // Note: Missing parameters in API.json, assumed void; Missing return type, assumed tword
void ADC_ClearFlag(void); // Note: Missing parameters in API.json, assumed void; Missing return type, assumed void

// Internal_EEPROM
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time Triggered OS)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif /* MCAL_H_ */