#include "MCAL.h"

// Register Definitions (Normalized for conceptual mapping, original names retained)
// FLASH
volatile uint32_t * const FLASH_ACR = (volatile uint32_t *)0x40023C00;
volatile uint32_t * const FLASH_KEYR = (volatile uint32_t *)0x40023C04;
volatile uint32_t * const FLASH_OPTKEYR = (volatile uint32_t *)0x40023C08;
volatile uint32_t * const FLASH_SR = (volatile uint32_t *)0x40023C0C;
volatile uint32_t * const FLASH_CR = (volatile uint32_t *)0x40023C10;
volatile uint32_t * const FLASH_OPTCR = (volatile uint32_t *)0x40023C14;

// CRC
volatile uint32_t * const CRC_DR = (volatile uint32_t *)0x40023000;
volatile uint32_t * const CRC_IDR = (volatile uint32_t *)0x40023004;
volatile uint32_t * const CRC_CR = (volatile uint32_t *)0x40023008;

// PWR
volatile uint32_t * const PWR_CR = (volatile uint32_t *)0x40007000;
volatile uint32_t * const PWR_CSR = (volatile uint32_t *)0x40007004;

// RCC
volatile uint32_t * const RCC_CR = (volatile uint32_t *)0x40023800;
volatile uint32_t * const RCC_PLLCFGR = (volatile uint32_t *)0x40023804;
volatile uint32_t * const RCC_CFGR = (volatile uint32_t *)0x40023808;
volatile uint32_t * const RCC_CIR = (volatile uint32_t *)0x4002380C;
volatile uint32_t * const RCC_AHB1RSTR = (volatile uint32_t *)0x40023810;
volatile uint32_t * const RCC_AHB2RSTR = (volatile uint32_t *)0x40023814;
volatile uint32_t * const RCC_APB1RSTR = (volatile uint32_t *)0x40023818;
volatile uint32_t * const RCC_APB2RSTR = (volatile uint32_t *)0x4002381C;
volatile uint32_t * const RCC_AHB1ENR = (volatile uint32_t *)0x40023830;
volatile uint32_t * const RCC_AHB2ENR = (volatile uint32_t *)0x40023834;
volatile uint32_t * const RCC_APB1ENR = (volatile uint32_t *)0x40023838;
volatile uint32_t * const RCC_APB2ENR = (volatile uint32_t *)0x4002383C;
volatile uint32_t * const RCC_AHB1LPENR = (volatile uint32_t *)0x40023850;
volatile uint32_t * const RCC_AHB2LPENR = (volatile uint32_t *)0x40023854;
volatile uint32_t * const RCC_APB1LPENR = (volatile uint32_t *)0x40023858;
volatile uint32_t * const RCC_APB2LPENR = (volatile uint32_t *)0x4002385C;
volatile uint32_t * const RCC_BDCR = (volatile uint32_t *)0x40023870;
volatile uint32_t * const RCC_CSR = (volatile uint32_t *)0x40023874;
volatile uint32_t * const RCC_SSCGR = (volatile uint32_t *)0x40023880;
volatile uint32_t * const RCC_PLLI2SCFGR = (volatile uint32_t *)0x40023884;
volatile uint32_t * const RCC_DCKCFGR = (volatile uint32_t *)0x4002388C;

// SYSCFG
volatile uint32_t * const SYSCFG_MEMRMP = (volatile uint32_t *)0x40013800;
volatile uint32_t * const SYSCFG_PMC = (volatile uint32_t *)0x40013804;
volatile uint32_t * const SYSCFG_EXTICR1 = (volatile uint32_t *)0x40013808;
volatile uint32_t * const SYSCFG_EXTICR2 = (volatile uint32_t *)0x4001380C;
volatile uint32_t * const SYSCFG_EXTICR3 = (volatile uint32_t *)0x40013810;
volatile uint32_t * const SYSCFG_EXTICR4 = (volatile uint32_t *)0x40013814;
volatile uint32_t * const SYSCFG_CMPCR = (volatile uint32_t *)0x40013820;

// GPIOA
volatile uint32_t * const GPIOA_MODER = (volatile uint32_t *)0x40020000;
volatile uint32_t * const GPIOA_OTYPER = (volatile uint32_t *)0x40020004;
volatile uint32_t * const GPIOA_OSPEEDR = (volatile uint32_t *)0x40020008;
volatile uint32_t * const GPIOA_PUPDR = (volatile uint32_t *)0x4002000C;
volatile uint32_t * const GPIOA_IDR = (volatile uint32_t *)0x40020010;
volatile uint32_t * const GPIOA_ODR = (volatile uint32_t *)0x40020014;
volatile uint32_t * const GPIOA_BSRR = (volatile uint32_t *)0x40020018;
volatile uint32_t * const GPIOA_LCKR = (volatile uint32_t *)0x4002001C;
volatile uint32_t * const GPIOA_AFRL = (volatile uint32_t *)0x40020020;
volatile uint32_t * const GPIOA_AFRH = (volatile uint32_t *)0x40020024;

// GPIOB
volatile uint32_t * const GPIOB_MODER = (volatile uint32_t *)0x40020400;
volatile uint32_t * const GPIOB_OTYPER = (volatile uint32_t *)0x40020404;
volatile uint32_t * const GPIOB_OSPEEDR = (volatile uint32_t *)0x40020408;
volatile uint32_t * const GPIOB_PUPDR = (volatile uint32_t *)0x4002040C;
volatile uint32_t * const GPIOB_IDR = (volatile uint32_t *)0x40020410;
volatile uint32_t * const GPIOB_ODR = (volatile uint32_t *)0x40020414;
volatile uint32_t * const GPIOB_BSRR = (volatile uint32_t *)0x40020418;
volatile uint32_t * const GPIOB_LCKR = (volatile uint32_t *)0x4002041C;
volatile uint32_t * const GPIOB_AFRL = (volatile uint32_t *)0x40020420;
volatile uint32_t * const GPIOB_AFRH = (volatile uint32_t *)0x40020424;

// GPIOC
volatile uint32_t * const GPIOC_MODER = (volatile uint32_t *)0x40020800;
volatile uint32_t * const GPIOC_OTYPER = (volatile uint32_t *)0x40020804;
volatile uint32_t * const GPIOC_OSPEEDR = (volatile uint32_t *)0x40020808;
volatile uint32_t * const GPIOC_PUPDR = (volatile uint32_t *)0x4002080C;
volatile uint32_t * const GPIOC_IDR = (volatile uint32_t *)0x40020810;
volatile uint32_t * const GPIOC_ODR = (volatile uint32_t *)0x40020814;
volatile uint32_t * const GPIOC_BSRR = (volatile uint32_t *)0x40020818;
volatile uint32_t * const GPIOC_LCKR = (volatile uint32_t *)0x4002081C;
volatile uint32_t * const GPIOC_AFRL = (volatile uint32_t *)0x40020820;
volatile uint32_t * const GPIOC_AFRH = (volatile uint32_t *)0x40020824;

// GPIOD
volatile uint32_t * const GPIOD_MODER = (volatile uint32_t *)0x40020C00;
volatile uint32_t * const GPIOD_OTYPER = (volatile uint32_t *)0x40020C04;
volatile uint32_t * const GPIOD_OSPEEDR = (volatile uint32_t *)0x40020C08;
volatile uint32_t * const GPIOD_PUPDR = (volatile uint32_t *)0x40020C0C;
volatile uint32_t * const GPIOD_IDR = (volatile uint32_t *)0x40020C10;
volatile uint32_t * const GPIOD_ODR = (volatile uint32_t *)0x40020C14;
volatile uint32_t * const GPIOD_BSRR = (volatile uint32_t *)0x40020C18;
volatile uint32_t * const GPIOD_LCKR = (volatile uint32_t *)0x40020C1C;
volatile uint32_t * const GPIOD_AFRL = (volatile uint32_t *)0x40020C20;
volatile uint32_t * const GPIOD_AFRH = (volatile uint32_t *)0x40020C24;

// GPIOE
volatile uint32_t * const GPIOE_MODER = (volatile uint32_t *)0x40021000;
volatile uint32_t * const GPIOE_OTYPER = (volatile uint32_t *)0x40021004;
volatile uint32_t * const GPIOE_OSPEEDR = (volatile uint32_t *)0x40021008;
volatile uint32_t * const GPIOE_PUPDR = (volatile uint32_t *)0x4002100C;
volatile uint32_t * const GPIOE_IDR = (volatile uint32_t *)0x40021010;
volatile uint32_t * const GPIOE_ODR = (volatile uint32_t *)0x40021014;
volatile uint32_t * const GPIOE_BSRR = (volatile uint32_t *)0x40021018;
volatile uint32_t * const GPIOE_LCKR = (volatile uint32_t *)0x4002101C;
volatile uint32_t * const GPIOE_AFRL = (volatile uint32_t *)0x40021020;
volatile uint32_t * const GPIOE_AFRH = (volatile uint32_t *)0x40021024;

// GPIOH
volatile uint32_t * const GPIOH_MODER = (volatile uint32_t *)0x40021C00;
volatile uint32_t * const GPIOH_OTYPER = (volatile uint32_t *)0x40021C04;
volatile uint32_t * const GPIOH_OSPEEDR = (volatile uint32_t *)0x40021C08;
volatile uint32_t * const GPIOH_PUPDR = (volatile uint32_t *)0x40021C0C;
volatile uint32_t * const GPIOH_IDR = (volatile uint32_t *)0x40021C10;
volatile uint32_t * const GPIOH_ODR = (volatile uint32_t *)0x40021C14;
volatile uint32_t * const GPIOH_BSRR = (volatile uint32_t *)0x40021C18;
volatile uint32_t * const GPIOH_LCKR = (volatile uint32_t *)0x40021C1C;
volatile uint32_t * const GPIOH_AFRL = (volatile uint32_t *)0x40021C20;
volatile uint32_t * const GPIOH_AFRH = (volatile uint32_t *)0x40021C24;

// DMA1
volatile uint32_t * const DMA1_LISR = (volatile uint32_t *)0x40026000;
volatile uint32_t * const DMA1_HISR = (volatile uint32_t *)0x40026004;
volatile uint32_t * const DMA1_LIFCR = (volatile uint32_t *)0x40026008;
volatile uint32_t * const DMA1_HIFCR = (volatile uint32_t *)0x4002600C;
volatile uint32_t * const DMA1_S0CR = (volatile uint32_t *)0x40026010;
volatile uint32_t * const DMA1_S0NDTR = (volatile uint32_t *)0x40026014;
volatile uint32_t * const DMA1_S0PAR = (volatile uint32_t *)0x40026018;
volatile uint32_t * const DMA1_S0M0AR = (volatile uint32_t *)0x4002601C;
volatile uint32_t * const DMA1_S0M1AR = (volatile uint32_t *)0x40026020;
volatile uint32_t * const DMA1_S0FCR = (volatile uint32_t *)0x40026024;
volatile uint32_t * const DMA1_S1CR = (volatile uint32_t *)0x40026028;
volatile uint32_t * const DMA1_S1NDTR = (volatile uint32_t *)0x4002602C;
volatile uint32_t * const DMA1_S1PAR = (volatile uint32_t *)0x40026030;
volatile uint32_t * const DMA1_S1M0AR = (volatile uint32_t *)0x40026034;
volatile uint32_t * const DMA1_S1M1AR = (volatile uint32_t *)0x40026038;
volatile uint32_t * const DMA1_S1FCR = (volatile uint32_t *)0x4002603C;
volatile uint32_t * const DMA1_S2CR = (volatile uint32_t *)0x40026040;
volatile uint32_t * const DMA1_S2NDTR = (volatile uint32_t *)0x40026044;
volatile uint32_t * const DMA1_S2PAR = (volatile uint32_t *)0x40026048;
volatile uint32_t * const DMA1_S2M0AR = (volatile uint32_t *)0x4002604C;
volatile uint32_t * const DMA1_S2M1AR = (volatile uint32_t *)0x40026050;
volatile uint32_t * const DMA1_S2FCR = (volatile uint32_t *)0x40026054;
volatile uint32_t * const DMA1_S3CR = (volatile uint32_t *)0x40026058;
volatile uint32_t * const DMA1_S3NDTR = (volatile uint32_t *)0x4002605C;
volatile uint32_t * const DMA1_S3PAR = (volatile uint32_t *)0x40026060;
volatile uint32_t * const DMA1_S3M0AR = (volatile uint32_t *)0x40026064;
volatile uint32_t * const DMA1_S3M1AR = (volatile uint32_t *)0x40026068;
volatile uint32_t * const DMA1_S3FCR = (volatile uint32_t *)0x4002606C;
volatile uint32_t * const DMA1_S4CR = (volatile uint32_t *)0x40026070;
volatile uint32_t * const DMA1_S4NDTR = (volatile uint32_t *)0x40026074;
volatile uint32_t * const DMA1_S4PAR = (volatile uint32_t *)0x40026078;
volatile uint32_t * const DMA1_S4M0AR = (volatile uint32_t *)0x4002607C;
volatile uint32_t * const DMA1_S4M1AR = (volatile uint32_t *)0x40026080;
volatile uint32_t * const DMA1_S4FCR = (volatile uint32_t *)0x40026084;
volatile uint32_t * const DMA1_S5CR = (volatile uint32_t *)0x40026088;
volatile uint32_t * const DMA1_S5NDTR = (volatile uint32_t *)0x4002608C;
volatile uint32_t * const DMA1_S5PAR = (volatile uint32_t *)0x40026090;
volatile uint32_t * const DMA1_S5M0AR = (volatile uint32_t *)0x40026094;
volatile uint32_t * const DMA1_S5M1AR = (volatile uint32_t *)0x40026098;
volatile uint32_t * const DMA1_S5FCR = (volatile uint32_t *)0x4002609C;
volatile uint32_t * const DMA1_S6CR = (volatile uint32_t *)0x400260A0;
volatile uint32_t * const DMA1_S6NDTR = (volatile uint32_t *)0x400260A4;
volatile uint32_t * const DMA1_S6PAR = (volatile uint32_t *)0x400260A8;
volatile uint32_t * const DMA1_S6M0AR = (volatile uint32_t *)0x400260AC;
volatile uint32_t * const DMA1_S6M1AR = (volatile uint32_t *)0x400260B0;
volatile uint32_t * const DMA1_S6FCR = (volatile uint32_t *)0x400260B4;
volatile uint32_t * const DMA1_S7CR = (volatile uint32_t *)0x400260B8;
volatile uint32_t * const DMA1_S7NDTR = (volatile uint32_t *)0x400260BC;
volatile uint32_t * const DMA1_S7PAR = (volatile uint32_t *)0x400260C0;
volatile uint32_t * const DMA1_S7M0AR = (volatile uint32_t *)0x400260C4;
volatile uint32_t * const DMA1_S7M1AR = (volatile uint32_t *)0x400260C8;
volatile uint32_t * const DMA1_S7FCR = (volatile uint32_t *)0x400260CC;

// DMA2
volatile uint32_t * const DMA2_LISR = (volatile uint32_t *)0x40026400;
volatile uint32_t * const DMA2_HISR = (volatile uint32_t *)0x40026404;
volatile uint32_t * const DMA2_LIFCR = (volatile uint32_t *)0x40026408;
volatile uint32_t * const DMA2_HIFCR = (volatile uint32_t *)0x4002640C;
volatile uint32_t * const DMA2_S0CR = (volatile uint32_t *)0x40026410;
volatile uint32_t * const DMA2_S0NDTR = (volatile uint32_t *)0x40026414;
volatile uint32_t * const DMA2_S0PAR = (volatile uint32_t *)0x40026418;
volatile uint32_t * const DMA2_S0M0AR = (volatile uint32_t *)0x4002641C;
volatile uint32_t * const DMA2_S0M1AR = (volatile uint32_t *)0x40026420;
volatile uint32_t * const DMA2_S0FCR = (volatile uint32_t *)0x40026424;
volatile uint32_t * const DMA2_S1CR = (volatile uint32_t *)0x40026428;
volatile uint32_t * const DMA2_S1NDTR = (volatile uint32_t *)0x4002642C;
volatile uint32_t * const DMA2_S1PAR = (volatile uint32_t *)0x40026430;
volatile uint32_t * const DMA2_S1M0AR = (volatile uint32_t *)0x40026434;
volatile uint32_t * const DMA2_S1M1AR = (volatile uint32_t *)0x40026438;
volatile uint32_t * const DMA2_S1FCR = (volatile uint32_t *)0x4002643C;
volatile uint32_t * const DMA2_S2CR = (volatile uint32_t *)0x40026440;
volatile uint32_t * const DMA2_S2NDTR = (volatile uint32_t *)0x40026444;
volatile uint32_t * const DMA2_S2PAR = (volatile uint32_t *)0x40026448;
volatile uint32_t * const DMA2_S2M0AR = (volatile uint32_t *)0x4002644C;
volatile uint32_t * const DMA2_S2M1AR = (volatile uint32_t *)0x40026450;
volatile uint32_t * const DMA2_S2FCR = (volatile uint32_t *)0x40026454;
volatile uint32_t * const DMA2_S3CR = (volatile uint32_t *)0x40026458;
volatile uint32_t * const DMA2_S3NDTR = (volatile uint32_t *)0x4002645C;
volatile uint32_t * const DMA2_S3PAR = (volatile uint32_t *)0x40026460;
volatile uint32_t * const DMA2_S3M0AR = (volatile uint32_t *)0x40026464;
volatile uint32_t * const DMA2_S3M1AR = (volatile uint32_t *)0x40026468;
volatile uint32_t * const DMA2_S3FCR = (volatile uint32_t *)0x4002646C;
volatile uint32_t * const DMA2_S4CR = (volatile uint32_t *)0x40026470;
volatile uint32_t * const DMA2_S4NDTR = (volatile uint32_t *)0x40026474;
volatile uint32_t * const DMA2_S4PAR = (volatile uint32_t *)0x40026478;
volatile uint32_t * const DMA2_S4M0AR = (volatile uint32_t *)0x4002647C;
volatile uint32_t * const DMA2_S4M1AR = (volatile uint32_t *)0x40026480;
volatile uint32_t * const DMA2_S4FCR = (volatile uint32_t *)0x40026484;
volatile uint32_t * const DMA2_S5CR = (volatile uint32_t *)0x40026488;
volatile uint32_t * const DMA2_S5NDTR = (volatile uint32_t *)0x4002648C;
volatile uint32_t * const DMA2_S5PAR = (volatile uint32_t *)0x40026490;
volatile uint32_t * const DMA2_S5M0AR = (volatile uint32_t *)0x40026494;
volatile uint32_t * const DMA2_S5M1AR = (volatile uint32_t *)0x40026498;
volatile uint32_t * const DMA2_S5FCR = (volatile uint32_t *)0x4002649C;
volatile uint32_t * const DMA2_S6CR = (volatile uint32_t *)0x400264A0;
volatile uint32_t * const DMA2_S6NDTR = (volatile uint32_t *)0x400264A4;
volatile uint32_t * const DMA2_S6PAR = (volatile uint32_t *)0x400264A8;
volatile uint32_t * const DMA2_S6M0AR = (volatile uint32_t *)0x400264AC;
volatile uint32_t * const DMA2_S6M1AR = (volatile uint32_t *)0x400264B0;
volatile uint32_t * const DMA2_S6FCR = (volatile uint32_t *)0x400264B4;
volatile uint32_t * const DMA2_S7CR = (volatile uint32_t *)0x400264B8;
volatile uint32_t * const DMA2_S7NDTR = (volatile uint32_t *)0x400264BC;
volatile uint32_t * const DMA2_S7PAR = (volatile uint32_t *)0x400264C0;
volatile uint32_t * const DMA2_S7M0AR = (volatile uint32_t *)0x400264C4;
volatile uint32_t * const DMA2_S7M1AR = (volatile uint32_t *)0x400264C8;
volatile uint32_t * const DMA2_S7FCR = (volatile uint32_t *)0x400264CC;

// EXTI
volatile uint32_t * const EXTI_IMR = (volatile uint32_t *)0x40013C00;
volatile uint32_t * const EXTI_EMR = (volatile uint32_t *)0x40013C04;
volatile uint32_t * const EXTI_RTSR = (volatile uint32_t *)0x40013C08;
volatile uint32_t * const EXTI_FTSR = (volatile uint32_t *)0x40013C0C;
volatile uint32_t * const EXTI_SWIER = (volatile uint32_t *)0x40013C10;
volatile uint32_t * const EXTI_PR = (volatile uint32_t *)0x40013C14;

// ADC
volatile uint32_t * const ADC_SR = (volatile uint32_t *)0x40012000;
volatile uint32_t * const ADC_CR1 = (volatile uint32_t *)0x40012004;
volatile uint32_t * const ADC_CR2 = (volatile uint32_t *)0x40012008;
volatile uint32_t * const ADC_SMPR1 = (volatile uint32_t *)0x4001200C;
volatile uint32_t * const ADC_SMPR2 = (volatile uint32_t *)0x40012010;
volatile uint32_t * const ADC_JOFR1 = (volatile uint32_t *)0x40012014;
volatile uint32_t * const ADC_JOFR2 = (volatile uint32_t *)0x40012018;
volatile uint32_t * const ADC_JOFR3 = (volatile uint32_t *)0x4001201C;
volatile uint32_t * const ADC_JOFR4 = (volatile uint32_t *)0x40012020;
volatile uint32_t * const ADC_HTR = (volatile uint32_t *)0x40012024;
volatile uint32_t * const ADC_LTR = (volatile uint32_t *)0x40012028;
volatile uint32_t * const ADC_SQR1 = (volatile uint32_t *)0x4001202C;
volatile uint32_t * const ADC_SQR2 = (volatile uint32_t *)0x40012030;
volatile uint32_t * const ADC_SQR3 = (volatile uint32_t *)0x40012034;
volatile uint32_t * const ADC_JSQR = (volatile uint32_t *)0x40012038;
volatile uint32_t * const ADC_JDR1 = (volatile uint32_t *)0x4001203C;
volatile uint32_t * const ADC_JDR2 = (volatile uint32_t *)0x40012040;
volatile uint32_t * const ADC_JDR3 = (volatile uint32_t *)0x40012044;
volatile uint32_t * const ADC_JDR4 = (volatile uint32_t *)0x40012048;
volatile uint32_t * const ADC_DR = (volatile uint32_t *)0x4001204C;
volatile uint32_t * const ADC_CCR = (volatile uint32_t *)0x40012300;

// TIM1
volatile uint32_t * const TIM1_CR1 = (volatile uint32_t *)0x40010000;
volatile uint32_t * const TIM1_CR2 = (volatile uint32_t *)0x40010004;
volatile uint32_t * const TIM1_SMCR = (volatile uint32_t *)0x40010008;
volatile uint32_t * const TIM1_DIER = (volatile uint32_t *)0x4001000C;
volatile uint32_t * const TIM1_SR = (volatile uint32_t *)0x40010010;
volatile uint32_t * const TIM1_EGR = (volatile uint32_t *)0x40010014;
volatile uint32_t * const TIM1_CCMR1 = (volatile uint32_t *)0x40010018;
volatile uint32_t * const TIM1_CCMR2 = (volatile uint32_t *)0x4001001C;
volatile uint32_t * const TIM1_CCER = (volatile uint32_t *)0x40010020;
volatile uint32_t * const TIM1_CNT = (volatile uint32_t *)0x40010024;
volatile uint32_t * const TIM1_PSC = (volatile uint32_t *)0x40010028;
volatile uint32_t * const TIM1_ARR = (volatile uint32_t *)0x4001002C;
volatile uint32_t * const TIM1_RCR = (volatile uint32_t *)0x40010030;
volatile uint32_t * const TIM1_CCR1 = (volatile uint32_t *)0x40010034;
volatile uint32_t * const TIM1_CCR2 = (volatile uint32_t *)0x40010038;
volatile uint32_t * const TIM1_CCR3 = (volatile uint32_t *)0x4001003C;
volatile uint32_t * const TIM1_CCR4 = (volatile uint32_t *)0x40010040;
volatile uint32_t * const TIM1_BDTR = (volatile uint32_t *)0x40010044;
volatile uint32_t * const TIM1_DCR = (volatile uint32_t *)0x40010048;
volatile uint32_t * const TIM1_DMAR = (volatile uint32_t *)0x4001004C;

// TIM2
volatile uint32_t * const TIM2_CR1 = (volatile uint32_t *)0x40000000;
volatile uint32_t * const TIM2_CR2 = (volatile uint32_t *)0x40000004;
volatile uint32_t * const TIM2_SMCR = (volatile uint32_t *)0x40000008;
volatile uint32_t * const TIM2_DIER = (volatile uint32_t *)0x4000000C;
volatile uint32_t * const TIM2_SR = (volatile uint32_t *)0x40000010;
volatile uint32_t * const TIM2_EGR = (volatile uint32_t *)0x40000014;
volatile uint32_t * const TIM2_CCMR1 = (volatile uint32_t *)0x40000018;
volatile uint32_t * const TIM2_CCMR2 = (volatile uint32_t *)0x4000001C;
volatile uint32_t * const TIM2_CCER = (volatile uint32_t *)0x40000020;
volatile uint32_t * const TIM2_CNT = (volatile uint32_t *)0x40000024;
volatile uint32_t * const TIM2_PSC = (volatile uint32_t *)0x40000028;
volatile uint32_t * const TIM2_ARR = (volatile uint32_t *)0x4000002C;
volatile uint32_t * const TIM2_CCR1 = (volatile uint32_t *)0x40000034;
volatile uint32_t * const TIM2_CCR2 = (volatile uint32_t *)0x40000038;
volatile uint32_t * const TIM2_CCR3 = (volatile uint32_t *)0x4000003C;
volatile uint32_t * const TIM2_CCR4 = (volatile uint32_t *)0x40000040;
volatile uint32_t * const TIM2_DCR = (volatile uint32_t *)0x40000048;
volatile uint32_t * const TIM2_DMAR = (volatile uint32_t *)0x4000004C;
volatile uint32_t * const TIM2_OR = (volatile uint32_t *)0x40000050;

// TIM3
volatile uint32_t * const TIM3_CR1 = (volatile uint32_t *)0x40000400;
volatile uint32_t * const TIM3_CR2 = (volatile uint32_t *)0x40000404;
volatile uint32_t * const TIM3_SMCR = (volatile uint32_t *)0x40000408;
volatile uint32_t * const TIM3_DIER = (volatile uint32_t *)0x4000040C;
volatile uint32_t * const TIM3_SR = (volatile uint32_t *)0x40000410;
volatile uint32_t * const TIM3_EGR = (volatile uint32_t *)0x40000414;
volatile uint32_t * const TIM3_CCMR1 = (volatile uint32_t *)0x40000418;
volatile uint32_t * const TIM3_CCMR2 = (volatile uint32_t *)0x4000041C;
volatile uint32_t * const TIM3_CCER = (volatile uint32_t *)0x40000420;
volatile uint32_t * const TIM3_CNT = (volatile uint32_t *)0x40000424;
volatile uint32_t * const TIM3_PSC = (volatile uint32_t *)0x40000428;
volatile uint32_t * const TIM3_ARR = (volatile uint32_t *)0x4000042C;
volatile uint32_t * const TIM3_CCR1 = (volatile uint32_t *)0x40000434;
volatile uint32_t * const TIM3_CCR2 = (volatile uint32_t *)0x40000438;
volatile uint32_t * const TIM3_CCR3 = (volatile uint32_t *)0x4000043C;
volatile uint32_t * const TIM3_CCR4 = (volatile uint32_t *)0x40000440;
volatile uint32_t * const TIM3_DCR = (volatile uint32_t *)0x40000448;
volatile uint32_t * const TIM3_DMAR = (volatile uint32_t *)0x4000044C;

// TIM4
volatile uint32_t * const TIM4_CR1 = (volatile uint32_t *)0x40000800;
volatile uint32_t * const TIM4_CR2 = (volatile uint32_t *)0x40000804;
volatile uint32_t * const TIM4_SMCR = (volatile uint32_t *)0x40000808;
volatile uint32_t * const TIM4_DIER = (volatile uint32_t *)0x4000080C;
volatile uint32_t * const TIM4_SR = (volatile uint32_t *)0x40000810;
volatile uint32_t * const TIM4_EGR = (volatile uint32_t *)0x40000814;
volatile uint32_t * const TIM4_CCMR1 = (volatile uint32_t *)0x40000818;
volatile uint32_t * const TIM4_CCMR2 = (volatile uint32_t *)0x4000081C;
volatile uint32_t * const TIM4_CCER = (volatile uint32_t *)0x40000820;
volatile uint32_t * const TIM4_CNT = (volatile uint32_t *)0x40000824;
volatile uint32_t * const TIM4_PSC = (volatile uint32_t *)0x40000828;
volatile uint32_t * const TIM4_ARR = (volatile uint32_t *)0x4000082C;
volatile uint32_t * const TIM4_CCR1 = (volatile uint32_t *)0x40000834;
volatile uint32_t * const TIM4_CCR2 = (volatile uint32_t *)0x40000838;
volatile uint32_t * const TIM4_CCR3 = (volatile uint32_t *)0x4000083C;
volatile uint32_t * const TIM4_CCR4 = (volatile uint32_t *)0x40000840;
volatile uint32_t * const TIM4_DCR = (volatile uint32_t *)0x40000848;
volatile uint32_t * const TIM4_DMAR = (volatile uint32_t *)0x4000084C;

// TIM5
volatile uint32_t * const TIM5_CR1 = (volatile uint32_t *)0x40000C00;
volatile uint32_t * const TIM5_CR2 = (volatile uint32_t *)0x40000C04;
volatile uint32_t * const TIM5_SMCR = (volatile uint32_t *)0x40000C08;
volatile uint32_t * const TIM5_DIER = (volatile uint32_t *)0x40000C0C;
volatile uint32_t * const TIM5_SR = (volatile uint32_t *)0x40000C10;
volatile uint32_t * const TIM5_EGR = (volatile uint32_t *)0x40000C14;
volatile uint32_t * const TIM5_CCMR1 = (volatile uint32_t *)0x40000C18;
volatile uint32_t * const TIM5_CCMR2 = (volatile uint32_t *)0x40000C1C;
volatile uint32_t * const TIM5_CCER = (volatile uint32_t *)0x40000C20;
volatile uint32_t * const TIM5_CNT = (volatile uint32_t *)0x40000C24;
volatile uint32_t * const TIM5_PSC = (volatile uint32_t *)0x40000C28;
volatile uint32_t * const TIM5_ARR = (volatile uint32_t *)0x40000C2C;
volatile uint32_t * const TIM5_CCR1 = (volatile uint32_t *)0x40000C34;
volatile uint32_t * const TIM5_CCR2 = (volatile uint32_t *)0x40000C38;
volatile uint32_t * const TIM5_CCR3 = (volatile uint32_t *)0x40000C3C;
volatile uint32_t * const TIM5_CCR4 = (volatile uint32_t *)0x40000C40;
volatile uint32_t * const TIM5_DCR = (volatile uint32_t *)0x40000C48;
volatile uint32_t * const TIM5_DMAR = (volatile uint32_t *)0x40000C4C;
volatile uint32_t * const TIM5_OR = (volatile uint32_t *)0x40000C50;

// TIM9
volatile uint32_t * const TIM9_CR1 = (volatile uint32_t *)0x40014000;
volatile uint32_t * const TIM9_SMCR = (volatile uint32_t *)0x40014008;
volatile uint32_t * const TIM9_DIER = (volatile uint32_t *)0x4001400C;
volatile uint32_t * const TIM9_SR = (volatile uint32_t *)0x40014010;
volatile uint32_t * const TIM9_EGR = (volatile uint32_t *)0x40014014;
volatile uint32_t * const TIM9_CCMR1 = (volatile uint32_t *)0x40014018;
volatile uint32_t * const TIM9_CCER = (volatile uint32_t *)0x40014020;
volatile uint32_t * const TIM9_CNT = (volatile uint32_t *)0x40014024;
volatile uint32_t * const TIM9_PSC = (volatile uint32_t *)0x40014028;
volatile uint32_t * const TIM9_ARR = (volatile uint32_t *)0x4001402C;
volatile uint32_t * const TIM9_CCR1 = (volatile uint32_t *)0x40014034;
volatile uint32_t * const TIM9_CCR2 = (volatile uint32_t *)0x40014038;

// TIM10
volatile uint32_t * const TIM10_CR1 = (volatile uint32_t *)0x40014400;
volatile uint32_t * const TIM10_DIER = (volatile uint32_t *)0x4001440C;
volatile uint32_t * const TIM10_SR = (volatile uint32_t *)0x40014410;
volatile uint32_t * const TIM10_EGR = (volatile uint32_t *)0x40014414;
volatile uint32_t * const TIM10_CCMR1 = (volatile uint32_t *)0x40014418;
volatile uint32_t * const TIM10_CCER = (volatile uint32_t *)0x40014420;
volatile uint32_t * const TIM10_CNT = (volatile uint32_t *)0x40014424;
volatile uint32_t * const TIM10_PSC = (volatile uint32_t *)0x40014428;
volatile uint32_t * const TIM10_ARR = (volatile uint32_t *)0x4001442C;
volatile uint32_t * const TIM10_CCR1 = (volatile uint32_t *)0x40014434;

// TIM11
volatile uint32_t * const TIM11_CR1 = (volatile uint32_t *)0x40014800;
volatile uint32_t * const TIM11_DIER = (volatile uint32_t *)0x4001480C;
volatile uint32_t * const TIM11_SR = (volatile uint32_t *)0x40014810;
volatile uint32_t * const TIM11_EGR = (volatile uint32_t *)0x40014814;
volatile uint32_t * const TIM11_CCMR1 = (volatile uint32_t *)0x40014818;
volatile uint32_t * const TIM11_CCER = (volatile uint32_t *)0x40014820;
volatile uint32_t * const TIM11_CNT = (volatile uint32_t *)0x40014824;
volatile uint32_t * const TIM11_PSC = (volatile uint32_t *)0x40014828;
volatile uint32_t * const TIM11_ARR = (volatile uint32_t *)0x4001482C;
volatile uint32_t * const TIM11_CCR1 = (volatile uint32_t *)0x40014834;

// USART1
volatile uint32_t * const USART1_SR = (volatile uint32_t *)0x40011000;
volatile uint32_t * const USART1_DR = (volatile uint32_t *)0x40011004;
volatile uint32_t * const USART1_BRR = (volatile uint32_t *)0x40011008;
volatile uint32_t * const USART1_CR1 = (volatile uint32_t *)0x4001100C;
volatile uint32_t * const USART1_CR2 = (volatile uint32_t *)0x40011010;
volatile uint32_t * const USART1_CR3 = (volatile uint32_t *)0x40011014;
volatile uint32_t * const USART1_GTPR = (volatile uint32_t *)0x40011018;

// USART2
volatile uint32_t * const USART2_SR = (volatile uint32_t *)0x40004400;
volatile uint32_t * const USART2_DR = (volatile uint32_t *)0x40004404;
volatile uint32_t * const USART2_BRR = (volatile uint32_t *)0x40004408;
volatile uint32_t * const USART2_CR1 = (volatile uint32_t *)0x4000440C;
volatile uint32_t * const USART2_CR2 = (volatile uint32_t *)0x40004410;
volatile uint32_t * const USART2_CR3 = (volatile uint32_t *)0x40004414;
volatile uint32_t * const USART2_GTPR = (volatile uint32_t *)0x40004418;

// USART6
volatile uint32_t * const USART6_SR = (volatile uint32_t *)0x40011400;
volatile uint32_t * const USART6_DR = (volatile uint32_t *)0x40011404;
volatile uint32_t * const USART6_BRR = (volatile uint32_t *)0x40011408;
volatile uint32_t * const USART6_CR1 = (volatile uint32_t *)0x4001140C;
volatile uint32_t * const USART6_CR2 = (volatile uint32_t *)0x40011410;
volatile uint32_t * const USART6_CR3 = (volatile uint32_t *)0x40011414;
volatile uint32_t * const USART6_GTPR = (volatile uint32_t *)0x40011418;

// I2C1
volatile uint32_t * const I2C1_CR1 = (volatile uint32_t *)0x40005400;
volatile uint32_t * const I2C1_CR2 = (volatile uint32_t *)0x40005404;
volatile uint32_t * const I2C1_OAR1 = (volatile uint32_t *)0x40005408;
volatile uint32_t * const I2C1_OAR2 = (volatile uint32_t *)0x4000540C;
volatile uint32_t * const I2C1_DR = (volatile uint32_t *)0x40005410;
volatile uint32_t * const I2C1_SR1 = (volatile uint32_t *)0x40005414;
volatile uint32_t * const I2C1_SR2 = (volatile uint32_t *)0x40005418;
volatile uint32_t * const I2C1_CCR = (volatile uint32_t *)0x4000541C;
volatile uint32_t * const I2C1_TRISE = (volatile uint32_t *)0x40005420;
volatile uint32_t * const I2C1_FLTR = (volatile uint32_t *)0x40005424;

// I2C2
volatile uint32_t * const I2C2_CR1 = (volatile uint32_t *)0x40005800;
volatile uint32_t * const I2C2_CR2 = (volatile uint32_t *)0x40005804;
volatile uint32_t * const I2C2_OAR1 = (volatile uint32_t *)0x40005808;
volatile uint32_t * const I2C2_OAR2 = (volatile uint32_t *)0x4000580C;
volatile uint32_t * const I2C2_DR = (volatile uint32_t *)0x40005810;
volatile uint32_t * const I2C2_SR1 = (volatile uint32_t *)0x40005814;
volatile uint32_t * const I2C2_SR2 = (volatile uint32_t *)0x40005818;
volatile uint32_t * const I2C2_CCR = (volatile uint32_t *)0x4000581C;
volatile uint32_t * const I2C2_TRISE = (volatile uint32_t *)0x40005820;
volatile uint32_t * const I2C2_FLTR = (volatile uint32_t *)0x40005824;

// I2C3
volatile uint32_t * const I2C3_CR1 = (volatile uint32_t *)0x40005C00;
volatile uint32_t * const I2C3_CR2 = (volatile uint32_t *)0x40005C04;
volatile uint32_t * const I2C3_OAR1 = (volatile uint32_t *)0x40005C08;
volatile uint32_t * const I2C3_OAR2 = (volatile uint32_t *)0x40005C0C;
volatile uint32_t * const I2C3_DR = (volatile uint32_t *)0x40005C10;
volatile uint32_t * const I2C3_SR1 = (volatile uint32_t *)0x40005C14;
volatile uint32_t * const I2C3_SR2 = (volatile uint32_t *)0x40005C18;
volatile uint32_t * const I2C3_CCR = (volatile uint32_t *)0x40005C1C;
volatile uint32_t * const I2C3_TRISE = (volatile uint32_t *)0x40005C20;
volatile uint32_t * const I2C3_FLTR = (volatile uint32_t *)0x40005C24;

// SPI1
volatile uint32_t * const SPI1_CR1 = (volatile uint32_t *)0x40013000;
volatile uint32_t * const SPI1_CR2 = (volatile uint32_t *)0x40013004;
volatile uint32_t * const SPI1_SR = (volatile uint32_t *)0x40013008;
volatile uint32_t * const SPI1_DR = (volatile uint32_t *)0x4001300C;
volatile uint32_t * const SPI1_CRCPR = (volatile uint32_t *)0x40013010;
volatile uint32_t * const SPI1_RXCRCR = (volatile uint32_t *)0x40013014;
volatile uint32_t * const SPI1_TXCRCR = (volatile uint32_t *)0x40013018;
volatile uint32_t * const SPI1_I2SCFGR = (volatile uint32_t *)0x4001301C;
volatile uint32_t * const SPI1_I2SPR = (volatile uint32_t *)0x40013020;

// SPI2
volatile uint32_t * const SPI2_CR1 = (volatile uint32_t *)0x40003800;
volatile uint32_t * const SPI2_CR2 = (volatile uint32_t *)0x40003804;
volatile uint32_t * const SPI2_SR = (volatile uint32_t *)0x40003808;
volatile uint32_t * const SPI2_DR = (volatile uint32_t *)0x4000380C;
volatile uint32_t * const SPI2_CRCPR = (volatile uint32_t *)0x40003810;
volatile uint32_t * const SPI2_RXCRCR = (volatile uint32_t *)0x40003814;
volatile uint32_t * const SPI2_TXCRCR = (volatile uint32_t *)0x40003818;
volatile uint32_t * const SPI2_I2SCFGR = (volatile uint32_t *)0x4000381C;
volatile uint32_t * const SPI2_I2SPR = (volatile uint32_t *)0x40003820;

// SPI3
volatile uint32_t * const SPI3_CR1 = (volatile uint32_t *)0x40003C00;
volatile uint32_t * const SPI3_CR2 = (volatile uint32_t *)0x40003C04;
volatile uint32_t * const SPI3_SR = (volatile uint32_t *)0x40003C08;
volatile uint32_t * const SPI3_DR = (volatile uint32_t *)0x40003C0C;
volatile uint32_t * const SPI3_CRCPR = (volatile uint32_t *)0x40003C10;
volatile uint32_t * const SPI3_RXCRCR = (volatile uint32_t *)0x40003C14;
volatile uint32_t * const SPI3_TXCRCR = (volatile uint32_t *)0x40003C18;
volatile uint32_t * const SPI3_I2SCFGR = (volatile uint32_t *)0x40003C1C;
volatile uint32_t * const SPI3_I2SPR = (volatile uint32_t *)0x40003C20;

// Array of GPIO port base addresses and MODER, ODR, IDR, PUPDR, OSPEEDR, OTYPER registers
static volatile uint32_t * const GPIO_MODER_ARRAY[] = {GPIOA_MODER, GPIOB_MODER, GPIOC_MODER, GPIOD_MODER, GPIOE_MODER, GPIOH_MODER};
static volatile uint32_t * const GPIO_ODR_ARRAY[] = {GPIOA_ODR, GPIOB_ODR, GPIOC_ODR, GPIOD_ODR, GPIOE_ODR, GPIOH_ODR};
static volatile uint32_t * const GPIO_IDR_ARRAY[] = {GPIOA_IDR, GPIOB_IDR, GPIOC_IDR, GPIOD_IDR, GPIOE_IDR, GPIOH_IDR};
static volatile uint32_t * const GPIO_PUPDR_ARRAY[] = {GPIOA_PUPDR, GPIOB_PUPDR, GPIOC_PUPDR, GPIOD_PUPDR, GPIOE_PUPDR, GPIOH_PUPDR};
static volatile uint32_t * const GPIO_OSPEEDR_ARRAY[] = {GPIOA_OSPEEDR, GPIOB_OSPEEDR, GPIOC_OSPEEDR, GPIOD_OSPEEDR, GPIOE_OSPEEDR, GPIOH_OSPEEDR};
static volatile uint32_t * const GPIO_OTYPER_ARRAY[] = {GPIOA_OTYPER, GPIOB_OTYPER, GPIOC_OTYPER, GPIOD_OTYPER, GPIOE_OTYPER, GPIOH_OTYPER};
static volatile uint32_t * const GPIO_BSRR_ARRAY[] = {GPIOA_BSRR, GPIOB_BSRR, GPIOC_BSRR, GPIOD_BSRR, GPIOE_BSRR, GPIOH_BSRR};
static const uint8_t GPIO_MAX_PINS[] = {16, 16, 16, 16, 16, 2}; // Max pins for each port A-H

// Forward declaration for internal use if needed (e.g. for clock enables)
static void MCAL_RCC_EnableClock(tbyte port_idx, uint32_t peripheral_bit_mask);

// --- MCU CONFIG API ---

void WDT_Reset(void) {
    // Placeholder: Insert actual WDT reset instruction here.
    // For HOLTEK HT46R24: ClrWdt();
    // For STM32, typically write to the IWDG_RLR or WWDG_CR register to reset counter.
    // Since no specific WDT registers are provided in register_json, this is a functional placeholder.
    __NOP(); // No Operation instruction as a minimal placeholder.
}

void Go_to_sleep_mode(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder: Insert actual sleep mode instruction here.
    // For HOLTEK HT46R24: _halt();
    // For STM32, typically involves WFI (Wait For Interrupt) instruction.
    __WFI(); // ARM Cortex-M WFI instruction
}

void Global_interrupt_Enable(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder: Insert actual global interrupt enable instruction.
    // For ARM Cortex-M, this is __enable_irq().
    __enable_irq();
}

void Global_interrupt_Disable(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder: Insert actual global interrupt disable instruction.
    // For ARM Cortex-M, this is __disable_irq().
    __disable_irq();
}


void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    // Set all GPIO pins to 0 and verify with while loop
    for (t_port port = GPIO_PORTA; port <= GPIO_PORTH; port++) {
        for (t_pin pin = GPIO_PIN_0; pin < GPIO_MAX_PINS[port]; pin++) {
            // Set value to 0 before setting direction as per GPIO_rules
            *GPIO_ODR_ARRAY[port] &= ~(1U << pin);
            // Verify value
            while ((*GPIO_ODR_ARRAY[port] & (1U << pin)) != 0) {
                // Wait until pin is low (or handle error)
            }
        }
    }

    // Set all GPIO pins direction to input and verify with while loop
    // All input pins have pull-up resistors and wakeup feature enabled (if available)
    for (t_port port = GPIO_PORTA; port <= GPIO_PORTH; port++) {
        // Enable GPIO Port Clock (inferred for STM32 AHB1 bus)
        // RCC_AHB1ENR bits: GPIOA=0, GPIOB=1, GPIOC=2, GPIOD=3, GPIOE=4, GPIOH=7
        *RCC_AHB1ENR |= (1U << port); // For A-E, this is direct mapping. For H, need specific bit.
        if (port == GPIO_PORTH) {
            *RCC_AHB1ENR |= (1U << 7); // Explicitly enable GPIOH clock
        }

        for (t_pin pin = GPIO_PIN_0; pin < GPIO_MAX_PINS[port]; pin++) {
            // Set direction to Input (MODER bits 00)
            *GPIO_MODER_ARRAY[port] &= ~(3U << (pin * 2)); // Clear mode bits
            // Set pull-up resistors (PUPDR bits 01 for pull-up)
            *GPIO_PUPDR_ARRAY[port] |= (1U << (pin * 2));

            // Verify direction (if possible by reading back, assuming 00 for input)
            while ((*GPIO_MODER_ARRAY[port] & (3U << (pin * 2))) != 0U) {
                // Wait until input mode is set
            }

            // Wakeup feature for input pins (if available) - requires EXTI configuration, not directly GPIO
            // This is handled by External_INT API later
        }
    }

    // Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();

    // Disable peripherals by clearing their enable bits in RCC registers
    // Inferred peripheral clock enable bits for STM32F401RC (0 for disable)
    *RCC_APB2ENR &= ~( (1U << 0)   // TIM1EN
                     | (1U << 4)   // USART1EN
                     | (1U << 5)   // USART6EN
                     | (1U << 8)   // ADC1EN
                     | (1U << 12)  // SPI1EN
                     | (1U << 14)  // SYSCFGEN (disable SYSCFG too)
                     | (1U << 16)  // TIM9EN
                     | (1U << 17)  // TIM10EN
                     | (1U << 18)  // TIM11EN
                     );

    *RCC_APB1ENR &= ~( (1U << 0)   // TIM2EN
                     | (1U << 1)   // TIM3EN
                     | (1U << 2)   // TIM4EN
                     | (1U << 3)   // TIM5EN
                     | (1U << 14)  // SPI2EN
                     | (1U << 15)  // SPI3EN
                     | (1U << 17)  // USART2EN
                     | (1U << 21)  // I2C1EN
                     | (1U << 22)  // I2C2EN
                     | (1U << 23)  // I2C3EN
                     | (1U << 28)  // PWRAPBEN (Power interface clock enable)
                     );

    // Also disable individual peripheral enable bits in their control registers if they exist
    *ADC_CR2 &= ~(1U << 0); // ADON bit for ADC1
    *PWR_CR &= ~(1U << 8); // DBP bit (Disable backup domain write protection - not really disable feature)

    // Clear and disable EXTI entirely for all lines
    *EXTI_IMR = 0x00000000; // Disable all interrupts
    *EXTI_EMR = 0x00000000; // Disable all events
    *EXTI_RTSR = 0x00000000; // Disable all rising triggers
    *EXTI_FTSR = 0x00000000; // Disable all falling triggers
    *EXTI_PR = 0xFFFFFFFF;   // Clear all pending flags

    // Clear CRC control register
    *CRC_CR = 0x00000000;

    // Flash control - clear CR. Bits for programming/erasing should be off.
    *FLASH_CR = 0x00000000;


    // Enable WDT (Watchdog Timer)
    // No specific WDT enable register found in register_json. Placeholder.
    // Typically involves IWDG_KR (Key Register) or WWDG_CFR.
    // __IO uint32_t IWDG_KEYR = 0x40003000; // Assuming IWDG for example
    // *IWDG_KEYR = 0xCCCC; // Start IWDG (if it existed)
    // *IWDG_KEYR = 0x5555; // Enable access to PR, RLR

    // Clear WDT timer (already called WDT_Reset() at function start, call again as per rule)
    WDT_Reset();

    // Set WDT period >= 8 msec
    // No specific WDT period register found in register_json. Placeholder.
    // If IWDG, configure IWDG_PR (prescaler) and IWDG_RLR (reload register).
    // This would require calculation based on LSI clock freq and desired period.

    // Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // Related to PWR_CR (PVD configuration)
    // Set PVDE bit (Programmable voltage detector enable) and PLS[2:0] (PVD Level Selection)
    // 0x40007000: PWR_CR, bits PLS[2:0] (PVD level selection) and PVDE (PVD enable)
    // Values inferred from common STM32 reference:
    // PLS: 000=2.2V, 001=2.3V, ..., 111=2.9V (for example levels)
    // Since levels are 0.5V, 1V etc., this would require mapping to actual MCU PVD levels.
    // Assuming specific levels are mapped to t_sys_volt:
    uint32_t pls_val = 0; // Default to lowest
    if (volt == Vsource_3V) {
        // For 3V system, set LVR to 2V (e.g. PLS to 000 for 2.2V on STM32)
        pls_val = 0U; // Assuming PLS=000 maps to ~2V level for 3V system.
    } else if (volt == Vsource_5V) {
        // For 5V system, set LVR to 3.5V (e.g. PLS to 101 for 2.7V, or higher if available)
        // STM32F401RC PVD levels are typically 2.2V to 2.9V. 3.5V is not directly achievable
        // without external circuitry or a different MCU feature. Using closest available.
        pls_val = 5U; // Assuming PLS=101 maps to ~2.7V level.
    }
    *PWR_CR = (*PWR_CR & ~(7U << 5)) | (pls_val << 5); // Clear PLS bits and set new value
    // *PWR_CR |= (1U << 4); // PVDE: Enable PVD

    // Enable LVR (Low Voltage Reset)
    // PVD is programmable, but POR/PDR (Power-On Reset/Power-Down Reset) is usually always enabled.
    // The rule implies PVD itself as LVR.
    *PWR_CR |= (1U << 4); // PVDE bit to enable PVD (Programmable Voltage Detector)
    // If a dedicated LVR enable bit for internal reset (not interrupt) is needed, it's not explicit in JSON.

    // Clear WDT again
    WDT_Reset();
}


// --- LVD API ---

void LVD_Init(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // LVD initialization might involve enabling the Power interface clock
    // RCC_APB1ENR bit 28 (PWREN)
    *RCC_APB1ENR |= (1U << 28); // Inferred: Enable Power Interface Clock
    // Placeholder for LVD specific initializations (e.g., enable PVD comparator, select threshold)
    // This is typically done through PWR_CR and PWR_CSR registers.
    // PVDE bit (bit 4) in PWR_CR enables PVD. PLS[2:0] (bits 5:7) selects level.
    *PWR_CR |= (1U << 4); // Enable PVD (Programmable Voltage Detector)
}

void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Configure PVD threshold level in PWR_CR, PLS[2:0] bits (5:7)
    // Note: The mapping of t_lvd_thrthresholdLevel to actual PLS bits is inferred.
    // STM32F401RC PVD levels are typically limited (e.g., 2.2V-2.9V).
    uint32_t pls_value = 0;
    switch(lvd_thresholdLevel) {
        case LVD_THRESHOLD_0_5V: pls_value = 0; break; // Placeholder, assuming lowest valid PLS
        case LVD_THRESHOLD_1V:   pls_value = 0; break; // Placeholder
        case LVD_THRESHOLD_1_5V: pls_value = 0; break; // Placeholder
        case LVD_THRESHOLD_2V:   pls_value = 0; break; // Closest for STM32 PVD: 2.2V (PLS=000)
        case LVD_THRESHOLD_2_5V: pls_value = 4; break; // Closest for STM32 PVD: 2.5V (PLS=100)
        case LVD_THRESHOLD_3V:   pls_value = 7; break; // Closest for STM32 PVD: 2.9V (PLS=111)
        case LVD_THRESHOLD_3_5V: pls_value = 7; break; // Using max available, actual 3.5V not directly supported
        case LVD_THRESHOLD_4V:   pls_value = 7; break; // Using max available
        case LVD_THRESHOLD_4_5V: pls_value = 7; break; // Using max available
        case LVD_THRESHOLD_5V:   pls_value = 7; break; // Using max available
        default: break;
    }
    *PWR_CR = (*PWR_CR & ~(7U << 5)) | (pls_value << 5); // Clear PLS bits and set new value
}

void LVD_Enable(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable Power interface clock (if not already)
    *RCC_APB1ENR |= (1U << 28); // Inferred: Enable Power Interface Clock
    // Enable PVD
    *PWR_CR |= (1U << 4); // PVDE bit
}

void LVD_Disable(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Disable PVD
    *PWR_CR &= ~(1U << 4); // PVDE bit
}

void LVD_ClearFlag(t_lvd_channel lvd_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    (void)lvd_channel; // Parameter not directly used as no specific LVD channel flag register
    // In STM32, PVD output is typically read from PWR_CSR PVDO bit.
    // There isn't a direct "clear flag" for PVD level itself, it's a real-time status.
    // If PVD interrupt is enabled (via EXTI), the EXTI pending bit would be cleared.
    // Placeholder as no clearable LVD flag explicitly found in register_json for LVD
    // independent of EXTI.
}

// --- UART API ---

static volatile uint32_t *const USART_SR_ARRAY[] = {USART1_SR, USART2_SR, USART6_SR};
static volatile uint32_t *const USART_DR_ARRAY[] = {USART1_DR, USART2_DR, USART6_DR};
static volatile uint32_t *const USART_BRR_ARRAY[] = {USART1_BRR, USART2_BRR, USART6_BRR};
static volatile uint32_t *const USART_CR1_ARRAY[] = {USART1_CR1, USART2_CR1, USART6_CR1};
static volatile uint32_t *const USART_CR2_ARRAY[] = {USART1_CR2, USART2_CR2, USART6_CR2};
static volatile uint32_t *const USART_CR3_ARRAY[] = {USART1_CR3, USART2_CR3, USART6_CR3};
// static volatile uint32_t *const USART_GTPR_ARRAY[] = {USART1_GTPR, USART2_GTPR, USART6_GTPR}; // Not used by init params

void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    // 1. Enable UART peripheral clock
    // USART1/6 on APB2, USART2 on APB1
    switch (uart_channel) {
        case UART_CHANNEL_1:
            *RCC_APB2ENR |= (1U << 4); // Inferred: USART1EN bit 4
            break;
        case UART_CHANNEL_2:
            *RCC_APB1ENR |= (1U << 17); // Inferred: USART2EN bit 17
            break;
        case UART_CHANNEL_6:
            *RCC_APB2ENR |= (1U << 5); // Inferred: USART6EN bit 5
            break;
        default:
            return; // Invalid channel
    }

    // 2. Configure Baud Rate (BRR register)
    // Assuming a system clock (e.g., 16MHz for APB1, 32MHz for APB2) and typical oversampling.
    // Calculation depends on actual clock setup. For simplicity, just set a placeholder value.
    uint16_t baud_val = 0;
    switch (uart_baud_rate) {
        case UART_BAUD_9600:
            // Placeholder value, e.g., for 16MHz APB1, 9600 baud, BRR = 16000000 / 9600 = 1666.66
            // For oversampling by 16, DIV_MANTISSA = 1666, DIV_FRACTION = 16 * 0.666 = 10.65 -> 11
            baud_val = (1666 << 4) | 11; // Example for 9600
            break;
        case UART_BAUD_19200:
            baud_val = (833 << 4) | 5; // Example for 19200
            break;
        case UART_BAUD_115200:
            baud_val = (138 << 4) | 9; // Example for 115200
            break;
        default:
            break;
    }
    *USART_BRR_ARRAY[uart_channel] = baud_val;

    // 3. Configure Data Length (CR1, M bit)
    // M bit: 0 = 8 data bits, 1 = 9 data bits
    if (uart_data_length == UART_DATA_9_BIT) {
        *USART_CR1_ARRAY[uart_channel] |= (1U << 12); // M bit
    } else {
        *USART_CR1_ARRAY[uart_channel] &= ~(1U << 12); // M bit
    }

    // 4. Configure Stop Bits (CR2, STOP[1:0] bits)
    // 00 = 1 stop bit, 01 = 0.5 stop bits, 10 = 2 stop bits, 11 = 1.5 stop bits
    uint32_t stop_bits_val = 0;
    switch (uart_stop_bit) {
        case UART_STOP_0_5_BIT: stop_bits_val = 1U; break;
        case UART_STOP_2_BIT:   stop_bits_val = 2U; break;
        case UART_STOP_1_5_BIT: stop_bits_val = 3U; break;
        case UART_STOP_1_BIT:   // Fallthrough for 0
        default: break;
    }
    *USART_CR2_ARRAY[uart_channel] = (*USART_CR2_ARRAY[uart_channel] & ~(3U << 12)) | (stop_bits_val << 12);

    // 5. Configure Parity (CR1, PCE and PS bits)
    // PCE = 1 enables parity control. PS = 0 (even), 1 (odd)
    if (uart_parity != UART_PARITY_NONE) {
        *USART_CR1_ARRAY[uart_channel] |= (1U << 10); // PCE bit
        if (uart_parity == UART_PARITY_ODD) {
            *USART_CR1_ARRAY[uart_channel] |= (1U << 9); // PS bit (odd parity)
        } else {
            *USART_CR1_ARRAY[uart_channel] &= ~(1U << 9); // PS bit (even parity)
        }
    } else {
        *USART_CR1_ARRAY[uart_channel] &= ~(1U << 10); // PCE bit
    }

    // Enable Transmit and Receive (TE, RE bits in CR1)
    *USART_CR1_ARRAY[uart_channel] |= ((1U << 3) | (1U << 2));
}

void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable peripheral clock first (as per peripheral_enable_rules)
    switch (uart_channel) {
        case UART_CHANNEL_1:
            *RCC_APB2ENR |= (1U << 4); // Inferred: USART1EN bit 4
            break;
        case UART_CHANNEL_2:
            *RCC_APB1ENR |= (1U << 17); // Inferred: USART2EN bit 17
            break;
        case UART_CHANNEL_6:
            *RCC_APB2ENR |= (1U << 5); // Inferred: USART6EN bit 5
            break;
        default:
            return;
    }
    // Enable UART (UE bit in CR1)
    *USART_CR1_ARRAY[uart_channel] |= (1U << 13); // UE bit
}

void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Disable UART (UE bit in CR1)
    *USART_CR1_ARRAY[uart_channel] &= ~(1U << 13); // UE bit
    // Optionally disable peripheral clock to save power
    switch (uart_channel) {
        case UART_CHANNEL_1:
            *RCC_APB2ENR &= ~(1U << 4);
            break;
        case UART_CHANNEL_2:
            *RCC_APB1ENR &= ~(1U << 17);
            break;
        case UART_CHANNEL_6:
            *RCC_APB2ENR &= ~(1U << 5);
            break;
        default:
            break;
    }
}

void UART_Update(t_uart_channel uart_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This function typically updates internal states or checks for flags.
    // For a simple MCAL, it could just be a no-op if no complex state machine.
    // If it's meant to clear status flags, then UART_ClearFlag should be used instead.
    (void)uart_channel; // Parameter not used in this generic implementation
}

void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Wait until Transmit data register empty (TXE bit in SR)
    while (!(*USART_SR_ARRAY[uart_channel] & (1U << 7))); // TXE flag
    // Write data to Data register (DR)
    *USART_DR_ARRAY[uart_channel] = byte;
    // Wait until Transmission complete (TC bit in SR)
    while (!(*USART_SR_ARRAY[uart_channel] & (1U << 6))); // TC flag
}

void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    while (*str != '\0') {
        UART_send_byte(uart_channel, (tbyte)*str);
        str++;
    }
}

tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Wait until Read data register not empty (RXNE bit in SR)
    while (!(*USART_SR_ARRAY[uart_channel] & (1U << 5))); // RXNE flag
    // Read data from Data register (DR)
    return (tbyte)(*USART_DR_ARRAY[uart_channel]);
}

void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    tbyte bytes_read = 0;
    for (int i = 0; i < max_length - 1; i++) { // Leave space for null terminator
        char received_char = (char)UART_Get_Byte(uart_channel);
        buffer[i] = received_char;
        bytes_read++;
        if (received_char == '\n' || received_char == '\r') { // Example termination
            break;
        }
    }
    buffer[bytes_read] = '\0'; // Null-terminate the string
    return bytes_read;
}

void UART_ClearFlag(t_uart_channel uart_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Clear specific flags by writing 0 to them (or reading SR then DR for RXNE)
    // For STM32, some flags (like RXNE) are cleared by reading DR.
    // Other flags (like ORE, NE, FE, PE) are cleared by reading SR then DR.
    // TC flag is cleared by writing 0 to TC bit.
    // General approach: Read SR, then take action.
    volatile uint32_t temp_sr = *USART_SR_ARRAY[uart_channel];
    (void)temp_sr; // Consume SR read for flag clearing where applicable
    // For TXE and TC, they are cleared by operations. Other error flags can be cleared by reading SR then DR.
    // Example: *USART_SR_ARRAY[uart_channel] &= ~(1U << 6); // Clearing TC, if direct write is supported
    // For STM32F4, TC is cleared by writing 0 to it or enabling interrupt.
    // This will clear only common flags by reading SR.
    *USART_DR_ARRAY[uart_channel]; // Read DR to clear RXNE and associated error flags
}

// --- I2C API ---

static volatile uint32_t *const I2C_CR1_ARRAY[] = {I2C1_CR1, I2C2_CR1, I2C3_CR1};
static volatile uint32_t *const I2C_CR2_ARRAY[] = {I2C1_CR2, I2C2_CR2, I2C3_CR2};
static volatile uint32_t *const I2C_OAR1_ARRAY[] = {I2C1_OAR1, I2C2_OAR1, I2C3_OAR1};
static volatile uint32_t *const I2C_DR_ARRAY[] = {I2C1_DR, I2C2_DR, I2C3_DR};
static volatile uint32_t *const I2C_SR1_ARRAY[] = {I2C1_SR1, I2C2_SR1, I2C3_SR1};
static volatile uint32_t *const I2C_SR2_ARRAY[] = {I2C1_SR2, I2C2_SR2, I2C3_SR2};
static volatile uint32_t *const I2C_CCR_ARRAY[] = {I2C1_CCR, I2C2_CCR, I2C3_CCR};
static volatile uint32_t *const I2C_TRISE_ARRAY[] = {I2C1_TRISE, I2C2_TRISE, I2C3_TRISE};

void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    // 1. Enable I2C peripheral clock
    // I2C1, I2C2, I2C3 on APB1
    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            *RCC_APB1ENR |= (1U << 21); // Inferred: I2C1EN bit 21
            break;
        case I2C_CHANNEL_2:
            *RCC_APB1ENR |= (1U << 22); // Inferred: I2C2EN bit 22
            break;
        case I2C_CHANNEL_3:
            *RCC_APB1ENR |= (1U << 23); // Inferred: I2C3EN bit 23
            break;
        default:
            return; // Invalid channel
    }

    // 2. Reset I2C peripheral (PE bit in CR1 cleared and set back)
    *I2C_CR1_ARRAY[i2c_channel] &= ~(1U << 0); // Clear PE bit to reset
    // Wait some cycles for reset to take effect. No register to verify.
    // For simplicity, directly reconfigure.

    // 3. Configure peripheral clock frequency (FREQ bits in CR2)
    // Assuming APB1 clock is 16MHz for F401
    // FREQ[5:0] bits in I2C_CR2 should be APB1 clock frequency in MHz.
    // For 16MHz, set FREQ = 16.
    *I2C_CR2_ARRAY[i2c_channel] = 16; // Inferred: Set peripheral clock frequency to 16MHz

    // 4. Configure Clock Speed (CCR register) - Always use fast mode as per rule
    // Sm mode (standard): Thigh = Tlow = CCR * TPCLK1. TPCLK1 = 1/PCLK1
    // Fm mode (fast): Tlow = CCR * TPCLK1, Thigh = 2 * CCR * TPCLK1 (or 1, depends on duty cycle)
    // Duty cycle can be controlled by F/S (bit 15) and DUTY (bit 14) in CCR.
    // Assuming 400kHz Fast Mode (400000 Hz) and PCLK1=16MHz (16000000 Hz)
    // For Fm (400kHz), CCR = PCLK1 / (3 * I2C_speed) (if DUTY=0)
    // CCR = 16000000 / (3 * 400000) = 16000000 / 1200000 = 13.33 -> 13
    uint16_t ccr_val = 13;
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST) {
        *I2C_CCR_ARRAY[i2c_channel] |= (1U << 15); // F/S bit for Fast Mode
        *I2C_CCR_ARRAY[i2c_channel] |= (1U << 14); // DUTY bit for Fm mode t_low/t_high = 2 (standard 400kHz)
        // For DUTY=1 (t_low/t_high = 2), CCR = PCLK1 / (3 * I2C_speed).
        // For DUTY=0 (t_low/t_high = 1), CCR = PCLK1 / (2 * I2C_speed).
        ccr_val = 13; // Example for 400kHz (16MHz / (3 * 400kHz))
    } else { // Standard mode
        *I2C_CCR_ARRAY[i2c_channel] &= ~(1U << 15); // F/S bit for Standard Mode
        // CCR = PCLK1 / (2 * I2C_speed). For 100kHz, CCR = 16000000 / (2 * 100000) = 80
        ccr_val = 80;
    }
    *I2C_CCR_ARRAY[i2c_channel] = (*I2C_CCR_ARRAY[i2c_channel] & ~0xFFFU) | ccr_val; // Set CCR value

    // 5. Configure TRISE register (Rise Time)
    // TRISE = (Maximum SCL rise time / PCLK1) + 1
    // For Sm mode: 1000ns. TRISE = (1000ns / (1/16MHz)) + 1 = (1000 * 16) + 1 = 16+1 = 17
    // For Fm mode: 300ns. TRISE = (300ns / (1/16MHz)) + 1 = (300 * 16) + 1 = 4.8+1 = 5.8 -> 6
    uint32_t trise_val = 0;
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST) {
        trise_val = 6;
    } else {
        trise_val = 17;
    }
    *I2C_TRISE_ARRAY[i2c_channel] = trise_val;


    // 6. Configure Own Address (OAR1 register) - Addressing Mode equals Device Address
    // OAR1 contains OA1[7:1] for 7-bit address or OA1[9:0] for 10-bit address.
    // MODE bit (bit 15) for 10-bit addressing mode.
    *I2C_OAR1_ARRAY[i2c_channel] = (i2c_device_address << 1) | (1U << 14); // Set 7-bit address and MSK bit 14

    // 7. Acknowledge (ACK bit in CR1)
    if (i2c_ack == I2C_ACK_ENABLE) {
        *I2C_CR1_ARRAY[i2c_channel] |= (1U << 10); // ACK bit
    } else {
        *I2C_CR1_ARRAY[i2c_channel] &= ~(1U << 10); // ACK bit
    }

    // 8. Data Length (not directly configured in I2C peripheral init for individual bytes)
    // The data length is implicit in how bytes are read/written (byte by byte)
    (void)i2c_datalength; // Parameter not used for register config

    // Enable I2C peripheral
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 0); // PE bit
}

void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable peripheral clock first (as per peripheral_enable_rules)
    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            *RCC_APB1ENR |= (1U << 21); // Inferred: I2C1EN bit 21
            break;
        case I2C_CHANNEL_2:
            *RCC_APB1ENR |= (1U << 22); // Inferred: I2C2EN bit 22
            break;
        case I2C_CHANNEL_3:
            *RCC_APB1ENR |= (1U << 23); // Inferred: I2C3EN bit 23
            break;
        default:
            return;
    }
    // Enable I2C (PE bit in CR1)
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 0); // PE bit
}

void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Disable I2C (PE bit in CR1)
    *I2C_CR1_ARRAY[i2c_channel] &= ~(1U << 0); // PE bit
    // Optionally disable peripheral clock to save power
    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            *RCC_APB1ENR &= ~(1U << 21);
            break;
        case I2C_CHANNEL_2:
            *RCC_APB1ENR &= ~(1U << 22);
            break;
        case I2C_CHANNEL_3:
            *RCC_APB1ENR &= ~(1U << 23);
            break;
        default:
            break;
    }
}

void I2C_Update(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This function typically updates internal states or checks for flags.
    // For a simple MCAL, it could just be a no-op if no complex state machine.
    (void)i2c_channel; // Parameter not used in this generic implementation
}

void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Common sequence for I2C Master Transmit:
    // 1. Generate Start condition (START bit in CR1)
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 8); // START bit
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 0))); // Wait for SB (Start bit) flag

    // 2. Send slave address + R/W bit (DR register)
    *I2C_DR_ARRAY[i2c_channel] = (0x00 << 1); // Placeholder for slave address 0x00, write mode (LSB 0)
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 1))); // Wait for ADDR (Address sent) flag

    // 3. Clear ADDR flag by reading SR2
    volatile uint32_t temp_sr2 = *I2C_SR2_ARRAY[i2c_channel];
    (void)temp_sr2;

    // 4. Send data byte (DR register)
    *I2C_DR_ARRAY[i2c_channel] = byte;
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 7))); // Wait for TxE (Data register empty) flag

    // 5. Generate Stop condition (STOP bit in CR1)
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 9); // STOP bit
    // No need to wait for stop bit to clear if it's the end of transmission.
}

void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    // Generate Start condition
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 8); // START bit
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 0))); // Wait for SB (Start bit) flag

    // Send slave address + R/W bit (DR register)
    // "Always use a repeated start condition instead of stop between transactions" means
    // no STOP condition between address and first byte, or between bytes if sending multiple frames.
    // For a single frame, this assumes the address is part of the "frame send" logic.
    *I2C_DR_ARRAY[i2c_channel] = (0x00 << 1); // Placeholder for slave address 0x00, write mode (LSB 0)
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 1))); // Wait for ADDR (Address sent) flag
    volatile uint32_t temp_sr2 = *I2C_SR2_ARRAY[i2c_channel]; // Clear ADDR flag
    (void)temp_sr2;

    for (int i = 0; i < length; i++) {
        *I2C_DR_ARRAY[i2c_channel] = (tbyte)data[i];
        while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 7))); // Wait for TxE (Data register empty) flag
    }
    // Generate Stop condition after all bytes are sent
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 9); // STOP bit
}

void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    I2C_send_frame(i2c_channel, str, strlen(str));
}

tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Master Receive sequence (simplified):
    // 1. Generate Start condition
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 8); // START bit
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 0))); // Wait for SB flag

    // 2. Send slave address + R/W bit (Read mode, LSB 1)
    *I2C_DR_ARRAY[i2c_channel] = (0x00 << 1) | 1U; // Placeholder slave address 0x00, read mode
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 1))); // Wait for ADDR flag

    // 3. Clear ADDR flag by reading SR2
    volatile uint32_t temp_sr2 = *I2C_SR2_ARRAY[i2c_channel];
    (void)temp_sr2;

    // 4. Disable Acknowledge before receiving the last byte (NACK on last byte)
    *I2C_CR1_ARRAY[i2c_channel] &= ~(1U << 10); // Clear ACK bit

    // 5. Generate Stop condition (NACK + STOP for single byte read)
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 9); // STOP bit

    // 6. Wait for RxNE (Data register not empty)
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 6))); // RxNE flag

    // 7. Read data
    tbyte received_byte = (tbyte)(*I2C_DR_ARRAY[i2c_channel]);

    // Re-enable Acknowledge for next transaction (if needed)
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 10); // ACK bit
    return received_byte;
}

void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Generate Start condition
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 8); // START bit
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 0))); // Wait for SB flag

    // Send slave address + R/W bit (Read mode)
    *I2C_DR_ARRAY[i2c_channel] = (0x00 << 1) | 1U; // Placeholder slave address 0x00, read mode
    while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 1))); // Wait for ADDR flag
    volatile uint32_t temp_sr2 = *I2C_SR2_ARRAY[i2c_channel]; // Clear ADDR flag
    (void)temp_sr2;

    for (int i = 0; i < max_length; i++) {
        if (i == max_length - 1) {
            // Last byte: disable ACK and generate STOP
            *I2C_CR1_ARRAY[i2c_channel] &= ~(1U << 10); // Clear ACK bit
            *I2C_CR1_ARRAY[i2c_channel] |= (1U << 9); // STOP bit
        }
        while (!(*I2C_SR1_ARRAY[i2c_channel] & (1U << 6))); // Wait for RxNE
        buffer[i] = (char)(*I2C_DR_ARRAY[i2c_channel]);
    }
    // Re-enable Acknowledge for next transaction (if needed)
    *I2C_CR1_ARRAY[i2c_channel] |= (1U << 10); // ACK bit
}

tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This is typically for character-by-character reading until a terminator.
    // I2C is byte-oriented, not string-oriented. Implementation will read up to max_length or until a '\0' (if applicable)
    tbyte bytes_read = 0;
    I2C_Get_frame(i2c_channel, buffer, max_length - 1); // Get frame, leave space for null
    buffer[max_length - 1] = '\0'; // Ensure null termination
    // Calculate actual bytes read (find null or end of buffer)
    for (int i = 0; i < max_length; i++) {
        if (buffer[i] == '\0') break;
        bytes_read++;
    }
    return bytes_read;
}

void I2C_ClearFlag(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // I2C flags are typically cleared by sequence of reading SR1 then SR2, or reading data register.
    // Example: To clear ADDR flag: read SR1 then read SR2.
    // To clear RxNE flag: read DR.
    // To clear STOPF flag: read SR1 then write CR1.
    // This generic function will attempt to clear common flags.
    volatile uint32_t temp_sr1 = *I2C_SR1_ARRAY[i2c_channel];
    volatile uint32_t temp_sr2 = *I2C_SR2_ARRAY[i2c_channel];
    (void)temp_sr1;
    (void)temp_sr2;
    // Clear any pending flags by reading status registers.
    // Explicitly clearing flags like PECerr, SMBAlert, TIMEOUT.
    // For specific event flags (like SB, ADDR, BTF, TxE, RxNE), they are cleared by operations.
    // This is a placeholder for more specific flag clearing logic.
}

// --- SPI API ---

static volatile uint32_t *const SPI_CR1_ARRAY[] = {SPI1_CR1, SPI2_CR1, SPI3_CR1};
static volatile uint32_t *const SPI_CR2_ARRAY[] = {SPI1_CR2, SPI2_CR2, SPI3_CR2};
static volatile uint32_t *const SPI_SR_ARRAY[] = {SPI1_SR, SPI2_SR, SPI3_SR};
static volatile uint32_t *const SPI_DR_ARRAY[] = {SPI1_DR, SPI2_DR, SPI3_DR};
static volatile uint32_t *const SPI_CRCPR_ARRAY[] = {SPI1_CRCPR, SPI2_CRCPR, SPI3_CRCPR};

void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    // 1. Enable SPI peripheral clock
    // SPI1 on APB2, SPI2/3 on APB1
    switch (spi_channel) {
        case SPI_CHANNEL_1:
            *RCC_APB2ENR |= (1U << 12); // Inferred: SPI1EN bit 12
            break;
        case SPI_CHANNEL_2:
            *RCC_APB1ENR |= (1U << 14); // Inferred: SPI2EN bit 14
            break;
        case SPI_CHANNEL_3:
            *RCC_APB1ENR |= (1U << 15); // Inferred: SPI3EN bit 15
            break;
        default:
            return; // Invalid channel
    }

    // 2. Configure CPOL (Clock Polarity, CPOL bit 1 in CR1)
    if (spi_cpol == SPI_CPOL_HIGH) {
        *SPI_CR1_ARRAY[spi_channel] |= (1U << 1); // Set CPOL
    } else {
        *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 1); // Clear CPOL
    }

    // 3. Configure CPHA (Clock Phase, CPHA bit 0 in CR1)
    if (spi_cpha == SPI_CPHA_SECOND) {
        *SPI_CR1_ARRAY[spi_channel] |= (1U << 0); // Set CPHA
    } else {
        *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 0); // Clear CPHA
    }

    // 4. Configure Data Frame Format (DFF bit 11 in CR1)
    if (spi_dff == SPI_DFF_16_BIT) {
        *SPI_CR1_ARRAY[spi_channel] |= (1U << 11); // Set DFF for 16-bit
    } else {
        *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 11); // Clear DFF for 8-bit
    }

    // 5. Configure Bit Order (LSBFIRST bit 7 in CR1)
    if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST) {
        *SPI_CR1_ARRAY[spi_channel] |= (1U << 7); // Set LSBFIRST
    } else {
        *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 7); // Clear LSBFIRST (MSB first)
    }

    // 6. Configure Master/Slave Mode (MSTR bit 2 in CR1)
    if (spi_mode == SPI_MODE_MASTER) {
        *SPI_CR1_ARRAY[spi_channel] |= (1U << 2); // Set MSTR
        // Always use fast speed - configure Baud Rate Control (BR[2:0] bits 5:3 in CR1)
        // Set to lowest prescaler for fastest speed. E.g., PCLK/2 (000)
        *SPI_CR1_ARRAY[spi_channel] &= ~(7U << 3); // Clear BR bits
        // *SPI_CR1_ARRAY[spi_channel] |= (0U << 3); // Set BR to PCLK/2 (fastest)
    } else {
        *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 2); // Clear MSTR (slave mode)
    }

    // 7. Slave Select always software-controlled (SSM bit 9 in CR1, SSI bit 8 in CR1)
    *SPI_CR1_ARRAY[spi_channel] |= (1U << 9);  // SSM: Software slave management enable
    *SPI_CR1_ARRAY[spi_channel] |= (1U << 8);  // SSI: Internal slave select (set to 1 for Master, 0 for Slave with NSS hardware)

    // 8. Always use full duplex (BIDIMODE bit 15 in CR1, RXONLY bit 10 in CR1)
    *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 15); // Clear BIDIMODE (full-duplex)
    *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 10); // Clear RXONLY (full-duplex)

    // 9. Always enable CRC (CRCEN bit 13 in CR1)
    *SPI_CR1_ARRAY[spi_channel] |= (1U << 13); // Set CRCEN

    // 10. Enable SPI peripheral (SPE bit 6 in CR1) is typically done in SPI_Enable API
}

void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable peripheral clock first (as per peripheral_enable_rules)
    switch (spi_channel) {
        case SPI_CHANNEL_1:
            *RCC_APB2ENR |= (1U << 12); // Inferred: SPI1EN bit 12
            break;
        case SPI_CHANNEL_2:
            *RCC_APB1ENR |= (1U << 14); // Inferred: SPI2EN bit 14
            break;
        case SPI_CHANNEL_3:
            *RCC_APB1ENR |= (1U << 15); // Inferred: SPI3EN bit 15
            break;
        default:
            return;
    }
    // Enable SPI (SPE bit 6 in CR1)
    *SPI_CR1_ARRAY[spi_channel] |= (1U << 6); // SPE bit
}

void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Disable SPI (SPE bit 6 in CR1)
    *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 6); // SPE bit
    // Optionally disable peripheral clock to save power
    switch (spi_channel) {
        case SPI_CHANNEL_1:
            *RCC_APB2ENR &= ~(1U << 12);
            break;
        case SPI_CHANNEL_2:
            *RCC_APB1ENR &= ~(1U << 14);
            break;
        case SPI_CHANNEL_3:
            *RCC_APB1ENR &= ~(1U << 15);
            break;
        default:
            break;
    }
}

void SPI_Update(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This function typically updates internal states or checks for flags.
    // For a simple MCAL, it could just be a no-op if no complex state machine.
}

void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Wait until Transmit buffer empty (TXE bit in SR)
    while (!(*SPI_SR_ARRAY[spi_channel] & (1U << 1))); // TXE flag
    // Write data to Data register (DR)
    *SPI_DR_ARRAY[spi_channel] = byte;
    // Wait until BSY flag is cleared (busy flag)
    while (*SPI_SR_ARRAY[spi_channel] & (1U << 7)); // BSY flag
}

void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

void SPI_send_string(t_spi_channel spi_channel, const char *str) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    while (*str != '\0') {
        SPI_Send_Byte(spi_channel, (tbyte)*str);
        str++;
    }
}

tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // To read a byte in full-duplex, you typically write a dummy byte to initiate clocking
    *SPI_DR_ARRAY[spi_channel] = 0xFF; // Write dummy byte
    // Wait until Receive buffer not empty (RXNE bit in SR)
    while (!(*SPI_SR_ARRAY[spi_channel] & (1U << 0))); // RXNE flag
    // Read data from Data register (DR)
    return (tbyte)(*SPI_DR_ARRAY[spi_channel]);
}

void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This is typically for character-by-character reading until a terminator.
    // SPI is byte-oriented, not string-oriented. Implementation will read up to max_length or until a '\0' (if applicable)
    tbyte bytes_read = 0;
    for (int i = 0; i < max_length - 1; i++) { // Leave space for null terminator
        char received_char = (char)SPI_Get_Byte(spi_channel);
        buffer[i] = received_char;
        bytes_read++;
        if (received_char == '\0') { // Example termination
            break;
        }
    }
    buffer[bytes_read] = '\0'; // Null-terminate the string
    return bytes_read;
}

void SPI_ClearFlag(t_spi_channel spi_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // SPI flags are typically cleared by reading status register or specific operations.
    // OVR (Overrun error) is cleared by reading SR then DR.
    // CRCERR is cleared by writing 0 to the bit.
    volatile uint32_t temp_sr = *SPI_SR_ARRAY[spi_channel];
    (void)temp_sr; // Reading SR
    // if (*SPI_SR_ARRAY[spi_channel] & (1U << 0)) // Check RXNE (read DR to clear)
    // {
    //     volatile uint32_t temp_dr = *SPI_DR_ARRAY[spi_channel];
    //     (void)temp_dr;
    // }
    // if (*SPI_SR_ARRAY[spi_channel] & (1U << 6)) // Check OVR (read SR, then DR to clear)
    // {
    //     volatile uint32_t temp_dr = *SPI_DR_ARRAY[spi_channel];
    //     (void)temp_dr;
    // }
    *SPI_CR1_ARRAY[spi_channel] &= ~(1U << 4); // Clear CRCERR flag
}

// --- External Interrupt API ---

static volatile uint32_t *const SYSCFG_EXTICR_ARRAY[] = {SYSCFG_EXTICR1, SYSCFG_EXTICR2, SYSCFG_EXTICR3, SYSCFG_EXTICR4};

void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    // 1. Enable SYSCFG clock (for EXTI configuration registers)
    *RCC_APB2ENR |= (1U << 14); // Inferred: SYSCFGEN bit 14

    // 2. Select the source input for the EXTIx external interrupt line (SYSCFG_EXTICRx)
    // EXTI0-3 -> EXTICR1, EXTI4-7 -> EXTICR2, EXTI8-11 -> EXTICR3, EXTI12-15 -> EXTICR4
    uint32_t exti_reg_idx = external_int_channel / 4;
    uint32_t exti_pin_idx = external_int_channel % 4;
    // Assuming Port A for simplicity if not assigned to specific pin.
    // For STM32, EXTI line is configured to a GPIO port (0x00 for PAx, 0x01 for PBx, etc.)
    // Since assigned_pin is not directly passed to the API, we assume the EXTI line corresponds to Pin X on PORTA.
    // Setting to 0x0 (Port A) for default, actual port should be passed by higher layer.
    uint32_t gpio_port_config_val = 0x0; // Assumed PAx

    *SYSCFG_EXTICR_ARRAY[exti_reg_idx] = (*SYSCFG_EXTICR_ARRAY[exti_reg_idx] & ~(0xFUL << (exti_pin_idx * 4))) | (gpio_port_config_val << (exti_pin_idx * 4));


    // 3. Configure trigger edge (RTSR for rising, FTSR for falling)
    if (external_int_edge == EXT_INT_EDGE_RISING || external_int_edge == EXT_INT_EDGE_BOTH_EDGES) {
        *EXTI_RTSR |= (1U << external_int_channel);
    } else {
        *EXTI_RTSR &= ~(1U << external_int_channel);
    }

    if (external_int_edge == EXT_INT_EDGE_FALLING || external_int_edge == EXT_INT_EDGE_BOTH_EDGES) {
        *EXTI_FTSR |= (1U << external_int_channel);
    } else {
        *EXTI_FTSR &= ~(1U << external_int_channel);
    }

    // 4. Optionally configure Event Mask (EMR) - not enabled by default for interrupt
    *EXTI_EMR &= ~(1U << external_int_channel); // Ensure event is disabled for interrupt API

    // 5. Enable Interrupt Mask (IMR) - this should be done in Enable function
}

void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable SYSCFG clock first (as per peripheral_enable_rules)
    *RCC_APB2ENR |= (1U << 14); // Inferred: SYSCFGEN bit 14

    // Enable Interrupt Mask (IMR) for the line
    *EXTI_IMR |= (1U << external_int_channel);
}

void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Disable Interrupt Mask (IMR) for the line
    *EXTI_IMR &= ~(1U << external_int_channel);
}

void External_INT_ClearFlag(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Clear pending flag by writing 1 to the corresponding bit in PR
    *EXTI_PR |= (1U << external_int_channel);
}

// --- GPIO API ---

void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable GPIO Port Clock (as per peripheral_enable_rules)
    // RCC_AHB1ENR bits: GPIOA=0, GPIOB=1, GPIOC=2, GPIOD=3, GPIOE=4, GPIOH=7
    if (port < GPIO_PORTH) {
        *RCC_AHB1ENR |= (1U << port);
    } else if (port == GPIO_PORTH) {
        *RCC_AHB1ENR |= (1U << 7); // Explicitly enable GPIOH clock
    }

    // Always set value before setting direction (GPIO_rules)
    if (value == 0) {
        *GPIO_ODR_ARRAY[port] &= ~(1U << pin); // Clear bit
    } else {
        *GPIO_ODR_ARRAY[port] |= (1U << pin);  // Set bit
    }
    // Verify value after setting
    if (value == 0) {
        while ((*GPIO_ODR_ARRAY[port] & (1U << pin)) != 0) { /* Wait */ }
    } else {
        while ((*GPIO_ODR_ARRAY[port] & (1U << pin)) == 0) { /* Wait */ }
    }


    // Set direction to Output (MODER bits 01)
    *GPIO_MODER_ARRAY[port] = (*GPIO_MODER_ARRAY[port] & ~(3U << (pin * 2))) | (1U << (pin * 2));
    // Verify direction
    while ((*GPIO_MODER_ARRAY[port] & (3U << (pin * 2))) != (1U << (pin * 2))) { /* Wait */ }

    // All output pins have pull-up resistors disabled (PUPDR bits 00)
    *GPIO_PUPDR_ARRAY[port] &= ~(3U << (pin * 2)); // Clear PUPDR bits

    // For current registers: use >=20mA sink current & >=10mA source current (No direct registers for this current setting in JSON, placeholder)
    // Output type (OTYPER) and output speed (OSPEEDR) should be configured for current.
    // Output Type: Push-pull (0) or Open-drain (1) - OTYPER bit. Defaulting to push-pull
    *GPIO_OTYPER_ARRAY[port] &= ~(1U << pin); // Push-pull
    // Output Speed: Low (00), Medium (01), High (10), Very high (11) - OSPEEDR bits. Setting to High/Very high.
    *GPIO_OSPEEDR_ARRAY[port] = (*GPIO_OSPEEDR_ARRAY[port] & ~(3U << (pin * 2))) | (3U << (pin * 2)); // Very high speed
}

void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable GPIO Port Clock (as per peripheral_enable_rules)
    // RCC_AHB1ENR bits: GPIOA=0, GPIOB=1, GPIOC=2, GPIOD=3, GPIOE=4, GPIOH=7
    if (port < GPIO_PORTH) {
        *RCC_AHB1ENR |= (1U << port);
    } else if (port == GPIO_PORTH) {
        *RCC_AHB1ENR |= (1U << 7); // Explicitly enable GPIOH clock
    }

    // Set direction to Input (MODER bits 00)
    *GPIO_MODER_ARRAY[port] &= ~(3U << (pin * 2)); // Clear mode bits
    // Verify direction
    while ((*GPIO_MODER_ARRAY[port] & (3U << (pin * 2))) != 0U) { /* Wait */ }

    // All input pins have pull-up resistors (PUPDR bits 01 for pull-up)
    *GPIO_PUPDR_ARRAY[port] = (*GPIO_PUPDR_ARRAY[port] & ~(3U << (pin * 2))) | (1U << (pin * 2)); // Set pull-up
    // And wakeup feature enabled (if available) - this is for EXTI, not directly GPIO setup
    // For EXTI, SYSCFG clock and EXTI registers need to be configured.
}

t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Check MODER register bits for the pin
    uint32_t mode = (*GPIO_MODER_ARRAY[port] >> (pin * 2)) & 3U;
    if (mode == 0U) { // 00: Input mode
        return GPIO_DIRECTION_INPUT;
    } else if (mode == 1U) { // 01: General purpose output mode
        return GPIO_DIRECTION_OUTPUT;
    }
    return (t_direction)-1; // Undefined/error, for unhandled modes like alternate function or analog
}

void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Use BSRR register for atomic set/reset
    // BSRR has lower 16 bits for set, upper 16 bits for reset
    if (value == 0) {
        *GPIO_BSRR_ARRAY[port] = (1U << (pin + 16)); // Set reset bit
    } else {
        *GPIO_BSRR_ARRAY[port] = (1U << pin);        // Set set bit
    }
    // Verify value after setting
    if (value == 0) {
        while ((*GPIO_ODR_ARRAY[port] & (1U << pin)) != 0) { /* Wait */ }
    } else {
        while ((*GPIO_ODR_ARRAY[port] & (1U << pin)) == 0) { /* Wait */ }
    }
}

tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Read from IDR for input, ODR for output (depends on mode)
    // Assuming we read the actual pin state (from IDR)
    if (*GPIO_IDR_ARRAY[port] & (1U << pin)) {
        return 1;
    } else {
        return 0;
    }
}

void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Toggle ODR bit
    *GPIO_ODR_ARRAY[port] ^= (1U << pin);
    // Verify (read back ODR)
    tbyte expected_value = (*GPIO_ODR_ARRAY[port] >> pin) & 1U;
    while (((*GPIO_ODR_ARRAY[port] >> pin) & 1U) != expected_value) { /* Wait */ }
}


// --- PWM API ---

// Pointers to TIM structures/registers for easier access
// Note: Actual TIM registers are offset from base addresses
// This uses the individual TIMx_ registers as defined by JSON, which are already specific addresses.

typedef struct {
    volatile uint32_t *CR1;
    volatile uint32_t *CR2;
    volatile uint32_t *SMCR;
    volatile uint32_t *DIER;
    volatile uint32_t *SR;
    volatile uint32_t *EGR;
    volatile uint32_t *CCMR1;
    volatile uint32_t *CCMR2;
    volatile uint32_t *CCER;
    volatile uint32_t *CNT;
    volatile uint32_t *PSC;
    volatile uint32_t *ARR;
    volatile uint32_t *RCR;
    volatile uint32_t *CCR1;
    volatile uint32_t *CCR2;
    volatile uint32_t *CCR3;
    volatile uint32_t *CCR4;
    volatile uint32_t *BDTR;
    volatile uint32_t *DCR;
    volatile uint32_t *DMAR;
    volatile uint32_t *OR; // Only for TIM2, TIM5
    // Add other fields if necessary
} TimerRegs_t;

static TimerRegs_t Timer_Instances[] = {
    { TIM1_CR1, TIM1_CR2, TIM1_SMCR, TIM1_DIER, TIM1_SR, TIM1_EGR, TIM1_CCMR1, TIM1_CCMR2, TIM1_CCER, TIM1_CNT, TIM1_PSC, TIM1_ARR, TIM1_RCR, TIM1_CCR1, TIM1_CCR2, TIM1_CCR3, TIM1_CCR4, TIM1_BDTR, TIM1_DCR, TIM1_DMAR, NULL }, // TIM1
    { TIM2_CR1, TIM2_CR2, TIM2_SMCR, TIM2_DIER, TIM2_SR, TIM2_EGR, TIM2_CCMR1, TIM2_CCMR2, TIM2_CCER, TIM2_CNT, TIM2_PSC, TIM2_ARR, NULL, TIM2_CCR1, TIM2_CCR2, TIM2_CCR3, TIM2_CCR4, NULL, TIM2_DCR, TIM2_DMAR, TIM2_OR }, // TIM2
    { TIM3_CR1, TIM3_CR2, TIM3_SMCR, TIM3_DIER, TIM3_SR, TIM3_EGR, TIM3_CCMR1, TIM3_CCMR2, TIM3_CCER, TIM3_CNT, TIM3_PSC, TIM3_ARR, NULL, TIM3_CCR1, TIM3_CCR2, TIM3_CCR3, TIM3_CCR4, NULL, TIM3_DCR, TIM3_DMAR, NULL }, // TIM3
    { TIM4_CR1, TIM4_CR2, TIM4_SMCR, TIM4_DIER, TIM4_SR, TIM4_EGR, TIM4_CCMR1, TIM4_CCMR2, TIM4_CCER, TIM4_CNT, TIM4_PSC, TIM4_ARR, NULL, TIM4_CCR1, TIM4_CCR2, TIM4_CCR3, TIM4_CCR4, NULL, TIM4_DCR, TIM4_DMAR, NULL }, // TIM4
    { TIM5_CR1, TIM5_CR2, TIM5_SMCR, TIM5_DIER, TIM5_SR, TIM5_EGR, TIM5_CCMR1, TIM5_CCMR2, TIM5_CCER, TIM5_CNT, TIM5_PSC, TIM5_ARR, NULL, TIM5_CCR1, TIM5_CCR2, TIM5_CCR3, TIM5_CCR4, NULL, TIM5_DCR, TIM5_DMAR, TIM5_OR }, // TIM5
    { TIM9_CR1, NULL, TIM9_SMCR, TIM9_DIER, TIM9_SR, TIM9_EGR, TIM9_CCMR1, NULL, TIM9_CCER, TIM9_CNT, TIM9_PSC, TIM9_ARR, NULL, TIM9_CCR1, TIM9_CCR2, NULL, NULL, NULL, NULL, NULL, NULL }, // TIM9
    { TIM10_CR1, NULL, NULL, TIM10_DIER, TIM10_SR, TIM10_EGR, TIM10_CCMR1, NULL, TIM10_CCER, TIM10_CNT, TIM10_PSC, TIM10_ARR, NULL, TIM10_CCR1, NULL, NULL, NULL, NULL, NULL, NULL, NULL }, // TIM10
    { TIM11_CR1, NULL, NULL, TIM11_DIER, TIM11_SR, TIM11_EGR, TIM11_CCMR1, NULL, TIM11_CCER, TIM11_CNT, TIM11_PSC, TIM11_ARR, NULL, TIM11_CCR1, NULL, NULL, NULL, NULL, NULL, NULL, NULL }  // TIM11
};

// Helper function to get TIM instance index from PWM channel
static int GetTimerInstanceIndex(t_pwm_channel pwm_channel) {
    if (pwm_channel >= PWM_CHANNEL_TIM1_CH1 && pwm_channel <= PWM_CHANNEL_TIM1_CH4) return 0; // TIM1
    if (pwm_channel >= PWM_CHANNEL_TIM2_CH1 && pwm_channel <= PWM_CHANNEL_TIM2_CH4) return 1; // TIM2
    if (pwm_channel >= PWM_CHANNEL_TIM3_CH1 && pwm_channel <= PWM_CHANNEL_TIM3_CH4) return 2; // TIM3
    if (pwm_channel >= PWM_CHANNEL_TIM4_CH1 && pwm_channel <= PWM_CHANNEL_TIM4_CH4) return 3; // TIM4
    if (pwm_channel >= PWM_CHANNEL_TIM5_CH1 && pwm_channel <= PWM_CHANNEL_TIM5_CH4) return 4; // TIM5
    if (pwm_channel >= PWM_CHANNEL_TIM9_CH1 && pwm_channel <= PWM_CHANNEL_TIM9_CH2) return 5; // TIM9
    if (pwm_channel == PWM_CHANNEL_TIM10_CH1) return 6; // TIM10
    if (pwm_channel == PWM_CHANNEL_TIM11_CH1) return 7; // TIM11
    return -1; // Invalid
}

// Helper function to get TIM channel number (1-4) from PWM channel enum
static int GetTimerChannelNumber(t_pwm_channel pwm_channel) {
    if (pwm_channel >= PWM_CHANNEL_TIM1_CH1 && pwm_channel <= PWM_CHANNEL_TIM1_CH4) return (pwm_channel - PWM_CHANNEL_TIM1_CH1) + 1;
    if (pwm_channel >= PWM_CHANNEL_TIM2_CH1 && pwm_channel <= PWM_CHANNEL_TIM2_CH4) return (pwm_channel - PWM_CHANNEL_TIM2_CH1) + 1;
    if (pwm_channel >= PWM_CHANNEL_TIM3_CH1 && pwm_channel <= PWM_CHANNEL_TIM3_CH4) return (pwm_channel - PWM_CHANNEL_TIM3_CH1) + 1;
    if (pwm_channel >= PWM_CHANNEL_TIM4_CH1 && pwm_channel <= PWM_CHANNEL_TIM4_CH4) return (pwm_channel - PWM_CHANNEL_TIM4_CH1) + 1;
    if (pwm_channel >= PWM_CHANNEL_TIM5_CH1 && pwm_channel <= PWM_CHANNEL_TIM5_CH4) return (pwm_channel - PWM_CHANNEL_TIM5_CH1) + 1;
    if (pwm_channel >= PWM_CHANNEL_TIM9_CH1 && pwm_channel <= PWM_CHANNEL_TIM9_CH2) return (pwm_channel - PWM_CHANNEL_TIM9_CH1) + 1;
    if (pwm_channel == PWM_CHANNEL_TIM10_CH1) return 1;
    if (pwm_channel == PWM_CHANNEL_TIM11_CH1) return 1;
    return -1;
}

// Helper function to enable timer clock based on instance index
static void EnableTimerClock(int timer_idx) {
    // Inferred RCC bits for Timers:
    // APB2: TIM1 (bit 0), TIM9 (bit 16), TIM10 (bit 17), TIM11 (bit 18)
    // APB1: TIM2 (bit 0), TIM3 (bit 1), TIM4 (bit 2), TIM5 (bit 3)
    if (timer_idx == 0) { // TIM1
        *RCC_APB2ENR |= (1U << 0);
    } else if (timer_idx >= 1 && timer_idx <= 4) { // TIM2-TIM5
        *RCC_APB1ENR |= (1U << (timer_idx - 1));
    } else if (timer_idx >= 5 && timer_idx <= 7) { // TIM9-TIM11
        *RCC_APB2ENR |= (1U << (16 + (timer_idx - 5)));
    }
}

void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    int timer_idx = GetTimerInstanceIndex(pwm_channel);
    int channel_num = GetTimerChannelNumber(pwm_channel);
    if (timer_idx == -1 || channel_num == -1) return;

    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Enable Timer Clock (as per peripheral_enable_rules)
    EnableTimerClock(timer_idx);

    // Clear available FREQUENCY Ranges for each channel as comments in PWM_Init()
    // For STM32, the timer frequency is F_TIM / (PSC+1) / (ARR+1)
    // Assuming F_TIM (APB1 for TIM2-5, APB2 for TIM1,9-11) is known (e.g., 16MHz or 32MHz)
    // Max PWM frequency is limited by minimum (PSC+1) and (ARR+1) (e.g., 1) and timer clock.
    // Min PWM frequency is limited by max (PSC+1) and (ARR+1) (65536 * 65536)
    // Example ranges:
    // With 16MHz timer clock:
    //  - Max Freq (PSC=0, ARR=0): 16MHz (16000 kHz)
    //  - Min Freq (PSC=65535, ARR=65535): 16MHz / (65536 * 65536) ~ 0.0037 Hz
    // Target pwm_khz_freq: 1 to 16000 kHz
    // Target pwm_duty: 0 to 100%

    // Stop timer before configuration
    *TIM->CR1 &= ~(1U << 0); // CEN bit

    // Configure Prescaler (PSC) and Auto-Reload Register (ARR) for frequency
    // Assuming a primary clock frequency of 16MHz for APB1 timers, 32MHz for APB2 timers.
    tlong timer_clk_freq = 0;
    if (timer_idx == 0 || (timer_idx >= 5 && timer_idx <= 7)) { // TIM1, TIM9-11 are on APB2
        timer_clk_freq = 32000000; // Assuming APB2 clock is 32MHz
    } else { // TIM2-5 are on APB1
        timer_clk_freq = 16000000; // Assuming APB1 clock is 16MHz
    }

    // Calculate ARR and PSC for desired frequency (pwm_khz_freq)
    // F_PWM = F_TIM / ((PSC + 1) * (ARR + 1))
    // (PSC + 1) * (ARR + 1) = F_TIM / F_PWM
    // A common approach is to set PSC to get a counting frequency that allows good ARR resolution.
    // Let's target ARR to max 0xFFFF (65535) for simplicity and then adjust PSC.
    tlong target_freq_hz = (tlong)pwm_khz_freq * 1000;
    if (target_freq_hz == 0) target_freq_hz = 1; // Prevent division by zero

    tword prescaler = 0;
    tword auto_reload = (tword)((timer_clk_freq / target_freq_hz) - 1);
    if (auto_reload > 0xFFFF) {
        // If ARR exceeds 16-bit, increase prescaler
        prescaler = (tword)((timer_clk_freq / (target_freq_hz * 0xFFFF)) - 1);
        auto_reload = (tword)((timer_clk_freq / ((tlong)prescaler + 1) / target_freq_hz) - 1);
    }
    // Ensure minimal prescaler if auto_reload is very small
    if (prescaler == 0 && auto_reload < 1) { // Freq too high for 1Hz
        auto_reload = 1;
    }

    *TIM->PSC = prescaler;
    *TIM->ARR = auto_reload;

    // Configure PWM Mode in CCMR1/CCMR2
    // Output Compare Mode: PWM mode 1 (110) or PWM mode 2 (111)
    // OCxM[2:0] bits (6:4 for OC1, 14:12 for OC2 in CCMR1, etc.)
    // OCxPE bit (3 for OC1, 11 for OC2 in CCMR1) for preload enable.
    uint32_t ccmr_val = (6U << 4) | (1U << 3); // PWM mode 1, Output Compare Preload Enable
    volatile uint32_t **ccr_reg_ptr;
    if (channel_num == 1 || channel_num == 2) {
        *TIM->CCMR1 &= ~(0xFFU << ((channel_num - 1) * 8)); // Clear OCxM and OCxPE
        *TIM->CCMR1 |= (ccmr_val << ((channel_num - 1) * 8));
        ccr_reg_ptr = (channel_num == 1) ? &TIM->CCR1 : &TIM->CCR2;
    } else if (channel_num == 3 || channel_num == 4) {
        if (TIM->CCMR2 != NULL) {
            *TIM->CCMR2 &= ~(0xFFU << ((channel_num - 3) * 8)); // Clear OCxM and OCxPE
            *TIM->CCMR2 |= (ccmr_val << ((channel_num - 3) * 8));
        }
        ccr_reg_ptr = (channel_num == 3) ? &TIM->CCR3 : &TIM->CCR4;
    } else {
        return; // Invalid channel number for PWM
    }

    // Configure Capture/Compare Register (CCR) for duty cycle
    // CCRx = (ARR + 1) * DutyCycle / 100
    uint32_t ccr_val_duty = (uint32_t)(((tlong)auto_reload + 1) * pwm_duty / 100);
    **ccr_reg_ptr = ccr_val_duty;

    // Enable output for the channel in CCER (Capture/Compare Enable Register)
    // CCxE bit (0 for CH1, 4 for CH2 etc.) for output enable.
    *TIM->CCER |= (1U << ((channel_num - 1) * 4));

    // For Advanced Timers (TIM1), enable main output (MOE bit in BDTR)
    // This allows the PWM signal to be output on pins.
    if (TIM->BDTR != NULL) {
        *TIM->BDTR |= (1U << 15); // MOE bit
    }

    // Generate update event to load PSC and ARR values
    *TIM->EGR |= (1U << 0); // UG bit
}

void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = GetTimerInstanceIndex(pwm_channel);
    if (timer_idx == -1) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Enable Timer Counter (CEN bit in CR1)
    *TIM->CR1 |= (1U << 0); // CEN bit
}

void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = GetTimerInstanceIndex(pwm_channel);
    if (timer_idx == -1) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Disable Timer Counter (CEN bit in CR1)
    *TIM->CR1 &= ~(1U << 0); // CEN bit
}

// --- ICU API ---

// ICU uses Timer Capture mode. Re-using TimerRegs_t and helper functions.

void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    int timer_idx = GetTimerInstanceIndex(icu_channel);
    int channel_num = GetTimerChannelNumber(icu_channel);
    if (timer_idx == -1 || channel_num == -1) return;

    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Enable Timer Clock (as per peripheral_enable_rules)
    EnableTimerClock(timer_idx);

    // Stop timer before configuration
    *TIM->CR1 &= ~(1U << 0); // CEN bit

    // Configure Prescaler
    uint16_t psc_val = 0;
    switch (icu_prescaller) {
        case ICU_PRESCALER_1:   psc_val = 0; break;
        case ICU_PRESCALER_8:   psc_val = 7; break; // 8-1
        case ICU_PRESCALER_64:  psc_val = 63; break; // 64-1
        case ICU_PRESCALER_256: psc_val = 255; break; // 256-1
        default: break;
    }
    *TIM->PSC = psc_val;
    *TIM->ARR = 0xFFFF; // Max auto-reload for frequency measurement

    // Configure Capture/Compare Mode Register (CCMR1/CCMR2) for Input Capture
    // CCxS bits (1:0 for CH1, 9:8 for CH2 in CCMR1)
    // 01: CCx channel is configured as input, ICx is mapped on TIxFPx
    // ICxPSC bits (3:2 for CH1, 11:10 for CH2) for input capture prescaler (capture every N events) - setting to 00 (no prescaler)
    // ICxF bits (7:4 for CH1, 15:12 for CH2) for input capture filter - setting to 0000 (no filter)
    uint32_t ccmr_input_config = (1U << 0); // CCxS = 01 (mapped on TIxFPx)
    volatile uint32_t *ccmr_reg = (channel_num == 1 || channel_num == 2) ? TIM->CCMR1 : TIM->CCMR2;
    uint32_t ccmr_shift = ((channel_num - 1) % 2) * 8; // 0 for CH1/3, 8 for CH2/4

    *ccmr_reg &= ~(0xFFU << ccmr_shift); // Clear CCxS, ICxPSC, ICxF
    *ccmr_reg |= (ccmr_input_config << ccmr_shift);

    // Configure polarity in CCER (Capture/Compare Enable Register)
    // CCxP bit (1 for CH1, 5 for CH2 etc.) for active low (falling edge). Default is active high (rising edge).
    // CCxNP bit (3 for CH1, 7 for CH2 etc.) combined with CCxP defines edge.
    // 00 = rising, 01 = falling, 11 = both
    uint32_t ccer_shift_ccx = (channel_num - 1) * 4;
    *TIM->CCER &= ~(3U << (ccer_shift_ccx + 1)); // Clear CCxP and CCxNP

    if (icu_edge == ICU_EDGE_RISING) {
        // Bits are 00 (default)
    } else if (icu_edge == ICU_EDGE_FALLING) {
        *TIM->CCER |= (1U << (ccer_shift_ccx + 1)); // Set CCxP to 1 for falling edge
    } else if (icu_edge == ICU_EDGE_BOTH_EDGES) {
        *TIM->CCER |= (3U << (ccer_shift_ccx + 1)); // Set CCxP and CCxNP for both edges
    }

    // Enable capture for the channel in CCER (CCxE bit)
    *TIM->CCER |= (1U << ccer_shift_ccx);

    // Generate update event to load PSC and ARR values
    *TIM->EGR |= (1U << 0); // UG bit
}

void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = GetTimerInstanceIndex(icu_channel);
    if (timer_idx == -1) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Enable Timer Counter (CEN bit in CR1)
    *TIM->CR1 |= (1U << 0); // CEN bit
}

void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = GetTimerInstanceIndex(icu_channel);
    if (timer_idx == -1) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Disable Timer Counter (CEN bit in CR1)
    *TIM->CR1 &= ~(1U << 0); // CEN bit
}

void ICU_Updatefrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This function can be used to trigger a re-measurement or refresh internal state.
    // For a simple MCAL, it could be a no-op if measurement is continuous.
    (void)icu_channel;
}

tlong ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This requires two captures (period measurement).
    // Requires timer interrupt to capture values or polling.
    // Not explicitly defined how the frequency is stored or computed from registers.
    // Placeholder, needs actual capture logic (e.g., store last two CCR values).
    (void)icu_channel;
    return 0; // Return dummy value
}

void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder for remote control buffer setup. This is application-specific.
    (void)number_of_keys;
    (void)key_digits_length;
}

void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder for remote control key digit storage.
    (void)key_num;
    (void)key_array_cell;
    (void)key_cell_value;
}

void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder for updating remote control signal parameters.
    (void)icu_channel;
    (void)strt_bit_us_value;
    (void)one_bit_us_value;
    (void)zero_bit_us_value;
    (void)stop_bit_us_value;
}

tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder for getting decoded remote control key.
    (void)icu_channel;
    return 0; // Dummy value
}

void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Placeholder for setting an interrupt callback.
    // Would typically be linked to a timer capture interrupt handler.
    (void)callback;
}

void ICU_ClearFlag(t_icu_channel icu_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = GetTimerInstanceIndex(icu_channel);
    if (timer_idx == -1) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];
    // Clear timer status flags, e.g., capture/compare flag.
    // Typically achieved by reading SR then CCRx for CCIF flag or writing 0 to bit if supported.
    *TIM->SR = 0x00000000; // Clear all status flags for simplicity (might not be correct for all flags)
}


// --- Timer API ---

// Re-using TimerRegs_t and helper functions

void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    int timer_idx = (int)timer_channel; // t_timer_channel enum maps directly to Timer_Instances array index
    if (timer_idx < 0 || timer_idx >= sizeof(Timer_Instances) / sizeof(TimerRegs_t)) return;

    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Enable Timer Clock (as per peripheral_enable_rules)
    EnableTimerClock(timer_idx);

    // Stop timer before configuration
    *TIM->CR1 &= ~(1U << 0); // CEN bit

    // Configure basic timer settings:
    // Direction: Up-counting (DIR bit 4 in CR1 = 0)
    *TIM->CR1 &= ~(1U << 4);
    // Auto-reload preload enable (ARPE bit 7 in CR1)
    *TIM->CR1 |= (1U << 7);
    // Clock division (CKD[1:0] bits 9:8 in CR1) to 0 (no division)
    *TIM->CR1 &= ~(3U << 8);

    // Set initial prescaler and auto-reload to default max values (for flexibility)
    *TIM->PSC = 0xFFFF;
    *TIM->ARR = 0xFFFF;

    // Generate update event to load initial PSC and ARR values
    *TIM->EGR |= (1U << 0); // UG bit
}

void TIMER_Set_us(t_timer_channel timer_channel, tword time_us) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    int timer_idx = (int)timer_channel;
    if (timer_idx < 0 || timer_idx >= sizeof(Timer_Instances) / sizeof(TimerRegs_t)) return;

    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    tlong timer_clk_freq = 0;
    if (timer_idx == 0 || (timer_idx >= 5 && timer_idx <= 7)) { // TIM1, TIM9-11 are on APB2
        timer_clk_freq = 32000000; // Assuming APB2 clock is 32MHz
    } else { // TIM2-5 are on APB1
        timer_clk_freq = 16000000; // Assuming APB1 clock is 16MHz
    }

    // Calculate PSC and ARR for microsecond resolution
    // Counter Clock Frequency = Timer Clock Frequency / (PSC + 1)
    // Desired counts = Counter Clock Frequency * time_s = F_counter * time_us / 1000000
    // ARR = Desired counts - 1
    // Let F_counter = 1MHz for microsecond accuracy, so PSC = (Timer Clock / 1MHz) - 1
    tword prescaler = (tword)((timer_clk_freq / 1000000) - 1); // For 1us tick
    tword auto_reload = time_us - 1;

    // Handle cases where calculated values exceed register limits
    if (prescaler > 0xFFFF) prescaler = 0xFFFF;
    if (auto_reload > 0xFFFF) auto_reload = 0xFFFF; // Max reload, may not achieve desired us if too large

    *TIM->PSC = prescaler;
    *TIM->ARR = auto_reload;

    // Generate update event
    *TIM->EGR |= (1U << 0); // UG bit
}

void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time_ms) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Reuse TIMER_Set_us by converting ms to us
    TIMER_Set_us(timer_channel, time_ms * 1000);
}

void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time_sec) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Reuse TIMER_Set_ms by converting sec to ms
    TIMER_Set_Time_ms(timer_channel, (tword)time_sec * 1000);
}

void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time_min) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Reuse TIMER_Set_Time_sec by converting min to sec
    TIMER_Set_Time_sec(timer_channel, time_min * 60);
}

void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time_hour) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Reuse TIMER_Set_Time_min by converting hour to min
    TIMER_Set_Time_min(timer_channel, time_hour * 60);
}

void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = (int)timer_channel;
    if (timer_idx < 0 || timer_idx >= sizeof(Timer_Instances) / sizeof(TimerRegs_t)) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Enable Timer Clock (as per peripheral_enable_rules)
    EnableTimerClock(timer_idx);

    // Enable Timer Counter (CEN bit in CR1)
    *TIM->CR1 |= (1U << 0); // CEN bit
}

void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = (int)timer_channel;
    if (timer_idx < 0 || timer_idx >= sizeof(Timer_Instances) / sizeof(TimerRegs_t)) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Disable Timer Counter (CEN bit in CR1)
    *TIM->CR1 &= ~(1U << 0); // CEN bit
}

void TIMER_ClearFlag(t_timer_channel timer_channel) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    int timer_idx = (int)timer_channel;
    if (timer_idx < 0 || timer_idx >= sizeof(Timer_Instances) / sizeof(TimerRegs_t)) return;
    TimerRegs_t *TIM = &Timer_Instances[timer_idx];

    // Clear Update Interrupt Flag (UIF bit 0 in SR)
    *TIM->SR &= ~(1U << 0); // Clear UIF
}


// --- ADC API ---

void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    (void)adc_channel; // Only one ADC (ADC1) in provided register_json

    // Enable ADC peripheral clock (as per peripheral_enable_rules)
    *RCC_APB2ENR |= (1U << 8); // Inferred: ADC1EN bit 8

    // Reset ADC (not explicit, typically ADC_CR2 ADON bit can be cleared and set, or software reset bit if available)
    *ADC_CR2 &= ~(1U << 0); // Clear ADON (ADC ON) bit to ensure it's off before config

    // Configure conversion mode (CR2, CONT bit for continuous)
    if (adc_mode == ADC_MODE_CONTINUOUS_CONVERSION) {
        *ADC_CR2 |= (1U << 1); // CONT bit
    } else { // Single conversion
        *ADC_CR2 &= ~(1U << 1); // CONT bit
    }

    // Configure data alignment (CR2, ALIGN bit) - right alignment default (0)
    *ADC_CR2 &= ~(1U << 11);

    // Set resolution (CR1, RES[1:0] bits 25:24) - default 12-bit (00)
    *ADC_CR1 &= ~(3U << 24);

    // Configure regular sequence length (SQR1, L[3:0] bits 23:20) - default 1 conversion
    *ADC_SQR1 &= ~(0xFUL << 20); // Clear L bits (set to 0 for 1 conversion)

    // Configure sample time for a channel (SMPR1/SMPR2)
    // Using a sample time of 3 cycles (shortest for basic operation)
    // Example for channel 0 (PA0) on SMPR2, bits 2:0
    *ADC_SMPR2 &= ~(7U << (0 * 3)); // Clear SMP0[2:0]
    *ADC_SMPR2 |= (0U << (0 * 3)); // Set 3 cycles (000)

    // Set common ADC settings (ADC_CCR)
    *ADC_CCR &= ~(3U << 16); // Clear ADCPRE (prescaler) - default PCLK2/2
    *ADC_CCR &= ~(3U << 30); // Clear TSVREFE (Temp sensor and VrefInt enable)
}

void ADC_Enable(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Enable peripheral clock first (as per peripheral_enable_rules)
    *RCC_APB2ENR |= (1U << 8); // Inferred: ADC1EN bit 8

    // Enable ADC (ADON bit in CR2)
    *ADC_CR2 |= (1U << 0); // ADON bit

    // Wait for ADC ready (ADRDY flag, not explicitly in JSON, common STM32)
    // Instead, a simple delay or wait for first conversion.
}

void ADC_Disable(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Disable ADC (ADON bit in CR2)
    *ADC_CR2 &= ~(1U << 0); // ADON bit

    // Optionally disable peripheral clock to save power
    *RCC_APB2ENR &= ~(1U << 8);
}

void ADC_Update(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This function can trigger a software start or check for completion.
    // For single conversion mode, trigger SWSTART (SWSTART bit 30 in CR2)
    *ADC_CR2 |= (1U << 30); // SWSTART bit
}

tword ADC_Get(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Wait for EOC (End Of Conversion) flag in SR
    while (!(*ADC_SR & (1U << 1))); // EOC flag
    // Read conversion result from DR
    return (tword)(*ADC_DR);
}

void ADC_ClearFlag(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Clear EOC (End Of Conversion) flag by writing 0 to it
    *ADC_SR &= ~(1U << 1); // Clear EOC
    // Clear OVR (Overrun) flag by writing 0 to it
    *ADC_SR &= ~(1U << 5); // Clear OVR
}

// --- Internal_EEPROM API ---

void Internal_EEPROM_Init(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Internal EEPROM is typically implemented using Flash memory for STM32F4.
    // Initialization involves unlocking Flash Controller and configuring latency.
    // Unlock Flash (FLASH_KEYR)
    // For STM32, there's a specific sequence of writing keys to FLASH_KEYR
    // This would require specific key values not present in JSON.
    // Placeholder as specific Flash programming details are not in register_json.
    // Latency setting (FLASH_ACR)
    *FLASH_ACR = (*FLASH_ACR & ~0xFUL) | (5UL << 0); // Example: 5 wait states for 84MHz (inferred)
    // Enable ART accelerator (ARTEN bit 9 in FLASH_ACR) and Prefetch buffer (PRFTEN bit 8 in FLASH_ACR)
    *FLASH_ACR |= ( (1U << 9) | (1U << 8) );
}

void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Writing to Flash requires specific programming steps:
    // 1. Check busy flag (BSY in FLASH_SR)
    // 2. Unlock FLASH_CR with FLASH_KEYR
    // 3. Select program operation (PG bit in FLASH_CR)
    // 4. Write data to address
    // 5. Wait for BSY cleared
    // 6. Check EOP (End of Program) flag in FLASH_SR and clear it.
    // This is a high-level placeholder as concrete programming steps/keys are missing.
    (void)address;
    (void)data;
    // Example sequence (conceptual):
    // while (*FLASH_SR & (1U << 16)); // Wait for BSY
    // *FLASH_KEYR = 0x45670123; // Key 1 (inferred)
    // *FLASH_KEYR = 0xCDEF89AB; // Key 2 (inferred)
    // *FLASH_CR |= (1U << 0); // Set PG bit (Program)
    // *(volatile tbyte*)(FLASH_BASE_ADDRESS + address) = data; // Write data (inferred FLASH_BASE_ADDRESS)
    // while (*FLASH_SR & (1U << 16)); // Wait for BSY
    // *FLASH_SR |= (1U << 0); // Clear EOP
}

tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // Reading from Flash is typically direct memory access after configuration.
    // Assuming FLASH_BASE_ADDRESS is 0x08000000 (inferred for code flash).
    // EEPROM emulation usually uses a dedicated flash sector.
    (void)address; // Not used without a base address.
    // tbyte data = *(volatile tbyte*)(FLASH_BASE_ADDRESS + address); // Inferred.
    return 0; // Dummy value
}

// --- TT API ---

// For TT, typically a general purpose timer like TIM2, TIM3, etc. is used.
// Let's use TIM2 for TT as an example, since it's a general purpose timer.
static void (*TT_Callback)(void) = NULL;
static tword TT_Period = 0;
static tword TT_Delay = 0;
static bool TT_TaskActive = false;

void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule

    // Use TIM2 for Time Triggered OS tick
    TIMER_Init(TIMER_CHANNEL_TIM2);
    TIMER_Set_Time_ms(TIMER_CHANNEL_TIM2, tick_time_ms);

    // Enable Update Interrupt for TIM2
    // DIER register, UIE bit 0
    *TIM2_DIER |= (1U << 0); // UIE bit (Update Interrupt Enable)

    // Enable Timer Interrupt in NVIC (inferred)
    // NVIC_EnableIRQ(TIM2_IRQn); // Needs CMSIS/MCU specific defines
    // Placeholder as NVIC registers are not in register_json.
}

void TT_Start(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    TIMER_Enable(TIMER_CHANNEL_TIM2);
}

void TT_Dispatch_task(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This function would typically check a flag set by the ISR and execute tasks.
    // In a simple model, if a task is "added", it implies it can be run.
    if (TT_TaskActive && TT_Callback != NULL) {
        TT_Callback(); // Execute the added task
        TT_TaskActive = false; // Reset for next period if not repetitive
    }
}

void TT_ISR(void) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // This is the Interrupt Service Routine for the Timer used for TT.
    // Clear the update interrupt flag
    TIMER_ClearFlag(TIMER_CHANNEL_TIM2);

    // Check if there is a task to dispatch
    if (TT_Callback != NULL) {
        // Simple one-shot task dispatch for demonstration.
        // A real TT OS would manage a list of tasks with periods/delays.
        TT_TaskActive = true;
    }
}

tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // For this simple implementation, only one task can be "added"
    // A more complex TT OS would manage a task list.
    if (TT_Callback == NULL) {
        TT_Callback = task;
        TT_Period = period; // Period and delay are for a full TT OS scheduler.
        TT_Delay = delay;
        TT_TaskActive = true; // Indicate task is ready to run
        return 0; // Success, task index 0
    }
    return 0xFF; // Failed, no space
}

void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // Adhering to API_implementation_sequence rule
    // For this simple implementation, only task 0 can be deleted.
    if (task_index == 0) {
        TT_Callback = NULL;
        TT_Period = 0;
        TT_Delay = 0;
        TT_TaskActive = false;
    }
}