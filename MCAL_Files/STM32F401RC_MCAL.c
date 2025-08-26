/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) Source File
 *
 * This file contains the implementation of the Microcontroller Abstraction Layer (MCAL)
 * for the STM32F401RC microcontroller, based on the provided register definitions,
 * API specifications, and coding rules.
 *
 * Developed according to MISRA C and CERT-C coding standards.
 *
 * @date YYYY-MM-DD (placeholder)
 */

#include "MCAL.h"

// CMSIS header for intrinsic functions like __enable_irq, __disable_irq, __WFI
#include "core_cm4.h"

/*
 * ===================================================================================================
 *                                  Register Definitions (Memory-mapped)
 * ===================================================================================================
 */

// Flash Registers
#define FLASH_ACR       (*(volatile tlong *)0x40023C00) /**< Flash access control register */
#define FLASH_KEYR      (*(volatile tlong *)0x40023C04) /**< Flash key register */
#define FLASH_OPTKEYR   (*(volatile tlong *)0x40023C08) /**< Flash option key register */
#define FLASH_SR        (*(volatile tlong *)0x40023C0C) /**< Flash status register */
#define FLASH_CR        (*(volatile tlong *)0x40023C10) /**< Flash control register */
#define FLASH_OPTCR     (*(volatile tlong *)0x40023C14) /**< Flash option control register */

// CRC Registers
#define CRC_DR          (*(volatile tlong *)0x40023000) /**< Data register for CRC calculation unit */
#define CRC_IDR         (*(volatile tlong *)0x40023004) /**< Independent data register for CRC calculation unit */
#define CRC_CR          (*(volatile tlong *)0x40023008) /**< Control register for CRC calculation unit */

// PWR Registers
#define PWR_CR          (*(volatile tlong *)0x40007000) /**< Power control register */
#define PWR_CSR         (*(volatile tlong *)0x40007004) /**< Power control/status register */

// RCC Registers
#define RCC_CR          (*(volatile tlong *)0x40023800) /**< Clock control register */
#define RCC_PLLCFGR     (*(volatile tlong *)0x40023804) /**< PLL configuration register */
#define RCC_CFGR        (*(volatile tlong *)0x40023808) /**< Clock configuration register */
#define RCC_CIR         (*(volatile tlong *)0x4002380C) /**< Clock interrupt register */
#define RCC_AHB1RSTR    (*(volatile tlong *)0x40023810) /**< AHB1 peripheral reset register */
#define RCC_AHB2RSTR    (*(volatile tlong *)0x40023814) /**< AHB2 peripheral reset register */
#define RCC_APB1RSTR    (*(volatile tlong *)0x40023818) /**< APB1 peripheral reset register */
#define RCC_APB2RSTR    (*(volatile tlong *)0x4002381C) /**< APB2 peripheral reset register */
#define RCC_AHB1ENR     (*(volatile tlong *)0x40023830) /**< AHB1 peripheral clock enable register */
#define RCC_AHB2ENR     (*(volatile tlong *)0x40023834) /**< AHB2 peripheral clock enable register */
#define RCC_APB1ENR     (*(volatile tlong *)0x40023838) /**< APB1 peripheral clock enable register */
#define RCC_APB2ENR     (*(volatile tlong *)0x4002383C) /**< APB2 peripheral clock enable register */
#define RCC_AHB1LPENR   (*(volatile tlong *)0x40023850) /**< AHB1 peripheral clock enable in low power mode register */
#define RCC_AHB2LPENR   (*(volatile tlong *)0x40023854) /**< AHB2 peripheral clock enable in low power mode register */
#define RCC_APB1LPENR   (*(volatile tlong *)0x40023858) /**< APB1 peripheral clock enable in low power mode register */
#define RCC_APB2LPENR   (*(volatile tlong *)0x4002385C) /**< APB2 peripheral clock enabled in low power mode register */
#define RCC_BDCR        (*(volatile tlong *)0x40023870) /**< Backup domain control register */
#define RCC_CSR         (*(volatile tlong *)0x40023874) /**< Clock control & status register */
#define RCC_SSCGR       (*(volatile tlong *)0x40023880) /**< Spread spectrum clock generation register */
#define RCC_PLLI2SCFGR  (*(volatile tlong *)0x40023884) /**< PLLI2S configuration register */
#define RCC_DCKCFGR     (*(volatile tlong *)0x4002388C) /**< Dedicated Clocks Configuration Register */

// SYSCFG Registers
#define SYSCFG_MEMRMP   (*(volatile tlong *)0x40013800) /**< Memory remap register */
#define SYSCFG_PMC      (*(volatile tlong *)0x40013804) /**< Peripheral mode configuration register */
#define SYSCFG_EXTICR1  (*(volatile tlong *)0x40013808) /**< External interrupt configuration register 1 (EXTI lines 0-3) */
#define SYSCFG_EXTICR2  (*(volatile tlong *)0x4001380C) /**< External interrupt configuration register 2 (EXTI lines 4-7) */
#define SYSCFG_EXTICR3  (*(volatile tlong *)0x40013810) /**< External interrupt configuration register 3 (EXTI lines 8-11) */
#define SYSCFG_EXTICR4  (*(volatile tlong *)0x40013814) /**< External interrupt configuration register 4 (EXTI lines 12-15) */
#define SYSCFG_CMPCR    (*(volatile tlong *)0x40013820) /**< Compensation cell control register */

// GPIO Registers
#define GPIOA_MODER     (*(volatile tlong *)0x40020000) /**< GPIO port mode register for Port A */
#define GPIOA_OTYPER    (*(volatile tlong *)0x40020004) /**< GPIO port output type register for Port A */
#define GPIOA_OSPEEDR   (*(volatile tlong *)0x40020008) /**< GPIO port output speed register for Port A */
#define GPIOA_PUPDR     (*(volatile tlong *)0x4002000C) /**< GPIO port pull-up/pull-down register for Port A */
#define GPIOA_IDR       (*(volatile tlong *)0x40020010) /**< GPIO port input data register for Port A */
#define GPIOA_ODR       (*(volatile tlong *)0x40020014) /**< GPIO port output data register for Port A */
#define GPIOA_BSRR      (*(volatile tlong *)0x40020018) /**< GPIO port bit set/reset register for Port A */
#define GPIOA_LCKR      (*(volatile tlong *)0x4002001C) /**< GPIO port configuration lock register for Port A */
#define GPIOA_AFRL      (*(volatile tlong *)0x40020020) /**< GPIO alternate function low register for Port A (pins 0-7) */
#define GPIOA_AFRH      (*(volatile tlong *)0x40020024) /**< GPIO alternate function high register for Port A (pins 8-15) */

#define GPIOB_MODER     (*(volatile tlong *)0x40020400) /**< GPIO port mode register for Port B */
#define GPIOB_OTYPER    (*(volatile tlong *)0x40020404) /**< GPIO port output type register for Port B */
#define GPIOB_OSPEEDR   (*(volatile tlong *)0x40020408) /**< GPIO port output speed register for Port B */
#define GPIOB_PUPDR     (*(volatile tlong *)0x4002040C) /**< GPIO port pull-up/pull-down register for Port B */
#define GPIOB_IDR       (*(volatile tlong *)0x40020410) /**< GPIO port input data register for Port B */
#define GPIOB_ODR       (*(volatile tlong *)0x40020414) /**< GPIO port output data register for Port B */
#define GPIOB_BSRR      (*(volatile tlong *)0x40020418) /**< GPIO port bit set/reset register for Port B */
#define GPIOB_LCKR      (*(volatile tlong *)0x4002041C) /**< GPIO port configuration lock register for Port B */
#define GPIOB_AFRL      (*(volatile tlong *)0x40020420) /**< GPIO alternate function low register for Port B (pins 0-7) */
#define GPIOB_AFRH      (*(volatile tlong *)0x40020424) /**< GPIO alternate function high register for Port B (pins 8-15) */

#define GPIOC_MODER     (*(volatile tlong *)0x40020800) /**< GPIO port mode register for Port C */
#define GPIOC_OTYPER    (*(volatile tlong *)0x40020804) /**< GPIO port output type register for Port C */
#define GPIOC_OSPEEDR   (*(volatile tlong *)0x40020808) /**< GPIO port output speed register for Port C */
#define GPIOC_PUPDR     (*(volatile tlong *)0x4002080C) /**< GPIO port pull-up/pull-down register for Port C */
#define GPIOC_IDR       (*(volatile tlong *)0x40020810) /**< GPIO port input data register for Port C */
#define GPIOC_ODR       (*(volatile tlong *)0x40020814) /**< GPIO port output data register for Port C */
#define GPIOC_BSRR      (*(volatile tlong *)0x40020818) /**< GPIO port bit set/reset register for Port C */
#define GPIOC_LCKR      (*(volatile tlong *)0x4002081C) /**< GPIO port configuration lock register for Port C */
#define GPIOC_AFRL      (*(volatile tlong *)0x40020820) /**< GPIO alternate function low register for Port C (pins 0-7) */
#define GPIOC_AFRH      (*(volatile tlong *)0x40020824) /**< GPIO alternate function high register for Port C (pins 8-15) */

#define GPIOD_MODER     (*(volatile tlong *)0x40020C00) /**< GPIO port mode register for Port D */
#define GPIOD_OTYPER    (*(volatile tlong *)0x40020C04) /**< GPIO port output type register for Port D */
#define GPIOD_OSPEEDR   (*(volatile tlong *)0x40020C08) /**< GPIO port output speed register for Port D */
#define GPIOD_PUPDR     (*(volatile tlong *)0x40020C0C) /**< GPIO port pull-up/pull-down register for Port D */
#define GPIOD_IDR       (*(volatile tlong *)0x40020C10) /**< GPIO port input data register for Port D */
#define GPIOD_ODR       (*(volatile tlong *)0x40020C14) /**< GPIO port output data register for Port D */
#define GPIOD_BSRR      (*(volatile tlong *)0x40020C18) /**< GPIO port bit set/reset register for Port D */
#define GPIOD_LCKR      (*(volatile tlong *)0x40020C1C) /**< GPIO port configuration lock register for Port D */
#define GPIOD_AFRL      (*(volatile tlong *)0x40020C20) /**< GPIO alternate function low register for Port D (pins 0-7) */
#define GPIOD_AFRH      (*(volatile tlong *)0x40020C24) /**< GPIO alternate function high register for Port D (pins 8-15) */

#define GPIOE_MODER     (*(volatile tlong *)0x40021000) /**< GPIO port mode register for Port E */
#define GPIOE_OTYPER    (*(volatile tlong *)0x40021004) /**< GPIO port output type register for Port E */
#define GPIOE_OSPEEDR   (*(volatile tlong *)0x40021008) /**< GPIO port output speed register for Port E */
#define GPIOE_PUPDR     (*(volatile tlong *)0x4002100C) /**< GPIO port pull-up/pull-down register for Port E */
#define GPIOE_IDR       (*(volatile tlong *)0x40021010) /**< GPIO port input data register for Port E */
#define GPIOE_ODR       (*(volatile tlong *)0x40021014) /**< GPIO port output data register for Port E */
#define GPIOE_BSRR      (*(volatile tlong *)0x40021018) /**< GPIO port bit set/reset register for Port E */
#define GPIOE_LCKR      (*(volatile tlong *)0x4002101C) /**< GPIO port configuration lock register for Port E */
#define GPIOE_AFRL      (*(volatile tlong *)0x40021020) /**< GPIO alternate function low register for Port E (pins 0-7) */
#define GPIOE_AFRH      (*(volatile tlong *)0x40021024) /**< GPIO alternate function high register for Port E (pins 8-15) */

#define GPIOH_MODER     (*(volatile tlong *)0x40021C00) /**< GPIO port mode register for Port H */
#define GPIOH_OTYPER    (*(volatile tlong *)0x40021C04) /**< GPIO port output type register for Port H */
#define GPIOH_OSPEEDR   (*(volatile tlong *)0x40021C08) /**< GPIO port output speed register for Port H */
#define GPIOH_PUPDR     (*(volatile tlong *)0x40021C0C) /**< GPIO port pull-up/pull-down register for Port H */
#define GPIOH_IDR       (*(volatile tlong *)0x40021C10) /**< GPIO port input data register for Port H */
#define GPIOH_ODR       (*(volatile tlong *)0x40021C14) /**< GPIO port output data register for Port H */
#define GPIOH_BSRR      (*(volatile tlong *)0x40021C18) /**< GPIO port bit set/reset register for Port H */
#define GPIOH_LCKR      (*(volatile tlong *)0x40021C1C) /**< GPIO port configuration lock register for Port H */
#define GPIOH_AFRL      (*(volatile tlong *)0x40021C20) /**< GPIO alternate function low register for Port H (pins 0-7) */
#define GPIOH_AFRH      (*(volatile tlong *)0x40021C24) /**< GPIO alternate function high register for Port H (pins 8-15) */

// DMA Registers (DMA1 and DMA2 streams 0-7)
// DMA1
#define DMA1_LISR       (*(volatile tlong *)0x40026000) /**< DMA1 low interrupt status register */
#define DMA1_HISR       (*(volatile tlong *)0x40026004) /**< DMA1 high interrupt status register */
#define DMA1_LIFCR      (*(volatile tlong *)0x40026008) /**< DMA1 low interrupt flag clear register */
#define DMA1_HIFCR      (*(volatile tlong *)0x4002600C) /**< DMA1 high interrupt flag clear register */
#define DMA1_S0CR       (*(volatile tlong *)0x40026010) /**< DMA1 stream 0 configuration register */
#define DMA1_S0NDTR     (*(volatile tlong *)0x40026014) /**< DMA1 stream 0 number of data register */
#define DMA1_S0PAR      (*(volatile tlong *)0x40026018) /**< DMA1 stream 0 peripheral address register */
#define DMA1_S0M0AR     (*(volatile tlong *)0x4002601C) /**< DMA1 stream 0 memory 0 address register */
#define DMA1_S0M1AR     (*(volatile tlong *)0x40026020) /**< DMA1 stream 0 memory 1 address register */
#define DMA1_S0FCR      (*(volatile tlong *)0x40026024) /**< DMA1 stream 0 FIFO control register */
#define DMA1_S1CR       (*(volatile tlong *)0x40026028) /**< DMA1 stream 1 configuration register */
#define DMA1_S1NDTR     (*(volatile tlong *)0x4002602C) /**< DMA1 stream 1 number of data register */
#define DMA1_S1PAR      (*(volatile tlong *)0x40026030) /**< DMA1 stream 1 peripheral address register */
#define DMA1_S1M0AR     (*(volatile tlong *)0x40026034) /**< DMA1 stream 1 memory 0 address register */
#define DMA1_S1M1AR     (*(volatile tlong *)0x40026038) /**< DMA1 stream 1 memory 1 address register */
#define DMA1_S1FCR      (*(volatile tlong *)0x4002603C) /**< DMA1 stream 1 FIFO control register */
#define DMA1_S2CR       (*(volatile tlong *)0x40026040) /**< DMA1 stream 2 configuration register */
#define DMA1_S2NDTR     (*(volatile tlong *)0x40026044) /**< DMA1 stream 2 number of data register */
#define DMA1_S2PAR      (*(volatile tlong *)0x40026048) /**< DMA1 stream 2 peripheral address register */
#define DMA1_S2M0AR     (*(volatile tlong *)0x4002604C) /**< DMA1 stream 2 memory 0 address register */
#define DMA1_S2M1AR     (*(volatile tlong *)0x40026050) /**< DMA1 stream 2 memory 1 address register */
#define DMA1_S2FCR      (*(volatile tlong *)0x40026054) /**< DMA1 stream 2 FIFO control register */
#define DMA1_S3CR       (*(volatile tlong *)0x40026058) /**< DMA1 stream 3 configuration register */
#define DMA1_S3NDTR     (*(volatile tlong *)0x4002605C) /**< DMA1 stream 3 number of data register */
#define DMA1_S3PAR      (*(volatile tlong *)0x40026060) /**< DMA1 stream 3 peripheral address register */
#define DMA1_S3M0AR     (*(volatile tlong *)0x40026064) /**< DMA1 stream 3 memory 0 address register */
#define DMA1_S3M1AR     (*(volatile tlong *)0x40026068) /**< DMA1 stream 3 memory 1 address register */
#define DMA1_S3FCR      (*(volatile tlong *)0x4002606C) /**< DMA1 stream 3 FIFO control register */
#define DMA1_S4CR       (*(volatile tlong *)0x40026070) /**< DMA1 stream 4 configuration register */
#define DMA1_S4NDTR     (*(volatile tlong *)0x40026074) /**< DMA1 stream 4 number of data register */
#define DMA1_S4PAR      (*(volatile tlong *)0x40026078) /**< DMA1 stream 4 peripheral address register */
#define DMA1_S4M0AR     (*(volatile tlong *)0x4002607C) /**< DMA1 stream 4 memory 0 address register */
#define DMA1_S4M1AR     (*(volatile tlong *)0x40026080) /**< DMA1 stream 4 memory 1 address register */
#define DMA1_S4FCR      (*(volatile tlong *)0x40026084) /**< DMA1 stream 4 FIFO control register */
#define DMA1_S5CR       (*(volatile tlong *)0x40026088) /**< DMA1 stream 5 configuration register */
#define DMA1_S5NDTR     (*(volatile tlong *)0x4002608C) /**< DMA1 stream 5 number of data register */
#define DMA1_S5PAR      (*(volatile tlong *)0x40026090) /**< DMA1 stream 5 peripheral address register */
#define DMA1_S5M0AR     (*(volatile tlong *)0x40026094) /**< DMA1 stream 5 memory 0 address register */
#define DMA1_S5M1AR     (*(volatile tlong *)0x40026098) /**< DMA1 stream 5 memory 1 address register */
#define DMA1_S5FCR      (*(volatile tlong *)0x4002609C) /**< DMA1 stream 5 FIFO control register */
#define DMA1_S6CR       (*(volatile tlong *)0x400260A0) /**< DMA1 stream 6 configuration register */
#define DMA1_S6NDTR     (*(volatile tlong *)0x400260A4) /**< DMA1 stream 6 number of data register */
#define DMA1_S6PAR      (*(volatile tlong *)0x400260A8) /**< DMA1 stream 6 peripheral address register */
#define DMA1_S6M0AR     (*(volatile tlong *)0x400260AC) /**< DMA1 stream 6 memory 0 address register */
#define DMA1_S6M1AR     (*(volatile tlong *)0x400260B0) /**< DMA1 stream 6 memory 1 address register */
#define DMA1_S6FCR      (*(volatile tlong *)0x400260B4) /**< DMA1 stream 6 FIFO control register */
#define DMA1_S7CR       (*(volatile tlong *)0x400260B8) /**< DMA1 stream 7 configuration register */
#define DMA1_S7NDTR     (*(volatile tlong *)0x400260BC) /**< DMA1 stream 7 number of data register */
#define DMA1_S7PAR      (*(volatile tlong *)0x400260C0) /**< DMA1 stream 7 peripheral address register */
#define DMA1_S7M0AR     (*(volatile tlong *)0x400260C4) /**< DMA1 stream 7 memory 0 address register */
#define DMA1_S7M1AR     (*(volatile tlong *)0x400260C8) /**< DMA1 stream 7 memory 1 address register */
#define DMA1_S7FCR      (*(volatile tlong *)0x400260CC) /**< DMA1 stream 7 FIFO control register */

// DMA2
#define DMA2_LISR       (*(volatile tlong *)0x40026400) /**< DMA2 low interrupt status register */
#define DMA2_HISR       (*(volatile tlong *)0x40026404) /**< DMA2 high interrupt status register */
#define DMA2_LIFCR      (*(volatile tlong *)0x40026408) /**< DMA2 low interrupt flag clear register */
#define DMA2_HIFCR      (*(volatile tlong *)0x4002640C) /**< DMA2 high interrupt flag clear register */
#define DMA2_S0CR       (*(volatile tlong *)0x40026410) /**< DMA2 stream 0 configuration register */
#define DMA2_S0NDTR     (*(volatile tlong *)0x40026414) /**< DMA2 stream 0 number of data register */
#define DMA2_S0PAR      (*(volatile tlong *)0x40026418) /**< DMA2 stream 0 peripheral address register */
#define DMA2_S0M0AR     (*(volatile tlong *)0x4002641C) /**< DMA2 stream 0 memory 0 address register */
#define DMA2_S0M1AR     (*(volatile tlong *)0x40026420) /**< DMA2 stream 0 memory 1 address register */
#define DMA2_S0FCR      (*(volatile tlong *)0x40026424) /**< DMA2 stream 0 FIFO control register */
#define DMA2_S1CR       (*(volatile tlong *)0x40026428) /**< DMA2 stream 1 configuration register */
#define DMA2_S1NDTR     (*(volatile tlong *)0x4002642C) /**< DMA2 stream 1 number of data register */
#define DMA2_S1PAR      (*(volatile tlong *)0x40026430) /**< DMA2 stream 1 peripheral address register */
#define DMA2_S1M0AR     (*(volatile tlong *)0x40026434) /**< DMA2 stream 1 memory 0 address register */
#define DMA2_S1M1AR     (*(volatile tlong *)0x40026438) /**< DMA2 stream 1 memory 1 address register */
#define DMA2_S1FCR      (*(volatile tlong *)0x4002643C) /**< DMA2 stream 1 FIFO control register */
#define DMA2_S2CR       (*(volatile tlong *)0x40026440) /**< DMA2 stream 2 configuration register */
#define DMA2_S2NDTR     (*(volatile tlong *)0x40026444) /**< DMA2 stream 2 number of data register */
#define DMA2_S2PAR      (*(volatile tlong *)0x40026448) /**< DMA2 stream 2 peripheral address register */
#define DMA2_S2M0AR     (*(volatile tlong *)0x4002644C) /**< DMA2 stream 2 memory 0 address register */
#define DMA2_S2M1AR     (*(volatile tlong *)0x40026450) /**< DMA2 stream 2 memory 1 address register */
#define DMA2_S2FCR      (*(volatile tlong *)0x40026454) /**< DMA2 stream 2 FIFO control register */
#define DMA2_S3CR       (*(volatile tlong *)0x40026458) /**< DMA2 stream 3 configuration register */
#define DMA2_S3NDTR     (*(volatile tlong *)0x4002645C) /**< DMA2 stream 3 number of data register */
#define DMA2_S3PAR      (*(volatile tlong *)0x40026460) /**< DMA2 stream 3 peripheral address register */
#define DMA2_S3M0AR     (*(volatile tlong *)0x40026464) /**< DMA2 stream 3 memory 0 address register */
#define DMA2_S3M1AR     (*(volatile tlong *)0x40026468) /**< DMA2 stream 3 memory 1 address register */
#define DMA2_S3FCR      (*(volatile tlong *)0x4002646C) /**< DMA2 stream 3 FIFO control register */
#define DMA2_S4CR       (*(volatile tlong *)0x40026470) /**< DMA2 stream 4 configuration register */
#define DMA2_S4NDTR     (*(volatile tlong *)0x40026474) /**< DMA2 stream 4 number of data register */
#define DMA2_S4PAR      (*(volatile tlong *)0x40026478) /**< DMA2 stream 4 peripheral address register */
#define DMA2_S4M0AR     (*(volatile tlong *)0x4002647C) /**< DMA2 stream 4 memory 0 address register */
#define DMA2_S4M1AR     (*(volatile tlong *)0x40026480) /**< DMA2 stream 4 memory 1 address register */
#define DMA2_S4FCR      (*(volatile tlong *)0x40026484) /**< DMA2 stream 4 FIFO control register */
#define DMA2_S5CR       (*(volatile tlong *)0x40026488) /**< DMA2 stream 5 configuration register */
#define DMA2_S5NDTR     (*(volatile tlong *)0x4002648C) /**< DMA2 stream 5 number of data register */
#define DMA2_S5PAR      (*(volatile tlong *)0x40026490) /**< DMA2 stream 5 peripheral address register */
#define DMA2_S5M0AR     (*(volatile tlong *)0x40026494) /**< DMA2 stream 5 memory 0 address register */
#define DMA2_S5M1AR     (*(volatile tlong *)0x40026498) /**< DMA2 stream 5 memory 1 address register */
#define DMA2_S5FCR      (*(volatile tlong *)0x4002649C) /**< DMA2 stream 5 FIFO control register */
#define DMA2_S6CR       (*(volatile tlong *)0x400264A0) /**< DMA2 stream 6 configuration register */
#define DMA2_S6NDTR     (*(volatile tlong *)0x400264A4) /**< DMA2 stream 6 number of data register */
#define DMA2_S6PAR      (*(volatile tlong *)0x400264A8) /**< DMA2 stream 6 peripheral address register */
#define DMA2_S6M0AR     (*(volatile tlong *)0x400264AC) /**< DMA2 stream 6 memory 0 address register */
#define DMA2_S6M1AR     (*(volatile tlong *)0x400264B0) /**< DMA2 stream 6 memory 1 address register */
#define DMA2_S6FCR      (*(volatile tlong *)0x400264B4) /**< DMA2 stream 6 FIFO control register */
#define DMA2_S7CR       (*(volatile tlong *)0x400264B8) /**< DMA2 stream 7 configuration register */
#define DMA2_S7NDTR     (*(volatile tlong *)0x400264BC) /**< DMA2 stream 7 number of data register */
#define DMA2_S7PAR      (*(volatile tlong *)0x400264C0) /**< DMA2 stream 7 peripheral address register */
#define DMA2_S7M0AR     (*(volatile tlong *)0x400264C4) /**< DMA2 stream 7 memory 0 address register */
#define DMA2_S7M1AR     (*(volatile tlong *)0x400264C8) /**< DMA2 stream 7 memory 1 address register */
#define DMA2_S7FCR      (*(volatile tlong *)0x400264CC) /**< DMA2 stream 7 FIFO control register */

// EXTI Registers
#define EXTI_IMR        (*(volatile tlong *)0x40013C00) /**< Interrupt mask register for EXTI lines */
#define EXTI_EMR        (*(volatile tlong *)0x40013C04) /**< Event mask register for EXTI lines */
#define EXTI_RTSR       (*(volatile tlong *)0x40013C08) /**< Rising trigger selection register for EXTI lines */
#define EXTI_FTSR       (*(volatile tlong *)0x40013C0C) /**< Falling trigger selection register for EXTI lines */
#define EXTI_SWIER      (*(volatile tlong *)0x40013C10) /**< Software interrupt event register for EXTI lines */
#define EXTI_PR         (*(volatile tlong *)0x40013C14) /**< Pending register for EXTI lines */

// ADC Registers
#define ADC_SR          (*(volatile tlong *)0x40012000) /**< ADC status register */
#define ADC_CR1         (*(volatile tlong *)0x40012004) /**< ADC control register 1 */
#define ADC_CR2         (*(volatile tlong *)0x40012008) /**< ADC control register 2 */
#define ADC_SMPR1       (*(volatile tlong *)0x4001200C) /**< ADC sample time register 1 (channels 10-18) */
#define ADC_SMPR2       (*(volatile tlong *)0x40012010) /**< ADC sample time register 2 (channels 0-9) */
#define ADC_JOFR1       (*(volatile tlong *)0x40012014) /**< ADC injected channel data offset register 1 */
#define ADC_JOFR2       (*(volatile tlong *)0x40012018) /**< ADC injected channel data offset register 2 */
#define ADC_JOFR3       (*(volatile tlong *)0x4001201C) /**< ADC injected channel data offset register 3 */
#define ADC_JOFR4       (*(volatile tlong *)0x40012020) /**< ADC injected channel data offset register 4 */
#define ADC_HTR         (*(volatile tlong *)0x40012024) /**< ADC watchdog higher threshold register */
#define ADC_LTR         (*(volatile tlong *)0x40012028) /**< ADC watchdog lower threshold register */
#define ADC_SQR1        (*(volatile tlong *)0x4001202C) /**< ADC regular sequence register 1 */
#define ADC_SQR2        (*(volatile tlong *)0x40012030) /**< ADC regular sequence register 2 */
#define ADC_SQR3        (*(volatile tlong *)0x40012034) /**< ADC regular sequence register 3 */
#define ADC_JSQR        (*(volatile tlong *)0x40012038) /**< ADC injected sequence register */
#define ADC_JDR1        (*(volatile tlong *)0x4001203C) /**< ADC injected data register 1 */
#define ADC_JDR2        (*(volatile tlong *)0x40012040) /**< ADC injected data register 2 */
#define ADC_JDR3        (*(volatile tlong *)0x40012044) /**< ADC injected data register 3 */
#define ADC_JDR4        (*(volatile tlong *)0x40012048) /**< ADC injected data register 4 */
#define ADC_DR          (*(volatile tlong *)0x4001204C) /**< ADC regular data register */
#define ADC_CCR         (*(volatile tlong *)0x40012300) /**< ADC common control register */

// TIMERS Registers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
// TIM1
#define TIM1_CR1        (*(volatile tlong *)0x40010000) /**< TIM1 control register 1 */
#define TIM1_CR2        (*(volatile tlong *)0x40010004) /**< TIM1 control register 2 */
#define TIM1_SMCR       (*(volatile tlong *)0x40010008) /**< TIM1 slave mode control register */
#define TIM1_DIER       (*(volatile tlong *)0x4001000C) /**< TIM1 DMA/interrupt enable register */
#define TIM1_SR         (*(volatile tlong *)0x40010010) /**< TIM1 status register */
#define TIM1_EGR        (*(volatile tlong *)0x40010014) /**< TIM1 event generation register */
#define TIM1_CCMR1      (*(volatile tlong *)0x40010018) /**< TIM1 capture/compare mode register 1 (channels 1 & 2) */
#define TIM1_CCMR2      (*(volatile tlong *)0x4001001C) /**< TIM1 capture/compare mode register 2 (channels 3 & 4) */
#define TIM1_CCER       (*(volatile tlong *)0x40010020) /**< TIM1 capture/compare enable register */
#define TIM1_CNT        (*(volatile tlong *)0x40010024) /**< TIM1 counter */
#define TIM1_PSC        (*(volatile tlong *)0x40010028) /**< TIM1 prescaler */
#define TIM1_ARR        (*(volatile tlong *)0x4001002C) /**< TIM1 auto-reload register */
#define TIM1_RCR        (*(volatile tlong *)0x40010030) /**< TIM1 repetition counter register */
#define TIM1_CCR1       (*(volatile tlong *)0x40010034) /**< TIM1 capture/compare register 1 */
#define TIM1_CCR2       (*(volatile tlong *)0x40010038) /**< TIM1 capture/compare register 2 */
#define TIM1_CCR3       (*(volatile tlong *)0x4001003C) /**< TIM1 capture/compare register 3 */
#define TIM1_CCR4       (*(volatile tlong *)0x40010040) /**< TIM1 capture/compare register 4 */
#define TIM1_BDTR       (*(volatile tlong *)0x40010044) /**< TIM1 break and dead-time register */
#define TIM1_DCR        (*(volatile tlong *)0x40010048) /**< TIM1 DMA control register */
#define TIM1_DMAR       (*(volatile tlong *)0x4001004C) /**< TIM1 DMA address for full transfer */
// TIM2
#define TIM2_CR1        (*(volatile tlong *)0x40000000) /**< TIM2 control register 1 */
#define TIM2_CR2        (*(volatile tlong *)0x40000004) /**< TIM2 control register 2 */
#define TIM2_SMCR       (*(volatile tlong *)0x40000008) /**< TIM2 slave mode control register */
#define TIM2_DIER       (*(volatile tlong *)0x4000000C) /**< TIM2 DMA/Interrupt enable register */
#define TIM2_SR         (*(volatile tlong *)0x40000010) /**< TIM2 status register */
#define TIM2_EGR        (*(volatile tlong *)0x40000014) /**< TIM2 event generation register */
#define TIM2_CCMR1      (*(volatile tlong *)0x40000018) /**< TIM2 capture/compare mode register 1 (channels 1 & 2) */
#define TIM2_CCMR2      (*(volatile tlong *)0x4000001C) /**< TIM2 capture/compare mode register 2 (channels 3 & 4) */
#define TIM2_CCER       (*(volatile tlong *)0x40000020) /**< TIM2 capture/compare enable register */
#define TIM2_CNT        (*(volatile tlong *)0x40000024) /**< TIM2 counter */
#define TIM2_PSC        (*(volatile tlong *)0x40000028) /**< TIM2 prescaler */
#define TIM2_ARR        (*(volatile tlong *)0x4000002C) /**< TIM2 auto-reload register */
#define TIM2_CCR1       (*(volatile tlong *)0x40000034) /**< TIM2 capture/compare register 1 */
#define TIM2_CCR2       (*(volatile tlong *)0x40000038) /**< TIM2 capture/compare register 2 */
#define TIM2_CCR3       (*(volatile tlong *)0x4000003C) /**< TIM2 capture/compare register 3 */
#define TIM2_CCR4       (*(volatile tlong *)0x40000040) /**< TIM2 capture/compare register 4 */
#define TIM2_DCR        (*(volatile tlong *)0x40000048) /**< TIM2 DMA control register */
#define TIM2_DMAR       (*(volatile tlong *)0x4000004C) /**< TIM2 DMA address for full transfer */
#define TIM2_OR         (*(volatile tlong *)0x40000050) /**< TIM2 option register */
// TIM3
#define TIM3_CR1        (*(volatile tlong *)0x40000400) /**< TIM3 control register 1 */
#define TIM3_CR2        (*(volatile tlong *)0x40000404) /**< TIM3 control register 2 */
#define TIM3_SMCR       (*(volatile tlong *)0x40000408) /**< TIM3 slave mode control register */
#define TIM3_DIER       (*(volatile tlong *)0x4000040C) /**< TIM3 DMA/Interrupt enable register */
#define TIM3_SR         (*(volatile tlong *)0x40000410) /**< TIM3 status register */
#define TIM3_EGR        (*(volatile tlong *)0x40000414) /**< TIM3 event generation register */
#define TIM3_CCMR1      (*(volatile tlong *)0x40000418) /**< TIM3 capture/compare mode register 1 (channels 1 & 2) */
#define TIM3_CCMR2      (*(volatile tlong *)0x4000041C) /**< TIM3 capture/compare mode register 2 (channels 3 & 4) */
#define TIM3_CCER       (*(volatile tlong *)0x40000420) /**< TIM3 capture/compare enable register */
#define TIM3_CNT        (*(volatile tlong *)0x40000424) /**< TIM3 counter */
#define TIM3_PSC        (*(volatile tlong *)0x40000428) /**< TIM3 prescaler */
#define TIM3_ARR        (*(volatile tlong *)0x4000042C) /**< TIM3 auto-reload register */
#define TIM3_CCR1       (*(volatile tlong *)0x40000434) /**< TIM3 capture/compare register 1 */
#define TIM3_CCR2       (*(volatile tlong *)0x40000438) /**< TIM3 capture/compare register 2 */
#define TIM3_CCR3       (*(volatile tlong *)0x4000043C) /**< TIM3 capture/compare register 3 */
#define TIM3_CCR4       (*(volatile tlong *)0x40000440) /**< TIM3 capture/compare register 4 */
#define TIM3_DCR        (*(volatile tlong *)0x40000448) /**< TIM3 DMA control register */
#define TIM3_DMAR       (*(volatile tlong *)0x4000044C) /**< TIM3 DMA address for full transfer */
// TIM4
#define TIM4_CR1        (*(volatile tlong *)0x40000800) /**< TIM4 control register 1 */
#define TIM4_CR2        (*(volatile tlong *)0x40000804) /**< TIM4 control register 2 */
#define TIM4_SMCR       (*(volatile tlong *)0x40000808) /**< TIM4 slave mode control register */
#define TIM4_DIER       (*(volatile tlong *)0x4000080C) /**< TIM4 DMA/Interrupt enable register */
#define TIM4_SR         (*(volatile tlong *)0x40000810) /**< TIM4 status register */
#define TIM4_EGR        (*(volatile tlong *)0x40000814) /**< TIM4 event generation register */
#define TIM4_CCMR1      (*(volatile tlong *)0x40000818) /**< TIM4 capture/compare mode register 1 (channels 1 & 2) */
#define TIM4_CCMR2      (*(volatile tlong *)0x4000081C) /**< TIM4 capture/compare mode register 2 (channels 3 & 4) */
#define TIM4_CCER       (*(volatile tlong *)0x40000820) /**< TIM4 capture/compare enable register */
#define TIM4_CNT        (*(volatile tlong *)0x40000824) /**< TIM4 counter */
#define TIM4_PSC        (*(volatile tlong *)0x40000828) /**< TIM4 prescaler */
#define TIM4_ARR        (*(volatile tlong *)0x4000082C) /**< TIM4 auto-reload register */
#define TIM4_CCR1       (*(volatile tlong *)0x40000834) /**< TIM4 capture/compare register 1 */
#define TIM4_CCR2       (*(volatile tlong *)0x40000838) /**< TIM4 capture/compare register 2 */
#define TIM4_CCR3       (*(volatile tlong *)0x4000083C) /**< TIM4 capture/compare register 3 */
#define TIM4_CCR4       (*(volatile tlong *)0x40000840) /**< TIM4 capture/compare register 4 */
#define TIM4_DCR        (*(volatile tlong *)0x40000848) /**< TIM4 DMA control register */
#define TIM4_DMAR       (*(volatile tlong *)0x4000084C) /**< TIM4 DMA address for full transfer */
// TIM5
#define TIM5_CR1        (*(volatile tlong *)0x40000C00) /**< TIM5 control register 1 */
#define TIM5_CR2        (*(volatile tlong *)0x40000C04) /**< TIM5 control register 2 */
#define TIM5_SMCR       (*(volatile tlong *)0x40000C08) /**< TIM5 slave mode control register */
#define TIM5_DIER       (*(volatile tlong *)0x40000C0C) /**< TIM5 DMA/Interrupt enable register */
#define TIM5_SR         (*(volatile tlong *)0x40000C10) /**< TIM5 status register */
#define TIM5_EGR        (*(volatile tlong *)0x40000C14) /**< TIM5 event generation register */
#define TIM5_CCMR1      (*(volatile tlong *)0x40000C18) /**< TIM5 capture/compare mode register 1 (channels 1 & 2) */
#define TIM5_CCMR2      (*(volatile tlong *)0x40000C1C) /**< TIM5 capture/compare mode register 2 (channels 3 & 4) */
#define TIM5_CCER       (*(volatile tlong *)0x40000C20) /**< TIM5 capture/compare enable register */
#define TIM5_CNT        (*(volatile tlong *)0x40000C24) /**< TIM5 counter */
#define TIM5_PSC        (*(volatile tlong *)0x40000C28) /**< TIM5 prescaler */
#define TIM5_ARR        (*(volatile tlong *)0x40000C2C) /**< TIM5 auto-reload register */
#define TIM5_CCR1       (*(volatile tlong *)0x40000C34) /**< TIM5 capture/compare register 1 */
#define TIM5_CCR2       (*(volatile tlong *)0x40000C38) /**< TIM5 capture/compare register 2 */
#define TIM5_CCR3       (*(volatile tlong *)0x40000C3C) /**< TIM5 capture/compare register 3 */
#define TIM5_CCR4       (*(volatile tlong *)0x40000C40) /**< TIM5 capture/compare register 4 */
#define TIM5_DCR        (*(volatile tlong *)0x40000C48) /**< TIM5 DMA control register */
#define TIM5_DMAR       (*(volatile tlong *)0x40000C4C) /**< TIM5 DMA address for full transfer */
#define TIM5_OR         (*(volatile tlong *)0x40000C50) /**< TIM5 option register */
// TIM9
#define TIM9_CR1        (*(volatile tlong *)0x40014000) /**< TIM9 control register 1 */
#define TIM9_SMCR       (*(volatile tlong *)0x40014008) /**< TIM9 slave mode control register */
#define TIM9_DIER       (*(volatile tlong *)0x4001400C) /**< TIM9 Interrupt enable register */
#define TIM9_SR         (*(volatile tlong *)0x40014010) /**< TIM9 status register */
#define TIM9_EGR        (*(volatile tlong *)0x40014014) /**< TIM9 event generation register */
#define TIM9_CCMR1      (*(volatile tlong *)0x40014018) /**< TIM9 capture/compare mode register 1 (channels 1 & 2) */
#define TIM9_CCER       (*(volatile tlong *)0x40014020) /**< TIM9 capture/compare enable register */
#define TIM9_CNT        (*(volatile tlong *)0x40014024) /**< TIM9 counter */
#define TIM9_PSC        (*(volatile tlong *)0x40014028) /**< TIM9 prescaler */
#define TIM9_ARR        (*(volatile tlong *)0x4001402C) /**< TIM9 auto-reload register */
#define TIM9_CCR1       (*(volatile tlong *)0x40014034) /**< TIM9 capture/compare register 1 */
#define TIM9_CCR2       (*(volatile tlong *)0x40014038) /**< TIM9 capture/compare register 2 */
// TIM10
#define TIM10_CR1       (*(volatile tlong *)0x40014400) /**< TIM10 control register 1 */
#define TIM10_DIER      (*(volatile tlong *)0x4001440C) /**< TIM10 Interrupt enable register */
#define TIM10_SR        (*(volatile tlong *)0x40014410) /**< TIM10 status register */
#define TIM10_EGR       (*(volatile tlong *)0x40014414) /**< TIM10 event generation register */
#define TIM10_CCMR1     (*(volatile tlong *)0x40014418) /**< TIM10 capture/compare mode register 1 (channel 1) */
#define TIM10_CCER      (*(volatile tlong *)0x40014420) /**< TIM10 capture/compare enable register */
#define TIM10_CNT       (*(volatile tlong *)0x40014424) /**< TIM10 counter */
#define TIM10_PSC       (*(volatile tlong *)0x40014428) /**< TIM10 prescaler */
#define TIM10_ARR       (*(volatile tlong *)0x4001442C) /**< TIM10 auto-reload register */
#define TIM10_CCR1      (*(volatile tlong *)0x40014434) /**< TIM10 capture/compare register 1 */
// TIM11
#define TIM11_CR1       (*(volatile tlong *)0x40014800) /**< TIM11 control register 1 */
#define TIM11_DIER      (*(volatile tlong *)0x4001480C) /**< TIM11 Interrupt enable register */
#define TIM11_SR        (*(volatile tlong *)0x40014810) /**< TIM11 status register */
#define TIM11_EGR       (*(volatile tlong *)0x40014814) /**< TIM11 event generation register */
#define TIM11_CCMR1     (*(volatile tlong *)0x40014818) /**< TIM11 capture/compare mode register 1 (channel 1) */
#define TIM11_CCER      (*(volatile tlong *)0x40014820) /**< TIM11 capture/compare enable register */
#define TIM11_CNT       (*(volatile tlong *)0x40014824) /**< TIM11 counter */
#define TIM11_PSC       (*(volatile tlong *)0x40014828) /**< TIM11 prescaler */
#define TIM11_ARR       (*(volatile tlong *)0x4001482C) /**< TIM11 auto-reload register */
#define TIM11_CCR1      (*(volatile tlong *)0x40014834) /**< TIM11 capture/compare register 1 */

// USART Registers (USART1, USART2, USART6)
// USART1
#define USART1_SR       (*(volatile tlong *)0x40011000) /**< USART1 status register */
#define USART1_DR       (*(volatile tlong *)0x40011004) /**< USART1 data register (transmit/receive data) */
#define USART1_BRR      (*(volatile tlong *)0x40011008) /**< USART1 baud rate register */
#define USART1_CR1      (*(volatile tlong *)0x4001100C) /**< USART1 control register 1 */
#define USART1_CR2      (*(volatile tlong *)0x40011010) /**< USART1 control register 2 */
#define USART1_CR3      (*(volatile tlong *)0x40011014) /**< USART1 control register 3 */
#define USART1_GTPR     (*(volatile tlong *)0x40011018) /**< USART1 guard time and prescaler register */
// USART2
#define USART2_SR       (*(volatile tlong *)0x40004400) /**< USART2 status register */
#define USART2_DR       (*(volatile tlong *)0x40004404) /**< USART2 data register (transmit/receive data) */
#define USART2_BRR      (*(volatile tlong *)0x40004408) /**< USART2 baud rate register */
#define USART2_CR1      (*(volatile tlong *)0x4000440C) /**< USART2 control register 1 */
#define USART2_CR2      (*(volatile tlong *)0x40004410) /**< USART2 control register 2 */
#define USART2_CR3      (*(volatile tlong *)0x40004414) /**< USART2 control register 3 */
#define USART2_GTPR     (*(volatile tlong *)0x40004418) /**< USART2 guard time and prescaler register */
// USART6
#define USART6_SR       (*(volatile tlong *)0x40011400) /**< USART6 status register */
#define USART6_DR       (*(volatile tlong *)0x40011404) /**< USART6 data register (transmit/receive data) */
#define USART6_BRR      (*(volatile tlong *)0x40011408) /**< USART6 baud rate register */
#define USART6_CR1      (*(volatile tlong *)0x4001140C) /**< USART6 control register 1 */
#define USART6_CR2      (*(volatile tlong *)0x40011410) /**< USART6 control register 2 */
#define USART6_CR3      (*(volatile tlong *)0x40011414) /**< USART6 control register 3 */
#define USART6_GTPR     (*(volatile tlong *)0x40011418) /**< USART6 guard time and prescaler register */

// I2C Registers (I2C1, I2C2, I2C3)
// I2C1
#define I2C1_CR1        (*(volatile tlong *)0x40005400) /**< I2C1 Control register 1 */
#define I2C1_CR2        (*(volatile tlong *)0x40005404) /**< I2C1 Control register 2 */
#define I2C1_OAR1       (*(volatile tlong *)0x40005408) /**< I2C1 Own address register 1 */
#define I2C1_OAR2       (*(volatile tlong *)0x4000540C) /**< I2C1 Own address register 2 */
#define I2C1_DR         (*(volatile tlong *)0x40005410) /**< I2C1 Data register */
#define I2C1_SR1        (*(volatile tlong *)0x40005414) /**< I2C1 Status register 1 */
#define I2C1_SR2        (*(volatile tlong *)0x40005418) /**< I2C1 Status register 2 */
#define I2C1_CCR        (*(volatile tlong *)0x4000541C) /**< I2C1 Clock control register */
#define I2C1_TRISE      (*(volatile tlong *)0x40005420) /**< I2C1 TRISE register */
#define I2C1_FLTR       (*(volatile tlong *)0x40005424) /**< I2C1 FLTR register */
// I2C2
#define I2C2_CR1        (*(volatile tlong *)0x40005800) /**< I2C2 Control register 1 */
#define I2C2_CR2        (*(volatile tlong *)0x40005804) /**< I2C2 Control register 2 */
#define I2C2_OAR1       (*(volatile tlong *)0x40005808) /**< I2C2 Own address register 1 */
#define I2C2_OAR2       (*(volatile tlong *)0x4000580C) /**< I2C2 Own address register 2 */
#define I2C2_DR         (*(volatile tlong *)0x40005810) /**< I2C2 Data register */
#define I2C2_SR1        (*(volatile tlong *)0x40005814) /**< I2C2 Status register 1 */
#define I2C2_SR2        (*(volatile tlong *)0x40005818) /**< I2C2 Status register 2 */
#define I2C2_CCR        (*(volatile tlong *)0x4000581C) /**< I2C2 Clock control register */
#define I2C2_TRISE      (*(volatile tlong *)0x40005820) /**< I2C2 TRISE register */
#define I2C2_FLTR       (*(volatile tlong *)0x40005824) /**< I2C2 FLTR register */
// I2C3
#define I2C3_CR1        (*(volatile tlong *)0x40005C00) /**< I2C3 Control register 1 */
#define I2C3_CR2        (*(volatile tlong *)0x40005C04) /**< I2C3 Control register 2 */
#define I2C3_OAR1       (*(volatile tlong *)0x40005C08) /**< I2C3 Own address register 1 */
#define I2C3_OAR2       (*(volatile tlong *)0x40005C0C) /**< I2C3 Own address register 2 */
#define I2C3_DR         (*(volatile tlong *)0x40005C10) /**< I2C3 Data register */
#define I2C3_SR1        (*(volatile tlong *)0x40005C14) /**< I2C3 Status register 1 */
#define I2C3_SR2        (*(volatile tlong *)0x40005C18) /**< I2C3 Status register 2 */
#define I2C3_CCR        (*(volatile tlong *)0x40005C1C) /**< I2C3 Clock control register */
#define I2C3_TRISE      (*(volatile tlong *)0x40005C20) /**< I2C3 TRISE register */
#define I2C3_FLTR       (*(volatile tlong *)0x40005C24) /**< I2C3 FLTR register */

// SPI Registers (SPI1, SPI2, SPI3)
// SPI1
#define SPI1_CR1        (*(volatile tlong *)0x40013000) /**< SPI1 Control register 1 */
#define SPI1_CR2        (*(volatile tlong *)0x40013004) /**< SPI1 Control register 2 */
#define SPI1_SR         (*(volatile tlong *)0x40013008) /**< SPI1 Status register */
#define SPI1_DR         (*(volatile tlong *)0x4001300C) /**< SPI1 Data register */
#define SPI1_CRCPR      (*(volatile tlong *)0x40013010) /**< SPI1 CRC polynomial register */
#define SPI1_RXCRCR     (*(volatile tlong *)0x40013014) /**< SPI1 RX CRC register */
#define SPI1_TXCRCR     (*(volatile tlong *)0x40013018) /**< SPI1 TX CRC register */
#define SPI1_I2SCFGR    (*(volatile tlong *)0x4001301C) /**< SPI1 I2S configuration register */
#define SPI1_I2SPR      (*(volatile tlong *)0x40013020) /**< SPI1 I2S prescaler register */
// SPI2
#define SPI2_CR1        (*(volatile tlong *)0x40003800) /**< SPI2 Control register 1 */
#define SPI2_CR2        (*(volatile tlong *)0x40003804) /**< SPI2 Control register 2 */
#define SPI2_SR         (*(volatile tlong *)0x40003808) /**< SPI2 Status register */
#define SPI2_DR         (*(volatile tlong *)0x4000380C) /**< SPI2 Data register */
#define SPI2_CRCPR      (*(volatile tlong *)0x40003810) /**< SPI2 CRC polynomial register */
#define SPI2_RXCRCR     (*(volatile tlong *)0x40003814) /**< SPI2 RX CRC register */
#define SPI2_TXCRCR     (*(volatile tlong *)0x40003818) /**< SPI2 TX CRC register */
#define SPI2_I2SCFGR    (*(volatile tlong *)0x4000381C) /**< SPI2 I2S configuration register */
#define SPI2_I2SPR      (*(volatile tlong *)0x40003820) /**< SPI2 I2S prescaler register */
// SPI3
#define SPI3_CR1        (*(volatile tlong *)0x40003C00) /**< SPI3 Control register 1 */
#define SPI3_CR2        (*(volatile tlong *)0x40003C04) /**< SPI3 Control register 2 */
#define SPI3_SR         (*(volatile tlong *)0x40003C08) /**< SPI3 Status register */
#define SPI3_DR         (*(volatile tlong *)0x40003C0C) /**< SPI3 Data register */
#define SPI3_CRCPR      (*(volatile tlong *)0x40003C10) /**< SPI3 CRC polynomial register */
#define SPI3_RXCRCR     (*(volatile tlong *)0x40003C14) /**< SPI3 RX CRC register */
#define SPI3_TXCRCR     (*(volatile tlong *)0x40003C18) /**< SPI3 TX CRC register */
#define SPI3_I2SCFGR    (*(volatile tlong *)0x40003C1C) /**< SPI3 I2S configuration register */
#define SPI3_I2SPR      (*(volatile tlong *)0x40003C20) /**< SPI3 I2S prescaler register */

/*
 * ===================================================================================================
 *                                  Inferred RCC Clock Enable Bits
 * ===================================================================================================
 *
 * These definitions are inferred for the STM32F401RC based on common STM32 peripheral mapping.
 */
// RCC AHB1 Peripheral Clock Enable Register (RCC_AHB1ENR)
#define RCC_AHB1ENR_GPIOAEN     (1UL << 0)  // GPIOA clock enable
#define RCC_AHB1ENR_GPIOBEN     (1UL << 1)  // GPIOB clock enable
#define RCC_AHB1ENR_GPIOCEN     (1UL << 2)  // GPIOC clock enable
#define RCC_AHB1ENR_GPIODEN     (1UL << 3)  // GPIOD clock enable
#define RCC_AHB1ENR_GPIOEEN     (1UL << 4)  // GPIOE clock enable
#define RCC_AHB1ENR_GPIOHEN     (1UL << 7)  // GPIOH clock enable
#define RCC_AHB1ENR_CRCEN       (1UL << 12) // CRC clock enable
#define RCC_AHB1ENR_DMA1EN      (1UL << 21) // DMA1 clock enable
#define RCC_AHB1ENR_DMA2EN      (1UL << 22) // DMA2 clock enable
// RCC APB1 Peripheral Clock Enable Register (RCC_APB1ENR)
#define RCC_APB1ENR_TIM2EN      (1UL << 0)  // TIM2 clock enable
#define RCC_APB1ENR_TIM3EN      (1UL << 1)  // TIM3 clock enable
#define RCC_APB1ENR_TIM4EN      (1UL << 2)  // TIM4 clock enable
#define RCC_APB1ENR_TIM5EN      (1UL << 3)  // TIM5 clock enable
#define RCC_APB1ENR_SPI2EN      (1UL << 14) // SPI2 clock enable
#define RCC_APB1ENR_SPI3EN      (1UL << 15) // SPI3 clock enable
#define RCC_APB1ENR_USART2EN    (1UL << 17) // USART2 clock enable
#define RCC_APB1ENR_I2C1EN      (1UL << 21) // I2C1 clock enable
#define RCC_APB1ENR_I2C2EN      (1UL << 22) // I2C2 clock enable
#define RCC_APB1ENR_I2C3EN      (1UL << 23) // I2C3 clock enable
#define RCC_APB1ENR_PWREN       (1UL << 28) // PWR clock enable
#define RCC_APB1ENR_WWDGEN      (1UL << 11) // Window Watchdog clock enable (inferred)
// RCC APB2 Peripheral Clock Enable Register (RCC_APB2ENR)
#define RCC_APB2ENR_TIM1EN      (1UL << 0)  // TIM1 clock enable
#define RCC_APB2ENR_USART1EN    (1UL << 4)  // USART1 clock enable
#define RCC_APB2ENR_USART6EN    (1UL << 5)  // USART6 clock enable
#define RCC_APB2ENR_ADC1EN      (1UL << 8)  // ADC1 clock enable
#define RCC_APB2ENR_SPI1EN      (1UL << 12) // SPI1 clock enable
#define RCC_APB2ENR_SYSCFGEN    (1UL << 14) // SYSCFG clock enable
#define RCC_APB2ENR_TIM9EN      (1UL << 16) // TIM9 clock enable
#define RCC_APB2ENR_TIM10EN     (1UL << 17) // TIM10 clock enable
#define RCC_APB2ENR_TIM11EN     (1UL << 18) // TIM11 clock enable

/*
 * ===================================================================================================
 *                                  Inferred Watchdog Registers (IWDG)
 * ===================================================================================================
 *
 * These definitions are inferred for the STM32F401RC based on common Independent Watchdog (IWDG)
 * peripheral registers. They are not explicitly listed in register_json, but are necessary
 * for WDT functions.
 */
#define IWDG_BASE               (0x40003000UL)
#define IWDG_KR                 (*(volatile tword *)(IWDG_BASE + 0x00UL)) // Key Register
#define IWDG_PR                 (*(volatile tword *)(IWDG_BASE + 0x04UL)) // Prescaler Register
#define IWDG_RLR                (*(volatile tword *)(IWDG_BASE + 0x08UL)) // Reload Register
#define IWDG_SR                 (*(volatile tword *)(IWDG_BASE + 0x0CUL)) // Status Register

// IWDG Key Register values
#define IWDG_KR_KEY_ENABLE      (0x5555UL) // Enable access to PR and RLR registers
#define IWDG_KR_KEY_REFRESH     (0xAAAAUL) // Reload watchdog counter
#define IWDG_KR_KEY_START       (0xCCCCUL) // Start watchdog

// IWDG Prescaler Register values (PR bits)
#define IWDG_PR_DIV4            (0x00UL)
#define IWDG_PR_DIV8            (0x01UL)
#define IWDG_PR_DIV16           (0x02UL)
#define IWDG_PR_DIV32           (0x03UL)
#define IWDG_PR_DIV64           (0x04UL)
#define IWDG_PR_DIV128          (0x05UL)
#define IWDG_PR_DIV256          (0x06UL)

/*
 * ===================================================================================================
 *                                  Static Variables and Callbacks
 * ===================================================================================================
 */
// Placeholder for ICU callback function
static void (*ICU_UserCallback)(void) = NULL;

/*
 * ===================================================================================================
 *                                  Local Helper Functions (if any)
 * ===================================================================================================
 */

/**
 * @brief Get a pointer to the GPIO_MODER register for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile pointer to the MODER register.
 */
static volatile tlong* get_gpio_moder_reg(t_port port) {
    volatile tlong* reg = NULL;
    switch (port) {
        case GPIO_PORT_A: reg = &GPIOA_MODER; break;
        case GPIO_PORT_B: reg = &GPIOB_MODER; break;
        case GPIO_PORT_C: reg = &GPIOC_MODER; break;
        case GPIO_PORT_D: reg = &GPIOD_MODER; break;
        case GPIO_PORT_E: reg = &GPIOE_MODER; break;
        case GPIO_PORT_H: reg = &GPIOH_MODER; break;
        default: /* Should not happen */ break;
    }
    return reg;
}

/**
 * @brief Get a pointer to the GPIO_OTYPER register for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile pointer to the OTYPER register.
 */
static volatile tlong* get_gpio_otyper_reg(t_port port) {
    volatile tlong* reg = NULL;
    switch (port) {
        case GPIO_PORT_A: reg = &GPIOA_OTYPER; break;
        case GPIO_PORT_B: reg = &GPIOB_OTYPER; break;
        case GPIO_PORT_C: reg = &GPIOC_OTYPER; break;
        case GPIO_PORT_D: reg = &GPIOD_OTYPER; break;
        case GPIO_PORT_E: reg = &GPIOE_OTYPER; break;
        case GPIO_PORT_H: reg = &GPIOH_OTYPER; break;
        default: /* Should not happen */ break;
    }
    return reg;
}

/**
 * @brief Get a pointer to the GPIO_OSPEEDR register for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile pointer to the OSPEEDR register.
 */
static volatile tlong* get_gpio_ospeedr_reg(t_port port) {
    volatile tlong* reg = NULL;
    switch (port) {
        case GPIO_PORT_A: reg = &GPIOA_OSPEEDR; break;
        case GPIO_PORT_B: reg = &GPIOB_OSPEEDR; break;
        case GPIO_PORT_C: reg = &GPIOC_OSPEEDR; break;
        case GPIO_PORT_D: reg = &GPIOD_OSPEEDR; break;
        case GPIO_PORT_E: reg = &GPIOE_OSPEEDR; break;
        case GPIO_PORT_H: reg = &GPIOH_OSPEEDR; break;
        default: /* Should not happen */ break;
    }
    return reg;
}

/**
 * @brief Get a pointer to the GPIO_PUPDR register for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile pointer to the PUPDR register.
 */
static volatile tlong* get_gpio_pupdr_reg(t_port port) {
    volatile tlong* reg = NULL;
    switch (port) {
        case GPIO_PORT_A: reg = &GPIOA_PUPDR; break;
        case GPIO_PORT_B: reg = &GPIOB_PUPDR; break;
        case GPIO_PORT_C: reg = &GPIOC_PUPDR; break;
        case GPIO_PORT_D: reg = &GPIOD_PUPDR; break;
        case GPIO_PORT_E: reg = &GPIOE_PUPDR; break;
        case GPIO_PORT_H: reg = &GPIOH_PUPDR; break;
        default: /* Should not happen */ break;
    }
    return reg;
}

/**
 * @brief Get a pointer to the GPIO_ODR register for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile pointer to the ODR register.
 */
static volatile tlong* get_gpio_odr_reg(t_port port) {
    volatile tlong* reg = NULL;
    switch (port) {
        case GPIO_PORT_A: reg = &GPIOA_ODR; break;
        case GPIO_PORT_B: reg = &GPIOB_ODR; break;
        case GPIO_PORT_C: reg = &GPIOC_ODR; break;
        case GPIO_PORT_D: reg = &GPIOD_ODR; break;
        case GPIO_PORT_E: reg = &GPIOE_ODR; break;
        case GPIO_PORT_H: reg = &GPIOH_ODR; break;
        default: /* Should not happen */ break;
    }
    return reg;
}

/**
 * @brief Get a pointer to the GPIO_IDR register for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile pointer to the IDR register.
 */
static volatile tlong* get_gpio_idr_reg(t_port port) {
    volatile tlong* reg = NULL;
    switch (port) {
        case GPIO_PORT_A: reg = &GPIOA_IDR; break;
        case GPIO_PORT_B: reg = &GPIOB_IDR; break;
        case GPIO_PORT_C: reg = &GPIOC_IDR; break;
        case GPIO_PORT_D: reg = &GPIOD_IDR; break;
        case GPIO_PORT_E: reg = &GPIOE_IDR; break;
        case GPIO_PORT_H: reg = &GPIOH_IDR; break;
        default: /* Should not happen */ break;
    }
    return reg;
}

/**
 * @brief Get a pointer to the GPIO_BSRR register for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile pointer to the BSRR register.
 */
static volatile tlong* get_gpio_bsrr_reg(t_port port) {
    volatile tlong* reg = NULL;
    switch (port) {
        case GPIO_PORT_A: reg = &GPIOA_BSRR; break;
        case GPIO_PORT_B: reg = &GPIOB_BSRR; break;
        case GPIO_PORT_C: reg = &GPIOC_BSRR; break;
        case GPIO_PORT_D: reg = &GPIOD_BSRR; break;
        case GPIO_PORT_E: reg = &GPIOE_BSRR; break;
        case GPIO_PORT_H: reg = &GPIOH_BSRR; break;
        default: /* Should not happen */ break;
    }
    return reg;
}

/**
 * @brief Helper function to map a t_uart_channel to its base address.
 * @param uart_channel The UART channel.
 * @return Base address of the UART peripheral.
 */
static volatile USART_TypeDef* get_usart_base(t_uart_channel uart_channel) {
    volatile USART_TypeDef* usart_base = NULL;
    switch (uart_channel) {
        case UART_CHANNEL_1:
            usart_base = (volatile USART_TypeDef*)0x40011000; /* USART1 Base Address */
            break;
        case UART_CHANNEL_2:
            usart_base = (volatile USART_TypeDef*)0x40004400; /* USART2 Base Address */
            break;
        case UART_CHANNEL_6:
            usart_base = (volatile USART_TypeDef*)0x40011400; /* USART6 Base Address */
            break;
        default:
            // Handle error or return a default/invalid address
            break;
    }
    return usart_base;
}

/**
 * @brief Helper function to map a t_i2c_channel to its base address.
 * @param i2c_channel The I2C channel.
 * @return Base address of the I2C peripheral.
 */
static volatile I2C_TypeDef* get_i2c_base(t_i2c_channel i2c_channel) {
    volatile I2C_TypeDef* i2c_base = NULL;
    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            i2c_base = (volatile I2C_TypeDef*)0x40005400; /* I2C1 Base Address */
            break;
        case I2C_CHANNEL_2:
            i2c_base = (volatile I2C_TypeDef*)0x40005800; /* I2C2 Base Address */
            break;
        case I2C_CHANNEL_3:
            i2c_base = (volatile I2C_TypeDef*)0x40005C00; /* I2C3 Base Address */
            break;
        default:
            // Handle error or return a default/invalid address
            break;
    }
    return i2c_base;
}

/**
 * @brief Helper function to map a t_spi_channel to its base address.
 * @param spi_channel The SPI channel.
 * @return Base address of the SPI peripheral.
 */
static volatile SPI_TypeDef* get_spi_base(t_spi_channel spi_channel) {
    volatile SPI_TypeDef* spi_base = NULL;
    switch (spi_channel) {
        case SPI_CHANNEL_1:
            spi_base = (volatile SPI_TypeDef*)0x40013000; /* SPI1 Base Address */
            break;
        case SPI_CHANNEL_2:
            spi_base = (volatile SPI_TypeDef*)0x40003800; /* SPI2 Base Address */
            break;
        case SPI_CHANNEL_3:
            spi_base = (volatile SPI_TypeDef*)0x40003C00; /* SPI3 Base Address */
            break;
        default:
            // Handle error or return a default/invalid address
            break;
    }
    return spi_base;
}

/**
 * @brief Helper function to map a t_pwm_channel to its timer base and channel number.
 * @param pwm_channel The PWM channel.
 * @param timer_base Pointer to store the TIM_TypeDef base address.
 * @param timer_ch Pointer to store the channel number (1-4 for most timers, 1-2 for TIM9, 1 for TIM10/11).
 */
static void get_timer_for_pwm(t_pwm_channel pwm_channel, volatile TIM_TypeDef** timer_base, tbyte* timer_ch) {
    *timer_base = NULL;
    *timer_ch = 0; // Invalid channel initially

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1: *timer_base = (volatile TIM_TypeDef*)0x40010000; *timer_ch = 1; break;
        case PWM_CHANNEL_TIM1_CH2: *timer_base = (volatile TIM_TypeDef*)0x40010000; *timer_ch = 2; break;
        case PWM_CHANNEL_TIM1_CH3: *timer_base = (volatile TIM_TypeDef*)0x40010000; *timer_ch = 3; break;
        case PWM_CHANNEL_TIM1_CH4: *timer_base = (volatile TIM_TypeDef*)0x40010000; *timer_ch = 4; break;
        case PWM_CHANNEL_TIM2_CH1: *timer_base = (volatile TIM_TypeDef*)0x40000000; *timer_ch = 1; break;
        case PWM_CHANNEL_TIM2_CH2: *timer_base = (volatile TIM_TypeDef*)0x40000000; *timer_ch = 2; break;
        case PWM_CHANNEL_TIM2_CH3: *timer_base = (volatile TIM_TypeDef*)0x40000000; *timer_ch = 3; break;
        case PWM_CHANNEL_TIM2_CH4: *timer_base = (volatile TIM_TypeDef*)0x40000000; *timer_ch = 4; break;
        case PWM_CHANNEL_TIM3_CH1: *timer_base = (volatile TIM_TypeDef*)0x40000400; *timer_ch = 1; break;
        case PWM_CHANNEL_TIM3_CH2: *timer_base = (volatile TIM_TypeDef*)0x40000400; *timer_ch = 2; break;
        case PWM_CHANNEL_TIM3_CH3: *timer_base = (volatile TIM_TypeDef*)0x40000400; *timer_ch = 3; break;
        case PWM_CHANNEL_TIM3_CH4: *timer_base = (volatile TIM_TypeDef*)0x40000400; *timer_ch = 4; break;
        case PWM_CHANNEL_TIM4_CH1: *timer_base = (volatile TIM_TypeDef*)0x40000800; *timer_ch = 1; break;
        case PWM_CHANNEL_TIM4_CH2: *timer_base = (volatile TIM_TypeDef*)0x40000800; *timer_ch = 2; break;
        case PWM_CHANNEL_TIM4_CH3: *timer_base = (volatile TIM_TypeDef*)0x40000800; *timer_ch = 3; break;
        case PWM_CHANNEL_TIM4_CH4: *timer_base = (volatile TIM_TypeDef*)0x40000800; *timer_ch = 4; break;
        case PWM_CHANNEL_TIM5_CH1: *timer_base = (volatile TIM_TypeDef*)0x40000C00; *timer_ch = 1; break;
        case PWM_CHANNEL_TIM5_CH2: *timer_base = (volatile TIM_TypeDef*)0x40000C00; *timer_ch = 2; break;
        case PWM_CHANNEL_TIM5_CH3: *timer_base = (volatile TIM_TypeDef*)0x40000C00; *timer_ch = 3; break;
        case PWM_CHANNEL_TIM5_CH4: *timer_base = (volatile TIM_TypeDef*)0x40000C00; *timer_ch = 4; break;
        case PWM_CHANNEL_TIM9_CH1: *timer_base = (volatile TIM_TypeDef*)0x40014000; *timer_ch = 1; break;
        case PWM_CHANNEL_TIM9_CH2: *timer_base = (volatile TIM_TypeDef*)0x40014000; *timer_ch = 2; break;
        case PWM_CHANNEL_TIM10_CH1: *timer_base = (volatile TIM_TypeDef*)0x40014400; *timer_ch = 1; break;
        case PWM_CHANNEL_TIM11_CH1: *timer_base = (volatile TIM_TypeDef*)0x40014800; *timer_ch = 1; break;
        default: /* Error or unhandled channel */ break;
    }
}

/*
 * ===================================================================================================
 *                                  API Implementations
 * ===================================================================================================
 */

/**
 * @brief Resets the Watchdog Timer (WDT).
 *
 * This function refreshes the Independent Watchdog (IWDG) counter to prevent a reset.
 * It's implemented using inferred IWDG registers for STM32F401RC.
 */
void WDT_Reset(void) {
    // Inferred IWDG KICK register for STM32F401RC
    IWDG_KR = IWDG_KR_KEY_REFRESH; // Write 0xAAAA to reload the watchdog counter
}

/**
 * @brief Initializes the Microcontroller Configuration.
 *
 * This function sets up the basic MCU parameters, including GPIO states,
 * disables various peripherals, configures and enables the Watchdog Timer,
 * and sets up Low Voltage Reset (LVR).
 *
 * @param volt The system voltage level (Vsource_3V or Vsource_5V).
 */
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset();

    // 1. Set all GPIO pins to 0 and verify with while loop
    // Enable all GPIO clocks first
    RCC_AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN);
    WDT_Reset();

    // Reset GPIO output data registers to 0
    GPIOA_ODR = 0x00000000UL;
    GPIOB_ODR = 0x00000000UL;
    GPIOC_ODR = 0x00000000UL;
    GPIOD_ODR = 0x00000000UL;
    GPIOE_ODR = 0x00000000UL;
    GPIOH_ODR = 0x00000000UL; // Port H has fewer pins (0, 1)

    // Verify GPIO output data registers are 0 (simple loop, real verification would involve reading back)
    while ( (GPIOA_ODR != 0x00000000UL) || (GPIOB_ODR != 0x00000000UL) || (GPIOC_ODR != 0x00000000UL) ||
            (GPIOD_ODR != 0x00000000UL) || (GPIOE_ODR != 0x00000000UL) || (GPIOH_ODR != 0x00000000UL) ) {
        // Error: GPIO ODR did not clear. Potentially loop indefinitely or implement a timeout.
        // For now, continue (assuming success or a watchdog reset will eventually occur).
        WDT_Reset();
    }
    WDT_Reset();

    // 2. Set all GPIO pins direction to input and verify with while loop
    // All MODER bits to '00' for Input mode
    GPIOA_MODER = 0x00000000UL;
    GPIOB_MODER = 0x00000000UL;
    GPIOC_MODER = 0x00000000UL;
    GPIOD_MODER = 0x00000000UL;
    GPIOE_MODER = 0x00000000UL;
    GPIOH_MODER = 0x00000000UL;

    // Set all input pins to pull-up (01) per GPIO rules, for Port A-E (Port H only has 2 pins, but full 16-bit register still).
    // This is 0x55555555 for full 16 pins, but for specific pins, needs masking.
    // For now, applying to all available 16 bits * 2 (for 2-bit per pin) for simplicity.
    // For input pins, also enable pull-up resistors as per GPIO rules
    GPIOA_PUPDR = 0x55555555UL; // All pins pull-up
    GPIOB_PUPDR = 0x55555555UL; // All pins pull-up
    GPIOC_PUPDR = 0x55555555UL; // All pins pull-up
    GPIOD_PUPDR = 0x55555555UL; // All pins pull-up
    GPIOE_PUPDR = 0x55555555UL; // All pins pull-up
    GPIOH_PUPDR = 0x00000005UL; // PH0, PH1 pull-up (0b01 for pin 0, 0b01 for pin 1)
    WDT_Reset();

    // Verify GPIO mode registers are input (simple loop)
    while ( (GPIOA_MODER != 0x00000000UL) || (GPIOB_MODER != 0x00000000UL) || (GPIOC_MODER != 0x00000000UL) ||
            (GPIOD_MODER != 0x00000000UL) || (GPIOE_MODER != 0x00000000UL) || (GPIOH_MODER != 0x00000000UL) ) {
        // Error: GPIO MODER did not set to input.
        WDT_Reset();
    }
    WDT_Reset();

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();

    // Disable peripheral clocks in RCC to ensure they are off
    RCC_AHB1ENR = 0x00000000UL; // Keep GPIO clocks enabled for basic operation, disable others
    RCC_AHB2ENR = 0x00000000UL;
    RCC_APB1ENR = 0x00000000UL; // Keep PWREN (bit 28) if PVD is used
    RCC_APB2ENR = 0x00000000UL; // Keep SYSCFGEN (bit 14) for EXTI if needed.

    // Re-enable essential clocks after mass-disabling
    RCC_AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN);
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;    // Power interface clock (for PVD)
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN; // SYSCFG clock (for EXTI configuration)
    WDT_Reset();

    // Explicitly disable major peripherals (if they have enable bits outside clock enables)
    // ADC
    ADC_CR2 &= ~(1UL << 0); // ADC_CR2_ADON bit to disable ADC (inferred)
    // UARTs
    USART1_CR1 &= ~(1UL << 13); // USART1_CR1_UE bit to disable USART (inferred)
    USART2_CR1 &= ~(1UL << 13); // USART2_CR1_UE bit to disable USART (inferred)
    USART6_CR1 &= ~(1UL << 13); // USART6_CR1_UE bit to disable USART (inferred)
    // I2Cs
    I2C1_CR1 &= ~(1UL << 0); // I2C1_CR1_PE bit to disable I2C (inferred)
    I2C2_CR1 &= ~(1UL << 0); // I2C2_CR1_PE bit to disable I2C (inferred)
    I2C3_CR1 &= ~(1UL << 0); // I2C3_CR1_PE bit to disable I2C (inferred)
    // SPIs
    SPI1_CR1 &= ~(1UL << 6); // SPI1_CR1_SPE bit to disable SPI (inferred)
    SPI2_CR1 &= ~(1UL << 6); // SPI2_CR1_SPE bit to disable SPI (inferred)
    SPI3_CR1 &= ~(1UL << 6); // SPI3_CR1_SPE bit to disable SPI (inferred)
    // TIMERS (disable main counter for all TIMERS, which typically have a CEN bit at 0)
    TIM1_CR1 &= ~0x1UL;
    TIM2_CR1 &= ~0x1UL;
    TIM3_CR1 &= ~0x1UL;
    TIM4_CR1 &= ~0x1UL;
    TIM5_CR1 &= ~0x1UL;
    TIM9_CR1 &= ~0x1UL;
    TIM10_CR1 &= ~0x1UL;
    TIM11_CR1 &= ~0x1UL;
    WDT_Reset();

    // 4. Enable WDT (Watchdog Timer)
    // Unlock IWDG registers
    IWDG_KR = IWDG_KR_KEY_ENABLE; // Inferred

    // 5. Clear WDT timer (reloaded by the refresh key)
    IWDG_KR = IWDG_KR_KEY_REFRESH; // Inferred

    // 6. Set WDT period >= 8 msec (example: LSI is ~32kHz. With prescaler 32, RLR=250 gives ~250ms)
    // (32000Hz / 32) = 1000Hz. RLR = 250 -> 250ms.
    // Prescaler 16, RLR=125 -> 125ms (max reload value is 0xFFF = 4095)
    IWDG_PR = IWDG_PR_DIV32; // Set prescaler to 32 (inferred)
    IWDG_RLR = 250;          // Set reload value for approx 250ms (inferred)
    WDT_Reset(); // Apply the reload value by refreshing

    // Start IWDG (if not already running from option bytes)
    IWDG_KR = IWDG_KR_KEY_START; // Inferred (this starts it if it was disabled or frozen)
    WDT_Reset();

    // 7. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // 8. Enable LVR (Low Voltage Reset)
    // LVR is typically PVD (Programmable Voltage Detector) in STM32
    PWR_CR |= RCC_APB1ENR_PWREN; // Ensure PWR clock is enabled for PVD control
    WDT_Reset();

    // Clear PVD level selection bits (PLS[2:0] are bits 7:5)
    PWR_CR &= ~(0x7UL << 5); // Clear PLS bits
    WDT_Reset();

    if (volt == Vsource_3V) {
        // For 3V system, set PVD threshold to 2.0V (PLS_0)
        PWR_CR |= (0x0UL << 5); // PLS = 000 (2.0V threshold)
    } else { // Vsource_5V
        // For 5V system, set PVD threshold to 3.5V (PLS_5, inferred based on typical STM32 range)
        PWR_CR |= (0x5UL << 5); // PLS = 101 (2.9V on STM32F4, closest to 3.5V with available options)
                                // Actual mapping for 3.5V might require different PLS or be unavailable.
                                // NOTE: On STM32F401, PLS_5 corresponds to 2.9V. There is no direct 3.5V.
                                // Choosing closest/safe option.
    }
    // Enable PVD
    PWR_CR |= (1UL << 4); // Set PVDE bit (Power Voltage Detector Enable)
    WDT_Reset();

    // 9. Clear WDT again
    WDT_Reset();
}

/**
 * @brief Enters low-power sleep mode.
 *
 * This function instructs the MCU to enter a low-power sleep state,
 * where code execution pauses until an interrupt occurs.
 * Implemented using the CMSIS intrinsic `__WFI()`.
 */
void Go_to_sleep_mode(void) {
    WDT_Reset();
    __WFI(); // Wait For Interrupt - a common low-power instruction for ARM Cortex-M
}

/**
 * @brief Enables global interrupts.
 *
 * This function enables the global interrupt mask, allowing interrupts to be processed.
 * Implemented using the CMSIS intrinsic `__enable_irq()`.
 */
void Global_interrupt_Enable(void) {
    WDT_Reset();
    __enable_irq();
}

/**
 * @brief Disables global interrupts.
 *
 * This function disables the global interrupt mask, preventing interrupts from being processed.
 * Implemented using the CMSIS intrinsic `__disable_irq()`.
 */
void Global_interrupt_Disable(void) {
    WDT_Reset();
    __disable_irq();
}

/**
 * @brief Initializes the Low Voltage Detector (LVD).
 *
 * This function enables the power interface clock for LVD (PVD on STM32) and
 * can set a default threshold.
 */
void LVD_Init(void) {
    WDT_Reset();
    RCC_APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock for PVD
    // Default to a safe LVD threshold, e.g., 2.0V
    PWR_CR &= ~(0x7UL << 5); // Clear PLS bits
    PWR_CR |= (0x0UL << 5); // Set PLS = 000 (2.0V threshold)
}

/**
 * @brief Sets the LVD threshold level.
 *
 * This function configures the Power Voltage Detector (PVD) threshold level.
 * @param lvd_thresholdLevel The desired LVD threshold level.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) { // Renamed from Get to SetLVDThreshold for clarity
    WDT_Reset();
    // Clear existing PVD level
    PWR_CR &= ~(0x7UL << 5); // Clear PLS bits (7:5)

    // Set new PVD level
    switch (lvd_thresholdLevel) {
        case LVD_THRESHOLD_1_9V: PWR_CR |= (0x0UL << 5); break;
        case LVD_THRESHOLD_2_1V: PWR_CR |= (0x1UL << 5); break;
        case LVD_THRESHOLD_2_3V: PWR_CR |= (0x2UL << 5); break;
        case LVD_THRESHOLD_2_5V: PWR_CR |= (0x3UL << 5); break;
        case LVD_THRESHOLD_2_7V: PWR_CR |= (0x4UL << 5); break;
        case LVD_THRESHOLD_2_9V: PWR_CR |= (0x5UL << 5); break;
        case LVD_THRESHOLD_3_1V: PWR_CR |= (0x6UL << 5); break;
        case LVD_THRESHOLD_EXT:  PWR_CR |= (0x7UL << 5); break;
        case LVD_THRESHOLD_2_0V: PWR_CR |= (0x0UL << 5); break; // Map to 1.9V for closest
        case LVD_THRESHOLD_3_5V: PWR_CR |= (0x5UL << 5); break; // Map to 2.9V for closest
        default: /* Invalid threshold, keep current setting or set a safe default */ break;
    }
}

/**
 * @brief Enables the Low Voltage Detector (LVD).
 *
 * This function enables the Power Voltage Detector (PVD).
 */
void LVD_Enable(void) {
    WDT_Reset();
    RCC_APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR clock (inferred)
    PWR_CR |= (1UL << 4); // Set PVDE bit (Power Voltage Detector Enable)
}

/**
 * @brief Disables the Low Voltage Detector (LVD).
 *
 * This function disables the Power Voltage Detector (PVD).
 */
void LVD_Disable(void) {
    WDT_Reset();
    PWR_CR &= ~(1UL << 4); // Clear PVDE bit (Power Voltage Detector Enable)
}

/**
 * @brief Clears the LVD flag.
 *
 * On STM32, PVD status is typically a read-only bit (PVDO) in PWR_CSR,
 * not a flag that needs to be cleared by software. This function is a placeholder.
 * @param lvd_channel The LVD channel (ignored for STM32 PVD).
 */
void LVD_ClearFlag(t_lvd_channel lvd_channel) {
    WDT_Reset();
    // STM32 PVD output (PVDO) in PWR_CSR is typically read-only.
    // No action needed to "clear" it as it reflects the current voltage status.
    (void)lvd_channel; // Suppress unused parameter warning
}

/**
 * @brief Initializes a UART channel.
 * @param uart_channel The UART channel to initialize.
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data length (8-bit or 9-bit).
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting (none, even, odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset();
    volatile USART_TypeDef* USARTx = get_usart_base(uart_channel);

    if (USARTx != NULL) {
        // Disable USART before configuration
        USARTx->CR1 &= ~(1UL << 13); // Clear UE bit (USART Enable)

        // Configure baud rate (example calculation for 16MHz APB clock)
        tlong pclk_freq = 16000000UL; // Assuming APB1 (for USART2) or APB2 (for USART1/6)
        if ((uart_channel == UART_CHANNEL_1) || (uart_channel == UART_CHANNEL_6)) {
             // For USART1/6, clock source is APB2, assume 84MHz (max for F401)
            pclk_freq = 84000000UL; // Inferred from STM32F401RC max clock
        } else { // UART_CHANNEL_2
            // For USART2, clock source is APB1, assume 42MHz (max for F401)
            pclk_freq = 42000000UL; // Inferred from STM32F401RC max clock
        }
        
        tlong baud_value = 0;
        switch (uart_baud_rate) {
            case UART_BAUD_RATE_9600: baud_value = 9600; break;
            case UART_BAUD_RATE_19200: baud_value = 19200; break;
            case UART_BAUD_RATE_38400: baud_value = 38400; break;
            case UART_BAUD_RATE_57600: baud_value = 57600; break;
            case UART_BAUD_RATE_115200: baud_value = 115200; break;
            default: baud_value = 9600; break; // Default to 9600
        }
        // Formula: Tx/Rx baud = f_PCLK / (8 * (2 - OVER8) * USARTDIV)
        // For OVER8 = 0 (default): USARTDIV = f_PCLK / (16 * baud)
        tword usartdiv = (tword)(pclk_freq / (16UL * baud_value));
        USARTx->BRR = usartdiv;

        // Configure data length (CR1_M bit)
        if (uart_data_length == UART_DATA_LENGTH_9B) {
            USARTx->CR1 |= (1UL << 12); // M=1 for 9-bit
        } else {
            USARTx->CR1 &= ~(1UL << 12); // M=0 for 8-bit
        }

        // Configure stop bits (CR2_STOP bits 13:12)
        USARTx->CR2 &= ~(0x3UL << 12); // Clear STOP bits
        switch (uart_stop_bit) {
            case UART_STOP_BIT_0_5: USARTx->CR2 |= (0x1UL << 12); break; // 01 for 0.5 Stop bit
            case UART_STOP_BIT_2:   USARTx->CR2 |= (0x2UL << 12); break; // 10 for 2 Stop bits
            case UART_STOP_BIT_1_5: USARTx->CR2 |= (0x3UL << 12); break; // 11 for 1.5 Stop bits
            case UART_STOP_BIT_1:   // Default, 00 for 1 Stop bit
            default: break;
        }

        // Configure parity (CR1_PCE bit 10, CR1_PS bit 9)
        if (uart_parity == UART_PARITY_NONE) {
            USARTx->CR1 &= ~(1UL << 10); // PCE=0 (Parity Control Disable)
        } else {
            USARTx->CR1 |= (1UL << 10); // PCE=1 (Parity Control Enable)
            if (uart_parity == UART_PARITY_EVEN) {
                USARTx->CR1 &= ~(1UL << 9); // PS=0 (Even Parity)
            } else { // UART_PARITY_ODD
                USARTx->CR1 |= (1UL << 9); // PS=1 (Odd Parity)
            }
        }

        // Enable Transmitter and Receiver (CR1_TE bit 3, CR1_RE bit 2)
        USARTx->CR1 |= (1UL << 3) | (1UL << 2);
    }
}

/**
 * @brief Enables a UART channel.
 *
 * This function enables the clock for the specified UART peripheral and then
 * enables the UART module itself.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset();

    // Enable peripheral clock
    if (uart_channel == UART_CHANNEL_1) {
        RCC_APB2ENR |= RCC_APB2ENR_USART1EN; // Inferred
    } else if (uart_channel == UART_CHANNEL_2) {
        RCC_APB1ENR |= RCC_APB1ENR_USART2EN; // Inferred
    } else if (uart_channel == UART_CHANNEL_6) {
        RCC_APB2ENR |= RCC_APB2ENR_USART6EN; // Inferred
    }
    WDT_Reset();

    // Enable USART module
    volatile USART_TypeDef* USARTx = get_usart_base(uart_channel);
    if (USARTx != NULL) {
        USARTx->CR1 |= (1UL << 13); // Set UE bit (USART Enable)
    }
}

/**
 * @brief Disables a UART channel.
 *
 * This function disables the UART module and its peripheral clock.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset();
    volatile USART_TypeDef* USARTx = get_usart_base(uart_channel);
    if (USARTx != NULL) {
        USARTx->CR1 &= ~(1UL << 13); // Clear UE bit (USART Enable)
    }

    // Disable peripheral clock
    if (uart_channel == UART_CHANNEL_1) {
        RCC_APB2ENR &= ~RCC_APB2ENR_USART1EN; // Inferred
    } else if (uart_channel == UART_CHANNEL_2) {
        RCC_APB1ENR &= ~RCC_APB1ENR_USART2EN; // Inferred
    } else if (uart_channel == UART_CHANNEL_6) {
        RCC_APB2ENR &= ~RCC_APB2ENR_USART6EN; // Inferred
    }
}

/**
 * @brief Updates a UART channel (placeholder, often used for status checks or re-configuration).
 * @param uart_channel The UART channel to update.
 */
void UART_Update(t_uart_channel uart_channel) {
    WDT_Reset();
    (void)uart_channel; // Suppress unused parameter warning
    // This function typically involves re-reading status registers or re-applying config.
    // Specific implementation depends on requirements. For now, a placeholder.
}

/**
 * @brief Sends a single byte over UART.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset();
    volatile USART_TypeDef* USARTx = get_usart_base(uart_channel);
    if (USARTx != NULL) {
        // Wait for Transmit Data Register Empty (TXE) flag
        while (!(USARTx->SR & (1UL << 7))) { // SR_TXE bit (inferred)
            WDT_Reset();
        }
        USARTx->DR = byte; // Write data to DR
    }
}

/**
 * @brief Sends a frame of data over UART.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset();
    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over UART.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset();
    while (*str != '\0') {
        UART_send_byte(uart_channel, (tbyte)*str);
        str++;
    }
}

/**
 * @brief Receives a single byte from UART.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset();
    volatile USART_TypeDef* USARTx = get_usart_base(uart_channel);
    tbyte received_byte = 0;
    if (USARTx != NULL) {
        // Wait for Read Data Register Not Empty (RXNE) flag
        while (!(USARTx->SR & (1UL << 5))) { // SR_RXNE bit (inferred)
            WDT_Reset();
        }
        received_byte = (tbyte)(USARTx->DR & 0xFFUL); // Read data from DR
    }
    return received_byte;
}

/**
 * @brief Receives a frame of data from UART.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset();
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Receives a null-terminated string from UART.
 *
 * Note: This function will block until `max_length` bytes are received or a specific stop character
 * (e.g., newline) is encountered. For this generic implementation, it receives `max_length` bytes.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum number of bytes to receive (including null terminator).
 * @return The number of bytes received (excluding null terminator).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset();
    tbyte count = 0;
    if ((buffer != NULL) && (max_length > 0)) {
        for (int i = 0; i < (max_length - 1); i++) { // Leave space for null terminator
            buffer[i] = (char)UART_Get_Byte(uart_channel);
            count++;
            // Optional: break if newline or carriage return is received
            // if (buffer[i] == '\n' || buffer[i] == '\r') {
            //     break;
            // }
        }
        buffer[count] = '\0'; // Null-terminate the string
    }
    return count;
}

/**
 * @brief Clears flags for a UART channel.
 *
 * This function clears relevant status flags (e.g., RXNE, TC) in the UART Status Register.
 * Note: Some flags are cleared by software sequence (read SR then DR), others by writing to them.
 * @param uart_channel The UART channel.
 */
void UART_ClearFlag(t_uart_channel uart_channel) {
    WDT_Reset();
    volatile USART_TypeDef* USARTx = get_usart_base(uart_channel);
    if (USARTx != NULL) {
        // Clear all writable flags by writing 0 to them.
        // On STM32F4, most status flags are cleared by reading SR then DR (for RXNE)
        // or reading SR then writing to DR (for TC), or by explicit bits in SR.
        // For simplicity, we'll try to clear specific bits if direct write is allowed.
        // If not, a read operation might be enough.
        // Assuming TC (Transmission Complete) flag needs clearing for example:
        USARTx->SR &= ~(1UL << 6); // Clear TC flag (inferred)
        // Other flags might clear on read or subsequent operations.
    }
}

/**
 * @brief Initializes an I2C channel.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (Standard or Fast mode).
 * @param i2c_device_address The device's own 7-bit address.
 * @param i2c_ack Acknowledge enable/disable.
 * @param i2c_datalength Data length (7-bit or 10-bit addressing for slave mode).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset();
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);

    if (I2Cx != NULL) {
        // Disable I2C peripheral to configure
        I2Cx->CR1 &= ~(1UL << 0); // Clear PE bit (Peripheral Enable)
        WDT_Reset();

        // 1. Configure clock speed (fast mode always used per rule)
        // Assume PCLK1 frequency for I2C (max 42MHz for F401)
        tlong pclk1_freq = 42000000UL; // Inferred for STM32F401RC
        tlong clock_period_ns = (tlong)(1000000000UL / pclk1_freq); // Nanoseconds per clock cycle

        // Set APB1 peripheral clock frequency in CR2 (FREQ bits 5:0)
        I2Cx->CR2 &= ~(0x3FUL << 0); // Clear FREQ bits
        I2Cx->CR2 |= (pclk1_freq / 1000000UL); // Set FREQ bits to PCLK1_MHz

        // Calculate CCR value for desired clock speed
        tword ccr_value = 0;
        tword trise_value = 0;
        if (i2c_clk_speed == I2C_CLK_SPEED_FAST) {
            // Fast mode: Duty cycle 2 (T_high/T_low = 1/2) for Fm mode
            // CCR = (PCLK1 / (3 * I2C_CLK))
            // I2C_CLK = 400kHz
            ccr_value = (tword)(pclk1_freq / (3UL * 400000UL));
            I2Cx->CCR |= (1UL << 15); // Set F/M bit for Fast Mode
            I2Cx->CCR |= (1UL << 14); // Set DUTY for T_low/T_high = 2 (used if Fm is 1)

            // TRISE for Fast Mode: (Trise_max / T_PCLK1) + 1, where Trise_max = 300ns
            trise_value = (tword)((300UL / clock_period_ns) + 1UL);

        } else { // I2C_CLK_SPEED_STANDARD
            // Standard mode: T_high/T_low = 1/1
            // CCR = (PCLK1 / (2 * I2C_CLK))
            // I2C_CLK = 100kHz
            ccr_value = (tword)(pclk1_freq / (2UL * 100000UL));
            I2Cx->CCR &= ~(1UL << 15); // Clear F/M bit for Standard Mode

            // TRISE for Standard Mode: (Trise_max / T_PCLK1) + 1, where Trise_max = 1000ns
            trise_value = (tword)((1000UL / clock_period_ns) + 1UL);
        }
        I2Cx->CCR &= ~(0xFFFUL << 0); // Clear CCR value bits
        I2Cx->CCR |= ccr_value;
        I2Cx->TRISE = trise_value;

        // 2. Configure own address (OAR1)
        I2Cx->OAR1 &= ~(0x3FFUL << 0); // Clear ADD bits
        I2Cx->OAR1 |= (tlong)(i2c_device_address << 1); // Set 7-bit address (A0 cleared)
        I2Cx->OAR1 |= (1UL << 14); // Set bit 14, always kept at 1 by hardware

        // 3. Configure ACK (CR1_ACK bit 10)
        if (i2c_ack == I2C_ACK_ENABLE) {
            I2Cx->CR1 |= (1UL << 10); // Set ACK bit
        } else {
            I2Cx->CR1 &= ~(1UL << 10); // Clear ACK bit
        }

        // 4. Configure data length (for addressing, not data frames) (OAR1_ADDMODE bit 15)
        // This is typically for own address mode (7-bit or 10-bit slave addressing)
        if (i2c_datalength == I2C_DATALENGTH_10BIT) {
            I2Cx->OAR1 |= (1UL << 15); // Set ADDMODE for 10-bit address
        } else {
            I2Cx->OAR1 &= ~(1UL << 15); // Clear ADDMODE for 7-bit address
        }

        // Set maximum timeout (No specific timeout register in I2C peripheral in JSON, usually application level)
        // Assuming a software timeout is handled by the application layer.
        // For hardware, it might refer to SCL/SDA Timeout registers if available (not in register_json)

        // Re-enable I2C peripheral
        I2Cx->CR1 |= (1UL << 0); // Set PE bit (Peripheral Enable)
    }
}

/**
 * @brief Enables an I2C channel.
 *
 * This function enables the clock for the specified I2C peripheral and then
 * enables the I2C module itself.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset();

    // Enable peripheral clock
    if (i2c_channel == I2C_CHANNEL_1) {
        RCC_APB1ENR |= RCC_APB1ENR_I2C1EN; // Inferred
    } else if (i2c_channel == I2C_CHANNEL_2) {
        RCC_APB1ENR |= RCC_APB1ENR_I2C2EN; // Inferred
    } else if (i2c_channel == I2C_CHANNEL_3) {
        RCC_APB1ENR |= RCC_APB1ENR_I2C3EN; // Inferred
    }
    WDT_Reset();

    // Enable I2C module
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);
    if (I2Cx != NULL) {
        I2Cx->CR1 |= (1UL << 0); // Set PE bit (Peripheral Enable)
    }
}

/**
 * @brief Disables an I2C channel.
 *
 * This function disables the I2C module and its peripheral clock.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset();
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);
    if (I2Cx != NULL) {
        I2Cx->CR1 &= ~(1UL << 0); // Clear PE bit (Peripheral Enable)
    }

    // Disable peripheral clock
    if (i2c_channel == I2C_CHANNEL_1) {
        RCC_APB1ENR &= ~RCC_APB1ENR_I2C1EN; // Inferred
    } else if (i2c_channel == I2C_CHANNEL_2) {
        RCC_APB1ENR &= ~RCC_APB1ENR_I2C2EN; // Inferred
    } else if (i2c_channel == I2C_CHANNEL_3) {
        RCC_APB1ENR &= ~RCC_APB1ENR_I2C3EN; // Inferred
    }
}

/**
 * @brief Updates an I2C channel (placeholder).
 * @param i2c_channel The I2C channel to update.
 */
void I2C_Update(t_i2c_channel i2c_channel) {
    WDT_Reset();
    (void)i2c_channel; // Suppress unused parameter warning
    // Specific update logic (e.g., refreshing internal state, checking errors)
    // would be implemented here. For now, a placeholder.
}

/**
 * @brief Sends a single byte over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 *
 * This function initiates a repeated start condition if needed,
 * sends the device address (write), sends the byte, and handles ACK/NACK.
 * This is a basic blocking implementation for a master sending to a slave.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset();
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);
    if (I2Cx != NULL) {
        // Assume start condition and address send is handled externally or within a transaction wrapper
        // For simple byte send, assume bus is in master transmit mode after address.
        // Wait until TXE (Transmit data register empty) is set
        while (!(I2Cx->SR1 & (1UL << 7))) { /* Wait for TxE */ WDT_Reset(); }
        I2Cx->DR = byte; // Write data to DR
        // Wait until BTF (Byte Transfer Finished) is set
        while (!(I2Cx->SR1 & (1UL << 2))) { /* Wait for BTF */ WDT_Reset(); }
    }
}

/**
 * @brief Sends a frame of data over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 *
 * This function generates a START condition, sends slave address with write bit,
 * sends all data bytes, and then generates a STOP condition.
 * Always generates a repeated start condition instead of stop between transactions (rule).
 * This function assumes a master mode operation.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset();
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);

    if ((I2Cx != NULL) && (data != NULL) && (length > 0)) {
        // 1. Generate START condition
        I2Cx->CR1 |= (1UL << 8); // Set START bit
        while (!(I2Cx->SR1 & (1UL << 0))) { /* Wait for SB (Start Bit) */ WDT_Reset(); } // SR1_SB

        // 2. Send slave address (write)
        // For simplicity, using a dummy address here. In real applications, this would be a parameter.
        tbyte slave_address = 0x50; // Example slave address (7-bit, then LSB for R/W)
        I2Cx->DR = (tbyte)(slave_address << 1); // Send address with R/W bit 0 (write)
        while (!(I2Cx->SR1 & (1UL << 1))) { /* Wait for ADDR (Address sent) */ WDT_Reset(); } // SR1_ADDR
        (void)I2Cx->SR2; // Clear ADDR by reading SR2

        // 3. Send data bytes
        for (int i = 0; i < length; i++) {
            I2C_send_byte(i2c_channel, (tbyte)data[i]);
            WDT_Reset();
        }

        // 4. Generate STOP condition (or repeated start if another transaction follows)
        // Rule: "Always generate a repeated start condition instead of stop between transactions"
        // This implies the current transaction is *not* the last in a sequence,
        // so a STOP is NOT generated here directly. The application or next function call
        // for a new transaction will generate a REPEATED START.
        // If this were the end of a master transaction, a STOP would be generated:
        // I2Cx->CR1 |= (1UL << 9); // Set STOP bit
    }
}

/**
 * @brief Sends a null-terminated string over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset();
    if (str != NULL) {
        I2C_send_frame(i2c_channel, str, strlen(str));
    }
}

/**
 * @brief Receives a single byte from I2C.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 *
 * This function expects the bus to be in master receive mode after address and read bit.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset();
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);
    tbyte received_byte = 0;

    if (I2Cx != NULL) {
        // Wait until RXNE (Receive data register not empty) is set
        while (!(I2Cx->SR1 & (1UL << 6))) { /* Wait for RxNE */ WDT_Reset(); }
        received_byte = (tbyte)(I2Cx->DR & 0xFFUL); // Read data from DR
    }
    return received_byte;
}

/**
 * @brief Receives a frame of data from I2C.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to receive.
 *
 * This function initiates a START condition, sends slave address with read bit,
 * receives all data bytes, and then generates a STOP condition.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset();
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);

    if ((I2Cx != NULL) && (buffer != NULL) && (max_length > 0)) {
        // 1. Generate START condition
        I2Cx->CR1 |= (1UL << 8); // Set START bit
        while (!(I2Cx->SR1 & (1UL << 0))) { /* Wait for SB (Start Bit) */ WDT_Reset(); } // SR1_SB

        // 2. Send slave address (read)
        // For simplicity, using a dummy address here.
        tbyte slave_address = 0x50; // Example slave address
        I2Cx->DR = (tbyte)((slave_address << 1) | 0x01UL); // Send address with R/W bit 1 (read)
        while (!(I2Cx->SR1 & (1UL << 1))) { /* Wait for ADDR (Address sent) */ WDT_Reset(); } // SR1_ADDR
        (void)I2Cx->SR2; // Clear ADDR by reading SR2

        // 3. Receive data bytes
        for (int i = 0; i < max_length; i++) {
            if (i == (max_length - 1)) {
                I2Cx->CR1 &= ~(1UL << 10); // Clear ACK bit for NACK on last byte
                I2Cx->CR1 |= (1UL << 9);   // Generate STOP condition after this byte
            }
            buffer[i] = (char)I2C_Get_Byte(i2c_channel);
            WDT_Reset();
        }
    }
}

/**
 * @brief Receives a null-terminated string from I2C.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum number of bytes to receive (including null terminator).
 * @return The number of bytes received (excluding null terminator).
 *
 * This function is similar to I2C_Get_frame, but adds a null terminator.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset();
    tbyte count = 0;
    if ((buffer != NULL) && (max_length > 0)) {
        I2C_Get_frame(i2c_channel, buffer, max_length - 1); // Get (max_length - 1) bytes
        count = (tbyte)(max_length - 1);
        buffer[count] = '\0'; // Null-terminate
    }
    return count;
}

/**
 * @brief Clears flags for an I2C channel.
 *
 * This function clears relevant status flags in the I2C Status Registers.
 * @param i2c_channel The I2C channel.
 */
void I2C_ClearFlag(t_i2c_channel i2c_channel) {
    WDT_Reset();
    volatile I2C_TypeDef* I2Cx = get_i2c_base(i2c_channel);
    if (I2Cx != NULL) {
        // Clearing flags in I2C can be complex. Some are cleared by read sequence (e.g., ADDR, STOPF).
        // Some might be cleared by writing 0 to them if they are write-to-clear.
        // For example, if an Overrun/Underrun error flag exists:
        // I2Cx->SR1 &= ~(1UL << 8); // Clear OVR flag (if write-to-clear)
        // Generally, the flags in SR1 and SR2 are read-only and cleared by specific event sequences.
        // This function would be a placeholder for specific error clearing logic.
        // For example, if a NACK (AF) error occurred, it needs to be cleared:
        // I2Cx->SR1 &= ~(1UL << 10); // Clear AF flag (if write-to-clear)
    }
}

/**
 * @brief Initializes a SPI channel.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode The SPI mode (Master/Slave).
 * @param spi_cpol Clock Polarity.
 * @param spi_cpha Clock Phase.
 * @param spi_dff Data Frame Format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset();
    volatile SPI_TypeDef* SPIx = get_spi_base(spi_channel);

    if (SPIx != NULL) {
        // Disable SPI peripheral before configuration
        SPIx->CR1 &= ~(1UL << 6); // Clear SPE bit (SPI Enable)
        WDT_Reset();

        // Configure Master/Slave mode (CR1_MSTR bit 2)
        if (spi_mode == SPI_MODE_MASTER) {
            SPIx->CR1 |= (1UL << 2); // Set MSTR bit
            // Always set Slave Select as software-controlled for master (CR1_SSM bit 9, CR1_SSI bit 8)
            SPIx->CR1 |= (1UL << 9); // Set SSM (Software Slave Management)
            SPIx->CR1 |= (1UL << 8); // Set SSI (Internal slave select)
        } else { // SPI_MODE_SLAVE
            SPIx->CR1 &= ~(1UL << 2); // Clear MSTR bit
            // For slave, SSM is typically set to 1 and SSI depends on configuration, or NSS pin is used
            SPIx->CR1 |= (1UL << 9); // Set SSM (Software Slave Management)
            SPIx->CR1 &= ~(1UL << 8); // Clear SSI (internal slave select, usually 0 for external NSS)
        }
        WDT_Reset();

        // Configure Clock Polarity (CR1_CPOL bit 1)
        if (spi_cpol == SPI_CPOL_HIGH) {
            SPIx->CR1 |= (1UL << 1);
        } else {
            SPIx->CR1 &= ~(1UL << 1);
        }
        WDT_Reset();

        // Configure Clock Phase (CR1_CPHA bit 0)
        if (spi_cpha == SPI_CPHA_2EDGE) {
            SPIx->CR1 |= (1UL << 0);
        } else {
            SPIx->CR1 &= ~(1UL << 0);
        }
        WDT_Reset();

        // Configure Data Frame Format (CR1_DFF bit 11)
        if (spi_dff == SPI_DFF_16BIT) {
            SPIx->CR1 |= (1UL << 11);
        } else {
            SPIx->CR1 &= ~(1UL << 11);
        }
        WDT_Reset();

        // Configure Bit Order (CR1_LSBFIRST bit 7)
        if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST) {
            SPIx->CR1 |= (1UL << 7);
        } else {
            SPIx->CR1 &= ~(1UL << 7);
        }
        WDT_Reset();

        // Always use fast speed (CR1_BR bits 5:3) - set prescaler to minimum (divide by 2)
        SPIx->CR1 &= ~(0x7UL << 3); // Clear BR bits
        SPIx->CR1 |= (0x0UL << 3);  // BR = 000 (PCLK / 2)
        WDT_Reset();

        // Always use full duplex (CR1_BIDIMODE bit 15, CR1_RXONLY bit 10)
        SPIx->CR1 &= ~(1UL << 15); // Clear BIDIMODE for 2-line unidirectional data mode (full duplex)
        SPIx->CR1 &= ~(1UL << 10); // Clear RXONLY (full-duplex)
        WDT_Reset();

        // Always enable CRC (CR1_CRCEN bit 13)
        SPIx->CR1 |= (1UL << 13); // Set CRCEN bit
        WDT_Reset();

        // Re-enable SPI peripheral
        SPIx->CR1 |= (1UL << 6); // Set SPE bit (SPI Enable)
    }
}

/**
 * @brief Enables a SPI channel.
 *
 * This function enables the clock for the specified SPI peripheral and then
 * enables the SPI module itself.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset();

    // Enable peripheral clock
    if (spi_channel == SPI_CHANNEL_1) {
        RCC_APB2ENR |= RCC_APB2ENR_SPI1EN; // Inferred
    } else if (spi_channel == SPI_CHANNEL_2) {
        RCC_APB1ENR |= RCC_APB1ENR_SPI2EN; // Inferred
    } else if (spi_channel == SPI_CHANNEL_3) {
        RCC_APB1ENR |= RCC_APB1ENR_SPI3EN; // Inferred
    }
    WDT_Reset();

    // Enable SPI module
    volatile SPI_TypeDef* SPIx = get_spi_base(spi_channel);
    if (SPIx != NULL) {
        SPIx->CR1 |= (1UL << 6); // Set SPE bit (SPI Enable)
    }
}

/**
 * @brief Disables a SPI channel.
 *
 * This function disables the SPI module and its peripheral clock.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset();
    volatile SPI_TypeDef* SPIx = get_spi_base(spi_channel);
    if (SPIx != NULL) {
        SPIx->CR1 &= ~(1UL << 6); // Clear SPE bit (SPI Enable)
    }

    // Disable peripheral clock
    if (spi_channel == SPI_CHANNEL_1) {
        RCC_APB2ENR &= ~RCC_APB2ENR_SPI1EN; // Inferred
    } else if (spi_channel == SPI_CHANNEL_2) {
        RCC_APB1ENR &= ~RCC_APB1ENR_SPI2EN; // Inferred
    } else if (spi_channel == SPI_CHANNEL_3) {
        RCC_APB1ENR &= ~RCC_APB1ENR_SPI3EN; // Inferred
    }
}

/**
 * @brief Updates a SPI channel (placeholder).
 */
void SPI_Update(void) {
    WDT_Reset();
    // Specific update logic (e.g., refreshing internal state, checking errors)
    // would be implemented here. For now, a placeholder.
}

/**
 * @brief Sends a single byte over SPI.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset();
    volatile SPI_TypeDef* SPIx = get_spi_base(spi_channel);
    if (SPIx != NULL) {
        // Wait for TXE (Transmit buffer empty) flag to be set
        while (!(SPIx->SR & (1UL << 1))) { /* Wait for TXE */ WDT_Reset(); }
        SPIx->DR = byte; // Write data to DR
        // Wait for BSY (Busy flag) to be cleared if master mode (optional for single byte)
        while (SPIx->SR & (1UL << 7)) { /* Wait for BSY */ WDT_Reset(); }
    }
}

/**
 * @brief Sends a frame of data over SPI.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset();
    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over SPI.
 * @param spi_channel The SPI channel to use.
 * @param str Pointer to the null-terminated string.
 */
void SPI_send_string(t_spi_channel spi_channel, const char *str) {
    WDT_Reset();
    while (*str != '\0') {
        SPI_Send_Byte(spi_channel, (tbyte)*str);
        str++;
    }
}

/**
 * @brief Receives a single byte from SPI.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset();
    volatile SPI_TypeDef* SPIx = get_spi_base(spi_channel);
    tbyte received_byte = 0;
    if (SPIx != NULL) {
        // To receive data in full-duplex, you usually need to transmit dummy data
        // Wait for TXE to be set
        while (!(SPIx->SR & (1UL << 1))) { /* Wait for TXE */ WDT_Reset(); }
        SPIx->DR = 0xFF; // Send dummy byte (for master to generate clock)

        // Wait for RXNE (Receive buffer not empty) flag to be set
        while (!(SPIx->SR & (1UL << 0))) { /* Wait for RXNE */ WDT_Reset(); }
        received_byte = (tbyte)(SPIx->DR & 0xFFUL); // Read data from DR
    }
    return received_byte;
}

/**
 * @brief Receives a frame of data from SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset();
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Receives a null-terminated string from SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum number of bytes to receive (including null terminator).
 * @return The number of bytes received (excluding null terminator).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset();
    tbyte count = 0;
    if ((buffer != NULL) && (max_length > 0)) {
        SPI_Get_frame(spi_channel, buffer, max_length - 1); // Get (max_length - 1) bytes
        count = (tbyte)(max_length - 1);
        buffer[count] = '\0'; // Null-terminate
    }
    return count;
}

/**
 * @brief Clears flags for a SPI channel.
 *
 * This function clears relevant status flags in the SPI Status Register.
 * @param spi_channel The SPI channel.
 */
void SPI_ClearFlag(t_spi_channel spi_channel) {
    WDT_Reset();
    volatile SPI_TypeDef* SPIx = get_spi_base(spi_channel);
    if (SPIx != NULL) {
        // Clearing flags in SPI depends on the specific flag.
        // For example, OVR (Overrun error) is cleared by reading SR then reading DR.
        // CRCERR (CRC Error) is cleared by writing 0 to the bit.
        SPIx->SR &= ~(1UL << 4); // Clear CRCERR flag (if write-to-clear)
    }
}

/**
 * @brief Initializes an External Interrupt channel.
 * @param external_int_channel The EXTI line (0-15).
 * @param external_int_edge The trigger edge (rising, falling, or both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset();

    // Enable SYSCFG clock (for EXTI configuration)
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Inferred
    WDT_Reset();

    // Configure the EXTI line source (GPIO Port) in SYSCFG_EXTICRx
    // The specific GPIO port for each EXTI line would need to be passed or derived.
    // For this generic implementation, we assume the desired port-pin connection is managed higher up.
    // Example: For EXTI0, select port A (0x0) or B (0x1) etc.
    // SYSCFG_EXTICR1: EXTI0[3:0], EXTI1[7:4], EXTI2[11:8], EXTI3[15:12]
    // SYSCFG_EXTICR2: EXTI4[3:0], EXTI5[7:4], EXTI6[11:8], EXTI7[15:12]
    // SYSCFG_EXTICR3: EXTI8[3:0], EXTI9[7:4], EXTI10[11:8], EXTI11[15:12]
    // SYSCFG_EXTICR4: EXTI12[3:0], EXTI13[7:4], EXTI14[11:8], EXTI15[15:12]

    // This implementation will only configure the trigger edge and mask, not the GPIO source.
    // A specific GPIO port for the EXTI line must be specified for SYSCFG_EXTICRx configuration.
    // Example: Assuming GPIOA for EXTI0 (0x0 in SYSCFG_EXTICR1 for EXTI0)
    tlong exti_line_mask = (1UL << external_int_channel);

    // Clear previous trigger settings
    EXTI_RTSR &= ~exti_line_mask;
    EXTI_FTSR &= ~exti_line_mask;
    WDT_Reset();

    // Set new trigger edge
    if (external_int_edge == EXTERNAL_INT_EDGE_RISING) {
        EXTI_RTSR |= exti_line_mask;
    } else if (external_int_edge == EXTERNAL_INT_EDGE_FALLING) {
        EXTI_FTSR |= exti_line_mask;
    } else { // EXTERNAL_INT_EDGE_BOTH
        EXTI_RTSR |= exti_line_mask;
        EXTI_FTSR |= exti_line_mask;
    }
    WDT_Reset();

    // Also unmask the interrupt line (EXTI_IMR) but do not enable NVIC here, use External_INT_Enable
    EXTI_IMR |= exti_line_mask; // Unmask interrupt request
}

/**
 * @brief Enables an External Interrupt channel.
 *
 * This function enables the interrupt request for the specified EXTI line.
 * It also enables the corresponding NVIC interrupt, assuming default priorities.
 * @param external_int_channel The EXTI line to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset();
    tlong exti_line_mask = (1UL << external_int_channel);

    // Unmask the interrupt line in EXTI
    EXTI_IMR |= exti_line_mask;
    WDT_Reset();

    // Enable the corresponding EXTI interrupt in the NVIC
    // This requires mapping EXTI lines to NVIC interrupt numbers
    // For STM32F401, EXTI0 to EXTI4 have dedicated IRQ channels.
    // EXTI5-9 share a single IRQ channel.
    // EXTI10-15 share another IRQ channel.
    IRQn_Type exti_irqn; // Inferred from STM32F401
    switch (external_int_channel) {
        case EXTERNAL_INT_LINE_0:  exti_irqn = EXTI0_IRQn; break;
        case EXTERTI_INT_LINE_1:  exti_irqn = EXTI1_IRQn; break;
        case EXTERNAL_INT_LINE_2:  exti_irqn = EXTI2_IRQn; break;
        case EXTERNAL_INT_LINE_3:  exti_irqn = EXTI3_IRQn; break;
        case EXTERNAL_INT_LINE_4:  exti_irqn = EXTI4_IRQn; break;
        case EXTERNAL_INT_LINE_5:
        case EXTERNAL_INT_LINE_6:
        case EXTERNAL_INT_LINE_7:
        case EXTERNAL_INT_LINE_8:
        case EXTERNAL_INT_LINE_9:  exti_irqn = EXTI9_5_IRQn; break;
        case EXTERNAL_INT_LINE_10:
        case EXTERNAL_INT_LINE_11:
        case EXTERNAL_INT_LINE_12:
        case EXTERNAL_INT_LINE_13:
        case EXTERNAL_INT_LINE_14:
        case EXTERNAL_INT_LINE_15: exti_irqn = EXTI15_10_IRQn; break;
        default: return; // Invalid EXTI line
    }
    NVIC_EnableIRQ(exti_irqn);
}

/**
 * @brief Disables an External Interrupt channel.
 *
 * This function disables the interrupt request for the specified EXTI line
 * and disables the corresponding NVIC interrupt.
 * @param external_int_channel The EXTI line to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset();
    tlong exti_line_mask = (1UL << external_int_channel);

    // Mask the interrupt line in EXTI
    EXTI_IMR &= ~exti_line_mask;
    WDT_Reset();

    // Disable the corresponding EXTI interrupt in the NVIC
    IRQn_Type exti_irqn; // Inferred from STM32F401
    switch (external_int_channel) {
        case EXTERNAL_INT_LINE_0:  exti_irqn = EXTI0_IRQn; break;
        case EXTERNAL_INT_LINE_1:  exti_irqn = EXTI1_IRQn; break;
        case EXTERNAL_INT_LINE_2:  exti_irqn = EXTI2_IRQn; break;
        case EXTERNAL_INT_LINE_3:  exti_irqn = EXTI3_IRQn; break;
        case EXTERNAL_INT_LINE_4:  exti_irqn = EXTI4_IRQn; break;
        case EXTERNAL_INT_LINE_5:
        case EXTERNAL_INT_LINE_6:
        case EXTERNAL_INT_LINE_7:
        case EXTERNAL_INT_LINE_8:
        case EXTERNAL_INT_LINE_9:  exti_irqn = EXTI9_5_IRQn; break;
        case EXTERNAL_INT_LINE_10:
        case EXTERNAL_INT_LINE_11:
        case EXTERNAL_INT_LINE_12:
        case EXTERNAL_INT_LINE_13:
        case EXTERNAL_INT_LINE_14:
        case EXTERNAL_INT_LINE_15: exti_irqn = EXTI15_10_IRQn; break;
        default: return; // Invalid EXTI line
    }
    NVIC_DisableIRQ(exti_irqn);
}

/**
 * @brief Clears the pending flag for an External Interrupt channel.
 *
 * This function clears the pending bit for the specified EXTI line.
 * @param external_int_channel The EXTI line to clear the flag for.
 */
void External_INT_ClearFlag(t_external_int_channel external_int_channel) {
    WDT_Reset();
    EXTI_PR = (1UL << external_int_channel); // Write 1 to clear the pending bit
}

/**
 * @brief Initializes a GPIO pin as output.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset();
    volatile tlong* p_odr = get_gpio_odr_reg(port);
    volatile tlong* p_moder = get_gpio_moder_reg(port);
    volatile tlong* p_otyper = get_gpio_otyper_reg(port);
    volatile tlong* p_ospeedr = get_gpio_ospeedr_reg(port);
    volatile tlong* p_pupdr = get_gpio_pupdr_reg(port);

    if ((p_odr != NULL) && (p_moder != NULL) && (p_otyper != NULL) && (p_ospeedr != NULL) && (p_pupdr != NULL)) {
        // 1. Set output value before setting direction (GPIO rule)
        GPIO_Value_Set(port, pin, value);
        WDT_Reset();

        // 2. Set pin mode to General purpose output mode (01)
        *p_moder &= ~(0x3UL << (pin * 2)); // Clear mode bits
        *p_moder |= (0x1UL << (pin * 2));  // Set to output mode (01)

        // 3. Output type: Push-pull (0) or Open-drain (1) - default to Push-pull (OTYPER bit)
        *p_otyper &= ~(1UL << pin); // Set to Push-pull (0)

        // 4. Output speed: Very high speed (11) (for >=20mA sink current & >=10mA source current)
        *p_ospeedr &= ~(0x3UL << (pin * 2)); // Clear speed bits
        *p_ospeedr |= (0x3UL << (pin * 2));  // Set to Very high speed (11)

        // 5. Pull-up/Pull-down: No pull-up/pull-down (00) (GPIO rule for output pins)
        *p_pupdr &= ~(0x3UL << (pin * 2)); // Clear pull-up/pull-down bits (00 for no PUPD)
        WDT_Reset();

        // 6. Verify with while loop (mode setting)
        while (((*p_moder >> (pin * 2)) & 0x3UL) != 0x1UL) { /* Wait for mode to be set */ WDT_Reset(); }
        // Verify output value again
        tbyte current_value = GPIO_Value_Get(port, pin);
        while (current_value != value) {
            GPIO_Value_Set(port, pin, value);
            current_value = GPIO_Value_Get(port, pin);
            WDT_Reset();
        }
    }
}

/**
 * @brief Initializes a GPIO pin as input.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 */
void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset();
    volatile tlong* p_moder = get_gpio_moder_reg(port);
    volatile tlong* p_pupdr = get_gpio_pupdr_reg(port);

    if ((p_moder != NULL) && (p_pupdr != NULL)) {
        // 1. Set pin mode to Input mode (00)
        *p_moder &= ~(0x3UL << (pin * 2)); // Clear mode bits (00 for input)

        // 2. All input pins have pull-up resistors (GPIO rule)
        *p_pupdr &= ~(0x3UL << (pin * 2)); // Clear pull-up/pull-down bits
        *p_pupdr |= (0x1UL << (pin * 2));  // Set to Pull-up (01)
        WDT_Reset();

        // 3. Verify with while loop
        while (((*p_moder >> (pin * 2)) & 0x3UL) != 0x0UL) { /* Wait for mode to be set */ WDT_Reset(); }
        while (((*p_pupdr >> (pin * 2)) & 0x3UL) != 0x1UL) { /* Wait for pull-up to be set */ WDT_Reset(); }
    }
}

/**
 * @brief Gets the direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @return The direction of the pin (Input or Output).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset();
    volatile tlong* p_moder = get_gpio_moder_reg(port);
    t_direction direction = GPIO_DIRECTION_INPUT; // Default to input

    if (p_moder != NULL) {
        if (((*p_moder >> (pin * 2)) & 0x3UL) == 0x0UL) {
            direction = GPIO_DIRECTION_INPUT;
        } else if (((*p_moder >> (pin * 2)) & 0x3UL) == 0x1UL) {
            direction = GPIO_DIRECTION_OUTPUT;
        } else {
            // Analog or Alternate function mode, treated as neither direct input nor output
            // For this API, returning input as a safe default.
            direction = GPIO_DIRECTION_INPUT;
        }
    }
    return direction;
}

/**
 * @brief Sets the output value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @param value The value to set (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset();
    volatile tlong* p_bsrr = get_gpio_bsrr_reg(port);

    if (p_bsrr != NULL) {
        if (value == 1) {
            *p_bsrr = (1UL << pin); // Set bit
        } else {
            *p_bsrr = (1UL << (pin + 16)); // Reset bit
        }
        WDT_Reset();

        // Verify value after setting (GPIO rule)
        tbyte current_value = GPIO_Value_Get(port, pin);
        while (current_value != value) {
            // Re-attempt setting the value if verification fails
            if (value == 1) {
                *p_bsrr = (1UL << pin);
            } else {
                *p_bsrr = (1UL << (pin + 16));
            }
            current_value = GPIO_Value_Get(port, pin);
            WDT_Reset();
        }
    }
}

/**
 * @brief Gets the input value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @return The input value (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset();
    volatile tlong* p_idr = get_gpio_idr_reg(port);
    tbyte value = 0;
    if (p_idr != NULL) {
        value = (tbyte)((*p_idr >> pin) & 0x1UL);
    }
    return value;
}

/**
 * @brief Toggles the output value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 */
void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset();
    volatile tlong* p_odr = get_gpio_odr_reg(port);
    if (p_odr != NULL) {
        *p_odr ^= (1UL << pin); // Toggle the bit
        WDT_Reset();
        // Verification (optional for toggle, but good practice if it matters)
        // tbyte expected_value = ((*p_odr >> pin) & 0x1UL); // Get the value that was *just* set
        // while (GPIO_Value_Get(port, pin) != expected_value) {
        //     *p_odr ^= (1UL << pin); // Re-toggle if verification fails
        //     expected_value = ((*p_odr >> pin) & 0x1UL);
        //     WDT_Reset();
        // }
    }
}

/**
 * @brief Initializes a PWM channel.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    tbyte timer_ch = 0;
    get_timer_for_pwm(pwm_channel, &TIMx, &timer_ch);

    if ((TIMx != NULL) && (timer_ch > 0)) {
        // Enable timer clock (inferred for specific TIMx)
        if (TIMx == (volatile TIM_TypeDef*)0x40010000) { // TIM1
            RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000000) { // TIM2
            RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000400) { // TIM3
            RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000800) { // TIM4
            RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000C00) { // TIM5
            RCC_APB1ENR |= RCC_APB1ENR_TIM5EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40014000) { // TIM9
            RCC_APB2ENR |= RCC_APB2ENR_TIM9EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40014400) { // TIM10
            RCC_APB2ENR |= RCC_APB2ENR_TIM10EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40014800) { // TIM11
            RCC_APB2ENR |= RCC_APB2ENR_TIM11EN;
        }
        WDT_Reset();

        // Disable timer (CR1_CEN bit 0)
        TIMx->CR1 &= ~0x1UL;
        WDT_Reset();

        // Assuming SystemCoreClock is available, or inferring PCLKs
        tlong pclk_freq = 0; // Default to 0
        // TIM1, TIM9, TIM10, TIM11 are on APB2
        if ((TIMx == (volatile TIM_TypeDef*)0x40010000) || (TIMx == (volatile TIM_TypeDef*)0x40014000) ||
            (TIMx == (volatile TIM_TypeDef*)0x40014400) || (TIMx == (volatile TIM_TypeDef*)0x40014800)) {
            pclk_freq = 84000000UL; // Max APB2 clock for F401
        } else { // TIM2, TIM3, TIM4, TIM5 are on APB1
            pclk_freq = 42000000UL; // Max APB1 clock for F401
        }
        
        // Calculate prescaler and auto-reload register (ARR) value
        // ARR = (PCLK / (PSC + 1)) / (PWM_FREQ * 1000) - 1
        // For simplicity, let's assume a fixed prescaler and adjust ARR, or vice versa.
        // Let's target a high resolution, so (PSC + 1) * ARR = PCLK / (PWM_FREQ_HZ)
        tlong target_freq_hz = (tlong)pwm_khz_freq * 1000UL;
        tword prescaler = 0;
        tword arr_value = 0;

        if (target_freq_hz > 0) {
            // Aim for a period of 65535 (max for 16-bit timer), calculate prescaler
            prescaler = (tword)((pclk_freq / target_freq_hz / 65535UL) + 1UL); // Initial estimate
            if (prescaler == 0) prescaler = 1; // Ensure prescaler is at least 1
            if (prescaler > 65535) prescaler = 65535; // Clamp to max 16-bit
            
            arr_value = (tword)((pclk_freq / ((prescaler + 1UL) * target_freq_hz)) - 1UL);
        } else {
             arr_value = 0xFFFF; // Max period if freq is 0
             prescaler = (tword)((pclk_freq / ((arr_value + 1UL) * 1000UL)) - 1UL);
             if (prescaler < 0) prescaler = 0;
        }

        // Set prescaler and auto-reload value
        TIMx->PSC = prescaler;
        TIMx->ARR = arr_value;
        WDT_Reset();

        // Configure Output Compare Mode for PWM
        tlong ccmr_val = 0;
        tlong ccer_val = 0;
        volatile tlong* p_ccr = NULL; // Pointer to CCRx register

        // Select PWM mode 1 (OCxM = 110, CCxS = 00)
        // Clear Output Compare Mode bits first (OCxM and CCxS)
        if (timer_ch == 1) {
            TIMx->CCMR1 &= ~((0x7UL << 4) | (0x3UL << 0)); // Clear OC1M, CC1S
            TIMx->CCMR1 |= (0x6UL << 4); // Set OC1M to 110 (PWM Mode 1)
            p_ccr = &TIMx->CCR1;
        } else if (timer_ch == 2) {
            TIMx->CCMR1 &= ~((0x7UL << 12) | (0x3UL << 8)); // Clear OC2M, CC2S
            TIMx->CCMR1 |= (0x6UL << 12); // Set OC2M to 110 (PWM Mode 1)
            p_ccr = &TIMx->CCR2;
        } else if (timer_ch == 3) {
            TIMx->CCMR2 &= ~((0x7UL << 4) | (0x3UL << 0)); // Clear OC3M, CC3S
            TIMx->CCMR2 |= (0x6UL << 4); // Set OC3M to 110 (PWM Mode 1)
            p_ccr = &TIMx->CCR3;
        } else if (timer_ch == 4) {
            TIMx->CCMR2 &= ~((0x7UL << 12) | (0x3UL << 8)); // Clear OC4M, CC4S
            TIMx->CCMR2 |= (0x6UL << 12); // Set OC4M to 110 (PWM Mode 1)
            p_ccr = &TIMx->CCR4;
        }
        WDT_Reset();

        // Preload enable (ARPE, OCxPE)
        TIMx->CR1 |= (1UL << 7); // ARR preload enable
        // Enable output compare preload (OCxPE)
        if (timer_ch == 1) TIMx->CCMR1 |= (1UL << 3);
        else if (timer_ch == 2) TIMx->CCMR1 |= (1UL << 11);
        else if (timer_ch == 3) TIMx->CCMR2 |= (1UL << 3);
        else if (timer_ch == 4) TIMx->CCMR2 |= (1UL << 11);
        WDT_Reset();

        // Configure Capture/Compare Enable Register (CCER)
        // Enable output (CCxE) and set polarity (CCxP=0 for active high)
        tlong ccer_ch_mask = (0x1UL << ((timer_ch - 1) * 4)); // CCxE bit
        TIMx->CCER |= ccer_ch_mask; // Enable output for selected channel
        WDT_Reset();
        
        // Set duty cycle (CCRx value)
        if (p_ccr != NULL) {
            *p_ccr = (tword)(((tlong)arr_value * pwm_duty) / 100UL);
        }
        WDT_Reset();

        // For advanced timers (TIM1), enable main output (BDTR_MOE bit)
        if (TIMx == (volatile TIM_TypeDef*)0x40010000) { // TIM1
            TIMx->BDTR |= (1UL << 15); // Set MOE bit (Main Output Enable)
        }
        WDT_Reset();
        
        // Generate an update event to load the prescaler and ARR values (EGR_UG bit 0)
        TIMx->EGR |= (1UL << 0);
        WDT_Reset();

        // Clear available FREQUENCY Ranges for each channel as comments (PWM_requirements rule)
        // TIM1 CH1: 0 - (PCLK / (PSC + 1)) Hz. Example: PCLK=84MHz, PSC=0 => 84MHz to 1.28 kHz (max ARR)
        // TIM2 CH1: 0 - (PCLK / (PSC + 1)) Hz. Example: PCLK=42MHz, PSC=0 => 42MHz to 641 Hz (max ARR)
        // ... (This would be extensive, general comment suffices for example)
    }
}

/**
 * @brief Starts a PWM channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    tbyte timer_ch = 0;
    get_timer_for_pwm(pwm_channel, &TIMx, &timer_ch);

    if (TIMx != NULL) {
        TIMx->CR1 |= (1UL << 0); // Set CEN bit (Counter Enable)
    }
}

/**
 * @brief Stops a PWM channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    tbyte timer_ch = 0;
    get_timer_for_pwm(pwm_channel, &TIMx, &timer_ch);

    if (TIMx != NULL) {
        TIMx->CR1 &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
    }
}

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler for the timer.
 * @param icu_edge The detection edge (rising, falling, or both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    tbyte timer_ch = 0;
    get_timer_for_pwm(icu_channel, &TIMx, &timer_ch); // Reuse PWM channel mapping for ICU

    if ((TIMx != NULL) && (timer_ch > 0)) {
        // Enable timer clock (inferred for specific TIMx)
        if (TIMx == (volatile TIM_TypeDef*)0x40010000) { // TIM1
            RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000000) { // TIM2
            RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000400) { // TIM3
            RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000800) { // TIM4
            RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40000C00) { // TIM5
            RCC_APB1ENR |= RCC_APB1ENR_TIM5EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40014000) { // TIM9
            RCC_APB2ENR |= RCC_APB2ENR_TIM9EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40014400) { // TIM10
            RCC_APB2ENR |= RCC_APB2ENR_TIM10EN;
        } else if (TIMx == (volatile TIM_TypeDef*)0x40014800) { // TIM11
            RCC_APB2ENR |= RCC_APB2ENR_TIM11EN;
        }
        WDT_Reset();

        // Disable timer counter during configuration
        TIMx->CR1 &= ~0x1UL;
        WDT_Reset();

        // Configure Prescaler (PSC)
        TIMx->PSC = icu_prescaller; // Direct mapping, ensure enum values match (0 for DIV1, 1 for DIV2, etc.)
        WDT_Reset();

        // Configure Input Capture Mode and Filter
        // CCxS = 01 (Input, ICx is mapped on TIx)
        // Clear CCxS bits first
        if (timer_ch == 1) {
            TIMx->CCMR1 &= ~(0x3UL << 0); // Clear CC1S
            TIMx->CCMR1 |= (0x1UL << 0);  // Set CC1S to 01
        } else if (timer_ch == 2) {
            TIMx->CCMR1 &= ~(0x3UL << 8); // Clear CC2S
            TIMx->CCMR1 |= (0x1UL << 8);  // Set CC2S to 01
        } else if (timer_ch == 3) {
            TIMx->CCMR2 &= ~(0x3UL << 0); // Clear CC3S
            TIMx->CCMR2 |= (0x1UL << 0);  // Set CC3S to 01
        } else if (timer_ch == 4) {
            TIMx->CCMR2 &= ~(0x3UL << 8); // Clear CC4S
            TIMx->CCMR2 |= (0x1UL << 8);  // Set CC4S to 01
        }
        WDT_Reset();

        // Configure Input Capture Edge (CCER_CCxP, CCER_CCxNP)
        // Clear edge selection bits first
        tlong ccer_clr_mask = (0x3UL << ((timer_ch - 1) * 4 + 1)); // CCxP and CCxNP
        TIMx->CCER &= ~ccer_clr_mask; // Clear polarity bits

        if (icu_edge == ICU_EDGE_RISING) {
            // CCxP = 0, CCxNP = 0 (rising edge)
        } else if (icu_edge == ICU_EDGE_FALLING) {
            // CCxP = 1, CCxNP = 0 (falling edge)
            TIMx->CCER |= (0x1UL << ((timer_ch - 1) * 4 + 1)); // Set CCxP
        } else { // ICU_EDGE_BOTH_EDGES
            // CCxP = 1, CCxNP = 1 (both edges - inverted + non-inverted, on STM32F4 it's often CCxP=1 and CCxNP=1 for both)
            TIMx->CCER |= (0x3UL << ((timer_ch - 1) * 4 + 1)); // Set both CCxP and CCxNP
        }
        WDT_Reset();

        // Enable the capture for the channel (CCER_CCxE bit)
        TIMx->CCER |= (0x1UL << ((timer_ch - 1) * 4));
        WDT_Reset();

        // Reset the counter (CNT) to 0 and ARR to max for continuous measurement
        TIMx->CNT = 0;
        TIMx->ARR = 0xFFFFFFFFUL; // Max value for continuous count
        WDT_Reset();
    }
}

/**
 * @brief Enables an ICU channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    tbyte timer_ch = 0;
    get_timer_for_pwm(icu_channel, &TIMx, &timer_ch);

    if (TIMx != NULL) {
        TIMx->CR1 |= (1UL << 0); // Set CEN bit (Counter Enable)
        // Enable capture/compare interrupt for the channel (DIER_CCxIE)
        TIMx->DIER |= (1UL << timer_ch); // CCxIE for channel x
    }
}

/**
 * @brief Disables an ICU channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    tbyte timer_ch = 0;
    get_timer_for_pwm(icu_channel, &TIMx, &timer_ch);

    if (TIMx != NULL) {
        TIMx->CR1 &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
        // Disable capture/compare interrupt for the channel (DIER_CCxIE)
        TIMx->DIER &= ~(1UL << timer_ch);
    }
}

/**
 * @brief Updates ICU frequency calculation (placeholder).
 * @param icu_channel The ICU channel.
 */
void ICU_Updatefrequency(t_icu_channel icu_channel) {
    WDT_Reset();
    (void)icu_channel; // Suppress unused parameter warning
    // This function typically involves re-calculating frequency based on latest capture values.
    // Real implementation would store capture values in an ISR and then compute here.
}

/**
 * @brief Gets the frequency measured by the ICU.
 * @param icu_channel The ICU channel.
 * @return The measured frequency (in Hz, placeholder implementation).
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset();
    (void)icu_channel; // Suppress unused parameter warning
    // To get frequency, two consecutive capture values are needed from the CCRx register.
    // This is a placeholder; actual implementation requires an ISR to store timestamps.
    return 0; // Placeholder for actual frequency calculation
}

/**
 * @brief Sets up the buffer for remote control keys (placeholder).
 * @param number_of_keys Number of keys.
 * @param key_digits_length Length of digits per key.
 */
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length) {
    WDT_Reset();
    (void)number_of_keys;   // Suppress unused parameter warning
    (void)key_digits_length; // Suppress unused parameter warning
    // This would involve allocating or configuring a memory buffer for key codes.
}

/**
 * @brief Sets remote control key digits (placeholder).
 * @param key_num Key number.
 * @param key_array_cell Array cell.
 * @param key_cell_value Cell value.
 */
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value) {
    WDT_Reset();
    (void)key_num;        // Suppress unused parameter warning
    (void)key_array_cell; // Suppress unused parameter warning
    (void)key_cell_value; // Suppress unused parameter warning
    // This would populate the remote control key buffer with specific digit patterns.
}

/**
 * @brief Updates remote control signal parameters (placeholder).
 * @param icu_channel The ICU channel.
 * @param strt_bit_us_value Start bit duration in microseconds.
 * @param one_bit_us_value One bit duration in microseconds.
 * @param zero_bit_us_value Zero bit duration in microseconds.
 * @param stop_bit_us_value Stop bit duration in microseconds.
 */
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value) {
    WDT_Reset();
    (void)icu_channel;           // Suppress unused parameter warning
    (void)strt_bit_us_value;     // Suppress unused parameter warning
    (void)one_bit_us_value;      // Suppress unused parameter warning
    (void)zero_bit_us_value;     // Suppress unused parameter warning
    (void)stop_bit_us_value;     // Suppress unused parameter warning
    // This would update internal parameters for decoding remote control signals.
}

/**
 * @brief Gets the pressed remote control key (placeholder).
 * @param icu_channel The ICU channel.
 * @return The detected key (0 if no key detected).
 */
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel) {
    WDT_Reset();
    (void)icu_channel; // Suppress unused parameter warning
    // This function would decode the received signal and return a key code.
    return 0; // Placeholder
}

/**
 * @brief Sets a callback function for ICU events.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset();
    ICU_UserCallback = callback;
}

/**
 * @brief Clears the pending flag for an ICU channel.
 * @param icu_channel The ICU channel.
 */
void ICU_ClearFlag(t_icu_channel icu_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    tbyte timer_ch = 0;
    get_timer_for_pwm(icu_channel, &TIMx, &timer_ch);

    if (TIMx != NULL) {
        // Clear Capture/Compare Interrupt Flag (CCxIF) by writing 0 to SR.
        // On STM32, CCxIF is cleared by reading the TIMx_SR register followed by reading the TIMx_CCRx register.
        (void)TIMx->SR; // Read SR
        if (timer_ch == 1)      (void)TIMx->CCR1;
        else if (timer_ch == 2) (void)TIMx->CCR2;
        else if (timer_ch == 3) (void)TIMx->CCR3;
        else if (timer_ch == 4) (void)TIMx->CCR4;
    }
}

/**
 * @brief Initializes a general-purpose timer channel.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    // Map timer_channel to TIM_TypeDef base address
    if (timer_channel == TIMER_CHANNEL_TIM1) TIMx = (volatile TIM_TypeDef*)0x40010000;
    else if (timer_channel == TIMER_CHANNEL_TIM2) TIMx = (volatile TIM_TypeDef*)0x40000000;
    else if (timer_channel == TIMER_CHANNEL_TIM3) TIMx = (volatile TIM_TypeDef*)0x40000400;
    else if (timer_channel == TIMER_CHANNEL_TIM4) TIMx = (volatile TIM_TypeDef*)0x40000800;
    else if (timer_channel == TIMER_CHANNEL_TIM5) TIMx = (volatile TIM_TypeDef*)0x40000C00;
    else if (timer_channel == TIMER_CHANNEL_TIM9) TIMx = (volatile TIM_TypeDef*)0x40014000;
    else if (timer_channel == TIMER_CHANNEL_TIM10) TIMx = (volatile TIM_TypeDef*)0x40014400;
    else if (timer_channel == TIMER_CHANNEL_TIM11) TIMx = (volatile TIM_TypeDef*)0x40014800;

    if (TIMx != NULL) {
        // Enable timer clock (inferred for specific TIMx)
        if (timer_channel == TIMER_CHANNEL_TIM1 || timer_channel == TIMER_CHANNEL_TIM9 ||
            timer_channel == TIMER_CHANNEL_TIM10 || timer_channel == TIMER_CHANNEL_TIM11) {
            RCC_APB2ENR |= (1UL << (timer_channel == TIMER_CHANNEL_TIM1 ? 0 : (timer_channel - TIMER_CHANNEL_TIM9 + 16)));
        } else { // APB1 timers
            RCC_APB1ENR |= (1UL << timer_channel);
        }
        WDT_Reset();

        // Disable timer counter during configuration
        TIMx->CR1 &= ~0x1UL;
        WDT_Reset();

        // Configure timer in Up-counting mode, disable auto-reload preload
        TIMx->CR1 = 0x0000; // Clear all control bits
        TIMx->CR1 &= ~(1UL << 7); // Disable ARPE (Auto-reload preload enable)

        // Clear interrupt enable flags
        TIMx->DIER = 0x0000;
        WDT_Reset();

        // Reset counter
        TIMx->CNT = 0;
        WDT_Reset();
    }
}

/**
 * @brief Sets a timer for a specific duration in microseconds.
 * @param timer_channel The timer channel.
 * @param time The duration in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    // Map timer_channel to TIM_TypeDef base address
    if (timer_channel == TIMER_CHANNEL_TIM1) TIMx = (volatile TIM_TypeDef*)0x40010000;
    else if (timer_channel == TIMER_CHANNEL_TIM2) TIMx = (volatile TIM_TypeDef*)0x40000000;
    else if (timer_channel == TIMER_CHANNEL_TIM3) TIMx = (volatile TIM_TypeDef*)0x40000400;
    else if (timer_channel == TIMER_CHANNEL_TIM4) TIMx = (volatile TIM_TypeDef*)0x40000800;
    else if (timer_channel == TIMER_CHANNEL_TIM5) TIMx = (volatile TIM_TypeDef*)0x40000C00;
    else if (timer_channel == TIMER_CHANNEL_TIM9) TIMx = (volatile TIM_TypeDef*)0x40014000;
    else if (timer_channel == TIMER_CHANNEL_TIM10) TIMx = (volatile TIM_TypeDef*)0x40014400;
    else if (timer_channel == TIMER_CHANNEL_TIM11) TIMx = (volatile TIM_TypeDef*)0x40014800;

    if (TIMx != NULL) {
        // Assuming SystemCoreClock is available, or inferring PCLKs
        tlong pclk_freq = 0; // Default to 0
        // TIM1, TIM9, TIM10, TIM11 are on APB2
        if (timer_channel == TIMER_CHANNEL_TIM1 || timer_channel == TIMER_CHANNEL_TIM9 ||
            timer_channel == TIMER_CHANNEL_TIM10 || timer_channel == TIMER_CHANNEL_TIM11) {
            pclk_freq = 84000000UL; // Max APB2 clock for F401
        } else { // TIM2, TIM3, TIM4, TIM5 are on APB1
            pclk_freq = 42000000UL; // Max APB1 clock for F401
        }

        // Calculate PSC and ARR for microsecond resolution
        // Period = (PSC + 1) * (ARR + 1) / PCLK_freq
        // For 1us tick, PCLK_freq / (PSC + 1) = 1MHz -> PSC = (PCLK_freq / 1MHz) - 1
        tword prescaler = (tword)((pclk_freq / 1000000UL) - 1UL);
        tword arr_value = time - 1;

        TIMx->PSC = prescaler;
        TIMx->ARR = arr_value;
        TIMx->CNT = 0; // Reset counter
        TIMx->EGR |= 0x1UL; // Generate an update event
        WDT_Reset();

        // Enable Update Interrupt (DIER_UIE)
        TIMx->DIER |= (1UL << 0);
    }
}

/**
 * @brief Sets a timer for a specific duration in milliseconds.
 * @param timer_channel The timer channel.
 * @param time The duration in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    WDT_Reset();
    // This calls the microsecond function, converting ms to us.
    // Max tword (65535) ms is 65.535 seconds. This is fine.
    TIMER_Set_us(timer_channel, time * 1000U);
}

/**
 * @brief Sets a timer for a specific duration in seconds.
 * @param timer_channel The timer channel.
 * @param time The duration in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    // This calls the millisecond function, converting seconds to ms.
    // Max tbyte (255) seconds is 4.25 minutes. This is fine.
    TIMER_Set_Time_ms(timer_channel, (tword)time * 1000U);
}

/**
 * @brief Sets a timer for a specific duration in minutes.
 * @param timer_channel The timer channel.
 * @param time The duration in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    // This calls the second function, converting minutes to seconds.
    // Max tbyte (255) minutes is 4.25 hours. This is fine.
    TIMER_Set_Time_sec(timer_channel, time * 60U);
}

/**
 * @brief Sets a timer for a specific duration in hours.
 * @param timer_channel The timer channel.
 * @param time The duration in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    // This calls the minute function, converting hours to minutes.
    // Max tbyte (255) hours is 10.6 days. This is fine.
    TIMER_Set_Time_min(timer_channel, time * 60U);
}

/**
 * @brief Enables a timer channel.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    // Map timer_channel to TIM_TypeDef base address
    if (timer_channel == TIMER_CHANNEL_TIM1) TIMx = (volatile TIM_TypeDef*)0x40010000;
    else if (timer_channel == TIMER_CHANNEL_TIM2) TIMx = (volatile TIM_TypeDef*)0x40000000;
    else if (timer_channel == TIMER_CHANNEL_TIM3) TIMx = (volatile TIM_TypeDef*)0x40000400;
    else if (timer_channel == TIMER_CHANNEL_TIM4) TIMx = (volatile TIM_TypeDef*)0x40000800;
    else if (timer_channel == TIMER_CHANNEL_TIM5) TIMx = (volatile TIM_TypeDef*)0x40000C00;
    else if (timer_channel == TIMER_CHANNEL_TIM9) TIMx = (volatile TIM_TypeDef*)0x40014000;
    else if (timer_channel == TIMER_CHANNEL_TIM10) TIMx = (volatile TIM_TypeDef*)0x40014400;
    else if (timer_channel == TIMER_CHANNEL_TIM11) TIMx = (volatile TIM_TypeDef*)0x40014800;

    if (TIMx != NULL) {
        TIMx->CR1 |= (1UL << 0); // Set CEN bit (Counter Enable)
    }
}

/**
 * @brief Disables a timer channel.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    // Map timer_channel to TIM_TypeDef base address
    if (timer_channel == TIMER_CHANNEL_TIM1) TIMx = (volatile TIM_TypeDef*)0x40010000;
    else if (timer_channel == TIMER_CHANNEL_TIM2) TIMx = (volatile TIM_TypeDef*)0x40000000;
    else if (timer_channel == TIMER_CHANNEL_TIM3) TIMx = (volatile TIM_TypeDef*)0x40000400;
    else if (timer_channel == TIMER_CHANNEL_TIM4) TIMx = (volatile TIM_TypeDef*)0x40000800;
    else if (timer_channel == TIMER_CHANNEL_TIM5) TIMx = (volatile TIM_TypeDef*)0x40000C00;
    else if (timer_channel == TIMER_CHANNEL_TIM9) TIMx = (volatile TIM_TypeDef*)0x40014000;
    else if (timer_channel == TIMER_CHANNEL_TIM10) TIMx = (volatile TIM_TypeDef*)0x40014400;
    else if (timer_channel == TIMER_CHANNEL_TIM11) TIMx = (volatile TIM_TypeDef*)0x40014800;

    if (TIMx != NULL) {
        TIMx->CR1 &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
    }
}

/**
 * @brief Clears the pending flag for a timer channel.
 * @param timer_channel The timer channel.
 */
void TIMER_ClearFlag(t_timer_channel timer_channel) {
    WDT_Reset();
    volatile TIM_TypeDef* TIMx = NULL;
    // Map timer_channel to TIM_TypeDef base address
    if (timer_channel == TIMER_CHANNEL_TIM1) TIMx = (volatile TIM_TypeDef*)0x40010000;
    else if (timer_channel == TIMER_CHANNEL_TIM2) TIMx = (volatile TIM_TypeDef*)0x40000000;
    else if (timer_channel == TIMER_CHANNEL_TIM3) TIMx = (volatile TIM_TypeDef*)0x40000400;
    else if (timer_channel == TIMER_CHANNEL_TIM4) TIMx = (volatile TIM_TypeDef*)0x40000800;
    else if (timer_channel == TIMER_CHANNEL_TIM5) TIMx = (volatile TIM_TypeDef*)0x40000C00;
    else if (timer_channel == TIMER_CHANNEL_TIM9) TIMx = (volatile TIM_TypeDef*)0x40014000;
    else if (timer_channel == TIMER_CHANNEL_TIM10) TIMx = (volatile TIM_TypeDef*)0x40014400;
    else if (timer_channel == TIMER_CHANNEL_TIM11) TIMx = (volatile TIM_TypeDef*)0x40014800;

    if (TIMx != NULL) {
        TIMx->SR &= ~(1UL << 0); // Clear UIF (Update Interrupt Flag)
    }
}

/**
 * @brief Initializes the ADC.
 * @param adc_channel The first ADC channel to be configured.
 * @param adc_mode The ADC operation mode (single, continuous, scan).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset();

    // Enable ADC1 clock
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN; // Inferred
    WDT_Reset();

    // Disable ADC before configuration
    ADC_CR2 &= ~(1UL << 0); // Clear ADON bit (ADC ON)
    WDT_Reset();

    // Clear CR1 and CR2 registers for a clean start
    ADC_CR1 = 0x00000000UL;
    ADC_CR2 = 0x00000000UL;
    WDT_Reset();

    // Configure ADC mode (CR2_CONT, CR1_SCAN)
    if (adc_mode == ADC_MODE_CONTINUOUS) {
        ADC_CR2 |= (1UL << 1); // Set CONT bit
    } else if (adc_mode == ADC_MODE_SCAN) {
        ADC_CR1 |= (1UL << 8); // Set SCAN bit
    }
    WDT_Reset();

    // Configure Resolution (CR1_RES bits 25:24) - default to 12-bit
    ADC_CR1 &= ~(0x3UL << 24); // Clear RES bits (00 for 12-bit)

    // Configure data alignment (CR2_ALIGN bit 11) - default to Right alignment
    ADC_CR2 &= ~(1UL << 11); // Clear ALIGN bit

    // Configure channel sequence (SQRx registers). Add the first channel to the sequence.
    // For single channel conversion, sequence length = 1 (SQR1_L = 0000)
    ADC_SQR3 = adc_channel; // Set the first conversion in the sequence
    ADC_SQR1 &= ~(0xFUL << 20); // Set L bits (23:20) to 0000 for 1 conversion in sequence

    // Configure sample time for the channel (SMPRx registers)
    // Default to 3 cycles sample time for example (000)
    if (adc_channel <= ADC_CHANNEL_9) { // Channels 0-9 in SMPR2
        ADC_SMPR2 &= ~(0x7UL << (adc_channel * 3)); // Clear SMP bits
        ADC_SMPR2 |= (0x0UL << (adc_channel * 3));  // Set 3 cycles sample time
    } else { // Channels 10-15 in SMPR1 (adjusted bit position)
        ADC_SMPR1 &= ~(0x7UL << ((adc_channel - 10) * 3)); // Clear SMP bits
        ADC_SMPR1 |= (0x0UL << ((adc_channel - 10) * 3));  // Set 3 cycles sample time
    }
    WDT_Reset();
}

/**
 * @brief Enables the ADC.
 *
 * This function enables the ADC module.
 */
void ADC_Enable(void) {
    WDT_Reset();
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock (inferred)
    WDT_Reset();
    ADC_CR2 |= (1UL << 0); // Set ADON bit (ADC ON)
    WDT_Reset();
    // Start conversion if not in continuous mode (CR2_SWSTART bit 30)
    ADC_CR2 |= (1UL << 30); // Software start conversion (inferred)
}

/**
 * @brief Disables the ADC.
 *
 * This function disables the ADC module.
 */
void ADC_Disable(void) {
    WDT_Reset();
    ADC_CR2 &= ~(1UL << 0); // Clear ADON bit (ADC ON)
    WDT_Reset();
    RCC_APB2ENR &= ~RCC_APB2ENR_ADC1EN; // Disable ADC1 clock (inferred)
}

/**
 * @brief Updates the ADC (placeholder).
 */
void ADC_Update(void) {
    WDT_Reset();
    // Specific update logic (e.g., re-starting conversion, checking status)
    // would be implemented here. For now, a placeholder.
}

/**
 * @brief Gets the ADC conversion result.
 * @return The 12-bit ADC conversion value.
 */
tword ADC_Get(void) {
    WDT_Reset();
    // Wait for EOC (End of Conversion) flag
    while (!(ADC_SR & (1UL << 1))) { // SR_EOC bit (inferred)
        WDT_Reset();
    }
    return (tword)(ADC_DR & 0xFFFFUL); // Read data from DR (12-bit value)
}

/**
 * @brief Clears ADC flags.
 */
void ADC_ClearFlag(void) {
    WDT_Reset();
    // Clear EOC (End of Conversion) flag by writing 0 (or reading DR based on reference manual)
    ADC_SR &= ~(1UL << 1); // Clear EOC flag (if write-to-clear, otherwise reading DR clears)
    ADC_SR &= ~(1UL << 5); // Clear OVR (Overrun) flag (inferred, often write-to-clear)
}

/**
 * @brief Initializes the Internal EEPROM.
 *
 * STM32F401RC does not have a dedicated internal EEPROM. EEPROM functionality
 * is typically emulated using the internal Flash memory. This function would
 * initialize the Flash memory interface for emulation.
 */
void Internal_EEPROM_Init(void) {
    WDT_Reset();
    // On STM32F401, there is no dedicated EEPROM. Flash memory is used for emulation.
    // This typically involves unlocking flash and configuring access.
    // Ensure Flash clock is enabled (it's usually always on for the core).
    // FLASH_ACR for latency, caches etc.
    FLASH_ACR = (0x5UL << 0); // Set latency to 5 wait states (for 84MHz)
    FLASH_ACR |= (1UL << 8);  // Enable ART Accelerator
    FLASH_ACR |= (1UL << 9);  // Enable prefetch buffer
    FLASH_ACR |= (1UL << 10); // Enable instruction cache
    FLASH_ACR |= (1UL << 11); // Enable data cache
    WDT_Reset();
}

/**
 * @brief Sets a byte in the emulated Internal EEPROM.
 * @param address The address within the emulated EEPROM to write to.
 * @param data The byte data to write.
 *
 * This function performs a Flash write operation. Requires careful handling of erase, unlock, and lock.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset();
    // This function requires Flash programming routines, which are typically more complex
    // than direct register writes and involve unlocking, erasing sectors, and programming.
    // The provided register JSON only includes Flash control and status registers, not
    // direct data/address registers for writing to a specific address within flash.
    // This implementation serves as a placeholder for a complex Flash write routine.

    // 1. Unlock the Flash Program/Erase Controller
    FLASH_KEYR = 0x45670123UL; // Key 1
    FLASH_KEYR = 0xCDEF89ABUL; // Key 2
    WDT_Reset();

    // 2. Check if Flash is busy
    while (FLASH_SR & (1UL << 16)) { /* Wait for BSY */ WDT_Reset(); } // SR_BSY bit 16 (inferred)

    // 3. Set PGM bit in FLASH_CR to enable programming
    FLASH_CR |= (1UL << 0); // CR_PG bit (Program)
    WDT_Reset();

    // 4. Perform the write operation. This is highly simplified.
    // In reality, this involves calculating the actual flash address,
    // ensuring the sector is erased, and then writing words or half-words.
    // For now, it's a dummy write to a hypothetical emulated EEPROM base address.
    volatile tbyte* emulated_eeprom_base = (volatile tbyte*)0x0800C000; // Example: start of a Flash sector
    *(emulated_eeprom_base + address) = data; // Dummy write
    WDT_Reset();

    // 5. Wait for programming to complete
    while (FLASH_SR & (1UL << 16)) { /* Wait for BSY */ WDT_Reset(); }

    // 6. Lock the Flash Program/Erase Controller
    FLASH_CR |= (1UL << 31); // CR_LOCK bit
    WDT_Reset();
}

/**
 * @brief Gets a byte from the emulated Internal EEPROM.
 * @param address The address within the emulated EEPROM to read from.
 * @return The byte data read.
 */
tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset();
    // Reading from Flash is a direct memory read operation after Flash is initialized.
    volatile tbyte* emulated_eeprom_base = (volatile tbyte*)0x0800C000; // Example: start of a Flash sector
    return *(emulated_eeprom_base + address);
}

// TT (Time-Triggered) OS APIs - These typically rely on a hardware timer for their ISR.
// Assuming a generic timer (e.g., SysTick or TIMx) provides the tick interrupt.

/**
 * @brief Initializes the Time-Triggered (TT) OS.
 * @param tick_time_ms The desired tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset();
    tlong tick_us = 0;
    if (tick_time_ms == TT_TICK_TIME_1MS) tick_us = 1000;
    else if (tick_time_ms == TT_TICK_TIME_10MS) tick_us = 10000;
    else if (tick_time_ms == TT_TICK_TIME_100MS) tick_us = 100000;

    // This function would typically configure a hardware timer (like SysTick)
    // to generate an interrupt at the specified tick rate.
    // For SysTick, reload value = (CPU_CLOCK_HZ / (1000000 / tick_us)) - 1
    // Assuming CPU clock is 84MHz for F401
    tlong cpu_clock = 84000000UL;
    if (tick_us > 0) {
        tlong reload_value = (cpu_clock / (1000000UL / tick_us)) - 1UL;
        SysTick->LOAD = reload_value; // Inferred SysTick registers from ARM CMSIS
        SysTick->VAL = 0UL;           // Clear current value
        SysTick->CTRL = (1UL << 2) |  // CLKSOURCE (Processor clock)
                        (1UL << 1);   // TICKINT (Enable SysTick interrupt)
                                      // Note: SysTick_CTRL_ENABLE (bit 0) is set in TT_Start
        WDT_Reset();
    }
}

/**
 * @brief Starts the Time-Triggered (TT) OS.
 */
void TT_Start(void) {
    WDT_Reset();
    SysTick->CTRL |= (1UL << 0); // Enable SysTick counter (CTRL_ENABLE bit)
}

/**
 * @brief Dispatches tasks in the Time-Triggered (TT) OS (placeholder).
 */
void TT_Dispatch_task(void) {
    WDT_Reset();
    // This function would typically iterate through a list of tasks,
    // checking their timing and executing them if due.
}

/**
 * @brief Time-Triggered (TT) OS Interrupt Service Routine handler (placeholder).
 *
 * This function is called by the underlying timer ISR (e.g., SysTick_Handler).
 */
void TT_ISR(void) {
    WDT_Reset();
    // This ISR would typically increment a tick counter and/or signal the dispatcher.
    // If a callback is set for ICU, and this is an ICU ISR, it might be called here.
    if (ICU_UserCallback != NULL) {
        ICU_UserCallback();
    }
}

// Global arrays for tasks storage (placeholder)
#define MAX_TT_TASKS 10 // Max number of tasks for TT
static void (*tt_tasks[MAX_TT_TASKS])(void);
static tword tt_periods[MAX_TT_TASKS];
static tword tt_delays[MAX_TT_TASKS];
static tbyte tt_task_count = 0;

/**
 * @brief Adds a task to the Time-Triggered (TT) OS scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks.
 * @param delay The initial delay of the task in ticks.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset();
    if (tt_task_count < MAX_TT_TASKS && task != NULL) {
        tt_tasks[tt_task_count] = task;
        tt_periods[tt_task_count] = period;
        tt_delays[tt_task_count] = delay;
        tt_task_count++;
        return (tbyte)(tt_task_count - 1);
    }
    return 0xFF; // Failed to add task
}

/**
 * @brief Deletes a task from the Time-Triggered (TT) OS scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset();
    if (task_index < tt_task_count) {
        // Shift remaining tasks to fill the gap
        for (tbyte i = task_index; i < (tt_task_count - 1); i++) {
            tt_tasks[i] = tt_tasks[i + 1];
            tt_periods[i] = tt_periods[i + 1];
            tt_delays[i] = tt_delays[i + 1];
        }
        tt_task_count--;
    }
}