
// Chunk 1
#ifndef MCAL_H
#define MCAL_H

// Core MCU header - deduced based on MCU NAME
#include "stm32f401xc.h" 

// Standard C library includes as per rules
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Data type definitions as per Rules.json
#define Unit_8 uint8_t
#define unit_16 uint16_t
#define unit_32 uint32_t

typedef Unit_8 tbyte;
typedef unit_16 tword;
typedef unit_32 tlong;

// Register Normalization and Definitions
// Note: Register names are kept as in register_json,
// but grouped conceptually for MCAL clarity.

// FLASH Registers
#define FLASH_ACR       (*(volatile uint32_t*)0x40023C00) // Flash access control register.
#define FLASH_KEYR      (*(volatile uint32_t*)0x40023C04) // Flash key register.
#define FLASH_OPTKEYR   (*(volatile uint32_t*)0x40023C08) // Flash option key register.
#define FLASH_SR        (*(volatile uint32_t*)0x40023C0C) // Flash status register.
#define FLASH_CR        (*(volatile uint32_t*)0x40023C10) // Flash control register.
#define FLASH_OPTCR     (*(volatile uint32_t*)0x40023C14) // Flash option control register.

// CRC Registers
#define CRC_DR          (*(volatile uint32_t*)0x40023000) // Data register for CRC calculation unit.
#define CRC_IDR         (*(volatile uint32_t*)0x40023004) // Independent data register for CRC calculation unit.
#define CRC_CR          (*(volatile uint32_t*)0x40023008) // Control register for CRC calculation unit.

// PWR Registers
#define PWR_CR          (*(volatile uint32_t*)0x40007000) // Power control register.
#define PWR_CSR         (*(volatile uint32_t*)0x40007004) // Power control/status register.

// RCC Registers
#define RCC_CR          (*(volatile uint32_t*)0x40023800) // Clock control register.
#define RCC_PLLCFGR     (*(volatile uint32_t*)0x40023804) // PLL configuration register.
#define RCC_CFGR        (*(volatile uint32_t*)0x40023808) // Clock configuration register.
#define RCC_CIR         (*(volatile uint32_t*)0x4002380C) // Clock interrupt register.
#define RCC_AHB1RSTR    (*(volatile uint32_t*)0x40023810) // AHB1 peripheral reset register.
#define RCC_AHB2RSTR    (*(volatile uint32_t*)0x40023814) // AHB2 peripheral reset register.
#define RCC_APB1RSTR    (*(volatile uint32_t*)0x40023818) // APB1 peripheral reset register.
#define RCC_APB2RSTR    (*(volatile uint32_t*)0x4002381C) // APB2 peripheral reset register.
#define RCC_AHB1ENR     (*(volatile uint32_t*)0x40023830) // AHB1 peripheral clock enable register.
#define RCC_AHB2ENR     (*(volatile uint32_t*)0x40023834) // AHB2 peripheral clock enable register.
#define RCC_APB1ENR     (*(volatile uint32_t*)0x40023838) // APB1 peripheral clock enable register.
#define RCC_APB2ENR     (*(volatile uint32_t*)0x4002383C) // APB2 peripheral clock enable register.
#define RCC_AHB1LPENR   (*(volatile uint32_t*)0x40023850) // AHB1 peripheral clock enable in low power mode register.
#define RCC_AHB2LPENR   (*(volatile uint32_t*)0x40023854) // AHB2 peripheral clock enable in low power mode register.
#define RCC_APB1LPENR   (*(volatile uint32_t*)0x40023858) // APB1 peripheral clock enable in low power mode register.
#define RCC_APB2LPENR   (*(volatile uint32_t*)0x4002385C) // APB2 peripheral clock enabled in low power mode register.
#define RCC_BDCR        (*(volatile uint32_t*)0x40023870) // Backup domain control register.
#define RCC_CSR         (*(volatile uint32_t*)0x40023874) // Clock control & status register.
#define RCC_SSCGR       (*(volatile uint32_t*)0x40023880) // Spread spectrum clock generation register.
#define RCC_PLLI2SCFGR  (*(volatile uint32_t*)0x40023884) // PLLI2S configuration register.
#define RCC_DCKCFGR     (*(volatile uint32_t*)0x4002388C) // Dedicated Clocks Configuration Register.

// SYSCFG Registers
#define SYSCFG_MEMRMP   (*(volatile uint32_t*)0x40013800) // Memory remap register.
#define SYSCFG_PMC      (*(volatile uint32_t*)0x40013804) // Peripheral mode configuration register.
#define SYSCFG_EXTICR1  (*(volatile uint32_t*)0x40013808) // External interrupt configuration register 1 (EXTI lines 0-3).
#define SYSCFG_EXTICR2  (*(volatile uint32_t*)0x4001380C) // External interrupt configuration register 2 (EXTI lines 4-7).
#define SYSCFG_EXTICR3  (*(volatile uint32_t*)0x40013810) // External interrupt configuration register 3 (EXTI lines 8-11).
#define SYSCFG_EXTICR4  (*(volatile uint32_t*)0x40013814) // External interrupt configuration register 4 (EXTI lines 12-15).
#define SYSCFG_CMPCR    (*(volatile uint32_t*)0x40013820) // Compensation cell control register.

// GPIO Registers (A, B, C, D, E, H)
#define GPIOA_MODER     (*(volatile uint32_t*)0x40020000) // GPIO port mode register for Port A.
#define GPIOA_OTYPER    (*(volatile uint32_t*)0x40020004) // GPIO port output type register for Port A.
#define GPIOA_OSPEEDR   (*(volatile uint32_t*)0x40020008) // GPIO port output speed register for Port A.
#define GPIOA_PUPDR     (*(volatile uint32_t*)0x4002000C) // GPIO port pull-up/pull-down register for Port A.
#define GPIOA_IDR       (*(volatile uint32_t*)0x40020010) // GPIO port input data register for Port A.
#define GPIOA_ODR       (*(volatile uint32_t*)0x40020014) // GPIO port output data register for Port A.
#define GPIOA_BSRR      (*(volatile uint32_t*)0x40020018) // GPIO port bit set/reset register for Port A.
#define GPIOA_LCKR      (*(volatile uint32_t*)0x4002001C) // GPIO port configuration lock register for Port A.
#define GPIOA_AFRL      (*(volatile uint32_t*)0x40020020) // GPIO alternate function low register for Port A (pins 0-7).
#define GPIOA_AFRH      (*(volatile uint32_t*)0x40020024) // GPIO alternate function high register for Port A (pins 8-15).

#define GPIOB_MODER     (*(volatile uint32_t*)0x40020400) // GPIO port mode register for Port B.
#define GPIOB_OTYPER    (*(volatile uint32_t*)0x40020404) // GPIO port output type register for Port B.
#define GPIOB_OSPEEDR   (*(volatile uint32_t*)0x40020408) // GPIO port output speed register for Port B.
#define GPIOB_PUPDR     (*(volatile uint32_t*)0x4002040C) // GPIO port pull-up/pull-down register for Port B.
#define GPIOB_IDR       (*(volatile uint32_t*)0x40020410) // GPIO port input data register for Port B.
#define GPIOB_ODR       (*(volatile uint32_t*)0x40020414) // GPIO port output data register for Port B.
#define GPIOB_BSRR      (*(volatile uint32_t*)0x40020418) // GPIO port bit set/reset register for Port B.
#define GPIOB_LCKR      (*(volatile uint32_t*)0x4002041C) // GPIO port configuration lock register for Port B.
#define GPIOB_AFRL      (*(volatile uint32_t*)0x40020420) // GPIO alternate function low register for Port B (pins 0-7).
#define GPIOB_AFRH      (*(volatile uint32_t*)0x40020424) // GPIO alternate function high register for Port B (pins 8-15).

#define GPIOC_MODER     (*(volatile uint32_t*)0x40020800) // GPIO port mode register for Port C.
#define GPIOC_OTYPER    (*(volatile uint32_t*)0x40020804) // GPIO port output type register for Port C.
#define GPIOC_OSPEEDR   (*(volatile uint32_t*)0x40020808) // GPIO port output speed register for Port C.
#define GPIOC_PUPDR     (*(volatile uint32_t*)0x4002080C) // GPIO port pull-up/pull-down register for Port C.
#define GPIOC_IDR       (*(volatile uint32_t*)0x40020810) // GPIO port input data register for Port C.
#define GPIOC_ODR       (*(volatile uint32_t*)0x40020814) // GPIO port output data register for Port C.
#define GPIOC_BSRR      (*(volatile uint32_t*)0x40020818) // GPIO port bit set/reset register for Port C.
#define GPIOC_LCKR      (*(volatile uint32_t*)0x4002081C) // GPIO port configuration lock register for Port C.
#define GPIOC_AFRL      (*(volatile uint32_t*)0x40020820) // GPIO alternate function low register for Port C (pins 0-7).
#define GPIOC_AFRH      (*(volatile uint32_t*)0x40020824) // GPIO alternate function high register for Port C (pins 8-15).

#define GPIOD_MODER     (*(volatile uint32_t*)0x40020C00) // GPIO port mode register for Port D.
#define GPIOD_OTYPER    (*(volatile uint32_t*)0x40020C04) // GPIO port output type register for Port D.
#define GPIOD_OSPEEDR   (*(volatile uint32_t*)0x40020C08) // GPIO port output speed register for Port D.
#define GPIOD_PUPDR     (*(volatile uint32_t*)0x40020C0C) // GPIO port pull-up/pull-down register for Port D.
#define GPIOD_IDR       (*(volatile uint32_t*)0x40020C10) // GPIO port input data register for Port D.
#define GPIOD_ODR       (*(volatile uint32_t*)0x40020C14) // GPIO port output data register for Port D.
#define GPIOD_BSRR      (*(volatile uint32_t*)0x40020C18) // GPIO port bit set/reset register for Port D.
#define GPIOD_LCKR      (*(volatile uint32_t*)0x40020C1C) // GPIO port configuration lock register for Port D.
#define GPIOD_AFRL      (*(volatile uint32_t*)0x40020C20) // GPIO alternate function low register for Port D (pins 0-7).
#define GPIOD_AFRH      (*(volatile uint32_t*)0x40020C24) // GPIO alternate function high register for Port D (pins 8-15).

#define GPIOE_MODER     (*(volatile uint32_t*)0x40021000) // GPIO port mode register for Port E.
#define GPIOE_OTYPER    (*(volatile uint32_t*)0x40021004) // GPIO port output type register for Port E.
#define GPIOE_OSPEEDR   (*(volatile uint32_t*)0x40021008) // GPIO port output speed register for Port E.
#define GPIOE_PUPDR     (*(volatile uint32_t*)0x4002100C) // GPIO port pull-up/pull-down register for Port E.
#define GPIOE_IDR       (*(volatile uint32_t*)0x40021010) // GPIO port input data register for Port E.
#define GPIOE_ODR       (*(volatile uint32_t*)0x40021014) // GPIO port output data register for Port E.
#define GPIOE_BSRR      (*(volatile uint32_t*)0x40021018) // GPIO port bit set/reset register for Port E.
#define GPIOE_LCKR      (*(volatile uint32_t*)0x4002101C) // GPIO port configuration lock register for Port E.
#define GPIOE_AFRL      (*(volatile uint32_t*)0x40021020) // GPIO alternate function low register for Port E (pins 0-7).
#define GPIOE_AFRH      (*(volatile uint32_t*)0x40021024) // GPIO alternate function high register for Port E (pins 8-15).

#define GPIOH_MODER     (*(volatile uint32_t*)0x40021C00) // GPIO port mode register for Port H.
#define GPIOH_OTYPER    (*(volatile uint32_t*)0x40021C04) // GPIO port output type register for Port H.
#define GPIOH_OSPEEDR   (*(volatile uint32_t*)0x40021C08) // GPIO port output speed register for Port H.
#define GPIOH_PUPDR     (*(volatile uint32_t*)0x40021C0C) // GPIO port pull-up/pull-down register for Port H.
#define GPIOH_IDR       (*(volatile uint32_t*)0x40021C10) // GPIO port input data register for Port H.
#define GPIOH_ODR       (*(volatile uint32_t*)0x40021C14) // GPIO port output data register for Port H.
#define GPIOH_BSRR      (*(volatile uint32_t*)0x40021C18) // GPIO port bit set/reset register for Port H.
#define GPIOH_LCKR      (*(volatile uint32_t*)0x40021C1C) // GPIO port configuration lock register for Port H.
#define GPIOH_AFRL      (*(volatile uint32_t*)0x40021C20) // GPIO alternate function low register for Port H (pins 0-7).
#define GPIOH_AFRH      (*(volatile uint32_t*)0x40021C24) // GPIO alternate function high register for Port H (pins 8-15).

// DMA Registers (DMA1, DMA2)
#define DMA1_LISR       (*(volatile uint32_t*)0x40026000) // DMA1 low interrupt status register.
#define DMA1_HISR       (*(volatile uint32_t*)0x40026004) // DMA1 high interrupt status register.
#define DMA1_LIFCR      (*(volatile uint32_t*)0x40026008) // DMA1 low interrupt flag clear register.
#define DMA1_HIFCR      (*(volatile uint32_t*)0x4002600C) // DMA1 high interrupt flag clear register.
#define DMA1_S0CR       (*(volatile uint32_t*)0x40026010) // DMA1 stream 0 configuration register.
#define DMA1_S0NDTR     (*(volatile uint32_t*)0x40026014) // DMA1 stream 0 number of data register.
#define DMA1_S0PAR      (*(volatile uint32_t*)0x40026018) // DMA1 stream 0 peripheral address register.
#define DMA1_S0M0AR     (*(volatile uint32_t*)0x4002601C) // DMA1 stream 0 memory 0 address register.
#define DMA1_S0M1AR     (*(volatile uint32_t*)0x40026020) // DMA1 stream 0 memory 1 address register.
#define DMA1_S0FCR      (*(volatile uint32_t*)0x40026024) // DMA1 stream 0 FIFO control register.
#define DMA1_S1CR       (*(volatile uint32_t*)0x40026028) // DMA1 stream 1 configuration register.
#define DMA1_S1NDTR     (*(volatile uint32_t*)0x4002602C) // DMA1 stream 1 number of data register.
#define DMA1_S1PAR      (*(volatile uint32_t*)0x40026030) // DMA1 stream 1 peripheral address register.
#define DMA1_S1M0AR     (*(volatile uint32_t*)0x40026034) // DMA1 stream 1 memory 0 address register.
#define DMA1_S1M1AR     (*(volatile uint32_t*)0x40026038) // DMA1 stream 1 memory 1 address register.
#define DMA1_S1FCR      (*(volatile uint32_t*)0x4002603C) // DMA1 stream 1 FIFO control register.
#define DMA1_S2CR       (*(volatile uint32_t*)0x40026040) // DMA1 stream 2 configuration register.
#define DMA1_S2NDTR     (*(volatile uint32_t*)0x40026044) // DMA1 stream 2 number of data register.
#define DMA1_S2PAR      (*(volatile uint32_t*)0x40026048) // DMA1 stream 2 peripheral address register.
#define DMA1_S2M0AR     (*(volatile uint32_t*)0x4002604C) // DMA1 stream 2 memory 0 address register.
#define DMA1_S2M1AR     (*(volatile uint32_t*)0x40026050) // DMA1 stream 2 memory 1 address register.
#define DMA1_S2FCR      (*(volatile uint32_t*)0x40026054) // DMA1 stream 2 FIFO control register.
#define DMA1_S3CR       (*(volatile uint32_t*)0x40026058) // DMA1 stream 3 configuration register.
#define DMA1_S3NDTR     (*(volatile uint32_t*)0x4002605C) // DMA1 stream 3 number of data register.
#define DMA1_S3PAR      (*(volatile uint32_t*)0x40026060) // DMA1 stream 3 peripheral address register.
#define DMA1_S3M0AR     (*(volatile uint32_t*)0x40026064) // DMA1 stream 3 memory 0 address register.
#define DMA1_S3M1AR     (*(volatile uint32_t*)0x40026068) // DMA1 stream 3 memory 1 address register.
#define DMA1_S3FCR      (*(volatile uint32_t*)0x4002606C) // DMA1 stream 3 FIFO control register.
#define DMA1_S4CR       (*(volatile uint32_t*)0x40026070) // DMA1 stream 4 configuration register.
#define DMA1_S4NDTR     (*(volatile uint32_t*)0x40026074) // DMA1 stream 4 number of data register.
#define DMA1_S4PAR      (*(volatile uint32_t*)0x40026078) // DMA1 stream 4 peripheral address register.
#define DMA1_S4M0AR     (*(volatile uint32_t*)0x4002607C) // DMA1 stream 4 memory 0 address register.
#define DMA1_S4M1AR     (*(volatile uint32_t*)0x40026080) // DMA1 stream 4 memory 1 address register.
#define DMA1_S4FCR      (*(volatile uint32_t*)0x40026084) // DMA1 stream 4 FIFO control register.
#define DMA1_S5CR       (*(volatile uint32_t*)0x40026088) // DMA1 stream 5 configuration register.
#define DMA1_S5NDTR     (*(volatile uint32_t*)0x4002608C) // DMA1 stream 5 number of data register.
#define DMA1_S5PAR      (*(volatile uint32_t*)0x40026090) // DMA1 stream 5 peripheral address register.
#define DMA1_S5M0AR     (*(volatile uint32_t*)0x40026094) // DMA1 stream 5 memory 0 address register.
#define DMA1_S5M1AR     (*(volatile uint32_t*)0x40026098) // DMA1 stream 5 memory 1 address register.
#define DMA1_S5FCR      (*(volatile uint32_t*)0x4002609C) // DMA1 stream 5 FIFO control register.
#define DMA1_S6CR       (*(volatile uint32_t*)0x400260A0) // DMA1 stream 6 configuration register.
#define DMA1_S6NDTR     (*(volatile uint32_t*)0x400260A4) // DMA1 stream 6 number of data register.
#define DMA1_S6PAR      (*(volatile uint32_t*)0x400260A8) // DMA1 stream 6 peripheral address register.
#define DMA1_S6M0AR     (*(volatile uint32_t*)0x400260AC) // DMA1 stream 6 memory 0 address register.
#define DMA1_S6M1AR     (*(volatile uint32_t*)0x400260B0) // DMA1 stream 6 memory 1 address register.
#define DMA1_S6FCR      (*(volatile uint32_t*)0x400260B4) // DMA1 stream 6 FIFO control register.
#define DMA1_S7CR       (*(volatile uint32_t*)0x400260B8) // DMA1 stream 7 configuration register.
#define DMA1_S7NDTR     (*(volatile uint32_t*)0x400260BC) // DMA1 stream 7 number of data register.
#define DMA1_S7PAR      (*(volatile uint32_t*)0x400260C0) // DMA1 stream 7 peripheral address register.
#define DMA1_S7M0AR     (*(volatile uint32_t*)0x400260C4) // DMA1 stream 7 memory 0 address register.
#define DMA1_S7M1AR     (*(volatile uint32_t*)0x400260C8) // DMA1 stream 7 memory 1 address register.
#define DMA1_S7FCR      (*(volatile uint32_t*)0x400260CC) // DMA1 stream 7 FIFO control register.

#define DMA2_LISR       (*(volatile uint32_t*)0x40026400) // DMA2 low interrupt status register.
#define DMA2_HISR       (*(volatile uint32_t*)0x40026404) // DMA2 high interrupt status register.
#define DMA2_LIFCR      (*(volatile uint32_t*)0x40026408) // DMA2 low interrupt flag clear register.
#define DMA2_HIFCR      (*(volatile uint32_t*)0x4002640C) // DMA2 high interrupt flag clear register.
#define DMA2_S0CR       (*(volatile uint32_t*)0x40026410) // DMA2 stream 0 configuration register.
#define DMA2_S0NDTR     (*(volatile uint32_t*)0x40026414) // DMA2 stream 0 number of data register.
#define DMA2_S0PAR      (*(volatile uint32_t*)0x40026418) // DMA2 stream 0 peripheral address register.
#define DMA2_S0M0AR     (*(volatile uint32_t*)0x4002641C) // DMA2 stream 0 memory 0 address register.
#define DMA2_S0M1AR     (*(volatile uint32_t*)0x40026420) // DMA2 stream 0 memory 1 address register.
#define DMA2_S0FCR      (*(volatile uint32_t*)0x40026424) // DMA2 stream 0 FIFO control register.
#define DMA2_S1CR       (*(volatile uint32_t*)0x40026428) // DMA2 stream 1 configuration register.
#define DMA2_S1NDTR     (*(volatile uint32_t*)0x4002642C) // DMA2 stream 1 number of data register.
#define DMA2_S1PAR      (*(volatile uint32_t*)0x40026430) // DMA2 stream 1 peripheral address register.
#define DMA2_S1M0AR     (*(volatile uint32_t*)0x40026434) // DMA2 stream 1 memory 0 address register.
#define DMA2_S1M1AR     (*(volatile uint32_t*)0x40026438) // DMA2 stream 1 memory 1 address register.
#define DMA2_S1FCR      (*(volatile uint32_t*)0x4002643C) // DMA2 stream 1 FIFO control register.
#define DMA2_S2CR       (*(volatile uint32_t*)0x40026440) // DMA2 stream 2 configuration register.
#define DMA2_S2NDTR     (*(volatile uint32_t*)0x40026444) // DMA2 stream 2 number of data register.
#define DMA2_S2PAR      (*(volatile uint32_t*)0x40026448) // DMA2 stream 2 peripheral address register.
#define DMA2_S2M0AR     (*(volatile uint32_t*)0x4002644C) // DMA2 stream 2 memory 0 address register.
#define DMA2_S2M1AR     (*(volatile uint32_t*)0x40026450) // DMA2 stream 2 memory 1 address register.
#define DMA2_S2FCR      (*(volatile uint32_t*)0x40026454) // DMA2 stream 2 FIFO control register.
#define DMA2_S3CR       (*(volatile uint32_t*)0x40026458) // DMA2 stream 3 configuration register.
#define DMA2_S3NDTR     (*(volatile uint32_t*)0x4002645C) // DMA2 stream 3 number of data register.
#define DMA2_S3PAR      (*(volatile uint32_t*)0x40026460) // DMA2 stream 3 peripheral address register.
#define DMA2_S3M0AR     (*(volatile uint32_t*)0x40026464) // DMA2 stream 3 memory 0 address register.
#define DMA2_S3M1AR     (*(volatile uint32_t*)0x40026468) // DMA2 stream 3 memory 1 address register.
#define DMA2_S3FCR      (*(volatile uint32_t*)0x4002646C) // DMA2 stream 3 FIFO control register.
#define DMA2_S4CR       (*(volatile uint32_t*)0x40026470) // DMA2 stream 4 configuration register.
#define DMA2_S4NDTR     (*(volatile uint32_t*)0x40026474) // DMA2 stream 4 number of data register.
#define DMA2_S4PAR      (*(volatile uint32_t*)0x40026478) // DMA2 stream 4 peripheral address register.
#define DMA2_S4M0AR     (*(volatile uint32_t*)0x4002647C) // DMA2 stream 4 memory 0 address register.
#define DMA2_S4M1AR     (*(volatile uint32_t*)0x40026480) // DMA2 stream 4 memory 1 address register.
#define DMA2_S4FCR      (*(volatile uint32_t*)0x40026484) // DMA2 stream 4 FIFO control register.
#define DMA2_S5CR       (*(volatile uint32_t*)0x40026488) // DMA2 stream 5 configuration register.
#define DMA2_S5NDTR     (*(volatile uint32_t*)0x4002648C) // DMA2 stream 5 number of data register.
#define DMA2_S5PAR      (*(volatile uint32_t*)0x40026490) // DMA2 stream 5 peripheral address register.
#define DMA2_S5M0AR     (*(volatile uint32_t*)0x40026494) // DMA2 stream 5 memory 0 address register.
#define DMA2_S5M1AR     (*(volatile uint32_t*)0x40026498) // DMA2 stream 5 memory 1 address register.
#define DMA2_S5FCR      (*(volatile uint32_t*)0x4002649C) // DMA2 stream 5 FIFO control register.
#define DMA2_S6CR       (*(volatile uint32_t*)0x400264A0) // DMA2 stream 6 configuration register.
#define DMA2_S6NDTR     (*(volatile uint32_t*)0x400264A4) // DMA2 stream 6 number of data register.
#define DMA2_S6PAR      (*(volatile uint32_t*)0x400264A8) // DMA2 stream 6 peripheral address register.
#define DMA2_S6M0AR     (*(volatile uint32_t*)0x400264AC) // DMA2 stream 6 memory 0 address register.
#define DMA2_S6M1AR     (*(volatile uint32_t*)0x400264B0) // DMA2 stream 6 memory 1 address register.
#define DMA2_S6FCR      (*(volatile uint32_t*)0x400264B4) // DMA2 stream 6 FIFO control register.
#define DMA2_S7CR       (*(volatile uint32_t*)0x400264B8) // DMA2 stream 7 configuration register.
#define DMA2_S7NDTR     (*(volatile uint32_t*)0x400264BC) // DMA2 stream 7 number of data register.
#define DMA2_S7PAR      (*(volatile uint32_t*)0x400264C0) // DMA2 stream 7 peripheral address register.
#define DMA2_S7M0AR     (*(volatile uint32_t*)0x400264C4) // DMA2 stream 7 memory 0 address register.
#define DMA2_S7M1AR     (*(volatile uint32_t*)0x400264C8) // DMA2 stream 7 memory 1 address register.
#define DMA2_S7FCR      (*(volatile uint32_t*)0x400264CC) // DMA2 stream 7 FIFO control register.

// EXTI Registers
#define EXTI_IMR        (*(volatile uint32_t*)0x40013C00) // Interrupt mask register for EXTI lines.
#define EXTI_EMR        (*(volatile uint32_t*)0x40013C04) // Event mask register for EXTI lines.
#define EXTI_RTSR       (*(volatile uint32_t*)0x40013C08) // Rising trigger selection register for EXTI lines.
#define EXTI_FTSR       (*(volatile uint32_t*)0x40013C0C) // Falling trigger selection register for EXTI lines.
#define EXTI_SWIER      (*(volatile uint32_t*)0x40013C10) // Software interrupt event register for EXTI lines.
#define EXTI_PR         (*(volatile uint32_t*)0x40013C14) // Pending register for EXTI lines.

// ADC Registers
#define ADC_SR          (*(volatile uint32_t*)0x40012000) // ADC status register.
#define ADC_CR1         (*(volatile uint32_t*)0x40012004) // ADC control register 1.
#define ADC_CR2         (*(volatile uint32_t*)0x40012008) // ADC control register 2.
#define ADC_SMPR1       (*(volatile uint32_t*)0x4001200C) // ADC sample time register 1 (channels 10-18).
#define ADC_SMPR2       (*(volatile uint32_t*)0x40012010) // ADC sample time register 2 (channels 0-9).
#define ADC_JOFR1       (*(volatile uint32_t*)0x40012014) // ADC injected channel data offset register 1.
#define ADC_JOFR2       (*(volatile uint32_t*)0x40012018) // ADC injected channel data offset register 2.
#define ADC_JOFR3       (*(volatile uint32_t*)0x4001201C) // ADC injected channel data offset register 3.
#define ADC_JOFR4       (*(volatile uint32_t*)0x40012020) // ADC injected channel data offset register 4.
#define ADC_HTR         (*(volatile uint32_t*)0x40012024) // ADC watchdog higher threshold register.
#define ADC_LTR         (*(volatile uint32_t*)0x40012028) // ADC watchdog lower threshold register.
#define ADC_SQR1        (*(volatile uint32_t*)0x4001202C) // ADC regular sequence register 1.
#define ADC_SQR2        (*(volatile uint32_t*)0x40012030) // ADC regular sequence register 2.
#define ADC_SQR3        (*(volatile uint32_t*)0x40012034) // ADC regular sequence register 3.
#define ADC_JSQR        (*(volatile uint32_t*)0x40012038) // ADC injected sequence register.
#define ADC_JDR1        (*(volatile uint32_t*)0x4001203C) // ADC injected data register 1.
#define ADC_JDR2        (*(volatile uint32_t*)0x40012040) // ADC injected data register 2.
#define ADC_JDR3        (*(volatile uint32_t*)0x40012044) // ADC injected data register 3.
#define ADC_JDR4        (*(volatile uint32_t*)0x40012048) // ADC injected data register 4.
#define ADC_DR          (*(volatile uint32_t*)0x4001204C) // ADC regular data register.
#define ADC_CCR         (*(volatile uint32_t*)0x40012300) // ADC common control register.

// TIM Registers (TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
#define TIM1_CR1        (*(volatile uint32_t*)0x40010000) // TIM1 control register 1.
#define TIM1_CR2        (*(volatile uint32_t*)0x40010004) // TIM1 control register 2.
#define TIM1_SMCR       (*(volatile uint32_t*)0x40010008) // TIM1 slave mode control register.
#define TIM1_DIER       (*(volatile uint32_t*)0x4001000C) // TIM1 DMA/interrupt enable register.
#define TIM1_SR         (*(volatile uint32_t*)0x40010010) // TIM1 status register.
#define TIM1_EGR        (*(volatile uint32_t*)0x40010014) // TIM1 event generation register.
#define TIM1_CCMR1      (*(volatile uint32_t*)0x40010018) // TIM1 capture/compare mode register 1 (channels 1 & 2).
#define TIM1_CCMR2      (*(volatile uint32_t*)0x4001001C) // TIM1 capture/compare mode register 2 (channels 3 & 4).
#define TIM1_CCER       (*(volatile uint32_t*)0x40010020) // TIM1 capture/compare enable register.
#define TIM1_CNT        (*(volatile uint32_t*)0x40010024) // TIM1 counter.
#define TIM1_PSC        (*(volatile uint32_t*)0x40010028) // TIM1 prescaler.
#define TIM1_ARR        (*(volatile uint32_t*)0x4001002C) // TIM1 auto-reload register.
#define TIM1_RCR        (*(volatile uint32_t*)0x40010030) // TIM1 repetition counter register.
#define TIM1_CCR1       (*(volatile uint32_t*)0x40010034) // TIM1 capture/compare register 1.
#define TIM1_CCR2       (*(volatile uint32_t*)0x40010038) // TIM1 capture/compare register 2.
#define TIM1_CCR3       (*(volatile uint32_t*)0x4001003C) // TIM1 capture/compare register 3.
#define TIM1_CCR4       (*(volatile uint32_t*)0x40010040) // TIM1 capture/compare register 4.
#define TIM1_BDTR       (*(volatile uint32_t*)0x40010044) // TIM1 break and dead-time register.
#define TIM1_DCR        (*(volatile uint32_t*)0x40010048) // TIM1 DMA control register.
#define TIM1_DMAR       (*(volatile uint32_t*)0x4001004C) // TIM1 DMA address for full transfer.

#define TIM2_CR1        (*(volatile uint32_t*)0x40000000) // TIM2 control register 1.
#define TIM2_CR2        (*(volatile uint32_t*)0x40000004) // TIM2 control register 2.
#define TIM2_SMCR       (*(volatile uint32_t*)0x40000008) // TIM2 slave mode control register.
#define TIM2_DIER       (*(volatile uint32_t*)0x4000000C) // TIM2 DMA/Interrupt enable register.
#define TIM2_SR         (*(volatile uint32_t*)0x40000010) // TIM2 status register.
#define TIM2_EGR        (*(volatile uint32_t*)0x40000014) // TIM2 event generation register.
#define TIM2_CCMR1      (*(volatile uint32_t*)0x40000018) // TIM2 capture/compare mode register 1 (channels 1 & 2).
#define TIM2_CCMR2      (*(volatile uint32_t*)0x4000001C) // TIM2 capture/compare mode register 2 (channels 3 & 4).
#define TIM2_CCER       (*(volatile uint32_t*)0x40000020) // TIM2 capture/compare enable register.
#define TIM2_CNT        (*(volatile uint32_t*)0x40000024) // TIM2 counter.
#define TIM2_PSC        (*(volatile uint32_t*)0x40000028) // TIM2 prescaler.
#define TIM2_ARR        (*(volatile uint32_t*)0x4000002C) // TIM2 auto-reload register.
#define TIM2_CCR1       (*(volatile uint32_t*)0x40000034) // TIM2 capture/compare register 1.
#define TIM2_CCR2       (*(volatile uint32_t*)0x40000038) // TIM2 capture/compare register 2.
#define TIM2_CCR3       (*(volatile uint32_t*)0x4000003C) // TIM2 capture/compare register 3.
#define TIM2_CCR4       (*(volatile uint32_t*)0x40000040) // TIM2 capture/compare register 4.
#define TIM2_DCR        (*(volatile uint32_t*)0x40000048) // TIM2 DMA control register.
#define TIM2_DMAR       (*(volatile uint32_t*)0x4000004C) // TIM2 DMA address for full transfer.
#define TIM2_OR         (*(volatile uint32_t*)0x40000050) // TIM2 option register.

#define TIM3_CR1        (*(volatile uint32_t*)0x40000400) // TIM3 control register 1.
#define TIM3_CR2        (*(volatile uint32_t*)0x40000404) // TIM3 control register 2.
#define TIM3_SMCR       (*(volatile uint32_t*)0x40000408) // TIM3 slave mode control register.
#define TIM3_DIER       (*(volatile uint32_t*)0x4000040C) // TIM3 DMA/Interrupt enable register.
#define TIM3_SR         (*(volatile uint32_t*)0x40000410) // TIM3 status register.
#define TIM3_EGR        (*(volatile uint32_t*)0x40000414) // TIM3 event generation register.
#define TIM3_CCMR1      (*(volatile uint32_t*)0x40000418) // TIM3 capture/compare mode register 1 (channels 1 & 2).
#define TIM3_CCMR2      (*(volatile uint32_t*)0x4000041C) // TIM3 capture/compare mode register 2 (channels 3 & 4).
#define TIM3_CCER       (*(volatile uint32_t*)0x40000420) // TIM3 capture/compare enable register.
#define TIM3_CNT        (*(volatile uint32_t*)0x40000424) // TIM3 counter.
#define TIM3_PSC        (*(volatile uint32_t*)0x40000428) // TIM3 prescaler.
#define TIM3_ARR        (*(volatile uint32_t*)0x4000042C) // TIM3 auto-reload register.
#define TIM3_CCR1       (*(volatile uint32_t*)0x40000434) // TIM3 capture/compare register 1.
#define TIM3_CCR2       (*(volatile uint32_t*)0x40000438) // TIM3 capture/compare register 2.
#define TIM3_CCR3       (*(volatile uint32_t*)0x4000043C) // TIM3 capture/compare register 3.
#define TIM3_CCR4       (*(volatile uint32_t*)0x40000440) // TIM3 capture/compare register 4.
#define TIM3_DCR        (*(volatile uint32_t*)0x40000448) // TIM3 DMA control register.
#define TIM3_DMAR       (*(volatile uint32_t*)0x4000044C) // TIM3 DMA address for full transfer.

#define TIM4_CR1        (*(volatile uint32_t*)0x40000800) // TIM4 control register 1.
#define TIM4_CR2        (*(volatile uint32_t*)0x40000804) // TIM4 control register 2.
#define TIM4_SMCR       (*(volatile uint32_t*)0x40000808) // TIM4 slave mode control register.
#define TIM4_DIER       (*(volatile uint32_t*)0x4000080C) // TIM4 DMA/Interrupt enable register.
#define TIM4_SR         (*(volatile uint32_t*)0x40000810) // TIM4 status register.
#define TIM4_EGR        (*(volatile uint32_t*)0x40000814) // TIM4 event generation register.
#define TIM4_CCMR1      (*(volatile uint32_t*)0x40000818) // TIM4 capture/compare mode register 1 (channels 1 & 2).
#define TIM4_CCMR2      (*(volatile uint32_t*)0x4000081C) // TIM4 capture/compare mode register 2 (channels 3 & 4).
#define TIM4_CCER       (*(volatile uint32_t*)0x40000820) // TIM4 capture/compare enable register.
#define TIM4_CNT        (*(volatile uint32_t*)0x40000824) // TIM4 counter.
#define TIM4_PSC        (*(volatile uint32_t*)0x40000828) // TIM4 prescaler.
#define TIM4_ARR        (*(volatile uint32_t*)0x4000082C) // TIM4 auto-reload register.
#define TIM4_CCR1       (*(volatile uint32_t*)0x40000834) // TIM4 capture/compare register 1.
#define TIM4_CCR2       (*(volatile uint32_t*)0x40000838) // TIM4 capture/compare register 2.
#define TIM4_CCR3       (*(volatile uint32_t*)0x4000083C) // TIM4 capture/compare register 3.
#define TIM4_CCR4       (*(volatile uint32_t*)0x40000840) // TIM4 capture/compare register 4.
#define TIM4_DCR        (*(volatile uint32_t*)0x40000848) // TIM4 DMA control register.
#define TIM4_DMAR       (*(volatile uint32_t*)0x4000084C) // TIM4 DMA address for full transfer.

#define TIM5_CR1        (*(volatile uint32_t*)0x40000C00) // TIM5 control register 1.
#define TIM5_CR2        (*(volatile uint32_t*)0x40000C04) // TIM5 control register 2.
#define TIM5_SMCR       (*(volatile uint32_t*)0x40000C08) // TIM5 slave mode control register.
#define TIM5_DIER       (*(volatile uint32_t*)0x40000C0C) // TIM5 DMA/Interrupt enable register.
#define TIM5_SR         (*(volatile uint32_t*)0x40000C10) // TIM5 status register.
#define TIM5_EGR        (*(volatile uint32_t*)0x40000C14) // TIM5 event generation register.
#define TIM5_CCMR1      (*(volatile uint32_t*)0x40000C18) // TIM5 capture/compare mode register 1 (channels 1 & 2).
#define TIM5_CCMR2      (*(volatile uint32_t*)0x40000C1C) // TIM5 capture/compare mode register 2 (channels 3 & 4).
#define TIM5_CCER       (*(volatile uint32_t*)0x40000C20) // TIM5 capture/compare enable register.
#define TIM5_CNT        (*(volatile uint32_t*)0x40000C24) // TIM5 counter.
#define TIM5_PSC        (*(volatile uint32_t*)0x40000C28) // TIM5 prescaler.
#define TIM5_ARR        (*(volatile uint32_t*)0x40000C2C) // TIM5 auto-reload register.
#define TIM5_CCR1       (*(volatile uint32_t*)0x40000C34) // TIM5 capture/compare register 1.
#define TIM5_CCR2       (*(volatile uint32_t*)0x40000C38) // TIM5 capture/compare register 2.
#define TIM5_CCR3       (*(volatile uint32_t*)0x40000C3C) // TIM5 capture/compare register 3.
#define TIM5_CCR4       (*(volatile uint32_t*)0x40000C40) // TIM5 capture/compare register 4.
#define TIM5_DCR        (*(volatile uint32_t*)0x40000C48) // TIM5 DMA control register.
#define TIM5_DMAR       (*(volatile uint32_t*)0x40000C4C) // TIM5 DMA address for full transfer.
#define TIM5_OR         (*(volatile uint32_t*)0x40000C50) // TIM5 option register.

#define TIM9_CR1        (*(volatile uint32_t*)0x40014000) // TIM9 control register 1.
#define TIM9_SMCR       (*(volatile uint32_t*)0x40014008) // TIM9 slave mode control register.
#define TIM9_DIER       (*(volatile uint32_t*)0x4001400C) // TIM9 Interrupt enable register.
#define TIM9_SR         (*(volatile uint32_t*)0x40014010) // TIM9 status register.
#define TIM9_EGR        (*(volatile uint32_t*)0x40014014) // TIM9 event generation register.
#define TIM9_CCMR1      (*(volatile uint32_t*)0x40014018) // TIM9 capture/compare mode register 1 (channels 1 & 2).
#define TIM9_CCER       (*(volatile uint32_t*)0x40014020) // TIM9 capture/compare enable register.
#define TIM9_CNT        (*(volatile uint32_t*)0x40014024) // TIM9 counter.
#define TIM9_PSC        (*(volatile uint32_t*)0x40014028) // TIM9 prescaler.
#define TIM9_ARR        (*(volatile uint32_t*)0x4001402C) // TIM9 auto-reload register.
#define TIM9_CCR1       (*(volatile uint32_t*)0x40014034) // TIM9 capture/compare register 1.
#define TIM9_CCR2       (*(volatile uint32_t*)0x40014038) // TIM9 capture/compare register 2.

#define TIM10_CR1       (*(volatile uint32_t*)0x40014400) // TIM10 control register 1.
#define TIM10_DIER      (*(volatile uint32_t*)0x4001440C) // TIM10 Interrupt enable register.
#define TIM10_SR        (*(volatile uint32_t*)0x40014410) // TIM10 status register.
#define TIM10_EGR       (*(volatile uint32_t*)0x40014414) // TIM10 event generation register.
#define TIM10_CCMR1     (*(volatile uint32_t*)0x40014418) // TIM10 capture/compare mode register 1 (channel 1).
#define TIM10_CCER      (*(volatile uint32_t*)0x40014420) // TIM10 capture/compare enable register.
#define TIM10_CNT       (*(volatile uint32_t*)0x40014424) // TIM10 counter.
#define TIM10_PSC       (*(volatile uint32_t*)0x40014428) // TIM10 prescaler.
#define TIM10_ARR       (*(volatile uint32_t*)0x4001442C) // TIM10 auto-reload register.
#define TIM10_CCR1      (*(volatile uint32_t*)0x40014434) // TIM10 capture/compare register 1.

#define TIM11_CR1       (*(volatile uint32_t*)0x40014800) // TIM11 control register 1.
#define TIM11_DIER      (*(volatile uint32_t*)0x4001480C) // TIM11 Interrupt enable register.
#define TIM11_SR        (*(volatile uint32_t*)0x40014810) // TIM11 status register.
#define TIM11_EGR       (*(volatile uint32_t*)0x40014814) // TIM11 event generation register.
#define TIM11_CCMR1     (*(volatile uint32_t*)0x40014818) // TIM11 capture/compare mode register 1 (channel 1).
#define TIM11_CCER      (*(volatile uint32_t*)0x40014820) // TIM11 capture/compare enable register.
#define TIM11_CNT       (*(volatile uint32_t*)0x40014824) // TIM11 counter.
#define TIM11_PSC       (*(volatile uint32_t*)0x40014828) // TIM11 prescaler.
#define TIM11_ARR       (*(volatile uint32_t*)0x4001482C) // TIM11 auto-reload register.
#define TIM11_CCR1      (*(volatile uint32_t*)0x40014834) // TIM11 capture/compare register 1.

// USART Registers (USART1, USART2, USART6)
#define USART1_SR       (*(volatile uint32_t*)0x40011000) // USART1 status register.
#define USART1_DR       (*(volatile uint32_t*)0x40011004) // USART1 data register (transmit/receive data).
#define USART1_BRR      (*(volatile uint32_t*)0x40011008) // USART1 baud rate register.
#define USART1_CR1      (*(volatile uint32_t*)0x4001100C) // USART1 control register 1.
#define USART1_CR2      (*(volatile uint32_t*)0x40011010) // USART1 control register 2.
#define USART1_CR3      (*(volatile uint32_t*)0x40011014) // USART1 control register 3.
#define USART1_GTPR     (*(volatile uint32_t*)0x40011018) // USART1 guard time and prescaler register.

#define USART2_SR       (*(volatile uint32_t*)0x40004400) // USART2 status register.
#define USART2_DR       (*(volatile uint32_t*)0x40004404) // USART2 data register (transmit/receive data).
#define USART2_BRR      (*(volatile uint32_t*)0x40004408) // USART2 baud rate register.
#define USART2_CR1      (*(volatile uint32_t*)0x4000440C) // USART2 control register 1.
#define USART2_CR2      (*(volatile uint32_t*)0x40004410) // USART2 control register 2.
#define USART2_CR3      (*(volatile uint32_t*)0x40004414) // USART2 control register 3.
#define USART2_GTPR     (*(volatile uint32_t*)0x40004418) // USART2 guard time and prescaler register.

#define USART6_SR       (*(volatile uint32_t*)0x40011400) // USART6 status register.
#define USART6_DR       (*(volatile uint32_t*)0x40011404) // USART6 data register (transmit/receive data).
#define USART6_BRR      (*(volatile uint32_t*)0x40011408) // USART6 baud rate register.
#define USART6_CR1      (*(volatile uint32_t*)0x4001140C) // USART6 control register 1.
#define USART6_CR2      (*(volatile uint32_t*)0x40011410) // USART6 control register 2.
#define USART6_CR3      (*(volatile uint32_t*)0x40011414) // USART6 control register 3.
#define USART6_GTPR     (*(volatile uint32_t*)0x40011418) // USART6 guard time and prescaler register.

// I2C Registers (I2C1, I2C2, I2C3)
#define I2C1_CR1        (*(volatile uint32_t*)0x40005400) // I2C1 Control register 1.
#define I2C1_CR2        (*(volatile uint32_t*)0x40005404) // I2C1 Control register 2.
#define I2C1_OAR1       (*(volatile uint32_t*)0x40005408) // I2C1 Own address register 1.
#define I2C1_OAR2       (*(volatile uint32_t*)0x4000540C) // I2C1 Own address register 2.
#define I2C1_DR         (*(volatile uint32_t*)0x40005410) // I2C1 Data register.
#define I2C1_SR1        (*(volatile uint32_t*)0x40005414) // I2C1 Status register 1.
#define I2C1_SR2        (*(volatile uint32_t*)0x40005418) // I2C1 Status register 2.
#define I2C1_CCR        (*(volatile uint32_t*)0x4000541C) // I2C1 Clock control register.
#define I2C1_TRISE      (*(volatile uint32_t*)0x40005420) // I2C1 TRISE register.
#define I2C1_FLTR       (*(volatile uint32_t*)0x40005424) // I2C1 FLTR register.

#define I2C2_CR1        (*(volatile uint32_t*)0x40005800) // I2C2 Control register 1.
#define I2C2_CR2        (*(volatile uint32_t*)0x40005804) // I2C2 Control register 2.
#define I2C2_OAR1       (*(volatile uint32_t*)0x40005808) // I2C2 Own address register 1.
#define I2C2_OAR2       (*(volatile uint32_t*)0x4000580C) // I2C2 Own address register 2.
#define I2C2_DR         (*(volatile uint32_t*)0x40005810) // I2C2 Data register.
#define I2C2_SR1        (*(volatile uint32_t*)0x40005814) // I2C2 Status register 1.
#define I2C2_SR2        (*(volatile uint32_t*)0x40005818) // I2C2 Status register 2.
#define I2C2_CCR        (*(volatile uint32_t*)0x4000581C) // I2C2 Clock control register.
#define I2C2_TRISE      (*(volatile uint32_t*)0x40005820) // I2C2 TRISE register.
#define I2C2_FLTR       (*(volatile uint32_t*)0x40005824) // I2C2 FLTR register.

#define I2C3_CR1        (*(volatile uint32_t*)0x40005C00) // I2C3 Control register 1.
#define I2C3_CR2        (*(volatile uint32_t*)0x40005C04) // I2C3 Control register 2.
#define I2C3_OAR1       (*(volatile uint32_t*)0x40005C08) // I2C3 Own address register 1.
#define I2C3_OAR2       (*(volatile uint32_t*)0x40005C0C) // I2C3 Own address register 2.
#define I2C3_DR         (*(volatile uint32_t*)0x40005C10) // I2C3 Data register.
#define I2C3_SR1        (*(volatile uint32_t*)0x40005C14) // I2C3 Status register 1.
#define I2C3_SR2        (*(volatile uint32_t*)0x40005C18) // I2C3 Status register 2.
#define I2C3_CCR        (*(volatile uint32_t*)0x40005C1C) // I2C3 Clock control register.
#define I2C3_TRISE      (*(volatile uint32_t*)0x40005C20) // I2C3 TRISE register.
#define I2C3_FLTR       (*(volatile uint32_t*)0x40005C24) // I2C3 FLTR register.

// SPI Registers (SPI1, SPI2, SPI3)
#define SPI1_CR1        (*(volatile uint32_t*)0x40013000) // SPI1 Control register 1.
#define SPI1_CR2        (*(volatile uint32_t*)0x40013004) // SPI1 Control register 2.
#define SPI1_SR         (*(volatile uint32_t*)0x40013008) // SPI1 Status register.
#define SPI1_DR         (*(volatile uint32_t*)0x4001300C) // SPI1 Data register.
#define SPI1_CRCPR      (*(volatile uint32_t*)0x40013010) // SPI1 CRC polynomial register.
#define SPI1_RXCRCR     (*(volatile uint32_t*)0x40013014) // SPI1 RX CRC register.
#define SPI1_TXCRCR     (*(volatile uint32_t*)0x40013018) // SPI1 TX CRC register.
#define SPI1_I2SCFGR    (*(volatile uint32_t*)0x4001301C) // SPI1 I2S configuration register.
#define SPI1_I2SPR      (*(volatile uint32_t*)0x40013020) // SPI1 I2S prescaler register.

#define SPI2_CR1        (*(volatile uint32_t*)0x40003800) // SPI2 Control register 1.
#define SPI2_CR2        (*(volatile uint32_t*)0x40003804) // SPI2 Control register 2.
#define SPI2_SR         (*(volatile uint32_t*)0x40003808) // SPI2 Status register.
#define SPI2_DR         (*(volatile uint32_t*)0x4000380C) // SPI2 Data register.
#define SPI2_CRCPR      (*(volatile uint32_t*)0x40003810) // SPI2 CRC polynomial register.
#define SPI2_RXCRCR     (*(volatile uint32_t*)0x40003814) // SPI2 RX CRC register.
#define SPI2_TXCRCR     (*(volatile uint32_t*)0x40003818) // SPI2 TX CRC register.
#define SPI2_I2SCFGR    (*(volatile uint32_t*)0x4000381C) // SPI2 I2S configuration register.
#define SPI2_I2SPR      (*(volatile uint32_t*)0x40003820) // SPI2 I2S prescaler register.

#define SPI3_CR1        (*(volatile uint32_t*)0x40003C00) // SPI3 Control register 1.
#define SPI3_CR2        (*(volatile uint32_t*)0x40003C04) // SPI3 Control register 2.
#define SPI3_SR         (*(volatile uint32_t*)0x40003C08) // SPI3 Status register.
#define SPI3_DR         (*(volatile uint32_t*)0x40003C0C) // SPI3 Data register.
#define SPI3_CRCPR      (*(volatile uint32_t*)0x40003C10) // SPI3 CRC polynomial register.
#define SPI3_RXCRCR     (*(volatile uint32_t*)0x40003C14) // SPI3 RX CRC register.
#define SPI3_TXCRCR     (*(volatile uint32_t*)0x40003C18) // SPI3 TX CRC register.
#define SPI3_I2SCFGR    (*(volatile uint32_t*)0x40003C1C) // SPI3 I2S configuration register.
#define SPI3_I2SPR      (*(volatile uint32_t*)0x40003C20) // SPI3 I2S prescaler register.


// API Typedefs and Enums
typedef enum {
    VOLT_3V = 0,
    VOLT_5V
} t_sys_volt;

typedef enum {
    LVD_THRESHOLD_0_5V = 0, // Placeholder values, specific register bits not provided
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1_5V,
    LVD_THRESHOLD_2V,
    LVD_THRESHOLD_2_5V, // Deduced intermediate values
    LVD_THRESHOLD_3V,
    LVD_THRESHOLD_3_5V,
    LVD_THRESHOLD_4V,
    LVD_THRESHOLD_4_5V,
    LVD_THRESHOLD_5V
} t_lvd_thrthresholdLevel;

typedef enum {
    LVD_CHANNEL_NONE = 0 // No specific LVD channels in register_json
} t_lvd_channel;

typedef enum {
    UART_CHANNEL_1 = 0,
    UART_CHANNEL_2,
    UART_CHANNEL_6
} t_uart_channel;

typedef enum {
    UART_BAUD_9600 = 0,
    UART_BAUD_19200,
    UART_BAUD_115200
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8BIT = 0,
    UART_DATA_9BIT
} t_uart_data_length;

typedef enum {
    UART_STOP_1BIT = 0,
    UART_STOP_2BIT
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

typedef enum {
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum {
    I2C_SPEED_STANDARD = 0, // 100 kHz
    I2C_SPEED_FAST          // 400 kHz (as per I2C rules)
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // Use tbyte for flexibility

typedef enum {
    I2C_ACK_DISABLE = 0,
    I2C_ACK_ENABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_8BIT = 0,
    I2C_DATA_16BIT // Though I2C is byte-oriented, some devices might combine
} t_i2c_datalength;

typedef enum {
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

typedef enum {
    SPI_MODE_SLAVE = 0,
    SPI_MODE_MASTER
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW = 0,
    SPI_CPOL_HIGH
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1EDGE = 0, // First clock transition is first data capture edge
    SPI_CPHA_2EDGE      // Second clock transition is first data capture edge
} t_spi_cpha;

typedef enum {
    SPI_DFF_8BIT = 0,
    SPI_DFF_16BIT
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
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
    EXTI_EDGE_RISING = 0,
    EXTI_EDGE_FALLING,
    EXTI_EDGE_RISING_FALLING
} t_external_int_edge;

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
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;

typedef enum {
    PWM_TIM1_CH1 = 0, // PA8, PE9
    PWM_TIM1_CH2,     // PA9, PE11
    PWM_TIM1_CH3,     // PA10, PE13
    PWM_TIM1_CH4,     // PA11, PE14
    PWM_TIM2_CH1,     // PA0, PA5, PA15
    PWM_TIM2_CH2,     // PA1, PB3
    PWM_TIM2_CH3,     // PA2, PB10
    PWM_TIM2_CH4,     // PA3, PB11
    PWM_TIM3_CH1,     // PA6, PB4, PC6
    PWM_TIM3_CH2,     // PA7, PB5, PC7
    PWM_TIM3_CH3,     // PB0, PC8
    PWM_TIM3_CH4,     // PB1, PC9
    PWM_TIM4_CH1,     // PB6, PD12
    PWM_TIM4_CH2,     // PB7, PD13
    PWM_TIM4_CH3,     // PB8, PD14
    PWM_TIM4_CH4,     // PB9, PD15
    PWM_TIM5_CH1,     // PA0
    PWM_TIM5_CH2,     // PA1
    PWM_TIM5_CH3,     // PA2
    PWM_TIM5_CH4,     // PA3
    PWM_TIM9_CH1,     // PA2, PE5
    PWM_TIM9_CH2,     // PA3, PE6
    PWM_TIM10_CH1,    // PA6, PB8
    PWM_TIM11_CH1     // PA7, PB9
} t_pwm_channel;

typedef t_pwm_channel t_icu_channel; // ICU uses timer channels

typedef enum {
    ICU_PRESCALER_1 = 0,
    ICU_PRESCALER_2,
    ICU_PRESCALER_4,
    ICU_PRESCALER_8,
    ICU_PRESCALER_16,
    ICU_PRESCALER_32,
    ICU_PRESCALER_64,
    ICU_PRESCALER_128,
    ICU_PRESCALER_256,
    ICU_PRESCALER_512,
    ICU_PRESCALER_1024 // Placeholder values, specific register bits not provided
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

typedef enum {
    TIMER_CHANNEL_1 = 0, // TIM1
    TIMER_CHANNEL_2,     // TIM2
    TIMER_CHANNEL_3,     // TIM3
    TIMER_CHANNEL_4,     // TIM4
    TIMER_CHANNEL_5,     // TIM5
    TIMER_CHANNEL_9,     // TIM9
    TIMER_CHANNEL_10,    // TIM10
    TIMER_CHANNEL_11     // TIM11
} t_timer_channel;

typedef enum {
    ADC_CHANNEL_0 = 0, // PA0
    ADC_CHANNEL_1,     // PA1
    ADC_CHANNEL_2,     // PA2
    ADC_CHANNEL_3,     // PA3
    ADC_CHANNEL_4,     // PA4
    ADC_CHANNEL_5,     // PA5
    ADC_CHANNEL_6,     // PA6
    ADC_CHANNEL_7,     // PA7
    ADC_CHANNEL_8,     // PB0
    ADC_CHANNEL_9,     // PB1
    ADC_CHANNEL_10,    // PC0
    ADC_CHANNEL_11,    // PC1
    ADC_CHANNEL_12,    // PC2
    ADC_CHANNEL_13,    // PC3
    ADC_CHANNEL_14,    // PC4
    ADC_CHANNEL_15,    // PC5
    // Note: Channels 16, 17, 18 typically internal (Temp sensor, Vrefint, VBAT)
    // Not explicitly mapped to pins in register_json, so limiting to 0-15 based on assigned_pin.
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE = 0,
    ADC_MODE_CONTINUOUS
} t_adc_mode_t;

typedef enum {
    TT_TICK_1MS = 1,
    TT_TICK_10MS = 10,
    TT_TICK_100MS = 100 // Example tick times in milliseconds
} t_tick_time;

// API Function Prototypes (from API.json)

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
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
tword ICU_GetFrequency(t_icu_channel icu_channel);
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

// Internal_EEPROM
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif // MCAL_H
