/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the public API definitions and type declarations for
 * the MCAL layer, providing a hardware-agnostic interface to the
 * STM32F401RC microcontroller's peripherals.
 *
 * @note This file is generated based on provided register and API definitions.
 *       Some register bit-level operations are commented out if specific bit
 *       definitions were not provided in the input, as per instructions.
 */

#ifndef MCAL_H
#define MCAL_H

// Core device header - Placeholder as per rule
#include "stm32f401xc.h"  // Core device header for STM32F401RC
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

// =============================================================================
// Register Addresses - Defined based on REGISTER JSON
// =============================================================================

// FLASH Registers
#define FLASH_ACR_ADDR          ((volatile uint32_t *)0x40023C00)
#define FLASH_KEYR_ADDR         ((volatile uint32_t *)0x40023C04)
#define FLASH_OPTKEYR_ADDR      ((volatile uint32_t *)0x40023C08)
#define FLASH_SR_ADDR           ((volatile uint32_t *)0x40023C0C)
#define FLASH_CR_ADDR           ((volatile uint32_t *)0x40023C10)
#define FLASH_OPTCR_ADDR        ((volatile uint32_t *)0x40023C14)

// CRC Registers
#define CRC_DR_ADDR             ((volatile uint32_t *)0x40023000)
#define CRC_IDR_ADDR            ((volatile uint32_t *)0x40023004)
#define CRC_CR_ADDR             ((volatile uint32_t *)0x40023008)

// PWR Registers
#define PWR_CR_ADDR             ((volatile uint32_t *)0x40007000)
#define PWR_CSR_ADDR            ((volatile uint32_t *)0x40007004)

// RCC Registers
#define RCC_CR_ADDR             ((volatile uint32_t *)0x40023800)
#define RCC_PLLCFGR_ADDR        ((volatile uint32_t *)0x40023804)
#define RCC_CFGR_ADDR           ((volatile uint32_t *)0x40023808)
#define RCC_CIR_ADDR            ((volatile uint32_t *)0x4002380C)
#define RCC_AHB1RSTR_ADDR       ((volatile uint32_t *)0x40023810)
#define RCC_AHB2RSTR_ADDR       ((volatile uint32_t *)0x40023814)
#define RCC_APB1RSTR_ADDR       ((volatile uint32_t *)0x40023818)
#define RCC_APB2RSTR_ADDR       ((volatile uint32_t *)0x4002381C)
#define RCC_AHB1ENR_ADDR        ((volatile uint32_t *)0x40023830)
#define RCC_AHB2ENR_ADDR        ((volatile uint32_t *)0x40023834)
#define RCC_APB1ENR_ADDR        ((volatile uint32_t *)0x40023838)
#define RCC_APB2ENR_ADDR        ((volatile uint32_t *)0x4002383C)
#define RCC_AHB1LPENR_ADDR      ((volatile uint32_t *)0x40023850)
#define RCC_AHB2LPENR_ADDR      ((volatile uint32_t *)0x40023854)
#define RCC_APB1LPENR_ADDR      ((volatile uint32_t *)0x40023858)
#define RCC_APB2LPENR_ADDR      ((volatile uint32_t *)0x4002385C)
#define RCC_BDCR_ADDR           ((volatile uint32_t *)0x40023870)
#define RCC_CSR_ADDR            ((volatile uint32_t *)0x40023874)
#define RCC_SSCGR_ADDR          ((volatile uint32_t *)0x40023880)
#define RCC_PLLI2SCFGR_ADDR     ((volatile uint32_t *)0x40023884)
#define RCC_DCKCFGR_ADDR        ((volatile uint32_t *)0x4002388C)

// SYSCFG Registers
#define SYSCFG_MEMRMP_ADDR      ((volatile uint32_t *)0x40013800)
#define SYSCFG_PMC_ADDR         ((volatile uint32_t *)0x40013804)
#define SYSCFG_EXTICR1_ADDR     ((volatile uint32_t *)0x40013808)
#define SYSCFG_EXTICR2_ADDR     ((volatile uint32_t *)0x4001380C)
#define SYSCFG_EXTICR3_ADDR     ((volatile uint32_t *)0x40013810)
#define SYSCFG_EXTICR4_ADDR     ((volatile uint32_t *)0x40013814)
#define SYSCFG_CMPCR_ADDR       ((volatile uint32_t *)0x40013820)

// GPIO Registers (Base Addresses)
#define GPIOA_BASE_ADDR         ((volatile uint32_t *)0x40020000)
#define GPIOB_BASE_ADDR         ((volatile uint32_t *)0x40020400)
#define GPIOC_BASE_ADDR         ((volatile uint32_t *)0x40020800)
#define GPIOD_BASE_ADDR         ((volatile uint32_t *)0x40020C00)
#define GPIOE_BASE_ADDR         ((volatile uint32_t *)0x40021000)
#define GPIOH_BASE_ADDR         ((volatile uint32_t *)0x40021C00)

// GPIO Register Offsets
#define GPIO_MODER_OFFSET       (0x00)
#define GPIO_OTYPER_OFFSET      (0x04)
#define GPIO_OSPEEDR_OFFSET     (0x08)
#define GPIO_PUPDR_OFFSET       (0x0C)
#define GPIO_IDR_OFFSET         (0x10)
#define GPIO_ODR_OFFSET         (0x14)
#define GPIO_BSRR_OFFSET        (0x18)
#define GPIO_LCKR_OFFSET        (0x1C)
#define GPIO_AFRL_OFFSET        (0x20)
#define GPIO_AFRH_OFFSET        (0x24)

// DMA Registers (Base Addresses for DMA1 & DMA2)
#define DMA1_BASE_ADDR          ((volatile uint32_t *)0x40026000)
#define DMA2_BASE_ADDR          ((volatile uint32_t *)0x40026400)

// DMA Register Offsets (relative to DMAx_BASE_ADDR)
#define DMA_LISR_OFFSET         (0x00)
#define DMA_HISR_OFFSET         (0x04)
#define DMA_LIFCR_OFFSET        (0x08)
#define DMA_HIFCR_OFFSET        (0x0C)
#define DMA_S0CR_OFFSET         (0x10)
#define DMA_S0NDTR_OFFSET       (0x14)
#define DMA_S0PAR_OFFSET        (0x18)
#define DMA_S0M0AR_OFFSET       (0x1C)
#define DMA_S0M1AR_OFFSET       (0x20)
#define DMA_S0FCR_OFFSET        (0x24)
// Stream 1 to 7 offsets would follow a similar pattern relative to Stream 0

// EXTI Registers
#define EXTI_IMR_ADDR           ((volatile uint32_t *)0x40013C00)
#define EXTI_EMR_ADDR           ((volatile uint32_t *)0x40013C04)
#define EXTI_RTSR_ADDR          ((volatile uint32_t *)0x40013C08)
#define EXTI_FTSR_ADDR          ((volatile uint32_t *)0x40013C0C)
#define EXTI_SWIER_ADDR         ((volatile uint32_t *)0x40013C10)
#define EXTI_PR_ADDR            ((volatile uint32_t *)0x40013C14)

// ADC Registers
#define ADC_SR_ADDR             ((volatile uint32_t *)0x40012000)
#define ADC_CR1_ADDR            ((volatile uint32_t *)0x40012004)
#define ADC_CR2_ADDR            ((volatile uint32_t *)0x40012008)
#define ADC_SMPR1_ADDR          ((volatile uint32_t *)0x4001200C)
#define ADC_SMPR2_ADDR          ((volatile uint32_t *)0x40012010)
#define ADC_JOFR1_ADDR          ((volatile uint32_t *)0x40012014)
#define ADC_JOFR2_ADDR          ((volatile uint32_t *)0x40012018)
#define ADC_JOFR3_ADDR          ((volatile uint32_t *)0x4001201C)
#define ADC_JOFR4_ADDR          ((volatile uint32_t *)0x40012020)
#define ADC_HTR_ADDR            ((volatile uint32_t *)0x40012024)
#define ADC_LTR_ADDR            ((volatile uint32_t *)0x40012028)
#define ADC_SQR1_ADDR           ((volatile uint32_t *)0x4001202C)
#define ADC_SQR2_ADDR           ((volatile uint32_t *)0x40012030)
#define ADC_SQR3_ADDR           ((volatile uint32_t *)0x40012034)
#define ADC_JSQR_ADDR           ((volatile uint32_t *)0x40012038)
#define ADC_JDR1_ADDR           ((volatile uint32_t *)0x4001203C)
#define ADC_JDR2_ADDR           ((volatile uint32_t *)0x40012040)
#define ADC_JDR3_ADDR           ((volatile uint32_t *)0x40012044)
#define ADC_JDR4_ADDR           ((volatile uint32_t *)0x40012048)
#define ADC_DR_ADDR             ((volatile uint32_t *)0x4001204C)
#define ADC_CCR_ADDR            ((volatile uint32_t *)0x40012300)

// TIM Registers (Base Addresses for TIM1-5, 9-11)
#define TIM1_BASE_ADDR          ((volatile uint32_t *)0x40010000)
#define TIM2_BASE_ADDR          ((volatile uint32_t *)0x40000000)
#define TIM3_BASE_ADDR          ((volatile uint32_t *)0x40000400)
#define TIM4_BASE_ADDR          ((volatile uint32_t *)0x40000800)
#define TIM5_BASE_ADDR          ((volatile uint32_t *)0x40000C00)
#define TIM9_BASE_ADDR          ((volatile uint32_t *)0x40014000)
#define TIM10_BASE_ADDR         ((volatile uint32_t *)0x40014400)
#define TIM11_BASE_ADDR         ((volatile uint32_t *)0x40014800)

// TIM Register Offsets (relative to TIMx_BASE_ADDR)
#define TIM_CR1_OFFSET          (0x00)
#define TIM_CR2_OFFSET          (0x04)
#define TIM_SMCR_OFFSET         (0x08)
#define TIM_DIER_OFFSET         (0x0C)
#define TIM_SR_OFFSET           (0x10)
#define TIM_EGR_OFFSET          (0x14)
#define TIM_CCMR1_OFFSET        (0x18)
#define TIM_CCMR2_OFFSET        (0x1C)
#define TIM_CCER_OFFSET         (0x20)
#define TIM_CNT_OFFSET          (0x24)
#define TIM_PSC_OFFSET          (0x28)
#define TIM_ARR_OFFSET          (0x2C)
#define TIM_RCR_OFFSET          (0x30) // Only for TIM1
#define TIM_CCR1_OFFSET         (0x34)
#define TIM_CCR2_OFFSET         (0x38)
#define TIM_CCR3_OFFSET         (0x3C)
#define TIM_CCR4_OFFSET         (0x40)
#define TIM_BDTR_OFFSET         (0x44) // Only for TIM1
#define TIM_DCR_OFFSET          (0x48)
#define TIM_DMAR_OFFSET         (0x4C)
#define TIM_OR_OFFSET           (0x50) // Only for TIM2, TIM5

// USART Registers (Base Addresses for USART1, 2, 6)
#define USART1_BASE_ADDR        ((volatile uint32_t *)0x40011000)
#define USART2_BASE_ADDR        ((volatile uint32_t *)0x40004400)
#define USART6_BASE_ADDR        ((volatile uint32_t *)0x40011400)

// USART Register Offsets (relative to USARTx_BASE_ADDR)
#define USART_SR_OFFSET         (0x00)
#define USART_DR_OFFSET         (0x04)
#define USART_BRR_OFFSET        (0x08)
#define USART_CR1_OFFSET        (0x0C)
#define USART_CR2_OFFSET        (0x10)
#define USART_CR3_OFFSET        (0x14)
#define USART_GTPR_OFFSET       (0x18)

// I2C Registers (Base Addresses for I2C1, 2, 3)
#define I2C1_BASE_ADDR          ((volatile uint32_t *)0x40005400)
#define I2C2_BASE_ADDR          ((volatile uint32_t *)0x40005800)
#define I2C3_BASE_ADDR          ((volatile uint32_t *)0x40005C00)

// I2C Register Offsets (relative to I2Cx_BASE_ADDR)
#define I2C_CR1_OFFSET          (0x00)
#define I2C_CR2_OFFSET          (0x04)
#define I2C_OAR1_OFFSET         (0x08)
#define I2C_OAR2_OFFSET         (0x0C)
#define I2C_DR_OFFSET           (0x10)
#define I2C_SR1_OFFSET          (0x14)
#define I2C_SR2_OFFSET          (0x18)
#define I2C_CCR_OFFSET          (0x1C)
#define I2C_TRISE_OFFSET        (0x20)
#define I2C_FLTR_OFFSET         (0x24)

// SPI Registers (Base Addresses for SPI1, 2, 3)
#define SPI1_BASE_ADDR          ((volatile uint32_t *)0x40013000)
#define SPI2_BASE_ADDR          ((volatile uint32_t *)0x40003800)
#define SPI3_BASE_ADDR          ((volatile uint32_t *)0x40003C00)

// SPI Register Offsets (relative to SPIx_BASE_ADDR)
#define SPI_CR1_OFFSET          (0x00)
#define SPI_CR2_OFFSET          (0x04)
#define SPI_SR_OFFSET           (0x08)
#define SPI_DR_OFFSET           (0x0C)
#define SPI_CRCPR_OFFSET        (0x10)
#define SPI_RXCRCR_OFFSET       (0x14)
#define SPI_TXCRCR_OFFSET       (0x18)
#define SPI_I2SCFGR_OFFSET      (0x1C)
#define SPI_I2SPR_OFFSET        (0x20)

// =============================================================================
// Type Definitions for API Parameters
// =============================================================================

// MCU CONFIG
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

// LVD (Low Voltage Detection)
typedef enum {
    LVD_VOLT_0_5V = 0,
    LVD_VOLT_1V,
    LVD_VOLT_1_5V,
    LVD_VOLT_2V,
    LVD_VOLT_2_5V, // Placeholder for levels not explicitly listed but implied by "..."
    LVD_VOLT_3V,
    LVD_VOLT_3_5V,
    LVD_VOLT_4V,
    LVD_VOLT_4_5V,
    LVD_VOLT_5V
} t_lvd_thrthresholdLevel;

typedef enum {
    LVD_CHANNEL_NONE, // No specific channel mentioned, assuming global or single flag.
    // LVD channels are not specified in the register JSON by 'assigned_pin'.
    // Assuming a global flag or a generic channel.
    // Placeholder if specific bit in PWR_CSR represents a channel flag
    LVD_CHANNEL_FLAG
} t_lvd_channel;

// UART
typedef enum {
    UART_CHANNEL_1 = 0, // Maps to USART1
    UART_CHANNEL_2,     // Maps to USART2
    UART_CHANNEL_6      // Maps to USART6
} t_uart_channel;

typedef enum {
    UART_BAUD_9600 = 0,
    UART_BAUD_19200,
    UART_BAUD_38400,
    UART_BAUD_57600,
    UART_BAUD_115200
    // Add more baud rates as needed
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8_BITS = 0,
    UART_DATA_9_BITS
} t_uart_data_length;

typedef enum {
    UART_STOP_BITS_1 = 0,
    UART_STOP_BITS_0_5,
    UART_STOP_BITS_2,
    UART_STOP_BITS_1_5
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// I2C
typedef enum {
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_STANDARD = 0, // Up to 100 kHz
    I2C_CLK_SPEED_FAST          // Up to 400 kHz (Always use fast mode as per rule)
} t_i2c_clk_speed;

typedef enum {
    I2C_7BIT_ADDRESS = 0,
    I2C_10BIT_ADDRESS
    // This enum doesn't represent the address value, but the addressing mode.
    // The actual device address is passed as a value.
} t_i2c_device_addressing_mode; // Renamed to avoid confusion with actual address value

typedef tbyte t_i2c_device_address; // This represents the 7-bit or 10-bit address value

typedef enum {
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATALENGTH_8BIT = 0,
    I2C_DATALENGTH_16BIT
} t_i2c_datalength;

// SPI
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
    SPI_CPOL_LOW = 0,  // Clock polarity low
    SPI_CPOL_HIGH      // Clock polarity high
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1_EDGE = 0, // Clock phase 1st edge
    SPI_CPHA_2_EDGE      // Clock phase 2nd edge
} t_spi_cpha;

typedef enum {
    SPI_DFF_8_BIT = 0,
    SPI_DFF_16_BIT
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt
typedef enum {
    EXT_INT_CHANNEL_0 = 0,  // EXTI Line 0 (PA0, PB0, PC0, PD0, PE0, PH0)
    EXT_INT_CHANNEL_1,      // EXTI Line 1 (PA1, PB1, PC1, PD1, PE1, PH1)
    EXT_INT_CHANNEL_2,      // EXTI Line 2 (PA2, PB2, PC2, PD2, PE2)
    EXT_INT_CHANNEL_3,      // EXTI Line 3 (PA3, PB3, PC3, PD3, PE3)
    EXT_INT_CHANNEL_4,      // EXTI Line 4 (PA4, PB4, PC4, PD4, PE4)
    EXT_INT_CHANNEL_5,      // EXTI Line 5 (PA5, PB5, PC5, PD5, PE5)
    EXT_INT_CHANNEL_6,      // EXTI Line 6 (PA6, PB6, PC6, PD6, PE6)
    EXT_INT_CHANNEL_7,      // EXTI Line 7 (PA7, PB7, PC7, PD7, PE7)
    EXT_INT_CHANNEL_8,      // EXTI Line 8 (PA8, PB8, PC8, PD8, PE8)
    EXT_INT_CHANNEL_9,      // EXTI Line 9 (PA9, PB9, PC9, PD9, PE9)
    EXT_INT_CHANNEL_10,     // EXTI Line 10 (PA10, PB10, PC10, PD10, PE10)
    EXT_INT_CHANNEL_11,     // EXTI Line 11 (PA11, PB11, PC11, PD11, PE11)
    EXT_INT_CHANNEL_12,     // EXTI Line 12 (PA12, PB12, PC12, PD12, PE12)
    EXT_INT_CHANNEL_13,     // EXTI Line 13 (PA13, PB13, PC13, PD13, PE13)
    EXT_INT_CHANNEL_14,     // EXTI Line 14 (PA14, PB14, PC14, PD14, PE14)
    EXT_INT_CHANNEL_15      // EXTI Line 15 (PA15, PB15, PC15, PD15, PE15)
} t_external_int_channel;

typedef enum {
    EXT_INT_EDGE_RISING = 0,
    EXT_INT_EDGE_FALLING,
    EXT_INT_EDGE_BOTH
} t_external_int_edge;

// GPIO
typedef enum {
    GPIO_PORTA = 0,
    GPIO_PORTB,
    GPIO_PORTC,
    GPIO_PORTD,
    GPIO_PORTE,
    GPIO_PORTH
} t_port;

typedef enum {
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

typedef enum {
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;

// PWM
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

// ICU
typedef enum {
    ICU_TIM1_CH1 = 0, // PA8, PE9
    ICU_TIM1_CH2,     // PA9, PE11
    ICU_TIM1_CH3,     // PA10, PE13
    ICU_TIM1_CH4,     // PA11, PE14
    ICU_TIM2_CH1,     // PA0, PA5, PA15
    ICU_TIM2_CH2,     // PA1, PB3
    ICU_TIM2_CH3,     // PA2, PB10
    ICU_TIM2_CH4,     // PA3, PB11
    ICU_TIM3_CH1,     // PA6, PB4, PC6
    ICU_TIM3_CH2,     // PA7, PB5, PC7
    ICU_TIM3_CH3,     // PB0, PC8
    ICU_TIM3_CH4,     // PB1, PC9
    ICU_TIM4_CH1,     // PB6, PD12
    ICU_TIM4_CH2,     // PB7, PD13
    ICU_TIM4_CH3,     // PB8, PD14
    ICU_TIM4_CH4,     // PB9, PD15
    ICU_TIM5_CH1,     // PA0
    ICU_TIM5_CH2,     // PA1
    ICU_TIM5_CH3,     // PA2
    ICU_TIM5_CH4,     // PA3
    ICU_TIM9_CH1,     // PA2, PE5
    ICU_TIM9_CH2,     // PA3, PE6
    ICU_TIM10_CH1,    // PA6, PB8
    ICU_TIM11_CH1     // PA7, PB9
} t_icu_channel;

typedef enum {
    ICU_PRESCALER_DIV1 = 0,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8
    // Add more prescaler values as needed based on TIMx_PSC
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer
typedef enum {
    TIMER_CHANNEL_1 = 0, // General Purpose Timer 1
    TIMER_CHANNEL_2,     // General Purpose Timer 2
    TIMER_CHANNEL_3,     // General Purpose Timer 3
    TIMER_CHANNEL_4,     // General Purpose Timer 4
    TIMER_CHANNEL_5,     // General Purpose Timer 5
    TIMER_CHANNEL_9,     // General Purpose Timer 9
    TIMER_CHANNEL_10,    // General Purpose Timer 10
    TIMER_CHANNEL_11     // General Purpose Timer 11
} t_timer_channel;

// ADC
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
    ADC_CHANNEL_16,    // Internal temperature sensor
    ADC_CHANNEL_17,    // Internal VREFINT
    ADC_CHANNEL_18     // VBAT
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE = 0,
    ADC_MODE_CONTINUOUS,
    ADC_MODE_SCAN,
    ADC_MODE_INJECTED
} t_adc_mode_t;

// TT (Time Triggered OS)
typedef enum {
    TT_TICK_1MS = 0,
    TT_TICK_10MS,
    TT_TICK_100MS,
    TT_TICK_1S
} t_tick_time;

// =============================================================================
// API Function Prototypes - Copied from API.json
// =============================================================================

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Assuming frequency is long
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