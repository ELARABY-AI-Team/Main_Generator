#ifndef MCAL_H
#define MCAL_H

/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File
 *
 * This file provides the declarations for the MCAL layer,
 * abstracting the hardware specific details of the STM32F401RC MCU.
 *
 * @date 2023-10-27
 * @author Your Name/Generator
 */

// Core MCU header and standard C library includes as per Rules.json
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Data Type Definitions as per Rules.json
#define Unit_8 uint8_t
#define unit_16 uint16_t
#define unit_32 uint32_t

typedef Unit_8 tbyte;
typedef unit_16 tword;
typedef unit_32 tlong;

// =============================================================================
// Enums and Type Definitions
// =============================================================================

/**
 * @brief System Voltage levels for MCU configuration.
 */
typedef enum
{
    Vsource_3V = 0, /**< System voltage 3V */
    Vsource_5V      /**< System voltage 5V */
} t_sys_volt;

/**
 * @brief LVD threshold levels. (Values are placeholders as specific bit mappings are not provided).
 */
typedef enum
{
    Volt_0_5V = 0, /**< LVD threshold 0.5V */
    Volt_1V,       /**< LVD threshold 1V */
    Volt_1_5V,     /**< LVD threshold 1.5V */
    Volt_2V,       /**< LVD threshold 2V */
    Volt_2_5V,     /**< LVD threshold 2.5V */
    Volt_3V,       /**< LVD threshold 3V */
    Volt_3_5V,     /**< LVD threshold 3.5V */
    Volt_4V,       /**< LVD threshold 4V */
    Volt_4_5V,     /**< LVD threshold 4.5V */
    Volt_5V        /**< LVD threshold 5V */
} t_lvd_thrthresholdLevel;

/**
 * @brief LVD Channel (Placeholder, as specific LVD channels are not detailed in register_json).
 */
typedef enum
{
    LVD_CHANNEL_NONE = 0, /**< Generic LVD channel */
    // Add specific channels if provided by detailed register descriptions (not available in input)
} t_lvd_channel;

/**
 * @brief UART Channel enumeration.
 */
typedef enum
{
    UART_CHANNEL_1 = 0, /**< USART1 Channel */
    UART_CHANNEL_2,     /**< USART2 Channel */
    UART_CHANNEL_6      /**< USART6 Channel */
} t_uart_channel;

/**
 * @brief UART Baud Rate (Values are placeholders; actual register values depend on clock configuration).
 */
typedef enum
{
    UART_BAUD_RATE_9600 = 0,  /**< 9600 baud */
    UART_BAUD_RATE_19200,     /**< 19200 baud */
    UART_BAUD_RATE_115200     /**< 115200 baud */
} t_uart_baud_rate;

/**
 * @brief UART Data Length (Values are placeholders).
 */
typedef enum
{
    UART_DATA_LENGTH_8_BIT = 0, /**< 8-bit data length */
    UART_DATA_LENGTH_9_BIT      /**< 9-bit data length */
} t_uart_data_length;

/**
 * @brief UART Stop Bit (Values are placeholders).
 */
typedef enum
{
    UART_STOP_BIT_1 = 0, /**< 1 stop bit */
    UART_STOP_BIT_2      /**< 2 stop bits */
} t_uart_stop_bit;

/**
 * @brief UART Parity (Values are placeholders).
 */
typedef enum
{
    UART_PARITY_NONE = 0, /**< No parity */
    UART_PARITY_EVEN,     /**< Even parity */
    UART_PARITY_ODD       /**< Odd parity */
} t_uart_parity;

/**
 * @brief I2C Channel enumeration.
 */
typedef enum
{
    I2C_CHANNEL_1 = 0, /**< I2C1 Channel */
    I2C_CHANNEL_2,     /**< I2C2 Channel */
    I2C_CHANNEL_3      /**< I2C3 Channel */
} t_i2c_channel;

/**
 * @brief I2C Clock Speed (Values are placeholders).
 */
typedef enum
{
    I2C_CLK_SPEED_STANDARD = 0, /**< Standard mode (100 kHz) */
    I2C_CLK_SPEED_FAST          /**< Fast mode (400 kHz) */
} t_i2c_clk_speed;

/**
 * @brief I2C Device Address (Value is placeholder).
 */
typedef uint16_t t_i2c_device_address;

/**
 * @brief I2C Acknowledge (Values are placeholders).
 */
typedef enum
{
    I2C_ACK_DISABLE = 0, /**< Acknowledge disable */
    I2C_ACK_ENABLE       /**< Acknowledge enable */
} t_i2c_ack;

/**
 * @brief I2C Data Length (Values are placeholders).
 */
typedef enum
{
    I2C_DATA_LENGTH_8_BIT = 0, /**< 8-bit data length */
    I2C_DATA_LENGTH_16_BIT     /**< 16-bit data length */
} t_i2c_datalength;

/**
 * @brief SPI Channel enumeration.
 */
typedef enum
{
    SPI_CHANNEL_1 = 0, /**< SPI1 Channel */
    SPI_CHANNEL_2,     /**< SPI2 Channel */
    SPI_CHANNEL_3      /**< SPI3 Channel */
} t_spi_channel;

/**
 * @brief SPI Mode (Values are placeholders for master/slave).
 */
typedef enum
{
    SPI_MODE_SLAVE = 0, /**< Slave mode */
    SPI_MODE_MASTER     /**< Master mode */
} t_spi_mode;

/**
 * @brief SPI Clock Polarity (CPOL) (Values are placeholders).
 */
typedef enum
{
    SPI_CPOL_LOW = 0,  /**< Clock polarity low */
    SPI_CPOL_HIGH      /**< Clock polarity high */
} t_spi_cpol;

/**
 * @brief SPI Clock Phase (CPHA) (Values are placeholders).
 */
typedef enum
{
    SPI_CPHA_1_EDGE = 0, /**< Clock phase 1st edge */
    SPI_CPHA_2_EDGE      /**< Clock phase 2nd edge */
} t_spi_cpha;

/**
 * @brief SPI Data Frame Format (DFF) (Values are placeholders).
 */
typedef enum
{
    SPI_DFF_8_BIT = 0, /**< 8-bit data frame */
    SPI_DFF_16_BIT     /**< 16-bit data frame */
} t_spi_dff;

/**
 * @brief SPI Bit Order (Values are placeholders).
 */
typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0, /**< MSB first */
    SPI_BIT_ORDER_LSB_FIRST      /**< LSB first */
} t_spi_bit_order;

/**
 * @brief External Interrupt Channel (EXTI Line numbers).
 */
typedef enum
{
    EXTI_LINE_0 = 0,  /**< EXTI Line 0 */
    EXTI_LINE_1,      /**< EXTI Line 1 */
    EXTI_LINE_2,      /**< EXTI Line 2 */
    EXTI_LINE_3,      /**< EXTI Line 3 */
    EXTI_LINE_4,      /**< EXTI Line 4 */
    EXTI_LINE_5,      /**< EXTI Line 5 */
    EXTI_LINE_6,      /**< EXTI Line 6 */
    EXTI_LINE_7,      /**< EXTI Line 7 */
    EXTI_LINE_8,      /**< EXTI Line 8 */
    EXTI_LINE_9,      /**< EXTI Line 9 */
    EXTI_LINE_10,     /**< EXTI Line 10 */
    EXTI_LINE_11,     /**< EXTI Line 11 */
    EXTI_LINE_12,     /**< EXTI Line 12 */
    EXTI_LINE_13,     /**< EXTI Line 13 */
    EXTI_LINE_14,     /**< EXTI Line 14 */
    EXTI_LINE_15      /**< EXTI Line 15 */
} t_external_int_channel;

/**
 * @brief External Interrupt Edge (Values are placeholders).
 */
typedef enum
{
    EXTI_EDGE_RISING = 0,  /**< Rising edge trigger */
    EXTI_EDGE_FALLING,     /**< Falling edge trigger */
    EXTI_EDGE_RISING_FALLING /**< Both rising and falling edge trigger */
} t_external_int_edge;

/**
 * @brief GPIO Port enumeration.
 */
typedef enum
{
    GPIO_PORTA = 0, /**< GPIO Port A */
    GPIO_PORTB,     /**< GPIO Port B */
    GPIO_PORTC,     /**< GPIO Port C */
    GPIO_PORTD,     /**< GPIO Port D */
    GPIO_PORTE,     /**< GPIO Port E */
    GPIO_PORTH      /**< GPIO Port H */
} t_port;

/**
 * @brief GPIO Pin enumeration.
 */
typedef enum
{
    GPIO_PIN_0 = 0, /**< Pin 0 */
    GPIO_PIN_1,     /**< Pin 1 */
    GPIO_PIN_2,     /**< Pin 2 */
    GPIO_PIN_3,     /**< Pin 3 */
    GPIO_PIN_4,     /**< Pin 4 */
    GPIO_PIN_5,     /**< Pin 5 */
    GPIO_PIN_6,     /**< Pin 6 */
    GPIO_PIN_7,     /**< Pin 7 */
    GPIO_PIN_8,     /**< Pin 8 */
    GPIO_PIN_9,     /**< Pin 9 */
    GPIO_PIN_10,    /**< Pin 10 */
    GPIO_PIN_11,    /**< Pin 11 */
    GPIO_PIN_12,    /**< Pin 12 */
    GPIO_PIN_13,    /**< Pin 13 */
    GPIO_PIN_14,    /**< Pin 14 */
    GPIO_PIN_15     /**< Pin 15 */
} t_pin;

/**
 * @brief GPIO Direction.
 */
typedef enum
{
    GPIO_DIRECTION_INPUT = 0,  /**< Input direction */
    GPIO_DIRECTION_OUTPUT      /**< Output direction */
} t_direction;

/**
 * @brief PWM Channel enumeration (mapping to specific Timer/Channel combinations based on assigned pins).
 */
typedef enum
{
    PWM_TIM1_CH1 = 0,  // PA8, PE9
    PWM_TIM1_CH2,      // PA9, PE11
    PWM_TIM1_CH3,      // PA10, PE13
    PWM_TIM1_CH4,      // PA11, PE14
    PWM_TIM2_CH1,      // PA0, PA5, PA15
    PWM_TIM2_CH2,      // PA1, PB3
    PWM_TIM2_CH3,      // PA2, PB10
    PWM_TIM2_CH4,      // PA3, PB11
    PWM_TIM3_CH1,      // PA6, PB4, PC6
    PWM_TIM3_CH2,      // PA7, PB5, PC7
    PWM_TIM3_CH3,      // PB0, PC8
    PWM_TIM3_CH4,      // PB1, PC9
    PWM_TIM4_CH1,      // PB6, PD12
    PWM_TIM4_CH2,      // PB7, PD13
    PWM_TIM4_CH3,      // PB8, PD14
    PWM_TIM4_CH4,      // PB9, PD15
    PWM_TIM5_CH1,      // PA0
    PWM_TIM5_CH2,      // PA1
    PWM_TIM5_CH3,      // PA2
    PWM_TIM5_CH4,      // PA3
    PWM_TIM9_CH1,      // PA2, PE5
    PWM_TIM9_CH2,      // PA3, PE6
    PWM_TIM10_CH1,     // PA6, PB8
    PWM_TIM11_CH1      // PA7, PB9
} t_pwm_channel;

/**
 * @brief ICU Channel enumeration (mapping to specific Timer/Channel combinations).
 */
typedef t_pwm_channel t_icu_channel; // Reusing PWM channel definitions as they map similarly to timer channels

/**
 * @brief ICU Prescaler (Values are placeholders).
 */
typedef enum
{
    ICU_PRESCALER_1 = 0,   /**< No prescaling */
    ICU_PRESCALER_2,       /**< Divide by 2 */
    ICU_PRESCALER_4,       /**< Divide by 4 */
    ICU_PRESCALER_8        /**< Divide by 8 */
} t_icu_prescaller;

/**
 * @brief ICU Edge (Values are placeholders).
 */
typedef enum
{
    ICU_EDGE_RISING = 0,   /**< Rising edge */
    ICU_EDGE_FALLING,      /**< Falling edge */
    ICU_EDGE_BOTH          /**< Both edges */
} t_icu_edge;

/**
 * @brief Timer Channel enumeration (for generic timer operations).
 */
typedef enum
{
    TIMER_CHANNEL_TIM1 = 0, /**< TIM1 */
    TIMER_CHANNEL_TIM2,     /**< TIM2 */
    TIMER_CHANNEL_TIM3,     /**< TIM3 */
    TIMER_CHANNEL_TIM4,     /**< TIM4 */
    TIMER_CHANNEL_TIM5,     /**< TIM5 */
    TIMER_CHANNEL_TIM9,     /**< TIM9 */
    TIMER_CHANNEL_TIM10,    /**< TIM10 */
    TIMER_CHANNEL_TIM11     /**< TIM11 */
} t_timer_channel;

/**
 * @brief ADC Channel enumeration (derived from assigned pins and common STM32 ADC channels).
 * Note: These map to internal ADC channels, the specific pin to channel mapping depends on the MCU datasheet.
 */
typedef enum
{
    ADC_CHANNEL_0 = 0, /**< ADC Channel 0 (e.g., PA0) */
    ADC_CHANNEL_1,     /**< ADC Channel 1 (e.g., PA1) */
    ADC_CHANNEL_2,     /**< ADC Channel 2 (e.g., PA2) */
    ADC_CHANNEL_3,     /**< ADC Channel 3 (e.g., PA3) */
    ADC_CHANNEL_4,     /**< ADC Channel 4 (e.g., PA4) */
    ADC_CHANNEL_5,     /**< ADC Channel 5 (e.g., PA5) */
    ADC_CHANNEL_6,     /**< ADC Channel 6 (e.g., PA6) */
    ADC_CHANNEL_7,     /**< ADC Channel 7 (e.g., PA7) */
    ADC_CHANNEL_8,     /**< ADC Channel 8 (e.g., PB0) */
    ADC_CHANNEL_9,     /**< ADC Channel 9 (e.g., PB1) */
    ADC_CHANNEL_10,    /**< ADC Channel 10 (e.g., PC0) */
    ADC_CHANNEL_11,    /**< ADC Channel 11 (e.g., PC1) */
    ADC_CHANNEL_12,    /**< ADC Channel 12 (e.g., PC2) */
    ADC_CHANNEL_13,    /**< ADC Channel 13 (e.g., PC3) */
    ADC_CHANNEL_14,    /**< ADC Channel 14 (e.g., PC4) */
    ADC_CHANNEL_15,    /**< ADC Channel 15 (e.g., PC5) */
    // Internal channels like VrefInt, TempSensor are not explicitly listed in register JSON pins.
    // Max channel for STM32F401 is typically 18 for Vref, Temp, VBAT. We'll include up to 18 as a general range.
    ADC_CHANNEL_16,    /**< ADC Channel 16 (e.g., Temp Sensor) */
    ADC_CHANNEL_17,    /**< ADC Channel 17 (e.g., VrefInt) */
    ADC_CHANNEL_18     /**< ADC Channel 18 (e.g., VBAT) */
} t_adc_channel;

/**
 * @brief ADC Mode (Values are placeholders).
 */
typedef enum
{
    ADC_MODE_SINGLE = 0,   /**< Single conversion mode */
    ADC_MODE_CONTINUOUS,   /**< Continuous conversion mode */
    ADC_MODE_DMA           /**< DMA mode */
} t_adc_mode_t;

/**
 * @brief Tick time for Time-Triggered OS.
 */
typedef tword t_tick_time; // milliseconds

// =============================================================================
// Register Address Macros (from Register JSON)
// =============================================================================

// Flash Registers
#define FLASH_ACR_REG       ((volatile uint32_t *)0x40023C00)
#define FLASH_KEYR_REG      ((volatile uint32_t *)0x40023C04)
#define FLASH_OPTKEYR_REG   ((volatile uint32_t *)0x40023C08)
#define FLASH_SR_REG        ((volatile uint32_t *)0x40023C0C)
#define FLASH_CR_REG        ((volatile uint32_t *)0x40023C10)
#define FLASH_OPTCR_REG     ((volatile uint32_t *)0x40023C14)

// CRC Registers
#define CRC_DR_REG          ((volatile uint32_t *)0x40023000)
#define CRC_IDR_REG         ((volatile uint32_t *)0x40023004)
#define CRC_CR_REG          ((volatile uint32_t *)0x40023008)

// PWR Registers
#define PWR_CR_REG          ((volatile uint32_t *)0x40007000)
#define PWR_CSR_REG         ((volatile uint32_t *)0x40007004)

// RCC Registers
#define RCC_CR_REG          ((volatile uint32_t *)0x40023800)
#define RCC_PLLCFGR_REG     ((volatile uint32_t *)0x40023804)
#define RCC_CFGR_REG        ((volatile uint32_t *)0x40023808)
#define RCC_CIR_REG         ((volatile uint32_t *)0x4002380C)
#define RCC_AHB1RSTR_REG    ((volatile uint32_t *)0x40023810)
#define RCC_AHB2RSTR_REG    ((volatile uint32_t *)0x40023814)
#define RCC_APB1RSTR_REG    ((volatile uint32_t *)0x40023818)
#define RCC_APB2RSTR_REG    ((volatile uint32_t *)0x4002381C)
#define RCC_AHB1ENR_REG     ((volatile uint32_t *)0x40023830)
#define RCC_AHB2ENR_REG     ((volatile uint32_t *)0x40023834)
#define RCC_APB1ENR_REG     ((volatile uint32_t *)0x40023838)
#define RCC_APB2ENR_REG     ((volatile uint32_t *)0x4002383C)
#define RCC_AHB1LPENR_REG   ((volatile uint32_t *)0x40023850)
#define RCC_AHB2LPENR_REG   ((volatile uint32_t *)0x40023854)
#define RCC_APB1LPENR_REG   ((volatile uint32_t *)0x40023858)
#define RCC_APB2LPENR_REG   ((volatile uint32_t *)0x4002385C)
#define RCC_BDCR_REG        ((volatile uint32_t *)0x40023870)
#define RCC_CSR_REG         ((volatile uint32_t *)0x40023874)
#define RCC_SSCGR_REG       ((volatile uint32_t *)0x40023880)
#define RCC_PLLI2SCFGR_REG  ((volatile uint32_t *)0x40023884)
#define RCC_DCKCFGR_REG     ((volatile uint32_t *)0x4002388C)

// SYSCFG Registers
#define SYSCFG_MEMRMP_REG   ((volatile uint32_t *)0x40013800)
#define SYSCFG_PMC_REG      ((volatile uint32_t *)0x40013804)
#define SYSCFG_EXTICR1_REG  ((volatile uint32_t *)0x40013808)
#define SYSCFG_EXTICR2_REG  ((volatile uint32_t *)0x4001380C)
#define SYSCFG_EXTICR3_REG  ((volatile uint32_t *)0x40013810)
#define SYSCFG_EXTICR4_REG  ((volatile uint32_t *)0x40013814)
#define SYSCFG_CMPCR_REG    ((volatile uint32_t *)0x40013820)

// GPIO Registers (Port A-H)
#define GPIOA_MODER_REG     ((volatile uint32_t *)0x40020000)
#define GPIOA_OTYPER_REG    ((volatile uint32_t *)0x40020004)
#define GPIOA_OSPEEDR_REG   ((volatile uint32_t *)0x40020008)
#define GPIOA_PUPDR_REG     ((volatile uint32_t *)0x4002000C)
#define GPIOA_IDR_REG       ((volatile uint32_t *)0x40020010)
#define GPIOA_ODR_REG       ((volatile uint32_t *)0x40020014)
#define GPIOA_BSRR_REG      ((volatile uint32_t *)0x40020018)
#define GPIOA_LCKR_REG      ((volatile uint32_t *)0x4002001C)
#define GPIOA_AFRL_REG      ((volatile uint32_t *)0x40020020)
#define GPIOA_AFRH_REG      ((volatile uint32_t *)0x40020024)

#define GPIOB_MODER_REG     ((volatile uint32_t *)0x40020400)
#define GPIOB_OTYPER_REG    ((volatile uint32_t *)0x40020404)
#define GPIOB_OSPEEDR_REG   ((volatile uint32_t *)0x40020408)
#define GPIOB_PUPDR_REG     ((volatile uint32_t *)0x4002040C)
#define GPIOB_IDR_REG       ((volatile uint32_t *)0x40020410)
#define GPIOB_ODR_REG       ((volatile uint32_t *)0x40020414)
#define GPIOB_BSRR_REG      ((volatile uint32_t *)0x40020418)
#define GPIOB_LCKR_REG      ((volatile uint32_t *)0x4002041C)
#define GPIOB_AFRL_REG      ((volatile uint32_t *)0x40020420)
#define GPIOB_AFRH_REG      ((volatile uint32_t *)0x40020424)

#define GPIOC_MODER_REG     ((volatile uint32_t *)0x40020800)
#define GPIOC_OTYPER_REG    ((volatile uint32_t *)0x40020804)
#define GPIOC_OSPEEDR_REG   ((volatile uint32_t *)0x40020808)
#define GPIOC_PUPDR_REG     ((volatile uint32_t *)0x4002080C)
#define GPIOC_IDR_REG       ((volatile uint32_t *)0x40020810)
#define GPIOC_ODR_REG       ((volatile uint32_t *)0x40020814)
#define GPIOC_BSRR_REG      ((volatile uint32_t *)0x40020818)
#define GPIOC_LCKR_REG      ((volatile uint32_t *)0x4002081C)
#define GPIOC_AFRL_REG      ((volatile uint32_t *)0x40020820)
#define GPIOC_AFRH_REG      ((volatile uint32_t *)0x40020824)

#define GPIOD_MODER_REG     ((volatile uint32_t *)0x40020C00)
#define GPIOD_OTYPER_REG    ((volatile uint32_t *)0x40020C04)
#define GPIOD_OSPEEDR_REG   ((volatile uint32_t *)0x40020C08)
#define GPIOD_PUPDR_REG     ((volatile uint32_t *)0x40020C0C)
#define GPIOD_IDR_REG       ((volatile uint32_t *)0x40020C10)
#define GPIOD_ODR_REG       ((volatile uint32_t *)0x40020C14)
#define GPIOD_BSRR_REG      ((volatile uint32_t *)0x40020C18)
#define GPIOD_LCKR_REG      ((volatile uint32_t *)0x40020C1C)
#define GPIOD_AFRL_REG      ((volatile uint32_t *)0x40020C20)
#define GPIOD_AFRH_REG      ((volatile uint32_t *)0x40020C24)

#define GPIOE_MODER_REG     ((volatile uint32_t *)0x40021000)
#define GPIOE_OTYPER_REG    ((volatile uint32_t *)0x40021004)
#define GPIOE_OSPEEDR_REG   ((volatile uint32_t *)0x40021008)
#define GPIOE_PUPDR_REG     ((volatile uint32_t *)0x4002100C)
#define GPIOE_IDR_REG       ((volatile uint32_t *)0x40021010)
#define GPIOE_ODR_REG       ((volatile uint32_t *)0x40021014)
#define GPIOE_BSRR_REG      ((volatile uint32_t *)0x40021018)
#define GPIOE_LCKR_REG      ((volatile uint32_t *)0x4002101C)
#define GPIOE_AFRL_REG      ((volatile uint32_t *)0x40021020)
#define GPIOE_AFRH_REG      ((volatile uint32_t *)0x40021024)

#define GPIOH_MODER_REG     ((volatile uint32_t *)0x40021C00)
#define GPIOH_OTYPER_REG    ((volatile uint32_t *)0x40021C04)
#define GPIOH_OSPEEDR_REG   ((volatile uint32_t *)0x40021C08)
#define GPIOH_PUPDR_REG     ((volatile uint32_t *)0x40021C0C)
#define GPIOH_IDR_REG       ((volatile uint32_t *)0x40021C10)
#define GPIOH_ODR_REG       ((volatile uint32_t *)0x40021C14)
#define GPIOH_BSRR_REG      ((volatile uint32_t *)0x40021C18)
#define GPIOH_LCKR_REG      ((volatile uint32_t *)0x40021C1C)
#define GPIOH_AFRL_REG      ((volatile uint32_t *)0x40021C20)
#define GPIOH_AFRH_REG      ((volatile uint32_t *)0x40021C24)

// DMA Registers (DMA1, DMA2)
#define DMA1_LISR_REG       ((volatile uint32_t *)0x40026000)
#define DMA1_HISR_REG       ((volatile uint32_t *)0x40026004)
#define DMA1_LIFCR_REG      ((volatile uint32_t *)0x40026008)
#define DMA1_HIFCR_REG      ((volatile uint32_t *)0x4002600C)
#define DMA1_S0CR_REG       ((volatile uint32_t *)0x40026010)
#define DMA1_S0NDTR_REG     ((volatile uint32_t *)0x40026014)
#define DMA1_S0PAR_REG      ((volatile uint32_t *)0x40026018)
#define DMA1_S0M0AR_REG     ((volatile uint32_t *)0x4002601C)
#define DMA1_S0M1AR_REG     ((volatile uint32_t *)0x40026020)
#define DMA1_S0FCR_REG      ((volatile uint32_t *)0x40026024)
#define DMA1_S1CR_REG       ((volatile uint32_t *)0x40026028)
#define DMA1_S1NDTR_REG     ((volatile uint32_t *)0x4002602C)
#define DMA1_S1PAR_REG      ((volatile uint32_t *)0x40026030)
#define DMA1_S1M0AR_REG     ((volatile uint32_t *)0x40026034)
#define DMA1_S1M1AR_REG     ((volatile uint32_t *)0x40026038)
#define DMA1_S1FCR_REG      ((volatile uint32_t *)0x4002603C)
#define DMA1_S2CR_REG       ((volatile uint32_t *)0x40026040)
#define DMA1_S2NDTR_REG     ((volatile uint32_t *)0x40026044)
#define DMA1_S2PAR_REG      ((volatile uint32_t *)0x40026048)
#define DMA1_S2M0AR_REG     ((volatile uint32_t *)0x4002604C)
#define DMA1_S2M1AR_REG     ((volatile uint32_t *)0x40026050)
#define DMA1_S2FCR_REG      ((volatile uint32_t *)0x40026054)
#define DMA1_S3CR_REG       ((volatile uint32_t *)0x40026058)
#define DMA1_S3NDTR_REG     ((volatile uint32_t *)0x4002605C)
#define DMA1_S3PAR_REG      ((volatile uint32_t *)0x40026060)
#define DMA1_S3M0AR_REG     ((volatile uint32_t *)0x40026064)
#define DMA1_S3M1AR_REG     ((volatile uint32_t *)0x40026068)
#define DMA1_S3FCR_REG      ((volatile uint32_t *)0x4002606C)
#define DMA1_S4CR_REG       ((volatile uint32_t *)0x40026070)
#define DMA1_S4NDTR_REG     ((volatile uint32_t *)0x40026074)
#define DMA1_S4PAR_REG      ((volatile uint32_t *)0x40026078)
#define DMA1_S4M0AR_REG     ((volatile uint32_t *)0x4002607C)
#define DMA1_S4M1AR_REG     ((volatile uint32_t *)0x40026080)
#define DMA1_S4FCR_REG      ((volatile uint32_t *)0x40026084)
#define DMA1_S5CR_REG       ((volatile uint32_t *)0x40026088)
#define DMA1_S5NDTR_REG     ((volatile uint32_t *)0x4002608C)
#define DMA1_S5PAR_REG      ((volatile uint32_t *)0x40026090)
#define DMA1_S5M0AR_REG     ((volatile uint32_t *)0x40026094)
#define DMA1_S5M1AR_REG     ((volatile uint32_t *)0x40026098)
#define DMA1_S5FCR_REG      ((volatile uint32_t *)0x4002609C)
#define DMA1_S6CR_REG       ((volatile uint32_t *)0x400260A0)
#define DMA1_S6NDTR_REG     ((volatile uint32_t *)0x400260A4)
#define DMA1_S6PAR_REG      ((volatile uint32_t *)0x400260A8)
#define DMA1_S6M0AR_REG     ((volatile uint32_t *)0x400260AC)
#define DMA1_S6M1AR_REG     ((volatile uint32_t *)0x400260B0)
#define DMA1_S6FCR_REG      ((volatile uint32_t *)0x400260B4)
#define DMA1_S7CR_REG       ((volatile uint32_t *)0x400260B8)
#define DMA1_S7NDTR_REG     ((volatile uint32_t *)0x400260BC)
#define DMA1_S7PAR_REG      ((volatile uint32_t *)0x400260C0)
#define DMA1_S7M0AR_REG     ((volatile uint32_t *)0x400260C4)
#define DMA1_S7M1AR_REG     ((volatile uint32_t *)0x400260C8)
#define DMA1_S7FCR_REG      ((volatile uint32_t *)0x400260CC)

#define DMA2_LISR_REG       ((volatile uint32_t *)0x40026400)
#define DMA2_HISR_REG       ((volatile uint32_t *)0x40026404)
#define DMA2_LIFCR_REG      ((volatile uint32_t *)0x40026408)
#define DMA2_HIFCR_REG      ((volatile uint32_t *)0x4002640C)
#define DMA2_S0CR_REG       ((volatile uint32_t *)0x40026410)
#define DMA2_S0NDTR_REG     ((volatile uint32_t *)0x40026414)
#define DMA2_S0PAR_REG      ((volatile uint32_t *)0x40026418)
#define DMA2_S0M0AR_REG     ((volatile uint32_t *)0x4002641C)
#define DMA2_S0M1AR_REG     ((volatile uint32_t *)0x40026420)
#define DMA2_S0FCR_REG      ((volatile uint32_t *)0x40026424)
#define DMA2_S1CR_REG       ((volatile uint32_t *)0x40026428)
#define DMA2_S1NDTR_REG     ((volatile uint32_t *)0x4002642C)
#define DMA2_S1PAR_REG      ((volatile uint32_t *)0x40026430)
#define DMA2_S1M0AR_REG     ((volatile uint32_t *)0x40026434)
#define DMA2_S1M1AR_REG     ((volatile uint32_t *)0x40026438)
#define DMA2_S1FCR_REG      ((volatile uint32_t *)0x4002643C)
#define DMA2_S2CR_REG       ((volatile uint32_t *)0x40026440)
#define DMA2_S2NDTR_REG     ((volatile uint32_t *)0x40026444)
#define DMA2_S2PAR_REG      ((volatile uint32_t *)0x40026448)
#define DMA2_S2M0AR_REG     ((volatile uint32_t *)0x4002644C)
#define DMA2_S2M1AR_REG     ((volatile uint32_t *)0x40026450)
#define DMA2_S2FCR_REG      ((volatile uint32_t *)0x40026454)
#define DMA2_S3CR_REG       ((volatile uint32_t *)0x40026458)
#define DMA2_S3NDTR_REG     ((volatile uint32_t *)0x4002645C)
#define DMA2_S3PAR_REG      ((volatile uint32_t *)0x40026460)
#define DMA2_S3M0AR_REG     ((volatile uint32_t *)0x40026464)
#define DMA2_S3M1AR_REG     ((volatile uint32_t *)0x40026468)
#define DMA2_S3FCR_REG      ((volatile uint32_t *)0x4002646C)
#define DMA2_S4CR_REG       ((volatile uint32_t *)0x40026470)
#define DMA2_S4NDTR_REG     ((volatile uint32_t *)0x40026474)
#define DMA2_S4PAR_REG      ((volatile uint32_t *)0x40026478)
#define DMA2_S4M0AR_REG     ((volatile uint32_t *)0x4002647C)
#define DMA2_S4M1AR_REG     ((volatile uint32_t *)0x40026480)
#define DMA2_S4FCR_REG      ((volatile uint32_t *)0x40026484)
#define DMA2_S5CR_REG       ((volatile uint32_t *)0x40026488)
#define DMA2_S5NDTR_REG     ((volatile uint32_t *)0x4002648C)
#define DMA2_S5PAR_REG      ((volatile uint32_t *)0x40026490)
#define DMA2_S5M0AR_REG     ((volatile uint32_t *)0x40026494)
#define DMA2_S5M1AR_REG     ((volatile uint32_t *)0x40026498)
#define DMA2_S5FCR_REG      ((volatile uint32_t *)0x4002649C)
#define DMA2_S6CR_REG       ((volatile uint32_t *)0x400264A0)
#define DMA2_S6NDTR_REG     ((volatile uint32_t *)0x400264A4)
#define DMA2_S6PAR_REG      ((volatile uint32_t *)0x400264A8)
#define DMA2_S6M0AR_REG     ((volatile uint32_t *)0x400264AC)
#define DMA2_S6M1AR_REG     ((volatile uint32_t *)0x400264B0)
#define DMA2_S6FCR_REG      ((volatile uint32_t *)0x400264B4)
#define DMA2_S7CR_REG       ((volatile uint32_t *)0x400264B8)
#define DMA2_S7NDTR_REG     ((volatile uint32_t *)0x400264BC)
#define DMA2_S7PAR_REG      ((volatile uint32_t *)0x400264C0)
#define DMA2_S7M0AR_REG     ((volatile uint32_t *)0x400264C4)
#define DMA2_S7M1AR_REG     ((volatile uint32_t *)0x400264C8)
#define DMA2_S7FCR_REG      ((volatile uint32_t *)0x400264CC)

// EXTI Registers
#define EXTI_IMR_REG        ((volatile uint32_t *)0x40013C00)
#define EXTI_EMR_REG        ((volatile uint32_t *)0x40013C04)
#define EXTI_RTSR_REG       ((volatile uint32_t *)0x40013C08)
#define EXTI_FTSR_REG       ((volatile uint32_t *)0x40013C0C)
#define EXTI_SWIER_REG      ((volatile uint32_t *)0x40013C10)
#define EXTI_PR_REG         ((volatile uint32_t *)0x40013C14)

// ADC Registers
#define ADC_SR_REG          ((volatile uint32_t *)0x40012000)
#define ADC_CR1_REG         ((volatile uint32_t *)0x40012004)
#define ADC_CR2_REG         ((volatile uint32_t *)0x40012008)
#define ADC_SMPR1_REG       ((volatile uint32_t *)0x4001200C)
#define ADC_SMPR2_REG       ((volatile uint32_t *)0x40012010)
#define ADC_JOFR1_REG       ((volatile uint32_t *)0x40012014)
#define ADC_JOFR2_REG       ((volatile uint32_t *)0x40012018)
#define ADC_JOFR3_REG       ((volatile uint32_t *)0x4001201C)
#define ADC_JOFR4_REG       ((volatile uint32_t *)0x40012020)
#define ADC_HTR_REG         ((volatile uint32_t *)0x40012024)
#define ADC_LTR_REG         ((volatile uint32_t *)0x40012028)
#define ADC_SQR1_REG        ((volatile uint32_t *)0x4001202C)
#define ADC_SQR2_REG        ((volatile uint32_t *)0x40012030)
#define ADC_SQR3_REG        ((volatile uint32_t *)0x40012034)
#define ADC_JSQR_REG        ((volatile uint32_t *)0x40012038)
#define ADC_JDR1_REG        ((volatile uint32_t *)0x4001203C)
#define ADC_JDR2_REG        ((volatile uint32_t *)0x40012040)
#define ADC_JDR3_REG        ((volatile uint32_t *)0x40012044)
#define ADC_JDR4_REG        ((volatile uint32_t *)0x40012048)
#define ADC_DR_REG          ((volatile uint32_t *)0x4001204C)
#define ADC_CCR_REG         ((volatile uint32_t *)0x40012300)

// TIM1 Registers
#define TIM1_CR1_REG        ((volatile uint32_t *)0x40010000)
#define TIM1_CR2_REG        ((volatile uint32_t *)0x40010004)
#define TIM1_SMCR_REG       ((volatile uint32_t *)0x40010008)
#define TIM1_DIER_REG       ((volatile uint32_t *)0x4001000C)
#define TIM1_SR_REG         ((volatile uint32_t *)0x40010010)
#define TIM1_EGR_REG        ((volatile uint32_t *)0x40010014)
#define TIM1_CCMR1_REG      ((volatile uint32_t *)0x40010018)
#define TIM1_CCMR2_REG      ((volatile uint32_t *)0x4001001C)
#define TIM1_CCER_REG       ((volatile uint32_t *)0x40010020)
#define TIM1_CNT_REG        ((volatile uint32_t *)0x40010024)
#define TIM1_PSC_REG        ((volatile uint32_t *)0x40010028)
#define TIM1_ARR_REG        ((volatile uint32_t *)0x4001002C)
#define TIM1_RCR_REG        ((volatile uint32_t *)0x40010030)
#define TIM1_CCR1_REG       ((volatile uint32_t *)0x40010034)
#define TIM1_CCR2_REG       ((volatile uint32_t *)0x40010038)
#define TIM1_CCR3_REG       ((volatile uint32_t *)0x4001003C)
#define TIM1_CCR4_REG       ((volatile uint32_t *)0x40010040)
#define TIM1_BDTR_REG       ((volatile uint32_t *)0x40010044)
#define TIM1_DCR_REG        ((volatile uint32_t *)0x40010048)
#define TIM1_DMAR_REG       ((volatile uint32_t *)0x4001004C)

// TIM2 Registers
#define TIM2_CR1_REG        ((volatile uint32_t *)0x40000000)
#define TIM2_CR2_REG        ((volatile uint32_t *)0x40000004)
#define TIM2_SMCR_REG       ((volatile uint32_t *)0x40000008)
#define TIM2_DIER_REG       ((volatile uint32_t *)0x4000000C)
#define TIM2_SR_REG         ((volatile uint32_t *)0x40000010)
#define TIM2_EGR_REG        ((volatile uint32_t *)0x40000014)
#define TIM2_CCMR1_REG      ((volatile uint32_t *)0x40000018)
#define TIM2_CCMR2_REG      ((volatile uint32_t *)0x4000001C)
#define TIM2_CCER_REG       ((volatile uint32_t *)0x40000020)
#define TIM2_CNT_REG        ((volatile uint32_t *)0x40000024)
#define TIM2_PSC_REG        ((volatile uint32_t *)0x40000028)
#define TIM2_ARR_REG        ((volatile uint32_t *)0x4000002C)
#define TIM2_CCR1_REG       ((volatile uint32_t *)0x40000034)
#define TIM2_CCR2_REG       ((volatile uint32_t *)0x40000038)
#define TIM2_CCR3_REG       ((volatile uint32_t *)0x4000003C)
#define TIM2_CCR4_REG       ((volatile uint32_t *)0x40000040)
#define TIM2_DCR_REG        ((volatile uint32_t *)0x40000048)
#define TIM2_DMAR_REG       ((volatile uint32_t *)0x4000004C)
#define TIM2_OR_REG         ((volatile uint32_t *)0x40000050)

// TIM3 Registers
#define TIM3_CR1_REG        ((volatile uint32_t *)0x40000400)
#define TIM3_CR2_REG        ((volatile uint32_t *)0x40000404)
#define TIM3_SMCR_REG       ((volatile uint32_t *)0x40000408)
#define TIM3_DIER_REG       ((volatile uint32_t *)0x4000040C)
#define TIM3_SR_REG         ((volatile uint32_t *)0x40000410)
#define TIM3_EGR_REG        ((volatile uint32_t *)0x40000414)
#define TIM3_CCMR1_REG      ((volatile uint32_t *)0x40000418)
#define TIM3_CCMR2_REG      ((volatile uint32_t *)0x4000041C)
#define TIM3_CCER_REG       ((volatile uint32_t *)0x40000420)
#define TIM3_CNT_REG        ((volatile uint32_t *)0x40000424)
#define TIM3_PSC_REG        ((volatile uint32_t *)0x40000428)
#define TIM3_ARR_REG        ((volatile uint32_t *)0x4000042C)
#define TIM3_CCR1_REG       ((volatile uint32_t *)0x40000434)
#define TIM3_CCR2_REG       ((volatile uint32_t *)0x40000438)
#define TIM3_CCR3_REG       ((volatile uint32_t *)0x4000043C)
#define TIM3_CCR4_REG       ((volatile uint32_t *)0x40000440)
#define TIM3_DCR_REG        ((volatile uint32_t *)0x40000448)
#define TIM3_DMAR_REG       ((volatile uint32_t *)0x4000044C)

// TIM4 Registers
#define TIM4_CR1_REG        ((volatile uint32_t *)0x40000800)
#define TIM4_CR2_REG        ((volatile uint32_t *)0x40000804)
#define TIM4_SMCR_REG       ((volatile uint32_t *)0x40000808)
#define TIM4_DIER_REG       ((volatile uint32_t *)0x4000080C)
#define TIM4_SR_REG         ((volatile uint32_t *)0x40000810)
#define TIM4_EGR_REG        ((volatile uint32_t *)0x40000814)
#define TIM4_CCMR1_REG      ((volatile uint32_t *)0x40000818)
#define TIM4_CCMR2_REG      ((volatile uint32_t *)0x4000081C)
#define TIM4_CCER_REG       ((volatile uint32_t *)0x40000820)
#define TIM4_CNT_REG        ((volatile uint32_t *)0x40000824)
#define TIM4_PSC_REG        ((volatile uint32_t *)0x40000828)
#define TIM4_ARR_REG        ((volatile uint32_t *)0x4000082C)
#define TIM4_CCR1_REG       ((volatile uint32_t *)0x40000834)
#define TIM4_CCR2_REG       ((volatile uint32_t *)0x40000838)
#define TIM4_CCR3_REG       ((volatile uint32_t *)0x4000083C)
#define TIM4_CCR4_REG       ((volatile uint32_t *)0x40000840)
#define TIM4_DCR_REG        ((volatile uint32_t *)0x40000848)
#define TIM4_DMAR_REG       ((volatile uint32_t *)0x4000084C)

// TIM5 Registers
#define TIM5_CR1_REG        ((volatile uint32_t *)0x40000C00)
#define TIM5_CR2_REG        ((volatile uint32_t *)0x40000C04)
#define TIM5_SMCR_REG       ((volatile uint32_t *)0x40000C08)
#define TIM5_DIER_REG       ((volatile uint32_t *)0x40000C0C)
#define TIM5_SR_REG         ((volatile uint32_t *)0x40000C10)
#define TIM5_EGR_REG        ((volatile uint32_t *)0x40000C14)
#define TIM5_CCMR1_REG      ((volatile uint32_t *)0x40000C18)
#define TIM5_CCMR2_REG      ((volatile uint32_t *)0x40000C1C)
#define TIM5_CCER_REG       ((volatile uint32_t *)0x40000C20)
#define TIM5_CNT_REG        ((volatile uint32_t *)0x40000C24)
#define TIM5_PSC_REG        ((volatile uint32_t *)0x40000C28)
#define TIM5_ARR_REG        ((volatile uint32_t *)0x40000C2C)
#define TIM5_CCR1_REG       ((volatile uint32_t *)0x40000C34)
#define TIM5_CCR2_REG       ((volatile uint32_t *)0x40000C38)
#define TIM5_CCR3_REG       ((volatile uint32_t *)0x40000C3C)
#define TIM5_CCR4_REG       ((volatile uint32_t *)0x40000C40)
#define TIM5_DCR_REG        ((volatile uint32_t *)0x40000C48)
#define TIM5_DMAR_REG       ((volatile uint32_t *)0x40000C4C)
#define TIM5_OR_REG         ((volatile uint32_t *)0x40000C50)

// TIM9 Registers
#define TIM9_CR1_REG        ((volatile uint32_t *)0x40014000)
#define TIM9_SMCR_REG       ((volatile uint32_t *)0x40014008)
#define TIM9_DIER_REG       ((volatile uint32_t *)0x4001400C)
#define TIM9_SR_REG         ((volatile uint32_t *)0x40014010)
#define TIM9_EGR_REG        ((volatile uint32_t *)0x40014014)
#define TIM9_CCMR1_REG      ((volatile uint32_t *)0x40014018)
#define TIM9_CCER_REG       ((volatile uint32_t *)0x40014020)
#define TIM9_CNT_REG        ((volatile uint32_t *)0x40014024)
#define TIM9_PSC_REG        ((volatile uint32_t *)0x40014028)
#define TIM9_ARR_REG        ((volatile uint32_t *)0x4001402C)
#define TIM9_CCR1_REG       ((volatile uint32_t *)0x40014034)
#define TIM9_CCR2_REG       ((volatile uint32_t *)0x40014038)

// TIM10 Registers
#define TIM10_CR1_REG       ((volatile uint32_t *)0x40014400)
#define TIM10_DIER_REG      ((volatile uint32_t *)0x4001440C)
#define TIM10_SR_REG        ((volatile uint32_t *)0x40014410)
#define TIM10_EGR_REG       ((volatile uint32_t *)0x40014414)
#define TIM10_CCMR1_REG     ((volatile uint32_t *)0x40014418)
#define TIM10_CCER_REG      ((volatile uint32_t *)0x40014420)
#define TIM10_CNT_REG       ((volatile uint32_t *)0x40014424)
#define TIM10_PSC_REG       ((volatile uint32_t *)0x40014428)
#define TIM10_ARR_REG       ((volatile uint32_t *)0x4001442C)
#define TIM10_CCR1_REG      ((volatile uint32_t *)0x40014434)

// TIM11 Registers
#define TIM11_CR1_REG       ((volatile uint32_t *)0x40014800)
#define TIM11_DIER_REG      ((volatile uint32_t *)0x4001480C)
#define TIM11_SR_REG        ((volatile uint32_t *)0x40014810)
#define TIM11_EGR_REG       ((volatile uint32_t *)0x40014814)
#define TIM11_CCMR1_REG     ((volatile uint32_t *)0x40014818)
#define TIM11_CCER_REG      ((volatile uint32_t *)0x40014820)
#define TIM11_CNT_REG       ((volatile uint32_t *)0x40014824)
#define TIM11_PSC_REG       ((volatile uint32_t *)0x40014828)
#define TIM11_ARR_REG       ((volatile uint32_t *)0x4001482C)
#define TIM11_CCR1_REG      ((volatile uint32_t *)0x40014834)

// USART Registers (USART1, USART2, USART6)
#define USART1_SR_REG       ((volatile uint32_t *)0x40011000)
#define USART1_DR_REG       ((volatile uint32_t *)0x40011004)
#define USART1_BRR_REG      ((volatile uint32_t *)0x40011008)
#define USART1_CR1_REG      ((volatile uint32_t *)0x4001100C)
#define USART1_CR2_REG      ((volatile uint32_t *)0x40011010)
#define USART1_CR3_REG      ((volatile uint32_t *)0x40011014)
#define USART1_GTPR_REG     ((volatile uint32_t *)0x40011018)

#define USART2_SR_REG       ((volatile uint32_t *)0x40004400)
#define USART2_DR_REG       ((volatile uint32_t *)0x40004404)
#define USART2_BRR_REG      ((volatile uint32_t *)0x40004408)
#define USART2_CR1_REG      ((volatile uint32_t *)0x4000440C)
#define USART2_CR2_REG      ((volatile uint32_t *)0x40004410)
#define USART2_CR3_REG      ((volatile uint32_t *)0x40004414)
#define USART2_GTPR_REG     ((volatile uint32_t *)0x40004418)

#define USART6_SR_REG       ((volatile uint32_t *)0x40011400)
#define USART6_DR_REG       ((volatile uint32_t *)0x40011404)
#define USART6_BRR_REG      ((volatile uint32_t *)0x40011408)
#define USART6_CR1_REG      ((volatile uint32_t *)0x4001140C)
#define USART6_CR2_REG      ((volatile uint32_t *)0x40011410)
#define USART6_CR3_REG      ((volatile uint32_t *)0x40011414)
#define USART6_GTPR_REG     ((volatile uint32_t *)0x40011418)

// I2C Registers (I2C1, I2C2, I2C3)
#define I2C1_CR1_REG        ((volatile uint32_t *)0x40005400)
#define I2C1_CR2_REG        ((volatile uint32_t *)0x40005404)
#define I2C1_OAR1_REG       ((volatile uint32_t *)0x40005408)
#define I2C1_OAR2_REG       ((volatile uint32_t *)0x4000540C)
#define I2C1_DR_REG         ((volatile uint32_t *)0x40005410)
#define I2C1_SR1_REG        ((volatile uint32_t *)0x40005414)
#define I2C1_SR2_REG        ((volatile uint32_t *)0x40005418)
#define I2C1_CCR_REG        ((volatile uint32_t *)0x4000541C)
#define I2C1_TRISE_REG      ((volatile uint32_t *)0x40005420)
#define I2C1_FLTR_REG       ((volatile uint32_t *)0x40005424)

#define I2C2_CR1_REG        ((volatile uint32_t *)0x40005800)
#define I2C2_CR2_REG        ((volatile uint32_t *)0x40005804)
#define I2C2_OAR1_REG       ((volatile uint32_t *)0x40005808)
#define I2C2_OAR2_REG       ((volatile uint32_t *)0x4000580C)
#define I2C2_DR_REG         ((volatile uint32_t *)0x40005810)
#define I2C2_SR1_REG        ((volatile uint32_t *)0x40005814)
#define I2C2_SR2_REG        ((volatile uint32_t *)0x40005818)
#define I2C2_CCR_REG        ((volatile uint32_t *)0x4000581C)
#define I2C2_TRISE_REG      ((volatile uint32_t *)0x40005820)
#define I2C2_FLTR_REG       ((volatile uint32_t *)0x40005824)

#define I2C3_CR1_REG        ((volatile uint32_t *)0x40005C00)
#define I2C3_CR2_REG        ((volatile uint32_t *)0x40005C04)
#define I2C3_OAR1_REG       ((volatile uint32_t *)0x40005C08)
#define I2C3_OAR2_REG       ((volatile uint32_t *)0x40005C0C)
#define I2C3_DR_REG         ((volatile uint32_t *)0x40005C10)
#define I2C3_SR1_REG        ((volatile uint32_t *)0x40005C14)
#define I2C3_SR2_REG        ((volatile uint32_t *)0x40005C18)
#define I2C3_CCR_REG        ((volatile uint32_t *)0x40005C1C)
#define I2C3_TRISE_REG      ((volatile uint32_t *)0x40005C20)
#define I2C3_FLTR_REG       ((volatile uint32_t *)0x40005C24)

// SPI Registers (SPI1, SPI2, SPI3)
#define SPI1_CR1_REG        ((volatile uint32_t *)0x40013000)
#define SPI1_CR2_REG        ((volatile uint32_t *)0x40013004)
#define SPI1_SR_REG         ((volatile uint32_t *)0x40013008)
#define SPI1_DR_REG         ((volatile uint32_t *)0x4001300C)
#define SPI1_CRCPR_REG      ((volatile uint32_t *)0x40013010)
#define SPI1_RXCRCR_REG     ((volatile uint32_t *)0x40013014)
#define SPI1_TXCRCR_REG     ((volatile uint32_t *)0x40013018)
#define SPI1_I2SCFGR_REG    ((volatile uint32_t *)0x4001301C)
#define SPI1_I2SPR_REG      ((volatile uint32_t *)0x40013020)

#define SPI2_CR1_REG        ((volatile uint32_t *)0x40003800)
#define SPI2_CR2_REG        ((volatile uint32_t *)0x40003804)
#define SPI2_SR_REG         ((volatile uint32_t *)0x40003808)
#define SPI2_DR_REG         ((volatile uint32_t *)0x4000380C)
#define SPI2_CRCPR_REG      ((volatile uint32_t *)0x40003810)
#define SPI2_RXCRCR_REG     ((volatile uint32_t *)0x40003814)
#define SPI2_TXCRCR_REG     ((volatile uint32_t *)0x40003818)
#define SPI2_I2SCFGR_REG    ((volatile uint32_t *)0x4000381C)
#define SPI2_I2SPR_REG      ((volatile uint32_t *)0x40003820)

#define SPI3_CR1_REG        ((volatile uint32_t *)0x40003C00)
#define SPI3_CR2_REG        ((volatile uint32_t *)0x40003C04)
#define SPI3_SR_REG         ((volatile uint32_t *)0x40003C08)
#define SPI3_DR_REG         ((volatile uint32_t *)0x40003C0C)
#define SPI3_CRCPR_REG      ((volatile uint32_t *)0x40003C10)
#define SPI3_RXCRCR_REG     ((volatile uint32_t *)0x40003C14)
#define SPI3_TXCRCR_REG     ((volatile uint32_t *)0x40003C18)
#define SPI3_I2SCFGR_REG    ((volatile uint32_t *)0x40003C1C)
#define SPI3_I2SPR_REG      ((volatile uint32_t *)0x40003C20)

// =============================================================================
// API Function Prototypes (from API.json)
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
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Return type fixed to void as per API.json convention (void if not specified, or a type like tbyte, tword, tlong, not a type name itself)

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
void UART_ClearFlag(t_uart_channel uart_channel); // Return type fixed to void

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
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Return type fixed to void

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
void SPI_ClearFlag(t_spi_channel spi_channel); // Return type fixed to void

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Return type fixed to void

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Return type fixed to tlong
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Return type fixed to void

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Return type fixed to void

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void); // Return type fixed to void

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