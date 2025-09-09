#ifndef MCAL_H
#define MCAL_H

// --- Core MCU Header and Standard C Libraries (Rules.json: core_includes) ---
#include "stm32f4xx.h"  // Core device header for STM32F4 series (placeholder)
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- Data Type Definitions (Rules.json: data_type_definitions) ---
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// Signed types for MQTT/HTTP (API.json)
typedef int8_t   tsbyte;
typedef int16_t  tsword;
typedef int32_t  tslong;


// --- Peripheral Base Addresses (from REGISTER_JSON) ---
#define FLASH_R_BASE            0x40023C00UL
#define RCC_R_BASE              0x40023800UL
#define SYSCFG_R_BASE           0x40013800UL
#define EXTI_R_BASE             0x40013C00UL
#define ADC1_R_BASE             0x40012000UL
#define ADC_COMMON_R_BASE       0x40012304UL // This is ADC_CCR, a common register
#define TIM1_R_BASE             0x40010000UL
#define TIM2_R_BASE             0x40000000UL
#define TIM3_R_BASE             0x40000400UL
#define TIM4_R_BASE             0x40000800UL
#define TIM5_R_BASE             0x40000C00UL
#define TIM9_R_BASE             0x40014000UL
#define TIM10_R_BASE            0x40014400UL
#define TIM11_R_BASE            0x40014800UL
#define USART1_R_BASE           0x40011000UL
#define USART2_R_BASE           0x40004400UL
#define USART6_R_BASE           0x40011400UL
#define I2C1_R_BASE             0x40005400UL
#define I2C2_R_BASE             0x40005800UL
#define I2C3_R_BASE             0x40005C00UL
#define SPI1_R_BASE             0x40013000UL
#define SPI2_R_BASE             0x40003800UL
#define SPI3_R_BASE             0x40003C00UL

// GPIO Port Base Addresses are grouped
#define GPIOA_R_BASE            0x40020000UL
#define GPIOB_R_BASE            0x40020400UL
#define GPIOC_R_BASE            0x40020800UL
#define GPIOD_R_BASE            0x40020C00UL
#define GPIOE_R_BASE            0x40021000UL
#define GPIOH_R_BASE            0x40021C00UL


// --- Peripheral Register Structure Definitions (from REGISTER_JSON) ---

// FLASH Registers
typedef struct
{
    volatile tlong ACR;       // 0x00 Flash access control register
    volatile tlong KEYR;      // 0x04 Flash key register
    volatile tlong OPTKEYR;   // 0x08 Flash option key register
    volatile tlong SR;        // 0x0C Flash status register
    volatile tlong CR;        // 0x10 Flash control register
    volatile tlong OPTCR;     // 0x14 Flash option control register
} FLASH_Registers_t;

// RCC Registers
typedef struct
{
    volatile tlong CR;        // 0x00 RCC clock control register
    volatile tlong PLLCFGR;   // 0x04 RCC PLL configuration register
    volatile tlong CFGR;      // 0x08 RCC clock configuration register
    volatile tlong CIR;       // 0x0C RCC clock interrupt register
    volatile tlong AHB1RSTR;  // 0x10 RCC AHB1 peripheral reset register
    volatile tlong AHB2RSTR;  // 0x14 RCC AHB2 peripheral reset register
    volatile tlong APB1RSTR;  // 0x18 RCC APB1 peripheral reset register
    volatile tlong APB2RSTR;  // 0x1C RCC APB2 peripheral reset register
    volatile tlong RESERVED0[2]; // Placeholder for alignment based on datasheet memory map
    volatile tlong AHB1ENR;   // 0x30 RCC AHB1 peripheral clock enable register
    volatile tlong AHB2ENR;   // 0x34 RCC AHB2 peripheral clock enable register
    volatile tlong APB1ENR;   // 0x38 RCC APB1 peripheral clock enable register
    volatile tlong APB2ENR;   // 0x3C RCC APB2 peripheral clock enable register
    volatile tlong AHB1LPENR; // 0x40 RCC AHB1 peripheral clock enable in low power mode register
    volatile tlong AHB2LPENR; // 0x44 RCC AHB2 peripheral clock enable in low power mode register
    volatile tlong APB1LPENR; // 0x48 RCC APB1 peripheral clock enable in low power mode register
    volatile tlong APB2LPENR; // 0x4C RCC APB2 peripheral clock enabled in low power mode register
    volatile tlong BDCR;      // 0x50 RCC Backup domain control register
    volatile tlong CSR;       // 0x54 RCC clock control & status register
    volatile tlong SSCGR;     // 0x58 RCC spread spectrum clock generation register
    volatile tlong PLLI2SCFGR;// 0x5C RCC PLLI2S configuration register
    volatile tlong RESERVED1; // Placeholder for alignment based on datasheet memory map
    volatile tlong DCKCFGR;   // 0x64 RCC Dedicated Clocks Configuration Register
} RCC_Registers_t;

// SYSCFG Registers
typedef struct
{
    volatile tlong MEMRMP;    // 0x00 SYSCFG memory remap register
    volatile tlong PMC;       // 0x04 SYSCFG peripheral mode configuration register
    volatile tlong EXTICR[4]; // 0x08-0x14 SYSCFG external interrupt configuration register 1-4
    volatile tlong RESERVED0[2]; // Placeholder for alignment based on datasheet memory map
    volatile tlong CMPCR;     // 0x20 Compensation cell control register
} SYSCFG_Registers_t;

// GPIO Registers (Generic structure, actual ports are instances of this)
typedef struct
{
    volatile tlong MODER;     // 0x00 GPIO port mode register
    volatile tlong OTYPER;    // 0x04 GPIO port output type register
    volatile tlong OSPEEDR;   // 0x08 GPIO port output speed register
    volatile tlong PUPDR;     // 0x0C GPIO port pull-up/pull-down register
    volatile tlong IDR;       // 0x10 GPIO port input data register
    volatile tlong ODR;       // 0x14 GPIO port output data register
    volatile tlong BSRR;      // 0x18 GPIO port bit set/reset register
    volatile tlong LCKR;      // 0x1C GPIO port configuration lock register
    volatile tlong AFRL;      // 0x20 GPIO alternate function low register
    volatile tlong AFRH;      // 0x24 GPIO alternate function high register
} GPIO_Registers_t;

// EXTI Registers
typedef struct
{
    volatile tlong IMR;       // 0x00 Interrupt mask register
    volatile tlong EMR;       // 0x04 Event mask register
    volatile tlong RTSR;      // 0x08 Rising trigger selection register
    volatile tlong FTSR;      // 0x0C Falling trigger selection register
    volatile tlong SWIER;     // 0x10 Software interrupt event register
    volatile tlong PR;        // 0x14 Pending register
} EXTI_Registers_t;

// ADC1 Registers
typedef struct
{
    volatile tlong SR;        // 0x00 ADC status register
    volatile tlong CR1;       // 0x04 ADC control register 1
    volatile tlong CR2;       // 0x08 ADC control register 2
    volatile tlong SMPR1;     // 0x0C ADC sample time register 1
    volatile tlong SMPR2;     // 0x10 ADC sample time register 2
    volatile tlong JOFR1;     // 0x14 ADC injected channel data offset register 1
    volatile tlong JOFR2;     // 0x18 ADC injected channel data offset register 2
    volatile tlong JOFR3;     // 0x1C ADC injected channel data offset register 3
    volatile tlong JOFR4;     // 0x20 ADC injected channel data offset register 4
    volatile tlong HTR;       // 0x24 ADC watchdog higher threshold register
    volatile tlong LTR;       // 0x28 ADC watchdog lower threshold register
    volatile tlong SQR1;      // 0x2C ADC regular sequence register 1
    volatile tlong SQR2;      // 0x30 ADC regular sequence register 2
    volatile tlong SQR3;      // 0x34 ADC regular sequence register 3
    volatile tlong JSQR;      // 0x38 ADC injected sequence register
    volatile tlong JDR1;      // 0x3C ADC injected data register 1
    volatile tlong JDR2;      // 0x40 ADC injected data register 2
    volatile tlong JDR3;      // 0x44 ADC injected data register 3
    volatile tlong JDR4;      // 0x48 ADC injected data register 4
    volatile tlong DR;        // 0x4C ADC regular data register
} ADC1_Registers_t;

// ADC Common Control Register (isolated as it's at a different base address)
typedef struct
{
    volatile tlong CCR;       // 0x00 ADC common control register
} ADC_COMMON_Registers_t;

// TIM (Timer) Registers (generic structure, some fields might not exist for all timers)
// Note: Some timers have more or fewer registers. This struct is a union of all possible registers.
// Accesses will need to be conditional or use specific TIMx_Registers_t if divergence is large.
// For F401RC: TIM1, TIM9, TIM10, TIM11 are advanced/general purpose, others are general purpose.
// The register JSON shows common registers across many timers, with some specifics for TIM1, TIM2, TIM5.
typedef struct
{
    volatile tlong CR1;       // 0x00 Control register 1
    volatile tlong CR2;       // 0x04 Control register 2
    volatile tlong SMCR;      // 0x08 Slave mode control register
    volatile tlong DIER;      // 0x0C DMA/Interrupt enable register
    volatile tlong SR;        // 0x10 Status register
    volatile tlong EGR;       // 0x14 Event generation register
    volatile tlong CCMR1;     // 0x18 Capture/compare mode register 1
    volatile tlong CCMR2;     // 0x1C Capture/compare mode register 2
    volatile tlong CCER;      // 0x20 Capture/compare enable register
    volatile tlong CNT;       // 0x24 Counter register
    volatile tlong PSC;       // 0x28 Prescaler register
    volatile tlong ARR;       // 0x2C Auto-reload register
    volatile tlong RCR;       // 0x30 Repetition counter register (only for TIM1)
    volatile tlong CCR1;      // 0x34 Capture/compare register 1
    volatile tlong CCR2;      // 0x38 Capture/compare register 2
    volatile tlong CCR3;      // 0x3C Capture/compare register 3
    volatile tlong CCR4;      // 0x40 Capture/compare register 4
    volatile tlong BDTR;      // 0x44 Break and dead-time register (only for TIM1)
    volatile tlong DCR;       // 0x48 DMA control register
    volatile tlong DMAR;      // 0x4C DMA address for full transfer
    volatile tlong OR;        // 0x50 Option register (TIM2 and TIM5 specific based on json)
} TIM_Registers_t;

// USART Registers
typedef struct
{
    volatile tlong SR;        // 0x00 Status register
    volatile tlong DR;        // 0x04 Data register
    volatile tlong BRR;       // 0x08 Baud rate register
    volatile tlong CR1;       // 0x0C Control register 1
    volatile tlong CR2;       // 0x10 Control register 2
    volatile tlong CR3;       // 0x14 Control register 3
    volatile tlong GTPR;      // 0x18 Guard time and prescaler register
} USART_Registers_t;

// I2C Registers
typedef struct
{
    volatile tlong CR1;       // 0x00 Control register 1
    volatile tlong CR2;       // 0x04 Control register 2
    volatile tlong OAR1;      // 0x08 Own address register 1
    volatile tlong OAR2;      // 0x0C Own address register 2
    volatile tlong DR;        // 0x10 Data register
    volatile tlong SR1;       // 0x14 Status register 1
    volatile tlong SR2;       // 0x18 Status register 2
    volatile tlong CCR;       // 0x1C Clock control register
    volatile tlong TRISE;     // 0x20 TRISE register
    volatile tlong FLTR;      // 0x24 Filter register
} I2C_Registers_t;

// SPI Registers (includes I2S config)
typedef struct
{
    volatile tlong CR1;       // 0x00 Control register 1
    volatile tlong CR2;       // 0x04 Control register 2
    volatile tlong SR;        // 0x08 Status register
    volatile tlong DR;        // 0x0C Data register
    volatile tlong CRCPR;     // 0x10 CRC polynomial register
    volatile tlong RXCRCR;    // 0x14 Rx CRC register
    volatile tlong TXCRCR;    // 0x18 Tx CRC register
    volatile tlong I2SCFGR;   // 0x1C I2S configuration register
    volatile tlong I2SPR;     // 0x20 I2S prescaler register
} SPI_Registers_t;


// --- Peripheral Instance Pointers ---
#define P_FLASH             ((FLASH_Registers_t *)FLASH_R_BASE)
#define P_RCC               ((RCC_Registers_t *)RCC_R_BASE)
#define P_SYSCFG            ((SYSCFG_Registers_t *)SYSCFG_R_BASE)
#define P_EXTI              ((EXTI_Registers_t *)EXTI_R_BASE)
#define P_ADC1              ((ADC1_Registers_t *)ADC1_R_BASE)
#define P_ADC_COMMON        ((ADC_COMMON_Registers_t *)ADC_COMMON_R_BASE) // This is just for CCR
#define P_TIM1              ((TIM_Registers_t *)TIM1_R_BASE)
#define P_TIM2              ((TIM_Registers_t *)TIM2_R_BASE)
#define P_TIM3              ((TIM_Registers_t *)TIM3_R_BASE)
#define P_TIM4              ((TIM_Registers_t *)TIM4_R_BASE)
#define P_TIM5              ((TIM_Registers_t *)TIM5_R_BASE)
#define P_TIM9              ((TIM_Registers_t *)TIM9_R_BASE)
#define P_TIM10             ((TIM_Registers_t *)TIM10_R_BASE)
#define P_TIM11             ((TIM_Registers_t *)TIM11_R_BASE)
#define P_USART1            ((USART_Registers_t *)USART1_R_BASE)
#define P_USART2            ((USART_Registers_t *)USART2_R_BASE)
#define P_USART6            ((USART_Registers_t *)USART6_R_BASE)
#define P_I2C1              ((I2C_Registers_t *)I2C1_R_BASE)
#define P_I2C2              ((I2C_Registers_t *)I2C2_R_BASE)
#define P_I2C3              ((I2C_Registers_t *)I2C3_R_BASE)
#define P_SPI1              ((SPI_Registers_t *)SPI1_R_BASE)
#define P_SPI2              ((SPI_Registers_t *)SPI2_R_BASE)
#define P_SPI3              ((SPI_Registers_t *)SPI3_R_BASE)

// GPIO Port Pointers
#define P_GPIOA             ((GPIO_Registers_t *)GPIOA_R_BASE)
#define P_GPIOB             ((GPIO_Registers_t *)GPIOB_R_BASE)
#define P_GPIOC             ((GPIO_Registers_t *)GPIOC_R_BASE)
#define P_GPIOD             ((GPIO_Registers_t *)GPIOD_R_BASE)
#define P_GPIOE             ((GPIO_Registers_t *)GPIOE_R_BASE)
#define P_GPIOH             ((GPIO_Registers_t *)GPIOH_R_BASE)


// --- Module-Specific Enum and Typedef Definitions (Rules.json: enum_naming, typedef_naming) ---

// MCU_CONFIG Module
typedef enum {
    sys_volt_3v = 0, // Assuming 0 for 3V, common for system voltage settings
    sys_volt_5v      // Assuming 1 for 5V
} t_sys_volt;

// LVD Module
typedef enum {
    lvd_threshold_level_0_5v = 0, // Placeholder values, actual mapping depends on MCU-specific LVD registers
    lvd_threshold_level_1v,
    lvd_threshold_level_1_5v,
    lvd_threshold_level_2v,
    lvd_threshold_level_2_5v,
    lvd_threshold_level_3v,
    lvd_threshold_level_3_5v,
    lvd_threshold_level_4v,
    lvd_threshold_level_4_5v,
    lvd_threshold_level_5v
} t_lvd_thrthreshold_level;

// UART Module
typedef enum {
    uart_channel_1 = 0, // Corresponds to USART1
    uart_channel_2,     // Corresponds to USART2
    uart_channel_6      // Corresponds to USART6
} t_uart_channel;

typedef enum {
    uart_baud_rate_9600 = 0,  // Example baud rates, actual register values vary
    uart_baud_rate_19200,
    uart_baud_rate_38400,
    uart_baud_rate_57600,
    uart_baud_rate_115200
} t_uart_baud_rate;

typedef enum {
    uart_data_length_8bit = 0,
    uart_data_length_9bit
} t_uart_data_length;

typedef enum {
    uart_stop_bit_1 = 0,
    uart_stop_bit_0_5, // Not typically common, but listed in some MCUs for specific modes
    uart_stop_bit_2,
    uart_stop_bit_1_5
} t_uart_stop_bit;

typedef enum {
    uart_parity_none = 0,
    uart_parity_even,
    uart_parity_odd
} t_uart_parity;

// I2C Module
typedef enum {
    i2c_channel_1 = 0,
    i2c_channel_2,
    i2c_channel_3
} t_i2c_channel;

typedef enum {
    i2c_clk_speed_standard = 0, // 100 kHz
    i2c_clk_speed_fast          // 400 kHz (Rules.json: Always use fast mode)
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // Device address is typically an 8-bit value

typedef enum {
    i2c_ack_disable = 0,
    i2c_ack_enable
} t_i2c_ack;

typedef enum {
    i2c_datalength_8bit = 0,
    i2c_datalength_16bit // STM32 I2C hardware data register is 8-bit, but 16-bit concept may be for internal buffers
} t_i2c_datalength;

// SPI (CSI) Module
typedef enum {
    spi_channel_1 = 0,
    spi_channel_2,
    spi_channel_3
} t_spi_channel;

typedef enum {
    spi_mode_master = 0,
    spi_mode_slave
} t_spi_mode;

typedef enum {
    spi_cpol_low = 0,  // Clock Polarity Low
    spi_cpol_high      // Clock Polarity High
} t_spi_cpol;

typedef enum {
    spi_cpha_1edge = 0, // Clock Phase 1st edge
    spi_cpha_2edge      // Clock Phase 2nd edge
} t_spi_cpha;

typedef enum {
    spi_dff_8bit = 0,
    spi_dff_16bit
} t_spi_dff;

typedef enum {
    spi_bit_order_msb_first = 0,
    spi_bit_order_lsb_first
} t_spi_bit_order;

// External Interrupt Module
typedef enum {
    exti_line_0 = 0,
    exti_line_1,
    exti_line_2,
    exti_line_3,
    exti_line_4,
    exti_line_5,
    exti_line_6,
    exti_line_7,
    exti_line_8,
    exti_line_9,
    exti_line_10,
    exti_line_11,
    exti_line_12,
    exti_line_13,
    exti_line_14,
    exti_line_15
} t_external_int_channel; // Maps to EXTI_IMR/EMR bit position for individual lines

typedef enum {
    exti_edge_rising = 0,
    exti_edge_falling,
    exti_edge_rising_falling // Both edges
} t_external_int_edge;

// GPIO Module
typedef enum {
    port_a = 0,
    port_b,
    port_c,
    port_d,
    port_e,
    port_h
} t_port;

typedef enum {
    pin_0 = 0,
    pin_1,
    pin_2,
    pin_3,
    pin_4,
    pin_5,
    pin_6,
    pin_7,
    pin_8,
    pin_9,
    pin_10,
    pin_11,
    pin_12,
    pin_13,
    pin_14,
    pin_15
} t_pin;

typedef enum {
    direction_input = 0,
    direction_output,
    direction_analog,
    direction_alternate_function
} t_direction; // For GPIO_Direction_get return type and internal config

// PWM Module
typedef enum {
    // TIM1 Channels
    pwm_channel_tim1_ch1 = 0, // PA8, PE9 - (TIM1_CCMR1, TIM1_CCER, TIM1_CCR1)
    pwm_channel_tim1_ch2,     // PA9, PE11 - (TIM1_CCMR1, TIM1_CCER, TIM1_CCR2)
    pwm_channel_tim1_ch3,     // PA10, PE13 - (TIM1_CCMR2, TIM1_CCER, TIM1_CCR3)
    pwm_channel_tim1_ch4,     // PA11, PE14 - (TIM1_CCMR2, TIM1_CCER, TIM1_CCR4)
    // TIM2 Channels
    pwm_channel_tim2_ch1,     // PA0, PA5, PA15, PB3 - (TIM2_CCMR1, TIM2_CCER, TIM2_CCR1)
    pwm_channel_tim2_ch2,     // PA1, PB3, PB10 - (TIM2_CCMR1, TIM2_CCER, TIM2_CCR2)
    pwm_channel_tim2_ch3,     // PA2, PB10 - (TIM2_CCMR2, TIM2_CCER, TIM2_CCR3)
    pwm_channel_tim2_ch4,     // PA3, PB11 - (TIM2_CCMR2, TIM2_CCER, TIM2_CCR4)
    // TIM3 Channels
    pwm_channel_tim3_ch1,     // PA6, PB4, PC6 - (TIM3_CCMR1, TIM3_CCER, TIM3_CCR1)
    pwm_channel_tim3_ch2,     // PA7, PB5, PC7 - (TIM3_CCMR1, TIM3_CCER, TIM3_CCR2)
    pwm_channel_tim3_ch3,     // PB0, PC8 - (TIM3_CCMR2, TIM3_CCER, TIM3_CCR3)
    pwm_channel_tim3_ch4,     // PB1, PC9 - (TIM3_CCMR2, TIM3_CCER, TIM3_CCR4)
    // TIM4 Channels
    pwm_channel_tim4_ch1,     // PB6 - (TIM4_CCMR1, TIM4_CCER, TIM4_CCR1)
    pwm_channel_tim4_ch2,     // PB7 - (TIM4_CCMR1, TIM4_CCER, TIM4_CCR2)
    pwm_channel_tim4_ch3,     // PB8 - (TIM4_CCMR2, TIM4_CCER, TIM4_CCR3)
    pwm_channel_tim4_ch4,     // PB9 - (TIM4_CCMR2, TIM4_CCER, TIM4_CCR4)
    // TIM5 Channels
    pwm_channel_tim5_ch1,     // PA0 - (TIM5_CCMR1, TIM5_CCER, TIM5_CCR1)
    pwm_channel_tim5_ch2,     // PA1 - (TIM5_CCMR1, TIM5_CCER, TIM5_CCR2)
    pwm_channel_tim5_ch3,     // PA2 - (TIM5_CCMR2, TIM5_CCER, TIM5_CCR3)
    pwm_channel_tim5_ch4,     // PA3 - (TIM5_CCMR2, TIM5_CCER, TIM5_CCR4)
    // TIM9 Channels
    pwm_channel_tim9_ch1,     // PA2, PE5 - (TIM9_CCMR1, TIM9_CCER, TIM9_CCR1)
    pwm_channel_tim9_ch2,     // PA3, PE6 - (TIM9_CCMR1, TIM9_CCER, TIM9_CCR2)
    // TIM10 Channels
    pwm_channel_tim10_ch1,    // PB8, PA6 - (TIM10_CCMR1, TIM10_CCER, TIM10_CCR1)
    // TIM11 Channels
    pwm_channel_tim11_ch1     // PB9, PA7 - (TIM11_CCMR1, TIM11_CCER, TIM11_CCR1)
} t_pwm_channel;

// ICU Module
typedef t_pwm_channel t_icu_channel; // ICU uses timer channels for input capture, so reusing the enum.

typedef enum {
    icu_prescaler_div1 = 0, // Placeholder values for prescaler configuration
    icu_prescaler_div2,
    icu_prescaler_div4,
    icu_prescaler_div8
} t_icu_prescaller;

typedef enum {
    icu_edge_rising = 0,
    icu_edge_falling,
    icu_edge_both
} t_icu_edge;

// Timer Module
typedef enum {
    timer_channel_1 = 0, // Corresponds to TIM1
    timer_channel_2,     // Corresponds to TIM2
    timer_channel_3,     // Corresponds to TIM3
    timer_channel_4,     // Corresponds to TIM4
    timer_channel_5,     // Corresponds to TIM5
    timer_channel_9,     // Corresponds to TIM9
    timer_channel_10,    // Corresponds to TIM10
    timer_channel_11     // Corresponds to TIM11
} t_timer_channel;

// ADC Module
typedef enum {
    adc_channel_0 = 0, // PA0 (ADC1_IN0)
    adc_channel_1,     // PA1 (ADC1_IN1)
    adc_channel_2,     // PA2 (ADC1_IN2)
    adc_channel_3,     // PA3 (ADC1_IN3)
    adc_channel_4,     // PA4 (ADC1_IN4)
    adc_channel_5,     // PA5 (ADC1_IN5)
    adc_channel_6,     // PA6 (ADC1_IN6)
    adc_channel_7,     // PA7 (ADC1_IN7)
    adc_channel_8,     // PB0 (ADC1_IN8)
    adc_channel_9,     // PB1 (ADC1_IN9)
    adc_channel_10,    // PC0 (ADC1_IN10)
    adc_channel_11,    // PC1 (ADC1_IN11)
    adc_channel_12,    // PC2 (ADC1_IN12)
    adc_channel_13,    // PC3 (ADC1_IN13)
    adc_channel_14,    // PC4 (ADC1_IN14)
    adc_channel_15     // PC5 (ADC1_IN15)
} t_adc_channel; // Channels explicitly listed in SMPR1/SMPR2 assigned_pin for ADC1

typedef enum {
    adc_mode_single_conversion = 0,
    adc_mode_continuous_conversion
} t_adc_mode_t;

// TT Module
typedef tword t_tick_time; // milliseconds

// MCAL_OUTPUT_BUZZER Module - Not supported on this MCU.
// DAC Module - Not supported on this MCU.

// I2S Module (Supported by SPI peripheral in STM32F401RC)
typedef enum {
    i2s_channel_spi1 = 0, // I2S functionality integrated into SPI1
    i2s_channel_spi2,     // I2S functionality integrated into SPI2
    i2s_channel_spi3      // I2S functionality integrated into SPI3
} t_i2s_channel;

typedef enum {
    i2s_mode_master_tx = 0,
    i2s_mode_master_rx,
    i2s_mode_slave_tx,
    i2s_mode_slave_rx
} I2S_Mode_t;

typedef enum {
    i2s_standard_philips = 0,
    i2s_standard_msb,
    i2s_standard_lsb,
    i2s_standard_pcm_short,
    i2s_standard_pcm_long
} I2S_Standard_t;

typedef enum {
    i2s_data_format_16b = 0,
    i2s_data_format_16b_extended,
    i2s_data_format_24b,
    i2s_data_format_32b
} I2S_DataFormat_t;

typedef enum {
    i2s_channel_mode_stereo = 0,
    i2s_channel_mode_mono
} I2S_ChannelMode_t;

// MQTT Protocol Module - Not supported on this MCU.
// HTTP Protocol Module - Not supported on this MCU.
// WiFi Driver Module - Not supported on this MCU.
// DTC_driver Module - Not supported on this MCU.


// --- API Function Prototypes (from API.json) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // This is also defined in WDT module, but included here as a core MCU function.
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthreshold_level lvd_threshold_level);
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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Assuming frequency return type is long for now.
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

// WDT
void WDT_Init(void);
// WDT_Reset prototype already declared in MCU_CONFIG section, no re-declaration needed.

// I2S (Supported on this MCU, using SPI peripheral for implementation)
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// MCAL_OUTPUT_BUZZER not supported on this MCU.
// DAC not supported on this MCU.
// MQTT Protocol not supported on this MCU.
// HTTP Protocol not supported on this MCU.
// WiFi Driver not supported on this MCU.
// DTC_driver not supported on this MCU.


#endif // MCAL_H