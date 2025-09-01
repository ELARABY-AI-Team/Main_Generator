/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file provides the declarations for the Microcontroller Abstraction Layer (MCAL)
 * functions, data types, and register definitions for the STM32F401RC microcontroller.
 * It abstracts the low-level hardware access, providing a standardized interface
 * for higher-layer software components.
 *
 * All register definitions, API prototypes, and data types are generated based on
 * the provided MCU register JSON and API specifications, adhering to the defined
 * coding rules and constraints.
 */

#ifndef MCAL_H
#define MCAL_H

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h> // Potentially for string-related API functions
#include <stdio.h>  // Potentially for printing/debugging related API functions
#include <stdlib.h> // Potentially for memory allocation or utility functions
#include <math.h>   // Potentially for math operations if any peripheral needs them

/*==================================================================================================
*                                      VERSION INFO
==================================================================================================*/
#define MCAL_VENDOR_ID                     (1U)
#define MCAL_MODULE_ID                     (1U)
#define MCAL_SW_MAJOR_VERSION              (1U)
#define MCAL_SW_MINOR_VERSION              (0U)
#define MCAL_SW_PATCH_VERSION              (0U)

/*==================================================================================================
*                                    FILE VERSION CHECKS
==================================================================================================*/
/* No external file version checks required for this simple MCAL header. */

/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/
// System Core Clock
#define F_CPU_MHZ                           16U // Default HSI clock frequency in MHz

/*==================================================================================================
*                                          MACROS
==================================================================================================*/
// Macro for casting an address to a volatile 32-bit unsigned integer pointer
#define REG_ACCESS(address)                 (*((volatile uint32_t *)(address)))

/*==================================================================================================
*                                        DATA TYPES
==================================================================================================*/
// Data type definitions as per Rules.json
#define Unit_8                              uint8_t
#define unit_16                             uint16_t
#define unit_32                             uint32_t

// Alias for standard integer types
typedef uint8_t                             tbyte;
typedef uint16_t                            tword;
typedef uint32_t                            tlong;
typedef int8_t                              tsbyte;
typedef int16_t                             tsword;

// MCU_Config_Init parameter
typedef enum
{
    SYS_VOLT_3V3 = 0,
    SYS_VOLT_5V
} t_sys_volt;

// LVD_Get parameter
typedef enum
{
    LVD_THRESHOLD_0_5V,
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1_5V,
    LVD_THRESHOLD_2V,
    LVD_THRESHOLD_2_5V,
    LVD_THRESHOLD_3V,
    LVD_THRESHOLD_3_5V,
    LVD_THRESHOLD_4V,
    LVD_THRESHOLD_4_5V,
    LVD_THRESHOLD_5V
} t_lvd_thrthresholdLevel; // Typo from API.json retained

// UART Channel
typedef enum
{
    UART_CHANNEL_1 = 0, // Assigned to USART1
    UART_CHANNEL_2,     // Assigned to USART2
    UART_CHANNEL_6      // Assigned to USART6
} t_uart_channel;

// UART Baud Rate (common values, actual implementation might map to register values)
typedef enum
{
    UART_BAUD_RATE_9600,
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_38400,
    UART_BAUD_RATE_57600,
    UART_BAUD_RATE_115200
} t_uart_baud_rate;

// UART Data Length
typedef enum
{
    UART_DATA_LENGTH_8B,
    UART_DATA_LENGTH_9B
} t_uart_data_length;

// UART Stop Bit
typedef enum
{
    UART_STOP_BIT_1,
    UART_STOP_BIT_0_5,
    UART_STOP_BIT_2,
    UART_STOP_BIT_1_5
} t_uart_stop_bit;

// UART Parity
typedef enum
{
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// I2C Channel
typedef enum
{
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

// I2C Clock Speed (example values, actual implementation maps to register values)
typedef enum
{
    I2C_CLK_SPEED_STANDARD_MODE_100KHZ, // 100 kHz
    I2C_CLK_SPEED_FAST_MODE_400KHZ      // 400 kHz (as per rules: always use fast mode)
} t_i2c_clk_speed;

// I2C Device Address (7-bit address)
typedef tbyte t_i2c_device_address; // 7-bit address value

// I2C Acknowledge
typedef enum
{
    I2C_ACK_DISABLE,
    I2C_ACK_ENABLE
} t_i2c_ack;

// I2C Data Length (not directly applicable to I2C protocol, might refer to word size or buffer)
typedef tbyte t_i2c_datalength;

// SPI Channel
typedef enum
{
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

// SPI Mode (Master/Slave)
typedef enum
{
    SPI_MODE_SLAVE,
    SPI_MODE_MASTER
} t_spi_mode;

// SPI Clock Polarity (CPOL)
typedef enum
{
    SPI_CPOL_LOW,  // Clock to 0 when idle
    SPI_CPOL_HIGH  // Clock to 1 when idle
} t_spi_cpol;

// SPI Clock Phase (CPHA)
typedef enum
{
    SPI_CPHA_1_EDGE, // Data sampled on first clock edge
    SPI_CPHA_2_EDGE  // Data sampled on second clock edge
} t_spi_cpha;

// SPI Data Frame Format (DFF)
typedef enum
{
    SPI_DFF_8BIT,
    SPI_DFF_16BIT
} t_spi_dff;

// SPI Bit Order (MSB/LSB first)
typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt Channel
typedef enum
{
    EXT_INT_CHANNEL_0 = 0,
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

// External Interrupt Edge
typedef enum
{
    EXT_INT_EDGE_RISING,
    EXT_INT_EDGE_FALLING,
    EXT_INT_EDGE_BOTH
} t_external_int_edge;

// GPIO Port
typedef enum
{
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    // PORT_F, // Not present on STM32F401RC (compact package)
    // PORT_G, // Not present on STM32F401RC (compact package)
    PORT_H,
    // PORT_I // Not present on STM32F401RC (compact package)
} t_port;

// GPIO Pin
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

// GPIO Direction
typedef enum
{
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;

// PWM Channel (mapped to TIMx_CHy)
typedef enum
{
    PWM_CHANNEL_TIM1_CH1,  // PA8,PE9,PB13,PA7
    PWM_CHANNEL_TIM1_CH2,  // PA9,PE11,PB0,PB14
    PWM_CHANNEL_TIM1_CH3,  // PA10,PE13,PB1,PB15
    PWM_CHANNEL_TIM1_CH4,  // PA11,PE14

    PWM_CHANNEL_TIM2_CH1,  // PA0,PA5,PA15,PB3
    PWM_CHANNEL_TIM2_CH2,  // PA1,PB3,PB10
    PWM_CHANNEL_TIM2_CH3,  // PA2,PB10
    PWM_CHANNEL_TIM2_CH4,  // PA3,PB11

    PWM_CHANNEL_TIM3_CH1,  // PA6,PB4,PC6
    PWM_CHANNEL_TIM3_CH2,  // PA7,PB5,PC7
    PWM_CHANNEL_TIM3_CH3,  // PB0,PC8
    PWM_CHANNEL_TIM3_CH4,  // PB1,PC9

    PWM_CHANNEL_TIM4_CH1,  // PB6
    PWM_CHANNEL_TIM4_CH2,  // PB7
    PWM_CHANNEL_TIM4_CH3,  // PB8
    PWM_CHANNEL_TIM4_CH4,  // PB9

    PWM_CHANNEL_TIM5_CH1,  // PA0
    PWM_CHANNEL_TIM5_CH2,  // PA1
    PWM_CHANNEL_TIM5_CH3,  // PA2
    PWM_CHANNEL_TIM5_CH4,  // PA3

    PWM_CHANNEL_TIM9_CH1,  // PA2,PE5
    PWM_CHANNEL_TIM9_CH2,  // PA3,PE6

    PWM_CHANNEL_TIM10_CH1, // PB8,PA6

    PWM_CHANNEL_TIM11_CH1  // PB9,PA7
} t_pwm_channel;

// ICU Channel (similar mapping as PWM, often using the same timer channels)
typedef enum
{
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

// ICU Prescaler (example values)
typedef enum
{
    ICU_PRESCALER_DIV1,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8,
    ICU_PRESCALER_DIV16,
    ICU_PRESCALER_DIV32,
    ICU_PRESCALER_DIV64,
    ICU_PRESCALER_DIV128,
    ICU_PRESCALER_DIV256
} t_icu_prescaller; // Typo from API.json retained

// ICU Edge
typedef enum
{
    ICU_EDGE_RISING,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH_EDGES
} t_icu_edge;

// Timer Channel
typedef enum
{
    TIMER_CHANNEL_1 = 0, // TIM1
    TIMER_CHANNEL_2,     // TIM2
    TIMER_CHANNEL_3,     // TIM3
    TIMER_CHANNEL_4,     // TIM4
    TIMER_CHANNEL_5,     // TIM5
    TIMER_CHANNEL_9,     // TIM9
    TIMER_CHANNEL_10,    // TIM10
    TIMER_CHANNEL_11     // TIM11
} t_timer_channel;

// ADC Channel
typedef enum
{
    ADC_CHANNEL_0 = 0,  // PA0
    ADC_CHANNEL_1,      // PA1
    ADC_CHANNEL_2,      // PA2
    ADC_CHANNEL_3,      // PA3
    ADC_CHANNEL_4,      // PA4
    ADC_CHANNEL_5,      // PA5
    ADC_CHANNEL_6,      // PA6
    ADC_CHANNEL_7,      // PA7
    ADC_CHANNEL_8,      // PB0
    ADC_CHANNEL_9,      // PB1
    ADC_CHANNEL_10,     // PC0
    ADC_CHANNEL_11,     // PC1
    ADC_CHANNEL_12,     // PC2
    ADC_CHANNEL_13,     // PC3
    ADC_CHANNEL_14,     // PC4
    ADC_CHANNEL_15      // PC5
    // Add internal channels like Temperature Sensor, VrefInt if applicable
} t_adc_channel;

// ADC Mode
typedef enum
{
    ADC_MODE_POLLING,
    ADC_MODE_INTERRUPT,
    ADC_MODE_DMA
} t_adc_mode_t;

// Tick Time for Time Triggered OS
typedef tword t_tick_time; // in ms

// DAC Channel (not supported, placeholder for future expansion if registers are added)
typedef enum
{
    DAC_CHANNEL_1 = 0, // Placeholder
    DAC_CHANNEL_2      // Placeholder
} dac_channel_t;

// I2S Channel
typedef enum
{
    I2S_CHANNEL_1 = 0, // SPI1 acting as I2S
    I2S_CHANNEL_2,     // SPI2 acting as I2S
    I2S_CHANNEL_3      // SPI3 acting as I2S
} t_i2s_channel;

// I2S Mode
typedef enum
{
    I2S_MODE_SLAVE_TX,
    I2S_MODE_SLAVE_RX,
    I2S_MODE_MASTER_TX,
    I2S_MODE_MASTER_RX
} I2S_Mode_t;

// I2S Standard
typedef enum
{
    I2S_STANDARD_PHILIPS,
    I2S_STANDARD_MSB,
    I2S_STANDARD_LSB,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

// I2S Data Format
typedef enum
{
    I2S_DATA_FORMAT_16B,
    I2S_DATA_FORMAT_24B,
    I2S_DATA_FORMAT_32B
} I2S_DataFormat_t;

// I2S Channel Mode
typedef enum
{
    I2S_CHANNEL_MODE_STEREO,
    I2S_CHANNEL_MODE_MONO
} I2S_ChannelMode_t;

// WiFi Transmit Mode (placeholder, if WiFi module were supported)
typedef enum
{
    WIFI_TX_MODE_DEFAULT,
    WIFI_TX_MODE_CUSTOM
} t_tx_mode;

/*==================================================================================================
*                                 GENERAL PERIPHERAL REGISTER DEFINITIONS
==================================================================================================*/

/**
 * @brief Flash Access Control Register (FLASH_ACR)
 * Base Address: 0x40023C00
 */
#define FLASH_ACR_ADDRESS       (0x40023C00UL)
#define FLASH_KEYR_ADDRESS      (0x40023C04UL)
#define FLASH_OPTKEYR_ADDRESS   (0x40023C08UL)
#define FLASH_SR_ADDRESS        (0x40023C0CUL)
#define FLASH_CR_ADDRESS        (0x40023C10UL)
#define FLASH_OPTCR_ADDRESS     (0x40023C14UL)

typedef struct
{
  volatile uint32_t ACR;         /*!< Flash access control register, Address offset: 0x00 */
  volatile uint32_t KEYR;        /*!< Flash key register,             Address offset: 0x04 */
  volatile uint32_t OPTKEYR;     /*!< Flash option key register,      Address offset: 0x08 */
  volatile uint32_t SR;          /*!< Flash status register,          Address offset: 0x0C */
  volatile uint32_t CR;          /*!< Flash control register,         Address offset: 0x10 */
  volatile uint32_t OPTCR;       /*!< Flash option control register,  Address offset: 0x14 */
} FLASH_TypeDef;

#define FLASH_BASE              (0x40023C00UL)
#define FLASH                   ((FLASH_TypeDef *) FLASH_BASE)

/**
 * @brief Reset and Clock Control (RCC)
 * Base Address: 0x40023800
 */
#define RCC_CR_ADDRESS          (0x40023800UL)
#define RCC_PLLCFGR_ADDRESS     (0x40023804UL)
#define RCC_CFGR_ADDRESS        (0x40023808UL)
#define RCC_CIR_ADDRESS         (0x4002380CUL)
#define RCC_AHB1RSTR_ADDRESS    (0x40023810UL)
#define RCC_AHB2RSTR_ADDRESS    (0x40023814UL)
#define RCC_APB1RSTR_ADDRESS    (0x40023818UL)
#define RCC_APB2RSTR_ADDRESS    (0x4002381CUL)
// ... (other RCC registers)
#define RCC_AHB1ENR_ADDRESS     (0x40023830UL)
#define RCC_AHB2ENR_ADDRESS     (0x40023834UL)
#define RCC_APB1ENR_ADDRESS     (0x40023838UL)
#define RCC_APB2ENR_ADDRESS     (0x4002383CUL)
// ... (other RCC registers)
#define RCC_BDCR_ADDRESS        (0x40023850UL)
#define RCC_CSR_ADDRESS         (0x40023854UL)
// ... (more RCC registers)

typedef struct
{
  volatile uint32_t CR;          /*!< RCC clock control register,                                  Address offset: 0x00 */
  volatile uint32_t PLLCFGR;     /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  volatile uint32_t CFGR;        /*!< RCC clock configuration register,                            Address offset: 0x08 */
  volatile uint32_t CIR;         /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  volatile uint32_t APB1RSTR;    /*!< RCC APB1 peripheral reset register,                          Address offset: 0x18 */
  volatile uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,                          Address offset: 0x1C */
  uint32_t      RESERVED0[2];  /*!< Reserved, 0x20-0x28 */
  volatile uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clock enable register,                   Address offset: 0x30 */
  volatile uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clock enable register,                   Address offset: 0x34 */
  volatile uint32_t APB1ENR;     /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x38 */
  volatile uint32_t APB2ENR;     /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x3C */
  volatile uint32_t AHB1LPENR;   /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x40 */
  volatile uint32_t AHB2LPENR;   /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x44 */
  volatile uint32_t APB1LPENR;   /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x48 */
  volatile uint32_t APB2LPENR;   /*!< RCC APB2 peripheral clock enabled in low power mode register,Address offset: 0x4C */
  volatile uint32_t BDCR;        /*!< RCC Backup domain control register,                          Address offset: 0x50 */
  volatile uint32_t CSR;         /*!< RCC clock control & status register,                         Address offset: 0x54 */
  volatile uint32_t SSCGR;       /*!< RCC spread spectrum clock generation register,               Address offset: 0x58 */
  volatile uint32_t PLLI2SCFGR;  /*!< RCC PLLI2S configuration register,                           Address offset: 0x5C */
  uint32_t      RESERVED1;     /*!< Reserved, 0x60 */
  volatile uint32_t DCKCFGR;     /*!< RCC Dedicated Clocks Configuration Register,                 Address offset: 0x64 */
} RCC_TypeDef;

#define RCC_BASE                (0x40023800UL)
#define RCC                     ((RCC_TypeDef *) RCC_BASE)

// RCC Bit Definitions (inferred for common peripherals based on STM32F401RC RM)
#define RCC_AHB1ENR_GPIOAEN_Pos     (0U)
#define RCC_AHB1ENR_GPIOBEN_Pos     (1U)
#define RCC_AHB1ENR_GPIOCEN_Pos     (2U)
#define RCC_AHB1ENR_GPIODEN_Pos     (3U)
#define RCC_AHB1ENR_GPIOEEN_Pos     (4U)
#define RCC_AHB1ENR_GPIOHEN_Pos     (7U)
#define RCC_AHB1ENR_GPIOAEN         (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOBEN         (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos)
#define RCC_AHB1ENR_GPIOCEN         (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos)
#define RCC_AHB1ENR_GPIODEN         (0x1UL << RCC_AHB1ENR_GPIODEN_Pos)
#define RCC_AHB1ENR_GPIOEEN         (0x1UL << RCC_AHB1ENR_GPIOEEN_Pos)
#define RCC_AHB1ENR_GPIOHEN         (0x1UL << RCC_AHB1ENR_GPIOHEN_Pos) // Inferred

#define RCC_APB1ENR_TIM2EN_Pos      (0U)
#define RCC_APB1ENR_TIM3EN_Pos      (1U)
#define RCC_APB1ENR_TIM4EN_Pos      (2U)
#define RCC_APB1ENR_TIM5EN_Pos      (3U)
#define RCC_APB1ENR_SPI2EN_Pos      (14U)
#define RCC_APB1ENR_SPI3EN_Pos      (15U)
#define RCC_APB1ENR_USART2EN_Pos    (17U)
#define RCC_APB1ENR_I2C1EN_Pos      (21U)
#define RCC_APB1ENR_I2C2EN_Pos      (22U)
#define RCC_APB1ENR_I2C3EN_Pos      (23U)

#define RCC_APB1ENR_TIM2EN          (0x1UL << RCC_APB1ENR_TIM2EN_Pos)
#define RCC_APB1ENR_TIM3EN          (0x1UL << RCC_APB1ENR_TIM3EN_Pos)
#define RCC_APB1ENR_TIM4EN          (0x1UL << RCC_APB1ENR_TIM4EN_Pos)
#define RCC_APB1ENR_TIM5EN          (0x1UL << RCC_APB1ENR_TIM5EN_Pos)
#define RCC_APB1ENR_SPI2EN          (0x1UL << RCC_APB1ENR_SPI2EN_Pos)
#define RCC_APB1ENR_SPI3EN          (0x1UL << RCC_APB1ENR_SPI3EN_Pos)
#define RCC_APB1ENR_USART2EN        (0x1UL << RCC_APB1ENR_USART2EN_Pos)
#define RCC_APB1ENR_I2C1EN          (0x1UL << RCC_APB1ENR_I2C1EN_Pos)
#define RCC_APB1ENR_I2C2EN          (0x1UL << RCC_APB1ENR_I2C2EN_Pos)
#define RCC_APB1ENR_I2C3EN          (0x1UL << RCC_APB1ENR_I2C3EN_Pos)

#define RCC_APB2ENR_TIM1EN_Pos      (0U)
#define RCC_APB2ENR_USART1EN_Pos    (4U)
#define RCC_APB2ENR_USART6EN_Pos    (5U)
#define RCC_APB2ENR_ADC1EN_Pos      (8U)
#define RCC_APB2ENR_SYSCFGEN_Pos    (14U)
#define RCC_APB2ENR_TIM9EN_Pos      (16U)
#define RCC_APB2ENR_TIM10EN_Pos     (17U)
#define RCC_APB2ENR_TIM11EN_Pos     (18U)
#define RCC_APB2ENR_SPI1EN_Pos      (12U) // Inferred, typically SPI1 is on APB2

#define RCC_APB2ENR_TIM1EN          (0x1UL << RCC_APB2ENR_TIM1EN_Pos)
#define RCC_APB2ENR_USART1EN        (0x1UL << RCC_APB2ENR_USART1EN_Pos)
#define RCC_APB2ENR_USART6EN        (0x1UL << RCC_APB2ENR_USART6EN_Pos)
#define RCC_APB2ENR_ADC1EN          (0x1UL << RCC_APB2ENR_ADC1EN_Pos)
#define RCC_APB2ENR_SYSCFGEN        (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)
#define RCC_APB2ENR_TIM9EN          (0x1UL << RCC_APB2ENR_TIM9EN_Pos)
#define RCC_APB2ENR_TIM10EN         (0x1UL << RCC_APB2ENR_TIM10EN_Pos)
#define RCC_APB2ENR_TIM11EN         (0x1UL << RCC_APB2ENR_TIM11EN_Pos)
#define RCC_APB2ENR_SPI1EN          (0x1UL << RCC_APB2ENR_SPI1EN_Pos)


/**
 * @brief System configuration controller (SYSCFG)
 * Base Address: 0x40013800
 */
#define SYSCFG_MEMRMP_ADDRESS   (0x40013800UL)
#define SYSCFG_PMC_ADDRESS      (0x40013804UL)
#define SYSCFG_EXTICR1_ADDRESS  (0x40013808UL)
#define SYSCFG_EXTICR2_ADDRESS  (0x4001380CUL)
#define SYSCFG_EXTICR3_ADDRESS  (0x40013810UL)
#define SYSCFG_EXTICR4_ADDRESS  (0x40013814UL)
#define SYSCFG_CMPCR_ADDRESS    (0x40013820UL)

typedef struct
{
  volatile uint32_t MEMRMP;    /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  volatile uint32_t PMC;       /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  volatile uint32_t EXTICR[4]; /*!< SYSCFG external interrupt configuration register 1-4, Address offset: 0x08-0x14 */
  uint32_t      RESERVED0[2];  /*!< Reserved, 0x18-0x1C                                */
  volatile uint32_t CMPCR;     /*!< Compensation cell control register,                Address offset: 0x20      */
} SYSCFG_TypeDef;

#define SYSCFG_BASE             (0x40013800UL)
#define SYSCFG                  ((SYSCFG_TypeDef *) SYSCFG_BASE)

/**
 * @brief External Interrupt/Event Controller (EXTI)
 * Base Address: 0x40013C00
 */
#define EXTI_IMR_ADDRESS    (0x40013C00UL)
#define EXTI_EMR_ADDRESS    (0x40013C04UL)
#define EXTI_RTSR_ADDRESS   (0x40013C08UL)
#define EXTI_FTSR_ADDRESS   (0x40013C0CUL)
#define EXTI_SWIER_ADDRESS  (0x40013C10UL)
#define EXTI_PR_ADDRESS     (0x40013C14UL)

typedef struct
{
  volatile uint32_t IMR;    /*!< Interrupt mask register,             Address offset: 0x00 */
  volatile uint32_t EMR;    /*!< Event mask register,               Address offset: 0x04 */
  volatile uint32_t RTSR;   /*!< Rising trigger selection register,     Address offset: 0x08 */
  volatile uint32_t FTSR;   /*!< Falling trigger selection register,    Address offset: 0x0C */
  volatile uint32_t SWIER;  /*!< Software interrupt event register,     Address offset: 0x10 */
  volatile uint32_t PR;     /*!< Pending register,                  Address offset: 0x14 */
} EXTI_TypeDef;

#define EXTI_BASE               (0x40013C00UL)
#define EXTI                    ((EXTI_TypeDef *) EXTI_BASE)


/**
 * @brief General-purpose I/Os (GPIO)
 * Base Addresses: 0x40020000 (GPIOA) to 0x40021C00 (GPIOH)
 */
typedef struct
{
  volatile uint32_t MODER;   /*!< GPIO port mode register,                     Address offset: 0x00 */
  volatile uint32_t OTYPER;  /*!< GPIO port output type register,              Address offset: 0x04 */
  volatile uint32_t OSPEEDR; /*!< GPIO port output speed register,             Address offset: 0x08 */
  volatile uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C */
  volatile uint32_t IDR;     /*!< GPIO port input data register,               Address offset: 0x10 */
  volatile uint32_t ODR;     /*!< GPIO port output data register,              Address offset: 0x14 */
  volatile uint32_t BSRR;    /*!< GPIO port bit set/reset register,            Address offset: 0x18 */
  volatile uint32_t LCKR;    /*!< GPIO port configuration lock register,       Address offset: 0x1C */
  volatile uint32_t AFRL;    /*!< GPIO alternate function low register,        Address offset: 0x20 */
  volatile uint32_t AFRH;    /*!< GPIO alternate function high register,       Address offset: 0x24 */
} GPIO_TypeDef;

#define GPIOA_BASE              (0x40020000UL)
#define GPIOB_BASE              (0x40020400UL)
#define GPIOC_BASE              (0x40020800UL)
#define GPIOD_BASE              (0x40020C00UL)
#define GPIOE_BASE              (0x40021000UL)
#define GPIOH_BASE              (0x40021C00UL) // Note the address jump for GPIOH

#define GPIOA                   ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB                   ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC                   ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD                   ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE                   ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH                   ((GPIO_TypeDef *) GPIOH_BASE)

// GPIO Bit Definitions (common settings)
#define GPIO_MODER_INPUT          (0U)
#define GPIO_MODER_OUTPUT         (1U)
#define GPIO_MODER_AF             (2U)
#define GPIO_MODER_ANALOG         (3U)

#define GPIO_OTYPER_PUSHPULL      (0U)
#define GPIO_OTYPER_OPENDRAIN     (1U)

#define GPIO_OSPEEDR_LOW          (0U)
#define GPIO_OSPEEDR_MEDIUM       (1U)
#define GPIO_OSPEEDR_HIGH         (2U)
#define GPIO_OSPEEDR_VERY_HIGH    (3U)

#define GPIO_PUPDR_NOPULL         (0U)
#define GPIO_PUPDR_PULLUP         (1U)
#define GPIO_PUPDR_PULLDOWN       (2U)

#define GPIO_BSRR_SET_Pos(PIN)    (PIN)
#define GPIO_BSRR_RESET_Pos(PIN)  ((PIN) + 16U)
#define GPIO_BSRR_SET(PIN)        (0x1UL << GPIO_BSRR_SET_Pos(PIN))
#define GPIO_BSRR_RESET(PIN)      (0x1UL << GPIO_BSRR_RESET_Pos(PIN))

/**
 * @brief Analog-to-Digital Converter (ADC)
 * Base Address: 0x40012000 (ADC1)
 * Common Control Register Base Address: 0x40012304 (ADC_CCR)
 */
#define ADC1_SR_ADDRESS         (0x40012000UL)
#define ADC1_CR1_ADDRESS        (0x40012004UL)
#define ADC1_CR2_ADDRESS        (0x40012008UL)
#define ADC1_SMPR1_ADDRESS      (0x4001200CUL)
#define ADC1_SMPR2_ADDRESS      (0x40012010UL)
// ... (other ADC1 registers)
#define ADC1_DR_ADDRESS         (0x4001204CUL)

typedef struct
{
  volatile uint32_t SR;          /*!< ADC status register,                                              Address offset: 0x00 */
  volatile uint32_t CR1;         /*!< ADC control register 1,                                           Address offset: 0x04 */
  volatile uint32_t CR2;         /*!< ADC control register 2,                                           Address offset: 0x08 */
  volatile uint32_t SMPR1;       /*!< ADC sample time register 1,                                       Address offset: 0x0C */
  volatile uint32_t SMPR2;       /*!< ADC sample time register 2,                                       Address offset: 0x10 */
  volatile uint32_t JOFR[4];     /*!< ADC injected channel data offset register 1-4,                    Address offset: 0x14-0x20 */
  volatile uint32_t HTR;         /*!< ADC watchdog higher threshold register,                           Address offset: 0x24 */
  volatile uint32_t LTR;         /*!< ADC watchdog lower threshold register,                            Address offset: 0x28 */
  volatile uint32_t SQR1;        /*!< ADC regular sequence register 1,                                  Address offset: 0x2C */
  volatile uint32_t SQR2;        /*!< ADC regular sequence register 2,                                  Address offset: 0x30 */
  volatile uint32_t SQR3;        /*!< ADC regular sequence register 3,                                  Address offset: 0x34 */
  volatile uint32_t JSQR;        /*!< ADC injected sequence register,                                   Address offset: 0x38 */
  volatile uint32_t JDR[4];      /*!< ADC injected data register 1-4,                                   Address offset: 0x3C-0x48 */
  volatile uint32_t DR;          /*!< ADC regular data register,                                        Address offset: 0x4C */
} ADC_TypeDef;

#define ADC1_BASE               (0x40012000UL)
#define ADC1                    ((ADC_TypeDef *) ADC1_BASE)

#define ADC_CCR_ADDRESS         (0x40012304UL)
typedef struct
{
  uint32_t      RESERVED0[77]; /*!< Reserved,                                 Address offset: 0x00-0x13F */
  volatile uint32_t CCR;         /*!< ADC common control register,                              Address offset: 0x304 */
} ADC_Common_TypeDef;

#define ADC_COMMON_BASE         (0x40012300UL) // The ADC_CCR is at 0x40012304, so base for common registers block is 0x40012300
#define ADC_COMMON              ((ADC_Common_TypeDef *) ADC_COMMON_BASE)

// ADC Bit Definitions (inferred)
#define ADC_CR2_ADON_Pos            (0U)
#define ADC_CR2_ADON                (0x1UL << ADC_CR2_ADON_Pos) /* ADC enable */


/**
 * @brief General-purpose Timers (TIM)
 * Multiple instances: TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11
 */
typedef struct
{
  volatile uint32_t CR1;         /*!< TIM control register 1,                      Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< TIM control register 2,                      Address offset: 0x04 */
  volatile uint32_t SMCR;        /*!< TIM slave mode control register,             Address offset: 0x08 */
  volatile uint32_t DIER;        /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  volatile uint32_t SR;          /*!< TIM status register,                         Address offset: 0x10 */
  volatile uint32_t EGR;         /*!< TIM event generation register,               Address offset: 0x14 */
  volatile uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,         Address offset: 0x18 */
  volatile uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,         Address offset: 0x1C */
  volatile uint32_t CCER;        /*!< TIM capture/compare enable register,         Address offset: 0x20 */
  volatile uint32_t CNT;         /*!< TIM counter register,                        Address offset: 0x24 */
  volatile uint32_t PSC;         /*!< TIM prescaler register,                      Address offset: 0x28 */
  volatile uint32_t ARR;         /*!< TIM auto-reload register,                    Address offset: 0x2C */
  volatile uint32_t RCR;         /*!< TIM repetition counter register,             Address offset: 0x30 */
  volatile uint32_t CCR[4];      /*!< TIM capture/compare register 1-4,            Address offset: 0x34-0x40 */
  volatile uint32_t BDTR;        /*!< TIM break and dead-time register,            Address offset: 0x44 */
  volatile uint32_t DCR;         /*!< TIM DMA control register,                    Address offset: 0x48 */
  volatile uint32_t DMAR;        /*!< TIM DMA address for full transfer,           Address offset: 0x4C */
  volatile uint32_t OR;          /*!< TIM option register (for TIM2/TIM5),         Address offset: 0x50/0x54 */
} TIM_TypeDef;

// For basic/general timers (TIM6, TIM7) - simplified struct
typedef struct
{
  volatile uint32_t CR1;         /*!< TIM control register 1,                      Address offset: 0x00 */
  uint32_t RESERVED0[1];         /*!< Reserved                                     Address offset: 0x04 */
  volatile uint32_t SMCR;        /*!< TIM slave mode control register,             Address offset: 0x08 */
  volatile uint32_t DIER;        /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  volatile uint32_t SR;          /*!< TIM status register,                         Address offset: 0x10 */
  volatile uint32_t EGR;         /*!< TIM event generation register,               Address offset: 0x14 */
  uint32_t RESERVED1[3];         /*!< Reserved                                     Address offset: 0x18-0x20 */
  volatile uint32_t CNT;         /*!< TIM counter register,                        Address offset: 0x24 */
  volatile uint32_t PSC;         /*!< TIM prescaler register,                      Address offset: 0x28 */
  volatile uint32_t ARR;         /*!< TIM auto-reload register,                    Address offset: 0x2C */
  volatile uint32_t RCR;         /*!< TIM repetition counter register,             Address offset: 0x30 */
  volatile uint32_t CCR[2];      /*!< TIM capture/compare register 1-2,            Address offset: 0x34-0x38 */
  uint32_t RESERVED2[3];         /*!< Reserved                                     Address offset: 0x3C-0x44 */
  volatile uint32_t DCR;         /*!< TIM DMA control register,                    Address offset: 0x48 */
  volatile uint32_t DMAR;        /*!< TIM DMA address for full transfer,           Address offset: 0x4C */
} TIM_GeneralPurpose_TypeDef;

// Specific Timer Addresses
#define TIM1_BASE               (0x40010000UL)
#define TIM2_BASE               (0x40000000UL)
#define TIM3_BASE               (0x40000400UL)
#define TIM4_BASE               (0x40000800UL)
#define TIM5_BASE               (0x40000C00UL)
#define TIM9_BASE               (0x40014000UL)
#define TIM10_BASE              (0x40014400UL)
#define TIM11_BASE              (0x40014800UL)

// Timer Instances
#define TIM1                    ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                    ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                    ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                    ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                    ((TIM_TypeDef *) TIM5_BASE)
#define TIM9                    ((TIM_GeneralPurpose_TypeDef *) TIM9_BASE) // TIM9/10/11 are limited features
#define TIM10                   ((TIM_GeneralPurpose_TypeDef *) TIM10_BASE)
#define TIM11                   ((TIM_GeneralPurpose_TypeDef *) TIM11_BASE)

// Timer Bit Definitions (inferred)
#define TIM_CR1_CEN_Pos             (0U)
#define TIM_CR1_CEN                 (0x1UL << TIM_CR1_CEN_Pos) /* Counter enable */
#define TIM_DIER_UIE_Pos            (0U)
#define TIM_DIER_UIE                (0x1UL << TIM_DIER_UIE_Pos) /* Update interrupt enable */
#define TIM_SR_UIF_Pos              (0U)
#define TIM_SR_UIF                  (0x1UL << TIM_SR_UIF_Pos) /* Update interrupt flag */
#define TIM_EGR_UG_Pos              (0U)
#define TIM_EGR_UG                  (0x1UL << TIM_EGR_UG_Pos) /* Update generation */

/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter (USART)
 * Multiple instances: USART1, USART2, USART6
 */
typedef struct
{
  volatile uint32_t SR;          /*!< USART Status register,                    Address offset: 0x00 */
  volatile uint32_t DR;          /*!< USART Data register,                      Address offset: 0x04 */
  volatile uint32_t BRR;         /*!< USART Baud rate register,                 Address offset: 0x08 */
  volatile uint32_t CR1;         /*!< USART Control register 1,                 Address offset: 0x0C */
  volatile uint32_t CR2;         /*!< USART Control register 2,                 Address offset: 0x10 */
  volatile uint32_t CR3;         /*!< USART Control register 3,                 Address offset: 0x14 */
  volatile uint32_t GTPR;        /*!< USART Guard time and prescaler register,  Address offset: 0x18 */
} USART_TypeDef;

// Specific USART Addresses
#define USART1_BASE             (0x40011000UL)
#define USART2_BASE             (0x40004400UL)
#define USART6_BASE             (0x40011400UL)

// USART Instances
#define USART1                  ((USART_TypeDef *) USART1_BASE)
#define USART2                  ((USART_TypeDef *) USART2_BASE)
#define USART6                  ((USART_TypeDef *) USART6_BASE)

// USART Bit Definitions (inferred)
#define USART_CR1_UE_Pos            (13U)
#define USART_CR1_UE                (0x1UL << USART_CR1_UE_Pos) /* USART enable */
#define USART_CR1_TE_Pos            (3U)
#define USART_CR1_TE                (0x1UL << USART_CR1_TE_Pos) /* Transmitter enable */
#define USART_CR1_RE_Pos            (2U)
#define USART_CR1_RE                (0x1UL << USART_CR1_RE_Pos) /* Receiver enable */
#define USART_SR_TXE_Pos            (7U)
#define USART_SR_TXE                (0x1UL << USART_SR_TXE_Pos) /* Transmit data register empty */
#define USART_SR_RXNE_Pos           (5U)
#define USART_SR_RXNE               (0x1UL << USART_SR_RXNE_Pos) /* Read data register not empty */

/**
 * @brief Inter-Integrated Circuit (I2C)
 * Multiple instances: I2C1, I2C2, I2C3
 */
typedef struct
{
  volatile uint32_t CR1;         /*!< I2C Control register 1,                   Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< I2C Control register 2,                   Address offset: 0x04 */
  volatile uint32_t OAR1;        /*!< I2C Own address register 1,               Address offset: 0x08 */
  volatile uint32_t OAR2;        /*!< I2C Own address register 2,               Address offset: 0x0C */
  volatile uint32_t DR;          /*!< I2C Data register,                        Address offset: 0x10 */
  volatile uint32_t SR1;         /*!< I2C Status register 1,                    Address offset: 0x14 */
  volatile uint32_t SR2;         /*!< I2C Status register 2,                    Address offset: 0x18 */
  volatile uint32_t CCR;         /*!< I2C Clock control register,               Address offset: 0x1C */
  volatile uint32_t TRISE;       /*!< I2C TRISE register,                       Address offset: 0x20 */
  volatile uint32_t FLTR;        /*!< I2C Filter register,                      Address offset: 0x24 */
} I2C_TypeDef;

// Specific I2C Addresses
#define I2C1_BASE               (0x40005400UL)
#define I2C2_BASE               (0x40005800UL)
#define I2C3_BASE               (0x40005C00UL)

// I2C Instances
#define I2C1                    ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                    ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                    ((I2C_TypeDef *) I2C3_BASE)

// I2C Bit Definitions (inferred)
#define I2C_CR1_PE_Pos              (0U)
#define I2C_CR1_PE                  (0x1UL << I2C_CR1_PE_Pos) /* Peripheral enable */
#define I2C_CR1_ACK_Pos             (10U)
#define I2C_CR1_ACK                 (0x1UL << I2C_CR1_ACK_Pos) /* Acknowledge enable */
#define I2C_CR1_START_Pos           (8U)
#define I2C_CR1_STOP_Pos            (9U)
#define I2C_SR1_SB_Pos              (0U)
#define I2C_SR1_ADDR_Pos            (1U)
#define I2C_SR1_TXE_Pos             (7U)
#define I2C_SR1_RXNE_Pos            (6U)
#define I2C_SR1_BTF_Pos             (2U)


/**
 * @brief Serial Peripheral Interface (SPI) / Inter-IC Sound (I2S)
 * Multiple instances: SPI1, SPI2, SPI3
 */
typedef struct
{
  volatile uint32_t CR1;         /*!< SPI Control register 1,                   Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< SPI Control register 2,                   Address offset: 0x04 */
  volatile uint32_t SR;          /*!< SPI Status register,                      Address offset: 0x08 */
  volatile uint32_t DR;          /*!< SPI Data register,                        Address offset: 0x0C */
  volatile uint32_t CRCPR;       /*!< SPI CRC polynomial register,              Address offset: 0x10 */
  volatile uint32_t RXCRCR;      /*!< SPI Rx CRC register,                      Address offset: 0x14 */
  volatile uint32_t TXCRCR;      /*!< SPI Tx CRC register,                      Address offset: 0x18 */
  volatile uint32_t I2SCFGR;     /*!< SPI I2S configuration register,           Address offset: 0x1C */
  volatile uint32_t I2SPR;       /*!< SPI I2S prescaler register,               Address offset: 0x20 */
} SPI_TypeDef;

// Specific SPI Addresses
#define SPI1_BASE               (0x40013000UL)
#define SPI2_BASE               (0x40003800UL)
#define SPI3_BASE               (0x40003C00UL)

// SPI Instances
#define SPI1                    ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                    ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                    ((SPI_TypeDef *) SPI3_BASE)

// SPI Bit Definitions (inferred)
#define SPI_CR1_SPE_Pos             (6U)
#define SPI_CR1_SPE                 (0x1UL << SPI_CR1_SPE_Pos) /* SPI enable */
#define SPI_CR1_MSTR_Pos            (2U)
#define SPI_CR1_MSTR                (0x1UL << SPI_CR1_MSTR_Pos) /* Master selection */
#define SPI_CR1_CPOL_Pos            (1U)
#define SPI_CR1_CPHA_Pos            (0U)
#define SPI_CR1_DFF_Pos             (11U)
#define SPI_CR1_LSBFIRST_Pos        (7U)
#define SPI_CR1_SSI_Pos             (8U)
#define SPI_CR1_SSM_Pos             (9U)
#define SPI_CR1_CRCEN_Pos           (13U)
#define SPI_SR_TXE_Pos              (1U)
#define SPI_SR_RXNE_Pos             (0U)
#define SPI_I2SCFGR_I2SMOD_Pos      (11U)
#define SPI_I2SCFGR_I2SMOD          (0x1UL << SPI_I2SCFGR_I2SMOD_Pos) /* I2S mode enable */


// IWDG (Independent Watchdog) registers (deduced for STM32, not in provided JSON)
#define IWDG_BASE               (0x40003000UL)
typedef struct
{
  volatile uint32_t KR;          /*!< IWDG Key register, Address offset: 0x00 */
  volatile uint32_t PR;          /*!< IWDG Prescaler register, Address offset: 0x04 */
  volatile uint32_t RLR;         /*!< IWDG Reload register, Address offset: 0x08 */
  volatile uint32_t SR;          /*!< IWDG Status register, Address offset: 0x0C */
} IWDG_TypeDef;

#define IWDG                    ((IWDG_TypeDef *) IWDG_BASE)

// PWR (Power Control) registers (deduced for STM32, not in provided JSON)
#define PWR_BASE                (0x40007000UL)
typedef struct
{
  volatile uint32_t CR;          /*!< PWR control register, Address offset: 0x00 */
  volatile uint32_t CSR;         /*!< PWR control/status register, Address offset: 0x04 */
} PWR_TypeDef;

#define PWR                     ((PWR_TypeDef *) PWR_BASE)

// PWR Bit Definitions (inferred)
#define PWR_CR_DBP_Pos              (8U)
#define PWR_CR_PVDE_Pos             (4U)
#define PWR_CR_PLS_Pos              (5U) /* PVD Level Selection (3 bits) */


/*==================================================================================================
*                                  FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief Initializes the Microcontroller Unit (MCU) configuration.
 *
 * This function performs essential MCU setup, including:
 * - Setting all GPIO pins to low and verifying.
 * - Setting all GPIO pin directions to input and verifying.
 * - Disabling all peripheral features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.).
 * - Enabling and clearing the Watchdog Timer (WDT) with a period >= 8ms.
 * - Setting the Low Voltage Reset (LVR) value based on the system voltage.
 * - Enabling the LVR.
 * - Clearing the WDT again.
 *
 * @param volt The desired system voltage (e.g., SYS_VOLT_3V3, SYS_VOLT_5V) to configure LVR.
 */
void MCU_Config_Init(t_sys_volt volt);

/**
 * @brief Resets the Watchdog Timer (WDT).
 *
 * This function clears the Watchdog Timer counter to prevent a system reset.
 * It should be called periodically within the WDT's timeout period.
 */
void WDT_Reset(void);

/**
 * @brief Puts the MCU into a low-power sleep mode.
 *
 * In this mode, the CPU stops executing instructions, but peripherals (except OS timer)
 * continue to function. The exact implementation depends on the MCU's power management
 * capabilities.
 */
void Go_to_sleep_mode(void);

/**
 * @brief Enables global interrupts.
 *
 * This function sets the global interrupt enable bit in the CPU's status register,
 * allowing the MCU to respond to interrupts.
 */
void Global_interrupt_Enable(void);

/**
 * @brief Disables global interrupts.
 *
 * This function clears the global interrupt enable bit in the CPU's status register,
 * preventing the MCU from responding to interrupts until re-enabled.
 */
void Global_interrupt_Disable(void);

/**
 * @brief Initializes the Low Voltage Detection (LVD) module.
 *
 * This function configures the LVD module, typically setting up its mode and initial state.
 * Specific threshold level configuration is handled by LVD_Get.
 */
void LVD_Init(void);

/**
 * @brief Sets the Low Voltage Detection (LVD) threshold level.
 *
 * This function configures the voltage level at which the LVD module will trigger
 * an event or interrupt.
 *
 * @param lvd_thresholdLevel The desired voltage threshold for LVD.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);

/**
 * @brief Enables the Low Voltage Detection (LVD) module.
 *
 * This function activates the LVD module, allowing it to monitor the supply voltage
 * and generate events or interrupts if the voltage drops below the configured threshold.
 */
void LVD_Enable(void);

/**
 * @brief Disables the Low Voltage Detection (LVD) module.
 *
 * This function deactivates the LVD module, stopping it from monitoring the supply voltage.
 */
void LVD_Disable(void);

/**
 * @brief Initializes a specified UART channel.
 *
 * Configures the selected UART channel with the given baud rate, data length,
 * stop bits, and parity settings.
 *
 * @param uart_channel The UART channel to initialize (e.g., UART_CHANNEL_1).
 * @param uart_baud_rate The desired baud rate for communication.
 * @param uart_data_length The number of data bits (e.g., 8-bit, 9-bit).
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting (none, even, or odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);

/**
 * @brief Enables a specified UART channel.
 *
 * Activates the peripheral clock for the UART channel and enables the UART module itself.
 *
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel);

/**
 * @brief Disables a specified UART channel.
 *
 * Deactivates the UART module and its peripheral clock.
 *
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel);

/**
 * @brief Sends a single byte over a specified UART channel.
 *
 * @param uart_channel The UART channel to use.
 * @param byte The byte to transmit.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);

/**
 * @brief Sends a frame of data over a specified UART channel.
 *
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the character array (frame) to transmit.
 * @param length The number of bytes in the frame.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string over a specified UART channel.
 *
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the null-terminated string to transmit.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str);

/**
 * @brief Reads a single byte from a specified UART channel.
 *
 * @param uart_channel The UART channel to read from.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel);

/**
 * @brief Reads a frame of data from a specified UART channel.
 *
 * @param uart_channel The UART channel to read from.
 * @param buffer Pointer to the buffer to store the received data.
 * @param max_length The maximum number of bytes to read into the buffer.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);

/**
 * @brief Reads a null-terminated string from a specified UART channel.
 *
 * Reads characters until a null terminator or max_length is reached.
 *
 * @param uart_channel The UART channel to read from.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum number of bytes (including null terminator) to read.
 * @return The first byte of the string, or 0 if no data is received within a timeout.
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

/**
 * @brief Initializes a specified I2C channel.
 *
 * Configures the selected I2C channel with the given clock speed, device address,
 * acknowledge setting, and data length (if applicable).
 *
 * @param i2c_channel The I2C channel to initialize (e.g., I2C_CHANNEL_1).
 * @param i2c_clk_speed The desired clock speed for I2C communication (e.g., 400kHz for fast mode).
 * @param i2c_device_address The 7-bit own device address for the I2C module.
 * @param i2c_ack The acknowledge setting (enable/disable).
 * @param i2c_datalength The data length setting (if applicable, typically byte-oriented).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);

/**
 * @brief Enables a specified I2C channel.
 *
 * Activates the peripheral clock for the I2C channel and enables the I2C module itself.
 *
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel);

/**
 * @brief Disables a specified I2C channel.
 *
 * Deactivates the I2C module and its peripheral clock.
 *
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel);

/**
 * @brief Sends a single byte over a specified I2C channel.
 *
 * Assumes a start condition has been generated and address sent.
 *
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to transmit.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);

/**
 * @brief Sends a frame of data over a specified I2C channel.
 *
 * Assumes a start condition has been generated and address sent.
 *
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the character array (frame) to transmit.
 * @param length The number of bytes in the frame.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string over a specified I2C channel.
 *
 * Assumes a start condition has been generated and address sent.
 *
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the null-terminated string to transmit.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);

/**
 * @brief Reads a single byte from a specified I2C channel.
 *
 * @param i2c_channel The I2C channel to read from.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);

/**
 * @brief Reads a frame of data from a specified I2C channel.
 *
 * @param i2c_channel The I2C channel to read from.
 * @param buffer Pointer to the buffer to store the received data.
 * @param max_length The maximum number of bytes to read into the buffer.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);

/**
 * @brief Reads a null-terminated string from a specified I2C channel.
 *
 * @param i2c_channel The I2C channel to read from.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum number of bytes (including null terminator) to read.
 * @return The first byte of the string, or 0 if no data is received within a timeout.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

/**
 * @brief Initializes a specified SPI channel (CSI).
 *
 * Configures the selected SPI channel with the given mode (master/slave),
 * clock polarity (CPOL), clock phase (CPHA), data frame format (DFF), and bit order.
 * Always uses fast speed, software-controlled slave select, full duplex, and CRC enabled.
 *
 * @param spi_channel The SPI channel to initialize (e.g., SPI_CHANNEL_1).
 * @param spi_mode The operating mode (master or slave).
 * @param spi_cpol Clock Polarity.
 * @param spi_cpha Clock Phase.
 * @param spi_dff Data Frame Format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);

/**
 * @brief Enables a specified SPI channel.
 *
 * Activates the peripheral clock for the SPI channel and enables the SPI module itself.
 *
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel);

/**
 * @brief Disables a specified SPI channel.
 *
 * Deactivates the SPI module and its peripheral clock.
 *
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel);

/**
 * @brief Sends a single byte over a specified SPI channel.
 *
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to transmit.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);

/**
 * @brief Sends a frame of data over a specified SPI channel.
 *
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the character array (frame) to transmit.
 * @param length The number of bytes in the frame.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);

/**
 * @brief Reads a single byte from a specified SPI channel.
 *
 * @param spi_channel The SPI channel to read from.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel);

/**
 * @brief Reads a frame of data from a specified SPI channel.
 *
 * @param spi_channel The SPI channel to read from.
 * @param buffer Pointer to the buffer to store the received data.
 * @param max_length The maximum number of bytes to read into the buffer.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);

/**
 * @brief Reads a null-terminated string from a specified SPI channel.
 *
 * @param spi_channel The SPI channel to read from.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum number of bytes (including null terminator) to read.
 * @return The first byte of the string, or 0 if no data is received within a timeout.
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

/**
 * @brief Initializes an external interrupt channel.
 *
 * Configures the specified external interrupt channel for a particular edge detection.
 * Also configures the corresponding GPIO pin for external interrupt functionality.
 *
 * @param external_int_channel The EXTI line number (0-15).
 * @param external_int_edge The trigger edge for the interrupt (rising, falling, or both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);

/**
 * @brief Enables an external interrupt channel.
 *
 * Unmasks the specified EXTI line, allowing it to generate an interrupt.
 *
 * @param external_int_channel The EXTI line number (0-15) to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel);

/**
 * @brief Disables an external interrupt channel.
 *
 * Masks the specified EXTI line, preventing it from generating an interrupt.
 *
 * @param external_int_channel The EXTI line number (0-15) to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel);

/**
 * @brief Initializes a GPIO pin for output.
 *
 * Sets the direction of the specified GPIO pin to output and sets its initial value.
 * Pull-up resistors are disabled for output pins.
 * The value is set before the direction.
 *
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number (e.g., PIN_0).
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);

/**
 * @brief Initializes a GPIO pin for input.
 *
 * Sets the direction of the specified GPIO pin to input.
 * Pull-up resistors are enabled, and wakeup features (if available) are enabled.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin);

/**
 * @brief Gets the direction of a specified GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction of the pin (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin);

/**
 * @brief Sets the output value of a specified GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The desired output value (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);

/**
 * @brief Gets the input value of a specified GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The current input value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin);

/**
 * @brief Toggles the output value of a specified GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin);

/**
 * @brief Initializes a specified PWM channel.
 *
 * Configures the selected PWM channel with the given frequency and duty cycle.
 * Clears available FREQUENCY Ranges for each channel as comments in PWM_Init() (comment here).
 * @param pwm_channel The PWM channel to initialize (e.g., PWM_CHANNEL_TIM1_CH1).
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);

/**
 * @brief Starts the PWM generation on a specified channel.
 *
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel);

/**
 * @brief Stops the PWM generation on a specified channel.
 *
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel);

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 *
 * Configures the selected ICU channel with the given prescaler and edge detection mode.
 *
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler setting for the timer used by ICU.
 * @param icu_edge The edge (rising, falling, or both) to detect for capture.
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);

/**
 * @brief Enables an Input Capture Unit (ICU) channel.
 *
 * Activates the ICU channel, allowing it to capture timer values on detected edges.
 *
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel);

/**
 * @brief Disables an Input Capture Unit (ICU) channel.
 *
 * Deactivates the ICU channel.
 *
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel);

/**
 * @brief Gets the frequency measured by an ICU channel.
 *
 * This function is intended to be called when an edge happens (e.g., in an ISR).
 *
 * @param icu_channel The ICU channel from which to get the frequency.
 * @return The measured frequency.
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Return type fixed to tlong for frequency

/**
 * @brief Sets a callback function for a specific ICU channel.
 *
 * The callback function will be invoked when an input capture event occurs.
 *
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void));

/**
 * @brief Initializes a specified Timer channel.
 *
 * Sets up the basic configuration for the timer, often including its clock source
 * and operating mode. Specific time settings are handled by other TIMER_Set_Time functions.
 *
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel);

/**
 * @brief Sets the timer period in microseconds.
 *
 * Configures the auto-reload register and prescaler to achieve the desired
 * timer period in microseconds.
 *
 * @param timer_channel The timer channel to configure.
 * @param time The desired time period in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time);

/**
 * @brief Sets the timer period in milliseconds.
 *
 * Configures the auto-reload register and prescaler to achieve the desired
 * timer period in milliseconds.
 *
 * @param timer_channel The timer channel to configure.
 * @param time The desired time period in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);

/**
 * @brief Sets the timer period in seconds.
 *
 * Configures the auto-reload register and prescaler to achieve the desired
 * timer period in seconds.
 *
 * @param timer_channel The timer channel to configure.
 * @param time The desired time period in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Sets the timer period in minutes.
 *
 * Configures the auto-reload register and prescaler to achieve the desired
 * timer period in minutes.
 *
 * @param timer_channel The timer channel to configure.
 * @param time The desired time period in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Sets the timer period in hours.
 *
 * Configures the auto-reload register and prescaler to achieve the desired
 * timer period in hours.
 *
 * @param timer_channel The timer channel to configure.
 * @param time The desired time period in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Enables a specified Timer channel.
 *
 * Activates the peripheral clock for the timer and starts the timer counter.
 *
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel);

/**
 * @brief Disables a specified Timer channel.
 *
 * Stops the timer counter and deactivates its peripheral clock.
 *
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel);

/**
 * @brief Initializes a specified ADC channel.
 *
 * Configures the selected ADC channel with the specified operating mode (polling, interrupt, or DMA).
 * This typically includes setting up sample times, resolution, and conversion sequence.
 *
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The desired operating mode for the ADC.
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);

/**
 * @brief Enables a specified ADC channel.
 *
 * Activates the peripheral clock for the ADC and enables the ADC module itself.
 *
 * @param adc_channel The ADC channel to enable.
 */
void ADC_Enable(t_adc_channel adc_channel);

/**
 * @brief Disables a specified ADC channel.
 *
 * Deactivates the ADC module and its peripheral clock.
 *
 * @param adc_channel The ADC channel to disable.
 */
void ADC_Disable(t_adc_channel adc_channel);

/**
 * @brief Performs an ADC conversion in polling mode for a specified channel.
 *
 * Initiates a conversion, waits for its completion, and returns the result.
 *
 * @param adc_channel The ADC channel to sample.
 * @return The 12-bit converted digital value.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel);

/**
 * @brief Gets the ADC conversion result when operating in interrupt mode.
 *
 * This function typically reads the last converted value after an interrupt has
 * indicated conversion completion. It does not initiate a conversion.
 *
 * @param adc_channel The ADC channel to read.
 * @return The 12-bit converted digital value.
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel);

/**
 * @brief Initializes the internal EEPROM module.
 *
 * Prepares the EEPROM for read and write operations, including any necessary clock
 * enables or unlocking sequences.
 */
void Internal_EEPROM_Init(void);

/**
 * @brief Writes a byte of data to a specified address in the internal EEPROM.
 *
 * @param address The target address within the EEPROM.
 * @param data The byte of data to write.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data);

/**
 * @brief Reads a byte of data from a specified address in the internal EEPROM.
 *
 * @param address The target address within the EEPROM.
 * @return The byte of data read from the EEPROM.
 */
tbyte Internal_EEPROM_Get(tbyte address);

/**
 * @brief Initializes the Time-Triggered (TT) OS scheduler.
 *
 * Configures the base tick time for the scheduler, which determines the frequency
 * of the TT_ISR and task dispatching.
 *
 * @param tick_time_ms The tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms);

/**
 * @brief Starts the Time-Triggered (TT) OS scheduler.
 *
 * Enables the timer responsible for generating the system ticks, effectively
 * starting the scheduler.
 */
void TT_Start(void);

/**
 * @brief Dispatches scheduled tasks in the Time-Triggered (TT) OS.
 *
 * This function should be called periodically (e.g., in the main loop or a low-priority task)
 * to check for and execute tasks that are due.
 */
void TT_Dispatch_task(void);

/**
 * @brief Interrupt Service Routine (ISR) handler for the Time-Triggered (TT) OS.
 *
 * This function is called by the timer interrupt at each system tick. It updates
 * task timing and flags tasks for dispatch.
 */
void TT_ISR(void);

/**
 * @brief Adds a task to the Time-Triggered (TT) OS scheduler.
 *
 * @param task Pointer to the task function to be executed.
 * @param period The period (in ticks) at which the task should repeat.
 * @param delay The initial delay (in ticks) before the task's first execution.
 * @return The index of the added task, or a negative value if the task list is full.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);

/**
 * @brief Deletes a task from the Time-Triggered (TT) OS scheduler.
 *
 * @param task_index The index of the task to be deleted.
 */
void TT_Delete_task(const tbyte task_index);

// MCAL_OUTPUT_BUZZER not supported on this MCU (no specific buzzer registers found).
// DAC not supported on this MCU (no specific DAC registers found).
// MQTT Protocol not supported on this MCU (requires external network stack and hardware).
// HTTP Protocol not supported on this MCU (requires external network stack and hardware).
// WiFi Driver not supported on this MCU (requires external WiFi module and driver).
// DTC_driver not supported on this MCU (no specific DTC registers found).

/**
 * @brief Initializes a specified I2S channel.
 *
 * Configures the selected I2S channel with the given mode (master/slave, transmit/receive),
 * standard (e.g., Philips), data format, channel mode (stereo/mono), sample rate,
 * master clock frequency, and DMA buffer size.
 *
 * @param channel The I2S channel to initialize (e.g., I2S_CHANNEL_1, which maps to SPI1_I2SCFGR/I2SPR).
 * @param mode I2S operating mode.
 * @param standard I2S protocol standard.
 * @param data_format I2S data length.
 * @param channel_mode I2S channel configuration.
 * @param sample_rate The desired audio sample rate (e.g., 44100 Hz).
 * @param mclk_freq The master clock frequency (if MCLK output is enabled).
 * @param dma_buffer_size The size of the DMA buffer for I2S transfers.
 */
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);

/**
 * @brief Enables a specified I2S channel.
 *
 * Activates the peripheral clock for the I2S channel and enables the I2S module itself.
 *
 * @param channel The I2S channel to enable.
 */
void I2S_Enable(t_i2s_channel channel);

/**
 * @brief Transmits data over a specified I2S channel.
 *
 * @param channel The I2S channel to use.
 * @param data Pointer to the data buffer to transmit.
 * @param length The number of bytes to transmit.
 */
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);

/**
 * @brief Receives data from a specified I2S channel.
 *
 * @param channel The I2S channel to use.
 * @param buffer Pointer to the buffer to store the received data.
 * @param length The number of bytes to receive.
 */
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);


#endif /* MCAL_H */