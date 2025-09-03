/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) header file for STM32F401RC.
 *
 * This file contains the definitions, types, and function prototypes
 * for the MCAL layer, providing an abstract interface to the microcontroller's
 * peripherals. It adheres to the specified coding rules and API definitions.
 *
 * MCU: STM32F401RC
 */

#ifndef MCAL_H
#define MCAL_H

// --- Core Includes ---
#include "stm32f401xc.h" // Core device header for STM32F401RC
#include <stdint.h>     // Standard integer types
#include <stdbool.h>    // Standard boolean type
#include <stddef.h>     // Standard definitions (e.g., size_t)

// --- Data Type Definitions (as per Rules.json) ---
typedef uint8_t  tbyte;  // Defines a byte (8-bit unsigned integer)
typedef uint16_t tword;  // Defines a word (16-bit unsigned integer)
typedef uint32_t tlong;  // Defines a long (32-bit unsigned integer)
typedef int8_t   tsbyte; // Defines a signed byte (8-bit signed integer)
typedef int32_t  tslong; // Defines a signed long (32-bit signed integer)


// --- Peripheral Base Addresses (Normalized) ---
// FLASH
#define FLASH_BASE          (0x40023C00UL)
// RCC
#define RCC_BASE            (0x40023800UL)
// SYSCFG
#define SYSCFG_BASE         (0x40013800UL)
// EXTI
#define EXTI_BASE           (0x40013C00UL)
// ADC
#define ADC1_BASE           (0x40012000UL)
#define ADC_COMMON_BASE     (0x40012300UL) // ADC_CCR is here
// GPIO
#define GPIOA_BASE          (0x40020000UL)
#define GPIOB_BASE          (0x40020400UL)
#define GPIOC_BASE          (0x40020800UL)
#define GPIOD_BASE          (0x40020C00UL)
#define GPIOE_BASE          (0x40021000UL)
#define GPIOH_BASE          (0x40021C00UL)
// TIMERS
#define TIM1_BASE           (0x40010000UL)
#define TIM2_BASE           (0x40000000UL)
#define TIM3_BASE           (0x40000400UL)
#define TIM4_BASE           (0x40000800UL)
#define TIM5_BASE           (0x40000C00UL)
#define TIM9_BASE           (0x40014000UL)
#define TIM10_BASE          (0x40014400UL)
#define TIM11_BASE          (0x40014800UL)
// USART
#define USART1_BASE         (0x40011000UL)
#define USART2_BASE         (0x40004400UL)
#define USART6_BASE         (0x40011400UL)
// I2C
#define I2C1_BASE           (0x40005400UL)
#define I2C2_BASE           (0x40005800UL)
#define I2C3_BASE           (0x40005C00UL)
// SPI
#define SPI1_BASE           (0x40013000UL)
#define SPI2_BASE           (0x40003800UL)
#define SPI3_BASE           (0x40003C00UL)


// --- Peripheral Register Structures (Typedefs) ---

/**
  * @brief FLASH Registers
  */
typedef struct
{
  volatile uint32_t ACR;         /*!< Flash access control register.,                      Address offset: 0x00 */
  volatile uint32_t KEYR;        /*!< Flash key register.,                                 Address offset: 0x04 */
  volatile uint32_t OPTKEYR;     /*!< Flash option key register.,                          Address offset: 0x08 */
  volatile uint32_t SR;          /*!< Flash status register.,                              Address offset: 0x0C */
  volatile uint32_t CR;          /*!< Flash control register.,                             Address offset: 0x10 */
  volatile uint32_t OPTCR;       /*!< Flash option control register.,                      Address offset: 0x14 */
} FLASH_TypeDef;

/**
  * @brief Reset and Clock Control (RCC) Registers
  */
typedef struct
{
  volatile uint32_t CR;          /*!< RCC clock control register.,                         Address offset: 0x00 */
  volatile uint32_t PLLCFGR;     /*!< RCC PLL configuration register.,                     Address offset: 0x04 */
  volatile uint32_t CFGR;        /*!< RCC clock configuration register.,                   Address offset: 0x08 */
  volatile uint32_t CIR;         /*!< RCC clock interrupt register.,                       Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register.,                 Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register.,                 Address offset: 0x14 */
  volatile uint32_t APB1RSTR;    /*!< RCC APB1 peripheral reset register.,                 Address offset: 0x18 */
  volatile uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register.,                 Address offset: 0x1C */
  volatile uint32_t RESERVED0[2];/*!< Reserved, 0x20-0x28 */
  volatile uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clock enable register.,          Address offset: 0x30 */
  volatile uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clock enable register.,          Address offset: 0x34 */
  volatile uint32_t APB1ENR;     /*!< RCC APB1 peripheral clock enable register.,          Address offset: 0x38 */
  volatile uint32_t APB2ENR;     /*!< RCC APB2 peripheral clock enable register.,          Address offset: 0x3C */
  volatile uint32_t AHB1LPENR;   /*!< RCC AHB1 peripheral clock enable in low power mode register., Address offset: 0x40 */
  volatile uint32_t AHB2LPENR;   /*!< RCC AHB2 peripheral clock enable in low power mode register., Address offset: 0x44 */
  volatile uint32_t APB1LPENR;   /*!< RCC APB1 peripheral clock enable in low power mode register., Address offset: 0x48 */
  volatile uint32_t APB2LPENR;   /*!< RCC APB2 peripheral clock enabled in low power mode register., Address offset: 0x4C */
  volatile uint32_t BDCR;        /*!< RCC Backup domain control register.,                 Address offset: 0x50 */
  volatile uint32_t CSR;         /*!< RCC clock control & status register.,                Address offset: 0x54 */
  volatile uint32_t SSCGR;       /*!< RCC spread spectrum clock generation register.,      Address offset: 0x58 */
  volatile uint32_t PLLI2SCFGR;  /*!< RCC PLLI2S configuration register.,                  Address offset: 0x5C */
  volatile uint32_t RESERVED1;   /*!< Reserved, 0x60 */
  volatile uint32_t DCKCFGR;     /*!< RCC Dedicated Clocks Configuration Register.,        Address offset: 0x64 */
} RCC_TypeDef;

/**
  * @brief System Configuration Controller (SYSCFG) Registers
  */
typedef struct
{
  volatile uint32_t MEMRMP;      /*!< SYSCFG memory remap register.,                       Address offset: 0x00 */
  volatile uint32_t PMC;         /*!< SYSCFG peripheral mode configuration register.,      Address offset: 0x04 */
  volatile uint32_t EXTICR1;     /*!< SYSCFG external interrupt configuration register 1 (EXTI0-EXTI3 source selection)., Address offset: 0x08 */
  volatile uint32_t EXTICR2;     /*!< SYSCFG external interrupt configuration register 2 (EXTI4-EXTI7 source selection)., Address offset: 0x0C */
  volatile uint32_t EXTICR3;     /*!< SYSCFG external interrupt configuration register 3 (EXTI8-EXTI11 source selection)., Address offset: 0x10 */
  volatile uint32_t EXTICR4;     /*!< SYSCFG external interrupt configuration register 4 (EXTI12-EXTI15 source selection)., Address offset: 0x14 */
  volatile uint32_t RESERVED[2]; /*!< Reserved, 0x18-0x1C */
  volatile uint32_t CMPCR;       /*!< Compensation cell control register.,                 Address offset: 0x20 */
} SYSCFG_TypeDef;

/**
  * @brief External Interrupt/Event Controller (EXTI) Registers
  */
typedef struct
{
  volatile uint32_t IMR;         /*!< Interrupt mask register.,                            Address offset: 0x00 */
  volatile uint32_t EMR;         /*!< Event mask register.,                                Address offset: 0x04 */
  volatile uint32_t RTSR;        /*!< Rising trigger selection register.,                  Address offset: 0x08 */
  volatile uint32_t FTSR;        /*!< Falling trigger selection register.,                 Address offset: 0x0C */
  volatile uint32_t SWIER;       /*!< Software interrupt event register.,                  Address offset: 0x10 */
  volatile uint32_t PR;          /*!< Pending register.,                                   Address offset: 0x14 */
} EXTI_TypeDef;

/**
  * @brief General Purpose I/O (GPIO) Registers
  */
typedef struct
{
  volatile uint32_t MODER;       /*!< GPIO port mode register,                             Address offset: 0x00 */
  volatile uint32_t OTYPER;      /*!< GPIO port output type register,                      Address offset: 0x04 */
  volatile uint32_t OSPEEDR;     /*!< GPIO port output speed register,                     Address offset: 0x08 */
  volatile uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,                Address offset: 0x0C */
  volatile uint32_t IDR;         /*!< GPIO port input data register,                       Address offset: 0x10 */
  volatile uint32_t ODR;         /*!< GPIO port output data register,                      Address offset: 0x14 */
  volatile uint32_t BSRR;        /*!< GPIO port bit set/reset register,                    Address offset: 0x18 */
  volatile uint32_t LCKR;        /*!< GPIO port configuration lock register,               Address offset: 0x1C */
  volatile uint32_t AFRL;        /*!< GPIO alternate function low register,                Address offset: 0x20 */
  volatile uint32_t AFRH;        /*!< GPIO alternate function high register,               Address offset: 0x24 */
} GPIO_TypeDef;

/**
  * @brief Analog-to-Digital Converter (ADC) Registers
  */
typedef struct
{
  volatile uint32_t SR;          /*!< ADC status register,                                 Address offset: 0x00 */
  volatile uint32_t CR1;         /*!< ADC control register 1,                              Address offset: 0x04 */
  volatile uint32_t CR2;         /*!< ADC control register 2,                              Address offset: 0x08 */
  volatile uint32_t SMPR1;       /*!< ADC sample time register 1,                          Address offset: 0x0C */
  volatile uint32_t SMPR2;       /*!< ADC sample time register 2,                          Address offset: 0x10 */
  volatile uint32_t JOFR1;       /*!< ADC injected channel data offset register 1,         Address offset: 0x14 */
  volatile uint32_t JOFR2;       /*!< ADC injected channel data offset register 2,         Address offset: 0x18 */
  volatile uint32_t JOFR3;       /*!< ADC injected channel data offset register 3,         Address offset: 0x1C */
  volatile uint32_t JOFR4;       /*!< ADC injected channel data offset register 4,         Address offset: 0x20 */
  volatile uint32_t HTR;         /*!< ADC watchdog higher threshold register,              Address offset: 0x24 */
  volatile uint32_t LTR;         /*!< ADC watchdog lower threshold register,               Address offset: 0x28 */
  volatile uint32_t SQR1;        /*!< ADC regular sequence register 1,                     Address offset: 0x2C */
  volatile uint32_t SQR2;        /*!< ADC regular sequence register 2,                     Address offset: 0x30 */
  volatile uint32_t SQR3;        /*!< ADC regular sequence register 3,                     Address offset: 0x34 */
  volatile uint32_t JSQR;        /*!< ADC injected sequence register,                      Address offset: 0x38 */
  volatile uint32_t JDR1;        /*!< ADC injected data register 1,                        Address offset: 0x3C */
  volatile uint32_t JDR2;        /*!< ADC injected data register 2,                        Address offset: 0x40 */
  volatile uint32_t JDR3;        /*!< ADC injected data register 3,                        Address offset: 0x44 */
  volatile uint32_t JDR4;        /*!< ADC injected data register 4,                        Address offset: 0x48 */
  volatile uint32_t DR;          /*!< ADC regular data register,                           Address offset: 0x4C */
} ADC_TypeDef;

/**
  * @brief ADC Common Registers
  */
typedef struct
{
  volatile uint32_t RESERVED[76]; /*!< Reserved, 0x00 - 0x12F*/
  volatile uint32_t CCR;          /*!< ADC common control register,                       Address offset: 0x304 (from ADC1_BASE) */
} ADC_Common_TypeDef;

/**
  * @brief Timer (TIM) Registers (Generic structure, specifics might vary)
  */
typedef struct
{
  volatile uint32_t CR1;         /*!< TIM control register 1,                              Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< TIM control register 2,                              Address offset: 0x04 */
  volatile uint32_t SMCR;        /*!< TIM slave mode control register,                     Address offset: 0x08 */
  volatile uint32_t DIER;        /*!< TIM DMA/interrupt enable register,                   Address offset: 0x0C */
  volatile uint32_t SR;          /*!< TIM status register,                                 Address offset: 0x10 */
  volatile uint32_t EGR;         /*!< TIM event generation register,                       Address offset: 0x14 */
  volatile uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,                 Address offset: 0x18 */
  volatile uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,                 Address offset: 0x1C */
  volatile uint32_t CCER;        /*!< TIM capture/compare enable register,                 Address offset: 0x20 */
  volatile uint32_t CNT;         /*!< TIM counter register,                                Address offset: 0x24 */
  volatile uint32_t PSC;         /*!< TIM prescaler register,                              Address offset: 0x28 */
  volatile uint32_t ARR;         /*!< TIM auto-reload register,                            Address offset: 0x2C */
  volatile uint32_t RCR;         /*!< TIM repetition counter register,                     Address offset: 0x30 */
  volatile uint32_t CCR1;        /*!< TIM capture/compare register 1,                      Address offset: 0x34 */
  volatile uint32_t CCR2;        /*!< TIM capture/compare register 2,                      Address offset: 0x38 */
  volatile uint32_t CCR3;        /*!< TIM capture/compare register 3,                      Address offset: 0x3C */
  volatile uint32_t CCR4;        /*!< TIM capture/compare register 4,                      Address offset: 0x40 */
  volatile uint32_t BDTR;        /*!< TIM break and dead-time register,                    Address offset: 0x44 */
  volatile uint32_t DCR;         /*!< TIM DMA control register,                            Address offset: 0x48 */
  volatile uint32_t DMAR;        /*!< TIM DMA address for full transfer,                   Address offset: 0x4C */
  volatile uint32_t OR;          /*!< TIM option register (Only for TIM2, TIM5),           Address offset: 0x50/0x54 */
} TIM_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter (USART) Registers
  */
typedef struct
{
  volatile uint32_t SR;          /*!< USART Status register,                               Address offset: 0x00 */
  volatile uint32_t DR;          /*!< USART Data register,                                 Address offset: 0x04 */
  volatile uint32_t BRR;         /*!< USART Baud rate register,                            Address offset: 0x08 */
  volatile uint32_t CR1;         /*!< USART Control register 1,                            Address offset: 0x0C */
  volatile uint32_t CR2;         /*!< USART Control register 2,                            Address offset: 0x10 */
  volatile uint32_t CR3;         /*!< USART Control register 3,                            Address offset: 0x14 */
  volatile uint32_t GTPR;        /*!< USART Guard time and prescaler register,             Address offset: 0x18 */
} USART_TypeDef;

/**
  * @brief Inter-Integrated Circuit (I2C) Registers
  */
typedef struct
{
  volatile uint32_t CR1;         /*!< I2C Control register 1,                              Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< I2C Control register 2,                              Address offset: 0x04 */
  volatile uint32_t OAR1;        /*!< I2C Own address register 1,                          Address offset: 0x08 */
  volatile uint32_t OAR2;        /*!< I2C Own address register 2,                          Address offset: 0x0C */
  volatile uint32_t DR;          /*!< I2C Data register,                                   Address offset: 0x10 */
  volatile uint32_t SR1;         /*!< I2C Status register 1,                               Address offset: 0x14 */
  volatile uint32_t SR2;         /*!< I2C Status register 2,                               Address offset: 0x18 */
  volatile uint32_t CCR;         /*!< I2C Clock control register,                          Address offset: 0x1C */
  volatile uint32_t TRISE;       /*!< I2C TRISE register,                                  Address offset: 0x20 */
  volatile uint32_t FLTR;        /*!< I2C Filter register,                                 Address offset: 0x24 */
} I2C_TypeDef;

/**
  * @brief Serial Peripheral Interface (SPI) Registers
  */
typedef struct
{
  volatile uint32_t CR1;         /*!< SPI Control register 1,                              Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< SPI Control register 2,                              Address offset: 0x04 */
  volatile uint32_t SR;          /*!< SPI Status register,                                 Address offset: 0x08 */
  volatile uint32_t DR;          /*!< SPI Data register,                                   Address offset: 0x0C */
  volatile uint32_t CRCPR;       /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
  volatile uint32_t RXCRCR;      /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
  volatile uint32_t TXCRCR;      /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
  volatile uint32_t I2SCFGR;     /*!< SPI I2S configuration register,                      Address offset: 0x1C */
  volatile uint32_t I2SPR;       /*!< SPI I2S prescaler register,                          Address offset: 0x20 */
} SPI_TypeDef;


// --- Peripheral Instance Pointers ---
#define FLASH               ((FLASH_TypeDef *) FLASH_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC_COMMON          ((ADC_Common_TypeDef *) ADC_COMMON_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)


// --- ENUM and TYPEDEF Definitions (as per API.json and Rules.json) ---

/**
 * @brief System voltage levels for MCU configuration.
 */
typedef enum
{
    SYS_VOLT_3V = 0,    /**< System voltage 3V */
    SYS_VOLT_5V         /**< System voltage 5V */
} t_sys_volt;

/**
 * @brief Low Voltage Detection (LVD) threshold levels.
 * (Note: LVD peripheral not explicitly found in register_json, but API requires this enum)
 */
typedef enum
{
    LVD_THRESHOLD_0_5V = 0,
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1_5V,
    LVD_THRESHOLD_2V,
    LVD_THRESHOLD_2_5V,
    LVD_THRESHOLD_3V,
    LVD_THRESHOLD_3_5V,
    LVD_THRESHOLD_4V,
    LVD_THRESHOLD_4_5V,
    LVD_THRESHOLD_5V
} t_lvd_thrthresholdLevel;

/**
 * @brief UART channel selection.
 */
typedef enum
{
    UART_CHANNEL_1 = 0, /**< USART1 Peripheral */
    UART_CHANNEL_2,     /**< USART2 Peripheral */
    UART_CHANNEL_6      /**< USART6 Peripheral */
} t_uart_channel;

/**
 * @brief UART baud rate selection.
 */
typedef enum
{
    UART_BAUD_9600 = 0,   /**< 9600 baud rate */
    UART_BAUD_19200,      /**< 19200 baud rate */
    UART_BAUD_38400,      /**< 38400 baud rate */
    UART_BAUD_57600,      /**< 57600 baud rate */
    UART_BAUD_115200      /**< 115200 baud rate */
} t_uart_baud_rate;

/**
 * @brief UART data length selection.
 */
typedef enum
{
    UART_DATA_8BIT = 0,   /**< 8-bit data frame */
    UART_DATA_9BIT        /**< 9-bit data frame */
} t_uart_data_length;

/**
 * @brief UART stop bit selection.
 */
typedef enum
{
    UART_STOP_1_BIT = 0,  /**< 1 stop bit */
    UART_STOP_0_5_BIT,    /**< 0.5 stop bits */
    UART_STOP_2_BIT,      /**< 2 stop bits */
    UART_STOP_1_5_BIT     /**< 1.5 stop bits */
} t_uart_stop_bit;

/**
 * @brief UART parity selection.
 */
typedef enum
{
    UART_PARITY_NONE = 0, /**< No parity */
    UART_PARITY_EVEN,     /**< Even parity */
    UART_PARITY_ODD       /**< Odd parity */
} t_uart_parity;

/**
 * @brief I2C channel selection.
 */
typedef enum
{
    I2C_CHANNEL_1 = 0,  /**< I2C1 Peripheral */
    I2C_CHANNEL_2,      /**< I2C2 Peripheral */
    I2C_CHANNEL_3       /**< I2C3 Peripheral */
} t_i2c_channel;

/**
 * @brief I2C clock speed selection.
 */
typedef enum
{
    I2C_CLK_SPEED_STANDARD_MODE = 0, /**< 100 kHz */
    I2C_CLK_SPEED_FAST_MODE          /**< 400 kHz */
} t_i2c_clk_speed;

/**
 * @brief I2C acknowledgment control.
 */
typedef enum
{
    I2C_ACK_DISABLE = 0, /**< ACK disabled */
    I2C_ACK_ENABLE       /**< ACK enabled */
} t_i2c_ack;

/**
 * @brief I2C data length for frame transfer.
 */
typedef enum
{
    I2C_DATALENGTH_8BIT = 0, /**< 8-bit data length */
    I2C_DATALENGTH_16BIT     /**< 16-bit data length */
} t_i2c_datalength;

/**
 * @brief I2C device address (Note: This is typically an integer value, not an enum).
 */
typedef uint16_t t_i2c_device_address;

/**
 * @brief SPI channel selection.
 */
typedef enum
{
    SPI_CHANNEL_1 = 0,  /**< SPI1 Peripheral */
    SPI_CHANNEL_2,      /**< SPI2 Peripheral */
    SPI_CHANNEL_3       /**< SPI3 Peripheral */
} t_spi_channel;

/**
 * @brief SPI mode selection (Master/Slave).
 */
typedef enum
{
    SPI_MODE_SLAVE = 0, /**< Slave mode */
    SPI_MODE_MASTER     /**< Master mode */
} t_spi_mode;

/**
 * @brief SPI clock polarity.
 */
typedef enum
{
    SPI_CPOL_LOW = 0,   /**< Clock to 0 when idle */
    SPI_CPOL_HIGH       /**< Clock to 1 when idle */
} t_spi_cpol;

/**
 * @brief SPI clock phase.
 */
typedef enum
{
    SPI_CPHA_1EDGE = 0, /**< First clock transition is first data capture edge */
    SPI_CPHA_2EDGE      /**< Second clock transition is first data capture edge */
} t_spi_cpha;

/**
 * @brief SPI data frame format.
 */
typedef enum
{
    SPI_DFF_8BIT = 0,   /**< 8-bit data frame */
    SPI_DFF_16BIT       /**< 16-bit data frame */
} t_spi_dff;

/**
 * @brief SPI bit order (MSB/LSB first).
 */
typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0, /**< Most significant bit first */
    SPI_BIT_ORDER_LSB_FIRST      /**< Least significant bit first */
} t_spi_bit_order;

/**
 * @brief External interrupt channel selection (EXTI line).
 */
typedef enum
{
    EXT_INT_CHANNEL_0 = 0,  /**< EXTI Line 0 */
    EXT_INT_CHANNEL_1,      /**< EXTI Line 1 */
    EXT_INT_CHANNEL_2,      /**< EXTI Line 2 */
    EXT_INT_CHANNEL_3,      /**< EXTI Line 3 */
    EXT_INT_CHANNEL_4,      /**< EXTI Line 4 */
    EXT_INT_CHANNEL_5,      /**< EXTI Line 5 */
    EXT_INT_CHANNEL_6,      /**< EXTI Line 6 */
    EXT_INT_CHANNEL_7,      /**< EXTI Line 7 */
    EXT_INT_CHANNEL_8,      /**< EXTI Line 8 */
    EXT_INT_CHANNEL_9,      /**< EXTI Line 9 */
    EXT_INT_CHANNEL_10,     /**< EXTI Line 10 */
    EXT_INT_CHANNEL_11,     /**< EXTI Line 11 */
    EXT_INT_CHANNEL_12,     /**< EXTI Line 12 */
    EXT_INT_CHANNEL_13,     /**< EXTI Line 13 */
    EXT_INT_CHANNEL_14,     /**< EXTI Line 14 */
    EXT_INT_CHANNEL_15      /**< EXTI Line 15 */
} t_external_int_channel;

/**
 * @brief External interrupt trigger edge selection.
 */
typedef enum
{
    EXT_INT_EDGE_RISING = 0,      /**< Rising edge trigger */
    EXT_INT_EDGE_FALLING,         /**< Falling edge trigger */
    EXT_INT_EDGE_RISING_FALLING   /**< Both rising and falling edge trigger */
} t_external_int_edge;

/**
 * @brief GPIO port selection.
 */
typedef enum
{
    GPIO_PORT_A = 0, /**< GPIO Port A */
    GPIO_PORT_B,     /**< GPIO Port B */
    GPIO_PORT_C,     /**< GPIO Port C */
    GPIO_PORT_D,     /**< GPIO Port D */
    GPIO_PORT_E,     /**< GPIO Port E */
    GPIO_PORT_H      /**< GPIO Port H */
} t_port;

/**
 * @brief GPIO pin selection.
 */
typedef enum
{
    GPIO_PIN_0 = 0,   /**< Pin 0 */
    GPIO_PIN_1,       /**< Pin 1 */
    GPIO_PIN_2,       /**< Pin 2 */
    GPIO_PIN_3,       /**< Pin 3 */
    GPIO_PIN_4,       /**< Pin 4 */
    GPIO_PIN_5,       /**< Pin 5 */
    GPIO_PIN_6,       /**< Pin 6 */
    GPIO_PIN_7,       /**< Pin 7 */
    GPIO_PIN_8,       /**< Pin 8 */
    GPIO_PIN_9,       /**< Pin 9 */
    GPIO_PIN_10,      /**< Pin 10 */
    GPIO_PIN_11,      /**< Pin 11 */
    GPIO_PIN_12,      /**< Pin 12 */
    GPIO_PIN_13,      /**< Pin 13 */
    GPIO_PIN_14,      /**< Pin 14 */
    GPIO_PIN_15       /**< Pin 15 */
} t_pin;

/**
 * @brief GPIO pin direction.
 */
typedef enum
{
    GPIO_DIRECTION_INPUT = 0, /**< Input direction */
    GPIO_DIRECTION_OUTPUT     /**< Output direction */
} t_direction;

/**
 * @brief PWM channel selection.
 */
typedef enum
{
    PWM_CHANNEL_TIM1_CH1 = 0,  // PA8, PE9, PB13, PA7
    PWM_CHANNEL_TIM1_CH2,      // PA9, PE11, PB0, PB14
    PWM_CHANNEL_TIM1_CH3,      // PA10, PE13, PB1, PB15
    PWM_CHANNEL_TIM1_CH4,      // PA11, PE14

    PWM_CHANNEL_TIM2_CH1,      // PA0, PA5, PA15, PB3
    PWM_CHANNEL_TIM2_CH2,      // PA1, PB3, PB10
    PWM_CHANNEL_TIM2_CH3,      // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,      // PA3, PB11

    PWM_CHANNEL_TIM3_CH1,      // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,      // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,      // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,      // PB1, PC9

    PWM_CHANNEL_TIM4_CH1,      // PB6
    PWM_CHANNEL_TIM4_CH2,      // PB7
    PWM_CHANNEL_TIM4_CH3,      // PB8
    PWM_CHANNEL_TIM4_CH4,      // PB9

    PWM_CHANNEL_TIM5_CH1,      // PA0
    PWM_CHANNEL_TIM5_CH2,      // PA1
    PWM_CHANNEL_TIM5_CH3,      // PA2
    PWM_CHANNEL_TIM5_CH4,      // PA3

    PWM_CHANNEL_TIM9_CH1,      // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,      // PA3, PE6

    PWM_CHANNEL_TIM10_CH1,     // PB8, PA6

    PWM_CHANNEL_TIM11_CH1      // PB9, PA7
} t_pwm_channel;

/**
 * @brief ICU channel selection (Input Capture Unit).
 */
typedef enum
{
    ICU_CHANNEL_TIM1_CH1 = 0,
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

/**
 * @brief ICU prescaler values.
 */
typedef enum
{
    ICU_PRESCALER_DIV1 = 0,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8
} t_icu_prescaller;

/**
 * @brief ICU edge detection selection.
 */
typedef enum
{
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

/**
 * @brief Timer channel selection.
 */
typedef enum
{
    TIMER_CHANNEL_TIM1 = 0, /**< TIM1 Peripheral */
    TIMER_CHANNEL_TIM2,     /**< TIM2 Peripheral */
    TIMER_CHANNEL_TIM3,     /**< TIM3 Peripheral */
    TIMER_CHANNEL_TIM4,     /**< TIM4 Peripheral */
    TIMER_CHANNEL_TIM5,     /**< TIM5 Peripheral */
    TIMER_CHANNEL_TIM9,     /**< TIM9 Peripheral */
    TIMER_CHANNEL_TIM10,    /**< TIM10 Peripheral */
    TIMER_CHANNEL_TIM11     /**< TIM11 Peripheral */
} t_timer_channel;

/**
 * @brief ADC channel selection.
 */
typedef enum
{
    ADC_CHANNEL_0 = 0,  /**< ADC Channel 0 (PA0) */
    ADC_CHANNEL_1,      /**< ADC Channel 1 (PA1) */
    ADC_CHANNEL_2,      /**< ADC Channel 2 (PA2) */
    ADC_CHANNEL_3,      /**< ADC Channel 3 (PA3) */
    ADC_CHANNEL_4,      /**< ADC Channel 4 (PA4) */
    ADC_CHANNEL_5,      /**< ADC Channel 5 (PA5) */
    ADC_CHANNEL_6,      /**< ADC Channel 6 (PA6, PC0) */
    ADC_CHANNEL_7,      /**< ADC Channel 7 (PA7, PC1) */
    ADC_CHANNEL_8,      /**< ADC Channel 8 (PB0, PC2) */
    ADC_CHANNEL_9,      /**< ADC Channel 9 (PB1, PC3) */
    ADC_CHANNEL_10,     /**< ADC Channel 10 (PC0, PE7) */
    ADC_CHANNEL_11,     /**< ADC Channel 11 (PC1, PE8) */
    ADC_CHANNEL_12,     /**< ADC Channel 12 (PC2, PE9) */
    ADC_CHANNEL_13,     /**< ADC Channel 13 (PC3, PE10) */
    ADC_CHANNEL_14,     /**< ADC Channel 14 (PC4, PE11) */
    ADC_CHANNEL_15      /**< ADC Channel 15 (PC5, PE12) */
    // Also internal channels (temp sensor, Vref) but not listed in assigned_pin
} t_adc_channel;

/**
 * @brief ADC mode selection.
 */
typedef enum
{
    ADC_MODE_POLLING = 0,    /**< Polling mode for ADC conversion */
    ADC_MODE_INTERRUPT,      /**< Interrupt mode for ADC conversion */
    ADC_MODE_DMA             /**< DMA mode for ADC conversion */
} t_adc_mode_t;

/**
 * @brief Tick time in milliseconds for Time Triggered OS.
 */
typedef enum
{
    TT_TICK_TIME_1MS = 1,
    TT_TICK_TIME_10MS = 10,
    TT_TICK_TIME_100MS = 100
} t_tick_time;

/**
 * @brief I2S channel selection.
 */
typedef enum
{
    I2S_CHANNEL_1 = 0, /**< I2S1 (via SPI1) */
    I2S_CHANNEL_2,     /**< I2S2 (via SPI2) */
    I2S_CHANNEL_3      /**< I2S3 (via SPI3) */
} t_i2s_channel;

/**
 * @brief I2S operation mode.
 */
typedef enum
{
    I2S_MODE_MASTER_TX = 0,
    I2S_MODE_MASTER_RX,
    I2S_MODE_SLAVE_TX,
    I2S_MODE_SLAVE_RX
} I2S_Mode_t;

/**
 * @brief I2S audio standard.
 */
typedef enum
{
    I2S_STANDARD_PHILIPS = 0,
    I2S_STANDARD_MSB,
    I2S_STANDARD_LSB,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

/**
 * @brief I2S data format.
 */
typedef enum
{
    I2S_DATAFORMAT_16B = 0,
    I2S_DATAFORMAT_16B_EXTENDED,
    I2S_DATAFORMAT_24B,
    I2S_DATAFORMAT_32B
} I2S_DataFormat_t;

/**
 * @brief I2S channel mode.
 */
typedef enum
{
    I2S_CHANNELMODE_STEREO = 0,
    I2S_CHANNELMODE_MONO
} I2S_ChannelMode_t;

/**
 * @brief Wi-Fi transmit mode.
 */
typedef enum
{
    WIFI_TX_MODE_LEGACY = 0,
    WIFI_TX_MODE_HT
} t_tx_mode;

// --- API Function Prototypes (as per API.json) ---

// --- MCU CONFIG ---
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // This WDT_Reset is from MCU CONFIG category, not the separate WDT module.
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// --- LVD ---
// LVD module skipped: No explicit LVD registers found in register_json for STM32F401RC.
// If an LVD equivalent exists (e.g., PVD in PWR controller), it would be implemented here.

// --- UART ---
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

// --- I2C ---
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

// --- SPI (CSI) ---
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

// --- External Interrupt ---
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);

// --- GPIO ---
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);

// --- PWM ---
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);

// --- ICU ---
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
// ICU_GetFrequency function needs a return type. Assuming uint32_t for frequency.
uint32_t ICU_GetFrequency(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));

// --- Timer ---
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);

// --- ADC ---
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(t_adc_channel adc_channel);
void ADC_Disable(t_adc_channel adc_channel);
tword ADC_Get_POLLING(t_adc_channel adc_channel);
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel);

// --- Internal_EEPROM ---
// Internal_EEPROM module skipped: No explicit EEPROM registers found in register_json for STM32F401RC.

// --- TT (Time Triggered OS) ---
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// --- MCAL_OUTPUT_BUZZER ---
// MCAL_OUTPUT_BUZZER module skipped: No explicit buzzer-specific registers found in register_json for STM32F401RC.

// --- WDT (Watchdog Timer) ---
void WDT_Init(void);
// WDT_Reset(void) is already declared under "MCU CONFIG" category, which is fine as it's a common functionality.
// Note: This WDT_Reset is expected to call WDT_Reset() again if called from a generic WDT API.

// --- DAC ---
// DAC module skipped: No explicit DAC registers found in register_json for STM32F401RC.

// --- I2S ---
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// --- MQTT Protocol ---
// MQTT Protocol module skipped: No MCU registers for this high-level protocol in register_json.

// --- HTTP Protocol ---
// HTTP Protocol module skipped: No MCU registers for this high-level protocol in register_json.

// --- WiFi Driver ---
// WiFi Driver module skipped: No MCU registers for this module in register_json.

// --- DTC_driver ---
// DTC_driver module skipped: No explicit DTC registers found in register_json for STM32F401RC.


#endif // MCAL_H