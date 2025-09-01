/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) header file for STM32F401RC.
 *
 * This file contains the declarations for MCAL functions and register definitions,
 * providing an abstract interface to the microcontroller's peripherals.
 *
 * @date 2023-10-27
 * @author Your Name/Generator
 */

#ifndef MCAL_H
#define MCAL_H

// global_includes from Rules.json
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
// MCU Specific Includes (if any, e.g., for common bit definitions or CMSIS)

// Define status codes for MCAL functions
typedef enum {
    MCAL_OK = 0U,
    MCAL_ERROR = 1U,
    MCAL_INVALID_PARAM = 2U,
    MCAL_NOT_IMPLEMENTED = 3U
} MCAL_Status_t;

// MCU Name: STM32F401RC

// --- Register Definitions ---

// FLASH Registers
#define FLASH_ACR_REG       ((volatile uint32_t*)0x40023C00)   /**< Flash access control register. */
#define FLASH_KEYR_REG      ((volatile uint32_t*)0x40023C04)   /**< Flash key register. Used to unlock the Flash control register. */
#define FLASH_OPTKEYR_REG   ((volatile uint32_t*)0x40023C08)   /**< Flash option key register. Used to unlock the Flash option control register. */
#define FLASH_SR_REG        ((volatile uint32_t*)0x40023C0C)   /**< Flash status register. */
#define FLASH_CR_REG        ((volatile uint32_t*)0x40023C10)   /**< Flash control register. */
#define FLASH_OPTCR_REG     ((volatile uint32_t*)0x40023C14)   /**< Flash option control register. */

// RCC Registers
#define RCC_CR_REG          ((volatile uint32_t*)0x40023800)   /**< RCC clock control register. */
#define RCC_PLLCFGR_REG     ((volatile uint32_t*)0x40023804)   /**< RCC PLL configuration register. */
#define RCC_CFGR_REG        ((volatile uint32_t*)0x40023808)   /**< RCC clock configuration register. */
#define RCC_CIR_REG         ((volatile uint32_t*)0x4002380C)   /**< RCC clock interrupt register. */
#define RCC_AHB1RSTR_REG    ((volatile uint32_t*)0x40023810)   /**< RCC AHB1 peripheral reset register. */
#define RCC_AHB2RSTR_REG    ((volatile uint32_t*)0x40023814)   /**< RCC AHB2 peripheral reset register. */
#define RCC_APB1RSTR_REG    ((volatile uint32_t*)0x40023818)   /**< RCC APB1 peripheral reset register. */
#define RCC_APB2RSTR_REG    ((volatile uint32_t*)0x4002381C)   /**< RCC APB2 peripheral reset register. */
#define RCC_AHB1ENR_REG     ((volatile uint32_t*)0x40023830)   /**< RCC AHB1 peripheral clock enable register. */
#define RCC_AHB2ENR_REG     ((volatile uint32_t*)0x40023834)   /**< RCC AHB2 peripheral clock enable register. */
#define RCC_APB1ENR_REG     ((volatile uint32_t*)0x40023838)   /**< RCC APB1 peripheral clock enable register. */
#define RCC_APB2ENR_REG     ((volatile uint32_t*)0x4002383C)   /**< RCC APB2 peripheral clock enable register. */
#define RCC_AHB1LPENR_REG   ((volatile uint32_t*)0x40023840)   /**< RCC AHB1 peripheral clock enable in low power mode register. */
#define RCC_AHB2LPENR_REG   ((volatile uint32_t*)0x40023844)   /**< RCC AHB2 peripheral clock enable in low power mode register. */
#define RCC_APB1LPENR_REG   ((volatile uint32_t*)0x40023848)   /**< RCC APB1 peripheral clock enable in low power mode register. */
#define RCC_APB2LPENR_REG   ((volatile uint32_t*)0x4002384C)   /**< RCC APB2 peripheral clock enabled in low power mode register. */
#define RCC_BDCR_REG        ((volatile uint32_t*)0x40023850)   /**< RCC Backup domain control register. */
#define RCC_CSR_REG         ((volatile uint32_t*)0x40023854)   /**< RCC clock control & status register. */
#define RCC_SSCGR_REG       ((volatile uint32_t*)0x40023858)   /**< RCC spread spectrum clock generation register. */
#define RCC_PLLI2SCFGR_REG  ((volatile uint32_t*)0x4002385C)   /**< RCC PLLI2S configuration register. */
#define RCC_DCKCFGR_REG     ((volatile uint32_t*)0x40023864)   /**< RCC Dedicated Clocks Configuration Register. */

// SYSCFG Registers
#define SYSCFG_MEMRMP_REG   ((volatile uint32_t*)0x40013800)   /**< SYSCFG memory remap register. */
#define SYSCFG_PMC_REG      ((volatile uint32_t*)0x40013804)   /**< SYSCFG peripheral mode configuration register. */
#define SYSCFG_EXTICR1_REG  ((volatile uint32_t*)0x40013808)   /**< SYSCFG external interrupt configuration register 1. */
#define SYSCFG_EXTICR2_REG  ((volatile uint32_t*)0x4001380C)   /**< SYSCFG external interrupt configuration register 2. */
#define SYSCFG_EXTICR3_REG  ((volatile uint32_t*)0x40013810)   /**< SYSCFG external interrupt configuration register 3. */
#define SYSCFG_EXTICR4_REG  ((volatile uint32_t*)0x40013814)   /**< SYSCFG external interrupt configuration register 4. */
#define SYSCFG_CMPCR_REG    ((volatile uint32_t*)0x40013820)   /**< Compensation cell control register. */

// GPIO Base Addresses (Used with offsets to form full register addresses)
#define GPIOA_BASE          0x40020000UL
#define GPIOB_BASE          0x40020400UL
#define GPIOC_BASE          0x40020800UL
#define GPIOD_BASE          0x40020C00UL
#define GPIOE_BASE          0x40021000UL
#define GPIOH_BASE          0x40021C00UL

// GPIO Register Offsets
#define GPIO_MODER_OFFSET   0x00U  /**< GPIO port mode register */
#define GPIO_OTYPER_OFFSET  0x04U  /**< GPIO port output type register */
#define GPIO_OSPEEDR_OFFSET 0x08U  /**< GPIO port output speed register */
#define GPIO_PUPDR_OFFSET   0x0CU  /**< GPIO port pull-up/pull-down register */
#define GPIO_IDR_OFFSET     0x10U  /**< GPIO port input data register */
#define GPIO_ODR_OFFSET     0x14U  /**< GPIO port output data register */
#define GPIO_BSRR_OFFSET    0x18U  /**< GPIO port bit set/reset register */
#define GPIO_LCKR_OFFSET    0x1CU  /**< GPIO port configuration lock register */
#define GPIO_AFRL_OFFSET    0x20U  /**< GPIO alternate function low register */
#define GPIO_AFRH_OFFSET    0x24U  /**< GPIO alternate function high register */

/** Macro to get a pointer to a specific GPIO register */
#define GPIO_REG(GPIO_BASE_ADDR, OFFSET) \
    ((volatile uint32_t*)(GPIO_BASE_ADDR + OFFSET))

// EXTI Registers
#define EXTI_IMR_REG        ((volatile uint32_t*)0x40013C00)   /**< Interrupt mask register. */
#define EXTI_EMR_REG        ((volatile uint32_t*)0x40013C04)   /**< Event mask register. */
#define EXTI_RTSR_REG       ((volatile uint32_t*)0x40013C08)   /**< Rising trigger selection register. */
#define EXTI_FTSR_REG       ((volatile uint32_t*)0x40013C0C)   /**< Falling trigger selection register. */
#define EXTI_SWIER_REG      ((volatile uint32_t*)0x40013C10)   /**< Software interrupt event register. */
#define EXTI_PR_REG         ((volatile uint32_t*)0x40013C14)   /**< Pending register. */

// ADC Registers
#define ADC1_SR_REG         ((volatile uint32_t*)0x40012000)   /**< ADC status register. */
#define ADC1_CR1_REG        ((volatile uint32_t*)0x40012004)   /**< ADC control register 1. */
#define ADC1_CR2_REG        ((volatile uint32_t*)0x40012008)   /**< ADC control register 2. */
#define ADC1_SMPR1_REG      ((volatile uint32_t*)0x4001200C)   /**< ADC sample time register 1. */
#define ADC1_SMPR2_REG      ((volatile uint32_t*)0x40012010)   /**< ADC sample time register 2. */
#define ADC1_JOFR1_REG      ((volatile uint32_t*)0x40012014)   /**< ADC injected channel data offset register 1. */
#define ADC1_JOFR2_REG      ((volatile uint32_t*)0x40012018)   /**< ADC injected channel data offset register 2. */
#define ADC1_JOFR3_REG      ((volatile uint32_t*)0x4001201C)   /**< ADC injected channel data offset register 3. */
#define ADC1_JOFR4_REG      ((volatile uint32_t*)0x40012020)   /**< ADC injected channel data offset register 4. */
#define ADC1_HTR_REG        ((volatile uint32_t*)0x40012024)   /**< ADC watchdog higher threshold register. */
#define ADC1_LTR_REG        ((volatile uint32_t*)0x40012028)   /**< ADC watchdog lower threshold register. */
#define ADC1_SQR1_REG       ((volatile uint32_t*)0x4001202C)   /**< ADC regular sequence register 1. */
#define ADC1_SQR2_REG       ((volatile uint32_t*)0x40012030)   /**< ADC regular sequence register 2. */
#define ADC1_SQR3_REG       ((volatile uint32_t*)0x40012034)   /**< ADC regular sequence register 3. */
#define ADC1_JSQR_REG       ((volatile uint32_t*)0x40012038)   /**< ADC injected sequence register. */
#define ADC1_JDR1_REG       ((volatile uint32_t*)0x4001203C)   /**< ADC injected data register 1. */
#define ADC1_JDR2_REG       ((volatile uint32_t*)0x40012040)   /**< ADC injected data register 2. */
#define ADC1_JDR3_REG       ((volatile uint32_t*)0x40012044)   /**< ADC injected data register 3. */
#define ADC1_JDR4_REG       ((volatile uint32_t*)0x40012048)   /**< ADC injected data register 4. */
#define ADC1_DR_REG         ((volatile uint32_t*)0x4001204C)   /**< ADC regular data register. */
#define ADC_CCR_REG         ((volatile uint32_t*)0x40012304)   /**< ADC common control register. */

// TIMERS Base Addresses
#define TIM1_BASE           0x40010000UL
#define TIM2_BASE           0x40000000UL
#define TIM3_BASE           0x40000400UL
#define TIM4_BASE           0x40000800UL
#define TIM5_BASE           0x40000C00UL
#define TIM9_BASE           0x40014000UL
#define TIM10_BASE          0x40014400UL
#define TIM11_BASE          0x40014800UL

// TIMER Register Offsets
#define TIMER_CR1_OFFSET    0x00U  /**< Control register 1 */
#define TIMER_CR2_OFFSET    0x04U  /**< Control register 2 */
#define TIMER_SMCR_OFFSET   0x08U  /**< Slave mode control register */
#define TIMER_DIER_OFFSET   0x0CU  /**< DMA/interrupt enable register */
#define TIMER_SR_OFFSET     0x10U  /**< Status register */
#define TIMER_EGR_OFFSET    0x14U  /**< Event generation register */
#define TIMER_CCMR1_OFFSET  0x18U  /**< Capture/compare mode register 1 */
#define TIMER_CCMR2_OFFSET  0x1CU  /**< Capture/compare mode register 2 */
#define TIMER_CCER_OFFSET   0x20U  /**< Capture/compare enable register */
#define TIMER_CNT_OFFSET    0x24U  /**< Counter register */
#define TIMER_PSC_OFFSET    0x28U  /**< Prescaler register */
#define TIMER_ARR_OFFSET    0x2CU  /**< Auto-reload register */
#define TIMER_RCR_OFFSET    0x30U  /**< Repetition counter register (TIM1 only) */
#define TIMER_CCR1_OFFSET   0x34U  /**< Capture/compare register 1 */
#define TIMER_CCR2_OFFSET   0x38U  /**< Capture/compare register 2 */
#define TIMER_CCR3_OFFSET   0x3CU  /**< Capture/compare register 3 */
#define TIMER_CCR4_OFFSET   0x40U  /**< Capture/compare register 4 */
#define TIMER_BDTR_OFFSET   0x44U  /**< Break and dead-time register (TIM1 only) */
#define TIMER_DCR_OFFSET    0x48U  /**< DMA control register */
#define TIMER_DMAR_OFFSET   0x4CU  /**< DMA address for full transfer */
#define TIMER_OR_OFFSET     0x50U  /**< Option register (TIM2, TIM5 only) */

/** Macro to get a pointer to a specific TIMER register */
#define TIMER_REG(TIMER_BASE_ADDR, OFFSET) \
    ((volatile uint32_t*)(TIMER_BASE_ADDR + OFFSET))

// USART Base Addresses
#define USART1_BASE         0x40011000UL
#define USART2_BASE         0x40004400UL
#define USART6_BASE         0x40011400UL

// USART Register Offsets
#define USART_SR_OFFSET     0x00U  /**< Status register */
#define USART_DR_OFFSET     0x04U  /**< Data register */
#define USART_BRR_OFFSET    0x08U  /**< Baud rate register */
#define USART_CR1_OFFSET    0x0CU  /**< Control register 1 */
#define USART_CR2_OFFSET    0x10U  /**< Control register 2 */
#define USART_CR3_OFFSET    0x14U  /**< Control register 3 */
#define USART_GTPR_OFFSET   0x18U  /**< Guard time and prescaler register */

/** Macro to get a pointer to a specific USART register */
#define USART_REG(USART_BASE_ADDR, OFFSET) \
    ((volatile uint32_t*)(USART_BASE_ADDR + OFFSET))

// I2C Base Addresses
#define I2C1_BASE           0x40005400UL
#define I2C2_BASE           0x40005800UL
#define I2C3_BASE           0x40005C00UL

// I2C Register Offsets
#define I2C_CR1_OFFSET      0x00U  /**< Control register 1 */
#define I2C_CR2_OFFSET      0x04U  /**< Control register 2 */
#define I2C_OAR1_OFFSET     0x08U  /**< Own address register 1 */
#define I2C_OAR2_OFFSET     0x0CU  /**< Own address register 2 */
#define I2C_DR_OFFSET       0x10U  /**< Data register */
#define I2C_SR1_OFFSET      0x14U  /**< Status register 1 */
#define I2C_SR2_OFFSET      0x18U  /**< Status register 2 */
#define I2C_CCR_OFFSET      0x1CU  /**< Clock control register */
#define I2C_TRISE_OFFSET    0x20U  /**< TRISE register */
#define I2C_FLTR_OFFSET     0x24U  /**< Filter register */

/** Macro to get a pointer to a specific I2C register */
#define I2C_REG(I2C_BASE_ADDR, OFFSET) \
    ((volatile uint32_t*)(I2C_BASE_ADDR + OFFSET))

// SPI Base Addresses
#define SPI1_BASE           0x40013000UL
#define SPI2_BASE           0x40003800UL
#define SPI3_BASE           0x40003C00UL

// SPI Register Offsets
#define SPI_CR1_OFFSET      0x00U  /**< Control register 1 */
#define SPI_CR2_OFFSET      0x04U  /**< Control register 2 */
#define SPI_SR_OFFSET       0x08U  /**< Status register */
#define SPI_DR_OFFSET       0x0CU  /**< Data register */
#define SPI_CRCPR_OFFSET    0x10U  /**< CRC polynomial register */
#define SPI_RXCRCR_OFFSET   0x14U  /**< Rx CRC register */
#define SPI_TXCRCR_OFFSET   0x18U  /**< Tx CRC register */
#define SPI_I2SCFGR_OFFSET  0x1CU  /**< I2S configuration register */
#define SPI_I2SPR_OFFSET    0x20U  /**< I2S prescaler register */

/** Macro to get a pointer to a specific SPI register */
#define SPI_REG(SPI_BASE_ADDR, OFFSET) \
    ((volatile uint32_t*)(SPI_BASE_ADDR + OFFSET))


// --- Common Definitions for Peripherals ---

// GPIO Port types
typedef enum {
    MCAL_GPIO_PORTA = 0U,
    MCAL_GPIO_PORTB = 1U,
    MCAL_GPIO_PORTC = 2U,
    MCAL_GPIO_PORTD = 3U,
    MCAL_GPIO_PORTE = 4U,
    MCAL_GPIO_PORTH = 5U // For STM32F401RC, PH0/PH1 are available
} MCAL_GPIO_Port_t;

// GPIO Pin types
typedef enum {
    MCAL_GPIO_PIN_0  = (1U << 0),
    MCAL_GPIO_PIN_1  = (1U << 1),
    MCAL_GPIO_PIN_2  = (1U << 2),
    MCAL_GPIO_PIN_3  = (1U << 3),
    MCAL_GPIO_PIN_4  = (1U << 4),
    MCAL_GPIO_PIN_5  = (1U << 5),
    MCAL_GPIO_PIN_6  = (1U << 6),
    MCAL_GPIO_PIN_7  = (1U << 7),
    MCAL_GPIO_PIN_8  = (1U << 8),
    MCAL_GPIO_PIN_9  = (1U << 9),
    MCAL_GPIO_PIN_10 = (1U << 10),
    MCAL_GPIO_PIN_11 = (1U << 11),
    MCAL_GPIO_PIN_12 = (1U << 12),
    MCAL_GPIO_PIN_13 = (1U << 13),
    MCAL_GPIO_PIN_14 = (1U << 14),
    MCAL_GPIO_PIN_15 = (1U << 15),
    MCAL_GPIO_PIN_ALL = 0xFFFFU
} MCAL_GPIO_Pin_t;

// GPIO Pin States
typedef enum {
    MCAL_GPIO_LOW  = 0U,
    MCAL_GPIO_HIGH = 1U
} MCAL_GPIO_PinState_t;

// GPIO Mode
typedef enum {
    MCAL_GPIO_MODE_INPUT       = 0U, // Input Floating
    MCAL_GPIO_MODE_OUTPUT_PP   = 1U, // General purpose output push-pull
    MCAL_GPIO_MODE_OUTPUT_OD   = 2U, // General purpose output open-drain
    MCAL_GPIO_MODE_AF_PP       = 3U, // Alternate function push-pull
    MCAL_GPIO_MODE_AF_OD       = 4U, // Alternate function open-drain
    MCAL_GPIO_MODE_ANALOG      = 5U, // Analog mode
    MCAL_GPIO_MODE_EXTI_IT_RISING = 6U, // External Interrupt with Rising trigger
    MCAL_GPIO_MODE_EXTI_IT_FALLING = 7U, // External Interrupt with Falling trigger
    MCAL_GPIO_MODE_EXTI_IT_RISING_FALLING = 8U // External Interrupt with Rising/Falling trigger
} MCAL_GPIO_Mode_t;

// GPIO Pull-up/Pull-down
typedef enum {
    MCAL_GPIO_NO_PULL    = 0U,
    MCAL_GPIO_PULLUP     = 1U,
    MCAL_GPIO_PULLDOWN   = 2U
} MCAL_GPIO_Pull_t;

// GPIO Output Speed
typedef enum {
    MCAL_GPIO_SPEED_LOW      = 0U,
    MCAL_GPIO_SPEED_MEDIUM   = 1U,
    MCAL_GPIO_SPEED_HIGH     = 2U,
    MCAL_GPIO_SPEED_VERY_HIGH = 3U
} MCAL_GPIO_Speed_t;

// GPIO Alternate Function enumeration (AF0-AF15 for STM32F4)
typedef enum {
    MCAL_GPIO_AF0  = 0U,
    MCAL_GPIO_AF1  = 1U,
    MCAL_GPIO_AF2  = 2U,
    MCAL_GPIO_AF3  = 3U,
    MCAL_GPIO_AF4  = 4U,
    MCAL_GPIO_AF5  = 5U,
    MCAL_GPIO_AF6  = 6U,
    MCAL_GPIO_AF7  = 7U,
    MCAL_GPIO_AF8  = 8U,
    MCAL_GPIO_AF9  = 9U,
    MCAL_GPIO_AF10 = 10U,
    MCAL_GPIO_AF11 = 11U,
    MCAL_GPIO_AF12 = 12U,
    MCAL_GPIO_AF13 = 13U,
    MCAL_GPIO_AF14 = 14U,
    MCAL_GPIO_AF15 = 15U
} MCAL_GPIO_AlternateFunction_t;

/**
 * @brief Structure for GPIO configuration.
 */
typedef struct {
    MCAL_GPIO_Port_t          Port;          /**< Specifies the GPIO port. */
    MCAL_GPIO_Pin_t           Pin;           /**< Specifies the GPIO pin(s) to be configured. */
    MCAL_GPIO_Mode_t          Mode;          /**< Specifies the operating mode for the selected pin. */
    MCAL_GPIO_Pull_t          Pull;          /**< Specifies the Pull-up or Pull-down activation for the selected pin. */
    MCAL_GPIO_Speed_t         Speed;         /**< Specifies the speed for the selected pin. */
    MCAL_GPIO_AlternateFunction_t Alternate; /**< Peripheral to be connected to the selected pin(s). Only relevant for AF modes. */
} MCAL_GPIO_Init_t;


// RCC Peripheral types (for clock enable/disable, reset)
typedef enum {
    MCAL_RCC_AHB1_GPIOA = (1U << 0),
    MCAL_RCC_AHB1_GPIOB = (1U << 1),
    MCAL_RCC_AHB1_GPIOC = (1U << 2),
    MCAL_RCC_AHB1_GPIOD = (1U << 3),
    MCAL_RCC_AHB1_GPIOE = (1U << 4),
    MCAL_RCC_AHB1_GPIOH = (1U << 7),
    MCAL_RCC_AHB1_CRC   = (1U << 12),
    MCAL_RCC_AHB1_DMA1  = (1U << 21),
    MCAL_RCC_AHB1_DMA2  = (1U << 22),

    MCAL_RCC_AHB2_OTGFS = (1U << 7), // USB OTG FS

    MCAL_RCC_APB1_TIM2  = (1U << 0),
    MCAL_RCC_APB1_TIM3  = (1U << 1),
    MCAL_RCC_APB1_TIM4  = (1U << 2),
    MCAL_RCC_APB1_TIM5  = (1U << 3),
    MCAL_RCC_APB1_WWDG  = (1U << 11), // Window watchdog
    MCAL_RCC_APB1_SPI2  = (1U << 14),
    MCAL_RCC_APB1_SPI3  = (1U << 15),
    MCAL_RCC_APB1_USART2= (1U << 17),
    MCAL_RCC_APB1_I2C1  = (1U << 21),
    MCAL_RCC_APB1_I2C2  = (1U << 22),
    MCAL_RCC_APB1_I2C3  = (1U << 23),
    MCAL_RCC_APB1_PWR   = (1U << 28),

    MCAL_RCC_APB2_TIM1  = (1U << 0),
    MCAL_RCC_APB2_USART1= (1U << 4),
    MCAL_RCC_APB2_USART6= (1U << 5),
    MCAL_RCC_APB2_ADC1  = (1U << 8),
    MCAL_RCC_APB2_SDIO  = (1U << 11),
    MCAL_RCC_APB2_SPI1  = (1U << 12),
    MCAL_RCC_APB2_SYSCFG= (1U << 14),
    MCAL_RCC_APB2_TIM9  = (1U << 16),
    MCAL_RCC_APB2_TIM10 = (1U << 17),
    MCAL_RCC_APB2_TIM11 = (1U << 18)
} MCAL_RCC_Peripheral_t;

// Clock Bus types for RCC operations
typedef enum {
    MCAL_RCC_AHB1_BUS = 0U,
    MCAL_RCC_AHB2_BUS = 1U,
    MCAL_RCC_APB1_BUS = 2U,
    MCAL_RCC_APB2_BUS = 3U
} MCAL_RCC_Bus_t;


// EXTI Line type (corresponds to pin number 0-15)
typedef enum {
    MCAL_EXTI_LINE_0  = 0U,
    MCAL_EXTI_LINE_1  = 1U,
    MCAL_EXTI_LINE_2  = 2U,
    MCAL_EXTI_LINE_3  = 3U,
    MCAL_EXTI_LINE_4  = 4U,
    MCAL_EXTI_LINE_5  = 5U,
    MCAL_EXTI_LINE_6  = 6U,
    MCAL_EXTI_LINE_7  = 7U,
    MCAL_EXTI_LINE_8  = 8U,
    MCAL_EXTI_LINE_9  = 9U,
    MCAL_EXTI_LINE_10 = 10U,
    MCAL_EXTI_LINE_11 = 11U,
    MCAL_EXTI_LINE_12 = 12U,
    MCAL_EXTI_LINE_13 = 13U,
    MCAL_EXTI_LINE_14 = 14U,
    MCAL_EXTI_LINE_15 = 15U
} MCAL_EXTI_Line_t;

// EXTI Trigger Type
typedef enum {
    MCAL_EXTI_TRIGGER_RISING        = 0U,
    MCAL_EXTI_TRIGGER_FALLING       = 1U,
    MCAL_EXTI_TRIGGER_RISING_FALLING = 2U
} MCAL_EXTI_Trigger_t;

// EXTI Port Source (for SYSCFG_EXTICRx registers)
typedef enum {
    MCAL_EXTI_PORT_GPIOA = 0U,
    MCAL_EXTI_PORT_GPIOB = 1U,
    MCAL_EXTI_PORT_GPIOC = 2U,
    MCAL_EXTI_PORT_GPIOD = 3U,
    MCAL_EXTI_PORT_GPIOE = 4U,
    MCAL_EXTI_PORT_GPIOH = 7U // PH0, PH1 are available for EXTI
} MCAL_EXTI_PortSource_t;

/**
 * @brief Structure for ADC initialization parameters.
 */
typedef struct {
    uint32_t Resolution;     /**< Specifies the ADC resolution. */
    uint32_t ClockPrescaler; /**< Specifies the ADC clock prescaler. */
    uint32_t ConversionMode; /**< Specifies the ADC conversion mode (e.g., Single, Continuous). */
    uint32_t DataAlign;      /**< Specifies ADC data alignment. */
} MCAL_ADC_Init_t;

// ADC specific bit definitions (simplified for common use-cases)
#define ADC_CR2_ADON        (1U << 0)   // A/D Converter ON / OFF
#define ADC_CR2_SWSTART     (1U << 30)  // Start conversion of regular channels
#define ADC_SR_EOC          (1U << 1)   // End of conversion flag

// Timer type
typedef enum {
    MCAL_TIMER_1 = 0U,
    MCAL_TIMER_2,
    MCAL_TIMER_3,
    MCAL_TIMER_4,
    MCAL_TIMER_5,
    MCAL_TIMER_9,
    MCAL_TIMER_10,
    MCAL_TIMER_11,
    MCAL_NUM_TIMERS // For boundary checks
} MCAL_Timer_t;

// UART type
typedef enum {
    MCAL_UART_1 = 0U,
    MCAL_UART_2,
    MCAL_UART_6,
    MCAL_NUM_UARTS // For boundary checks
} MCAL_UART_t;

/**
 * @brief Structure for UART initialization parameters.
 */
typedef struct {
    uint32_t BaudRate;   /**< Specifies the baud rate. */
    uint32_t WordLength; /**< Specifies the number of data bits transmitted or received in a frame. */
    uint32_t StopBits;   /**< Specifies the number of stop bits transmitted. */
    uint32_t Parity;     /**< Specifies the parity mode. */
    uint32_t Mode;       /**< Specifies whether the Reciever or Transmitter mode is enabled or disabled. */
} MCAL_UART_Init_t;

// I2C Type
typedef enum {
    MCAL_I2C_1 = 0U,
    MCAL_I2C_2,
    MCAL_I2C_3,
    MCAL_NUM_I2CS // For boundary checks
} MCAL_I2C_t;

/**
 * @brief Structure for I2C initialization parameters.
 */
typedef struct {
    uint32_t ClockSpeed;     /**< Specifies the clock frequency. */
    uint32_t DutyCycle;      /**< Specifies the I2C fast mode duty cycle. */
    uint32_t OwnAddress1;    /**< Specifies the first device own address. */
    uint32_t AddressingMode; /**< Specifies the addressing mode (7-bit or 10-bit). */
    uint32_t DualAddressMode;/**< Specifies the dual addressing mode. */
    uint32_t OwnAddress2;    /**< Specifies the second device own address. */
    uint32_t GeneralCallMode;/**< Specifies the General Call mode. */
    uint32_t NoStretchMode;  /**< Specifies the No Stretch mode. */
} MCAL_I2C_Init_t;

// SPI Type
typedef enum {
    MCAL_SPI_1 = 0U,
    MCAL_SPI_2,
    MCAL_SPI_3,
    MCAL_NUM_SPIS // For boundary checks
} MCAL_SPI_t;

/**
 * @brief Structure for SPI initialization parameters.
 */
typedef struct {
    uint32_t Mode;         /**< Specifies the SPI operating mode (Master/Slave). */
    uint32_t Direction;    /**< Specifies the SPI data direction mode. */
    uint32_t DataSize;     /**< Specifies the SPI data size. */
    uint32_t CLKPolarity;  /**< Specifies the serial clock steady state. */
    uint32_t CLKPhase;     /**< Specifies the clock active edge for the bit capture. */
    uint32_t NSS;          /**< Specifies whether the NSS signal is managed by hardware (NSS pin) or by software. */
    uint32_t BaudRatePrescaler; /**< Specifies the Baud Rate prescaler value which will be used to generate the SCK clock. */
    uint32_t FirstBit;     /**< Specifies whether data transfers start from MSB or LSB. */
    uint32_t TIMode;       /**< Specifies the TI mode for SPI. */
    uint32_t CRCCalculation; /**< Specifies if the CRC calculation is enabled or disabled. */
    uint32_t CRCPolynomial;  /**< Specifies the polynomial used for the CRC calculation. */
} MCAL_SPI_Init_t;


// --- API Function Declarations ---

/**
 * @brief Initializes the Microcontroller Unit (MCU) configuration.
 *        This includes basic clock setup, Flash configuration, etc.
 * @param None
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_MCU_Config_Init(void);

/**
 * @brief Initializes a GPIO pin with the specified parameters.
 * @param p_GPIO_Init Pointer to a MCAL_GPIO_Init_t structure that contains
 *                    the configuration information for the specified GPIO pin.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_GPIO_Init(const MCAL_GPIO_Init_t *p_GPIO_Init);

/**
 * @brief Writes the specified state to a GPIO pin.
 * @param Port The GPIO port to write to (e.g., MCAL_GPIO_PORTA).
 * @param Pin The GPIO pin to write to (e.g., MCAL_GPIO_PIN_5).
 * @param State The desired state (MCAL_GPIO_LOW or MCAL_GPIO_HIGH).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_GPIO_WritePin(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin, MCAL_GPIO_PinState_t State);

/**
 * @brief Reads the input state of a GPIO pin.
 * @param Port The GPIO port to read from (e.g., MCAL_GPIO_PORTA).
 * @param Pin The GPIO pin to read (e.g., MCAL_GPIO_PIN_5).
 * @return MCAL_GPIO_PinState_t The state of the pin (MCAL_GPIO_LOW or MCAL_GPIO_HIGH).
 */
MCAL_GPIO_PinState_t MCAL_GPIO_ReadPin(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin);

/**
 * @brief Toggles the output state of a GPIO pin.
 * @param Port The GPIO port to toggle (e.g., MCAL_GPIO_PORTA).
 * @param Pin The GPIO pin to toggle (e.g., MCAL_GPIO_PIN_5).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_GPIO_TogglePin(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin);

/**
 * @brief Sets the alternate function for a GPIO pin.
 * @param Port The GPIO port (e.g., MCAL_GPIO_PORTA).
 * @param Pin The GPIO pin (e.g., MCAL_GPIO_PIN_5).
 * @param AlternateFunction The desired alternate function (e.g., MCAL_GPIO_AF1).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_GPIO_SetAlternateFunction(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin, MCAL_GPIO_AlternateFunction_t AlternateFunction);

/**
 * @brief Enables the clock for a specific peripheral.
 * @param Bus The bus the peripheral is connected to (AHB1, AHB2, APB1, APB2).
 * @param Peripheral The peripheral to enable the clock for.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_RCC_EnableClock(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral);

/**
 * @brief Disables the clock for a specific peripheral.
 * @param Bus The bus the peripheral is connected to (AHB1, AHB2, APB1, APB2).
 * @param Peripheral The peripheral to disable the clock for.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_RCC_DisableClock(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral);

/**
 * @brief Resets a specific peripheral.
 * @param Bus The bus the peripheral is connected to (AHB1, AHB2, APB1, APB2).
 * @param Peripheral The peripheral to reset.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_RCC_ResetPeripheral(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral);

/**
 * @brief Gets the clock status of a specific peripheral (e.g., if it's ready/enabled).
 * @param Bus The bus the peripheral is connected to.
 * @param Peripheral The peripheral to check.
 * @return bool True if clock is active/ready, false otherwise.
 */
bool MCAL_RCC_GetClockStatus(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral);

/**
 * @brief Initializes an EXTI line, including port source and trigger type.
 * @param Line The EXTI line number (0-15).
 * @param PortSource The GPIO port to associate with the EXTI line.
 * @param Trigger The trigger type (Rising, Falling, Rising/Falling).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_EXTI_Init(MCAL_EXTI_Line_t Line, MCAL_EXTI_PortSource_t PortSource, MCAL_EXTI_Trigger_t Trigger);

/**
 * @brief Enables the interrupt for a specific EXTI line.
 * @param Line The EXTI line number (0-15).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_EXTI_EnableInterrupt(MCAL_EXTI_Line_t Line);

/**
 * @brief Disables the interrupt for a specific EXTI line.
 * @param Line The EXTI line number (0-15).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_EXTI_DisableInterrupt(MCAL_EXTI_Line_t Line);

/**
 * @brief Sets the trigger type for a specific EXTI line.
 * @param Line The EXTI line number (0-15).
 * @param Trigger The trigger type (Rising, Falling, Rising/Falling).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_EXTI_SetTrigger(MCAL_EXTI_Line_t Line, MCAL_EXTI_Trigger_t Trigger);

/**
 * @brief Checks if an EXTI line has a pending interrupt.
 * @param Line The EXTI line number (0-15).
 * @return bool True if pending, false otherwise.
 */
bool MCAL_EXTI_GetPending(MCAL_EXTI_Line_t Line);

/**
 * @brief Clears the pending interrupt flag for a specific EXTI line.
 * @param Line The EXTI line number (0-15).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_EXTI_ClearPending(MCAL_EXTI_Line_t Line);

/**
 * @brief Initializes the ADC peripheral.
 * @param p_ADC_Init Pointer to ADC initialization structure.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_ADC_Init(const MCAL_ADC_Init_t *p_ADC_Init);

/**
 * @brief Starts ADC conversion.
 * @param None
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_ADC_StartConversion(void);

/**
 * @brief Reads the converted ADC value.
 * @param None
 * @return uint32_t The 12-bit converted ADC value.
 */
uint32_t MCAL_ADC_ReadValue(void);

/**
 * @brief Selects a specific channel for ADC conversion.
 * @param Channel The ADC channel to select (e.g., 0-18).
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_ADC_SetChannel(uint32_t Channel);

/**
 * @brief Initializes a Timer peripheral.
 * @param Timer The specific timer instance (e.g., MCAL_TIMER_2).
 * @param Prescaler The timer prescaler value.
 * @param AutoReload The auto-reload value.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_TIMER_Init(MCAL_Timer_t Timer, uint16_t Prescaler, uint32_t AutoReload);

/**
 * @brief Starts a Timer.
 * @param Timer The specific timer instance.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_TIMER_Start(MCAL_Timer_t Timer);

/**
 * @brief Stops a Timer.
 * @param Timer The specific timer instance.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_TIMER_Stop(MCAL_Timer_t Timer);

/**
 * @brief Sets the prescaler for a Timer.
 * @param Timer The specific timer instance.
 * @param Prescaler The prescaler value.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_TIMER_SetPrescaler(MCAL_Timer_t Timer, uint16_t Prescaler);

/**
 * @brief Sets the auto-reload value for a Timer.
 * @param Timer The specific timer instance.
 * @param AutoReload The auto-reload value.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_TIMER_SetAutoReload(MCAL_Timer_t Timer, uint32_t AutoReload);

/**
 * @brief Sets the capture/compare value for a specific timer channel.
 * @param Timer The specific timer instance.
 * @param Channel The timer channel (1-4, or 1-2 for TIM9, 1 for TIM10/11).
 * @param CompareValue The compare value.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_TIMER_SetCaptureCompare(MCAL_Timer_t Timer, uint8_t Channel, uint32_t CompareValue);

/**
 * @brief Reads the current counter value of a Timer.
 * @param Timer The specific timer instance.
 * @return uint32_t The current counter value.
 */
uint32_t MCAL_TIMER_ReadCounter(MCAL_Timer_t Timer);

/**
 * @brief Initializes a UART peripheral.
 * @param Uart The specific UART instance (e.g., MCAL_UART_1).
 * @param p_UART_Init Pointer to UART initialization structure.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_UART_Init(MCAL_UART_t Uart, const MCAL_UART_Init_t *p_UART_Init);

/**
 * @brief Sends a single byte of data via UART.
 * @param Uart The specific UART instance.
 * @param Data The byte to send.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_UART_SendByte(MCAL_UART_t Uart, uint8_t Data);

/**
 * @brief Receives a single byte of data via UART.
 * @param Uart The specific UART instance.
 * @return uint8_t The received byte of data.
 */
uint8_t MCAL_UART_ReceiveByte(MCAL_UART_t Uart);

/**
 * @brief Initializes an I2C peripheral.
 * @param I2c The specific I2C instance (e.g., MCAL_I2C_1).
 * @param p_I2C_Init Pointer to I2C initialization structure.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_I2C_Init(MCAL_I2C_t I2c, const MCAL_I2C_Init_t *p_I2C_Init);

/**
 * @brief Generates an I2C Start condition.
 * @param I2c The specific I2C instance.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_I2C_Start(MCAL_I2C_t I2c);

/**
 * @brief Generates an I2C Stop condition.
 * @param I2c The specific I2C instance.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_I2C_Stop(MCAL_I2C_t I2c);

/**
 * @brief Sends a byte of data over I2C.
 * @param I2c The specific I2C instance.
 * @param Data The byte to send.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_I2C_SendByte(MCAL_I2C_t I2c, uint8_t Data);

/**
 * @brief Receives a byte of data over I2C.
 * @param I2c The specific I2C instance.
 * @param Ack  Whether to send an ACK after receiving the byte (true for ACK, false for NACK).
 * @return uint8_t The received byte of data.
 */
uint8_t MCAL_I2C_ReceiveByte(MCAL_I2C_t I2c, bool Ack);

/**
 * @brief Initializes an SPI peripheral.
 * @param Spi The specific SPI instance (e.g., MCAL_SPI_1).
 * @param p_SPI_Init Pointer to SPI initialization structure.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_SPI_Init(MCAL_SPI_t Spi, const MCAL_SPI_Init_t *p_SPI_Init);

/**
 * @brief Transfers data over SPI (sends and receives a byte).
 * @param Spi The specific SPI instance.
 * @param TxData The byte to transmit.
 * @return uint8_t The received byte of data.
 */
uint8_t MCAL_SPI_Transfer(MCAL_SPI_t Spi, uint8_t TxData);

/**
 * @brief Reads or writes multiple bytes over SPI (simplified to single byte logic).
 *        For a more complete implementation, this would handle buffers.
 * @param Spi The specific SPI instance.
 * @param p_TxData Pointer to transmit data buffer (can be NULL for read-only).
 * @param p_RxData Pointer to receive data buffer (can be NULL for write-only).
 * @param Size Number of bytes to transfer.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_SPI_ReadWrite(MCAL_SPI_t Spi, const uint8_t *p_TxData, uint8_t *p_RxData, uint16_t Size);

/**
 * @brief Initializes the Watchdog Timer (WDT).
 * @param TimeoutMs The desired watchdog timeout in milliseconds.
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_WDT_Init(uint32_t TimeoutMs);

/**
 * @brief Refreshes the Watchdog Timer (WDT).
 * @param None
 * @return MCAL_Status_t MCAL_OK if successful, MCAL_ERROR otherwise.
 */
MCAL_Status_t MCAL_WDT_Refresh(void);

#endif // MCAL_H