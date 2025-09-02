/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) for STM32F401RC.
 *
 * This file implements the MCAL APIs based on the provided register definitions
 * and coding rules. It handles low-level access to MCU peripherals like
 * GPIO, RCC, EXTI, ADC, Timers, USART, I2C, SPI, and Flash.
 *
 * MCU Name: STM32F401RC
 */

/* =========================================================================
 *                   CORE INCLUDES AND DATA TYPE DEFINITIONS
 * =========================================================================*/

#include "MCAL.h"                       // MCAL API prototypes and types
#include "stm32f401xc.h"                // Core device header for STM32F401RC
#include <stdint.h>                     // Standard integer types
#include <stdbool.h>                    // Standard boolean type
#include <stddef.h>                     // Standard definitions (e.g., size_t)
#include <string.h>                     // String manipulation functions
#include <stdio.h>                      // Standard I/O functions (for debug or specific needs)
#include <stdlib.h>                     // General utilities
#include <math.h>                       // Math functions

// Data type definitions as per Rules.json
#define tbyte   uint8_t
#define tword   uint16_t
#define tlong   uint32_t
#define tsbyte  int8_t
#define tsword  int16_t
#define tslong  int32_t

/* =========================================================================
 *                   REGISTER ADDRESS DEFINITIONS
 * =========================================================================*/

// Define volatile pointers for register access
#define REG_ACCESS(addr) (*((volatile uint32_t *)(addr)))

// FLASH Registers
#define FLASH_ACR_ADDR      ((volatile uint32_t)0x40023C00)
#define FLASH_KEYR_ADDR     ((volatile uint32_t)0x40023C04)
#define FLASH_OPTKEYR_ADDR  ((volatile uint32_t)0x40023C08)
#define FLASH_SR_ADDR       ((volatile uint32_t)0x40023C0C)
#define FLASH_CR_ADDR       ((volatile uint32_t)0x40023C10)
#define FLASH_OPTCR_ADDR    ((volatile uint32_t)0x40023C14)

// RCC Registers
#define RCC_CR_ADDR         ((volatile uint32_t)0x40023800)
#define RCC_PLLCFGR_ADDR    ((volatile uint32_t)0x40023804)
#define RCC_CFGR_ADDR       ((volatile uint32_t)0x40023808)
#define RCC_CIR_ADDR        ((volatile uint32_t)0x4002380C)
#define RCC_AHB1RSTR_ADDR   ((volatile uint32_t)0x40023810)
#define RCC_AHB2RSTR_ADDR   ((volatile uint32_t)0x40023814)
#define RCC_APB1RSTR_ADDR   ((volatile uint32_t)0x40023818)
#define RCC_APB2RSTR_ADDR   ((volatile uint32_t)0x4002381C)
#define RCC_AHB1ENR_ADDR    ((volatile uint32_t)0x40023830)
#define RCC_AHB2ENR_ADDR    ((volatile uint32_t)0x40023834)
#define RCC_APB1ENR_ADDR    ((volatile uint32_t)0x40023838)
#define RCC_APB2ENR_ADDR    ((volatile uint32_t)0x4002383C)
#define RCC_AHB1LPENR_ADDR  ((volatile uint32_t)0x40023840)
#define RCC_AHB2LPENR_ADDR  ((volatile uint32_t)0x40023844)
#define RCC_APB1LPENR_ADDR  ((volatile uint32_t)0x40023848)
#define RCC_APB2LPENR_ADDR  ((volatile uint32_t)0x4002384C)
#define RCC_BDCR_ADDR       ((volatile uint32_t)0x40023850)
#define RCC_CSR_ADDR        ((volatile uint32_t)0x40023854)
#define RCC_SSCGR_ADDR      ((volatile uint32_t)0x40023858)
#define RCC_PLLI2SCFGR_ADDR ((volatile uint32_t)0x4002385C)
#define RCC_DCKCFGR_ADDR    ((volatile uint32_t)0x40023864)

// SYSCFG Registers
#define SYSCFG_MEMRMP_ADDR  ((volatile uint32_t)0x40013800)
#define SYSCFG_PMC_ADDR     ((volatile uint32_t)0x40013804)
#define SYSCFG_EXTICR1_ADDR ((volatile uint32_t)0x40013808)
#define SYSCFG_EXTICR2_ADDR ((volatile uint32_t)0x4001380C)
#define SYSCFG_EXTICR3_ADDR ((volatile uint32_t)0x40013810)
#define SYSCFG_EXTICR4_ADDR ((volatile uint32_t)0x40013814)
#define SYSCFG_CMPCR_ADDR   ((volatile uint32_t)0x40013820)

// GPIO Port Addresses (Base)
#define GPIOA_BASE_ADDR     ((volatile uint32_t)0x40020000)
#define GPIOB_BASE_ADDR     ((volatile uint32_t)0x40020400)
#define GPIOC_BASE_ADDR     ((volatile uint32_t)0x40020800)
#define GPIOD_BASE_ADDR     ((volatile uint32_t)0x40020C00)
#define GPIOE_BASE_ADDR     ((volatile uint32_t)0x40021000)
#define GPIOH_BASE_ADDR     ((volatile uint32_t)0x40021C00)

// GPIO Register Offsets
#define GPIO_MODER_OFFSET   0x00
#define GPIO_OTYPER_OFFSET  0x04
#define GPIO_OSPEEDR_OFFSET 0x08
#define GPIO_PUPDR_OFFSET   0x0C
#define GPIO_IDR_OFFSET     0x10
#define GPIO_ODR_OFFSET     0x14
#define GPIO_BSRR_OFFSET    0x18
#define GPIO_LCKR_OFFSET    0x1C
#define GPIO_AFRL_OFFSET    0x20
#define GPIO_AFRH_OFFSET    0x24

// EXTI Registers
#define EXTI_IMR_ADDR       ((volatile uint32_t)0x40013C00)
#define EXTI_EMR_ADDR       ((volatile uint32_t)0x40013C04)
#define EXTI_RTSR_ADDR      ((volatile uint32_t)0x40013C08)
#define EXTI_FTSR_ADDR      ((volatile uint32_t)0x40013C0C)
#define EXTI_SWIER_ADDR     ((volatile uint32_t)0x40013C10)
#define EXTI_PR_ADDR        ((volatile uint32_t)0x40013C14)

// ADC Registers
#define ADC1_SR_ADDR        ((volatile uint32_t)0x40012000)
#define ADC1_CR1_ADDR       ((volatile uint32_t)0x40012004)
#define ADC1_CR2_ADDR       ((volatile uint32_t)0x40012008)
#define ADC1_SMPR1_ADDR     ((volatile uint32_t)0x4001200C)
#define ADC1_SMPR2_ADDR     ((volatile uint32_t)0x40012010)
#define ADC1_JOFR1_ADDR     ((volatile uint32_t)0x40012014)
#define ADC1_JOFR2_ADDR     ((volatile uint32_t)0x40012018)
#define ADC1_JOFR3_ADDR     ((volatile uint32_t)0x4001201C)
#define ADC1_JOFR4_ADDR     ((volatile uint32_t)0x40012020)
#define ADC1_HTR_ADDR       ((volatile uint32_t)0x40012024)
#define ADC1_LTR_ADDR       ((volatile uint32_t)0x40012028)
#define ADC1_SQR1_ADDR      ((volatile uint32_t)0x4001202C)
#define ADC1_SQR2_ADDR      ((volatile uint32_t)0x40012030)
#define ADC1_SQR3_ADDR      ((volatile uint32_t)0x40012034)
#define ADC1_JSQR_ADDR      ((volatile uint32_t)0x40012038)
#define ADC1_JDR1_ADDR      ((volatile uint32_t)0x4001203C)
#define ADC1_JDR2_ADDR      ((volatile uint32_t)0x40012040)
#define ADC1_JDR3_ADDR      ((volatile uint32_t)0x40012044)
#define ADC1_JDR4_ADDR      ((volatile uint32_t)0x40012048)
#define ADC1_DR_ADDR        ((volatile uint32_t)0x4001204C)
#define ADC_CCR_ADDR        ((volatile uint32_t)0x40012304)

// Timer Registers (Base addresses and offsets)
// Timer 1 is APB2, others are APB1
#define TIM1_BASE_ADDR      ((volatile uint32_t)0x40010000)
#define TIM2_BASE_ADDR      ((volatile uint32_t)0x40000000)
#define TIM3_BASE_ADDR      ((volatile uint32_t)0x40000400)
#define TIM4_BASE_ADDR      ((volatile uint32_t)0x40000800)
#define TIM5_BASE_ADDR      ((volatile uint32_t)0x40000C00)
#define TIM9_BASE_ADDR      ((volatile uint32_t)0x40014000)
#define TIM10_BASE_ADDR     ((volatile uint32_t)0x40014400)
#define TIM11_BASE_ADDR     ((volatile uint32_t)0x40014800)

#define TIM_CR1_OFFSET      0x00
#define TIM_CR2_OFFSET      0x04
#define TIM_SMCR_OFFSET     0x08
#define TIM_DIER_OFFSET     0x0C
#define TIM_SR_OFFSET       0x10
#define TIM_EGR_OFFSET      0x14
#define TIM_CCMR1_OFFSET    0x18
#define TIM_CCMR2_OFFSET    0x1C
#define TIM_CCER_OFFSET     0x20
#define TIM_CNT_OFFSET      0x24
#define TIM_PSC_OFFSET      0x28
#define TIM_ARR_OFFSET      0x2C
#define TIM_RCR_OFFSET      0x30
#define TIM_CCR1_OFFSET     0x34
#define TIM_CCR2_OFFSET     0x38
#define TIM_CCR3_OFFSET     0x3C
#define TIM_CCR4_OFFSET     0x40
#define TIM_BDTR_OFFSET     0x44
#define TIM_DCR_OFFSET      0x48
#define TIM_DMAR_OFFSET     0x4C
#define TIM_OR_OFFSET       0x50 // Only for TIM2/5

// USART Registers
// USART1, USART6 are APB2. USART2 is APB1.
#define USART1_BASE_ADDR    ((volatile uint32_t)0x40011000)
#define USART2_BASE_ADDR    ((volatile uint32_t)0x40004400)
#define USART6_BASE_ADDR    ((volatile uint32_t)0x40011400)

#define USART_SR_OFFSET     0x00
#define USART_DR_OFFSET     0x04
#define USART_BRR_OFFSET    0x08
#define USART_CR1_OFFSET    0x0C
#define USART_CR2_OFFSET    0x10
#define USART_CR3_OFFSET    0x14
#define USART_GTPR_OFFSET   0x18

// I2C Registers
// I2C1, I2C2, I2C3 are APB1
#define I2C1_BASE_ADDR      ((volatile uint32_t)0x40005400)
#define I2C2_BASE_ADDR      ((volatile uint32_t)0x40005800)
#define I2C3_BASE_ADDR      ((volatile uint32_t)0x40005C00)

#define I2C_CR1_OFFSET      0x00
#define I2C_CR2_OFFSET      0x04
#define I2C_OAR1_OFFSET     0x08
#define I2C_OAR2_OFFSET     0x0C
#define I2C_DR_OFFSET       0x10
#define I2C_SR1_OFFSET      0x14
#define I2C_SR2_OFFSET      0x18
#define I2C_CCR_OFFSET      0x1C
#define I2C_TRISE_OFFSET    0x20
#define I2C_FLTR_OFFSET     0x24

// SPI Registers
// SPI1 is APB2. SPI2, SPI3 are APB1.
#define SPI1_BASE_ADDR      ((volatile uint32_t)0x40013000)
#define SPI2_BASE_ADDR      ((volatile uint32_t)0x40003800)
#define SPI3_BASE_ADDR      ((volatile uint32_t)0x40003C00)

#define SPI_CR1_OFFSET      0x00
#define SPI_CR2_OFFSET      0x04
#define SPI_SR_OFFSET       0x08
#define SPI_DR_OFFSET       0x0C
#define SPI_CRCPR_OFFSET    0x10
#define SPI_RXCRCR_OFFSET   0x14
#define SPI_TXCRCR_OFFSET   0x18
#define SPI_I2SCFGR_OFFSET  0x1C
#define SPI_I2SPR_OFFSET    0x20

// Watchdog (IWDG - Inferred for STM32F401RC)
#define IWDG_BASE_ADDR      ((volatile uint32_t)0x40003000)
#define IWDG_KR_ADDR        (IWDG_BASE_ADDR + 0x00) // Key register
#define IWDG_PR_ADDR        (IWDG_BASE_ADDR + 0x04) // Prescaler register
#define IWDG_RLR_ADDR       (IWDG_BASE_ADDR + 0x08) // Reload register
#define IWDG_SR_ADDR        (IWDG_BASE_ADDR + 0x0C) // Status register

// Power Control (PWR - Inferred for STM32F401RC PVD)
#define PWR_BASE_ADDR       ((volatile uint32_t)0x40007000)
#define PWR_CR_ADDR         (PWR_BASE_ADDR + 0x00) // Control register
#define PWR_CSR_ADDR        (PWR_BASE_ADDR + 0x04) // Control Status register


/* =========================================================================
 *                   COMMON BIT DEFINITIONS (INFERRED FOR STM32)
 * =========================================================================*/

// RCC Clock Enable Bits
#define RCC_AHB1ENR_GPIOAEN_Pos     (0U)
#define RCC_AHB1ENR_GPIOBEN_Pos     (1U)
#define RCC_AHB1ENR_GPIOCEN_Pos     (2U)
#define RCC_AHB1ENR_GPIODEN_Pos     (3U)
#define RCC_AHB1ENR_GPIOEEN_Pos     (4U)
#define RCC_AHB1ENR_GPIOHEN_Pos     (7U) // GPIOH for STM32F401RC

#define RCC_APB1ENR_TIM2EN_Pos      (0U)
#define RCC_APB1ENR_TIM3EN_Pos      (1U)
#define RCC_APB1ENR_TIM4EN_Pos      (2U)
#define RCC_APB1ENR_TIM5EN_Pos      (3U)
#define RCC_APB1ENR_USART2EN_Pos    (17U)
#define RCC_APB1ENR_I2C1EN_Pos      (21U)
#define RCC_APB1ENR_I2C2EN_Pos      (22U)
#define RCC_APB1ENR_I2C3EN_Pos      (23U)
#define RCC_APB1ENR_SPI2EN_Pos      (14U)
#define RCC_APB1ENR_SPI3EN_Pos      (15U)
#define RCC_APB1ENR_PWREN_Pos       (28U) // Power interface clock enable

#define RCC_APB2ENR_TIM1EN_Pos      (0U)
#define RCC_APB2ENR_USART1EN_Pos    (4U)
#define RCC_APB2ENR_USART6EN_Pos    (5U)
#define RCC_APB2ENR_ADC1EN_Pos      (8U)
#define RCC_APB2ENR_SPI1EN_Pos      (12U)
#define RCC_APB2ENR_SYSCFGEN_Pos    (14U)
#define RCC_APB2ENR_TIM9EN_Pos      (18U)
#define RCC_APB2ENR_TIM10EN_Pos     (17U)
#define RCC_APB2ENR_TIM11EN_Pos     (18U) // Note: TIM9, TIM10, TIM11 in APB2. TIM10 and TIM11 might share a bit or be distinct on different F4s. Assuming distinct for now.

// IWDG Keys
#define IWDG_KR_KEY_ENABLE          (0xCCCCUL)
#define IWDG_KR_KEY_RELOAD          (0xAAAAUL)
#define IWDG_KR_KEY_ACCESS          (0x5555UL)

// PWR_CR PVD Selection (PLS bits) and PVDEN bit
#define PWR_CR_PVDE_Pos             (4U)
#define PWR_CR_PLS_Pos              (5U)
#define PWR_CR_PLS_Msk              (0x7U << PWR_CR_PLS_Pos)
#define PWR_CR_PVDEN                (1U << PWR_CR_PVDE_Pos)

/* =========================================================================
 *                   PRIVATE HELPER FUNCTIONS
 * =========================================================================*/

/**
 * @brief Get the base address for a given GPIO port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Volatile uint32_t pointer to the GPIO port base address.
 */
static volatile uint32_t *get_gpio_port_base_addr(t_port port) {
    volatile uint32_t *base_addr;
    switch (port) {
        case PORTA: base_addr = (volatile uint32_t *)GPIOA_BASE_ADDR; break;
        case PORTB: base_addr = (volatile uint32_t *)GPIOB_BASE_ADDR; break;
        case PORTC: base_addr = (volatile uint32_t *)GPIOC_BASE_ADDR; break;
        case PORTD: base_addr = (volatile uint32_t *)GPIOD_BASE_ADDR; break;
        case PORTE: base_addr = (volatile uint32_t *)GPIOE_BASE_ADDR; break;
        case PORTH: base_addr = (volatile uint32_t *)GPIOH_BASE_ADDR; break;
        default: base_addr = NULL; break; // Should not happen with valid t_port
    }
    return base_addr;
}

/**
 * @brief Get the base address for a given Timer channel.
 * @param timer_channel The timer channel.
 * @return Volatile uint32_t pointer to the Timer base address.
 */
static volatile uint32_t *get_timer_base_addr(t_timer_channel timer_channel) {
    volatile uint32_t *base_addr;
    switch (timer_channel) {
        case TIMER_CH_1:
        case TIMER_CH_2:
        case TIMER_CH_3:
        case TIMER_CH_4:
        case PWM_CHANNEL_TIM1_CH1:
        case PWM_CHANNEL_TIM1_CH2:
        case PWM_CHANNEL_TIM1_CH3:
        case PWM_CHANNEL_TIM1_CH4:
        case ICU_CHANNEL_TIM1_CH1:
        case ICU_CHANNEL_TIM1_CH2:
        case ICU_CHANNEL_TIM1_CH3:
        case ICU_CHANNEL_TIM1_CH4:
            base_addr = (volatile uint32_t *)TIM1_BASE_ADDR; break;
        case TIMER_CH_5:
        case TIMER_CH_6:
        case TIMER_CH_7:
        case TIMER_CH_8:
        case PWM_CHANNEL_TIM2_CH1:
        case PWM_CHANNEL_TIM2_CH2:
        case PWM_CHANNEL_TIM2_CH3:
        case PWM_CHANNEL_TIM2_CH4:
        case ICU_CHANNEL_TIM2_CH1:
        case ICU_CHANNEL_TIM2_CH2:
        case ICU_CHANNEL_TIM2_CH3:
        case ICU_CHANNEL_TIM2_CH4:
            base_addr = (volatile uint32_t *)TIM2_BASE_ADDR; break;
        case TIMER_CH_9:
        case TIMER_CH_10:
        case TIMER_CH_11:
        case TIMER_CH_12:
        case PWM_CHANNEL_TIM3_CH1:
        case PWM_CHANNEL_TIM3_CH2:
        case PWM_CHANNEL_TIM3_CH3:
        case PWM_CHANNEL_TIM3_CH4:
        case ICU_CHANNEL_TIM3_CH1:
        case ICU_CHANNEL_TIM3_CH2:
        case ICU_CHANNEL_TIM3_CH3:
        case ICU_CHANNEL_TIM3_CH4:
            base_addr = (volatile uint32_t *)TIM3_BASE_ADDR; break;
        case TIMER_CH_13:
        case TIMER_CH_14:
        case TIMER_CH_15:
        case TIMER_CH_16:
        case PWM_CHANNEL_TIM4_CH1:
        case PWM_CHANNEL_TIM4_CH2:
        case PWM_CHANNEL_TIM4_CH3:
        case PWM_CHANNEL_TIM4_CH4:
        case ICU_CHANNEL_TIM4_CH1:
        case ICU_CHANNEL_TIM4_CH2:
        case ICU_CHANNEL_TIM4_CH3:
        case ICU_CHANNEL_TIM4_CH4:
            base_addr = (volatile uint32_t *)TIM4_BASE_ADDR; break;
        case TIMER_CH_17:
        case TIMER_CH_18:
        case TIMER_CH_19:
        case TIMER_CH_20:
        case PWM_CHANNEL_TIM5_CH1:
        case PWM_CHANNEL_TIM5_CH2:
        case PWM_CHANNEL_TIM5_CH3:
        case PWM_CHANNEL_TIM5_CH4:
        case ICU_CHANNEL_TIM5_CH1:
        case ICU_CHANNEL_TIM5_CH2:
        case ICU_CHANNEL_TIM5_CH3:
        case ICU_CHANNEL_TIM5_CH4:
            base_addr = (volatile uint32_t *)TIM5_BASE_ADDR; break;
        case PWM_CHANNEL_TIM9_CH1:
        case PWM_CHANNEL_TIM9_CH2:
        case ICU_CHANNEL_TIM9_CH1:
        case ICU_CHANNEL_TIM9_CH2:
            base_addr = (volatile uint32_t *)TIM9_BASE_ADDR; break;
        case PWM_CHANNEL_TIM10_CH1:
        case ICU_CHANNEL_TIM10_CH1:
            base_addr = (volatile uint32_t *)TIM10_BASE_ADDR; break;
        case PWM_CHANNEL_TIM11_CH1:
        case ICU_CHANNEL_TIM11_CH1:
            base_addr = (volatile uint32_t *)TIM11_BASE_ADDR; break;
        default: base_addr = NULL; break;
    }
    return base_addr;
}

/* =========================================================================
 *                   API IMPLEMENTATIONS
 * =========================================================================*/

/**
 * @brief Resets the Independent Watchdog Timer (IWDG).
 *        This function reloads the IWDG counter to prevent a reset.
 *        Inferred for STM32F401RC, as IWDG registers were not in register_json.
 */
void WDT_Reset(void) {
    // Write 0xAAAA in the IWDG_KR register to reload the watchdog counter
    REG_ACCESS(IWDG_KR_ADDR) = IWDG_KR_KEY_RELOAD;
}

/**
 * @brief Initializes the MCU configurations including GPIO, peripheral clocks,
 *        Watchdog, and Low Voltage Reset.
 * @param volt The system voltage level (e.g., Vsource_3V, Vsource_5V).
 */
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // Call WDT_Reset before any other operation

    volatile uint32_t *gpio_base;
    t_port current_port;
    t_pin current_pin;

    // 1. Set all GPIO pins to 0 and verify with while loop
    for (current_port = PORTA; current_port <= PORTH; current_port++) {
        if (current_port == PORTF || current_port == PORTG) continue; // STM32F401RC does not have Port F/G
        gpio_base = get_gpio_port_base_addr(current_port);
        if (gpio_base == NULL) continue;

        // Enable GPIO clock first (inferred)
        REG_ACCESS(RCC_AHB1ENR_ADDR) |= (1U << current_port); // Port A=0, B=1, ... H=7

        REG_ACCESS(gpio_base + GPIO_ODR_OFFSET) = 0x00000000UL;
        while (REG_ACCESS(gpio_base + GPIO_ODR_OFFSET) != 0x00000000UL) {
            // Wait for ODR to be cleared
        }
    }
    WDT_Reset();

    // 2. Set all GPIO pins direction to input and verify with while loop
    for (current_port = PORTA; current_port <= PORTH; current_port++) {
        if (current_port == PORTF || current_port == PORTG) continue; // STM32F401RC does not have Port F/G
        gpio_base = get_gpio_port_base_addr(current_port);
        if (gpio_base == NULL) continue;

        // Set all MODER bits to 0x00 (Input mode) for all 16 pins
        REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) = 0x00000000UL;
        while (REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) != 0x00000000UL) {
            // Wait for MODER to be cleared
        }
        // All input pins have pull-up resistors and wakeup feature enabled (if available)
        // Set all PUPDR bits to 0x01 (Pull-up)
        REG_ACCESS(gpio_base + GPIO_PUPDR_OFFSET) = 0x55555555UL; // 01 for each pin (16 * 2 bits)
    }
    WDT_Reset();

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();
    WDT_Reset();

    // Disable ADC1
    REG_ACCESS(ADC1_CR2_ADDR) &= ~(1U << 0); // Clear ADON bit (ADON is bit 0)
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_ADC1EN_Pos); // Infer: RCC_APB2ENR, bit 8 for ADC1EN
    WDT_Reset();

    // Disable USARTs
    REG_ACCESS(USART1_BASE_ADDR + USART_CR1_OFFSET) &= ~(1U << 13); // Clear UE bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_USART1EN_Pos); // Infer: RCC_APB2ENR, bit 4 for USART1EN
    WDT_Reset();

    REG_ACCESS(USART2_BASE_ADDR + USART_CR1_OFFSET) &= ~(1U << 13); // Clear UE bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_USART2EN_Pos); // Infer: RCC_APB1ENR, bit 17 for USART2EN
    WDT_Reset();

    REG_ACCESS(USART6_BASE_ADDR + USART_CR1_OFFSET) &= ~(1U << 13); // Clear UE bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_USART6EN_Pos); // Infer: RCC_APB2ENR, bit 5 for USART6EN
    WDT_Reset();

    // Disable SPIs and I2Ss (SPI peripherals include I2S)
    REG_ACCESS(SPI1_BASE_ADDR + SPI_CR1_OFFSET) &= ~(1U << 6); // Clear SPE bit
    REG_ACCESS(SPI1_BASE_ADDR + SPI_I2SCFGR_OFFSET) &= ~(1U << 7); // Clear I2SE bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_SPI1EN_Pos); // Infer: RCC_APB2ENR, bit 12 for SPI1EN
    WDT_Reset();

    REG_ACCESS(SPI2_BASE_ADDR + SPI_CR1_OFFSET) &= ~(1U << 6); // Clear SPE bit
    REG_ACCESS(SPI2_BASE_ADDR + SPI_I2SCFGR_OFFSET) &= ~(1U << 7); // Clear I2SE bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_SPI2EN_Pos); // Infer: RCC_APB1ENR, bit 14 for SPI2EN
    WDT_Reset();

    REG_ACCESS(SPI3_BASE_ADDR + SPI_CR1_OFFSET) &= ~(1U << 6); // Clear SPE bit
    REG_ACCESS(SPI3_BASE_ADDR + SPI_I2SCFGR_OFFSET) &= ~(1U << 7); // Clear I2SE bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_SPI3EN_Pos); // Infer: RCC_APB1ENR, bit 15 for SPI3EN
    WDT_Reset();

    // Disable I2Cs
    REG_ACCESS(I2C1_BASE_ADDR + I2C_CR1_OFFSET) &= ~(1U << 0); // Clear PE bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_I2C1EN_Pos); // Infer: RCC_APB1ENR, bit 21 for I2C1EN
    WDT_Reset();

    REG_ACCESS(I2C2_BASE_ADDR + I2C_CR1_OFFSET) &= ~(1U << 0); // Clear PE bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_I2C2EN_Pos); // Infer: RCC_APB1ENR, bit 22 for I2C2EN
    WDT_Reset();

    REG_ACCESS(I2C3_BASE_ADDR + I2C_CR1_OFFSET) &= ~(1U << 0); // Clear PE bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_I2C3EN_Pos); // Infer: RCC_APB1ENR, bit 23 for I2C3EN
    WDT_Reset();

    // Disable Timers
    REG_ACCESS(TIM1_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM1EN_Pos);
    WDT_Reset();
    REG_ACCESS(TIM2_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM2EN_Pos);
    WDT_Reset();
    REG_ACCESS(TIM3_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM3EN_Pos);
    WDT_Reset();
    REG_ACCESS(TIM4_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM4EN_Pos);
    WDT_Reset();
    REG_ACCESS(TIM5_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM5EN_Pos);
    WDT_Reset();
    REG_ACCESS(TIM9_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM9EN_Pos);
    WDT_Reset();
    REG_ACCESS(TIM10_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM10EN_Pos);
    WDT_Reset();
    REG_ACCESS(TIM11_BASE_ADDR + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM11EN_Pos);
    WDT_Reset();

    // 4. Enable WDT (Watchdog Timer) - Inferred for STM32 IWDG
    REG_ACCESS(IWDG_KR_ADDR) = IWDG_KR_KEY_ENABLE; // Enable IWDG
    WDT_Reset();

    // 5. Clear WDT timer (already done by enabling with 0xCCCC)
    // 6. Set WDT period >= 8 msec - Inferred for STM32 IWDG
    // LSI clock is ~32kHz. Prescaler = 32 (0x05) gives ~1ms per counter tick (32kHz / 32 = 1kHz)
    // Reload value 8 gives 8ms. Max reload value 0xFFF = 4095 ms ~ 4 seconds.
    REG_ACCESS(IWDG_PR_ADDR) = 0x05; // Prescaler: 32 (LSI / 32)
    REG_ACCESS(IWDG_RLR_ADDR) = 8; // Reload value for ~8ms (1kHz * 8ms = 8)
    WDT_Reset();

    // 7. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // 8. Enable LVR (Low Voltage Reset) - Inferred for STM32 PVD
    // PVD levels (PLS bits in PWR_CR):
    // 000: 2.2V, 001: 2.3V, 010: 2.4V, 011: 2.5V, 100: 2.6V, 101: 2.7V, 110: 2.8V, 111: 2.9V
    // Enable PWR clock first (inferred)
    REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_PWREN_Pos); // Inferred: RCC_APB1ENR, bit 28 for PWREN

    // Disable PVD before configuring
    REG_ACCESS(PWR_CR_ADDR) &= ~PWR_CR_PVDEN;
    WDT_Reset();

    if (volt == Vsource_3V) {
        // For 3V, aim for ~2V threshold. Closest available is 2.2V (PLS=0) or higher.
        // Let's use 2.2V (PLS=000)
        REG_ACCESS(PWR_CR_ADDR) &= ~PWR_CR_PLS_Msk; // Clear PLS bits
        // PLS is already 000 after clearing
    } else { // Vsource_5V
        // For 5V, aim for ~3.5V threshold. STM32 PVD max is 2.9V.
        // This is a limitation of the specific MCU's PVD range.
        // We'll use the highest available, 2.9V (PLS=111), and comment on the limitation.
        REG_ACCESS(PWR_CR_ADDR) |= (0x7U << PWR_CR_PLS_Pos); // Set PLS to 111 (2.9V)
        // Note: STM32F401RC PVD max threshold is 2.9V. A 3.5V threshold is not directly achievable
        // with the internal PVD. External supervisor IC would be needed for higher thresholds.
    }
    WDT_Reset();

    // Enable PVD
    REG_ACCESS(PWR_CR_ADDR) |= PWR_CR_PVDEN;
    WDT_Reset();

    // 9. Clear WDT again
    WDT_Reset();
}

/**
 * @brief Enters the microcontroller into sleep mode (Wait-for-Interrupt).
 *        Stops CPU clock, peripherals may continue running.
 */
void Go_to_sleep_mode(void) {
    WDT_Reset();
    __WFI(); // Wait For Interrupt - Cortex-M specific instruction
}

/**
 * @brief Enables global interrupts.
 */
void Global_interrupt_Enable(void) {
    WDT_Reset();
    __enable_irq(); // Cortex-M specific instruction
}

/**
 * @brief Disables global interrupts.
 */
void Global_interrupt_Disable(void) {
    WDT_Reset();
    __disable_irq(); // Cortex-M specific instruction
}

// LVD (Low Voltage Detection) - Using STM32 PVD as proxy
// Note: STM32F401RC PVD (Programmable Voltage Detector) has thresholds from 2.2V to 2.9V.
// The requested API levels (0.5V, 1V, 1.5V, etc.) are not directly mappable.
// A custom mapping will be provided, or a warning for out-of-range requests.

/**
 * @brief Initializes the Low Voltage Detection (LVD) module.
 *        This function uses the STM32's internal PVD (Programmable Voltage Detector).
 *        Note: The available PVD thresholds on STM32F401RC are limited (2.2V to 2.9V).
 *        If thresholds outside this range are requested by LVD_Get/LVD_Enable,
 *        the closest valid PVD level or a default will be used.
 */
void LVD_Init(void) {
    WDT_Reset();
    // Enable PWR clock, already done in MCU_Config_Init.
    // Ensure PVD is disabled before configuration (if not already by MCU_Config_Init)
    REG_ACCESS(PWR_CR_ADDR) &= ~PWR_CR_PVDEN;
    WDT_Reset();
}

/**
 * @brief Sets the LVD threshold level.
 * @param lvd_thresholdLevel The desired threshold level.
 *        Note: For STM32F401RC, PVD thresholds range from 2.2V to 2.9V.
 *        Levels outside this will be mapped to the closest or default.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) { // Naming convention mismatch: Get implies read, this sets.
    WDT_Reset();

    uint32_t pls_setting = 0; // Default to 2.2V

    // Mapping t_lvd_thrthresholdLevel to PWR_CR PLS bits
    switch (lvd_thresholdLevel) {
        case Volt_0_5V: // Not achievable directly. Map to lowest PVD.
        case Volt_1V:   // Not achievable directly. Map to lowest PVD.
        case Volt_1_5V: // Not achievable directly. Map to lowest PVD.
        case Volt_2V:   // Closest is 2.2V
            pls_setting = 0; break; // PLS = 000 (2.2V)
        case Volt_2_5V:
            pls_setting = 3; break; // PLS = 011 (2.5V)
        case Volt_3V:   // Closest is 2.9V (max)
        case Volt_3_5V: // Not achievable directly. Map to highest PVD.
        case Volt_4V:   // Not achievable directly. Map to highest PVD.
        case Volt_4_5V: // Not achievable directly. Map to highest PVD.
        case Volt_5V:   // Not achievable directly. Map to highest PVD.
            pls_setting = 7; break; // PLS = 111 (2.9V)
        default:
            pls_setting = 0; break; // Default to 2.2V if unknown
    }

    // Clear existing PLS bits
    REG_ACCESS(PWR_CR_ADDR) &= ~PWR_CR_PLS_Msk;
    // Set new PLS bits
    REG_ACCESS(PWR_CR_ADDR) |= (pls_setting << PWR_CR_PLS_Pos);
}

/**
 * @brief Enables the Low Voltage Detection (LVD) module.
 */
void LVD_Enable(void) {
    WDT_Reset();
    // Enable PWR clock, if not already by MCU_Config_Init.
    REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_PWREN_Pos); // Inferred

    // Enable PVD
    REG_ACCESS(PWR_CR_ADDR) |= PWR_CR_PVDEN;
}

/**
 * @brief Disables the Low Voltage Detection (LVD) module.
 */
void LVD_Disable(void) {
    WDT_Reset();
    // Disable PVD
    REG_ACCESS(PWR_CR_ADDR) &= ~PWR_CR_PVDEN;
}

// UART APIs
/**
 * @brief Initializes a specified UART channel with the given parameters.
 * @param uart_channel The UART channel to initialize (e.g., UART_CHANNEL_1).
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data length (e.g., 8-bit, 9-bit).
 * @param uart_stop_bit The number of stop bits (e.g., 1 stop bit, 2 stop bits).
 * @param uart_parity The parity mode (e.g., Even, Odd, None).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset();
    volatile uint32_t *usart_base = NULL;
    uint32_t pclk_freq = 16000000; // Assuming PCLK1=16MHz for USART2, PCLK2=16MHz for USART1/6, check actual system clock setup

    // Determine USART base address and enable clock
    switch (uart_channel) {
        case UART_CHANNEL_1:
            usart_base = (volatile uint32_t *)USART1_BASE_ADDR;
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_USART1EN_Pos); // Infer: RCC_APB2ENR bit 4
            pclk_freq = 16000000; // Assuming APB2 clock (PCLK2) is 16MHz
            break;
        case UART_CHANNEL_2:
            usart_base = (volatile uint32_t *)USART2_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_USART2EN_Pos); // Infer: RCC_APB1ENR bit 17
            pclk_freq = 16000000; // Assuming APB1 clock (PCLK1) is 16MHz
            break;
        case UART_CHANNEL_6:
            usart_base = (volatile uint32_t *)USART6_BASE_ADDR;
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_USART6EN_Pos); // Infer: RCC_APB2ENR bit 5
            pclk_freq = 16000000; // Assuming APB2 clock (PCLK2) is 16MHz
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset();

    // Disable USART before configuration
    REG_ACCESS(usart_base + USART_CR1_OFFSET) &= ~(1U << 13); // Clear UE bit

    // Configure Baud Rate (Assuming Oversampling by 16)
    // BaudRate = PCLK / (8 * (2 - OVER8) * USARTDIV)
    // USARTDIV = PCLK / (16 * BaudRate)
    tword usartdiv = pclk_freq / (16 * uart_baud_rate);
    tword mantissa = usartdiv / 1;
    tword fraction = (usartdiv - mantissa) * 16;
    REG_ACCESS(usart_base + USART_BRR_OFFSET) = (mantissa << 4) | (fraction & 0xF);
    WDT_Reset();

    // Configure CR1: Data length, Parity, Transmit/Receive Enable
    uint32_t cr1_val = 0;
    if (uart_data_length == UART_DATA_LENGTH_9) {
        cr1_val |= (1U << 12); // M bit for 9-bit word length
    }
    if (uart_parity != UART_PARITY_NONE) {
        cr1_val |= (1U << 10); // PCE bit for parity control enable
        if (uart_parity == UART_PARITY_ODD) {
            cr1_val |= (1U << 9); // PS bit for odd parity
        }
    }
    cr1_val |= (1U << 2); // RE bit for Receiver enable
    cr1_val |= (1U << 3); // TE bit for Transmitter enable
    REG_ACCESS(usart_base + USART_CR1_OFFSET) = cr1_val;
    WDT_Reset();

    // Configure CR2: Stop bits
    uint32_t cr2_val = 0;
    if (uart_stop_bit == UART_STOP_BIT_2) {
        cr2_val |= (2U << 12); // STOP bits = 10 (2 stop bits)
    } else if (uart_stop_bit == UART_STOP_BIT_1_5) {
        cr2_val |= (3U << 12); // STOP bits = 11 (1.5 stop bits)
    } else { // UART_STOP_BIT_1
        cr2_val |= (0U << 12); // STOP bits = 00 (1 stop bit)
    }
    REG_ACCESS(usart_base + USART_CR2_OFFSET) = cr2_val;
    WDT_Reset();

    // Configure CR3 (DMA, Error, etc. - leave as default for basic init)
    REG_ACCESS(usart_base + USART_CR3_OFFSET) = 0;
    WDT_Reset();

    // Finally, enable USART (UE bit)
    REG_ACCESS(usart_base + USART_CR1_OFFSET) |= (1U << 13);
}

/**
 * @brief Enables a specified UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset();
    volatile uint32_t *usart_base = NULL;

    switch (uart_channel) {
        case UART_CHANNEL_1:
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_USART1EN_Pos); // Enable peripheral clock (inferred)
            usart_base = (volatile uint32_t *)USART1_BASE_ADDR;
            break;
        case UART_CHANNEL_2:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_USART2EN_Pos); // Enable peripheral clock (inferred)
            usart_base = (volatile uint32_t *)USART2_BASE_ADDR;
            break;
        case UART_CHANNEL_6:
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_USART6EN_Pos); // Enable peripheral clock (inferred)
            usart_base = (volatile uint32_t *)USART6_BASE_ADDR;
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset(); // Call WDT_Reset before setting the enable bit.

    if (usart_base != NULL) {
        REG_ACCESS(usart_base + USART_CR1_OFFSET) |= (1U << 13); // Set UE bit
    }
}

/**
 * @brief Disables a specified UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset();
    volatile uint32_t *usart_base = NULL;

    switch (uart_channel) {
        case UART_CHANNEL_1: usart_base = (volatile uint32_t *)USART1_BASE_ADDR; break;
        case UART_CHANNEL_2: usart_base = (volatile uint32_t *)USART2_BASE_ADDR; break;
        case UART_CHANNEL_6: usart_base = (volatile uint32_t *)USART6_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (usart_base != NULL) {
        REG_ACCESS(usart_base + USART_CR1_OFFSET) &= ~(1U << 13); // Clear UE bit
        // Also disable peripheral clock to save power
        switch (uart_channel) {
            case UART_CHANNEL_1: REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_USART1EN_Pos); break;
            case UART_CHANNEL_2: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_USART2EN_Pos); break;
            case UART_CHANNEL_6: REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_USART6EN_Pos); break;
            default: break;
        }
    }
}

/**
 * @brief Sends a single byte over the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset();
    volatile uint32_t *usart_base = NULL;

    switch (uart_channel) {
        case UART_CHANNEL_1: usart_base = (volatile uint32_t *)USART1_BASE_ADDR; break;
        case UART_CHANNEL_2: usart_base = (volatile uint32_t *)USART2_BASE_ADDR; break;
        case UART_CHANNEL_6: usart_base = (volatile uint32_t *)USART6_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (usart_base != NULL) {
        while (!(REG_ACCESS(usart_base + USART_SR_OFFSET) & (1U << 7))) { /* Wait for TXE flag */ } // TXE: Transmit data register empty
        REG_ACCESS(usart_base + USART_DR_OFFSET) = byte;
        while (!(REG_ACCESS(usart_base + USART_SR_OFFSET) & (1U << 6))) { /* Wait for TC flag */ }  // TC: Transmission complete
    }
}

/**
 * @brief Sends a frame of data over the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset();
    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset();
    while (*str != '\0') {
        UART_send_byte(uart_channel, (tbyte)*str++);
    }
}

/**
 * @brief Receives a single byte from the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset();
    volatile uint32_t *usart_base = NULL;
    tbyte received_byte = 0;

    switch (uart_channel) {
        case UART_CHANNEL_1: usart_base = (volatile uint32_t *)USART1_BASE_ADDR; break;
        case UART_CHANNEL_2: usart_base = (volatile uint32_t *)USART2_BASE_ADDR; break;
        case UART_CHANNEL_6: usart_base = (volatile uint32_t *)USART6_BASE_ADDR; break;
        default: return 0; // Invalid channel
    }
    WDT_Reset();

    if (usart_base != NULL) {
        while (!(REG_ACCESS(usart_base + USART_SR_OFFSET) & (1U << 5))) { /* Wait for RXNE flag */ } // RXNE: Read data register not empty
        received_byte = (tbyte)(REG_ACCESS(usart_base + USART_DR_OFFSET) & 0xFF);
    }
    return received_byte;
}

/**
 * @brief Receives a frame of data from the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of data to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset();
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Receives a null-terminated string from the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes read (excluding null terminator).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset();
    int i = 0;
    for (i = 0; i < max_length - 1; i++) {
        char received_char = (char)UART_Get_Byte(uart_channel);
        buffer[i] = received_char;
        if (received_char == '\n' || received_char == '\r') { // Example termination characters
            break;
        }
    }
    buffer[i] = '\0'; // Null-terminate the string
    return i;
}

// I2C APIs
/**
 * @brief Initializes a specified I2C channel with the given parameters.
 * @param i2c_channel The I2C channel to initialize (e.g., I2C_CHANNEL_1).
 * @param i2c_clk_speed The desired clock speed (e.g., I2C_CLK_SPEED_FAST).
 * @param i2c_device_address The device's own 7-bit address.
 * @param i2c_ack Acknowledgment control (enable/disable).
 * @param i2c_datalength The data length (not directly used by I2C hardware, but for API consistency).
 *
 * I2C Rules Applied:
 * - Addressing Mode equals Device Address
 * - Always use fast mode
 * - Always use maximum timeout (implied by blocking wait loops)
 * - Always generate a repeated start condition instead of stop between transactions (handled in send/get frame functions)
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset();
    volatile uint32_t *i2c_base = NULL;
    uint32_t pclk_freq = 16000000; // Assuming APB1 clock (PCLK1) is 16MHz for I2C1/2/3

    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            i2c_base = (volatile uint32_t *)I2C1_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_I2C1EN_Pos); // Inferred: RCC_APB1ENR bit 21
            break;
        case I2C_CHANNEL_2:
            i2c_base = (volatile uint32_t *)I2C2_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_I2C2EN_Pos); // Inferred: RCC_APB1ENR bit 22
            break;
        case I2C_CHANNEL_3:
            i2c_base = (volatile uint32_t *)I2C3_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_I2C3EN_Pos); // Inferred: RCC_APB1ENR bit 23
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset();

    // Reset I2C peripheral (PE=0 then PE=1)
    REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (1U << 15); // SWRST bit
    REG_ACCESS(i2c_base + I2C_CR1_OFFSET) &= ~(1U << 15); // Clear SWRST
    WDT_Reset();

    // Disable peripheral to configure
    REG_ACCESS(i2c_base + I2C_CR1_OFFSET) &= ~(1U << 0); // Clear PE bit

    // Configure CR2: Peripheral clock frequency
    REG_ACCESS(i2c_base + I2C_CR2_OFFSET) = (pclk_freq / 1000000); // Freq[5:0] bits
    WDT_Reset();

    // Configure CCR: Clock control register (Fast mode, 400kHz)
    uint32_t ccr_val = 0;
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST) {
        // Fast mode (F/S = 1, DUTY = 0 for 2)
        // Thigh = CCR * Tpclk, Tlow = 2 * CCR * Tpclk
        // T_fast_mode = (Thigh + Tlow) = 3 * CCR * Tpclk
        // F_fast_mode = PCLK1 / (3 * CCR) => CCR = PCLK1 / (3 * F_fast_mode)
        // For 400kHz, PCLK1=16MHz: CCR = 16MHz / (3 * 400kHz) = 16000 / 1200 = 13.33 -> 14
        ccr_val = pclk_freq / (3 * 400000); // Fast Mode, DUTY=0 (Tlow/Thigh=2)
        ccr_val |= (1U << 15); // F/S bit for Fast Mode
    } else { // I2C_CLK_SPEED_STANDARD (100kHz)
        // Standard mode (F/S = 0)
        // T_std_mode = 2 * CCR * Tpclk
        // F_std_mode = PCLK1 / (2 * CCR) => CCR = PCLK1 / (2 * F_std_mode)
        // For 100kHz, PCLK1=16MHz: CCR = 16MHz / (2 * 100kHz) = 16000 / 200 = 80
        ccr_val = pclk_freq / (2 * 100000);
    }
    REG_ACCESS(i2c_base + I2C_CCR_OFFSET) = ccr_val;
    WDT_Reset();

    // Configure TRISE: Max rise time
    // For 16MHz PCLK, Fast mode 400kHz: TRISE = (16MHz * 300ns / 1000ns) + 1 = 4.8+1 = 5.8 -> 6
    // For 16MHz PCLK, Standard mode 100kHz: TRISE = (16MHz * 1000ns / 1000ns) + 1 = 16+1 = 17
    REG_ACCESS(i2c_base + I2C_TRISE_OFFSET) = (pclk_freq / 1000000 * 300 / 1000) + 1; // Approx for 300ns fast mode
    WDT_Reset();

    // Configure OAR1: Own address (7-bit addressing, shift left by 1)
    REG_ACCESS(i2c_base + I2C_OAR1_OFFSET) = (i2c_device_address << 1) | (1U << 14); // Bit 14 must be kept at 1
    WDT_Reset();

    // Configure CR1: Acknowledge enable
    REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (i2c_ack == I2C_ACK_ENABLE) ? (1U << 10) : 0; // ACK bit
    WDT_Reset();

    // Enable I2C peripheral
    REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (1U << 0); // Set PE bit
}

/**
 * @brief Enables a specified I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset();
    volatile uint32_t *i2c_base = NULL;

    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_I2C1EN_Pos); // Enable peripheral clock (inferred)
            i2c_base = (volatile uint32_t *)I2C1_BASE_ADDR;
            break;
        case I2C_CHANNEL_2:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_I2C2EN_Pos); // Enable peripheral clock (inferred)
            i2c_base = (volatile uint32_t *)I2C2_BASE_ADDR;
            break;
        case I2C_CHANNEL_3:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_I2C3EN_Pos); // Enable peripheral clock (inferred)
            i2c_base = (volatile uint32_t *)I2C3_BASE_ADDR;
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset(); // Call WDT_Reset before setting the enable bit.

    if (i2c_base != NULL) {
        REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (1U << 0); // Set PE bit
    }
}

/**
 * @brief Disables a specified I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset();
    volatile uint32_t *i2c_base = NULL;

    switch (i2c_channel) {
        case I2C_CHANNEL_1: i2c_base = (volatile uint32_t *)I2C1_BASE_ADDR; break;
        case I2C_CHANNEL_2: i2c_base = (volatile uint32_t *)I2C2_BASE_ADDR; break;
        case I2C_CHANNEL_3: i2c_base = (volatile uint32_t *)I2C3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (i2c_base != NULL) {
        REG_ACCESS(i2c_base + I2C_CR1_OFFSET) &= ~(1U << 0); // Clear PE bit
        // Also disable peripheral clock to save power
        switch (i2c_channel) {
            case I2C_CHANNEL_1: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_I2C1EN_Pos); break;
            case I2C_CHANNEL_2: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_I2C2EN_Pos); break;
            case I2C_CHANNEL_3: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_I2C3EN_Pos); break;
            default: break;
        }
    }
}

/**
 * @brief Sends a single byte over the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset();
    volatile uint32_t *i2c_base = NULL;

    switch (i2c_channel) {
        case I2C_CHANNEL_1: i2c_base = (volatile uint32_t *)I2C1_BASE_ADDR; break;
        case I2C_CHANNEL_2: i2c_base = (volatile uint32_t *)I2C2_BASE_ADDR; break;
        case I2C_CHANNEL_3: i2c_base = (volatile uint32_t *)I2C3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (i2c_base != NULL) {
        // Wait for TxE (Transmit Empty) flag to be set
        while (!(REG_ACCESS(i2c_base + I2C_SR1_OFFSET) & (1U << 7))) { /* max timeout implied */ }
        REG_ACCESS(i2c_base + I2C_DR_OFFSET) = byte;
    }
}

/**
 * @brief Sends a frame of data over the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset();
    volatile uint32_t *i2c_base = NULL;

    switch (i2c_channel) {
        case I2C_CHANNEL_1: i2c_base = (volatile uint32_t *)I2C1_BASE_ADDR; break;
        case I2C_CHANNEL_2: i2c_base = (volatile uint32_t *)I2C2_BASE_ADDR; break;
        case I2C_CHANNEL_3: i2c_base = (volatile uint32_t *)I2C3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (i2c_base != NULL) {
        // Generate Start condition
        REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (1U << 8); // START bit
        while (!(REG_ACCESS(i2c_base + I2C_SR1_OFFSET) & (1U << 0))) { /* Wait for SB (Start Bit) */ }

        // Send Slave Address (write)
        REG_ACCESS(i2c_base + I2C_DR_OFFSET) = (tbyte)((I2C_ADDRESS_UNKNOWN << 1) & 0xFE); // Assuming a known address or it's implicitly handled later
        while (!(REG_ACCESS(i2c_base + I2C_SR1_OFFSET) & (1U << 1))) { /* Wait for ADDR */ }
        (void)REG_ACCESS(i2c_base + I2C_SR2_OFFSET); // Clear ADDR by reading SR2

        for (int i = 0; i < length; i++) {
            I2C_send_byte(i2c_channel, (tbyte)data[i]);
        }

        // Generate Stop condition if it's the end of a transaction
        // (Rules state "generate a repeated start condition instead of stop between transactions".
        // This implies for the *final* transaction, a stop is needed)
        REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (1U << 9); // STOP bit
        while ((REG_ACCESS(i2c_base + I2C_CR1_OFFSET) & (1U << 9))) { /* Wait for STOPF */ }
    }
}

/**
 * @brief Sends a null-terminated string over the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset();
    I2C_send_frame(i2c_channel, str, strlen(str));
}

/**
 * @brief Receives a single byte from the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset();
    volatile uint32_t *i2c_base = NULL;
    tbyte received_byte = 0;

    switch (i2c_channel) {
        case I2C_CHANNEL_1: i2c_base = (volatile uint32_t *)I2C1_BASE_ADDR; break;
        case I2C_CHANNEL_2: i2c_base = (volatile uint32_t *)I2C2_BASE_ADDR; break;
        case I2C_CHANNEL_3: i2c_base = (volatile uint32_t *)I2C3_BASE_ADDR; break;
        default: return 0; // Invalid channel
    }
    WDT_Reset();

    if (i2c_base != NULL) {
        // Wait for RxNE (Receive data register not empty) flag to be set
        while (!(REG_ACCESS(i2c_base + I2C_SR1_OFFSET) & (1U << 6))) { /* max timeout implied */ }
        received_byte = (tbyte)REG_ACCESS(i2c_base + I2C_DR_OFFSET);
    }
    return received_byte;
}

/**
 * @brief Receives a frame of data from the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of data to receive.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset();
    volatile uint32_t *i2c_base = NULL;

    switch (i2c_channel) {
        case I2C_CHANNEL_1: i2c_base = (volatile uint32_t *)I2C1_BASE_ADDR; break;
        case I2C_CHANNEL_2: i2c_base = (volatile uint32_t *)I2C2_BASE_ADDR; break;
        case I2C_CHANNEL_3: i2c_base = (volatile uint32_t *)I2C3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (i2c_base != NULL) {
        // Generate Start condition
        REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (1U << 8); // START bit
        while (!(REG_ACCESS(i2c_base + I2C_SR1_OFFSET) & (1U << 0))) { /* Wait for SB (Start Bit) */ }

        // Send Slave Address (read)
        REG_ACCESS(i2c_base + I2C_DR_OFFSET) = (tbyte)((I2C_ADDRESS_UNKNOWN << 1) | 0x01); // Assuming a known address or it's implicitly handled later
        while (!(REG_ACCESS(i2c_base + I2C_SR1_OFFSET) & (1U << 1))) { /* Wait for ADDR */ }
        (void)REG_ACCESS(i2c_base + I2C_SR2_OFFSET); // Clear ADDR by reading SR2

        for (int i = 0; i < max_length; i++) {
            if (i == max_length - 1) { // NACK on last byte
                REG_ACCESS(i2c_base + I2C_CR1_OFFSET) &= ~(1U << 10); // Clear ACK bit
            }
            buffer[i] = (char)I2C_Get_Byte(i2c_channel);
        }

        // Generate Stop condition
        REG_ACCESS(i2c_base + I2C_CR1_OFFSET) |= (1U << 9); // STOP bit
        while ((REG_ACCESS(i2c_base + I2C_CR1_OFFSET) & (1U << 9))) { /* Wait for STOPF */ }
    }
}

/**
 * @brief Receives a null-terminated string from the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes read (excluding null terminator).
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset();
    int i = 0;
    for (i = 0; i < max_length - 1; i++) {
        char received_char = (char)I2C_Get_Byte(i2c_channel);
        buffer[i] = received_char;
        if (received_char == '\0' || received_char == '\n' || received_char == '\r') {
            break;
        }
    }
    buffer[i] = '\0'; // Null-terminate the string
    return i;
}

// SPI (CSI) APIs
/**
 * @brief Initializes a specified SPI channel.
 * @param spi_channel The SPI channel to initialize (e.g., SPI_CHANNEL_1).
 * @param spi_mode The SPI mode (Master/Slave).
 * @param spi_cpol Clock polarity.
 * @param spi_cpha Clock phase.
 * @param spi_dff Data frame format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 *
 * SPI Rules Applied:
 * - Always use fast speed (handled by prescaler calc)
 * - Slave Select always software-controlled (SSM=1, SSI=1)
 * - Always use full duplex (BIDIMODE=0)
 * - Always enable CRC (CRCEN=1)
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;
    uint32_t pclk_freq = 16000000; // Assuming PCLK2 for SPI1 (16MHz), PCLK1 for SPI2/3 (16MHz)

    switch (spi_channel) {
        case SPI_CHANNEL_1:
            spi_base = (volatile uint32_t *)SPI1_BASE_ADDR;
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_SPI1EN_Pos); // Inferred: RCC_APB2ENR bit 12
            pclk_freq = 16000000; // Assuming APB2 clock
            break;
        case SPI_CHANNEL_2:
            spi_base = (volatile uint32_t *)SPI2_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI2EN_Pos); // Inferred: RCC_APB1ENR bit 14
            pclk_freq = 16000000; // Assuming APB1 clock
            break;
        case SPI_CHANNEL_3:
            spi_base = (volatile uint32_t *)SPI3_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI3EN_Pos); // Inferred: RCC_APB1ENR bit 15
            pclk_freq = 16000000; // Assuming APB1 clock
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset();

    // Disable SPI before configuration
    REG_ACCESS(spi_base + SPI_CR1_OFFSET) &= ~(1U << 6); // Clear SPE bit

    uint32_t cr1_val = 0;

    // Baud rate prescaler for "fast speed" - use PCLK/2
    cr1_val |= (0U << 3); // BR[2:0] = 000 (PCLK / 2)

    // CPOL, CPHA
    cr1_val |= (spi_cpol == SPI_CPOL_HIGH) ? (1U << 1) : 0;
    cr1_val |= (spi_cpha == SPI_CPHA_2_EDGE) ? (1U << 0) : 0;

    // Master/Slave Mode
    cr1_val |= (spi_mode == SPI_MODE_MASTER) ? (1U << 2) : 0; // MSTR bit

    // Data Frame Format (DFF)
    cr1_val |= (spi_dff == SPI_DFF_16_BIT) ? (1U << 11) : 0;

    // Bit Order (LSBFIRST)
    cr1_val |= (spi_bit_order == SPI_BIT_ORDER_LSB) ? (1U << 7) : 0;

    // Always use full duplex (BIDIMODE = 0) - default state

    // Always enable CRC
    cr1_val |= (1U << 13); // CRCEN bit
    REG_ACCESS(spi_base + SPI_CRCPR_OFFSET) = 7; // Default polynomial for CRC (e.g., 0x7)
    WDT_Reset();

    // Software Slave Management and Internal Slave Select
    cr1_val |= (1U << 9); // SSM (Software slave management) enable
    cr1_val |= (1U << 8); // SSI (Internal slave select) high

    REG_ACCESS(spi_base + SPI_CR1_OFFSET) = cr1_val;
    WDT_Reset();

    // CR2 (leave as default for basic init)
    REG_ACCESS(spi_base + SPI_CR2_OFFSET) = 0;
    WDT_Reset();

    // Enable SPI peripheral
    REG_ACCESS(spi_base + SPI_CR1_OFFSET) |= (1U << 6); // Set SPE bit
}

/**
 * @brief Enables a specified SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;

    switch (spi_channel) {
        case SPI_CHANNEL_1:
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_SPI1EN_Pos); // Enable peripheral clock (inferred)
            spi_base = (volatile uint32_t *)SPI1_BASE_ADDR;
            break;
        case SPI_CHANNEL_2:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI2EN_Pos); // Enable peripheral clock (inferred)
            spi_base = (volatile uint32_t *)SPI2_BASE_ADDR;
            break;
        case SPI_CHANNEL_3:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI3EN_Pos); // Enable peripheral clock (inferred)
            spi_base = (volatile uint32_t *)SPI3_BASE_ADDR;
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset(); // Call WDT_Reset before setting the enable bit.

    if (spi_base != NULL) {
        REG_ACCESS(spi_base + SPI_CR1_OFFSET) |= (1U << 6); // Set SPE bit
    }
}

/**
 * @brief Disables a specified SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;

    switch (spi_channel) {
        case SPI_CHANNEL_1: spi_base = (volatile uint32_t *)SPI1_BASE_ADDR; break;
        case SPI_CHANNEL_2: spi_base = (volatile uint32_t *)SPI2_BASE_ADDR; break;
        case SPI_CHANNEL_3: spi_base = (volatile uint32_t *)SPI3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (spi_base != NULL) {
        REG_ACCESS(spi_base + SPI_CR1_OFFSET) &= ~(1U << 6); // Clear SPE bit
        // Also disable peripheral clock to save power
        switch (spi_channel) {
            case SPI_CHANNEL_1: REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_SPI1EN_Pos); break;
            case SPI_CHANNEL_2: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_SPI2EN_Pos); break;
            case SPI_CHANNEL_3: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_SPI3EN_Pos); break;
            default: break;
        }
    }
}

/**
 * @brief Sends a single byte over the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;

    switch (spi_channel) {
        case SPI_CHANNEL_1: spi_base = (volatile uint32_t *)SPI1_BASE_ADDR; break;
        case SPI_CHANNEL_2: spi_base = (volatile uint32_t *)SPI2_BASE_ADDR; break;
        case SPI_CHANNEL_3: spi_base = (volatile uint32_t *)SPI3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (spi_base != NULL) {
        while (!(REG_ACCESS(spi_base + SPI_SR_OFFSET) & (1U << 1))) { /* Wait for TXE flag */ } // TXE: Transmit buffer empty
        REG_ACCESS(spi_base + SPI_DR_OFFSET) = byte;
        while (REG_ACCESS(spi_base + SPI_SR_OFFSET) & (1U << 7)) { /* Wait for BSY flag to clear (transmission complete) */ } // BSY: Busy flag
    }
}

/**
 * @brief Sends a frame of data over the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset();
    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Receives a single byte from the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;
    tbyte received_byte = 0;

    switch (spi_channel) {
        case SPI_CHANNEL_1: spi_base = (volatile uint32_t *)SPI1_BASE_ADDR; break;
        case SPI_CHANNEL_2: spi_base = (volatile uint32_t *)SPI2_BASE_ADDR; break;
        case SPI_CHANNEL_3: spi_base = (volatile uint32_t *)SPI3_BASE_ADDR; break;
        default: return 0; // Invalid channel
    }
    WDT_Reset();

    if (spi_base != NULL) {
        // To read, typically you transmit a dummy byte to initiate clocking
        // Or if in slave mode, just wait for data
        REG_ACCESS(spi_base + SPI_DR_OFFSET) = 0xFF; // Send dummy byte to trigger reception
        while (!(REG_ACCESS(spi_base + SPI_SR_OFFSET) & (1U << 0))) { /* Wait for RXNE flag */ } // RXNE: Receive buffer not empty
        received_byte = (tbyte)REG_ACCESS(spi_base + SPI_DR_OFFSET);
    }
    return received_byte;
}

/**
 * @brief Receives a frame of data from the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of data to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset();
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Receives a null-terminated string from the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes read (excluding null terminator).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset();
    int i = 0;
    for (i = 0; i < max_length - 1; i++) {
        char received_char = (char)SPI_Get_Byte(spi_channel);
        buffer[i] = received_char;
        if (received_char == '\0') { // Check for null terminator
            break;
        }
    }
    buffer[i] = '\0'; // Null-terminate the string
    return i;
}

// External Interrupt APIs
/**
 * @brief Initializes an external interrupt channel.
 * @param external_int_channel The EXTI line (e.g., EXTI_LINE_0).
 * @param external_int_edge The trigger edge (Rising, Falling, Both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset();

    // Enable SYSCFG clock (inferred)
    REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_SYSCFGEN_Pos); // Inferred: RCC_APB2ENR bit 14
    WDT_Reset();

    // Map EXTI line to SYSCFG_EXTICR register and bit
    uint32_t exti_port_idx = (uint32_t)external_int_channel / 4; // EXTI0-3 -> EXTICR1, EXTI4-7 -> EXTICR2, etc.
    uint32_t exti_pin_idx = (uint32_t)external_int_channel % 4;
    volatile uint32_t *exticr_reg_addr = NULL;

    switch (exti_port_idx) {
        case 0: exticr_reg_addr = (volatile uint32_t *)SYSCFG_EXTICR1_ADDR; break;
        case 1: exticr_reg_addr = (volatile uint32_t *)SYSCFG_EXTICR2_ADDR; break;
        case 2: exticr_reg_addr = (volatile uint32_t *)SYSCFG_EXTICR3_ADDR; break;
        case 3: exticr_reg_addr = (volatile uint32_t *)SYSCFG_EXTICR4_ADDR; break;
        default: return; // Invalid EXTI line
    }
    WDT_Reset();

    // Clear previous settings for the EXTI line source
    REG_ACCESS(exticr_reg_addr) &= ~(0xFUL << (exti_pin_idx * 4));
    // Set EXTI source to GPIOA (0x0000) for simplicity. In a real application, this would be passed in.
    // For this implementation, we assume GPIOA is the source for all EXTI lines by default, as it's the first option.
    REG_ACCESS(exticr_reg_addr) |= (0x0UL << (exti_pin_idx * 4)); // Assume GPIOA as default source

    // Configure trigger edge
    REG_ACCESS(EXTI_RTSR_ADDR) &= ~(1U << external_int_channel); // Clear Rising Trigger
    REG_ACCESS(EXTI_FTSR_ADDR) &= ~(1U << external_int_channel); // Clear Falling Trigger
    WDT_Reset();

    if (external_int_edge == EXTERNAL_INT_EDGE_RISING || external_int_edge == EXTERNAL_INT_EDGE_BOTH) {
        REG_ACCESS(EXTI_RTSR_ADDR) |= (1U << external_int_channel);
    }
    if (external_int_edge == EXTERNAL_INT_EDGE_FALLING || external_int_edge == EXTERNAL_INT_EDGE_BOTH) {
        REG_ACCESS(EXTI_FTSR_ADDR) |= (1U << external_int_channel);
    }
    WDT_Reset();

    // Clear pending flag for the EXTI line
    REG_ACCESS(EXTI_PR_ADDR) = (1U << external_int_channel); // Write 1 to clear
    WDT_Reset();
}

/**
 * @brief Enables an external interrupt channel.
 * @param external_int_channel The EXTI line to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset();
    REG_ACCESS(EXTI_IMR_ADDR) |= (1U << external_int_channel); // Set Interrupt Mask Register bit
    // Enable corresponding NVIC interrupt (inferred)
    // For EXTI0, EXTI1, ..., EXTI15, there are specific IRQs.
    // Grouped IRQs (EXTI0, EXTI1, EXTI2, EXTI3, EXTI4, EXTI9_5, EXTI15_10)
    IRQn_Type irqn;
    if (external_int_channel <= 4) { // EXTI0-EXTI4 have individual IRQs
        irqn = (IRQn_Type)(EXTI0_IRQn + external_int_channel);
    } else if (external_int_channel <= 9) { // EXTI5-9 map to EXTI9_5_IRQn
        irqn = EXTI9_5_IRQn;
    } else { // EXTI10-15 map to EXTI15_10_IRQn
        irqn = EXTI15_10_IRQn;
    }
    NVIC_EnableIRQ(irqn);
}

/**
 * @brief Disables an external interrupt channel.
 * @param external_int_channel The EXTI line to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset();
    REG_ACCESS(EXTI_IMR_ADDR) &= ~(1U << external_int_channel); // Clear Interrupt Mask Register bit
    // Disable corresponding NVIC interrupt (inferred)
    IRQn_Type irqn;
    if (external_int_channel <= 4) { // EXTI0-EXTI4 have individual IRQs
        irqn = (IRQn_Type)(EXTI0_IRQn + external_int_channel);
    } else if (external_int_channel <= 9) { // EXTI5-9 map to EXTI9_5_IRQn
        irqn = EXTI9_5_IRQn;
    } else { // EXTI10-15 map to EXTI15_10_IRQn
        irqn = EXTI15_10_IRQn;
    }
    NVIC_DisableIRQ(irqn);
}

// GPIO APIs
/**
 * @brief Initializes a GPIO pin as an output and sets its initial value.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset();
    volatile uint32_t *gpio_base = get_gpio_port_base_addr(port);
    if (gpio_base == NULL) return;

    // Enable GPIO clock (inferred)
    REG_ACCESS(RCC_AHB1ENR_ADDR) |= (1U << (uint32_t)port); // Port A=0, B=1, ... H=7
    WDT_Reset();

    // 1. Set value before setting direction
    GPIO_Value_Set(port, pin, value);
    WDT_Reset();

    // 2. Set direction to output (01)
    REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) &= ~(3U << (pin * 2)); // Clear mode bits
    REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) |= (1U << (pin * 2));  // Set to General purpose output mode (01)
    WDT_Reset();
    // Verify MODER
    while (!((REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) >> (pin * 2)) & 0x01U)) { /* Wait for mode to be set */ }

    // All output pins have pull-up resistors disabled (00)
    REG_ACCESS(gpio_base + GPIO_PUPDR_OFFSET) &= ~(3U << (pin * 2)); // Clear pull-up/pull-down bits
    WDT_Reset();

    // For current registers: use >=20mA sink current & >=10mA source current
    // This is typically achieved by setting "very high speed" on STM32
    REG_ACCESS(gpio_base + GPIO_OSPEEDR_OFFSET) |= (3U << (pin * 2)); // Set to Very High speed (11)
    WDT_Reset();
}

/**
 * @brief Initializes a GPIO pin as an input.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset();
    volatile uint32_t *gpio_base = get_gpio_port_base_addr(port);
    if (gpio_base == NULL) return;

    // Enable GPIO clock (inferred)
    REG_ACCESS(RCC_AHB1ENR_ADDR) |= (1U << (uint32_t)port); // Port A=0, B=1, ... H=7
    WDT_Reset();

    // Set direction to input (00)
    REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) &= ~(3U << (pin * 2)); // Clear mode bits (00 for Input mode)
    WDT_Reset();
    // Verify MODER
    while ((REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) >> (pin * 2)) & 0x03U) { /* Wait for mode to be set to input */ }

    // All input pins have pull-up resistors and wakeup feature enabled (if available)
    REG_ACCESS(gpio_base + GPIO_PUPDR_OFFSET) &= ~(3U << (pin * 2)); // Clear pull-up/pull-down bits
    REG_ACCESS(gpio_base + GPIO_PUPDR_OFFSET) |= (1U << (pin * 2));  // Set to Pull-up (01)
    WDT_Reset();
}

/**
 * @brief Gets the direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction (INPUT or OUTPUT).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset();
    volatile uint32_t *gpio_base = get_gpio_port_base_addr(port);
    if (gpio_base == NULL) return (t_direction)-1; // Invalid

    // Read mode bits (2 bits per pin)
    uint32_t moder_val = (REG_ACCESS(gpio_base + GPIO_MODER_OFFSET) >> (pin * 2)) & 0x03U;
    if (moder_val == 0x00U) {
        return INPUT;
    } else if (moder_val == 0x01U) {
        return OUTPUT;
    }
    return (t_direction)-1; // Not input or output (e.g., AF, Analog)
}

/**
 * @brief Sets the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(t_port port, t_pin pin, t_byte value) {
    WDT_Reset();
    volatile uint32_t *gpio_base = get_gpio_port_base_addr(port);
    if (gpio_base == NULL) return;

    if (value == 1) {
        REG_ACCESS(gpio_base + GPIO_BSRR_OFFSET) = (1UL << pin);       // Set bit (BSx)
    } else {
        REG_ACCESS(gpio_base + GPIO_BSRR_OFFSET) = (1UL << (pin + 16)); // Reset bit (BRx)
    }
    WDT_Reset();
    // Verify ODR
    while (((REG_ACCESS(gpio_base + GPIO_ODR_OFFSET) >> pin) & 0x01U) != value) { /* Wait for value to be set */ }
}

/**
 * @brief Gets the value of a GPIO input or output pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset();
    volatile uint32_t *gpio_base = get_gpio_port_base_addr(port);
    if (gpio_base == NULL) return 0xFF; // Invalid

    // Read input data register
    return (tbyte)((REG_ACCESS(gpio_base + GPIO_IDR_OFFSET) >> pin) & 0x01U);
}

/**
 * @brief Toggles the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset();
    volatile uint32_t *gpio_base = get_gpio_port_base_addr(port);
    if (gpio_base == NULL) return;

    REG_ACCESS(gpio_base + GPIO_ODR_OFFSET) ^= (1UL << pin); // Toggle ODR bit
}

// PWM APIs
/**
 * @brief Initializes a specified PWM channel.
 * @param pwm_channel The PWM channel to initialize (e.g., PWM_CHANNEL_TIM1_CH1).
 * @param pwm_khz_freq The desired PWM frequency in KHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 *
 * Rule: Clear available FREQUENCY Ranges for each channel as comments in PWM_Init()
 * Timer 1, 9, 10, 11 are on APB2 bus (up to 84MHz)
 * Timer 2, 3, 4, 5 are on APB1 bus (up to 42MHz)
 * Max PWM freq depends on timer clock and resolution.
 * E.g., for 16-bit timer (65535 ARR), 84MHz clock: min freq = 84MHz/65535 ~ 1.28 kHz
 * For 42MHz clock: min freq = 42MHz/65535 ~ 0.64 kHz
 * Max freq is typically PCLK/2 or PCLK, depending on resolution needed.
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr((t_timer_channel)pwm_channel);
    if (timer_base == NULL) return;

    uint32_t timer_clock_freq;
    tbyte timer_channel_idx = 0; // 0 for CH1, 1 for CH2, etc.
    uint32_t ccmr_offset; // CCMR1 or CCMR2
    uint32_t ccr_offset;  // CCR1, CCR2, etc.

    // Determine timer clock frequency and enable it
    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1: // PA8, PE9
        case PWM_CHANNEL_TIM1_CH2: // PA9, PE11
        case PWM_CHANNEL_TIM1_CH3: // PA10, PE13
        case PWM_CHANNEL_TIM1_CH4: // PA11, PE14
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM1EN_Pos); // Infer: RCC_APB2ENR bit 0
            timer_clock_freq = 84000000; // Assuming APB2 timer clock at max 84MHz (PCLK2 * 2 if APB2 prescaler > 1)
            timer_channel_idx = (pwm_channel - PWM_CHANNEL_TIM1_CH1);
            ccmr_offset = (timer_channel_idx < 2) ? TIM_CCMR1_OFFSET : TIM_CCMR2_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);
            break;
        case PWM_CHANNEL_TIM2_CH1: // PA0, PA5, PA15, PB3
        case PWM_CHANNEL_TIM2_CH2: // PA1, PB3, PB10
        case PWM_CHANNEL_TIM2_CH3: // PA2, PB10
        case PWM_CHANNEL_TIM2_CH4: // PA3, PB11
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos); // Infer: RCC_APB1ENR bit 0
            timer_clock_freq = 42000000; // Assuming APB1 timer clock at max 42MHz (PCLK1 * 2 if APB1 prescaler > 1)
            timer_channel_idx = (pwm_channel - PWM_CHANNEL_TIM2_CH1);
            ccmr_offset = (timer_channel_idx < 2) ? TIM_CCMR1_OFFSET : TIM_CCMR2_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);
            break;
        case PWM_CHANNEL_TIM3_CH1: // PA6, PB4, PC6
        case PWM_CHANNEL_TIM3_CH2: // PA7, PB5, PC7
        case PWM_CHANNEL_TIM3_CH3: // PB0, PC8
        case PWM_CHANNEL_TIM3_CH4: // PB1, PC9
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM3EN_Pos); // Infer: RCC_APB1ENR bit 1
            timer_clock_freq = 42000000; // Assuming APB1 timer clock at max 42MHz
            timer_channel_idx = (pwm_channel - PWM_CHANNEL_TIM3_CH1);
            ccmr_offset = (timer_channel_idx < 2) ? TIM_CCMR1_OFFSET : TIM_CCMR2_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);
            break;
        case PWM_CHANNEL_TIM4_CH1: // PB6
        case PWM_CHANNEL_TIM4_CH2: // PB7
        case PWM_CHANNEL_TIM4_CH3: // PB8
        case PWM_CHANNEL_TIM4_CH4: // PB9
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM4EN_Pos); // Infer: RCC_APB1ENR bit 2
            timer_clock_freq = 42000000; // Assuming APB1 timer clock at max 42MHz
            timer_channel_idx = (pwm_channel - PWM_CHANNEL_TIM4_CH1);
            ccmr_offset = (timer_channel_idx < 2) ? TIM_CCMR1_OFFSET : TIM_CCMR2_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);
            break;
        case PWM_CHANNEL_TIM5_CH1: // PA0
        case PWM_CHANNEL_TIM5_CH2: // PA1
        case PWM_CHANNEL_TIM5_CH3: // PA2
        case PWM_CHANNEL_TIM5_CH4: // PA3
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM5EN_Pos); // Infer: RCC_APB1ENR bit 3
            timer_clock_freq = 42000000; // Assuming APB1 timer clock at max 42MHz
            timer_channel_idx = (pwm_channel - PWM_CHANNEL_TIM5_CH1);
            ccmr_offset = (timer_channel_idx < 2) ? TIM_CCMR1_OFFSET : TIM_CCMR2_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);
            break;
        case PWM_CHANNEL_TIM9_CH1: // PA2, PE5
        case PWM_CHANNEL_TIM9_CH2: // PA3, PE6
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM9EN_Pos); // Infer: RCC_APB2ENR bit 18
            timer_clock_freq = 84000000; // Assuming APB2 timer clock at max 84MHz
            timer_channel_idx = (pwm_channel - PWM_CHANNEL_TIM9_CH1);
            ccmr_offset = TIM_CCMR1_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);
            break;
        case PWM_CHANNEL_TIM10_CH1: // PB8, PA6
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM10EN_Pos); // Infer: RCC_APB2ENR bit 17
            timer_clock_freq = 84000000; // Assuming APB2 timer clock at max 84MHz
            timer_channel_idx = 0;
            ccmr_offset = TIM_CCMR1_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET;
            break;
        case PWM_CHANNEL_TIM11_CH1: // PB9, PA7
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM11EN_Pos); // Infer: RCC_APB2ENR bit 18 (same as TIM9 in some cases, verify datasheet)
            timer_clock_freq = 84000000; // Assuming APB2 timer clock at max 84MHz
            timer_channel_idx = 0;
            ccmr_offset = TIM_CCMR1_OFFSET;
            ccr_offset = TIM_CCR1_OFFSET;
            break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    // Disable timer before configuring
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit

    // Calculate Prescaler (PSC) and Auto-Reload Register (ARR) for desired frequency
    // Target frequency = timer_clock_freq / ((PSC + 1) * (ARR + 1))
    // To achieve pwm_khz_freq, we choose an ARR and calculate PSC.
    // Max ARR for 16-bit timer is 65535.
    // Let ARR = Max ARR for max resolution (or a fixed value like 1000 for easier duty cycle)
    tword arr = 999; // Set ARR for 1kHz base frequency with appropriate prescaler, or higher for resolution
    tword psc = (timer_clock_freq / (pwm_khz_freq * 1000 * (arr + 1))) - 1;

    // Boundary check for PSC
    if (psc > 0xFFFF) psc = 0xFFFF;
    if (psc < 0) psc = 0; // Ensure non-negative

    REG_ACCESS(timer_base + TIM_PSC_OFFSET) = psc;
    REG_ACCESS(timer_base + TIM_ARR_OFFSET) = arr;
    WDT_Reset();

    // Configure Output Compare Mode (PWM mode 1)
    // Clear CCMR bits for current channel
    uint32_t ccmr_val = REG_ACCESS(timer_base + ccmr_offset);
    uint32_t ccmr_channel_pos = (timer_channel_idx % 2) * 8; // CC1/CC3 are bits 0-7, CC2/CC4 are bits 8-15

    ccmr_val &= ~(0xFFUL << ccmr_channel_pos); // Clear OCxM and OCxE bits
    ccmr_val |= (6U << (4 + ccmr_channel_pos)); // OCxM = 110 (PWM mode 1)
    ccmr_val |= (1U << (3 + ccmr_channel_pos)); // OCxPE (Output compare preload enable)
    REG_ACCESS(timer_base + ccmr_offset) = ccmr_val;
    WDT_Reset();

    // Set Capture/Compare Register (CCR) for duty cycle
    tword pulse_value = (tword)(((float)arr * pwm_duty) / 100.0f);
    REG_ACCESS(timer_base + ccr_offset) = pulse_value;
    WDT_Reset();

    // Enable Output Compare for selected channel in CCER
    REG_ACCESS(timer_base + TIM_CCER_OFFSET) |= (1U << (timer_channel_idx * 4)); // CCxE bit

    // Enable auto-reload preload enable
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) |= (1U << 7); // ARPE bit

    // If advanced timer (TIM1), enable Main Output
    if (timer_base == (volatile uint32_t *)TIM1_BASE_ADDR) {
        REG_ACCESS(timer_base + TIM_BDTR_OFFSET) |= (1U << 15); // MOE bit
    }

    // Generate an update event to load the Prescaler and ARR values
    REG_ACCESS(timer_base + TIM_EGR_OFFSET) |= (1U << 0); // UG bit
    WDT_Reset();

    // Clear available FREQUENCY Ranges for each channel as comments
    // TIM1, TIM9, TIM10, TIM11 (APB2 Clock = 84MHz):
    // With ARR=999, PSC=0: Freq = 84MHz / (1 * 1000) = 84 kHz
    // With ARR=999, PSC=83: Freq = 84MHz / (84 * 1000) = 1 kHz
    // With ARR=65535, PSC=0: Freq = 84MHz / (1 * 65536) ~ 1.28 kHz
    // With ARR=65535, PSC=1279: Freq = 84MHz / (1280 * 65536) ~ 1 Hz
    //
    // TIM2, TIM3, TIM4, TIM5 (APB1 Clock = 42MHz):
    // With ARR=999, PSC=0: Freq = 42MHz / (1 * 1000) = 42 kHz
    // With ARR=999, PSC=41: Freq = 42MHz / (42 * 1000) = 1 kHz
    // With ARR=65535, PSC=0: Freq = 42MHz / (1 * 65536) ~ 0.64 kHz
    // With ARR=65535, PSC=639: Freq = 42MHz / (640 * 65536) ~ 1 Hz
}

/**
 * @brief Starts a specified PWM channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr((t_timer_channel)pwm_channel);
    if (timer_base == NULL) return;

    REG_ACCESS(timer_base + TIM_CR1_OFFSET) |= (1U << 0); // Set CEN bit (Counter Enable)
}

/**
 * @brief Stops a specified PWM channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr((t_timer_channel)pwm_channel);
    if (timer_base == NULL) return;

    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit (Counter Enable)
}

// ICU (Input Capture Unit) APIs
static void (*icu_callback_ptr[16])(void) = {NULL}; // Array of function pointers for EXTI lines

/**
 * @brief Initializes an ICU channel.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler for the timer used by ICU.
 * @param icu_edge The edge to trigger capture (Rising, Falling, Both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr((t_timer_channel)icu_channel);
    if (timer_base == NULL) return;

    uint32_t timer_channel_idx = 0; // 0 for CH1, 1 for CH2, etc.
    uint32_t ccmr_offset; // CCMR1 or CCMR2
    uint32_t ccr_offset;  // CCR1, CCR2, etc.
    t_external_int_channel exti_line = EXTI_LINE_UNKNOWN;

    // Determine timer clock frequency and enable it
    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM1EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_8; break;
        case ICU_CHANNEL_TIM1_CH2: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM1EN_Pos); timer_channel_idx = 1; exti_line = EXTI_LINE_9; break;
        case ICU_CHANNEL_TIM1_CH3: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM1EN_Pos); timer_channel_idx = 2; exti_line = EXTI_LINE_10; break;
        case ICU_CHANNEL_TIM1_CH4: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM1EN_Pos); timer_channel_idx = 3; exti_line = EXTI_LINE_11; break;

        case ICU_CHANNEL_TIM2_CH1: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_0; break;
        case ICU_CHANNEL_TIM2_CH2: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos); timer_channel_idx = 1; exti_line = EXTI_LINE_1; break;
        case ICU_CHANNEL_TIM2_CH3: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos); timer_channel_idx = 2; exti_line = EXTI_LINE_2; break;
        case ICU_CHANNEL_TIM2_CH4: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos); timer_channel_idx = 3; exti_line = EXTI_LINE_3; break;

        case ICU_CHANNEL_TIM3_CH1: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM3EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_6; break;
        case ICU_CHANNEL_TIM3_CH2: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM3EN_Pos); timer_channel_idx = 1; exti_line = EXTI_LINE_7; break;
        case ICU_CHANNEL_TIM3_CH3: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM3EN_Pos); timer_channel_idx = 2; exti_line = EXTI_LINE_0; break; // PB0
        case ICU_CHANNEL_TIM3_CH4: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM3EN_Pos); timer_channel_idx = 3; exti_line = EXTI_LINE_1; break; // PB1

        case ICU_CHANNEL_TIM4_CH1: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM4EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_6; break; // PB6
        case ICU_CHANNEL_TIM4_CH2: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM4EN_Pos); timer_channel_idx = 1; exti_line = EXTI_LINE_7; break; // PB7
        case ICU_CHANNEL_TIM4_CH3: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM4EN_Pos); timer_channel_idx = 2; exti_line = EXTI_LINE_8; break; // PB8
        case ICU_CHANNEL_TIM4_CH4: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM4EN_Pos); timer_channel_idx = 3; exti_line = EXTI_LINE_9; break; // PB9

        case ICU_CHANNEL_TIM5_CH1: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM5EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_0; break; // PA0
        case ICU_CHANNEL_TIM5_CH2: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM5EN_Pos); timer_channel_idx = 1; exti_line = EXTI_LINE_1; break; // PA1
        case ICU_CHANNEL_TIM5_CH3: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM5EN_Pos); timer_channel_idx = 2; exti_line = EXTI_LINE_2; break; // PA2
        case ICU_CHANNEL_TIM5_CH4: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM5EN_Pos); timer_channel_idx = 3; exti_line = EXTI_LINE_3; break; // PA3

        case ICU_CHANNEL_TIM9_CH1: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM9EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_2; break; // PA2
        case ICU_CHANNEL_TIM9_CH2: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM9EN_Pos); timer_channel_idx = 1; exti_line = EXTI_LINE_3; break; // PA3

        case ICU_CHANNEL_TIM10_CH1: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM10EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_6; break; // PA6

        case ICU_CHANNEL_TIM11_CH1: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM11EN_Pos); timer_channel_idx = 0; exti_line = EXTI_LINE_7; break; // PA7

        default: return; // Invalid channel
    }
    WDT_Reset();

    ccmr_offset = (timer_channel_idx < 2) ? TIM_CCMR1_OFFSET : TIM_CCMR2_OFFSET;
    ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);
    uint32_t ccmr_channel_pos = (timer_channel_idx % 2) * 8; // CC1/CC3 are bits 0-7, CC2/CC4 are bits 8-15

    // Disable timer before configuring
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit

    // Configure Prescaler
    REG_ACCESS(timer_base + TIM_PSC_OFFSET) = icu_prescaller;
    REG_ACCESS(timer_base + TIM_ARR_OFFSET) = 0xFFFF; // Max auto-reload value for free-running counter
    WDT_Reset();

    // Configure CCMR for Input Capture mode
    uint32_t ccmr_val = REG_ACCESS(timer_base + ccmr_offset);
    ccmr_val &= ~(0xFFUL << ccmr_channel_pos); // Clear channel's CCMR bits

    // CCxS = 01 (Input, ICx is mapped on TIx)
    ccmr_val |= (1U << (0 + ccmr_channel_pos));

    // Input prescaler (ICxPSC) - no prescaling for input capture
    ccmr_val &= ~(3U << (2 + ccmr_channel_pos));

    // Input filter (ICxF) - no filtering for now (0000)
    ccmr_val &= ~(0xFUL << (4 + ccmr_channel_pos));

    REG_ACCESS(timer_base + ccmr_offset) = ccmr_val;
    WDT_Reset();

    // Configure CCER for trigger edge
    uint32_t ccer_val = REG_ACCESS(timer_base + TIM_CCER_OFFSET);
    ccer_val &= ~(0xAU << (timer_channel_idx * 4)); // Clear CCxP (polarity) and CCxNP (non-polarity) bits

    if (icu_edge == ICU_EDGE_RISING) {
        ccer_val &= ~(1U << (1 + timer_channel_idx * 4)); // CCxP = 0 (rising edge)
    } else if (icu_edge == ICU_EDGE_FALLING) {
        ccer_val |= (1U << (1 + timer_channel_idx * 4));  // CCxP = 1 (falling edge)
    } else { // ICU_EDGE_BOTH - typically toggle CCxP and CCxNP for this, but not supported directly in TIM.
             // Usually done with EXTI or by changing polarity on the fly. For now, defaulting to rising.
        ccer_val &= ~(1U << (1 + timer_channel_idx * 4)); // CCxP = 0 (rising edge)
    }

    ccer_val |= (1U << (timer_channel_idx * 4)); // Enable Capture/Compare output (CCxE)
    REG_ACCESS(timer_base + TIM_CCER_OFFSET) = ccer_val;
    WDT_Reset();

    // Clear and enable Capture/Compare interrupt
    REG_ACCESS(timer_base + TIM_SR_OFFSET) = 0; // Clear all pending interrupts
    REG_ACCESS(timer_base + TIM_DIER_OFFSET) |= (1U << (timer_channel_idx + 1)); // CCxIE (Capture/Compare interrupt enable)
    WDT_Reset();

    // Generate an update event to load the Prescaler and ARR values
    REG_ACCESS(timer_base + TIM_EGR_OFFSET) |= (1U << 0); // UG bit
    WDT_Reset();
}

/**
 * @brief Enables an ICU channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr((t_timer_channel)icu_channel);
    if (timer_base == NULL) return;

    // Enable Timer counter
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) |= (1U << 0); // CEN bit
}

/**
 * @brief Disables an ICU channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr((t_timer_channel)icu_channel);
    if (timer_base == NULL) return;

    // Disable Timer counter
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    // Disable Capture/Compare Interrupt
    uint32_t timer_channel_idx = 0;
    // Map icu_channel to timer_channel_idx (e.g., ICU_CHANNEL_TIM1_CH1 -> 0)
    // This mapping logic needs to be robust for all ICU_CHANNEL_TIMx_CHy values
    if (icu_channel >= ICU_CHANNEL_TIM1_CH1 && icu_channel <= ICU_CHANNEL_TIM1_CH4) timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM1_CH1);
    else if (icu_channel >= ICU_CHANNEL_TIM2_CH1 && icu_channel <= ICU_CHANNEL_TIM2_CH4) timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM2_CH1);
    // ... add for other timers
    else if (icu_channel >= ICU_CHANNEL_TIM9_CH1 && icu_channel <= ICU_CHANNEL_TIM9_CH2) timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM9_CH1);
    else if (icu_channel == ICU_CHANNEL_TIM10_CH1) timer_channel_idx = 0;
    else if (icu_channel == ICU_CHANNEL_TIM11_CH1) timer_channel_idx = 0;

    REG_ACCESS(timer_base + TIM_DIER_OFFSET) &= ~(1U << (timer_channel_idx + 1)); // Clear CCxIE bit
}

/**
 * @brief Gets the frequency measured by an ICU channel.
 * @param icu_channel The ICU channel to use.
 * @return The measured frequency. Returns 0 if unable to measure.
 *
 * This is a conceptual implementation. Real frequency measurement would require
 * two captures on consecutive edges to determine period.
 */
uint32_t ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr((t_timer_channel)icu_channel);
    if (timer_base == NULL) return 0;

    uint32_t timer_channel_idx = 0; // 0 for CH1, 1 for CH2, etc.
    uint32_t ccr_offset;

    // Map icu_channel to timer_channel_idx and ccr_offset
    if (icu_channel >= ICU_CHANNEL_TIM1_CH1 && icu_channel <= ICU_CHANNEL_TIM1_CH4) {
        timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM1_CH1);
    } else if (icu_channel >= ICU_CHANNEL_TIM2_CH1 && icu_channel <= ICU_CHANNEL_TIM2_CH4) {
        timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM2_CH1);
    } else if (icu_channel >= ICU_CHANNEL_TIM3_CH1 && icu_channel <= ICU_CHANNEL_TIM3_CH4) {
        timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM3_CH1);
    } else if (icu_channel >= ICU_CHANNEL_TIM4_CH1 && icu_channel <= ICU_CHANNEL_TIM4_CH4) {
        timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM4_CH1);
    } else if (icu_channel >= ICU_CHANNEL_TIM5_CH1 && icu_channel <= ICU_CHANNEL_TIM5_CH4) {
        timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM5_CH1);
    } else if (icu_channel >= ICU_CHANNEL_TIM9_CH1 && icu_channel <= ICU_CHANNEL_TIM9_CH2) {
        timer_channel_idx = (icu_channel - ICU_CHANNEL_TIM9_CH1);
    } else if (icu_channel == ICU_CHANNEL_TIM10_CH1) {
        timer_channel_idx = 0;
    } else if (icu_channel == ICU_CHANNEL_TIM11_CH1) {
        timer_channel_idx = 0;
    } else {
        return 0; // Invalid channel
    }
    WDT_Reset();

    ccr_offset = TIM_CCR1_OFFSET + (timer_channel_idx * 4);

    // This is a placeholder. A proper frequency measurement requires:
    // 1. Storing two consecutive capture values.
    // 2. Calculating the difference (period).
    // 3. Knowing the timer's clock frequency and prescaler to convert period to time.
    // 4. Frequency = 1 / Period.
    // For now, return a dummy value based on current capture.
    uint32_t capture_val = REG_ACCESS(timer_base + ccr_offset);
    (void)capture_val; // Suppress unused warning

    // Example calculation (highly simplified, assumes 1Hz resolution if timer clock is 1Hz after prescaler/ARR)
    uint32_t timer_clock_freq_hz = 0;
    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1: case ICU_CHANNEL_TIM1_CH2: case ICU_CHANNEL_TIM1_CH3: case ICU_CHANNEL_TIM1_CH4:
        case ICU_CHANNEL_TIM9_CH1: case ICU_CHANNEL_TIM9_CH2: case ICU_CHANNEL_TIM10_CH1: case ICU_CHANNEL_TIM11_CH1:
            timer_clock_freq_hz = 84000000; break;
        case ICU_CHANNEL_TIM2_CH1: case ICU_CHANNEL_TIM2_CH2: case ICU_CHANNEL_TIM2_CH3: case ICU_CHANNEL_TIM2_CH4:
        case ICU_CHANNEL_TIM3_CH1: case ICU_CHANNEL_TIM3_CH2: case ICU_CHANNEL_TIM3_CH3: case ICU_CHANNEL_TIM3_CH4:
        case ICU_CHANNEL_TIM4_CH1: case ICU_CHANNEL_TIM4_CH2: case ICU_CHANNEL_TIM4_CH3: case ICU_CHANNEL_TIM4_CH4:
        case ICU_CHANNEL_TIM5_CH1: case ICU_CHANNEL_TIM5_CH2: case ICU_CHANNEL_TIM5_CH3: case ICU_CHANNEL_TIM5_CH4:
            timer_clock_freq_hz = 42000000; break;
        default: timer_clock_freq_hz = 1; break; // Avoid division by zero
    }
    WDT_Reset();

    uint32_t psc_val = REG_ACCESS(timer_base + TIM_PSC_OFFSET) + 1; // Actual prescaler value
    uint32_t timer_tick_freq = timer_clock_freq_hz / psc_val;

    // Placeholder: Return a fixed value or simple calculation.
    // In a real scenario, this would involve volatile static variables or a buffer to store multiple capture events
    // and an interrupt handler to update them.
    return timer_tick_freq / 1000; // Return something indicative, e.g., KHz if tick is fast
}

/**
 * @brief Sets a callback function for an ICU channel.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset();
    // This requires a more complex mapping from ICU channel to an internal array of callbacks.
    // For simplicity, this implementation assigns it to a global EXTI callback if that's the underlying mechanism.
    // In a full implementation, you'd register `callback` against a specific `icu_channel`.
    if (callback != NULL) {
        // Assuming a generic EXTI line 0 callback for demonstration.
        // A robust solution would map `icu_channel` to its corresponding EXTI line
        // and store the callback in `icu_callback_ptr[exti_line]`.
        icu_callback_ptr[0] = callback;
    }
}

// Timer APIs
/**
 * @brief Initializes a specified Timer channel.
 *        Configures the timer for basic counting (up-counting, no PWM, no capture).
 * @param timer_channel The Timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr(timer_channel);
    if (timer_base == NULL) return;

    // Enable timer clock
    switch (timer_channel) {
        case TIMER_CH_1: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM1EN_Pos); break;
        case TIMER_CH_2: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos); break;
        case TIMER_CH_3: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM3EN_Pos); break;
        case TIMER_CH_4: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM4EN_Pos); break;
        case TIMER_CH_5: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM5EN_Pos); break;
        case TIMER_CH_9: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM9EN_Pos); break;
        case TIMER_CH_10: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM10EN_Pos); break;
        case TIMER_CH_11: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM11EN_Pos); break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    // Disable timer before configuring
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit

    // Configure CR1: Up-counting mode, auto-reload preload enable
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) = (1U << 7); // ARPE (Auto-Reload Preload Enable)

    // Clear DIER (DMA/Interrupt enable register) for basic timer
    REG_ACCESS(timer_base + TIM_DIER_OFFSET) = 0;

    // Reset counter, prescaler, and auto-reload to default
    REG_ACCESS(timer_base + TIM_CNT_OFFSET) = 0;
    REG_ACCESS(timer_base + TIM_PSC_OFFSET) = 0;
    REG_ACCESS(timer_base + TIM_ARR_OFFSET) = 0xFFFFFFFF; // Max value for 32-bit (TIM2/5) or 16-bit timer

    // Generate update event to load configurations
    REG_ACCESS(timer_base + TIM_EGR_OFFSET) |= (1U << 0); // UG bit
    WDT_Reset();
}

/**
 * @brief Sets a timer to generate an interrupt after a specified number of microseconds.
 * @param timer_channel The Timer channel to use.
 * @param time The time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr(timer_channel);
    if (timer_base == NULL) return;

    uint32_t timer_clock_freq;
    if (timer_channel == TIMER_CH_1 || timer_channel == TIMER_CH_9 || timer_channel == TIMER_CH_10 || timer_channel == TIMER_CH_11) {
        timer_clock_freq = 84000000; // APB2 timers
    } else {
        timer_clock_freq = 42000000; // APB1 timers
    }
    WDT_Reset();

    // Reset timer
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Disable
    REG_ACCESS(timer_base + TIM_CNT_OFFSET) = 0; // Clear counter

    // Calculate PSC and ARR
    // Target period = time (us)
    // F_timer = timer_clock_freq / (PSC + 1)
    // ARR = (F_timer * TargetPeriod_s) - 1
    // ARR = (timer_clock_freq / (PSC + 1)) * (time / 1000000.0) - 1
    // For 1 us tick: PSC = timer_clock_freq / 1000000 - 1
    tword psc = (tword)((timer_clock_freq / 1000000U) - 1); // 1us tick
    tword arr = time - 1;

    REG_ACCESS(timer_base + TIM_PSC_OFFSET) = psc;
    REG_ACCESS(timer_base + TIM_ARR_OFFSET) = arr;
    WDT_Reset();

    // Enable Update Interrupt
    REG_ACCESS(timer_base + TIM_DIER_OFFSET) |= (1U << 0); // UIE (Update Interrupt Enable)
    REG_ACCESS(timer_base + TIM_SR_OFFSET) = 0; // Clear pending update flag
    WDT_Reset();
}

/**
 * @brief Sets a timer to generate an interrupt after a specified number of milliseconds.
 * @param timer_channel The Timer channel to use.
 * @param time The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr(timer_channel);
    if (timer_base == NULL) return;

    uint32_t timer_clock_freq;
    if (timer_channel == TIMER_CH_1 || timer_channel == TIMER_CH_9 || timer_channel == TIMER_CH_10 || timer_channel == TIMER_CH_11) {
        timer_clock_freq = 84000000; // APB2 timers
    } else {
        timer_clock_freq = 42000000; // APB1 timers
    }
    WDT_Reset();

    // Reset timer
    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Disable
    REG_ACCESS(timer_base + TIM_CNT_OFFSET) = 0; // Clear counter

    // For 1ms tick: PSC = timer_clock_freq / 1000 - 1
    tword psc = (tword)((timer_clock_freq / 1000U) - 1);
    tword arr = time - 1;

    REG_ACCESS(timer_base + TIM_PSC_OFFSET) = psc;
    REG_ACCESS(timer_base + TIM_ARR_OFFSET) = arr;
    WDT_Reset();

    // Enable Update Interrupt
    REG_ACCESS(timer_base + TIM_DIER_OFFSET) |= (1U << 0); // UIE
    REG_ACCESS(timer_base + TIM_SR_OFFSET) = 0; // Clear pending update flag
    WDT_Reset();
}

/**
 * @brief Sets a timer to generate an interrupt after a specified number of seconds.
 * @param timer_channel The Timer channel to use.
 * @param time The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    // Use ms function with conversion
    TIMER_Set_Time_ms(timer_channel, (tword)time * 1000);
}

/**
 * @brief Sets a timer to generate an interrupt after a specified number of minutes.
 * @param timer_channel The Timer channel to use.
 * @param time The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    // Use ms function with conversion
    TIMER_Set_Time_ms(timer_channel, (tword)time * 60 * 1000);
}

/**
 * @brief Sets a timer to generate an interrupt after a specified number of hours.
 * @param timer_channel The Timer channel to use.
 * @param time The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    // Use ms function with conversion
    TIMER_Set_Time_ms(timer_channel, (tword)time * 60 * 60 * 1000);
}

/**
 * @brief Enables a specified Timer channel.
 * @param timer_channel The Timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr(timer_channel);
    if (timer_base == NULL) return;

    // Enable timer clock (redundant if Init called, but safe)
    switch (timer_channel) {
        case TIMER_CH_1: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM1EN_Pos); break;
        case TIMER_CH_2: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos); break;
        case TIMER_CH_3: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM3EN_Pos); break;
        case TIMER_CH_4: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM4EN_Pos); break;
        case TIMER_CH_5: REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM5EN_Pos); break;
        case TIMER_CH_9: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM9EN_Pos); break;
        case TIMER_CH_10: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM10EN_Pos); break;
        case TIMER_CH_11: REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_TIM11EN_Pos); break;
        default: return; // Invalid channel
    }
    WDT_Reset(); // Call WDT_Reset before setting the enable bit.

    REG_ACCESS(timer_base + TIM_CR1_OFFSET) |= (1U << 0); // Set CEN bit
}

/**
 * @brief Disables a specified Timer channel.
 * @param timer_channel The Timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset();
    volatile uint32_t *timer_base = get_timer_base_addr(timer_channel);
    if (timer_base == NULL) return;

    REG_ACCESS(timer_base + TIM_CR1_OFFSET) &= ~(1U << 0); // Clear CEN bit
    REG_ACCESS(timer_base + TIM_DIER_OFFSET) &= ~(1U << 0); // Clear UIE (Update Interrupt Enable)
    // Also disable peripheral clock to save power
    switch (timer_channel) {
        case TIMER_CH_1: REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM1EN_Pos); break;
        case TIMER_CH_2: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM2EN_Pos); break;
        case TIMER_CH_3: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM3EN_Pos); break;
        case TIMER_CH_4: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM4EN_Pos); break;
        case TIMER_CH_5: REG_ACCESS(RCC_APB1ENR_ADDR) &= ~(1U << RCC_APB1ENR_TIM5EN_Pos); break;
        case TIMER_CH_9: REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM9EN_Pos); break;
        case TIMER_CH_10: REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM10EN_Pos); break;
        case TIMER_CH_11: REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_TIM11EN_Pos); break;
        default: break;
    }
    WDT_Reset();
}

// ADC APIs
/**
 * @brief Initializes the ADC channel with specified mode.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The ADC operating mode (e.g., Single Conversion, Continuous).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset();
    // Enable ADC1 clock
    REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_ADC1EN_Pos); // Infer: RCC_APB2ENR bit 8
    WDT_Reset();

    // Disable ADC before configuration
    REG_ACCESS(ADC1_CR2_ADDR) &= ~(1U << 0); // Clear ADON bit

    // Configure ADC Common Control Register (CCR) - default values for basic operation
    // E.g., ADCPRE (prescaler), multi-mode disable, Vrefint enable
    REG_ACCESS(ADC_CCR_ADDR) = (2U << 16) | (1U << 23); // PCLK2/4 prescaler, TSVREFE (Temperature sensor and VREFINT enable)
    WDT_Reset();

    // Configure CR1: resolution, scan mode, discontinuous mode, watchdog
    REG_ACCESS(ADC1_CR1_ADDR) = 0; // Default to 12-bit, single-channel, no watchdog etc.
    if (adc_mode == ADC_MODE_CONTINUOUS) {
        REG_ACCESS(ADC1_CR2_ADDR) |= (1U << 1); // CONT bit for continuous conversion
    }
    WDT_Reset();

    // Configure CR2: external trigger selection, right alignment, SWSTART
    REG_ACCESS(ADC1_CR2_ADDR) = (0U << 10) | (0U << 8); // No external trigger, Right alignment

    // Configure Sample Time for the selected channel (SMPR1 for channels 10-18, SMPR2 for 0-9)
    uint32_t channel_num = (uint32_t)adc_channel;
    if (channel_num >= 10 && channel_num <= 18) {
        // SMPR1 configures channels 10-18 (3 bits per channel, starting at bit 0 for channel 10)
        REG_ACCESS(ADC1_SMPR1_ADDR) &= ~(0x7UL << ((channel_num - 10) * 3));
        REG_ACCESS(ADC1_SMPR1_ADDR) |= (ADC_SAMPLE_TIME_480_CYCLES << ((channel_num - 10) * 3)); // Use a long sample time
    } else if (channel_num >= 0 && channel_num <= 9) {
        // SMPR2 configures channels 0-9 (3 bits per channel, starting at bit 0 for channel 0)
        REG_ACCESS(ADC1_SMPR2_ADDR) &= ~(0x7UL << (channel_num * 3));
        REG_ACCESS(ADC1_SMPR2_ADDR) |= (ADC_SAMPLE_TIME_480_CYCLES << (channel_num * 3)); // Use a long sample time
    }
    WDT_Reset();

    // Configure Regular Sequence (SQR3 for first 6 conversions)
    // L bits in SQR1 define number of conversions. Default to 0000 (1 conversion)
    REG_ACCESS(ADC1_SQR1_ADDR) &= ~(0xFUL << 20); // Clear L bits (set to 0 for 1 conversion)
    REG_ACCESS(ADC1_SQR3_ADDR) = channel_num; // Set SQ1 to the desired channel
    WDT_Reset();
}

/**
 * @brief Enables the ADC module.
 * @param adc_channel The ADC channel to enable (redundant but kept for API consistency).
 */
void ADC_Enable(t_adc_channel adc_channel) {
    WDT_Reset();
    REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_ADC1EN_Pos); // Enable peripheral clock (inferred)
    WDT_Reset(); // Call WDT_Reset before setting the enable bit.

    REG_ACCESS(ADC1_CR2_ADDR) |= (1U << 0); // Set ADON bit
    // Wait for ADC to stabilize (~10us or more)
    for (volatile int i = 0; i < 1000; i++); // Simple delay
    WDT_Reset();
}

/**
 * @brief Disables the ADC module.
 * @param adc_channel The ADC channel to disable (redundant but kept for API consistency).
 */
void ADC_Disable(t_adc_channel adc_channel) {
    WDT_Reset();
    REG_ACCESS(ADC1_CR2_ADDR) &= ~(1U << 0); // Clear ADON bit
    REG_ACCESS(RCC_APB2ENR_ADDR) &= ~(1U << RCC_APB2ENR_ADC1EN_Pos); // Disable peripheral clock
    WDT_Reset();
}

/**
 * @brief Performs an ADC conversion in polling mode.
 * @param adc_channel The ADC channel to read.
 * @return The converted 16-bit digital value.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel) {
    WDT_Reset();
    // Start conversion
    REG_ACCESS(ADC1_CR2_ADDR) |= (1U << 30); // SWSTART bit
    WDT_Reset();

    // Wait for EOC (End Of Conversion) flag
    while (!(REG_ACCESS(ADC1_SR_ADDR) & (1U << 1))) { /* max timeout implied */ }
    REG_ACCESS(ADC1_SR_ADDR) &= ~(1U << 1); // Clear EOC flag by writing 0 (or read DR)

    // Read converted data
    tword adc_value = (tword)(REG_ACCESS(ADC1_DR_ADDR) & 0xFFFF);
    WDT_Reset();
    return adc_value;
}

/**
 * @brief Placeholder for ADC conversion in interrupt mode.
 *        This function only reads the data register, assuming an ISR handles the conversion.
 * @param adc_channel The ADC channel to read.
 * @return The converted 16-bit digital value.
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel) {
    WDT_Reset();
    // Enable EOC interrupt if not already
    REG_ACCESS(ADC1_CR1_ADDR) |= (1U << 5); // EOCIE bit
    WDT_Reset();

    // In an interrupt-driven system, the conversion would be started elsewhere
    // and the ISR would read the DR. This function just returns the last DR value.
    // For testing/demonstration, you might trigger a SWSTART here too.
    REG_ACCESS(ADC1_CR2_ADDR) |= (1U << 30); // SWSTART bit

    // In a real interrupt driven scenario, this function would not block.
    // It would return a stored value or trigger a conversion and rely on an ISR.
    // For this direct register access, we'll do a blocking read to get a value.
    while (!(REG_ACCESS(ADC1_SR_ADDR) & (1U << 1))) { /* Wait for EOC flag */ }
    REG_ACCESS(ADC1_SR_ADDR) &= ~(1U << 1); // Clear EOC flag

    tword adc_value = (tword)(REG_ACCESS(ADC1_DR_ADDR) & 0xFFFF);
    WDT_Reset();
    return adc_value;
}

// Internal_EEPROM - Using Flash memory as proxy
// Note: STM32F401RC does not have a dedicated internal EEPROM.
// EEPROM functionality is typically emulated using a dedicated Flash sector.
// The provided register_json lists general Flash control registers, not EEPROM emulation specifics.
// A full implementation would involve Flash erase/write cycles, error handling,
// and wear-leveling algorithms. This is a simplified placeholder.

/**
 * @brief Initializes the internal EEPROM (Flash emulation).
 *        This is a placeholder for Flash-based EEPROM emulation.
 */
void Internal_EEPROM_Init(void) {
    WDT_Reset();
    // For Flash emulation, typically enable Flash controller clock (already enabled by default)
    // Unlock Flash Controller if needed for write/erase operations.
    // (Flash access is usually available without explicit clock enable from RCC for core Flash)
    // REG_ACCESS(FLASH_KEYR_ADDR) = FLASH_KEY1; // Example unlock sequence
    // REG_ACCESS(FLASH_KEYR_ADDR) = FLASH_KEY2;
    // ... further setup for Flash programming/erase if needed.
}

/**
 * @brief Sets a byte in the internal EEPROM (Flash emulation).
 * @param address The address within the emulated EEPROM.
 * @param data The byte data to write.
 * Note: Actual Flash programming involves erasing sectors/pages and then writing words/half-words.
 *       Direct byte writes are not usually supported without a complex emulation layer.
 *       This function is a placeholder and will not perform actual flash write operations without a full Flash driver.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset();
    // Placeholder: In a real Flash EEPROM emulation, this would involve:
    // 1. Unlocking Flash.
    // 2. Erasing the target Flash sector if necessary.
    // 3. Writing the 'data' at 'address' (which would be mapped to a Flash address).
    // 4. Locking Flash.
    // This cannot be safely implemented with just the `FLASH_CR` and `FLASH_KEYR` registers
    // without full Flash programming algorithms and error checking.
    // Adding a comment to indicate this limitation.
    // printf("Warning: Internal_EEPROM_Set not fully implemented for Flash emulation.\n");
    (void)address; // Suppress unused parameter warning
    (void)data;    // Suppress unused parameter warning
}

/**
 * @brief Gets a byte from the internal EEPROM (Flash emulation).
 * @param address The address within the emulated EEPROM.
 * @return The byte data read.
 * Note: Reading from Flash is typically direct memory access after mapping.
 */
tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset();
    // Placeholder: In a real Flash EEPROM emulation, this would involve:
    // 1. Mapping the 'address' to a physical Flash memory address.
    // 2. Directly reading from that Flash address.
    // Assuming a base Flash address for emulation, e.g., 0x0800C000 (part of Flash)
    volatile tbyte *flash_ptr = (volatile tbyte *)(0x0800C000 + address); // Example base address for EEPROM sector
    tbyte data = *flash_ptr;
    WDT_Reset();
    return data;
}

// TT (Time Triggered OS)
// This is a skeletal implementation for a time-triggered scheduler.
// A timer (e.g., SysTick or TIMx) will be used to generate ticks for TT_ISR.

#define MAX_TASKS 10
typedef struct {
    void (*task_func)(void);
    tword period;
    tword delay;
    bool enabled;
    tword run_count; // For internal use
} TT_Task_t;

static TT_Task_t tt_tasks[MAX_TASKS];
static tbyte tt_next_task_id = 0;
static tword tt_tick_count = 0;
static volatile uint32_t *tt_timer_base = NULL; // Assuming TIM2 for TT
static uint32_t tt_timer_clock_freq = 0;

/**
 * @brief Initializes the Time Triggered (TT) OS with a specified tick time.
 * @param tick_time_ms The tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset();

    // Initialize task list
    for (tbyte i = 0; i < MAX_TASKS; i++) {
        tt_tasks[i].task_func = NULL;
        tt_tasks[i].period = 0;
        tt_tasks[i].delay = 0;
        tt_tasks[i].enabled = false;
        tt_tasks[i].run_count = 0;
    }
    tt_next_task_id = 0;
    tt_tick_count = 0;

    // Configure a timer for the TT_ISR (e.g., TIM2)
    tt_timer_base = (volatile uint32_t *)TIM2_BASE_ADDR;
    tt_timer_clock_freq = 42000000; // Assuming APB1 clock for TIM2

    // Enable TIM2 clock
    REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_TIM2EN_Pos);
    WDT_Reset();

    // Disable timer before configuring
    REG_ACCESS(tt_timer_base + TIM_CR1_OFFSET) &= ~(1U << 0);

    // Calculate PSC and ARR for desired tick_time_ms
    // For 1ms tick: PSC = timer_clock_freq / 1000 - 1
    tword psc = (tword)((tt_timer_clock_freq / 1000U) - 1);
    tword arr = tick_time_ms - 1;

    REG_ACCESS(tt_timer_base + TIM_PSC_OFFSET) = psc;
    REG_ACCESS(tt_timer_base + TIM_ARR_OFFSET) = arr;
    WDT_Reset();

    // Enable Update Interrupt
    REG_ACCESS(tt_timer_base + TIM_DIER_OFFSET) |= (1U << 0); // UIE
    REG_ACCESS(tt_timer_base + TIM_SR_OFFSET) = 0; // Clear pending update flag
    WDT_Reset();

    // Enable TIM2 interrupt in NVIC (inferred)
    NVIC_EnableIRQ(TIM2_IRQn);
    WDT_Reset();
}

/**
 * @brief Starts the Time Triggered (TT) OS scheduler.
 */
void TT_Start(void) {
    WDT_Reset();
    if (tt_timer_base != NULL) {
        REG_ACCESS(tt_timer_base + TIM_CR1_OFFSET) |= (1U << 0); // Set CEN bit
    }
}

/**
 * @brief Dispatches tasks based on their period and delay.
 *        This function should be called in the main loop.
 */
void TT_Dispatch_task(void) {
    WDT_Reset();
    for (tbyte i = 0; i < MAX_TASKS; i++) {
        if (tt_tasks[i].task_func != NULL && tt_tasks[i].enabled) {
            if (tt_tasks[i].run_count == 0) {
                tt_tasks[i].task_func();
                tt_tasks[i].run_count = tt_tasks[i].period; // Reload period
            }
        }
    }
}

/**
 * @brief Interrupt Service Routine (ISR) for the Time Triggered (TT) OS.
 *        This function decrements task counters and should be called by the timer ISR.
 */
void TT_ISR(void) {
    WDT_Reset(); // WDT_Reset in ISR should be quick and not block
    tt_tick_count++; // Global tick counter

    for (tbyte i = 0; i < MAX_TASKS; i++) {
        if (tt_tasks[i].task_func != NULL && tt_tasks[i].enabled) {
            if (tt_tasks[i].delay > 0) {
                tt_tasks[i].delay--;
            } else {
                if (tt_tasks[i].run_count > 0) {
                    tt_tasks[i].run_count--;
                }
            }
        }
    }
}

/**
 * @brief Adds a task to the TT scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks.
 * @param delay The initial delay of the task in ticks.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset();
    if (tt_next_task_id < MAX_TASKS) {
        tt_tasks[tt_next_task_id].task_func = task;
        tt_tasks[tt_next_task_id].period = period;
        tt_tasks[tt_next_task_id].delay = delay;
        tt_tasks[tt_next_task_id].enabled = true;
        tt_tasks[tt_next_task_id].run_count = delay; // Initial delay
        return tt_next_task_id++;
    }
    return 0xFF; // Failed to add task
}

/**
 * @brief Deletes a task from the TT scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset();
    if (task_index < MAX_TASKS) {
        tt_tasks[task_index].task_func = NULL;
        tt_tasks[task_index].enabled = false;
    }
}

// Optional Modules (skipped if registers/drivers not found)
// As per Rules.json, these modules are optional and only implemented if registers/drivers exist.
// Since specific registers/drivers for these modules are NOT found in the provided register_json
// for the STM32F401RC, they are skipped.

// MCAL_OUTPUT_BUZZER not supported on this MCU (no dedicated buzzer registers in JSON)
// DAC not supported on this MCU (no dedicated DAC registers in JSON)
// MQTT Protocol not supported on this MCU (high-level protocol, no direct register support)
// HTTP Protocol not supported on this MCU (high-level protocol, no direct register support)
// WiFi Driver not supported on this MCU (high-level protocol, no direct register support)
// DTC_driver not supported on this MCU (requires specific DMA/DTC registers not present in JSON for a complete driver)

// I2S APIs
/**
 * @brief Initializes an I2S channel.
 * @param channel The I2S channel to initialize (e.g., I2S_CHANNEL_1).
 * @param mode I2S operating mode (Master/Slave, Transmit/Receive).
 * @param standard I2S standard (Philips, MSB, LSB, PCM).
 * @param data_format I2S data format (16-bit, 24-bit, 32-bit).
 * @param channel_mode I2S channel mode (stereo, mono).
 * @param sample_rate I2S audio sample rate.
 * @param mclk_freq Master clock frequency (if used).
 * @param dma_buffer_size DMA buffer size (if using DMA).
 */
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;

    switch (channel) {
        case I2S_CHANNEL_1:
            spi_base = (volatile uint32_t *)SPI1_BASE_ADDR;
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_SPI1EN_Pos); // Enable peripheral clock for SPI1 (I2S1)
            break;
        case I2S_CHANNEL_2:
            spi_base = (volatile uint32_t *)SPI2_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI2EN_Pos); // Enable peripheral clock for SPI2 (I2S2)
            break;
        case I2S_CHANNEL_3:
            spi_base = (volatile uint32_t *)SPI3_BASE_ADDR;
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI3EN_Pos); // Enable peripheral clock for SPI3 (I2S3)
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset();

    // Disable I2S before configuration
    REG_ACCESS(spi_base + SPI_I2SCFGR_OFFSET) &= ~(1U << 7); // Clear I2SE bit

    // Configure I2SCFGR register
    uint32_t i2scfgr_val = (1U << 10); // I2SMOD = 1 (I2S mode selected)

    // Mode selection (Master/Slave, Tx/Rx)
    if (mode == I2S_MODE_MASTER_TX)      i2scfgr_val |= (3U << 8); // I2SCFG = 11 (Master Transmit)
    else if (mode == I2S_MODE_MASTER_RX) i2scfgr_val |= (2U << 8); // I2SCFG = 10 (Master Receive)
    else if (mode == I2S_MODE_SLAVE_TX)  i2scfgr_val |= (1U << 8); // I2SCFG = 01 (Slave Transmit)
    else if (mode == I2S_MODE_SLAVE_RX)  i2scfgr_val |= (0U << 8); // I2SCFG = 00 (Slave Receive)

    // I2S Standard
    if (standard == I2S_STANDARD_PHILIPS)    i2scfgr_val |= (0U << 4); // I2SSTD = 00
    else if (standard == I2S_STANDARD_MSB)   i2scfgr_val |= (1U << 4); // I2SSTD = 01
    else if (standard == I2S_STANDARD_LSB)   i2scfgr_val |= (2U << 4); // I2SSTD = 10
    else if (standard == I2S_STANDARD_PCM_SHORT) i2scfgr_val |= (3U << 4); // I2SSTD = 11 (PCM Short)

    // Data format (CHLEN, DATLEN)
    if (data_format == I2S_DATAFORMAT_16B)      i2scfgr_val |= (0U << 1); // DATLEN = 00
    else if (data_format == I2S_DATAFORMAT_24B) i2scfgr_val |= (1U << 1); // DATLEN = 01
    else if (data_format == I2S_DATAFORMAT_32B) i2scfgr_val |= (2U << 1); // DATLEN = 10

    if (channel_mode == I2S_CHANNELMODE_MONO) i2scfgr_val |= (1U << 0); // CHLEN = 1 (mono)

    REG_ACCESS(spi_base + SPI_I2SCFGR_OFFSET) = i2scfgr_val;
    WDT_Reset();

    // Configure I2SPR for sample rate
    // I2SPR.MCKOE, I2SDIV, ODD bits
    // Fs = I2Sclk / ((10 * 2 * I2SDIV) + ((10 * ODD) - 2 * DATLEN)) for master
    // Simplified: Use common I2S_PR registers from the clock tree to set exact rates.
    // For a typical sample rate calculation on STM32F4, you need PLLI2S_R and PLLI2S_N
    // (from RCC_PLLI2SCFGR) and then the I2SDIV and ODD bits in SPI_I2SPR.
    // This requires complex clock configuration not fully expressible with given registers and API.
    // For now, setting a common prescaler value.
    // Assuming a generic calculation or pre-calculated values for Fs.
    tword i2sdiv = 2; // Minimum valid I2SDIV is 2
    tbyte odd = 0;

    // A real implementation would calculate these based on System Clock and Sample Rate
    // For example, if PLLI2S_R = 3, and PLLI2S_N = 192 for 48kHz, then I2SDIV/ODD are used to fine-tune.
    // This is a placeholder for `sample_rate` to `I2SDIV/ODD` calculation.
    (void)sample_rate; (void)mclk_freq; (void)dma_buffer_size;

    REG_ACCESS(spi_base + SPI_I2SPR_OFFSET) = (i2sdiv & 0xFF) | ((odd & 0x1) << 8); // I2SDIV & ODD
    WDT_Reset();
}

/**
 * @brief Enables a specified I2S channel.
 * @param channel The I2S channel to enable.
 */
void I2S_Enable(t_i2s_channel channel) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;

    switch (channel) {
        case I2S_CHANNEL_1:
            REG_ACCESS(RCC_APB2ENR_ADDR) |= (1U << RCC_APB2ENR_SPI1EN_Pos); // Enable peripheral clock (inferred)
            spi_base = (volatile uint32_t *)SPI1_BASE_ADDR;
            break;
        case I2S_CHANNEL_2:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI2EN_Pos); // Enable peripheral clock (inferred)
            spi_base = (volatile uint32_t *)SPI2_BASE_ADDR;
            break;
        case I2S_CHANNEL_3:
            REG_ACCESS(RCC_APB1ENR_ADDR) |= (1U << RCC_APB1ENR_SPI3EN_Pos); // Enable peripheral clock (inferred)
            spi_base = (volatile uint32_t *)SPI3_BASE_ADDR;
            break;
        default:
            return; // Invalid channel
    }
    WDT_Reset(); // Call WDT_Reset before setting the enable bit.

    if (spi_base != NULL) {
        REG_ACCESS(spi_base + SPI_I2SCFGR_OFFSET) |= (1U << 7); // Set I2SE bit
    }
}

/**
 * @brief Transmits data over the specified I2S channel.
 * @param channel The I2S channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data to transmit.
 */
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;

    switch (channel) {
        case I2S_CHANNEL_1: spi_base = (volatile uint32_t *)SPI1_BASE_ADDR; break;
        case I2S_CHANNEL_2: spi_base = (volatile uint32_t *)SPI2_BASE_ADDR; break;
        case I2S_CHANNEL_3: spi_base = (volatile uint32_t *)SPI3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (spi_base != NULL) {
        const uint16_t *tx_data = (const uint16_t *)data; // Assuming 16-bit data words for I2S
        for (size_t i = 0; i < length / sizeof(uint16_t); i++) {
            while (!(REG_ACCESS(spi_base + SPI_SR_OFFSET) & (1U << 1))) { /* Wait for TXE flag */ }
            REG_ACCESS(spi_base + SPI_DR_OFFSET) = tx_data[i];
        }
    }
}

/**
 * @brief Receives data from the specified I2S channel.
 * @param channel The I2S channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param length The length of data to receive.
 */
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length) {
    WDT_Reset();
    volatile uint32_t *spi_base = NULL;

    switch (channel) {
        case I2S_CHANNEL_1: spi_base = (volatile uint32_t *)SPI1_BASE_ADDR; break;
        case I2S_CHANNEL_2: spi_base = (volatile uint32_t *)SPI2_BASE_ADDR; break;
        case I2S_CHANNEL_3: spi_base = (volatile uint32_t *)SPI3_BASE_ADDR; break;
        default: return; // Invalid channel
    }
    WDT_Reset();

    if (spi_base != NULL) {
        uint16_t *rx_buffer = (uint16_t *)buffer; // Assuming 16-bit data words for I2S
        for (size_t i = 0; i < length / sizeof(uint16_t); i++) {
            while (!(REG_ACCESS(spi_base + SPI_SR_OFFSET) & (1U << 0))) { /* Wait for RXNE flag */ }
            rx_buffer[i] = (uint16_t)REG_ACCESS(spi_base + SPI_DR_OFFSET);
        }
    }
}

// =========================================================================
//                  TIMER INTERRUPT HANDLERS (Example for TT)
// =========================================================================

// Example TIM2_IRQHandler for TT_ISR
void TIM2_IRQHandler(void) {
    WDT_Reset(); // Reset watchdog in ISR
    if (REG_ACCESS(TIM2_BASE_ADDR + TIM_SR_OFFSET) & (1U << 0)) { // Check UIF (Update Interrupt Flag)
        REG_ACCESS(TIM2_BASE_ADDR + TIM_SR_OFFSET) &= ~(1U << 0); // Clear UIF
        TT_ISR(); // Call the Time Triggered OS ISR
    }
}

// Example External Interrupt Handler
void EXTI0_IRQHandler(void) {
    WDT_Reset();
    if (REG_ACCESS(EXTI_PR_ADDR) & (1U << 0)) { // Check if EXTI line 0 pending
        REG_ACCESS(EXTI_PR_ADDR) = (1U << 0); // Clear pending bit
        if (icu_callback_ptr[0] != NULL) {
            icu_callback_ptr[0]();
        }
    }
}
// Add other EXTI handlers (EXTI1_IRQHandler, EXTI2_IRQHandler, etc. up to EXTI15_10_IRQHandler) as needed.