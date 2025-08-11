/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) Source File for STM32F401RC.
 *
 * This file contains the implementation of the MCAL APIs for the STM32F401RC
 * microcontroller. It directly interacts with the hardware registers to provide
 * an abstract interface to the peripherals.
 *
 * This source file follows the rules and API definitions provided in API.json and Rules.json,
 * and uses register definitions from the provided REGISTER_JSON.
 *
 * MCU: STM32F401RC
 */

#include "MCAL.h"

// Standard CMSIS core functions for global interrupts (not direct register access based on provided JSON)
// These are typical ARM Cortex-M intrinsics.
#ifdef __GNUC__
    #define __enable_irq()    __asm volatile ("cpsie i" : : : "memory")
    #define __disable_irq()   __asm volatile ("cpsid i" : : : "memory")
#elif defined(__ICCARM__)
    #include <cmsis_iar.h>
#elif defined(__ARMCC_VERSION)
    #include <cmsis_armcc.h>
#endif


// =============================================================================
//                             REGISTER ADDRESS DEFINITIONS
// =============================================================================

// FLASH Registers
#define FLASH_ACR_ADDR      ((volatile tlong*)0x40023C00UL)
#define FLASH_KEYR_ADDR     ((volatile tlong*)0x40023C04UL)
#define FLASH_OPTKEYR_ADDR  ((volatile tlong*)0x40023C08UL)
#define FLASH_SR_ADDR       ((volatile tlong*)0x40023C0CUL)
#define FLASH_CR_ADDR       ((volatile tlong*)0x40023C10UL)
#define FLASH_OPTCR_ADDR    ((volatile tlong*)0x40023C14UL)

// CRC Registers
#define CRC_DR_ADDR         ((volatile tlong*)0x40023000UL)
#define CRC_IDR_ADDR        ((volatile tlong*)0x40023004UL)
#define CRC_CR_ADDR         ((volatile tlong*)0x40023008UL)

// PWR Registers
#define PWR_CR_ADDR         ((volatile tlong*)0x40007000UL)
#define PWR_CSR_ADDR        ((volatile tlong*)0x40007004UL)

// RCC Registers
#define RCC_CR_ADDR             ((volatile tlong*)0x40023800UL)
#define RCC_PLLCFGR_ADDR        ((volatile tlong*)0x40023804UL)
#define RCC_CFGR_ADDR           ((volatile tlong*)0x40023808UL)
#define RCC_CIR_ADDR            ((volatile tlong*)0x4002380CUL)
#define RCC_AHB1RSTR_ADDR       ((volatile tlong*)0x40023810UL)
#define RCC_AHB2RSTR_ADDR       ((volatile tlong*)0x40023814UL)
#define RCC_APB1RSTR_ADDR       ((volatile tlong*)0x40023818UL)
#define RCC_APB2RSTR_ADDR       ((volatile tlong*)0x4002381CUL)
#define RCC_AHB1ENR_ADDR        ((volatile tlong*)0x40023830UL)
#define RCC_AHB2ENR_ADDR        ((volatile tlong*)0x40023834UL)
#define RCC_APB1ENR_ADDR        ((volatile tlong*)0x40023838UL)
#define RCC_APB2ENR_ADDR        ((volatile tlong*)0x4002383CUL)
#define RCC_AHB1LPENR_ADDR      ((volatile tlong*)0x40023850UL)
#define RCC_AHB2LPENR_ADDR      ((volatile tlong*)0x40023854UL)
#define RCC_APB1LPENR_ADDR      ((volatile tlong*)0x40023858UL)
#define RCC_APB2LPENR_ADDR      ((volatile tlong*)0x4002385CUL)
#define RCC_BDCR_ADDR           ((volatile tlong*)0x40023870UL)
#define RCC_CSR_ADDR            ((volatile tlong*)0x40023874UL)
#define RCC_SSCGR_ADDR          ((volatile tlong*)0x40023880UL)
#define RCC_PLLI2SCFGR_ADDR     ((volatile tlong*)0x40023884UL)
#define RCC_DCKCFGR_ADDR        ((volatile tlong*)0x4002388CUL)

// SYSCFG Registers
#define SYSCFG_MEMRMP_ADDR  ((volatile tlong*)0x40013800UL)
#define SYSCFG_PMC_ADDR     ((volatile tlong*)0x40013804UL)
#define SYSCFG_EXTICR1_ADDR ((volatile tlong*)0x40013808UL)
#define SYSCFG_EXTICR2_ADDR ((volatile tlong*)0x4001380CUL)
#define SYSCFG_EXTICR3_ADDR ((volatile tlong*)0x40013810UL)
#define SYSCFG_EXTICR4_ADDR ((volatile tlong*)0x40013814UL)
#define SYSCFG_CMPCR_ADDR   ((volatile tlong*)0x40013820UL)

// GPIO Registers (Base Addresses)
#define GPIOA_BASE_ADDR     0x40020000UL
#define GPIOB_BASE_ADDR     0x40020400UL
#define GPIOC_BASE_ADDR     0x40020800UL
#define GPIOD_BASE_ADDR     0x40020C00UL
#define GPIOE_BASE_ADDR     0x40021000UL
#define GPIOH_BASE_ADDR     0x40021C00UL

// GPIO Register Offsets
#define GPIO_MODER_OFFSET   0x00UL
#define GPIO_OTYPER_OFFSET  0x04UL
#define GPIO_OSPEEDR_OFFSET 0x08UL
#define GPIO_PUPDR_OFFSET   0x0CUL
#define GPIO_IDR_OFFSET     0x10UL
#define GPIO_ODR_OFFSET     0x14UL
#define GPIO_BSRR_OFFSET    0x18UL
#define GPIO_LCKR_OFFSET    0x1CUL
#define GPIO_AFRL_OFFSET    0x20UL
#define GPIO_AFRH_OFFSET    0x24UL

// Helper to get GPIO register address
#define GPIO_REG_ADDR(PORT_BASE, OFFSET) ((volatile tlong*)(PORT_BASE + OFFSET))

// DMA Registers (Base Addresses)
#define DMA1_BASE_ADDR      0x40026000UL
#define DMA2_BASE_ADDR      0x40026400UL

// DMA Register Offsets
#define DMA_LISR_OFFSET     0x00UL
#define DMA_HISR_OFFSET     0x04UL
#define DMA_LIFCR_OFFSET    0x08UL
#define DMA_HIFCR_OFFSET    0x0CUL
// Stream 0
#define DMA_S0CR_OFFSET     0x10UL
#define DMA_S0NDTR_OFFSET   0x14UL
#define DMA_S0PAR_OFFSET    0x18UL
#define DMA_S0M0AR_OFFSET   0x1CUL
#define DMA_S0M1AR_OFFSET   0x20UL
#define DMA_S0FCR_OFFSET    0x24UL
// Stream 1
#define DMA_S1CR_OFFSET     0x28UL
#define DMA_S1NDTR_OFFSET   0x2CUL
#define DMA_S1PAR_OFFSET    0x30UL
#define DMA_S1M0AR_OFFSET   0x34UL
#define DMA_S1M1AR_OFFSET   0x38UL
#define DMA_S1FCR_OFFSET    0x3CUL
// ... and so on for streams 2-7. This would be very long.
// For brevity, I'll define a helper function for DMA streams.

// EXTI Registers
#define EXTI_IMR_ADDR       ((volatile tlong*)0x40013C00UL)
#define EXTI_EMR_ADDR       ((volatile tlong*)0x40013C04UL)
#define EXTI_RTSR_ADDR      ((volatile tlong*)0x40013C08UL)
#define EXTI_FTSR_ADDR      ((volatile tlong*)0x40013C0CUL)
#define EXTI_SWIER_ADDR     ((volatile tlong*)0x40013C10UL)
#define EXTI_PR_ADDR        ((volatile tlong*)0x40013C14UL)

// ADC Registers
#define ADC_SR_ADDR         ((volatile tlong*)0x40012000UL)
#define ADC_CR1_ADDR        ((volatile tlong*)0x40012004UL)
#define ADC_CR2_ADDR        ((volatile tlong*)0x40012008UL)
#define ADC_SMPR1_ADDR      ((volatile tlong*)0x4001200CUL)
#define ADC_SMPR2_ADDR      ((volatile tlong*)0x40012010UL)
#define ADC_JOFR1_ADDR      ((volatile tlong*)0x40012014UL)
#define ADC_JOFR2_ADDR      ((volatile tlong*)0x40012018UL)
#define ADC_JOFR3_ADDR      ((volatile tlong*)0x4001201CUL)
#define ADC_JOFR4_ADDR      ((volatile tlong*)0x40012020UL)
#define ADC_HTR_ADDR        ((volatile tlong*)0x40012024UL)
#define ADC_LTR_ADDR        ((volatile tlong*)0x40012028UL)
#define ADC_SQR1_ADDR       ((volatile tlong*)0x4001202CUL)
#define ADC_SQR2_ADDR       ((volatile tlong*)0x40012030UL)
#define ADC_SQR3_ADDR       ((volatile tlong*)0x40012034UL)
#define ADC_JSQR_ADDR       ((volatile tlong*)0x40012038UL)
#define ADC_JDR1_ADDR       ((volatile tlong*)0x4001203CUL)
#define ADC_JDR2_ADDR       ((volatile tlong*)0x40012040UL)
#define ADC_JDR3_ADDR       ((volatile tlong*)0x40012044UL)
#define ADC_JDR4_ADDR       ((volatile tlong*)0x40012048UL)
#define ADC_DR_ADDR         ((volatile tlong*)0x4001204CUL)
#define ADC_CCR_ADDR        ((volatile tlong*)0x40012300UL)

// TIM Registers (Base Addresses)
#define TIM1_BASE_ADDR      0x40010000UL
#define TIM2_BASE_ADDR      0x40000000UL
#define TIM3_BASE_ADDR      0x40000400UL
#define TIM4_BASE_ADDR      0x40000800UL
#define TIM5_BASE_ADDR      0x40000C00UL
#define TIM9_BASE_ADDR      0x40014000UL
#define TIM10_BASE_ADDR     0x40014400UL
#define TIM11_BASE_ADDR     0x40014800UL

// TIM Register Offsets
#define TIM_CR1_OFFSET      0x00UL
#define TIM_CR2_OFFSET      0x04UL // Not all TIMs have this
#define TIM_SMCR_OFFSET     0x08UL // Not all TIMs have this
#define TIM_DIER_OFFSET     0x0CUL
#define TIM_SR_OFFSET       0x10UL
#define TIM_EGR_OFFSET      0x14UL
#define TIM_CCMR1_OFFSET    0x18UL
#define TIM_CCMR2_OFFSET    0x1CUL // Not all TIMs have this
#define TIM_CCER_OFFSET     0x20UL
#define TIM_CNT_OFFSET      0x24UL
#define TIM_PSC_OFFSET      0x28UL
#define TIM_ARR_OFFSET      0x2CUL
#define TIM_RCR_OFFSET      0x30UL // Not all TIMs have this
#define TIM_CCR1_OFFSET     0x34UL
#define TIM_CCR2_OFFSET     0x38UL
#define TIM_CCR3_OFFSET     0x3CUL
#define TIM_CCR4_OFFSET     0x40UL
#define TIM_BDTR_OFFSET     0x44UL // Not all TIMs have this
#define TIM_DCR_OFFSET      0x48UL // Not all TIMs have this
#define TIM_DMAR_OFFSET     0x4CUL // Not all TIMs have this
#define TIM_OR_OFFSET       0x50UL // Not all TIMs have this

// Helper to get TIM register address
#define TIM_REG_ADDR(TIMER_BASE, OFFSET) ((volatile tlong*)(TIMER_BASE + OFFSET))


// USART Registers (Base Addresses)
#define USART1_BASE_ADDR    0x40011000UL
#define USART2_BASE_ADDR    0x40004400UL
#define USART6_BASE_ADDR    0x40011400UL

// USART Register Offsets
#define USART_SR_OFFSET     0x00UL
#define USART_DR_OFFSET     0x04UL
#define USART_BRR_OFFSET    0x08UL
#define USART_CR1_OFFSET    0x0CUL
#define USART_CR2_OFFSET    0x10UL
#define USART_CR3_OFFSET    0x14UL
#define USART_GTPR_OFFSET   0x18UL

// Helper to get USART register address
#define USART_REG_ADDR(USART_BASE, OFFSET) ((volatile tlong*)(USART_BASE + OFFSET))


// I2C Registers (Base Addresses)
#define I2C1_BASE_ADDR      0x40005400UL
#define I2C2_BASE_ADDR      0x40005800UL
#define I2C3_BASE_ADDR      0x40005C00UL

// I2C Register Offsets
#define I2C_CR1_OFFSET      0x00UL
#define I2C_CR2_OFFSET      0x04UL
#define I2C_OAR1_OFFSET     0x08UL
#define I2C_OAR2_OFFSET     0x0CUL
#define I2C_DR_OFFSET       0x10UL
#define I2C_SR1_OFFSET      0x14UL
#define I2C_SR2_OFFSET      0x18UL
#define I2C_CCR_OFFSET      0x1CUL
#define I2C_TRISE_OFFSET    0x20UL
#define I2C_FLTR_OFFSET     0x24UL

// Helper to get I2C register address
#define I2C_REG_ADDR(I2C_BASE, OFFSET) ((volatile tlong*)(I2C_BASE + OFFSET))


// SPI Registers (Base Addresses)
#define SPI1_BASE_ADDR      0x40013000UL
#define SPI2_BASE_ADDR      0x40003800UL
#define SPI3_BASE_ADDR      0x40003C00UL

// SPI Register Offsets
#define SPI_CR1_OFFSET      0x00UL
#define SPI_CR2_OFFSET      0x04UL
#define SPI_SR_OFFSET       0x08UL
#define SPI_DR_OFFSET       0x0CUL
#define SPI_CRCPR_OFFSET    0x10UL
#define SPI_RXCRCR_OFFSET   0x14UL
#define SPI_TXCRCR_OFFSET   0x18UL
#define SPI_I2SCFGR_OFFSET  0x1CUL
#define SPI_I2SPR_OFFSET    0x20UL

// Helper to get SPI register address
#define SPI_REG_ADDR(SPI_BASE, OFFSET) ((volatile tlong*)(SPI_BASE + OFFSET))


// =============================================================================
//                             PRIVATE HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Get the base address of a given GPIO port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return The base address of the GPIO port, or 0 if invalid.
 */
static tlong get_gpio_base_addr(t_port port) {
    tlong base_addr = 0;
    switch (port) {
        case GPIO_PORTA: base_addr = GPIOA_BASE_ADDR; break;
        case GPIO_PORTB: base_addr = GPIOB_BASE_ADDR; break;
        case GPIO_PORTC: base_addr = GPIOC_BASE_ADDR; break;
        case GPIO_PORTD: base_addr = GPIOD_BASE_ADDR; break;
        case GPIO_PORTE: base_addr = GPIOE_BASE_ADDR; break;
        case GPIO_PORTH: base_addr = GPIOH_BASE_ADDR; break;
        default: break; // Should not happen with valid enum
    }
    return base_addr;
}

/**
 * @brief Get the base address of a given USART channel.
 * @param uart_channel The USART channel (1, 2, 6).
 * @return The base address of the USART peripheral, or 0 if invalid.
 */
static tlong get_usart_base_addr(t_uart_channel uart_channel) {
    tlong base_addr = 0;
    switch (uart_channel) {
        case UART_CHANNEL_1: base_addr = USART1_BASE_ADDR; break;
        case UART_CHANNEL_2: base_addr = USART2_BASE_ADDR; break;
        case UART_CHANNEL_6: base_addr = USART6_BASE_ADDR; break;
        default: break;
    }
    return base_addr;
}

/**
 * @brief Get the base address of a given I2C channel.
 * @param i2c_channel The I2C channel (1, 2, 3).
 * @return The base address of the I2C peripheral, or 0 if invalid.
 */
static tlong get_i2c_base_addr(t_i2c_channel i2c_channel) {
    tlong base_addr = 0;
    switch (i2c_channel) {
        case I2C_CHANNEL_1: base_addr = I2C1_BASE_ADDR; break;
        case I2C_CHANNEL_2: base_addr = I2C2_BASE_ADDR; break;
        case I2C_CHANNEL_3: base_addr = I2C3_BASE_ADDR; break;
        default: break;
    }
    return base_addr;
}

/**
 * @brief Get the base address of a given SPI channel.
 * @param spi_channel The SPI channel (1, 2, 3).
 * @return The base address of the SPI peripheral, or 0 if invalid.
 */
static tlong get_spi_base_addr(t_spi_channel spi_channel) {
    tlong base_addr = 0;
    switch (spi_channel) {
        case SPI_CHANNEL_1: base_addr = SPI1_BASE_ADDR; break;
        case SPI_CHANNEL_2: base_addr = SPI2_BASE_ADDR; break;
        case SPI_CHANNEL_3: base_addr = SPI3_BASE_ADDR; break;
        default: break;
    }
    return base_addr;
}

/**
 * @brief Get the base address of a given Timer channel.
 * @param timer_channel The Timer channel (1, 2, 3, 4, 5, 9, 10, 11).
 * @return The base address of the Timer peripheral, or 0 if invalid.
 */
static tlong get_timer_base_addr(t_timer_channel timer_channel) {
    tlong base_addr = 0;
    switch (timer_channel) {
        case TIMER_CHANNEL_1: base_addr = TIM1_BASE_ADDR; break;
        case TIMER_CHANNEL_2: base_addr = TIM2_BASE_ADDR; break;
        case TIMER_CHANNEL_3: base_addr = TIM3_BASE_ADDR; break;
        case TIMER_CHANNEL_4: base_addr = TIM4_BASE_ADDR; break;
        case TIMER_CHANNEL_5: base_addr = TIM5_BASE_ADDR; break;
        case TIMER_CHANNEL_9: base_addr = TIM9_BASE_ADDR; break;
        case TIMER_CHANNEL_10: base_addr = TIM10_BASE_ADDR; break;
        case TIMER_CHANNEL_11: base_addr = TIM11_BASE_ADDR; break;
        default: break;
    }
    return base_addr;
}

/**
 * @brief Get the base address of a given PWM channel's timer.
 * This helper function maps a PWM channel to its underlying timer base address.
 * @param pwm_channel The PWM channel.
 * @return The base address of the timer peripheral associated with the PWM channel, or 0 if invalid.
 */
static tlong get_pwm_timer_base_addr(t_pwm_channel pwm_channel) {
    tlong base_addr = 0;
    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1:
        case PWM_CHANNEL_TIM1_CH2:
        case PWM_CHANNEL_TIM1_CH3:
        case PWM_CHANNEL_TIM1_CH4:
            base_addr = TIM1_BASE_ADDR;
            break;
        case PWM_CHANNEL_TIM2_CH1:
        case PWM_CHANNEL_TIM2_CH2:
        case PWM_CHANNEL_TIM2_CH3:
        case PWM_CHANNEL_TIM2_CH4:
            base_addr = TIM2_BASE_ADDR;
            break;
        case PWM_CHANNEL_TIM3_CH1:
        case PWM_CHANNEL_TIM3_CH2:
        case PWM_CHANNEL_TIM3_CH3:
        case PWM_CHANNEL_TIM3_CH4:
            base_addr = TIM3_BASE_ADDR;
            break;
        case PWM_CHANNEL_TIM4_CH1:
        case PWM_CHANNEL_TIM4_CH2:
        case PWM_CHANNEL_TIM4_CH3:
        case PWM_CHANNEL_TIM4_CH4:
            base_addr = TIM4_BASE_ADDR;
            break;
        case PWM_CHANNEL_TIM5_CH1:
        case PWM_CHANNEL_TIM5_CH2:
        case PWM_CHANNEL_TIM5_CH3:
        case PWM_CHANNEL_TIM5_CH4:
            base_addr = TIM5_BASE_ADDR;
            break;
        case PWM_CHANNEL_TIM9_CH1:
        case PWM_CHANNEL_TIM9_CH2:
            base_addr = TIM9_BASE_ADDR;
            break;
        case PWM_CHANNEL_TIM10_CH1:
            base_addr = TIM10_BASE_ADDR;
            break;
        case PWM_CHANNEL_TIM11_CH1:
            base_addr = TIM11_BASE_ADDR;
            break;
        default:
            break;
    }
    return base_addr;
}

/**
 * @brief Get the base address of a given ICU channel's timer.
 * This helper function maps an ICU channel to its underlying timer base address.
 * @param icu_channel The ICU channel.
 * @return The base address of the timer peripheral associated with the ICU channel, or 0 if invalid.
 */
static tlong get_icu_timer_base_addr(t_icu_channel icu_channel) {
    tlong base_addr = 0;
    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1:
        case ICU_CHANNEL_TIM1_CH2:
        case ICU_CHANNEL_TIM1_CH3:
        case ICU_CHANNEL_TIM1_CH4:
            base_addr = TIM1_BASE_ADDR;
            break;
        case ICU_CHANNEL_TIM2_CH1:
        case ICU_CHANNEL_TIM2_CH2:
        case ICU_CHANNEL_TIM2_CH3:
        case ICU_CHANNEL_TIM2_CH4:
            base_addr = TIM2_BASE_ADDR;
            break;
        case ICU_CHANNEL_TIM3_CH1:
        case ICU_CHANNEL_TIM3_CH2:
        case ICU_CHANNEL_TIM3_CH3:
        case ICU_CHANNEL_TIM3_CH4:
            base_addr = TIM3_BASE_ADDR;
            break;
        case ICU_CHANNEL_TIM4_CH1:
        case ICU_CHANNEL_TIM4_CH2:
        case ICU_CHANNEL_TIM4_CH3:
        case ICU_CHANNEL_TIM4_CH4:
            base_addr = TIM4_BASE_ADDR;
            break;
        case ICU_CHANNEL_TIM5_CH1:
        case ICU_CHANNEL_TIM5_CH2:
        case ICU_CHANNEL_TIM5_CH3:
        case ICU_CHANNEL_TIM5_CH4:
            base_addr = TIM5_BASE_ADDR;
            break;
        case ICU_CHANNEL_TIM9_CH1:
        case ICU_CHANNEL_TIM9_CH2:
            base_addr = TIM9_BASE_ADDR;
            break;
        case ICU_CHANNEL_TIM10_CH1:
            base_addr = TIM10_BASE_ADDR;
            break;
        case ICU_CHANNEL_TIM11_CH1:
            base_addr = TIM11_BASE_ADDR;
            break;
        default:
            break;
    }
    return base_addr;
}


// =============================================================================
//                                 API IMPLEMENTATIONS
// =============================================================================

/**
 * @brief Resets the Watchdog Timer.
 *
 * This function clears (reloads) the watchdog timer to prevent it from expiring.
 * Based on Rules.json, this function is called at the beginning of every API.
 *
 * Note: The provided REGISTER_JSON does not contain explicit Watchdog Timer
 * (IWDG/WWDG) registers for direct manipulation (e.g., a specific "clear" register).
 * Therefore, this implementation is a placeholder. In a real STM32 MCAL, this
 * would involve writing to the IWDG_KR register (Key Register) with 0xAAAA.
 * For now, this function is an empty placeholder as direct register access
 * for WDT reset is not available in the given JSON.
 * Example in rules.json is specific to HOLTEK HT46R24 (ClrWdt()).
 */
void WDT_Reset(void) {
    // WDT_Reset is called in every API as per Rules.json.
    // However, the provided REGISTER_JSON does not include specific
    // registers for Watchdog Timer (WDT) control (e.g., IWDG_KR to write 0xAAAA).
    // This function remains a placeholder for the STM32F401RC based on the given inputs.
    // No action is performed.
}

/**
 * @brief Puts the MCU into sleep mode.
 *
 * This function initiates a low-power sleep mode for the microcontroller.
 * Based on Rules.json, this is intended to stop code execution and most peripherals.
 *
 * Note: The provided REGISTER_JSON contains `PWR_CR` (Power control register)
 * and `PWR_CSR` (Power control/status register), which are used for power
 * management. However, specific bits for directly entering "sleep" or "stop"
 * mode (e.g., SLEEPONEXIT, PDDS, LPDS bits in SCR and CR registers) are not detailed
 * in the register descriptions. The example in Rules.json is specific to HOLTEK HT46R24 (_halt()).
 * Therefore, this implementation is a placeholder.
 */
void Go_to_sleep_mode(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // The provided REGISTER_JSON does not include specific bits/registers
    // for direct "sleep mode" entry like SCB->SCR (SLEEPONEXIT) or PWR_CR low power modes.
    // This function is a placeholder based on the given inputs.
    // In a real STM32 MCAL, this would typically involve setting SCB->SCR register bits
    // and then executing WFI/WFE instruction.
}

/**
 * @brief Enables global interrupts.
 *
 * This function enables the master interrupt enable bit in the ARM Cortex-M processor.
 *
 * Note: Global interrupt control (`__enable_irq()`) is typically handled via CMSIS intrinsics,
 * not direct access to a register listed in the provided JSON.
 */
void Global_interrupt_Enable(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    __enable_irq(); // CMSIS intrinsic for enabling global interrupts
}

/**
 * @brief Disables global interrupts.
 *
 * This function disables the master interrupt enable bit in the ARM Cortex-M processor.
 *
 * Note: Global interrupt control (`__disable_irq()`) is typically handled via CMSIS intrinsics,
 * not direct access to a register listed in the provided JSON.
 */
void Global_interrupt_Disable(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    __disable_irq(); // CMSIS intrinsic for disabling global interrupts
}

/**
 * @brief Initializes the MCU configuration based on system voltage.
 *
 * This function sets up basic MCU configurations, including GPIO, peripheral disables,
 * WDT, and LVR based on the rules.
 *
 * @param volt The system voltage (VOLT_3V or VOLT_5V).
 */
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json

    volatile tlong *reg_ptr;
    tlong base_addr;
    t_port current_port;
    t_pin current_pin;

    // 1. Set all GPIO pins to 0 and verify with while loop
    for (current_port = GPIO_PORTA; current_port <= GPIO_PORTH; current_port++) {
        base_addr = get_gpio_base_addr(current_port);
        if (base_addr == 0) continue; // Skip invalid ports

        // Set all output data register bits to 0
        reg_ptr = GPIO_REG_ADDR(base_addr, GPIO_ODR_OFFSET);
        (*reg_ptr) = 0x00000000UL;

        // Verify (for all 16 pins, though some ports like H only have a few)
        for (current_pin = GPIO_PIN_0; current_pin < 16; current_pin++) {
            // Need to read ODR to verify.
            while (((*reg_ptr) >> current_pin) & 0x1UL) {
                // Pin is still high, try setting to 0 again or handle error
                (*reg_ptr) &= ~(1UL << current_pin);
            }
        }
    }

    // 2. Set all GPIO pins direction to input and verify with while loop
    for (current_port = GPIO_PORTA; current_port <= GPIO_PORTH; current_port++) {
        base_addr = get_gpio_base_addr(current_port);
        if (base_addr == 0) continue; // Skip invalid ports

        // Set MODER to 0x00 for all pins (Input mode)
        reg_ptr = GPIO_REG_ADDR(base_addr, GPIO_MODER_OFFSET);
        (*reg_ptr) = 0x00000000UL; // All pins to input mode (00b per pin)

        // Verify MODER (all bits should be 0 for input mode)
        while ((*reg_ptr) != 0x00000000UL) {
            // Re-attempt setting to input
            (*reg_ptr) = 0x00000000UL;
        }

        // Rule: All input pins have pull-up resistors and wakeup feature enabled (if available)
        // Set PUPDR to 0x01 for all pins (Pull-up mode)
        reg_ptr = GPIO_REG_ADDR(base_addr, GPIO_PUPDR_OFFSET);
        (*reg_ptr) = 0x55555555UL; // Set all pins to pull-up mode (01b per pin)

        // Wakeup feature: Not directly available in GPIO registers in the JSON, typically EXTI or specific WKUP pins.
        // SYSCFG_EXTICR registers are available for interrupt configuration, but not a general "wakeup feature enable".
        // Comment: Wakeup feature configuration for input pins is not directly supported by the provided GPIO registers.
    }

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable(); // Disable global interrupts

    // Disable ADC: ADC_CR2 ADON bit
    // Set to 0 to disable ADC
    (*ADC_CR2_ADDR) &= ~(1UL << 0); // Clear ADON bit (ADON is bit 0 in CR2 for ADC on/off)

    // Disable UART: USARTx_CR1 UE bit
    (*USART_REG_ADDR(USART1_BASE_ADDR, USART_CR1_OFFSET)) &= ~(1UL << 13); // USART1_CR1 UE (USART Enable)
    (*USART_REG_ADDR(USART2_BASE_ADDR, USART_CR1_OFFSET)) &= ~(1UL << 13); // USART2_CR1 UE
    (*USART_REG_ADDR(USART6_BASE_ADDR, USART_CR1_OFFSET)) &= ~(1UL << 13); // USART6_CR1 UE

    // Disable I2C: I2Cx_CR1 PE bit
    (*I2C_REG_ADDR(I2C1_BASE_ADDR, I2C_CR1_OFFSET)) &= ~(1UL << 0); // I2C1_CR1 PE (Peripheral Enable)
    (*I2C_REG_ADDR(I2C2_BASE_ADDR, I2C_CR1_OFFSET)) &= ~(1UL << 0); // I2C2_CR1 PE
    (*I2C_REG_ADDR(I2C3_BASE_ADDR, I2C_CR1_OFFSET)) &= ~(1UL << 0); // I2C3_CR1 PE

    // Disable SPI: SPIx_CR1 SPE bit
    (*SPI_REG_ADDR(SPI1_BASE_ADDR, SPI_CR1_OFFSET)) &= ~(1UL << 6); // SPI1_CR1 SPE (SPI Enable)
    (*SPI_REG_ADDR(SPI2_BASE_ADDR, SPI_CR1_OFFSET)) &= ~(1UL << 6); // SPI2_CR1 SPE
    (*SPI_REG_ADDR(SPI3_BASE_ADDR, SPI_CR1_OFFSET)) &= ~(1UL << 6); // SPI3_CR1 SPE

    // Disable TIMERS: TIMx_CR1 CEN bit
    (*TIM_REG_ADDR(TIM1_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM1_CR1 CEN (Counter Enable)
    (*TIM_REG_ADDR(TIM2_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM2_CR1 CEN
    (*TIM_REG_ADDR(TIM3_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM3_CR1 CEN
    (*TIM_REG_ADDR(TIM4_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM4_CR1 CEN
    (*TIM_REG_ADDR(TIM5_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM5_CR1 CEN
    (*TIM_REG_ADDR(TIM9_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM9_CR1 CEN
    (*TIM_REG_ADDR(TIM10_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM10_CR1 CEN
    (*TIM_REG_ADDR(TIM11_BASE_ADDR, TIM_CR1_OFFSET)) &= ~(1UL << 0); // TIM11_CR1 CEN

    // Disable DMA: DMA1_SxCR EN bit, DMA2_SxCR EN bit
    // This requires iterating through all streams. For simplicity, setting main DMA enable if available,
    // otherwise commenting on stream-level control.
    // The provided JSON has DMA1_S0CR to DMA1_S7CR and DMA2_S0CR to DMA2_S7CR.
    // Each of these stream CRs has an EN bit (bit 0) to enable/disable the stream.
    // Disabling all DMA streams for DMA1 and DMA2
    int i;
    for (i = 0; i < 8; i++) {
        // DMA1 streams
        volatile tlong* dma1_scr_addr = (volatile tlong*)(DMA1_BASE_ADDR + DMA_S0CR_OFFSET + (i * 0x18UL)); // Each stream block is 0x18 apart
        (*dma1_scr_addr) &= ~(1UL << 0); // Clear EN bit (Stream Enable)
        // DMA2 streams
        volatile tlong* dma2_scr_addr = (volatile tlong*)(DMA2_BASE_ADDR + DMA_S0CR_OFFSET + (i * 0x18UL));
        (*dma2_scr_addr) &= ~(1UL << 0); // Clear EN bit (Stream Enable)
    }

    // Disable CRC: CRC_CR RESET bit (writing 1 to reset, not disable. No explicit disable bit in CRC_CR description)
    // Comment: No explicit 'disable' bit for CRC unit in provided JSON. CRC_CR has a reset bit.

    // 4. Enable WDT (Watchdog Timer)
    // 5. Clear WDT timer
    // 6. Set WDT period >= 8 msec
    // The provided REGISTER_JSON does not explicitly contain WDT (IWDG/WWDG) registers.
    // The rules mention "ClrWdt()" for HOLTEK, but no STM32 equivalent registers are present.
    // Therefore, WDT configuration cannot be implemented based on the provided registers.
    // In a real STM32 system:
    // IWDG->KR = 0xCCCC; // Enable LSI oscillator and start IWDG
    // IWDG->PR = 0xX;    // Set prescaler
    // IWDG->RLR = Y;    // Set reload value for >= 8ms period
    // WDT_Reset(); // IWDG->KR = 0xAAAA;

    // 7. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // 8. Enable LVR (Low Voltage Reset)
    tlong pwr_cr_val = (*PWR_CR_ADDR);
    pwr_cr_val &= ~(0x7UL << 5); // Clear PVLS[2:0] bits (bits 7:5) for PVD level selection

    // PVLS[2:0] bits in PWR_CR (Power control register)
    // V_PVD(mV)  PVLS[2:0]  Description (STM32F4 Reference Manual - simplified)
    // 2.0        000        PVD level 0
    // 2.1        001        PVD level 1
    // 2.3        010        PVD level 2
    // 2.4        011        PVD level 3
    // 2.6        100        PVD level 4
    // 2.7        101        PVD level 5
    // 2.9        110        PVD level 6
    // 3.0        111        PVD level 7 (highest)
    // We need 2V for 3V system and 3.5V for 5V system.
    // Assuming 'PVD level 0' is 2.0V and 'PVD level 7' is 3.0V.
    // For 3.5V, we might need a higher level than default STM32 PVD levels,
    // which usually cap around 3V. This might require external supervisor.
    // Based *only* on the provided JSON and common STM32 PVLS interpretation:
    // 2V for 3V system: use PVLS level 0 (000)
    // 3.5V for 5V system: Highest available is 3.0V (PVLS level 7). We will set to max if 3.5V is not directly supported by PVLS.
    // Using an approximation:
    if (volt == VOLT_3V) {
        pwr_cr_val |= (0UL << 5); // Set PVLS to level 0 (approx 2.0V)
    } else if (volt == VOLT_5V) {
        pwr_cr_val |= (7UL << 5); // Set PVLS to level 7 (approx 3.0V, highest available PVD on chip)
        // Comment: 3.5V LVR is not directly achievable with internal PVD levels according to common STM32 datasheets;
        // setting to maximum internal PVD level (approx 3.0V) if 3.5V is specified for a 5V system.
    }
    pwr_cr_val |= (1UL << 4); // Set PVDEN bit (Power Voltage Detector Enable)

    (*PWR_CR_ADDR) = pwr_cr_val;

    // 9. Clear WDT again
    WDT_Reset(); // Call WDT_Reset as per Rules.json
}

/**
 * @brief Initializes the Low Voltage Detection (LVD).
 *
 * Configures the LVD threshold.
 */
void LVD_Init(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // LVD (PVD on STM32) initialization.
    // This typically involves enabling the PWR clock and then configuring PVD.
    // The PWR clock is not directly enabled by a register in the provided JSON.
    // (It's usually in RCC_APB1ENR for STM32, but that register's description doesn't mention PWR)
    // Example: (*RCC_APB1ENR_ADDR) |= (1UL << 28); // Enable PWR clock (bit 28 for PWR)

    // For LVD_Init, we set a default threshold (e.g., LVD_THRESHOLD_2V as a general safe level)
    // (PVLS bits in PWR_CR)
    // Clear PVLS bits first
    (*PWR_CR_ADDR) &= ~(0x7UL << 5); // Clear PVLS[2:0]
    // Set a default threshold, e.g., 2V (PVLS level 0)
    (*PWR_CR_ADDR) |= (0UL << 5); // PVLS = 000 for approx 2.0V
}

/**
 * @brief Gets the current LVD status based on a given threshold.
 *
 * This function is specified to take a threshold level, but the provided
 * `REGISTER_JSON` for `PWR_CSR` only has `power_status` function and `PVDO` bit
 * (PVD Output) which indicates if the voltage is below the *configured* threshold,
 * not to check against an arbitrary threshold on demand.
 *
 * @param lvd_thresholdLevel The threshold level to get status for.
 * Note: This parameter is currently ignored as the hardware register only
 * reports against its pre-configured threshold.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // The PWR_CSR register's PVDO bit (bit 2) indicates if V_IN below PVD threshold.
    // It does not allow reading status for an arbitrary threshold without reconfiguring PVD.
    // The parameter lvd_thresholdLevel cannot be directly used to "get" status against it.
    // This function can only report if the current voltage is below the *already configured* threshold.
    if (((*PWR_CSR_ADDR) >> 2) & 0x1UL) {
        // Voltage is below the configured PVD threshold
        // (In a real system, you might return a boolean or status code)
    } else {
        // Voltage is above the configured PVD threshold
    }
    // Comment: The `lvd_thresholdLevel` parameter is not utilized for direct status check
    // as the hardware's LVD (PVD) registers only report status against a pre-configured threshold.
}

/**
 * @brief Enables the Low Voltage Detection (LVD).
 */
void LVD_Enable(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Set PVDEN bit in PWR_CR (bit 4)
    (*PWR_CR_ADDR) |= (1UL << 4);
}

/**
 * @brief Disables the Low Voltage Detection (LVD).
 */
void LVD_Disable(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Clear PVDEN bit in PWR_CR (bit 4)
    (*PWR_CR_ADDR) &= ~(1UL << 4);
}

/**
 * @brief Clears the LVD flag.
 *
 * Note: The provided `REGISTER_JSON` does not contain a specific LVD clear flag register.
 * For STM32's PVD, flags are usually cleared by specific sequences or they are level-sensitive.
 * `PWR_CSR` only provides status. This function will be a placeholder.
 * @param lvd_channel The LVD channel (ignored as LVD is typically single).
 */
void LVD_ClearFlag(t_lvd_channel lvd_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)lvd_channel; // Parameter is unused based on available registers
    // The provided REGISTER_JSON does not contain a register specifically for clearing
    // LVD/PVD flags. PWR_CSR reports status but doesn't have a clearable flag bit.
    // This function is a placeholder.
}

/**
 * @brief Initializes a UART channel.
 *
 * Configures baud rate, data length, stop bits, and parity.
 *
 * @param uart_channel The UART channel to initialize (USART1, USART2, USART6).
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data length (8N1 or 9N1).
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting.
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_usart_base_addr(uart_channel);
    if (base_addr == 0) return;

    volatile tlong *cr1_reg = USART_REG_ADDR(base_addr, USART_CR1_OFFSET);
    volatile tlong *brr_reg = USART_REG_ADDR(base_addr, USART_BRR_OFFSET);
    volatile tlong *cr2_reg = USART_REG_ADDR(base_addr, USART_CR2_OFFSET);

    // Disable UART before configuration (UE bit 13 in CR1)
    (*cr1_reg) &= ~(1UL << 13);

    // Configure Baud Rate (BRR)
    // Calculation depends on PCLK speed. Assuming a default PCLK for example.
    // For simplicity and without clock config in JSON, map directly to nominal values.
    tword usartdiv = 0; // Placeholder, real calculation needs clock frequency
    switch (uart_baud_rate) {
        case UART_BAUD_RATE_9600: usartdiv = 0x683; break; // Example for 16MHz PCLK
        case UART_BAUD_RATE_19200: usartdiv = 0x341; break;
        case UART_BAUD_RATE_115200: usartdiv = 0x5B; break; // Example for 16MHz PCLK
        default: usartdiv = 0x683; break; // Default to 9600
    }
    (*brr_reg) = usartdiv;

    // Configure Data Length (M bit 12 in CR1)
    // 0: 1 start bit, 8 data bits, n stop bits
    // 1: 1 start bit, 9 data bits, n stop bits
    if (uart_data_length == UART_DATA_LENGTH_9N1) {
        (*cr1_reg) |= (1UL << 12); // Set M bit for 9 data bits
    } else {
        (*cr1_reg) &= ~(1UL << 12); // Clear M bit for 8 data bits
    }

    // Configure Stop Bits (STOP bits 13:12 in CR2)
    // 00: 1 stop bit
    // 01: 0.5 stop bit
    // 10: 2 stop bits
    // 11: 1.5 stop bits
    (*cr2_reg) &= ~(0x3UL << 12); // Clear STOP bits
    switch (uart_stop_bit) {
        case UART_STOP_BIT_0_5: (*cr2_reg) |= (0x1UL << 12); break;
        case UART_STOP_BIT_2:   (*cr2_reg) |= (0x2UL << 12); break;
        case UART_STOP_BIT_1_5: (*cr2_reg) |= (0x3UL << 12); break;
        case UART_STOP_BIT_1:
        default: break; // Default to 1 stop bit (00)
    }

    // Configure Parity (PCE bit 10, PS bit 9 in CR1)
    // PCE=1, PS=0: Even parity
    // PCE=1, PS=1: Odd parity
    // PCE=0: No parity
    (*cr1_reg) &= ~((1UL << 10) | (1UL << 9)); // Clear PCE and PS bits
    switch (uart_parity) {
        case UART_PARITY_EVEN:
            (*cr1_reg) |= (1UL << 10); // PCE = 1
            break;
        case UART_PARITY_ODD:
            (*cr1_reg) |= (1UL << 10) | (1UL << 9); // PCE = 1, PS = 1
            break;
        case UART_PARITY_NONE:
        default: break;
    }
}

/**
 * @brief Enables a UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_usart_base_addr(uart_channel);
    if (base_addr == 0) return;
    volatile tlong *cr1_reg = USART_REG_ADDR(base_addr, USART_CR1_OFFSET);
    (*cr1_reg) |= (1UL << 13); // Set UE bit (USART Enable)
}

/**
 * @brief Disables a UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_usart_base_addr(uart_channel);
    if (base_addr == 0) return;
    volatile tlong *cr1_reg = USART_REG_ADDR(base_addr, USART_CR1_OFFSET);
    (*cr1_reg) &= ~(1UL << 13); // Clear UE bit (USART Enable)
}

/**
 * @brief Performs a dummy UART update.
 * @param uart_channel The UART channel to update.
 * Note: The API suggests "update status each period", but no specific "update"
 * register or function is available in the provided JSON. This function will be a placeholder.
 */
void UART_Update(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)uart_channel; // Parameter is unused based on available registers
    // No specific UART "update" register is defined in the provided JSON.
    // This function is a placeholder.
}

/**
 * @brief Sends a single byte over UART.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_usart_base_addr(uart_channel);
    if (base_addr == 0) return;
    volatile tlong *sr_reg = USART_REG_ADDR(base_addr, USART_SR_OFFSET);
    volatile tlong *dr_reg = USART_REG_ADDR(base_addr, USART_DR_OFFSET);

    // Wait until Transmit Data Register Empty (TXE) flag is set (SR bit 7)
    while (!(((*sr_reg) >> 7) & 0x1UL));
    (*dr_reg) = byte; // Write data to DR
    // Wait until Transmission Complete (TC) flag is set (SR bit 6)
    while (!(((*sr_reg) >> 6) & 0x1UL));
}

/**
 * @brief Sends a frame of data over UART.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over UART.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    while (*str != '\0') {
        UART_send_byte(uart_channel, (tbyte)*str++);
    }
}

/**
 * @brief Gets a single byte from UART.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_usart_base_addr(uart_channel);
    if (base_addr == 0) return 0;
    volatile tlong *sr_reg = USART_REG_ADDR(base_addr, USART_SR_OFFSET);
    volatile tlong *dr_reg = USART_REG_ADDR(base_addr, USART_DR_OFFSET);

    // Wait until Receive Data Register Not Empty (RXNE) flag is set (SR bit 5)
    while (!(((*sr_reg) >> 5) & 0x1UL));
    return (tbyte)(*dr_reg); // Read data from DR
}

/**
 * @brief Gets a frame of data from UART.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of the buffer.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    int i;
    for (i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Gets a null-terminated string from UART.
 * This function will block until max_length bytes are received or a specific termination
 * character is received (not defined by API, assuming max_length).
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the string.
 * @param max_length Maximum length of the buffer.
 * @return Number of bytes received.
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tbyte bytes_received = 0;
    for (bytes_received = 0; bytes_received < max_length - 1; bytes_received++) {
        buffer[bytes_received] = (char)UART_Get_Byte(uart_channel);
        // Assuming a null terminator is desired for a "string"
        // No specific termination condition given, so filling up to max_length-1
        // or until a typical string terminator (e.g., '\n' or '\r')
        // For simplicity, just filling max_length-1 bytes and null-terminating.
    }
    buffer[bytes_received] = '\0';
    return bytes_received;
}

/**
 * @brief Clears UART flags.
 *
 * For STM32 USART, flags in SR are typically cleared by reading SR followed by reading DR,
 * or by writing to specific bits in SR (if applicable).
 * The API doesn't specify which flags to clear, assuming all pending flags.
 * @param uart_channel The UART channel to clear flags for.
 */
void UART_ClearFlag(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_usart_base_addr(uart_channel);
    if (base_addr == 0) return;
    volatile tlong *sr_reg = USART_REG_ADDR(base_addr, USART_SR_OFFSET);

    // Clear specific flags by reading SR followed by DR (for RXNE, TC)
    // For other flags (ORE, NE, FE, PE), they are cleared by reading SR
    (void)(*sr_reg); // Read SR
    // For RXNE and TC, reading DR after SR clears them.
    // For ORE, NE, FE, PE, they are cleared by reading SR.
    // DUMMY read to ensure RXNE clears if it was set
    tbyte dummy_read = (tbyte)(*USART_REG_ADDR(base_addr, USART_DR_OFFSET));
    (void)dummy_read; // Suppress unused warning
}

/**
 * @brief Initializes an I2C channel.
 *
 * Configures clock speed, device address, ACK, and data length.
 *
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (Standard or Fast mode).
 * @param i2c_device_address The device's own address.
 * @param i2c_ack Acknowledge control (Enable/Disable).
 * @param i2c_datalength Placeholder for data length.
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_i2c_base_addr(i2c_channel);
    if (base_addr == 0) return;

    volatile tlong *cr1_reg = I2C_REG_ADDR(base_addr, I2C_CR1_OFFSET);
    volatile tlong *cr2_reg = I2C_REG_ADDR(base_addr, I2C_CR2_OFFSET);
    volatile tlong *oar1_reg = I2C_REG_ADDR(base_addr, I2C_OAR1_OFFSET);
    volatile tlong *ccr_reg = I2C_REG_ADDR(base_addr, I2C_CCR_OFFSET);
    volatile tlong *trise_reg = I2C_REG_ADDR(base_addr, I2C_TRISE_OFFSET);

    // Disable I2C peripheral before configuration (PE bit 0 in CR1)
    (*cr1_reg) &= ~(1UL << 0);

    // Rules: Always use fast mode
    // Configure Clock Control Register (CCR)
    // F/S bit 15: 0 for Standard, 1 for Fast Mode
    // DUTY bit 14: 0 for Fm mode Tlow/Thigh = 2, 1 for Fm mode Tlow/Thigh = 16/9
    // CCR[11:0] for clock control in Sm or Fm
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST) {
        (*ccr_reg) |= (1UL << 15); // Fast mode
        // Assuming PCLK1 = 16MHz for I2C (typical for APB1 on STM32F401)
        // For 400kHz Fast Mode (CCR = PCLK1 / (2 * I2C_speed) if DUTY=0)
        // Or CCR = PCLK1 / (25 * I2C_speed) if DUTY=1 (16/9)
        // Let's pick 400kHz: CCR = 16MHz / (2 * 400kHz) = 20.
        // Or for DUTY=1: CCR = 16MHz / (25 * 400kHz) = 1.6 -> min value is 1
        // Let's use DUTY=0 for simpler calculation and set CCR to 20 for 400kHz.
        (*ccr_reg) &= ~(1UL << 14); // DUTY = 0 (Tlow/Thigh = 2)
        (*ccr_reg) |= 20UL; // 400kHz
    } else { // Standard Mode
        (*ccr_reg) &= ~(1UL << 15); // Standard mode
        // For 100kHz Standard Mode: CCR = PCLK1 / (2 * I2C_speed) = 16MHz / (2 * 100kHz) = 80.
        (*ccr_reg) |= 80UL; // 100kHz
    }

    // Rules: Always use maximum timeout.
    // The provided JSON does not list any explicit timeout registers for I2C.
    // Comment: Max timeout setting is not directly supported by registers in provided JSON.

    // Configure TRISE (Rise time register)
    // TRISE = (Input clock freq / 1000000) + 1 for standard mode (100kHz)
    // TRISE = ((Input clock freq / 1000000) * 300 / 1000) + 1 for fast mode (400kHz)
    // For 16MHz PCLK:
    // Sm: TRISE = 16 + 1 = 17
    // Fm: TRISE = (16 * 300 / 1000) + 1 = 4.8 + 1 = 5.8 => 6
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST) {
        (*trise_reg) = 6UL;
    } else {
        (*trise_reg) = 17UL;
    }


    // Configure Own Address Register 1 (OAR1)
    // Acknowledging the rule "Addressing Mode equals Device Address". This is specific for slave mode.
    // Assuming 7-bit addressing mode for simplicity. ADDMODE bit 15, ADD7[7:1] bits.
    (*oar1_reg) = (i2c_device_address & 0x7FUL) << 1; // Set 7-bit address, ADDM=0 for 7-bit
    (*oar1_reg) |= (1UL << 14); // Always set bit 14 as per manual for 7-bit addressing.

    // Configure Acknowledge (ACK bit 10 in CR1)
    if (i2c_ack == I2C_ACK_ENABLE) {
        (*cr1_reg) |= (1UL << 10); // Set ACK bit
    } else {
        (*cr1_reg) &= ~(1UL << 10); // Clear ACK bit
    }

    // Data Length: This parameter (t_i2c_datalength) is unusual for I2C_Init.
    // I2C communication is byte-by-byte. The data length is determined during actual transfer.
    // The provided JSON doesn't have a specific register bit for "data length" in init.
    // It might refer to packet size but this is handled by higher layers or transfer APIs.
    // Comment: The `i2c_datalength` parameter is not directly mapped to a specific I2C register
    // configuration in the provided JSON. Data length is handled during read/write operations.

    // Rules: Always generate a repeated start condition instead of stop between transactions.
    // This is handled during transfer, not initialisation. This means START bit must be set
    // again without clearing STOP bit in CR1.
}

/**
 * @brief Enables an I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_i2c_base_addr(i2c_channel);
    if (base_addr == 0) return;
    volatile tlong *cr1_reg = I2C_REG_ADDR(base_addr, I2C_CR1_OFFSET);
    (*cr1_reg) |= (1UL << 0); // Set PE bit (Peripheral Enable)
}

/**
 * @brief Disables an I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_i2c_base_addr(i2c_channel);
    if (base_addr == 0) return;
    volatile tlong *cr1_reg = I2C_REG_ADDR(base_addr, I2C_CR1_OFFSET);
    (*cr1_reg) &= ~(1UL << 0); // Clear PE bit (Peripheral Enable)
}

/**
 * @brief Performs a dummy I2C update.
 * @param i2c_channel The I2C channel to update.
 * Note: The API suggests "update status each period", but no specific "update"
 * register or function is available in the provided JSON. This function will be a placeholder.
 */
void I2C_Update(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)i2c_channel; // Parameter is unused based on available registers
    // No specific I2C "update" register is defined in the provided JSON.
    // This function is a placeholder.
}

/**
 * @brief Sends a single byte over I2C.
 * This is a low-level function that assumes I2C master mode and a start condition has been generated.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_i2c_base_addr(i2c_channel);
    if (base_addr == 0) return;

    volatile tlong *sr1_reg = I2C_REG_ADDR(base_addr, I2C_SR1_OFFSET);
    volatile tlong *dr_reg = I2C_REG_ADDR(base_addr, I2C_DR_OFFSET);

    // Wait until Transmit data register empty (TxE) (SR1 bit 7)
    // or until Byte Transfer Finished (BTF) (SR1 bit 2)
    while (!(((*sr1_reg) >> 7) & 0x1UL) && !(((*sr1_reg) >> 2) & 0x1UL));
    (*dr_reg) = byte; // Write data to DR
}

/**
 * @brief Sends a frame of data over I2C.
 * This is a basic implementation assuming master mode and target address is already sent.
 * It does not handle START/STOP/ACK.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data to send.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // This function needs higher level I2C master control for Start, Address, Stop.
    // For now, it sends bytes sequentially assuming the bus is ready.
    for (int i = 0; i < length; i++) {
        I2C_send_byte(i2c_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over I2C.
 * This is a basic implementation assuming master mode and target address is already sent.
 * It does not handle START/STOP/ACK.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // This function needs higher level I2C master control for Start, Address, Stop.
    // For now, it sends bytes sequentially assuming the bus is ready.
    while (*str != '\0') {
        I2C_send_byte(i2c_channel, (tbyte)*str++);
    }
}

/**
 * @brief Gets a single byte from I2C.
 * This is a low-level function that assumes I2C master mode and a read operation is in progress.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_i2c_base_addr(i2c_channel);
    if (base_addr == 0) return 0;

    volatile tlong *sr1_reg = I2C_REG_ADDR(base_addr, I2C_SR1_OFFSET);
    volatile tlong *dr_reg = I2C_REG_ADDR(base_addr, I2C_DR_OFFSET);

    // Wait until Receive data register not empty (RxNE) (SR1 bit 6)
    while (!(((*sr1_reg) >> 6) & 0x1UL));
    return (tbyte)(*dr_reg); // Read data from DR
}

/**
 * @brief Gets a frame of data from I2C.
 * This is a basic implementation assuming master mode and read operation setup.
 * It does not handle START/STOP/ACK for multi-byte reads.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of the buffer.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // This function needs higher level I2C master control for Start, Address, Stop, ACK.
    // For now, it reads bytes sequentially.
    int i;
    for (i = 0; i < max_length; i++) {
        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
        // For last byte in multi-byte read, NACK typically sent, then STOP/Repeated START.
        // This basic implementation doesn't cover that.
    }
}

/**
 * @brief Gets a null-terminated string from I2C.
 * Similar to I2C_Get_frame, this function is a basic wrapper.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the string.
 * @param max_length Maximum length of the buffer.
 * @return Number of bytes received.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // This function needs higher level I2C master control.
    tbyte bytes_received = 0;
    for (bytes_received = 0; bytes_received < max_length - 1; bytes_received++) {
        buffer[bytes_received] = (char)I2C_Get_Byte(i2c_channel);
        // Assuming a null terminator is desired for a "string"
        // No specific termination condition given, so filling up to max_length-1
    }
    buffer[bytes_received] = '\0';
    return bytes_received;
}

/**
 * @brief Clears I2C flags.
 *
 * For STM32 I2C, flags in SR1 and SR2 are cleared by specific read/write sequences.
 * The API doesn't specify which flags to clear, assuming all pending flags.
 * @param i2c_channel The I2C channel to clear flags for.
 */
void I2C_ClearFlag(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_i2c_base_addr(i2c_channel);
    if (base_addr == 0) return;
    volatile tlong *sr1_reg = I2C_REG_ADDR(base_addr, I2C_SR1_OFFSET);
    volatile tlong *sr2_reg = I2C_REG_ADDR(base_addr, I2C_SR2_OFFSET);

    // Clear specific flags by reading SR1 followed by SR2 (for SB, ADDR, BTF)
    // Other flags are cleared by specific sequences or writing to CR1.
    (void)(*sr1_reg); // Read SR1
    (void)(*sr2_reg); // Read SR2 (clears some flags like ADDR, STOPF)
    // Comment: Other flags might require specific bit manipulation in CR1 or repeated sequences.
    // This implementation provides a basic clear for main status flags.
}

/**
 * @brief Initializes an SPI channel.
 *
 * Configures mode, clock polarity/phase, data frame format, and bit order.
 * Applies rules for fast speed, software SS, full duplex, and CRC.
 *
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode Master/Slave mode.
 * @param spi_cpol Clock Polarity.
 * @param spi_cpha Clock Phase.
 * @param spi_dff Data Frame Format.
 * @param spi_bit_order Bit Order (MSB/LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_spi_base_addr(spi_channel);
    if (base_addr == 0) return;

    volatile tlong *cr1_reg = SPI_REG_ADDR(base_addr, SPI_CR1_OFFSET);
    volatile tlong *cr2_reg = SPI_REG_ADDR(base_addr, SPI_CR2_OFFSET);

    // Disable SPI peripheral before configuration (SPE bit 6 in CR1)
    (*cr1_reg) &= ~(1UL << 6);

    // Configure Master/Slave Mode (MSTR bit 2 in CR1)
    if (spi_mode == SPI_MODE_MASTER) {
        (*cr1_reg) |= (1UL << 2); // Master mode
    } else {
        (*cr1_reg) &= ~(1UL << 2); // Slave mode
    }

    // Configure Clock Polarity (CPOL bit 1 in CR1)
    if (spi_cpol == SPI_CPOL_HIGH) {
        (*cr1_reg) |= (1UL << 1);
    } else {
        (*cr1_reg) &= ~(1UL << 1);
    }

    // Configure Clock Phase (CPHA bit 0 in CR1)
    if (spi_cpha == SPI_CPHA_2EDGE) {
        (*cr1_reg) |= (1UL << 0);
    } else {
        (*cr1_reg) &= ~(1UL << 0);
    }

    // Configure Data Frame Format (DFF bit 11 in CR1)
    if (spi_dff == SPI_DFF_16BIT) {
        (*cr1_reg) |= (1UL << 11); // 16-bit data frame format
    } else {
        (*cr1_reg) &= ~(1UL << 11); // 8-bit data frame format
    }

    // Configure Bit Order (LSBFIRST bit 7 in CR1)
    if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST) {
        (*cr1_reg) |= (1UL << 7); // LSB first
    } else {
        (*cr1_reg) &= ~(1UL << 7); // MSB first
    }

    // Rules: Always use fast speed
    // Baud Rate Control (BR[2:0] bits 5:3 in CR1)
    // Values from 000 (PCLK/2) to 111 (PCLK/256). Setting to 000 for PCLK/2 (fastest)
    (*cr1_reg) &= ~(0x7UL << 3); // Clear BR bits (sets to PCLK/2)

    // Rules: Slave Select always software-controlled (SSM bit 9, SSI bit 8 in CR1)
    (*cr1_reg) |= (1UL << 9); // SSM = 1 (Software Slave Management enabled)
    (*cr1_reg) |= (1UL << 8); // SSI = 1 (Internal slave select set to 1, acting as NSS HIGH)

    // Rules: Always use full duplex (BIDIMODE bit 15 in CR1, RXONLY bit 10 in CR1)
    // BIDIMODE=0 for 2-line unidirectional, RXONLY=0 for full-duplex transmit and receive
    (*cr1_reg) &= ~(1UL << 15); // Clear BIDIMODE for 2-line unidirectional (full duplex)
    (*cr1_reg) &= ~(1UL << 10); // Clear RXONLY for full duplex

    // Rules: Always enable CRC (CRCEN bit 13 in CR1)
    (*cr1_reg) |= (1UL << 13); // Enable CRC calculation

    // Configure CR2 for default settings/no specific rules
    (*cr2_reg) = 0x00000000UL;
}

/**
 * @brief Enables an SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_spi_base_addr(spi_channel);
    if (base_addr == 0) return;
    volatile tlong *cr1_reg = SPI_REG_ADDR(base_addr, SPI_CR1_OFFSET);
    (*cr1_reg) |= (1UL << 6); // Set SPE bit (SPI Enable)
}

/**
 * @brief Disables an SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_spi_base_addr(spi_channel);
    if (base_addr == 0) return;
    volatile tlong *cr1_reg = SPI_REG_ADDR(base_addr, SPI_CR1_OFFSET);
    (*cr1_reg) &= ~(1UL << 6); // Clear SPE bit (SPI Enable)
}

/**
 * @brief Performs a dummy SPI update.
 * Note: The API suggests "update status each period", but no specific "update"
 * register or function is available in the provided JSON. This function will be a placeholder.
 */
void SPI_Update(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // No specific SPI "update" register is defined in the provided JSON.
    // This function is a placeholder.
}

/**
 * @brief Sends a single byte over SPI.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_spi_base_addr(spi_channel);
    if (base_addr == 0) return;
    volatile tlong *sr_reg = SPI_REG_ADDR(base_addr, SPI_SR_OFFSET);
    volatile tlong *dr_reg = SPI_REG_ADDR(base_addr, SPI_DR_OFFSET);

    // Wait until Transmit buffer empty (TXE) flag is set (SR bit 1)
    while (!(((*sr_reg) >> 1) & 0x1UL));
    (*dr_reg) = byte; // Write data to DR
    // Wait until BSY flag is cleared (SR bit 7)
    while (((*sr_reg) >> 7) & 0x1UL);
}

/**
 * @brief Sends a frame of data over SPI.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over SPI.
 * @param spi_channel The SPI channel to use.
 * @param str Pointer to the string.
 */
void SPI_send_string(t_spi_channel spi_channel, const char *str) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    while (*str != '\0') {
        SPI_Send_Byte(spi_channel, (tbyte)*str++);
    }
}

/**
 * @brief Gets a single byte from SPI.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_spi_base_addr(spi_channel);
    if (base_addr == 0) return 0;
    volatile tlong *sr_reg = SPI_REG_ADDR(base_addr, SPI_SR_OFFSET);
    volatile tlong *dr_reg = SPI_REG_ADDR(base_addr, SPI_DR_OFFSET);

    // Wait until Receive buffer not empty (RXNE) flag is set (SR bit 0)
    while (!((*sr_reg) & 0x1UL));
    return (tbyte)(*dr_reg); // Read data from DR
}

/**
 * @brief Gets a frame of data from SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of the buffer.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    int i;
    for (i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Gets a null-terminated string from SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the string.
 * @param max_length Maximum length of the buffer.
 * @return Number of bytes received.
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tbyte bytes_received = 0;
    for (bytes_received = 0; bytes_received < max_length - 1; bytes_received++) {
        buffer[bytes_received] = (char)SPI_Get_Byte(spi_channel);
    }
    buffer[bytes_received] = '\0';
    return bytes_received;
}

/**
 * @brief Clears SPI flags.
 *
 * For STM32 SPI, flags in SR are cleared by reading SR followed by reading DR (for RXNE)
 * or by writing to specific registers.
 * The API doesn't specify which flags to clear.
 * @param spi_channel The SPI channel to clear flags for.
 */
void SPI_ClearFlag(t_spi_channel spi_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_spi_base_addr(spi_channel);
    if (base_addr == 0) return;
    volatile tlong *sr_reg = SPI_REG_ADDR(base_addr, SPI_SR_OFFSET);

    // Read SR. Some flags clear on read.
    (void)(*sr_reg);
    // For RXNE, reading DR also clears it.
    // Comment: Specific flag clearing might require more context (e.g., specific error flags).
}


/**
 * @brief Initializes an External Interrupt channel.
 *
 * Configures the trigger edge for the specified EXTI line.
 * Assumes the GPIO pin for the EXTI line is already configured as input.
 *
 * @param external_int_channel The EXTI line number (0-15).
 * @param external_int_edge The desired trigger edge (rising, falling, or both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json

    // Configure SYSCFG_EXTICRx to select the source input for the EXTIx line.
    // SYSCFG_EXTICR1: EXTI0-3, SYSCFG_EXTICR2: EXTI4-7, SYSCFG_EXTICR3: EXTI8-11, SYSCFG_EXTICR4: EXTI12-15
    // Each EXTI line has 4 bits (EXTIx[3:0]) to select port. 0000: Port A, 0001: Port B, etc.
    // The API does not specify which GPIO port to use for the EXTI line.
    // As per rules, it has assigned pins. For simplicity, we'll configure EXTI to listen to GPIOA by default.
    // This part requires mapping EXTI line to SYSCFG_EXTICR register and then selecting the port (e.g. GPIOA).
    volatile tlong *syscfg_exticr_reg;
    uint8_t reg_idx = external_int_channel / 4; // Which EXTICR register (0-3)
    uint8_t bit_pos = (external_int_channel % 4) * 4; // Start bit position within the register

    switch (reg_idx) {
        case 0: syscfg_exticr_reg = SYSCFG_EXTICR1_ADDR; break;
        case 1: syscfg_exticr_reg = SYSCFG_EXTICR2_ADDR; break;
        case 2: syscfg_exticr_reg = SYSCFG_EXTICR3_ADDR; break;
        case 3: syscfg_exticr_reg = SYSCFG_EXTICR4_ADDR; break;
        default: return; // Invalid channel
    }

    // Clear current port selection for the EXTI line (4 bits)
    (*syscfg_exticr_reg) &= ~(0xFUL << bit_pos);
    // Select Port A (0000b) as the source for the EXTI line.
    // This is a default choice due to lack of specific port parameter in API.
    // To select other ports: PAx=0, PBx=1, PCx=2, PDx=3, PEx=4, PHx=7 for STM32F401
    // Example: To select PB0 for EXTI0: (*SYSCFG_EXTICR1_ADDR) |= (1UL << 0);
    // As per rule, do not invent API functions, so cannot add port parameter.
    // Setting to 0 (GPIOA)
    (*syscfg_exticr_reg) |= (0UL << bit_pos);

    // Configure EXTI_RTSR (Rising Trigger Selection Register) and EXTI_FTSR (Falling Trigger Selection Register)
    // Clear relevant bits first
    (*EXTI_RTSR_ADDR) &= ~(1UL << external_int_channel);
    (*EXTI_FTSR_ADDR) &= ~(1UL << external_int_channel);

    switch (external_int_edge) {
        case EXT_INT_EDGE_RISING:
            (*EXTI_RTSR_ADDR) |= (1UL << external_int_channel);
            break;
        case EXT_INT_EDGE_FALLING:
            (*EXTI_FTSR_ADDR) |= (1UL << external_int_channel);
            break;
        case EXT_INT_EDGE_RISING_FALLING:
            (*EXTI_RTSR_ADDR) |= (1UL << external_int_channel);
            (*EXTI_FTSR_ADDR) |= (1UL << external_int_channel);
            break;
        default:
            break;
    }
}

/**
 * @brief Enables an External Interrupt channel.
 *
 * Unmasks the interrupt for the specified EXTI line.
 * @param external_int_channel The EXTI line number to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Set corresponding bit in EXTI_IMR (Interrupt Mask Register) to unmask (enable) the interrupt
    (*EXTI_IMR_ADDR) |= (1UL << external_int_channel);
}

/**
 * @brief Disables an External Interrupt channel.
 *
 * Masks the interrupt for the specified EXTI line.
 * @param external_int_channel The EXTI line number to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Clear corresponding bit in EXTI_IMR (Interrupt Mask Register) to mask (disable) the interrupt
    (*EXTI_IMR_ADDR) &= ~(1UL << external_int_channel);
}

/**
 * @brief Clears the pending flag for an External Interrupt channel.
 *
 * For STM32, pending flags in EXTI_PR are cleared by writing 1 to the corresponding bit.
 * @param external_int_channel The EXTI line number to clear the flag for.
 */
void External_INT_ClearFlag(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Write 1 to the corresponding bit in EXTI_PR (Pending Register) to clear the flag
    (*EXTI_PR_ADDR) = (1UL << external_int_channel);
}

/**
 * @brief Initializes a GPIO pin as output.
 *
 * Configures the pin mode, output type, output speed, pull-up/pull-down,
 * and sets its initial value.
 *
 * @param port The GPIO port (A, B, C, D, E, H).
 * @param pin The pin number (0-15).
 * @param value The initial value to set (0 for low, non-zero for high).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_gpio_base_addr(port);
    if (base_addr == 0) return;

    volatile tlong *moder_reg = GPIO_REG_ADDR(base_addr, GPIO_MODER_OFFSET);
    volatile tlong *otyper_reg = GPIO_REG_ADDR(base_addr, GPIO_OTYPER_OFFSET);
    volatile tlong *ospeedr_reg = GPIO_REG_ADDR(base_addr, GPIO_OSPEEDR_OFFSET);
    volatile tlong *pupdr_reg = GPIO_REG_ADDR(base_addr, GPIO_PUPDR_OFFSET);
    volatile tlong *odr_reg = GPIO_REG_ADDR(base_addr, GPIO_ODR_OFFSET);

    // Rule: Always set value before setting direction
    GPIO_Value_Set(port, pin, value); // Sets ODR bit

    // Configure MODER for Output mode (01b for each pin)
    (*moder_reg) &= ~(0x3UL << (pin * 2)); // Clear current mode bits
    (*moder_reg) |= (0x1UL << (pin * 2));  // Set to General purpose output mode

    // Rule: After setting GPIO direction, verify with while loop
    while (!((((*moder_reg) >> (pin * 2)) & 0x3UL) == 0x1UL)) {
        (*moder_reg) |= (0x1UL << (pin * 2)); // Re-attempt setting
    }

    // Configure OTYPER for Push-pull (0 for push-pull, 1 for open-drain)
    (*otyper_reg) &= ~(1UL << pin); // Set to Push-pull output type

    // Configure OSPEEDR for High speed (10b for High speed, 11b for Very high speed)
    (*ospeedr_reg) &= ~(0x3UL << (pin * 2)); // Clear current speed bits
    (*ospeedr_reg) |= (0x2UL << (pin * 2)); // Set to High speed

    // Rule: All output pins have pull-up resistors disabled
    // Set PUPDR to No Pull-up/Pull-down (00b for each pin)
    (*pupdr_reg) &= ~(0x3UL << (pin * 2)); // Clear pull-up/pull-down bits

    // For current registers: use >=20mA sink current & >=10mA source current
    // This typically means setting output drive strength bits, which are usually part of OSPEEDR
    // or specific configuration registers not explicitly available with 'current' description in JSON.
    // The OSPEEDR (output speed register) influences drive strength. Setting to 'High speed' implies
    // higher drive current, which typically meets >=20mA sink and >=10mA source.
    // Comment: Drive current requirements are implicitly handled by setting output speed to High/Very High.
}

/**
 * @brief Initializes a GPIO pin as input.
 *
 * Configures the pin mode and pull-up/pull-down resistors.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_gpio_base_addr(port);
    if (base_addr == 0) return;

    volatile tlong *moder_reg = GPIO_REG_ADDR(base_addr, GPIO_MODER_OFFSET);
    volatile tlong *pupdr_reg = GPIO_REG_ADDR(base_addr, GPIO_PUPDR_OFFSET);

    // Configure MODER for Input mode (00b for each pin)
    (*moder_reg) &= ~(0x3UL << (pin * 2)); // Clear mode bits

    // Rule: After setting GPIO direction, verify with while loop
    while (!((((*moder_reg) >> (pin * 2)) & 0x3UL) == 0x0UL)) {
        (*moder_reg) &= ~(0x3UL << (pin * 2)); // Re-attempt setting
    }

    // Rule: All input pins have pull-up resistors and wakeup feature enabled (if available)
    // Set PUPDR to Pull-up (01b for each pin)
    (*pupdr_reg) &= ~(0x3UL << (pin * 2)); // Clear pull-up/pull-down bits
    (*pupdr_reg) |= (0x1UL << (pin * 2));  // Set to Pull-up
}

/**
 * @brief Gets the current direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction of the pin.
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_gpio_base_addr(port);
    if (base_addr == 0) return (t_direction)-1; // Return invalid direction

    volatile tlong *moder_reg = GPIO_REG_ADDR(base_addr, GPIO_MODER_OFFSET);
    tbyte mode = (tbyte)(((*moder_reg) >> (pin * 2)) & 0x3UL);

    switch (mode) {
        case 0x0UL: return GPIO_DIRECTION_INPUT;
        case 0x1UL: return GPIO_DIRECTION_OUTPUT;
        case 0x2UL: return GPIO_DIRECTION_ALTERNATE_FUNCTION;
        case 0x3UL: return GPIO_DIRECTION_ANALOG;
        default: return (t_direction)-1; // Should not happen
    }
}

/**
 * @brief Sets the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 for low, non-zero for high).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_gpio_base_addr(port);
    if (base_addr == 0) return;

    volatile tlong *odr_reg = GPIO_REG_ADDR(base_addr, GPIO_ODR_OFFSET);
    volatile tlong *bsrr_reg = GPIO_REG_ADDR(base_addr, GPIO_BSRR_OFFSET);

    if (value) {
        // Set bit using BSRR (Atomic Set, Bit 0-15)
        (*bsrr_reg) = (1UL << pin);
    } else {
        // Reset bit using BSRR (Atomic Reset, Bit 16-31)
        (*bsrr_reg) = (1UL << (pin + 16));
    }

    // Rule: After setting GPIO value, verify with while loop
    volatile tlong *idr_reg = GPIO_REG_ADDR(base_addr, GPIO_IDR_OFFSET);
    if (value) {
        while (!(((*odr_reg) >> pin) & 0x1UL)) {
            (*bsrr_reg) = (1UL << pin); // Re-attempt setting
        }
    } else {
        while (((*odr_reg) >> pin) & 0x1UL) {
            (*bsrr_reg) = (1UL << (pin + 16)); // Re-attempt clearing
        }
    }
}

/**
 * @brief Gets the input value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_gpio_base_addr(port);
    if (base_addr == 0) return 0xFF; // Return error code

    volatile tlong *idr_reg = GPIO_REG_ADDR(base_addr, GPIO_IDR_OFFSET);
    return (tbyte)(((*idr_reg) >> pin) & 0x1UL);
}

/**
 * @brief Toggles the output value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_gpio_base_addr(port);
    if (base_addr == 0) return;

    volatile tlong *odr_reg = GPIO_REG_ADDR(base_addr, GPIO_ODR_OFFSET);
    volatile tlong *bsrr_reg = GPIO_REG_ADDR(base_addr, GPIO_BSRR_OFFSET);

    if (((*odr_reg) >> pin) & 0x1UL) {
        // If currently high, set to low (using BSRR reset bit)
        (*bsrr_reg) = (1UL << (pin + 16));
    } else {
        // If currently low, set to high (using BSRR set bit)
        (*bsrr_reg) = (1UL << pin);
    }

    // Rule: After setting GPIO value, verify with while loop
    // Check if the output data register bit has indeed toggled
    tbyte expected_value = !(((tbyte)(((*odr_reg) >> pin) & 0x1UL))); // Get current ODR and invert
    while (!((((tbyte)(((*odr_reg) >> pin) & 0x1UL))) == expected_value)) {
        // Re-attempt toggle based on current state
        if (((*odr_reg) >> pin) & 0x1UL) {
            (*bsrr_reg) = (1UL << (pin + 16));
        } else {
            (*bsrr_reg) = (1UL << pin);
        }
        expected_value = !expected_value; // Re-invert expected
    }
}


/**
 * @brief Initializes a PWM channel.
 *
 * Configures the PWM frequency and duty cycle.
 *
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_pwm_timer_base_addr(pwm_channel);
    if (timer_base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(timer_base_addr, TIM_CR1_OFFSET);
    volatile tlong *psc_reg = TIM_REG_ADDR(timer_base_addr, TIM_PSC_OFFSET);
    volatile tlong *arr_reg = TIM_REG_ADDR(timer_base_addr, TIM_ARR_OFFSET);
    volatile tlong *ccmr_reg_ptr;
    volatile tlong *ccr_reg_ptr;
    volatile tlong *ccer_reg = TIM_REG_ADDR(timer_base_addr, TIM_CCER_OFFSET);

    // Disable timer before configuration (CEN bit 0 in CR1)
    (*cr1_reg) &= ~(1UL << 0);

    // Determine CCR and CCMR registers based on channel
    uint8_t channel_num; // 1, 2, 3, or 4
    uint8_t ccmr_offset_bit; // 0, 8, 16, 24 for CCMR1 or CCMR2
    uint8_t ccer_bit; // 0, 4, 8, 12 for CCER

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1:
        case PWM_CHANNEL_TIM2_CH1:
        case PWM_CHANNEL_TIM3_CH1:
        case PWM_CHANNEL_TIM4_CH1:
        case PWM_CHANNEL_TIM5_CH1:
        case PWM_CHANNEL_TIM9_CH1:
        case PWM_CHANNEL_TIM10_CH1:
        case PWM_CHANNEL_TIM11_CH1:
            channel_num = 1; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR1_OFFSET); ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR1_OFFSET); ccmr_offset_bit = 0; ccer_bit = 0;
            break;
        case PWM_CHANNEL_TIM1_CH2:
        case PWM_CHANNEL_TIM2_CH2:
        case PWM_CHANNEL_TIM3_CH2:
        case PWM_CHANNEL_TIM4_CH2:
        case PWM_CHANNEL_TIM5_CH2:
        case PWM_CHANNEL_TIM9_CH2:
            channel_num = 2; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR1_OFFSET); ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR2_OFFSET); ccmr_offset_bit = 8; ccer_bit = 4;
            break;
        case PWM_CHANNEL_TIM1_CH3:
        case PWM_CHANNEL_TIM2_CH3:
        case PWM_CHANNEL_TIM3_CH3:
        case PWM_CHANNEL_TIM4_CH3:
        case PWM_CHANNEL_TIM5_CH3:
            channel_num = 3; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR2_OFFSET); ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR3_OFFSET); ccmr_offset_bit = 0; ccer_bit = 8;
            break;
        case PWM_CHANNEL_TIM1_CH4:
        case PWM_CHANNEL_TIM2_CH4:
        case PWM_CHANNEL_TIM3_CH4:
        case PWM_CHANNEL_TIM4_CH4:
        case PWM_CHANNEL_TIM5_CH4:
            channel_num = 4; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR2_OFFSET); ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR4_OFFSET); ccmr_offset_bit = 8; ccer_bit = 12;
            break;
        default: return;
    }

    // Clear available FREQUENCY Ranges for each channel as comments in PWM_Init()
    // Example ranges (highly dependent on PCLK and specific timer capabilities):
    // For TIM2, TIM3, TIM4, TIM5 (APB1 bus, max 42MHz PCLK):
    //   If PCLK = 42MHz, Prescaler = 0, ARR = 41999: Freq = 42MHz / (41999+1) = 1kHz
    //   If Prescaler = 41, ARR = 999: Freq = 42MHz / ((41+1)*(999+1)) = 1kHz
    // For TIM1, TIM9, TIM10, TIM11 (APB2 bus, max 84MHz PCLK):
    //   If PCLK = 84MHz, Prescaler = 0, ARR = 83999: Freq = 84MHz / (83999+1) = 1kHz
    //   If Prescaler = 83, ARR = 999: Freq = 84MHz / ((83+1)*(999+1)) = 1kHz

    // For a requested pwm_khz_freq, calculate PSC and ARR.
    // Assuming PCLK = 84MHz (for APB2 timers like TIM1, TIM9, 10, 11) or 42MHz (for APB1 timers like TIM2,3,4,5)
    // Let's assume a simplified PCLK_TIMER = 84000000 Hz for all timers for this exercise, or specify based on bus.
    // For STM32F401, APB1 (TIM2-5) is max 42MHz, APB2 (TIM1, 9-11) is max 84MHz.
    // To simplify and use a general frequency for calculation. Let PCLK_TIMER be a generic constant.
    const tlong PCLK_TIMER = 84000000UL; // Max PCLK for timers (APB2)

    tlong target_freq_hz = (tlong)pwm_khz_freq * 1000UL;
    tword prescaler = 0;
    tword auto_reload = 0;

    // Simple calculation: Try to get ARR close to max (65535) for better resolution
    if (target_freq_hz > 0) {
        tlong temp_arr = PCLK_TIMER / target_freq_hz;
        if (temp_arr > 65535UL) { // If ARR too large, scale with prescaler
            prescaler = (tword)((temp_arr / 65535UL) + 1);
            if (prescaler > 65535UL) prescaler = 65535UL; // Cap prescaler
            auto_reload = (tword)(PCLK_TIMER / (target_freq_hz * (tlong)(prescaler + 1))) - 1;
            if (auto_reload > 65535UL) auto_reload = 65535UL;
        } else {
            prescaler = 0;
            auto_reload = (tword)(temp_arr - 1);
        }
    }
    if (auto_reload == (tword)-1) auto_reload = 0; // Prevent underflow if target_freq_hz is too high.

    (*psc_reg) = prescaler;
    (*arr_reg) = auto_reload;

    // Configure Capture/Compare Mode Register (CCMR1 or CCMR2) for PWM Mode 1
    // OCxM[2:0] bits 6:4 or 14:12 (for CCMR1/CCMR2 for channels 1/2 or 3/4)
    // 110: PWM mode 1 - In upcounting, channel x is active when TIMx_CNT < TIMx_CCRx, and inactive otherwise.
    // OCxPE bit 3 or 11: Output compare preload enable (buffers CCRx)
    tlong ccmr_val = (*ccmr_reg_ptr);
    ccmr_val &= ~(0x7UL << ccmr_offset_bit); // Clear OCxM bits
    ccmr_val |= (0x6UL << ccmr_offset_bit);  // Set to PWM Mode 1 (110)
    ccmr_val |= (1UL << (ccmr_offset_bit + 3)); // Enable Output Compare Preload Enable (OCxPE)
    (*ccmr_reg_ptr) = ccmr_val;

    // Configure Capture/Compare Enable Register (CCER) for Output Enable (CCxE bit 0, 4, 8, 12)
    // OCxP bit 1, 5, 9, 13 (Output polarity, 0 for active high)
    // CCxE bit 0, 4, 8, 12 (Capture/Compare Output Enable)
    tlong ccer_val = (*ccer_reg);
    ccer_val &= ~(1UL << (ccer_bit + 1)); // Clear CCxP (polarity to active high)
    ccer_val |= (1UL << ccer_bit);       // Set CCxE (output enable)

    // For advanced timers like TIM1, also need BDTR's MOE bit (Main Output Enable)
    if (timer_base_addr == TIM1_BASE_ADDR) {
        volatile tlong *bdtr_reg = TIM_REG_ADDR(timer_base_addr, TIM_BDTR_OFFSET);
        (*bdtr_reg) |= (1UL << 15); // Set MOE bit (Main Output Enable)
    }

    (*ccer_reg) = ccer_val;

    // Set Duty Cycle (CCRx)
    tword compare_value = (tword)(((tlong)auto_reload * pwm_duty) / 100UL);
    (*ccr_reg_ptr) = compare_value;

    // Enable Auto-reload preload (ARPE bit 7 in CR1)
    (*cr1_reg) |= (1UL << 7);
}

/**
 * @brief Starts a PWM channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_pwm_timer_base_addr(pwm_channel);
    if (timer_base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(timer_base_addr, TIM_CR1_OFFSET);
    (*cr1_reg) |= (1UL << 0); // Set CEN bit (Counter Enable)
}

/**
 * @brief Stops a PWM channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_pwm_timer_base_addr(pwm_channel);
    if (timer_base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(timer_base_addr, TIM_CR1_OFFSET);
    (*cr1_reg) &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
}

/**
 * @brief Initializes an ICU (Input Capture Unit) channel.
 *
 * Configures the prescaler and edge detection for input capture.
 *
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler setting.
 * @param icu_edge The edge to detect (rising, falling, or both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_icu_timer_base_addr(icu_channel);
    if (timer_base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(timer_base_addr, TIM_CR1_OFFSET);
    volatile tlong *psc_reg = TIM_REG_ADDR(timer_base_addr, TIM_PSC_OFFSET);
    volatile tlong *ccmr_reg_ptr;
    volatile tlong *ccer_reg = TIM_REG_ADDR(timer_base_addr, TIM_CCER_OFFSET);

    // Disable timer before configuration
    (*cr1_reg) &= ~(1UL << 0);

    // Determine CCR and CCMR registers based on channel
    uint8_t channel_num;
    uint8_t ccmr_offset_bit;
    uint8_t ccer_bit;

    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1:
        case ICU_CHANNEL_TIM2_CH1:
        case ICU_CHANNEL_TIM3_CH1:
        case ICU_CHANNEL_TIM4_CH1:
        case ICU_CHANNEL_TIM5_CH1:
        case ICU_CHANNEL_TIM9_CH1:
        case ICU_CHANNEL_TIM10_CH1:
        case ICU_CHANNEL_TIM11_CH1:
            channel_num = 1; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR1_OFFSET); ccmr_offset_bit = 0; ccer_bit = 0;
            break;
        case ICU_CHANNEL_TIM1_CH2:
        case ICU_CHANNEL_TIM2_CH2:
        case ICU_CHANNEL_TIM3_CH2:
        case ICU_CHANNEL_TIM4_CH2:
        case ICU_CHANNEL_TIM5_CH2:
        case ICU_CHANNEL_TIM9_CH2:
            channel_num = 2; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR1_OFFSET); ccmr_offset_bit = 8; ccer_bit = 4;
            break;
        case ICU_CHANNEL_TIM1_CH3:
        case ICU_CHANNEL_TIM2_CH3:
        case ICU_CHANNEL_TIM3_CH3:
        case ICU_CHANNEL_TIM4_CH3:
        case ICU_CHANNEL_TIM5_CH3:
            channel_num = 3; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR2_OFFSET); ccmr_offset_bit = 0; ccer_bit = 8;
            break;
        case ICU_CHANNEL_TIM1_CH4:
        case ICU_CHANNEL_TIM2_CH4:
        case ICU_CHANNEL_TIM3_CH4:
        case ICU_CHANNEL_TIM4_CH4:
        case ICU_CHANNEL_TIM5_CH4:
            channel_num = 4; ccmr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCMR2_OFFSET); ccmr_offset_bit = 8; ccer_bit = 12;
            break;
        default: return;
    }

    // Configure Prescaler (PSC)
    (*psc_reg) = (tword)icu_prescaller; // Maps directly to prescaler value

    // Configure Capture/Compare Mode Register (CCMR) for Input Capture
    // CCxS[1:0] bits 1:0 or 9:8 (for CCMR1/CCMR2 for channels 1/2 or 3/4)
    // 01: CCx channel is configured as input, ICx is mapped on TIxFP1.
    tlong ccmr_val = (*ccmr_reg_ptr);
    ccmr_val &= ~(0x3UL << ccmr_offset_bit); // Clear CCxS bits
    ccmr_val |= (0x1UL << ccmr_offset_bit);  // Set to Input Capture mode (01)
    // Optional: Filter (ICxF[3:0]), Prescaler (ICxPSC[1:0]) in CCMR if needed
    (*ccmr_reg_ptr) = ccmr_val;

    // Configure Capture/Compare Enable Register (CCER) for edge detection
    // CCxP bit (0, 4, 8, 12): Input capture polarity (0=rising, 1=falling)
    // CCxNP bit (1, 5, 9, 13): Input capture polarity (0=rising, 1=falling) (for non-inverted)
    // Together CCxNP:CCxP determine edge: 00=rising, 01=falling, 11=both
    tlong ccer_val = (*ccer_reg);
    ccer_val &= ~((0x3UL << ccer_bit)); // Clear CCxP and CCxNP bits

    switch (icu_edge) {
        case ICU_EDGE_RISING:
            // 00: rising edge, 0 for CCxP, 0 for CCxNP
            break;
        case ICU_EDGE_FALLING:
            // 01: falling edge, 1 for CCxP, 0 for CCxNP
            ccer_val |= (1UL << (ccer_bit + 1)); // Set CCxP (ICx_POLARITY)
            break;
        case ICU_EDGE_BOTH:
            // 11: both edges, 1 for CCxP, 1 for CCxNP
            ccer_val |= (0x3UL << (ccer_bit + 0)); // Set both CCxP and CCxNP
            break;
        default:
            break;
    }
    ccer_val |= (1UL << ccer_bit); // Enable Capture/Compare output (CCxE)
    (*ccer_reg) = ccer_val;

    // Enable Update Interrupt / DMA request (UIE bit 0 in DIER)
    // Enable Capture/Compare Interrupt for the specific channel
    volatile tlong *dier_reg = TIM_REG_ADDR(timer_base_addr, TIM_DIER_OFFSET);
    (*dier_reg) |= (1UL << 0); // UIE (Update Interrupt Enable)
    // For specific channel interrupt enable (CC1IE bit 1, CC2IE bit 2, etc.)
    (*dier_reg) |= (1UL << channel_num); // CCxIE
}

/**
 * @brief Enables an ICU channel.
 *
 * This typically means enabling the associated timer and its capture interrupt.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_icu_timer_base_addr(icu_channel);
    if (timer_base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(timer_base_addr, TIM_CR1_OFFSET);
    (*cr1_reg) |= (1UL << 0); // Set CEN bit (Counter Enable)
}

/**
 * @brief Disables an ICU channel.
 *
 * This typically means disabling the associated timer and its capture interrupt.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_icu_timer_base_addr(icu_channel);
    if (timer_base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(timer_base_addr, TIM_CR1_OFFSET);
    (*cr1_reg) &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
}

/**
 * @brief Updates the frequency for an ICU channel.
 * Note: The API suggests "update status each period", but there's no direct "update frequency"
 * register. This likely refers to reading input capture values to derive frequency.
 * This function will be a placeholder.
 * @param icu_channel The ICU channel to update.
 */
void ICU_Updatefrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)icu_channel; // Parameter is unused based on available registers
    // No specific ICU "update frequency" register or direct mechanism is defined.
    // Frequency is derived by reading capture values. This function is a placeholder.
}

/**
 * @brief Gets the frequency measured by an ICU channel.
 * This requires reading Capture/Compare Register (CCRx) values and calculating.
 * This is a simplified placeholder, as actual frequency measurement needs logic
 * to handle multiple captures and calculate period/frequency.
 * @param icu_channel The ICU channel to get frequency from.
 * @return The measured frequency (placeholder return).
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_icu_timer_base_addr(icu_channel);
    if (timer_base_addr == 0) return 0;

    volatile tlong *ccr_reg_ptr = 0;
    // Determine CCR register based on channel
    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1:
        case ICU_CHANNEL_TIM2_CH1:
        case ICU_CHANNEL_TIM3_CH1:
        case ICU_CHANNEL_TIM4_CH1:
        case ICU_CHANNEL_TIM5_CH1:
        case ICU_CHANNEL_TIM9_CH1:
        case ICU_CHANNEL_TIM10_CH1:
        case ICU_CHANNEL_TIM11_CH1:
            ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR1_OFFSET);
            break;
        case ICU_CHANNEL_TIM1_CH2:
        case ICU_CHANNEL_TIM2_CH2:
        case ICU_CHANNEL_TIM3_CH2:
        case ICU_CHANNEL_TIM4_CH2:
        case ICU_CHANNEL_TIM5_CH2:
        case ICU_CHANNEL_TIM9_CH2:
            ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR2_OFFSET);
            break;
        case ICU_CHANNEL_TIM1_CH3:
        case ICU_CHANNEL_TIM2_CH3:
        case ICU_CHANNEL_TIM3_CH3:
        case ICU_CHANNEL_TIM4_CH3:
        case ICU_CHANNEL_TIM5_CH3:
            ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR3_OFFSET);
            break;
        case ICU_CHANNEL_TIM1_CH4:
        case ICU_CHANNEL_TIM2_CH4:
        case ICU_CHANNEL_TIM3_CH4:
        case ICU_CHANNEL_TIM4_CH4:
        case ICU_CHANNEL_TIM5_CH4:
            ccr_reg_ptr = TIM_REG_ADDR(timer_base_addr, TIM_CCR4_OFFSET);
            break;
        default: return 0;
    }

    if (ccr_reg_ptr == 0) return 0;

    // Read the last captured value. For frequency, typically two capture events
    // are needed to calculate a period (rising edge to next rising edge).
    // This is a simplified approach, returning the current capture value.
    // In a real application, an interrupt handler would capture values and calculate.
    tlong captured_value = (*ccr_reg_ptr);

    // To get frequency, you'd need:
    // PCLK_TIMER / ((Prescaler + 1) * Period_Ticks)
    // As we only have one capture value here, returning it as a placeholder.
    return captured_value; // Placeholder for frequency, this is actually period in ticks.
}

/**
 * @brief Placeholder for setting remote control keys buffer.
 * Not related to direct register manipulation.
 * @param number_of_keys Number of keys.
 * @param key_digits_length Length of key digits.
 */
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)number_of_keys; // Unused
    (void)key_digits_length; // Unused
    // This function is for a higher-level remote control decoding logic, not direct register access.
}

/**
 * @brief Placeholder for setting remote control key digits.
 * Not related to direct register manipulation.
 * @param key_num Key number.
 * @param key_array_cell Array cell.
 * @param key_cell_value Cell value.
 */
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)key_num; // Unused
    (void)key_array_cell; // Unused
    (void)key_cell_value; // Unused
    // This function is for a higher-level remote control decoding logic, not direct register access.
}

/**
 * @brief Placeholder for updating remote control signal parameters.
 * Not related to direct register manipulation.
 * @param icu_channel ICU channel.
 * @param strt_bit_us_value Start bit duration in microseconds.
 * @param one_bit_us_value One bit duration in microseconds.
 * @param zero_bit_us_value Zero bit duration in microseconds.
 * @param stop_bit_us_value Stop bit duration in microseconds.
 */
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)icu_channel; // Unused
    (void)strt_bit_us_value; // Unused
    (void)one_bit_us_value; // Unused
    (void)zero_bit_us_value; // Unused
    (void)stop_bit_us_value; // Unused
    // This function is for a higher-level remote control decoding logic, not direct register access.
}

/**
 * @brief Placeholder for getting the pressed remote control key.
 * Not related to direct register manipulation.
 * @param icu_channel ICU channel.
 * @return The detected key (placeholder return).
 */
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)icu_channel; // Unused
    // This function is for a higher-level remote control decoding logic, not direct register access.
    return 0; // Placeholder
}

/**
 * @brief Sets a callback function for ICU events.
 *
 * This function registers a callback that will be invoked when an ICU event occurs (e.g., capture interrupt).
 * The actual invocation would happen in the timer's interrupt service routine (ISR).
 * @param callback Pointer to the callback function.
 */
static void (*icu_callback_function)(void) = NULL; // Internal storage for the callback

void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    icu_callback_function = callback;
    // In a real system, the ISR for the relevant timer would check if this callback is set and call it.
    // Example in TIMx_IRQHandler: if (icu_callback_function != NULL) { icu_callback_function(); }
}

/**
 * @brief Clears the pending flag for an ICU channel.
 *
 * For STM32 Timers, flags in TIMx_SR are cleared by writing 0 to them (except for read-only ones)
 * or by reading SR and CCRx for certain flags.
 * The API doesn't specify which flags to clear.
 * @param icu_channel The ICU channel to clear flags for.
 */
void ICU_ClearFlag(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong timer_base_addr = get_icu_timer_base_addr(icu_channel);
    if (timer_base_addr == 0) return;

    volatile tlong *sr_reg = TIM_REG_ADDR(timer_base_addr, TIM_SR_OFFSET);

    // Clear the capture/compare interrupt flag for the channel by writing 0 to it.
    uint8_t channel_num;
    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1:
        case ICU_CHANNEL_TIM2_CH1:
        case ICU_CHANNEL_TIM3_CH1:
        case ICU_CHANNEL_TIM4_CH1:
        case ICU_CHANNEL_TIM5_CH1:
        case ICU_CHANNEL_TIM9_CH1:
        case ICU_CHANNEL_TIM10_CH1:
        case ICU_CHANNEL_TIM11_CH1:
            channel_num = 1; break;
        case ICU_CHANNEL_TIM1_CH2:
        case ICU_CHANNEL_TIM2_CH2:
        case ICU_CHANNEL_TIM3_CH2:
        case ICU_CHANNEL_TIM4_CH2:
        case ICU_CHANNEL_TIM5_CH2:
        case ICU_CHANNEL_TIM9_CH2:
            channel_num = 2; break;
        case ICU_CHANNEL_TIM1_CH3:
        case ICU_CHANNEL_TIM2_CH3:
        case ICU_CHANNEL_TIM3_CH3:
        case ICU_CHANNEL_TIM4_CH3:
        case ICU_CHANNEL_TIM5_CH3:
            channel_num = 3; break;
        case ICU_CHANNEL_TIM1_CH4:
        case ICU_CHANNEL_TIM2_CH4:
        case ICU_CHANNEL_TIM3_CH4:
        case ICU_CHANNEL_TIM4_CH4:
        case ICU_CHANNEL_TIM5_CH4:
            channel_num = 4; break;
        default: return;
    }
    // For CCxIF flags, they are cleared by software by writing 0 to the CCxIF bit in TIMx_SR.
    (*sr_reg) &= ~(1UL << channel_num); // Clear CCxIF
    (*sr_reg) &= ~(1UL << 0); // Clear UIF (Update Interrupt Flag)
}


/**
 * @brief Initializes a Timer channel.
 *
 * Configures the timer for basic operation.
 * @param timer_channel The Timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_timer_base_addr(timer_channel);
    if (base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(base_addr, TIM_CR1_OFFSET);
    volatile tlong *psc_reg = TIM_REG_ADDR(base_addr, TIM_PSC_OFFSET);
    volatile tlong *arr_reg = TIM_REG_ADDR(base_addr, TIM_ARR_OFFSET);
    volatile tlong *dier_reg = TIM_REG_ADDR(base_addr, TIM_DIER_OFFSET);

    // Disable timer before configuration
    (*cr1_reg) &= ~(1UL << 0); // Clear CEN (Counter Enable)

    // Set default prescaler and auto-reload for basic operation (e.g., 1ms tick)
    // Assuming a PCLK_TIMER of 84MHz. For a 1ms tick:
    // PCLK_TIMER / (PSC+1) = 1kHz (1ms period)
    // 84MHz / (PSC+1) = 1kHz => PSC+1 = 84000 => PSC = 83999
    // ARR = 1 (for one tick per period, or adjust for longer period)
    // For a simple counter, set PSC to 0 and ARR to max (65535)
    (*psc_reg) = 0; // No prescaling by default
    (*arr_reg) = 0xFFFFFFFFUL; // Max auto-reload value

    // Enable Update Interrupt if this timer will be used for time-triggered tasks (TT_Init)
    (*dier_reg) |= (1UL << 0); // UIE (Update Interrupt Enable)
}

/**
 * @brief Sets the timer period in microseconds.
 * @param timer_channel The Timer channel.
 * @param time The desired time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_timer_base_addr(timer_channel);
    if (base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(base_addr, TIM_CR1_OFFSET);
    volatile tlong *psc_reg = TIM_REG_ADDR(base_addr, TIM_PSC_OFFSET);
    volatile tlong *arr_reg = TIM_REG_ADDR(base_addr, TIM_ARR_OFFSET);

    // Disable timer
    (*cr1_reg) &= ~(1UL << 0);

    // Assuming a PCLK_TIMER of 84MHz (84 ticks/us).
    // Target ticks = time_us * (PCLK_TIMER / 1000000)
    tlong target_ticks = (tlong)time * (84000000UL / 1000000UL); // 84 ticks/us

    tword prescaler = 0;
    tword auto_reload = 0;

    if (target_ticks > 0) {
        if (target_ticks > 0xFFFFFFFFUL) { // If ARR too large for 32-bit (for TIM2/5)
            prescaler = (tword)((target_ticks / 0xFFFFFFFFUL) + 1);
            if (prescaler > 0xFFFFUL) prescaler = 0xFFFFUL;
            auto_reload = (tword)(target_ticks / (prescaler + 1)) - 1;
        } else {
            prescaler = 0;
            auto_reload = (tword)(target_ticks - 1);
        }
    }
    if (auto_reload == (tword)-1) auto_reload = 0; // Prevent underflow if target_ticks is too small.

    (*psc_reg) = prescaler;
    (*arr_reg) = auto_reload;
}

/**
 * @brief Sets the timer period in milliseconds.
 * @param timer_channel The Timer channel.
 * @param time The desired time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Convert ms to us and call TIMER_Set_us
    TIMER_Set_us(timer_channel, time * 1000UL);
}

/**
 * @brief Sets the timer period in seconds.
 * @param timer_channel The Timer channel.
 * @param time The desired time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Convert s to ms and call TIMER_Set_Time_ms
    TIMER_Set_Time_ms(timer_channel, (tword)time * 1000UL);
}

/**
 * @brief Sets the timer period in minutes.
 * @param timer_channel The Timer channel.
 * @param time The desired time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Convert min to s and call TIMER_Set_Time_sec
    TIMER_Set_Time_sec(timer_channel, time * 60UL);
}

/**
 * @brief Sets the timer period in hours.
 * @param timer_channel The Timer channel.
 * @param time The desired time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Convert hours to min and call TIMER_Set_Time_min
    TIMER_Set_Time_min(timer_channel, time * 60UL);
}

/**
 * @brief Enables a Timer channel.
 * @param timer_channel The Timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_timer_base_addr(timer_channel);
    if (base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(base_addr, TIM_CR1_OFFSET);
    (*cr1_reg) |= (1UL << 0); // Set CEN bit (Counter Enable)
}

/**
 * @brief Disables a Timer channel.
 * @param timer_channel The Timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_timer_base_addr(timer_channel);
    if (base_addr == 0) return;

    volatile tlong *cr1_reg = TIM_REG_ADDR(base_addr, TIM_CR1_OFFSET);
    (*cr1_reg) &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
}

/**
 * @brief Clears the pending flag for a Timer channel.
 *
 * For STM32 Timers, flags in TIMx_SR (e.g., Update Interrupt Flag UIF)
 * are cleared by writing 0 to them (except for read-only ones).
 * @param timer_channel The Timer channel to clear the flag for.
 */
void TIMER_ClearFlag(t_timer_channel timer_channel) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = get_timer_base_addr(timer_channel);
    if (base_addr == 0) return;

    volatile tlong *sr_reg = TIM_REG_ADDR(base_addr, TIM_SR_OFFSET);
    (*sr_reg) &= ~(1UL << 0); // Clear UIF (Update Interrupt Flag)
}


/**
 * @brief Initializes the ADC.
 *
 * Configures the ADC channel and mode.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The ADC conversion mode (single or continuous).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json

    // Enable ADC clock (not explicitly present in RCC_AHBxENR or similar in JSON, so omit specific bit set)
    // Example: (*RCC_APB2ENR_ADDR) |= (1UL << 8); // Enable ADC1 clock (bit 8 for ADC1)

    // Disable ADC before configuration (ADON bit 0 in CR2)
    (*ADC_CR2_ADDR) &= ~(1UL << 0);

    // Configure Continuous Conversion Mode (CONT bit 1 in CR2)
    if (adc_mode == ADC_MODE_CONTINUOUS) {
        (*ADC_CR2_ADDR) |= (1UL << 1);
    } else {
        (*ADC_CR2_ADDR) &= ~(1UL << 1); // Single conversion mode
    }

    // Configure Regular Channel Sequence Length (L[3:0] bits 23:20 in SQR1)
    // For single channel, set length to 0000 (1 conversion)
    (*ADC_SQR1_ADDR) &= ~(0xFUL << 20); // Clear L bits

    // Configure Regular Channel Sequence (SQx[4:0] in SQR1, SQR2, SQR3)
    // For single channel, set the channel number to SQ1 (bits 4:0 in SQR3)
    // Clear SQ1 bits first
    (*ADC_SQR3_ADDR) &= ~(0x1FUL << 0); // Clear SQ1[4:0]
    (*ADC_SQR3_ADDR) |= ((tlong)adc_channel << 0); // Set SQ1 to selected channel

    // Configure Sample Time (SMPx[2:0]) in SMPR1 or SMPR2
    // Each channel has dedicated sample time bits. Choose a reasonable sample time (e.g., 3 cycles).
    // SMPR2 for channels 0-9 (bits 2:0 for CH0, 5:3 for CH1, etc.)
    // SMPR1 for channels 10-18 (bits 2:0 for CH10, etc.)
    tlong sample_time_cycles = 0x0UL; // 3 cycles (000 in SMPx)
    if (adc_channel < ADC_CHANNEL_10) { // Channels 0-9 use SMPR2
        (*ADC_SMPR2_ADDR) &= ~(0x7UL << (adc_channel * 3)); // Clear SMPx bits
        (*ADC_SMPR2_ADDR) |= (sample_time_cycles << (adc_channel * 3)); // Set sample time
    } else { // Channels 10-15 use SMPR1
        // (adc_channel - 10) for offset in SMPR1
        (*ADC_SMPR1_ADDR) &= ~(0x7UL << ((adc_channel - ADC_CHANNEL_10) * 3)); // Clear SMPx bits
        (*ADC_SMPR1_ADDR) |= (sample_time_cycles << ((adc_channel - ADC_CHANNEL_10) * 3)); // Set sample time
    }
}

/**
 * @brief Enables the ADC.
 */
void ADC_Enable(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (*ADC_CR2_ADDR) |= (1UL << 0); // Set ADON bit (ADC On)
}

/**
 * @brief Disables the ADC.
 */
void ADC_Disable(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (*ADC_CR2_ADDR) &= ~(1UL << 0); // Clear ADON bit (ADC Off)
}

/**
 * @brief Performs an ADC update (starts conversion for single mode).
 * For continuous mode, this might not be explicitly needed after init.
 * @note This function is assumed to initiate a regular conversion for a configured channel.
 */
void ADC_Update(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Start regular conversion (SWSTART bit 30 in CR2)
    // Note: SWSTART is a software trigger, writing 1 to it starts conversion.
    (*ADC_CR2_ADDR) |= (1UL << 30);
}

/**
 * @brief Gets the ADC conversion result.
 * @return The 12-bit ADC conversion value from the regular data register.
 */
tword ADC_Get(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Wait for EOC (End Of Conversion) flag in SR (bit 1)
    while (!(((*ADC_SR_ADDR) >> 1) & 0x1UL));
    // Clear EOC flag by reading SR and then DR
    (*ADC_SR_ADDR) &= ~(1UL << 1); // Clear EOC (can be cleared by reading DR as well)

    return (tword)(*ADC_DR_ADDR); // Read the regular data register
}

/**
 * @brief Clears ADC flags.
 *
 * For STM32 ADC, flags in ADC_SR (e.g., EOC) are cleared by reading them.
 * @note This function clears the EOC flag by reading the DR and by writing 0 to SR.
 */
void ADC_ClearFlag(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Clear EOC (End Of Conversion) flag by writing 0 to bit 1 in SR.
    // Some flags are cleared by software writing 0 to the flag or by reading DR.
    (*ADC_SR_ADDR) &= ~(1UL << 1); // Clear EOC flag
    // Comment: Overrun (OVR) flag is cleared by reading SR followed by reading DR.
    // This function provides a general clear for common flags.
}

/**
 * @brief Initializes the Internal EEPROM.
 *
 * Note: The provided REGISTER_JSON does not contain any registers
 * explicitly described as "Internal EEPROM" or related to Data EEPROM.
 * STM32F401RC does not have a dedicated internal EEPROM, typically uses
 * Flash memory emulation. Therefore, this function is a placeholder.
 */
void Internal_EEPROM_Init(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // The provided REGISTER_JSON does not contain specific Internal EEPROM registers.
    // STM32F401RC uses Flash memory for EEPROM emulation.
    // This function is a placeholder.
}

/**
 * @brief Sets a byte in the Internal EEPROM.
 *
 * Note: Placeholder due to lack of EEPROM registers.
 * @param address The address in EEPROM.
 * @param data The byte data to write.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)address; // Unused
    (void)data;    // Unused
    // The provided REGISTER_JSON does not contain specific Internal EEPROM registers.
    // This function is a placeholder.
}

/**
 * @brief Gets a byte from the Internal EEPROM.
 *
 * Note: Placeholder due to lack of EEPROM registers.
 * @param address The address in EEPROM.
 * @return The byte data read.
 */
tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    (void)address; // Unused
    // The provided REGISTER_JSON does not contain specific Internal EEPROM registers.
    // This function is a placeholder.
    return 0xFF; // Placeholder
}


// Internal structure for TT tasks
typedef struct {
    void (*task_func)(void);
    tword period;
    tword delay;
    tword elapsed_time;
    bool  is_running;
} TT_Task_t;

#define MAX_TT_TASKS 10 // Max tasks for the scheduler
static TT_Task_t tt_tasks[MAX_TT_TASKS];
static tbyte tt_num_tasks = 0;
static volatile tword tt_tick_counter = 0;

/**
 * @brief Initializes the Time-Triggered (TT) scheduler.
 *
 * This function sets up a hardware timer to generate periodic ticks for the scheduler.
 * @param tick_time_ms The desired tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    // Use a general purpose timer, e.g., TIM2 for the TT scheduler.
    // Enable TIM2 clock first (RCC_APB1ENR bit 0 for TIM2)
    // (*RCC_APB1ENR_ADDR) |= (1UL << 0); // Not in JSON description for RCC_APB1ENR

    // Initialize TIM2 for the desired tick_time_ms
    // Assuming PCLK1 = 42MHz for TIM2
    // For 1ms tick: (42000000 / (PSC+1)) = 1000 => PSC+1 = 42000 => PSC = 41999, ARR = 0 (count to 0 then update)
    // Or if ARR is the period: (42000000 / (PSC+1)) * (ARR+1) = time in ticks (1ms = 1000 ticks)
    // Use PSC = (PCLK_TIMER / 1000) - 1, ARR = 999. So (84MHz / (83999+1)) * (999+1) = 1kHz * 1000 = 1MHz -> 1ms.
    // This means PCLK_TIMER / (PSC+1) = 1kHz. So PSC = (PCLK_TIMER/1000)-1.
    const tlong PCLK1_TIMER = 42000000UL; // Max PCLK for APB1 timers

    tlong base_addr = TIM2_BASE_ADDR; // Using TIM2 for TT
    volatile tlong *cr1_reg = TIM_REG_ADDR(base_addr, TIM_CR1_OFFSET);
    volatile tlong *psc_reg = TIM_REG_ADDR(base_addr, TIM_PSC_OFFSET);
    volatile tlong *arr_reg = TIM_REG_ADDR(base_addr, TIM_ARR_OFFSET);
    volatile tlong *dier_reg = TIM_REG_ADDR(base_addr, TIM_DIER_OFFSET);

    (*cr1_reg) &= ~(1UL << 0); // Disable TIM2 counter

    tword prescaler = (tword)((PCLK1_TIMER / 1000UL) - 1); // For 1kHz clock
    tword arr_value = tick_time_ms - 1; // Count up to tick_time_ms-1 for tick_time_ms period

    (*psc_reg) = prescaler;
    (*arr_reg) = arr_value;

    (*dier_reg) |= (1UL << 0); // Enable Update Interrupt (UIE)
    // The TIM2_IRQHandler would then increment tt_tick_counter and call TT_ISR.

    // Initialize tasks array
    for (tbyte i = 0; i < MAX_TT_TASKS; i++) {
        tt_tasks[i].task_func = NULL;
        tt_tasks[i].is_running = false;
    }
    tt_num_tasks = 0;
    tt_tick_counter = 0;
}

/**
 * @brief Starts the Time-Triggered (TT) scheduler.
 * @note This function only starts the underlying timer, the TT_Dispatch_task
 * must be called periodically (e.g., from main loop or another timer).
 */
void TT_Start(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tlong base_addr = TIM2_BASE_ADDR; // Using TIM2 for TT
    volatile tlong *cr1_reg = TIM_REG_ADDR(base_addr, TIM_CR1_OFFSET);
    (*cr1_reg) |= (1UL << 0); // Enable TIM2 counter (CEN bit)
}

/**
 * @brief Dispatches tasks in the Time-Triggered (TT) scheduler.
 * This function should be called periodically by the main loop or an ISR.
 */
void TT_Dispatch_task(void) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    tword current_tick_count = tt_tick_counter; // Read atomically or disable interrupts if complex logic

    for (tbyte i = 0; i < tt_num_tasks; i++) {
        if (tt_tasks[i].is_running && tt_tasks[i].task_func != NULL) {
            if (current_tick_count >= tt_tasks[i].elapsed_time + tt_tasks[i].delay) {
                if (tt_tasks[i].task_func != NULL) {
                    tt_tasks[i].task_func();
                }
                tt_tasks[i].elapsed_time = current_tick_count; // Update elapsed time
                // For periodic tasks, reset delay for next period (if not one-shot)
                // This simple model re-schedules based on current_tick_count.
            }
        }
    }
}

/**
 * @brief The Interrupt Service Routine (ISR) for the Time-Triggered scheduler.
 * This function should be called from the underlying timer's ISR (e.g., TIM2_IRQHandler).
 */
void TT_ISR(void) {
    // This ISR will be called by the timer's update interrupt (UIF)
    // Clear the update interrupt flag (UIF) in TIM2_SR (bit 0)
    (*TIM_REG_ADDR(TIM2_BASE_ADDR, TIM_SR_OFFSET)) &= ~(1UL << 0);
    tt_tick_counter++; // Increment the global tick counter
}

/**
 * @brief Adds a task to the Time-Triggered (TT) scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks.
 * @param delay The initial delay before the first execution in ticks.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    if (tt_num_tasks < MAX_TT_TASKS) {
        tt_tasks[tt_num_tasks].task_func = task;
        tt_tasks[tt_num_tasks].period = period;
        tt_tasks[tt_num_tasks].delay = delay;
        tt_tasks[tt_num_tasks].elapsed_time = tt_tick_counter; // Initialize elapsed time
        tt_tasks[tt_num_tasks].is_running = true;
        return tt_num_tasks++;
    }
    return 0xFF; // Failed to add task
}

/**
 * @brief Deletes a task from the Time-Triggered (TT) scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // Call WDT_Reset as per Rules.json
    if (task_index < tt_num_tasks) {
        tt_tasks[task_index].is_running = false;
        tt_tasks[task_index].task_func = NULL; // Clear function pointer
        // Shift remaining tasks to fill the gap (optional, for dense packing)
        for (tbyte i = task_index; i < tt_num_tasks - 1; i++) {
            tt_tasks[i] = tt_tasks[i + 1];
        }
        tt_num_tasks--;
    }
}

// Example of a Timer ISR that would call TT_ISR
// This function name should match your startup file vector table entry for TIM2_IRQHandler
// void TIM2_IRQHandler(void) {
//     if (((*TIM_REG_ADDR(TIM2_BASE_ADDR, TIM_SR_OFFSET)) & (1UL << 0)) != 0) { // Check UIF flag
//         TT_ISR(); // Call the Time-Triggered OS ISR
//     }
// }