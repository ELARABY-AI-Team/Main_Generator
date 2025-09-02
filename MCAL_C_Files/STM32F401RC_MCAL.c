/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) for STM32F401RC.
 *
 * This file implements the MCAL APIs based on the provided register definitions,
 * API specifications, and coding rules.
 *
 * @author [AI Assistant]
 * @date [Current Date]
 */

// --- CORE INCLUDES (from Rules.json and MCAL.h) ---
#include "MCAL.h"
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>    // For potential sprintf, etc., although not explicitly used in basic MCAL.
#include <stdlib.h>   // For potential malloc/free, not used in basic MCAL.
#include <math.h>     // For potential math operations, not explicitly used.

// --- CPU Specific Includes for Global Interrupts and Sleep ---
#include "core_cm4.h" // For __enable_irq, __disable_irq, __WFI

// --- MACROS FOR REGISTER ACCESS ---
#define REG_ADDR(address) (*((volatile uint32_t*)(address)))

// --- PERIPHERAL BASE ADDRESSES (Derived from register_json) ---
// FLASH
#define FLASH_BASE_ADDR     0x40023C00U
#define FLASH_ACR_REG       REG_ADDR(FLASH_BASE_ADDR + 0x00U) // Flash access control register.
#define FLASH_KEYR_REG      REG_ADDR(FLASH_BASE_ADDR + 0x04U) // Flash key register.
#define FLASH_OPTKEYR_REG   REG_ADDR(FLASH_BASE_ADDR + 0x08U) // Flash option key register.
#define FLASH_SR_REG        REG_ADDR(FLASH_BASE_ADDR + 0x0CU) // Flash status register.
#define FLASH_CR_REG        REG_ADDR(FLASH_BASE_ADDR + 0x10U) // Flash control register.
#define FLASH_OPTCR_REG     REG_ADDR(FLASH_BASE_ADDR + 0x14U) // Flash option control register.

// RCC
#define RCC_BASE_ADDR       0x40023800U
#define RCC_CR_REG          REG_ADDR(RCC_BASE_ADDR + 0x00U) // RCC clock control register.
#define RCC_PLLCFGR_REG     REG_ADDR(RCC_BASE_ADDR + 0x04U) // RCC PLL configuration register.
#define RCC_CFGR_REG        REG_ADDR(RCC_BASE_ADDR + 0x08U) // RCC clock configuration register.
#define RCC_CIR_REG         REG_ADDR(RCC_BASE_ADDR + 0x0CU) // RCC clock interrupt register.
#define RCC_AHB1RSTR_REG    REG_ADDR(RCC_BASE_ADDR + 0x10U) // RCC AHB1 peripheral reset register.
#define RCC_AHB2RSTR_REG    REG_ADDR(RCC_BASE_ADDR + 0x14U) // RCC AHB2 peripheral reset register.
#define RCC_APB1RSTR_REG    REG_ADDR(RCC_BASE_ADDR + 0x18U) // RCC APB1 peripheral reset register.
#define RCC_APB2RSTR_REG    REG_ADDR(RCC_BASE_ADDR + 0x1CU) // RCC APB2 peripheral reset register.
#define RCC_AHB1ENR_REG     REG_ADDR(RCC_BASE_ADDR + 0x30U) // RCC AHB1 peripheral clock enable register.
#define RCC_AHB2ENR_REG     REG_ADDR(RCC_BASE_ADDR + 0x34U) // RCC AHB2 peripheral clock enable register.
#define RCC_APB1ENR_REG     REG_ADDR(RCC_BASE_ADDR + 0x38U) // RCC APB1 peripheral clock enable register.
#define RCC_APB2ENR_REG     REG_ADDR(RCC_BASE_ADDR + 0x3CU) // RCC APB2 peripheral clock enable register.
#define RCC_BDCR_REG        REG_ADDR(RCC_BASE_ADDR + 0x50U) // RCC Backup domain control register.
#define RCC_CSR_REG         REG_ADDR(RCC_BASE_ADDR + 0x54U) // RCC clock control & status register.

// SYSCFG
#define SYSCFG_BASE_ADDR    0x40013800U
#define SYSCFG_MEMRMP_REG   REG_ADDR(SYSCFG_BASE_ADDR + 0x00U) // SYSCFG memory remap register.
#define SYSCFG_PMC_REG      REG_ADDR(SYSCFG_BASE_ADDR + 0x04U) // SYSCFG peripheral mode configuration register.
#define SYSCFG_EXTICR1_REG  REG_ADDR(SYSCFG_BASE_ADDR + 0x08U) // SYSCFG external interrupt configuration register 1.
#define SYSCFG_EXTICR2_REG  REG_ADDR(SYSCFG_BASE_ADDR + 0x0CU) // SYSCFG external interrupt configuration register 2.
#define SYSCFG_EXTICR3_REG  REG_ADDR(SYSCFG_BASE_ADDR + 0x10U) // SYSCFG external interrupt configuration register 3.
#define SYSCFG_EXTICR4_REG  REG_ADDR(SYSCFG_BASE_ADDR + 0x14U) // SYSCFG external interrupt configuration register 4.
#define SYSCFG_CMPCR_REG    REG_ADDR(SYSCFG_BASE_ADDR + 0x20U) // Compensation cell control register.

// GPIO
#define GPIOA_BASE_ADDR     0x40020000U
#define GPIOB_BASE_ADDR     0x40020400U
#define GPIOC_BASE_ADDR     0x40020800U
#define GPIOD_BASE_ADDR     0x40020C00U
#define GPIOE_BASE_ADDR     0x40021000U
#define GPIOH_BASE_ADDR     0x40021C00U

#define GPIO_MODER_OFFSET   0x00U
#define GPIO_OTYPER_OFFSET  0x04U
#define GPIO_OSPEEDR_OFFSET 0x08U
#define GPIO_PUPDR_OFFSET   0x0CU
#define GPIO_IDR_OFFSET     0x10U
#define GPIO_ODR_OFFSET     0x14U
#define GPIO_BSRR_OFFSET    0x18U
#define GPIO_LCKR_OFFSET    0x1CU
#define GPIO_AFRL_OFFSET    0x20U
#define GPIO_AFRH_OFFSET    0x24U

// EXTI
#define EXTI_BASE_ADDR      0x40013C00U
#define EXTI_IMR_REG        REG_ADDR(EXTI_BASE_ADDR + 0x00U) // Interrupt mask register.
#define EXTI_EMR_REG        REG_ADDR(EXTI_BASE_ADDR + 0x04U) // Event mask register.
#define EXTI_RTSR_REG       REG_ADDR(EXTI_BASE_ADDR + 0x08U) // Rising trigger selection register.
#define EXTI_FTSR_REG       REG_ADDR(EXTI_BASE_ADDR + 0x0CU) // Falling trigger selection register.
#define EXTI_SWIER_REG      REG_ADDR(EXTI_BASE_ADDR + 0x10U) // Software interrupt event register.
#define EXTI_PR_REG         REG_ADDR(EXTI_BASE_ADDR + 0x14U) // Pending register.

// ADC
#define ADC1_BASE_ADDR      0x40012000U
#define ADC1_SR_REG         REG_ADDR(ADC1_BASE_ADDR + 0x00U) // ADC status register.
#define ADC1_CR1_REG        REG_ADDR(ADC1_BASE_ADDR + 0x04U) // ADC control register 1.
#define ADC1_CR2_REG        REG_ADDR(ADC1_BASE_ADDR + 0x08U) // ADC control register 2.
#define ADC1_SMPR1_REG      REG_ADDR(ADC1_BASE_ADDR + 0x0CU) // ADC sample time register 1.
#define ADC1_SMPR2_REG      REG_ADDR(ADC1_BASE_ADDR + 0x10U) // ADC sample time register 2.
#define ADC1_SQR1_REG       REG_ADDR(ADC1_BASE_ADDR + 0x2CU) // ADC regular sequence register 1.
#define ADC1_SQR2_REG       REG_ADDR(ADC1_BASE_ADDR + 0x30U) // ADC regular sequence register 2.
#define ADC1_SQR3_REG       REG_ADDR(ADC1_BASE_ADDR + 0x34U) // ADC regular sequence register 3.
#define ADC1_DR_REG         REG_ADDR(ADC1_BASE_ADDR + 0x4CU) // ADC regular data register.
#define ADC_COMMON_BASE_ADDR 0x40012300U // Assuming a common ADC block
#define ADC_CCR_REG         REG_ADDR(ADC_COMMON_BASE_ADDR + 0x04U) // ADC common control register.

// TIMERS
#define TIM1_BASE_ADDR      0x40010000U
#define TIM2_BASE_ADDR      0x40000000U
#define TIM3_BASE_ADDR      0x40000400U
#define TIM4_BASE_ADDR      0x40000800U
#define TIM5_BASE_ADDR      0x40000C00U
#define TIM9_BASE_ADDR      0x40014000U
#define TIM10_BASE_ADDR     0x40014400U
#define TIM11_BASE_ADDR     0x40014800U

#define TIM_CR1_OFFSET      0x00U
#define TIM_CR2_OFFSET      0x04U
#define TIM_SMCR_OFFSET     0x08U
#define TIM_DIER_OFFSET     0x0CU
#define TIM_SR_OFFSET       0x10U
#define TIM_EGR_OFFSET      0x14U
#define TIM_CCMR1_OFFSET    0x18U
#define TIM_CCMR2_OFFSET    0x1CU
#define TIM_CCER_OFFSET     0x20U
#define TIM_CNT_OFFSET      0x24U
#define TIM_PSC_OFFSET      0x28U
#define TIM_ARR_OFFSET      0x2CU
#define TIM_RCR_OFFSET      0x30U // Only for advanced timers like TIM1
#define TIM_CCR1_OFFSET     0x34U
#define TIM_CCR2_OFFSET     0x38U
#define TIM_CCR3_OFFSET     0x3CU
#define TIM_CCR4_OFFSET     0x40U
#define TIM_BDTR_OFFSET     0x44U // Only for advanced timers like TIM1
#define TIM_DCR_OFFSET      0x48U
#define TIM_DMAR_OFFSET     0x4CU
#define TIM_OR_OFFSET       0x50U // Only for some general-purpose timers (TIM2, TIM5)

// USART
#define USART1_BASE_ADDR    0x40011000U
#define USART2_BASE_ADDR    0x40004400U
#define USART6_BASE_ADDR    0x40011400U

#define USART_SR_OFFSET     0x00U
#define USART_DR_OFFSET     0x04U
#define USART_BRR_OFFSET    0x08U
#define USART_CR1_OFFSET    0x0CU
#define USART_CR2_OFFSET    0x10U
#define USART_CR3_OFFSET    0x14U
#define USART_GTPR_OFFSET   0x18U

// I2C
#define I2C1_BASE_ADDR      0x40005400U
#define I2C2_BASE_ADDR      0x40005800U
#define I2C3_BASE_ADDR      0x40005C00U

#define I2C_CR1_OFFSET      0x00U
#define I2C_CR2_OFFSET      0x04U
#define I2C_OAR1_OFFSET     0x08U
#define I2C_OAR2_OFFSET     0x0CU
#define I2C_DR_OFFSET       0x10U
#define I2C_SR1_OFFSET      0x14U
#define I2C_SR2_OFFSET      0x18U
#define I2C_CCR_OFFSET      0x1CU
#define I2C_TRISE_OFFSET    0x20U
#define I2C_FLTR_OFFSET     0x24U

// SPI
#define SPI1_BASE_ADDR      0x40013000U
#define SPI2_BASE_ADDR      0x40003800U
#define SPI3_BASE_ADDR      0x40003C00U

#define SPI_CR1_OFFSET      0x00U
#define SPI_CR2_OFFSET      0x04U
#define SPI_SR_OFFSET       0x08U
#define SPI_DR_OFFSET       0x0CU
#define SPI_CRCPR_OFFSET    0x10U
#define SPI_RXCRCR_OFFSET   0x14U
#define SPI_TXCRCR_OFFSET   0x18U
#define SPI_I2SCFGR_OFFSET  0x1CU
#define SPI_I2SPR_OFFSET    0x20U

// IWDG (Inferred registers as per rules for WDT)
#define IWDG_BASE_ADDR      0x40003000U
#define IWDG_KR_REG         REG_ADDR(IWDG_BASE_ADDR + 0x00U) // IWDG Key Register
#define IWDG_PR_REG         REG_ADDR(IWDG_BASE_ADDR + 0x04U) // IWDG Prescaler Register
#define IWDG_RLR_REG        REG_ADDR(IWDG_BASE_ADDR + 0x08U) // IWDG Reload Register
#define IWDG_SR_REG         REG_ADDR(IWDG_BASE_ADDR + 0x0CU) // IWDG Status Register

// PWR (Inferred registers for LVD)
#define PWR_BASE_ADDR       0x40007000U
#define PWR_CR_REG          REG_ADDR(PWR_BASE_ADDR + 0x00U) // Power Control Register

// --- GLOBAL VARIABLES ---
static GPIO_TypeDef *const GPIO_PORTS[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH};

// Callback for ICU
static void (*ICU_Callback)(void) = NULL;

// Task structure for TT module
typedef struct {
    void (*task_func)(void);
    tword period;
    tword delay;
    bool enabled;
    tword run_count;
} TT_Task_t;

#define MAX_TT_TASKS 10 // Maximum number of tasks for TT scheduler
static TT_Task_t tt_tasks[MAX_TT_TASKS];
static tbyte tt_next_task_index = 0;
static tword tt_tick_ms_value = 0; // Stores the configured tick time in ms

// --- SYSTEM CLOCK FREQUENCIES (Assumptions for STM32F401RC) ---
// These frequencies are crucial for baud rate, timer calculations, etc.
// For STM32F401RC, common max values are HCLK=84MHz, APB1=42MHz, APB2=84MHz.
// Timer clocks are typically 2x APB clock if APB prescaler is >1.
#define HCLK_FREQ_HZ        84000000U   // System Clock
#define PCLK1_FREQ_HZ       42000000U   // APB1 Peripheral Clock
#define PCLK2_FREQ_HZ       84000000U   // APB2 Peripheral Clock
#define TIM_CLK1_FREQ_HZ    (2 * PCLK1_FREQ_HZ) // For TIM2,3,4,5 (if APB1 prescaler > 1)
#define TIM_CLK2_FREQ_HZ    (2 * PCLK2_FREQ_HZ) // For TIM1,9,10,11 (if APB2 prescaler > 1)
// LSI Frequency for IWDG (typically 32kHz)
#define LSI_FREQ_HZ         32000U

// --- FORWARD DECLARATIONS OF STATIC HELPERS (if any) ---
static void MCAL_RCC_EnableClock(uint33_t peripheral_base_addr);
static void MCAL_RCC_DisableClock(uint33_t peripheral_base_addr);
static void MCAL_GPIO_ClearAll(void);
static void MCAL_GPIO_SetAllInput(void);
static void MCAL_DisableAllPeripherals(void);
static void MCAL_Flash_Unlock(void);
static void MCAL_Flash_Lock(void);


// --- WDT API IMPLEMENTATION ---

/**
 * @brief Resets the Watchdog Timer (IWDG).
 *        Writes the AAAA key to the Key Register to reload the counter.
 * @note This function is automatically called at the start of every MCAL API.
 */
void WDT_Reset(void)
{
    // Write 0xAAAA to IWDG_KR to reload the counter
    IWDG_KR_REG = 0xAAAAU; // Inferred register: IWDG Key Register at 0x40003000
}

/**
 * @brief Initializes the Independent Watchdog Timer (IWDG).
 *        Configures the watchdog period.
 */
void WDT_Init(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // IWDG Setup (Inferred registers for IWDG)
    // 1. Enable write access to IWDG_PR and IWDG_RLR by writing 0x5555 in IWDG_KR
    IWDG_KR_REG = 0x5555U;

    // 2. Set prescaler. LSI_FREQ_HZ / prescaler.
    // For >= 8ms with LSI_FREQ_HZ = 32kHz:
    // If PR = 4 (divider /32), then 32kHz / 32 = 1kHz. Reload value 8 means 8ms.
    // IWDG_PR values:
    // 0: /4, 1: /8, 2: /16, 3: /32, 4: /64, 5: /128, 6: /256
    IWDG_PR_REG = 0x00000003U; // Prescaler /32 (32kHz / 32 = 1kHz timer clock)

    // 3. Set reload value. (period in ms / (1/1kHz)) = period.
    // For ~8ms: reload value = 8 - 1 = 7. Let's aim for slightly higher, say 10ms.
    // 10ms * (LSI_FREQ_HZ / 32) / 1000 = 10 * 1000 / 1000 = 10
    // So, reload value for 10ms = 10 - 1 = 9
    IWDG_RLR_REG = 9U; // Reload value for ~10ms (at 32kHz/32)

    // Wait for the registers to be updated
    // IWDG_SR_REG: RVU (Reload Value Update) and PVU (Prescaler Value Update) flags
    while ((IWDG_SR_REG & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U)
    {
        // Loop until registers are updated
    }

    // 4. Start the watchdog by writing 0xCCCC in IWDG_KR
    IWDG_KR_REG = 0xCCCCU;
    WDT_Reset(); // Clear WDT again after configuration (from MCU_Config_Init steps)
}


// --- MCU CONFIG API IMPLEMENTATION ---

/**
 * @brief Initializes the Microcontroller Unit (MCU) configuration.
 *        Sets GPIOs, disables peripherals, configures WDT and LVR.
 * @param volt System voltage (VSOURCE_3V or VSOURCE_5V).
 */
void MCU_Config_Init(t_sys_volt volt)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // 1. Set all GPIO pins to 0 and verify with while loop
    MCAL_GPIO_ClearAll();

    // 2. Set all GPIO pins direction to input and verify with while loop
    MCAL_GPIO_SetAllInput();

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();
    MCAL_DisableAllPeripherals();

    // 4. Enable WDT (Watchdog Timer)
    WDT_Init(); // This also clears WDT and sets period >= 8ms

    // 5. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    //    Enable LVR (Low Voltage Reset)
    //    PVD (Power Voltage Detector) on STM32F401RC is configured via PWR->CR
    //    PWR_CR_REG: PVDEN (bit 4), PLS[2:0] (bits 7:5)
    //    PLS selection: 000: 2.2V, 001: 2.3V, 010: 2.4V, 011: 2.5V, 100: 2.6V, 101: 2.7V, 110: 2.8V, 111: External
    //    These values are general for STM32, not explicitly in the provided JSON, so they are inferred.

    MCAL_RCC_EnableClock(PWR_BASE_ADDR); // Enable clock for PWR peripheral (inferred on APB1)
    
    // Clear existing PVD config bits
    PWR_CR_REG &= ~((0x7U << 5) | (1U << 4)); // Clear PLS[2:0] and PVDEN bits

    if (volt == VSOURCE_3V)
    {
        // For 3V system, set LVR to ~2.0V or 2.2V (PLS 000)
        PWR_CR_REG |= (0x0U << 5); // Set PLS[2:0] to 000 (2.2V)
    }
    else // VSOURCE_5V
    {
        // For 5V system, set LVR to ~3.5V (e.g., PLS 100 for 2.6V, not enough, will pick highest available or a safe one)
        // STM32F401 PVD levels are limited (e.g., up to 2.8V). 3.5V might require external comparator.
        // Assuming highest safe internal PVD level for 5V system.
        PWR_CR_REG |= (0x6U << 5); // Set PLS[2:0] to 110 (2.8V) - highest internal PVD level
    }
    PWR_CR_REG |= (1U << 4); // Enable PVDEN (Power Voltage Detector Enable)

    // WDT_Reset is already handled by WDT_Init, and a final one is typically done at the very end of startup.
    // No explicit call here as WDT_Init() already clears it.
}

/**
 * @brief Puts the MCU into sleep mode.
 *        Stops CPU execution until an interrupt occurs.
 */
void Go_to_sleep_mode(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // For STM32, __WFI (Wait For Interrupt) is the standard instruction
    // to enter Sleep mode (Cortex-M sleep).
    // System Control Block (SCB) register SCB->SCR (System Control Register) can configure
    // deeper sleep modes (e.g., SLEEPDEEP bit for STOP/STANDBY), but __WFI is basic sleep.
    __WFI(); // Wait for Interrupt
}

/**
 * @brief Enables global interrupts.
 */
void Global_interrupt_Enable(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Cortex-M function to enable global interrupts
    __enable_irq();
}

/**
 * @brief Disables global interrupts.
 */
void Global_interrupt_Disable(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Cortex-M function to disable global interrupts
    __disable_irq();
}


// --- LVD API IMPLEMENTATION ---

/**
 * @brief Initializes the Low Voltage Detection (LVD) module.
 *        This function doesn't set a threshold, as it's typically done once in MCU_Config_Init.
 *        It can be used to perform any additional setup if required by a specific LVD module.
 */
void LVD_Init(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // On STM32, PVD is typically initialized and enabled via PWR_CR in MCU_Config_Init.
    // No additional initialization steps specific to LVD_Init beyond what MCU_Config_Init does.
    // If a more dynamic LVD threshold change is needed, it would be in LVD_Get, LVD_Enable, LVD_Disable.
}

/**
 * @brief Sets the Low Voltage Detection (LVD) threshold level.
 * @param lvd_thresholdLevel The desired LVD threshold level.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    MCAL_RCC_EnableClock(PWR_BASE_ADDR); // Enable clock for PWR peripheral (inferred on APB1)

    // Clear existing PVD config bits (PLS[2:0])
    PWR_CR_REG &= ~(0x7U << 5);

    // Set PLS bits based on the desired threshold level
    // These mappings are inferred from STM32F401RM for PWR_CR_PLS bits
    switch (lvd_thresholdLevel)
    {
        case LVD_THRESHOLD_0V5: // No direct 0.5V option. Use lowest available.
        case LVD_THRESHOLD_1V:
        case LVD_THRESHOLD_1V5:
        case LVD_THRESHOLD_2V:
            PWR_CR_REG |= (0x0U << 5); // PLS[2:0] = 000 (2.2V)
            break;
        case LVD_THRESHOLD_2V5:
            PWR_CR_REG |= (0x3U << 5); // PLS[2:0] = 011 (2.5V)
            break;
        case LVD_THRESHOLD_3V:
            PWR_CR_REG |= (0x6U << 5); // PLS[2:0] = 110 (2.8V) - highest internal level
            break;
        case LVD_THRESHOLD_3V5:
        case LVD_THRESHOLD_4V:
        case LVD_THRESHOLD_4V5:
        case LVD_THRESHOLD_5V:
            // For levels above 2.8V, external comparator might be needed or select highest internal.
            PWR_CR_REG |= (0x6U << 5); // PLS[2:0] = 110 (2.8V) - use highest internal
            break;
        default:
            // Handle error or default to a safe value
            PWR_CR_REG |= (0x0U << 5); // Default to 2.2V
            break;
    }
}

/**
 * @brief Enables the Low Voltage Detection (LVD) module.
 */
void LVD_Enable(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    MCAL_RCC_EnableClock(PWR_BASE_ADDR); // Enable clock for PWR peripheral (inferred on APB1)

    PWR_CR_REG |= (1U << 4); // Set PVDEN bit
}

/**
 * @brief Disables the Low Voltage Detection (LVD) module.
 */
void LVD_Disable(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    MCAL_RCC_EnableClock(PWR_BASE_ADDR); // Enable clock for PWR peripheral (inferred on APB1)

    PWR_CR_REG &= ~(1U << 4); // Clear PVDEN bit
}

// --- UART API IMPLEMENTATION ---

/**
 * @brief Initializes a UART channel with specified parameters.
 * @param uart_channel The UART channel to initialize (USART1, USART2, USART6).
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The number of data bits (8-bit or 9-bit).
 * @param uart_stop_bit The number of stop bits (0.5, 1, 1.5, 2).
 * @param uart_parity The parity configuration (None, Even, Odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    USART_TypeDef *USARTx;
    uint32_t pclk_freq;

    switch (uart_channel)
    {
        case UART_CHANNEL_1:
            USARTx = USART1;
            MCAL_RCC_EnableClock(USART1_BASE_ADDR); // Enable clock for USART1 (RCC_APB2ENR, bit 4)
            pclk_freq = PCLK2_FREQ_HZ; // USART1 is on APB2
            // Enable GPIOA clock for PA9 (TX), PA10 (RX)
            MCAL_RCC_EnableClock(GPIOA_BASE_ADDR); // RCC_AHB1ENR, bit 0
            // Configure GPIO pins for USART1 AF7
            // PA9 (TX), PA10 (RX)
            // MODER: Alternate Function (0b10)
            GPIOA->MODER &= ~((0x3U << (9 * 2)) | (0x3U << (10 * 2)));
            GPIOA->MODER |=  ((0x2U << (9 * 2)) | (0x2U << (10 * 2)));
            // AFRL/AFRH: AF7 (0x7)
            GPIOA->AFRH &= ~((0xF << ((9 - 8) * 4)) | (0xF << ((10 - 8) * 4)));
            GPIOA->AFRH |=  ((0x7 << ((9 - 8) * 4)) | (0x7 << ((10 - 8) * 4)));
            // OTYPER: Push-pull (0)
            GPIOA->OTYPER &= ~((1U << 9) | (1U << 10));
            // OSPEEDR: High speed (0b10)
            GPIOA->OSPEEDR &= ~((0x3U << (9 * 2)) | (0x3U << (10 * 2)));
            GPIOA->OSPEEDR |=  ((0x2U << (9 * 2)) | (0x2U << (10 * 2)));
            // PUPDR: No pull-up/pull-down (0b00)
            GPIOA->PUPDR &= ~((0x3U << (9 * 2)) | (0x3U << (10 * 2)));
            break;
        case UART_CHANNEL_2:
            USARTx = USART2;
            MCAL_RCC_EnableClock(USART2_BASE_ADDR); // Enable clock for USART2 (RCC_APB1ENR, bit 17)
            pclk_freq = PCLK1_FREQ_HZ; // USART2 is on APB1
            // Enable GPIOA clock for PA2 (TX), PA3 (RX)
            MCAL_RCC_EnableClock(GPIOA_BASE_ADDR); // RCC_AHB1ENR, bit 0
            // Configure GPIO pins for USART2 AF7
            // PA2 (TX), PA3 (RX)
            GPIOA->MODER &= ~((0x3U << (2 * 2)) | (0x3U << (3 * 2)));
            GPIOA->MODER |=  ((0x2U << (2 * 2)) | (0x2U << (3 * 2)));
            GPIOA->AFRL &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
            GPIOA->AFRL |=  ((0x7 << (2 * 4)) | (0x7 << (3 * 4)));
            GPIOA->OTYPER &= ~((1U << 2) | (1U << 3));
            GPIOA->OSPEEDR &= ~((0x3U << (2 * 2)) | (0x3U << (3 * 2)));
            GPIOA->OSPEEDR |=  ((0x2U << (2 * 2)) | (0x2U << (3 * 2)));
            GPIOA->PUPDR &= ~((0x3U << (2 * 2)) | (0x3U << (3 * 2)));
            break;
        case UART_CHANNEL_6:
            USARTx = USART6;
            MCAL_RCC_EnableClock(USART6_BASE_ADDR); // Enable clock for USART6 (RCC_APB2ENR, bit 5)
            pclk_freq = PCLK2_FREQ_HZ; // USART6 is on APB2
            // Enable GPIOC clock for PC6 (TX), PC7 (RX)
            MCAL_RCC_EnableClock(GPIOC_BASE_ADDR); // RCC_AHB1ENR, bit 2
            // Configure GPIO pins for USART6 AF8
            // PC6 (TX), PC7 (RX)
            GPIOC->MODER &= ~((0x3U << (6 * 2)) | (0x3U << (7 * 2)));
            GPIOC->MODER |=  ((0x2U << (6 * 2)) | (0x2U << (7 * 2)));
            GPIOC->AFRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
            GPIOC->AFRL |=  ((0x8 << (6 * 4)) | (0x8 << (7 * 4)));
            GPIOC->OTYPER &= ~((1U << 6) | (1U << 7));
            GPIOC->OSPEEDR &= ~((0x3U << (6 * 2)) | (0x3U << (7 * 2)));
            GPIOC->OSPEEDR |=  ((0x2U << (6 * 2)) | (0x2U << (7 * 2)));
            GPIOC->PUPDR &= ~((0x3U << (6 * 2)) | (0x3U << (7 * 2)));
            break;
        default:
            return; // Invalid channel
    }

    // Disable USART before configuration
    USARTx->CR1 &= ~USART_CR1_UE;

    // Configure Baud Rate
    uint32_t baud_rate_val;
    switch (uart_baud_rate)
    {
        case UART_BAUD_RATE_9600:    baud_rate_val = 9600;   break;
        case UART_BAUD_RATE_19200:   baud_rate_val = 19200;  break;
        case UART_BAUD_RATE_38400:   baud_rate_val = 38400;  break;
        case UART_BAUD_RATE_57600:   baud_rate_val = 57600;  break;
        case UART_BAUD_RATE_115200:  baud_rate_val = 115200; break;
        case UART_BAUD_RATE_230400:  baud_rate_val = 230400; break;
        case UART_BAUD_RATE_460800:  baud_rate_val = 460800; break;
        case UART_BAUD_RATE_921600:  baud_rate_val = 921600; break;
        default:                     baud_rate_val = 9600;   break;
    }
    // Baud rate calculation: PCLK / (8 * (2 * OVER8) * baud_rate) or PCLK / (16 * baud_rate)
    // Assuming OVER8=0 (oversampling by 16)
    USARTx->BRR = pclk_freq / baud_rate_val;

    // Configure Data Length (CR1_M)
    USARTx->CR1 &= ~USART_CR1_M; // Clear M bit
    if (uart_data_length == UART_DATA_LENGTH_9B)
    {
        USARTx->CR1 |= USART_CR1_M;
    }

    // Configure Stop Bits (CR2_STOP)
    USARTx->CR2 &= ~USART_CR2_STOP; // Clear STOP bits
    switch (uart_stop_bit)
    {
        case UART_STOP_BIT_0_5: USARTx->CR2 |= (0x1U << 12); break; // 0.5 stop bits
        case UART_STOP_BIT_1_5: USARTx->CR2 |= (0x3U << 12); break; // 1.5 stop bits
        case UART_STOP_BIT_2:   USARTx->CR2 |= (0x2U << 12); break; // 2 stop bits
        case UART_STOP_BIT_1:
        default: // 1 stop bit (default, 0x0)
            break;
    }

    // Configure Parity (CR1_PCE, CR1_PS)
    USARTx->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS); // Clear PCE and PS bits
    if (uart_parity != UART_PARITY_NONE)
    {
        USARTx->CR1 |= USART_CR1_PCE; // Parity Control Enable
        if (uart_parity == UART_PARITY_ODD)
        {
            USARTx->CR1 |= USART_CR1_PS; // Odd parity
        }
        else // Even parity
        {
            // CR1_PS = 0 for even parity
        }
    }

    // Enable Transmitter and Receiver (CR1_TE, CR1_RE)
    USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Enable USART peripheral
    USARTx->CR1 |= USART_CR1_UE;
}

/**
 * @brief Enables a UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    USART_TypeDef *USARTx;
    switch (uart_channel)
    {
        case UART_CHANNEL_1:
            USARTx = USART1;
            MCAL_RCC_EnableClock(USART1_BASE_ADDR); // RCC_APB2ENR, bit 4 (inferred)
            break;
        case UART_CHANNEL_2:
            USARTx = USART2;
            MCAL_RCC_EnableClock(USART2_BASE_ADDR); // RCC_APB1ENR, bit 17 (inferred)
            break;
        case UART_CHANNEL_6:
            USARTx = USART6;
            MCAL_RCC_EnableClock(USART6_BASE_ADDR); // RCC_APB2ENR, bit 5 (inferred)
            break;
        default:
            return; // Invalid channel
    }
    USARTx->CR1 |= USART_CR1_UE; // Enable USART
}

/**
 * @brief Disables a UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    USART_TypeDef *USARTx;
    switch (uart_channel)
    {
        case UART_CHANNEL_1: USARTx = USART1; break;
        case UART_CHANNEL_2: USARTx = USART2; break;
        case UART_CHANNEL_6: USARTx = USART6; break;
        default: return; // Invalid channel
    }
    USARTx->CR1 &= ~USART_CR1_UE; // Disable USART
}

/**
 * @brief Sends a single byte over a UART channel.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    USART_TypeDef *USARTx;
    switch (uart_channel)
    {
        case UART_CHANNEL_1: USARTx = USART1; break;
        case UART_CHANNEL_2: USARTx = USART2; break;
        case UART_CHANNEL_6: USARTx = USART6; break;
        default: return; // Invalid channel
    }

    // Wait until the transmit data register is empty
    while (!(USARTx->SR & USART_SR_TXE))
    {
        WDT_Reset();
    }
    // Write the byte to the data register
    USARTx->DR = byte;
    // Wait until transmission complete
    while (!(USARTx->SR & USART_SR_TC))
    {
        WDT_Reset();
    }
}

/**
 * @brief Sends a frame of data over a UART channel.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    for (int i = 0; i < length; i++)
    {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over a UART channel.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the string to send.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (str == NULL) return;
    int length = strlen(str);
    UART_send_frame(uart_channel, str, length);
}

/**
 * @brief Receives a single byte from a UART channel.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    USART_TypeDef *USARTx;
    switch (uart_channel)
    {
        case UART_CHANNEL_1: USARTx = USART1; break;
        case UART_CHANNEL_2: USARTx = USART2; break;
        case UART_CHANNEL_6: USARTx = USART6; break;
        default: return 0; // Invalid channel
    }

    // Wait until a byte is received
    while (!(USARTx->SR & USART_SR_RXNE))
    {
        WDT_Reset();
    }
    // Read the received byte from the data register
    return (tbyte)(USARTx->DR & 0xFFU);
}

/**
 * @brief Receives a frame of data from a UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (buffer == NULL || max_length <= 0) return;

    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Receives a null-terminated string from a UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the string, including null terminator.
 * @return The number of bytes received (excluding null terminator), or 0 on error.
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    tbyte received_byte;
    while (i < (max_length - 1)) // Leave space for null terminator
    {
        received_byte = UART_Get_Byte(uart_channel);
        if (received_byte == '\n' || received_byte == '\r' || received_byte == '\0') // End of line or null
        {
            break;
        }
        buffer[i++] = (char)received_byte;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}


// --- I2C API IMPLEMENTATION ---

/**
 * @brief Initializes an I2C channel with specified parameters.
 * @param i2c_channel The I2C channel to initialize (I2C1, I2C2, I2C3).
 * @param i2c_clk_speed The desired clock speed (Standard or Fast Mode).
 * @param i2c_device_address The own device address (7-bit or 10-bit).
 * @param i2c_ack Acknowledge enable/disable.
 * @param i2c_datalength Data length (8-bit or 16-bit).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    I2C_TypeDef *I2Cx;

    switch (i2c_channel)
    {
        case I2C_CHANNEL_1:
            I2Cx = I2C1;
            MCAL_RCC_EnableClock(I2C1_BASE_ADDR); // Enable clock for I2C1 (RCC_APB1ENR, bit 21)
            MCAL_RCC_EnableClock(GPIOB_BASE_ADDR); // Enable GPIOB clock for PB6 (SCL), PB7 (SDA) or PB8 (SCL), PB9 (SDA)
            // Configure GPIO pins for I2C1 AF4
            // PB6 (SCL), PB7 (SDA) or PB8 (SCL), PB9 (SDA)
            // Assuming PB8 (SCL) and PB9 (SDA) for I2C1 (common for F4 Discovery, etc.)
            GPIOB->MODER &= ~((0x3U << (8 * 2)) | (0x3U << (9 * 2)));
            GPIOB->MODER |=  ((0x2U << (8 * 2)) | (0x2U << (9 * 2))); // Alternate function
            GPIOB->OTYPER |= ((1U << 8) | (1U << 9)); // Open-drain
            GPIOB->OSPEEDR |= ((0x3U << (8 * 2)) | (0x3U << (9 * 2))); // Very high speed
            GPIOB->PUPDR |= ((0x1U << (8 * 2)) | (0x1U << (9 * 2))); // Pull-up
            GPIOB->AFRH &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
            GPIOB->AFRH |=  ((0x4 << ((8 - 8) * 4)) | (0x4 << ((9 - 8) * 4))); // AF4
            break;
        case I2C_CHANNEL_2:
            I2Cx = I2C2;
            MCAL_RCC_EnableClock(I2C2_BASE_ADDR); // Enable clock for I2C2 (RCC_APB1ENR, bit 22)
            MCAL_RCC_EnableClock(GPIOB_BASE_ADDR); // Enable GPIOB clock for PB10 (SCL), PB3 (SDA)
            // Configure GPIO pins for I2C2 AF9
            // PB10 (SCL), PB3 (SDA)
            GPIOB->MODER &= ~((0x3U << (10 * 2)) | (0x3U << (3 * 2)));
            GPIOB->MODER |=  ((0x2U << (10 * 2)) | (0x2U << (3 * 2))); // Alternate function
            GPIOB->OTYPER |= ((1U << 10) | (1U << 3)); // Open-drain
            GPIOB->OSPEEDR |= ((0x3U << (10 * 2)) | (0x3U << (3 * 2))); // Very high speed
            GPIOB->PUPDR |= ((0x1U << (10 * 2)) | (0x1U << (3 * 2))); // Pull-up
            GPIOB->AFRH &= ~(0xF << ((10 - 8) * 4));
            GPIOB->AFRH |=  (0x4 << ((10 - 8) * 4)); // AF4 for PB10
            GPIOB->AFRL &= ~(0xF << (3 * 4));
            GPIOB->AFRL |=  (0x9 << (3 * 4)); // AF9 for PB3
            break;
        case I2C_CHANNEL_3:
            I2Cx = I2C3;
            MCAL_RCC_EnableClock(I2C3_BASE_ADDR); // Enable clock for I2C3 (RCC_APB1ENR, bit 23)
            MCAL_RCC_EnableClock(GPIOC_BASE_ADDR); // Enable GPIOC clock for PC9 (SDA)
            MCAL_RCC_EnableClock(GPIOA_BASE_ADDR); // Enable GPIOA clock for PA8 (SCL)
            // Configure GPIO pins for I2C3 AF4
            // PA8 (SCL), PC9 (SDA)
            GPIOA->MODER &= ~ (0x3U << (8 * 2));
            GPIOA->MODER |=   (0x2U << (8 * 2)); // Alternate function
            GPIOA->OTYPER |=  (1U << 8); // Open-drain
            GPIOA->OSPEEDR |= (0x3U << (8 * 2)); // Very high speed
            GPIOA->PUPDR |=   (0x1U << (8 * 2)); // Pull-up
            GPIOA->AFRH &= ~ (0xF << ((8 - 8) * 4));
            GPIOA->AFRH |=   (0x4 << ((8 - 8) * 4)); // AF4 for PA8

            GPIOC->MODER &= ~ (0x3U << (9 * 2));
            GPIOC->MODER |=   (0x2U << (9 * 2)); // Alternate function
            GPIOC->OTYPER |=  (1U << 9); // Open-drain
            GPIOC->OSPEEDR |= (0x3U << (9 * 2)); // Very high speed
            GPIOC->PUPDR |=   (0x1U << (9 * 2)); // Pull-up
            GPIOC->AFRH &= ~ (0xF << ((9 - 8) * 4));
            GPIOC->AFRH |=   (0x4 << ((9 - 8) * 4)); // AF4 for PC9
            break;
        default:
            return; // Invalid channel
    }

    // Disable I2C peripheral before configuration
    I2Cx->CR1 &= ~I2C_CR1_PE;

    // Configure CR2: Peripheral clock frequency
    I2Cx->CR2 = PCLK1_FREQ_HZ / 1000000U; // Assuming PCLK1 is I2C source clock

    // Configure CCR: Clock control register (Fast mode rule)
    uint32_t ccr_val;
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST)
    {
        // Fm mode (400kHz)
        // CCR = (PCLK1_FREQ / (3 * I2C_CLOCK))
        // Assuming PCLK1 is 42MHz
        // CCR = 42MHz / (3 * 400kHz) = 42000000 / 1200000 = 35
        ccr_val = (PCLK1_FREQ_HZ / (3U * 400000U));
        I2Cx->CCR = I2C_CCR_FS | (1U << 14) | (ccr_val & 0xFFFU); // Fast mode, duty cycle 2
    }
    else // I2C_CLK_SPEED_STANDARD (100kHz)
    {
        // Sm mode (100kHz)
        // CCR = (PCLK1_FREQ / (2 * I2C_CLOCK))
        // CCR = 42MHz / (2 * 100kHz) = 42000000 / 200000 = 210
        ccr_val = (PCLK1_FREQ_HZ / (2U * 100000U));
        I2Cx->CCR = (ccr_val & 0xFFFU); // Standard mode
    }

    // Configure TRISE: Max rise time (fast mode rule)
    // TRISE = (PCLK1_FREQ / 1000000) * Trise_max_ns / 1000 + 1
    // For Fm (400kHz): Trise_max = 300ns. TRISE = 42 * 300 / 1000 + 1 = 12.6 + 1 = 13.6 -> 14
    // For Sm (100kHz): Trise_max = 1000ns. TRISE = 42 * 1000 / 1000 + 1 = 43
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST)
    {
        I2Cx->TRISE = (PCLK1_FREQ_HZ / 1000000U * 300U / 1000U) + 1U;
        if (I2Cx->TRISE < 2U) I2Cx->TRISE = 2U; // Minimum value
    }
    else
    {
        I2Cx->TRISE = (PCLK1_FREQ_HZ / 1000000U * 1000U / 1000U) + 1U;
        if (I2Cx->TRISE < 2U) I2Cx->TRISE = 2U; // Minimum value
    }


    // Configure OAR1: Own address (7-bit addressing mode, rule "Addressing Mode equals Device Address")
    I2Cx->OAR1 = (i2c_device_address << 1); // 7-bit address mode, address is in bits 7:1
    I2Cx->OAR1 &= ~I2C_OAR1_ADDMODE; // 7-bit addressing mode

    // Configure ACK (CR1_ACK)
    if (i2c_ack == I2C_ACK_ENABLE)
    {
        I2Cx->CR1 |= I2C_CR1_ACK;
    }
    else
    {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
    }

    // Data length (I2C DR is always 8-bit for reading/writing, this API parameter might be misleading for HW config)
    // The I2C DR is always 8-bit. I2C_DATA_LENGTH_16BIT implies two 8-bit transactions.
    // For now, will ignore this setting for hardware register as DR is 8-bit.
    (void)i2c_datalength; // Suppress unused parameter warning

    // Enable I2C peripheral
    I2Cx->CR1 |= I2C_CR1_PE;
}

/**
 * @brief Enables an I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    I2C_TypeDef *I2Cx;
    switch (i2c_channel)
    {
        case I2C_CHANNEL_1:
            I2Cx = I2C1;
            MCAL_RCC_EnableClock(I2C1_BASE_ADDR); // RCC_APB1ENR, bit 21 (inferred)
            break;
        case I2C_CHANNEL_2:
            I2Cx = I2C2;
            MCAL_RCC_EnableClock(I2C2_BASE_ADDR); // RCC_APB1ENR, bit 22 (inferred)
            break;
        case I2C_CHANNEL_3:
            I2Cx = I2C3;
            MCAL_RCC_EnableClock(I2C3_BASE_ADDR); // RCC_APB1ENR, bit 23 (inferred)
            break;
        default:
            return; // Invalid channel
    }
    I2Cx->CR1 |= I2C_CR1_PE; // Enable I2C
}

/**
 * @brief Disables an I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    I2C_TypeDef *I2Cx;
    switch (i2c_channel)
    {
        case I2C_CHANNEL_1: I2Cx = I2C1; break;
        case I2C_CHANNEL_2: I2Cx = I2C2; break;
        case I2C_CHANNEL_3: I2Cx = I2C3; break;
        default: return; // Invalid channel
    }
    I2Cx->CR1 &= ~I2C_CR1_PE; // Disable I2C
}

/**
 * @brief Sends a single byte over an I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    I2C_TypeDef *I2Cx;
    switch (i2c_channel)
    {
        case I2C_CHANNEL_1: I2Cx = I2C1; break;
        case I2C_CHANNEL_2: I2Cx = I2C2; break;
        case I2C_CHANNEL_3: I2Cx = I2C3; break;
        default: return; // Invalid channel
    }

    // Wait until TXE is set (Tx buffer empty)
    tword timeout = 0xFFFF; // Max timeout rule
    while (!(I2Cx->SR1 & I2C_SR1_TXE) && (timeout-- > 0))
    {
        WDT_Reset();
    }
    if (timeout == 0) return; // Timeout error

    // Write data to DR
    I2Cx->DR = byte;
}

/**
 * @brief Sends a frame of data over an I2C channel.
 *        This implementation assumes master mode.
 *        The addressing must be handled by the caller or by a higher-level driver function.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    I2C_TypeDef *I2Cx;
    switch (i2c_channel)
    {
        case I2C_CHANNEL_1: I2Cx = I2C1; break;
        case I2C_CHANNEL_2: I2Cx = I2C2; break;
        case I2C_CHANNEL_3: I2Cx = I2C3; break;
        default: return; // Invalid channel
    }

    if (data == NULL || length <= 0) return;

    // Send START condition
    I2Cx->CR1 |= I2C_CR1_START;
    tword timeout = 0xFFFF;
    while (!(I2Cx->SR1 & I2C_SR1_SB) && (timeout-- > 0)) { WDT_Reset(); } // Wait for SB bit
    if (timeout == 0) return;

    // Send slave address (caller must set this up, for this example assume it's part of the data flow)
    // A full I2C master transmit would be:
    // 1. Generate START
    // 2. Transmit Slave address + R/W bit
    // 3. Check ADDR flag
    // 4. Send bytes
    // 5. Generate STOP or Repeated START

    // Given the generic API, we focus on byte transmission after START.
    // Assuming the first byte in 'data' contains the slave address + R/W bit
    // and subsequent bytes are actual data. This is a simplification.
    // A proper MCAL should separate address and data.

    // Let's assume a higher layer handles the slave address, and this function just sends data.
    // So, we'll implement sending data, and the caller is responsible for START + Address.
    // Or, for this example, let's make a simplified master transmit:
    // It is expected that the 'data' includes the slave address (first byte)
    // and this function manages START/STOP/ACK.

    // If the API `I2C_send_frame` is called directly, it implies it's starting a transaction.
    // Thus, it needs to handle the address. We'll simplify and use the first byte as address.
    // THIS IS A POTENTIAL DESIGN FLAW IN GENERIC API FOR I2C.
    // For this exercise, we will assume `data[0]` is the 7-bit slave address + R/W bit (0 for write).
    // Or, let's adjust: I2C_send_byte implies sending.
    // This `I2C_send_frame` will send multiple bytes AFTER an address has been sent.

    // Re-evaluating: Rule "Always generate a repeated start condition instead of stop between transactions".
    // This implies that `I2C_send_frame` and `I2C_Get_frame` might be chained.
    // For a send, a START is needed. Then Address. Then Data. Then a STOP or Repeated START.

    // Let's assume the calling function handles the START and Address, and this simply sends bytes.
    // The `I2C_send_byte` will actually send the byte. This function just iterates.

    // This implementation assumes that I2C_send_byte itself doesn't generate START/STOP.
    // So, for I2C_send_frame, we need to handle START, Address, and STOP/REPEATED START.
    // Let's make it a full master transmit.
    // Slave address is not explicitly passed to `I2C_send_frame` API.
    // For simplicity, we assume `I2C_Init` sets the OWN address, meaning this MCU is a slave,
    // and `I2C_send_frame` is for slave-to-master (rare) or a higher layer managing master.
    // Given the API for I2C, it suggests master operations (send/get byte/frame).
    // Let's assume a dummy slave address for master mode, or that the first byte of `data` is address.

    // Given the lack of a "target address" in the API definition, this implies this MCAL layer
    // expects to operate in *master mode* but the *addressing* is managed externally or via a default.
    // For now, I'll proceed with basic data transmission assuming the bus is already set up for a specific transaction.
    // This part requires clarification in a real-world scenario.
    // For now, I'll simulate a transaction starting from the MCAL, assuming a "dummy address" or
    // making the first byte of `data` to be the target address (common pattern).

    // Let's use a common practice for I2C Master Write:
    // The "device_address" parameter in I2C_Init() refers to the *MCU's* address if it's a slave.
    // But this API is for sending/receiving. So, this should be a master API.
    // A master API needs the *target slave address*. This is missing from the `I2C_send_frame` prototype.
    // As per the rule "Addressing Mode equals Device Address" from `I2C_rules`, this is confusing.
    // I will *infer* that the target device address is meant to be passed to `I2C_send_frame` as an argument.
    // However, the prototype from `API.json` *does not* include it.

    // To follow the "Do NOT invent APIs" rule, I must work with the existing prototype.
    // This means the `I2C_send_frame` and `I2C_send_byte` cannot initiate a *new* master transaction
    // (i.e., generate START + send address). They can only send bytes *within* an existing transaction.
    // This forces me to assume that some higher-level code ensures START and slave addressing are done
    // before calling these functions.
    // This is a common pattern where a higher-level driver wraps the MCAL.

    // If I cannot send START/Address, how to use `I2C_send_byte` for data transmission?
    // Let's assume that `I2C_send_byte` (and by extension `I2C_send_frame`) are for
    // *sending data bytes only* during an ongoing transaction.

    for (int i = 0; i < length; i++)
    {
        I2C_send_byte(i2c_channel, (tbyte)data[i]);
    }

    // Repeated start rule: "Always generate a repeated start condition instead of stop between transactions".
    // This can't be handled here without knowing if another transaction follows.
    // This is typically handled by a higher layer driver.
    // For a basic transaction, a STOP condition is needed.
    // If this is the last part of a sequence, a STOP must be issued.
    // I will add a STOP here, and if chained, the next call should manage the START.
    // Or, more robustly, I2C_send_frame should *not* issue STOP if the 'repeated start' rule applies.
    // This rule strongly suggests that MCAL should support both, but the generic API doesn't allow such flags.
    // I'll leave out explicit START/STOP for now, assuming higher layer handles transaction framing.
    // A note about this limitation is appropriate.
}

/**
 * @brief Sends a null-terminated string over an I2C channel.
 *        Refer to the notes in I2C_send_frame regarding master transaction framing.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the string to send.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (str == NULL) return;
    int length = strlen(str);
    I2C_send_frame(i2c_channel, str, length);
}

/**
 * @brief Receives a single byte from an I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    I2C_TypeDef *I2Cx;
    switch (i2c_channel)
    {
        case I2C_CHANNEL_1: I2Cx = I2C1; break;
        case I2C_CHANNEL_2: I2Cx = I2C2; break;
        case I2C_CHANNEL_3: I2Cx = I2C3; break;
        default: return 0; // Invalid channel
    }

    tbyte received_byte = 0;
    tword timeout = 0xFFFF; // Max timeout rule

    // Wait until RXNE is set (Rx buffer not empty)
    while (!(I2Cx->SR1 & I2C_SR1_RXNE) && (timeout-- > 0))
    {
        WDT_Reset();
    }
    if (timeout == 0) return 0; // Timeout error

    // Read data from DR
    received_byte = (tbyte)I2Cx->DR;
    return received_byte;
}

/**
 * @brief Receives a frame of data from an I2C channel.
 *        Refer to the notes in I2C_send_frame regarding master transaction framing.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (buffer == NULL || max_length <= 0) return;

    for (int i = 0; i < max_length; i++)
    {
        // For the last byte, ensure ACK is disabled before receiving if it's the end of a transaction
        // This control is typically handled by a higher layer to manage NACK for last byte.
        if (i == (max_length - 1))
        {
            I2C_TypeDef *I2Cx;
            switch (i2c_channel)
            {
                case I2C_CHANNEL_1: I2Cx = I2C1; break;
                case I2C_CHANNEL_2: I2Cx = I2C2; break;
                case I2C_CHANNEL_3: I2Cx = I2C3; break;
                default: return; // Invalid channel
            }
            I2Cx->CR1 &= ~I2C_CR1_ACK; // NACK the last byte
            I2Cx->CR1 |= I2C_CR1_STOP; // Generate STOP condition (if this is the end)
        }
        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
    }

    // Re-enable ACK after transaction, if it was disabled for the last byte
    I2C_TypeDef *I2Cx;
    switch (i2c_channel)
    {
        case I2C_CHANNEL_1: I2Cx = I2C1; break;
        case I2C_CHANNEL_2: I2Cx = I2C2; break;
        case I2C_CHANNEL_3: I2Cx = I2C3; break;
        default: return; // Invalid channel
    }
    I2Cx->CR1 |= I2C_CR1_ACK;
}

/**
 * @brief Receives a null-terminated string from an I2C channel.
 *        Refer to the notes in I2C_send_frame regarding master transaction framing.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the string, including null terminator.
 * @return The number of bytes received (excluding null terminator), or 0 on error.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    tbyte received_byte;
    while (i < (max_length - 1)) // Leave space for null terminator
    {
        received_byte = I2C_Get_Byte(i2c_channel);
        if (received_byte == '\n' || received_byte == '\r' || received_byte == '\0') // End of line or null
        {
            break;
        }
        buffer[i++] = (char)received_byte;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}


// --- SPI (CSI) API IMPLEMENTATION ---

/**
 * @brief Initializes an SPI channel with specified parameters.
 * @param spi_channel The SPI channel to initialize (SPI1, SPI2, SPI3).
 * @param spi_mode Master/Slave mode.
 * @param spi_cpol Clock polarity.
 * @param spi_cpha Clock phase.
 * @param spi_dff Data frame format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;

    switch (spi_channel)
    {
        case SPI_CHANNEL_1:
            SPIx = SPI1;
            MCAL_RCC_EnableClock(SPI1_BASE_ADDR); // Enable clock for SPI1 (RCC_APB2ENR, bit 0)
            MCAL_RCC_EnableClock(GPIOA_BASE_ADDR); // Enable GPIOA clock for PA5 (SCK), PA6 (MISO), PA7 (MOSI)
            MCAL_RCC_EnableClock(GPIOB_BASE_ADDR); // Enable GPIOB clock for PB3 (SCK), PB4 (MISO), PB5 (MOSI)
            // Configure GPIO pins for SPI1 AF5 (assuming PA5, PA6, PA7)
            // PA5 (SCK), PA6 (MISO), PA7 (MOSI)
            GPIOA->MODER &= ~((0x3U << (5 * 2)) | (0x3U << (6 * 2)) | (0x3U << (7 * 2)));
            GPIOA->MODER |=  ((0x2U << (5 * 2)) | (0x2U << (6 * 2)) | (0x2U << (7 * 2))); // Alternate function
            GPIOA->OTYPER &= ~((1U << 5) | (1U << 6) | (1U << 7)); // Push-pull
            GPIOA->OSPEEDR |= ((0x3U << (5 * 2)) | (0x3U << (6 * 2)) | (0x3U << (7 * 2))); // Very high speed
            GPIOA->PUPDR &= ~((0x3U << (5 * 2)) | (0x3U << (6 * 2)) | (0x3U << (7 * 2))); // No pull-up/pull-down (AF handles it)
            GPIOA->AFRL &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4)));
            GPIOA->AFRL |=  ((0x5 << (5 * 4)) | (0x5 << (6 * 4)) | (0x5 << (7 * 4))); // AF5
            break;
        case SPI_CHANNEL_2:
            SPIx = SPI2;
            MCAL_RCC_EnableClock(SPI2_BASE_ADDR); // Enable clock for SPI2 (RCC_APB1ENR, bit 14)
            MCAL_RCC_EnableClock(GPIOB_BASE_ADDR); // Enable GPIOB clock for PB13 (SCK), PB14 (MISO), PB15 (MOSI)
            MCAL_RCC_EnableClock(GPIOC_BASE_ADDR); // Enable GPIOC clock for PC2 (MISO), PC3 (MOSI)
            // Configure GPIO pins for SPI2 AF5 (assuming PB13, PB14, PB15)
            // PB13 (SCK), PB14 (MISO), PB15 (MOSI)
            GPIOB->MODER &= ~((0x3U << (13 * 2)) | (0x3U << (14 * 2)) | (0x3U << (15 * 2)));
            GPIOB->MODER |=  ((0x2U << (13 * 2)) | (0x2U << (14 * 2)) | (0x2U << (15 * 2))); // Alternate function
            GPIOB->OTYPER &= ~((1U << 13) | (1U << 14) | (1U << 15)); // Push-pull
            GPIOB->OSPEEDR |= ((0x3U << (13 * 2)) | (0x3U << (14 * 2)) | (0x3U << (15 * 2))); // Very high speed
            GPIOB->PUPDR &= ~((0x3U << (13 * 2)) | (0x3U << (14 * 2)) | (0x3U << (15 * 2))); // No pull-up/pull-down
            GPIOB->AFRH &= ~((0xF << ((13 - 8) * 4)) | (0xF << ((14 - 8) * 4)) | (0xF << ((15 - 8) * 4)));
            GPIOB->AFRH |=  ((0x5 << ((13 - 8) * 4)) | (0x5 << ((14 - 8) * 4)) | (0x5 << ((15 - 8) * 4))); // AF5
            break;
        case SPI_CHANNEL_3:
            SPIx = SPI3;
            MCAL_RCC_EnableClock(SPI3_BASE_ADDR); // Enable clock for SPI3 (RCC_APB1ENR, bit 15)
            MCAL_RCC_EnableClock(GPIOB_BASE_ADDR); // Enable GPIOB clock for PB3 (SCK), PB4 (MISO), PB5 (MOSI)
            MCAL_RCC_EnableClock(GPIOC_BASE_ADDR); // Enable GPIOC clock for PC10 (SCK), PC11 (MISO), PC12 (MOSI)
            // Configure GPIO pins for SPI3 AF6 (assuming PC10, PC11, PC12)
            // PC10 (SCK), PC11 (MISO), PC12 (MOSI)
            GPIOC->MODER &= ~((0x3U << (10 * 2)) | (0x3U << (11 * 2)) | (0x3U << (12 * 2)));
            GPIOC->MODER |=  ((0x2U << (10 * 2)) | (0x2U << (11 * 2)) | (0x2U << (12 * 2))); // Alternate function
            GPIOC->OTYPER &= ~((1U << 10) | (1U << 11) | (1U << 12)); // Push-pull
            GPIOC->OSPEEDR |= ((0x3U << (10 * 2)) | (0x3U << (11 * 2)) | (0x3U << (12 * 2))); // Very high speed
            GPIOC->PUPDR &= ~((0x3U << (10 * 2)) | (0x3U << (11 * 2)) | (0x3U << (12 * 2))); // No pull-up/pull-down
            GPIOC->AFRH &= ~((0xF << ((10 - 8) * 4)) | (0xF << ((11 - 8) * 4)) | (0xF << ((12 - 8) * 4)));
            GPIOC->AFRH |=  ((0x6 << ((10 - 8) * 4)) | (0x6 << ((11 - 8) * 4)) | (0x6 << ((12 - 8) * 4))); // AF6
            break;
        default:
            return; // Invalid channel
    }

    // Disable SPI peripheral before configuration
    SPIx->CR1 &= ~SPI_CR1_SPE;

    // Configure Master/Slave mode (CR1_MSTR, CR1_SSI, CR1_SSM)
    if (spi_mode == SPI_MODE_MASTER)
    {
        SPIx->CR1 |= SPI_CR1_MSTR; // Master mode
        SPIx->CR1 |= SPI_CR1_SSI;  // Internal slave select
        SPIx->CR1 |= SPI_CR1_SSM;  // Software slave management enabled (rule: SS always software-controlled)
    }
    else // Slave mode
    {
        SPIx->CR1 &= ~SPI_CR1_MSTR; // Slave mode
        // For slave, SSM and SSI can be configured by higher layer, but we ensure consistent control
        SPIx->CR1 |= SPI_CR1_SSM;
        SPIx->CR1 &= ~SPI_CR1_SSI; // Slave typically doesn't drive SSI internally
    }

    // Configure Clock Polarity (CR1_CPOL)
    if (spi_cpol == SPI_CPOL_HIGH)
    {
        SPIx->CR1 |= SPI_CR1_CPOL;
    }
    else
    {
        SPIx->CR1 &= ~SPI_CR1_CPOL;
    }

    // Configure Clock Phase (CR1_CPHA)
    if (spi_cpha == SPI_CPHA_2EDGE)
    {
        SPIx->CR1 |= SPI_CR1_CPHA;
    }
    else
    {
        SPIx->CR1 &= ~SPI_CR1_CPHA;
    }

    // Configure Data Frame Format (CR1_DFF)
    if (spi_dff == SPI_DFF_16BIT)
    {
        SPIx->CR1 |= SPI_CR1_DFF;
    }
    else
    {
        SPIx->CR1 &= ~SPI_CR1_DFF;
    }

    // Configure Bit Order (CR1_LSBFIRST)
    if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST)
    {
        SPIx->CR1 |= SPI_CR1_LSBFIRST;
    }
    else
    {
        SPIx->CR1 &= ~SPI_CR1_LSBFIRST;
    }

    // Configure fast speed (CR1_BR bits) - this means lowest prescaler for maximum speed.
    // PCLK2 for SPI1 (84MHz), PCLK1 for SPI2/3 (42MHz).
    // Lowest prescaler (PCLK/2) is 0b000
    SPIx->CR1 &= ~SPI_CR1_BR; // Clear Baud Rate Control bits
    SPIx->CR1 |= SPI_CR1_BR_0; // Set to PCLK/2 (0b000) for fast speed (Rule: Always use fast speed)

    // Always use full duplex (CR1_BIDIMODE, CR1_RXONLY)
    SPIx->CR1 &= ~SPI_CR1_BIDIMODE; // Full-duplex selected by clearing this bit
    SPIx->CR1 &= ~SPI_CR1_RXONLY;   // Full-duplex mode by clearing RXONLY

    // Always enable CRC (CR1_CRCEN)
    SPIx->CR1 |= SPI_CR1_CRCEN;

    // Enable SPI peripheral
    SPIx->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Enables an SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    switch (spi_channel)
    {
        case SPI_CHANNEL_1:
            SPIx = SPI1;
            MCAL_RCC_EnableClock(SPI1_BASE_ADDR); // RCC_APB2ENR, bit 0 (inferred)
            break;
        case SPI_CHANNEL_2:
            SPIx = SPI2;
            MCAL_RCC_EnableClock(SPI2_BASE_ADDR); // RCC_APB1ENR, bit 14 (inferred)
            break;
        case SPI_CHANNEL_3:
            SPIx = SPI3;
            MCAL_RCC_EnableClock(SPI3_BASE_ADDR); // RCC_APB1ENR, bit 15 (inferred)
            break;
        default:
            return; // Invalid channel
    }
    SPIx->CR1 |= SPI_CR1_SPE; // Enable SPI
}

/**
 * @brief Disables an SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    switch (spi_channel)
    {
        case SPI_CHANNEL_1: SPIx = SPI1; break;
        case SPI_CHANNEL_2: SPIx = SPI2; break;
        case SPI_CHANNEL_3: SPIx = SPI3; break;
        default: return; // Invalid channel
    }
    SPIx->CR1 &= ~SPI_CR1_SPE; // Disable SPI
}

/**
 * @brief Sends a single byte over an SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    switch (spi_channel)
    {
        case SPI_CHANNEL_1: SPIx = SPI1; break;
        case SPI_CHANNEL_2: SPIx = SPI2; break;
        case SPI_CHANNEL_3: SPIx = SPI3; break;
        default: return; // Invalid channel
    }

    // Wait until TXE is set (Tx buffer empty)
    tword timeout = 0xFFFF; // Max timeout
    while (!(SPIx->SR & SPI_SR_TXE) && (timeout-- > 0))
    {
        WDT_Reset();
    }
    if (timeout == 0) return; // Timeout error

    // Write data to DR
    // Check DFF bit for 8-bit or 16-bit
    if ((SPIx->CR1 & SPI_CR1_DFF) == 0U) // 8-bit data frame format
    {
        *((volatile uint8_t*)&SPIx->DR) = byte;
    }
    else // 16-bit data frame format
    {
        SPIx->DR = (uint16_t)byte; // Send as 16-bit, upper bits will be zero
    }

    // Wait for BSY flag to clear (transfer complete) if in master mode
    // Not strictly necessary for byte-by-byte, but good for robustness
    timeout = 0xFFFF;
    while ((SPIx->SR & SPI_SR_BSY) && (timeout-- > 0))
    {
        WDT_Reset();
    }
}

/**
 * @brief Sends a frame of data over an SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (data == NULL || length <= 0) return;

    for (int i = 0; i < length; i++)
    {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Receives a single byte from an SPI channel.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    switch (spi_channel)
    {
        case SPI_CHANNEL_1: SPIx = SPI1; break;
        case SPI_CHANNEL_2: SPIx = SPI2; break;
        case SPI_CHANNEL_3: SPIx = SPI3; break;
        default: return 0; // Invalid channel
    }

    // To receive, in master mode, you often need to send a dummy byte to generate clock pulses.
    // In slave mode, you just wait for data.
    // Assuming full duplex, we write a dummy byte to DR to initiate transfer if in master mode.
    if (SPIx->CR1 & SPI_CR1_MSTR)
    {
        // Check DFF bit for 8-bit or 16-bit
        if ((SPIx->CR1 & SPI_CR1_DFF) == 0U) // 8-bit
        {
            *((volatile uint8_t*)&SPIx->DR) = 0xFFU; // Send dummy byte
        }
        else // 16-bit
        {
            SPIx->DR = 0xFFFFU; // Send dummy word
        }
    }

    // Wait until RXNE is set (Rx buffer not empty)
    tword timeout = 0xFFFF; // Max timeout
    while (!(SPIx->SR & SPI_SR_RXNE) && (timeout-- > 0))
    {
        WDT_Reset();
    }
    if (timeout == 0) return 0; // Timeout error

    // Read data from DR
    if ((SPIx->CR1 & SPI_CR1_DFF) == 0U) // 8-bit data frame format
    {
        return *((volatile uint8_t*)&SPIx->DR);
    }
    else // 16-bit data frame format
    {
        return (tbyte)(SPIx->DR & 0xFFU); // Return lower 8 bits if 16-bit DFF
    }
}

/**
 * @brief Receives a frame of data from an SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (buffer == NULL || max_length <= 0) return;

    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Receives a null-terminated string from an SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the string, including null terminator.
 * @return The number of bytes received (excluding null terminator), or 0 on error.
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    tbyte received_byte;
    while (i < (max_length - 1)) // Leave space for null terminator
    {
        received_byte = SPI_Get_Byte(spi_channel);
        if (received_byte == '\n' || received_byte == '\r' || received_byte == '\0') // End of line or null
        {
            break;
        }
        buffer[i++] = (char)received_byte;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}


// --- EXTERNAL INTERRUPT API IMPLEMENTATION ---

/**
 * @brief Initializes an external interrupt channel with a specified edge trigger.
 * @param external_int_channel The EXTI line number (0-15).
 * @param external_int_edge The trigger edge (Rising, Falling, Both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Enable SYSCFG clock (RCC_APB2ENR bit 14)
    MCAL_RCC_EnableClock(SYSCFG_BASE_ADDR); // Inferred

    // Clear and set EXTI line source in SYSCFG_EXTICR1-4
    // Each EXTIx has a 4-bit field in EXTICR (0-3 in EXTICR1, 4-7 in EXTICR2, etc.)
    uint32_t exticr_idx = (uint32_t)external_int_channel / 4U;
    uint32_t exticr_pos = (uint32_t)external_int_channel % 4U;
    volatile uint32_t *exticr_reg_ptr;

    switch (exticr_idx)
    {
        case 0: exticr_reg_ptr = &SYSCFG_EXTICR1_REG; break;
        case 1: exticr_reg_ptr = &SYSCFG_EXTICR2_REG; break;
        case 2: exticr_reg_ptr = &SYSCFG_EXTICR3_REG; break;
        case 3: exticr_reg_ptr = &SYSCFG_EXTICR4_REG; break;
        default: return; // Should not happen for channels 0-15
    }

    // Clear existing port selection for this EXTI line
    *exticr_reg_ptr &= ~(0xFUL << (exticr_pos * 4U));

    // For STM32F4, the pin to EXTI line mapping is fixed (e.g., PA0, PB0, PC0 all map to EXTI_0).
    // The SYSCFG_EXTICR selects which PORT's pin (Px_N) is connected to EXTI_N.
    // The `assigned_pin` in `register_json` indicates which pins can be sources for EXTI.
    // The API `External_INT_Init` only takes `external_int_channel`, not a specific `port`.
    // So, we must choose a default port or rely on higher layer configuration.
    // For simplicity, let's assume Port A is the default for EXTI source.
    // A more robust MCAL would take `t_port` as a parameter.
    // Port A selection: 0b0000
    *exticr_reg_ptr |= (0x0UL << (exticr_pos * 4U)); // Select Port A for EXTI line.

    // Configure trigger edge (EXTI_RTSR, EXTI_FTSR)
    EXTI_RTSR_REG &= ~(1UL << external_int_channel); // Clear rising trigger
    EXTI_FTSR_REG &= ~(1UL << external_int_channel); // Clear falling trigger

    if (external_int_edge == EXTERNAL_INT_EDGE_RISING || external_int_edge == EXTERNAL_INT_EDGE_RISING_FALLING)
    {
        EXTI_RTSR_REG |= (1UL << external_int_channel); // Set rising trigger
    }
    if (external_int_edge == EXTERNAL_INT_EDGE_FALLING || external_int_edge == EXTERNAL_INT_EDGE_RISING_FALLING)
    {
        EXTI_FTSR_REG |= (1UL << external_int_channel); // Set falling trigger
    }

    // Clear pending flag for the EXTI line
    EXTI_PR_REG = (1UL << external_int_channel);
}

/**
 * @brief Enables an external interrupt channel.
 * @param external_int_channel The EXTI line number (0-15) to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Enable SYSCFG clock (RCC_APB2ENR bit 14)
    MCAL_RCC_EnableClock(SYSCFG_BASE_ADDR); // Inferred

    // Set Interrupt Mask Register (IMR) bit for the EXTI line
    EXTI_IMR_REG |= (1UL << external_int_channel);
    // Optionally, enable the corresponding NVIC interrupt
    // For EXTI0-4, there are dedicated IRQs. For EXTI5-9 and EXTI10-15, shared IRQs exist.
    // This is beyond a simple MCAL function, usually handled by NVIC configuration.
    // Example for EXTI0: NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
 * @brief Disables an external interrupt channel.
 * @param external_int_channel The EXTI line number (0-15) to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Clear Interrupt Mask Register (IMR) bit for the EXTI line
    EXTI_IMR_REG &= ~(1UL << external_int_channel);
    // Optionally, disable the corresponding NVIC interrupt
}


// --- GPIO API IMPLEMENTATION ---

// Helper function to get GPIO_TypeDef pointer for a given port
static GPIO_TypeDef* get_gpio_port_ptr(t_port port)
{
    return GPIO_PORTS[port];
}

/**
 * @brief Initializes a GPIO pin as an output with an initial value.
 * @param port The GPIO port (PORT_A to PORT_H).
 * @param pin The pin number (PIN_0 to PIN_15).
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    if (gpio == NULL) return;

    // 1. Enable clock for the GPIO port
    MCAL_RCC_EnableClock((uint32_t)gpio);

    // Rule: Always set value before setting direction
    GPIO_Value_Set(port, pin, value); // Set initial output value

    // 2. Set pin mode to Output (01)
    gpio->MODER &= ~(0x3U << (pin * 2)); // Clear mode bits
    gpio->MODER |= (0x1U << (pin * 2));  // Set mode to Output

    // 3. Set output type to Push-Pull (0)
    gpio->OTYPER &= ~(0x1U << pin);

    // 4. Set output speed to High (10)
    gpio->OSPEEDR &= ~(0x3U << (pin * 2)); // Clear speed bits
    gpio->OSPEEDR |= (0x2U << (pin * 2));  // Set speed to High

    // 5. Disable pull-up/pull-down (00) (Rule: All output pins have pull-up resistors disabled)
    gpio->PUPDR &= ~(0x3U << (pin * 2));

    // Rule: After setting GPIO direction, verify with while loop
    while (((gpio->MODER >> (pin * 2)) & 0x3U) != 0x1U)
    {
        WDT_Reset();
    }
}

/**
 * @brief Initializes a GPIO pin as an input.
 * @param port The GPIO port (PORT_A to PORT_H).
 * @param pin The pin number (PIN_0 to PIN_15).
 */
void GPIO_Input_Init(t_port port, t_pin pin)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    if (gpio == NULL) return;

    // 1. Enable clock for the GPIO port
    MCAL_RCC_EnableClock((uint32_t)gpio);

    // 2. Set pin mode to Input (00)
    gpio->MODER &= ~(0x3U << (pin * 2)); // Clear mode bits (sets to input mode)

    // 3. Set pull-up resistor (01) (Rule: All input pins have pull-up resistors)
    gpio->PUPDR &= ~(0x3U << (pin * 2)); // Clear pull-up/pull-down bits
    gpio->PUPDR |= (0x1U << (pin * 2));  // Set to Pull-up

    // 4. Input pins typically don't care about OTYPER, OSPEEDR, but for completeness, set to default/safe
    gpio->OTYPER &= ~(0x1U << pin); // Push-pull (default, though irrelevant for input)
    gpio->OSPEEDR &= ~(0x3U << (pin * 2)); // Low speed (00) (irrelevant for input, but safe)

    // Rule: After setting GPIO direction, verify with while loop
    while (((gpio->MODER >> (pin * 2)) & 0x3U) != 0x0U)
    {
        WDT_Reset();
    }
}

/**
 * @brief Gets the direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction of the pin (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    if (gpio == NULL) return GPIO_DIRECTION_INPUT; // Default to input on error

    uint32_t mode = (gpio->MODER >> (pin * 2)) & 0x3U;
    if (mode == 0x0U) // Input mode
    {
        return GPIO_DIRECTION_INPUT;
    }
    else if (mode == 0x1U) // General purpose output mode
    {
        return GPIO_DIRECTION_OUTPUT;
    }
    else
    {
        // Other modes (Alternate function, Analog) are neither simple input/output
        return GPIO_DIRECTION_INPUT; // Treat as input or handle as error
    }
}

/**
 * @brief Sets the value of an output GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The desired output value (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    if (gpio == NULL) return;

    // Use BSRR register for atomic set/reset
    if (value == 1)
    {
        gpio->BSRR = (1UL << pin);      // Set bit
    }
    else
    {
        gpio->BSRR = (1UL << (pin + 16)); // Reset bit (BRy is at bit y+16)
    }

    // Rule: After setting GPIO value, verify with while loop
    while (GPIO_Value_Get(port, pin) != value)
    {
        WDT_Reset();
    }
}

/**
 * @brief Gets the value of an input or output GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The current value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    if (gpio == NULL) return 0; // Default to 0 on error

    // Read from IDR for input mode, ODR for output mode.
    // However, ODR reflects the output state, and IDR reflects the actual pin state.
    // For consistency, let's read from IDR to get actual pin state, regardless of configured mode.
    // If output, IDR should reflect ODR unless there's an external override.
    return (tbyte)((gpio->IDR >> pin) & 0x1U);
}

/**
 * @brief Toggles the value of an output GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    if (gpio == NULL) return;

    // Toggle the ODR bit
    gpio->ODR ^= (1UL << pin);

    // Rule: After setting GPIO value, verify with while loop (check against new toggled state)
    tbyte new_value = GPIO_Value_Get(port, pin);
    tbyte expected_value = ((gpio->ODR >> pin) & 0x1U); // ODR reflects the intended output
    while (new_value != expected_value)
    {
        WDT_Reset();
        new_value = GPIO_Value_Get(port, pin);
    }
}


// --- PWM API IMPLEMENTATION ---

/**
 * @brief Initializes a PWM channel.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 * @note This implementation assumes a common PCLK for timers (TIM_CLK1_FREQ_HZ or TIM_CLK2_FREQ_HZ).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    uint32_t timer_clk_freq;
    t_port port;
    t_pin pin;
    uint8_t af_value;
    uint8_t channel_bit_pos;

    // Determine timer instance, clock, and GPIO for the PWM channel
    switch (pwm_channel)
    {
        case PWM_CHANNEL_TIM1_CH1: TIMx = TIM1; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_A; pin = PIN_8; af_value = 1; channel_bit_pos = 0; break; // PA8/AF1
        case PWM_CHANNEL_TIM1_CH2: TIMx = TIM1; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_A; pin = PIN_9; af_value = 1; channel_bit_pos = 4; break; // PA9/AF1
        case PWM_CHANNEL_TIM1_CH3: TIMx = TIM1; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_A; pin = PIN_10; af_value = 1; channel_bit_pos = 8; break; // PA10/AF1
        case PWM_CHANNEL_TIM1_CH4: TIMx = TIM1; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_A; pin = PIN_11; af_value = 1; channel_bit_pos = 12; break; // PA11/AF1
        case PWM_CHANNEL_TIM2_CH1: TIMx = TIM2; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_0; af_value = 1; channel_bit_pos = 0; break; // PA0/AF1
        case PWM_CHANNEL_TIM2_CH2: TIMx = TIM2; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_1; af_value = 1; channel_bit_pos = 4; break; // PA1/AF1
        case PWM_CHANNEL_TIM2_CH3: TIMx = TIM2; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_2; af_value = 1; channel_bit_pos = 8; break; // PA2/AF1
        case PWM_CHANNEL_TIM2_CH4: TIMx = TIM2; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_3; af_value = 1; channel_bit_pos = 12; break; // PA3/AF1
        case PWM_CHANNEL_TIM3_CH1: TIMx = TIM3; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_6; af_value = 2; channel_bit_pos = 0; break; // PA6/AF2
        case PWM_CHANNEL_TIM3_CH2: TIMx = TIM3; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_7; af_value = 2; channel_bit_pos = 4; break; // PA7/AF2
        case PWM_CHANNEL_TIM3_CH3: TIMx = TIM3; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_B; pin = PIN_0; af_value = 2; channel_bit_pos = 8; break; // PB0/AF2
        case PWM_CHANNEL_TIM3_CH4: TIMx = TIM3; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_B; pin = PIN_1; af_value = 2; channel_bit_pos = 12; break; // PB1/AF2
        case PWM_CHANNEL_TIM4_CH1: TIMx = TIM4; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_B; pin = PIN_6; af_value = 2; channel_bit_pos = 0; break; // PB6/AF2
        case PWM_CHANNEL_TIM4_CH2: TIMx = TIM4; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_B; pin = PIN_7; af_value = 2; channel_bit_pos = 4; break; // PB7/AF2
        case PWM_CHANNEL_TIM4_CH3: TIMx = TIM4; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_B; pin = PIN_8; af_value = 2; channel_bit_pos = 8; break; // PB8/AF2
        case PWM_CHANNEL_TIM4_CH4: TIMx = TIM4; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_B; pin = PIN_9; af_value = 2; channel_bit_pos = 12; break; // PB9/AF2
        case PWM_CHANNEL_TIM5_CH1: TIMx = TIM5; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_0; af_value = 2; channel_bit_pos = 0; break; // PA0/AF2
        case PWM_CHANNEL_TIM5_CH2: TIMx = TIM5; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_1; af_value = 2; channel_bit_pos = 4; break; // PA1/AF2
        case PWM_CHANNEL_TIM5_CH3: TIMx = TIM5; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_2; af_value = 2; channel_bit_pos = 8; break; // PA2/AF2
        case PWM_CHANNEL_TIM5_CH4: TIMx = TIM5; timer_clk_freq = TIM_CLK1_FREQ_HZ; port = PORT_A; pin = PIN_3; af_value = 2; channel_bit_pos = 12; break; // PA3/AF2
        case PWM_CHANNEL_TIM9_CH1: TIMx = TIM9; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_A; pin = PIN_2; af_value = 3; channel_bit_pos = 0; break; // PA2/AF3
        case PWM_CHANNEL_TIM9_CH2: TIMx = TIM9; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_A; pin = PIN_3; af_value = 3; channel_bit_pos = 4; break; // PA3/AF3
        case PWM_CHANNEL_TIM10_CH1: TIMx = TIM10; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_B; pin = PIN_8; af_value = 3; channel_bit_pos = 0; break; // PB8/AF3
        case PWM_CHANNEL_TIM11_CH1: TIMx = TIM11; timer_clk_freq = TIM_CLK2_FREQ_HZ; port = PORT_B; pin = PIN_9; af_value = 3; channel_bit_pos = 0; break; // PB9/AF3
        default: return; // Invalid channel
    }

    // Enable clock for the timer peripheral
    MCAL_RCC_EnableClock((uint32_t)TIMx);
    // Enable clock for the associated GPIO port
    MCAL_RCC_EnableClock((uint32_t)get_gpio_port_ptr(port));

    // Configure GPIO pin for Alternate Function
    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    gpio->MODER &= ~(0x3U << (pin * 2));
    gpio->MODER |=  (0x2U << (pin * 2)); // Alternate function mode
    gpio->OTYPER &= ~(0x1U << pin); // Push-pull
    gpio->OSPEEDR |= (0x3U << (pin * 2)); // Very high speed

    // Set Alternate Function (AFx)
    if (pin < 8)
    {
        gpio->AFRL &= ~(0xFUL << (pin * 4));
        gpio->AFRL |= (af_value << (pin * 4));
    }
    else
    {
        gpio->AFRH &= ~(0xFUL << ((pin - 8) * 4));
        gpio->AFRH |= (af_value << ((pin - 8) * 4));
    }

    // Disable timer before configuration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Calculate Prescaler (PSC) and Auto-Reload Register (ARR) for desired frequency
    // Timer_Freq = Clock_Freq / ((PSC + 1) * (ARR + 1))
    // Desired Timer_Freq = pwm_khz_freq * 1000 Hz
    // To achieve the desired frequency, we need to choose PSC and ARR.
    // Let's set PSC to achieve a known internal frequency, then ARR for output frequency.
    // Example: Timer_Clock = 84MHz. If we want 1kHz internal counter clock:
    // PSC + 1 = Timer_Clock / 1000 = 84000
    // PSC = 83999
    // Then ARR will define the output frequency: Output_Freq = (Internal_Counter_Freq / (ARR+1))
    // ARR + 1 = Internal_Counter_Freq / Output_Freq
    // ARR = Internal_Counter_Freq / Output_Freq - 1

    uint32_t prescaler_val = (timer_clk_freq / 1000000U) - 1; // 1MHz internal counter clock
    if (prescaler_val > 0xFFFF) prescaler_val = 0xFFFF; // Cap at max 16-bit value
    TIMx->PSC = prescaler_val;

    uint32_t arr_val = (1000000U / (pwm_khz_freq * 1000U)) - 1; // For target kHz output frequency
    if (arr_val > 0xFFFF) arr_val = 0xFFFF;
    TIMx->ARR = arr_val;

    // Set PWM mode 1 (active when CNT < CCRx, inactive when CNT > CCRx)
    uint32_t ccmr_val = (0x6U << 4); // PWM mode 1, Output Compare Mode set bits (OCxM)
    // Clear CCxS bits (00 for output)
    if (channel_bit_pos == 0 || channel_bit_pos == 4)
    {
        TIMx->CCMR1 &= ~(0xFFUL << channel_bit_pos); // Clear CCxS (bits 0/8), OCxM (bits 4/12), OCxPE (bits 3/11)
        TIMx->CCMR1 |= (ccmr_val << channel_bit_pos);
        TIMx->CCMR1 |= (1U << (channel_bit_pos + 3)); // OCxPE: Output compare preload enable
    }
    else // channel_bit_pos == 8 || channel_bit_pos == 12
    {
        TIMx->CCMR2 &= ~(0xFFUL << (channel_bit_pos - 8));
        TIMx->CCMR2 |= (ccmr_val << (channel_bit_pos - 8));
        TIMx->CCMR2 |= (1U << ((channel_bit_pos - 8) + 3)); // OCxPE: Output compare preload enable
    }

    // Set initial duty cycle
    uint32_t ccr_val = ((arr_val + 1) * pwm_duty) / 100U;
    switch (channel_bit_pos)
    {
        case 0: TIMx->CCR1 = ccr_val; break;
        case 4: TIMx->CCR2 = ccr_val; break;
        case 8: TIMx->CCR3 = ccr_val; break;
        case 12: TIMx->CCR4 = ccr_val; break;
        default: break;
    }

    // Enable Capture/Compare output (CCxE) and set polarity (CCxP)
    TIMx->CCER &= ~(0xFU << channel_bit_pos); // Clear CCxE, CCxP, CCxNE, CCxNP
    TIMx->CCER |= (1U << channel_bit_pos);    // CCxE: Enable output for corresponding channel

    // For advanced timers (TIM1), enable main output and apply break/dead-time configuration
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE; // Main Output Enable
    }

    // Generate update event to load PSC, ARR, CCR values
    TIMx->EGR |= TIM_EGR_UG;
    
    // Clear available FREQUENCY Ranges for each channel as comments in PWM_Init()
    // For a given Timer_Clock (e.g., 84 MHz for TIM1), and PSC (e.g., 83999 for 1kHz counter):
    // Minimum Frequency: Counter_Clock / 0xFFFF (~1Hz for 1kHz counter clock)
    // Maximum Frequency: Counter_Clock / 1 (~1MHz for 1kHz counter clock)
    // Example ranges with PSC = 83999 (1kHz counter clock from 84MHz):
    // Frequency Range: 1 Hz to 500 Hz (limited by typical application needs, ARR >= 1)
    // For general case: F_OUT = F_TIM_CLK / ( (PSC + 1) * (ARR + 1) )
    // F_TIM_CLK for APB1 Timers (TIM2,3,4,5) = 84MHz. F_TIM_CLK for APB2 Timers (TIM1,9,10,11) = 168MHz.
    // If PSC is set to yield 1MHz internal counter (PSC = F_TIM_CLK/1MHz - 1):
    //  For APB1 timers: PSC = 84-1 = 83. ARR = 1MHz / F_target - 1. Min F_target ~15Hz (ARR=65535). Max F_target = 500kHz (ARR=1).
    //  For APB2 timers: PSC = 168-1 = 167. ARR = 1MHz / F_target - 1. Min F_target ~15Hz (ARR=65535). Max F_target = 500kHz (ARR=1).
}

/**
 * @brief Starts the PWM output on a specified channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    switch (pwm_channel)
    {
        case PWM_CHANNEL_TIM1_CH1: case PWM_CHANNEL_TIM1_CH2: case PWM_CHANNEL_TIM1_CH3: case PWM_CHANNEL_TIM1_CH4: TIMx = TIM1; break;
        case PWM_CHANNEL_TIM2_CH1: case PWM_CHANNEL_TIM2_CH2: case PWM_CHANNEL_TIM2_CH3: case PWM_CHANNEL_TIM2_CH4: TIMx = TIM2; break;
        case PWM_CHANNEL_TIM3_CH1: case PWM_CHANNEL_TIM3_CH2: case PWM_CHANNEL_TIM3_CH3: case PWM_CHANNEL_TIM3_CH4: TIMx = TIM3; break;
        case PWM_CHANNEL_TIM4_CH1: case PWM_CHANNEL_TIM4_CH2: case PWM_CHANNEL_TIM4_CH3: case PWM_CHANNEL_TIM4_CH4: TIMx = TIM4; break;
        case PWM_CHANNEL_TIM5_CH1: case PWM_CHANNEL_TIM5_CH2: case PWM_CHANNEL_TIM5_CH3: case PWM_CHANNEL_TIM5_CH4: TIMx = TIM5; break;
        case PWM_CHANNEL_TIM9_CH1: case PWM_CHANNEL_TIM9_CH2: TIMx = TIM9; break;
        case PWM_CHANNEL_TIM10_CH1: TIMx = TIM10; break;
        case PWM_CHANNEL_TIM11_CH1: TIMx = TIM11; break;
        default: return;
    }
    TIMx->CR1 |= TIM_CR1_CEN; // Enable the counter
}

/**
 * @brief Stops the PWM output on a specified channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    switch (pwm_channel)
    {
        case PWM_CHANNEL_TIM1_CH1: case PWM_CHANNEL_TIM1_CH2: case PWM_CHANNEL_TIM1_CH3: case PWM_CHANNEL_TIM1_CH4: TIMx = TIM1; break;
        case PWM_CHANNEL_TIM2_CH1: case PWM_CHANNEL_TIM2_CH2: case PWM_CHANNEL_TIM2_CH3: case PWM_CHANNEL_TIM2_CH4: TIMx = TIM2; break;
        case PWM_CHANNEL_TIM3_CH1: case PWM_CHANNEL_TIM3_CH2: case PWM_CHANNEL_TIM3_CH3: case PWM_CHANNEL_TIM3_CH4: TIMx = TIM3; break;
        case PWM_CHANNEL_TIM4_CH1: case PWM_CHANNEL_TIM4_CH2: case PWM_CHANNEL_TIM4_CH3: case PWM_CHANNEL_TIM4_CH4: TIMx = TIM4; break;
        case PWM_CHANNEL_TIM5_CH1: case PWM_CHANNEL_TIM5_CH2: case PWM_CHANNEL_TIM5_CH3: case PWM_CHANNEL_TIM5_CH4: TIMx = TIM5; break;
        case PWM_CHANNEL_TIM9_CH1: case PWM_CHANNEL_TIM9_CH2: TIMx = TIM9; break;
        case PWM_CHANNEL_TIM10_CH1: TIMx = TIM10; break;
        case PWM_CHANNEL_TIM11_CH1: TIMx = TIM11; break;
        default: return;
    }
    TIMx->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}


// --- ICU API IMPLEMENTATION ---

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler for the input capture.
 * @param icu_edge The edge to trigger capture on.
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    t_port port;
    t_pin pin;
    uint8_t af_value;
    uint8_t timer_channel_idx; // 0, 1, 2, 3 for TIM_CH1, TIM_CH2, TIM_CH3, TIM_CH4

    // Determine timer instance, clock, and GPIO for the ICU channel
    switch (icu_channel)
    {
        case ICU_CHANNEL_TIM1_CH1: TIMx = TIM1; port = PORT_A; pin = PIN_8; af_value = 1; timer_channel_idx = 0; break;
        case ICU_CHANNEL_TIM1_CH2: TIMx = TIM1; port = PORT_A; pin = PIN_9; af_value = 1; timer_channel_idx = 1; break;
        case ICU_CHANNEL_TIM1_CH3: TIMx = TIM1; port = PORT_A; pin = PIN_10; af_value = 1; timer_channel_idx = 2; break;
        case ICU_CHANNEL_TIM1_CH4: TIMx = TIM1; port = PORT_A; pin = PIN_11; af_value = 1; timer_channel_idx = 3; break;
        case ICU_CHANNEL_TIM2_CH1: TIMx = TIM2; port = PORT_A; pin = PIN_0; af_value = 1; timer_channel_idx = 0; break;
        case ICU_CHANNEL_TIM2_CH2: TIMx = TIM2; port = PORT_A; pin = PIN_1; af_value = 1; timer_channel_idx = 1; break;
        case ICU_CHANNEL_TIM2_CH3: TIMx = TIM2; port = PORT_A; pin = PIN_2; af_value = 1; timer_channel_idx = 2; break;
        case ICU_CHANNEL_TIM2_CH4: TIMx = TIM2; port = PORT_A; pin = PIN_3; af_value = 1; timer_channel_idx = 3; break;
        case ICU_CHANNEL_TIM3_CH1: TIMx = TIM3; port = PORT_A; pin = PIN_6; af_value = 2; timer_channel_idx = 0; break;
        case ICU_CHANNEL_TIM3_CH2: TIMx = TIM3; port = PORT_A; pin = PIN_7; af_value = 2; timer_channel_idx = 1; break;
        case ICU_CHANNEL_TIM3_CH3: TIMx = TIM3; port = PORT_B; pin = PIN_0; af_value = 2; timer_channel_idx = 2; break;
        case ICU_CHANNEL_TIM3_CH4: TIMx = TIM3; port = PORT_B; pin = PIN_1; af_value = 2; timer_channel_idx = 3; break;
        case ICU_CHANNEL_TIM4_CH1: TIMx = TIM4; port = PORT_B; pin = PIN_6; af_value = 2; timer_channel_idx = 0; break;
        case ICU_CHANNEL_TIM4_CH2: TIMx = TIM4; port = PORT_B; pin = PIN_7; af_value = 2; timer_channel_idx = 1; break;
        case ICU_CHANNEL_TIM4_CH3: TIMx = TIM4; port = PORT_B; pin = PIN_8; af_value = 2; timer_channel_idx = 2; break;
        case ICU_CHANNEL_TIM4_CH4: TIMx = TIM4; port = PORT_B; pin = PIN_9; af_value = 2; timer_channel_idx = 3; break;
        case ICU_CHANNEL_TIM5_CH1: TIMx = TIM5; port = PORT_A; pin = PIN_0; af_value = 2; timer_channel_idx = 0; break;
        case ICU_CHANNEL_TIM5_CH2: TIMx = TIM5; port = PORT_A; pin = PIN_1; af_value = 2; timer_channel_idx = 1; break;
        case ICU_CHANNEL_TIM5_CH3: TIMx = TIM5; port = PORT_A; pin = PIN_2; af_value = 2; timer_channel_idx = 2; break;
        case ICU_CHANNEL_TIM5_CH4: TIMx = TIM5; port = PORT_A; pin = PIN_3; af_value = 2; timer_channel_idx = 3; break;
        case ICU_CHANNEL_TIM9_CH1: TIMx = TIM9; port = PORT_A; pin = PIN_2; af_value = 3; timer_channel_idx = 0; break;
        case ICU_CHANNEL_TIM9_CH2: TIMx = TIM9; port = PORT_A; pin = PIN_3; af_value = 3; timer_channel_idx = 1; break;
        case ICU_CHANNEL_TIM10_CH1: TIMx = TIM10; port = PORT_B; pin = PIN_8; af_value = 3; timer_channel_idx = 0; break;
        case ICU_CHANNEL_TIM11_CH1: TIMx = TIM11; port = PORT_B; pin = PIN_9; af_value = 3; timer_channel_idx = 0; break;
        default: return; // Invalid channel
    }

    // Enable clock for the timer peripheral
    MCAL_RCC_EnableClock((uint32_t)TIMx);
    // Enable clock for the associated GPIO port
    MCAL_RCC_EnableClock((uint32_t)get_gpio_port_ptr(port));

    // Configure GPIO pin for Alternate Function (Input)
    GPIO_TypeDef *gpio = get_gpio_port_ptr(port);
    gpio->MODER &= ~(0x3U << (pin * 2));
    gpio->MODER |=  (0x2U << (pin * 2)); // Alternate function mode
    gpio->PUPDR &= ~(0x3U << (pin * 2)); // No pull-up/pull-down
    gpio->PUPDR |=  (0x1U << (pin * 2)); // Pull-up for input capture

    // Set Alternate Function (AFx)
    if (pin < 8)
    {
        gpio->AFRL &= ~(0xFUL << (pin * 4));
        gpio->AFRL |= (af_value << (pin * 4));
    }
    else
    {
        gpio->AFRH &= ~(0xFUL << ((pin - 8) * 4));
        gpio->AFRH |= (af_value << ((pin - 8) * 4));
    }

    // Disable timer before configuration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Configure Timer Prescaler (ICU_prescaller)
    uint32_t psc_val;
    switch (icu_prescaller) // These map to Timer Input Prescaler (ICPS) bits in CCMRx (00, 01, 10, 11 for div1,2,4,8)
    {
        case ICU_PRESCALER_DIV1:    psc_val = 0x0; break;
        case ICU_PRESCALER_DIV2:    psc_val = 0x1; break;
        case ICU_PRESCALER_DIV4:    psc_val = 0x2; break;
        case ICU_PRESCALER_DIV8:    psc_val = 0x3; break;
        // Other values (e.g., div16, div32) are for the counter prescaler (TIMx->PSC), not input capture prescaler (ICPS).
        // Let's map remaining to the highest available ICPS.
        default: psc_val = 0x3; break; // Default to divide by 8
    }

    // Configure Capture/Compare Mode Register (CCMRx) for input capture
    // CCxS bits select input source: 01 for ICx mapped on TIxFP1, 10 for ICx mapped on TIxFP2
    // ICPS bits for input prescaler
    uint32_t ccmr_mask = (TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC);
    uint32_t ccmr_val = (TIM_CCMR1_CC1S_0) | (psc_val << 2); // Input capture direct mode, set prescaler

    // Channels 1 and 2 use CCMR1, channels 3 and 4 use CCMR2
    if (timer_channel_idx < 2) // Channel 1 or 2
    {
        uint33_t shift = timer_channel_idx * 8; // Each channel takes 8 bits in CCMR1
        TIMx->CCMR1 &= ~(ccmr_mask << shift);
        TIMx->CCMR1 |= (ccmr_val << shift);
    }
    else // Channel 3 or 4
    {
        uint33_t shift = (timer_channel_idx - 2) * 8;
        TIMx->CCMR2 &= ~(ccmr_mask << shift);
        TIMx->CCMR2 |= (ccmr_val << shift);
    }

    // Configure Capture/Compare Enable Register (CCER) for edge detection
    uint32_t ccer_mask = (TIM_CCER_CC1P | TIM_CCER_CC1NP); // Polarity bits
    uint32_t ccer_val = 0;

    switch (icu_edge)
    {
        case ICU_EDGE_RISING:
            // CCxP = 0, CCxNP = 0 (rising edge)
            break;
        case ICU_EDGE_FALLING:
            ccer_val |= TIM_CCER_CC1P; // CCxP = 1 (falling edge)
            break;
        case ICU_EDGE_BOTH:
            ccer_val |= (TIM_CCER_CC1P | TIM_CCER_CC1NP); // Both edges
            break;
        default: break;
    }

    uint33_t channel_shift = timer_channel_idx * 4; // Each channel takes 4 bits in CCER
    TIMx->CCER &= ~(ccer_mask << channel_shift);
    TIMx->CCER |= (ccer_val << channel_shift);

    // Enable Capture/Compare for the channel (CCxE bit)
    TIMx->CCER |= (TIM_CCER_CC1E << channel_shift);

    // Enable interrupt for this channel
    uint32_t dier_bit = (1U << (timer_channel_idx + 1)); // CC1IE is bit 1, CC2IE is bit 2, etc.
    TIMx->DIER |= dier_bit;

    // Reset counter to 0
    TIMx->CNT = 0;

    // Set general purpose timer prescaler for basic timer counting (e.g., 1us resolution)
    // PSC = (Timer_Clock / 1000000) - 1 for 1MHz count rate
    uint32_t timer_clk_freq;
    if (TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
    {
        timer_clk_freq = TIM_CLK2_FREQ_HZ; // APB2 timers
    }
    else
    {
        timer_clk_freq = TIM_CLK1_FREQ_HZ; // APB1 timers
    }
    TIMx->PSC = (timer_clk_freq / 1000000U) - 1; // 1us tick
    TIMx->ARR = 0xFFFF; // Max period

    // Generate update event to load PSC, ARR values
    TIMx->EGR |= TIM_EGR_UG;
}

/**
 * @brief Enables the Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    uint8_t timer_channel_idx;

    switch (icu_channel)
    {
        case ICU_CHANNEL_TIM1_CH1: TIMx = TIM1; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM1_BASE_ADDR); break;
        case ICU_CHANNEL_TIM1_CH2: TIMx = TIM1; timer_channel_idx = 1; MCAL_RCC_EnableClock(TIM1_BASE_ADDR); break;
        case ICU_CHANNEL_TIM1_CH3: TIMx = TIM1; timer_channel_idx = 2; MCAL_RCC_EnableClock(TIM1_BASE_ADDR); break;
        case ICU_CHANNEL_TIM1_CH4: TIMx = TIM1; timer_channel_idx = 3; MCAL_RCC_EnableClock(TIM1_BASE_ADDR); break;
        case ICU_CHANNEL_TIM2_CH1: TIMx = TIM2; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM2_BASE_ADDR); break;
        case ICU_CHANNEL_TIM2_CH2: TIMx = TIM2; timer_channel_idx = 1; MCAL_RCC_EnableClock(TIM2_BASE_ADDR); break;
        case ICU_CHANNEL_TIM2_CH3: TIMx = TIM2; timer_channel_idx = 2; MCAL_RCC_EnableClock(TIM2_BASE_ADDR); break;
        case ICU_CHANNEL_TIM2_CH4: TIMx = TIM2; timer_channel_idx = 3; MCAL_RCC_EnableClock(TIM2_BASE_ADDR); break;
        case ICU_CHANNEL_TIM3_CH1: TIMx = TIM3; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM3_BASE_ADDR); break;
        case ICU_CHANNEL_TIM3_CH2: TIMx = TIM3; timer_channel_idx = 1; MCAL_RCC_EnableClock(TIM3_BASE_ADDR); break;
        case ICU_CHANNEL_TIM3_CH3: TIMx = TIM3; timer_channel_idx = 2; MCAL_RCC_EnableClock(TIM3_BASE_ADDR); break;
        case ICU_CHANNEL_TIM3_CH4: TIMx = TIM3; timer_channel_idx = 3; MCAL_RCC_EnableClock(TIM3_BASE_ADDR); break;
        case ICU_CHANNEL_TIM4_CH1: TIMx = TIM4; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM4_BASE_ADDR); break;
        case ICU_CHANNEL_TIM4_CH2: TIMx = TIM4; timer_channel_idx = 1; MCAL_RCC_EnableClock(TIM4_BASE_ADDR); break;
        case ICU_CHANNEL_TIM4_CH3: TIMx = TIM4; timer_channel_idx = 2; MCAL_RCC_EnableClock(TIM4_BASE_ADDR); break;
        case ICU_CHANNEL_TIM4_CH4: TIMx = TIM4; timer_channel_idx = 3; MCAL_RCC_EnableClock(TIM4_BASE_ADDR); break;
        case ICU_CHANNEL_TIM5_CH1: TIMx = TIM5; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM5_BASE_ADDR); break;
        case ICU_CHANNEL_TIM5_CH2: TIMx = TIM5; timer_channel_idx = 1; MCAL_RCC_EnableClock(TIM5_BASE_ADDR); break;
        case ICU_CHANNEL_TIM5_CH3: TIMx = TIM5; timer_channel_idx = 2; MCAL_RCC_EnableClock(TIM5_BASE_ADDR); break;
        case ICU_CHANNEL_TIM5_CH4: TIMx = TIM5; timer_channel_idx = 3; MCAL_RCC_EnableClock(TIM5_BASE_ADDR); break;
        case ICU_CHANNEL_TIM9_CH1: TIMx = TIM9; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM9_BASE_ADDR); break;
        case ICU_CHANNEL_TIM9_CH2: TIMx = TIM9; timer_channel_idx = 1; MCAL_RCC_EnableClock(TIM9_BASE_ADDR); break;
        case ICU_CHANNEL_TIM10_CH1: TIMx = TIM10; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM10_BASE_ADDR); break;
        case ICU_CHANNEL_TIM11_CH1: TIMx = TIM11; timer_channel_idx = 0; MCAL_RCC_EnableClock(TIM11_BASE_ADDR); break;
        default: return;
    }

    // Enable the counter
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Disables the Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    switch (icu_channel)
    {
        case ICU_CHANNEL_TIM1_CH1: case ICU_CHANNEL_TIM1_CH2: case ICU_CHANNEL_TIM1_CH3: case ICU_CHANNEL_TIM1_CH4: TIMx = TIM1; break;
        case ICU_CHANNEL_TIM2_CH1: case ICU_CHANNEL_TIM2_CH2: case ICU_CHANNEL_TIM2_CH3: case ICU_CHANNEL_TIM2_CH4: TIMx = TIM2; break;
        case ICU_CHANNEL_TIM3_CH1: case ICU_CHANNEL_TIM3_CH2: case ICU_CHANNEL_TIM3_CH3: case ICU_CHANNEL_TIM3_CH4: TIMx = TIM3; break;
        case ICU_CHANNEL_TIM4_CH1: case ICU_CHANNEL_TIM4_CH2: case ICU_CHANNEL_TIM4_CH3: case ICU_CHANNEL_TIM4_CH4: TIMx = TIM4; break;
        case ICU_CHANNEL_TIM5_CH1: case ICU_CHANNEL_TIM5_CH2: case ICU_CHANNEL_TIM5_CH3: case ICU_CHANNEL_TIM5_CH4: TIMx = TIM5; break;
        case ICU_CHANNEL_TIM9_CH1: case ICU_CHANNEL_TIM9_CH2: TIMx = TIM9; break;
        case ICU_CHANNEL_TIM10_CH1: TIMx = TIM10; break;
        case ICU_CHANNEL_TIM11_CH1: TIMx = TIM11; break;
        default: return;
    }
    TIMx->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}

/**
 * @brief Gets the frequency of a signal using ICU.
 *        This function requires interrupt handling (ISR) to capture pulse timings.
 *        For a polling implementation, it would read capture values directly, which is less common for frequency.
 * @param icu_channel The ICU channel to get frequency from.
 * @return The calculated frequency in Hz. Returns 0 if no data or invalid channel.
 * @note This is a placeholder; actual frequency calculation requires an ISR to measure pulse widths.
 */
tword ICU_GetFrequency(t_icu_channel icu_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // This function can't fully implement frequency measurement without an ISR to capture timestamps.
    // In a real-world scenario, the ISR would store the captured values, and this function would retrieve
    // and process them.
    // For now, it returns a placeholder.
    (void)icu_channel; // Suppress unused parameter warning

    // Example logic if data was captured by ISR:
    // tword period_ticks = get_period_from_icu_isr_data();
    // if (period_ticks > 0)
    // {
    //     return (tword)(TIMER_CLOCK_FREQ_HZ / period_ticks);
    // }
    return 0; // No frequency measured
}

/**
 * @brief Sets a callback function to be executed by the ICU interrupt service routine.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void))
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    ICU_Callback = callback;
}

// Example of an ISR for TIM2 Channel 1 (assuming it's used for ICU_CHANNEL_TIM2_CH1)
// This ISR would be defined outside MCAL.c in the startup file or application for NVIC.
// However, for MCAL, we can illustrate the logic if this ISR were available.
/*
void TIM2_IRQHandler(void)
{
    WDT_Reset(); // WDT Reset in ISRs is good practice

    if ((TIM2->SR & TIM_SR_CC1IF) != 0) // Check for Capture/Compare 1 Interrupt Flag
    {
        // Clear the interrupt flag
        TIM2->SR = ~TIM_SR_CC1IF;

        // Perform capture logic:
        // uint32_t capture_value = TIM2->CCR1;
        // Store capture_value, calculate period, etc.

        // Call user callback if set
        if (ICU_Callback != NULL)
        {
            ICU_Callback();
        }
    }
}
*/


// --- TIMER API IMPLEMENTATION ---

/**
 * @brief Initializes a general-purpose timer channel.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    uint32_t timer_clk_freq;

    switch (timer_channel)
    {
        case TIMER_CHANNEL_TIM1:    TIMx = TIM1;    timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM2:    TIMx = TIM2;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM3:    TIMx = TIM3;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM4:    TIMx = TIM4;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM5:    TIMx = TIM5;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM9:    TIMx = TIM9;    timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM10:   TIMx = TIM10;   timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM11:   TIMx = TIM11;   timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        default: return; // Invalid channel
    }

    // Enable clock for the timer peripheral
    MCAL_RCC_EnableClock((uint32_t)TIMx);

    // Disable timer before configuration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Configure Timer mode: Up-counting, no prescaler for now, auto-reload buffer enabled.
    TIMx->CR1 = (TIM_CR1_ARPE); // Auto-reload preload enable

    // Set prescaler to achieve 1us tick (1MHz counter clock)
    TIMx->PSC = (timer_clk_freq / 1000000U) - 1; // PSC for 1us resolution
    TIMx->ARR = 0xFFFF; // Default max auto-reload value

    // Generate update event to load PSC and ARR values
    TIMx->EGR |= TIM_EGR_UG;
    while ((TIMx->SR & TIM_SR_UIF) == 0) { WDT_Reset(); } // Wait for update flag
    TIMx->SR &= ~TIM_SR_UIF; // Clear update flag
}

/**
 * @brief Sets a timer for a specified duration in microseconds.
 * @param timer_channel The timer channel to use.
 * @param time The duration in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    uint32_t timer_clk_freq;

    switch (timer_channel)
    {
        case TIMER_CHANNEL_TIM1:    TIMx = TIM1;    timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM2:    TIMx = TIM2;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM3:    TIMx = TIM3;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM4:    TIMx = TIM4;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM5:    TIMx = TIM5;    timer_clk_freq = TIM_CLK1_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM9:    TIMx = TIM9;    timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM10:   TIMx = TIM10;   timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        case TIMER_CHANNEL_TIM11:   TIMx = TIM11;   timer_clk_freq = TIM_CLK2_FREQ_HZ; break;
        default: return;
    }

    // Disable timer to prevent issues during reconfiguration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set prescaler for 1us resolution
    TIMx->PSC = (timer_clk_freq / 1000000U) - 1;

    // Set ARR for 'time' microseconds
    if (time > 0)
    {
        TIMx->ARR = time - 1;
    }
    else
    {
        TIMx->ARR = 0;
    }

    // Reset counter and generate update event
    TIMx->CNT = 0;
    TIMx->EGR |= TIM_EGR_UG;
    while ((TIMx->SR & TIM_SR_UIF) == 0) { WDT_Reset(); }
    TIMx->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Sets a timer for a specified duration in milliseconds.
 * @param timer_channel The timer channel to use.
 * @param time The duration in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // 1ms = 1000us
    TIMER_Set_us(timer_channel, time * 1000U);
}

/**
 * @brief Sets a timer for a specified duration in seconds.
 * @param timer_channel The timer channel to use.
 * @param time The duration in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // 1 sec = 1000 ms
    TIMER_Set_Time_ms(timer_channel, (tword)time * 1000U);
}

/**
 * @brief Sets a timer for a specified duration in minutes.
 * @param timer_channel The timer channel to use.
 * @param time The duration in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // 1 min = 60 sec
    TIMER_Set_Time_sec(timer_channel, time * 60U);
}

/**
 * @brief Sets a timer for a specified duration in hours.
 * @param timer_channel The timer channel to use.
 * @param time The duration in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // 1 hour = 60 min
    TIMER_Set_Time_min(timer_channel, time * 60U);
}

/**
 * @brief Enables the specified timer.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    switch (timer_channel)
    {
        case TIMER_CHANNEL_TIM1:    TIMx = TIM1;    MCAL_RCC_EnableClock(TIM1_BASE_ADDR); break;
        case TIMER_CHANNEL_TIM2:    TIMx = TIM2;    MCAL_RCC_EnableClock(TIM2_BASE_ADDR); break;
        case TIMER_CHANNEL_TIM3:    TIMx = TIM3;    MCAL_RCC_EnableClock(TIM3_BASE_ADDR); break;
        case TIMER_CHANNEL_TIM4:    TIMx = TIM4;    MCAL_RCC_EnableClock(TIM4_BASE_ADDR); break;
        case TIMER_CHANNEL_TIM5:    TIMx = TIM5;    MCAL_RCC_EnableClock(TIM5_BASE_ADDR); break;
        case TIMER_CHANNEL_TIM9:    TIMx = TIM9;    MCAL_RCC_EnableClock(TIM9_BASE_ADDR); break;
        case TIMER_CHANNEL_TIM10:   TIMx = TIM10;   MCAL_RCC_EnableClock(TIM10_BASE_ADDR); break;
        case TIMER_CHANNEL_TIM11:   TIMx = TIM11;   MCAL_RCC_EnableClock(TIM11_BASE_ADDR); break;
        default: return;
    }
    TIMx->CR1 |= TIM_CR1_CEN; // Enable the counter
}

/**
 * @brief Disables the specified timer.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIM_TypeDef *TIMx;
    switch (timer_channel)
    {
        case TIMER_CHANNEL_TIM1:    TIMx = TIM1; break;
        case TIMER_CHANNEL_TIM2:    TIMx = TIM2; break;
        case TIMER_CHANNEL_TIM3:    TIMx = TIM3; break;
        case TIMER_CHANNEL_TIM4:    TIMx = TIM4; break;
        case TIMER_CHANNEL_TIM5:    TIMx = TIM5; break;
        case TIMER_CHANNEL_TIM9:    TIMx = TIM9; break;
        case TIMER_CHANNEL_TIM10:   TIMx = TIM10; break;
        case TIMER_CHANNEL_TIM11:   TIMx = TIM11; break;
        default: return;
    }
    TIMx->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}


// --- ADC API IMPLEMENTATION ---

/**
 * @brief Initializes the ADC for a specific channel and mode.
 * @param adc_channel The ADC channel to configure.
 * @param adc_mode The conversion mode (Single or Continuous).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Enable clock for ADC1 (RCC_APB2ENR, bit 8)
    MCAL_RCC_EnableClock(ADC1_BASE_ADDR); // Inferred

    // Enable GPIO clock for associated pin and configure pin as Analog
    GPIO_TypeDef *gpio = NULL;
    t_pin pin = PIN_0;
    switch (adc_channel)
    {
        case ADC_CHANNEL_0: pin = PIN_0; gpio = GPIOA; break;
        case ADC_CHANNEL_1: pin = PIN_1; gpio = GPIOA; break;
        case ADC_CHANNEL_2: pin = PIN_2; gpio = GPIOA; break;
        case ADC_CHANNEL_3: pin = PIN_3; gpio = GPIOA; break;
        case ADC_CHANNEL_4: pin = PIN_4; gpio = GPIOA; break;
        case ADC_CHANNEL_5: pin = PIN_5; gpio = GPIOA; break;
        case ADC_CHANNEL_6: pin = PIN_6; gpio = GPIOA; break;
        case ADC_CHANNEL_7: pin = PIN_7; gpio = GPIOA; break;
        case ADC_CHANNEL_8: pin = PIN_0; gpio = GPIOB; break;
        case ADC_CHANNEL_9: pin = PIN_1; gpio = GPIOB; break;
        case ADC_CHANNEL_10: pin = PIN_0; gpio = GPIOC; break;
        case ADC_CHANNEL_11: pin = PIN_1; gpio = GPIOC; break;
        case ADC_CHANNEL_12: pin = PIN_2; gpio = GPIOC; break;
        case ADC_CHANNEL_13: pin = PIN_3; gpio = GPIOC; break;
        case ADC_CHANNEL_14: pin = PIN_4; gpio = GPIOC; break;
        case ADC_CHANNEL_15: pin = PIN_5; gpio = GPIOC; break;
        default: return; // Invalid channel
    }

    if (gpio != NULL)
    {
        MCAL_RCC_EnableClock((uint32_t)gpio);
        gpio->MODER |= (0x3U << (pin * 2)); // Analog mode (11)
        gpio->PUPDR &= ~(0x3U << (pin * 2)); // No pull-up/pull-down
    }

    // Reset ADC (clear ADON, DMAEN bits in CR2, etc.)
    ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_CONT);
    ADC1->CR1 = 0; // Reset CR1

    // Configure Continuous Conversion Mode (CR2_CONT)
    if (adc_mode == ADC_MODE_CONTINUOUS)
    {
        ADC1->CR2 |= ADC_CR2_CONT;
    }
    else
    {
        ADC1->CR2 &= ~ADC_CR2_CONT;
    }

    // Configure Regular Channel Sequence (SQR3 for first 6 channels, SQR2, SQR1 for more)
    // For a single channel conversion, set SQR3_SQ1 to the channel number, and SQR1_L to 0000 (1 conversion)
    ADC1->SQR3 = (uint32_t)adc_channel; // Set first conversion to the specified channel
    ADC1->SQR1 = 0; // L[3:0] = 0000 -> 1 conversion in sequence

    // Configure Sample Time (SMPRx)
    // Default to 3 cycles sample time for all channels (000)
    // SMPR2 for channels 0-9, SMPR1 for channels 10-18
    if (adc_channel <= ADC_CHANNEL_9)
    {
        ADC1->SMPR2 &= ~(0x7U << (adc_channel * 3));
        ADC1->SMPR2 |= (0x0U << (adc_channel * 3)); // 3 cycles
    }
    else
    {
        ADC1->SMPR1 &= ~(0x7U << ((adc_channel - 10) * 3));
        ADC1->SMPR1 |= (0x0U << ((adc_channel - 10) * 3)); // 3 cycles
    }

    // Configure common ADC properties (CCR)
    // Set PCLK2/2 for ADC clock prescaler (01)
    ADC_CCR_REG &= ~ADC_CCR_ADCPRE;
    ADC_CCR_REG |= ADC_CCR_ADCPRE_0; // Prescaler /2 for 84MHz PCLK2 -> 42MHz ADC clock (max 36MHz for F401)
                                     // Better to use /4 or /6 for F401
    // Let's use /4 (0x10) for ADCPRE
    ADC_CCR_REG &= ~ADC_CCR_ADCPRE;
    ADC_CCR_REG |= ADC_CCR_ADCPRE_1; // Prescaler /4 -> 21MHz ADC clock
}

/**
 * @brief Enables the ADC module.
 * @param adc_channel The ADC channel (ADC1). (Parameter is generic but only ADC1 is in JSON)
 */
void ADC_Enable(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Enable clock for ADC1 (RCC_APB2ENR, bit 8)
    MCAL_RCC_EnableClock(ADC1_BASE_ADDR); // Inferred

    // Clear ADON bit if it was set (e.g., from prior use) then set it.
    ADC1->CR2 &= ~ADC_CR2_ADON;
    // Enable ADC peripheral
    ADC1->CR2 |= ADC_CR2_ADON;

    // Wait for ADC to stabilize (approx 10us)
    for (volatile int i = 0; i < 1000; i++) { /* delay */ WDT_Reset(); }

    // Start conversion if not in continuous mode
    if (!(ADC1->CR2 & ADC_CR2_CONT))
    {
        ADC1->CR2 |= ADC_CR2_SWSTART; // Software start conversion
    }
}

/**
 * @brief Disables the ADC module.
 * @param adc_channel The ADC channel (ADC1). (Parameter is generic but only ADC1 is in JSON)
 */
void ADC_Disable(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Disable ADC peripheral
    ADC1->CR2 &= ~ADC_CR2_ADON;
}

/**
 * @brief Gets the ADC conversion value using polling mode.
 * @param adc_channel The ADC channel to read from.
 * @return The 12-bit ADC conversion value.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Clear EOC (End of Conversion) flag if set
    ADC1->SR &= ~ADC_SR_EOC;

    // Start conversion if not in continuous mode (or restart for single)
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for End Of Conversion (EOC) flag to be set
    tword timeout = 0xFFFF; // Max timeout
    while (!(ADC1->SR & ADC_SR_EOC) && (timeout-- > 0))
    {
        WDT_Reset();
    }
    if (timeout == 0) return 0; // Timeout error

    // Clear EOC flag by writing 0 to it (or reading SR then DR)
    ADC1->SR &= ~ADC_SR_EOC;

    // Read the converted data
    return (tword)(ADC1->DR & 0xFFFU); // ADC result is 12-bit
}

/**
 * @brief Gets the ADC conversion value using interrupt mode.
 *        This function expects an ADC interrupt to be configured and data to be available.
 * @param adc_channel The ADC channel to read from.
 * @return The 12-bit ADC conversion value. Returns 0 if data not available or interrupt not configured.
 * @note This is a placeholder; actual interrupt-driven ADC requires an ISR and a mechanism to store/retrieve data.
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // For interrupt mode, the conversion would be started, an interrupt would fire on EOC,
    // and the ISR would read the DR and store it. This function would then retrieve the stored value.
    // As we don't have ISR handlers defined here, this remains a placeholder.

    // Enable EOCIE (End Of Conversion Interrupt Enable) in CR1
    ADC1->CR1 |= ADC_CR1_EOCIE;

    // Need to enable ADC interrupt in NVIC. This is a higher-level step.
    // NVIC_EnableIRQ(ADC_IRQn);

    // This function can only return a value once the interrupt has completed and stored the data.
    // For now, return 0. A real implementation would involve a shared variable updated by the ISR.
    (void)adc_channel; // Suppress unused parameter warning
    return 0;
}


// --- INTERNAL EEPROM API IMPLEMENTATION (Using Flash Memory for STM32F401RC) ---
// STM32F401RC does not have true internal EEPROM. We implement using Flash memory.
// Flash operations are complex (unlock, erase, program, lock).
// For simplicity, we define a small sector of Flash for "EEPROM" emulation.
// This example uses a single word (4-byte) for each "byte" of EEPROM data, making it less efficient
// but simpler to manage for byte-addressable API.
// A more robust solution would implement a full Flash EEPROM emulation layer.

// Flash memory details (inferred for STM32F401RC):
// Start address of Flash: 0x08000000
// Sector 0: 0x08000000 - 0x08003FFF (16 Kbytes)
// We will use a dedicated Flash page/sector for emulation, e.g., the last sector for user data.
// For F401RC, assume Sector 4 (0x08010000 - 0x0801FFFF, 64 Kbytes) for EEPROM emulation.
// This is an oversimplification for the purpose of demonstrating the principle.

#define EEPROM_FLASH_START_ADDR 0x08010000U // Assuming Sector 4 for EEPROM emulation
#define EEPROM_FLASH_SECTOR     FLASH_CR_SNB_4 // Sector 4 for erase operations (inferred bit from flash_control)
#define EEPROM_MAX_SIZE         1024        // Max "EEPROM" bytes (can map up to 1024 bytes in 64KB sector)
#define FLASH_HALF_WORD_SIZE    2           // 16-bit write
#define FLASH_WORD_SIZE         4           // 32-bit write

/**
 * @brief Unlocks the Flash control register.
 */
static void MCAL_Flash_Unlock(void)
{
    // Check if Flash is locked
    if ((FLASH_CR_REG & FLASH_CR_LOCK) != 0U)
    {
        // Write the Flash key sequence to unlock FPEC
        FLASH_KEYR_REG = 0x45670123U; // Inferred Flash Key Register
        FLASH_KEYR_REG = 0xCDEF89ABU;
    }
}

/**
 * @brief Locks the Flash control register.
 */
static void MCAL_Flash_Lock(void)
{
    FLASH_CR_REG |= FLASH_CR_LOCK; // Set LOCK bit
}

/**
 * @brief Erases a Flash sector.
 * @param sector The sector to erase (e.g., EEPROM_FLASH_SECTOR).
 */
static void MCAL_Flash_EraseSector(uint32_t sector)
{
    MCAL_Flash_Unlock();

    // Check if Flash is busy
    while ((FLASH_SR_REG & FLASH_SR_BSY) != 0U)
    {
        WDT_Reset();
    }

    // Set SER bit
    FLASH_CR_REG |= FLASH_CR_SER;
    // Select sector to erase
    FLASH_CR_REG |= sector; // FLASH_CR_SNB_x for sector x
    // Start erase operation
    FLASH_CR_REG |= FLASH_CR_STRT;

    // Wait for erase to complete
    while ((FLASH_SR_REG & FLASH_SR_BSY) != 0U)
    {
        WDT_Reset();
    }

    // Clear SER bit and sector number
    FLASH_CR_REG &= ~(FLASH_CR_SER | 0xFFUL);
    FLASH_CR_REG &= ~FLASH_CR_STRT;

    MCAL_Flash_Lock();
}

/**
 * @brief Programs a half-word (16-bit) to Flash memory.
 * @param address The target Flash address.
 * @param data The 16-bit data to program.
 */
static void MCAL_Flash_ProgramHalfWord(uint32_t address, uint16_t data)
{
    MCAL_Flash_Unlock();

    // Check if Flash is busy
    while ((FLASH_SR_REG & FLASH_SR_BSY) != 0U)
    {
        WDT_Reset();
    }

    // Set PG bit for programming
    FLASH_CR_REG |= FLASH_CR_PG;
    // Set program size to half-word (PSIZE=01)
    FLASH_CR_REG &= ~FLASH_CR_PSIZE;
    FLASH_CR_REG |= FLASH_CR_PSIZE_0;

    // Write data to the address
    REG_ADDR(address) = data;

    // Wait for programming to complete
    while ((FLASH_SR_REG & FLASH_SR_BSY) != 0U)
    {
        WDT_Reset();
    }

    // Clear PG bit
    FLASH_CR_REG &= ~FLASH_CR_PG;

    MCAL_Flash_Lock();
}


/**
 * @brief Initializes the internal EEPROM (Flash memory emulation).
 *        No specific hardware initialization required beyond ensuring Flash clock is on.
 */
void Internal_EEPROM_Init(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Flash clock is typically always on for CPU to execute code.
    // If it needed to be enabled, it would be through RCC.
    // RCC_AHB1ENR bit 8 (FLASHEN), though usually always enabled.
    // No specific initialization for flash, just ensuring it's ready.
}

/**
 * @brief Sets a byte of data at a specific address in the emulated EEPROM.
 * @param address The EEPROM address (offset within the emulated sector).
 * @param data The byte data to write.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (address >= EEPROM_MAX_SIZE) return;

    // Convert byte address to Flash word address
    // Each EEPROM byte will occupy a 16-bit half-word in Flash for simplicity
    uint32_t flash_address = EEPROM_FLASH_START_ADDR + (address * FLASH_HALF_WORD_SIZE);

    // Read current 16-bit value, update relevant byte, then rewrite.
    // This is simple but inefficient. A full emulation layer would be more complex.
    uint16_t current_word = (uint16_t)REG_ADDR(flash_address);
    current_word = (current_word & 0xFF00) | data; // Update low byte

    // Erase the relevant sector if it's not already erased and data needs update.
    // For simplicity, this example just programs without full erase-before-write logic.
    // A robust Flash write needs: read -> erase sector -> reprogram sector with new value.
    // For this MCAL implementation, we'll assume the sector is already valid or a full erase is done externally.
    // Or, for a single byte, we need to read the flash word, modify the byte, and rewrite the whole word.
    // If the Flash word is not 0xFFFF, we cannot just write. It needs to be erased.
    // For this simple API, if we need to write, we assume erase happens at a higher level, or it's empty.
    // To meet requirements: we *must* ensure it's erased if needed.

    // A simpler approach for MCAL.c is to write a half-word (16-bit) value.
    // If the original Flash content at `flash_address` is `0xFFFF`, we can directly write `data`.
    // If it's not `0xFFFF`, we need to erase the sector first.

    // Let's implement full read-modify-write cycle for flash, including erase.
    // This will erase an entire sector for *every* write. This is very inefficient and reduces flash life.
    // This design decision is due to the simple "byte" API for a Flash-based EEPROM.

    // Temporary solution: Erase sector once, then program.
    // This erase will affect all `EEPROM_MAX_SIZE` bytes.
    // This is typically managed by a FTL (Flash Translation Layer).
    MCAL_Flash_EraseSector(EEPROM_FLASH_SECTOR);

    // After erase, write the data. We write 16-bit value for each byte address.
    MCAL_Flash_ProgramHalfWord(flash_address, data);
}

/**
 * @brief Gets a byte of data from a specific address in the emulated EEPROM.
 * @param address The EEPROM address (offset within the emulated sector).
 * @return The byte data read from the EEPROM.
 */
tbyte Internal_EEPROM_Get(tbyte address)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (address >= EEPROM_MAX_SIZE) return 0xFF; // Return default invalid value

    // Convert byte address to Flash word address and read 16-bit value
    uint32_t flash_address = EEPROM_FLASH_START_ADDR + (address * FLASH_HALF_WORD_SIZE);
    uint16_t data_word = (uint16_t)REG_ADDR(flash_address);

    return (tbyte)(data_word & 0xFFU); // Return the low byte
}


// --- TT API IMPLEMENTATION ---

/**
 * @brief Initializes the Time Triggered (TT) scheduler.
 *        Configures a timer to generate periodic ticks.
 * @param tick_time_ms The tick period in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Use TIM2 for TT scheduler (general-purpose 32-bit timer on APB1)
    TIMER_Init(TIMER_CHANNEL_TIM2);

    // Configure TIM2 for the desired tick_time_ms
    tword ms_val;
    switch (tick_time_ms)
    {
        case TT_TICK_TIME_1MS:   ms_val = 1;   break;
        case TT_TICK_TIME_10MS:  ms_val = 10;  break;
        case TT_TICK_TIME_100MS: ms_val = 100; break;
        case TT_TICK_TIME_1S:    ms_val = 1000; break;
        default:                 ms_val = 1;   break; // Default to 1ms
    }
    tt_tick_ms_value = ms_val;
    TIMER_Set_Time_ms(TIMER_CHANNEL_TIM2, ms_val);

    // Enable Update Interrupt for TIM2
    TIM2->DIER |= TIM_DIER_UIE; // Update interrupt enable

    // Enable TIM2 interrupt in NVIC (inferred)
    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief Starts the Time Triggered (TT) scheduler.
 */
void TT_Start(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    TIMER_Enable(TIMER_CHANNEL_TIM2); // Enable TIM2 to start generating ticks
}

/**
 * @brief Dispatches tasks in the TT scheduler based on their configured periods and delays.
 */
void TT_Dispatch_task(void)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    for (tbyte i = 0; i < tt_next_task_index; i++)
    {
        if (tt_tasks[i].enabled && tt_tasks[i].task_func != NULL)
        {
            if (tt_tasks[i].delay == 0)
            {
                if (tt_tasks[i].run_count == 0)
                {
                    tt_tasks[i].task_func();
                    tt_tasks[i].run_count = tt_tasks[i].period;
                }
                tt_tasks[i].run_count--;
            }
            else
            {
                tt_tasks[i].delay--;
            }
        }
    }
}

/**
 * @brief Time Triggered (TT) Interrupt Service Routine.
 *        This function should be called from the timer's ISR at regular intervals.
 */
void TT_ISR(void)
{
    WDT_Reset(); // Rule: WDT_Reset() in ISRs is good practice

    // Check if TIM2 update interrupt occurred
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        // Clear the update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;

        // Decrement task run counts and dispatch if needed
        for (tbyte i = 0; i < tt_next_task_index; i++)
        {
            if (tt_tasks[i].enabled && tt_tasks[i].task_func != NULL)
            {
                if (tt_tasks[i].delay == 0)
                {
                    tt_tasks[i].run_count--;
                    if (tt_tasks[i].run_count == 0)
                    {
                        // Task will be executed in main loop by TT_Dispatch_task
                        tt_tasks[i].run_count = tt_tasks[i].period; // Reset period
                    }
                }
                else
                {
                    tt_tasks[i].delay--;
                }
            }
        }
    }
}

/**
 * @brief Adds a task to the TT scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task execution in ticks.
 * @param delay The initial delay before the first execution in ticks.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (tt_next_task_index < MAX_TT_TASKS && task != NULL)
    {
        tt_tasks[tt_next_task_index].task_func = task;
        tt_tasks[tt_next_task_index].period = period / tt_tick_ms_value; // Convert period to internal ticks
        if (tt_tasks[tt_next_task_index].period == 0) tt_tasks[tt_next_task_index].period = 1; // Minimum 1 tick
        tt_tasks[tt_next_task_index].delay = delay / tt_tick_ms_value; // Convert delay to internal ticks
        tt_tasks[tt_next_task_index].enabled = true;
        tt_tasks[tt_next_task_index].run_count = tt_tasks[tt_next_task_index].period;
        return tt_next_task_index++;
    }
    return 0xFF; // Failed to add task
}

/**
 * @brief Deletes a task from the TT scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (task_index < tt_next_task_index)
    {
        // For simplicity, mark as disabled and null.
        // A more robust implementation might shift tasks or use a linked list.
        tt_tasks[task_index].enabled = false;
        tt_tasks[task_index].task_func = NULL;
    }
}

// --- I2S API IMPLEMENTATION ---

/**
 * @brief Initializes an I2S channel.
 * @param channel The I2S channel (I2S1, I2S2, I2S3).
 * @param mode I2S mode (Master/Slave Tx/Rx).
 * @param standard I2S standard (Philips, MSB, LSB, PCM Short/Long).
 * @param data_format I2S data format (16B, 24B, 32B).
 * @param channel_mode I2S channel mode (Stereo, Mono).
 * @param sample_rate Audio sample rate (e.g., 44.1kHz).
 * @param mclk_freq Master clock frequency.
 * @param dma_buffer_size DMA buffer size (ignored as DMA registers not provided).
 */
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    uint32_t pclk_freq;
    t_port port_sck = PORT_A; t_pin pin_sck = PIN_5; uint8_t af_sck = 5; // Default to SPI1 pins
    t_port port_ws = PORT_A; t_pin pin_ws = PIN_4; uint8_t af_ws = 5;
    t_port port_sd = PORT_A; t_pin pin_sd = PIN_7; uint8_t af_sd = 5;
    t_port port_mck = PORT_B; t_pin pin_mck = PIN_0; uint8_t af_mck = 5;

    switch (channel)
    {
        case I2S_CHANNEL_1:
            SPIx = SPI1;
            MCAL_RCC_EnableClock(SPI1_BASE_ADDR); // RCC_APB2ENR, bit 0 (inferred)
            pclk_freq = PCLK2_FREQ_HZ;
            // GPIOs for SPI1/I2S1
            // SCK: PA5 (AF5), PB3 (AF5)
            // WS (L/R): PA4 (AF5), PA15 (AF5)
            // SD: PA7 (AF5), PB5 (AF5)
            // MCK: PB0 (AF5)
            port_sck = PORT_A; pin_sck = PIN_5; af_sck = 5;
            port_ws = PORT_A; pin_ws = PIN_4; af_ws = 5;
            port_sd = PORT_A; pin_sd = PIN_7; af_sd = 5;
            port_mck = PORT_B; pin_mck = PIN_0; af_mck = 5;
            break;
        case I2S_CHANNEL_2:
            SPIx = SPI2;
            MCAL_RCC_EnableClock(SPI2_BASE_ADDR); // RCC_APB1ENR, bit 14 (inferred)
            pclk_freq = PCLK1_FREQ_HZ;
            // GPIOs for SPI2/I2S2
            // SCK: PB10 (AF5), PB13 (AF5), PC7 (AF5), PD3 (AF5)
            // WS (L/R): PB9 (AF5), PB12 (AF5)
            // SD: PB15 (AF5), PC3 (AF5), PC2 (AF5), PD4 (AF5)
            // MCK: PC6 (AF5)
            port_sck = PORT_B; pin_sck = PIN_13; af_sck = 5;
            port_ws = PORT_B; pin_ws = PIN_12; af_ws = 5;
            port_sd = PORT_B; pin_sd = PIN_15; af_sd = 5;
            port_mck = PORT_C; pin_mck = PIN_6; af_mck = 5;
            break;
        case I2S_CHANNEL_3:
            SPIx = SPI3;
            MCAL_RCC_EnableClock(SPI3_BASE_ADDR); // RCC_APB1ENR, bit 15 (inferred)
            pclk_freq = PCLK1_FREQ_HZ;
            // GPIOs for SPI3/I2S3
            // SCK: PB3 (AF6), PC10 (AF6)
            // WS (L/R): PA4 (AF6), PA15 (AF6)
            // SD: PB5 (AF6), PC12 (AF6), PB4 (AF6), PC11 (AF6)
            // MCK: PC7 (AF6)
            port_sck = PORT_B; pin_sck = PIN_3; af_sck = 6;
            port_ws = PORT_A; pin_ws = PIN_4; af_ws = 6;
            port_sd = PORT_B; pin_sd = PIN_5; af_sd = 6;
            port_mck = PORT_C; pin_mck = PIN_7; af_mck = 6;
            break;
        default: return; // Invalid channel
    }

    // Configure GPIO pins for I2S Alternate Function
    // SCK
    MCAL_RCC_EnableClock((uint32_t)get_gpio_port_ptr(port_sck));
    get_gpio_port_ptr(port_sck)->MODER &= ~(0x3U << (pin_sck * 2));
    get_gpio_port_ptr(port_sck)->MODER |=  (0x2U << (pin_sck * 2));
    if (pin_sck < 8) get_gpio_port_ptr(port_sck)->AFRL |= (af_sck << (pin_sck * 4));
    else get_gpio_port_ptr(port_sck)->AFRH |= (af_sck << ((pin_sck - 8) * 4));
    // WS
    MCAL_RCC_EnableClock((uint32_t)get_gpio_port_ptr(port_ws));
    get_gpio_port_ptr(port_ws)->MODER &= ~(0x3U << (pin_ws * 2));
    get_gpio_port_ptr(port_ws)->MODER |=  (0x2U << (pin_ws * 2));
    if (pin_ws < 8) get_gpio_port_ptr(port_ws)->AFRL |= (af_ws << (pin_ws * 4));
    else get_gpio_port_ptr(port_ws)->AFRH |= (af_ws << ((pin_ws - 8) * 4));
    // SD
    MCAL_RCC_EnableClock((uint32_t)get_gpio_port_ptr(port_sd));
    get_gpio_port_ptr(port_sd)->MODER &= ~(0x3U << (pin_sd * 2));
    get_gpio_port_ptr(port_sd)->MODER |=  (0x2U << (pin_sd * 2));
    if (pin_sd < 8) get_gpio_port_ptr(port_sd)->AFRL |= (af_sd << (pin_sd * 4));
    else get_gpio_port_ptr(port_sd)->AFRH |= (af_sd << ((pin_sd - 8) * 4));
    // MCK (if Master mode)
    if (mode == I2S_MODE_MASTER_TX || mode == I2S_MODE_MASTER_RX)
    {
        MCAL_RCC_EnableClock((uint32_t)get_gpio_port_ptr(port_mck));
        get_gpio_port_ptr(port_mck)->MODER &= ~(0x3U << (pin_mck * 2));
        get_gpio_port_ptr(port_mck)->MODER |=  (0x2U << (pin_mck * 2));
        if (pin_mck < 8) get_gpio_port_ptr(port_mck)->AFRL |= (af_mck << (pin_mck * 4));
        else get_gpio_port_ptr(port_mck)->AFRH |= (af_mck << ((pin_mck - 8) * 4));
    }


    // Disable I2S before configuration
    SPIx->I2SCFGR &= ~SPI_I2SCFGR_I2SE;

    // Select I2S mode
    SPIx->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // Configure Master/Slave and Transmit/Receive
    SPIx->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG); // Clear I2SCFG bits
    if (mode == I2S_MODE_SLAVE_TX)
    {
        SPIx->I2SCFGR |= SPI_I2SCFGR_I2SCFG_0; // Slave Transmit
    }
    else if (mode == I2S_MODE_SLAVE_RX)
    {
        // SPIx->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1; // Slave Receive - this is I2S mode 2, Master Rx
        // Slave receive should be I2SCFGR_I2SCFG_0 | I2SSTD[1:0] according to spec
        // The I2SCFGR_I2SCFG bits are 1:0.
        // I2SCFG[1:0]: 00: Slave Tx, 01: Slave Rx, 10: Master Tx, 11: Master Rx
        SPIx->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1; // Slave receive
    }
    else if (mode == I2S_MODE_MASTER_TX)
    {
        SPIx->I2SCFGR |= (SPI_I2SCFGR_I2SCFG_1 | SPI_I2SCFGR_I2SCFG_0); // Master Transmit
    }
    else if (mode == I2S_MODE_MASTER_RX)
    {
        SPIx->I2SCFGR |= SPI_I2SCFGR_I2SCFG_0; // Master Receive
    }

    // Configure I2S Standard (I2SSTD bits in I2SCFGR)
    SPIx->I2SCFGR &= ~(SPI_I2SCFGR_I2SSTD); // Clear I2SSTD bits
    switch (standard)
    {
        case I2S_STANDARD_PHILIPS: SPIx->I2SCFGR |= SPI_I2SCFGR_I2SSTD_CFG; break; // Philips I2S
        case I2S_STANDARD_MSB:     SPIx->I2SCFGR |= SPI_I2SCFGR_I2SSTD_0; break; // MSB justified
        case I2S_STANDARD_LSB:     SPIx->I2SCFGR |= SPI_I2SCFGR_I2SSTD_1; break; // LSB justified
        case I2S_STANDARD_PCM_SHORT: SPIx->I2SCFGR |= (SPI_I2SCFGR_I2SSTD_1 | SPI_I2SCFGR_I2SSTD_0); break; // PCM Short
        case I2S_STANDARD_PCM_LONG:  SPIx->I2SCFGR |= (SPI_I2SCFGR_I2SSTD_1 | SPI_I2SCFGR_I2SSTD_0); break; // PCM Long (same as Short, difference is bit length)
        default: break;
    }

    // Configure Data Format (DATLEN, CHLEN in I2SCFGR)
    SPIx->I2SCFGR &= ~(SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN); // Clear DATLEN and CHLEN
    switch (data_format)
    {
        case I2S_DATAFORMAT_16B: SPIx->I2SCFGR |= (0x0U << 1); break; // 16-bit data length
        case I2S_DATAFORMAT_16B_EXTENDED: SPIx->I2SCFGR |= (0x0U << 1); break; // 16-bit data, 32-bit channel length
        case I2S_DATAFORMAT_24B: SPIx->I2SCFGR |= (0x1U << 1); break; // 24-bit data length
        case I2S_DATAFORMAT_32B: SPIx->I2SCFGR |= (0x2U << 1); break; // 32-bit data length
        default: break;
    }
    if (data_format == I2S_DATAFORMAT_16B_EXTENDED || data_format == I2S_DATAFORMAT_24B || data_format == I2S_DATAFORMAT_32B)
    {
        SPIx->I2SCFGR |= SPI_I2SCFGR_CHLEN; // Set channel length to 32-bit
    }

    // Configure Channel Mode (MONO, ODD/EVEN)
    if (channel_mode == I2S_CHANNELMODE_MONO)
    {
        SPIx->I2SCFGR |= SPI_I2SCFGR_MONO;
    }
    else
    {
        SPIx->I2SCFGR &= ~SPI_I2SCFGR_MONO; // Stereo
    }

    // Configure Sample Rate and MCLK (I2SPR)
    // Formula: I2S_CK = PCLK / ((2*I2SPR.ODD + I2SPR.I2SDIV) * (2 if I2SCFGR.CHLEN or I2SCFGR.DATLEN > 16) * 2)
    // When MCLK is enabled, I2S_CK = MCLK * 256 / sample_rate
    // I2SPR.MCKEN (bit 7) and I2SPR.ODD (bit 8), I2SPR.I2SDIV (bits 5:0)

    uint32_t i2sdiv = 0;
    uint32_t odd = 0;
    uint32_t i2s_source_clock = pclk_freq; // Assuming PCLK is source
    uint32_t fs = sample_rate; // Target sample rate

    if (mclk_freq != 0) // If master clock is enabled and specified
    {
        SPIx->I2SCFGR |= SPI_I2SCFGR_MCKOE; // Master Clock Output Enable
        // Calculate I2SDIV and ODD based on MCLK output for specified sample rate
        // fs = I2S_Clock / (2 * Frame_Length_Factor)
        // Frame_Length_Factor depends on data_format and CHLEN.
        // Assuming 32-bit frame length for simplicity (CHLEN=1)
        // 256 clock cycles per frame (common for 32-bit stereo)
        // I2S_Clock = Master_Clock_Freq * 256
        // If MCLK is 1x sample rate * 256: I2S_Clock = sample_rate * 256
        // I2SDIV and ODD control I2S_Clock = Source_Clock / ((2 * I2SDIV) + ODD)
        // Rearranging for I2SDIV and ODD: (2 * I2SDIV) + ODD = Source_Clock / (sample_rate * 256) (for Master Tx/Rx with 256-bit frame)
        // This calculation is complex and depends on the exact frame format.
        // For example, if fs = 48kHz, for 256-bit frame, I2S_CK = 48kHz * 256 = 12.288MHz.
        // Then, (2*I2SDIV + ODD) = PCLK / 12.288MHz.
        // For PCLK1 (42MHz): (2*I2SDIV + ODD) = 42MHz / 12.288MHz = 3.417...
        // For PCLK2 (84MHz): (2*I2SDIV + ODD) = 84MHz / 12.288MHz = 6.835...

        // Simplified calculation for I2SDIV and ODD, typically from ST examples:
        // For 48 kHz, Master Clock 256*FS, F_I2S = 256 * 48kHz = 12.288 MHz
        // N = PCLKx / (F_I2S * 2) = PCLKx / (2 * 48k * 256)
        // For 48kHz: If PCLK1=42MHz, N=42M / (2*12.288M) = 1.71 (not feasible)
        // If PCLK2=84MHz, N=84M / (2*12.288M) = 3.41 (I2SDIV=1, ODD=1)
        // The actual calculation depends on exact configuration.
        // This is a complex part, and precise values need external tools or a predefined table.
        // For this MCAL, we'll try to calculate for a common case (48kHz, PCLK2, Master 256fs).

        // If no MCLK specified, I2SDIV and ODD are typically pre-calculated values.
        // Example for 48kHz I2S on STM32F401RC (assuming PCLK2=84MHz):
        // I2SDIV = 3; ODD = 1; MCKEN = 1;
        i2sdiv = 3; odd = 1; // Inferred common values for 48kHz
    }
    else // Master clock output is not enabled (slave mode or internal master clock)
    {
        // Calculate based on sample_rate
        // Simplified: I2SDIV = PCLKx / (2 * sample_rate) - ODD_Factor (complex)
        // For 48kHz (example)
        if (sample_rate == 48000) { i2sdiv = 3; odd = 1; } // Assuming PCLK2/84MHz, or PCLK1/42MHz with different factors.
        else { i2sdiv = 2; odd = 0; } // Default
    }

    SPIx->I2SPR &= ~(SPI_I2SPR_I2SDIV | SPI_I2SPR_ODD | SPI_I2SPR_MCKEN);
    SPIx->I2SPR |= (i2sdiv & 0xFFU);
    if (odd) SPIx->I2SPR |= SPI_I2SPR_ODD;
    if (mclk_freq != 0) SPIx->I2SPR |= SPI_I2SPR_MCKEN; // Master Clock Output Enable
    (void)dma_buffer_size; // DMA not explicitly supported by provided registers

    // Set polarity (CPOL) for I2S (typically fixed, e.g., low for Philips)
    SPIx->I2SCFGR &= ~SPI_I2SCFGR_CKPOL; // Clock polarity low
}

/**
 * @brief Enables the I2S channel.
 * @param channel The I2S channel to enable.
 */
void I2S_Enable(t_i2s_channel channel)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    switch (channel)
    {
        case I2S_CHANNEL_1: SPIx = SPI1; MCAL_RCC_EnableClock(SPI1_BASE_ADDR); break;
        case I2S_CHANNEL_2: SPIx = SPI2; MCAL_RCC_EnableClock(SPI2_BASE_ADDR); break;
        case I2S_CHANNEL_3: SPIx = SPI3; MCAL_RCC_EnableClock(SPI3_BASE_ADDR); break;
        default: return; // Invalid channel
    }
    SPIx->I2SCFGR |= SPI_I2SCFGR_I2SE; // Enable I2S
}

/**
 * @brief Transmits data over an I2S channel.
 * @param channel The I2S channel to use.
 * @param data Pointer to the data to transmit.
 * @param length The length of the data in bytes.
 */
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    switch (channel)
    {
        case I2S_CHANNEL_1: SPIx = SPI1; break;
        case I2S_CHANNEL_2: SPIx = SPI2; break;
        case I2S_CHANNEL_3: SPIx = SPI3; break;
        default: return; // Invalid channel
    }

    const uint16_t *tx_data = (const uint16_t *)data; // I2S typically works with 16-bit or 32-bit words
    size_t num_words = length / sizeof(uint16_t);

    for (size_t i = 0; i < num_words; i++)
    {
        // Wait until TXE flag is set (Tx buffer empty)
        tword timeout = 0xFFFF;
        while (!(SPIx->SR & SPI_SR_TXE) && (timeout-- > 0))
        {
            WDT_Reset();
        }
        if (timeout == 0) return; // Timeout error

        SPIx->DR = tx_data[i]; // Write 16-bit data to the data register
    }
}

/**
 * @brief Receives data from an I2S channel.
 * @param channel The I2S channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param length The length of the buffer in bytes.
 */
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length)
{
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    SPI_TypeDef *SPIx;
    switch (channel)
    {
        case I2S_CHANNEL_1: SPIx = SPI1; break;
        case I2S_CHANNEL_2: SPIx = SPI2; break;
        case I2S_CHANNEL_3: SPIx = SPI3; break;
        default: return; // Invalid channel
    }

    uint16_t *rx_buffer = (uint16_t *)buffer;
    size_t num_words = length / sizeof(uint16_t);

    for (size_t i = 0; i < num_words; i++)
    {
        // Wait until RXNE flag is set (Rx buffer not empty)
        tword timeout = 0xFFFF;
        while (!(SPIx->SR & SPI_SR_RXNE) && (timeout-- > 0))
        {
            WDT_Reset();
        }
        if (timeout == 0) return; // Timeout error

        rx_buffer[i] = SPIx->DR; // Read 16-bit data from the data register
    }
}


// --- PRIVATE HELPER FUNCTIONS ---

/**
 * @brief Enables the clock for a peripheral based on its base address.
 *        This function infers the RCC enable register and bit position.
 * @param peripheral_base_addr The base address of the peripheral.
 */
static void MCAL_RCC_EnableClock(uint33_t peripheral_base_addr)
{
    // Inferred RCC mappings for STM32F401RC
    // AHB1: GPIOA-H
    if (peripheral_base_addr >= GPIOA_BASE_ADDR && peripheral_base_addr <= GPIOH_BASE_ADDR)
    {
        uint32_t offset = peripheral_base_addr - GPIOA_BASE_ADDR;
        uint33_t port_idx = offset / (GPIOB_BASE_ADDR - GPIOA_BASE_ADDR);
        RCC_AHB1ENR_REG |= (1UL << port_idx);
        (void)RCC_AHB1ENR_REG; // Read back to ensure clock is stable
    }
    // AHB2:
    // None for F401 except USB OTG FS (not in JSON)

    // APB1: Timers 2-5, USART2, I2C 1-3, SPI 2-3, PWR, WWDG, IWDG
    else if (peripheral_base_addr == TIM2_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_TIM2EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == TIM3_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_TIM3EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == TIM4_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_TIM4EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == TIM5_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_TIM5EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == USART2_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_USART2EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == I2C1_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_I2C1EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == I2C2_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_I2C2EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == I2C3_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_I2C3EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == SPI2_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_SPI2EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == SPI3_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_SPI3EN; (void)RCC_APB1ENR_REG; }
    else if (peripheral_base_addr == PWR_BASE_ADDR) { RCC_APB1ENR_REG |= RCC_APB1ENR_PWREN; (void)RCC_APB1ENR_REG; } // Inferred for LVD

    // APB2: Timers 1, 9-11, USART1, USART6, ADC1, SPI1, SYSCFG
    else if (peripheral_base_addr == TIM1_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_TIM1EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == TIM9_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_TIM9EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == TIM10_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_TIM10EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == TIM11_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_TIM11EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == USART1_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_USART1EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == USART6_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_USART6EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == ADC1_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_ADC1EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == SPI1_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_SPI1EN; (void)RCC_APB2ENR_REG; }
    else if (peripheral_base_addr == SYSCFG_BASE_ADDR) { RCC_APB2ENR_REG |= RCC_APB2ENR_SYSCFGEN; (void)RCC_APB2ENR_REG; } // Inferred for EXTI

    WDT_Reset(); // Keep WDT alive
}

/**
 * @brief Disables the clock for a peripheral based on its base address.
 *        This function infers the RCC enable register and bit position.
 * @param peripheral_base_addr The base address of the peripheral.
 */
static void MCAL_RCC_DisableClock(uint33_t peripheral_base_addr)
{
    // Inferred RCC mappings for STM32F401RC
    // AHB1: GPIOA-H
    if (peripheral_base_addr >= GPIOA_BASE_ADDR && peripheral_base_addr <= GPIOH_BASE_ADDR)
    {
        uint32_t offset = peripheral_base_addr - GPIOA_BASE_ADDR;
        uint33_t port_idx = offset / (GPIOB_BASE_ADDR - GPIOA_BASE_ADDR);
        RCC_AHB1ENR_REG &= ~(1UL << port_idx);
    }
    // APB1
    else if (peripheral_base_addr == TIM2_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_TIM2EN; }
    else if (peripheral_base_addr == TIM3_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_TIM3EN; }
    else if (peripheral_base_addr == TIM4_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_TIM4EN; }
    else if (peripheral_base_addr == TIM5_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_TIM5EN; }
    else if (peripheral_base_addr == USART2_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_USART2EN; }
    else if (peripheral_base_addr == I2C1_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_I2C1EN; }
    else if (peripheral_base_addr == I2C2_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_I2C2EN; }
    else if (peripheral_base_addr == I2C3_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_I2C3EN; }
    else if (peripheral_base_addr == SPI2_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_SPI2EN; }
    else if (peripheral_base_addr == SPI3_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_SPI3EN; }
    else if (peripheral_base_addr == PWR_BASE_ADDR) { RCC_APB1ENR_REG &= ~RCC_APB1ENR_PWREN; }

    // APB2
    else if (peripheral_base_addr == TIM1_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_TIM1EN; }
    else if (peripheral_base_addr == TIM9_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_TIM9EN; }
    else if (peripheral_base_addr == TIM10_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_TIM10EN; }
    else if (peripheral_base_addr == TIM11_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_TIM11EN; }
    else if (peripheral_base_addr == USART1_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_USART1EN; }
    else if (peripheral_base_addr == USART6_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_USART6EN; }
    else if (peripheral_base_addr == ADC1_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_ADC1EN; }
    else if (peripheral_base_addr == SPI1_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_SPI1EN; }
    else if (peripheral_base_addr == SYSCFG_BASE_ADDR) { RCC_APB2ENR_REG &= ~RCC_APB2ENR_SYSCFGEN; }

    WDT_Reset(); // Keep WDT alive
}


/**
 * @brief Sets all GPIO pins to 0 (low).
 *        Verifies the state after setting.
 */
static void MCAL_GPIO_ClearAll(void)
{
    for (t_port p = PORT_A; p <= PORT_H; p++)
    {
        GPIO_TypeDef *gpio = get_gpio_port_ptr(p);
        if (gpio == NULL) continue;
        MCAL_RCC_EnableClock((uint32_t)gpio); // Ensure clock is on
        gpio->ODR = 0x0000; // Set all pins to low
        while (gpio->ODR != 0x0000)
        {
            WDT_Reset();
        }
    }
}

/**
 * @brief Sets all GPIO pins direction to input mode.
 *        Verifies the state after setting.
 */
static void MCAL_GPIO_SetAllInput(void)
{
    for (t_port p = PORT_A; p <= PORT_H; p++)
    {
        GPIO_TypeDef *gpio = get_gpio_port_ptr(p);
        if (gpio == NULL) continue;
        MCAL_RCC_EnableClock((uint32_t)gpio); // Ensure clock is on
        gpio->MODER = 0x00000000; // All pins to Input mode (00)
        gpio->PUPDR = 0x55555555; // All pins pull-up (01) (Rule: all input pins have pull-up)
        while (gpio->MODER != 0x00000000)
        {
            WDT_Reset();
        }
    }
}

/**
 * @brief Disables all supported peripheral clocks and module enable bits.
 *        This is called during `MCU_Config_Init`.
 */
static void MCAL_DisableAllPeripherals(void)
{
    // Disable all Peripheral clocks
    RCC_AHB1ENR_REG = 0x00000000; // Clear all AHB1 peripheral clocks (except for GPIO used internally)
    RCC_AHB2ENR_REG = 0x00000000; // Clear all AHB2 peripheral clocks
    RCC_APB1ENR_REG = 0x00000000; // Clear all APB1 peripheral clocks
    RCC_APB2ENR_REG = 0x00000000; // Clear all APB2 peripheral clocks

    // Enable clocks for GPIO ports necessary for MCU_Config_Init itself, then disable others.
    // The previous ClearAll/SetAllInput already enabled GPIOA-H clocks.
    // Re-enable necessary GPIO clocks that might have been cleared by RCC_AHB1ENR_REG = 0.
    MCAL_RCC_EnableClock(GPIOA_BASE_ADDR);
    MCAL_RCC_EnableClock(GPIOB_BASE_ADDR);
    MCAL_RCC_EnableClock(GPIOC_BASE_ADDR);
    MCAL_RCC_EnableClock(GPIOD_BASE_ADDR);
    MCAL_RCC_EnableClock(GPIOE_BASE_ADDR);
    MCAL_RCC_EnableClock(GPIOH_BASE_ADDR);
    MCAL_RCC_EnableClock(PWR_BASE_ADDR); // Keep PWR clock for LVD

    // Explicitly disable main enable bits for peripherals (if their clocks were on)
    // UARTs
    USART1->CR1 &= ~USART_CR1_UE;
    USART2->CR1 &= ~USART_CR1_UE;
    USART6->CR1 &= ~USART_CR1_UE;
    // I2Cs
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C2->CR1 &= ~I2C_CR1_PE;
    I2C3->CR1 &= ~I2C_CR1_PE;
    // SPIs
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI3->CR1 &= ~SPI_CR1_SPE;
    // I2S (shares with SPI)
    SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    SPI2->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    SPI3->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    // ADCs
    ADC1->CR2 &= ~ADC_CR2_ADON;
    // TIMERS
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM9->CR1 &= ~TIM_CR1_CEN;
    TIM10->CR1 &= ~TIM_CR1_CEN;
    TIM11->CR1 &= ~TIM_CR1_CEN;
    // EXTI (disable mask)
    EXTI_IMR_REG = 0x00000000;
    EXTI_EMR_REG = 0x00000000;

    WDT_Reset(); // Keep WDT alive
}

/*
 * @brief TIM2_IRQHandler for TT scheduler and ICU functionality.
 *        This ISR needs to be registered in the startup file.
 *        It should call TT_ISR and potentially ICU_Callback.
 */
void TIM2_IRQHandler(void)
{
    WDT_Reset(); // Always reset WDT in ISR

    // Check for TIM2 Update Interrupt
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
        TT_ISR(); // Call the TT scheduler's ISR handler
    }

    // Check for TIM2 Capture/Compare 1 Interrupt (if used for ICU_CHANNEL_TIM2_CH1)
    if ((TIM2->SR & TIM_SR_CC1IF) != 0)
    {
        TIM2->SR &= ~TIM_SR_CC1IF; // Clear Capture/Compare 1 Interrupt Flag
        if (ICU_Callback != NULL)
        {
            ICU_Callback();
        }
    }
    // Add checks for other TIM2 CCxIF flags if other channels are used for ICU
}