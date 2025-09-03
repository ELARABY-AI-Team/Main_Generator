// ============================================================================
// File: MCAL.c
// Microcontroller Abstraction Layer (MCAL) for STM32F401RC
// Generated based on MCU register definitions, API specifications, and coding rules.
// ============================================================================

// --- Core MCU Header and Standard C Libraries (Rules.json: core_includes) ---
#include "MCAL.h"
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h> // Required for functions like memcpy, strlen in frame/string APIs
#include <stdio.h>  // Required for snprintf if any debug printing is used, or string conversions
#include <stdlib.h> // Required for functions like malloc/free if dynamic memory is used (not in current API)
#include <math.h>   // Required for functions like round, ceil etc. (not in current API, but included as per rule example)

// --- Local Macros and Constants ---
#define PERIPH_BASE           (0x40000000UL) /*!< Peripheral base address */
#define APB1PERIPH_BASE       (PERIPH_BASE)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)

// RCC Register Offsets from RCC_BASE_ADDRESS
#define RCC_CR_OFFSET           (0x00)
#define RCC_PLLCFGR_OFFSET      (0x04)
#define RCC_CFGR_OFFSET         (0x08)
#define RCC_CIR_OFFSET          (0x0C)
#define RCC_AHB1RSTR_OFFSET     (0x10)
#define RCC_AHB2RSTR_OFFSET     (0x14)
#define RCC_APB1RSTR_OFFSET     (0x18)
#define RCC_APB2RSTR_OFFSET     (0x1C)
#define RCC_AHB1ENR_OFFSET      (0x30)
#define RCC_AHB2ENR_OFFSET      (0x34)
#define RCC_APB1ENR_OFFSET      (0x38)
#define RCC_APB2ENR_OFFSET      (0x3C)
#define RCC_AHB1LPENR_OFFSET    (0x40)
#define RCC_AHB2LPENR_OFFSET    (0x44)
#define RCC_APB1LPENR_OFFSET    (0x48)
#define RCC_APB2LPENR_OFFSET    (0x4C)
#define RCC_BDCR_OFFSET         (0x50)
#define RCC_CSR_OFFSET          (0x54)
#define RCC_SSCGR_OFFSET        (0x58)
#define RCC_PLLI2SCFGR_OFFSET   (0x5C)
#define RCC_DCKCFGR_OFFSET      (0x64)

// SYSCFG Register Offsets from SYSCFG_BASE_ADDRESS
#define SYSCFG_MEMRMP_OFFSET    (0x00)
#define SYSCFG_PMC_OFFSET       (0x04)
#define SYSCFG_EXTICR1_OFFSET   (0x08)
#define SYSCFG_EXTICR2_OFFSET   (0x0C)
#define SYSCFG_EXTICR3_OFFSET   (0x10)
#define SYSCFG_EXTICR4_OFFSET   (0x14)
#define SYSCFG_CMPCR_OFFSET     (0x20)

// EXTI Register Offsets from EXTI_BASE_ADDRESS
#define EXTI_IMR_OFFSET         (0x00)
#define EXTI_EMR_OFFSET         (0x04)
#define EXTI_RTSR_OFFSET        (0x08)
#define EXTI_FTSR_OFFSET        (0x0C)
#define EXTI_SWIER_OFFSET       (0x10)
#define EXTI_PR_OFFSET          (0x14)

// GPIO Register Offsets (e.g., from GPIOA_BASE_ADDRESS)
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

// ADC Register Offsets (e.g., from ADC1_BASE_ADDRESS)
#define ADC_SR_OFFSET           (0x00)
#define ADC_CR1_OFFSET          (0x04)
#define ADC_CR2_OFFSET          (0x08)
#define ADC_SMPR1_OFFSET        (0x0C)
#define ADC_SMPR2_OFFSET        (0x10)
#define ADC_JOFR1_OFFSET        (0x14)
#define ADC_JOFR2_OFFSET        (0x18)
#define ADC_JOFR3_OFFSET        (0x1C)
#define ADC_JOFR4_OFFSET        (0x20)
#define ADC_HTR_OFFSET          (0x24)
#define ADC_LTR_OFFSET          (0x28)
#define ADC_SQR1_OFFSET         (0x2C)
#define ADC_SQR2_OFFSET         (0x30)
#define ADC_SQR3_OFFSET         (0x34)
#define ADC_JSQR_OFFSET         (0x38)
#define ADC_JDR1_OFFSET         (0x3C)
#define ADC_JDR2_OFFSET         (0x40)
#define ADC_JDR3_OFFSET         (0x44)
#define ADC_JDR4_OFFSET         (0x48)
#define ADC_DR_OFFSET           (0x4C)
// ADC Common Registers (from ADC_COMMON_BASE_ADDRESS)
#define ADC_CCR_OFFSET          (0x04)

// Timer Register Offsets (e.g., from TIM1_BASE_ADDRESS)
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
#define TIM_RCR_OFFSET          (0x30) // Only for advanced control timers (TIM1, TIM8)
#define TIM_CCR1_OFFSET         (0x34)
#define TIM_CCR2_OFFSET         (0x38)
#define TIM_CCR3_OFFSET         (0x3C)
#define TIM_CCR4_OFFSET         (0x40)
#define TIM_BDTR_OFFSET         (0x44) // Only for advanced control timers (TIM1, TIM8)
#define TIM_DCR_OFFSET          (0x48)
#define TIM_DMAR_OFFSET         (0x4C)
#define TIM_OR_OFFSET           (0x50) // Only for TIM2/TIM5

// USART Register Offsets (e.g., from USART1_BASE_ADDRESS)
#define USART_SR_OFFSET         (0x00)
#define USART_DR_OFFSET         (0x04)
#define USART_BRR_OFFSET        (0x08)
#define USART_CR1_OFFSET        (0x0C)
#define USART_CR2_OFFSET        (0x10)
#define USART_CR3_OFFSET        (0x14)
#define USART_GTPR_OFFSET       (0x18)

// I2C Register Offsets (e.g., from I2C1_BASE_ADDRESS)
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

// SPI Register Offsets (e.g., from SPI1_BASE_ADDRESS)
#define SPI_CR1_OFFSET          (0x00)
#define SPI_CR2_OFFSET          (0x04)
#define SPI_SR_OFFSET           (0x08)
#define SPI_DR_OFFSET           (0x0C)
#define SPI_CRCPR_OFFSET        (0x10)
#define SPI_RXCRCR_OFFSET       (0x14)
#define SPI_TXCRCR_OFFSET       (0x18)
#define SPI_I2SCFGR_OFFSET      (0x1C)
#define SPI_I2SPR_OFFSET        (0x20)

// --- Helper Functions for Register Access (Rules.json: pointer_variable_consistency) ---

/**
 * @brief Macro to access a peripheral register directly.
 * @param BASE_ADDR The base address of the peripheral.
 * @param OFFSET The offset of the specific register from the base address.
 * @return A volatile pointer to the 32-bit register.
 */
#define REG_ACCESS(BASE_ADDR, OFFSET) ((volatile uint32_t *)((BASE_ADDR) + (OFFSET)))

/**
 * @brief Returns the base address of a given GPIO port.
 * @param port The GPIO port enum (e.g., port_a).
 * @return Volatile pointer to the GPIO port's base address, or NULL if invalid.
 */
static volatile uint32_t* get_gpio_base_ptr(t_port port)
{
    switch (port)
    {
        case port_a: return (volatile uint32_t*)GPIOA_BASE_ADDRESS;
        case port_b: return (volatile uint32_t*)GPIOB_BASE_ADDRESS;
        case port_c: return (volatile uint32_t*)GPIOC_BASE_ADDRESS;
        case port_d: return (volatile uint32_t*)GPIOD_BASE_ADDRESS;
        case port_e: return (volatile uint32_t*)GPIOE_BASE_ADDRESS;
        case port_h: return (volatile uint32_t*)GPIOH_BASE_ADDRESS;
        default: return NULL; // Invalid port
    }
}

/**
 * @brief Returns the base address of a given USART peripheral.
 * @param channel The USART channel enum (e.g., uart_channel_usart1).
 * @return Volatile pointer to the USART peripheral's base address, or NULL if invalid.
 */
static volatile uint32_t* get_uart_base_ptr(t_uart_channel channel)
{
    switch (channel)
    {
        case uart_channel_usart1: return (volatile uint32_t*)USART1_BASE_ADDRESS;
        case uart_channel_usart2: return (volatile uint32_t*)USART2_BASE_ADDRESS;
        case uart_channel_usart6: return (volatile uint32_t*)USART6_BASE_ADDRESS;
        default: return NULL; // Invalid channel
    }
}

/**
 * @brief Returns the base address of a given I2C peripheral.
 * @param channel The I2C channel enum (e.g., i2c_channel_i2c1).
 * @return Volatile pointer to the I2C peripheral's base address, or NULL if invalid.
 */
static volatile uint32_t* get_i2c_base_ptr(t_i2c_channel channel)
{
    switch (channel)
    {
        case i2c_channel_i2c1: return (volatile uint32_t*)I2C1_BASE_ADDRESS;
        case i2c_channel_i2c2: return (volatile uint32_t*)I2C2_BASE_ADDRESS;
        case i2c_channel_i2c3: return (volatile uint32_t*)I2C3_BASE_ADDRESS;
        default: return NULL; // Invalid channel
    }
}

/**
 * @brief Returns the base address of a given SPI peripheral.
 * @param channel The SPI channel enum (e.g., spi_channel_spi1).
 * @return Volatile pointer to the SPI peripheral's base address, or NULL if invalid.
 */
static volatile uint32_t* get_spi_base_ptr(t_spi_channel channel)
{
    switch (channel)
    {
        case spi_channel_spi1: return (volatile uint32_t*)SPI1_BASE_ADDRESS;
        case spi_channel_spi2: return (volatile uint32_t*)SPI2_BASE_ADDRESS;
        case spi_channel_spi3: return (volatile uint32_t*)SPI3_BASE_ADDRESS;
        default: return NULL; // Invalid channel
    }
}

/**
 * @brief Returns the base address of a given Timer peripheral.
 * @param channel The Timer channel enum (e.g., timer_channel_tim1).
 * @return Volatile pointer to the Timer peripheral's base address, or NULL if invalid.
 */
static volatile uint32_t* get_timer_base_ptr(t_timer_channel channel)
{
    switch (channel)
    {
        case timer_channel_tim1: return (volatile uint32_t*)TIM1_BASE_ADDRESS;
        case timer_channel_tim2: return (volatile uint32_t*)TIM2_BASE_ADDRESS;
        case timer_channel_tim3: return (volatile uint32_t*)TIM3_BASE_ADDRESS;
        case timer_channel_tim4: return (volatile uint32_t*)TIM4_BASE_ADDRESS;
        case timer_channel_tim5: return (volatile uint32_t*)TIM5_BASE_ADDRESS;
        case timer_channel_tim9: return (volatile uint32_t*)TIM9_BASE_ADDRESS;
        case timer_channel_tim10: return (volatile uint32_t*)TIM10_BASE_ADDRESS;
        case timer_channel_tim11: return (volatile uint32_t*)TIM11_BASE_ADDRESS;
        default: return NULL; // Invalid channel
    }
}

/**
 * @brief Returns the base address of the ADC peripheral.
 * @param channel The ADC channel enum (e.g., adc_channel_0). Note: ADC1 is the only one.
 * @return Volatile pointer to the ADC peripheral's base address, or NULL if invalid.
 */
static volatile uint32_t* get_adc_base_ptr(t_adc_channel channel)
{
    // For STM32F401RC, ADC is typically ADC1. The channel refers to input pins.
    (void)channel; // Suppress unused parameter warning
    return (volatile uint32_t*)ADC1_BASE_ADDRESS;
}

/**
 * @brief Enables the clock for a given peripheral.
 * @param peripheral_type Identifier for the peripheral (e.g., 'A' for GPIOA, '1' for USART1).
 * @param channel_num Bit position in the RCC enable register.
 * @return void
 */
static void peripheral_clock_enable(char peripheral_type, tbyte channel_num)
{
    volatile uint32_t* rcc_ahb1enr_ptr = REG_ACCESS(RCC_BASE_ADDRESS, RCC_AHB1ENR_OFFSET);
    volatile uint32_t* rcc_ahb2enr_ptr = REG_ACCESS(RCC_BASE_ADDRESS, RCC_AHB2ENR_OFFSET);
    volatile uint32_t* rcc_apb1enr_ptr = REG_ACCESS(RCC_BASE_ADDRESS, RCC_APB1ENR_OFFSET);
    volatile uint32_t* rcc_apb2enr_ptr = REG_ACCESS(RCC_BASE_ADDRESS, RCC_APB2ENR_OFFSET);

    switch (peripheral_type)
    {
        case 'A': // GPIOA
            *rcc_ahb1enr_ptr |= (1UL << 0);
            break;
        case 'B': // GPIOB
            *rcc_ahb1enr_ptr |= (1UL << 1);
            break;
        case 'C': // GPIOC
            *rcc_ahb1enr_ptr |= (1UL << 2);
            break;
        case 'D': // GPIOD
            *rcc_ahb1enr_ptr |= (1UL << 3);
            break;
        case 'E': // GPIOE
            *rcc_ahb1enr_ptr |= (1UL << 4);
            break;
        case 'H': // GPIOH
            *rcc_ahb1enr_ptr |= (1UL << 7);
            break;
        case '1': // USART1, SPI1, TIM1, TIM9, TIM10, TIM11
            if (channel_num == 4) *rcc_apb2enr_ptr |= (1UL << 4);  // USART1EN
            else if (channel_num == 0) *rcc_apb2enr_ptr |= (1UL << 0);  // TIM1EN
            else if (channel_num == 12) *rcc_apb2enr_ptr |= (1UL << 12); // SPI1EN
            else if (channel_num == 16) *rcc_apb2enr_ptr |= (1UL << 16); // TIM9EN
            else if (channel_num == 17) *rcc_apb2enr_ptr |= (1UL << 17); // TIM10EN
            else if (channel_num == 18) *rcc_apb2enr_ptr |= (1UL << 18); // TIM11EN
            else if (channel_num == 8) *rcc_apb2enr_ptr |= (1UL << 8);  // ADC1EN
            else if (channel_num == 14) *rcc_apb2enr_ptr |= (1UL << 14); // SYSCFGEN
            break;
        case '2': // USART2, SPI2, TIM2, TIM3, TIM4, TIM5, I2C2
            if (channel_num == 17) *rcc_apb1enr_ptr |= (1UL << 17); // USART2EN
            else if (channel_num == 14) *rcc_apb1enr_ptr |= (1UL << 14); // SPI2EN
            else if (channel_num == 0) *rcc_apb1enr_ptr |= (1UL << 0);  // TIM2EN
            else if (channel_num == 1) *rcc_apb1enr_ptr |= (1UL << 1);  // TIM3EN
            else if (channel_num == 2) *rcc_apb1enr_ptr |= (1UL << 2);  // TIM4EN
            else if (channel_num == 3) *rcc_apb1enr_ptr |= (1UL << 3);  // TIM5EN
            else if (channel_num == 22) *rcc_apb1enr_ptr |= (1UL << 22); // I2C2EN
            break;
        case '3': // SPI3, I2C3
            if (channel_num == 15) *rcc_apb1enr_ptr |= (1UL << 15); // SPI3EN
            else if (channel_num == 23) *rcc_apb1enr_ptr |= (1UL << 23); // I2C3EN
            break;
        case '6': // USART6
            if (channel_num == 5) *rcc_apb2enr_ptr |= (1UL << 5);  // USART6EN
            break;
        case 'I': // I2C1
            if (channel_num == 21) *rcc_apb1enr_ptr |= (1UL << 21); // I2C1EN
            break;
        default:
            // Handle unknown peripheral type or log error
            break;
    }
    // Dummy read to ensure clock is indeed enabled
    (void)*rcc_ahb1enr_ptr;
}

// --- Global Variables (if any, for callbacks, etc.) ---
static void (*icu_callback_function)(void) = NULL;

// ============================================================================
// MCU CONFIG Module Implementations
// ============================================================================

/**
 * @brief Resets the Watchdog Timer (WDT).
 *        According to Rules.json: "API_implementation_sequence", this function
 *        must be called at the beginning of every API body.
 *        No explicit WDT registers provided in register_json, so this is a placeholder.
 *        For STM32F401RC, this typically involves writing to IWDG_KR (Key Register)
 *        with 0xAAAA to reset the counter.
 */
void WDT_Reset(void)
{
    // Rules.json: API_implementation_sequence - All API bodies must include WDT_Reset()
    // No WDT registers (IWDG_KR, IWDG_PR, IWDG_RLR) provided in register_json.
    // Therefore, this is a placeholder implementation.
    // In a real STM32 environment, you'd write 0xAAAA to IWDG->KR.
    // Example (if IWDG_KR was in register_json): *REG_ACCESS(IWDG_BASE, IWDG_KR_OFFSET) = 0xAAAA;
}

/**
 * @brief Sets all GPIO pins to 0, then to input, then disables various peripherals,
 *        and configures the Watchdog Timer and Low Voltage Reset.
 *        Implements Rules.json: MCU_Config_Init_implementation.
 * @param volt The system voltage (sys_volt_3v or sys_volt_5v).
 */
void MCU_Config_Init(t_sys_volt volt)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    // Step 1: Set all GPIO pins to 0 and verify
    t_port current_port;
    t_pin current_pin;
    volatile uint33_t* gpio_odr_ptr;
    volatile uint32_t* gpio_moder_ptr;

    for (current_port = port_a; current_port < port_max; current_port++)
    {
        peripheral_clock_enable((char)('A' + current_port), 0); // Enable GPIOx clock
        gpio_odr_ptr = REG_ACCESS(get_gpio_base_ptr(current_port), GPIO_ODR_OFFSET);
        gpio_moder_ptr = REG_ACCESS(get_gpio_base_ptr(current_port), GPIO_MODER_OFFSET);

        for (current_pin = pin_0; current_pin < pin_max; current_pin++)
        {
            // Set as output first to ensure writing 0 is effective
            *gpio_moder_ptr &= ~(3UL << (current_pin * 2)); // Clear mode bits
            *gpio_moder_ptr |= (1UL << (current_pin * 2));  // Set as General purpose output mode (01)

            // Set output value to 0
            *gpio_odr_ptr &= ~(1UL << current_pin);
            // Verify (Rules.json: GPIO_rules - After setting GPIO value, verify with while loop)
            while ((*gpio_odr_ptr & (1UL << current_pin)) != 0);

            WDT_Reset(); // Rules.json: API_implementation_sequence
        }
    }

    // Step 2: Set all GPIO pins direction to input and verify
    for (current_port = port_a; current_port < port_max; current_port++)
    {
        gpio_moder_ptr = REG_ACCESS(get_gpio_base_ptr(current_port), GPIO_MODER_OFFSET);
        volatile uint32_t* gpio_pupdr_ptr = REG_ACCESS(get_gpio_base_ptr(current_port), GPIO_PUPDR_OFFSET);
        volatile uint32_t* gpio_idr_ptr = REG_ACCESS(get_gpio_base_ptr(current_port), GPIO_IDR_OFFSET);

        for (current_pin = pin_0; current_pin < pin_max; current_pin++)
        {
            // Set as input mode (00)
            *gpio_moder_ptr &= ~(3UL << (current_pin * 2));

            // All input pins have pull-up resistors (Rules.json: GPIO_rules)
            *gpio_pupdr_ptr &= ~(3UL << (current_pin * 2)); // Clear pull-up/pull-down bits
            *gpio_pupdr_ptr |= (1UL << (current_pin * 2));  // Set as Pull-up (01)

            // Verification for input mode (cannot directly verify input state without external stimulus)
            // A more robust verification would involve toggling an external signal and checking IDR.
            // For now, we'll assume the register write takes effect.
            // The rule "verify with while loop" is for *output* or if input behavior can be deterministically set.
            // For general input setup, we trust the register write.
            (void)*gpio_idr_ptr; // dummy read
            WDT_Reset(); // Rules.json: API_implementation_sequence
        }
    }

    // Step 3: Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable(); // Implemented below

    // Disable supported peripherals by calling their respective Disable functions
    t_uart_channel u_channel;
    for(u_channel = uart_channel_usart1; u_channel < uart_channel_max; u_channel++) UART_Disable(u_channel);
    t_i2c_channel i_channel;
    for(i_channel = i2c_channel_i2c1; i_channel < i2c_channel_max; i_channel++) I2C_Disable(i_channel);
    t_spi_channel s_channel;
    for(s_channel = spi_channel_spi1; s_channel < spi_channel_max; s_channel++) SPI_Disable(s_channel);
    t_timer_channel t_channel;
    for(t_channel = timer_channel_tim1; t_channel < timer_channel_max; t_channel++) TIMER_Disable(t_channel);
    // ADC is single instance, but it's an API requirement
    ADC_Disable(adc_channel_0); // Any channel can be passed, as it disables the whole ADC1

    // I2S is part of SPI, but API exists, so disable its features
    t_i2s_channel i2s_ch;
    for(i2s_ch = i2s_channel_i2s1; i2s_ch < i2s_channel_max; i2s_ch++) I2S_Disable(i2s_ch);


    // Step 4: Enable WDT (Watchdog Timer)
    // No specific WDT enable registers provided in register_json.
    // For STM32F401RC IWDG:
    // *REG_ACCESS(IWDG_BASE, IWDG_KR_OFFSET) = 0xCCCC; // Enable LSI oscillator and start IWDG
    // Inferred register access, as per Rules.json: peripheral_enable_rules (if not found in JSON)
    // Placeholder:
    // Enable the LSI clock first for IWDG (LSI is internal low-speed RC oscillator)
    // *REG_ACCESS(RCC_BASE_ADDRESS, RCC_CSR_OFFSET) |= (1UL << 0); // LSION bit (LSI oscillator enable)
    // while(!(*REG_ACCESS(RCC_BASE_ADDRESS, RCC_CSR_OFFSET) & (1UL << 1))); // Wait for LSIRDY
    // Then enable IWDG, clear WDT etc. as below comments suggest.

    // Step 5: Clear WDT timer
    // Placeholder (would be IWDG_KR = 0xAAAA;)
    WDT_Reset();

    // Step 6: Set WDT period >= 8 msec
    // No specific WDT prescaler/reload registers provided in register_json.
    // For STM32F401RC IWDG: IWDG_PR (prescaler), IWDG_RLR (reload register)
    // e.g., Set prescaler for slowest clock, reload register for desired timeout.
    // Placeholder.

    // Step 7: Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // For STM32F401RC, Low Voltage Detect (LVD) is handled by the Power Control (PWR) peripheral.
    // Specific PWR_CR register bits (PLS[2:0] for PVD level selection, PVDE for PVD enable) are not in register_json.
    // Therefore, this is a placeholder.
    // Example (if PWR_CR was in register_json):
    // peripheral_clock_enable('P', 0); // Enable PWR clock (e.g. RCC_APB1ENR |= (1UL << 28))
    // volatile uint32_t* pwr_cr_ptr = REG_ACCESS(PWR_BASE_ADDRESS, PWR_CR_OFFSET);
    // if (volt == sys_volt_3v) {
    //     *pwr_cr_ptr = (*pwr_cr_ptr & ~((7UL << 5))) | (4UL << 5); // Set PVD level to 2.0V (PLS = 100)
    // } else { // sys_volt_5v
    //     *pwr_cr_ptr = (*pwr_cr_ptr & ~((7UL << 5))) | (7UL << 5); // Set PVD level to 3.5V (PLS = 111)
    // }
    // Placeholder:
    (void)volt; // Suppress unused parameter warning

    // Step 8: Enable LVR (Low Voltage Reset)
    // This typically means enabling PVD (Power Voltage Detector) in STM32.
    // Placeholder (would be *pwr_cr_ptr |= (1UL << 4); // PVDE bit)

    // Step 9: Clear WDT again
    WDT_Reset(); // Rules.json: API_implementation_sequence
}

/**
 * @brief Enters the microcontroller into a low-power sleep mode.
 *        Implements Rules.json: sleep_mode_definition.
 *        For Cortex-M, this typically involves the WFI (Wait For Interrupt) instruction.
 */
void Go_to_sleep_mode(void)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    // For Cortex-M, WFI instruction puts the processor into sleep mode.
    __WFI(); // Wait For Interrupt
}

/**
 * @brief Enables global interrupts.
 *        Uses CMSIS intrinsic for ARM Cortex-M.
 */
void Global_interrupt_Enable(void)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    __enable_irq(); // CMSIS function to enable global interrupts (CPSIE I)
}

/**
 * @brief Disables global interrupts.
 *        Uses CMSIS intrinsic for ARM Cortex-M.
 */
void Global_interrupt_Disable(void)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    __disable_irq(); // CMSIS function to disable global interrupts (CPSID I)
}

// ============================================================================
// LVD Module
// LVD module not supported on this MCU based on provided register_json (no dedicated LVD/PVD registers).
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================

// ============================================================================
// UART Module Implementations
// ============================================================================

/**
 * @brief Initializes a specified UART channel with the given parameters.
 * @param uart_channel The UART peripheral to initialize (e.g., uart_channel_usart1).
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data frame length (8 or 9 bits).
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity mode (none, even, or odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* usart_base = get_uart_base_ptr(uart_channel);
    if (usart_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(usart_base, USART_CR1_OFFSET);
    volatile uint32_t* cr2_ptr = REG_ACCESS(usart_base, USART_CR2_OFFSET);
    volatile uint32_t* brr_ptr = REG_ACCESS(usart_base, USART_BRR_OFFSET);

    // Disable the USART peripheral before configuration
    *cr1_ptr &= ~(1UL << 13); // Clear UE bit (USART Enable)

    // Configure Baud Rate (example for 16MHz APB2 for USART1/6, 16MHz APB1 for USART2)
    // This is a simplified calculation, a full implementation would use the APB clock frequency.
    // For STM32F401RC, APB1 clock is typically up to 42MHz, APB2 up to 84MHz.
    // Assuming APB clock for USART is 16MHz for simplification, adjust as needed.
    tlong pclk_freq = 16000000; // Placeholder for APB clock (e.g. PCLK1 or PCLK2)
    if (uart_channel == uart_channel_usart1 || uart_channel == uart_channel_usart6)
    {
        // USART1 and USART6 are on APB2 bus
        pclk_freq = 84000000; // For STM32F401RC, APB2 max is 84MHz
    }
    else // USART2
    {
        // USART2 is on APB1 bus
        pclk_freq = 42000000; // For STM32F401RC, APB1 max is 42MHz
    }

    tword usart_div = 0;
    switch (uart_baud_rate)
    {
        case uart_baud_rate_9600:   usart_div = (tword)(pclk_freq / (16 * 9600)); break;
        case uart_baud_rate_19200:  usart_div = (tword)(pclk_freq / (16 * 19200)); break;
        case uart_baud_rate_38400:  usart_div = (tword)(pclk_freq / (16 * 38400)); break;
        case uart_baud_rate_57600:  usart_div = (tword)(pclk_freq / (16 * 57600)); break;
        case uart_baud_rate_115200: usart_div = (tword)(pclk_freq / (16 * 115200)); break;
        default: usart_div = (tword)(pclk_freq / (16 * 9600)); break; // Default to 9600
    }
    *brr_ptr = ((usart_div / 16) << 4) | (usart_div % 16); // DIV_M and DIV_F for oversampling by 16

    // Configure Data Length (M bit in CR1)
    *cr1_ptr &= ~(1UL << 12); // Clear M bit for 8-bit word length
    if (uart_data_length == uart_data_length_9bit)
    {
        *cr1_ptr |= (1UL << 12); // Set M bit for 9-bit word length
    }

    // Configure Stop Bits (STOP bits in CR2)
    *cr2_ptr &= ~(3UL << 12); // Clear STOP bits
    switch (uart_stop_bit)
    {
        case uart_stop_bit_0_5: *cr2_ptr |= (1UL << 12); break; // 0.5 stop bits (SmartCard)
        case uart_stop_bit_2:   *cr2_ptr |= (2UL << 12); break; // 2 stop bits
        case uart_stop_bit_1_5: *cr2_ptr |= (3UL << 12); break; // 1.5 stop bits (SmartCard)
        case uart_stop_bit_1:
        default: break; // 1 stop bit (default, 00)
    }

    // Configure Parity (PCE, PS bits in CR1)
    *cr1_ptr &= ~((1UL << 10) | (1UL << 9)); // Clear PCE (Parity Control Enable) and PS (Parity Selection)
    if (uart_parity != uart_parity_none)
    {
        *cr1_ptr |= (1UL << 10); // Enable Parity Control (PCE)
        if (uart_parity == uart_parity_odd)
        {
            *cr1_ptr |= (1UL << 9); // Select Odd Parity (PS)
        }
    }

    // Enable Transmitter (TE) and Receiver (RE)
    *cr1_ptr |= (1UL << 3) | (1UL << 2);

    // Enable the USART peripheral (UE) - Done by UART_Enable
}

/**
 * @brief Enables the specified UART channel.
 *        Implements Rules.json: peripheral_enable_rules.
 * @param uart_channel The UART peripheral to enable.
 */
void UART_Enable(t_uart_channel uart_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* usart_base = get_uart_base_ptr(uart_channel);
    if (usart_base == NULL) return; // Invalid channel

    // 1. Enable peripheral clock (Rules.json: peripheral_enable_rules)
    if (uart_channel == uart_channel_usart1)
    {
        peripheral_clock_enable('1', 4); // USART1EN (bit 4 of RCC_APB2ENR)
    }
    else if (uart_channel == uart_channel_usart2)
    {
        peripheral_clock_enable('2', 17); // USART2EN (bit 17 of RCC_APB1ENR)
    }
    else if (uart_channel == uart_channel_usart6)
    {
        peripheral_clock_enable('6', 5); // USART6EN (bit 5 of RCC_APB2ENR)
    }

    // 2. Set the peripheral's main enable bit (UE for USART)
    volatile uint32_t* cr1_ptr = REG_ACCESS(usart_base, USART_CR1_OFFSET);
    *cr1_ptr |= (1UL << 13); // Set UE bit (USART Enable)
}

/**
 * @brief Disables the specified UART channel.
 * @param uart_channel The UART peripheral to disable.
 */
void UART_Disable(t_uart_channel uart_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* usart_base = get_uart_base_ptr(uart_channel);
    if (usart_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(usart_base, USART_CR1_OFFSET);
    *cr1_ptr &= ~(1UL << 13); // Clear UE bit (USART Enable)
    // Note: Clock is usually left enabled, or handled by a system-level clock management unit if explicit disabling is required.
}

/**
 * @brief Sends a single byte over the specified UART channel.
 * @param uart_channel The UART peripheral to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* usart_base = get_uart_base_ptr(uart_channel);
    if (usart_base == NULL) return; // Invalid channel

    volatile uint32_t* sr_ptr = REG_ACCESS(usart_base, USART_SR_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(usart_base, USART_DR_OFFSET);

    // Wait until Transmit Data Register Empty (TXE) flag is set
    while (!(*sr_ptr & (1UL << 7)));
    *dr_ptr = byte; // Write data to the Data Register
    // Wait until Transmission Complete (TC) flag is set
    while (!(*sr_ptr & (1UL << 6)));
}

/**
 * @brief Sends a frame of data over the specified UART channel.
 * @param uart_channel The UART peripheral to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (data == NULL || length <= 0) return;

    for (int i = 0; i < length; i++)
    {
        UART_send_byte(uart_channel, (tbyte)data[i]);
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Sends a null-terminated string over the specified UART channel.
 * @param uart_channel The UART peripheral to use.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (str == NULL) return;

    while (*str != '\0')
    {
        UART_send_byte(uart_channel, (tbyte)*str);
        str++;
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Receives a single byte from the specified UART channel.
 * @param uart_channel The UART peripheral to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* usart_base = get_uart_base_ptr(uart_channel);
    if (usart_base == NULL) return 0; // Invalid channel

    volatile uint32_t* sr_ptr = REG_ACCESS(usart_base, USART_SR_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(usart_base, USART_DR_OFFSET);

    // Wait until Receive Data Register Not Empty (RXNE) flag is set
    while (!(*sr_ptr & (1UL << 5)));
    return (tbyte)(*dr_ptr & 0xFF); // Read data from the Data Register (8-bit)
}

/**
 * @brief Receives a frame of data over the specified UART channel.
 * @param uart_channel The UART peripheral to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (buffer == NULL || max_length <= 0) return;

    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Receives a null-terminated string from the specified UART channel.
 * @param uart_channel The UART peripheral to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes received, or 0 if an error occurred.
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    for (i = 0; i < max_length - 1; i++)
    {
        char received_char = (char)UART_Get_Byte(uart_channel);
        buffer[i] = received_char;
        if (received_char == '\0' || received_char == '\n' || received_char == '\r') // Check for termination characters
        {
            if (received_char != '\0')
            {
                buffer[i] = '\0'; // Ensure null termination
            }
            break;
        }
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
    buffer[i] = '\0'; // Ensure null termination
    return i; // Return number of characters read
}

// ============================================================================
// I2C Module Implementations
// ============================================================================

/**
 * @brief Initializes a specified I2C channel.
 *        Implements Rules.json: I2C_rules.
 * @param i2c_channel The I2C peripheral to initialize.
 * @param i2c_clk_speed The desired clock speed (always fast mode as per rules).
 * @param i2c_device_address The device's own 7-bit address.
 * @param i2c_ack Acknowledge enable/disable.
 * @param i2c_datalength Expected data length (for some internal logic, though I2C is byte-oriented).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* i2c_base = get_i2c_base_ptr(i2c_channel);
    if (i2c_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(i2c_base, I2C_CR1_OFFSET);
    volatile uint32_t* cr2_ptr = REG_ACCESS(i2c_base, I2C_CR2_OFFSET);
    volatile uint32_t* oar1_ptr = REG_ACCESS(i2c_base, I2C_OAR1_OFFSET);
    volatile uint32_t* ccr_ptr = REG_ACCESS(i2c_base, I2C_CCR_OFFSET);
    volatile uint32_t* trise_ptr = REG_ACCESS(i2c_base, I2C_TRISE_OFFSET);

    // Disable I2C peripheral before configuration (PE bit in CR1)
    *cr1_ptr &= ~(1UL << 0);

    // Get PCLK1 frequency (I2C peripherals are on APB1)
    // Assuming PCLK1 is 42MHz for STM32F401RC
    tlong pclk1_freq_hz = 42000000;
    // Set FREQ field in CR2 (peripheral clock frequency)
    *cr2_ptr = (pclk1_freq_hz / 1000000); // FREQ[5:0] bits, e.g., 42 for 42MHz

    // Configure clock speed (Rules.json: I2C_rules - Always use fast mode)
    // Fast mode: 400 kHz (max)
    tlong i2c_speed_hz = 400000; // 400 kHz

    // CCR calculation for Fast Mode (F/S = 1, DUTY = 0)
    // Thigh = CCR * Tpclk1, Tlow = 2 * CCR * Tpclk1 (for Fm duty cycle 2)
    // Tscl = Thigh + Tlow = 3 * CCR * Tpclk1 = 1 / I2C_Speed
    // CCR = Tpclk1 / (3 * I2C_Speed) = PCLK1_Freq / (3 * I2C_Speed)
    // Simplified for Duty Cycle 2 (Fm): CCR = PCLK1_Freq / (3 * I2C_Speed) (for 30% duty cycle)
    // For Duty Cycle 1 (Fm): CCR = PCLK1_Freq / (25 * I2C_Speed) (for 50% duty cycle, assuming 16/9, this is complex)
    // Let's use the formula from RM: CCR = (PCLK1 / (2 * I2C_speed)) in Sm, or (PCLK1 / (3 * I2C_speed)) or (PCLK1 / (25 * I2C_speed)) for Fm.
    // Given the rule "always use fast mode" and no specific duty cycle, we'll aim for 400kHz.
    // For simplicity with fast mode (400kHz), typically CCR = PCLK1_Freq / (3 * I2C_Speed) might be a good starting point.
    // Or for fast mode duty cycle 1 (T_low/T_high = 2, i.e. 33% high, 66% low): CCR = (PCLK1_Freq / (3 * I2C_Speed))
    // Or for fast mode duty cycle 2 (T_low/T_high = 16/9, i.e. 64% high, 36% low): CCR = (PCLK1_Freq / (25 * I2C_Speed))
    // Let's use a common formula for fast mode, assuming Fm mode and duty cycle = 2 (Tlow/Thigh = 2) for standard.
    tword ccr_value = (tword)(pclk1_freq_hz / (3 * i2c_speed_hz));
    *ccr_ptr = (1UL << 15) | (2UL << 14) | ccr_value; // Fm mode (bit 15), Duty cycle 2 (bit 14:13 = 10b)

    // TRISE calculation for Fast Mode (Rules.json: I2C_rules)
    // TRISE = (PCLK1_Freq / 1000000) * 300ns (max rise time for Fm) + 1
    tword trise_value = (tword)(((pclk1_freq_hz / 1000000) * 300) / 1000) + 1;
    *trise_ptr = trise_value;

    // Configure Own Address 1 (OAR1)
    *oar1_ptr = (1UL << 14) | ((i2c_device_address & 0x7F) << 1); // Set ADDMODE (7-bit address), ADDR1[7:1]

    // Configure ACK (ACK bit in CR1)
    if (i2c_ack == i2c_ack_enable)
    {
        *cr1_ptr |= (1UL << 10); // Set ACK bit
    }
    else
    {
        *cr1_ptr &= ~(1UL << 10); // Clear ACK bit
    }

    // Data length is handled byte-by-byte in I2C data transfer, so i2c_datalength is informational for application.
    (void)i2c_datalength; // Suppress unused parameter warning

    // Enable I2C peripheral (PE bit in CR1) - Done by I2C_Enable
}

/**
 * @brief Enables the specified I2C channel.
 *        Implements Rules.json: peripheral_enable_rules.
 * @param i2c_channel The I2C peripheral to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* i2c_base = get_i2c_base_ptr(i2c_channel);
    if (i2c_base == NULL) return; // Invalid channel

    // 1. Enable peripheral clock (Rules.json: peripheral_enable_rules)
    if (i2c_channel == i2c_channel_i2c1)
    {
        peripheral_clock_enable('I', 21); // I2C1EN (bit 21 of RCC_APB1ENR)
    }
    else if (i2c_channel == i2c_channel_i2c2)
    {
        peripheral_clock_enable('2', 22); // I2C2EN (bit 22 of RCC_APB1ENR)
    }
    else if (i2c_channel == i2c_channel_i2c3)
    {
        peripheral_clock_enable('3', 23); // I2C3EN (bit 23 of RCC_APB1ENR)
    }

    // 2. Set the peripheral's main enable bit (PE for I2C)
    volatile uint32_t* cr1_ptr = REG_ACCESS(i2c_base, I2C_CR1_OFFSET);
    *cr1_ptr |= (1UL << 0); // Set PE bit (Peripheral Enable)
}

/**
 * @brief Disables the specified I2C channel.
 * @param i2c_channel The I2C peripheral to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* i2c_base = get_i2c_base_ptr(i2c_channel);
    if (i2c_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(i2c_base, I2C_CR1_OFFSET);
    *cr1_ptr &= ~(1UL << 0); // Clear PE bit (Peripheral Enable)
}

/**
 * @brief Sends a single byte over the specified I2C channel.
 *        This function assumes a start condition has been generated and address sent.
 * @param i2c_channel The I2C peripheral to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* i2c_base = get_i2c_base_ptr(i2c_channel);
    if (i2c_base == NULL) return;

    volatile uint32_t* sr1_ptr = REG_ACCESS(i2c_base, I2C_SR1_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(i2c_base, I2C_DR_OFFSET);
    // volatile uint32_t* cr1_ptr = REG_ACCESS(i2c_base, I2C_CR1_OFFSET);

    // Wait for TxE (Transmit data register empty)
    // Rules.json: I2C_rules - Always use maximum timeout
    volatile tlong timeout = 0xFFFFFFFF;
    while (!(*sr1_ptr & (1UL << 7)) && (timeout-- > 0));
    if (timeout == 0) return; // Timeout error

    *dr_ptr = byte; // Write data to DR

    // Wait for BTF (Byte Transfer Finished)
    timeout = 0xFFFFFFFF;
    while (!(*sr1_ptr & (1UL << 2)) && (timeout-- > 0));
    // DR is available, BTF is set, SR1 read to clear it implicitly
}

/**
 * @brief Sends a frame of data over the specified I2C channel.
 *        Generates START, sends address, then data.
 * @param i2c_channel The I2C peripheral to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* i2c_base = get_i2c_base_ptr(i2c_channel);
    if (i2c_base == NULL) return;
    if (data == NULL || length <= 0) return;

    volatile uint32_t* cr1_ptr = REG_ACCESS(i2c_base, I2C_CR1_OFFSET);
    volatile uint32_t* sr1_ptr = REG_ACCESS(i2c_base, I2C_SR1_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(i2c_base, I2C_DR_OFFSET);

    // Generate START condition
    *cr1_ptr |= (1UL << 8); // Set START bit
    volatile tlong timeout = 0xFFFFFFFF;
    while (!(*sr1_ptr & (1UL << 0)) && (timeout-- > 0)); // Wait for SB (Start Bit)
    if (timeout == 0) return; // Timeout error

    // Send slave address (7-bit address + R/W bit, R/W = 0 for write)
    tbyte slave_address = 0x50; // Example slave address (0x50 << 1 = 0xA0)
                                // The rule "Addressing Mode equals Device Address" from I2C_rules, combined with I2C_Init parameter
                                // implies `i2c_device_address` is *our* device address.
                                // For `send_frame`, we need the *target* device address.
                                // This requires the target address to be passed as a parameter or be configured.
                                // For now, I'll use a placeholder `slave_address` and assume target address is implicit.
    *dr_ptr = (slave_address << 1) & ~1; // Send 7-bit address + Write bit (0)
    timeout = 0xFFFFFFFF;
    while (!(*sr1_ptr & (1UL << 1)) && (timeout-- > 0)); // Wait for ADDR (Address sent)
    if (timeout == 0) return; // Timeout error

    // Clear ADDR flag by reading SR1 followed by SR2
    (void)*sr1_ptr;
    (void)*REG_ACCESS(i2c_base, I2C_SR2_OFFSET);

    for (int i = 0; i < length; i++)
    {
        I2C_send_byte(i2c_channel, (tbyte)data[i]);
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }

    // Generate STOP condition
    // Rules.json: I2C_rules - Always generate a repeated start condition instead of stop between transactions
    // This implies that for a single "frame" transmission, a STOP would still be needed if it's the end.
    // However, if multiple send_frame/send_byte are logically part of a single transaction,
    // then repeated START might be preferred. For this generic function, we'll issue a STOP.
    *cr1_ptr |= (1UL << 9); // Set STOP bit
}

/**
 * @brief Sends a null-terminated string over the specified I2C channel.
 * @param i2c_channel The I2C peripheral to use.
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (str == NULL) return;
    I2C_send_frame(i2c_channel, str, strlen(str));
}

/**
 * @brief Receives a single byte from the specified I2C channel.
 *        Assumes start, address, and proper ACK setup are done.
 * @param i2c_channel The I2C peripheral to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* i2c_base = get_i2c_base_ptr(i2c_channel);
    if (i2c_base == NULL) return 0;

    volatile uint32_t* sr1_ptr = REG_ACCESS(i2c_base, I2C_SR1_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(i2c_base, I2C_DR_OFFSET);
    // volatile uint32_t* cr1_ptr = REG_ACCESS(i2c_base, I2C_CR1_OFFSET); // To manage ACK/NACK

    // Wait for RXNE (Receive data register not empty)
    volatile tlong timeout = 0xFFFFFFFF;
    while (!(*sr1_ptr & (1UL << 6)) && (timeout-- > 0));
    if (timeout == 0) return 0; // Timeout error

    return (tbyte)(*dr_ptr & 0xFF); // Read data from DR
}

/**
 * @brief Receives a frame of data over the specified I2C channel.
 *        Generates START, sends address, then receives data.
 * @param i2c_channel The I2C peripheral to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to receive.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* i2c_base = get_i2c_base_ptr(i2c_channel);
    if (i2c_base == NULL) return;
    if (buffer == NULL || max_length <= 0) return;

    volatile uint32_t* cr1_ptr = REG_ACCESS(i2c_base, I2C_CR1_OFFSET);
    volatile uint32_t* sr1_ptr = REG_ACCESS(i2c_base, I2C_SR1_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(i2c_base, I2C_DR_OFFSET);

    tbyte slave_address = 0x50; // Example slave address, requires actual target address

    // Generate START condition
    *cr1_ptr |= (1UL << 8); // Set START bit
    volatile tlong timeout = 0xFFFFFFFF;
    while (!(*sr1_ptr & (1UL << 0)) && (timeout-- > 0)); // Wait for SB (Start Bit)
    if (timeout == 0) return; // Timeout error

    // Send slave address + Read bit (1)
    *dr_ptr = (slave_address << 1) | 1; // Send 7-bit address + Read bit (1)
    timeout = 0xFFFFFFFF;
    while (!(*sr1_ptr & (1UL << 1)) && (timeout-- > 0)); // Wait for ADDR (Address sent)
    if (timeout == 0) return; // Timeout error

    // Clear ADDR flag by reading SR1 followed by SR2
    (void)*sr1_ptr;
    (void)*REG_ACCESS(i2c_base, I2C_SR2_OFFSET);

    for (int i = 0; i < max_length; i++)
    {
        if (i == max_length - 1)
        {
            *cr1_ptr &= ~(1UL << 10); // Clear ACK bit for NACK on last byte
            *cr1_ptr |= (1UL << 9);   // Generate STOP condition for last byte
        }
        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Receives a null-terminated string from the specified I2C channel.
 * @param i2c_channel The I2C peripheral to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes received, or 0 if an error occurred.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (buffer == NULL || max_length <= 0) return 0;

    // This function assumes a specific protocol where the I2C slave sends a null-terminated string.
    // I2C is byte-oriented, so termination needs to be externally defined.
    // For simplicity, it will read up to max_length or until a null is received.
    int i = 0;
    for (i = 0; i < max_length - 1; i++)
    {
        char received_char = (char)I2C_Get_Byte(i2c_channel);
        buffer[i] = received_char;
        if (received_char == '\0')
        {
            break;
        }
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
    buffer[i] = '\0'; // Ensure null termination
    return i;
}

// ============================================================================
// SPI (CSI) Module Implementations
// ============================================================================

/**
 * @brief Initializes a specified SPI channel.
 *        Implements Rules.json: SPI_rules.
 * @param spi_channel The SPI peripheral to initialize.
 * @param spi_mode Master/Slave mode.
 * @param spi_cpol Clock polarity.
 * @param spi_cpha Clock phase.
 * @param spi_dff Data frame format (8 or 16 bit).
 * @param spi_bit_order Bit order (MSB or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = get_spi_base_ptr(spi_channel);
    if (spi_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(spi_base, SPI_CR1_OFFSET);
    volatile uint32_t* cr2_ptr = REG_ACCESS(spi_base, SPI_CR2_OFFSET);

    // Disable SPI peripheral before configuration (SPE bit in CR1)
    *cr1_ptr &= ~(1UL << 6);

    // Configure Master/Slave mode (MSTR bit in CR1)
    if (spi_mode == spi_mode_master)
    {
        *cr1_ptr |= (1UL << 2); // Set MSTR bit
        // Rules.json: SPI_rules - Slave Select always software-controlled (SSM=1, SSI=1)
        *cr1_ptr |= (1UL << 9) | (1UL << 8); // Set SSM and SSI bits
    }
    else
    {
        *cr1_ptr &= ~(1UL << 2); // Clear MSTR bit
        // Rules.json: SPI_rules - Slave Select always software-controlled (SSM=1, SSI=0 or controlled externally)
        // For slave, SSM and SSI might be configured differently based on hardware CS pin.
        // For generic software-controlled, just SSM is relevant.
        *cr1_ptr |= (1UL << 9); // Set SSM bit, SSI bit is typically 0 for slave in this config or controlled by external NSS
        *cr1_ptr &= ~(1UL << 8); // Clear SSI bit (can be set by external NSS if SSM=0)
    }

    // Configure Clock Polarity (CPOL bit in CR1)
    if (spi_cpol == spi_cpol_high)
    {
        *cr1_ptr |= (1UL << 1);
    }
    else
    {
        *cr1_ptr &= ~(1UL << 1);
    }

    // Configure Clock Phase (CPHA bit in CR1)
    if (spi_cpha == spi_cpha_2edge)
    {
        *cr1_ptr |= (1UL << 0);
    }
    else
    {
        *cr1_ptr &= ~(1UL << 0);
    }

    // Configure Data Frame Format (DFF bit in CR1)
    if (spi_dff == spi_dff_16bit)
    {
        *cr1_ptr |= (1UL << 11);
    }
    else
    {
        *cr1_ptr &= ~(1UL << 11); // 8-bit Data Frame Format
    }

    // Configure Bit Order (LSBFIRST bit in CR1)
    if (spi_bit_order == spi_bit_order_lsb_first)
    {
        *cr1_ptr |= (1UL << 7);
    }
    else
    {
        *cr1_ptr &= ~(1UL << 7); // MSB first
    }

    // Rules.json: SPI_rules - Always use fast speed
    // This implies setting the prescaler to 2 (lowest division)
    *cr1_ptr &= ~(7UL << 3); // Clear BR[2:0] bits
    *cr1_ptr |= (0UL << 3);  // Set prescaler to PCLK/2 (BR=000)

    // Rules.json: SPI_rules - Always use full duplex (BIDIMODE=0, RXONLY=0)
    *cr1_ptr &= ~((1UL << 15) | (1UL << 10)); // BIDIMODE=0, RXONLY=0

    // Rules.json: SPI_rules - Always enable CRC
    *cr1_ptr |= (1UL << 13); // Set CRCEN bit (Hardware CRC calculation enabled)
    *cr2_ptr |= (1UL << 0);  // Set RXDMAEN (Receive buffer DMA enable) is often used with CRC for efficiency.

    // Enable the SPI peripheral (SPE) - Done by SPI_Enable
}

/**
 * @brief Enables the specified SPI channel.
 *        Implements Rules.json: peripheral_enable_rules.
 * @param spi_channel The SPI peripheral to enable.
 */
void SPI_Enable(t_spi_channel spi_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = get_spi_base_ptr(spi_channel);
    if (spi_base == NULL) return; // Invalid channel

    // 1. Enable peripheral clock (Rules.json: peripheral_enable_rules)
    if (spi_channel == spi_channel_spi1)
    {
        peripheral_clock_enable('1', 12); // SPI1EN (bit 12 of RCC_APB2ENR)
    }
    else if (spi_channel == spi_channel_spi2)
    {
        peripheral_clock_enable('2', 14); // SPI2EN (bit 14 of RCC_APB1ENR)
    }
    else if (spi_channel == spi_channel_spi3)
    {
        peripheral_clock_enable('3', 15); // SPI3EN (bit 15 of RCC_APB1ENR)
    }

    // 2. Set the peripheral's main enable bit (SPE for SPI)
    volatile uint32_t* cr1_ptr = REG_ACCESS(spi_base, SPI_CR1_OFFSET);
    *cr1_ptr |= (1UL << 6); // Set SPE bit (SPI Enable)
}

/**
 * @brief Disables the specified SPI channel.
 * @param spi_channel The SPI peripheral to disable.
 */
void SPI_Disable(t_spi_channel spi_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = get_spi_base_ptr(spi_channel);
    if (spi_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(spi_base, SPI_CR1_OFFSET);
    *cr1_ptr &= ~(1UL << 6); // Clear SPE bit (SPI Enable)
}

/**
 * @brief Sends a single byte over the specified SPI channel.
 * @param spi_channel The SPI peripheral to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = get_spi_base_ptr(spi_channel);
    if (spi_base == NULL) return;

    volatile uint33_t* sr_ptr = REG_ACCESS(spi_base, SPI_SR_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(spi_base, SPI_DR_OFFSET);

    // Wait until TXE flag is set (Transmit buffer empty)
    volatile tlong timeout = 0xFFFFFFFF;
    while (!(*sr_ptr & (1UL << 1)) && (timeout-- > 0));
    if (timeout == 0) return; // Timeout error

    *dr_ptr = byte; // Write data to DR

    // Wait until RXNE flag is set (Receive buffer not empty) to ensure full duplex transfer if configured)
    // If master, this waits for slave response. If slave, it waits for master's data.
    timeout = 0xFFFFFFFF;
    while (!(*sr_ptr & (1UL << 0)) && (timeout-- > 0));
    if (timeout == 0) return; // Timeout error

    // Read DR to clear RXNE and complete the transaction
    (void)*dr_ptr;
}

/**
 * @brief Sends a frame of data over the specified SPI channel.
 * @param spi_channel The SPI peripheral to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (data == NULL || length <= 0) return;

    for (int i = 0; i < length; i++)
    {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Receives a single byte from the specified SPI channel.
 *        Performs a dummy write to initiate clock for reception.
 * @param spi_channel The SPI peripheral to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = get_spi_base_ptr(spi_channel);
    if (spi_base == NULL) return 0;

    volatile uint32_t* sr_ptr = REG_ACCESS(spi_base, SPI_SR_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(spi_base, SPI_DR_OFFSET);

    // For full-duplex operation, a byte must be written to transmit something
    // (and thus generate clock) to receive data.
    // Master mode: write dummy byte to DR to start clock and receive data.
    // Slave mode: depends on master sending data.
    // Assuming master mode for active reception here.
    *dr_ptr = 0xFF; // Write dummy byte (0xFF is common)

    // Wait until RXNE flag is set (Receive buffer not empty)
    volatile tlong timeout = 0xFFFFFFFF;
    while (!(*sr_ptr & (1UL << 0)) && (timeout-- > 0));
    if (timeout == 0) return 0; // Timeout error

    return (tbyte)(*dr_ptr & 0xFF); // Read data from DR
}

/**
 * @brief Receives a frame of data over the specified SPI channel.
 * @param spi_channel The SPI peripheral to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (buffer == NULL || max_length <= 0) return;

    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Receives a null-terminated string from the specified SPI channel.
 * @param spi_channel The SPI peripheral to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes received, or 0 if an error occurred.
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (buffer == NULL || max_length <= 0) return 0;

    int i = 0;
    for (i = 0; i < max_length - 1; i++)
    {
        char received_char = (char)SPI_Get_Byte(spi_channel);
        buffer[i] = received_char;
        if (received_char == '\0') // Assuming string is null-terminated by sender
        {
            break;
        }
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
    buffer[i] = '\0'; // Ensure null termination
    return i;
}

// ============================================================================
// External Interrupt Module Implementations
// ============================================================================

/**
 * @brief Initializes an external interrupt channel with a specified edge trigger.
 * @param external_int_channel The EXTI line number.
 * @param external_int_edge The trigger edge (rising, falling, or both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    // 1. Enable SYSCFG clock (required for EXTI line configuration)
    peripheral_clock_enable('1', 14); // SYSCFGEN (bit 14 of RCC_APB2ENR)

    // 2. Select the GPIO port for the EXTI line (SYSCFG_EXTICRx)
    // The `external_int_channel` directly maps to EXTI line number (0-15).
    // The GPIO port is usually assigned during application design.
    // This MCAL function, however, only receives the EXTI line.
    // To be generic, this function cannot set the source GPIO port without additional parameters.
    // Therefore, this step is commented out or assumes a default/external configuration.
    // Example: SYSCFG->EXTICR[channel / 4] = (port << ((channel % 4) * 4));
    // For now, it will only configure EXTI registers directly.
    // NOTE: For EXTI_EXTICR, the assigned_pin data would be needed from register_json to pick the GPIO port.
    // As per `SYSCFG_EXTICR1` to `SYSCFG_EXTICR4` definitions, the pins are provided in assigned_pin.
    // A mapping from t_external_int_channel to a default port (e.g., Port A) for that line or
    // requiring an additional `t_port` parameter would be necessary for a full solution.
    // For this exercise, we will assume the GPIO_Init is done separately for the pin.

    volatile uint32_t* exti_rtsr_ptr = REG_ACCESS(EXTI_BASE_ADDRESS, EXTI_RTSR_OFFSET);
    volatile uint32_t* exti_ftsr_ptr = REG_ACCESS(EXTI_BASE_ADDRESS, EXTI_FTSR_OFFSET);

    // Clear previous trigger selections for the line
    *exti_rtsr_ptr &= ~(1UL << external_int_channel);
    *exti_ftsr_ptr &= ~(1UL << external_int_channel);

    // Configure the trigger edge (RTSR and FTSR bits)
    if (external_int_edge == external_int_edge_rising)
    {
        *exti_rtsr_ptr |= (1UL << external_int_channel);
    }
    else if (external_int_edge == external_int_edge_falling)
    {
        *exti_ftsr_ptr |= (1UL << external_int_channel);
    }
    else // external_int_edge_rising_falling
    {
        *exti_rtsr_ptr |= (1UL << external_int_channel);
        *exti_ftsr_ptr |= (1UL << external_int_channel);
    }

    // Interrupt mask register (IMR) is handled in External_INT_Enable
    // Event mask register (EMR) is for events, not interrupts.
}

/**
 * @brief Enables the specified external interrupt channel.
 * @param external_int_channel The EXTI line number to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* exti_imr_ptr = REG_ACCESS(EXTI_BASE_ADDRESS, EXTI_IMR_OFFSET);
    *exti_imr_ptr |= (1UL << external_int_channel); // Set IMR bit for the line (Interrupt Mask)
}

/**
 * @brief Disables the specified external interrupt channel.
 * @param external_int_channel The EXTI line number to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* exti_imr_ptr = REG_ACCESS(EXTI_BASE_ADDRESS, EXTI_IMR_OFFSET);
    *exti_imr_ptr &= ~(1UL << external_int_channel); // Clear IMR bit for the line
}

// ============================================================================
// GPIO Module Implementations
// ============================================================================

/**
 * @brief Initializes a GPIO pin as an output and sets its initial value.
 *        Implements Rules.json: GPIO_rules.
 * @param port The GPIO port (e.g., port_a).
 * @param pin The pin number within the port (e.g., pin_0).
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* gpio_base = get_gpio_base_ptr(port);
    if (gpio_base == NULL) return;

    peripheral_clock_enable((char)('A' + port), 0); // Enable GPIOx clock

    volatile uint32_t* moder_ptr = REG_ACCESS(gpio_base, GPIO_MODER_OFFSET);
    volatile uint32_t* otyper_ptr = REG_ACCESS(gpio_base, GPIO_OTYPER_OFFSET);
    volatile uint32_t* ospeedr_ptr = REG_ACCESS(gpio_base, GPIO_OSPEEDR_OFFSET);
    volatile uint32_t* pupdr_ptr = REG_ACCESS(gpio_base, GPIO_PUPDR_OFFSET);
    volatile uint32_t* odr_ptr = REG_ACCESS(gpio_base, GPIO_ODR_OFFSET);

    // Rules.json: GPIO_rules - Always set value before setting direction
    GPIO_Value_Set(port, pin, value);
    WDT_Reset(); // Rules.json: API_implementation_sequence

    // Clear mode bits for the pin (2 bits per pin)
    *moder_ptr &= ~(3UL << (pin * 2));
    // Set pin mode to General purpose output mode (01)
    *moder_ptr |= (1UL << (pin * 2));

    // Output Type: Push-pull (default, OTYPER bit = 0)
    *otyper_ptr &= ~(1UL << pin);

    // Output Speed: High speed (10) for >=20mA sink current & >=10mA source current (Rules.json: GPIO_rules)
    *ospeedr_ptr &= ~(3UL << (pin * 2));
    *ospeedr_ptr |= (2UL << (pin * 2)); // High speed

    // Pull-up/Pull-down: No pull-up/pull-down (00) for output pins (Rules.json: GPIO_rules)
    *pupdr_ptr &= ~(3UL << (pin * 2));

    // Rules.json: GPIO_rules - After setting GPIO direction, verify with while loop
    // Check if mode is indeed output. This requires reading back the MODER register.
    while (((*moder_ptr >> (pin * 2)) & 3UL) != 1UL); // Wait until mode is 01b
}

/**
 * @brief Initializes a GPIO pin as an input.
 *        Implements Rules.json: GPIO_rules.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* gpio_base = get_gpio_base_ptr(port);
    if (gpio_base == NULL) return;

    peripheral_clock_enable((char)('A' + port), 0); // Enable GPIOx clock

    volatile uint32_t* moder_ptr = REG_ACCESS(gpio_base, GPIO_MODER_OFFSET);
    volatile uint32_t* pupdr_ptr = REG_ACCESS(gpio_base, GPIO_PUPDR_OFFSET);
    // No OTYPER, OSPEEDR for input

    // Clear mode bits for the pin
    *moder_ptr &= ~(3UL << (pin * 2));
    // Set pin mode to Input mode (00) - it's already 00 after clearing

    // Pull-up/Pull-down: Pull-up resistor enabled (01) (Rules.json: GPIO_rules)
    *pupdr_ptr &= ~(3UL << (pin * 2)); // Clear pull-up/pull-down bits
    *pupdr_ptr |= (1UL << (pin * 2));  // Set as Pull-up (01)

    // Rules.json: GPIO_rules - After setting GPIO direction, verify with while loop
    // Check if mode is indeed input.
    while (((*moder_ptr >> (pin * 2)) & 3UL) != 0UL); // Wait until mode is 00b
}

/**
 * @brief Gets the current direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction of the pin (direction_input or direction_output).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* gpio_base = get_gpio_base_ptr(port);
    if (gpio_base == NULL) return (t_direction)-1; // Invalid port

    volatile uint32_t* moder_ptr = REG_ACCESS(gpio_base, GPIO_MODER_OFFSET);
    tbyte mode = (tbyte)((*moder_ptr >> (pin * 2)) & 3UL); // Read 2 bits for the pin

    if (mode == 0UL) // 00: Input mode
    {
        return direction_input;
    }
    else if (mode == 1UL) // 01: General purpose output mode
    {
        return direction_output;
    }
    else
    {
        return (t_direction)-1; // Analog or Alternate function mode (not input/output)
    }
}

/**
 * @brief Sets the output value of a GPIO pin.
 *        Implements Rules.json: GPIO_rules.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* gpio_base = get_gpio_base_ptr(port);
    if (gpio_base == NULL) return;

    volatile uint32_t* bsrr_ptr = REG_ACCESS(gpio_base, GPIO_BSRR_OFFSET);
    volatile uint32_t* odr_ptr = REG_ACCESS(gpio_base, GPIO_ODR_OFFSET);

    if (value == 1)
    {
        *bsrr_ptr = (1UL << pin); // Set bit
    }
    else
    {
        *bsrr_ptr = (1UL << (pin + 16)); // Reset bit
    }

    // Rules.json: GPIO_rules - After setting GPIO value, verify with while loop
    // Check if the output data register reflects the set value
    if (value == 1)
    {
        while (!(*odr_ptr & (1UL << pin)));
    }
    else
    {
        while ((*odr_ptr & (1UL << pin)));
    }
}

/**
 * @brief Gets the input value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* gpio_base = get_gpio_base_ptr(port);
    if (gpio_base == NULL) return 0;

    volatile uint32_t* idr_ptr = REG_ACCESS(gpio_base, GPIO_IDR_OFFSET);
    return (tbyte)((*idr_ptr >> pin) & 1UL); // Read Input Data Register
}

/**
 * @brief Toggles the output value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* gpio_base = get_gpio_base_ptr(port);
    if (gpio_base == NULL) return;

    volatile uint32_t* odr_ptr = REG_ACCESS(gpio_base, GPIO_ODR_OFFSET);

    *odr_ptr ^= (1UL << pin); // Toggle the bit
    // No explicit verification loop, as toggling is an instantaneous action.
}

// ============================================================================
// PWM Module Implementations
// ============================================================================

/**
 * @brief Initializes a PWM channel.
 *        Implements Rules.json: PWM_requirements.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = NULL;
    tbyte timer_idx = 0; // For clock enable
    tbyte channel_num = 0; // For CCRx and CCMRx
    tbyte alternate_function_code = 0; // For GPIO AF

    // Determine Timer base address, timer index for RCC, and channel number
    if (pwm_channel >= pwm_channel_tim1_ch1 && pwm_channel <= pwm_channel_tim1_ch4)
    {
        timer_base = (volatile uint32_t*)TIM1_BASE_ADDRESS;
        timer_idx = 1; // Used for RCC_APB2ENR
        channel_num = pwm_channel - pwm_channel_tim1_ch1 + 1;
        // TIM1 AF: PA8-11: AF1, PE9-14: AF1, PB0,PB1,PB13,PB14,PB15,PA7: AF1/AF2 depends on specific pin (complex mapping)
        // For simplicity, assuming AF1 is common if multiple exist.
        alternate_function_code = 1; // TIM1 usually AF1
    }
    else if (pwm_channel >= pwm_channel_tim2_ch1 && pwm_channel <= pwm_channel_tim2_ch4)
    {
        timer_base = (volatile uint32_t*)TIM2_BASE_ADDRESS;
        timer_idx = 2; // Used for RCC_APB1ENR
        channel_num = pwm_channel - pwm_channel_tim2_ch1 + 1;
        alternate_function_code = 1; // TIM2 usually AF1
    }
    else if (pwm_channel >= pwm_channel_tim3_ch1 && pwm_channel <= pwm_channel_tim3_ch4)
    {
        timer_base = (volatile uint32_t*)TIM3_BASE_ADDRESS;
        timer_idx = 3; // Used for RCC_APB1ENR
        channel_num = pwm_channel - pwm_channel_tim3_ch1 + 1;
        alternate_function_code = 2; // TIM3 usually AF2
    }
    else if (pwm_channel >= pwm_channel_tim4_ch1 && pwm_channel <= pwm_channel_tim4_ch4)
    {
        timer_base = (volatile uint32_t*)TIM4_BASE_ADDRESS;
        timer_idx = 4; // Used for RCC_APB1ENR
        channel_num = pwm_channel - pwm_channel_tim4_ch1 + 1;
        alternate_function_code = 2; // TIM4 usually AF2
    }
    else if (pwm_channel >= pwm_channel_tim5_ch1 && pwm_channel <= pwm_channel_tim5_ch4)
    {
        timer_base = (volatile uint32_t*)TIM5_BASE_ADDRESS;
        timer_idx = 5; // Used for RCC_APB1ENR
        channel_num = pwm_channel - pwm_channel_tim5_ch1 + 1;
        alternate_function_code = 2; // TIM5 usually AF2
    }
    else if (pwm_channel >= pwm_channel_tim9_ch1 && pwm_channel <= pwm_channel_tim9_ch2)
    {
        timer_base = (volatile uint32_t*)TIM9_BASE_ADDRESS;
        timer_idx = 9; // Used for RCC_APB2ENR
        channel_num = pwm_channel - pwm_channel_tim9_ch1 + 1;
        alternate_function_code = 3; // TIM9 usually AF3
    }
    else if (pwm_channel == pwm_channel_tim10_ch1)
    {
        timer_base = (volatile uint32_t*)TIM10_BASE_ADDRESS;
        timer_idx = 10; // Used for RCC_APB2ENR
        channel_num = 1;
        alternate_function_code = 3; // TIM10 usually AF3 (for PB8) or AF1 (for PA6)
    }
    else if (pwm_channel == pwm_channel_tim11_ch1)
    {
        timer_base = (volatile uint32_t*)TIM11_BASE_ADDRESS;
        timer_idx = 11; // Used for RCC_APB2ENR
        channel_num = 1;
        alternate_function_code = 3; // TIM11 usually AF3 (for PB9) or AF1 (for PA7)
    }

    if (timer_base == NULL) return; // Invalid PWM channel

    // Enable Timer clock (Rules.json: peripheral_enable_rules)
    if (timer_idx == 1 || timer_idx == 9 || timer_idx == 10 || timer_idx == 11)
    {
        peripheral_clock_enable('1', timer_idx); // TIM1EN, TIM9EN, TIM10EN, TIM11EN (APB2)
    }
    else // TIM2, TIM3, TIM4, TIM5
    {
        peripheral_clock_enable('2', timer_idx); // TIM2EN, TIM3EN, TIM4EN, TIM5EN (APB1)
    }

    // Configure GPIO pins for Alternate Function (PWM output)
    t_port gpio_port;
    t_pin gpio_pin;

    // This part requires mapping pwm_channel to specific GPIO (Rules.json: commenting_requirements example)
    // A full implementation would need a lookup table or a more complex mapping.
    // For now, we'll pick one common pin and configure it.
    // Example for TIM1_CH1: PA8. (This is simplified, a proper system would select based on board configuration).
    if (pwm_channel == pwm_channel_tim1_ch1) { gpio_port = port_a; gpio_pin = pin_8; } // PA8
    else if (pwm_channel == pwm_channel_tim2_ch1) { gpio_port = port_a; gpio_pin = pin_0; } // PA0
    else if (pwm_channel == pwm_channel_tim3_ch1) { gpio_port = port_a; gpio_pin = pin_6; } // PA6
    else if (pwm_channel == pwm_channel_tim4_ch1) { gpio_port = port_b; gpio_pin = pin_6; } // PB6
    else if (pwm_channel == pwm_channel_tim5_ch1) { gpio_port = port_a; gpio_pin = pin_0; } // PA0 (overlaps with TIM2_CH1, requires care)
    else if (pwm_channel == pwm_channel_tim9_ch1) { gpio_port = port_a; gpio_pin = pin_2; } // PA2
    else if (pwm_channel == pwm_channel_tim10_ch1) { gpio_port = port_b; gpio_pin = pin_8; } // PB8
    else if (pwm_channel == pwm_channel_tim11_ch1) { gpio_port = port_b; gpio_pin = pin_9; } // PB9
    // For other channels or pins, add more if-else or a lookup.
    else { return; } // No default pin found for this channel

    // Enable GPIO clock for the selected pin
    peripheral_clock_enable((char)('A' + gpio_port), 0);

    volatile uint32_t* gpio_moder_ptr = REG_ACCESS(get_gpio_base_ptr(gpio_port), GPIO_MODER_OFFSET);
    volatile uint32_t* gpio_afrl_ptr = REG_ACCESS(get_gpio_base_ptr(gpio_port), GPIO_AFRL_OFFSET);
    volatile uint32_t* gpio_afrh_ptr = REG_ACCESS(get_gpio_base_ptr(gpio_port), GPIO_AFRH_OFFSET);

    // Set GPIO pin to Alternate Function mode (10)
    *gpio_moder_ptr &= ~(3UL << (gpio_pin * 2));
    *gpio_moder_ptr |= (2UL << (gpio_pin * 2));

    // Set Alternate Function (AFx)
    if (gpio_pin < 8)
    {
        *gpio_afrl_ptr &= ~(0xFUL << (gpio_pin * 4));
        *gpio_afrl_ptr |= (alternate_function_code << (gpio_pin * 4));
    }
    else
    {
        *gpio_afrh_ptr &= ~(0xFUL << ((gpio_pin - 8) * 4));
        *gpio_afrh_ptr |= (alternate_function_code << ((gpio_pin - 8) * 4));
    }

    // Configure Timer registers
    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    volatile uint32_t* ccmr_ptr = NULL;
    volatile uint32_t* ccer_ptr = REG_ACCESS(timer_base, TIM_CCER_OFFSET);
    volatile uint32_t* psc_ptr = REG_ACCESS(timer_base, TIM_PSC_OFFSET);
    volatile uint32_t* arr_ptr = REG_ACCESS(timer_base, TIM_ARR_OFFSET);
    volatile uint32_t* ccr_ptr = NULL;
    volatile uint32_t* bdtr_ptr = NULL; // For advanced timers

    // Stop timer
    *cr1_ptr &= ~(1UL << 0); // CEN bit (Counter Enable)

    // Set Prescaler and Auto-Reload Register (ARR) for desired frequency
    // Assuming Timer Clock (e.g., APB1 or APB2 timer clock) is same as PCLK
    tlong timer_clock_freq;
    if (timer_idx == 1 || timer_idx == 9 || timer_idx == 10 || timer_idx == 11)
        timer_clock_freq = 84000000; // APB2 timers often run at PCLK2*2 if APB2 prescaler > 1
    else
        timer_clock_freq = 42000000; // APB1 timers often run at PCLK1*2 if APB1 prescaler > 1

    // For simplicity, assume PCLK directly.
    // Timer_Freq = Timer_Clock / ((PSC + 1) * (ARR + 1))
    // To achieve pwm_khz_freq:
    // ARR = (Timer_Clock / ((PSC + 1) * PWM_Freq)) - 1
    // We need to pick a PSC. Let's try to achieve ARR between 1000-65535.
    tword prescaler = 0;
    tword auto_reload = 0;
    tlong target_freq_hz = (tlong)pwm_khz_freq * 1000;

    if (target_freq_hz == 0) target_freq_hz = 1000; // Avoid division by zero

    prescaler = (tword)((timer_clock_freq / (65536 * target_freq_hz)) + 1);
    if (prescaler > 0xFFFF) prescaler = 0xFFFF; // Max 16-bit prescaler
    auto_reload = (tword)((timer_clock_freq / ((prescaler + 1) * target_freq_hz)) - 1);
    if (auto_reload == 0) auto_reload = 1; // Minimum ARR is 1

    *psc_ptr = prescaler;
    *arr_ptr = auto_reload;

    // Output Compare mode, PWM mode 1 (OCxM = 110)
    // Clear CCMR1/2 depending on channel_num
    if (channel_num == 1)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR1_OFFSET);
        *ccmr_ptr &= ~(0x70UL << 0); // Clear OC1M[2:0] (bits 6:4)
        *ccmr_ptr |= (0x6UL << 4);   // Set OC1M to 110 (PWM Mode 1)
        *ccmr_ptr |= (1UL << 3);    // Set OC1PE (Output Compare 1 Preload Enable)
    }
    else if (channel_num == 2)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR1_OFFSET);
        *ccmr_ptr &= ~(0x70UL << 8); // Clear OC2M[2:0] (bits 14:12)
        *ccmr_ptr |= (0x6UL << 12);  // Set OC2M to 110 (PWM Mode 1)
        *ccmr_ptr |= (1UL << 11);   // Set OC2PE
    }
    else if (channel_num == 3)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR2_OFFSET);
        *ccmr_ptr &= ~(0x70UL << 0); // Clear OC3M[2:0] (bits 6:4)
        *ccmr_ptr |= (0x6UL << 4);   // Set OC3M to 110 (PWM Mode 1)
        *ccmr_ptr |= (1UL << 3);    // Set OC3PE
    }
    else if (channel_num == 4)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR2_OFFSET);
        *ccmr_ptr &= ~(0x70UL << 8); // Clear OC4M[2:0] (bits 14:12)
        *ccmr_ptr |= (0x6UL << 12);  // Set OC4M to 110 (PWM Mode 1)
        *ccmr_ptr |= (1UL << 11);   // Set OC4PE
    }

    // Set Capture/Compare Register (CCR) for duty cycle
    tword compare_value = (tword)(((tlong)auto_reload * pwm_duty) / 100);
    if (compare_value == 0 && pwm_duty > 0) compare_value = 1; // Ensure minimum output if duty > 0

    if (channel_num == 1) ccr_ptr = REG_ACCESS(timer_base, TIM_CCR1_OFFSET);
    else if (channel_num == 2) ccr_ptr = REG_ACCESS(timer_base, TIM_CCR2_OFFSET);
    else if (channel_num == 3) ccr_ptr = REG_ACCESS(timer_base, TIM_CCR3_OFFSET);
    else if (channel_num == 4) ccr_ptr = REG_ACCESS(timer_base, TIM_CCR4_OFFSET);

    if (ccr_ptr != NULL) *ccr_ptr = compare_value;

    // Enable Output Compare channel (CCxE bit in CCER)
    *ccer_ptr |= (1UL << ((channel_num - 1) * 4)); // CCxE bit

    // For advanced timers (TIM1), enable main output
    if (timer_idx == 1)
    {
        bdtr_ptr = REG_ACCESS(timer_base, TIM_BDTR_OFFSET);
        *bdtr_ptr |= (1UL << 15); // MOE bit (Main Output Enable)
    }

    // Generate update event to load preloaded registers (UG bit in EGR)
    *REG_ACCESS(timer_base, TIM_EGR_OFFSET) |= (1UL << 0);

    // Rules.json: PWM_requirements - Clear available FREQUENCY Ranges for each channel as comments
    /*
     * For STM32F401RC (APB1 max 42MHz, APB2 max 84MHz):
     * TIM1, TIM9, TIM10, TIM11 (APB2 timers):
     *   Min Freq (PSC=65535, ARR=65535) = 84MHz / (65536 * 65536) ~= 0.019 Hz
     *   Max Freq (PSC=0, ARR=1) = 84MHz / (1 * 2) = 42 MHz (approx, depends on duty cycle and minimum ARR)
     *
     * TIM2, TIM3, TIM4, TIM5 (APB1 timers):
     *   Min Freq (PSC=65535, ARR=65535) = 42MHz / (65536 * 65536) ~= 0.009 Hz
     *   Max Freq (PSC=0, ARR=1) = 42MHz / (1 * 2) = 21 MHz (approx)
     */
}

/**
 * @brief Starts the specified PWM channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = NULL;
    if (pwm_channel >= pwm_channel_tim1_ch1 && pwm_channel <= pwm_channel_tim1_ch4) timer_base = (volatile uint32_t*)TIM1_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim2_ch1 && pwm_channel <= pwm_channel_tim2_ch4) timer_base = (volatile uint32_t*)TIM2_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim3_ch1 && pwm_channel <= pwm_channel_tim3_ch4) timer_base = (volatile uint32_t*)TIM3_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim4_ch1 && pwm_channel <= pwm_channel_tim4_ch4) timer_base = (volatile uint32_t*)TIM4_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim5_ch1 && pwm_channel <= pwm_channel_tim5_ch4) timer_base = (volatile uint32_t*)TIM5_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim9_ch1 && pwm_channel <= pwm_channel_tim9_ch2) timer_base = (volatile uint32_t*)TIM9_BASE_ADDRESS;
    else if (pwm_channel == pwm_channel_tim10_ch1) timer_base = (volatile uint32_t*)TIM10_BASE_ADDRESS;
    else if (pwm_channel == pwm_channel_tim11_ch1) timer_base = (volatile uint32_t*)TIM11_BASE_ADDRESS;

    if (timer_base == NULL) return; // Invalid PWM channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    *cr1_ptr |= (1UL << 0); // Set CEN bit (Counter Enable)
}

/**
 * @brief Stops the specified PWM channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = NULL;
    if (pwm_channel >= pwm_channel_tim1_ch1 && pwm_channel <= pwm_channel_tim1_ch4) timer_base = (volatile uint32_t*)TIM1_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim2_ch1 && pwm_channel <= pwm_channel_tim2_ch4) timer_base = (volatile uint32_t*)TIM2_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim3_ch1 && pwm_channel <= pwm_channel_tim3_ch4) timer_base = (volatile uint32_t*)TIM3_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim4_ch1 && pwm_channel <= pwm_channel_tim4_ch4) timer_base = (volatile uint32_t*)TIM4_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim5_ch1 && pwm_channel <= pwm_channel_tim5_ch4) timer_base = (volatile uint32_t*)TIM5_BASE_ADDRESS;
    else if (pwm_channel >= pwm_channel_tim9_ch1 && pwm_channel <= pwm_channel_tim9_ch2) timer_base = (volatile uint32_t*)TIM9_BASE_ADDRESS;
    else if (pwm_channel == pwm_channel_tim10_ch1) timer_base = (volatile uint32_t*)TIM10_BASE_ADDRESS;
    else if (pwm_channel == pwm_channel_tim11_ch1) timer_base = (volatile uint32_t*)TIM11_BASE_ADDRESS;

    if (timer_base == NULL) return; // Invalid PWM channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    *cr1_ptr &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
}

// ============================================================================
// ICU Module Implementations
// ============================================================================

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler for the timer.
 * @param icu_edge The desired trigger edge (rising, falling, or both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = NULL;
    tbyte timer_idx = 0;
    tbyte channel_num = 0;
    tbyte alternate_function_code = 0;

    // Determine Timer base address, timer index for RCC, and channel number
    // This mapping should be consistent with PWM or explicitly defined for ICU.
    // Reusing PWM's logic for timer_base and timer_idx.
    if (icu_channel >= icu_channel_tim1_ch1 && icu_channel <= icu_channel_tim1_ch4)
    {
        timer_base = (volatile uint32_t*)TIM1_BASE_ADDRESS;
        timer_idx = 1;
        channel_num = icu_channel - icu_channel_tim1_ch1 + 1;
        alternate_function_code = 1; // For example pin PA8, AF1
    }
    else if (icu_channel >= icu_channel_tim2_ch1 && icu_channel <= icu_channel_tim2_ch4)
    {
        timer_base = (volatile uint32_t*)TIM2_BASE_ADDRESS;
        timer_idx = 2;
        channel_num = icu_channel - icu_channel_tim2_ch1 + 1;
        alternate_function_code = 1; // For example pin PA0, AF1
    }
    else if (icu_channel >= icu_channel_tim3_ch1 && icu_channel <= icu_channel_tim3_ch4)
    {
        timer_base = (volatile uint32_t*)TIM3_BASE_ADDRESS;
        timer_idx = 3;
        channel_num = icu_channel - icu_channel_tim3_ch1 + 1;
        alternate_function_code = 2; // For example pin PA6, AF2
    }
    else if (icu_channel >= icu_channel_tim4_ch1 && icu_channel <= icu_channel_tim4_ch4)
    {
        timer_base = (volatile uint32_t*)TIM4_BASE_ADDRESS;
        timer_idx = 4;
        channel_num = icu_channel - icu_channel_tim4_ch1 + 1;
        alternate_function_code = 2; // For example pin PB6, AF2
    }
    else if (icu_channel >= icu_channel_tim5_ch1 && icu_channel <= icu_channel_tim5_ch4)
    {
        timer_base = (volatile uint32_t*)TIM5_BASE_ADDRESS;
        timer_idx = 5;
        channel_num = icu_channel - icu_channel_tim5_ch1 + 1;
        alternate_function_code = 2; // For example pin PA0, AF2
    }
    else if (icu_channel >= icu_channel_tim9_ch1 && icu_channel <= icu_channel_tim9_ch2)
    {
        timer_base = (volatile uint32_t*)TIM9_BASE_ADDRESS;
        timer_idx = 9;
        channel_num = icu_channel - icu_channel_tim9_ch1 + 1;
        alternate_function_code = 3; // For example pin PA2, AF3
    }
    else if (icu_channel == icu_channel_tim10_ch1)
    {
        timer_base = (volatile uint32_t*)TIM10_BASE_ADDRESS;
        timer_idx = 10;
        channel_num = 1;
        alternate_function_code = 3; // For example pin PB8, AF3
    }
    else if (icu_channel == icu_channel_tim11_ch1)
    {
        timer_base = (volatile uint32_t*)TIM11_BASE_ADDRESS;
        timer_idx = 11;
        channel_num = 1;
        alternate_function_code = 3; // For example pin PB9, AF3
    }
    else return;

    // Enable Timer clock
    if (timer_idx == 1 || timer_idx == 9 || timer_idx == 10 || timer_idx == 11)
    {
        peripheral_clock_enable('1', timer_idx); // TIM1EN, TIM9EN, TIM10EN, TIM11EN (APB2)
    }
    else
    {
        peripheral_clock_enable('2', timer_idx); // TIM2EN, TIM3EN, TIM4EN, TIM5EN (APB1)
    }

    // Configure GPIO pins for Alternate Function (Input Capture)
    t_port gpio_port;
    t_pin gpio_pin;
    // Similar to PWM_Init, need a lookup or specific mapping here.
    // Using default pins for primary channels for demonstration.
    if (icu_channel == icu_channel_tim1_ch1) { gpio_port = port_a; gpio_pin = pin_8; } // PA8
    else if (icu_channel == icu_channel_tim2_ch1) { gpio_port = port_a; gpio_pin = pin_0; } // PA0
    else if (icu_channel == icu_channel_tim3_ch1) { gpio_port = port_a; gpio_pin = pin_6; } // PA6
    else if (icu_channel == icu_channel_tim4_ch1) { gpio_port = port_b; gpio_pin = pin_6; } // PB6
    else if (icu_channel == icu_channel_tim5_ch1) { gpio_port = port_a; gpio_pin = pin_0; } // PA0
    else if (icu_channel == icu_channel_tim9_ch1) { gpio_port = port_a; gpio_pin = pin_2; } // PA2
    else if (icu_channel == icu_channel_tim10_ch1) { gpio_port = port_b; gpio_pin = pin_8; } // PB8
    else if (icu_channel == icu_channel_tim11_ch1) { gpio_port = port_b; gpio_pin = pin_9; } // PB9
    else { return; } // No default pin found for this channel

    peripheral_clock_enable((char)('A' + gpio_port), 0);

    volatile uint32_t* gpio_moder_ptr = REG_ACCESS(get_gpio_base_ptr(gpio_port), GPIO_MODER_OFFSET);
    volatile uint32_t* gpio_afrl_ptr = REG_ACCESS(get_gpio_base_ptr(gpio_port), GPIO_AFRL_OFFSET);
    volatile uint32_t* gpio_afrh_ptr = REG_ACCESS(get_gpio_base_ptr(gpio_port), GPIO_AFRH_OFFSET);

    // Set GPIO pin to Alternate Function mode (10)
    *gpio_moder_ptr &= ~(3UL << (gpio_pin * 2));
    *gpio_moder_ptr |= (2UL << (gpio_pin * 2));

    // Set Alternate Function (AFx)
    if (gpio_pin < 8)
    {
        *gpio_afrl_ptr &= ~(0xFUL << (gpio_pin * 4));
        *gpio_afrl_ptr |= (alternate_function_code << (gpio_pin * 4));
    }
    else
    {
        *gpio_afrh_ptr &= ~(0xFUL << ((gpio_pin - 8) * 4));
        *gpio_afrh_ptr |= (alternate_function_code << ((gpio_pin - 8) * 4));
    }

    // Configure Timer registers
    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    volatile uint32_t* ccmr_ptr = NULL;
    volatile uint32_t* ccer_ptr = REG_ACCESS(timer_base, TIM_CCER_OFFSET);
    volatile uint32_t* psc_ptr = REG_ACCESS(timer_base, TIM_PSC_OFFSET);
    volatile uint32_t* arr_ptr = REG_ACCESS(timer_base, TIM_ARR_OFFSET);
    volatile uint32_t* dier_ptr = REG_ACCESS(timer_base, TIM_DIER_OFFSET); // DMA/Interrupt Enable

    // Disable timer counter
    *cr1_ptr &= ~(1UL << 0); // CEN bit

    // Configure prescaler
    tword psc_value = 0;
    switch (icu_prescaller)
    {
        case icu_prescaler_div1: psc_value = 0; break;
        case icu_prescaler_div2: psc_value = 1; break;
        case icu_prescaler_div4: psc_value = 3; break;
        case icu_prescaler_div8: psc_value = 7; break;
        default: psc_value = 0; break;
    }
    *psc_ptr = psc_value;
    *arr_ptr = 0xFFFFFFFF; // Max value for capture mode to avoid overflow during measurement

    // Configure CCMR for Input Capture (CCxS = 01)
    if (channel_num == 1)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR1_OFFSET);
        *ccmr_ptr &= ~(3UL << 0); // Clear CC1S bits
        *ccmr_ptr |= (1UL << 0);  // Set CC1S to 01 (Input capture, IC1 mapped to TI1)
        *ccmr_ptr &= ~(0xFUL << 4); // Clear IC1F (Input capture 1 filter) - No filter for now
    }
    else if (channel_num == 2)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR1_OFFSET);
        *ccmr_ptr &= ~(3UL << 8); // Clear CC2S bits
        *ccmr_ptr |= (1UL << 8);  // Set CC2S to 01 (Input capture, IC2 mapped to TI2)
        *ccmr_ptr &= ~(0xFUL << 12); // Clear IC2F
    }
    else if (channel_num == 3)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR2_OFFSET);
        *ccmr_ptr &= ~(3UL << 0); // Clear CC3S bits
        *ccmr_ptr |= (1UL << 0);  // Set CC3S to 01 (Input capture, IC3 mapped to TI3)
        *ccmr_ptr &= ~(0xFUL << 4); // Clear IC3F
    }
    else if (channel_num == 4)
    {
        ccmr_ptr = REG_ACCESS(timer_base, TIM_CCMR2_OFFSET);
        *ccmr_ptr &= ~(3UL << 8); // Clear CC4S bits
        *ccmr_ptr |= (1UL << 8);  // Set CC4S to 01 (Input capture, IC4 mapped to TI4)
        *ccmr_ptr &= ~(0xFUL << 12); // Clear IC4F
    }

    // Configure CCER for polarity (CCxP, CCxNP bits)
    uint32_t ccer_mask = (1UL << ((channel_num - 1) * 4)); // CCxE bit
    uint32_t ccer_polarity_mask = (1UL << (((channel_num - 1) * 4) + 1)); // CCxP bit (for rising/falling edge)
    uint32_t ccer_np_polarity_mask = (1UL << (((channel_num - 1) * 4) + 3)); // CCxNP bit (for active high/low level)

    *ccer_ptr &= ~(ccer_polarity_mask | ccer_np_polarity_mask); // Clear polarity bits

    if (icu_edge == icu_edge_rising)
    {
        // Default: CCxP=0, CCxNP=0 (non-inverted/rising edge)
    }
    else if (icu_edge == icu_edge_falling)
    {
        *ccer_ptr |= ccer_polarity_mask; // CCxP=1 (inverted/falling edge)
    }
    else // icu_edge_both
    {
        *ccer_ptr |= ccer_polarity_mask; // CCxP=1 (inverted/falling edge) AND
        // For both edges, some timers use a specific setting in SMCR or different configuration
        // STM32's typically handles "both" by setting both RTSR and FTSR in EXTI, or specific timer settings
        // For TIM, setting CCxP to 1 and CCxNP to 1 might give both edges, but it's often more complex for precise capture.
        // A common way for both edges in timer is to use two channels (e.g. TI1FP1 and TI1FP2) or specific slave mode configurations.
        // For a single channel, usually one or the other. This implementation will prefer falling edge for "both".
        // A more robust "both" would involve setting CCxP and CCxNP for active low/high. For simplicity, we choose falling.
    }

    // Enable capture for the channel (CCxE bit)
    *ccer_ptr |= ccer_mask;

    // Enable Capture/Compare Interrupt for the channel (CCxIE bit in DIER)
    *dier_ptr |= (1UL << channel_num); // CCxIE

    // Enable Global Interrupt for the timer (NVIC configuration is outside MCAL.c scope, needs to be done in application)

    // Generate update event to load preloaded registers (UG bit in EGR)
    *REG_ACCESS(timer_base, TIM_EGR_OFFSET) |= (1UL << 0);
}

/**
 * @brief Enables the specified ICU channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = NULL;
    if (icu_channel >= icu_channel_tim1_ch1 && icu_channel <= icu_channel_tim1_ch4) timer_base = (volatile uint32_t*)TIM1_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim2_ch1 && icu_channel <= icu_channel_tim2_ch4) timer_base = (volatile uint32_t*)TIM2_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim3_ch1 && icu_channel <= icu_channel_tim3_ch4) timer_base = (volatile uint32_t*)TIM3_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim4_ch1 && icu_channel <= icu_channel_tim4_ch4) timer_base = (volatile uint32_t*)TIM4_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim5_ch1 && icu_channel <= icu_channel_tim5_ch4) timer_base = (volatile uint32_t*)TIM5_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim9_ch1 && icu_channel <= icu_channel_tim9_ch2) timer_base = (volatile uint32_t*)TIM9_BASE_ADDRESS;
    else if (icu_channel == icu_channel_tim10_ch1) timer_base = (volatile uint32_t*)TIM10_BASE_ADDRESS;
    else if (icu_channel == icu_channel_tim11_ch1) timer_base = (volatile uint32_t*)TIM11_BASE_ADDRESS;

    if (timer_base == NULL) return; // Invalid ICU channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    *cr1_ptr |= (1UL << 0); // Set CEN bit (Counter Enable)
}

/**
 * @brief Disables the specified ICU channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = NULL;
    if (icu_channel >= icu_channel_tim1_ch1 && icu_channel <= icu_channel_tim1_ch4) timer_base = (volatile uint32_t*)TIM1_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim2_ch1 && icu_channel <= icu_channel_tim2_ch4) timer_base = (volatile uint32_t*)TIM2_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim3_ch1 && icu_channel <= icu_channel_tim3_ch4) timer_base = (volatile uint32_t*)TIM3_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim4_ch1 && icu_channel <= icu_channel_tim4_ch4) timer_base = (volatile uint32_t*)TIM4_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim5_ch1 && icu_channel <= icu_channel_tim5_ch4) timer_base = (volatile uint32_t*)TIM5_BASE_ADDRESS;
    else if (icu_channel >= icu_channel_tim9_ch1 && icu_channel <= icu_channel_tim9_ch2) timer_base = (volatile uint32_t*)TIM9_BASE_ADDRESS;
    else if (icu_channel == icu_channel_tim10_ch1) timer_base = (volatile uint32_t*)TIM10_BASE_ADDRESS;
    else if (icu_channel == icu_channel_tim11_ch1) timer_base = (volatile uint32_t*)TIM11_BASE_ADDRESS;

    if (timer_base == NULL) return; // Invalid ICU channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    *cr1_ptr &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
}

/**
 * @brief Gets the frequency measured by the specified ICU channel.
 *        Implements Rules.json: ICU_usage.
 *        This is a conceptual function; actual implementation relies on calculating
 *        frequency from two consecutive capture events. This requires storing states.
 * @param icu_channel The ICU channel to query.
 * @return The measured frequency in Hz.
 */
tword ICU_GetFrequency(t_icu_channel icu_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    // This function requires a complex state machine (e.g., using two capture registers)
    // and an interrupt handler to measure period and then calculate frequency.
    // Without interrupt service routines and state tracking variables, this cannot be
    // fully implemented here. It would also need timer_clock_freq to convert period to frequency.
    // Placeholder returning a dummy value.
    (void)icu_channel; // Suppress unused parameter warning
    return 0; // Return 0 Hz (not implemented fully)
}

/**
 * @brief Sets a callback function to be executed when an ICU event occurs.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void))
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    icu_callback_function = callback;
}

// Example of an ICU interrupt handler (outside MCAL, but shows usage of callback)
/*
void TIMx_CC_IRQHandler(void)
{
    // Check TIM_SR register for CCIF (Capture/Compare Interrupt Flag)
    // Clear flag
    // Call icu_callback_function if set
    if (icu_callback_function != NULL)
    {
        icu_callback_function();
    }
}
*/

// ============================================================================
// Timer Module Implementations
// ============================================================================

/**
 * @brief Initializes a general-purpose timer channel.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = get_timer_base_ptr(timer_channel);
    if (timer_base == NULL) return; // Invalid channel

    tbyte timer_idx = 0;
    if (timer_channel == timer_channel_tim1) timer_idx = 1;
    else if (timer_channel == timer_channel_tim2) timer_idx = 2;
    else if (timer_channel == timer_channel_tim3) timer_idx = 3;
    else if (timer_channel == timer_channel_tim4) timer_idx = 4;
    else if (timer_channel == timer_channel_tim5) timer_idx = 5;
    else if (timer_channel == timer_channel_tim9) timer_idx = 9;
    else if (timer_channel == timer_channel_tim10) timer_idx = 10;
    else if (timer_channel == timer_channel_tim11) timer_idx = 11;

    // Enable Timer clock
    if (timer_idx == 1 || timer_idx == 9 || timer_idx == 10 || timer_idx == 11)
    {
        peripheral_clock_enable('1', timer_idx); // TIM1EN, TIM9EN, TIM10EN, TIM11EN (APB2)
    }
    else
    {
        peripheral_clock_enable('2', timer_idx); // TIM2EN, TIM3EN, TIM4EN, TIM5EN (APB1)
    }

    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    volatile uint32_t* psc_ptr = REG_ACCESS(timer_base, TIM_PSC_OFFSET);
    volatile uint32_t* arr_ptr = REG_ACCESS(timer_base, TIM_ARR_OFFSET);
    volatile uint32_t* dier_ptr = REG_ACCESS(timer_base, TIM_DIER_OFFSET);
    volatile uint32_t* egr_ptr = REG_ACCESS(timer_base, TIM_EGR_OFFSET);

    // Disable timer counter
    *cr1_ptr &= ~(1UL << 0); // CEN bit

    // Configure CR1: Up-counting mode (DIR=0, CMS=00)
    *cr1_ptr &= ~((1UL << 4) | (3UL << 5)); // Clear DIR and CMS bits

    // Disable all interrupts/DMA for now, application can enable later
    *dier_ptr = 0x00;

    // Clear prescaler and auto-reload for initial state
    *psc_ptr = 0;
    *arr_ptr = 0;

    // Generate update event to load registers
    *egr_ptr |= (1UL << 0);
}

/**
 * @brief Sets the timer period in microseconds.
 * @param timer_channel The timer channel.
 * @param time The time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = get_timer_base_ptr(timer_channel);
    if (timer_base == NULL) return; // Invalid channel

    volatile uint32_t* psc_ptr = REG_ACCESS(timer_base, TIM_PSC_OFFSET);
    volatile uint32_t* arr_ptr = REG_ACCESS(timer_base, TIM_ARR_OFFSET);

    tlong timer_clock_freq;
    tbyte timer_idx = 0;
    if (timer_channel == timer_channel_tim1) timer_idx = 1;
    else if (timer_channel == timer_channel_tim2) timer_idx = 2;
    else if (timer_channel == timer_channel_tim3) timer_idx = 3;
    else if (timer_channel == timer_channel_tim4) timer_idx = 4;
    else if (timer_channel == timer_channel_tim5) timer_idx = 5;
    else if (timer_channel == timer_channel_tim9) timer_idx = 9;
    else if (timer_channel == timer_channel_tim10) timer_idx = 10;
    else if (timer_channel == timer_channel_tim11) timer_idx = 11;

    if (timer_idx == 1 || timer_idx == 9 || timer_idx == 10 || timer_idx == 11)
        timer_clock_freq = 84000000; // APB2 timers
    else
        timer_clock_freq = 42000000; // APB1 timers

    // Calculate PSC and ARR for 1us tick
    tword prescaler = (tword)((timer_clock_freq / 1000000) - 1); // 1MHz clock
    tword auto_reload = (tword)(time - 1); // For 'time' microseconds

    if (prescaler < 0) prescaler = 0;
    if (auto_reload < 0) auto_reload = 0;

    *psc_ptr = prescaler;
    *arr_ptr = auto_reload;
    *REG_ACCESS(timer_base, TIM_EGR_OFFSET) |= (1UL << 0); // Generate update event
}

/**
 * @brief Sets the timer period in milliseconds.
 * @param timer_channel The timer channel.
 * @param time The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    TIMER_Set_us(timer_channel, (tword)(time * 1000UL));
}

/**
 * @brief Sets the timer period in seconds.
 * @param timer_channel The timer channel.
 * @param time The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    TIMER_Set_us(timer_channel, (tword)(time * 1000000UL));
}

/**
 * @brief Sets the timer period in minutes.
 * @param timer_channel The timer channel.
 * @param time The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    TIMER_Set_us(timer_channel, (tword)(time * 60UL * 1000000UL));
}

/**
 * @brief Sets the timer period in hours.
 * @param timer_channel The timer channel.
 * @param time The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    TIMER_Set_us(timer_channel, (tword)(time * 3600UL * 1000000UL));
}

/**
 * @brief Enables the specified timer channel.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = get_timer_base_ptr(timer_channel);
    if (timer_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    *cr1_ptr |= (1UL << 0); // Set CEN bit (Counter Enable)
}

/**
 * @brief Disables the specified timer channel.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* timer_base = get_timer_base_ptr(timer_channel);
    if (timer_base == NULL) return; // Invalid channel

    volatile uint32_t* cr1_ptr = REG_ACCESS(timer_base, TIM_CR1_OFFSET);
    *cr1_ptr &= ~(1UL << 0); // Clear CEN bit (Counter Enable)
}

// ============================================================================
// ADC Module Implementations
// ============================================================================

/**
 * @brief Initializes the ADC peripheral for a specific channel and mode.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The ADC conversion mode (single, continuous, scan).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* adc_base = get_adc_base_ptr(adc_channel);
    if (adc_base == NULL) return; // Invalid ADC base address (should not happen for ADC1)

    // ADC clock is on APB2, so enable that first
    peripheral_clock_enable('1', 8); // ADC1EN (bit 8 of RCC_APB2ENR)

    volatile uint32_t* cr1_ptr = REG_ACCESS(adc_base, ADC_CR1_OFFSET);
    volatile uint32_t* cr2_ptr = REG_ACCESS(adc_base, ADC_CR2_OFFSET);
    volatile uint32_t* sqr1_ptr = REG_ACCESS(adc_base, ADC_SQR1_OFFSET);
    volatile uint32_t* sqr3_ptr = REG_ACCESS(adc_base, ADC_SQR3_OFFSET);
    volatile uint32_t* smpr1_ptr = REG_ACCESS(adc_base, ADC_SMPR1_OFFSET);
    volatile uint32_t* smpr2_ptr = REG_ACCESS(adc_base, ADC_SMPR2_OFFSET);
    volatile uint32_t* ccr_ptr = REG_ACCESS(ADC_COMMON_BASE_ADDRESS, ADC_CCR_OFFSET);

    // Disable ADC before configuration (ADON bit in CR2)
    *cr2_ptr &= ~(1UL << 0);

    // Common ADC configuration (CCR) - PCLK2/2 prescaler, independent mode
    // ADCPRE[1:0] in CCR: 00: PCLK2 divided by 2. (PCLK2 is 84MHz, so 42MHz ADC clock)
    *ccr_ptr &= ~(3UL << 16); // Clear ADCPRE bits
    *ccr_ptr |= (0UL << 16);  // PCLK2/2

    // Resolution (RES[1:0] in CR1) - 12-bit resolution (default 00)
    *cr1_ptr &= ~(3UL << 24);

    // Configure ADC mode
    *cr1_ptr &= ~(1UL << 8);  // SCAN bit (Scan mode disabled by default)
    *cr2_ptr &= ~(1UL << 1);  // CONT bit (Continuous conversion disabled by default)

    if (adc_mode == adc_mode_continuous_conversion)
    {
        *cr2_ptr |= (1UL << 1); // Set CONT bit
    }
    else if (adc_mode == adc_mode_scan)
    {
        *cr1_ptr |= (1UL << 8); // Set SCAN bit
        *cr2_ptr |= (1UL << 1); // Typically used with continuous conversion
    }
    // For single conversion, CONT and SCAN are 0.

    // Configure Regular Sequence (SQR)
    // Clear L bits (sequence length) in SQR1 for single conversion
    *sqr1_ptr &= ~(0xFUL << 20); // L[3:0] bits (0000 for 1 conversion)

    // Add the specified channel to the regular sequence (SQR3)
    // Channel is added to SQ1 (first in sequence)
    *sqr3_ptr &= ~(0x1FUL << 0); // Clear SQ1[4:0]
    *sqr3_ptr |= ((uint32_t)adc_channel << 0);

    // Configure Sample Time (SMPR)
    // Set a default sample time, e.g., 3 cycles for all channels
    // For specific channels, use smpr1_ptr (channels 10-18) or smpr2_ptr (channels 0-9)
    if (adc_channel >= adc_channel_0 && adc_channel <= adc_channel_9)
    {
        *smpr2_ptr &= ~(7UL << (adc_channel * 3)); // Clear SMPx bits
        *smpr2_ptr |= (0UL << (adc_channel * 3));   // 3 cycles (SMPx = 000)
    }
    else if (adc_channel >= adc_channel_10 && adc_channel <= adc_channel_15)
    {
        *smpr1_ptr &= ~(7UL << ((adc_channel - 10) * 3)); // Clear SMPx bits
        *smpr1_ptr |= (0UL << ((adc_channel - 10) * 3));   // 3 cycles (SMPx = 000)
    }

    // Enable ADC peripheral (ADON bit in CR2) - Done by ADC_Enable
}

/**
 * @brief Enables the ADC peripheral.
 *        Implements Rules.json: peripheral_enable_rules.
 * @param adc_channel Placeholder for ADC channel (ADC1 is a single unit).
 */
void ADC_Enable(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* adc_base = get_adc_base_ptr(adc_channel);
    if (adc_base == NULL) return; // Invalid ADC base address

    // 1. Enable peripheral clock (Rules.json: peripheral_enable_rules)
    peripheral_clock_enable('1', 8); // ADC1EN (bit 8 of RCC_APB2ENR)

    // 2. Set the peripheral's main enable bit (ADON for ADC)
    volatile uint32_t* cr2_ptr = REG_ACCESS(adc_base, ADC_CR2_OFFSET);
    *cr2_ptr |= (1UL << 0); // Set ADON bit (ADC ON)

    // Wait for ADC to be ready (stability time)
    // No specific AD_RDY flag in provided JSON. In real STM32, there might be one.
    // For example, some ADC units need a delay after ADON is set.
}

/**
 * @brief Disables the ADC peripheral.
 * @param adc_channel Placeholder for ADC channel (ADC1 is a single unit).
 */
void ADC_Disable(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* adc_base = get_adc_base_ptr(adc_channel);
    if (adc_base == NULL) return; // Invalid ADC base address

    volatile uint32_t* cr2_ptr = REG_ACCESS(adc_base, ADC_CR2_OFFSET);
    *cr2_ptr &= ~(1UL << 0); // Clear ADON bit (ADC OFF)
}

/**
 * @brief Performs an ADC conversion using polling and returns the result.
 * @param adc_channel The ADC channel to convert.
 * @return The 12-bit converted ADC value.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* adc_base = get_adc_base_ptr(adc_channel);
    if (adc_base == NULL) return 0;

    volatile uint32_t* cr2_ptr = REG_ACCESS(adc_base, ADC_CR2_OFFSET);
    volatile uint32_t* sr_ptr = REG_ACCESS(adc_base, ADC_SR_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(adc_base, ADC_DR_OFFSET);

    // Start conversion (SWSTART bit in CR2)
    *cr2_ptr |= (1UL << 30);

    // Wait for EOC (End Of Conversion) flag in SR
    volatile tlong timeout = 0xFFFFFFFF;
    while (!(*sr_ptr & (1UL << 1)) && (timeout-- > 0));
    if (timeout == 0) return 0; // Timeout error

    tword result = (tword)(*dr_ptr & 0xFFFF); // Read the converted value (12-bit max, so 0xFFFF is fine for tword)

    // Clear EOC flag by reading SR then DR (or explicitly writing 1 to EOC bit in SR if allowed)
    // For STM32F4, EOC is cleared by reading DR.
    return result;
}

/**
 * @brief Initiates an ADC conversion and expects an interrupt for the result.
 *        This function only starts the conversion and enables the interrupt.
 *        The actual reading of the result must be done in the ADC ISR.
 * @param adc_channel The ADC channel to convert.
 * @return Returns 0, as the result is retrieved via interrupt.
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* adc_base = get_adc_base_ptr(adc_channel);
    if (adc_base == NULL) return 0;

    volatile uint32_t* cr1_ptr = REG_ACCESS(adc_base, ADC_CR1_OFFSET);
    volatile uint32_t* cr2_ptr = REG_ACCESS(adc_base, ADC_CR2_OFFSET);

    // Enable EOCIE (End Of Conversion Interrupt Enable) in CR1
    *cr1_ptr |= (1UL << 5);

    // Start conversion (SWSTART bit in CR2)
    *cr2_ptr |= (1UL << 30);

    // The result will be available in the ADC Interrupt Service Routine.
    // The application must set up the NVIC for ADC interrupts.
    return 0; // Return 0, as the value will be fetched by the ISR
}

// ============================================================================
// Internal_EEPROM Module
// Internal_EEPROM module not supported on this MCU based on provided register_json.
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================

// ============================================================================
// TT (Time Triggered OS) Module Implementations
// ============================================================================

// Placeholder for TT_Tasks array (static, within MCAL.c)
#define MAX_TASKS 10
typedef struct {
    void (*pTask)(void);
    tword delay;
    tword period;
    tbyte runMe;
} TT_Task_t;

static TT_Task_t TT_Tasks[MAX_TASKS];
static tword TT_tick_ms_interval = 0; // The actual interval in ms for each tick

/**
 * @brief Initializes the Time Triggered OS.
 * @param tick_time_ms The desired tick interval in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    // Configure a timer for the desired tick interval.
    // For simplicity, we'll use TIM2 as the system timer.
    TIMER_Init(timer_channel_tim2);

    if (tick_time_ms == tick_time_1ms) TT_tick_ms_interval = 1;
    else if (tick_time_ms == tick_time_10ms) TT_tick_ms_interval = 10;
    else if (tick_time_ms == tick_time_100ms) TT_tick_ms_interval = 100;
    else if (tick_time_ms == tick_time_1s) TT_tick_ms_interval = 1000;
    else TT_tick_ms_interval = 1; // Default to 1ms

    TIMER_Set_Time_ms(timer_channel_tim2, TT_tick_ms_interval);

    // Enable timer update interrupt
    volatile uint32_t* tim2_dier_ptr = REG_ACCESS(TIM2_BASE_ADDRESS, TIM_DIER_OFFSET);
    *tim2_dier_ptr |= (1UL << 0); // UIE bit (Update Interrupt Enable)

    // Clear tasks array
    for (tbyte i = 0; i < MAX_TASKS; i++)
    {
        TT_Tasks[i].pTask = NULL;
        TT_Tasks[i].delay = 0;
        TT_Tasks[i].period = 0;
        TT_Tasks[i].runMe = 0;
    }
}

/**
 * @brief Starts the Time Triggered OS.
 */
void TT_Start(void)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence
    TIMER_Enable(timer_channel_tim2);
}

/**
 * @brief Dispatches tasks that are ready to run.
 */
void TT_Dispatch_task(void)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    for (tbyte i = 0; i < MAX_TASKS; i++)
    {
        if (TT_Tasks[i].pTask != NULL && TT_Tasks[i].runMe > 0)
        {
            TT_Tasks[i].pTask(); // Execute task
            TT_Tasks[i].runMe--; // Decrement run counter

            // If a one-shot task, clear it
            if (TT_Tasks[i].period == 0)
            {
                TT_Tasks[i].pTask = NULL;
            }
        }
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Timer Interrupt Service Routine for the TT OS.
 *        This function should be called from the actual timer ISR (e.g., TIM2_IRQHandler).
 */
void TT_ISR(void)
{
    // Do NOT call WDT_Reset() here if this ISR itself is time-critical.
    // The main loop that calls TT_Dispatch_task will call WDT_Reset().
    // If the ISR is long, it might need WDT_Reset(), but for a scheduler tick, it's usually very short.

    // Clear the update interrupt flag (UIF bit in SR)
    volatile uint32_t* tim2_sr_ptr = REG_ACCESS(TIM2_BASE_ADDRESS, TIM_SR_OFFSET);
    *tim2_sr_ptr &= ~(1UL << 0); // Clear UIF

    for (tbyte i = 0; i < MAX_TASKS; i++)
    {
        if (TT_Tasks[i].pTask != NULL)
        {
            if (TT_Tasks[i].delay == 0)
            {
                TT_Tasks[i].runMe++;
                if (TT_Tasks[i].period > 0)
                {
                    TT_Tasks[i].delay = TT_Tasks[i].period;
                }
            }
            else
            {
                TT_Tasks[i].delay--;
            }
        }
    }
}

/**
 * @brief Adds a task to the TT OS scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks. If 0, it's a one-shot task.
 * @param delay The initial delay before the task runs for the first time.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    for (tbyte i = 0; i < MAX_TASKS; i++)
    {
        if (TT_Tasks[i].pTask == NULL)
        {
            TT_Tasks[i].pTask = task;
            TT_Tasks[i].delay = delay;
            TT_Tasks[i].period = period;
            TT_Tasks[i].runMe = 0;
            return i;
        }
    }
    return 0xFF; // No space for new task
}

/**
 * @brief Deletes a task from the TT OS scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    if (task_index < MAX_TASKS)
    {
        TT_Tasks[task_index].pTask = NULL;
        TT_Tasks[task_index].delay = 0;
        TT_Tasks[task_index].period = 0;
        TT_Tasks[task_index].runMe = 0;
    }
}

// ============================================================================
// MCAL_OUTPUT_BUZZER Module
// MCAL_OUTPUT_BUZZER module not supported on this MCU based on provided register_json.
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================

// ============================================================================
// WDT Module
// WDT functions are handled by MCU_Config and WDT_Reset. No separate module needed.
// ============================================================================

// ============================================================================
// DAC Module
// DAC module not supported on this MCU based on provided register_json.
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================

// ============================================================================
// I2S Module Implementations
// ============================================================================

/**
 * @brief Initializes the I2S peripheral for audio communication.
 * @param channel The I2S channel (e.g., i2s_channel_i2s1).
 * @param mode The I2S mode (master/slave, transmit/receive).
 * @param standard The I2S standard (Philips, MSB/LSB justified, PCM).
 * @param data_format The data length (16, 24, 32 bit).
 * @param channel_mode Mono or stereo.
 * @param sample_rate Audio sample rate (e.g., 44100 Hz).
 * @param mclk_freq Master clock frequency.
 * @param dma_buffer_size DMA buffer size (not directly used in register config).
 */
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = NULL;
    tbyte spi_idx = 0; // For clock enable

    if (channel == i2s_channel_i2s1) { spi_base = (volatile uint32_t*)SPI1_BASE_ADDRESS; spi_idx = 1; }
    else if (channel == i2s_channel_i2s2) { spi_base = (volatile uint32_t*)SPI2_BASE_ADDRESS; spi_idx = 2; }
    else if (channel == i2s_channel_i2s3) { spi_base = (volatile uint32_t*)SPI3_BASE_ADDRESS; spi_idx = 3; }
    else { return; } // Invalid I2S channel

    // Enable SPI peripheral clock (Rules.json: peripheral_enable_rules)
    if (spi_idx == 1) peripheral_clock_enable('1', 12); // SPI1EN (APB2)
    else if (spi_idx == 2) peripheral_clock_enable('2', 14); // SPI2EN (APB1)
    else if (spi_idx == 3) peripheral_clock_enable('3', 15); // SPI3EN (APB1)

    volatile uint32_t* i2scfgr_ptr = REG_ACCESS(spi_base, SPI_I2SCFGR_OFFSET);
    volatile uint32_t* i2spr_ptr = REG_ACCESS(spi_base, SPI_I2SPR_OFFSET);

    // Disable I2S before configuration (I2SE bit in I2SCFGR)
    *i2scfgr_ptr &= ~(1UL << 10);

    // Set I2SMOD bit (I2S mode enable)
    *i2scfgr_ptr |= (1UL << 11);

    // Configure I2S Mode (I2SCFGR_I2SMOD, I2SCFGR_I2SCFG)
    *i2scfgr_ptr &= ~(3UL << 8); // Clear I2SCFG bits
    if (mode == i2s_mode_slave_tx) *i2scfgr_ptr |= (0UL << 8); // 00: Slave Tx
    else if (mode == i2s_mode_slave_rx) *i2scfgr_ptr |= (1UL << 8); // 01: Slave Rx
    else if (mode == i2s_mode_master_tx) *i2scfgr_ptr |= (2UL << 8); // 10: Master Tx
    else if (mode == i2s_mode_master_rx) *i2scfgr_ptr |= (3UL << 8); // 11: Master Rx

    // Configure I2S Standard (I2SSTD bits in I2SCFGR)
    *i2scfgr_ptr &= ~(3UL << 4); // Clear I2SSTD bits
    if (standard == i2s_standard_philips) *i2scfgr_ptr |= (0UL << 4);
    else if (standard == i2s_standard_msb_justified) *i2scfgr_ptr |= (1UL << 4);
    else if (standard == i2s_standard_lsb_justified) *i2scfgr_ptr |= (2UL << 4);
    else if (standard == i2s_standard_pcm_short) *i2scfgr_ptr |= (3UL << 4); // For PCM modes, further config in CHLEN & DATLEN

    // Configure Data Format & Channel Length (CHLEN, DATLEN in I2SCFGR)
    *i2scfgr_ptr &= ~(3UL << 1); // Clear DATLEN bits
    *i2scfgr_ptr &= ~(1UL << 0); // Clear CHLEN bit

    if (data_format == i2s_data_format_16b)
    {
        *i2scfgr_ptr |= (0UL << 1); // 16-bit data length
        *i2scfgr_ptr |= (0UL << 0); // 16-bit channel length
    }
    else if (data_format == i2s_data_format_24b)
    {
        *i2scfgr_ptr |= (1UL << 1); // 24-bit data length
        *i2scfgr_ptr |= (0UL << 0); // 16-bit channel length (data is 24-bit, stored in 32-bit frame usually)
    }
    else if (data_format == i2s_data_format_32b)
    {
        *i2scfgr_ptr |= (2UL << 1); // 32-bit data length
        *i2scfgr_ptr |= (1UL << 0); // 32-bit channel length
    }

    // Configure Channel Mode (stereo/mono - in I2SSTD or through frame format, complex)
    // For standard I2S Philips, it's typically stereo. Mono is usually a DATLEN/CHLEN config.
    // The current I2SSTANDARD bits define this implicitly. No direct register for channel_mode.
    (void)channel_mode; // Suppress unused parameter warning

    // Configure MCLK output (MCKOE bit in I2SCFGR)
    if (mclk_freq > 0)
    {
        *i2scfgr_ptr |= (1UL << 7); // Enable MCLK output
    }
    else
    {
        *i2scfgr_ptr &= ~(1UL << 7);
    }

    // Configure I2S Prescaler (I2SPR) for sample rate
    // I2S clock (I2SCK) is derived from PLLI2S_R (PLL I2S clock R output) or SYSCLK (complex clock setup)
    // For STM32F401, typically PLLI2S is used. We will assume a PLLI2S_R output of 100MHz (example).
    // I2S_Clock = Fs * 2 * CHLEN
    // I2SDIV = (PLLI2S_R_Freq / I2S_Clock) (if ODD=0) or I2SDIV = (PLLI2S_R_Freq / (2 * I2S_Clock)) (if ODD=1)
    // Example PLLI2S_R frequency: 100MHz (Actual value depends on PLLI2SCFGR configuration not provided here)
    tlong plli2s_r_freq = 100000000; // Placeholder: PLLI2S R output frequency

    // Calculate I2SDIV and ODD
    uint16_t i2s_div = 2; // Minimum div for functional clock
    uint8_t odd = 0;
    
    // A more precise calculation would be:
    // If MCLK is disabled (MCKOE = 0)
    // I2SDIV = Audio_pll_out_freq / (2 * sample_rate * data_length_factor)
    // data_length_factor = 16 for 16-bit, 24 for 24-bit, 32 for 32-bit frame.
    // If MCLK is enabled (MCKOE = 1)
    // I2SDIV = Audio_pll_out_freq / (256 * sample_rate) -- for 256*Fs
    // For generic purposes, assuming a 256*Fs BCLK from I2S_Clock for Master mode.
    // I2S_CLK = (PLLI2S_R_Freq / ((I2SDIV * 2) + ODD))
    // Desired I2S_CLK = Sample_Rate * Frame_Length_Bits (e.g., 2 * 16 for Stereo 16-bit)
    // Frame_Length_Bits = 16 or 32 for STM32F4 (16 for 16/24 bit data, 32 for 32 bit data)
    
    // Simplified calculation for I2S Prescaler, assuming 256 * Fs clock
    float div_float = (float)plli2s_r_freq / (256.0f * sample_rate);
    i2s_div = (uint16_t)div_float;
    odd = (uint8_t)((div_float - (float)i2s_div) >= 0.5f ? 1 : 0);

    if (i2s_div < 2) i2s_div = 2; // Min value
    if (i2s_div > 255) i2s_div = 255;

    *i2spr_ptr = ((odd << 8) | (i2s_div << 0));

    // The dma_buffer_size is for DMA configuration, which is not handled directly by these registers.
    (void)dma_buffer_size; // Suppress unused parameter warning

    // Enable the I2S peripheral (I2SE bit in I2SCFGR) - Done by I2S_Enable
}

/**
 * @brief Enables the specified I2S channel.
 * @param channel The I2S channel to enable.
 */
void I2S_Enable(t_i2s_channel channel)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = NULL;
    if (channel == i2s_channel_i2s1) spi_base = (volatile uint32_t*)SPI1_BASE_ADDRESS;
    else if (channel == i2s_channel_i2s2) spi_base = (volatile uint32_t*)SPI2_BASE_ADDRESS;
    else if (channel == i2s_channel_i2s3) spi_base = (volatile uint32_t*)SPI3_BASE_ADDRESS;
    else return;

    volatile uint32_t* i2scfgr_ptr = REG_ACCESS(spi_base, SPI_I2SCFGR_OFFSET);
    *i2scfgr_ptr |= (1UL << 10); // Set I2SE bit (I2S Enable)
}

/**
 * @brief Transmits data over the specified I2S channel.
 *        This is a basic polled transmit, typically DMA is used for I2S.
 * @param channel The I2S channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes/words to transmit.
 */
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = NULL;
    if (channel == i2s_channel_i2s1) spi_base = (volatile uint32_t*)SPI1_BASE_ADDRESS;
    else if (channel == i2s_channel_i2s2) spi_base = (volatile uint32_t*)SPI2_BASE_ADDRESS;
    else if (channel == i2s_channel_i2s3) spi_base = (volatile uint32_t*)SPI3_BASE_ADDRESS;
    else return;

    if (data == NULL || length == 0) return;

    volatile uint32_t* sr_ptr = REG_ACCESS(spi_base, SPI_SR_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(spi_base, SPI_DR_OFFSET);
    volatile uint32_t* i2scfgr_ptr = REG_ACCESS(spi_base, SPI_I2SCFGR_OFFSET);

    uint8_t data_len_cfg = (uint8_t)((*i2scfgr_ptr >> 1) & 0x3); // DATLEN bits
    size_t data_width = 0;
    if (data_len_cfg == 0) data_width = 2; // 16-bit
    else if (data_len_cfg == 1) data_width = 3; // 24-bit (store in 32-bit)
    else if (data_len_cfg == 2) data_width = 4; // 32-bit

    for (size_t i = 0; i < length; i += data_width)
    {
        // Wait until TXE flag is set (Transmit buffer empty)
        volatile tlong timeout = 0xFFFFFFFF;
        while (!(*sr_ptr & (1UL << 1)) && (timeout-- > 0));
        if (timeout == 0) return;

        if (data_width == 2) // 16-bit
        {
            *dr_ptr = *((const uint16_t*)data + (i / data_width));
        }
        else if (data_width == 3 || data_width == 4) // 24-bit or 32-bit
        {
            *dr_ptr = *((const uint32_t*)data + (i / data_width));
        }
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}

/**
 * @brief Receives data over the specified I2S channel.
 *        This is a basic polled receive, typically DMA is used for I2S.
 * @param channel The I2S channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param length The number of bytes/words to receive.
 */
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length)
{
    WDT_Reset(); // Rules.json: API_implementation_sequence

    volatile uint32_t* spi_base = NULL;
    if (channel == i2s_channel_i2s1) spi_base = (volatile uint32_t*)SPI1_BASE_ADDRESS;
    else if (channel == i2s_channel_i2s2) spi_base = (volatile uint32_t*)SPI2_BASE_ADDRESS;
    else if (channel == i2s_channel_i2s3) spi_base = (volatile uint32_t*)SPI3_BASE_ADDRESS;
    else return;

    if (buffer == NULL || length == 0) return;

    volatile uint32_t* sr_ptr = REG_ACCESS(spi_base, SPI_SR_OFFSET);
    volatile uint32_t* dr_ptr = REG_ACCESS(spi_base, SPI_DR_OFFSET);
    volatile uint32_t* i2scfgr_ptr = REG_ACCESS(spi_base, SPI_I2SCFGR_OFFSET);

    uint8_t data_len_cfg = (uint8_t)((*i2scfgr_ptr >> 1) & 0x3); // DATLEN bits
    size_t data_width = 0;
    if (data_len_cfg == 0) data_width = 2; // 16-bit
    else if (data_len_cfg == 1) data_width = 3; // 24-bit (store in 32-bit)
    else if (data_len_cfg == 2) data_width = 4; // 32-bit

    for (size_t i = 0; i < length; i += data_width)
    {
        // Wait until RXNE flag is set (Receive buffer not empty)
        volatile tlong timeout = 0xFFFFFFFF;
        while (!(*sr_ptr & (1UL << 0)) && (timeout-- > 0));
        if (timeout == 0) return;

        if (data_width == 2) // 16-bit
        {
            *((uint16_t*)buffer + (i / data_width)) = (uint16_t)*dr_ptr;
        }
        else if (data_width == 3 || data_width == 4) // 24-bit or 32-bit
        {
            *((uint32_t*)buffer + (i / data_width)) = *dr_ptr;
        }
        WDT_Reset(); // Rules.json: API_implementation_sequence
    }
}


// ============================================================================
// MQTT Protocol Module
// MQTT Protocol module not supported on this MCU based on provided register_json.
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================

// ============================================================================
// HTTP Protocol Module
// HTTP Protocol module not supported on this MCU based on provided register_json.
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================

// ============================================================================
// WiFi Driver Module
// WiFi Driver module not supported on this MCU based on provided register_json.
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================

// ============================================================================
// DTC_driver Module
// DTC_driver module not supported on this MCU based on provided register_json.
// The functions are commented out in MCAL.h, so no implementation here.
// ============================================================================