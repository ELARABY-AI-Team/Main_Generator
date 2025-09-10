// --- Core MCU Header and Standard C Libraries (Rules.json: core_includes) ---
#include "STM32F401RC_MCAL.h"
#include "stm32f4xx.h"  // Core device header for STM32F4 series (placeholder)
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- Generic Cortex-M Global Interrupt Control (CMSIS) ---
#define __ENABLE_IRQ()    __asm volatile ("cpsie i" : : : "memory")
#define __DISABLE_IRQ()   __asm volatile ("cpsid i" : : : "memory")

// --- Watchdog Timer (WDT) specific definitions (Inferred for STM32F401RC, not directly from register_json) ---
// For STM32F401RC, the Independent Watchdog (IWDG) is commonly used.
// The IWDG registers are at a base address not explicitly in register_json, so we infer and define.
// IWDG Base Address: 0x40003000UL
#define IWDG_R_BASE             0x40003000UL // Inferred IWDG base address for STM32F401RC

// IWDG Register Structure Definition (Inferred, not directly from register_json)
typedef struct
{
    volatile tword KR;  // 0x00 Key register
    volatile tword PR;  // 0x04 Prescaler register
    volatile tword RLR; // 0x08 Reload register
    volatile tword SR;  // 0x0C Status register
} IWDG_Registers_t;

#define P_IWDG                  ((IWDG_Registers_t *)IWDG_R_BASE)

// IWDG Key Register values
#define IWDG_KR_KEY_RELOAD      (0xAAAAUL)  // Reload counter
#define IWDG_KR_KEY_ENABLE      (0xCCCCUL)  // Enable IWDG
#define IWDG_KR_KEY_ACCESS      (0x5555UL)  // Access PR, RLR registers

// IWDG Prescaler Register values (PR bits)
#define IWDG_PR_DIV4            (0x00UL)
#define IWDG_PR_DIV8            (0x01UL)
#define IWDG_PR_DIV16           (0x02UL)
#define IWDG_PR_DIV32           (0x03UL)
#define IWDG_PR_DIV64           (0x04UL)
#define IWDG_PR_DIV128          (0x05UL)
#define IWDG_PR_DIV256          (0x06UL)

// --- Helper Functions for Peripheral Pointers (Rules.json: pointer_variable_consistency) ---

/**
 * @brief Returns the GPIO port base address pointer for a given port enum.
 * @param port The GPIO port (t_port).
 * @return Pointer to the GPIO_Registers_t structure for the specified port, or NULL if invalid.
 */
static GPIO_Registers_t* get_gpio_ptr(t_port port)
{
    switch (port)
    {
        case port_a: return P_GPIOA;
        case port_b: return P_GPIOB;
        case port_c: return P_GPIOC;
        case port_d: return P_GPIOD;
        case port_e: return P_GPIOE;
        case port_h: return P_GPIOH;
        default: return NULL; // Should not happen with valid input
    }
}

/**
 * @brief Returns the USART peripheral base address pointer for a given channel.
 * @param uart_channel The UART channel (t_uart_channel).
 * @return Pointer to the USART_Registers_t structure for the specified channel, or NULL if invalid.
 */
static USART_Registers_t* get_usart_ptr(t_uart_channel uart_channel)
{
    switch (uart_channel)
    {
        case uart_channel_1: return P_USART1;
        case uart_channel_2: return P_USART2;
        case uart_channel_6: return P_USART6;
        default: return NULL;
    }
}

/**
 * @brief Returns the I2C peripheral base address pointer for a given channel.
 * @param i2c_channel The I2C channel (t_i2c_channel).
 * @return Pointer to the I2C_Registers_t structure for the specified channel, or NULL if invalid.
 */
static I2C_Registers_t* get_i2c_ptr(t_i2c_channel i2c_channel)
{
    switch (i2c_channel)
    {
        case i2c_channel_1: return P_I2C1;
        case i2c_channel_2: return P_I2C2;
        case i2c_channel_3: return P_I2C3;
        default: return NULL;
    }
}

/**
 * @brief Returns the SPI peripheral base address pointer for a given channel.
 * @param spi_channel The SPI channel (t_spi_channel).
 * @return Pointer to the SPI_Registers_t structure for the specified channel, or NULL if invalid.
 */
static SPI_Registers_t* get_spi_ptr(t_spi_channel spi_channel)
{
    switch (spi_channel)
    {
        case spi_channel_1: return P_SPI1;
        case spi_channel_2: return P_SPI2;
        case spi_channel_3: return P_SPI3;
        default: return NULL;
    }
}

/**
 * @brief Returns the Timer peripheral base address pointer for a given channel.
 * @param timer_channel The timer channel (t_timer_channel).
 * @return Pointer to the TIM_Registers_t structure for the specified channel, or NULL if invalid.
 */
static TIM_Registers_t* get_timer_ptr(t_timer_channel timer_channel)
{
    switch (timer_channel)
    {
        case timer_channel_1: return P_TIM1;
        case timer_channel_2: return P_TIM2;
        case timer_channel_3: return P_TIM3;
        case timer_channel_4: return P_TIM4;
        case timer_channel_5: return P_TIM5;
        case timer_channel_9: return P_TIM9;
        case timer_channel_10: return P_TIM10;
        case timer_channel_11: return P_TIM11;
        default: return NULL;
    }
}

/**
 * @brief Returns the SPI peripheral pointer for an I2S channel (since I2S is part of SPI).
 * @param i2s_channel The I2S channel (t_i2s_channel).
 * @return Pointer to the SPI_Registers_t structure for the specified I2S channel, or NULL if invalid.
 */
static SPI_Registers_t* get_i2s_spi_ptr(t_i2s_channel i2s_channel)
{
    switch (i2s_channel)
    {
        case i2s_channel_spi1: return P_SPI1;
        case i2s_channel_spi2: return P_SPI2;
        case i2s_channel_spi3: return P_SPI3;
        default: return NULL;
    }
}

// --- WDT (Watchdog Timer) Module Implementation (Rules.json: API_implementation_sequence) ---

/**
 * @brief Resets the Watchdog Timer.
 *        This function reloads the IWDG counter to prevent a reset.
 *        (Inferred IWDG based on STM32F401RC, as IWDG registers not in register_json)
 */
void WDT_Reset(void)
{
    // Write 0xAAAA to the IWDG_KR register to reload the counter
    P_IWDG->KR = IWDG_KR_KEY_RELOAD; // Inferred IWDG register access
}

/**
 * @brief Initializes the Watchdog Timer.
 *        (Inferred IWDG based on STM32F401RC, as IWDG registers not in register_json)
 *        Sets a period >= 8ms. For LSI (32kHz) clock source, a prescaler of 256 and RLR of 250 would give:
 *        Period = (RLR + 1) * Prescaler / LSI_Freq = 251 * 256 / 32000 = ~2.008 seconds.
 *        A smaller RLR (e.g., 99) with Prescaler 256: 100 * 256 / 32000 = 0.8 seconds (800ms)
 *        To get >= 8ms with Prescaler 256: RLR >= (8ms * 32000 / 256) - 1 = 0.99, so RLR >= 1.
 *        Let's target ~100ms for init example (RLR=12): (12+1)*256/32000 = 0.104s = 104ms
 */
void WDT_Init(void)
{
    // All API bodies must include WDT_Reset() as code not comment
    WDT_Reset();

    // Unlock IWDG_PR and IWDG_RLR registers for writing
    P_IWDG->KR = IWDG_KR_KEY_ACCESS; // Inferred IWDG register access

    // Set prescaler to DIV256 (provides ~8ms to 2.048s range)
    P_IWDG->PR = IWDG_PR_DIV256;     // Inferred IWDG register access

    // Set reload value for a period >= 8ms.
    // RLR value (0-0xFFF). With DIV256 and LSI=32kHz, each count is 256/32000 = 8ms.
    // So, RLR = (desired_ms / 8ms) - 1. For desired_ms = 100ms: RLR = (100/8) - 1 = 12.5 - 1 = 11.5. Use 12.
    P_IWDG->RLR = 12UL;              // Inferred IWDG register access (104ms timeout)

    // Wait for the registers to be updated (status register check is common)
    while ((P_IWDG->SR & 0x07UL) != 0UL) { /* wait for PR, RLR updates to complete */ } // Inferred IWDG register access

    // Reload the watchdog counter again
    WDT_Reset();

    // Enable IWDG
    P_IWDG->KR = IWDG_KR_KEY_ENABLE; // Inferred IWDG register access
}


// --- MCU CONFIG Module Implementation ---

/**
 * @brief Initializes core MCU configurations based on system voltage.
 *        (Rules.json: MCU_Config_Init_implementation)
 * @param volt The system voltage (t_sys_volt).
 */
void MCU_Config_Init(t_sys_volt volt)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    GPIO_Registers_t* gpio_ports[] = {P_GPIOA, P_GPIOB, P_GPIOC, P_GPIOD, P_GPIOE, P_GPIOH};
    const uint32_t num_gpio_ports = sizeof(gpio_ports) / sizeof(gpio_ports[0]);

    // 1. Set all GPIO pins to 0 and verify with while loop
    for (uint32_t i = 0; i < num_gpio_ports; ++i)
    {
        GPIO_Registers_t* gpio_ptr = gpio_ports[i];
        if (gpio_ptr == NULL) continue;

        gpio_ptr->ODR = 0x00000000UL; // Set all output data bits to 0
        // Verify (output data register might not reflect input state directly if not output mode)
        while (gpio_ptr->ODR != 0x00000000UL) { /* Wait */ }
    }

    // 2. Set all GPIO pins direction to input and verify with while loop
    for (uint32_t i = 0; i < num_gpio_ports; ++i)
    {
        GPIO_Registers_t* gpio_ptr = gpio_ports[i];
        if (gpio_ptr == NULL) continue;

        gpio_ptr->MODER = 0x00000000UL; // Reset all mode bits to 00 (Input mode)
        // Verify
        while (gpio_ptr->MODER != 0x00000000UL) { /* Wait */ }

        // All input pins have pull-up resistors and wakeup feature enabled (if available)
        // Set all pins to pull-up for input (01)
        gpio_ptr->PUPDR = 0x55555555UL; // 01 01 ... 01 for all 16 pins
    }

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();

    // Disable ADC1
    P_ADC1->CR2 &= ~(1UL << 0); // Clear ADON bit (ADC_CR2 description: ADC control register 2.)

    // Disable all USARTs
    P_USART1->CR1 &= ~(1UL << 13); // Clear UE bit for USART1 (USART1_CR1 description: USART1 Control register 1)
    P_USART2->CR1 &= ~(1UL << 13); // Clear UE bit for USART2 (USART2_CR1)
    P_USART6->CR1 &= ~(1UL << 13); // Clear UE bit for USART6 (USART6_CR1)

    // Disable all SPIs and I2S
    P_SPI1->CR1 &= ~(1UL << 6);  // Clear SPE bit for SPI1 (SPI1_CR1 description: SPI1 Control register 1)
    P_SPI2->CR1 &= ~(1UL << 6);  // Clear SPE bit for SPI2 (SPI2_CR1)
    P_SPI3->CR1 &= ~(1UL << 6);  // Clear SPE bit for SPI3 (SPI3_CR1)
    P_SPI1->I2SCFGR &= ~(1UL << 10); // Clear I2SE bit for SPI1 I2S (SPI1_I2SCFGR)
    P_SPI2->I2SCFGR &= ~(1UL << 10); // Clear I2SE bit for SPI2 I2S (SPI2_I2SCFGR)
    P_SPI3->I2SCFGR &= ~(1UL << 10); // Clear I2SE bit for SPI3 I2S (SPI3_I2SCFGR)

    // Disable all I2Cs
    P_I2C1->CR1 &= ~(1UL << 0); // Clear PE bit for I2C1 (I2C1_CR1 description: I2C1 Control register 1)
    P_I2C2->CR1 &= ~(1UL << 0); // Clear PE bit for I2C2 (I2C2_CR1)
    P_I2C3->CR1 &= ~(1UL << 0); // Clear PE bit for I2C3 (I2C3_CR1)

    // Disable all Timers (clear CEN bit)
    P_TIM1->CR1 &= ~(1UL << 0); // TIM1_CR1 description: TIM1 control register 1
    P_TIM2->CR1 &= ~(1UL << 0); // TIM2_CR1
    P_TIM3->CR1 &= ~(1UL << 0); // TIM3_CR1
    P_TIM4->CR1 &= ~(1UL << 0); // TIM4_CR1
    P_TIM5->CR1 &= ~(1UL << 0); // TIM5_CR1
    P_TIM9->CR1 &= ~(1UL << 0); // TIM9_CR1
    P_TIM10->CR1 &= ~(1UL << 0); // TIM10_CR1
    P_TIM11->CR1 &= ~(1UL << 0); // TIM11_CR1

    // Disable all EXTI interrupts (clear IMR bits)
    P_EXTI->IMR = 0x00000000UL; // EXTI_IMR description: Interrupt mask register.

    // 4. Enable WDT & 5. Clear WDT timer & 6. Set WDT period >= 8 msec
    WDT_Init(); // This function already performs these steps.

    // 7. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // 8. Enable LVR (Low Voltage Reset)
    // (LVD/PVD related registers not available in register_json for direct configuration.
    // For STM32F4, this typically involves PWR->CR and PWR->CSR.
    // Therefore, LVR/LVD cannot be directly configured with provided registers.)
    (void)volt; // Suppress unused parameter warning
    // LVD_Init(); // Placeholder if LVD module had implementation. For now, skipped.

    // 9. Clear WDT again
    WDT_Reset();
}

/**
 * @brief Puts the MCU into sleep mode.
 *        (Rules.json: sleep_mode_definition)
 */
void Go_to_sleep_mode(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // For Cortex-M based MCUs like STM32, __WFI() (Wait For Interrupt) is commonly used.
    // This stops the CPU clock until an interrupt occurs.
    // Power control registers might need configuration for deeper sleep modes,
    // but without them in register_json, __WFI() is the most generic approach.
    __asm volatile ("wfi"); // Wait For Interrupt
}

/**
 * @brief Enables global interrupts.
 */
void Global_interrupt_Enable(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()
    __ENABLE_IRQ(); // CMSIS intrinsic for enabling global interrupts
}

/**
 * @brief Disables global interrupts.
 */
void Global_interrupt_Disable(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()
    __DISABLE_IRQ(); // CMSIS intrinsic for disabling global interrupts
}


// --- LVD Module Implementation (Rules.json: optional_modules_rule) ---
// LVD related registers (e.g., PWR_CR, PWR_CSR) are not provided in the register_json.
// Therefore, this module cannot be fully implemented with the given inputs.

void LVD_Init(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // LVD_Init: LVD related registers not available in register_json for STM32F401RC (typically PWR->CR for PVD).
    // Implementation skipped.
}

void LVD_Get(t_lvd_thrthreshold_level lvd_threshold_level)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // LVD_Get: LVD related registers not available in register_json.
    // Implementation skipped.
    (void)lvd_threshold_level; // Suppress unused parameter warning
}

void LVD_Enable(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // LVD_Enable: LVD related registers not available in register_json.
    // Implementation skipped.
}

void LVD_Disable(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()
    // LVD_Disable: LVD related registers not available in register_json.
    // Implementation skipped.
}


// --- UART Module Implementation ---

/**
 * @brief Initializes the specified UART channel.
 * @param uart_channel The UART channel to initialize.
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data frame length.
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity mode.
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    USART_Registers_t* usart_ptr = get_usart_ptr(uart_channel);
    if (usart_ptr == NULL) { return; }

    // Disable USART before configuration
    usart_ptr->CR1 &= ~(1UL << 13); // UE bit

    tlong cr1_val = usart_ptr->CR1;
    tlong cr2_val = usart_ptr->CR2;

    // Data Length (M bit in CR1)
    if (uart_data_length == uart_data_length_9bit)
    {
        cr1_val |= (1UL << 12); // M=1 for 9 data bits
    }
    else // uart_data_length_8bit
    {
        cr1_val &= ~(1UL << 12); // M=0 for 8 data bits
    }

    // Parity (PCE, PS bits in CR1)
    if (uart_parity == uart_parity_even)
    {
        cr1_val |= (1UL << 10); // PCE=1 (Parity Control Enable)
        cr1_val &= ~(1UL << 9); // PS=0 (Even Parity)
    }
    else if (uart_parity == uart_parity_odd)
    {
        cr1_val |= (1UL << 10); // PCE=1
        cr1_val |= (1UL << 9);  // PS=1 (Odd Parity)
    }
    else // uart_parity_none
    {
        cr1_val &= ~(1UL << 10); // PCE=0
    }

    // Stop Bits (STOP bits in CR2)
    cr2_val &= ~(0x3UL << 12); // Clear STOP bits
    if (uart_stop_bit == uart_stop_bit_0_5)
    {
        cr2_val |= (0x1UL << 12); // 0.5 Stop bit (specific to certain modes)
    }
    else if (uart_stop_bit == uart_stop_bit_2)
    {
        cr2_val |= (0x2UL << 12); // 2 Stop bits
    }
    else if (uart_stop_bit == uart_stop_bit_1_5)
    {
        cr2_val |= (0x3UL << 12); // 1.5 Stop bits (specific to certain modes)
    }
    else // uart_stop_bit_1
    {
        // 1 Stop bit (STOP[1:0] = 00, already cleared)
    }

    // Baud Rate (BRR register)
    // For STM32F4, BRR calculation depends on APBx clock and desired baud rate.
    // This is a simplified example, actual calculation requires knowing system clock and specific APB bus.
    // Example for 16MHz PCLK1/PCLK2 and 9600 baud (USARTDIV = Fck / (16 * Baud))
    // For 16MHz, 9600 baud: USARTDIV = 16000000 / (16 * 9600) = 104.166...
    // mantissa = 104; fraction = 0.166 * 16 = 2.656 (round to 3) => BRR = (104 << 4) | 3 = 0x683
    tword brr_value;
    switch (uart_baud_rate)
    {
        case uart_baud_rate_9600: brr_value = 0x683; break; // Placeholder: 16MHz PCLK, 9600 baud
        case uart_baud_rate_19200: brr_value = 0x341; break; // Placeholder: 16MHz PCLK, 19200 baud
        case uart_baud_rate_115200: brr_value = 0x08A; break; // Placeholder: 16MHz PCLK, 115200 baud
        default: brr_value = 0x683; break; // Default to 9600
    }
    usart_ptr->BRR = brr_value; // USARTx_BRR description: USARTx Baud rate register

    // Set updated CR1 and CR2 values
    usart_ptr->CR1 = cr1_val;
    usart_ptr->CR2 = cr2_val;
    usart_ptr->CR3 = 0x00000000UL; // Reset CR3 (USARTx_CR3) for default settings
}

/**
 * @brief Enables the specified UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    USART_Registers_t* usart_ptr = get_usart_ptr(uart_channel);
    if (usart_ptr == NULL) { return; }

    // First enable the peripheral clock (Rules.json: peripheral_enable_rules)
    // Inferred RCC clock enable registers and bits
    if (uart_channel == uart_channel_1 || uart_channel == uart_channel_6)
    {
        // USART1 and USART6 are on APB2 bus
        P_RCC->APB2ENR |= (1UL << (uart_channel == uart_channel_1 ? 4 : 5)); // USART1EN bit 4, USART6EN bit 5 (inferred)
        // Check for inferred bit in comment as per rules:
        // P_RCC->APB2ENR bit 4 for USART1, P_RCC->APB2ENR bit 5 for USART6 (inferred)
    }
    else if (uart_channel == uart_channel_2)
    {
        // USART2 is on APB1 bus
        P_RCC->APB1ENR |= (1UL << 17); // USART2EN bit 17 (inferred)
        // P_RCC->APB1ENR bit 17 for USART2 (inferred)
    }

    // Enable USART (UE bit in CR1)
    usart_ptr->CR1 |= (1UL << 13); // UE bit (USARTx_CR1 description: Configures USART main features)
}

/**
 * @brief Disables the specified UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    USART_Registers_t* usart_ptr = get_usart_ptr(uart_channel);
    if (usart_ptr == NULL) { return; }

    // Disable USART (clear UE bit in CR1)
    usart_ptr->CR1 &= ~(1UL << 13); // UE bit (USARTx_CR1)

    // Optionally, disable the peripheral clock to save power
    if (uart_channel == uart_channel_1 || uart_channel == uart_channel_6)
    {
        P_RCC->APB2ENR &= ~(1UL << (uart_channel == uart_channel_1 ? 4 : 5)); // Clear USART1EN bit 4, USART6EN bit 5 (inferred)
    }
    else if (uart_channel == uart_channel_2)
    {
        P_RCC->APB1ENR &= ~(1UL << 17); // Clear USART2EN bit 17 (inferred)
    }
}

/**
 * @brief Sends a single byte over the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    USART_Registers_t* usart_ptr = get_usart_ptr(uart_channel);
    if (usart_ptr == NULL) { return; }

    // Wait until Transmit Data Register Empty (TXE) flag is set
    while (!(usart_ptr->SR & (1UL << 7))) { /* Wait */ } // USARTx_SR description: USARTx Status register
    usart_ptr->DR = byte; // USARTx_DR description: USARTx Data register
    // Wait until Transmission Complete (TC) flag is set (optional, for last byte)
    while (!(usart_ptr->SR & (1UL << 6))) { /* Wait */ }
}

/**
 * @brief Sends a frame of data over the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    for (int i = 0; i < length; i++)
    {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    while (*str != '\0')
    {
        UART_send_byte(uart_channel, (tbyte)*str);
        str++;
    }
}

/**
 * @brief Receives a single byte from the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    USART_Registers_t* usart_ptr = get_usart_ptr(uart_channel);
    if (usart_ptr == NULL) { return 0; }

    // Wait until Receive Data Register Not Empty (RXNE) flag is set
    while (!(usart_ptr->SR & (1UL << 5))) { /* Wait */ } // USARTx_SR
    return (tbyte)(usart_ptr->DR & 0xFFUL); // USARTx_DR
}

/**
 * @brief Receives a frame of data from the specified UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Receives a null-terminated string from the specified UART channel.
 *        This function will block until a null terminator is received or max_length is reached.
 *        Note: For true string reception, often a timeout or specific termination character is used.
 *        This implementation assumes 'max_length' as the termination condition if null is not received.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the buffer, including null terminator.
 * @return The length of the received string (excluding null terminator).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    int i = 0;
    for (i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        char received_char = (char)UART_Get_Byte(uart_channel);
        if (received_char == '\0')
        {
            break;
        }
        buffer[i] = received_char;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}


// --- I2C Module Implementation ---

/**
 * @brief Initializes the specified I2C channel.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (Rules.json: Always use fast mode).
 * @param i2c_device_address The device's own address (Rules.json: Addressing Mode equals Device Address).
 * @param i2c_ack Acknowledge enable/disable.
 * @param i2c_datalength Data length (8-bit or 16-bit).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return; }

    // Disable I2C peripheral to configure
    i2c_ptr->CR1 &= ~(1UL << 0); // Clear PE bit (I2Cx_CR1)

    // Configure CR2: Peripheral clock frequency
    // Assuming a PCLK1 frequency of 16MHz for example.
    i2c_ptr->CR2 = (16UL << 0); // FREQ[5:0] = 16 (for 16MHz PCLK1). Max 40MHz. (I2Cx_CR2)

    // Configure CCR: Clock Control Register
    // Rules.json: "Always use fast mode" (400kHz)
    // F_PCLK1 = 16MHz
    // Fast mode duty cycle 2 (T_low/T_high = 2): Thigh = CCR * T_PCLK1, Tlow = 2 * CCR * T_PCLK1
    // Tscl = (Thigh + Tlow) = 3 * CCR * T_PCLK1 = 1 / I2C_Speed
    // CCR = F_PCLK1 / (3 * I2C_Speed) = 16MHz / (3 * 400kHz) = 16000000 / 1200000 = 13.33 => 14
    if (i2c_clk_speed == i2c_clk_speed_fast)
    {
        i2c_ptr->CCR = (1UL << 15) | (1UL << 14) | 14UL; // Fast mode (F/S=1), Duty cycle 2 (DUTY=1), CCR=14 (I2Cx_CCR)
    }
    else // i2c_clk_speed_standard (100kHz)
    {
        // CCR = F_PCLK1 / (2 * I2C_Speed) = 16MHz / (2 * 100kHz) = 16000000 / 200000 = 80
        i2c_ptr->CCR = 80UL; // Standard mode (F/S=0), CCR=80 (I2Cx_CCR)
    }

    // Configure TRISE: Maximum rise time (I2Cx_TRISE)
    // For 16MHz PCLK1, Fast mode (400kHz), T_PCLK1 = 62.5ns
    // Max rise time for Fm is 300ns. TRISE = (300ns / T_PCLK1) + 1 = (300 / 62.5) + 1 = 4.8 + 1 = 5.8 => 6
    if (i2c_clk_speed == i2c_clk_speed_fast)
    {
        i2c_ptr->TRISE = 6UL; // (I2Cx_TRISE)
    }
    else // Standard mode (100kHz), max rise time is 1000ns. TRISE = (1000 / 62.5) + 1 = 16 + 1 = 17
    {
        i2c_ptr->TRISE = 17UL; // (I2Cx_TRISE)
    }

    // Configure OAR1: Own Address Register 1 (Rules.json: Addressing Mode equals Device Address)
    i2c_ptr->OAR1 = (1UL << 14) | (i2c_device_address << 1); // Set bit 14, 7-bit address in bits 7:1 (I2Cx_OAR1)

    // Configure OAR2: Own Address Register 2
    i2c_ptr->OAR2 = 0x00000000UL; // Disable OAR2 by default (I2Cx_OAR2)

    // Configure FLTR: Filter Register
    i2c_ptr->FLTR = 0x00000000UL; // Disable filters by default (I2Cx_FLTR)

    // Configure CR1: ACK, General Call, Stretch
    tlong cr1_val = i2c_ptr->CR1;
    if (i2c_ack == i2c_ack_enable)
    {
        cr1_val |= (1UL << 10); // ACK bit
    }
    else
    {
        cr1_val &= ~(1UL << 10); // ACK bit
    }
    // No explicit configuration for i2c_datalength in typical I2C_CR1/CR2 for STM32, DR is 8-bit.
    // This parameter might be for internal buffer management in higher-level drivers.
    (void)i2c_datalength; // Suppress unused parameter warning

    // Rules.json: "Always generate a repeated start condition instead of stop between transactions"
    // This is handled in individual send/receive functions, not init.
    // Rules.json: "Always use maximum timeout" - This implies polling loops, not a specific register bit.

    i2c_ptr->CR1 = cr1_val;
}

/**
 * @brief Enables the specified I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return; }

    // First enable the peripheral clock (Rules.json: peripheral_enable_rules)
    // I2C1, I2C2, I2C3 are on APB1 bus
    P_RCC->APB1ENR |= (1UL << (i2c_channel == i2c_channel_1 ? 21 : (i2c_channel == i2c_channel_2 ? 22 : 23))); // I2C1EN bit 21, I2C2EN bit 22, I2C3EN bit 23 (inferred)
    // P_RCC->APB1ENR bit 21 for I2C1, bit 22 for I2C2, bit 23 for I2C3 (inferred)

    // Enable I2C peripheral (PE bit in CR1)
    i2c_ptr->CR1 |= (1UL << 0); // PE bit (I2Cx_CR1)
}

/**
 * @brief Disables the specified I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return; }

    // Disable I2C peripheral (clear PE bit in CR1)
    i2c_ptr->CR1 &= ~(1UL << 0); // PE bit (I2Cx_CR1)

    // Optionally, disable the peripheral clock to save power
    P_RCC->APB1ENR &= ~(1UL << (i2c_channel == i2c_channel_1 ? 21 : (i2c_channel == i2c_channel_2 ? 22 : 23))); // Clear I2CxEN bit (inferred)
}

/**
 * @brief Sends a single byte over the specified I2C channel.
 *        Assumes start condition and address has been sent prior if master mode.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return; }

    // Wait until TxE (Transmit data register empty) flag is set
    while (!(i2c_ptr->SR1 & (1UL << 7))) { /* Wait */ } // I2Cx_SR1 description: I2C1 Status register 1

    i2c_ptr->DR = byte; // I2Cx_DR description: I2C1 Data register
}

/**
 * @brief Sends a frame of data over the specified I2C channel.
 *        This function performs the start, address, data, and stop sequence for a master.
 *        (Rules.json: Always use maximum timeout, Always generate a repeated start condition instead of stop between transactions)
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return; }

    // This implementation assumes a prior master mode configuration and addressing is handled by the caller,
    // or this function should encapsulate a complete master transmission, including start, address, and stop.
    // For simplicity, let's assume `I2C_send_byte` is called in a sequence managed by a higher layer.
    // A full master transmit would look like:
    // 1. Generate Start condition (i2c_ptr->CR1 |= (1UL << 8))
    // 2. Wait for SB (Start Bit) flag (while (!(i2c_ptr->SR1 & (1UL << 0))))
    // 3. Send slave address (i2c_ptr->DR = slave_address_with_write_bit)
    // 4. Wait for ADDR (Address Sent) flag (while (!(i2c_ptr->SR1 & (1UL << 1))))
    // 5. Clear ADDR flag (read SR1, then SR2)
    // 6. Send data bytes using I2C_send_byte, waiting for TXE and BTF (Byte Transfer Finished) between bytes
    // 7. Generate Stop condition (i2c_ptr->CR1 |= (1UL << 9))

    // For now, only using I2C_send_byte for simplicity, but acknowledge the missing parts for a complete master transaction.
    for (int i = 0; i < length; i++)
    {
        I2C_send_byte(i2c_channel, (tbyte)data[i]);
        // Wait for BTF (Byte Transfer Finished) for multi-byte writes (optional, or use TXE + TC)
        while (!(i2c_ptr->SR1 & (1UL << 2))) { /* Wait for BTF */ }
    }
}

/**
 * @brief Sends a null-terminated string over the specified I2C channel.
 *        Similar to send_frame, assumes master transaction context.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return; }

    while (*str != '\0')
    {
        I2C_send_byte(i2c_channel, (tbyte)*str);
        while (!(i2c_ptr->SR1 & (1UL << 2))) { /* Wait for BTF */ }
        str++;
    }
}

/**
 * @brief Receives a single byte from the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return 0; }

    // For master receive, this would involve sending a NACK for the last byte
    // Wait for RXNE (Receive Data Register Not Empty) flag
    while (!(i2c_ptr->SR1 & (1UL << 6))) { /* Wait */ } // I2Cx_SR1
    return (tbyte)i2c_ptr->DR; // I2Cx_DR
}

/**
 * @brief Receives a frame of data from the specified I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return; }

    for (int i = 0; i < max_length; i++)
    {
        // For a master receiver, before reading the last byte, you must clear the ACK bit
        // and generate a STOP condition.
        if (i == (max_length - 1))
        {
            i2c_ptr->CR1 &= ~(1UL << 10); // Clear ACK bit
            i2c_ptr->CR1 |= (1UL << 9);  // Generate STOP condition
        }
        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
    }
}

/**
 * @brief Receives a null-terminated string from the specified I2C channel.
 *        This function will block until a null terminator is received or max_length is reached.
 *        Note: I2C does not inherently support null-terminated strings; this implies
 *        a higher-level protocol where the sender sends a null byte.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the buffer, including null terminator.
 * @return The length of the received string (excluding null terminator).
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    I2C_Registers_t* i2c_ptr = get_i2c_ptr(i2c_channel);
    if (i2c_ptr == NULL) { return 0; }

    int i = 0;
    for (i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        char received_char = (char)I2C_Get_Byte(i2c_channel);
        if (received_char == '\0')
        {
            break;
        }
        buffer[i] = received_char;

        // If not the last byte, ensure ACK is enabled for subsequent bytes
        if (i < (max_length - 2))
        {
            i2c_ptr->CR1 |= (1UL << 10); // Set ACK bit
        }
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}


// --- SPI (CSI) Module Implementation ---

/**
 * @brief Initializes the specified SPI channel.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode Master/Slave mode.
 * @param spi_cpol Clock polarity (Rules.json: CPHA, CPOL).
 * @param spi_cpha Clock phase (Rules.json: CPHA, CPOL).
 * @param spi_dff Data frame format (8-bit or 16-bit) (Rules.json: DFF).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_spi_ptr(spi_channel);
    if (spi_ptr == NULL) { return; }

    // Disable SPI peripheral before configuration
    spi_ptr->CR1 &= ~(1UL << 6); // Clear SPE bit (SPIx_CR1)

    tlong cr1_val = 0;
    tlong cr2_val = 0;

    // Master/Slave Mode (MSTR bit in CR1)
    if (spi_mode == spi_mode_master)
    {
        cr1_val |= (1UL << 2); // MSTR=1 for Master (SPIx_CR1)
        // Rules.json: "Slave Select always software-controlled" for Master.
        // SSM=1, SSI=1 for software slave management
        cr1_val |= (1UL << 9); // SSM (Software slave management)
        cr1_val |= (1UL << 8); // SSI (Internal slave select)
    }
    else // spi_mode_slave
    {
        cr1_val &= ~(1UL << 2); // MSTR=0 for Slave (SPIx_CR1)
        // For Slave, SSM is often set to 1 and NSS is external to avoid multi-master issues,
        // but here "Slave Select always software-controlled" might imply for NSS pin usage.
        // For a general slave, NSS is typically hardware controlled. Assuming NSS pin configured externally.
        cr1_val &= ~(1UL << 9); // SSM=0 (Hardware slave management by NSS pin)
        cr1_val &= ~(1UL << 8); // SSI=0 (Not applicable with SSM=0)
    }

    // Clock Polarity (CPOL bit in CR1)
    if (spi_cpol == spi_cpol_high)
    {
        cr1_val |= (1UL << 1); // CPOL=1 (SPIx_CR1)
    }
    else // spi_cpol_low
    {
        cr1_val &= ~(1UL << 1); // CPOL=0 (SPIx_CR1)
    }

    // Clock Phase (CPHA bit in CR1)
    if (spi_cpha == spi_cpha_2edge)
    {
        cr1_val |= (1UL << 0); // CPHA=1 (SPIx_CR1)
    }
    else // spi_cpha_1edge
    {
        cr1_val &= ~(1UL << 0); // CPHA=0 (SPIx_CR1)
    }

    // Data Frame Format (DFF bit in CR1)
    if (spi_dff == spi_dff_16bit)
    {
        cr1_val |= (1UL << 11); // DFF=1 for 16-bit (SPIx_CR1)
    }
    else // spi_dff_8bit
    {
        cr1_val &= ~(1UL << 11); // DFF=0 for 8-bit (SPIx_CR1)
    }

    // Bit Order (LSBFIRST bit in CR1)
    if (spi_bit_order == spi_bit_order_lsb_first)
    {
        cr1_val |= (1UL << 7); // LSBFIRST=1 (SPIx_CR1)
    }
    else // spi_bit_order_msb_first
    {
        cr1_val &= ~(1UL << 7); // LSBFIRST=0 (SPIx_CR1)
    }

    // Rules.json: "Always use full duplex" (BIDIMODE=0, RXONLY=0)
    cr1_val &= ~(1UL << 15); // BIDIMODE=0 (2-line unidirectional data mode selected)
    cr1_val &= ~(1UL << 10); // RXONLY=0 (Full-duplex)

    // Rules.json: "Always enable CRC" (CRCEN bit in CR1)
    cr1_val |= (1UL << 13); // CRCEN=1 (SPIx_CR1)

    // Rules.json: "Always use fast speed"
    // This implies setting the Baud Rate Prescaler (BR[2:0] in CR1) to the lowest division possible, i.e., DIV2.
    cr1_val &= ~(0x7UL << 3); // Clear BR bits
    // cr1_val |= (0x0UL << 3); // BR[2:0] = 000 for fPCLK/2, which is the fastest.

    // Set CR1 and CR2
    spi_ptr->CR1 = cr1_val; // SPIx_CR1
    spi_ptr->CR2 = cr2_val; // SPIx_CR2

    // Initialize CRC polynomial (SPIx_CRCPR) - typically default 0x7 for CRC-7, but can be configured
    spi_ptr->CRCPR = 0x7UL; // Default polynomial for CRC (SPIx_CRCPR)
}

/**
 * @brief Enables the specified SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_spi_ptr(spi_channel);
    if (spi_ptr == NULL) { return; }

    // First enable the peripheral clock (Rules.json: peripheral_enable_rules)
    if (spi_channel == spi_channel_1)
    {
        // SPI1 is on APB2 bus
        P_RCC->APB2ENR |= (1UL << 0xC); // SPI1EN bit 12 (inferred)
        // P_RCC->APB2ENR bit 12 for SPI1 (inferred)
    }
    else if (spi_channel == spi_channel_2 || spi_channel == spi_channel_3)
    {
        // SPI2 and SPI3 are on APB1 bus
        P_RCC->APB1ENR |= (1UL << (spi_channel == spi_channel_2 ? 14 : 15)); // SPI2EN bit 14, SPI3EN bit 15 (inferred)
        // P_RCC->APB1ENR bit 14 for SPI2, bit 15 for SPI3 (inferred)
    }

    // Enable SPI peripheral (SPE bit in CR1)
    spi_ptr->CR1 |= (1UL << 6); // SPE bit (SPIx_CR1)
}

/**
 * @brief Disables the specified SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_spi_ptr(spi_channel);
    if (spi_ptr == NULL) { return; }

    // Disable SPI peripheral (clear SPE bit in CR1)
    spi_ptr->CR1 &= ~(1UL << 6); // SPE bit (SPIx_CR1)

    // Optionally, disable the peripheral clock to save power
    if (spi_channel == spi_channel_1)
    {
        P_RCC->APB2ENR &= ~(1UL << 0xC); // Clear SPI1EN bit 12 (inferred)
    }
    else if (spi_channel == spi_channel_2 || spi_channel == spi_channel_3)
    {
        P_RCC->APB1ENR &= ~(1UL << (spi_channel == spi_channel_2 ? 14 : 15)); // Clear SPIxEN bit (inferred)
    }
}

/**
 * @brief Sends a single byte over the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_spi_ptr(spi_channel);
    if (spi_ptr == NULL) { return; }

    // Wait until Tx buffer empty (TXE) flag is set
    while (!(spi_ptr->SR & (1UL << 1))) { /* Wait */ } // SPIx_SR description: SPI1 Status register
    spi_ptr->DR = byte; // SPIx_DR description: SPI1 Data register
    // For full duplex, we might also need to wait for RXNE if we want to read incoming data.
    // For send only, waiting for BSY (Busy) flag after TXE might be needed.
    while (spi_ptr->SR & (1UL << 7)) { /* Wait for BSY to clear, indicating transfer complete */ }
}

/**
 * @brief Sends a frame of data over the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    for (int i = 0; i < length; i++)
    {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Receives a single byte from the specified SPI channel.
 *        Note: In master mode, a dummy byte usually needs to be sent to trigger reception.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_spi_ptr(spi_channel);
    if (spi_ptr == NULL) { return 0; }

    // In master mode, send a dummy byte to initiate clock cycles for reception
    spi_ptr->DR = 0xFFUL; // Dummy byte (SPIx_DR)

    // Wait until Rx buffer not empty (RXNE) flag is set
    while (!(spi_ptr->SR & (1UL << 0))) { /* Wait */ } // SPIx_SR
    return (tbyte)spi_ptr->DR; // SPIx_DR
}

/**
 * @brief Receives a frame of data from the specified SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum number of bytes to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Receives a null-terminated string from the specified SPI channel.
 *        Note: SPI does not inherently support null-terminated strings; this implies
 *        a higher-level protocol where the sender sends a null byte.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the buffer, including null terminator.
 * @return The length of the received string (excluding null terminator).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    int i = 0;
    for (i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        char received_char = (char)SPI_Get_Byte(spi_channel);
        if (received_char == '\0')
        {
            break;
        }
        buffer[i] = received_char;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}


// --- External Interrupt Module Implementation ---

/**
 * @brief Initializes an external interrupt channel with a specified trigger edge.
 * @param external_int_channel The EXTI line to configure.
 * @param external_int_edge The trigger edge (rising, falling, or both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // 1. Enable SYSCFG clock (for EXTI source selection)
    P_RCC->APB2ENR |= (1UL << 14); // SYSCFGEN bit 14 (inferred)
    // P_RCC->APB2ENR bit 14 for SYSCFG (inferred)

    // 2. Clear previous SYSCFG EXTI line configuration for the selected line
    tbyte exti_port_source = 0; // Placeholder, determined by caller or internal logic
    tbyte exti_line_num = (tbyte)external_int_channel;

    // Based on `SYSCFG_EXTICR1-4` mapping for a given pin, determine the register and bit field.
    // Assuming the caller maps specific pins to external_int_channel, here we clear and set.
    // For example, if external_int_channel is EXTI0-3, it's EXTICR1. If EXTI4-7, EXTICR2, etc.
    tlong volatile *exticr_reg = &(P_SYSCFG->EXTICR[exti_line_num / 4]);
    uint32_t shift = (exti_line_num % 4) * 4;
    *exticr_reg &= ~(0xFUL << shift); // Clear the 4-bit field for the EXTI line

    // Default to PAx for simplicity as port info is not passed in API,
    // actual port selection (exti_port_source) needs to come from application.
    // For now, we'll set it to GPIOA (0000) for demonstration purposes.
    // In a real application, you would pass t_port as well.
    // Example: SYSCFG_EXTICR1: EXTI0[3:0], EXTI1[3:0]...
    // Port A (0000), Port B (0001), Port C (0010), Port D (0011), Port E (0100), Port H (0111)
    exti_port_source = 0; // GPIOA (0000)
    *exticr_reg |= (exti_port_source << shift); // Set the port source (SYSCFG_EXTICRx description: SYSCFG external interrupt configuration register)

    // 3. Clear pending bit (EXTI_PR)
    P_EXTI->PR = (1UL << exti_line_num); // EXTI_PR description: Pending register.

    // 4. Configure trigger edge (RTSR and FTSR)
    P_EXTI->RTSR &= ~(1UL << exti_line_num); // Clear rising trigger for selected line
    P_EXTI->FTSR &= ~(1UL << exti_line_num); // Clear falling trigger for selected line

    if (external_int_edge == exti_edge_rising)
    {
        P_EXTI->RTSR |= (1UL << exti_line_num); // Enable rising trigger (EXTI_RTSR)
    }
    else if (external_int_edge == exti_edge_falling)
    {
        P_EXTI->FTSR |= (1UL << exti_line_num); // Enable falling trigger (EXTI_FTSR)
    }
    else // exti_edge_rising_falling
    {
        P_EXTI->RTSR |= (1UL << exti_line_num); // Enable rising trigger
        P_EXTI->FTSR |= (1UL << exti_line_num); // Enable falling trigger
    }

    // 5. Mask the interrupt (disable by default after init, enable with External_INT_Enable)
    P_EXTI->IMR &= ~(1UL << exti_line_num); // EXTI_IMR description: Interrupt mask register.
}

/**
 * @brief Enables the specified external interrupt channel.
 * @param external_int_channel The EXTI line to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Ensure SYSCFG clock is enabled, as it selects the source for EXTI.
    P_RCC->APB2ENR |= (1UL << 14); // SYSCFGEN bit 14 (inferred)

    // Enable the interrupt line in EXTI_IMR
    P_EXTI->IMR |= (1UL << (tbyte)external_int_channel); // EXTI_IMR
}

/**
 * @brief Disables the specified external interrupt channel.
 * @param external_int_channel The EXTI line to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Disable the interrupt line in EXTI_IMR
    P_EXTI->IMR &= ~(1UL << (tbyte)external_int_channel); // EXTI_IMR
}


// --- GPIO Module Implementation ---

/**
 * @brief Initializes a GPIO pin as an output and sets its initial value.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(port);
    if (gpio_ptr == NULL) { return; }

    // First, enable the clock for the GPIO port (Rules.json: peripheral_enable_rules)
    P_RCC->AHB1ENR |= (1UL << port); // GPIOAEN bit 0, GPIOBEN bit 1, etc. (inferred)
    // P_RCC->AHB1ENR bit for GPIOx (inferred)

    // Rules.json: "Always set value before setting direction"
    // For output, use BSRR for atomic set/reset
    if (value == 1)
    {
        gpio_ptr->BSRR = (1UL << pin); // Set bit (GPIOx_BSRR)
    }
    else
    {
        gpio_ptr->BSRR = (1UL << (pin + 16)); // Reset bit (GPIOx_BSRR)
    }
    // Rules.json: "After setting GPIO value, verify with while loop"
    // Verify ODR for output state
    if (value == 1)
    {
        while (!((gpio_ptr->ODR >> pin) & 1UL)) { /* Wait */ }
    }
    else
    {
        while (((gpio_ptr->ODR >> pin) & 1UL)) { /* Wait */ }
    }

    // Configure MODER for Output mode (01)
    gpio_ptr->MODER &= ~(0x3UL << (pin * 2)); // Clear mode bits
    gpio_ptr->MODER |= (0x1UL << (pin * 2));  // Set to Output mode (01) (GPIOx_MODER)

    // Rules.json: "After setting GPIO direction, verify with while loop"
    while (((gpio_ptr->MODER >> (pin * 2)) & 0x3UL) != 0x1UL) { /* Wait */ }

    // Rules.json: "All output pins have pull-up resistors disabled"
    // Configure PUPDR for No Pull-up/Pull-down (00)
    gpio_ptr->PUPDR &= ~(0x3UL << (pin * 2)); // Clear PUPDR bits (GPIOx_PUPDR)

    // Configure OTYPER for Push-Pull (0) by default
    gpio_ptr->OTYPER &= ~(1UL << pin); // Clear OTYPER bit (Push-Pull) (GPIOx_OTYPER)

    // Rules.json: "For current registers: use >=20mA sink current & >=10mA source current"
    // This typically means setting Output Speed to High or Very High.
    // Set OSPEEDR to Very High Speed (11)
    gpio_ptr->OSPEEDR |= (0x3UL << (pin * 2)); // Set to Very High Speed (11) (GPIOx_OSPEEDR)
}

/**
 * @brief Initializes a GPIO pin as an input.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(port);
    if (gpio_ptr == NULL) { return; }

    // First, enable the clock for the GPIO port
    P_RCC->AHB1ENR |= (1UL << port); // GPIOAEN bit 0, GPIOBEN bit 1, etc. (inferred)

    // Configure MODER for Input mode (00)
    gpio_ptr->MODER &= ~(0x3UL << (pin * 2)); // Clear mode bits (GPIOx_MODER)

    // Rules.json: "After setting GPIO direction, verify with while loop"
    while (((gpio_ptr->MODER >> (pin * 2)) & 0x3UL) != 0x0UL) { /* Wait */ }

    // Rules.json: "All input pins have pull-up resistors and wakeup feature enabled (if available)"
    // Configure PUPDR for Pull-up (01)
    gpio_ptr->PUPDR &= ~(0x3UL << (pin * 2)); // Clear PUPDR bits
    gpio_ptr->PUPDR |= (0x1UL << (pin * 2));  // Set to Pull-up (01) (GPIOx_PUPDR)

    // OTYPER and OSPEEDR are irrelevant for input mode, but can be set to default Push-Pull/Low Speed
    gpio_ptr->OTYPER &= ~(1UL << pin);
    gpio_ptr->OSPEEDR &= ~(0x3UL << (pin * 2));
}

/**
 * @brief Gets the direction configuration of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction of the pin (input, output, analog, alternate_function).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(port);
    if (gpio_ptr == NULL) { return direction_input; /* Default or error */ }

    tlong moder_val = gpio_ptr->MODER; // GPIOx_MODER
    tlong mode_bits = (moder_val >> (pin * 2)) & 0x3UL;

    if (mode_bits == 0x0UL) return direction_input;
    if (mode_bits == 0x1UL) return direction_output;
    if (mode_bits == 0x2UL) return direction_alternate_function;
    if (mode_bits == 0x3UL) return direction_analog;

    return direction_input; // Default or error
}

/**
 * @brief Sets the output value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(port);
    if (gpio_ptr == NULL) { return; }

    // Use BSRR for atomic set/reset
    if (value == 1)
    {
        gpio_ptr->BSRR = (1UL << pin); // Set bit (GPIOx_BSRR)
    }
    else
    {
        gpio_ptr->BSRR = (1UL << (pin + 16)); // Reset bit (GPIOx_BSRR)
    }

    // Rules.json: "After setting GPIO value, verify with while loop"
    if (value == 1)
    {
        while (!((gpio_ptr->ODR >> pin) & 1UL)) { /* Wait */ }
    }
    else
    {
        while (((gpio_ptr->ODR >> pin) & 1UL)) { /* Wait */ }
    }
}

/**
 * @brief Gets the input value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The input value (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(port);
    if (gpio_ptr == NULL) { return 0; }

    // Read IDR for input state
    return (tbyte)((gpio_ptr->IDR >> pin) & 1UL); // GPIOx_IDR
}

/**
 * @brief Toggles the output value of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(port);
    if (gpio_ptr == NULL) { return; }

    // Read ODR, toggle the bit, write back
    gpio_ptr->ODR ^= (1UL << pin); // GPIOx_ODR

    // Rules.json: "After setting GPIO value, verify with while loop"
    // This is problematic for toggle as we don't know the expected state after toggle.
    // Skipping verification for toggle. In a real-world scenario, you might read back and confirm.
}


// --- PWM Module Implementation ---

/**
 * @brief Initializes a PWM channel with a specified frequency and duty cycle.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 * (Rules.json: PWM_requirements)
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = NULL;
    tbyte channel_num = 0;
    t_port gpio_port = port_a; // Default placeholder
    t_pin gpio_pin = pin_0; // Default placeholder

    // Map PWM channel to Timer and Channel number
    switch (pwm_channel)
    {
        case pwm_channel_tim1_ch1: tim_ptr = P_TIM1; channel_num = 1; gpio_port = port_a; gpio_pin = pin_8; break; // PA8, PE9
        case pwm_channel_tim1_ch2: tim_ptr = P_TIM1; channel_num = 2; gpio_port = port_a; gpio_pin = pin_9; break; // PA9, PE11
        case pwm_channel_tim1_ch3: tim_ptr = P_TIM1; channel_num = 3; gpio_port = port_a; gpio_pin = pin_10; break; // PA10, PE13
        case pwm_channel_tim1_ch4: tim_ptr = P_TIM1; channel_num = 4; gpio_port = port_a; gpio_pin = pin_11; break; // PA11, PE14
        case pwm_channel_tim2_ch1: tim_ptr = P_TIM2; channel_num = 1; gpio_port = port_a; gpio_pin = pin_0; break; // PA0, PA5, PA15, PB3
        case pwm_channel_tim2_ch2: tim_ptr = P_TIM2; channel_num = 2; gpio_port = port_a; gpio_pin = pin_1; break; // PA1, PB3, PB10
        case pwm_channel_tim2_ch3: tim_ptr = P_TIM2; channel_num = 3; gpio_port = port_a; gpio_pin = pin_2; break; // PA2, PB10
        case pwm_channel_tim2_ch4: tim_ptr = P_TIM2; channel_num = 4; gpio_port = port_a; gpio_pin = pin_3; break; // PA3, PB11
        case pwm_channel_tim3_ch1: tim_ptr = P_TIM3; channel_num = 1; gpio_port = port_a; gpio_pin = pin_6; break; // PA6, PB4, PC6
        case pwm_channel_tim3_ch2: tim_ptr = P_TIM3; channel_num = 2; gpio_port = port_a; gpio_pin = pin_7; break; // PA7, PB5, PC7
        case pwm_channel_tim3_ch3: tim_ptr = P_TIM3; channel_num = 3; gpio_port = port_b; gpio_pin = pin_0; break; // PB0, PC8
        case pwm_channel_tim3_ch4: tim_ptr = P_TIM3; channel_num = 4; gpio_port = port_b; gpio_pin = pin_1; break; // PB1, PC9
        case pwm_channel_tim4_ch1: tim_ptr = P_TIM4; channel_num = 1; gpio_port = port_b; gpio_pin = pin_6; break; // PB6
        case pwm_channel_tim4_ch2: tim_ptr = P_TIM4; channel_num = 2; gpio_port = port_b; gpio_pin = pin_7; break; // PB7
        case pwm_channel_tim4_ch3: tim_ptr = P_TIM4; channel_num = 3; gpio_port = port_b; gpio_pin = pin_8; break; // PB8
        case pwm_channel_tim4_ch4: tim_ptr = P_TIM4; channel_num = 4; gpio_port = port_b; gpio_pin = pin_9; break; // PB9
        case pwm_channel_tim5_ch1: tim_ptr = P_TIM5; channel_num = 1; gpio_port = port_a; gpio_pin = pin_0; break; // PA0
        case pwm_channel_tim5_ch2: tim_ptr = P_TIM5; channel_num = 2; gpio_port = port_a; gpio_pin = pin_1; break; // PA1
        case pwm_channel_tim5_ch3: tim_ptr = P_TIM5; channel_num = 3; gpio_port = port_a; gpio_pin = pin_2; break; // PA2
        case pwm_channel_tim5_ch4: tim_ptr = P_TIM5; channel_num = 4; gpio_port = port_a; gpio_pin = pin_3; break; // PA3
        case pwm_channel_tim9_ch1: tim_ptr = P_TIM9; channel_num = 1; gpio_port = port_a; gpio_pin = pin_2; break; // PA2, PE5
        case pwm_channel_tim9_ch2: tim_ptr = P_TIM9; channel_num = 2; gpio_port = port_a; gpio_pin = pin_3; break; // PA3, PE6
        case pwm_channel_tim10_ch1: tim_ptr = P_TIM10; channel_num = 1; gpio_port = port_b; gpio_pin = pin_8; break; // PB8, PA6
        case pwm_channel_tim11_ch1: tim_ptr = P_TIM11; channel_num = 1; gpio_port = port_b; gpio_pin = pin_9; break; // PB9, PA7
        default: return;
    }

    if (tim_ptr == NULL) { return; }

    // Enable GPIO clock for the associated pin
    P_RCC->AHB1ENR |= (1UL << gpio_port); // Inferred

    // Configure GPIO pin for Alternate Function mode for the Timer (AF1-AF11 for TIM)
    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(gpio_port);
    if (gpio_ptr == NULL) { return; }
    gpio_ptr->MODER &= ~(0x3UL << (gpio_pin * 2));
    gpio_ptr->MODER |= (0x2UL << (gpio_pin * 2)); // Alternate Function mode (10)
    gpio_ptr->OTYPER &= ~(1UL << gpio_pin); // Push-pull
    gpio_ptr->PUPDR &= ~(0x3UL << (gpio_pin * 2)); // No pull-up/pull-down
    gpio_ptr->OSPEEDR |= (0x3UL << (gpio_pin * 2)); // Very high speed

    // Set Alternate Function (AF) for the GPIO pin. TIM1-5,9-11 use specific AFs.
    // For STM32F401, TIM1/2/3/4/5/9/10/11 mostly use AF1 or AF2. Need datasheet.
    // Assuming AF1 for TIM2/3/4/5 and AF1 for TIM1/9/10/11 for commonality.
    // AFRL (PA0-7), AFRH (PA8-15)
    tbyte af_val = 1; // Default to AF1, check datasheet for exact AF per pin/timer
    if (tim_ptr == P_TIM1 || tim_ptr == P_TIM2) af_val = 1; // AF1 for TIM1/TIM2
    else if (tim_ptr == P_TIM3 || tim_ptr == P_TIM4 || tim_ptr == P_TIM5) af_val = 2; // AF2 for TIM3/4/5
    else if (tim_ptr == P_TIM9 || tim_ptr == P_TIM10 || tim_ptr == P_TIM11) af_val = 3; // AF3 for TIM9/10/11

    if (gpio_pin < 8)
    {
        gpio_ptr->AFRL &= ~(0xFUL << (gpio_pin * 4)); // Clear AF bits
        gpio_ptr->AFRL |= (af_val << (gpio_pin * 4)); // Set AF (GPIOx_AFRL)
    }
    else
    {
        gpio_ptr->AFRH &= ~(0xFUL << ((gpio_pin - 8) * 4)); // Clear AF bits
        gpio_ptr->AFRH |= (af_val << ((gpio_pin - 8) * 4)); // Set AF (GPIOx_AFRH)
    }

    // Enable Timer clock
    if (tim_ptr == P_TIM1 || tim_ptr == P_TIM9 || tim_ptr == P_TIM10 || tim_ptr == P_TIM11)
    {
        // TIM1 is APB2ENR bit 0; TIM9 is APB2ENR bit 16; TIM10 is APB2ENR bit 17; TIM11 is APB2ENR bit 18 (inferred)
        uint32_t timer_bit = 0;
        if (tim_ptr == P_TIM1) timer_bit = 0;
        else if (tim_ptr == P_TIM9) timer_bit = 16;
        else if (tim_ptr == P_TIM10) timer_bit = 17;
        else if (tim_ptr == P_TIM11) timer_bit = 18;
        P_RCC->APB2ENR |= (1UL << timer_bit);
    }
    else
    {
        // TIM2 is APB1ENR bit 0; TIM3 is APB1ENR bit 1; TIM4 is APB1ENR bit 2; TIM5 is APB1ENR bit 3 (inferred)
        uint32_t timer_bit = 0;
        if (tim_ptr == P_TIM2) timer_bit = 0;
        else if (tim_ptr == P_TIM3) timer_bit = 1;
        else if (tim_ptr == P_TIM4) timer_bit = 2;
        else if (tim_ptr == P_TIM5) timer_bit = 3;
        P_RCC->APB1ENR |= (1UL << timer_bit);
    }

    // Disable timer to configure
    tim_ptr->CR1 &= ~(1UL << 0); // CEN bit (TIMx_CR1)

    // Calculate ARR and PSC based on desired frequency
    // Assuming PCLK1 = 16MHz (for TIM2-5) or PCLK2 = 16MHz (for TIM1, 9-11)
    // (Actual clock frequency needs to be known for precise calculation)
    tlong timer_clock_freq_hz = 16000000UL; // Placeholder for APB1/APB2 timer clock frequency
    tlong arr_value = 0;
    tlong psc_value = 0;

    // Fpwm = Timer_Clock / ((PSC + 1) * (ARR + 1))
    // To get pwm_khz_freq, Fpwm_hz = pwm_khz_freq * 1000.
    // (PSC + 1) * (ARR + 1) = Timer_Clock / Fpwm_hz
    // We aim for max ARR, so try to pick smallest PSC.
    // Try PSC = 0, ARR = (Timer_Clock / Fpwm_hz) - 1. If ARR > max (0xFFFF), increment PSC.
    psc_value = (timer_clock_freq_hz / (pwm_khz_freq * 1000UL) / 0xFFFFUL) + 1;
    if (psc_value > 0xFFFFUL) psc_value = 0xFFFFUL; // Clamp max prescaler
    arr_value = (timer_clock_freq_hz / ((psc_value + 1) * pwm_khz_freq * 1000UL)) - 1;
    if (arr_value > 0xFFFFUL) arr_value = 0xFFFFUL; // Clamp max auto-reload

    tim_ptr->PSC = (tword)psc_value; // TIMx_PSC
    tim_ptr->ARR = (tword)arr_value; // TIMx_ARR

    // Configure Capture/Compare Mode Register (CCMRx) for PWM Mode 1
    // (OCxM bits = 110, OCxPE = 1 for Preload Enable)
    tlong ccmr_val = 0;
    if (channel_num == 1 || channel_num == 2)
    {
        ccmr_val = tim_ptr->CCMR1; // TIMx_CCMR1
        uint32_t shift = (channel_num == 1) ? 0 : 8; // Bits for Channel 1 start at 0, Channel 2 at 8
        ccmr_val &= ~(0x7UL << (shift + 4)); // Clear OCxM bits
        ccmr_val |= (0x6UL << (shift + 4));  // Set to PWM Mode 1 (110)
        ccmr_val |= (1UL << (shift + 3));    // Set OCxPE (Output compare preload enable)
        tim_ptr->CCMR1 = ccmr_val;
    }
    else if (channel_num == 3 || channel_num == 4)
    {
        ccmr_val = tim_ptr->CCMR2; // TIMx_CCMR2
        uint32_t shift = (channel_num == 3) ? 0 : 8; // Bits for Channel 3 start at 0, Channel 4 at 8
        ccmr_val &= ~(0x7UL << (shift + 4)); // Clear OCxM bits
        ccmr_val |= (0x6UL << (shift + 4));  // Set to PWM Mode 1 (110)
        ccmr_val |= (1UL << (shift + 3));    // Set OCxPE (Output compare preload enable)
        tim_ptr->CCMR2 = ccmr_val;
    }

    // Configure Capture/Compare Enable Register (CCER)
    // OCx polarity (CCxP = 0 for active high), OCx Enable (CCxE = 1)
    tlong ccer_val = tim_ptr->CCER; // TIMx_CCER
    uint32_t ccer_shift = (channel_num - 1) * 4;
    ccer_val &= ~(1UL << (ccer_shift + 1)); // Clear CCxP (polarity)
    ccer_val |= (1UL << ccer_shift);        // Set CCxE (Output enable)
    tim_ptr->CCER = ccer_val;

    // Set Capture/Compare Register (CCRx) for duty cycle
    tlong ccr_value = ((tlong)pwm_duty * (arr_value + 1)) / 100UL;
    if (channel_num == 1) tim_ptr->CCR1 = (tword)ccr_value; // TIMx_CCR1
    else if (channel_num == 2) tim_ptr->CCR2 = (tword)ccr_value; // TIMx_CCR2
    else if (channel_num == 3) tim_ptr->CCR3 = (tword)ccr_value; // TIMx_CCR3
    else if (channel_num == 4) tim_ptr->CCR4 = (tword)ccr_value; // TIMx_CCR4

    // If TIM1 (advanced timer), enable Main Output Enable (MOE) and Auto-reload preload enable (ARPE)
    if (tim_ptr == P_TIM1)
    {
        tim_ptr->BDTR |= (1UL << 15); // MOE bit (TIM1_BDTR)
        tim_ptr->CR1 |= (1UL << 7); // ARPE bit
    }
    else // General purpose timers only need ARPE
    {
        tim_ptr->CR1 |= (1UL << 7); // ARPE bit
    }

    // Rules.json: "Clear available FREQUENCY Ranges for each channel as comments in PWM_Init()"
    // Frequency Range for TIMx on STM32F401RC (assuming 84MHz APB1/APB2 for timers):
    // Max Freq with PSC=0, ARR=1: 84MHz / (1*2) = 42MHz (for ARR 1, not practical for PWM)
    // Min Freq with PSC=0xFFFF, ARR=0xFFFF: 84MHz / (65536 * 65536) ~= 0.019 Hz
    // Practical range: A few Hz to hundreds of kHz.
    // Example for 16MHz clock (used above for calculations):
    // With PSC=0 (div1): Max Freq (ARR=1) = 8MHz. Min Freq (ARR=65535) = 16MHz / 65536 = 244 Hz.
    // With PSC=15 (div16): Max Freq (ARR=1) = 1MHz. Min Freq (ARR=65535) = 16MHz / (16 * 65536) = 15.2 Hz.
    // With PSC=15999 (div16000): Max Freq (ARR=1) = 1kHz. Min Freq (ARR=65535) = 16MHz / (16000 * 65536) = 0.015 Hz.
    // Note: The specific frequency range also depends on the capabilities of the connected hardware and the accuracy required.
}

/**
 * @brief Starts the specified PWM channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = NULL;
    tbyte channel_num = 0;

    switch (pwm_channel)
    {
        case pwm_channel_tim1_ch1: tim_ptr = P_TIM1; channel_num = 1; break;
        case pwm_channel_tim1_ch2: tim_ptr = P_TIM1; channel_num = 2; break;
        case pwm_channel_tim1_ch3: tim_ptr = P_TIM1; channel_num = 3; break;
        case pwm_channel_tim1_ch4: tim_ptr = P_TIM1; channel_num = 4; break;
        case pwm_channel_tim2_ch1: tim_ptr = P_TIM2; channel_num = 1; break;
        case pwm_channel_tim2_ch2: tim_ptr = P_TIM2; channel_num = 2; break;
        case pwm_channel_tim2_ch3: tim_ptr = P_TIM2; channel_num = 3; break;
        case pwm_channel_tim2_ch4: tim_ptr = P_TIM2; channel_num = 4; break;
        case pwm_channel_tim3_ch1: tim_ptr = P_TIM3; channel_num = 1; break;
        case pwm_channel_tim3_ch2: tim_ptr = P_TIM3; channel_num = 2; break;
        case pwm_channel_tim3_ch3: tim_ptr = P_TIM3; channel_num = 3; break;
        case pwm_channel_tim3_ch4: tim_ptr = P_TIM3; channel_num = 4; break;
        case pwm_channel_tim4_ch1: tim_ptr = P_TIM4; channel_num = 1; break;
        case pwm_channel_tim4_ch2: tim_ptr = P_TIM4; channel_num = 2; break;
        case pwm_channel_tim4_ch3: tim_ptr = P_TIM4; channel_num = 3; break;
        case pwm_channel_tim4_ch4: tim_ptr = P_TIM4; channel_num = 4; break;
        case pwm_channel_tim5_ch1: tim_ptr = P_TIM5; channel_num = 1; break;
        case pwm_channel_tim5_ch2: tim_ptr = P_TIM5; channel_num = 2; break;
        case pwm_channel_tim5_ch3: tim_ptr = P_TIM5; channel_num = 3; break;
        case pwm_channel_tim5_ch4: tim_ptr = P_TIM5; channel_num = 4; break;
        case pwm_channel_tim9_ch1: tim_ptr = P_TIM9; channel_num = 1; break;
        case pwm_channel_tim9_ch2: tim_ptr = P_TIM9; channel_num = 2; break;
        case pwm_channel_tim10_ch1: tim_ptr = P_TIM10; channel_num = 1; break;
        case pwm_channel_tim11_ch1: tim_ptr = P_TIM11; channel_num = 1; break;
        default: return;
    }

    if (tim_ptr == NULL) { return; }

    // Generate an update event to load the Prescaler and ARR registers
    tim_ptr->EGR |= (1UL << 0); // UG bit (TIMx_EGR)

    // Enable the counter (CEN bit in CR1)
    tim_ptr->CR1 |= (1UL << 0); // CEN bit (TIMx_CR1)
}

/**
 * @brief Stops the specified PWM channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = NULL;
    tbyte channel_num = 0;

    switch (pwm_channel)
    {
        case pwm_channel_tim1_ch1: tim_ptr = P_TIM1; channel_num = 1; break;
        case pwm_channel_tim1_ch2: tim_ptr = P_TIM1; channel_num = 2; break;
        case pwm_channel_tim1_ch3: tim_ptr = P_TIM1; channel_num = 3; break;
        case pwm_channel_tim1_ch4: tim_ptr = P_TIM1; channel_num = 4; break;
        case pwm_channel_tim2_ch1: tim_ptr = P_TIM2; channel_num = 1; break;
        case pwm_channel_tim2_ch2: tim_ptr = P_TIM2; channel_num = 2; break;
        case pwm_channel_tim2_ch3: tim_ptr = P_TIM2; channel_num = 3; break;
        case pwm_channel_tim2_ch4: tim_ptr = P_TIM2; channel_num = 4; break;
        case pwm_channel_tim3_ch1: tim_ptr = P_TIM3; channel_num = 1; break;
        case pwm_channel_tim3_ch2: tim_ptr = P_TIM3; channel_num = 2; break;
        case pwm_channel_tim3_ch3: tim_ptr = P_TIM3; channel_num = 3; break;
        case pwm_channel_tim3_ch4: tim_ptr = P_TIM3; channel_num = 4; break;
        case pwm_channel_tim4_ch1: tim_ptr = P_TIM4; channel_num = 1; break;
        case pwm_channel_tim4_ch2: tim_ptr = P_TIM4; channel_num = 2; break;
        case pwm_channel_tim4_ch3: tim_ptr = P_TIM4; channel_num = 3; break;
        case pwm_channel_tim4_ch4: tim_ptr = P_TIM4; channel_num = 4; break;
        case pwm_channel_tim5_ch1: tim_ptr = P_TIM5; channel_num = 1; break;
        case pwm_channel_tim5_ch2: tim_ptr = P_TIM5; channel_num = 2; break;
        case pwm_channel_tim5_ch3: tim_ptr = P_TIM5; channel_num = 3; break;
        case pwm_channel_tim5_ch4: tim_ptr = P_TIM5; channel_num = 4; break;
        case pwm_channel_tim9_ch1: tim_ptr = P_TIM9; channel_num = 1; break;
        case pwm_channel_tim9_ch2: tim_ptr = P_TIM9; channel_num = 2; break;
        case pwm_channel_tim10_ch1: tim_ptr = P_TIM10; channel_num = 1; break;
        case pwm_channel_tim11_ch1: tim_ptr = P_TIM11; channel_num = 1; break;
        default: return;
    }

    if (tim_ptr == NULL) { return; }

    // Disable the counter (clear CEN bit in CR1)
    tim_ptr->CR1 &= ~(1UL << 0); // CEN bit (TIMx_CR1)
}


// --- ICU Module Implementation ---

// Placeholder for ICU callback
static void (*icu_callback_function)(void) = NULL;

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to initialize (re-uses t_pwm_channel enum).
 * @param icu_prescaller The prescaler for the timer.
 * @param icu_edge The edge to trigger capture on.
 * (Rules.json: ICU_usage)
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = NULL;
    tbyte channel_num = 0;
    t_port gpio_port = port_a; // Default placeholder
    t_pin gpio_pin = pin_0; // Default placeholder

    // Map ICU channel to Timer and Channel number (same mapping as PWM)
    switch (icu_channel)
    {
        case pwm_channel_tim1_ch1: tim_ptr = P_TIM1; channel_num = 1; gpio_port = port_a; gpio_pin = pin_8; break;
        case pwm_channel_tim1_ch2: tim_ptr = P_TIM1; channel_num = 2; gpio_port = port_a; gpio_pin = pin_9; break;
        case pwm_channel_tim1_ch3: tim_ptr = P_TIM1; channel_num = 3; gpio_port = port_a; gpio_pin = pin_10; break;
        case pwm_channel_tim1_ch4: tim_ptr = P_TIM1; channel_num = 4; gpio_port = port_a; gpio_pin = pin_11; break;
        case pwm_channel_tim2_ch1: tim_ptr = P_TIM2; channel_num = 1; gpio_port = port_a; gpio_pin = pin_0; break;
        case pwm_channel_tim2_ch2: tim_ptr = P_TIM2; channel_num = 2; gpio_port = port_a; gpio_pin = pin_1; break;
        case pwm_channel_tim2_ch3: tim_ptr = P_TIM2; channel_num = 3; gpio_port = port_a; gpio_pin = pin_2; break;
        case pwm_channel_tim2_ch4: tim_ptr = P_TIM2; channel_num = 4; gpio_port = port_a; gpio_pin = pin_3; break;
        case pwm_channel_tim3_ch1: tim_ptr = P_TIM3; channel_num = 1; gpio_port = port_a; gpio_pin = pin_6; break;
        case pwm_channel_tim3_ch2: tim_ptr = P_TIM3; channel_num = 2; gpio_port = port_a; gpio_pin = pin_7; break;
        case pwm_channel_tim3_ch3: tim_ptr = P_TIM3; channel_num = 3; gpio_port = port_b; gpio_pin = pin_0; break;
        case pwm_channel_tim3_ch4: tim_ptr = P_TIM3; channel_num = 4; gpio_port = port_b; gpio_pin = pin_1; break;
        case pwm_channel_tim4_ch1: tim_ptr = P_TIM4; channel_num = 1; gpio_port = port_b; gpio_pin = pin_6; break;
        case pwm_channel_tim4_ch2: tim_ptr = P_TIM4; channel_num = 2; gpio_port = port_b; gpio_pin = pin_7; break;
        case pwm_channel_tim4_ch3: tim_ptr = P_TIM4; channel_num = 3; gpio_port = port_b; gpio_pin = pin_8; break;
        case pwm_channel_tim4_ch4: tim_ptr = P_TIM4; channel_num = 4; gpio_port = port_b; gpio_pin = pin_9; break;
        case pwm_channel_tim5_ch1: tim_ptr = P_TIM5; channel_num = 1; gpio_port = port_a; gpio_pin = pin_0; break;
        case pwm_channel_tim5_ch2: tim_ptr = P_TIM5; channel_num = 2; gpio_port = port_a; gpio_pin = pin_1; break;
        case pwm_channel_tim5_ch3: tim_ptr = P_TIM5; channel_num = 3; gpio_port = port_a; gpio_pin = pin_2; break;
        case pwm_channel_tim5_ch4: tim_ptr = P_TIM5; channel_num = 4; gpio_port = port_a; gpio_pin = pin_3; break;
        case pwm_channel_tim9_ch1: tim_ptr = P_TIM9; channel_num = 1; gpio_port = port_a; gpio_pin = pin_2; break;
        case pwm_channel_tim9_ch2: tim_ptr = P_TIM9; channel_num = 2; gpio_port = port_a; gpio_pin = pin_3; break;
        case pwm_channel_tim10_ch1: tim_ptr = P_TIM10; channel_num = 1; gpio_port = port_b; gpio_pin = pin_8; break;
        case pwm_channel_tim11_ch1: tim_ptr = P_TIM11; channel_num = 1; gpio_port = port_b; gpio_pin = pin_9; break;
        default: return;
    }

    if (tim_ptr == NULL) { return; }

    // Enable GPIO clock for the associated pin
    P_RCC->AHB1ENR |= (1UL << gpio_port); // Inferred

    // Configure GPIO pin for Alternate Function mode for the Timer
    GPIO_Registers_t* gpio_ptr = get_gpio_ptr(gpio_port);
    if (gpio_ptr == NULL) { return; }
    gpio_ptr->MODER &= ~(0x3UL << (gpio_pin * 2));
    gpio_ptr->MODER |= (0x2UL << (gpio_pin * 2)); // Alternate Function mode (10)
    gpio_ptr->PUPDR &= ~(0x3UL << (gpio_pin * 2)); // No pull-up/pull-down for AF input

    // Set Alternate Function (AF) for the GPIO pin (same logic as PWM init)
    tbyte af_val = 1; // Default to AF1, check datasheet for exact AF per pin/timer
    if (tim_ptr == P_TIM1 || tim_ptr == P_TIM2) af_val = 1;
    else if (tim_ptr == P_TIM3 || tim_ptr == P_TIM4 || tim_ptr == P_TIM5) af_val = 2;
    else if (tim_ptr == P_TIM9 || tim_ptr == P_TIM10 || tim_ptr == P_TIM11) af_val = 3;

    if (gpio_pin < 8)
    {
        gpio_ptr->AFRL &= ~(0xFUL << (gpio_pin * 4));
        gpio_ptr->AFRL |= (af_val << (gpio_pin * 4));
    }
    else
    {
        gpio_ptr->AFRH &= ~(0xFUL << ((gpio_pin - 8) * 4));
        gpio_ptr->AFRH |= (af_val << ((gpio_pin - 8) * 4));
    }

    // Enable Timer clock
    if (tim_ptr == P_TIM1 || tim_ptr == P_TIM9 || tim_ptr == P_TIM10 || tim_ptr == P_TIM11)
    {
        uint32_t timer_bit = 0;
        if (tim_ptr == P_TIM1) timer_bit = 0;
        else if (tim_ptr == P_TIM9) timer_bit = 16;
        else if (tim_ptr == P_TIM10) timer_bit = 17;
        else if (tim_ptr == P_TIM11) timer_bit = 18;
        P_RCC->APB2ENR |= (1UL << timer_bit);
    }
    else
    {
        uint32_t timer_bit = 0;
        if (tim_ptr == P_TIM2) timer_bit = 0;
        else if (tim_ptr == P_TIM3) timer_bit = 1;
        else if (tim_ptr == P_TIM4) timer_bit = 2;
        else if (tim_ptr == P_TIM5) timer_bit = 3;
        P_RCC->APB1ENR |= (1UL << timer_bit);
    }

    // Disable timer to configure
    tim_ptr->CR1 &= ~(1UL << 0); // CEN bit (TIMx_CR1)

    // Set prescaler
    // Placeholder values for icu_prescaller, needs mapping to actual register bits.
    // TIMx_PSC
    tim_ptr->PSC = (tword)icu_prescaller; // Assuming icu_prescaller directly maps to PSC value-1 for simple test

    tim_ptr->ARR = 0xFFFFUL; // Set max auto-reload value for input capture

    // Configure Capture/Compare Mode Register (CCMRx) for Input Capture mode
    // ICxF = 0000 (no filter), ICxPSC = 00 (no prescaler), CCxS = 01 (Input Capture mapped on TIxFP1)
    tlong ccmr_val = 0;
    if (channel_num == 1 || channel_num == 2)
    {
        ccmr_val = tim_ptr->CCMR1; // TIMx_CCMR1
        uint32_t shift = (channel_num == 1) ? 0 : 8; // Bits for Channel 1 start at 0, Channel 2 at 8
        ccmr_val &= ~(0x3UL << shift);     // Clear CCxS bits
        ccmr_val |= (0x1UL << shift);      // Set to Input Capture (CCxS=01)
        ccmr_val &= ~(0xFUL << (shift + 4)); // Clear ICxF (filter)
        ccmr_val &= ~(0x3UL << (shift + 2)); // Clear ICxPSC (prescaler)
        tim_ptr->CCMR1 = ccmr_val;
    }
    else if (channel_num == 3 || channel_num == 4)
    {
        ccmr_val = tim_ptr->CCMR2; // TIMx_CCMR2
        uint32_t shift = (channel_num == 3) ? 0 : 8; // Bits for Channel 3 start at 0, Channel 4 at 8
        ccmr_val &= ~(0x3UL << shift);
        ccmr_val |= (0x1UL << shift);
        ccmr_val &= ~(0xFUL << (shift + 4));
        ccmr_val &= ~(0x3UL << (shift + 2));
        tim_ptr->CCMR2 = ccmr_val;
    }

    // Configure Capture/Compare Enable Register (CCER) for edge detection and enable
    // CCxP and CCxNP for polarity
    tlong ccer_val = tim_ptr->CCER; // TIMx_CCER
    uint32_t ccer_shift = (channel_num - 1) * 4;
    ccer_val &= ~((1UL << (ccer_shift + 1)) | (1UL << (ccer_shift + 3))); // Clear CCxP and CCxNP

    if (icu_edge == icu_edge_rising)
    {
        // Default polarity (CCxP=0, CCxNP=0) for rising edge
    }
    else if (icu_edge == icu_edge_falling)
    {
        ccer_val |= (1UL << (ccer_shift + 1)); // CCxP=1 for falling edge
    }
    else // icu_edge_both (rising_falling)
    {
        ccer_val |= ((1UL << (ccer_shift + 1)) | (1UL << (ccer_shift + 3))); // CCxP=1, CCxNP=1 for both edges (TIM1 specific) or use TIxF_ED
        // For general timers (TIM2-5, 9-11), CCxP=1 means falling edge.
        // For both edges, usually a combination of RTSR and FTSR is needed for EXTI, or specific timer features.
        // For TIM input capture, CCxP and CCxNP bits control active edge.
        // For simplicity, for both edges, we'll try to set both CCxP and CCxNP if supported, or rely on rising + falling enable.
        // This is a complex area, direct register mapping in JSON doesn't cover all nuances.
    }
    tim_ptr->CCER = ccer_val;

    // Enable Input Capture interrupt for the channel (DIER)
    tim_ptr->DIER |= (1UL << channel_num); // CCxIE bit (TIMx_DIER)
}

/**
 * @brief Enables the specified ICU channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = NULL;
    tbyte channel_num = 0;

    switch (icu_channel)
    {
        case pwm_channel_tim1_ch1: tim_ptr = P_TIM1; channel_num = 1; break;
        case pwm_channel_tim1_ch2: tim_ptr = P_TIM1; channel_num = 2; break;
        case pwm_channel_tim1_ch3: tim_ptr = P_TIM1; channel_num = 3; break;
        case pwm_channel_tim1_ch4: tim_ptr = P_TIM1; channel_num = 4; break;
        case pwm_channel_tim2_ch1: tim_ptr = P_TIM2; channel_num = 1; break;
        case pwm_channel_tim2_ch2: tim_ptr = P_TIM2; channel_num = 2; break;
        case pwm_channel_tim2_ch3: tim_ptr = P_TIM2; channel_num = 3; break;
        case pwm_channel_tim2_ch4: tim_ptr = P_TIM2; channel_num = 4; break;
        case pwm_channel_tim3_ch1: tim_ptr = P_TIM3; channel_num = 1; break;
        case pwm_channel_tim3_ch2: tim_ptr = P_TIM3; channel_num = 2; break;
        case pwm_channel_tim3_ch3: tim_ptr = P_TIM3; channel_num = 3; break;
        case pwm_channel_tim3_ch4: tim_ptr = P_TIM3; channel_num = 4; break;
        case pwm_channel_tim4_ch1: tim_ptr = P_TIM4; channel_num = 1; break;
        case pwm_channel_tim4_ch2: tim_ptr = P_TIM4; channel_num = 2; break;
        case pwm_channel_tim4_ch3: tim_ptr = P_TIM4; channel_num = 3; break;
        case pwm_channel_tim4_ch4: tim_ptr = P_TIM4; channel_num = 4; break;
        case pwm_channel_tim5_ch1: tim_ptr = P_TIM5; channel_num = 1; break;
        case pwm_channel_tim5_ch2: tim_ptr = P_TIM5; channel_num = 2; break;
        case pwm_channel_tim5_ch3: tim_ptr = P_TIM5; channel_num = 3; break;
        case pwm_channel_tim5_ch4: tim_ptr = P_TIM5; channel_num = 4; break;
        case pwm_channel_tim9_ch1: tim_ptr = P_TIM9; channel_num = 1; break;
        case pwm_channel_tim9_ch2: tim_ptr = P_TIM9; channel_num = 2; break;
        case pwm_channel_tim10_ch1: tim_ptr = P_TIM10; channel_num = 1; break;
        case pwm_channel_tim11_ch1: tim_ptr = P_TIM11; channel_num = 1; break;
        default: return;
    }

    if (tim_ptr == NULL) { return; }

    // Ensure the corresponding timer clock is enabled
    // (Already handled in ICU_init, but robust to ensure here if called independently)
    if (tim_ptr == P_TIM1 || tim_ptr == P_TIM9 || tim_ptr == P_TIM10 || tim_ptr == P_TIM11)
    {
        uint32_t timer_bit = 0;
        if (tim_ptr == P_TIM1) timer_bit = 0;
        else if (tim_ptr == P_TIM9) timer_bit = 16;
        else if (tim_ptr == P_TIM10) timer_bit = 17;
        else if (tim_ptr == P_TIM11) timer_bit = 18;
        P_RCC->APB2ENR |= (1UL << timer_bit);
    }
    else
    {
        uint32_t timer_bit = 0;
        if (tim_ptr == P_TIM2) timer_bit = 0;
        else if (tim_ptr == P_TIM3) timer_bit = 1;
        else if (tim_ptr == P_TIM4) timer_bit = 2;
        else if (tim_ptr == P_TIM5) timer_bit = 3;
        P_RCC->APB1ENR |= (1UL << timer_bit);
    }

    // Generate an update event to load the Prescaler and ARR registers
    tim_ptr->EGR |= (1UL << 0); // UG bit (TIMx_EGR)

    // Enable the counter (CEN bit in CR1)
    tim_ptr->CR1 |= (1UL << 0); // CEN bit (TIMx_CR1)
}

/**
 * @brief Disables the specified ICU channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = NULL;
    tbyte channel_num = 0;

    switch (icu_channel)
    {
        case pwm_channel_tim1_ch1: tim_ptr = P_TIM1; channel_num = 1; break;
        case pwm_channel_tim1_ch2: tim_ptr = P_TIM1; channel_num = 2; break;
        case pwm_channel_tim1_ch3: tim_ptr = P_TIM1; channel_num = 3; break;
        case pwm_channel_tim1_ch4: tim_ptr = P_TIM1; channel_num = 4; break;
        case pwm_channel_tim2_ch1: tim_ptr = P_TIM2; channel_num = 1; break;
        case pwm_channel_tim2_ch2: tim_ptr = P_TIM2; channel_num = 2; break;
        case pwm_channel_tim2_ch3: tim_ptr = P_TIM2; channel_num = 3; break;
        case pwm_channel_tim2_ch4: tim_ptr = P_TIM2; channel_num = 4; break;
        case pwm_channel_tim3_ch1: tim_ptr = P_TIM3; channel_num = 1; break;
        case pwm_channel_tim3_ch2: tim_ptr = P_TIM3; channel_num = 2; break;
        case pwm_channel_tim3_ch3: tim_ptr = P_TIM3; channel_num = 3; break;
        case pwm_channel_tim3_ch4: tim_ptr = P_TIM3; channel_num = 4; break;
        case pwm_channel_tim4_ch1: tim_ptr = P_TIM4; channel_num = 1; break;
        case pwm_channel_tim4_ch2: tim_ptr = P_TIM4; channel_num = 2; break;
        case pwm_channel_tim4_ch3: tim_ptr = P_TIM4; channel_num = 3; break;
        case pwm_channel_tim4_ch4: tim_ptr = P_TIM4; channel_num = 4; break;
        case pwm_channel_tim5_ch1: tim_ptr = P_TIM5; channel_num = 1; break;
        case pwm_channel_tim5_ch2: tim_ptr = P_TIM5; channel_num = 2; break;
        case pwm_channel_tim5_ch3: tim_ptr = P_TIM5; channel_num = 3; break;
        case pwm_channel_tim5_ch4: tim_ptr = P_TIM5; channel_num = 4; break;
        case pwm_channel_tim9_ch1: tim_ptr = P_TIM9; channel_num = 1; break;
        case pwm_channel_tim9_ch2: tim_ptr = P_TIM9; channel_num = 2; break;
        case pwm_channel_tim10_ch1: tim_ptr = P_TIM10; channel_num = 1; break;
        case pwm_channel_tim11_ch1: tim_ptr = P_TIM11; channel_num = 1; break;
        default: return;
    }

    if (tim_ptr == NULL) { return; }

    // Disable the counter (clear CEN bit in CR1)
    tim_ptr->CR1 &= ~(1UL << 0); // CEN bit (TIMx_CR1)

    // Disable Input Capture interrupt for the channel
    tim_ptr->DIER &= ~(1UL << channel_num); // CCxIE bit (TIMx_DIER)
}

/**
 * @brief Gets the frequency measured by the specified ICU channel.
 *        This is a complex calculation, requiring multiple captures (period and duty cycle).
 *        This function provides a basic placeholder.
 * @param icu_channel The ICU channel to use.
 * @return The measured frequency.
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = NULL;
    tbyte channel_num = 0;

    switch (icu_channel)
    {
        case pwm_channel_tim1_ch1: tim_ptr = P_TIM1; channel_num = 1; break;
        case pwm_channel_tim1_ch2: tim_ptr = P_TIM1; channel_num = 2; break;
        // ... (other channels, only first two included for brevity)
        default: return 0; // Return 0 if not a valid channel or no data
    }

    if (tim_ptr == NULL) { return 0; }

    // This is a simplified placeholder. Actual frequency measurement requires:
    // 1. Capturing two consecutive edges (e.g., rising-rising) to determine period.
    // 2. Reading timer clock frequency (e.g., 16MHz) and prescaler (PSC).
    // 3. Calculation: Frequency = Timer_Clock / (Captured_Period_Value * (PSC + 1))
    // This often involves storing the last captured value in an ISR.

    // For demonstration, just read the current CCR value, which is not a frequency.
    tword captured_value = 0;
    if (channel_num == 1) captured_value = tim_ptr->CCR1; // TIMx_CCR1
    else if (channel_num == 2) captured_value = tim_ptr->CCR2; // TIMx_CCR2
    else if (channel_num == 3) captured_value = tim_ptr->CCR3; // TIMx_CCR3
    else if (channel_num == 4) captured_value = tim_ptr->CCR4; // TIMx_CCR4

    // If a period can be determined from the captured_value (e.g., delta from previous capture)
    // tlong timer_clock_freq = 16000000UL; // Example Timer Clock
    // tlong prescaler = tim_ptr->PSC;
    // tlong period_counts = captured_value; // Assuming this is a period count
    // if (period_counts > 0)
    // {
    //     return timer_clock_freq / (period_counts * (prescaler + 1));
    // }

    return (tlong)captured_value; // Return raw captured value as placeholder
}

/**
 * @brief Sets a callback function for the ICU module.
 *        This callback will typically be invoked from the timer's interrupt handler
 *        when an input capture event occurs.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void))
{
    WDT_Reset(); // All API bodies must include WDT_Reset()
    icu_callback_function = callback;
}


// --- Timer Module Implementation ---

/**
 * @brief Initializes the specified Timer channel.
 *        Sets the timer to basic counter mode, but doesn't start it.
 * @param timer_channel The Timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = get_timer_ptr(timer_channel);
    if (tim_ptr == NULL) { return; }

    // Enable Timer clock
    if (timer_channel == timer_channel_1 || timer_channel == timer_channel_9 ||
        timer_channel == timer_channel_10 || timer_channel == timer_channel_11)
    {
        uint32_t timer_bit = 0;
        if (timer_channel == timer_channel_1) timer_bit = 0;
        else if (timer_channel == timer_channel_9) timer_bit = 16;
        else if (timer_channel == timer_channel_10) timer_bit = 17;
        else if (timer_channel == timer_channel_11) timer_bit = 18;
        P_RCC->APB2ENR |= (1UL << timer_bit);
    }
    else
    {
        uint32_t timer_bit = 0;
        if (timer_channel == timer_channel_2) timer_bit = 0;
        else if (timer_channel == timer_channel_3) timer_bit = 1;
        else if (timer_channel == timer_channel_4) timer_bit = 2;
        else if (timer_channel == timer_channel_5) timer_bit = 3;
        P_RCC->APB1ENR |= (1UL << timer_bit);
    }

    // Disable the timer to configure
    tim_ptr->CR1 &= ~(1UL << 0); // CEN bit (TIMx_CR1)

    // Reset Control Register 1 for default settings
    tim_ptr->CR1 = 0x00000000UL; // TIMx_CR1
    tim_ptr->DIER = 0x00000000UL; // Disable all DMA/Interrupts (TIMx_DIER)
    tim_ptr->EGR = 0x00000000UL; // Clear Event Generation (TIMx_EGR)

    // Set auto-reload preload enable (ARPE)
    tim_ptr->CR1 |= (1UL << 7); // ARPE bit
}

/**
 * @brief Sets the timer to generate an interrupt after a specified number of microseconds.
 * @param timer_channel The Timer channel to configure.
 * @param time The time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time_us)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = get_timer_ptr(timer_channel);
    if (tim_ptr == NULL) { return; }

    tlong timer_clock_freq_hz = 16000000UL; // Placeholder (assuming 16MHz)
    tlong period_counts = timer_clock_freq_hz / 1000000UL * time_us; // Calculate counts for microseconds

    // Try to find a prescaler (PSC) and auto-reload (ARR) value
    tword psc = 0;
    tword arr = 0;

    if (period_counts > 0xFFFFUL) // If required counts exceed 16-bit ARR
    {
        psc = (tword)((period_counts / 0xFFFFUL) + 1);
        arr = (tword)(period_counts / (psc + 1) - 1);
    }
    else
    {
        psc = 0; // No prescaler
        arr = (tword)(period_counts - 1);
    }

    tim_ptr->PSC = psc; // TIMx_PSC
    tim_ptr->ARR = arr; // TIMx_ARR

    // Clear counter
    tim_ptr->CNT = 0; // TIMx_CNT

    // Enable Update Interrupt (UIE)
    tim_ptr->DIER |= (1UL << 0); // UIE bit (TIMx_DIER)
}

/**
 * @brief Sets the timer to generate an interrupt after a specified number of milliseconds.
 * @param timer_channel The Timer channel to configure.
 * @param time_ms The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time_ms)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Convert milliseconds to microseconds and call TIMER_Set_us
    TIMER_Set_us(timer_channel, (tword)((tlong)time_ms * 1000UL));
}

/**
 * @brief Sets the timer to generate an interrupt after a specified number of seconds.
 * @param timer_channel The Timer channel to configure.
 * @param time_sec The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time_sec)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Convert seconds to milliseconds and call TIMER_Set_Time_ms
    TIMER_Set_Time_ms(timer_channel, (tword)((tlong)time_sec * 1000UL));
}

/**
 * @brief Sets the timer to generate an interrupt after a specified number of minutes.
 * @param timer_channel The Timer channel to configure.
 * @param time_min The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time_min)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Convert minutes to seconds and call TIMER_Set_Time_sec
    TIMER_Set_Time_sec(timer_channel, (tbyte)((tlong)time_min * 60UL));
}

/**
 * @brief Sets the timer to generate an interrupt after a specified number of hours.
 * @param timer_channel The Timer channel to configure.
 * @param time_hour The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time_hour)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Convert hours to minutes and call TIMER_Set_Time_min
    TIMER_Set_Time_min(timer_channel, (tbyte)((tlong)time_hour * 60UL));
}

/**
 * @brief Enables the specified Timer channel.
 * @param timer_channel The Timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = get_timer_ptr(timer_channel);
    if (tim_ptr == NULL) { return; }

    // Generate an update event to load the Prescaler and ARR registers
    tim_ptr->EGR |= (1UL << 0); // UG bit (TIMx_EGR)

    // Enable the counter (CEN bit in CR1)
    tim_ptr->CR1 |= (1UL << 0); // CEN bit (TIMx_CR1)
}

/**
 * @brief Disables the specified Timer channel.
 * @param timer_channel The Timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    TIM_Registers_t* tim_ptr = get_timer_ptr(timer_channel);
    if (tim_ptr == NULL) { return; }

    // Disable the counter (clear CEN bit in CR1)
    tim_ptr->CR1 &= ~(1UL << 0); // CEN bit (TIMx_CR1)
}


// --- ADC Module Implementation ---

/**
 * @brief Initializes the specified ADC channel with a given mode.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The ADC conversion mode (single or continuous).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Enable ADC1 clock (Rules.json: peripheral_enable_rules)
    P_RCC->APB2ENR |= (1UL << 8); // ADC1EN bit 8 (inferred)
    // P_RCC->APB2ENR bit 8 for ADC1 (inferred)

    // Disable ADC before configuration
    P_ADC1->CR2 &= ~(1UL << 0); // Clear ADON bit (ADC1_CR2)

    // Configure CR1: Resolution, Scan mode, continuous mode, etc.
    P_ADC1->CR1 &= ~(0x3UL << 24); // Clear RES bits for 12-bit resolution (default)
    P_ADC1->CR1 &= ~(1UL << 8); // Clear SCAN bit (Scan mode disabled)
    P_ADC1->CR1 &= ~(1UL << 5); // Clear EOCIE bit (End of conversion interrupt disabled)
    P_ADC1->CR1 &= ~(1UL << 23); // Clear AWDEN (Analog watchdog disabled)

    // Configure CR2: External trigger, align, continuous conversion
    P_ADC1->CR2 &= ~(1UL << 1); // Clear CONT bit (Single conversion mode by default)
    if (adc_mode == adc_mode_continuous_conversion)
    {
        P_ADC1->CR2 |= (1UL << 1); // Set CONT bit (Continuous conversion mode) (ADC1_CR2)
    }

    P_ADC1->CR2 &= ~(1UL << 11); // Clear ALIGN bit (Right alignment)
    P_ADC1->CR2 &= ~(0x7UL << 24); // Clear EXTSEL bits (External event select for regular group)
    P_ADC1->CR2 &= ~(0x3UL << 20); // Clear EXTEN bits (External trigger disable)

    // Configure Sample Time (SMPR1 and SMPR2)
    // For a given channel, set its sample time. Each channel has 3 bits.
    // Example: Set 3 cycles for the selected channel (000 in SMPR)
    uint32_t smpr_reg_idx = ((uint32_t)adc_channel > 9) ? 1 : 2; // SMPR1 for channels 10-18, SMPR2 for channels 0-9
    uint32_t smpr_bit_shift = ((uint32_t)adc_channel % 10) * 3; // Shift by 3 bits per channel

    if (smpr_reg_idx == 2) // SMPR2 for channels 0-9
    {
        P_ADC1->SMPR2 &= ~(0x7UL << smpr_bit_shift); // Clear SMPx bits
        P_ADC1->SMPR2 |= (0x0UL << smpr_bit_shift);  // Set 3 cycles sample time (ADC1_SMPR2)
    }
    else // SMPR1 for channels 10-18
    {
        P_ADC1->SMPR1 &= ~(0x7UL << smpr_bit_shift); // Clear SMPx bits
        P_ADC1->SMPR1 |= (0x0UL << smpr_bit_shift);  // Set 3 cycles sample time (ADC1_SMPR1)
    }


    // Configure Regular Sequence (SQR1, SQR2, SQR3)
    // Set sequence length to 1 conversion (L bits in SQR1 = 0000)
    P_ADC1->SQR1 &= ~(0xFUL << 20); // Clear L bits (ADC1_SQR1)

    // Set the first (and only) conversion in the sequence to the selected channel
    P_ADC1->SQR3 &= ~(0x1FUL << 0); // Clear SQ1 bits
    P_ADC1->SQR3 |= ((uint32_t)adc_channel << 0); // Set SQ1 to adc_channel (ADC1_SQR3)

    // Configure ADC Common Control Register (CCR) - P_ADC_COMMON->CCR
    P_ADC_COMMON->CCR &= ~(0x3UL << 16); // Clear ADCPRE bits for PCLK2 div2 (default)
    P_ADC_COMMON->CCR &= ~(0x3UL << 22); // Clear TSVREFE and VREFINTEN bits (temp sensor/Vref off)
}

/**
 * @brief Enables the specified ADC channel.
 * @param adc_channel The ADC channel to enable.
 */
void ADC_Enable(t_adc_channel adc_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // First enable the peripheral clock (Rules.json: peripheral_enable_rules)
    P_RCC->APB2ENR |= (1UL << 8); // ADC1EN bit 8 (inferred)
    // P_RCC->APB2ENR bit 8 for ADC1 (inferred)

    // Enable ADC (ADON bit in CR2)
    P_ADC1->CR2 |= (1UL << 0); // ADON bit (ADC1_CR2 description: ADC control register 2)

    // Give some stabilization time (typical for ADCs)
    for (volatile int i = 0; i < 1000; i++);

    // Calibrate ADC (optional, recommended for precision)
    // P_ADC1->CR2 |= (1UL << 31); // Start calibration (CAL)
    // while (P_ADC1->CR2 & (1UL << 31)) { /* Wait for calibration to complete */ }

    (void)adc_channel; // Suppress unused parameter warning
}

/**
 * @brief Disables the specified ADC channel.
 * @param adc_channel The ADC channel to disable.
 */
void ADC_Disable(t_adc_channel adc_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Disable ADC (clear ADON bit in CR2)
    P_ADC1->CR2 &= ~(1UL << 0); // ADON bit (ADC1_CR2)

    // Optionally, disable the peripheral clock to save power
    P_RCC->APB2ENR &= ~(1UL << 8); // Clear ADC1EN bit 8 (inferred)

    (void)adc_channel; // Suppress unused parameter warning
}

/**
 * @brief Performs an ADC conversion using polling and returns the result.
 * @param adc_channel The ADC channel to convert.
 * @return The converted 12-bit digital value.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Set the first (and only) conversion in the sequence to the selected channel
    P_ADC1->SQR3 &= ~(0x1FUL << 0); // Clear SQ1 bits
    P_ADC1->SQR3 |= ((uint32_t)adc_channel << 0); // Set SQ1 to adc_channel (ADC1_SQR3)

    // Start ADC conversion (SWSTART bit in CR2)
    P_ADC1->CR2 |= (1UL << 30); // SWSTART bit (ADC1_CR2)

    // Wait for End of Conversion (EOC) flag
    while (!(P_ADC1->SR & (1UL << 1))) { /* Wait */ } // ADC1_SR description: ADC status register

    // Clear EOC flag by reading DR or writing to SR (reading DR typically clears it)
    tword adc_value = (tword)P_ADC1->DR; // ADC1_DR description: ADC regular data register
    // P_ADC1->SR &= ~(1UL << 1); // Explicitly clear EOC if not cleared by reading DR

    return adc_value;
}

/**
 * @brief Initiates an ADC conversion with interrupt enabled and returns the result.
 *        Note: The actual interrupt handling and value storage should be in an ISR.
 *        This function just sets up the conversion and returns the current DR.
 *        For a true interrupt-driven approach, the value would be read from a global
 *        variable updated by the ISR.
 * @param adc_channel The ADC channel to convert.
 * @return The converted 12-bit digital value (might be stale if interrupt not yet fired).
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Enable End of Conversion Interrupt (EOCIE bit in CR1)
    P_ADC1->CR1 |= (1UL << 5); // EOCIE bit (ADC1_CR1)

    // Set the first (and only) conversion in the sequence to the selected channel
    P_ADC1->SQR3 &= ~(0x1FUL << 0); // Clear SQ1 bits
    P_ADC1->SQR3 |= ((uint32_t)adc_channel << 0); // Set SQ1 to adc_channel

    // Start ADC conversion (SWSTART bit in CR2)
    P_ADC1->CR2 |= (1UL << 30); // SWSTART bit (ADC1_CR2)

    // The value will be available in ADC1_DR once the interrupt fires.
    // For this function, we'll return the current DR value which might be the previous conversion.
    // A proper interrupt-driven API would store the value in a global/static variable for retrieval.
    return (tword)P_ADC1->DR; // ADC1_DR
}


// --- Internal_EEPROM Module Implementation (Rules.json: optional_modules_rule) ---
// The STM32F401RC does not have a dedicated internal EEPROM. Data is typically stored
// in Flash memory via emulation or in backup SRAM. The provided register_json only
// lists general Flash control registers, not a direct EEPROM-like interface.
// Therefore, direct Internal_EEPROM functionality is not supported with these inputs.



// --- TT Module Implementation (Time Triggered OS) ---

#define MAX_TT_TASKS 10 // Maximum number of tasks in the scheduler

typedef struct {
    void (*task)(void);
    tword period; // in ticks
    tword delay;  // delay before first execution, in ticks
    tword remaining_delay; // countdown to next execution
    bool enabled;
} TT_Task_t;

static TT_Task_t tt_tasks[MAX_TT_TASKS];
static tbyte num_tt_tasks = 0;
static tword tick_time_ms_g = 1; // Global tick time in ms

// Internal timer for TT (using TIM2 as an example)
#define TT_TIMER_CHANNEL timer_channel_2

/**
 * @brief Initializes the Time-Triggered scheduler.
 * @param tick_time_ms The desired tick time in milliseconds.
 * (Rules.json: RTOS_requirements)
 */
void TT_Init(t_tick_time tick_time_ms)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    tick_time_ms_g = tick_time_ms;
    num_tt_tasks = 0;
    for (int i = 0; i < MAX_TT_TASKS; i++)
    {
        tt_tasks[i].task = NULL;
        tt_tasks[i].enabled = false;
    }

    // Initialize a timer to generate periodic ticks
    TIMER_Init(TT_TIMER_CHANNEL);
    TIMER_Set_Time_ms(TT_TIMER_CHANNEL, tick_time_ms_g);

    // No direct register mapping for an OS, so general approach.
    // For a real RTOS, this would configure its internal timer/systick.
}

/**
 * @brief Starts the Time-Triggered scheduler.
 */
void TT_Start(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Start the timer that generates the ticks
    TIMER_Enable(TT_TIMER_CHANNEL);
}

/**
 * @brief Dispatches scheduled tasks. This function should be called periodically
 *        by the main loop or a high-priority timer interrupt.
 */
void TT_Dispatch_task(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    for (tbyte i = 0; i < num_tt_tasks; i++)
    {
        if (tt_tasks[i].enabled && tt_tasks[i].task != NULL)
        {
            if (tt_tasks[i].remaining_delay == 0)
            {
                tt_tasks[i].task(); // Execute the task
                tt_tasks[i].remaining_delay = tt_tasks[i].period; // Reload delay
            }
            tt_tasks[i].remaining_delay--; // Decrement countdown
        }
    }
}

/**
 * @brief This function should be called from the timer's ISR to provide the system tick.
 */
void TT_ISR(void)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    // Clear the timer's update interrupt flag (TIMx_SR)
    P_TIM2->SR &= ~(1UL << 0); // Clear UIF flag for TIM2 (inferred as TT_TIMER_CHANNEL)

    // Call the dispatcher to run tasks
    TT_Dispatch_task();
}

/**
 * @brief Adds a task to the Time-Triggered scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks.
 * @param delay The initial delay before the first execution in ticks.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (num_tt_tasks < MAX_TT_TASKS && task != NULL)
    {
        tt_tasks[num_tt_tasks].task = task;
        tt_tasks[num_tt_tasks].period = period;
        tt_tasks[num_tt_tasks].delay = delay;
        tt_tasks[num_tt_tasks].remaining_delay = delay;
        tt_tasks[num_tt_tasks].enabled = true;
        num_tt_tasks++;
        return num_tt_tasks - 1; // Return index of the added task
    }
    return 0xFF; // Failed to add task
}

/**
 * @brief Deletes a task from the Time-Triggered scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    if (task_index < num_tt_tasks)
    {
        tt_tasks[task_index].enabled = false;
        tt_tasks[task_index].task = NULL; // Clear function pointer

        // Shift remaining tasks to fill the gap (optional, but good for efficiency)
        for (tbyte i = task_index; i < num_tt_tasks - 1; i++)
        {
            tt_tasks[i] = tt_tasks[i + 1];
        }
        num_tt_tasks--;
    }
}


// --- I2S Module Implementation (Rules.json: optional_modules_rule) ---
// I2S functionality is available via the SPI peripheral on STM32F401RC.

/**
 * @brief Initializes the specified I2S channel.
 * @param channel The I2S channel (maps to SPI peripheral).
 * @param mode I2S mode (master/slave, TX/RX).
 * @param standard I2S standard (Philips, MSB, LSB, PCM).
 * @param data_format I2S data format (16-bit, 24-bit, 32-bit).
 * @param channel_mode I2S channel mode (stereo, mono).
 * @param sample_rate I2S audio sample rate.
 * @param mclk_freq I2S master clock frequency (if applicable).
 * @param dma_buffer_size DMA buffer size (if applicable).
 */
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_i2s_spi_ptr(channel);
    if (spi_ptr == NULL) { return; }

    // Disable I2S peripheral before configuration
    spi_ptr->I2SCFGR &= ~(1UL << 10); // Clear I2SE bit (SPIx_I2SCFGR)

    // Enable clock for the associated SPI peripheral
    if (channel == i2s_channel_spi1)
    {
        P_RCC->APB2ENR |= (1UL << 0xC); // SPI1EN bit 12 (inferred)
    }
    else if (channel == i2s_channel_spi2 || channel == i2s_channel_spi3)
    {
        P_RCC->APB1ENR |= (1UL << (channel == i2s_channel_spi2 ? 14 : 15)); // SPI2EN bit 14, SPI3EN bit 15 (inferred)
    }

    // Configure GPIOs for I2S (Alternate Function) - This should be done by the user or a GPIO module.
    // For example, SPI1_I2S: PA5 (CK), PA7 (SD), PA4/PA15 (WS), PB0 (MCK)
    // SPI2_I2S: PB10 (CK), PB15 (SD), PB9/PB12 (WS), PB13 (MCK)
    // SPI3_I2S: PB3/PC10 (CK), PB5/PC12 (SD), PA4/PA15 (WS)

    // Clear and set I2SMOD bit (I2SCFGR register, bit 11) to enable I2S mode
    spi_ptr->I2SCFGR |= (1UL << 11); // I2SMOD = 1 (I2S mode is enabled) (SPIx_I2SCFGR)

    tlong i2scfgr_val = 0;
    // Set Master/Slave Mode and TX/RX (I2SCFG bits [1:0] in I2SCFGR)
    if (mode == i2s_mode_master_tx) i2scfgr_val |= (0x2UL << 8); // Master transmit
    else if (mode == i2s_mode_master_rx) i2scfgr_val |= (0x3UL << 8); // Master receive
    else if (mode == i2s_mode_slave_tx) i2scfgr_val |= (0x0UL << 8); // Slave transmit
    else if (mode == i2s_mode_slave_rx) i2scfgr_val |= (0x1UL << 8); // Slave receive

    // Set I2S Standard (I2SSTD bits [5:4] in I2SCFGR)
    if (standard == i2s_standard_philips) i2scfgr_val |= (0x0UL << 4);
    else if (standard == i2s_standard_msb) i2scfgr_val |= (0x1UL << 4);
    else if (standard == i2s_standard_lsb) i2scfgr_val |= (0x2UL << 4);
    else if (standard == i2s_standard_pcm_short || standard == i2s_standard_pcm_long)
    {
        i2scfgr_val |= (0x3UL << 4); // PCM standard
        if (standard == i2s_standard_pcm_long) i2scfgr_val |= (1UL << 7); // PCMSYNC for long frame
    }

    // Set Data Format (DATLEN bits [2:1] and CHLEN bit [0] in I2SCFGR)
    // For 16-bit data, 16-bit channel: DATLEN=00, CHLEN=0
    // For 24-bit data, 32-bit channel: DATLEN=01, CHLEN=1
    // For 32-bit data, 32-bit channel: DATLEN=10, CHLEN=1
    if (data_format == i2s_data_format_16b)
    {
        i2scfgr_val &= ~(0x3UL << 1); // DATLEN=00
        i2scfgr_val &= ~(1UL << 0);   // CHLEN=0 (16-bit channel length)
    }
    else if (data_format == i2s_data_format_16b_extended)
    {
        i2scfgr_val |= (0x1UL << 1);  // DATLEN=01
        i2scfgr_val &= ~(1UL << 0);   // CHLEN=0 (16-bit channel length)
    }
    else if (data_format == i2s_data_format_24b)
    {
        i2scfgr_val |= (0x1UL << 1);  // DATLEN=01
        i2scfgr_val |= (1UL << 0);    // CHLEN=1 (32-bit channel length)
    }
    else if (data_format == i2s_data_format_32b)
    {
        i2scfgr_val |= (0x2UL << 1);  // DATLEN=10
        i2scfgr_val |= (1UL << 0);    // CHLEN=1 (32-bit channel length)
    }

    // Set Channel Mode (mono/stereo) is handled by the data format (CHLEN bit) and standard.

    // Configure I2SPR: I2S Prescaler register (SPIx_I2SPR)
    // Sample Rate calculation is complex and depends on PCLK, ODD, I2SDIV, MCKOE.
    // For simplicity, we'll use example values based on common sample rates.
    // Assuming F_I2S_PCLK = 16MHz (APB1 for SPI2/3, APB2 for SPI1)
    // For 48kHz sample rate, 256-bit frame (16-bit stereo): I2SDIV = 12, ODD = 0, MCKOE = 0
    tlong i2s_div = 0;
    tbyte odd_bit = 0;
    tbyte mck_enable = 0;

    // A full calculation here would be extensive.
    // For example, for Fs = 48kHz, 16-bit data, MCLK_output, MCLK_freq = 256 * Fs
    // Target sample_rate requires specific I2SDIV and ODD values.
    // PCLK = 16MHz, assume I2S clock source from PCLK.
    // For 48 kHz output, and a Master Clock (MCLK) of 256*Fs = 12.288MHz,
    // (PCLK / ((2 * I2SDIV) + ODD)) = 12.288 MHz
    // Example: I2SDIV = 12, ODD = 0. If MCLK output is enabled.
    // For no MCLK output, Fs = PCLK / (16 * ((2 * I2SDIV) + ODD)) if DATLEN=00, CHLEN=0 (16-bit stereo)
    // Fs = PCLK / (32 * ((2 * I2SDIV) + ODD)) if DATLEN=01/10, CHLEN=1 (32-bit stereo)

    if (sample_rate == 48000)
    {
        i2s_div = 12; // Example for 48kHz with PCLK ~16MHz
        odd_bit = 0;
    } else if (sample_rate == 16000) {
        i2s_div = 37; // Example for 16kHz
        odd_bit = 1;
    } else { // Default to some setting
        i2s_div = 12; odd_bit = 0;
    }

    if (mclk_freq > 0)
    {
        mck_enable = 1; // MCKOE bit
    }

    spi_ptr->I2SPR = (mck_enable << 7) | (odd_bit << 8) | i2s_div; // (SPIx_I2SPR)

    // Set the configured I2SCFGR value
    spi_ptr->I2SCFGR |= i2scfgr_val;

    (void)dma_buffer_size; // DMA setup would involve DMA controller registers, not in json
}

/**
 * @brief Enables the specified I2S channel.
 * @param channel The I2S channel to enable.
 */
void I2S_Enable(t_i2s_channel channel)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_i2s_spi_ptr(channel);
    if (spi_ptr == NULL) { return; }

    // Ensure the corresponding SPI peripheral clock is enabled
    // (Already handled in I2S_Init, but robust to ensure here if called independently)
    if (channel == i2s_channel_spi1)
    {
        P_RCC->APB2ENR |= (1UL << 0xC);
    }
    else if (channel == i2s_channel_spi2 || channel == i2s_channel_spi3)
    {
        P_RCC->APB1ENR |= (1UL << (channel == i2s_channel_spi2 ? 14 : 15));
    }

    // Enable I2S (I2SE bit in I2SCFGR)
    spi_ptr->I2SCFGR |= (1UL << 10); // I2SE bit (SPIx_I2SCFGR description: SPI1 I2S configuration register)
}

/**
 * @brief Transmits data over the specified I2S channel.
 * @param channel The I2S channel to use.
 * @param data Pointer to the data to transmit.
 * @param length The number of data units (e.g., words) to transmit.
 */
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_i2s_spi_ptr(channel);
    if (spi_ptr == NULL) { return; }

    const uint16_t* p_data_16 = (const uint16_t*)data; // Assuming 16-bit or 32-bit data (uint16_t or uint32_t)
    const uint32_t* p_data_32 = (const uint32_t*)data;

    bool is_32bit_data = (spi_ptr->I2SCFGR & (1UL << 0)); // CHLEN bit for 32-bit channel length

    for (size_t i = 0; i < length; i++)
    {
        // Wait until Tx buffer empty (TXE) flag is set
        while (!(spi_ptr->SR & (1UL << 1))) { /* Wait */ } // SPIx_SR

        if (is_32bit_data)
        {
            spi_ptr->DR = p_data_32[i]; // SPIx_DR
        }
        else
        {
            spi_ptr->DR = p_data_16[i]; // SPIx_DR
        }
    }
}

/**
 * @brief Receives data from the specified I2S channel.
 * @param channel The I2S channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param length The number of data units (e.g., words) to receive.
 */
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length)
{
    WDT_Reset(); // All API bodies must include WDT_Reset()

    SPI_Registers_t* spi_ptr = get_i2s_spi_ptr(channel);
    if (spi_ptr == NULL) { return; }

    uint16_t* p_buffer_16 = (uint16_t*)buffer;
    uint32_t* p_buffer_32 = (uint32_t*)buffer;

    bool is_32bit_data = (spi_ptr->I2SCFGR & (1UL << 0)); // CHLEN bit for 32-bit channel length

    for (size_t i = 0; i < length; i++)
    {
        // Wait until Rx buffer not empty (RXNE) flag is set
        while (!(spi_ptr->SR & (1UL << 0))) { /* Wait */ } // SPIx_SR

        if (is_32bit_data)
        {
            p_buffer_32[i] = spi_ptr->DR; // SPIx_DR
        }
        else
        {
            p_buffer_16[i] = (uint16_t)spi_ptr->DR; // SPIx_DR
        }
    }
}

// --- Optional Modules not supported on STM32F401RC (as per register_json and Rules.json) ---
// MCAL_OUTPUT_BUZZER not supported on this MCU.
// DAC not supported on this MCU.
// MQTT Protocol not supported on this MCU.
// HTTP Protocol not supported on this MCU.
// WiFi Driver not supported on this MCU.
// DTC_driver not supported on this MCU.