/**
 * @file main.h
 * @brief Global include file for the embedded project on ESP32.
 *        Contains common definitions, types, and safeguard functions.
 * @author Technology Inovation Software Team
 * @device ESP32
 * @creation_date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H
#define MAIN_H

/*******************************************************************************
 *                              Include Directives                             *
 ******************************************************************************/

/**
 * @brief Include MCU-specific header file.
 *        Note: The specific path/file might depend on the build system and SDK (e.g., ESP-IDF).
 *        This header is assumed to provide definitions for registers and core peripherals.
 *        For ESP-IDF, this might indirectly pull in necessary SOC headers.
 */
#include "esp_system.h" // Common ESP32 base header in ESP-IDF context

/**
 * @brief Standard C Library Includes.
 *        Included as specified in the requirements.
 */
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <iso646.h> // Defines operators like 'and', 'or', 'not'
#include <limits.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h> // Defines NULL, size_t, ptrdiff_t
#include <stdint.h> // Defines fixed-width integer types
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 *                               Useful Typedefs                               *
 ******************************************************************************/

/** @brief Typedef for an unsigned 8-bit integer. */
typedef uint8_t tbyte;

/** @brief Typedef for an unsigned 16-bit integer. */
typedef uint16_t tword;

/** @brief Typedef for an unsigned 32-bit integer. */
typedef uint32_t tlong;

/*******************************************************************************
 *                                Core Macros                                  *
 ******************************************************************************/

/**
 * @brief Set a specific bit in a register.
 * @param[in,out] reg Register variable.
 * @param[in] bit Bit position (0-indexed).
 */
#define SET_BIT(reg, bit)     ((reg) |= (1U << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param[in,out] reg Register variable.
 * @param[in] bit Bit position (0-indexed).
 */
#define CLR_BIT(reg, bit)     ((reg) &= ~(1U << (bit)))

/**
 * @brief Get the value of a specific bit in a register.
 * @param[in] reg Register variable.
 * @param[in] bit Bit position (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)     (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param[in,out] reg Register variable.
 * @param[in] bit Bit position (0-indexed).
 */
#define TOG_BIT(reg, bit)     ((reg) ^= (1U << (bit)))

/**
 * @brief Disable global interrupts.
 *        Uses ESP-IDF function. Requires linking against ESP-IDF libraries.
 */
#define DI()                  esp_intr_disable()

/**
 * @brief Enable global interrupts.
 *        Uses ESP-IDF function. Requires linking against ESP-IDF libraries.
 */
#define EI()                  esp_intr_enable()

/** @brief Alias for DI(). */
#define Global_Int_Disable()  DI()

/** @brief Alias for EI(). */
#define Global_Int_Enable()   EI()

/**
 * @brief Halts program execution in an infinite loop.
 */
#define HALT() do { while(1); } while(0)

/**
 * @brief Executes a No-Operation instruction.
 */
#define NOP()                 __asm__("nop")


/*******************************************************************************
 *                             SAFEGUARD MACROS                                *
 ******************************************************************************/

/**
 * @brief Initialize all GPIO pins to a safe state.
 *        Sets output data low, configures as inputs, disables pulls,
 *        disables wake-up.
 *        Note: This implementation directly manipulates conceptual register
 *        names based on ESP32 architecture and common SDK definitions.
 *        Actual register definitions (addresses, bitmasks) must be provided
 *        by the MCU-specific include or the build environment.
 *        Iterates over a common range of potential GPIOs (0-39).
 */
static inline void GPIO_SAFEGUARD_Init(void)
{
    /*
     * This implementation assumes access to the following conceptual register
     * definitions, typically found in ESP32 SOC headers like soc/gpio_reg.h,
     * soc/io_mux_reg.h, etc., often included indirectly via esp_system.h or
     * specific driver headers in an ESP-IDF environment.
     *
     * Examples of conceptual registers/fields used:
     * DR_REG_GPIO_BASE (Base address for GPIO registers)
     * GPIO_OUT_W1TC_REG (Register to clear output bits)
     * GPIO_ENABLE_W1TC_REG (Register to clear output enable bits)
     * DR_REG_IO_MUX_BASE (Base address for IO MUX registers)
     * IO_MUX_GPIO0_REG, IO_MUX_GPIO1_REG, ... (Registers per pin)
     * IO_MUX_GPIO_FUN_PU (Bitmask for pull-up function in IO_MUX)
     * IO_MUX_GPIO_FUN_PD (Bitmask for pull-down function in IO_MUX)
     * IO_MUX_GPIO_FUN_SEL_S (Shift for function select bits in IO_MUX)
     * GPIO_PIN_WAKEUP_ENABLE_S (Shift for wakeup enable bit in GPIO pin config)
     * GPIO_PIN_PAD_DRIVER_S (Shift for pad driver bit in GPIO pin config)
     * GPIO_PIN_INT_ENA_S (Shift for interrupt enable bits in GPIO pin config)
     * GPIO_PIN0_REG, GPIO_PIN1_REG, ... (Pin-specific configuration registers)
     * DR_REG_RTCIO_BASE (Base address for RTCIO registers - relevant for low-power GPIOs)
     * RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_CFG_REG, etc.
     *
     * Direct hardcoded addresses are avoided for robustness, relying on names.
     * If these names are not available, appropriate SOC headers need to be included.
     */

#ifdef DR_REG_GPIO_BASE // Check if core GPIO registers are likely defined
    volatile uint32_t* gpio_out_w1tc_reg = (volatile uint32_t*)(DR_REG_GPIO_BASE + 0x0010); /* Example offset for GPIO_OUT_W1TC_REG */
    volatile uint32_t* gpio_enable_w1tc_reg = (volatile uint32_t*)(DR_REG_GPIO_BASE + 0x0020); /* Example offset for GPIO_ENABLE_W1TC_REG */
    volatile uint32_t* gpio_pin_reg_base = (volatile uint32_t*)(DR_REG_GPIO_BASE + 0x002C); /* Example offset for GPIO_PIN0_REG */
    volatile uint32_t* iomux_gpio_reg_base = (volatile uint32_t*)(DR_REG_IO_MUX_BASE + 0x000); /* Example offset for IO_MUX_GPIO0_REG */

    const uint32_t FUN_PU_MASK = (1U << 7);  /* Example bit for pull-up in IO_MUX */
    const uint32_t FUN_PD_MASK = (1U << 6);  /* Example bit for pull-down in IO_MUX */
    const uint32_t FUN_SEL_MASK = (0x7U << 11); /* Example mask for function select */
    const uint32_t WAKEUP_ENABLE_MASK = (1U << 10); /* Example bit for wakeup enable in GPIO_PIN */
    const uint32_t PAD_DRIVER_MASK = (1U << 2); /* Example bit for pad driver in GPIO_PIN */
    const uint32_t INT_ENA_MASK = (0x1FU << 13); /* Example mask for interrupt enable in GPIO_PIN */


    /* Clear output data for all potential GPIOs (affects outputs currently enabled) */
    /* Assuming GPIOs 0-31 are in lower register, 32-39 in higher */
    *gpio_out_w1tc_reg = 0xFFFFFFFFU;
    /* Note: GPIO32-39 might need a different register, check actual header defines */
#ifdef GPIO_OUT1_W1TC_REG
    volatile uint32_t* gpio_out1_w1tc_reg = (volatile uint32_t*)(DR_REG_GPIO_BASE + 0x0014); /* Example offset */
    *gpio_out1_w1tc_reg = 0xFFFFFFFFU; // Clear bits 32-63 (only 32-39 relevant for ESP32)
#endif


    /* Disable output enable for all potential GPIOs -> configures them as inputs */
    *gpio_enable_w1tc_reg = 0xFFFFFFFFU;
#ifdef GPIO_ENABLE1_W1TC_REG
    volatile uint32_t* gpio_enable1_w1tc_reg = (volatile uint32_t*)(DR_REG_GPIO_BASE + 0x0024); /* Example offset */
    *gpio_enable1_w1tc_reg = 0xFFFFFFFFU; // Clear bits 32-63
#endif

    /* Iterate through potential GPIO pins to disable pulls, wakeup, interrupts, set to simple I/O */
    /* Consider GPIOs 0-39 (most usable pins) */
    for (int i = 0; i <= 39; ++i)
    {
        /*
         * IMPORTANT: Pins 6-11 are typically used for SPI flash/PSRAM and might be problematic
         * to reconfigure. Pins 34-39 are input-only. Strapping pins (0, 2, 4, 5, 12, 15)
         * have boot functions. Reconfiguring these should be done with caution.
         * This loop is a *safeguard* attempt to put *most* pins in a known state.
         * A production system might need a more targeted approach excluding certain pins.
         * RTC GPIOs (0-21 typically mapped to ESP32 GPIOs 0-21) also have separate configs.
         * This example primarily focuses on the digital GPIO matrix.
         */

        if ((i == 6) || (i == 7) || (i == 8) || (i == 9) || (i == 10) || (i == 11))
        {
             // Skip flash/PSRAM pins for basic safeguard, or add specific handling
             continue;
        }

        // Disable pull-up and pull-down in IO MUX for the pin
        // Need to access the correct IO_MUX register for pin 'i'
        // Example: IO_MUX_GPIO0_REG = DR_REG_IO_MUX_BASE + 0x0, IO_MUX_GPIO1_REG = DR_REG_IO_MUX_BASE + 0x4, etc.
        // This requires correct base address and register offsets.
        // Assuming IO_MUX_GPIOi_REG exists and is addressable via base + i*4
        volatile uint32_t* iomux_pin_reg = (volatile uint32_t*)(iomux_gpio_reg_base + (i * 4));
        *iomux_pin_reg &= ~(FUN_PU_MASK | FUN_PD_MASK); // Clear pull bits

        // Set pin function to simple GPIO (usually FUN_SEL = 0)
        // This requires knowing the correct FUN_SEL bitmask and shift for the chip variant
        // Assuming FUN_SEL bits are in IO_MUX_GPIOi_REG
        *iomux_pin_reg &= ~FUN_SEL_MASK; // Clear function select bits

        // Disable wake-up for the pin
        // Pin config registers are typically GPIO_PIN0_REG, GPIO_PIN1_REG, etc.
        // Address is often gpio_pin_reg_base + i*4
        volatile uint32_t* gpio_pin_conf_reg = (volatile uint32_t*)(gpio_pin_reg_base + (i * 4));
        *gpio_pin_conf_reg &= ~(WAKEUP_ENABLE_MASK); // Clear wakeup enable bit

        // Disable interrupt enable for the pin
        *gpio_pin_conf_reg &= ~(INT_ENA_MASK); // Clear interrupt enable bits

        // Ensure pad driver is not configured in a special mode (e.g., open drain if not intended)
        // Clear the PAD_DRIVER bit (assuming 0 is push-pull)
        *gpio_pin_conf_reg &= ~(PAD_DRIVER_MASK);
    }

#else
    /* Placeholder if necessary register definitions are not found */
    /* This section should ideally not be reached if proper headers are included */
    #warning "ESP32 GPIO register definitions not found. GPIO_SAFEGUARD_Init is non-functional."
    // User: Add code here to configure GPIOs to a safe state using available HAL/driver functions if raw register access is not possible.
    // for (int i = 0; i < CONFIG_GPIO_PIN_COUNT; ++i) {
    //     gpio_reset_pin(i); // Resets pin to default state (input, pulls disabled, etc.)
    //     gpio_set_direction(i, GPIO_MODE_INPUT); // Explicitly set as input
    //     gpio_set_pull_mode(i, GPIO_PULLUP_DISABLE);
    //     gpio_set_pull_mode(i, GPIO_PULLDOWN_DISABLE);
    //     // Note: Disabling wakeup requires different functions
    // }
#endif
}

/**
 * @brief Initialize key peripheral registers to a disabled or safe state.
 *        Disables global interrupts, Timers, PWM (LEDC), WDT, PCNT (ICU/Capture),
 *        ADC, UART, I2C, SPI, and ensures GPIOs are set to simple I/O mode
 *        (peripheral multiplexing disabled).
 *        Note: This implementation directly manipulates conceptual register
 *        names based on ESP32 architecture and common SDK definitions.
 *        Actual register definitions (addresses, bitmasks) must be provided
 *        by the MCU-specific include or the build environment.
 */
static inline void Registers_SAFEGUARD_Init(void)
{
    /*
     * This implementation assumes access to conceptual register definitions
     * for various peripherals (TIMG, LEDC, WDT, PCNT, ADC, UART, I2C, SPI,
     * SYSTEM) typically found in ESP32 SOC headers.
     *
     * Examples of conceptual registers/fields used:
     * TIMG0, TIMG1 (Timer Group structures)
     * TIMGn.config.en (Timer enable bit)
     * TIMGn.int_ena.val (Timer interrupt enable register)
     * LEDC (LEDC structure)
     * LEDC.conf.low_speed_en (LEDC low speed mode enable)
     * LEDC.channel[n].conf0.val (LEDC channel config)
     * TIMGn.wdtconfig0.en (Timer Group WDT enable)
     * RTCCNTL.wdtconfig0.en (RTC WDT enable) - Requires RTC register access
     * PCNT (Pulse Counter structure)
     * PCNT.conf[n].val (PCNT unit config)
     * ADC1, ADC2 (ADC structures)
     * ADCn.ctrl.start_force (ADC conversion start control)
     * ADCn.int_ena.val (ADC interrupt enable)
     * UART0, UART1, UART2 (UART structures)
     * UARTn.conf0.val (UART config register)
     * UARTn.conf0.rxfifo_rst, UARTn.conf0.txfifo_rst (FIFO reset bits)
     * UARTn.int_ena.val (UART interrupt enable)
     * I2C0, I2C1 (I2C structures)
     * I2Cn.ctr.trans_start (I2C transfer start)
     * I2Cn.fifo_conf.rxfifo_rst, I2Cn.fifo_conf.txfifo_rst (FIFO reset bits)
     * I2Cn.int_ena.val (I2C interrupt enable)
     * SPI0, SPI1 (flash), SPI2, SPI3 (general)
     * SPIx.cmd.usr (SPI user command start)
     * SPIx.ctrl.val (SPI control register)
     * SPIx.dma_conf.rx_fifo_rst, SPIx.dma_conf.tx_fifo_rst (DMA FIFO reset bits)
     * SPIx.slave.val (SPI slave control)
     * SPIx.misc.int_ena (SPI interrupt enable)
     * DR_REG_IO_MUX_BASE (Base address for IO MUX registers)
     * IO_MUX_GPIOi_REG (Registers per pin)
     * IO_MUX_GPIO_FUN_SEL_S (Shift for function select bits in IO_MUX)
     *
     * Direct hardcoded addresses are avoided for robustness, relying on names.
     * If these names are not available, appropriate SOC headers need to be included.
     */

#ifdef DR_REG_SYSTEM_BASE // Check if core system registers are likely defined

    /* Disable global interrupts (using ESP-IDF function, safest way) */
    esp_intr_disable();

    /* Disable Timers (TIMG0, TIMG1) */
    /* Assuming TIMG0/TIMG1 structures or base addresses are defined */
    volatile uint32_t* timg0_conf_reg = (volatile uint32_t*)(DR_REG_TIMG0_BASE + 0x0000); /* Example offset for T0CONFIG */
    volatile uint32_t* timg1_conf_reg = (volatile uint32_t*)(DR_REG_TIMG1_BASE + 0x0080); /* Example offset for T0CONFIG */
    volatile uint32_t* timg0_int_ena_reg = (volatile uint32_t*)(DR_REG_TIMG0_BASE + 0x0098); /* Example offset for INT_ENA_TIMERS */
    volatile uint32_t* timg1_int_ena_reg = (volatile uint32_t*)(DR_REG_TIMG1_BASE + 0x0118); /* Example offset for INT_ENA_TIMERS */
    const uint32_t T_EN_MASK = (1U << 31); /* Example bit for timer enable */

    if (timg0_conf_reg != NULL) { *timg0_conf_reg &= ~T_EN_MASK; }
    if (timg1_conf_reg != NULL) { *timg1_conf_reg &= ~T_EN_MASK; }
    if (timg0_int_ena_reg != NULL) { *timg0_int_ena_reg = 0; } /* Disable all timer interrupts */
    if (timg1_int_ena_reg != NULL) { *timg1_int_ena_reg = 0; } /* Disable all timer interrupts */
    // Need to disable other timers if applicable (e.g., RTC timers) - complex, focusing on TIMG

    /* Disable PWM (LEDC Peripheral) */
    /* Assuming LEDC structure or base address is defined */
    volatile uint32_t* ledc_conf_reg = (volatile uint32_t*)(DR_REG_LEDC_BASE + 0x0000); /* Example offset for LEDC_CONF_REG */
    volatile uint32_t* ledc_int_ena_reg = (volatile uint32_t*)(DR_REG_LEDC_BASE + 0x000C); /* Example offset for LEDC_INT_ENA_REG */
    const uint32_t LEDC_LS_EN_MASK = (1U << 0); /* Example bit for low speed enable */

    if (ledc_conf_reg != NULL) { *ledc_conf_reg &= ~LEDC_LS_EN_MASK; } /* Disable low speed mode */
    if (ledc_int_ena_reg != NULL) { *ledc_int_ena_reg = 0; } /* Disable LEDC interrupts */

    /* Disable all LEDC channels */
    /* Assuming LEDC_LSCH0_CONF0_REG base address, 0x10 per channel */
    volatile uint32_t* ledc_ls_ch_conf0_base = (volatile uint32_t*)(DR_REG_LEDC_BASE + 0x0020); /* Example offset */
    const uint32_t LEDC_CHANNEL_COUNT = 8; /* ESP32 has 8 LEDC channels */
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i)
    {
         volatile uint32_t* ch_conf0_reg = (volatile uint32_t*)(ledc_ls_ch_conf0_base + (i * 0x10));
         if (ch_conf0_reg != NULL) { *ch_conf0_reg = 0; } /* Reset channel config */
         // Also need to reset hpoint, duty, etc. - setting CONF0 to 0 is a start.
    }


    /* Disable Watchdog Timers (TIMG WDTs, RTC WDT) */
    volatile uint32_t* timg0_wdt_conf_reg = (volatile uint32_t*)(DR_REG_TIMG0_BASE + 0x0048); /* Example offset for WDTCONFIG0 */
    volatile uint32_t* timg1_wdt_conf_reg = (volatile uint32_t*)(DR_REG_TIMG1_BASE + 0x00C8); /* Example offset for WDTCONFIG0 */
    const uint32_t WDT_EN_MASK = (1U << 31); /* Example bit for WDT enable */
    const uint32_t WDT_FLASHBOOT_MOD_EN_MASK = (1U << 12); /* Example bit for WDT flashboot mode enable */

    /* Feeding the watchdog might be required before disabling it */
    /* This is complex and potentially risky, direct disable is attempted */
    if (timg0_wdt_conf_reg != NULL) {
        *timg0_wdt_conf_reg &= ~WDT_FLASHBOOT_MOD_EN_MASK; // Disable flashboot mode if enabled
        *timg0_wdt_conf_reg &= ~WDT_EN_MASK;             // Disable TIMG0 WDT
    }
     if (timg1_wdt_conf_reg != NULL) {
        *timg1_wdt_conf_reg &= ~WDT_FLASHBOOT_MOD_EN_MASK; // Disable flashboot mode if enabled
        *timg1_wdt_conf_reg &= ~WDT_EN_MASK;             // Disable TIMG1 WDT
    }

    /* Disabling RTC WDT requires accessing RTC registers, which have a different base */
    /* Assuming RTCCNTL_BASE is defined */
    volatile uint32_t* rtc_wdt_conf_reg = (volatile uint32_t*)(DR_REG_RTCCNTL_BASE + 0x0094); /* Example offset for WDTCONFIG0 */
     if (rtc_wdt_conf_reg != NULL) {
        // Similar logic to TIMG WDT, clear enable bit
        *rtc_wdt_conf_reg &= ~WDT_EN_MASK; // Disable RTC WDT
        // May need to feed first or write unlock sequence - check specific docs.
        // A simpler approach might be using esp_deep_sleep(0) or esp_restart(), but this is a safeguard.
     }


    /* Disable Input Capture Unit (PCNT Peripheral commonly used for this) */
    /* Assuming PCNT structure or base address is defined */
    volatile uint32_t* pcnt_conf_base = (volatile uint32_t*)(DR_REG_PCNT_BASE + 0x0000); /* Example offset for CNT_CONF0 */
    volatile uint32_t* pcnt_int_ena_reg = (volatile uint32_t*)(DR_REG_PCNT_BASE + 0x00D0); /* Example offset for INT_ENA */
    const uint32_t PCNT_UNIT_COUNT = 8; /* ESP32 has 8 PCNT units */

    for (int i = 0; i < PCNT_UNIT_COUNT; ++i)
    {
        volatile uint32_t* unit_conf_reg = (volatile uint32_t*)(pcnt_conf_base + (i * 0x10)); /* Config is 0x10 apart */
        if (unit_conf_reg != NULL) { *unit_conf_reg = 0; } /* Reset unit configuration */
    }
    if (pcnt_int_ena_reg != NULL) { *pcnt_int_ena_reg = 0; } /* Disable all PCNT interrupts */


    /* Disable Analog to Digital Converter (ADC1, ADC2) */
    /* Assuming ADC1/ADC2 structures or base addresses are defined */
    volatile uint32_t* adc1_ctrl_reg = (volatile uint32_t*)(DR_REG_SARADC_BASE + 0x0000); /* Example offset for ADC1_CTRL_REG */
    volatile uint32_t* adc2_ctrl_reg = (volatile uint32_t*)(DR_REG_SARADC_BASE + 0x0004); /* Example offset for ADC2_CTRL_REG */
    volatile uint32_t* saradc_int_ena_reg = (volatile uint32_t*)(DR_REG_SARADC_BASE + 0x0018); /* Example offset for SARADC_INT_ENA_REG */

    const uint32_t ADC_START_FORCE_MASK = (1U << 27); /* Example bit for start force */

    if (adc1_ctrl_reg != NULL) { *adc1_ctrl_reg &= ~ADC_START_FORCE_MASK; } /* Stop potential ongoing conversion */
    if (adc2_ctrl_reg != NULL) { *adc2_ctrl_reg &= ~ADC_START_FORCE_MASK; } /* Stop potential ongoing conversion */
    if (saradc_int_ena_reg != NULL) { *saradc_int_ena_reg = 0; } /* Disable ADC interrupts */
    // More configuration registers might need clearing depending on setup (e.g., channel selection, attenuation)


    /* Disable UART (UART0, UART1, UART2) */
    /* Assuming UART0/1/2 structures or base addresses are defined */
    volatile uint32_t* uart_reg_bases[] = {
        (volatile uint32_t*)DR_REG_UART_BASE,      /* UART0 */
        (volatile uint32_t*)DR_REG_UART1_BASE,     /* UART1 */
        (volatile uint32_t*)DR_REG_UART2_BASE      /* UART2 */
    };
    const size_t UART_COUNT = 3;

    const uint32_t UART_CONF0_RST_MASK = (1U << 8) | (1U << 9); /* Example bits for RX/TX FIFO reset in CONF0 */

    for (size_t i = 0; i < UART_COUNT; ++i)
    {
        if (uart_reg_bases[i] != NULL)
        {
            volatile uint32_t* conf0_reg = (volatile uint32_t*)(uart_reg_bases[i] + 0x0008); /* Example offset for CONF0_REG */
            volatile uint32_t* int_ena_reg = (volatile uint32_t*)(uart_reg_bases[i] + 0x0014); /* Example offset for INT_ENA_REG */
            volatile uint32_t* cmd_reg = (volatile uint32_t*)(uart_reg_bases[i] + 0x0018); /* Example offset for UART_CMD_REG */

            if (cmd_reg != NULL) { *cmd_reg = 0; } /* Clear any pending commands */
            if (int_ena_reg != NULL) { *int_ena_reg = 0; } /* Disable all UART interrupts */
            if (conf0_reg != NULL)
            {
                *conf0_reg |= UART_CONF0_RST_MASK; /* Reset FIFOs */
                *conf0_reg &= ~UART_CONF0_RST_MASK; /* De-assert reset */
                *conf0_reg = 0; // Clear other configuration bits (baud rate, data bits, etc.)
            }
        }
    }

    /* Disable I2C (I2C0, I2C1) */
     /* Assuming I2C0/I2C1 structures or base addresses are defined */
    volatile uint32_t* i2c_reg_bases[] = {
        (volatile uint32_t*)DR_REG_I2C_EXT0_BASE,   /* I2C0 */
        (volatile uint32_t*)DR_REG_I2C_EXT1_BASE    /* I2C1 */
    };
    const size_t I2C_COUNT = 2;

    const uint32_t I2C_CTR_TRANS_START_MASK = (1U << 0); /* Example bit for transfer start in CTR */
    const uint32_t I2C_FIFO_RST_MASK = (1U << 0) | (1U << 1); /* Example bits for RX/TX FIFO reset in FIFO_CONF */

    for (size_t i = 0; i < I2C_COUNT; ++i)
    {
         if (i2c_reg_bases[i] != NULL)
        {
            volatile uint32_t* ctr_reg = (volatile uint32_t*)(i2c_reg_bases[i] + 0x001C); /* Example offset for CTR_REG */
            volatile uint32_t* fifo_conf_reg = (volatile uint32_t*)(i2c_reg_bases[i] + 0x0024); /* Example offset for FIFO_CONF_REG */
            volatile uint32_t* int_ena_reg = (volatile uint32_t*)(i2c_reg_bases[i] + 0x0034); /* Example offset for INT_ENA_REG */

            if (ctr_reg != NULL) { *ctr_reg &= ~I2C_CTR_TRANS_START_MASK; } /* Stop transfer */
            if (int_ena_reg != NULL) { *int_ena_reg = 0; } /* Disable all I2C interrupts */
            if (fifo_conf_reg != NULL)
            {
                 *fifo_conf_reg |= I2C_FIFO_RST_MASK; /* Reset FIFOs */
                 *fifo_conf_reg &= ~I2C_FIFO_RST_MASK; /* De-assert reset */
            }
            // Clear other config registers like I2C_TO_REG, I2C_SCL_LOW_PERIOD_REG etc. - complex, partial reset here.
        }
    }


    /* Disable SPI communication (SPI2, SPI3 - General Purpose SPIs) */
    /* SPI0/SPI1 are typically dedicated to flash/PSRAM and shouldn't be messed with */
    /* Assuming SPI2/SPI3 structures or base addresses are defined */
    volatile uint32_t* spi_reg_bases[] = {
        (volatile uint32_t*)DR_REG_SPI2_BASE, /* SPI2 */
        (volatile uint32_t*)DR_REG_SPI3_BASE  /* SPI3 */
    };
    const size_t SPI_GP_COUNT = 2; // General purpose SPIs

    const uint32_t SPI_CMD_USR_MASK = (1U << 24); /* Example bit for user command start in CMD */
    const uint32_t SPI_DMA_RX_FIFO_RST_MASK = (1U << 9); /* Example bit for RX FIFO reset in DMA_CONF */
    const uint32_t SPI_DMA_TX_FIFO_RST_MASK = (1U << 8); /* Example bit for TX FIFO reset in DMA_CONF */
    const uint32_t SPI_SLAVE_EN_MASK = (1U << 0); /* Example bit for slave enable in SLAVE */

    for (size_t i = 0; i < SPI_GP_COUNT; ++i)
    {
        if (spi_reg_bases[i] != NULL)
        {
            volatile uint32_t* cmd_reg = (volatile uint32_t*)(spi_reg_bases[i] + 0x0004); /* Example offset for CMD_REG */
            volatile uint32_t* ctrl_reg = (volatile uint32_t*)(spi_reg_bases[i] + 0x0008); /* Example offset for CTRL_REG */
            volatile uint32_t* dma_conf_reg = (volatile uint32_t*)(spi_reg_bases[i] + 0x0040); /* Example offset for DMA_CONF_REG */
            volatile uint32_t* slave_reg = (volatile uint32_t*)(spi_reg_bases[i] + 0x0064); /* Example offset for SLAVE_REG */
            volatile uint32_t* misc_reg = (volatile uint32_t*)(spi_reg_bases[i] + 0x0014); /* Example offset for MISC_REG */
             volatile uint32_t* int_ena_reg = (volatile uint32_t*)(spi_reg_bases[i] + 0x001C); /* Example offset for INT_ENA */


            if (cmd_reg != NULL) { *cmd_reg &= ~SPI_CMD_USR_MASK; } /* Stop any user command */
            if (ctrl_reg != NULL) { *ctrl_reg = 0; } /* Clear main control config */
            if (slave_reg != NULL) { *slave_reg &= ~SPI_SLAVE_EN_MASK; } /* Disable slave mode */
            if (misc_reg != NULL) { *misc_reg = 0; } /* Clear miscellaneous config */
            if (int_ena_reg != NULL) { *int_ena_reg = 0; } /* Disable all SPI interrupts */

            if (dma_conf_reg != NULL)
            {
                *dma_conf_reg |= SPI_DMA_RX_FIFO_RST_MASK | SPI_DMA_TX_FIFO_RST_MASK; /* Reset DMA FIFOs */
                *dma_conf_reg &= ~(SPI_DMA_RX_FIFO_RST_MASK | SPI_DMA_TX_FIFO_RST_MASK); /* De-assert reset */
                *dma_conf_reg &= ~((1U << 10) | (1U << 11)); // Example: Clear DMA_CONTINUOUS_RX/TX_EN
            }
             // More registers might need clearing (clock config, user registers, etc.)
        }
    }

    /* Configure all GPIOs as simple I/O (disable peripheral muxing) */
    /* This iterates through pins and sets their function select bits to GPIO (typically 0) */
    /* Reuses logic from GPIO_SAFEGUARD_Init for IO_MUX configuration */

    volatile uint32_t* iomux_gpio_reg_base = (volatile uint32_t*)(DR_REG_IO_MUX_BASE + 0x000); /* Example offset for IO_MUX_GPIO0_REG */
    const uint32_t FUN_SEL_MASK = (0x7U << 11); /* Example mask for function select bits in IO_MUX */

    for (int i = 0; i <= 39; ++i)
    {
        if ((i == 6) || (i == 7) || (i == 8) || (i == 9) || (i == 10) || (i == 11))
        {
             // Skip flash/PSRAM pins
             continue;
        }
         // Assuming IO_MUX_GPIOi_REG exists and is addressable via base + i*4
        volatile uint32_t* iomux_pin_reg = (volatile uint32_t*)(iomux_gpio_reg_base + (i * 4));
        if (iomux_pin_reg != NULL) {
             // Clear the function select bits to choose simple GPIO function (usually index 0)
             *iomux_pin_reg &= ~FUN_SEL_MASK;
        }
    }


#else
    /* Placeholder if necessary register definitions are not found */
    /* This section should ideally not be reached if proper headers are included */
    #warning "ESP32 peripheral register definitions not found. Registers_SAFEGUARD_Init is non-functional."
    // User: Add code here to disable peripherals using available HAL/driver functions if raw register access is not possible.
    // This is highly dependent on the SDK/framework used. Example (using ESP-IDF driver functions - NOT raw register access):
    // esp_intr_disable();
    // timer_pause(TIMG_GROUP0, TIMER_0);
    // timer_pause(TIMG_GROUP0, TIMER_1);
    // timer_pause(TIMG_GROUP1, TIMER_0);
    // timer_pause(TIMG_GROUP1, TIMER_1);
    // ledc_fade_func_stop(); // Stop any LEDC fades
    // esp_task_wdt_deinit(); // Disable task WDT if enabled
    // // Disabling peripherals like UART, I2C, SPI etc. involves specific driver deinit/disable functions.
    // // For GPIO muxing, gpio_set_direction or gpio_set_pull_mode indirectly handle some muxing.
#endif
}


/*******************************************************************************
 *                           Global Variable Declarations                      *
 ******************************************************************************/

/* Add any global variables here with 'extern' keyword if needed */
/* extern int my_global_variable; */


/*******************************************************************************
 *                            Function Declarations                            *
 ******************************************************************************/

/* Add any global function prototypes here */
/* void my_init_function(void); */


#endif // MAIN_H

/*******************************************************************************
 *                                End of File                                  *
 ******************************************************************************/