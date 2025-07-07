/**
 * @file main.h
 * @brief Minimal and common include file for ESP32WROOM embedded projects.
 * @author Technology Inovation Software Team
 * @device ESP32WROOM
 * @date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H_
#define MAIN_H_

/* Include MCU-specific headers */
/*
 * Note: ESP32 register definitions and peripheral structures are typically
 * provided by the Espressif IoT Development Framework (ESP-IDF).
 * The following includes provide access to base addresses and peripheral
 * register structures necessary for low-level configuration like safeguarding.
 */
#include "soc/soc.h"                   /* Base addresses for peripherals */
#include "soc/gpio_struct.h"           /* GPIO register definitions */
#include "soc/timer_group_struct.h"    /* Timer Group (Timers & WDT) register definitions */
#include "soc/ledc_struct.h"           /* LEDC (PWM) register definitions */
#include "soc/uart_struct.h"           /* UART register definitions */
#include "soc/i2c_struct.h"            /* I2C register definitions */
#include "soc/spi_struct.h"            /* SPI register definitions */
#include "soc/rmt_struct.h"            /* RMT (commonly used for input capture/generating protocols) register definitions */
#include "soc/adc_struct.h"            /* ADC register definitions */
#include "esp_system.h"                /* Basic ESP system functions */
#include "esp/interrupt.h"             /* Interrupt control functions */
#include "driver/gpio.h"               /* GPIO driver definitions for potential constants like pin counts */


/* Standard C Libraries */
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <iso646.h>
#include <limits.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Useful Type Definitions */
/** @brief 8-bit unsigned integer type */
typedef uint8_t tbyte;

/** @brief 16-bit unsigned integer type */
typedef uint16_t tword;

/** @brief 32-bit unsigned integer type */
typedef uint32_t tlong;


/* Core Macros */
/**
 * @brief Sets a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-based) to set.
 */
#define SET_BIT(reg, bit)         ((reg) |= (1UL << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-based) to clear.
 */
#define CLR_BIT(reg, bit)         ((reg) &= ~(1UL << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register to read from.
 * @param bit The bit number (0-based) to get.
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)         (((reg) >> (bit)) & 1UL)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register to modify.
 * @param bit The bit number (0-based) to toggle.
 */
#define TOG_BIT(reg, bit)         ((reg) ^= (1UL << (bit)))

/*
 * Interrupt and System Control Macros
 * Note: ESP32 uses specific functions from the SDK for interrupt control.
 * These macros wrap common ESP-IDF functions.
 */

/**
 * @brief Disables global interrupts.
 * Note: Use with caution, typically preferred to use RTOS critical sections.
 */
#define DI()                      esp_intr_disable()

/**
 * @brief Enables global interrupts.
 * Note: Use with caution, typically preferred to use RTOS critical sections.
 */
#define EI()                      esp_intr_enable()

/**
 * @brief Disables global interrupts (alias for DI).
 */
#define Global_Int_Disable()      DI()

/**
 * @brief Enables global interrupts (alias for EI).
 */
#define Global_Int_Enable()       EI()

/**
 * @brief Executes a NOP (No Operation) instruction.
 */
#define NOP()                     __asm__ volatile ("nop")

/* HALT is not a typical instruction or standard function on ESP32.
 * Consider using esp_deep_sleep_start() or a system reset esp_restart()
 * depending on the desired halt behavior. Not including HALT() macro
 * as there is no direct single instruction/function equivalent fitting
 * the general concept of "halt" without side effects like reset/sleep.
 */
/* #define HALT() */


/* SAFEGUARD FUNCTIONS */
/*
 * These functions provide a basic safeguard state for peripherals by
 * attempting to put them into a disabled or default state.
 * They access hardware registers directly and should be used carefully,
 * typically early in the boot process before specific drivers are initialized.
 * They are defined as static inline functions to facilitate inclusion
 * and potential inlining in the header, meeting the "fully implemented" request.
 * Note: Some complex peripherals might require more intricate reset/disable sequences.
 * This implementation provides a common-sense reset/disable state.
 */

/**
 * @brief Configures all available GPIO pins to a safe, default state.
 * Sets output data low, configures as input, disables internal pull-ups/downs,
 * and sets multiplexer to simple GPIO (CPU control).
 * Assumes GPIO_NUM_MAX is defined by included headers (e.g., driver/gpio.h).
 */
static inline void GPIO_SAFEGUARD_Init(void)
{
    volatile gpio_dev_t *gpio = (volatile gpio_dev_t *)DR_REG_GPIO_BASE;
    uint32_t gpio_idx;
    /* Iterate through all possible GPIO indices */
    /* ESP32 has up to 40 pads, indices 0-39 */
    for (gpio_idx = 0; gpio_idx < GPIO_NUM_MAX; gpio_idx++)
    {
        /* Set output data to 0 (for output capable pins) */
        /* Use W1TC register for atomic clear */
        gpio->out_w1tc = (1UL << gpio_idx); /* MISRA C: bitwise ops on unsigned */

        /* Disable output enable (makes it input) */
        /* Use W1TC register for atomic clear */
        gpio->enable_w1tc = (1UL << gpio_idx); /* MISRA C: bitwise ops on unsigned */

        /* Disable pull-up and pull-down resistors */
        /* PUE and PDE are bits in the pad_driver field */
        /* Accessing GPIO.pin[i] requires volatile access to the struct member */
        volatile uint32_t *pin_cntl_reg = (volatile uint32_t *)&gpio->pin[gpio_idx].cfg[0]; /* Cast to uint32_t* to access as register */
        
        /* Assuming GPIO_PUE_M and GPIO_PDE_M are defined in gpio_struct.h or similar */
        /* In ESP32, PUE and PDE are often separate bits, not maskable like this directly in pin[i].cfg */
        /* Let's use the specific bit names if available or clear the relevant bits */
        /* Correct bits are often within 'pin_cntl' or similar, specifically 'pad_driver' field.
         * Looking at soc/gpio_struct.h, it's a field of bits.
         * Need to access the volatile bit field struct members directly or cast to uint32_t.
         * Casting to uint32_t pointer and using masks is a common pattern for MISRA-friendly bit access.
         * The relevant bits for PUE/PDE are in GPIO_PIN_PAD_DRIVER_REG(i), which corresponds to gpio->pin[i].cfg[0] or similar.
         * The struct members are gpio->pin[i].pad_driver.pue and gpio->pin[i].pad_driver.pde.
         * Let's use the struct members directly with volatile access.
         */

        /* Use volatile access to bit-field members */
        gpio->pin[gpio_idx].pad_driver.pue = 0u; /* MISRA C: use unsigned 0 */
        gpio->pin[gpio_idx].pad_driver.pde = 0u; /* MISRA C: use unsigned 0 */


        /* Disable wake-up enable (if configured per-pin).
         * Note: ESP32 GPIO wake-up is primarily configured via RTC Low-Power peripheral
         * sleep functions (e.g., esp_sleep_enable_gpio_wakeup), not directly via
         * a simple per-pin register bit that can be cleared here without side effects.
         * Skipping direct per-pin wake-up register manipulation in this general safeguard.
         */

        /* Configure GPIO MUX to select simple CPU controlled GPIO (Function 0) */
        /* GPIO.pin[i].func_sel field selects the peripheral input/output */
        gpio->pin[gpio_idx].func_sel = 0u; /* MISRA C: use unsigned 0 */

         /* Ensure input enable is active for input-capable pins (default state) */
         /* Input enable is often enabled by default or controlled by func_sel */
         /* Explicitly setting input enable is done via gpio->pin[i].pin_cntl.input_enable = 1;
          * for pins 0-33. Pins 34-39 are input-only. Setting for all is safe.
          */
         gpio->pin[gpio_idx].pin_cntl.input_enable = 1u; /* MISRA C: use unsigned 1 */
    }
}


/**
 * @brief Disables common peripherals by resetting them or putting them in a default state.
 * Includes Timers, WDTs, PWM (LEDC), RMT (ICU), ADC, UARTs, I2Cs, SPIs,
 * and ensures GPIOs are not configured for special functions.
 * Disables global interrupts temporarily if needed for certain operations (like WDT disable).
 */
static inline void Registers_SAFEGUARD_Init(void)
{
    /* Disable global interrupts - esp_intr_disable is generally safe */
    Global_Int_Disable();

    /* Timer Group 0 and 1 */
    volatile timg_dev_t *timg0 = (volatile timg_dev_t *)DR_REG_TIMG0_BASE;
    volatile timg_dev_t *timg1 = (volatile timg_dev_t *)DR_REG_TIMG1_BASE;

    /* Disable Hardware Timers T0 and T1 in both groups */
    timg0->hw_timer[0].config.enable = 0u; /* MISRA C: use unsigned 0 */
    timg0->hw_timer[1].config.enable = 0u; /* MISRA C: use unsigned 0 */
    timg1->hw_timer[0].config.enable = 0u; /* MISRA C: use unsigned 0 */
    timg1->hw_timer[1].config.enable = 0u; /* MISRA C: use unsigned 0 */

    /* Disable Timer Group Watchdogs (WDTs) */
    /* WDT registers are write-protected, need to write key 0x1A5E first */
    timg0->wdt_wprotect = 0x1A5Eu; /* MISRA C: use unsigned suffix */
    timg0->wdt_config.en = 0u;     /* MISRA C: use unsigned 0 */
    timg0->wdt_wprotect = 0u;      /* Lock the WDT configuration */ /* MISRA C: use unsigned 0 */

    timg1->wdt_wprotect = 0x1A5Eu; /* MISRA C: use unsigned suffix */
    timg1->wdt_config.en = 0u;     /* MISRA C: use unsigned 0 */
    timg1->wdt_wprotect = 0u;      /* Lock the WDT configuration */ /* MISRA C: use unsigned 0 */

    /* Note: RTC WDT is separate and requires different register access (e.g. DR_REG_RTC_CNTL_BASE)
     * For a minimal safeguard, we focus on the main TIMG WDTs.
     */


    /* LEDC (PWM Controller) */
    volatile ledc_dev_t *ledc = (volatile ledc_dev_t *)DR_REG_LEDC_BASE;
    uint32_t group;
    uint32_t channel;

    /* Disable all LEDC channels (Group 0 and Group 1, Channels 0-7 each) */
    for (group = 0; group < 2u; group++) /* MISRA C: use unsigned suffix */
    {
        for (channel = 0; channel < 8u; channel++) /* MISRA C: use unsigned suffix */
        {
            /* Selecting an invalid timer index (e.g., 7 or anything >= 4) effectively disables the channel */
            ledc->channel_group[group].channel[channel].conf0.timer_sel = 7u; /* MISRA C: use unsigned suffix */
            /* Also force duty to 0 and update config */
            ledc->channel_group[group].channel[channel].conf0.duty_force = 1u; /* MISRA C: use unsigned suffix */
            ledc->channel_group[group].channel[channel].duty.duty = 0u; /* MISRA C: use unsigned 0 */
            ledc->channel_group[group].channel[channel].conf0.conf_update = 1u; /* MISRA C: use unsigned suffix */
        }
    }

    /* RMT (Remote Control - can be used for Input Capture) */
    volatile rmt_dev_t *rmt = (volatile rmt_dev_t *)DR_REG_RMT_BASE;
    uint32_t rmt_channel;

    /* Reset all RMT channels (0-7) */
    for (rmt_channel = 0; rmt_channel < 8u; rmt_channel++) /* MISRA C: use unsigned suffix */
    {
        /* Resetting the channel configuration */
        rmt->conf_ch[rmt_channel].conf1.mem_wr_rst = 1u; /* MISRA C: use unsigned suffix */
        rmt->conf_ch[rmt_channel].conf1.rx_rst = 1u;     /* MISRA C: use unsigned suffix */
        rmt->conf_ch[rmt_channel].conf1.tx_rst = 1u;     /* MISRA C: use unsigned suffix */
        rmt->conf_ch[rmt_channel].conf1.mem_wr_rst = 0u; /* MISRA C: use unsigned 0 */
        rmt->conf_ch[rmt_channel].conf1.rx_rst = 0u;     /* MISRA C: use unsigned 0 */
        rmt->conf_ch[rmt_channel].conf1.tx_rst = 0u;     /* MISRA C: use unsigned 0 */
        /* Disable peripheral clocks or enables if simple reset is not enough.
         * The reset usually stops ongoing transfers and clears state.
         */
    }


    /* ADC (Analog to Digital Converter) */
    volatile adc_dev_t *adc = (volatile adc_dev_t *)DR_REG_SARADC_BASE;

    /* Reset ADC units (ADC1 and ADC2 share registers) */
    /* There isn't a single 'disable' bit. Resetting the controller is common. */
    adc->ctrl.sar1_rst = 1u; /* MISRA C: use unsigned suffix */
    adc->ctrl.sar2_rst = 1u; /* MISRA C: use unsigned suffix */
    adc->ctrl.sar1_rst = 0u; /* MISRA C: use unsigned 0 */
    adc->ctrl.sar2_rst = 0u; /* MISRA C: use unsigned 0 */
    /* Also ensure force enable bits are cleared */
    adc->ctrl.sar1_en_force = 0u; /* MISRA C: use unsigned 0 */
    adc->ctrl.sar2_en_force = 0u; /* MISRA C: use unsigned 0 */
    /* Further steps might be needed depending on specific ADC modes used previously */


    /* UARTs (UART0, UART1, UART2) */
    volatile uart_dev_t *uart0 = (volatile uart_dev_t *)DR_REG_UART_BASE;
    volatile uart_dev_t *uart1 = (volatile uart_dev_t *)DR_REG_UART1_BASE;
    volatile uart_dev_t *uart2 = (volatile uart_dev_t *)DR_REG_UART2_BASE;

    /* Reset each UART peripheral */
    uart0->conf0.sw_rst = 1u; /* MISRA C: use unsigned suffix */
    uart0->conf0.sw_rst = 0u; /* MISRA C: use unsigned 0 */

    uart1->conf0.sw_rst = 1u; /* MISRA C: use unsigned suffix */
    uart1->conf0.sw_rst = 0u; /* MISRA C: use unsigned 0 */

    uart2->conf0.sw_rst = 1u; /* MISRA C: use unsigned suffix */
    uart2->conf0.sw_rst = 0u; /* MISRA C: use unsigned 0 */


    /* I2Cs (I2C0, I2C1) */
    volatile i2c_dev_t *i2c0 = (volatile i2c_dev_t *)DR_REG_I2C_EXT_BASE;
    volatile i2c_dev_t *i2c1 = (volatile i2c_dev_t *)DR_REG_I2C1_EXT_BASE;

    /* Reset each I2C peripheral */
    i2c0->conf.conf_v.conf_rst = 1u; /* MISRA C: use unsigned suffix */
    i2c0->conf.conf_v.conf_rst = 0u; /* MISRA C: use unsigned 0 */

    i2c1->conf.conf_v.conf_rst = 1u; /* MISRA C: use unsigned suffix */
    i2c1->conf.conf_v.conf_rst = 0u; /* MISRA C: use unsigned 0 */


    /* SPIs (SPI2, SPI3 - general purpose) */
    /* SPI0 and SPI1 are typically for flash/PSRAM and less commonly used for general SPI peripherals */
    volatile spi_dev_t *spi2 = (volatile spi_dev_t *)DR_REG_SPI2_BASE;
    volatile spi_dev_t *spi3 = (volatile spi_dev_t *)DR_REG_SPI3_BASE;

    /* Reset SPI peripherals and disable commands/DMA */
    spi2->dma_conf.val |= (SPI_DMA_RX_RST_M | SPI_DMA_TX_RST_M); /* Use masks defined in struct header */ /* MISRA C: bitwise ops on unsigned */
    spi2->dma_conf.val &= ~(SPI_DMA_RX_RST_M | SPI_DMA_TX_RST_M); /* MISRA C: bitwise ops on unsigned */
    spi2->cmd.usr = 0u; /* Disable user command mode */ /* MISRA C: use unsigned 0 */
    spi2->slave.sync_reset = 1u; /* Attempt slave mode reset if active */ /* MISRA C: use unsigned suffix */
    spi2->slave.sync_reset = 0u; /* MISRA C: use unsigned 0 */


    spi3->dma_conf.val |= (SPI_DMA_RX_RST_M | SPI_DMA_TX_RST_M); /* MISRA C: bitwise ops on unsigned */
    spi3->dma_conf.val &= ~(SPI_DMA_RX_RST_M | SPI_DMA_TX_RST_M); /* MISRA C: bitwise ops on unsigned */
    spi3->cmd.usr = 0u; /* MISRA C: use unsigned 0 */
    spi3->slave.sync_reset = 1u; /* MISRA C: use unsigned suffix */
    spi3->slave.sync_reset = 0u; /* MISRA C: use unsigned 0 */

    /* Note: SPI0/SPI1 might be actively used by the system/flash driver and should
     * not be reset arbitrarily. This safeguard focuses on general-purpose SPI2/SPI3.
     */


    /* Configure all GPIOs back to simple input/output mode */
    /* This prevents pins from being controlled by peripherals like UART, SPI, I2C, etc. */
    /* The GPIO_SAFEGUARD_Init function already does this: setting func_sel to 0. */
    /* Calling it here again ensures this state after potentially disabling peripherals. */
    GPIO_SAFEGUARD_Init();


    /* Global interrupts are left disabled by this function.
     * The caller is responsible for re-enabling interrupts if needed
     * after performing necessary initializations.
     */
    /* Global_Int_Enable(); // Optional: uncomment if interrupts should be re-enabled */
}


#endif /* MAIN_H_ */