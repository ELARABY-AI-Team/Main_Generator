/**
 * @file main.h
 * @brief Central header file for ESP32 embedded projects.
 *        Includes common standard libraries, useful typedefs,
 *        and essential hardware safeguard macros.
 *
 * @author Technology Inovation Software Team
 * @device ESP32
 * @creation date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H
#define MAIN_H

/* MISRA C Note:
 * This header file includes direct register access macros
 * within the safeguard functions (GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init).
 * Direct register access may deviate from strict MISRA C guidelines
 * depending on the specific rule interpretation and project context (e.g., Rule 11.3/11.4).
 * In a production environment aiming for strict MISRA compliance,
 * these register accesses would typically be abstracted through a hardware abstraction layer (HAL)
 * or device-specific drivers, which might involve deviations managed at the module level.
 * This implementation provides the requested direct register control using
 * standard ESP-IDF SOC header definitions.
 */

/* ----------------------------------------------------------------------------
 *                                 Includes
 * ----------------------------------------------------------------------------
 */

/* MCU-specific include(s) */
/*
 * Including specific peripheral headers to access their structs and registers.
 * These paths assume a standard ESP-IDF environment or equivalent build setup
 * where SOC headers are in the include path.
 */
#include <soc/gpio_struct.h>
#include <soc/timer_group_struct.h>
#include <soc/rtc_cntl_struct.h>
#include <soc/ledc_struct.h>
#include <soc/sens_struct.h>
#include <soc/uart_struct.h>
#include <soc/i2c_struct.h>
#include <soc/spi_struct.h>
#include <soc/rmt_struct.h> // Used for RMT, which can implement Input Capture
#include <soc/apb_ctrl_struct.h> // Might be needed for system resets/clocks
#include <esp_intr_noniram.h>   // For portable interrupt disable/enable

/* Standard C headers */
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


/* ----------------------------------------------------------------------------
 *                                Typedefs
 * ----------------------------------------------------------------------------
 */

/**
 * @brief 8-bit unsigned integer type.
 */
typedef uint8_t tbyte;

/**
 * @brief 16-bit unsigned integer type.
 */
typedef uint16_t tword;

/**
 * @brief 32-bit unsigned integer type.
 */
typedef uint32_t tlong;


/* ----------------------------------------------------------------------------
 *                                 Core Macros
 * ----------------------------------------------------------------------------
 */

/**
 * @brief Sets a specific bit in a register.
 * @param reg Register variable.
 * @param bit Bit number (0-31).
 */
#define SET_BIT(reg, bit)           ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg Register variable.
 * @param bit Bit number (0-31).
 */
#define CLR_BIT(reg, bit)           ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg Register variable.
 * @param bit Bit number (0-31).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)           (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg Register variable.
 * @param bit Bit number (0-31).
 */
#define TOG_BIT(reg, bit)           ((reg) ^= (1U << (bit)))

/**
 * @brief Disable global interrupts.
 *        Uses ESP-IDF portable macro.
 */
#define DI()                        portDISABLE_INTERRUPTS()

/**
 * @brief Enable global interrupts.
 *        Uses ESP-IDF portable macro.
 */
#define EI()                        portENABLE_INTERRUPTS()

/**
 * @brief Alias for DI(). Disable global interrupts.
 */
#define Global_Int_Disable()        DI()

/**
 * @brief Alias for EI(). Enable global interrupts.
 */
#define Global_Int_Enable()         EI()

/**
 * @brief Halts program execution in an infinite loop.
 */
#define HALT()                      do { while(1); } while(0)

/**
 * @brief Inserts a No Operation instruction.
 */
#define NOP()                       __asm__ __volatile__("nop")


/* ----------------------------------------------------------------------------
 *                                SAFEGUARD MACROS
 * ----------------------------------------------------------------------------
 */

/**
 * @brief Configures all usable GPIOs to a safe, known state: input with no pulls/wakeup.
 *
 * Sets output state low (if any are configured as output), then configures
 * all pins as inputs, disables pull-ups, pull-downs, and wakeup capabilities.
 *
 * Note: Iterates through GPIOs 0-39. Not all pins may be bonded out or available
 * for general GPIO use on all ESP32 packages, but clearing their state registers
 * is harmless.
 */
#define GPIO_SAFEGUARD_Init() do { \
    /* Ensure all output drivers are set to low state if currently enabled */ \
    /* Using write-1-to-clear registers for atomic clearing */ \
    GPIO.out_w1tc = 0xFFFFFFFFU;     /* Clear output bits for GPIO0-31 */ \
    GPIO.out1_w1tc.val = 0xFFFFFFFFU; /* Clear output bits for GPIO32-63 (covers 32-39) */ \
    \
    /* Configure all pins as inputs by disabling output drivers */ \
    GPIO.enable_w1tc = 0xFFFFFFFFU;    /* Disable output for GPIO0-31 */ \
    GPIO.enable1_w1tc.val = 0xFFFFFFFFU;/* Disable output for GPIO32-63 (covers 32-39) */ \
    \
    /* Disable pull-up and pull-down resistors and open-drain for all pins (0-39) */ \
    for (int i = 0; i <= 39; ++i) { \
        GPIO.pin[i].pull_up = 0U; \
        GPIO.pin[i].pull_down = 0U; \
        GPIO.pin[i].open_drain_enable = 0U; \
        GPIO.pin[i].int_type = 0U; /* Disable pin interrupts */ \
    } \
    \
    /* Disable wakeup for all pins (0-39) */ \
    for (int i = 0; i <= 39; ++i) { \
       GPIO.pin[i].wakeup_enable = 0U; \
    } \
} while(0)

/**
 * @brief Disables various peripherals and configures GPIOs away from peripheral functions.
 *
 * Disables global interrupts, hardware timers, LEDC (PWM), Watchdog Timers,
 * RMT (Input Capture like functionality), ADC, UART, I2C, and SPI peripherals.
 * Also sets GPIO pin function select back to simple GPIO mode.
 *
 * Note: This is a basic safeguard. More complex peripherals (e.g., Wi-Fi, BT, USB, SDMMC, CAN)
 * and clock gating might require additional steps depending on the desired reset state.
 * Disabling WDTs might require specific sequences involving write protection, which
 * is simplified here by direct register write.
 */
#define Registers_SAFEGUARD_Init() do { \
    /* Disable global interrupts */ \
    DI(); \
    \
    /* Disable Timers (Timer Group 0 and 1, Timer 0 and 1 in each) */ \
    TG0.hw_timer[0].config.en = 0U; \
    TG0.hw_timer[1].config.en = 0U; \
    TG1.hw_timer[0].config.en = 0U; \
    TG1.hw_timer[1].config.en = 0U; \
    /* Clear pending timer interrupts */ \
    TG0.int_clr_timers.t0 = 1U; \
    TG0.int_clr_timers.t1 = 1U; \
    TG1.int_clr_timers.t0 = 1U; \
    TG1.int_clr_timers.t1 = 1U; \
    /* Disable local timer interrupt enables */ \
    TG0.internal_evt_timers.t0_intr_en = 0U; \
    TG0.internal_evt_timers.t1_intr_en = 0U; \
    TG1.internal_evt_timers.t0_intr_en = 0U; \
    TG1.internal_evt_timers.t1_intr_en = 0U; \
    \
    /* Disable LEDC (PWM) channels (0-15) */ \
    for (int i = 0; i < 16; ++i) { \
        LEDC.channel[i].conf0.sig_out_en = 0U; /* Disable output signal */ \
        LEDC.channel[i].conf0.clk_en = 0U;     /* Disable channel clock */ \
        LEDC.channel[i].conf1.duty_start = 0U; /* Reset duty cycle state */ \
        LEDC.channel[i].conf0.param_update = 1U; /* Apply settings */ \
    } \
    LEDC.int_ena.val = 0U; /* Disable LEDC interrupts */ \
    \
    /* Disable Watchdog Timers (TG0 WDT, TG1 WDT, RTC WDT) */ \
    /* Note: Disabling WDTs might require specific sequences or bypassing write protection */ \
    /* This is a simplified direct disable attempt */ \
    TG0.wdt_config0.en = 0U; \
    TG1.wdt_config0.en = 0U; \
    RTC_CNTL.wdtconfig0.en = 0U; \
    \
    /* Disable RMT (simulating ICU) channels (0-7) */ \
    for (int i = 0; i < 8; ++i) { \
        RMT.conf_ch[i].conf0.mem_rd_rst = 1U; /* Reset memory read pointer */ \
        RMT.conf_ch[i].conf0.mem_wr_rst = 1U; /* Reset memory write pointer */ \
        RMT.conf_ch[i].conf0.tx_start = 0U;   /* Stop transmit */ \
        RMT.conf_ch[i].conf0.rx_en = 0U;      /* Stop receive */ \
        RMT.conf_ch[i].conf1.mem_owner = 0U;  /* Set owner to RMT (not DMA) */ \
        RMT.conf_ch[i].conf0.carrier_en = 0U; /* Disable carrier */ \
        RMT.conf_ch[i].conf0.idle_level = 0U; /* Set idle level low */ \
        RMT.conf_ch[i].conf0.idle_out_en = 0U;/* Disable idle output */ \
    } \
    RMT.int_ena.val = 0U; /* Disable RMT interrupts */ \
    \
    /* Disable ADC (ADC1, ADC2) */ \
    /* Power down internal SAR ADCs */ \
    SENS.sar_meas_wait2.force_xpd_sar = 0U; /* Clears bits for both ADC1 and ADC2 */ \
    /* Disable ADC interrupts */ \
    SENS.sar_meas_ctrl2.meas_done_int_en = 0U; \
    \
    /* Disable UARTs (UART0, UART1, UART2) */ \
    /* Note: UART0 is often used for console, disabling might stop logging */ \
    UART0.conf0.val = 0U; /* Reset configuration */ \
    UART1.conf0.val = 0U; \
    UART2.conf0.val = 0U; \
    UART0.int_ena.val = 0U; /* Disable interrupts */ \
    UART1.int_ena.val = 0U; \
    UART2.int_ena.val = 0U; \
    /* Software reset (toggle sw_rst bit) */ \
    UART0.conf0.sw_rst = 1U; UART0.conf0.sw_rst = 0U; \
    UART1.conf0.sw_rst = 1U; UART1.conf0.sw_rst = 0U; \
    UART2.conf0.sw_rst = 1U; UART2.conf0.sw_rst = 0U; \
    \
    /* Disable I2C (I2C0, I2C1) */ \
    I2C0.ctr.val = 0U;      /* Reset control */ \
    I2C1.ctr.val = 0U; \
    I2C0.slave_conf.val = 0U; /* Reset slave config */ \
    I2C1.slave_conf.val = 0U; \
    I2C0.int_ena.val = 0U;  /* Disable interrupts */ \
    I2C1.int_ena.val = 0U; \
    /* I2C Software Reset - often done via system registers or specific sequence */ \
    /* A minimal approach is clearing control & interrupt registers. */ \
    \
    /* Disable SPI (SPI2, SPI3 - general purpose) */ \
    /* Note: SPI0/SPI1 often used for flash/PSRAM, typically not reset */ \
    SPI2.slave.val = 0U; SPI2.user.val = 0U; SPI2.pin.val = 0U; SPI2.ctrl.val = 0U; SPI2.cmd.val = 0U; SPI2.misc.val = 0U; SPI2.int_ena.val = 0U; \
    SPI3.slave.val = 0U; SPI3.user.val = 0U; SPI3.pin.val = 0U; SPI3.ctrl.val = 0U; SPI3.cmd.val = 0U; SPI3.misc.val = 0U; SPI3.int_ena.val = 0U; \
     /* SPI Software Reset - often done via system registers */ \
    /* A minimal approach is clearing control & interrupt registers. */ \
    \
    /* Configure all GPIOs as standard I/O, disconnecting them from peripheral matrix outputs */ \
    /* This is done by setting func_sel to 0 for all pins 0-39, selecting simple GPIO */ \
    /* Input connections are handled by GPIO_SAFEGUARD_Init by configuring input enable */ \
    for (int i = 0; i <= 39; ++i) { \
         GPIO.pin[i].func_sel = 0U; /* Set to simple GPIO function */ \
    } \
    \
} while(0)


#endif /* MAIN_H */