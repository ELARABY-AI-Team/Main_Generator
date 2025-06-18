/**
 * @file main.h
 * @brief Main project header file for ESP32.
 *
 * This header includes necessary standard and MCU-specific headers,
 * common typedefs, and core utility macros for ESP32 projects.
 * It also provides safeguard functions for initial hardware state configuration.
 *
 * @author Your Name/Company
 * @device ESP32
 * @creation date 2025-06-18
 * @copyright Copyright (c) 2025
 */

#ifndef ESP32_MAIN_H_
#define ESP32_MAIN_H_

/* ====================================================================================================================
 *                                               STANDARD LIBRARY INCLUDES
 * ====================================================================================================================
 */
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

/* ====================================================================================================================
 *                                               MCU-SPECIFIC INCLUDES
 * ====================================================================================================================
 */
// Include basic ESP-IDF system header
#include "esp_system.h"

// Include hardware register definitions for GPIO and DPORT (clock/reset gating)
// These paths assume a standard ESP-IDF installation structure.
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"
#include "soc/dport_reg.h"

/* ====================================================================================================================
 *                                                   TYPEDEFS
 * ====================================================================================================================
 */
typedef uint8_t  tbyte; /**< Unsigned 8-bit integer */
typedef uint16_t tword; /**< Unsigned 16-bit integer */
typedef uint32_t tlong; /**< Unsigned 32-bit integer */

/* ====================================================================================================================
 *                                                    MACROS
 * ====================================================================================================================
 */

/**
 * @brief Set a specific bit in a register.
 * @param reg Register to modify.
 * @param bit Bit number to set (0-indexed).
 */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/**
 * @brief Clear a specific bit in a register.
 * @param reg Register to modify.
 * @param bit Bit number to clear (0-indexed).
 */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/**
 * @brief Get the state of a specific bit in a register.
 * @param reg Register to read from.
 * @param bit Bit number to get (0-indexed).
 * @return The state of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/**
 * @brief Toggle a specific bit in a register.
 * @param reg Register to modify.
 * @param bit Bit number to toggle (0-indexed).
 */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/*
 * ESP32 interrupt handling is complex and typically managed by the RTOS (FreeRTOS in ESP-IDF).
 * Simple DI/EI macros like on 8-bit MCUs are not standard practice or easily implemented
 * portably in C without specific compiler intrinsics or assembly within ESP-IDF.
 * The RTOS functions (e.g., portENTER_CRITICAL, portEXIT_CRITICAL) are preferred.
 * HALT/NOP are typically compiler intrinsics or inline assembly.
 * We omit these simple macros to keep the header minimal and reflect common ESP32 practice.
 */

/* ====================================================================================================================
 *                                                  SAFEGUARD MACROS
 * ====================================================================================================================
 */

/**
 * @brief Configures all general-purpose GPIO pins to input mode with no pull-up/down.
 *
 * This macro iterates through all potential GPIO pins (0-39) and configures
 * their mode to input and disables internal pull-up and pull-down resistors.
 * It also clears the global GPIO output enable register.
 * Note: Some GPIOs (e.g., strapping pins, input-only) have specific behaviors
 * and this generic configuration might not override them or be fully applicable.
 * Consult the ESP32 Technical Reference Manual for specific pin details.
 */
#define GPIO_SAFEGUARD_Init() \
    do { \
        /* Disable all GPIO output enables */ \
        WRITE_PERI_REG(GPIO_ENABLE_W1TC_REG, 0xFFFFFFFF); /* Clear all bits in the output enable register */ \
        /* Configure all pins (0-39) as inputs with no pull-up/down */ \
        for (int i = 0; i < 40; i++) { \
            /* Access the pad configuration register for the pin */ \
            volatile gpio_ll_pin_reg_t* pin_reg = &GPIO.pin[i]; \
            /* Clear pull-up and pull-down enable bits */ \
            pin_reg->pull_up = 0; \
            pin_reg->pull_down = 0; \
            /* Ensure digital function is selected if applicable, though clearing output enable is primary for input */ \
            /* Note: Selecting input vs output is handled by the ENABLE register primarily. */ \
        } \
    } while(0)

/**
 * @brief Disables clocks for most unused peripherals via DPORT clock gating.
 *
 * This macro clears the clock enable bits for common peripherals in the DPORT_PERIP_CLK_EN_REG.
 * This helps reduce power consumption and ensures peripherals are in a known, disabled state
 * before being explicitly initialized.
 * Watchdog timers (WDTs) and essential system clocks are *not* disabled by this macro
 * to avoid system instability or resets.
 * Consult the ESP32 Technical Reference Manual and soc/dport_reg.h for a full list of bits.
 */
#define Registers_SAFEGUARD_Init() \
    do { \
        uint32_t clear_mask = 0; \
        /* Build a mask to clear clocks for common peripherals */ \
        clear_mask |= DPORT_SPI0_CLK_EN_M;    /* SPI0 */ \
        clear_mask |= DPORT_SPI1_CLK_EN_M;    /* SPI1 (SPI Flash) - Be cautious if using external flash/PSRAM via SPI */ \
        clear_mask |= DPORT_SPI2_CLK_EN_M;    /* SPI2 */ \
        clear_mask |= DPORT_SPI3_CLK_EN_M;    /* SPI3 */ \
        clear_mask |= DPORT_UART_CLK_EN_M;    /* UART0 - Be cautious if needed for console output */ \
        clear_mask |= DPORT_UART1_CLK_EN_M;   /* UART1 */ \
        clear_mask |= DPORT_UART2_CLK_EN_M;   /* UART2 */ \
        clear_mask |= DPORT_I2C_EXT0_CLK_EN_M;/* I2C0 */ \
        clear_mask |= DPORT_I2C_EXT1_CLK_EN_M;/* I2C1 */ \
        clear_mask |= DPORT_TIMERS_CLK_EN_M;  /* Timer Group 0 & 1 */ \
        /* clear_mask |= DPORT_SYSTIMER_CLK_EN_M; Keep SYSTIMER enabled as it's often crucial */ \
        /* clear_mask |= DPORT_WDG_CLK_EN_M; Do NOT disable Watchdog Timer clock */ \
        clear_mask |= DPORT_ADC_CLK_EN_M;     /* ADC */ \
        clear_mask |= DPORT_DAC_CLK_EN_M;     /* DAC */ \
        clear_mask |= DPORT_I2S0_CLK_EN_M;    /* I2S0 */ \
        clear_mask |= DPORT_I2S1_CLK_EN_M;    /* I2S1 */ \
        clear_mask |= DPORT_RMT_CLK_EN_M;     /* RMT */ \
        clear_mask |= DPORT_TWAI_CLK_EN_M;    /* TWAI (CAN) */ \
        \
        /* Apply the mask to the peripheral clock enable register */ \
        WRITE_PERI_REG(DPORT_PERIP_CLK_EN_REG, READ_PERI_REG(DPORT_PERIP_CLK_EN_REG) & ~clear_mask); \
        \
        /* Optional: Assert reset for these peripherals */ \
        /* WRITE_PERI_REG(DPORT_PERIP_RST_EN_REG, READ_PERI_REG(DPORT_PERIP_RST_EN_REG) | clear_mask); */ \
        /* Add a small delay or synchronization barrier if needed after clock/reset changes */ \
        asm volatile("nop"); /* Simple instruction barrier */ \
\
    } while(0)


#endif /* _MAIN_H_ */