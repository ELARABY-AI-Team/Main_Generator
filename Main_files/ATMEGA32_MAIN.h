/**
 * @file main.h
 * @brief Main header file for embedded projects.
 * @author Technology Inovation Software Team
 * @device ESP32
 * @creation date 2025-07-07
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H
#define MAIN_H

/*------------------------------------------------------------------------------
* Includes
------------------------------------------------------------------------------*/

/* MCU-specific includes */
/* Include necessary ESP-IDF SOC headers for register definitions */
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/timer_group_reg.h"
#include "soc/ledc_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/uart_reg.h"
#include "soc/i2c_reg.h"
#include "soc/spi_reg.h"
#include "soc/sens_reg.h"
#include "esp_cpu.h" /* For esp_cpu_intr_disable/enable */
#include "esp_system.h" /* For system functions like esp_restart */


/* Standard C includes */
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

/*------------------------------------------------------------------------------
* Typedefs
------------------------------------------------------------------------------*/

/** @brief Alias for unsigned 8-bit integer. */
typedef uint8_t tbyte;

/** @brief Alias for unsigned 16-bit integer. */
typedef uint16_t tword;

/** @brief Alias for unsigned 32-bit integer. */
typedef uint32_t tlong;

/*------------------------------------------------------------------------------
* Core Macros
------------------------------------------------------------------------------*/

/**
 * @brief Sets a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 */
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))

/**
 * @brief Clears a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 */
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))

/**
 * @brief Gets the value of a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 * @return The value of the bit (0 or 1).
 */
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)

/**
 * @brief Toggles a specific bit in a register.
 * @param reg The register variable.
 * @param bit The bit position (0-indexed).
 */
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

/**
 * @brief Disables global interrupts.
 * This uses the ESP-IDF CPU abstraction for disabling interrupts.
 */
#define DI() esp_cpu_intr_disable()

/**
 * @brief Enables global interrupts.
 * This uses the ESP-IDF CPU abstraction for enabling interrupts.
 */
#define EI() esp_cpu_intr_enable()

/**
 * @brief Disables global interrupts. Alias for DI().
 */
#define Global_Int_Disable() DI()

/**
 * @brief Enables global interrupts. Alias for EI().
 */
#define Global_Int_Enable() EI()

/**
 * @brief Halts program execution.
 * On ESP32, this is typically an infinite loop.
 */
#define HALT() while(1)

/**
 * @brief Executes a No Operation instruction.
 * Uses GCC intrinsic for NOP.
 */
#define NOP() __asm__("nop")

/*------------------------------------------------------------------------------
* Safeguard Macros
------------------------------------------------------------------------------*/

/**
 * @brief Initializes GPIOs to a safe default state.
 * Sets output value to 0, configures as inputs, disables pull-up/down,
 * disables wake-up, and sets function select to GPIO (0).
 * This implementation uses direct register access for ESP32 (ESP32-D0WDQ6 chip registers).
 * Note: This function performs low-level hardware access and may conflict
 *       with standard MISRA rules related to pointer casting and direct memory access.
 */
#define GPIO_SAFEGUARD_Init() do { \
    /* Configure all GPIOs (0-39) */ \
    tlong i; \
    for (i = 0; i <= 39; ++i) { \
        /* Set output data/value to 0 */ \
        if (i < 32) { \
            WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1U << i)); \
        } else { \
            WRITE_PERI_REG(GPIO_OUT1_W1TC_REG, (1U << (i - 32))); \
        } \
        \
        /* Configure as input (disable output enable) */ \
        if (i < 32) { \
            CLEAR_PERI_REG_BITS(GPIO_ENABLE_REG, 1, i); \
        } else { \
            CLEAR_PERI_REG_BITS(GPIO_ENABLE1_REG, 1, (i - 32)); \
        } \
        \
        /* Disable pull-up, pull-down, and set function select to GPIO (0) */ \
        /* Also disable input enable (FUN_IE) for extra safety, though often needed for input */ \
        /* The base address for IO_MUX_GPIO*_REG is IO_MUX_GPIO0_REG */ \
        /* Bits in IO_MUX_GPIO*_REG: FUN_PU (bit 7), FUN_PD (bit 6), FUN_IE (bit 9), FUN_SEL (bits 5:0) */ \
        /* Let's clear FUN_PU, FUN_PD, FUN_IE, and FUN_SEL */ \
        /* Using CLEAR_PERI_REG_MASK for specific bits */ \
        /* FUN_SEL is 6 bits (0x3F) */ \
        /* Combined mask for FUN_PU, FUN_PD, FUN_IE, FUN_SEL: (1<<7)|(1<<6)|(1<<9)|(0x3F) */ \
        /* Value to write (0) to these masked bits */ \
        WRITE_PERI_REG(IO_MUX_GPIO0_REG + (i * 4U), \
                       READ_PERI_REG(IO_MUX_GPIO0_REG + (i * 4U)) & \
                       ~((1U << IO_MUX_GPIO_FUN_PU_S) | (1U << IO_MUX_GPIO_FUN_PD_S) | \
                         (1U << IO_MUX_GPIO_FUN_IE_S) | (IO_MUX_GPIO_FUN_SEL_V << IO_MUX_GPIO_FUN_SEL_S))); \
        \
        /* Disable wake-up enable for this pin */ \
        /* GPIO_PIN_REG(i) contains wake-up enable bits INT_ENA_WAKEUP */ \
        /* CLEAR_PERI_REG_BITS(GPIO_PIN_REG(i), GPIO_PIN_INT_ENA_WAKEUP_M, GPIO_PIN_INT_ENA_WAKEUP_S); */ \
        /* GPIO_PIN register bits are specific to each pin. It's easier to clear the entire INT_ENA field or use the W1TC register */ \
        /* Let's clear the INT_ENA_WAKEUP bit. It's part of the INT_ENA_REG */ \
        /* Clearing the WAKEUP bits in the INT_ENA registers */ \
        if (i < 32) { \
            /* INT_ENA register for GPIO0-31 */ \
            CLEAR_PERI_REG_BITS(GPIO_PIN_REG(i), GPIO_PIN_INT_ENA_WAKEUP_M, GPIO_PIN_INT_ENA_WAKEUP_S); \
        } else { \
            /* INT_ENA registers for GPIO32-39 are separate */ \
            /* The wake-up enable bit is GPIO_PIN_INT_ENA_WAKEUP_M within GPIO_PIN_REG(i) */ \
            CLEAR_PERI_REG_BITS(GPIO_PIN_REG(i), GPIO_PIN_INT_ENA_WAKEUP_M, GPIO_PIN_INT_ENA_WAKEUP_S); \
        } \
    } \
} while(0)


/**
 * @brief Initializes various peripherals to a safe default state.
 * Disables global interrupts, timers, PWM (LEDC), watchdog timers,
 * ADC, UARTs, I2Cs, and SPIs. Also configures all GPIOs as I/O (done in GPIO_SAFEGUARD_Init).
 * This implementation uses direct register access for ESP32 (ESP32-D0WDQ6 chip registers).
 * Note: This function performs low-level hardware access and may conflict
 *       with standard MISRA rules related to pointer casting and direct memory access.
 *       Disabling certain crucial peripherals like the RTC WDT might make the system unstable
 *       depending on the bootloader and startup code configuration.
 */
#define Registers_SAFEGUARD_Init() do { \
    tlong j; \
    \
    /* Disable global interrupts */ \
    DI(); \
    \
    /* Disable Timer Group 0 (Timer 0 and Timer 1) */ \
    WRITE_PERI_REG(TIMG_T0_CTRL_REG(0), 0); /* TIMG0 Timer0 Ctrl */ \
    WRITE_PERI_REG(TIMG_T1_CTRL_REG(0), 0); /* TIMG0 Timer1 Ctrl */ \
    \
    /* Disable Timer Group 1 (Timer 0 and Timer 1) */ \
    WRITE_PERI_REG(TIMG_T0_CTRL_REG(1), 0); /* TIMG1 Timer0 Ctrl */ \
    WRITE_PERI_REG(TIMG_T1_CTRL_REG(1), 0); /* TIMG1 Timer1 Ctrl */ \
    \
    /* Disable LEDC (PWM) - Disable all channels and timers */ \
    /* There are 16 LEDC channels (0-15) */ \
    for (j = 0; j < 16; ++j) { \
        WRITE_PERI_REG(LEDC_CH_CONF0_REG(j), 0); /* Clear channel config */ \
        WRITE_PERI_REG(LEDC_CH_DUTY_REG(j), 0);   /* Clear duty cycle */ \
    } \
    /* Disable LEDC timers (0-3) - configure to use no clock or stop */ \
    for (j = 0; j < 4; ++j) { \
        WRITE_PERI_REG(LEDC_TIMER_CONF_REG(j), 0); /* Clear timer config */ \
    } \
    \
    /* Disable Watchdog Timers */ \
    /* Disable TIMG0 WDT */ \
    WRITE_PERI_REG(TIMG_WDT_CONFIG_REG(0), 0); /* WDT_EN=0 */ \
    /* Disable TIMG1 WDT */ \
    WRITE_PERI_REG(TIMG_WDT_CONFIG_REG(1), 0); /* WDT_EN=0 */ \
    /* Disable RTC WDT (Careful: This might stop the system if not managed by bootloader) */ \
    CLEAR_PERI_REG_MASK(RTC_CNTL_WDTCONFIG_REG, RTC_CNTL_WDT_FLASHBOOT_MOD_EN_M | RTC_CNTL_WDT_SYS_RESET_LENGTH_M | RTC_CNTL_WDT_CPU_RESET_LENGTH_M); \
    WRITE_PERI_REG(RTC_CNTL_WDTFEED_REG, 0); /* Clear feed register? Or just rely on config=0 */ \
    \
    /* Disable ADC (SAR ADC1 and ADC2) */ \
    /* Clear control registers - this is a simplified approach */ \
    WRITE_PERI_REG(APB_ADC_CTRL_REG, 0); \
    WRITE_PERI_REG(SENS_SAR_READ_CTRL_REG, 0); \
    WRITE_PERI_REG(SENS_SAR_MEAS_CTRL_REG, 0); \
    WRITE_PERI_REG(SENS_SAR_ATTEN1_REG, 0); /* Clear attenuation settings */ \
    WRITE_PERI_REG(SENS_SAR_ATTEN2_REG, 0); \
    WRITE_PERI_REG(SENS_SAR_POWER_XPD_SAR_REG, 0); /* Power down ADC units */ \
    \
    /* Disable UARTs (UART0, UART1, UART2) */ \
    /* Clear control registers - this is a simplified approach */ \
    WRITE_PERI_REG(UART_CONF0_REG(0), 0); \
    WRITE_PERI_REG(UART_CONF0_REG(1), 0); \
    WRITE_PERI_REG(UART_CONF0_REG(2), 0); \
    \
    /* Disable I2Cs (I2C0, I2C1) */ \
    /* Clear control registers - this is a simplified approach */ \
    WRITE_PERI_REG(I2C_CTR_REG(0), 0); \
    WRITE_PERI_REG(I2C_CTR_REG(1), 0); \
    \
    /* Disable SPI communication (SPI2, SPI3 - user SPIs) */ \
    /* Clear control registers - this is a simplified approach */ \
    WRITE_PERI_REG(SPI_CMD_REG(2), 0); \
    WRITE_PERI_REG(SPI_USER_REG(2), 0); \
    WRITE_PERI_REG(SPI_SLAVE_REG(2), 0); \
    WRITE_PERI_REG(SPI_CTRL_REG(2), 0); \
    \
    WRITE_PERI_REG(SPI_CMD_REG(3), 0); \
    WRITE_PERI_REG(SPI_USER_REG(3), 0); \
    WRITE_PERI_REG(SPI_SLAVE_REG(3), 0); \
    WRITE_PERI_REG(SPI_CTRL_REG(3), 0); \
    \
    /* Configure all GPIOS as I/O (not special function registers) - done in GPIO_SAFEGUARD_Init */ \
    /* Note: The IO_MUX function select is set to 0 (GPIO function) in GPIO_SAFEGUARD_Init */ \
    \
    /* Re-enable interrupts only if they were enabled before calling this function (optional, but safer) */ \
    /* However, the request is just to disable global interrupts as part of this safeguard. */ \
    /* EI(); */ /* Do NOT re-enable here based on the specific request. */ \
\
} while(0)


/*------------------------------------------------------------------------------
* Global Variables / Function Prototypes
------------------------------------------------------------------------------*/

/* Add global variables or function prototypes here if needed by the project */
/* Example: extern void SomePeripheral_Init(void); */

#endif /* MAIN_H */