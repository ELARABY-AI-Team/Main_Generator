/**
 * @file main.h
 * @brief Main header file for the ESP32 microcontroller project.
 *        Contains essential types, macros, and core initialization safeguards.
 * @author Technology Innovation Software Team
 * @date 2025-07-07
 * @device ESP32
 * @standard MISRA C
 * @copyright ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP
 */

#ifndef MAIN_H
#define MAIN_H

/*--- Includes ---------------------------------------------------------------*/

/* MCU-specific includes (Espressif SOC low-level headers) */
/* These headers provide definitions for peripheral registers and structures */
#include <soc/soc.h>              /* Base addresses and general SOC definitions */
#include <soc/gpio_reg.h>         /* GPIO register definitions */
#include <soc/gpio_struct.h>      /* GPIO register struct definitions */
#include <soc/timer_group_reg.h>  /* Timer Group register definitions */
#include <soc/timer_group_struct.h> /* Timer Group register struct definitions */
#include <soc/ledc_reg.h>         /* LEDC register definitions (often used for PWM) */
#include <soc/mcpwm_reg.h>        /* MCPWM register definitions (often used for PWM) */
#include <soc/adc_reg.h>          /* ADC register definitions */
#include <soc/uart_reg.h>         /* UART register definitions */
#include <soc/i2c_reg.h>          /* I2C register definitions */
#include <soc/spi_reg.h>          /* SPI register definitions */
#include <soc/rtc_cntl_reg.h>     /* RTC Control register definitions */
#include <soc/dport_reg.h>        /* DPORT (System) register definitions */
#include <xtensa/xtruntime.h>     /* Xtensa runtime/intrinsics for interrupt control */
#include <freertos/FreeRTOS.h>    /* Required for portENTER_CRITICAL if used */
#include <freertos/task.h>        /* Required for portENTER_CRITICAL if used */


/* Standard C Headers */
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


/*--- Typedefs ---------------------------------------------------------------*/

/**
 * @typedef tbyte
 * @brief Alias for uint8_t (8-bit unsigned integer).
 */
typedef uint8_t tbyte;

/**
 * @typedef tword
 * @brief Alias for uint16_t (16-bit unsigned integer).
 */
typedef uint16_t tword;

/**
 * @typedef tlong
 * @brief Alias for uint32_t (32-bit unsigned integer).
 */
typedef uint32_t tlong;


/*--- Core Macros ------------------------------------------------------------*/

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
 *        Note: This uses a low-level Xtensa intrinsic to set the PS register's interrupt level.
 *        In a FreeRTOS environment, consider using portENTER_CRITICAL() for safer context handling.
 */
#define Global_Int_Disable()    XTOS_DISABLE_ALL_INTERRUPTS() // Sets PS.INTLEVEL to 15

/**
 * @brief Enables global interrupts.
 *        Note: This sets the PS register's interrupt level back to a default state (usually 0 or based on compiler/RTOS).
 *        In a FreeRTOS environment, consider using portEXIT_CRITICAL() for safer context handling.
 */
#define Global_Int_Enable()     XTOS_RESTORE_INTLEVEL(0)      // Restores previous PS.INTLEVEL or sets to 0

/**
 * @brief Executes a No Operation (NOP) instruction.
 */
#define NOP()                   __asm__ volatile ("nop")

/* HALT() macro is not standardly available or recommended for user code
 * in typical ESP32 environments due to RTOS and watchdog timers. */
/* #define HALT() while(1) */


/*--- SAFEGUARD MACROS (Implemented) -----------------------------------------*/

/**
 * @brief Initializes GPIOs to a safe, known state (inputs, output low, pull-up/down disabled, wake-up disabled).
 *        This function directly manipulates low-level GPIO registers for all possible pins (0-39 on ESP32).
 *        Note: Using direct register access bypasses the ESP-IDF drivers/HAL.
 */
static inline void GPIO_SAFEGUARD_Init(void)
{
    /* Set all outputs to low by writing 1 to the clear registers */
    /* GPIO0-31 */
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, 0xFFFFFFFFU);
    /* GPIO32-39 (uses lower bits of OUT1) */
    WRITE_PERI_REG(GPIO_OUT1_W1TC_REG, 0xFFFFFFFFU);

    /* Configure all pins as inputs by disabling the output enable */
    /* GPIO0-31 */
    WRITE_PERI_REG(GPIO_ENABLE_W1TC_REG, 0xFFFFFFFFU);
    /* GPIO32-39 (uses lower bits of ENABLE1) */
    WRITE_PERI_REG(GPIO_ENABLE1_W1TC_REG, 0xFFFFFFFFU);

    /* Disable pull-up/down resistors and wake-up for all GPIO pins (0-39) */
    /* Iterate through all GPIO pins */
    for (int i = 0; i < GPIO_PIN_COUNT; i++)
    {
        /* Pointer to the PIN configuration register for the current pin */
        volatile uint32_t *pin_reg = &GPIO.pin[i].pin_pad_driver.val; /* Assuming pin_pad_driver field exists and has .val */

        /* Read modify write to clear PULLUP_EN, PULLDOWN_EN, and WAKEUP_ENABLE bits */
        uint32_t reg_val = READ_PERI_REG(pin_reg);

        /* Clear PULLUP_EN, PULLDOWN_EN (specific bit locations in the PIN register) */
        /* Note: Specific bit masks might vary slightly between ESP32 revisions/headers. */
        /* Using definitions from soc/gpio_reg.h or soc/gpio_struct.h */
        reg_val &= ~(GPIO_PIN_PULLUP_EN_M | GPIO_PIN_PULLDWN_EN_M | GPIO_PIN_WAKEUP_ENABLE_M);

        /* Write the modified value back */
        WRITE_PERI_REG(pin_reg, reg_val);

        /* Additionally, clear the interrupt type for the pin */
        WRITE_PERI_REG(GPIO_PIN_INT_TYPE_REG(i), 0U); /* Assuming GPIO_PIN_INT_TYPE_REG(i) macro or similar exists */
                                                     /* In soc/gpio_struct.h this is GPIO.pin[i].int_type */
        GPIO.pin[i].int_type = 0; /* Using struct access if available */
    }
}

/**
 * @brief Disables various peripherals and configures GPIOs away from special functions.
 *        This function directly manipulates low-level peripheral registers.
 *        Note: Using direct register access bypasses the ESP-IDF drivers/HAL.
 *        Configuring *all* GPIOs away from *any* special function is complex via registers,
 *        this primarily focuses on disabling the *peripherals* which typically releases
 *        their control over GPIO pins via the GPIO matrix.
 */
static inline void Registers_SAFEGUARD_Init(void)
{
    /* Disable global interrupts using a low-level intrinsic */
    XTOS_DISABLE_ALL_INTERRUPTS();

    /* Disable Timers (TIMG0, TIMG1) */
    TIMG0.hw_timer[0].config.en = 0;
    TIMG0.hw_timer[1].config.en = 0;
    TIMING_FORCE_INVERT_REG(TIMG0_BASE); // Ensure writes propagate (common ESP32 register access pattern)
    TIMG1.hw_timer[0].config.en = 0;
    TIMG1.hw_timer[1].config.en = 0;
    TIMING_FORCE_INVERT_REG(TIMG1_BASE); // Ensure writes propagate

    /* Disable Pulse Width Modulation (PWM) - LEDC and MCPWM */
    /* Reset LEDC module */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST);
    /* Reset MCPWM modules (MCPWM0 and MCPWM1) */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_MCPWM0_RST | DPORT_MCPWM1_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_MCPWM0_RST | DPORT_MCPWM1_RST);


    /* Disable Watchdog Timers (TIMG WDTs, Main System WDT, RTC WDT) */
    TIMG0.wdt_config.en = 0;
    TIMING_FORCE_INVERT_REG(TIMG0_BASE); // Ensure writes propagate
    TIMG1.wdt_config.en = 0;
    TIMING_FORCE_INVERT_REG(TIMG1_BASE); // Ensure writes propagate
    /* Disable DPORT System WDT */
    CLEAR_PERI_REG_MASK(DPORT_SYSCLK_CONF_REG, DPORT_SOC_WDT_RST_EN);
    /* Disable RTC WDT */
    RTC_CNTL.wdtconfig0.en = 0;

    /* Disable Input Capture Unit (ICU) - Functionality typically part of Timers or RMT. */
    /* Disabling Timers and potentially RMT (if used) covers this. RMT reset: */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);


    /* Disable Analog to Digital Converter (ADC) */
    /* Disable SAR ADC force start and enable bits */
    CLEAR_PERI_REG_MASK(APB_CTRL_SARADC_CTRL_REG, APB_CTRL_SARADC_START_FORCE | APB_CTRL_SARADC_EN);
    /* Power down ADC digital/analog parts via RTC_CNTL (simplified example) */
    RTC_CNTL.dig_pwc.sar_pwc_force = 1; // Force power down
    RTC_CNTL.dig_pwc.sar_pwc_m = 0;     // Mask power down bit
    RTC_CNTL.dig_pwc.sar_pwc = 0;       // Clear power down bit
    SET_PERI_REG_MASK(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_SAR_PWC_M); // Re-enable power down mask (clears it)

    /* Disable UART communication (UART0, UART1, UART2) */
    /* Reset UART modules */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART_RST); /* Resets UART0 */
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART_RST);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART1_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART1_RST);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART2_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART2_RST);

    /* Disable I2C communication (I2C0, I2C1) */
    /* Reset I2C modules */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_RST); /* Resets I2C0 */
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_RST);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C1_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C1_RST);


    /* Disable SPI communication (SPI2, SPI3 - Avoid touching SPI0/SPI1 used for flash/PSRAM) */
    /* Reset SPI modules */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI2_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI2_RST);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);


    /* Configure all GPIOs as Input/Output (I/O) not special function registers */
    /* Disabling the peripheral modules above typically releases the GPIO matrix connections */
    /* controlled by those peripherals, allowing the pin's core GPIO function (set by */
    /* GPIO_SAFEGUARD_Init) to take precedence. Explicitly re-configuring the GPIO matrix */
    /* for *all* pins to a neutral state is complex and varies per peripheral function. */
    /* The GPIO_SAFEGUARD_Init function already sets the basic GPIO properties (direction, output). */
}


#endif /* MAIN_H */