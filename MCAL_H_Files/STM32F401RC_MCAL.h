/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File
 *
 * This file provides the definitions and function prototypes for the Microcontroller
 * Abstraction Layer (MCAL) for the STM32F401RC microcontroller. It abstracts
 * direct register manipulation, providing a standardized API for peripheral
 * control as defined in API.json, and adheres to the coding rules in Rules.json.
 *
 * MCU: STM32F401RC
 *
 * Copyright (c) [Year] [Author/Company]
 * SPDX-License-Identifier: MIT
 */

#ifndef MCAL_H
#define MCAL_H

// Core MCU header file
#include "stm32f401xc.h" // Placeholder for the specific device header for STM32F401RC
#include <stdint.h>     // Standard integer types (uint8_t, uint16_t, uint32_t)
#include <stdbool.h>    // Standard boolean type (bool)
#include <stddef.h>     // Standard definitions (size_t)
#include <string.h>     // String manipulation functions (if needed by some APIs)
#include <stdio.h>      // Standard input/output functions (if needed by some APIs)
#include <stdlib.h>     // General utilities (if needed by some APIs)
#include <math.h>       // Mathematical functions (if needed by some APIs)

// ==============================================================================
// ------------------------------ Data Type Definitions -------------------------
// ==============================================================================

/**
 * @brief Defines a 8-bit unsigned integer type.
 */
typedef uint8_t tbyte;

/**
 * @brief Defines a 16-bit unsigned integer type.
 */
typedef uint16_t tword;

/**
 * @brief Defines a 32-bit unsigned integer type.
 */
typedef uint32_t tlong;

/**
 * @brief Defines a 8-bit signed integer type, used for signed bytes in APIs.
 */
typedef int8_t tsbyte;

/**
 * @brief Defines a 16-bit signed integer type, used for signed words in APIs.
 */
typedef int16_t tsword;

/**
 * @brief Defines a 32-bit signed integer type, used for signed longs in APIs.
 */
typedef int32_t tslong;

// ==============================================================================
// --------------------------------- Enum Definitions ---------------------------
// ==============================================================================

/**
 * @brief Enumeration for system voltage levels.
 */
typedef enum
{
    volt_3v = 0, /**< System voltage level 3V */
    volt_5v,     /**< System voltage level 5V */
    _volt_max
} t_sys_volt;

/**
 * @brief Enumeration for Low Voltage Detection (LVD) threshold levels.
 *        These levels correspond to common PVD (Programmable Voltage Detector) thresholds on STM32F4.
 */
typedef enum
{
    lvd_threshold_level_2v2 = 0, /**< LVD threshold around 2.2V (PVD Level 0) */
    lvd_threshold_level_2v4,     /**< LVD threshold around 2.4V (PVD Level 1) */
    lvd_threshold_level_2v6,     /**< LVD threshold around 2.6V (PVD Level 2) */
    lvd_threshold_level_2v8,     /**< LVD threshold around 2.8V (PVD Level 3) */
    lvd_threshold_level_2v9,     /**< LVD threshold around 2.9V (PVD Level 4) */
    lvd_threshold_level_3v1,     /**< LVD threshold around 3.1V (PVD Level 5) */
    lvd_threshold_level_3v3,     /**< LVD threshold around 3.3V (PVD Level 6) */
    lvd_threshold_level_3v5,     /**< Custom/Inferred LVD threshold, not a direct PVD Level */
    lvd_threshold_level_4v0,     /**< LVD threshold around 4.0V (PVD Level 7) */
    _lvd_threshold_max
} t_lvd_thrthresholdLevel;

/**
 * @brief Enumeration for UART channel selection.
 */
typedef enum
{
    uart_channel_1 = 0, /**< USART1 channel */
    uart_channel_2,     /**< USART2 channel */
    uart_channel_6,     /**< USART6 channel */
    _uart_channel_max
} t_uart_channel;

/**
 * @brief Enumeration for common UART baud rates.
 */
typedef enum
{
    uart_baud_rate_9600 = 0,   /**< 9600 baud */
    uart_baud_rate_19200,      /**< 19200 baud */
    uart_baud_rate_38400,      /**< 38400 baud */
    uart_baud_rate_57600,      /**< 57600 baud */
    uart_baud_rate_115200,     /**< 115200 baud */
    uart_baud_rate_230400,     /**< 230400 baud */
    uart_baud_rate_460800,     /**< 460800 baud */
    uart_baud_rate_921600,     /**< 921600 baud */
    _uart_baud_rate_max
} t_uart_baud_rate;

/**
 * @brief Enumeration for UART data length.
 */
typedef enum
{
    uart_data_length_8_bit = 0, /**< 8-bit data frame */
    uart_data_length_9_bit,     /**< 9-bit data frame */
    _uart_data_length_max
} t_uart_data_length;

/**
 * @brief Enumeration for UART stop bits.
 */
typedef enum
{
    uart_stop_bit_1 = 0,   /**< 1 stop bit */
    uart_stop_bit_0_5,     /**< 0.5 stop bits */
    uart_stop_bit_2,       /**< 2 stop bits */
    uart_stop_bit_1_5,     /**< 1.5 stop bits */
    _uart_stop_bit_max
} t_uart_stop_bit;

/**
 * @brief Enumeration for UART parity modes.
 */
typedef enum
{
    uart_parity_none = 0, /**< No parity */
    uart_parity_even,     /**< Even parity */
    uart_parity_odd,      /**< Odd parity */
    _uart_parity_max
} t_uart_parity;

/**
 * @brief Enumeration for I2C channel selection.
 */
typedef enum
{
    i2c_channel_1 = 0, /**< I2C1 channel */
    i2c_channel_2,     /**< I2C2 channel */
    i2c_channel_3,     /**< I2C3 channel */
    _i2c_channel_max
} t_i2c_channel;

/**
 * @brief Enumeration for I2C clock speed.
 */
typedef enum
{
    i2c_clk_speed_standard_mode = 0, /**< 100 kHz Standard Mode */
    i2c_clk_speed_fast_mode,         /**< 400 kHz Fast Mode */
    _i2c_clk_speed_max
} t_i2c_clk_speed;

/**
 * @brief Type definition for I2C device address.
 */
typedef uint16_t t_i2c_device_address; // 7-bit or 10-bit address

/**
 * @brief Enumeration for I2C Acknowledge (ACK) control.
 */
typedef enum
{
    i2c_ack_enable = 0, /**< Enable Acknowledge */
    i2c_ack_disable,    /**< Disable Acknowledge */
    _i2c_ack_max
} t_i2c_ack;

/**
 * @brief Enumeration for I2C data length.
 */
typedef enum
{
    i2c_datalength_8_bit = 0, /**< 8-bit data transfer */
    i2c_datalength_16_bit,    /**< 16-bit data transfer (if supported, e.g., dual-byte access) */
    _i2c_datalength_max
} t_i2c_datalength;

/**
 * @brief Enumeration for SPI (CSI) channel selection.
 */
typedef enum
{
    spi_channel_1 = 0, /**< SPI1 channel */
    spi_channel_2,     /**< SPI2 channel */
    spi_channel_3,     /**< SPI3 channel */
    _spi_channel_max
} t_spi_channel;

/**
 * @brief Enumeration for SPI operational mode (Master/Slave).
 */
typedef enum
{
    spi_mode_master = 0, /**< SPI Master mode */
    spi_mode_slave,      /**< SPI Slave mode */
    _spi_mode_max
} t_spi_mode;

/**
 * @brief Enumeration for SPI clock polarity (CPOL).
 */
typedef enum
{
    spi_cpol_low = 0, /**< Clock to 0 when idle */
    spi_cpol_high,    /**< Clock to 1 when idle */
    _spi_cpol_max
} t_spi_cpol;

/**
 * @brief Enumeration for SPI clock phase (CPHA).
 */
typedef enum
{
    spi_cpha_1st_edge = 0, /**< Data captured on first clock transition */
    spi_cpha_2nd_edge,     /**< Data captured on second clock transition */
    _spi_cpha_max
} t_spi_cpha;

/**
 * @brief Enumeration for SPI Data Frame Format (DFF).
 */
typedef enum
{
    spi_dff_8_bit = 0, /**< 8-bit data frame */
    spi_dff_16_bit,    /**< 16-bit data frame */
    _spi_dff_max
} t_spi_dff;

/**
 * @brief Enumeration for SPI bit order.
 */
typedef enum
{
    spi_bit_order_msb_first = 0, /**< Most significant bit first */
    spi_bit_order_lsb_first,     /**< Least significant bit first */
    _spi_bit_order_max
} t_spi_bit_order;

/**
 * @brief Enumeration for External Interrupt (EXTI) channel selection (line number).
 */
typedef enum
{
    ext_int_line_0 = 0,  /**< EXTI Line 0 */
    ext_int_line_1,      /**< EXTI Line 1 */
    ext_int_line_2,      /**< EXTI Line 2 */
    ext_int_line_3,      /**< EXTI Line 3 */
    ext_int_line_4,      /**< EXTI Line 4 */
    ext_int_line_5,      /**< EXTI Line 5 */
    ext_int_line_6,      /**< EXTI Line 6 */
    ext_int_line_7,      /**< EXTI Line 7 */
    ext_int_line_8,      /**< EXTI Line 8 */
    ext_int_line_9,      /**< EXTI Line 9 */
    ext_int_line_10,     /**< EXTI Line 10 */
    ext_int_line_11,     /**< EXTI Line 11 */
    ext_int_line_12,     /**< EXTI Line 12 */
    ext_int_line_13,     /**< EXTI Line 13 */
    ext_int_line_14,     /**< EXTI Line 14 */
    ext_int_line_15,     /**< EXTI Line 15 */
    _ext_int_line_max
} t_external_int_channel;

/**
 * @brief Enumeration for External Interrupt (EXTI) trigger edge.
 */
typedef enum
{
    ext_int_edge_rising = 0,       /**< Rising edge trigger */
    ext_int_edge_falling,          /**< Falling edge trigger */
    ext_int_edge_rising_falling,   /**< Both rising and falling edge trigger */
    _ext_int_edge_max
} t_external_int_edge;

/**
 * @brief Enumeration for GPIO port selection.
 */
typedef enum
{
    port_a = 0, /**< GPIO Port A */
    port_b,     /**< GPIO Port B */
    port_c,     /**< GPIO Port C */
    port_d,     /**< GPIO Port D */
    port_e,     /**< GPIO Port E */
    port_h,     /**< GPIO Port H */
    _port_max
} t_port;

/**
 * @brief Enumeration for GPIO pin selection (0 to 15).
 */
typedef enum
{
    pin_0 = 0,  /**< Pin 0 */
    pin_1,      /**< Pin 1 */
    pin_2,      /**< Pin 2 */
    pin_3,      /**< Pin 3 */
    pin_4,      /**< Pin 4 */
    pin_5,      /**< Pin 5 */
    pin_6,      /**< Pin 6 */
    pin_7,      /**< Pin 7 */
    pin_8,      /**< Pin 8 */
    pin_9,      /**< Pin 9 */
    pin_10,     /**< Pin 10 */
    pin_11,     /**< Pin 11 */
    pin_12,     /**< Pin 12 */
    pin_13,     /**< Pin 13 */
    pin_14,     /**< Pin 14 */
    pin_15,     /**< Pin 15 */
    _pin_max
} t_pin;

/**
 * @brief Enumeration for GPIO direction.
 */
typedef enum
{
    direction_input = 0,  /**< GPIO pin configured as input */
    direction_output,     /**< GPIO pin configured as output */
    _direction_max
} t_direction;

/**
 * @brief Enumeration for PWM channel selection.
 */
typedef enum
{
    // TIM1 Channels
    pwm_channel_tim1_ch1 = 0,  /**< TIM1 Channel 1 (PA8, PE9) */
    pwm_channel_tim1_ch2,      /**< TIM1 Channel 2 (PA9, PE11) */
    pwm_channel_tim1_ch3,      /**< TIM1 Channel 3 (PA10, PE13) */
    pwm_channel_tim1_ch4,      /**< TIM1 Channel 4 (PA11, PE14) */
    // TIM2 Channels
    pwm_channel_tim2_ch1,      /**< TIM2 Channel 1 (PA0, PA5, PA15, PB3) */
    pwm_channel_tim2_ch2,      /**< TIM2 Channel 2 (PA1, PB3, PB10) */
    pwm_channel_tim2_ch3,      /**< TIM2 Channel 3 (PA2, PB10) */
    pwm_channel_tim2_ch4,      /**< TIM2 Channel 4 (PA3, PB11) */
    // TIM3 Channels
    pwm_channel_tim3_ch1,      /**< TIM3 Channel 1 (PA6, PB4, PC6) */
    pwm_channel_tim3_ch2,      /**< TIM3 Channel 2 (PA7, PB5, PC7) */
    pwm_channel_tim3_ch3,      /**< TIM3 Channel 3 (PB0, PC8) */
    pwm_channel_tim3_ch4,      /**< TIM3 Channel 4 (PB1, PC9) */
    // TIM4 Channels
    pwm_channel_tim4_ch1,      /**< TIM4 Channel 1 (PB6) */
    pwm_channel_tim4_ch2,      /**< TIM4 Channel 2 (PB7) */
    pwm_channel_tim4_ch3,      /**< TIM4 Channel 3 (PB8) */
    pwm_channel_tim4_ch4,      /**< TIM4 Channel 4 (PB9) */
    // TIM5 Channels
    pwm_channel_tim5_ch1,      /**< TIM5 Channel 1 (PA0) */
    pwm_channel_tim5_ch2,      /**< TIM5 Channel 2 (PA1) */
    pwm_channel_tim5_ch3,      /**< TIM5 Channel 3 (PA2) */
    pwm_channel_tim5_ch4,      /**< TIM5 Channel 4 (PA3) */
    // TIM9 Channels
    pwm_channel_tim9_ch1,      /**< TIM9 Channel 1 (PA2, PE5) */
    pwm_channel_tim9_ch2,      /**< TIM9 Channel 2 (PA3, PE6) */
    // TIM10 Channel
    pwm_channel_tim10_ch1,     /**< TIM10 Channel 1 (PB8, PA6) */
    // TIM11 Channel
    pwm_channel_tim11_ch1,     /**< TIM11 Channel 1 (PB9, PA7) */
    _pwm_channel_max
} t_pwm_channel;

/**
 * @brief Enumeration for ICU channel selection.
 */
typedef enum
{
    // TIM1 Channels
    icu_channel_tim1_ch1 = 0,  /**< TIM1 Channel 1 (PA8, PE9, PB13, PA7) */
    icu_channel_tim1_ch2,      /**< TIM1 Channel 2 (PA9, PE11, PB0, PB14) */
    icu_channel_tim1_ch3,      /**< TIM1 Channel 3 (PA10, PE13, PB1, PB15) */
    icu_channel_tim1_ch4,      /**< TIM1 Channel 4 (PA11, PE14) */
    // TIM2 Channels
    icu_channel_tim2_ch1,      /**< TIM2 Channel 1 (PA0, PA5, PA15, PB3) */
    icu_channel_tim2_ch2,      /**< TIM2 Channel 2 (PA1, PB3, PB10) */
    icu_channel_tim2_ch3,      /**< TIM2 Channel 3 (PA2, PB10) */
    icu_channel_tim2_ch4,      /**< TIM2 Channel 4 (PA3, PB11) */
    // TIM3 Channels
    icu_channel_tim3_ch1,      /**< TIM3 Channel 1 (PA6, PB4, PC6) */
    icu_channel_tim3_ch2,      /**< TIM3 Channel 2 (PA7, PB5, PC7) */
    icu_channel_tim3_ch3,      /**< TIM3 Channel 3 (PB0, PC8) */
    icu_channel_tim3_ch4,      /**< TIM3 Channel 4 (PB1, PC9) */
    // TIM4 Channels
    icu_channel_tim4_ch1,      /**< TIM4 Channel 1 (PB6) */
    icu_channel_tim4_ch2,      /**< TIM4 Channel 2 (PB7) */
    icu_channel_tim4_ch3,      /**< TIM4 Channel 3 (PB8) */
    icu_channel_tim4_ch4,      /**< TIM4 Channel 4 (PB9) */
    // TIM5 Channels
    icu_channel_tim5_ch1,      /**< TIM5 Channel 1 (PA0) */
    icu_channel_tim5_ch2,      /**< TIM5 Channel 2 (PA1) */
    icu_channel_tim5_ch3,      /**< TIM5 Channel 3 (PA2) */
    icu_channel_tim5_ch4,      /**< TIM5 Channel 4 (PA3) */
    // TIM9 Channels
    icu_channel_tim9_ch1,      /**< TIM9 Channel 1 (PA2, PE5) */
    icu_channel_tim9_ch2,      /**< TIM9 Channel 2 (PA3, PE6) */
    // TIM10 Channel
    icu_channel_tim10_ch1,     /**< TIM10 Channel 1 (PB8, PA6) */
    // TIM11 Channel
    icu_channel_tim11_ch1,     /**< TIM11 Channel 1 (PB9, PA7) */
    _icu_channel_max
} t_icu_channel;

/**
 * @brief Enumeration for ICU prescaler values.
 */
typedef enum
{
    icu_prescaler_div1 = 0, /**< No division, capture is done each event */
    icu_prescaler_div2,     /**< Capture is done once every 2 events */
    icu_prescaler_div4,     /**< Capture is done once every 4 events */
    icu_prescaler_div8,     /**< Capture is done once every 8 events */
    _icu_prescaler_max
} t_icu_prescaller;

/**
 * @brief Enumeration for ICU capture edge.
 */
typedef enum
{
    icu_edge_rising = 0,       /**< Capture on rising edge */
    icu_edge_falling,          /**< Capture on falling edge */
    icu_edge_both,             /**< Capture on both rising and falling edges */
    _icu_edge_max
} t_icu_edge;

/**
 * @brief Enumeration for Timer channel selection.
 */
typedef enum
{
    timer_channel_1 = 0, /**< Timer 1 */
    timer_channel_2,     /**< Timer 2 */
    timer_channel_3,     /**< Timer 3 */
    timer_channel_4,     /**< Timer 4 */
    timer_channel_5,     /**< Timer 5 */
    timer_channel_9,     /**< Timer 9 */
    timer_channel_10,    /**< Timer 10 */
    timer_channel_11,    /**< Timer 11 */
    _timer_channel_max
} t_timer_channel;

/**
 * @brief Enumeration for ADC channel selection.
 */
typedef enum
{
    adc_channel_0 = 0,  /**< ADC Channel 0 (PA0) */
    adc_channel_1,      /**< ADC Channel 1 (PA1) */
    adc_channel_2,      /**< ADC Channel 2 (PA2) */
    adc_channel_3,      /**< ADC Channel 3 (PA3) */
    adc_channel_4,      /**< ADC Channel 4 (PA4) */
    adc_channel_5,      /**< ADC Channel 5 (PA5) */
    adc_channel_6,      /**< ADC Channel 6 (PA6) */
    adc_channel_7,      /**< ADC Channel 7 (PA7) */
    adc_channel_8,      /**< ADC Channel 8 (PB0) */
    adc_channel_9,      /**< ADC Channel 9 (PB1) */
    adc_channel_10,     /**< ADC Channel 10 (PC0) */
    adc_channel_11,     /**< ADC Channel 11 (PC1) */
    adc_channel_12,     /**< ADC Channel 12 (PC2) */
    adc_channel_13,     /**< ADC Channel 13 (PC3) */
    adc_channel_14,     /**< ADC Channel 14 (PC4) */
    adc_channel_15,     /**< ADC Channel 15 (PC5) */
    _adc_channel_max
} t_adc_channel;

/**
 * @brief Enumeration for ADC conversion mode.
 */
typedef enum
{
    adc_mode_polling = 0,    /**< ADC conversion via polling */
    adc_mode_interrupt,      /**< ADC conversion via interrupt */
    _adc_mode_max
} t_adc_mode_t;

/**
 * @brief Enumeration for Time Triggered (TT) OS tick time.
 */
typedef enum
{
    tick_time_1ms = 0,   /**< 1 millisecond tick time */
    tick_time_10ms,      /**< 10 milliseconds tick time */
    tick_time_100ms,     /**< 100 milliseconds tick time */
    tick_time_1s,        /**< 1 second tick time */
    _tick_time_max
} t_tick_time;

/**
 * @brief Enumeration for DAC channel selection.
 */
typedef enum
{
    dac_channel_1 = 0, /**< DAC Channel 1 */
    dac_channel_2,     /**< DAC Channel 2 */
    _dac_channel_max
} dac_channel_t;

/**
 * @brief Enumeration for I2S channel selection.
 */
typedef enum
{
    i2s_channel_1 = 0, /**< I2S Channel 1 (via SPI1) */
    i2s_channel_2,     /**< I2S Channel 2 (via SPI2) */
    i2s_channel_3,     /**< I2S Channel 3 (via SPI3) */
    _i2s_channel_max
} t_i2s_channel;

/**
 * @brief Enumeration for I2S operation modes.
 */
typedef enum
{
    i2s_mode_master_transmit = 0, /**< Master Transmit Mode */
    i2s_mode_master_receive,      /**< Master Receive Mode */
    i2s_mode_slave_transmit,      /**< Slave Transmit Mode */
    i2s_mode_slave_receive,       /**< Slave Receive Mode */
    _i2s_mode_max
} I2S_Mode_t;

/**
 * @brief Enumeration for I2S audio standard.
 */
typedef enum
{
    i2s_standard_philips = 0, /**< I2S Philips standard */
    i2s_standard_msb,         /**< MSB justified standard */
    i2s_standard_lsb,         /**< LSB justified standard */
    i2s_standard_pcm_short,   /**< PCM short frame standard */
    i2s_standard_pcm_long,    /**< PCM long frame standard */
    _i2s_standard_max
} I2S_Standard_t;

/**
 * @brief Enumeration for I2S data format (audio frame length).
 */
typedef enum
{
    i2s_data_format_16b = 0, /**< 16-bit data length, 16-bit frame length */
    i2s_data_format_24b,     /**< 24-bit data length, 32-bit frame length */
    i2s_data_format_32b,     /**< 32-bit data length, 32-bit frame length */
    _i2s_data_format_max
} I2S_DataFormat_t;

/**
 * @brief Enumeration for I2S channel mode (stereo/mono).
 */
typedef enum
{
    i2s_channel_mode_stereo = 0, /**< Stereo communication */
    i2s_channel_mode_mono,       /**< Mono communication */
    _i2s_channel_mode_max
} I2S_ChannelMode_t;

/**
 * @brief Enumeration for WiFi transmit mode.
 */
typedef enum
{
    tx_mode_high_power = 0, /**< High power transmit mode */
    tx_mode_low_power,      /**< Low power transmit mode */
    _tx_mode_max
} t_tx_mode;


// ==============================================================================
// ---------------------------- Public API Prototypes ---------------------------
// ==============================================================================

// --- MCU CONFIG ---
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // This API is also part of WDT module, but listed here for core config as per API.json
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// --- LVD ---
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
void LVD_Enable(void);
void LVD_Disable(void);

// --- UART ---
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

// --- I2C ---
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

// --- SPI (CSI) ---
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

// --- External Interrupt ---
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);

// --- GPIO ---
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);

// --- PWM ---
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);

// --- ICU ---
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
// ICU_GetFrequency function declaration expects a return type, but API.json doesn't specify it.
// Assuming it returns a frequency value, e.g., tlong.
tlong ICU_GetFrequency(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));

// --- Timer ---
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);

// --- ADC ---
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(t_adc_channel adc_channel);
void ADC_Disable(t_adc_channel adc_channel);
tword ADC_Get_POLLING(t_adc_channel adc_channel);
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel);

// --- Internal_EEPROM ---
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// --- TT ---
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// --- MCAL_OUTPUT_BUZZER ---
void BUZZER_OUTPUT_Init(tbyte buzzer_number);
void BUZZER_OUTPUT_Start(tbyte NUMBER_BUZZER);
void BUZZER_OUTPUT_Stop(tbyte NUMBER_BUZZER);

// --- WDT ---
// WDT_Reset is already declared in MCU CONFIG section.
void WDT_Init(void);

// --- DAC ---
// DAC module not supported on STM32F401RC based on provided register_json.

// --- I2S ---
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// --- MQTT Protocol ---
// MQTT Protocol not supported on STM32F401RC based on provided register_json (typically requires network interface/OS support).

// --- HTTP Protocol ---
// HTTP Protocol not supported on STM32F401RC based on provided register_json (typically requires network interface/OS support).

// --- WiFi Driver ---
// WiFi Driver not supported on STM32F401RC based on provided register_json (typically requires dedicated WiFi hardware).

// --- DTC_driver ---
// DTC_driver not supported on STM32F401RC based on provided register_json.

#endif /* MCAL_H */