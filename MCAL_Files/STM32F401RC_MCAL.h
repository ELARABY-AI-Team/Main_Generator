/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the API declarations and type definitions for the MCAL.
 * It provides an abstract interface to the microcontroller's peripherals,
 * allowing higher-level software to interact with the hardware without
 * needing to know the low-level register details.
 *
 * This header follows the rules and API definitions provided in API.json and Rules.json,
 * and uses register definitions from the provided REGISTER_JSON.
 *
 * MCU: STM32F401RC
 */

#ifndef MCAL_H
#define MCAL_H

// Core MCU header file (placeholder, specific part number for STM32F401RC)
#include "stm32f401xc.h" 

// Standard C library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> // Not directly used in typical MCAL, but included as per example in rules.json

// Data type definitions as per Rules.json
#define tbyte  uint8_t
#define tword  uint16_t
#define tlong  uint32_t

// =============================================================================
//                                TYPE DEFINITIONS
// =============================================================================

/**
 * @brief System Voltage Levels.
 * Defines the nominal system voltage for configuration.
 */
typedef enum {
    VOLT_3V = 0, /**< 3V System Voltage */
    VOLT_5V      /**< 5V System Voltage */
} t_sys_volt;

/**
 * @brief LVD (Low Voltage Detection) Threshold Levels.
 * Defines the voltage thresholds for LVD as per Rules.json.
 */
typedef enum {
    LVD_THRESHOLD_0P5V = 0, /**< LVD threshold at 0.5V */
    LVD_THRESHOLD_1V,       /**< LVD threshold at 1V */
    LVD_THRESHOLD_1P5V,     /**< LVD threshold at 1.5V */
    LVD_THRESHOLD_2V,       /**< LVD threshold at 2V */
    LVD_THRESHOLD_2P5V,     /**< LVD threshold at 2.5V */
    LVD_THRESHOLD_3V,       /**< LVD threshold at 3V */
    LVD_THRESHOLD_3P5V,     /**< LVD threshold at 3.5V */
    LVD_THRESHOLD_4V,       /**< LVD threshold at 4V */
    LVD_THRESHOLD_4P5V,     /**< LVD threshold at 4.5V */
    LVD_THRESHOLD_5V        /**< LVD threshold at 5V */
} t_lvd_thrthresholdLevel;

/**
 * @brief LVD Channel (placeholder, typically not multi-channel for basic LVD).
 */
typedef enum {
    LVD_CHANNEL_NONE = 0 /**< Placeholder for LVD channel, as not explicitly defined as multi-channel in registers */
} t_lvd_channel;

/**
 * @brief UART Channel Selection.
 */
typedef enum {
    UART_CHANNEL_1 = 0, /**< USART1 Peripheral */
    UART_CHANNEL_2,     /**< USART2 Peripheral */
    UART_CHANNEL_6      /**< USART6 Peripheral */
} t_uart_channel;

/**
 * @brief UART Baud Rate.
 */
typedef enum {
    UART_BAUD_RATE_9600 = 0,    /**< 9600 Baud */
    UART_BAUD_RATE_19200,   /**< 19200 Baud */
    UART_BAUD_RATE_38400,   /**< 38400 Baud */
    UART_BAUD_RATE_57600,   /**< 57600 Baud */
    UART_BAUD_RATE_115200   /**< 115200 Baud */
} t_uart_baud_rate;

/**
 * @brief UART Data Length.
 */
typedef enum {
    UART_DATA_LENGTH_8N1 = 0, /**< 8 data bits, no parity, 1 stop bit */
    UART_DATA_LENGTH_9N1      /**< 9 data bits, no parity, 1 stop bit */
} t_uart_data_length;

/**
 * @brief UART Stop Bit.
 */
typedef enum {
    UART_STOP_BIT_1 = 0, /**< 1 Stop Bit */
    UART_STOP_BIT_0_5,   /**< 0.5 Stop Bit */
    UART_STOP_BIT_2,     /**< 2 Stop Bit */
    UART_STOP_BIT_1_5    /**< 1.5 Stop Bit */
} t_uart_stop_bit;

/**
 * @brief UART Parity.
 */
typedef enum {
    UART_PARITY_NONE = 0, /**< No Parity */
    UART_PARITY_EVEN,     /**< Even Parity */
    UART_PARITY_ODD       /**< Odd Parity */
} t_uart_parity;

/**
 * @brief I2C Channel Selection.
 */
typedef enum {
    I2C_CHANNEL_1 = 0, /**< I2C1 Peripheral */
    I2C_CHANNEL_2,     /**< I2C2 Peripheral */
    I2C_CHANNEL_3      /**< I2C3 Peripheral */
} t_i2c_channel;

/**
 * @brief I2C Clock Speed.
 */
typedef enum {
    I2C_CLK_SPEED_STANDARD = 0, /**< Standard Mode (up to 100 kHz) */
    I2C_CLK_SPEED_FAST          /**< Fast Mode (up to 400 kHz) */
} t_i2c_clk_speed;

/**
 * @brief I2C Device Address (7-bit or 10-bit).
 * Placeholder for device address type.
 */
typedef uint16_t t_i2c_device_address;

/**
 * @brief I2C ACK Control.
 */
typedef enum {
    I2C_ACK_DISABLE = 0, /**< Disable Acknowledge */
    I2C_ACK_ENABLE       /**< Enable Acknowledge */
} t_i2c_ack;

/**
 * @brief I2C Data Length (number of bytes, typically used for transfers).
 */
typedef uint32_t t_i2c_datalength;

/**
 * @brief SPI Channel Selection.
 */
typedef enum {
    SPI_CHANNEL_1 = 0, /**< SPI1 Peripheral */
    SPI_CHANNEL_2,     /**< SPI2 Peripheral */
    SPI_CHANNEL_3      /**< SPI3 Peripheral */
} t_spi_channel;

/**
 * @brief SPI Master/Slave Mode.
 */
typedef enum {
    SPI_MODE_SLAVE = 0, /**< SPI Slave Mode */
    SPI_MODE_MASTER     /**< SPI Master Mode */
} t_spi_mode;

/**
 * @brief SPI Clock Polarity (CPOL).
 */
typedef enum {
    SPI_CPOL_LOW = 0,  /**< Clock polarity low (idle is low) */
    SPI_CPOL_HIGH      /**< Clock polarity high (idle is high) */
} t_spi_cpol;

/**
 * @brief SPI Clock Phase (CPHA).
 */
typedef enum {
    SPI_CPHA_1EDGE = 0, /**< Clock phase 1st edge */
    SPI_CPHA_2EDGE      /**< Clock phase 2nd edge */
} t_spi_cpha;

/**
 * @brief SPI Data Frame Format (DFF).
 */
typedef enum {
    SPI_DFF_8BIT = 0, /**< 8-bit data frame format */
    SPI_DFF_16BIT     /**< 16-bit data frame format */
} t_spi_dff;

/**
 * @brief SPI Bit Order (MSB/LSB first).
 */
typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0, /**< Most Significant Bit first */
    SPI_BIT_ORDER_LSB_FIRST      /**< Least Significant Bit first */
} t_spi_bit_order;

/**
 * @brief External Interrupt Channel Selection.
 * Maps to EXTI Line numbers (0-15).
 */
typedef enum {
    EXT_INT_CHANNEL_0 = 0,  /**< EXTI Line 0 (PA0, PB0, etc.) */
    EXT_INT_CHANNEL_1,      /**< EXTI Line 1 (PA1, PB1, etc.) */
    EXT_INT_CHANNEL_2,      /**< EXTI Line 2 */
    EXT_INT_CHANNEL_3,      /**< EXTI Line 3 */
    EXT_INT_CHANNEL_4,      /**< EXTI Line 4 */
    EXT_INT_CHANNEL_5,      /**< EXTI Line 5 */
    EXT_INT_CHANNEL_6,      /**< EXTI Line 6 */
    EXT_INT_CHANNEL_7,      /**< EXTI Line 7 */
    EXT_INT_CHANNEL_8,      /**< EXTI Line 8 */
    EXT_INT_CHANNEL_9,      /**< EXTI Line 9 */
    EXT_INT_CHANNEL_10,     /**< EXTI Line 10 */
    EXT_INT_CHANNEL_11,     /**< EXTI Line 11 */
    EXT_INT_CHANNEL_12,     /**< EXTI Line 12 */
    EXT_INT_CHANNEL_13,     /**< EXTI Line 13 */
    EXT_INT_CHANNEL_14,     /**< EXTI Line 14 */
    EXT_INT_CHANNEL_15      /**< EXTI Line 15 */
} t_external_int_channel;

/**
 * @brief External Interrupt Edge Trigger.
 */
typedef enum {
    EXT_INT_EDGE_RISING = 0,  /**< Rising edge trigger */
    EXT_INT_EDGE_FALLING,     /**< Falling edge trigger */
    EXT_INT_EDGE_RISING_FALLING /**< Both rising and falling edge trigger */
} t_external_int_edge;

/**
 * @brief GPIO Port Selection.
 */
typedef enum {
    GPIO_PORTA = 0, /**< GPIO Port A */
    GPIO_PORTB,     /**< GPIO Port B */
    GPIO_PORTC,     /**< GPIO Port C */
    GPIO_PORTD,     /**< GPIO Port D */
    GPIO_PORTE,     /**< GPIO Port E */
    GPIO_PORTH      /**< GPIO Port H */
} t_port;

/**
 * @brief GPIO Pin Selection (0-15).
 */
typedef enum {
    GPIO_PIN_0 = 0,  /**< Pin 0 */
    GPIO_PIN_1,      /**< Pin 1 */
    GPIO_PIN_2,      /**< Pin 2 */
    GPIO_PIN_3,      /**< Pin 3 */
    GPIO_PIN_4,      /**< Pin 4 */
    GPIO_PIN_5,      /**< Pin 5 */
    GPIO_PIN_6,      /**< Pin 6 */
    GPIO_PIN_7,      /**< Pin 7 */
    GPIO_PIN_8,      /**< Pin 8 */
    GPIO_PIN_9,      /**< Pin 9 */
    GPIO_PIN_10,     /**< Pin 10 */
    GPIO_PIN_11,     /**< Pin 11 */
    GPIO_PIN_12,     /**< Pin 12 */
    GPIO_PIN_13,     /**< Pin 13 */
    GPIO_PIN_14,     /**< Pin 14 */
    GPIO_PIN_15      /**< Pin 15 */
} t_pin;

/**
 * @brief GPIO Direction.
 */
typedef enum {
    GPIO_DIRECTION_INPUT = 0,  /**< GPIO Pin is configured as input */
    GPIO_DIRECTION_OUTPUT,     /**< GPIO Pin is configured as output */
    GPIO_DIRECTION_ALTERNATE_FUNCTION, /**< GPIO Pin is configured for alternate function */
    GPIO_DIRECTION_ANALOG      /**< GPIO Pin is configured for analog mode */
} t_direction;

/**
 * @brief PWM Channel Selection.
 * Channels are tied to specific timers and pins.
 */
typedef enum {
    PWM_CHANNEL_TIM1_CH1 = 0, /**< TIM1 Channel 1 (PA8, PE9) */
    PWM_CHANNEL_TIM1_CH2,     /**< TIM1 Channel 2 (PA9, PE11) */
    PWM_CHANNEL_TIM1_CH3,     /**< TIM1 Channel 3 (PA10, PE13) */
    PWM_CHANNEL_TIM1_CH4,     /**< TIM1 Channel 4 (PA11, PE14) */
    PWM_CHANNEL_TIM2_CH1,     /**< TIM2 Channel 1 (PA0, PA5, PA15) */
    PWM_CHANNEL_TIM2_CH2,     /**< TIM2 Channel 2 (PA1, PB3) */
    PWM_CHANNEL_TIM2_CH3,     /**< TIM2 Channel 3 (PA2, PB10) */
    PWM_CHANNEL_TIM2_CH4,     /**< TIM2 Channel 4 (PA3, PB11) */
    PWM_CHANNEL_TIM3_CH1,     /**< TIM3 Channel 1 (PA6, PB4, PC6) */
    PWM_CHANNEL_TIM3_CH2,     /**< TIM3 Channel 2 (PA7, PB5, PC7) */
    PWM_CHANNEL_TIM3_CH3,     /**< TIM3 Channel 3 (PB0, PC8) */
    PWM_CHANNEL_TIM3_CH4,     /**< TIM3 Channel 4 (PB1, PC9) */
    PWM_CHANNEL_TIM4_CH1,     /**< TIM4 Channel 1 (PB6, PD12) */
    PWM_CHANNEL_TIM4_CH2,     /**< TIM4 Channel 2 (PB7, PD13) */
    PWM_CHANNEL_TIM4_CH3,     /**< TIM4 Channel 3 (PB8, PD14) */
    PWM_CHANNEL_TIM4_CH4,     /**< TIM4 Channel 4 (PB9, PD15) */
    PWM_CHANNEL_TIM5_CH1,     /**< TIM5 Channel 1 (PA0) */
    PWM_CHANNEL_TIM5_CH2,     /**< TIM5 Channel 2 (PA1) */
    PWM_CHANNEL_TIM5_CH3,     /**< TIM5 Channel 3 (PA2) */
    PWM_CHANNEL_TIM5_CH4,     /**< TIM5 Channel 4 (PA3) */
    PWM_CHANNEL_TIM9_CH1,     /**< TIM9 Channel 1 (PA2, PE5) */
    PWM_CHANNEL_TIM9_CH2,     /**< TIM9 Channel 2 (PA3, PE6) */
    PWM_CHANNEL_TIM10_CH1,    /**< TIM10 Channel 1 (PA6, PB8) */
    PWM_CHANNEL_TIM11_CH1     /**< TIM11 Channel 1 (PA7, PB9) */
} t_pwm_channel;

/**
 * @brief ICU Channel Selection.
 * Channels are tied to specific timers and pins for input capture.
 */
typedef enum {
    ICU_CHANNEL_TIM1_CH1 = 0, /**< TIM1 Channel 1 (PA8, PE9) */
    ICU_CHANNEL_TIM1_CH2,     /**< TIM1 Channel 2 (PA9, PE11) */
    ICU_CHANNEL_TIM1_CH3,     /**< TIM1 Channel 3 (PA10, PE13) */
    ICU_CHANNEL_TIM1_CH4,     /**< TIM1 Channel 4 (PA11, PE14) */
    ICU_CHANNEL_TIM2_CH1,     /**< TIM2 Channel 1 (PA0, PA5, PA15) */
    ICU_CHANNEL_TIM2_CH2,     /**< TIM2 Channel 2 (PA1, PB3) */
    ICU_CHANNEL_TIM2_CH3,     /**< TIM2 Channel 3 (PA2, PB10) */
    ICU_CHANNEL_TIM2_CH4,     /**< TIM2 Channel 4 (PA3, PB11) */
    ICU_CHANNEL_TIM3_CH1,     /**< TIM3 Channel 1 (PA6, PB4, PC6) */
    ICU_CHANNEL_TIM3_CH2,     /**< TIM3 Channel 2 (PA7, PB5, PC7) */
    ICU_CHANNEL_TIM3_CH3,     /**< TIM3 Channel 3 (PB0, PC8) */
    ICU_CHANNEL_TIM3_CH4,     /**< TIM3 Channel 4 (PB1, PC9) */
    ICU_CHANNEL_TIM4_CH1,     /**< TIM4 Channel 1 (PB6, PD12) */
    ICU_CHANNEL_TIM4_CH2,     /**< TIM4 Channel 2 (PB7, PD13) */
    ICU_CHANNEL_TIM4_CH3,     /**< TIM4 Channel 3 (PB8, PD14) */
    ICU_CHANNEL_TIM4_CH4,     /**< TIM4 Channel 4 (PB9, PD15) */
    ICU_CHANNEL_TIM5_CH1,     /**< TIM5 Channel 1 (PA0) */
    ICU_CHANNEL_TIM5_CH2,     /**< TIM5 Channel 2 (PA1) */
    ICU_CHANNEL_TIM5_CH3,     /**< TIM5 Channel 3 (PA2) */
    ICU_CHANNEL_TIM5_CH4,     /**< TIM5 Channel 4 (PA3) */
    ICU_CHANNEL_TIM9_CH1,     /**< TIM9 Channel 1 (PA2, PE5) */
    ICU_CHANNEL_TIM9_CH2,     /**< TIM9 Channel 2 (PA3, PE6) */
    ICU_CHANNEL_TIM10_CH1,    /**< TIM10 Channel 1 (PA6, PB8) */
    ICU_CHANNEL_TIM11_CH1     /**< TIM11 Channel 1 (PA7, PB9) */
} t_icu_channel;

/**
 * @brief ICU Prescaler.
 */
typedef enum {
    ICU_PRESCALER_DIV1 = 0, /**< No prescaling */
    ICU_PRESCALER_DIV2,     /**< Divide by 2 */
    ICU_PRESCALER_DIV4,     /**< Divide by 4 */
    ICU_PRESCALER_DIV8      /**< Divide by 8 */
} t_icu_prescaller;

/**
 * @brief ICU Edge Detection.
 */
typedef enum {
    ICU_EDGE_RISING = 0,  /**< Detect rising edge */
    ICU_EDGE_FALLING,     /**< Detect falling edge */
    ICU_EDGE_BOTH         /**< Detect both edges */
} t_icu_edge;

/**
 * @brief Timer Channel Selection.
 */
typedef enum {
    TIMER_CHANNEL_1 = 0, /**< TIM1 Peripheral */
    TIMER_CHANNEL_2,     /**< TIM2 Peripheral */
    TIMER_CHANNEL_3,     /**< TIM3 Peripheral */
    TIMER_CHANNEL_4,     /**< TIM4 Peripheral */
    TIMER_CHANNEL_5,     /**< TIM5 Peripheral */
    TIMER_CHANNEL_9,     /**< TIM9 Peripheral */
    TIMER_CHANNEL_10,    /**< TIM10 Peripheral */
    TIMER_CHANNEL_11     /**< TIM11 Peripheral */
} t_timer_channel;

/**
 * @brief ADC Channel Selection.
 * Channels are tied to specific pins.
 */
typedef enum {
    ADC_CHANNEL_0 = 0,  /**< ADC Channel 0 (PA0) */
    ADC_CHANNEL_1,      /**< ADC Channel 1 (PA1) */
    ADC_CHANNEL_2,      /**< ADC Channel 2 (PA2) */
    ADC_CHANNEL_3,      /**< ADC Channel 3 (PA3) */
    ADC_CHANNEL_4,      /**< ADC Channel 4 (PA4) */
    ADC_CHANNEL_5,      /**< ADC Channel 5 (PA5) */
    ADC_CHANNEL_6,      /**< ADC Channel 6 (PA6) */
    ADC_CHANNEL_7,      /**< ADC Channel 7 (PA7) */
    ADC_CHANNEL_8,      /**< ADC Channel 8 (PB0) */
    ADC_CHANNEL_9,      /**< ADC Channel 9 (PB1) */
    ADC_CHANNEL_10,     /**< ADC Channel 10 (PC0) */
    ADC_CHANNEL_11,     /**< ADC Channel 11 (PC1) */
    ADC_CHANNEL_12,     /**< ADC Channel 12 (PC2) */
    ADC_CHANNEL_13,     /**< ADC Channel 13 (PC3) */
    ADC_CHANNEL_14,     /**< ADC Channel 14 (PC4) */
    ADC_CHANNEL_15      /**< ADC Channel 15 (PC5) */
} t_adc_channel;

/**
 * @brief ADC Conversion Mode.
 */
typedef enum {
    ADC_MODE_SINGLE = 0, /**< Single conversion mode */
    ADC_MODE_CONTINUOUS  /**< Continuous conversion mode */
} t_adc_mode_t;

/**
 * @brief TT (Time-Triggered) Tick Time.
 * Defines the period of the time-triggered scheduler's tick.
 */
typedef tword t_tick_time;

// =============================================================================
//                                 API DECLARATIONS
// =============================================================================

// MCU CONFIG APIs
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD APIs
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
void LVD_Enable(void);
void LVD_Disable(void);
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Return type fixed to void as per API style

// UART APIs
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_Update(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);
void UART_ClearFlag(t_uart_channel uart_channel); // Return type fixed to void as per API style

// I2C APIs
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_Update(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Return type fixed to void as per API style

// SPI APIs
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Update(void);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
void SPI_send_string(t_spi_channel spi_channel, const char *str);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);
void SPI_ClearFlag(t_spi_channel spi_channel); // Return type fixed to void as per API style

// External Interrupt APIs
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Return type fixed to void as per API style

// GPIO APIs
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);

// PWM APIs
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);

// ICU APIs
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
void ICU_Updatefrequency(t_icu_channel icu_channel);
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Return type fixed (was missing)
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Return type fixed to void as per API style

// Timer APIs
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Return type fixed to void as per API style

// ADC APIs
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void); // Return type fixed to void as per API style

// Internal_EEPROM APIs
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time-Triggered) APIs
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif /* MCAL_H */