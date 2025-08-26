/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File
 *
 * This file contains the declarations for the Microcontroller Abstraction Layer (MCAL)
 * for the STM32F401RC microcontroller. It defines standard data types,
 * enums for peripheral configuration, and API function prototypes.
 *
 * Developed according to MISRA C and CERT-C coding standards.
 *
 * @date YYYY-MM-DD (placeholder)
 */

#ifndef MCAL_H_
#define MCAL_H_

// Core device header for STM32F401RC
#include "stm32f4xx.h"  // Placeholder for the specific device header, e.g., stm32f401xc.h or stm32f4xx.h
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h> // Included per core_includes rule, though not explicitly used by all APIs
#include <stdio.h>  // Included per core_includes rule, though not explicitly used by all APIs
#include <stdlib.h> // Included per core_includes rule, though not explicitly used by all APIs
#include <math.h>   // Included per core_includes rule, though not explicitly used by all APIs

/*
 * ===================================================================================================
 *                                  Data Type Definitions
 * ===================================================================================================
 */

#define tbyte uint8_t  /**< 8-bit unsigned integer */
#define tword uint16_t /**< 16-bit unsigned integer */
#define tlong uint32_t /**< 32-bit unsigned integer */

/*
 * ===================================================================================================
 *                                  Enum and Type Definitions for APIs
 * ===================================================================================================
 */

/**
 * @brief System voltage levels for MCU configuration.
 */
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

/**
 * @brief LVD threshold levels.
 */
typedef enum {
    LVD_THRESHOLD_2_0V = 0, // For 3V system, maps to PWR_CR PLS_0
    LVD_THRESHOLD_3_5V,     // For 5V system, maps to PWR_CR PLS_5
    // Add more thresholds as per datasheet, mapping to PWR_CR PLS bits
    LVD_THRESHOLD_1_9V, // PLS_0 (000)
    LVD_THRESHOLD_2_1V, // PLS_1 (001)
    LVD_THRESHOLD_2_3V, // PLS_2 (010)
    LVD_THRESHOLD_2_5V, // PLS_3 (011)
    LVD_THRESHOLD_2_7V, // PLS_4 (100)
    LVD_THRESHOLD_2_9V, // PLS_5 (101)
    LVD_THRESHOLD_3_1V, // PLS_6 (110)
    LVD_THRESHOLD_EXT   // PLS_7 (111) - External input
} t_lvd_thrthresholdLevel;

/**
 * @brief LVD channel (for clearing flags). STM32 PVD is usually single channel.
 */
typedef enum {
    LVD_CHANNEL_PVD = 0 /**< Placeholder for a single PVD channel */
} t_lvd_channel;

/**
 * @brief UART channel selection.
 */
typedef enum {
    UART_CHANNEL_1 = 0,
    UART_CHANNEL_2,
    UART_CHANNEL_6
} t_uart_channel;

/**
 * @brief UART baud rate selection.
 */
typedef enum {
    UART_BAUD_RATE_9600 = 0,
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_38400,
    UART_BAUD_RATE_57600,
    UART_BAUD_RATE_115200
} t_uart_baud_rate;

/**
 * @brief UART data length selection.
 */
typedef enum {
    UART_DATA_LENGTH_8B = 0,
    UART_DATA_LENGTH_9B
} t_uart_data_length;

/**
 * @brief UART stop bit selection.
 */
typedef enum {
    UART_STOP_BIT_1 = 0,
    UART_STOP_BIT_0_5,
    UART_STOP_BIT_2,
    UART_STOP_BIT_1_5
} t_uart_stop_bit;

/**
 * @brief UART parity selection.
*/
typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

/**
 * @brief I2C channel selection.
 */
typedef enum {
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

/**
 * @brief I2C clock speed selection.
 */
typedef enum {
    I2C_CLK_SPEED_STANDARD = 0, /**< 100 kHz */
    I2C_CLK_SPEED_FAST          /**< 400 kHz */
} t_i2c_clk_speed;

/**
 * @brief I2C device address type.
 */
typedef tbyte t_i2c_device_address;

/**
 * @brief I2C ACK control.
 */
typedef enum {
    I2C_ACK_DISABLE = 0,
    I2C_ACK_ENABLE
} t_i2c_ack;

/**
 * @brief I2C data length (for 7-bit or 10-bit addressing, not really a length but a mode).
 */
typedef enum {
    I2C_DATALENGTH_7BIT = 0,
    I2C_DATALENGTH_10BIT
} t_i2c_datalength;

/**
 * @brief SPI channel selection.
 */
typedef enum {
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

/**
 * @brief SPI mode selection (Master/Slave).
 */
typedef enum {
    SPI_MODE_SLAVE = 0,
    SPI_MODE_MASTER
} t_spi_mode;

/**
 * @brief SPI clock polarity.
 */
typedef enum {
    SPI_CPOL_LOW = 0,
    SPI_CPOL_HIGH
} t_spi_cpol;

/**
 * @brief SPI clock phase.
 */
typedef enum {
    SPI_CPHA_1EDGE = 0, /**< Data sampled on first clock edge */
    SPI_CPHA_2EDGE      /**< Data sampled on second clock edge */
} t_spi_cpha;

/**
 * @brief SPI data frame format (8-bit or 16-bit).
 */
typedef enum {
    SPI_DFF_8BIT = 0,
    SPI_DFF_16BIT
} t_spi_dff;

/**
 * @brief SPI bit order (MSB or LSB first).
 */
typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

/**
 * @brief External Interrupt channel selection.
 */
typedef enum {
    EXTERNAL_INT_LINE_0 = 0,
    EXTERNAL_INT_LINE_1,
    EXTERNAL_INT_LINE_2,
    EXTERNAL_INT_LINE_3,
    EXTERNAL_INT_LINE_4,
    EXTERNAL_INT_LINE_5,
    EXTERNAL_INT_LINE_6,
    EXTERNAL_INT_LINE_7,
    EXTERNAL_INT_LINE_8,
    EXTERNAL_INT_LINE_9,
    EXTERNAL_INT_LINE_10,
    EXTERNAL_INT_LINE_11,
    EXTERNAL_INT_LINE_12,
    EXTERNAL_INT_LINE_13,
    EXTERNAL_INT_LINE_14,
    EXTERNAL_INT_LINE_15
} t_external_int_channel;

/**
 * @brief External Interrupt edge selection.
 */
typedef enum {
    EXTERNAL_INT_EDGE_RISING = 0,
    EXTERNAL_INT_EDGE_FALLING,
    EXTERNAL_INT_EDGE_BOTH
} t_external_int_edge;

/**
 * @brief GPIO Port selection.
 */
typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_H
} t_port;

/**
 * @brief GPIO Pin selection.
 */
typedef enum {
    GPIO_PIN_0 = 0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15
} t_pin;

/**
 * @brief GPIO Direction type.
 */
typedef enum {
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;

/**
 * @brief PWM channel selection.
 */
typedef enum {
    PWM_CHANNEL_TIM1_CH1 = 0, // PA8, PE9
    PWM_CHANNEL_TIM1_CH2,     // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,     // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,     // PA11, PE14
    PWM_CHANNEL_TIM2_CH1,     // PA0, PA5, PA15
    PWM_CHANNEL_TIM2_CH2,     // PA1, PB3
    PWM_CHANNEL_TIM2_CH3,     // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,     // PA3, PB11
    PWM_CHANNEL_TIM3_CH1,     // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,     // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,     // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,     // PB1, PC9
    PWM_CHANNEL_TIM4_CH1,     // PB6, PD12
    PWM_CHANNEL_TIM4_CH2,     // PB7, PD13
    PWM_CHANNEL_TIM4_CH3,     // PB8, PD14
    PWM_CHANNEL_TIM4_CH4,     // PB9, PD15
    PWM_CHANNEL_TIM5_CH1,     // PA0
    PWM_CHANNEL_TIM5_CH2,     // PA1
    PWM_CHANNEL_TIM5_CH3,     // PA2
    PWM_CHANNEL_TIM5_CH4,     // PA3
    PWM_CHANNEL_TIM9_CH1,     // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,     // PA3, PE6
    PWM_CHANNEL_TIM10_CH1,    // PA6, PB8
    PWM_CHANNEL_TIM11_CH1     // PA7, PB9
} t_pwm_channel;

/**
 * @brief ICU channel selection. Same as PWM channels, as they typically use the same timer modules.
 */
typedef t_pwm_channel t_icu_channel;

/**
 * @brief ICU prescaler selection.
 */
typedef enum {
    ICU_PRESCALER_DIV1 = 0,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8,
    ICU_PRESCALER_DIV16,
    ICU_PRESCALER_DIV32,
    ICU_PRESCALER_DIV64,
    ICU_PRESCALER_DIV128
} t_icu_prescaller;

/**
 * @brief ICU edge selection.
 */
typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH_EDGES
} t_icu_edge;

/**
 * @brief Timer channel selection.
 */
typedef enum {
    TIMER_CHANNEL_TIM1 = 0,
    TIMER_CHANNEL_TIM2,
    TIMER_CHANNEL_TIM3,
    TIMER_CHANNEL_TIM4,
    TIMER_CHANNEL_TIM5,
    TIMER_CHANNEL_TIM9,
    TIMER_CHANNEL_TIM10,
    TIMER_CHANNEL_TIM11
} t_timer_channel;

/**
 * @brief ADC channel selection.
 */
typedef enum {
    ADC_CHANNEL_0 = 0,  // PA0
    ADC_CHANNEL_1,      // PA1
    ADC_CHANNEL_2,      // PA2
    ADC_CHANNEL_3,      // PA3
    ADC_CHANNEL_4,      // PA4
    ADC_CHANNEL_5,      // PA5
    ADC_CHANNEL_6,      // PA6
    ADC_CHANNEL_7,      // PA7
    ADC_CHANNEL_8,      // PB0
    ADC_CHANNEL_9,      // PB1
    ADC_CHANNEL_10,     // PC0
    ADC_CHANNEL_11,     // PC1
    ADC_CHANNEL_12,     // PC2
    ADC_CHANNEL_13,     // PC3
    ADC_CHANNEL_14,     // PC4
    ADC_CHANNEL_15      // PC5
} t_adc_channel;

/**
 * @brief ADC mode selection (e.g., single conversion, continuous, scan).
 */
typedef enum {
    ADC_MODE_SINGLE = 0,
    ADC_MODE_CONTINUOUS,
    ADC_MODE_SCAN
} t_adc_mode_t;

/**
 * @brief Tick time for TT (Time-Triggered) OS.
 */
typedef enum {
    TT_TICK_TIME_1MS = 0,
    TT_TICK_TIME_10MS,
    TT_TICK_TIME_100MS
} t_tick_time;

/*
 * ===================================================================================================
 *                                  API Function Declarations
 * ===================================================================================================
 */

// MCU CONFIG APIs
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD APIs
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel); // Note: Get implies reading, but this sets threshold
void LVD_Enable(void);
void LVD_Disable(void);
void LVD_ClearFlag(t_lvd_channel lvd_channel);

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
void UART_ClearFlag(t_uart_channel uart_channel);

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
void I2C_ClearFlag(t_i2c_channel i2c_channel);

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
void SPI_ClearFlag(t_spi_channel spi_channel);

// External Interrupt APIs
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel);

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
tlong ICU_GetFrequency(t_icu_channel icu_channel);
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel);

// Timer APIs
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel);

// ADC APIs
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void);

// Internal_EEPROM APIs
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time-Triggered) OS APIs
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif /* MCAL_H_ */