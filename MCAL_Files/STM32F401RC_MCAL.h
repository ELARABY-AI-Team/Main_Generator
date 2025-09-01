/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the API prototypes and type definitions for the MCAL
 * layer, providing an abstraction over the STM32F401RC microcontroller's
 * peripherals.
 *
 * It adheres to the coding standards and architectural rules defined for the project.
 */

#ifndef MCAL_H
#define MCAL_H

// Core MCU header file (STM32F401RC specific)
#include "stm32f401xc.h" 

// Standard C library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Data Type Definitions (from Rules.json)
#define tbyte uint8_t
#define tword uint16_t
#define tlong uint32_t
#define tsbyte int8_t
#define tsword int16_t
#define tslong int32_t

/**
 * @brief Enumeration for system voltage levels.
 * (Used in MCU_Config_Init)
 */
typedef enum {
    Vsource_3V,
    Vsource_5V
} t_sys_volt;

/**
 * @brief Enumeration for Low Voltage Detection (LVD) threshold levels.
 * (From Rules.json: LVD_requirements)
 */
typedef enum {
    LVD_THRESHOLD_0_5V,  /**< LVD threshold at 0.5V */
    LVD_THRESHOLD_1_0V,  /**< LVD threshold at 1.0V */
    LVD_THRESHOLD_1_5V,  /**< LVD threshold at 1.5V */
    LVD_THRESHOLD_2_0V,  /**< LVD threshold at 2.0V */
    LVD_THRESHOLD_2_5V,  /**< LVD threshold at 2.5V */
    LVD_THRESHOLD_3_0V,  /**< LVD threshold at 3.0V */
    LVD_THRESHOLD_3_5V,  /**< LVD threshold at 3.5V */
    LVD_THRESHOLD_4_0V,  /**< LVD threshold at 4.0V */
    LVD_THRESHOLD_4_5V,  /**< LVD threshold at 4.5V */
    LVD_THRESHOLD_5_0V   /**< LVD threshold at 5.0V */
} t_lvd_thrthresholdLevel;


/**
 * @brief Enumeration for GPIO port selection.
 */
typedef enum {
    GPIO_PORT_A,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_H
} t_port;

/**
 * @brief Enumeration for GPIO pin selection (0-15).
 */
typedef enum {
    GPIO_PIN_0,
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
 * @brief Enumeration for GPIO pin direction.
 */
typedef enum {
    GPIO_DIRECTION_INPUT,
    GPIO_DIRECTION_OUTPUT,
    GPIO_DIRECTION_AF,      // Alternate Function
    GPIO_DIRECTION_ANALOG   // Analog Mode
} t_direction;

/**
 * @brief Enumeration for UART channel selection.
 */
typedef enum {
    UART_CHANNEL_1,
    UART_CHANNEL_2,
    UART_CHANNEL_6
} t_uart_channel;

/**
 * @brief Enumeration for UART baud rates.
 */
typedef enum {
    UART_BAUD_RATE_9600,
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_38400,
    UART_BAUD_RATE_57600,
    UART_BAUD_RATE_115200
} t_uart_baud_rate;

/**
 * @brief Enumeration for UART data length.
 */
typedef enum {
    UART_DATA_LENGTH_8BIT,
    UART_DATA_LENGTH_9BIT
} t_uart_data_length;

/**
 * @brief Enumeration for UART stop bits.
 */
typedef enum {
    UART_STOP_BIT_1,
    UART_STOP_BIT_0_5,
    UART_STOP_BIT_2,
    UART_STOP_BIT_1_5
} t_uart_stop_bit;

/**
 * @brief Enumeration for UART parity.
 */
typedef enum {
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

/**
 * @brief Enumeration for I2C channel selection.
 */
typedef enum {
    I2C_CHANNEL_1,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

/**
 * @brief Enumeration for I2C clock speed.
 * (From Rules.json: I2C_rules - always use fast mode, so these are nominal)
 */
typedef enum {
    I2C_CLK_SPEED_STANDARD = 100000, /**< 100 kHz */
    I2C_CLK_SPEED_FAST     = 400000   /**< 400 kHz (default for fast mode) */
} t_i2c_clk_speed;

/**
 * @brief Enumeration for I2C device address.
 */
typedef enum {
    I2C_ADDRESS_7BIT,
    I2C_ADDRESS_10BIT
} t_i2c_device_address; // Placeholder, as only 7-bit addressing used in most MCALs

/**
 * @brief Enumeration for I2C Acknowledge control.
 */
typedef enum {
    I2C_ACK_DISABLE,
    I2C_ACK_ENABLE
} t_i2c_ack;

/**
 * @brief Enumeration for I2C data length.
 */
typedef enum {
    I2C_DATALENGTH_8BIT, // Used for internal buffer if needed
    I2C_DATALENGTH_16BIT
} t_i2c_datalength; // Placeholder, not typically directly configured in I2C peripheral

/**
 * @brief Enumeration for SPI channel selection.
 */
typedef enum {
    SPI_CHANNEL_1,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

/**
 * @brief Enumeration for SPI mode (Master/Slave).
 */
typedef enum {
    SPI_MODE_MASTER,
    SPI_MODE_SLAVE
} t_spi_mode;

/**
 * @brief Enumeration for SPI Clock Polarity (CPOL).
 */
typedef enum {
    SPI_CPOL_LOW,   // Clock is low when idle
    SPI_CPOL_HIGH   // Clock is high when idle
} t_spi_cpol;

/**
 * @brief Enumeration for SPI Clock Phase (CPHA).
 */
typedef enum {
    SPI_CPHA_1EDGE, // Data sampled on first clock edge
    SPI_CPHA_2EDGE  // Data sampled on second clock edge
} t_spi_cpha;

/**
 * @brief Enumeration for SPI Data Frame Format (DFF).
 */
typedef enum {
    SPI_DFF_8BIT,   // 8-bit data frame format
    SPI_DFF_16BIT   // 16-bit data frame format
} t_spi_dff;

/**
 * @brief Enumeration for SPI Bit Order (MSB First/LSB First).
 */
typedef enum {
    SPI_BIT_ORDER_MSBFIRST, // MSB transmitted first
    SPI_BIT_ORDER_LSBFIRST  // LSB transmitted first
} t_spi_bit_order;

/**
 * @brief Enumeration for External Interrupt channel selection (EXTI line).
 * (Corresponds to pin number 0-15 across all ports)
 */
typedef enum {
    EXT_INT_CHANNEL_0,
    EXT_INT_CHANNEL_1,
    EXT_INT_CHANNEL_2,
    EXT_INT_CHANNEL_3,
    EXT_INT_CHANNEL_4,
    EXT_INT_CHANNEL_5,
    EXT_INT_CHANNEL_6,
    EXT_INT_CHANNEL_7,
    EXT_INT_CHANNEL_8,
    EXT_INT_CHANNEL_9,
    EXT_INT_CHANNEL_10,
    EXT_INT_CHANNEL_11,
    EXT_INT_CHANNEL_12,
    EXT_INT_CHANNEL_13,
    EXT_INT_CHANNEL_14,
    EXT_INT_CHANNEL_15
} t_external_int_channel;

/**
 * @brief Enumeration for External Interrupt trigger edge.
 */
typedef enum {
    EXT_INT_EDGE_RISING,
    EXT_INT_EDGE_FALLING,
    EXT_INT_EDGE_RISING_FALLING
} t_external_int_edge;

/**
 * @brief Enumeration for PWM channel selection.
 * (Mapping to Timer Capture/Compare channels)
 */
typedef enum {
    PWM_CHANNEL_TIM1_CH1,  // PA8, PE9
    PWM_CHANNEL_TIM1_CH2,  // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,  // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,  // PA11, PE14
    PWM_CHANNEL_TIM2_CH1,  // PA0, PA5, PA15, PB3
    PWM_CHANNEL_TIM2_CH2,  // PA1, PB3, PB10
    PWM_CHANNEL_TIM2_CH3,  // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,  // PA3, PB11
    PWM_CHANNEL_TIM3_CH1,  // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,  // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,  // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,  // PB1, PC9
    PWM_CHANNEL_TIM4_CH1,  // PB6
    PWM_CHANNEL_TIM4_CH2,  // PB7
    PWM_CHANNEL_TIM4_CH3,  // PB8
    PWM_CHANNEL_TIM4_CH4,  // PB9
    PWM_CHANNEL_TIM5_CH1,  // PA0
    PWM_CHANNEL_TIM5_CH2,  // PA1
    PWM_CHANNEL_TIM5_CH3,  // PA2
    PWM_CHANNEL_TIM5_CH4,  // PA3
    PWM_CHANNEL_TIM9_CH1,  // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,  // PA3, PE6
    PWM_CHANNEL_TIM10_CH1, // PB8, PA6
    PWM_CHANNEL_TIM11_CH1  // PB9, PA7
} t_pwm_channel;

/**
 * @brief Enumeration for Input Capture Unit (ICU) channel selection.
 * (Mapping to Timer Capture/Compare channels)
 */
typedef enum {
    ICU_CHANNEL_TIM1_CH1,  // PA8, PE9
    ICU_CHANNEL_TIM1_CH2,  // PA9, PE11
    ICU_CHANNEL_TIM1_CH3,  // PA10, PE13
    ICU_CHANNEL_TIM1_CH4,  // PA11, PE14
    ICU_CHANNEL_TIM2_CH1,  // PA0, PA5, PA15, PB3
    ICU_CHANNEL_TIM2_CH2,  // PA1, PB3, PB10
    ICU_CHANNEL_TIM2_CH3,  // PA2, PB10
    ICU_CHANNEL_TIM2_CH4,  // PA3, PB11
    ICU_CHANNEL_TIM3_CH1,  // PA6, PB4, PC6
    ICU_CHANNEL_TIM3_CH2,  // PA7, PB5, PC7
    ICU_CHANNEL_TIM3_CH3,  // PB0, PC8
    ICU_CHANNEL_TIM3_CH4,  // PB1, PC9
    ICU_CHANNEL_TIM4_CH1,  // PB6
    ICU_CHANNEL_TIM4_CH2,  // PB7
    ICU_CHANNEL_TIM4_CH3,  // PB8
    ICU_CHANNEL_TIM4_CH4,  // PB9
    ICU_CHANNEL_TIM5_CH1,  // PA0
    ICU_CHANNEL_TIM5_CH2,  // PA1
    ICU_CHANNEL_TIM5_CH3,  // PA2
    ICU_CHANNEL_TIM5_CH4,  // PA3
    ICU_CHANNEL_TIM9_CH1,  // PA2, PE5
    ICU_CHANNEL_TIM9_CH2,  // PA3, PE6
    ICU_CHANNEL_TIM10_CH1, // PB8, PA6
    ICU_CHANNEL_TIM11_CH1  // PB9, PA7
} t_icu_channel;

/**
 * @brief Enumeration for ICU prescaler settings.
 * (Typical values, actual values depend on specific timer)
 */
typedef enum {
    ICU_PRESCALER_DIV1,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8
} t_icu_prescaller;

/**
 * @brief Enumeration for ICU trigger edge.
 */
typedef enum {
    ICU_EDGE_RISING,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

/**
 * @brief Enumeration for Timer channel selection.
 */
typedef enum {
    TIMER_CHANNEL_1,
    TIMER_CHANNEL_2,
    TIMER_CHANNEL_3,
    TIMER_CHANNEL_4,
    TIMER_CHANNEL_5,
    TIMER_CHANNEL_9,
    TIMER_CHANNEL_10,
    TIMER_CHANNEL_11
} t_timer_channel;

/**
 * @brief Enumeration for ADC channel selection.
 * (For STM32F401RC, ADC1 supports multiple channels)
 */
typedef enum {
    ADC_CHANNEL_0,  // PA0
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5,  // PA5
    ADC_CHANNEL_6,  // PA6
    ADC_CHANNEL_7,  // PA7
    ADC_CHANNEL_8,  // PB0
    ADC_CHANNEL_9,  // PB1
    ADC_CHANNEL_10, // PC0
    ADC_CHANNEL_11, // PC1
    ADC_CHANNEL_12, // PC2
    ADC_CHANNEL_13, // PC3
    ADC_CHANNEL_14, // PC4
    ADC_CHANNEL_15, // PC5
    // Add internal channels if desired (Vrefint, Vbat, Temp Sensor)
    ADC_CHANNEL_VREFINT,
    ADC_CHANNEL_TEMPSENSOR
} t_adc_channel;

/**
 * @brief Enumeration for ADC mode.
 */
typedef enum {
    ADC_MODE_SINGLE_CONVERSION,
    ADC_MODE_CONTINUOUS_CONVERSION,
    ADC_MODE_SCAN_CONVERSION
} t_adc_mode_t;

/**
 * @brief Enumeration for Time-Triggered OS tick time.
 */
typedef enum {
    TT_TICK_TIME_1MS,
    TT_TICK_TIME_10MS,
    TT_TICK_TIME_100MS
} t_tick_time;

/**
 * @brief Enumeration for I2S channel selection.
 * (I2S functionality is typically integrated within SPI peripherals)
 */
typedef enum {
    I2S_CHANNEL_1, // Corresponds to SPI1
    I2S_CHANNEL_2, // Corresponds to SPI2
    I2S_CHANNEL_3  // Corresponds to SPI3
} t_i2s_channel;

/**
 * @brief Enumeration for I2S mode.
 */
typedef enum {
    I2S_MODE_SLAVE_TX,
    I2S_MODE_SLAVE_RX,
    I2S_MODE_MASTER_TX,
    I2S_MODE_MASTER_RX
} I2S_Mode_t;

/**
 * @brief Enumeration for I2S standard.
 */
typedef enum {
    I2S_STANDARD_PHILIPS,
    I2S_STANDARD_MSB,
    I2S_STANDARD_LSB,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

/**
 * @brief Enumeration for I2S data format.
 */
typedef enum {
    I2S_DATAFORMAT_16B,
    I2S_DATAFORMAT_16B_EXTENDED,
    I2S_DATAFORMAT_24B,
    I2S_DATAFORMAT_32B
} I2S_DataFormat_t;

/**
 * @brief Enumeration for I2S channel mode.
 */
typedef enum {
    I2S_CHANNELMODE_STEREO,
    I2S_CHANNELMODE_MONO
} I2S_ChannelMode_t;

/**
 * @brief Enumeration for WiFi Transmit Mode.
 */
typedef enum {
    WIFI_TX_MODE_NONE,
    WIFI_TX_MODE_11B,
    WIFI_TX_MODE_11G,
    WIFI_TX_MODE_11N
} t_tx_mode;

// =============================================================================
// MCAL API Prototypes (from API.json)
// =============================================================================

// --- MCU CONFIG ---
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // This WDT_Reset is a generic API, there's another WDT_Reset under WDT module.
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// --- LVD ---
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel); // This function seems to set the threshold, not get a value. Renamed internally for clarity.
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
void SPI_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Returning tlong for frequency
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

// --- WDT ---
void WDT_Init(void);
// WDT_Reset(void) is already declared under MCU_CONFIG. No need for re-declaration.

// --- I2S ---
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, tlong sample_rate, tlong mclk_freq, tlong dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// --- Optional Modules not supported on STM32F401RC based on provided register_json ---
// MCAL_OUTPUT_BUZZER not supported on this MCU (no dedicated buzzer registers in provided JSON).
// DAC not supported on this MCU (no dedicated DAC registers in provided JSON).
// MQTT Protocol not supported on this MCU (requires higher-level network stack/hardware not in provided JSON).
// HTTP Protocol not supported on this MCU (requires higher-level network stack/hardware not in provided JSON).
// WiFi Driver not supported on this MCU (requires higher-level network stack/hardware not in provided JSON).
// DTC_driver not supported on this MCU (no dedicated DTC registers in provided JSON, standard DMA is present but not mapped to this API).

#endif // MCAL_H