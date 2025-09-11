#ifndef MCAL_H
#define MCAL_H

/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) header file for RENESAS_R5F11BBC.
 *
 * This file contains the API prototypes and type definitions for the MCAL layer,
 * abstracting the underlying microcontroller hardware.
 */

//==================================================================================================
// Includes
//==================================================================================================

// Core MCU header file (placeholder as specific header not provided)
#include "renesas_r5f11bbc.h" // Placeholder for device-specific header
#include <stdint.h>           // Standard integer types (uint8_t, uint16_t, etc.)
#include <stdbool.h>          // Standard boolean type
#include <stddef.h>           // Standard definitions (size_t, NULL)
// <string.h>, <stdio.h>, <stdlib.h>, <math.h> are typically for .c implementation if used.

//==================================================================================================
// Data Type Definitions (from Rules.json)
//==================================================================================================
#define tbyte  uint8_t  // 8-bit unsigned integer
#define tword  uint16_t // 16-bit unsigned integer
#define tlong  uint32_t // 32-bit unsigned integer
#define tsbyte int8_t   // 8-bit signed integer (from MQTT API)
#define tslong int32_t  // 32-bit signed integer (from MQTT API)

// Generic error type
typedef enum
{
    MCAL_OK    = 0,
    MCAL_NOK   = 1,
    MCAL_BUSY  = 2,
    MCAL_ERROR = 3
} mcal_error_t;

//==================================================================================================
// Enumerations and Type Definitions (derived from API.json and Register JSON)
//==================================================================================================

// MCU CONFIG
/**
 * @brief System voltage levels for MCU configuration and Low Voltage Detection.
 */
typedef enum
{
    sys_volt_3v = 0, ///< System voltage 3V
    sys_volt_5v      ///< System voltage 5V
} t_sys_volt;

// LVD
/**
 * @brief Low Voltage Detection (LVD) threshold levels.
 */
typedef enum
{
    lvd_threshold_volt_0_5v = 0, ///< LVD threshold 0.5V
    lvd_threshold_volt_1v,       ///< LVD threshold 1.0V
    lvd_threshold_volt_1_5v,     ///< LVD threshold 1.5V
    lvd_threshold_volt_2v,       ///< LVD threshold 2.0V
    lvd_threshold_volt_2_5v,     ///< LVD threshold 2.5V
    lvd_threshold_volt_3v,       ///< LVD threshold 3.0V
    lvd_threshold_volt_3_5v,     ///< LVD threshold 3.5V
    lvd_threshold_volt_4v,       ///< LVD threshold 4.0V
    lvd_threshold_volt_4_5v,     ///< LVD threshold 4.5V
    lvd_threshold_volt_5v        ///< LVD threshold 5.0V
} t_lvd_thrthresholdLevel;

// UART
/**
 * @brief UART channel identifiers.
 */
typedef enum
{
    uart_channel_0 = 0, ///< Serial Array Unit 0, Channel 1 (SAU0_SDR01)
    uart_channel_1 = 1, ///< Serial Array Unit 1, Channel 0 (SAU1_SDR10)
    uart_channel_2 = 2  ///< Serial Array Unit 2, Channel 0 (SAU2_SDR20)
} t_uart_channel;

/**
 * @brief UART baud rate settings.
 */
typedef enum
{
    uart_baud_rate_9600 = 0,
    uart_baud_rate_19200,
    uart_baud_rate_57600,
    uart_baud_rate_115200
    // Add other common baud rates as needed
} t_uart_baud_rate;

/**
 * @brief UART data length settings.
 */
typedef enum
{
    uart_data_length_7bit = 0,
    uart_data_length_8bit,
    uart_data_length_9bit
} t_uart_data_length;

/**
 * @brief UART stop bit settings.
 */
typedef enum
{
    uart_stop_bit_1 = 0,
    uart_stop_bit_2
} t_uart_stop_bit;

/**
 * @brief UART parity settings.
 */
typedef enum
{
    uart_parity_none = 0,
    uart_parity_even,
    uart_parity_odd
} t_uart_parity;

// I2C
/**
 * @brief I2C channel identifiers.
 */
typedef enum
{
    i2c_channel_0 = 0 ///< I2C Bus Channel 0 (I2C0)
} t_i2c_channel;

/**
 * @brief I2C clock speed settings.
 */
typedef enum
{
    i2c_clk_speed_standard = 0, ///< Standard mode (100 kHz)
    i2c_clk_speed_fast          ///< Fast mode (400 kHz)
    // Add other speeds if supported, e.g., i2c_clk_speed_fast_plus
} t_i2c_clk_speed;

/**
 * @brief I2C acknowledgment settings.
 */
typedef enum
{
    i2c_ack_disable = 0, ///< No Acknowledge
    i2c_ack_enable       ///< Acknowledge
} t_i2c_ack;

/**
 * @brief I2C data length settings.
 */
typedef enum
{
    i2c_datalength_7bit = 0, ///< 7-bit addressing
    i2c_datalength_8bit      ///< 8-bit addressing
} t_i2c_datalength;

// SPI (CSI)
/**
 * @brief SPI channel identifiers.
 */
typedef enum
{
    spi_channel_0 = 0 ///< Serial Array Unit 0, Channel 0 (SAU0_SDR00)
} t_spi_channel;

/**
 * @brief SPI mode settings (Master/Slave).
 */
typedef enum
{
    spi_mode_master = 0, ///< SPI Master mode
    spi_mode_slave       ///< SPI Slave mode
} t_spi_mode;

/**
 * @brief SPI Clock Polarity (CPOL) settings.
 */
typedef enum
{
    spi_cpol_low = 0, ///< Clock is low when idle
    spi_cpol_high     ///< Clock is high when idle
} t_spi_cpol;

/**
 * @brief SPI Clock Phase (CPHA) settings.
 */
typedef enum
{
    spi_cpha_1edge = 0, ///< Data captured on first clock edge
    spi_cpha_2edge      ///< Data captured on second clock edge
} t_spi_cpha;

/**
 * @brief SPI Data Frame Format (DFF) settings.
 */
typedef enum
{
    spi_dff_8bit = 0, ///< 8-bit data frame
    spi_dff_16bit     ///< 16-bit data frame
} t_spi_dff;

/**
 * @brief SPI Bit Order settings (MSB first / LSB first).
 */
typedef enum
{
    spi_bit_order_msb_first = 0, ///< Most Significant Bit first
    spi_bit_order_lsb_first      ///< Least Significant Bit first
} t_spi_bit_order;


// External Interrupt
/**
 * @brief External Interrupt channel identifiers.
 *        Mapped to P1.0-P1.7 as per TAU_ISC assigned pins.
 */
typedef enum
{
    external_int_channel_0 = 0, ///< External Interrupt Channel 0 (P1.0)
    external_int_channel_1,     ///< External Interrupt Channel 1 (P1.1)
    external_int_channel_2,     ///< External Interrupt Channel 2 (P1.2)
    external_int_channel_3,     ///< External Interrupt Channel 3 (P1.3)
    external_int_channel_4,     ///< External Interrupt Channel 4 (P1.4)
    external_int_channel_5,     ///< External Interrupt Channel 5 (P1.5)
    external_int_channel_6,     ///< External Interrupt Channel 6 (P1.6)
    external_int_channel_7      ///< External Interrupt Channel 7 (P1.7)
} t_external_int_channel;

/**
 * @brief External Interrupt edge sensitivity settings.
 */
typedef enum
{
    external_int_edge_rising = 0,  ///< Interrupt on rising edge
    external_int_edge_falling,     ///< Interrupt on falling edge
    external_int_edge_both         ///< Interrupt on both rising and falling edges
} t_external_int_edge;

// GPIO
/**
 * @brief GPIO Port identifiers.
 */
typedef enum
{
    port_0 = 0,
    port_1,
    port_2,
    port_3,
    port_4,
    port_5,
    port_6,
    port_7,
    port_12 = 12, // Non-contiguous port numbers
    port_13,
    port_14,
    port_15
} t_port;

/**
 * @brief GPIO Pin identifiers (0 to 7 for an 8-bit port).
 */
typedef enum
{
    pin_0 = 0,
    pin_1,
    pin_2,
    pin_3,
    pin_4,
    pin_5,
    pin_6,
    pin_7
} t_pin;

/**
 * @brief GPIO Pin direction settings.
 */
typedef enum
{
    gpio_direction_input = 0, ///< Pin configured as input
    gpio_direction_output     ///< Pin configured as output
} t_direction;

// PWM
/**
 * @brief PWM channel identifiers, mapped to Timer Array Unit 0 channels.
 */
typedef enum
{
    pwm_channel_0 = 0, ///< PWM Channel 0 (TAU0 Channel 0, P1.0)
    pwm_channel_1,     ///< PWM Channel 1 (TAU0 Channel 1, P1.1)
    pwm_channel_2,     ///< PWM Channel 2 (TAU0 Channel 2, P1.2)
    pwm_channel_3,     ///< PWM Channel 3 (TAU0 Channel 3, P1.3)
    pwm_channel_4,     ///< PWM Channel 4 (TAU0 Channel 4, P1.4)
    pwm_channel_5,     ///< PWM Channel 5 (TAU0 Channel 5, P1.5)
    pwm_channel_6,     ///< PWM Channel 6 (TAU0 Channel 6, P1.6)
    pwm_channel_7      ///< PWM Channel 7 (TAU0 Channel 7, P1.7)
} t_pwm_channel;

// ICU
/**
 * @brief ICU channel identifiers, mapped to Timer Array Unit 0 channels.
 */
typedef enum
{
    icu_channel_0 = 0, ///< ICU Channel 0 (TAU0 Channel 0, P1.0)
    icu_channel_1,     ///< ICU Channel 1 (TAU0 Channel 1, P1.1)
    icu_channel_2,     ///< ICU Channel 2 (TAU0 Channel 2, P1.2)
    icu_channel_3,     ///< ICU Channel 3 (TAU0 Channel 3, P1.3)
    icu_channel_4,     ///< ICU Channel 4 (TAU0 Channel 4, P1.4)
    icu_channel_5,     ///< ICU Channel 5 (TAU0 Channel 5, P1.5)
    icu_channel_6,     ///< ICU Channel 6 (TAU0 Channel 6, P1.6)
    icu_channel_7      ///< ICU Channel 7 (TAU0 Channel 7, P1.7)
} t_icu_channel;

/**
 * @brief ICU prescaler settings.
 */
typedef enum
{
    icu_prescaler_1 = 0, ///< Prescaler 1
    icu_prescaler_2,     ///< Prescaler 2
    icu_prescaler_4,     ///< Prescaler 4
    icu_prescaler_8,     ///< Prescaler 8
    icu_prescaler_16,    ///< Prescaler 16
    icu_prescaler_32,    ///< Prescaler 32
    icu_prescaler_64,    ///< Prescaler 64
    icu_prescaler_128    ///< Prescaler 128
} t_icu_prescaller;

/**
 * @brief ICU edge detection settings.
 */
typedef enum
{
    icu_edge_rising = 0,  ///< Detect rising edge
    icu_edge_falling,     ///< Detect falling edge
    icu_edge_both         ///< Detect both rising and falling edges
} t_icu_edge;

// Timer
/**
 * @brief Timer channel identifiers, mapped to Timer Array Unit 0 channels.
 */
typedef enum
{
    timer_channel_0 = 0, ///< Timer Channel 0 (TAU0 Channel 0, P1.0)
    timer_channel_1,     ///< Timer Channel 1 (TAU0 Channel 1, P1.1)
    timer_channel_2,     ///< Timer Channel 2 (TAU0 Channel 2, P1.2)
    timer_channel_3,     ///< Timer Channel 3 (TAU0 Channel 3, P1.3)
    timer_channel_4,     ///< Timer Channel 4 (TAU0 Channel 4, P1.4)
    timer_channel_5,     ///< Timer Channel 5 (TAU0 Channel 5, P1.5)
    timer_channel_6,     ///< Timer Channel 6 (TAU0 Channel 6, P1.6)
    timer_channel_7      ///< Timer Channel 7 (TAU0 Channel 7, P1.7)
} t_timer_channel;

// ADC
/**
 * @brief ADC channel identifiers, mapped to physical pins.
 */
typedef enum
{
    adc_channel_p0_0 = 0, ///< ADC Channel on P0.0
    adc_channel_p0_1,     ///< ADC Channel on P0.1
    adc_channel_p2_0,     ///< ADC Channel on P2.0
    adc_channel_p2_1,     ///< ADC Channel on P2.1
    adc_channel_p2_2,     ///< ADC Channel on P2.2
    adc_channel_p2_3,     ///< ADC Channel on P2.3
    adc_channel_p2_4,     ///< ADC Channel on P2.4
    adc_channel_p7_0,     ///< ADC Channel on P7.0
    adc_channel_p7_1,     ///< ADC Channel on P7.1
    adc_channel_p7_2,     ///< ADC Channel on P7.2
    adc_channel_p7_3,     ///< ADC Channel on P7.3
    adc_channel_p7_4,     ///< ADC Channel on P7.4
    adc_channel_p7_5,     ///< ADC Channel on P7.5
    adc_channel_p7_6,     ///< ADC Channel on P7.6
    adc_channel_p7_7      ///< ADC Channel on P7.7
} t_adc_channel;

/**
 * @brief ADC operation modes.
 */
typedef enum
{
    adc_mode_single_conversion = 0, ///< Single conversion mode
    adc_mode_continuous_scan        ///< Continuous scan mode
} t_adc_mode_t;

// TT (Time Triggered OS)
/**
 * @brief Tick time interval for the Time Triggered OS.
 */
typedef enum
{
    tick_time_1ms = 0,  ///< 1 millisecond tick
    tick_time_10ms,     ///< 10 milliseconds tick
    tick_time_100ms,    ///< 100 milliseconds tick
    tick_time_1s        ///< 1 second tick
} t_tick_time;

// DAC
/**
 * @brief DAC channel identifiers.
 */
typedef enum
{
    dac_channel_0 = 0, ///< DAC Channel 0 (P6.0)
    dac_channel_1      ///< DAC Channel 1 (P6.1)
} dac_channel_t; // Renamed from t_dac_channel to dac_channel_t for exact match with API.json

//==================================================================================================
// API Prototypes (from API.json)
//==================================================================================================

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // Defined in API.json under MCU CONFIG and WDT
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
void LVD_Enable(void);
void LVD_Disable(void);

// UART
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

// I2C
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

// SPI (CSI)
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);

// GPIO
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);

// PWM
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);

// ICU
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
// ICU_GetFrequency is not explicitly typed in API.json, assuming returns a frequency value (e.g., tword)
tword ICU_GetFrequency(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(t_adc_channel adc_channel);
void ADC_Disable(t_adc_channel adc_channel);
tword ADC_Get_POLLING(t_adc_channel adc_channel);
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel);

// Internal_EEPROM
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// WDT (Watchdog Timer)
void WDT_Init(void);
// WDT_Reset() is already listed under MCU CONFIG and will be implemented once.

// DAC
void DAC_Init(dac_channel_t channel);
void DAC_Enable(dac_channel_t channel);
void DAC_Disable(dac_channel_t channel);
void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue);

//==================================================================================================
// Optional Modules (Skipped - No corresponding registers/drivers found in register_json)
//==================================================================================================
// MCAL_OUTPUT_BUZZER not supported on this MCU.
// I2S not supported on this MCU.
// MQTT Protocol not supported on this MCU.
// HTTP Protocol not supported on this MCU.
// WiFi Driver not supported on this MCU.
// DTC_driver not supported on this MCU.

#endif // MCAL_H