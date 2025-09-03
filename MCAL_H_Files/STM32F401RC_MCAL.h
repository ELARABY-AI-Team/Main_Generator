#ifndef MCAL_H_
#define MCAL_H_

// --- Core MCU Header and Standard C Libraries ---
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// The following standard C libraries are generally used in the MCAL.c file,
// but included here as per Rules.json: core_includes examples.
// #include <string.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

// --- Data Type Definitions as per Rules.json: data_type_definitions ---
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// --- Peripheral Base Addresses (for internal use within MCAL.c) ---
// These define the start addresses of peripheral blocks. Specific register access
// will be handled in MCAL.c via pointer arithmetic, adhering to
// Rules.json: pointer_variable_consistency.
#define FLASH_BASE_ADDRESS      ((tlong)0x40023C00)
#define RCC_BASE_ADDRESS        ((tlong)0x40023800)
#define SYSCFG_BASE_ADDRESS     ((tlong)0x40013800)
#define EXTI_BASE_ADDRESS       ((tlong)0x40013C00)

#define GPIOA_BASE_ADDRESS      ((tlong)0x40020000)
#define GPIOB_BASE_ADDRESS      ((tlong)0x40020400)
#define GPIOC_BASE_ADDRESS      ((tlong)0x40020800)
#define GPIOD_BASE_ADDRESS      ((tlong)0x40020C00)
#define GPIOE_BASE_ADDRESS      ((tlong)0x40021000)
#define GPIOH_BASE_ADDRESS      ((tlong)0x40021C00)

#define ADC1_BASE_ADDRESS       ((tlong)0x40012000)
#define ADC_COMMON_BASE_ADDRESS ((tlong)0x40012300) // ADC_CCR is at 0x40012304

#define TIM1_BASE_ADDRESS       ((tlong)0x40010000)
#define TIM2_BASE_ADDRESS       ((tlong)0x40000000)
#define TIM3_BASE_ADDRESS       ((tlong)0x40000400)
#define TIM4_BASE_ADDRESS       ((tlong)0x40000800)
#define TIM5_BASE_ADDRESS       ((tlong)0x40000C00)
#define TIM9_BASE_ADDRESS       ((tlong)0x40014000)
#define TIM10_BASE_ADDRESS      ((tlong)0x40014400)
#define TIM11_BASE_ADDRESS      ((tlong)0x40014800)

#define USART1_BASE_ADDRESS     ((tlong)0x40011000)
#define USART2_BASE_ADDRESS     ((tlong)0x40004400)
#define USART6_BASE_ADDRESS     ((tlong)0x40011400)

#define I2C1_BASE_ADDRESS       ((tlong)0x40005400)
#define I2C2_BASE_ADDRESS       ((tlong)0x40005800)
#define I2C3_BASE_ADDRESS       ((tlong)0x40005C00)

#define SPI1_BASE_ADDRESS       ((tlong)0x40013000)
#define SPI2_BASE_ADDRESS       ((tlong)0x40003800)
#define SPI3_BASE_ADDRESS       ((tlong)0x40003C00)


// --- MCU CONFIG Module ---
typedef enum
{
    sys_volt_3v = 0,
    sys_volt_5v
} t_sys_volt;

void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);


// --- LVD Module ---
// LVD module not supported on this MCU based on provided register_json (no dedicated LVD/PVD registers).
// The following typedef and prototypes are commented out as per Rules.json: optional_modules_rule.
/*
typedef enum
{
    lvd_threshold_level_volt_0_5v,
    lvd_threshold_level_volt_1v,
    lvd_threshold_level_volt_1_5v,
    lvd_threshold_level_volt_2v,
    lvd_threshold_level_volt_2_5v,
    lvd_threshold_level_volt_3v,
    lvd_threshold_level_volt_3_5v, // Often default for 5V systems
    lvd_threshold_level_volt_4v,
    lvd_threshold_level_volt_4_5v,
    lvd_threshold_level_volt_5v
} t_lvd_thrthreshold_level;

void LVD_Init(void);
void LVD_Get(t_lvd_thrthreshold_level lvd_threshold_level);
void LVD_Enable(void);
void LVD_Disable(void);
*/


// --- UART Module ---
typedef enum
{
    uart_channel_usart1,
    uart_channel_usart2,
    uart_channel_usart6,
    uart_channel_max
} t_uart_channel;

typedef enum
{
    uart_baud_rate_9600,
    uart_baud_rate_19200,
    uart_baud_rate_38400,
    uart_baud_rate_57600,
    uart_baud_rate_115200,
    uart_baud_rate_max
} t_uart_baud_rate;

typedef enum
{
    uart_data_length_8bit,
    uart_data_length_9bit
} t_uart_data_length;

typedef enum
{
    uart_stop_bit_1,
    uart_stop_bit_0_5, // For SmartCard mode
    uart_stop_bit_2,
    uart_stop_bit_1_5  // For SmartCard mode
} t_uart_stop_bit;

typedef enum
{
    uart_parity_none,
    uart_parity_even,
    uart_parity_odd
} t_uart_parity;

void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);


// --- I2C Module ---
typedef enum
{
    i2c_channel_i2c1,
    i2c_channel_i2c2,
    i2c_channel_i2c3,
    i2c_channel_max
} t_i2c_channel;

typedef enum
{
    i2c_clk_speed_standard_mode,    // up to 100 kHz
    i2c_clk_speed_fast_mode         // up to 400 kHz (as per Rules.json: I2C_rules, always use fast mode)
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // As per API.json and to hold a 7 or 10-bit address

typedef enum
{
    i2c_ack_disable,
    i2c_ack_enable
} t_i2c_ack;

typedef enum
{
    i2c_datalength_8bit,
    i2c_datalength_16bit // Application-level interpretation for some devices
} t_i2c_datalength;

void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);


// --- SPI (CSI) Module ---
typedef enum
{
    spi_channel_spi1,
    spi_channel_spi2,
    spi_channel_spi3,
    spi_channel_max
} t_spi_channel;

typedef enum
{
    spi_mode_master,
    spi_mode_slave
} t_spi_mode;

typedef enum
{
    spi_cpol_low,   // Clock to 0 when idle
    spi_cpol_high   // Clock to 1 when idle
} t_spi_cpol;

typedef enum
{
    spi_cpha_1edge, // First clock transition is first data capture edge
    spi_cpha_2edge  // Second clock transition is first data capture edge
} t_spi_cpha;

typedef enum
{
    spi_dff_8bit,
    spi_dff_16bit
} t_spi_dff;

typedef enum
{
    spi_bit_order_msb_first,
    spi_bit_order_lsb_first
} t_spi_bit_order;

void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);


// --- External Interrupt Module ---
typedef enum
{
    external_int_channel_line_0,
    external_int_channel_line_1,
    external_int_channel_line_2,
    external_int_channel_line_3,
    external_int_channel_line_4,
    external_int_channel_line_5,
    external_int_channel_line_6,
    external_int_channel_line_7,
    external_int_channel_line_8,
    external_int_channel_line_9,
    external_int_channel_line_10,
    external_int_channel_line_11,
    external_int_channel_line_12,
    external_int_channel_line_13,
    external_int_channel_line_14,
    external_int_channel_line_15,
    external_int_channel_max
} t_external_int_channel;

typedef enum
{
    external_int_edge_rising,
    external_int_edge_falling,
    external_int_edge_rising_falling
} t_external_int_edge;

void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);


// --- GPIO Module ---
typedef enum
{
    port_a,
    port_b,
    port_c,
    port_d,
    port_e,
    port_h,
    port_max
} t_port;

typedef enum
{
    pin_0,
    pin_1,
    pin_2,
    pin_3,
    pin_4,
    pin_5,
    pin_6,
    pin_7,
    pin_8,
    pin_9,
    pin_10,
    pin_11,
    pin_12,
    pin_13,
    pin_14,
    pin_15,
    pin_max
} t_pin;

typedef enum
{
    direction_input,
    direction_output
} t_direction;

void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);


// --- PWM Module ---
typedef enum
{
    // TIM1 Advanced-control Timer
    pwm_channel_tim1_ch1,  // AF1: PA8, PE9; AF2: PB13, PA7
    pwm_channel_tim1_ch2,  // AF1: PA9, PE11; AF2: PB0, PB14
    pwm_channel_tim1_ch3,  // AF1: PA10, PE13; AF2: PB1, PB15
    pwm_channel_tim1_ch4,  // AF1: PA11, PE14
    // TIM2 General-purpose Timer
    pwm_channel_tim2_ch1,  // AF1: PA0, PA5, PA15; AF2: PB3
    pwm_channel_tim2_ch2,  // AF1: PA1; AF2: PB3, PB10
    pwm_channel_tim2_ch3,  // AF1: PA2; AF2: PB10
    pwm_channel_tim2_ch4,  // AF1: PA3; AF2: PB11
    // TIM3 General-purpose Timer
    pwm_channel_tim3_ch1,  // AF2: PA6, PB4, PC6
    pwm_channel_tim3_ch2,  // AF2: PA7, PB5, PC7
    pwm_channel_tim3_ch3,  // AF2: PB0, PC8
    pwm_channel_tim3_ch4,  // AF2: PB1, PC9
    // TIM4 General-purpose Timer
    pwm_channel_tim4_ch1,  // AF2: PB6
    pwm_channel_tim4_ch2,  // AF2: PB7
    pwm_channel_tim4_ch3,  // AF2: PB8
    pwm_channel_tim4_ch4,  // AF2: PB9
    // TIM5 General-purpose Timer (32-bit)
    pwm_channel_tim5_ch1,  // AF2: PA0
    pwm_channel_tim5_ch2,  // AF2: PA1
    pwm_channel_tim5_ch3,  // AF2: PA2
    pwm_channel_tim5_ch4,  // AF2: PA3
    // TIM9 General-purpose Timer
    pwm_channel_tim9_ch1,  // AF3: PA2, PE5
    pwm_channel_tim9_ch2,  // AF3: PA3, PE6
    // TIM10 General-purpose Timer
    pwm_channel_tim10_ch1, // AF3: PB8; AF1: PA6
    // TIM11 General-purpose Timer
    pwm_channel_tim11_ch1, // AF3: PB9; AF1: PA7
    pwm_channel_max
} t_pwm_channel;

void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);


// --- ICU Module ---
typedef enum
{
    // ICU channels are often the same as PWM channels configured for input capture
    icu_channel_tim1_ch1,  // PA8,PE9,PB13,PA7
    icu_channel_tim1_ch2,  // PA9,PE11,PB0,PB14
    icu_channel_tim1_ch3,  // PA10,PE13,PB1,PB15
    icu_channel_tim1_ch4,  // PA11,PE14
    icu_channel_tim2_ch1,  // PA0,PA5,PA15,PB3
    icu_channel_tim2_ch2,  // PA1,PB3,PB10
    icu_channel_tim2_ch3,  // PA2,PB10
    icu_channel_tim2_ch4,  // PA3,PB11
    icu_channel_tim3_ch1,  // PA6,PB4,PC6
    icu_channel_tim3_ch2,  // PA7,PB5,PC7
    icu_channel_tim3_ch3,  // PB0,PC8
    icu_channel_tim3_ch4,  // PB1,PC9
    icu_channel_tim4_ch1,  // PB6
    icu_channel_tim4_ch2,  // PB7
    icu_channel_tim4_ch3,  // PB8
    icu_channel_tim4_ch4,  // PB9
    icu_channel_tim5_ch1,  // PA0
    icu_channel_tim5_ch2,  // PA1
    icu_channel_tim5_ch3,  // PA2
    icu_channel_tim5_ch4,  // PA3
    icu_channel_tim9_ch1,  // PA2,PE5
    icu_channel_tim9_ch2,  // PA3,PE6
    icu_channel_tim10_ch1, // PB8,PA6
    icu_channel_tim11_ch1, // PB9,PA7
    icu_channel_max
} t_icu_channel;

typedef enum
{
    icu_prescaler_div1,
    icu_prescaler_div2,
    icu_prescaler_div4,
    icu_prescaler_div8
} t_icu_prescaller;

typedef enum
{
    icu_edge_rising,
    icu_edge_falling,
    icu_edge_both
} t_icu_edge;

void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
// Note: ICU_GetFrequency() return type not specified in API.json, assuming tword for frequency value
tword ICU_GetFrequency(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));


// --- Timer Module ---
typedef enum
{
    timer_channel_tim1,
    timer_channel_tim2,
    timer_channel_tim3,
    timer_channel_tim4,
    timer_channel_tim5,
    timer_channel_tim9,
    timer_channel_tim10,
    timer_channel_tim11,
    timer_channel_max
} t_timer_channel;

void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);


// --- ADC Module ---
typedef enum
{
    adc_channel_0,  // PA0 (ADC1_IN0)
    adc_channel_1,  // PA1 (ADC1_IN1)
    adc_channel_2,  // PA2 (ADC1_IN2)
    adc_channel_3,  // PA3 (ADC1_IN3)
    adc_channel_4,  // PA4 (ADC1_IN4)
    adc_channel_5,  // PA5 (ADC1_IN5)
    adc_channel_6,  // PA6 (ADC1_IN6)
    adc_channel_7,  // PA7 (ADC1_IN7)
    adc_channel_8,  // PB0 (ADC1_IN8)
    adc_channel_9,  // PB1 (ADC1_IN9)
    adc_channel_10, // PC0 (ADC1_IN10)
    adc_channel_11, // PC1 (ADC1_IN11)
    adc_channel_12, // PC2 (ADC1_IN12)
    adc_channel_13, // PC3 (ADC1_IN13)
    adc_channel_14, // PC4 (ADC1_IN14)
    adc_channel_15, // PC5 (ADC1_IN15)
    // Internal channels like VREFINT, VBAT, Temperature Sensor can be added if needed
    adc_channel_max
} t_adc_channel;

typedef enum
{
    adc_mode_single_conversion,
    adc_mode_continuous_conversion,
    adc_mode_scan
} t_adc_mode_t;

void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(t_adc_channel adc_channel);
void ADC_Disable(t_adc_channel adc_channel);
tword ADC_Get_POLLING(t_adc_channel adc_channel);
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel);


// --- Internal_EEPROM Module ---
// For STM32F401RC, Internal_EEPROM functionality is typically implemented using
// Flash memory's Option Bytes or emulated EEPROM libraries.
// As no direct "EEPROM" registers are present in the provided register_json,
// this module is considered not directly supported via dedicated hardware registers.
// The following prototypes are commented out as per Rules.json: optional_modules_rule.
/*
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);
*/


// --- TT (Time Triggered OS) Module ---
typedef enum
{
    tick_time_1ms,
    tick_time_10ms,
    tick_time_100ms,
    tick_time_1s,
    tick_time_max
} t_tick_time;

void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);


// --- MCAL_OUTPUT_BUZZER Module ---
// MCAL_OUTPUT_BUZZER module not supported on this MCU based on provided register_json (no dedicated buzzer registers).
// The following prototypes are commented out as per Rules.json: optional_modules_rule.
/*
void BUZZER_OUTPUT_Init(tbyte buzzer_number);
void BUZZER_OUTPUT_Start(tbyte NUMBER_BUZZER);
void BUZZER_OUTPUT_Stop(tbyte NUMBER_BUZZER);
*/


// --- WDT Module ---
// WDT_Init and WDT_Reset are already part of MCU CONFIG module for this implementation.
// WDT is typically integrated with the MCU's System Control, often using a dedicated watchdog peripheral.
// The functions from API.json are already covered by MCU_Config, so no separate prototypes here.
/*
void WDT_Init(void); // Covered by MCU_Config_Init or implicit
void WDT_Reset(void); // Covered by MCU_Config module
*/


// --- DAC Module ---
// DAC module not supported on this MCU based on provided register_json (no dedicated DAC registers).
// The following typedef and prototypes are commented out as per Rules.json: optional_modules_rule.
/*
typedef enum
{
    dac_channel_1,
    dac_channel_2
} dac_channel_t;

void DAC_Init(dac_channel_t channel);
void DAC_Enable(dac_channel_t channel);
void DAC_Disable(dac_channel_t channel);
void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue);
*/


// --- I2S Module ---
// I2S is supported through SPI peripherals with I2S capability (SPI1, SPI2, SPI3).
typedef enum
{
    i2s_channel_i2s1, // Based on SPI1_I2SCFGR
    i2s_channel_i2s2, // Based on SPI2_I2SCFGR
    i2s_channel_i2s3, // Based on SPI3_I2SCFGR
    i2s_channel_max
} t_i2s_channel;

typedef enum
{
    i2s_mode_slave_tx,
    i2s_mode_slave_rx,
    i2s_mode_master_tx,
    i2s_mode_master_rx
} I2S_Mode_t;

typedef enum
{
    i2s_standard_philips,
    i2s_standard_msb_justified,
    i2s_standard_lsb_justified,
    i2s_standard_pcm_short,
    i2s_standard_pcm_long
} I2S_Standard_t;

typedef enum
{
    i2s_data_format_16b,
    i2s_data_format_24b,
    i2s_data_format_32b
} I2S_DataFormat_t;

typedef enum
{
    i2s_channel_mode_mono,
    i2s_channel_mode_stereo
} I2S_ChannelMode_t;

void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);


// --- MQTT Protocol Module ---
// MQTT Protocol module not supported on this MCU based on provided register_json (no dedicated hardware for MQTT).
// This is a higher-level network protocol, typically implemented on top of TCP/IP stack.
// The following prototypes are commented out as per Rules.json: optional_modules_rule.
/*
// Note: tsbyte type not defined, assuming it's signed char/int8_t
// typedef int8_t tsbyte;

void MQTT_Init(void);
// Note: esp_event_base_t, tslong (long signed int) types are ESP-IDF specific and not standard C types.
// If this module were to be implemented, these types would need to be re-defined or aliased.
// void MQTT_Subscribe_Topic(void *handler_args, esp_event_base_t base, tslong event_id, void *event_data);
// void MQTT_Publish_Message(const tsbyte *message);
// void AZURE_IoT_Hub_Client_Init(void);
// void AZURE_Connection_Enable(void);
*/


// --- HTTP Protocol Module ---
// HTTP Protocol module not supported on this MCU based on provided register_json (no dedicated hardware for HTTP).
// This is a higher-level network protocol, typically implemented on top of TCP/IP stack.
// The following prototypes are commented out as per Rules.json: optional_modules_rule.
/*
// Note: httpd_req_t type is ESP-IDF specific and not a standard C type.
// If this module were to be implemented, this type would need to be re-defined or aliased.
// #include "esp_httpd.h" // Placeholder if ESP-IDF were used.

void HTTP_Get_Device_ID(char *device_id, size_t size);
void HTTP_Server_Init(void);
void HTTP_Server_Start(void);
void HTTP_Server_Stop(void);
void HTTP_Reset_SSID_PASSWORD(void);
// esp_err_t HTTP_Config_Handler(httpd_req_t *req);
*/


// --- WiFi Driver Module ---
// WiFi Driver module not supported on this MCU based on provided register_json (STM32F401RC does not have integrated WiFi).
// The API functions (esp_err_t, tsbyte) suggest an ESP-IDF environment.
// The following prototypes are commented out as per Rules.json: optional_modules_rule.
/*
// Note: tsbyte type not defined, assuming it's signed char/int8_t
// typedef int8_t tsbyte;

typedef enum
{
    tx_mode_active,
    tx_mode_power_save
} t_tx_mode;

void WiFi_Init(void);
// void WiFi_Connect(const tsbyte *ssid, const tsbyte *password);
void WiFi_Enable(void);
void WiFi_Disable(void);
void WiFi_SetTxMode(t_tx_mode mode);
// tsword WiFi_Check_Connection(void); // tsword not defined, assuming int16_t
// int WiFi_Check_Internet(void);
*/


// --- DTC_driver Module ---
// DTC_driver module not supported on this MCU based on provided register_json (STM32 uses DMA, not a dedicated DTC).
// The following prototypes are commented out as per Rules.json: optional_modules_rule.
/*
void DTC_Init();
void DTC_EnableSource(uint8_t source_id, uint8_t channel);
void DTC_DisableSource(uint8_t source_id);
void DTC_Start(void);
void DTC_Stop(void);
void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int);
*/

#endif /* MCAL_H_ */