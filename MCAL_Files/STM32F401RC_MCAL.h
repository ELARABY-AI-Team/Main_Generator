/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File
 *
 * This file contains the API definitions and type declarations for the MCAL
 * for STM32F401RC microcontroller.
 *
 * @author Your Name/Tool
 * @date YYYY-MM-DD
 *
 * @note This file is generated based on provided register definitions and API specifications.
 *       STM32F401RC specific register addresses and bitfields are used.
 */

#ifndef MCAL_H
#define MCAL_H

// --- Core MCU Header and Standard C Library Includes ---
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>   // For NULL, if used by some string functions in .c
#include <stdlib.h>  // For dynamic memory, not used in this MCAL for simplicity
#include <math.h>    // For mathematical operations, not used in this MCAL for simplicity


// --- Data Type Definitions (from Rules.json) ---
#define Unit_8 uint8_t
#define unit_16 uint16_t
#define unit_32 uint32_t

typedef Unit_8 tbyte;
typedef unit_16 tword;
typedef unit_32 tlong;
typedef const char tsbyte; // For string literals in API


// --- Generic Register Access Macros ---
#define REG_WRITE(ADDRESS, VALUE) (*((volatile uint32_t*)(ADDRESS)) = (VALUE))
#define REG_READ(ADDRESS)         (*((volatile uint32_t*)(ADDRESS)))
#define SET_BIT(ADDRESS, BIT)     (*((volatile uint32_t*)(ADDRESS)) |= (1U << (BIT)))
#define CLEAR_BIT(ADDRESS, BIT)   (*((volatile uint32_t*)(ADDRESS)) &= ~(1U << (BIT)))
#define TOGGLE_BIT(ADDRESS, BIT)  (*((volatile uint32_t*)(ADDRESS)) ^= (1U << (BIT)))
#define READ_BIT(ADDRESS, BIT)    (((*((volatile uint32_t*)(ADDRESS))) >> (BIT)) & 0x1U)


// --- MCU CONFIG Typedefs ---
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;


// --- UART Typedefs ---
typedef enum {
    UART_CHANNEL_1 = 0,
    UART_CHANNEL_2,
    UART_CHANNEL_6
} t_uart_channel;

typedef enum {
    UART_BAUD_9600,
    UART_BAUD_19200,
    UART_BAUD_38400,
    UART_BAUD_57600,
    UART_BAUD_115200,
    // Add more common baud rates if needed
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8_BITS,
    UART_DATA_9_BITS
} t_uart_data_length;

typedef enum {
    UART_STOP_1_BIT,
    UART_STOP_0_5_BIT, // Only for USART
    UART_STOP_2_BITS,
    UART_STOP_1_5_BIT  // Only for USART
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;


// --- I2C Typedefs ---
typedef enum {
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_STANDARD = 100000, // 100 kHz
    I2C_CLK_SPEED_FAST     = 400000  // 400 kHz
    // Note: Fast mode is enforced by rules.json, but enum allows flexibility.
} t_i2c_clk_speed;

typedef uint8_t t_i2c_device_address; // 7-bit address
typedef enum {
    I2C_ACK_ENABLE,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_8_BITS,
    I2C_DATA_16_BITS // For some I2C transactions
} t_i2c_datalength;


// --- SPI (CSI) Typedefs ---
typedef enum {
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

typedef enum {
    SPI_MODE_SLAVE,
    SPI_MODE_MASTER
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW,  // Clock polarity low
    SPI_CPOL_HIGH  // Clock polarity high
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1EDGE, // 1st clock transition is first data capture edge
    SPI_CPHA_2EDGE  // 2nd clock transition is first data capture edge
} t_spi_cpha;

typedef enum {
    SPI_DFF_8_BITS,  // 8-bit data frame format
    SPI_DFF_16_BITS  // 16-bit data frame format
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;


// --- External Interrupt Typedefs ---
typedef enum {
    EXTI_CHANNEL_0 = 0,
    EXTI_CHANNEL_1,
    EXTI_CHANNEL_2,
    EXTI_CHANNEL_3,
    EXTI_CHANNEL_4,
    EXTI_CHANNEL_5,
    EXTI_CHANNEL_6,
    EXTI_CHANNEL_7,
    EXTI_CHANNEL_8,
    EXTI_CHANNEL_9,
    EXTI_CHANNEL_10,
    EXTI_CHANNEL_11,
    EXTI_CHANNEL_12,
    EXTI_CHANNEL_13,
    EXTI_CHANNEL_14,
    EXTI_CHANNEL_15,
    // Note: EXTI lines 16, 17, 18, 19, 20, 21, 22 are internal and not directly pin-assignable
} t_external_int_channel;

typedef enum {
    EXTI_EDGE_RISING,
    EXTI_EDGE_FALLING,
    EXTI_EDGE_RISING_FALLING
} t_external_int_edge;


// --- GPIO Typedefs ---
typedef enum {
    GPIO_PORTA = 0,
    GPIO_PORTB,
    GPIO_PORTC,
    GPIO_PORTD,
    GPIO_PORTE,
    GPIO_PORTH
} t_port;

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

typedef enum {
    GPIO_DIRECTION_IN,
    GPIO_DIRECTION_OUT
} t_direction;

typedef tbyte t_gpio_value; // For setting/getting pin value (0 or 1)


// --- PWM Typedefs ---
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


// --- ICU Typedefs ---
typedef enum {
    ICU_CHANNEL_TIM1_CH1,
    ICU_CHANNEL_TIM1_CH2,
    ICU_CHANNEL_TIM1_CH3,
    ICU_CHANNEL_TIM1_CH4,
    ICU_CHANNEL_TIM2_CH1,
    ICU_CHANNEL_TIM2_CH2,
    ICU_CHANNEL_TIM2_CH3,
    ICU_CHANNEL_TIM2_CH4,
    ICU_CHANNEL_TIM3_CH1,
    ICU_CHANNEL_TIM3_CH2,
    ICU_CHANNEL_TIM3_CH3,
    ICU_CHANNEL_TIM3_CH4,
    ICU_CHANNEL_TIM4_CH1,
    ICU_CHANNEL_TIM4_CH2,
    ICU_CHANNEL_TIM4_CH3,
    ICU_CHANNEL_TIM4_CH4,
    ICU_CHANNEL_TIM5_CH1,
    ICU_CHANNEL_TIM5_CH2,
    ICU_CHANNEL_TIM5_CH3,
    ICU_CHANNEL_TIM5_CH4,
    ICU_CHANNEL_TIM9_CH1,
    ICU_CHANNEL_TIM9_CH2,
    ICU_CHANNEL_TIM10_CH1,
    ICU_CHANNEL_TIM11_CH1
} t_icu_channel;

typedef enum {
    ICU_PRESCALER_DIV1 = 0,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH // Only if supported by timer mode
} t_icu_edge;


// --- Timer Typedefs ---
typedef enum {
    TIMER_CHANNEL_1 = 0,
    TIMER_CHANNEL_2,
    TIMER_CHANNEL_3,
    TIMER_CHANNEL_4,
    TIMER_CHANNEL_5,
    TIMER_CHANNEL_9,
    TIMER_CHANNEL_10,
    TIMER_CHANNEL_11
} t_timer_channel;


// --- ADC Typedefs ---
typedef enum {
    ADC_CHANNEL_1 = 0 // STM32F401RC has only ADC1
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE_CONVERSION,
    ADC_MODE_CONTINUOUS_CONVERSION
} t_adc_mode_t;


// --- TT Typedefs ---
typedef uint32_t t_tick_time; // Time in milliseconds for the tick

// For TT task management
typedef struct {
    void (*pTask)(void);
    tword Delay;
    tword Period;
    bool RunMe;
    tbyte TaskID; // To uniquely identify tasks
} sTask;

#define MAX_TASKS 10 // Example maximum number of tasks


// --- DAC Typedefs ---
typedef enum {
    DAC_CHANNEL_1 = 0, // Assigned pin: PA4
    DAC_CHANNEL_2      // Assigned pin: PA5
} dac_channel_t;


// --- I2S Typedefs ---
typedef enum {
    I2S_CHANNEL_1 = 0, // Mapped to SPI1
    I2S_CHANNEL_2,     // Mapped to SPI2
    I2S_CHANNEL_3      // Mapped to SPI3
} t_i2s_channel;

typedef enum {
    I2S_MODE_SLAVE_TX,
    I2S_MODE_SLAVE_RX,
    I2S_MODE_MASTER_TX,
    I2S_MODE_MASTER_RX
} I2S_Mode_t;

typedef enum {
    I2S_STANDARD_PHILIPS,
    I2S_STANDARD_MSB,
    I2S_STANDARD_LSB,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

typedef enum {
    I2S_DATAFORMAT_16B,
    I2S_DATAFORMAT_16B_EXTENDED, // For 24-bit data, stored in 32-bit frame
    I2S_DATAFORMAT_24B,          // For 24-bit data, stored in 32-bit frame
    I2S_DATAFORMAT_32B
} I2S_DataFormat_t;

typedef enum {
    I2S_CHANNELMODE_STEREO,
    I2S_CHANNELMODE_MONO // Not directly configurable for all standards
} I2S_ChannelMode_t;


// --- API Function Prototypes (from API.json) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// WDT
void WDT_Init(void); // Defined in WDT section, but also relevant for MCU CONFIG
void WDT_Reset(void);

// LVD (Low Voltage Detection) - NOT SUPPORTED ON THIS MCU (registers not found in JSON)
// void LVD_Init(void);
// void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
// void LVD_Enable(void);
// void LVD_Disable(void);

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
void GPIO_Value_Set(t_port port, t_pin pin, t_gpio_value value);
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
tlong ICU_GetFrequency(t_icu_channel icu_channel);
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

// Internal_EEPROM - NOT SUPPORTED ON THIS MCU (registers not found in JSON)
// void Internal_EEPROM_Init(void);
// void Internal_EEPROM_Set(tbyte address, tbyte data);
// tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time Triggered)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// MCAL_OUTPUT_BUZZER - NOT SUPPORTED ON THIS MCU (registers not found in JSON)
// void BUZZER_OUTPUT_Init(tbyte buzzer_number);
// void BUZZER_OUTPUT_Start(tbyte NUMBER_BUZZER);
// void BUZZER_OUTPUT_Stop(tbyte NUMBER_BUZZER);

// DAC
void DAC_Init(dac_channel_t channel);
void DAC_Enable(dac_channel_t channel);
void DAC_Disable(dac_channel_t channel);
void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue);

// I2S
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// MQTT Protocol - NOT SUPPORTED ON THIS MCU (registers/SDK dependencies not found in JSON)
// void MQTT_Init(void);
// void MQTT_Subscribe_Topic(void *handler_args, esp_event_base_t base, tslong event_id, void *event_data);
// void MQTT_Publish_Message(const tsbyte *message);
// void AZURE_IoT_Hub_Client_Init(void);
// void AZURE_Connection_Enable(void);

// HTTP Protocol - NOT SUPPORTED ON THIS MCU (registers/SDK dependencies not found in JSON)
// void HTTP_Get_Device_ID(char *device_id, size_t size);
// void HTTP_Server_Init(void);
// void HTTP_Server_Start(void);
// void HTTP_Server_Stop(void);
// void HTTP_Reset_SSID_PASSWORD(void);
// esp_err_t HTTP_Config_Handler(httpd_req_t *req);

// WiFi Driver - NOT SUPPORTED ON THIS MCU (registers/SDK dependencies not found in JSON)
// void WiFi_Init(void);
// void WiFi_Connect(const tsbyte *ssid, const tsbyte *password);
// void WiFi_Enable(void);
// void WiFi_Disable(void);
// void WiFi_SetTxMode(t_tx_mode mode);
// tsword WiFi_Check_Connection(void);
// int WiFi_Check_Internet(void);

// DTC_driver - NOT SUPPORTED ON THIS MCU (registers/SDK dependencies not found in JSON)
// void DTC_Init();
// void DTC_EnableSource(uint8_t source_id, uint8_t channel);
// void DTC_DisableSource(uint8_t source_id);
// void DTC_Start(void);
// void DTC_Stop(void);
// void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int);

#endif // MCAL_H