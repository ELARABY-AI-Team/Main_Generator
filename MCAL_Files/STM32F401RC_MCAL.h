#ifndef MCAL_H
#define MCAL_H

#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// Other standard includes if necessary:
// #include <string.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

// Data type definitions as per Rules.json
typedef uint8_t tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

#define Unit_8 tbyte
#define unit_16 tword
#define unit_32 tlong

// MCU CONFIG API type definitions
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

// LVD API type definitions
typedef enum {
    LVD_THRESHOLD_0_5V = 0,
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1_5V,
    LVD_THRESHOLD_2V,
    // Add other thresholds up to 5V as per rule if applicable for specific hardware
    LVD_THRESHOLD_2_5V, // Inferred
    LVD_THRESHOLD_3V,   // Inferred
    LVD_THRESHOLD_3_5V, // Inferred for 5V system voltage
    LVD_THRESHOLD_4V,   // Inferred
    LVD_THRESHOLD_4_5V, // Inferred
    LVD_THRESHOLD_5V    // Inferred
} t_lvd_thrthresholdLevel;

typedef enum {
    LVD_CHANNEL_NONE = 0 // Placeholder, as no explicit LVD channels are defined in register_json
} t_lvd_channel;

// UART API type definitions
typedef enum {
    UART_CHANNEL_1 = 0, // Maps to USART1
    UART_CHANNEL_2,     // Maps to USART2
    UART_CHANNEL_6      // Maps to USART6
} t_uart_channel;

typedef enum {
    UART_BAUD_9600 = 0, // Example baud rate, actual value depends on clock
    UART_BAUD_19200,    // Inferred
    UART_BAUD_115200    // Inferred
    // Add other common baud rates as needed
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8_BIT = 0, // Example data length
    UART_DATA_9_BIT      // Inferred
} t_uart_data_length;

typedef enum {
    UART_STOP_1_BIT = 0, // Example stop bits
    UART_STOP_0_5_BIT,   // Inferred
    UART_STOP_2_BIT,     // Inferred
    UART_STOP_1_5_BIT    // Inferred
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0, // Example parity options
    UART_PARITY_EVEN,     // Inferred
    UART_PARITY_ODD       // Inferred
} t_uart_parity;

// I2C API type definitions
typedef enum {
    I2C_CHANNEL_1 = 0, // Maps to I2C1
    I2C_CHANNEL_2,     // Maps to I2C2
    I2C_CHANNEL_3      // Maps to I2C3
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_STANDARD = 0, // 100 kHz (inferred)
    I2C_CLK_SPEED_FAST          // 400 kHz (inferred)
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // 7-bit or 10-bit address

typedef enum {
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATALENGTH_8BIT = 0,
    I2C_DATALENGTH_16BIT // Inferred
} t_i2c_datalength;

// SPI API type definitions
typedef enum {
    SPI_CHANNEL_1 = 0, // Maps to SPI1
    SPI_CHANNEL_2,     // Maps to SPI2
    SPI_CHANNEL_3      // Maps to SPI3
} t_spi_channel;

typedef enum {
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW = 0,  // Clock Phase Low
    SPI_CPOL_HIGH      // Clock Phase High
} t_spi_cpol;

typedef enum {
    SPI_CPHA_FIRST = 0, // Clock Phase First Edge
    SPI_CPHA_SECOND     // Clock Phase Second Edge
} t_spi_cpha;

typedef enum {
    SPI_DFF_8_BIT = 0,  // Data Frame Format 8-bit
    SPI_DFF_16_BIT      // Data Frame Format 16-bit
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt API type definitions
typedef enum {
    EXT_INT_CHANNEL_0 = 0,
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

typedef enum {
    EXT_INT_EDGE_RISING = 0,
    EXT_INT_EDGE_FALLING,
    EXT_INT_EDGE_BOTH // Inferred for combined RTSR/FTSR configuration
} t_external_int_edge;

// GPIO API type definitions
typedef enum {
    GPIO_PORTA = 0,
    GPIO_PORTB,
    GPIO_PORTC,
    GPIO_PORTD,
    GPIO_PORTE,
    // GPIOH has fewer pins but is a separate port
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
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;


// PWM API type definitions (mapping to TIM channels based on assigned_pin in register_json)
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


// ICU API type definitions (similar mapping to TIM channels)
typedef t_pwm_channel t_icu_channel; // Re-use channel enum for simplicity

typedef enum {
    ICU_PRESCALER_1 = 0, // Example prescalers
    ICU_PRESCALER_8,     // Inferred
    ICU_PRESCALER_64,    // Inferred
    ICU_PRESCALER_256    // Inferred
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH_EDGES // Inferred
} t_icu_edge;


// Timer API type definitions
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

// ADC API type definitions
typedef enum {
    ADC_CHANNEL_1 = 0 // Represents the single ADC peripheral (ADC1)
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE_CONVERSION = 0,
    ADC_MODE_CONTINUOUS_CONVERSION // Inferred
} t_adc_mode_t;


// TT API type definitions
typedef enum {
    TT_TICK_TIME_MS_1 = 1,
    TT_TICK_TIME_MS_10 = 10,
    TT_TICK_TIME_MS_100 = 100
    // Add other common tick times
} t_tick_time;


// Function Prototypes (from API.json)
// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
void LVD_Enable(void);
void LVD_Disable(void);
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Corrected return type from API.json

// UART
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
void UART_ClearFlag(t_uart_channel uart_channel); // Corrected return type from API.json

// I2C
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
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Corrected return type from API.json

// SPI
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
void SPI_ClearFlag(t_spi_channel spi_channel); // Corrected return type from API.json

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Corrected return type from API.json

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
void ICU_Updatefrequency(t_icu_channel icu_channel);
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Corrected return type from API.json
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Corrected return type from API.json

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Corrected return type from API.json

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void); // Corrected return type from API.json

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

#endif // MCAL_H