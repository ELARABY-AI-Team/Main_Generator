#ifndef MCAL_H
#define MCAL_H

// --- MCU NAME ---
// STM32F401RC

// --- CORE INCLUDES (from Rules.json) ---
#include "stm32f401xc.h"  // Core device header for STM32F401RC (as per rules example)
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- DATA TYPE DEFINITIONS (from Rules.json) ---
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// --- ENUMERATIONS AND TYPEDEFS (from API.json and Rules.json) ---

// MCU CONFIG Module
typedef enum {
    VSOURCE_3V = 0,
    VSOURCE_5V
} t_sys_volt;

// LVD Module
typedef enum {
    LVD_THRESHOLD_0V5 = 0, // Placeholder, actual values depend on specific LVD register bits
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1V5,
    LVD_THRESHOLD_2V,
    LVD_THRESHOLD_2V5,     // Assumed LVD levels for a general MCU
    LVD_THRESHOLD_3V,
    LVD_THRESHOLD_3V5,
    LVD_THRESHOLD_4V,
    LVD_THRESHOLD_4V5,
    LVD_THRESHOLD_5V
} t_lvd_thrthresholdLevel; // Typo 'thrthresholdLevel' maintained for exact match with API.json

// UART Module
typedef enum {
    UART_CHANNEL_1 = 0, // Maps to USART1
    UART_CHANNEL_2,     // Maps to USART2
    UART_CHANNEL_6      // Maps to USART6
} t_uart_channel;

typedef enum {
    UART_BAUD_RATE_9600 = 0, // Common standard baud rates
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_38400,
    UART_BAUD_RATE_57600,
    UART_BAUD_RATE_115200,
    UART_BAUD_RATE_230400,
    UART_BAUD_RATE_460800,
    UART_BAUD_RATE_921600
} t_uart_baud_rate;

typedef enum {
    UART_DATA_LENGTH_8B = 0,
    UART_DATA_LENGTH_9B
} t_uart_data_length;

typedef enum {
    UART_STOP_BIT_1 = 0,
    UART_STOP_BIT_0_5,
    UART_STOP_BIT_2,
    UART_STOP_BIT_1_5
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// I2C Module
typedef enum {
    I2C_CHANNEL_1 = 0, // Maps to I2C1
    I2C_CHANNEL_2,     // Maps to I2C2
    I2C_CHANNEL_3      // Maps to I2C3
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_STANDARD = 0, // 100 kHz
    I2C_CLK_SPEED_FAST          // 400 kHz (as per rule: "Always use fast mode")
} t_i2c_clk_speed;

typedef uint16_t t_i2c_device_address; // To hold 7-bit or 10-bit I2C address

typedef enum {
    I2C_ACK_DISABLE = 0,
    I2C_ACK_ENABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_LENGTH_8BIT = 0,
    I2C_DATA_LENGTH_16BIT // Though I2C DR is typically 8-bit, API requested this.
} t_i2c_datalength;

// SPI (CSI) Module
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
    SPI_CPOL_LOW = 0,  // Clock polarity low
    SPI_CPOL_HIGH      // Clock polarity high
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1EDGE = 0, // Clock phase, 1st edge
    SPI_CPHA_2EDGE      // Clock phase, 2nd edge
} t_spi_cpha;

typedef enum {
    SPI_DFF_8BIT = 0, // Data frame format 8-bit
    SPI_DFF_16BIT     // Data frame format 16-bit
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0, // MSB transmitted first
    SPI_BIT_ORDER_LSB_FIRST      // LSB transmitted first
} t_spi_bit_order;

// External Interrupt Module
// EXTI channels map to the line number (0-15), not individual pins.
// Any pin Px_N can be an EXTI_N source.
typedef enum {
    EXTERNAL_INT_CHANNEL_0 = 0,
    EXTERNAL_INT_CHANNEL_1,
    EXTERNAL_INT_CHANNEL_2,
    EXTERNAL_INT_CHANNEL_3,
    EXTERNAL_INT_CHANNEL_4,
    EXTERNAL_INT_CHANNEL_5,
    EXTERNAL_INT_CHANNEL_6,
    EXTERNAL_INT_CHANNEL_7,
    EXTERNAL_INT_CHANNEL_8,
    EXTERNAL_INT_CHANNEL_9,
    EXTERNAL_INT_CHANNEL_10,
    EXTERNAL_INT_CHANNEL_11,
    EXTERNAL_INT_CHANNEL_12,
    EXTERNAL_INT_CHANNEL_13,
    EXTERNAL_INT_CHANNEL_14,
    EXTERNAL_INT_CHANNEL_15
} t_external_int_channel;

typedef enum {
    EXTERNAL_INT_EDGE_RISING = 0,
    EXTERNAL_INT_EDGE_FALLING,
    EXTERNAL_INT_EDGE_RISING_FALLING
} t_external_int_edge;

// GPIO Module
typedef enum {
    PORT_A = 0, // GPIOA Base Address
    PORT_B,     // GPIOB Base Address
    PORT_C,     // GPIOC Base Address
    PORT_D,     // GPIOD Base Address
    PORT_E,     // GPIOE Base Address
    PORT_H      // GPIOH Base Address
} t_port;

typedef enum {
    PIN_0 = 0,
    PIN_1,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7,
    PIN_8,
    PIN_9,
    PIN_10,
    PIN_11,
    PIN_12,
    PIN_13,
    PIN_14,
    PIN_15
} t_pin;

typedef enum {
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;

// PWM Module (Derived from TIMx_CHy available in register_json)
typedef enum {
    PWM_CHANNEL_TIM1_CH1 = 0, // PA8, PE9
    PWM_CHANNEL_TIM1_CH2,     // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,     // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,     // PA11, PE14
    PWM_CHANNEL_TIM2_CH1,     // PA0, PA5, PA15, PB3
    PWM_CHANNEL_TIM2_CH2,     // PA1, PB3, PB10
    PWM_CHANNEL_TIM2_CH3,     // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,     // PA3, PB11
    PWM_CHANNEL_TIM3_CH1,     // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,     // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,     // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,     // PB1, PC9
    PWM_CHANNEL_TIM4_CH1,     // PB6
    PWM_CHANNEL_TIM4_CH2,     // PB7
    PWM_CHANNEL_TIM4_CH3,     // PB8
    PWM_CHANNEL_TIM4_CH4,     // PB9
    PWM_CHANNEL_TIM5_CH1,     // PA0
    PWM_CHANNEL_TIM5_CH2,     // PA1
    PWM_CHANNEL_TIM5_CH3,     // PA2
    PWM_CHANNEL_TIM5_CH4,     // PA3
    PWM_CHANNEL_TIM9_CH1,     // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,     // PA3, PE6
    PWM_CHANNEL_TIM10_CH1,    // PB8, PA6
    PWM_CHANNEL_TIM11_CH1     // PB9, PA7
} t_pwm_channel;

// ICU Module (Derived from TIMx_CHy, as ICU uses timer capture modes)
typedef enum {
    ICU_CHANNEL_TIM1_CH1 = 0, // PA8, PE9
    ICU_CHANNEL_TIM1_CH2,     // PA9, PE11
    ICU_CHANNEL_TIM1_CH3,     // PA10, PE13
    ICU_CHANNEL_TIM1_CH4,     // PA11, PE14
    ICU_CHANNEL_TIM2_CH1,     // PA0, PA5, PA15, PB3
    ICU_CHANNEL_TIM2_CH2,     // PA1, PB3, PB10
    ICU_CHANNEL_TIM2_CH3,     // PA2, PB10
    ICU_CHANNEL_TIM2_CH4,     // PA3, PB11
    ICU_CHANNEL_TIM3_CH1,     // PA6, PB4, PC6
    ICU_CHANNEL_TIM3_CH2,     // PA7, PB5, PC7
    ICU_CHANNEL_TIM3_CH3,     // PB0, PC8
    ICU_CHANNEL_TIM3_CH4,     // PB1, PC9
    ICU_CHANNEL_TIM4_CH1,     // PB6
    ICU_CHANNEL_TIM4_CH2,     // PB7
    ICU_CHANNEL_TIM4_CH3,     // PB8
    ICU_CHANNEL_TIM4_CH4,     // PB9
    ICU_CHANNEL_TIM5_CH1,     // PA0
    ICU_CHANNEL_TIM5_CH2,     // PA1
    ICU_CHANNEL_TIM5_CH3,     // PA2
    ICU_CHANNEL_TIM5_CH4,     // PA3
    ICU_CHANNEL_TIM9_CH1,     // PA2, PE5
    ICU_CHANNEL_TIM9_CH2,     // PA3, PE6
    ICU_CHANNEL_TIM10_CH1,    // PB8, PA6
    ICU_CHANNEL_TIM11_CH1     // PB9, PA7
} t_icu_channel;

typedef enum {
    ICU_PRESCALER_DIV1 = 0, // Placeholder values, actual depend on TIMx_PSC register bits
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8,
    ICU_PRESCALER_DIV16,
    ICU_PRESCALER_DIV32,
    ICU_PRESCALER_DIV64,
    ICU_PRESCALER_DIV128,
    ICU_PRESCALER_DIV256,
    ICU_PRESCALER_DIV512
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer Module (Derived from TIMx instances in register_json)
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

// ADC Module (Derived from ADC1_INx assigned to pins in register_json)
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
    // Internal ADC channels like VREFINT, TEMPSENSOR are not explicitly listed
    // with assigned_pin in the register_json, so they are not included here.
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE = 0,
    ADC_MODE_CONTINUOUS
} t_adc_mode_t;

// TT Module
typedef enum {
    TT_TICK_TIME_1MS = 0,
    TT_TICK_TIME_10MS,
    TT_TICK_TIME_100MS,
    TT_TICK_TIME_1S
} t_tick_time;

// I2S Module (Optional module, implemented because SPIx_I2SCFGR/I2SPR registers exist)
typedef enum {
    I2S_CHANNEL_1 = 0, // Corresponds to SPI1
    I2S_CHANNEL_2,     // Corresponds to SPI2
    I2S_CHANNEL_3      // Corresponds to SPI3
} t_i2s_channel;

typedef enum {
    I2S_MODE_SLAVE_TX = 0,
    I2S_MODE_SLAVE_RX,
    I2S_MODE_MASTER_TX,
    I2S_MODE_MASTER_RX
} I2S_Mode_t;

typedef enum {
    I2S_STANDARD_PHILIPS = 0,
    I2S_STANDARD_MSB,
    I2S_STANDARD_LSB,
    I2S_STANDARD_PCM_SHORT,
    I2S_STANDARD_PCM_LONG
} I2S_Standard_t;

typedef enum {
    I2S_DATAFORMAT_16B = 0,
    I2S_DATAFORMAT_16B_EXTENDED,
    I2S_DATAFORMAT_24B,
    I2S_DATAFORMAT_32B
} I2S_DataFormat_t;

typedef enum {
    I2S_CHANNELMODE_STEREO = 0,
    I2S_CHANNELMODE_MONO
} I2S_ChannelMode_t;


// --- API FUNCTION PROTOTYPES (from API.json) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // Prototyped here as per API.json structure
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
void GPIO_Value_Set(t_port port, t_pin pin, t_byte value);
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
tword ICU_GetFrequency(t_icu_channel icu_channel); // Return type assumed as tword
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

// MCAL_OUTPUT_BUZZER not supported on this MCU (no dedicated registers found in input JSON)
// DAC not supported on this MCU (no dedicated registers found in input JSON)
// MQTT Protocol not supported on this MCU (software-level protocol, no dedicated registers found in input JSON)
// HTTP Protocol not supported on this MCU (software-level protocol, no dedicated registers found in input JSON)
// WiFi Driver not supported on this MCU (external hardware/software-level driver, no dedicated registers found in input JSON)
// DTC_driver not supported on this MCU (no dedicated registers found in input JSON)

// WDT
void WDT_Init(void);
// WDT_Reset is already declared under MCU_Config

// I2S (Optional module, implemented because registers like SPIx_I2SCFGR and SPIx_I2SPR exist)
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

#endif /* MCAL_H */