#ifndef MCAL_H
#define MCAL_H

// global_includes from Rules.json
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// data_type_definitions from Rules.json
#define tbyte  uint8_t
#define tword  uint16_t
#define tlong  uint32_t

// --- General MCAL Configuration ---
#define MCU_NAME "STM32F401RC"

// Register access macro
#define REG_ACCESS_U32(address) (*((volatile tlong *)(address)))

// --- Enums and Macros based on API.json and Register JSON ---

// MCU CONFIG
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

// LVD
typedef enum {
    Volt_0_5V,
    Volt_1V,
    Volt_1_5V,
    Volt_2V,
    Volt_2_5V,
    Volt_3V,
    Volt_3_5V,
    Volt_4V,
    Volt_4_5V,
    Volt_5V
} t_lvd_thrthresholdLevel;

// UART
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
    UART_BAUD_115200
    // Add more baud rates as needed
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8_BITS,
    UART_DATA_9_BITS
} t_uart_data_length;

typedef enum {
    UART_STOP_1_BIT,
    UART_STOP_0_5_BIT, // Not typically supported on STM32 for standard UART
    UART_STOP_2_BITS,
    UART_STOP_1_5_BITS // Not typically supported on STM32 for standard UART
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// I2C
typedef enum {
    I2C_CHANNEL_1 = 0,
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum {
    I2C_CLK_STANDARD_MODE = 100000, // 100 kHz
    I2C_CLK_FAST_MODE = 400000      // 400 kHz (as per rule: always use fast mode)
} t_i2c_clk_speed;

typedef uint8_t t_i2c_device_address; // Device address is passed as a parameter

typedef enum {
    I2C_ACK_ENABLE,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_8_BITS,
    I2C_DATA_16_BITS // Standard I2C transfers are 8-bit, but some internal registers might be 16-bit
} t_i2c_datalength;

// SPI (CSI)
typedef enum {
    SPI_CHANNEL_1 = 0,
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

typedef enum {
    SPI_MODE_SLAVE = 0,
    SPI_MODE_MASTER
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW = 0,  // Clock polarity low
    SPI_CPOL_HIGH      // Clock polarity high
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1_EDGE = 0, // Clock phase, first clock transition is the first data capture edge
    SPI_CPHA_2_EDGE      // Clock phase, second clock transition is the first data capture edge
} t_spi_cpha;

typedef enum {
    SPI_DFF_8_BITS = 0,
    SPI_DFF_16_BITS
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt
typedef enum {
    EXTI_LINE_0 = 0,
    EXTI_LINE_1,
    EXTI_LINE_2,
    EXTI_LINE_3,
    EXTI_LINE_4,
    EXTI_LINE_5,
    EXTI_LINE_6,
    EXTI_LINE_7,
    EXTI_LINE_8,
    EXTI_LINE_9,
    EXTI_LINE_10,
    EXTI_LINE_11,
    EXTI_LINE_12,
    EXTI_LINE_13,
    EXTI_LINE_14,
    EXTI_LINE_15
} t_external_int_channel;

typedef enum {
    EXTI_EDGE_RISING,
    EXTI_EDGE_FALLING,
    EXTI_EDGE_RISING_FALLING
} t_external_int_edge;

// GPIO
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
    GPIO_INPUT = 0,
    GPIO_OUTPUT
} t_direction;

// PWM
typedef enum {
    PWM_TIM1_CH1,  // PA8, PE9, PB13, PA7
    PWM_TIM1_CH2,  // PA9, PE11, PB0, PB14
    PWM_TIM1_CH3,  // PA10, PE13, PB1, PB15
    PWM_TIM1_CH4,  // PA11, PE14
    PWM_TIM2_CH1,  // PA0, PA5, PA15, PB3
    PWM_TIM2_CH2,  // PA1, PB3, PB10
    PWM_TIM2_CH3,  // PA2, PB10
    PWM_TIM2_CH4,  // PA3, PB11
    PWM_TIM3_CH1,  // PA6, PB4, PC6
    PWM_TIM3_CH2,  // PA7, PB5, PC7
    PWM_TIM3_CH3,  // PB0, PC8
    PWM_TIM3_CH4,  // PB1, PC9
    PWM_TIM4_CH1,  // PB6
    PWM_TIM4_CH2,  // PB7
    PWM_TIM4_CH3,  // PB8
    PWM_TIM4_CH4,  // PB9
    PWM_TIM5_CH1,  // PA0
    PWM_TIM5_CH2,  // PA1
    PWM_TIM5_CH3,  // PA2
    PWM_TIM5_CH4,  // PA3
    PWM_TIM9_CH1,  // PA2, PE5
    PWM_TIM9_CH2,  // PA3, PE6
    PWM_TIM10_CH1, // PB8, PA6
    PWM_TIM11_CH1  // PB9, PA7
} t_pwm_channel;

// ICU
typedef enum {
    ICU_TIM1_CH1,  // PA8, PE9, PB13, PA7
    ICU_TIM1_CH2,  // PA9, PE11, PB0, PB14
    ICU_TIM1_CH3,  // PA10, PE13, PB1, PB15
    ICU_TIM1_CH4,  // PA11, PE14
    ICU_TIM2_CH1,  // PA0, PA5, PA15, PB3
    ICU_TIM2_CH2,  // PA1, PB3, PB10
    ICU_TIM2_CH3,  // PA2, PB10
    ICU_TIM2_CH4,  // PA3, PB11
    ICU_TIM3_CH1,  // PA6, PB4, PC6
    ICU_TIM3_CH2,  // PA7, PB5, PC7
    ICU_TIM3_CH3,  // PB0, PC8
    ICU_TIM3_CH4,  // PB1, PC9
    ICU_TIM4_CH1,  // PB6
    ICU_TIM4_CH2,  // PB7
    ICU_TIM4_CH3,  // PB8
    ICU_TIM4_CH4,  // PB9
    ICU_TIM5_CH1,  // PA0
    ICU_TIM5_CH2,  // PA1
    ICU_TIM5_CH3,  // PA2
    ICU_TIM5_CH4,  // PA3
    ICU_TIM9_CH1,  // PA2, PE5
    ICU_TIM9_CH2,  // PA3, PE6
    ICU_TIM10_CH1, // PB8, PA6
    ICU_TIM11_CH1  // PB9, PA7
} t_icu_channel;

typedef enum {
    ICU_PRESCALER_DIV1,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8,
    // Add more as per specific timer capabilities (e.g., 256 for 8-bit, 65536 for 16-bit)
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer
typedef enum {
    TIMER_CHANNEL_1 = 0, // TIM1
    TIMER_CHANNEL_2,     // TIM2
    TIMER_CHANNEL_3,     // TIM3
    TIMER_CHANNEL_4,     // TIM4
    TIMER_CHANNEL_5,     // TIM5
    TIMER_CHANNEL_9,     // TIM9
    TIMER_CHANNEL_10,    // TIM10
    TIMER_CHANNEL_11     // TIM11
} t_timer_channel;

// ADC
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
    ADC_CHANNEL_15,     // PC5
    ADC_CHANNEL_TEMP,   // Internal Temperature Sensor
    ADC_CHANNEL_VREF    // Internal VREFINT
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE_CONVERSION,
    ADC_MODE_CONTINUOUS_CONVERSION,
    ADC_MODE_SCAN_CONVERSION
} t_adc_mode_t;

// TT (Time Triggered)
typedef enum {
    TT_TICK_1MS = 1,
    TT_TICK_10MS = 10,
    TT_TICK_100MS = 100
} t_tick_time;

// I2S
typedef enum {
    I2S_CHANNEL_1, // SPI1_I2S
    I2S_CHANNEL_2, // SPI2_I2S
    I2S_CHANNEL_3  // SPI3_I2S
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
    I2S_DATAFORMAT_16B_EXTENDED,
    I2S_DATAFORMAT_24B,
    I2S_DATAFORMAT_32B
} I2S_DataFormat_t;

typedef enum {
    I2S_CHANNELMODE_STEREO,
    I2S_CHANNELMODE_MONO
} I2S_ChannelMode_t;

// DAC not supported on this MCU (no DAC registers in register_json).
// MQTT Protocol not supported on this MCU (no specific registers/drivers).
// HTTP Protocol not supported on this MCU (no specific registers/drivers).
// WiFi Driver not supported on this MCU (no specific registers/drivers).
// DTC_driver not supported on this MCU (no DTC registers/drivers).
// MCAL_OUTPUT_BUZZER not supported on this MCU (no specific buzzer registers).


// --- API Function Prototypes (as per API.json) ---

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Assuming frequency can be long
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

// Internal_EEPROM (STM32F401RC uses Flash for EEPROM emulation)
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time Triggered)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// WDT (Watchdog Timer)
void WDT_Init(void); // Declared again here as per API.json separate category

// I2S
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

#endif // MCAL_H