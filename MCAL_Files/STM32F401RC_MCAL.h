#ifndef _MCAL_H_
#define _MCAL_H_

// Core MCU header file and standard C library includes
#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// Data type definitions as per Rules.json
#define tbyte   uint8_t
#define tword   uint16_t
#define tlong   uint32_t

// =============================================================================
// MCU CONFIG Types
// =============================================================================
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

// =============================================================================
// LVD Types (Low Voltage Detection)
// =============================================================================
typedef enum {
    Volt_0_5V = 0,
    Volt_1V,
    Volt_1_5V,
    Volt_2V,
    Volt_2_5V,
    Volt_2_6V,
    Volt_2_7V,
    Volt_2_8V,
    Volt_2_9V,
    Volt_3V,
    Volt_3_1V,
    Volt_3_2V,
    Volt_3_3V,
    Volt_3_4V,
    Volt_3_5V,
    Volt_3_6V,
    Volt_3_7V,
    Volt_3_8V,
    Volt_3_9V,
    Volt_4V,
    Volt_4_1V,
    Volt_4_2V,
    Volt_4_3V,
    Volt_4_4V,
    Volt_4_5V,
    Volt_4_6V,
    Volt_4_7V,
    Volt_4_8V,
    Volt_4_9V,
    Volt_5V,
    LVD_THRESHOLD_MAX_LEVEL // Placeholder, no direct channel in register_json
} t_lvd_thrthresholdLevel;

typedef enum {
    LVD_CHANNEL_DEFAULT = 0 // Placeholder, no direct channel concept in register_json for LVD
} t_lvd_channel;

// =============================================================================
// UART Types
// =============================================================================
typedef enum {
    UART_CH1 = 0,  // Corresponds to USART1
    UART_CH2,      // Corresponds to USART2
    UART_CH6       // Corresponds to USART6
} t_uart_channel;

typedef enum {
    UART_BAUD_9600 = 0,
    UART_BAUD_19200,
    UART_BAUD_38400,
    UART_BAUD_57600,
    UART_BAUD_115200
    // Add more common baud rates as needed
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8BIT = 0,
    UART_DATA_9BIT
} t_uart_data_length;

typedef enum {
    UART_STOP_1BIT = 0,
    UART_STOP_0_5BIT,
    UART_STOP_2BIT,
    UART_STOP_1_5BIT
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// =============================================================================
// I2C Types
// =============================================================================
typedef enum {
    I2C_CH1 = 0,
    I2C_CH2,
    I2C_CH3
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_STANDARD = 0, // 100 kHz
    I2C_CLK_SPEED_FAST          // 400 kHz (as per rule: always use fast mode)
} t_i2c_clk_speed;

typedef uint8_t t_i2c_device_address; // 7-bit or 10-bit address

typedef enum {
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_LEN_8BIT = 0,
    I2C_DATA_LEN_16BIT // Not directly used by I2C_DR but for general length concept
} t_i2c_datalength;

// =============================================================================
// SPI Types
// =============================================================================
typedef enum {
    SPI_CH1 = 0,
    SPI_CH2,
    SPI_CH3
} t_spi_channel;

typedef enum {
    SPI_MODE_SLAVE = 0,
    SPI_MODE_MASTER
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW = 0,  // Clock Polarity Low
    SPI_CPOL_HIGH      // Clock Polarity High
} t_spi_cpol;

typedef enum {
    SPI_CPHA_FIRST = 0, // Clock Phase 1st edge
    SPI_CPHA_SECOND     // Clock Phase 2nd edge
} t_spi_cpha;

typedef enum {
    SPI_DFF_8BIT = 0,
    SPI_DFF_16BIT
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// =============================================================================
// External Interrupt Types
// =============================================================================
typedef enum {
    EXTI_LINE0 = 0,
    EXTI_LINE1,
    EXTI_LINE2,
    EXTI_LINE3,
    EXTI_LINE4,
    EXTI_LINE5,
    EXTI_LINE6,
    EXTI_LINE7,
    EXTI_LINE8,
    EXTI_LINE9,
    EXTI_LINE10,
    EXTI_LINE11,
    EXTI_LINE12,
    EXTI_LINE13,
    EXTI_LINE14,
    EXTI_LINE15
} t_external_int_channel;

typedef enum {
    EXTI_EDGE_RISING = 0,
    EXTI_EDGE_FALLING,
    EXTI_EDGE_RISING_FALLING
} t_external_int_edge;

// =============================================================================
// GPIO Types
// =============================================================================
typedef enum {
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    PORT_H
} t_port;

typedef enum {
    PIN0 = 0,
    PIN1,
    PIN2,
    PIN3,
    PIN4,
    PIN5,
    PIN6,
    PIN7,
    PIN8,
    PIN9,
    PIN10,
    PIN11,
    PIN12,
    PIN13,
    PIN14,
    PIN15
} t_pin;

typedef enum {
    INPUT = 0,
    OUTPUT
} t_direction;

// =============================================================================
// PWM Types
// =============================================================================
// PWM Channels based on Timer and Channel combination
typedef enum {
    PWM_TIM1_CH1 = 0, // PA8, PE9
    PWM_TIM1_CH2,     // PA9, PE11
    PWM_TIM1_CH3,     // PA10, PE13
    PWM_TIM1_CH4,     // PA11, PE14
    PWM_TIM2_CH1,     // PA0, PA5, PA15
    PWM_TIM2_CH2,     // PA1, PB3
    PWM_TIM2_CH3,     // PA2, PB10
    PWM_TIM2_CH4,     // PA3, PB11
    PWM_TIM3_CH1,     // PA6, PB4, PC6
    PWM_TIM3_CH2,     // PA7, PB5, PC7
    PWM_TIM3_CH3,     // PB0, PC8
    PWM_TIM3_CH4,     // PB1, PC9
    PWM_TIM4_CH1,     // PB6, PD12
    PWM_TIM4_CH2,     // PB7, PD13
    PWM_TIM4_CH3,     // PB8, PD14
    PWM_TIM4_CH4,     // PB9, PD15
    PWM_TIM5_CH1,     // PA0
    PWM_TIM5_CH2,     // PA1
    PWM_TIM5_CH3,     // PA2
    PWM_TIM5_CH4,     // PA3
    PWM_TIM9_CH1,     // PA2, PE5
    PWM_TIM9_CH2,     // PA3, PE6
    PWM_TIM10_CH1,    // PA6, PB8
    PWM_TIM11_CH1,    // PA7, PB9
    PWM_CHANNEL_MAX
} t_pwm_channel;

// =============================================================================
// ICU Types (Input Capture Unit)
// =============================================================================
// ICU Channels based on Timer and Channel combination (same as PWM for flexibility)
typedef t_pwm_channel t_icu_channel; // Input capture often uses same timer channels as PWM

typedef enum {
    ICU_PRESCALER_DIV1 = 0,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8,
    // Add more as per specific timer capabilities
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH // Rising and Falling
} t_icu_edge;

// =============================================================================
// Timer Types
// =============================================================================
typedef enum {
    TIMER_TIM1 = 0,
    TIMER_TIM2,
    TIMER_TIM3,
    TIMER_TIM4,
    TIMER_TIM5,
    TIMER_TIM9,
    TIMER_TIM10,
    TIMER_TIM11,
    TIMER_MAX
} t_timer_channel;

// =============================================================================
// ADC Types (Analog to Digital Converter)
// =============================================================================
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
    // Internal channels like Vrefint, TempSensor are typically channel 16, 17, 18
    ADC_CHANNEL_VREFINT,
    ADC_CHANNEL_TEMPSENSOR,
    ADC_CHANNEL_VBAT,
    ADC_CHANNEL_MAX
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE_CONVERSION = 0,
    ADC_MODE_CONTINUOUS_CONVERSION,
    ADC_MODE_SCAN_CONVERSION
} t_adc_mode_t;

// =============================================================================
// TT Types (Time-Triggered OS)
// =============================================================================
typedef tword t_tick_time; // Time in ms for the tick

// =============================================================================
// API Function Prototypes
// =============================================================================

// MCU CONFIG APIs
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD APIs
void LVD_Init(t_lvd_thrthresholdLevel lvd_thresholdLevel); // Modified to take threshold level
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
void LVD_Enable(void);
void LVD_Disable(void);
void LVD_ClearFlag(t_lvd_channel lvd_channel); // t_lvd_channel is a placeholder

// UART APIs
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_Update(t_uart_channel uart_channel); // Placeholder for internal status update
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length); // Returns num bytes read
void UART_ClearFlag(t_uart_channel uart_channel);

// I2C APIs
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_Update(t_i2c_channel i2c_channel); // Placeholder for internal status update
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length); // Returns num bytes read
void I2C_ClearFlag(t_i2c_channel i2c_channel);

// SPI APIs
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Update(void); // Placeholder for internal status update
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
void SPI_send_string(t_spi_channel spi_channel, const char *str);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length); // Returns num bytes read
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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Assuming frequency can be long
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
void ADC_Update(void); // Placeholder for internal status update
tword ADC_Get(void);
void ADC_ClearFlag(void);

// Internal EEPROM APIs (Note: STM32F401RC uses Flash for data storage, not true EEPROM)
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT APIs (Time-Triggered OS)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif // _MCAL_H_