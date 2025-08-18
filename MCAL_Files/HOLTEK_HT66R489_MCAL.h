/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for HOLTEK_HT66R489
 *
 * This file contains the API declarations, type definitions, and register
 * definitions for the MCAL layer, designed for the HOLTEK_HT66R489 microcontroller.
 * It adheres to the provided API and Rules specifications.
 */

#ifndef MCAL_H
#define MCAL_H

// Core MCU header file - Placeholder as specific Holtek header not provided
#include "HOLTEK_HT66R489.h" // Placeholder for device-specific header
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Data type definitions as per Rules.json
#define tbyte uint8_t
#define tword uint16_t
#define tlong uint32_t

// --- Register Definitions (Memory-mapped Pointers) ---
// Note: All registers are 8-bit based on addresses.
#define REG_MP0                     ((volatile tbyte*)0x00)
#define REG_MP1L                    ((volatile tbyte*)0x01)
#define REG_MP1H                    ((volatile tbyte*)0x02)
#define REG_MP2L                    ((volatile tbyte*)0x03)
#define REG_MP2H                    ((volatile tbyte*)0x04)
#define REG_IAR0                    ((volatile tbyte*)0x05)
#define REG_IAR1                    ((volatile tbyte*)0x06)
#define REG_IAR2                    ((volatile tbyte*)0x07)
#define REG_GPIOA_DATA              ((volatile tbyte*)0x08)
#define REG_GPIOB_DATA              ((volatile tbyte*)0x09)
#define REG_GPIOC_DATA              ((volatile tbyte*)0x0A)
#define REG_GPIOD_DATA              ((volatile tbyte*)0x0B)
#define REG_GPIOA_PULLUP            ((volatile tbyte*)0x0C)
#define REG_GPIOB_PULLUP            ((volatile tbyte*)0x0D)
#define REG_GPIOC_PULLUP            ((volatile tbyte*)0x0E)
#define REG_GPIOD_PULLUP            ((volatile tbyte*)0x0F)
#define REG_TBLP                    ((volatile tbyte*)0x10)
#define REG_TBHP                    ((volatile tbyte*)0x11)
#define REG_TBLH                    ((volatile tbyte*)0x12)
#define REG_DPH                     ((volatile tbyte*)0x13)
#define REG_DPL                     ((volatile tbyte*)0x14)
#define REG_ACC                     ((volatile tbyte*)0x15)
#define REG_PCL                     ((volatile tbyte*)0x16)
#define REG_STATUS                  ((volatile tbyte*)0x17)
#define REG_INT_CR0                 ((volatile tbyte*)0x18)
#define REG_INT_CR1                 ((volatile tbyte*)0x19)
#define REG_INT_CR2                 ((volatile tbyte*)0x1A)
#define REG_INT_CR3                 ((volatile tbyte*)0x1B)
#define REG_INT_MFI0                ((volatile tbyte*)0x1C)
#define REG_INT_MFI1                ((volatile tbyte*)0x1D)
#define REG_INT_MFI2                ((volatile tbyte*)0x1E)
#define REG_INT_MFI3                ((volatile tbyte*)0x1F)
#define REG_ADC_CR0                 ((volatile tbyte*)0x20)
#define REG_ADC_CR1                 ((volatile tbyte*)0x21)
#define REG_ADC_CHER                ((volatile tbyte*)0x22)
#define REG_ADC_DR_L                ((volatile tbyte*)0x23)
#define REG_ADC_DR_H                ((volatile tbyte*)0x24)
#define REG_CTM_CNT_H               ((volatile tbyte*)0x25)
#define REG_CTM_CNT_L               ((volatile tbyte*)0x26)
#define REG_CTM_COMPA               ((volatile tbyte*)0x27)
#define REG_CTM_CR0                 ((volatile tbyte*)0x28)
#define REG_CTM_CR1                 ((volatile tbyte*)0x29)
#define REG_STM_CNT_H               ((volatile tbyte*)0x2A)
#define REG_STM_CNT_L               ((volatile tbyte*)0x2B)
#define REG_STM_COMPA               ((volatile tbyte*)0x2C)
#define REG_STM_CR0                 ((volatile tbyte*)0x2D)
#define REG_STM_CR1                 ((volatile tbyte*)0x2E)
#define REG_PTM0_CNT_H              ((volatile tbyte*)0x2F)
#define REG_PTM0_CNT_L              ((volatile tbyte*)0x30)
#define REG_PTM0_COMPA_H            ((volatile tbyte*)0x31)
#define REG_PTM0_COMPA_L            ((volatile tbyte*)0x32)
#define REG_PTM0_CR0                ((volatile tbyte*)0x33)
#define REG_PTM0_CR1                ((volatile tbyte*)0x34)
#define REG_PTM1_CNT_H              ((volatile tbyte*)0x35)
#define REG_PTM1_CNT_L              ((volatile tbyte*)0x36)
#define REG_PTM1_COMPA_H            ((volatile tbyte*)0x37)
#define REG_PTM1_COMPA_L            ((volatile tbyte*)0x38)
#define REG_PTM1_CR0                ((volatile tbyte*)0x39)
#define REG_PTM1_CR1                ((volatile tbyte*)0x3A)
#define REG_TM_PIN_CR               ((volatile tbyte*)0x3B)
#define REG_SPI_SR                  ((volatile tbyte*)0x3C)
#define REG_SPI_CR0                 ((volatile tbyte*)0x3D)
#define REG_SPI_CR1                 ((volatile tbyte*)0x3E)
#define REG_SPI_DR                  ((volatile tbyte*)0x3F)
#define REG_I2C_SR                  ((volatile tbyte*)0x40)
#define REG_I2C_CR                  ((volatile tbyte*)0x41)
#define REG_I2C_DR                  ((volatile tbyte*)0x42)
#define REG_I2C_TMR                 ((volatile tbyte*)0x43)
#define REG_UART_SR                 ((volatile tbyte*)0x44)
#define REG_UART_CR0                ((volatile tbyte*)0x45)
#define REG_UART_CR1                ((volatile tbyte*)0x46)
#define REG_UART_TX_DR              ((volatile tbyte*)0x47)
#define REG_UART_RX_DR              ((volatile tbyte*)0x48)
#define REG_UART_BRG_H              ((volatile tbyte*)0x49)
#define REG_UART_BRG_L              ((volatile tbyte*)0x4A)
#define REG_LCD_CR0                 ((volatile tbyte*)0x4B)
#define REG_LCD_CR1                 ((volatile tbyte*)0x4C)
#define REG_LCD_CST                 ((volatile tbyte*)0x4D)
#define REG_LCD_DB0                 ((volatile tbyte*)0x4E)
#define REG_LCD_DB1                 ((volatile tbyte*)0x4F)
#define REG_LCD_DB2                 ((volatile tbyte*)0x50)
#define REG_LCD_DB3                 ((volatile tbyte*)0x51)
#define REG_LCD_DB4                 ((volatile tbyte*)0x52)
#define REG_LCD_DB5                 ((volatile tbyte*)0x53)
#define REG_EEPROM_ADDR_L           ((volatile tbyte*)0x54)
#define REG_EEPROM_ADDR_H           ((volatile tbyte*)0x55)
#define REG_EEPROM_DATA_L           ((volatile tbyte*)0x56)
#define REG_EEPROM_DATA_H           ((volatile tbyte*)0x57)
#define REG_EEPROM_CR0              ((volatile tbyte*)0x58)
#define REG_EEPROM_CR1              ((volatile tbyte*)0x59)
#define REG_RCC_CLKSEL              ((volatile tbyte*)0x5A)
#define REG_RCC_CLKDIV              ((volatile tbyte*)0x5B)
#define REG_RCC_PWRCON              ((volatile tbyte*)0x5C)
#define REG_LVD_CR                  ((volatile tbyte*)0x5D)
#define REG_RCC_RSTF                ((volatile tbyte*)0x5E)
#define REG_WDT_CR                  ((volatile tbyte*)0x5F)
#define REG_GPIOA_TRANSISTOR        ((volatile tbyte*)0x60)
#define REG_GPIOB_TRANSISTOR        ((volatile tbyte*)0x61)
#define REG_GPIOC_TRANSISTOR        ((volatile tbyte*)0x62)
#define REG_GPIOD_TRANSISTOR        ((volatile tbyte*)0x63)
#define REG_GPIOA_SOURCE_CURRENT    ((volatile tbyte*)0x64)
#define REG_GPIOB_SOURCE_CURRENT    ((volatile tbyte*)0x65)
#define REG_GPIOC_SOURCE_CURRENT    ((volatile tbyte*)0x66)
#define REG_GPIOD_SOURCE_CURRENT    ((volatile tbyte*)0x67)

// --- Type Definitions for API Parameters ---

// MCU CONFIG Types
typedef enum {
    Vsource_3V = 0, // Maps to LVD threshold 2.0V
    Vsource_5V      // Maps to LVD threshold 3.5V
} t_sys_volt;

// LVD Types
typedef enum {
    Volt_0_5V = 0,
    Volt_1V,
    Volt_1_5V,
    Volt_2V, // Suitable for 3V system
    Volt_2_5V,
    Volt_3V,
    Volt_3_5V, // Suitable for 5V system
    Volt_4V,
    Volt_4_5V,
    Volt_5V
    // ... other levels as per specific LVD_CR bit field definitions (not provided)
} t_lvd_thrthresholdLevel;

typedef enum {
    LVD_CHANNEL_NONE = 0 // No specific channels mentioned in register JSON, only LVD_CR
} t_lvd_channel;

// UART Types
typedef enum {
    UART_CH0 = 0 // Only one UART module implicitly defined by registers
} t_uart_channel;

typedef enum {
    UART_BAUD_9600 = 0,
    UART_BAUD_19200,
    UART_BAUD_115200
    // Specific values for UART_BRG_H/L would be determined by MCU clock and desired baud rate.
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8_BITS = 0,
    UART_DATA_9_BITS
} t_uart_data_length;

typedef enum {
    UART_STOP_1_BIT = 0,
    UART_STOP_2_BITS
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// I2C Types
typedef enum {
    I2C_CH0 = 0 // Only one I2C module implicitly defined by registers
} t_i2c_channel;

typedef enum {
    I2C_CLK_FAST_MODE = 0 // Rules specify always fast mode
} t_i2c_clk_speed;

typedef enum {
    I2C_ADDR_7_BIT = 0,
    I2C_ADDR_10_BIT
} t_i2c_device_address;

typedef enum {
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_8_BITS = 0
} t_i2c_datalength;

// SPI Types
typedef enum {
    SPI_CH0 = 0 // Only one SPI module implicitly defined by registers
} t_spi_channel;

typedef enum {
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW = 0,
    SPI_CPOL_HIGH
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1_EDGE = 0,
    SPI_CPHA_2_EDGE
} t_spi_cpha;

typedef enum {
    SPI_DFF_8_BIT = 0,
    SPI_DFF_16_BIT
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt Types
typedef enum {
    EXT_INT_PB2 = 0, // INT_CRx assigned pin PB2
    EXT_INT_PB1,     // INT_CRx assigned pin PB1
    EXT_INT_PC6,     // INT_CRx assigned pin PC6
    EXT_INT_PC5,     // INT_CRx assigned pin PC5
    EXT_INT_PB0,     // INT_CRx assigned pin PB0
    EXT_INT_PC7      // INT_CRx assigned pin PC7
} t_external_int_channel;

typedef enum {
    EXT_INT_EDGE_RISING = 0,
    EXT_INT_EDGE_FALLING,
    EXT_INT_EDGE_BOTH // Not explicitly supported by INT_CRx, typical for some MCUs
} t_external_int_edge;

// GPIO Types
typedef enum {
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D
} t_port;

typedef enum {
    PIN0 = 0,
    PIN1,
    PIN2,
    PIN3,
    PIN4,
    PIN5,
    PIN6,
    PIN7
} t_pin;

typedef enum {
    INPUT = 0,
    OUTPUT
} t_direction;

// PWM Types
typedef enum {
    PWM_CTM = 0,   // CTM is assigned to PB2
    PWM_STM,       // STM is assigned to PB7
    PWM_PTM0,      // PTM0 is assigned to PB0, PB1
    PWM_PTM1       // PTM1 is assigned to PA5, PA6
} t_pwm_channel;

// ICU Types
typedef enum {
    ICU_CTM_PB2 = 0,  // CTM assigned to PB2
    ICU_STM_PB7,      // STM assigned to PB7
    ICU_PTM0_PB0,     // PTM0 assigned to PB0
    ICU_PTM0_PB1,     // PTM0 assigned to PB1
    ICU_PTM1_PA5,     // PTM1 assigned to PA5
    ICU_PTM1_PA6      // PTM1 assigned to PA6
} t_icu_channel;

typedef enum {
    ICU_PRESCALER_1 = 0,
    ICU_PRESCALER_2,
    ICU_PRESCALER_4,
    ICU_PRESCALER_8
    // Specific prescaler values for timers are usually found in CR0/CR1 registers
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer Types
typedef enum {
    TIMER_CTM = 0,
    TIMER_STM,
    TIMER_PTM0,
    TIMER_PTM1
} t_timer_channel;

// ADC Types
typedef enum {
    ADC_CH_PB1 = 0, // PB1 assigned to ADC
    ADC_CH_PB6,     // PB6 assigned to ADC
    ADC_CH_PB7,     // PB7 assigned to ADC
    ADC_CH_PC0,     // PC0 assigned to ADC
    ADC_CH_PC3,     // PC3 assigned to ADC
    ADC_CH_PC4,     // PC4 assigned to ADC
    ADC_CH_PC5,     // PC5 assigned to ADC
    ADC_CH_PC6      // PC6 assigned to ADC
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE_CONVERSION = 0,
    ADC_MODE_CONTINUOUS_CONVERSION
} t_adc_mode_t;

// TT (Time Triggered OS) Types
typedef enum {
    TICK_TIME_MS_1 = 1,  // 1ms tick
    TICK_TIME_MS_5 = 5,  // 5ms tick
    TICK_TIME_MS_10 = 10 // 10ms tick
    // More granular options can be added
} t_tick_time;


// --- API Declarations ---

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
void LVD_ClearFlag(t_lvd_channel lvd_channel);

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
void UART_ClearFlag(t_uart_channel uart_channel);

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
void I2C_ClearFlag(t_i2c_channel i2c_channel);

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
void SPI_ClearFlag(t_spi_channel spi_channel);

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel);

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Assuming tlong for frequency
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel);

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel);

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void);

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