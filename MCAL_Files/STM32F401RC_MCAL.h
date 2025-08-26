/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the public API definitions and type declarations for the
 * Microcontroller Abstraction Layer (MCAL). It provides a high-level interface
 * to interact with the STM32F401RC microcontroller's peripherals.
 *
 * @note This file is generated based on provided register definitions, API specifications,
 *       and coding rules. Some bit field positions and inferred registers may be
 *       approximated based on typical STM32F4xx conventions.
 */

#ifndef MCAL_H_
#define MCAL_H_

// Core device header for STM32F401RC
#include "stm32f401xc.h"  // Placeholder for the specific device header file.
                         // Actual file might be "stm32f4xx.h" or "stm32f401xx.h"
                         // depending on the project's CMSIS setup.

// Standard C library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// #include <string.h> // Not strictly needed for MCAL, but might be for higher layers
// #include <stdio.h>  // Not strictly needed for MCAL
// #include <stdlib.h> // Not strictly needed for MCAL
// #include <math.h>   // Not strictly needed for MCAL

// Data type definitions as per coding rules
#define tbyte  uint8_t   ///< 8-bit unsigned integer type
#define tword  uint16_t  ///< 16-bit unsigned integer type
#define tlong  uint32_t  ///< 32-bit unsigned integer type

// =============================================================================
// MCU CONFIG related types
// =============================================================================
/**
 * @brief Enumeration for system voltage levels.
 */
typedef enum
{
    Vsource_3V = 0, ///< System voltage is 3V
    Vsource_5V      ///< System voltage is 5V
} t_sys_volt;

// =============================================================================
// LVD related types (Low Voltage Detection)
// =============================================================================
/**
 * @brief Enumeration for LVD threshold levels.
 * @note These values are illustrative and would map to specific bits in PVD (Programmable Voltage Detector)
 *       control registers on STM32F401RC, typically in PWR_CR.
 */
typedef enum
{
    Volt_0_5V = 0,  ///< LVD threshold level 0.5V
    Volt_1V,        ///< LVD threshold level 1V
    Volt_1_5V,      ///< LVD threshold level 1.5V
    Volt_2V,        ///< LVD threshold level 2V
    Volt_2_5V,      ///< LVD threshold level 2.5V
    Volt_2_6V,      ///< LVD threshold level 2.6V
    Volt_2_7V,      ///< LVD threshold level 2.7V
    Volt_2_8V,      ///< LVD threshold level 2.8V
    Volt_2_9V,      ///< LVD threshold level 2.9V
    Volt_3V,        ///< LVD threshold level 3V
    Volt_3_1V,      ///< LVD threshold level 3.1V
    Volt_3_2V,      ///< LVD threshold level 3.2V
    Volt_3_3V,      ///< LVD threshold level 3.3V
    Volt_3_4V,      ///< LVD threshold level 3.4V
    Volt_3_5V,      ///< LVD threshold level 3.5V
    Volt_3_6V,      ///< LVD threshold level 3.6V
    Volt_3_7V,      ///< LVD threshold level 3.7V
    Volt_3_8V,      ///< LVD threshold level 3.8V
    Volt_3_9V,      ///< LVD threshold level 3.9V
    Volt_4V,        ///< LVD threshold level 4V
    Volt_4_1V,      ///< LVD threshold level 4.1V
    Volt_4_2V,      ///< LVD threshold level 4.2V
    Volt_4_3V,      ///< LVD threshold level 4.3V
    Volt_4_4V,      ///< LVD threshold level 4.4V
    Volt_4_5V,      ///< LVD threshold level 4.5V
    Volt_4_6V,      ///< LVD threshold level 4.6V
    Volt_4_7V,      ///< LVD threshold level 4.7V
    Volt_4_8V,      ///< LVD threshold level 4.8V
    Volt_4_9V,      ///< LVD threshold level 4.9V
    Volt_5V         ///< LVD threshold level 5V
} t_lvd_thrthresholdLevel;

/**
 * @brief Enumeration for LVD channels.
 * @note On STM32F4, PVD is typically a single module, not multiple channels.
 *       This enum is for API compatibility, a single 'PVD_CHANNEL' is used.
 */
typedef enum
{
    PVD_CHANNEL_0 = 0 ///< Placeholder for PVD channel
} t_lvd_channel;

// =============================================================================
// UART related types
// =============================================================================
/**
 * @brief Enumeration for UART channels.
 */
typedef enum
{
    UART_CHANNEL_1 = 0, ///< USART1 Peripheral
    UART_CHANNEL_2,     ///< USART2 Peripheral
    UART_CHANNEL_6      ///< USART6 Peripheral
} t_uart_channel;

/**
 * @brief Enumeration for UART baud rates.
 * @note These are common values, actual division might vary based on clock source.
 */
typedef enum
{
    UART_BAUD_RATE_9600 = 0,  ///< 9600 baud
    UART_BAUD_RATE_19200,     ///< 19200 baud
    UART_BAUD_RATE_38400,     ///< 38400 baud
    UART_BAUD_RATE_57600,     ///< 57600 baud
    UART_BAUD_RATE_115200     ///< 115200 baud
} t_uart_baud_rate;

/**
 * @brief Enumeration for UART data length.
 */
typedef enum
{
    UART_DATA_LENGTH_8_BITS = 0, ///< 8 data bits
    UART_DATA_LENGTH_9_BITS      ///< 9 data bits
} t_uart_data_length;

/**
 * @brief Enumeration for UART stop bits.
 */
typedef enum
{
    UART_STOP_BIT_1 = 0, ///< 1 stop bit
    UART_STOP_BIT_0_5,   ///< 0.5 stop bits (only for certain modes, not always applicable)
    UART_STOP_BIT_2,     ///< 2 stop bits
    UART_STOP_BIT_1_5    ///< 1.5 stop bits (only for certain modes, not always applicable)
} t_uart_stop_bit;

/**
 * @brief Enumeration for UART parity.
 */
typedef enum
{
    UART_PARITY_NONE = 0, ///< No parity
    UART_PARITY_EVEN,     ///< Even parity
    UART_PARITY_ODD       ///< Odd parity
} t_uart_parity;

// =============================================================================
// I2C related types
// =============================================================================
/**
 * @brief Enumeration for I2C channels.
 */
typedef enum
{
    I2C_CHANNEL_1 = 0, ///< I2C1 Peripheral
    I2C_CHANNEL_2,     ///< I2C2 Peripheral
    I2C_CHANNEL_3      ///< I2C3 Peripheral
} t_i2c_channel;

/**
 * @brief Enumeration for I2C clock speed.
 */
typedef enum
{
    I2C_CLK_SPEED_STANDARD = 0, ///< Standard mode (up to 100 kHz)
    I2C_CLK_SPEED_FAST          ///< Fast mode (up to 400 kHz)
} t_i2c_clk_speed;

/**
 * @brief Enumeration for I2C device address mode.
 */
typedef enum
{
    I2C_ADDRESS_7_BIT = 0, ///< 7-bit addressing mode
    I2C_ADDRESS_10_BIT     ///< 10-bit addressing mode
} t_i2c_device_address; // This represents addressing mode, not the address itself.

/**
 * @brief Enumeration for I2C ACK control.
 */
typedef enum
{
    I2C_ACK_DISABLE = 0, ///< Acknowledge disabled
    I2C_ACK_ENABLE       ///< Acknowledge enabled
} t_i2c_ack;

/**
 * @brief Enumeration for I2C data length (for 10-bit addressing).
 */
typedef enum
{
    I2C_DATALENGTH_8_BIT = 0, ///< 8-bit data length
    I2C_DATALENGTH_16_BIT     ///< 16-bit data length
} t_i2c_datalength; // This might be more related to PEC or SMbus than general data transfer.

// =============================================================================
// SPI related types
// =============================================================================
/**
 * @brief Enumeration for SPI channels.
 */
typedef enum
{
    SPI_CHANNEL_1 = 0, ///< SPI1 Peripheral
    SPI_CHANNEL_2,     ///< SPI2 Peripheral
    SPI_CHANNEL_3      ///< SPI3 Peripheral
} t_spi_channel;

/**
 * @brief Enumeration for SPI mode (Master/Slave).
 */
typedef enum
{
    SPI_MODE_SLAVE = 0, ///< SPI Slave mode
    SPI_MODE_MASTER     ///< SPI Master mode
} t_spi_mode;

/**
 * @brief Enumeration for SPI Clock Polarity (CPOL).
 */
typedef enum
{
    SPI_CPOL_LOW = 0, ///< Clock to 0 when idle
    SPI_CPOL_HIGH     ///< Clock to 1 when idle
} t_spi_cpol;

/**
 * @brief Enumeration for SPI Clock Phase (CPHA).
 */
typedef enum
{
    SPI_CPHA_1_EDGE = 0, ///< First clock transition is first data capture edge
    SPI_CPHA_2_EDGE      ///< Second clock transition is first data capture edge
} t_spi_cpha;

/**
 * @brief Enumeration for SPI Data Frame Format (DFF).
 */
typedef enum
{
    SPI_DFF_8_BIT = 0, ///< 8-bit data frame format
    SPI_DFF_16_BIT     ///< 16-bit data frame format
} t_spi_dff;

/**
 * @brief Enumeration for SPI bit order (MSB/LSB first).
 */
typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0, ///< MSB transmitted first
    SPI_BIT_ORDER_LSB_FIRST      ///< LSB transmitted first
} t_spi_bit_order;

// =============================================================================
// External Interrupt related types
// =============================================================================
/**
 * @brief Enumeration for external interrupt channels (EXTI lines).
 * @note Corresponds to EXTI lines 0-15.
 */
typedef enum
{
    EXTI_LINE_0 = 0, ///< EXTI Line 0 (can be PA0, PB0, PC0, PD0, PE0, PH0)
    EXTI_LINE_1,     ///< EXTI Line 1 (can be PA1, PB1, PC1, PD1, PE1, PH1)
    EXTI_LINE_2,     ///< EXTI Line 2 (can be PA2, PB2, PC2, PD2, PE2)
    EXTI_LINE_3,     ///< EXTI Line 3 (can be PA3, PB3, PC3, PD3, PE3)
    EXTI_LINE_4,     ///< EXTI Line 4 (can be PA4, PB4, PC4, PD4, PE4)
    EXTI_LINE_5,     ///< EXTI Line 5 (can be PA5, PB5, PC5, PD5, PE5)
    EXTI_LINE_6,     ///< EXTI Line 6 (can be PA6, PB6, PC6, PD6, PE6)
    EXTI_LINE_7,     ///< EXTI Line 7 (can be PA7, PB7, PC7, PD7, PE7)
    EXTI_LINE_8,     ///< EXTI Line 8 (can be PA8, PB8, PC8, PD8, PE8)
    EXTI_LINE_9,     ///< EXTI Line 9 (can be PA9, PB9, PC9, PD9, PE9)
    EXTI_LINE_10,    ///< EXTI Line 10 (can be PA10, PB10, PC10, PD10, PE10)
    EXTI_LINE_11,    ///< EXTI Line 11 (can be PA11, PB11, PC11, PD11, PE11)
    EXTI_LINE_12,    ///< EXTI Line 12 (can be PA12, PB12, PC12, PD12, PE12)
    EXTI_LINE_13,    ///< EXTI Line 13 (can be PA13, PB13, PC13, PD13, PE13)
    EXTI_LINE_14,    ///< EXTI Line 14 (can be PA14, PB14, PC14, PD14, PE14)
    EXTI_LINE_15     ///< EXTI Line 15 (can be PA15, PB15, PC15, PD15, PE15)
} t_external_int_channel;

/**
 * @brief Enumeration for external interrupt trigger edges.
 */
typedef enum
{
    EXTI_EDGE_RISING = 0,      ///< Interrupt on rising edge
    EXTI_EDGE_FALLING,         ///< Interrupt on falling edge
    EXTI_EDGE_RISING_FALLING   ///< Interrupt on both rising and falling edges
} t_external_int_edge;

// =============================================================================
// GPIO related types
// =============================================================================
/**
 * @brief Enumeration for GPIO ports.
 */
typedef enum
{
    GPIO_PORT_A = 0, ///< GPIO Port A
    GPIO_PORT_B,     ///< GPIO Port B
    GPIO_PORT_C,     ///< GPIO Port C
    GPIO_PORT_D,     ///< GPIO Port D
    GPIO_PORT_E,     ///< GPIO Port E
    GPIO_PORT_H      ///< GPIO Port H (limited pins)
} t_port;

/**
 * @brief Enumeration for GPIO pins (0-15).
 */
typedef enum
{
    GPIO_PIN_0 = 0,  ///< Pin 0
    GPIO_PIN_1,      ///< Pin 1
    GPIO_PIN_2,      ///< Pin 2
    GPIO_PIN_3,      ///< Pin 3
    GPIO_PIN_4,      ///< Pin 4
    GPIO_PIN_5,      ///< Pin 5
    GPIO_PIN_6,      ///< Pin 6
    GPIO_PIN_7,      ///< Pin 7
    GPIO_PIN_8,      ///< Pin 8
    GPIO_PIN_9,      ///< Pin 9
    GPIO_PIN_10,     ///< Pin 10
    GPIO_PIN_11,     ///< Pin 11
    GPIO_PIN_12,     ///< Pin 12
    GPIO_PIN_13,     ///< Pin 13
    GPIO_PIN_14,     ///< Pin 14
    GPIO_PIN_15      ///< Pin 15
} t_pin;

/**
 * @brief Enumeration for GPIO pin direction.
 */
typedef enum
{
    GPIO_DIRECTION_INPUT = 0,  ///< Pin is configured as input
    GPIO_DIRECTION_OUTPUT      ///< Pin is configured as output
} t_direction;

/**
 * @brief Enumeration for GPIO pin values (LOW/HIGH).
 */
typedef enum
{
    GPIO_LOW = 0,    ///< Pin output is low (0)
    GPIO_HIGH        ///< Pin output is high (1)
} t_gpio_value;

// =============================================================================
// PWM related types
// =============================================================================
/**
 * @brief Enumeration for PWM channels.
 * @note Pin assignments are comments as per rule.
 */
typedef enum
{
    PWM_CHANNEL_TIM1_CH1 = 0,  ///< TIM1 Channel 1 (PA8, PE9)
    PWM_CHANNEL_TIM1_CH2,      ///< TIM1 Channel 2 (PA9, PE11)
    PWM_CHANNEL_TIM1_CH3,      ///< TIM1 Channel 3 (PA10, PE13)
    PWM_CHANNEL_TIM1_CH4,      ///< TIM1 Channel 4 (PA11, PE14)
    PWM_CHANNEL_TIM2_CH1,      ///< TIM2 Channel 1 (PA0, PA5, PA15)
    PWM_CHANNEL_TIM2_CH2,      ///< TIM2 Channel 2 (PA1, PB3)
    PWM_CHANNEL_TIM2_CH3,      ///< TIM2 Channel 3 (PA2, PB10)
    PWM_CHANNEL_TIM2_CH4,      ///< TIM2 Channel 4 (PA3, PB11)
    PWM_CHANNEL_TIM3_CH1,      ///< TIM3 Channel 1 (PA6, PB4, PC6)
    PWM_CHANNEL_TIM3_CH2,      ///< TIM3 Channel 2 (PA7, PB5, PC7)
    PWM_CHANNEL_TIM3_CH3,      ///< TIM3 Channel 3 (PB0, PC8)
    PWM_CHANNEL_TIM3_CH4,      ///< TIM3 Channel 4 (PB1, PC9)
    PWM_CHANNEL_TIM4_CH1,      ///< TIM4 Channel 1 (PB6, PD12)
    PWM_CHANNEL_TIM4_CH2,      ///< TIM4 Channel 2 (PB7, PD13)
    PWM_CHANNEL_TIM4_CH3,      ///< TIM4 Channel 3 (PB8, PD14)
    PWM_CHANNEL_TIM4_CH4,      ///< TIM4 Channel 4 (PB9, PD15)
    PWM_CHANNEL_TIM5_CH1,      ///< TIM5 Channel 1 (PA0)
    PWM_CHANNEL_TIM5_CH2,      ///< TIM5 Channel 2 (PA1)
    PWM_CHANNEL_TIM5_CH3,      ///< TIM5 Channel 3 (PA2)
    PWM_CHANNEL_TIM5_CH4,      ///< TIM5 Channel 4 (PA3)
    PWM_CHANNEL_TIM9_CH1,      ///< TIM9 Channel 1 (PA2, PE5)
    PWM_CHANNEL_TIM9_CH2,      ///< TIM9 Channel 2 (PA3, PE6)
    PWM_CHANNEL_TIM10_CH1,     ///< TIM10 Channel 1 (PA6, PB8)
    PWM_CHANNEL_TIM11_CH1      ///< TIM11 Channel 1 (PA7, PB9)
} t_pwm_channel;


// =============================================================================
// ICU related types (Input Capture Unit)
// =============================================================================
/**
 * @brief Enumeration for ICU channels.
 * @note ICU often uses Timer Capture Compare channels. These map to the same
 *       channels as PWM, but are configured for input capture.
 *       Pin assignments are comments as per rule.
 */
typedef enum
{
    ICU_CHANNEL_TIM1_CH1 = 0,  ///< TIM1 Channel 1 (PA8, PE9)
    ICU_CHANNEL_TIM1_CH2,      ///< TIM1 Channel 2 (PA9, PE11)
    ICU_CHANNEL_TIM1_CH3,      ///< TIM1 Channel 3 (PA10, PE13)
    ICU_CHANNEL_TIM1_CH4,      ///< TIM1 Channel 4 (PA11, PE14)
    ICU_CHANNEL_TIM2_CH1,      ///< TIM2 Channel 1 (PA0, PA5, PA15)
    ICU_CHANNEL_TIM2_CH2,      ///< TIM2 Channel 2 (PA1, PB3)
    ICU_CHANNEL_TIM2_CH3,      ///< TIM2 Channel 3 (PA2, PB10)
    ICU_CHANNEL_TIM2_CH4,      ///< TIM2 Channel 4 (PA3, PB11)
    ICU_CHANNEL_TIM3_CH1,      ///< TIM3 Channel 1 (PA6, PB4, PC6)
    ICU_CHANNEL_TIM3_CH2,      ///< TIM3 Channel 2 (PA7, PB5, PC7)
    ICU_CHANNEL_TIM3_CH3,      ///< TIM3 Channel 3 (PB0, PC8)
    ICU_CHANNEL_TIM3_CH4,      ///< TIM3 Channel 4 (PB1, PC9)
    ICU_CHANNEL_TIM4_CH1,      ///< TIM4 Channel 1 (PB6, PD12)
    ICU_CHANNEL_TIM4_CH2,      ///< TIM4 Channel 2 (PB7, PD13)
    ICU_CHANNEL_TIM4_CH3,      ///< TIM4 Channel 3 (PB8, PD14)
    ICU_CHANNEL_TIM4_CH4,      ///< TIM4 Channel 4 (PB9, PD15)
    ICU_CHANNEL_TIM5_CH1,      ///< TIM5 Channel 1 (PA0)
    ICU_CHANNEL_TIM5_CH2,      ///< TIM5 Channel 2 (PA1)
    ICU_CHANNEL_TIM5_CH3,      ///< TIM5 Channel 3 (PA2)
    ICU_CHANNEL_TIM5_CH4,      ///< TIM5 Channel 4 (PA3)
    ICU_CHANNEL_TIM9_CH1,      ///< TIM9 Channel 1 (PA2, PE5)
    ICU_CHANNEL_TIM9_CH2,      ///< TIM9 Channel 2 (PA3, PE6)
    ICU_CHANNEL_TIM10_CH1,     ///< TIM10 Channel 1 (PA6, PB8)
    ICU_CHANNEL_TIM11_CH1      ///< TIM11 Channel 1 (PA7, PB9)
} t_icu_channel;

/**
 * @brief Enumeration for ICU prescaler settings.
 */
typedef enum
{
    ICU_PRESCALER_DIV1 = 0, ///< No prescaling
    ICU_PRESCALER_DIV2,     ///< Capture every 2 events
    ICU_PRESCALER_DIV4,     ///< Capture every 4 events
    ICU_PRESCALER_DIV8      ///< Capture every 8 events
} t_icu_prescaller;

/**
 * @brief Enumeration for ICU edge detection.
 * @note Corresponds to the trigger edge for input capture.
 */
typedef enum
{
    ICU_EDGE_RISING = 0, ///< Capture on rising edge
    ICU_EDGE_FALLING,    ///< Capture on falling edge
    ICU_EDGE_BOTH        ///< Capture on both edges (not all timers support this directly)
} t_icu_edge;

// =============================================================================
// Timer related types
// =============================================================================
/**
 * @brief Enumeration for Timer channels.
 * @note These refer to the base timers, not specific capture/compare channels.
 */
typedef enum
{
    TIMER_CHANNEL_1 = 0, ///< TIM1 Peripheral
    TIMER_CHANNEL_2,     ///< TIM2 Peripheral
    TIMER_CHANNEL_3,     ///< TIM3 Peripheral
    TIMER_CHANNEL_4,     ///< TIM4 Peripheral
    TIMER_CHANNEL_5,     ///< TIM5 Peripheral
    TIMER_CHANNEL_9,     ///< TIM9 Peripheral
    TIMER_CHANNEL_10,    ///< TIM10 Peripheral
    TIMER_CHANNEL_11     ///< TIM11 Peripheral
} t_timer_channel;

// =============================================================================
// ADC related types
// =============================================================================
/**
 * @brief Enumeration for ADC channels.
 * @note STM32F401RC has 16 external channels (0-15) plus internal channels.
 */
typedef enum
{
    ADC_CHANNEL_0 = 0,  ///< ADC Channel 0 (PA0)
    ADC_CHANNEL_1,      ///< ADC Channel 1 (PA1)
    ADC_CHANNEL_2,      ///< ADC Channel 2 (PA2)
    ADC_CHANNEL_3,      ///< ADC Channel 3 (PA3)
    ADC_CHANNEL_4,      ///< ADC Channel 4 (PA4)
    ADC_CHANNEL_5,      ///< ADC Channel 5 (PA5)
    ADC_CHANNEL_6,      ///< ADC Channel 6 (PA6)
    ADC_CHANNEL_7,      ///< ADC Channel 7 (PA7)
    ADC_CHANNEL_8,      ///< ADC Channel 8 (PB0)
    ADC_CHANNEL_9,      ///< ADC Channel 9 (PB1)
    ADC_CHANNEL_10,     ///< ADC Channel 10 (PC0)
    ADC_CHANNEL_11,     ///< ADC Channel 11 (PC1)
    ADC_CHANNEL_12,     ///< ADC Channel 12 (PC2)
    ADC_CHANNEL_13,     ///< ADC Channel 13 (PC3)
    ADC_CHANNEL_14,     ///< ADC Channel 14 (PC4)
    ADC_CHANNEL_15,     ///< ADC Channel 15 (PC5)
    // Internal channels typically 16, 17, 18
    ADC_CHANNEL_TEMP_SENSOR, ///< Internal Temperature Sensor (Channel 16)
    ADC_CHANNEL_VREF_INT,    ///< Internal VREFINT (Channel 17)
    ADC_CHANNEL_VBAT         ///< Internal VBAT (Channel 18)
} t_adc_channel;

/**
 * @brief Enumeration for ADC operating modes.
 */
typedef enum
{
    ADC_MODE_SINGLE_CONVERSION = 0, ///< Single conversion mode
    ADC_MODE_CONTINUOUS_CONVERSION, ///< Continuous conversion mode
    ADC_MODE_SCAN_CONVERSION        ///< Scan conversion mode (for multiple channels)
} t_adc_mode_t;

// =============================================================================
// Internal EEPROM related types
// @note STM32F401RC does not have a dedicated internal EEPROM. Flash memory
//       is used for emulation. This API would interface with Flash routines.
// =============================================================================
// No specific enums needed for Internal EEPROM beyond tbyte for address/data.

// =============================================================================
// TT (Time Triggered OS) related types
// =============================================================================
/**
 * @brief Enumeration for TT tick time.
 */
typedef enum
{
    TICK_TIME_1MS = 0, ///< 1 millisecond tick time
    TICK_TIME_10MS,    ///< 10 millisecond tick time
    TICK_TIME_100MS,   ///< 100 millisecond tick time
    TICK_TIME_1S       ///< 1 second tick time
} t_tick_time;

// =============================================================================
// MCAL API Function Prototypes
// =============================================================================

// MCU CONFIG functions
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD functions
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel); // Note: Original API signature had typo 'lvd_thrthresholdLevel'
void LVD_Enable(void);
void LVD_Disable(void);
void LVD_ClearFlag(t_lvd_channel lvd_channel);

// UART functions
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

// I2C functions
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

// SPI functions
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

// External Interrupt functions
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel);

// GPIO functions
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);

// PWM functions
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);

// ICU functions
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
void ICU_Updatefrequency(t_icu_channel icu_channel);
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Assuming frequency is a long value
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel);

// Timer functions
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel);

// ADC functions
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void);

// Internal_EEPROM functions
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT functions
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);


#endif /* MCAL_H_ */