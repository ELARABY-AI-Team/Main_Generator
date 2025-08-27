/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) header file for STM32F401RC.
 *
 * This file contains the API function prototypes and type definitions for the
 * MCAL layer, providing an abstract interface to the microcontroller's
 * peripherals. It adheres to MISRA C and CERT-C coding standards where applicable.
 */

#ifndef MCAL_H
#define MCAL_H

/*=============================================================================
 * INCLUDES
 *===========================================================================*/

#include "stm32f401xc.h"  // Core device header for STM32F401RC
#include <stdint.h>     // Standard integer types
#include <stdbool.h>    // Standard boolean type
#include <stddef.h>     // Standard definitions (e.g., size_t)

/*=============================================================================
 * GLOBAL MACROS AND TYPE DEFINITIONS
 *===========================================================================*/

// Data type definitions as per Rules.json
#define Unit_8 uint8_t
#define unit_16 uint16_t
#define unit_32 uint32_t

typedef uint8_t tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/**
 * @brief System voltage selection for MCU configuration.
 */
typedef enum {
    Vsource_3V = 0, // Placeholder, specific voltage levels depend on PVD configuration
    Vsource_5V      // Placeholder
} t_sys_volt;

/**
 * @brief Low Voltage Detection (LVD) threshold levels.
 *        Mapped to Power Voltage Detector (PVD) levels in STM32F401RC.
 *        Actual voltage levels for PLS[2:0] bits in PWR_CR are typically:
 *        000: 2.2V, 001: 2.3V, 010: 2.4V, 011: 2.5V, 100: 2.6V, 101: 2.7V, 110: 2.8V, 111: 2.9V.
 *        The enum values are symbolic and will be mapped to the closest PVD level.
 */
typedef enum {
    Volt_0_5V = 0, // Maps to PVD Level 0 (2.2V on STM32F401RC)
    Volt_1V,       // Maps to PVD Level 1 (2.3V on STM32F401RC)
    Volt_1_5V,     // Maps to PVD Level 2 (2.4V on STM32F401RC)
    Volt_2V,       // Maps to PVD Level 3 (2.5V on STM32F401RC)
    Volt_2_5V,     // Maps to PVD Level 4 (2.6V on STM32F401RC)
    Volt_3V,       // Maps to PVD Level 5 (2.7V on STM32F401RC)
    Volt_3_5V,     // Maps to PVD Level 6 (2.8V on STM32F401RC)
    Volt_4V,       // Maps to PVD Level 7 (2.9V on STM32F401RC)
    Volt_4_5V = 7, // Re-use PVD Level 7 as max available, as there are only 8 levels
    Volt_5V = 7    // Re-use PVD Level 7 as max available
} t_lvd_thrthresholdLevel;

/**
 * @brief LVD channel definition.
 *        For STM32, PVD is a global feature, so no specific channels.
 */
typedef enum {
    LVD_CHANNEL_NONE = 0 // STM32 PVD is global, not channel-specific
} t_lvd_channel;

/**
 * @brief UART peripheral channels.
 */
typedef enum {
    UART_CHANNEL_USART1 = 0,
    UART_CHANNEL_USART2,
    UART_CHANNEL_USART6
} t_uart_channel;

/**
 * @brief UART baud rate settings.
 *        Actual register values are calculated based on the peripheral clock.
 */
typedef enum {
    UART_BAUD_RATE_9600 = 0,
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_115200
} t_uart_baud_rate;

/**
 * @brief UART data length settings.
 */
typedef enum {
    UART_DATA_LENGTH_8B = 0, // 8-bit word length
    UART_DATA_LENGTH_9B      // 9-bit word length
} t_uart_data_length;

/**
 * @brief UART stop bit settings.
 */
typedef enum {
    UART_STOP_BIT_1 = 0, // 1 stop bit
    UART_STOP_BIT_0_5,   // 0.5 stop bits (for synchronous master)
    UART_STOP_BIT_1_5,   // 1.5 stop bits (for synchronous master)
    UART_STOP_BIT_2      // 2 stop bits
} t_uart_stop_bit;

/**
 * @brief UART parity settings.
 */
typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

/**
 * @brief I2C peripheral channels.
 */
typedef enum {
    I2C_CHANNEL_I2C1 = 0,
    I2C_CHANNEL_I2C2,
    I2C_CHANNEL_I2C3
} t_i2c_channel;

/**
 * @brief I2C clock speed settings.
 */
typedef enum {
    I2C_CLK_SPEED_STANDARD = 0, // 100 kHz
    I2C_CLK_SPEED_FAST          // 400 kHz (Rule: Always use fast mode)
} t_i2c_clk_speed;

/**
 * @brief I2C device address type.
 *        Typically 7-bit or 10-bit addressing.
 */
typedef tbyte t_i2c_device_address;

/**
 * @brief I2C ACK control.
 */
typedef enum {
    I2C_ACK_DISABLE = 0,
    I2C_ACK_ENABLE
} t_i2c_ack;

/**
 * @brief I2C data length setting.
 *        I2C transfers are typically 8-bit.
 */
typedef enum {
    I2C_DATA_LENGTH_8BIT = 0 // Standard 8-bit I2C data transfer
} t_i2c_datalength;

/**
 * @brief SPI peripheral channels.
 */
typedef enum {
    SPI_CHANNEL_SPI1 = 0,
    SPI_CHANNEL_SPI2,
    SPI_CHANNEL_SPI3
} t_spi_channel;

/**
 * @brief SPI mode (Master/Slave).
 */
typedef enum {
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE
} t_spi_mode;

/**
 * @brief SPI Clock Polarity (CPOL).
 */
typedef enum {
    SPI_CPOL_LOW = 0,  // Clock is low when idle
    SPI_CPOL_HIGH      // Clock is high when idle
} t_spi_cpol;

/**
 * @brief SPI Clock Phase (CPHA).
 */
typedef enum {
    SPI_CPHA_1EDGE = 0, // Data sampled on the first clock transition
    SPI_CPHA_2EDGE      // Data sampled on the second clock transition
} t_spi_cpha;

/**
 * @brief SPI Data Frame Format (DFF).
 */
typedef enum {
    SPI_DFF_8BIT = 0,   // 8-bit data frame format
    SPI_DFF_16BIT       // 16-bit data frame format
} t_spi_dff;

/**
 * @brief SPI Bit Order (MSB/LSB first).
 */
typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0, // Most Significant Bit first
    SPI_BIT_ORDER_LSB_FIRST      // Least Significant Bit first
} t_spi_bit_order;

/**
 * @brief External Interrupt (EXTI) lines.
 *        These map to specific pin numbers (0-15) across different GPIO ports.
 */
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

/**
 * @brief External Interrupt (EXTI) trigger edge.
 */
typedef enum {
    EXTI_EDGE_RISING = 0,
    EXTI_EDGE_FALLING,
    EXTI_EDGE_BOTH
} t_external_int_edge;

/**
 * @brief GPIO peripheral ports.
 */
typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_H
} t_port;

/**
 * @brief GPIO pin numbers within a port.
 */
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

/**
 * @brief GPIO pin direction (Input/Output).
 */
typedef enum {
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT
} t_direction;

/**
 * @brief PWM (Pulse Width Modulation) channels.
 *        Each channel maps to a specific Timer (TIMx) and Capture/Compare (CHy) unit,
 *        along with its associated GPIO pins.
 */
typedef enum {
    // TIM1 Channels (Advanced-control timer)
    PWM_CHANNEL_TIM1_CH1,  // PA8, PE9
    PWM_CHANNEL_TIM1_CH2,  // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,  // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,  // PA11, PE14
    // TIM2 Channels (General-purpose timer)
    PWM_CHANNEL_TIM2_CH1,  // PA0, PA5, PA15
    PWM_CHANNEL_TIM2_CH2,  // PA1, PB3
    PWM_CHANNEL_TIM2_CH3,  // PA2, PB10
    PWM_CHANNEL_TIM2_CH4,  // PA3, PB11
    // TIM3 Channels (General-purpose timer)
    PWM_CHANNEL_TIM3_CH1,  // PA6, PB4, PC6
    PWM_CHANNEL_TIM3_CH2,  // PA7, PB5, PC7
    PWM_CHANNEL_TIM3_CH3,  // PB0, PC8
    PWM_CHANNEL_TIM3_CH4,  // PB1, PC9
    // TIM4 Channels (General-purpose timer)
    PWM_CHANNEL_TIM4_CH1,  // PB6, PD12
    PWM_CHANNEL_TIM4_CH2,  // PB7, PD13
    PWM_CHANNEL_TIM4_CH3,  // PB8, PD14
    PWM_CHANNEL_TIM4_CH4,  // PB9, PD15
    // TIM5 Channels (General-purpose timer)
    PWM_CHANNEL_TIM5_CH1,  // PA0
    PWM_CHANNEL_TIM5_CH2,  // PA1
    PWM_CHANNEL_TIM5_CH3,  // PA2
    PWM_CHANNEL_TIM5_CH4,  // PA3
    // TIM9 Channels (General-purpose timer, 2 channels)
    PWM_CHANNEL_TIM9_CH1,  // PA2, PE5
    PWM_CHANNEL_TIM9_CH2,  // PA3, PE6
    // TIM10 Channels (General-purpose timer, 1 channel)
    PWM_CHANNEL_TIM10_CH1, // PA6, PB8
    // TIM11 Channels (General-purpose timer, 1 channel)
    PWM_CHANNEL_TIM11_CH1  // PA7, PB9
} t_pwm_channel;

/**
 * @brief ICU (Input Capture Unit) channels.
 *        These typically share the same hardware as PWM channels for timers.
 */
typedef enum {
    ICU_CHANNEL_TIM1_CH1 = PWM_CHANNEL_TIM1_CH1,
    ICU_CHANNEL_TIM1_CH2 = PWM_CHANNEL_TIM1_CH2,
    ICU_CHANNEL_TIM1_CH3 = PWM_CHANNEL_TIM1_CH3,
    ICU_CHANNEL_TIM1_CH4 = PWM_CHANNEL_TIM1_CH4,
    ICU_CHANNEL_TIM2_CH1 = PWM_CHANNEL_TIM2_CH1,
    ICU_CHANNEL_TIM2_CH2 = PWM_CHANNEL_TIM2_CH2,
    ICU_CHANNEL_TIM2_CH3 = PWM_CHANNEL_TIM2_CH3,
    ICU_CHANNEL_TIM2_CH4 = PWM_CHANNEL_TIM2_CH4,
    ICU_CHANNEL_TIM3_CH1 = PWM_CHANNEL_TIM3_CH1,
    ICU_CHANNEL_TIM3_CH2 = PWM_CHANNEL_TIM3_CH2,
    ICU_CHANNEL_TIM3_CH3 = PWM_CHANNEL_TIM3_CH3,
    ICU_CHANNEL_TIM3_CH4 = PWM_CHANNEL_TIM3_CH4,
    ICU_CHANNEL_TIM4_CH1 = PWM_CHANNEL_TIM4_CH1,
    ICU_CHANNEL_TIM4_CH2 = PWM_CHANNEL_TIM4_CH2,
    ICU_CHANNEL_TIM4_CH3 = PWM_CHANNEL_TIM4_CH3,
    ICU_CHANNEL_TIM4_CH4 = PWM_CHANNEL_TIM4_CH4,
    ICU_CHANNEL_TIM5_CH1 = PWM_CHANNEL_TIM5_CH1,
    ICU_CHANNEL_TIM5_CH2 = PWM_CHANNEL_TIM5_CH2,
    ICU_CHANNEL_TIM5_CH3 = PWM_CHANNEL_TIM5_CH3,
    ICU_CHANNEL_TIM5_CH4 = PWM_CHANNEL_TIM5_CH4,
    ICU_CHANNEL_TIM9_CH1 = PWM_CHANNEL_TIM9_CH1,
    ICU_CHANNEL_TIM9_CH2 = PWM_CHANNEL_TIM9_CH2,
    ICU_CHANNEL_TIM10_CH1 = PWM_CHANNEL_TIM10_CH1,
    ICU_CHANNEL_TIM11_CH1 = PWM_CHANNEL_TIM11_CH1
} t_icu_channel;

/**
 * @brief ICU prescaler settings.
 */
typedef enum {
    ICU_PRESCALER_1 = 0, // No division
    ICU_PRESCALER_2,     // Divide by 2
    ICU_PRESCALER_4,     // Divide by 4
    ICU_PRESCALER_8      // Divide by 8
} t_icu_prescaller;

/**
 * @brief ICU edge detection settings.
 */
typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH_EDGES
} t_icu_edge;

/**
 * @brief Generic Timer peripheral channels.
 */
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

/**
 * @brief ADC (Analog-to-Digital Converter) channels.
 *        Includes external analog pins and internal sensors.
 */
typedef enum {
    ADC_CHANNEL_0 = 0,  // Associated with PA0/PC0 (depends on ADC instance)
    ADC_CHANNEL_1,      // Associated with PA1/PC1
    ADC_CHANNEL_2,      // Associated with PA2/PC2
    ADC_CHANNEL_3,      // Associated with PA3/PC3
    ADC_CHANNEL_4,      // Associated with PA4/PC4
    ADC_CHANNEL_5,      // Associated with PA5/PC5
    ADC_CHANNEL_6,      // Associated with PA6
    ADC_CHANNEL_7,      // Associated with PA7
    ADC_CHANNEL_8,      // Associated with PB0
    ADC_CHANNEL_9,      // Associated with PB1
    ADC_CHANNEL_10,     // Associated with PC0
    ADC_CHANNEL_11,     // Associated with PC1
    ADC_CHANNEL_12,     // Associated with PC2
    ADC_CHANNEL_13,     // Associated with PC3
    ADC_CHANNEL_14,     // Associated with PC4
    ADC_CHANNEL_15,     // Associated with PC5
    ADC_CHANNEL_TEMPSENSOR, // Internal temperature sensor
    ADC_CHANNEL_VREFINT,    // Internal voltage reference
    ADC_CHANNEL_VBAT        // Vbat battery voltage
} t_adc_channel;

/**
 * @brief ADC operation modes.
 */
typedef enum {
    ADC_MODE_SINGLE = 0,    // Single conversion mode
    ADC_MODE_CONTINUOUS     // Continuous conversion mode
} t_adc_mode_t;

/**
 * @brief Tick time configurations for Time-Triggered (TT) OS.
 */
typedef enum {
    TT_TICK_TIME_1MS = 1,
    TT_TICK_TIME_10MS = 10,
    TT_TICK_TIME_100MS = 100
} t_tick_time;

/*=============================================================================
 * API FUNCTION PROTOTYPES
 *===========================================================================*/

/* --- MCU CONFIG APIs --- */

/**
 * @brief Initializes the basic MCU configuration, including GPIO, WDT, and LVR.
 *        Sets all GPIOs to input, disables peripherals, enables WDT, and
 *        configures Low Voltage Reset based on the system voltage.
 *
 * @param volt The system voltage (Vsource_3V or Vsource_5V) to configure LVR.
 */
void MCU_Config_Init(t_sys_volt volt);

/**
 * @brief Resets/refreshes the Watchdog Timer (WDT) to prevent a system reset.
 *        This function should be called periodically by the application.
 */
void WDT_Reset(void);

/**
 * @brief Puts the MCU into a low-power sleep mode.
 *        Execution stops, and most peripherals are halted (except OS timer if configured).
 */
void Go_to_sleep_mode(void);

/**
 * @brief Enables global interrupts, allowing the MCU to respond to interrupt requests.
 */
void Global_interrupt_Enable(void);

/**
 * @brief Disables global interrupts, preventing the MCU from responding to
 *        interrupt requests until re-enabled.
 */
void Global_interrupt_Disable(void);

/* --- LVD APIs --- */

/**
 * @brief Initializes the Low Voltage Detection (LVD) feature.
 *        On STM32, this typically involves enabling the Power Voltage Detector (PVD).
 */
void LVD_Init(void);

/**
 * @brief Configures the LVD threshold level.
 *
 * @param lvd_thresholdLevel The desired voltage threshold level.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);

/**
 * @brief Enables the Low Voltage Detection (LVD) feature.
 */
void LVD_Enable(void);

/**
 * @brief Disables the Low Voltage Detection (LVD) feature.
 */
void LVD_Disable(void);

/**
 * @brief Clears the LVD flag.
 *
 * @param lvd_channel The LVD channel (placeholder, as STM32 PVD is global).
 */
void LVD_ClearFlag(t_lvd_channel lvd_channel);

/* --- UART APIs --- */

/**
 * @brief Initializes the specified UART channel with given parameters.
 *
 * @param uart_channel The UART peripheral to initialize (USART1, USART2, USART6).
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data word length (8-bit or 9-bit).
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting (None, Even, Odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);

/**
 * @brief Enables the specified UART peripheral.
 *        This includes enabling its clock and the peripheral itself.
 *
 * @param uart_channel The UART peripheral to enable.
 */
void UART_Enable(t_uart_channel uart_channel);

/**
 * @brief Disables the specified UART peripheral.
 *
 * @param uart_channel The UART peripheral to disable.
 */
void UART_Disable(t_uart_channel uart_channel);

/**
 * @brief Updates the status or configuration of the specified UART channel.
 *        This function might be used for re-initialization or parameter changes.
 *
 * @param uart_channel The UART peripheral to update.
 */
void UART_Update(t_uart_channel uart_channel);

/**
 * @brief Sends a single byte of data via the specified UART channel.
 *
 * @param uart_channel The UART peripheral to use.
 * @param byte The byte of data to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);

/**
 * @brief Sends a frame (array of bytes) via the specified UART channel.
 *
 * @param uart_channel The UART peripheral to use.
 * @param data Pointer to the character array (frame) to send.
 * @param length The number of bytes in the frame.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string via the specified UART channel.
 *
 * @param uart_channel The UART peripheral to use.
 * @param str Pointer to the null-terminated string to send.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str);

/**
 * @brief Receives a single byte of data from the specified UART channel.
 *
 * @param uart_channel The UART peripheral to use.
 * @return The received byte of data.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel);

/**
 * @brief Receives a frame (sequence of bytes) from the specified UART channel.
 *
 * @param uart_channel The UART peripheral to use.
 * @param buffer Pointer to the buffer where received data will be stored.
 * @param max_length The maximum number of bytes to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);

/**
 * @brief Receives a null-terminated string from the specified UART channel.
 *        The function will stop receiving when max_length is reached or a
 *        specific termination character (e.g., newline) is received.
 *
 * @param uart_channel The UART peripheral to use.
 * @param buffer Pointer to the buffer where the received string will be stored.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes received, or a status code.
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

/**
 * @brief Clears pending flags for the specified UART channel.
 *
 * @param uart_channel The UART peripheral.
 */
void UART_ClearFlag(t_uart_channel uart_channel);

/* --- I2C APIs --- */

/**
 * @brief Initializes the specified I2C channel with given parameters.
 *
 * @param i2c_channel The I2C peripheral to initialize (I2C1, I2C2, I2C3).
 * @param i2c_clk_speed The desired I2C clock speed (Rule: Always fast mode).
 * @param i2c_device_address The 7-bit or 10-bit device address for the MCU in slave mode.
 * @param i2c_ack Acknowledge control (Enable/Disable).
 * @param i2c_datalength The data length for transfers (typically 8-bit).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);

/**
 * @brief Enables the specified I2C peripheral.
 *        This includes enabling its clock and the peripheral itself.
 *
 * @param i2c_channel The I2C peripheral to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel);

/**
 * @brief Disables the specified I2C peripheral.
 *
 * @param i2c_channel The I2C peripheral to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel);

/**
 * @brief Updates the status or configuration of the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral to update.
 */
void I2C_Update(t_i2c_channel i2c_channel);

/**
 * @brief Sends a single byte of data via the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral to use.
 * @param byte The byte of data to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);

/**
 * @brief Sends a frame (array of bytes) via the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral to use.
 * @param data Pointer to the character array (frame) to send.
 * @param length The number of bytes in the frame.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string via the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral to use.
 * @param str Pointer to the null-terminated string to send.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);

/**
 * @brief Receives a single byte of data from the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral to use.
 * @return The received byte of data.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);

/**
 * @brief Receives a frame (sequence of bytes) from the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral to use.
 * @param buffer Pointer to the buffer where received data will be stored.
 * @param max_length The maximum number of bytes to receive.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);

/**
 * @brief Receives a null-terminated string from the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral to use.
 * @param buffer Pointer to the buffer where the received string will be stored.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes received, or a status code.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

/**
 * @brief Clears pending flags for the specified I2C channel.
 *
 * @param i2c_channel The I2C peripheral.
 */
void I2C_ClearFlag(t_i2c_channel i2c_channel);

/* --- SPI APIs --- */

/**
 * @brief Initializes the specified SPI channel with given parameters.
 *
 * @param spi_channel The SPI peripheral to initialize (SPI1, SPI2, SPI3).
 * @param spi_mode The operating mode (Master/Slave).
 * @param spi_cpol Clock Polarity.
 * @param spi_cpha Clock Phase.
 * @param spi_dff Data Frame Format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);

/**
 * @brief Enables the specified SPI peripheral.
 *        This includes enabling its clock and the peripheral itself.
 *
 * @param spi_channel The SPI peripheral to enable.
 */
void SPI_Enable(t_spi_channel spi_channel);

/**
 * @brief Disables the specified SPI peripheral.
 *
 * @param spi_channel The SPI peripheral to disable.
 */
void SPI_Disable(t_spi_channel spi_channel);

/**
 * @brief Updates the status or configuration of the SPI peripheral.
 *        Note: The API.json specifies `void SPI_Update(void)`, not channel specific.
 */
void SPI_Update(void);

/**
 * @brief Sends a single byte of data via the specified SPI channel.
 *
 * @param spi_channel The SPI peripheral to use.
 * @param byte The byte of data to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);

/**
 * @brief Sends a frame (array of bytes) via the specified SPI channel.
 *
 * @param spi_channel The SPI peripheral to use.
 * @param data Pointer to the character array (frame) to send.
 * @param length The number of bytes in the frame.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string via the specified SPI channel.
 *
 * @param spi_channel The SPI peripheral to use.
 * @param str Pointer to the null-terminated string to send.
 */
void SPI_send_string(t_spi_channel spi_channel, const char *str);

/**
 * @brief Receives a single byte of data from the specified SPI channel.
 *
 * @param spi_channel The SPI peripheral to use.
 * @return The received byte of data.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel);

/**
 * @brief Receives a frame (sequence of bytes) from the specified SPI channel.
 *
 * @param spi_channel The SPI peripheral to use.
 * @param buffer Pointer to the buffer where received data will be stored.
 * @param max_length The maximum number of bytes to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);

/**
 * @brief Receives a null-terminated string from the specified SPI channel.
 *
 * @param spi_channel The SPI peripheral to use.
 * @param buffer Pointer to the buffer where the received string will be stored.
 * @param max_length The maximum length of the string to receive (including null terminator).
 * @return The number of bytes received, or a status code.
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

/**
 * @brief Clears pending flags for the specified SPI channel.
 *
 * @param spi_channel The SPI peripheral.
 */
void SPI_ClearFlag(t_spi_channel spi_channel);

/* --- External Interrupt APIs --- */

/**
 * @brief Initializes an External Interrupt (EXTI) line.
 *        Configures the source GPIO port and the trigger edge.
 *
 * @param external_int_channel The EXTI line to configure (EXTI_LINE_0 to EXTI_LINE_15).
 * @param external_int_edge The trigger edge (Rising, Falling, or Both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);

/**
 * @brief Enables the specified External Interrupt (EXTI) line.
 *
 * @param external_int_channel The EXTI line to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel);

/**
 * @brief Disables the specified External Interrupt (EXTI) line.
 *
 * @param external_int_channel The EXTI line to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel);

/**
 * @brief Clears the pending flag for the specified External Interrupt (EXTI) line.
 *
 * @param external_int_channel The EXTI line for which to clear the flag.
 */
void External_INT_ClearFlag(t_external_int_channel external_int_channel);

/* --- GPIO APIs --- */

/**
 * @brief Initializes a GPIO pin as an output and sets its initial value.
 *
 * @param port The GPIO port (e.g., GPIO_PORT_A).
 * @param pin The pin number within the port (e.g., GPIO_PIN_0).
 * @param value The initial output value for the pin (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);

/**
 * @brief Initializes a GPIO pin as an input.
 *        Input pins will have pull-up resistors enabled (as per rules).
 *
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 */
void GPIO_Input_Init(t_port port, t_pin pin);

/**
 * @brief Gets the current direction of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction of the pin (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin);

/**
 * @brief Sets the output value of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);

/**
 * @brief Gets the input value of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The input value of the pin (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin);

/**
 * @brief Toggles the output value of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin);

/* --- PWM APIs --- */

/**
 * @brief Initializes a PWM channel with a specified frequency and duty cycle.
 *
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);

/**
 * @brief Starts the specified PWM channel.
 *
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel);

/**
 * @brief Stops the specified PWM channel.
 *
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel);

/* --- ICU APIs --- */

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 *
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler setting for the timer.
 * @param icu_edge The edge to trigger capture on (Rising, Falling, Both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);

/**
 * @brief Enables the specified ICU channel.
 *
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel);

/**
 * @brief Disables the specified ICU channel.
 *
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel);

/**
 * @brief Updates the frequency capture for the specified ICU channel.
 *        This might involve re-triggering or reading capture values.
 *
 * @param icu_channel The ICU channel to update.
 */
void ICU_Updatefrequency(t_icu_channel icu_channel);

/**
 * @brief Gets the measured frequency from the specified ICU channel.
 *
 * @param icu_channel The ICU channel.
 * @return The captured frequency value.
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel);

/**
 * @brief Sets up the buffer for remote control keys.
 *
 * @param number_of_keys The total number of remote control keys to store.
 * @param key_digits_length The length (number of digits/bits) for each key.
 */
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);

/**
 * @brief Sets individual digits/bits for a specific remote control key.
 *
 * @param key_num The index of the key.
 * @param key_array_cell The cell (position) within the key's digit array.
 * @param key_cell_value The value to set for that cell.
 */
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);

/**
 * @brief Updates parameters for decoding remote control signals.
 *
 * @param icu_channel The ICU channel used for remote control signal capture.
 * @param strt_bit_us_value The expected duration of the start bit in microseconds.
 * @param one_bit_us_value The expected duration of a '1' bit in microseconds.
 * @param zero_bit_us_value The expected duration of a '0' bit in microseconds.
 * @param stop_bit_us_value The expected duration of the stop bit in microseconds.
 */
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);

/**
 * @brief Gets the identified remote control key from the specified ICU channel.
 *
 * @param icu_channel The ICU channel.
 * @return The detected remote control key, or a special value if no key is detected.
 */
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);

/**
 * @brief Sets a callback function to be executed when an ICU event occurs.
 *
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void));

/**
 * @brief Clears pending flags for the specified ICU channel.
 *
 * @param icu_channel The ICU peripheral.
 */
void ICU_ClearFlag(t_icu_channel icu_channel);

/* --- Timer APIs --- */

/**
 * @brief Initializes a generic timer channel.
 *
 * @param timer_channel The timer peripheral to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel);

/**
 * @brief Sets the timer to generate an interrupt or event after a specified time in microseconds.
 *
 * @param timer_channel The timer peripheral.
 * @param time The time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time);

/**
 * @brief Sets the timer to generate an interrupt or event after a specified time in milliseconds.
 *
 * @param timer_channel The timer peripheral.
 * @param time The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);

/**
 * @brief Sets the timer to generate an interrupt or event after a specified time in seconds.
 *
 * @param timer_channel The timer peripheral.
 * @param time The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Sets the timer to generate an interrupt or event after a specified time in minutes.
 *
 * @param timer_channel The timer peripheral.
 * @param time The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Sets the timer to generate an interrupt or event after a specified time in hours.
 *
 * @param timer_channel The timer peripheral.
 * @param time The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Enables the specified timer channel.
 *
 * @param timer_channel The timer peripheral to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel);

/**
 * @brief Disables the specified timer channel.
 *
 * @param timer_channel The timer peripheral to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel);

/**
 * @brief Clears pending flags for the specified timer channel.
 *
 * @param timer_channel The timer peripheral.
 */
void TIMER_ClearFlag(t_timer_channel timer_channel);

/* --- ADC APIs --- */

/**
 * @brief Initializes the ADC peripheral for a specific channel and mode.
 *
 * @param adc_channel The analog input channel to configure.
 * @param adc_mode The ADC operation mode (single or continuous conversion).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);

/**
 * @brief Enables the ADC peripheral.
 *        This includes enabling its clock and the peripheral itself.
 */
void ADC_Enable(void);

/**
 * @brief Disables the ADC peripheral.
 */
void ADC_Disable(void);

/**
 * @brief Triggers or updates the ADC conversion process.
 */
void ADC_Update(void);

/**
 * @brief Gets the latest converted ADC value.
 *
 * @return The 16-bit digital value from the ADC.
 */
tword ADC_Get(void);

/**
 * @brief Clears pending flags for the ADC peripheral.
 */
void ADC_ClearFlag(void);

/* --- Internal_EEPROM APIs --- */

/**
 * @brief Initializes the internal EEPROM (Flash memory for emulation on STM32F4).
 */
void Internal_EEPROM_Init(void);

/**
 * @brief Sets (writes) a byte of data to a specified EEPROM address.
 *
 * @param address The address within the EEPROM to write to.
 * @param data The byte of data to write.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data);

/**
 * @brief Gets (reads) a byte of data from a specified EEPROM address.
 *
 * @param address The address within the EEPROM to read from.
 * @return The byte of data read from the EEPROM.
 */
tbyte Internal_EEPROM_Get(tbyte address);

/* --- TT (Time-Triggered OS) APIs --- */

/**
 * @brief Initializes the Time-Triggered OS with a specified tick time.
 *        This typically configures a timer to generate periodic interrupts.
 *
 * @param tick_time_ms The duration of one tick in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms);

/**
 * @brief Starts the Time-Triggered OS scheduler.
 */
void TT_Start(void);

/**
 * @brief Dispatches tasks based on their configured periods and delays.
 *        This function is usually called periodically in the main loop or from a high-priority task.
 */
void TT_Dispatch_task(void);

/**
 * @brief Interrupt Service Routine (ISR) for the Time-Triggered OS timer.
 *        This function should be called by the timer interrupt handler.
 */
void TT_ISR(void);

/**
 * @brief Adds a new task to the TT OS scheduler.
 *
 * @param task Pointer to the task function.
 * @param period The periodicity of the task in ticks.
 * @param delay The initial delay before the task first runs, in ticks.
 * @return The index of the added task, or 0xFF if addition failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);

/**
 * @brief Deletes a task from the TT OS scheduler.
 *
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index);

#endif // MCAL_H