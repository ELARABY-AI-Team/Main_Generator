/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the API prototypes and type definitions for the MCAL layer,
 * providing a hardware-independent interface to the STM32F401RC microcontroller's
 * peripherals.
 *
 * @note This file is generated based on provided register definitions and API specifications.
 *       Some functionalities might be inferred or commented out if specific registers
 *       are not available in the input JSON.
 */

#ifndef MCAL_H
#define MCAL_H

// Core device header for STM32F401RC
#include "stm32f401xc.h" 

// Standard C library includes as per core_includes rule
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Type Definitions as per data_type_definitions rule
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;
typedef const char tsbyte; // For MQTT/HTTP/WiFi string literals
typedef long tslong;       // For MQTT event_id

/*******************************************************************************
 *                              GPIO Module Types                              *
 *******************************************************************************/
/**
 * @brief Enumeration for GPIO ports.
 */
typedef enum {
    GPIO_PORTA, /**< GPIO Port A */
    GPIO_PORTB, /**< GPIO Port B */
    GPIO_PORTC, /**< GPIO Port C */
    GPIO_PORTD, /**< GPIO Port D */
    GPIO_PORTE, /**< GPIO Port E */
    GPIO_PORTH  /**< GPIO Port H */
} t_port;

/**
 * @brief Enumeration for GPIO pins within a port.
 */
typedef enum {
    GPIO_PIN0,  /**< Pin 0 */
    GPIO_PIN1,  /**< Pin 1 */
    GPIO_PIN2,  /**< Pin 2 */
    GPIO_PIN3,  /**< Pin 3 */
    GPIO_PIN4,  /**< Pin 4 */
    GPIO_PIN5,  /**< Pin 5 */
    GPIO_PIN6,  /**< Pin 6 */
    GPIO_PIN7,  /**< Pin 7 */
    GPIO_PIN8,  /**< Pin 8 */
    GPIO_PIN9,  /**< Pin 9 */
    GPIO_PIN10, /**< Pin 10 */
    GPIO_PIN11, /**< Pin 11 */
    GPIO_PIN12, /**< Pin 12 */
    GPIO_PIN13, /**< Pin 13 */
    GPIO_PIN14, /**< Pin 14 */
    GPIO_PIN15  /**< Pin 15 */
} t_pin;

/**
 * @brief Enumeration for GPIO pin direction.
 */
typedef enum {
    GPIO_INPUT,  /**< Pin configured as input */
    GPIO_OUTPUT  /**< Pin configured as output */
} t_direction;

/**
 * @brief Enumeration for GPIO pin logic level.
 */
typedef enum {
    GPIO_LOW,  /**< Pin logic low (0) */
    GPIO_HIGH  /**< Pin logic high (1) */
} t_gpio_value;

/*******************************************************************************
 *                              UART Module Types                              *
 *******************************************************************************/
/**
 * @brief Enumeration for UART channels.
 */
typedef enum {
    UART_CHANNEL_USART1, /**< USART1 channel */
    UART_CHANNEL_USART2, /**< USART2 channel */
    UART_CHANNEL_USART6  /**< USART6 channel */
} t_uart_channel;

/**
 * @brief Enumeration for common UART baud rates.
 */
typedef enum {
    UART_BAUD_9600,   /**< 9600 baud rate */
    UART_BAUD_19200,  /**< 19200 baud rate */
    UART_BAUD_38400,  /**< 38400 baud rate */
    UART_BAUD_57600,  /**< 57600 baud rate */
    UART_BAUD_115200  /**< 115200 baud rate */
} t_uart_baud_rate;

/**
 * @brief Enumeration for UART data length.
 */
typedef enum {
    UART_DATA_8BIT, /**< 8 data bits */
    UART_DATA_9BIT  /**< 9 data bits */
} t_uart_data_length;

/**
 * @brief Enumeration for UART stop bits.
 */
typedef enum {
    UART_STOPBIT_1,    /**< 1 stop bit */
    UART_STOPBIT_0_5,  /**< 0.5 stop bits */
    UART_STOPBIT_2,    /**< 2 stop bits */
    UART_STOPBIT_1_5   /**< 1.5 stop bits */
} t_uart_stop_bit;

/**
 * @brief Enumeration for UART parity.
 */
typedef enum {
    UART_PARITY_NONE, /**< No parity */
    UART_PARITY_EVEN, /**< Even parity */
    UART_PARITY_ODD   /**< Odd parity */
} t_uart_parity;

/*******************************************************************************
 *                              I2C Module Types                               *
 *******************************************************************************/
/**
 * @brief Enumeration for I2C channels.
 */
typedef enum {
    I2C_CHANNEL_I2C1, /**< I2C1 channel */
    I2C_CHANNEL_I2C2, /**< I2C2 channel */
    I2C_CHANNEL_I2C3  /**< I2C3 channel */
} t_i2c_channel;

/**
 * @brief Enumeration for I2C clock speeds.
 */
typedef enum {
    I2C_CLK_SPEED_STANDARD, /**< Standard mode (up to 100 kHz) */
    I2C_CLK_SPEED_FAST      /**< Fast mode (up to 400 kHz) */
} t_i2c_clk_speed;

/**
 * @brief Type definition for I2C device address.
 */
typedef uint8_t t_i2c_device_address;

/**
 * @brief Enumeration for I2C Acknowledge control.
 */
typedef enum {
    I2C_ACK_ENABLE,  /**< Acknowledge enabled */
    I2C_ACK_DISABLE  /**< Acknowledge disabled */
} t_i2c_ack;

/**
 * @brief Enumeration for I2C data length.
 */
typedef enum {
    I2C_DATALENGTH_8BIT,  /**< 8-bit data length */
    I2C_DATALENGTH_16BIT  /**< 16-bit data length (not directly supported by all I2C peripherals) */
} t_i2c_datalength;

/*******************************************************************************
 *                              SPI Module Types                               *
 *******************************************************************************/
/**
 * @brief Enumeration for SPI channels.
 */
typedef enum {
    SPI_CHANNEL_SPI1, /**< SPI1 channel */
    SPI_CHANNEL_SPI2, /**< SPI2 channel */
    SPI_CHANNEL_SPI3  /**< SPI3 channel */
} t_spi_channel;

/**
 * @brief Enumeration for SPI mode (Master/Slave).
 */
typedef enum {
    SPI_MODE_SLAVE,  /**< SPI Slave mode */
    SPI_MODE_MASTER  /**< SPI Master mode */
} t_spi_mode;

/**
 * @brief Enumeration for SPI Clock Polarity (CPOL).
 */
typedef enum {
    SPI_CPOL_LOW,  /**< Clock to 0 when idle */
    SPI_CPOL_HIGH  /**< Clock to 1 when idle */
} t_spi_cpol;

/**
 * @brief Enumeration for SPI Clock Phase (CPHA).
 */
typedef enum {
    SPI_CPHA_1EDGE, /**< Data sampled on first clock edge */
    SPI_CPHA_2EDGE  /**< Data sampled on second clock edge */
} t_spi_cpha;

/**
 * @brief Enumeration for SPI Data Frame Format (DFF).
 */
typedef enum {
    SPI_DFF_8BIT,  /**< 8-bit data frame format */
    SPI_DFF_16BIT  /**< 16-bit data frame format */
} t_spi_dff;

/**
 * @brief Enumeration for SPI Bit Order (MSB/LSB first).
 */
typedef enum {
    SPI_BIT_ORDER_MSB_FIRST, /**< Most Significant Bit first */
    SPI_BIT_ORDER_LSB_FIRST  /**< Least Significant Bit first */
} t_spi_bit_order;

/*******************************************************************************
 *                         External Interrupt Module Types                     *
 *******************************************************************************/
/**
 * @brief Enumeration for External Interrupt (EXTI) lines.
 */
typedef enum {
    EXTI_LINE0,  /**< EXTI Line 0 */
    EXTI_LINE1,  /**< EXTI Line 1 */
    EXTI_LINE2,  /**< EXTI Line 2 */
    EXTI_LINE3,  /**< EXTI Line 3 */
    EXTI_LINE4,  /**< EXTI Line 4 */
    EXTI_LINE5,  /**< EXTI Line 5 */
    EXTI_LINE6,  /**< EXTI Line 6 */
    EXTI_LINE7,  /**< EXTI Line 7 */
    EXTI_LINE8,  /**< EXTI Line 8 */
    EXTI_LINE9,  /**< EXTI Line 9 */
    EXTI_LINE10, /**< EXTI Line 10 */
    EXTI_LINE11, /**< EXTI Line 11 */
    EXTI_LINE12, /**< EXTI Line 12 */
    EXTI_LINE13, /**< EXTI Line 13 */
    EXTI_LINE14, /**< EXTI Line 14 */
    EXTI_LINE15  /**< EXTI Line 15 */
} t_external_int_channel;

/**
 * @brief Enumeration for External Interrupt trigger edge.
 */
typedef enum {
    EXTI_EDGE_RISING,  /**< Rising edge trigger */
    EXTI_EDGE_FALLING, /**< Falling edge trigger */
    EXTI_EDGE_BOTH     /**< Both rising and falling edge trigger */
} t_external_int_edge;

/*******************************************************************************
 *                            Timer Module Types                               *
 *******************************************************************************/
/**
 * @brief Enumeration for generic Timer channels.
 */
typedef enum {
    TIMER_CHANNEL_TIM1,  /**< Timer 1 */
    TIMER_CHANNEL_TIM2,  /**< Timer 2 */
    TIMER_CHANNEL_TIM3,  /**< Timer 3 */
    TIMER_CHANNEL_TIM4,  /**< Timer 4 */
    TIMER_CHANNEL_TIM5,  /**< Timer 5 */
    TIMER_CHANNEL_TIM9,  /**< Timer 9 */
    TIMER_CHANNEL_TIM10, /**< Timer 10 */
    TIMER_CHANNEL_TIM11  /**< Timer 11 */
} t_timer_channel;

/*******************************************************************************
 *                              PWM Module Types                               *
 *******************************************************************************/
/**
 * @brief Enumeration for PWM channels.
 */
typedef enum {
    PWM_CHANNEL_TIM1_CH1, /**< TIM1 Channel 1 (PA8, PE9) */
    PWM_CHANNEL_TIM1_CH2, /**< TIM1 Channel 2 (PA9, PE11) */
    PWM_CHANNEL_TIM1_CH3, /**< TIM1 Channel 3 (PA10, PE13) */
    PWM_CHANNEL_TIM1_CH4, /**< TIM1 Channel 4 (PA11, PE14) */
    PWM_CHANNEL_TIM2_CH1, /**< TIM2 Channel 1 (PA0, PA5, PA15, PB3) */
    PWM_CHANNEL_TIM2_CH2, /**< TIM2 Channel 2 (PA1, PB3, PB10) */
    PWM_CHANNEL_TIM2_CH3, /**< TIM2 Channel 3 (PA2, PB10) */
    PWM_CHANNEL_TIM2_CH4, /**< TIM2 Channel 4 (PA3, PB11) */
    PWM_CHANNEL_TIM3_CH1, /**< TIM3 Channel 1 (PA6, PB4, PC6) */
    PWM_CHANNEL_TIM3_CH2, /**< TIM3 Channel 2 (PA7, PB5, PC7) */
    PWM_CHANNEL_TIM3_CH3, /**< TIM3 Channel 3 (PB0, PC8) */
    PWM_CHANNEL_TIM3_CH4, /**< TIM3 Channel 4 (PB1, PC9) */
    PWM_CHANNEL_TIM4_CH1, /**< TIM4 Channel 1 (PB6) */
    PWM_CHANNEL_TIM4_CH2, /**< TIM4 Channel 2 (PB7) */
    PWM_CHANNEL_TIM4_CH3, /**< TIM4 Channel 3 (PB8) */
    PWM_CHANNEL_TIM4_CH4, /**< TIM4 Channel 4 (PB9) */
    PWM_CHANNEL_TIM5_CH1, /**< TIM5 Channel 1 (PA0) */
    PWM_CHANNEL_TIM5_CH2, /**< TIM5 Channel 2 (PA1) */
    PWM_CHANNEL_TIM5_CH3, /**< TIM5 Channel 3 (PA2) */
    PWM_CHANNEL_TIM5_CH4, /**< TIM5 Channel 4 (PA3) */
    PWM_CHANNEL_TIM9_CH1, /**< TIM9 Channel 1 (PA2, PE5) */
    PWM_CHANNEL_TIM9_CH2, /**< TIM9 Channel 2 (PA3, PE6) */
    PWM_CHANNEL_TIM10_CH1,/**< TIM10 Channel 1 (PB8, PA6) */
    PWM_CHANNEL_TIM11_CH1 /**< TIM11 Channel 1 (PB9, PA7) */
} t_pwm_channel;

/*******************************************************************************
 *                               ICU Module Types                              *
 *******************************************************************************/
/**
 * @brief Enumeration for ICU channels.
 */
typedef enum {
    ICU_CHANNEL_TIM1_CH1, /**< TIM1 Channel 1 (PA8, PE9) */
    ICU_CHANNEL_TIM1_CH2, /**< TIM1 Channel 2 (PA9, PE11) */
    ICU_CHANNEL_TIM1_CH3, /**< TIM1 Channel 3 (PA10, PE13) */
    ICU_CHANNEL_TIM1_CH4, /**< TIM1 Channel 4 (PA11, PE14) */
    ICU_CHANNEL_TIM2_CH1, /**< TIM2 Channel 1 (PA0, PA5, PA15, PB3) */
    ICU_CHANNEL_TIM2_CH2, /**< TIM2 Channel 2 (PA1, PB3, PB10) */
    ICU_CHANNEL_TIM2_CH3, /**< TIM2 Channel 3 (PA2, PB10) */
    ICU_CHANNEL_TIM2_CH4, /**< TIM2 Channel 4 (PA3, PB11) */
    ICU_CHANNEL_TIM3_CH1, /**< TIM3 Channel 1 (PA6, PB4, PC6) */
    ICU_CHANNEL_TIM3_CH2, /**< TIM3 Channel 2 (PA7, PB5, PC7) */
    ICU_CHANNEL_TIM3_CH3, /**< TIM3 Channel 3 (PB0, PC8) */
    ICU_CHANNEL_TIM3_CH4, /**< TIM3 Channel 4 (PB1, PC9) */
    ICU_CHANNEL_TIM4_CH1, /**< TIM4 Channel 1 (PB6) */
    ICU_CHANNEL_TIM4_CH2, /**< TIM4 Channel 2 (PB7) */
    ICU_CHANNEL_TIM4_CH3, /**< TIM4 Channel 3 (PB8) */
    ICU_CHANNEL_TIM4_CH4, /**< TIM4 Channel 4 (PB9) */
    ICU_CHANNEL_TIM5_CH1, /**< TIM5 Channel 1 (PA0) */
    ICU_CHANNEL_TIM5_CH2, /**< TIM5 Channel 2 (PA1) */
    ICU_CHANNEL_TIM5_CH3, /**< TIM5 Channel 3 (PA2) */
    ICU_CHANNEL_TIM5_CH4, /**< TIM5 Channel 4 (PA3) */
    ICU_CHANNEL_TIM9_CH1, /**< TIM9 Channel 1 (PA2, PE5) */
    ICU_CHANNEL_TIM9_CH2, /**< TIM9 Channel 2 (PA3, PE6) */
    ICU_CHANNEL_TIM10_CH1,/**< TIM10 Channel 1 (PB8, PA6) */
    ICU_CHANNEL_TIM11_CH1 /**< TIM11 Channel 1 (PB9, PA7) */
} t_icu_channel;

/**
 * @brief Enumeration for ICU prescaler values.
 */
typedef enum {
    ICU_PRESCALER_1, /**< No prescaling */
    ICU_PRESCALER_2, /**< Divide by 2 */
    ICU_PRESCALER_4, /**< Divide by 4 */
    ICU_PRESCALER_8  /**< Divide by 8 */
} t_icu_prescaller;

/**
 * @brief Enumeration for ICU trigger edge.
 */
typedef enum {
    ICU_EDGE_RISING,     /**< Rising edge trigger */
    ICU_EDGE_FALLING,    /**< Falling edge trigger */
    ICU_EDGE_BOTH_EDGES  /**< Both rising and falling edge trigger */
} t_icu_edge;

/*******************************************************************************
 *                               ADC Module Types                              *
 *******************************************************************************/
/**
 * @brief Enumeration for ADC channels.
 */
typedef enum {
    ADC_CHANNEL_0,  /**< ADC Channel 0 (PA0) */
    ADC_CHANNEL_1,  /**< ADC Channel 1 (PA1) */
    ADC_CHANNEL_2,  /**< ADC Channel 2 (PA2) */
    ADC_CHANNEL_3,  /**< ADC Channel 3 (PA3) */
    ADC_CHANNEL_4,  /**< ADC Channel 4 (PA4) */
    ADC_CHANNEL_5,  /**< ADC Channel 5 (PA5) */
    ADC_CHANNEL_6,  /**< ADC Channel 6 (PA6) */
    ADC_CHANNEL_7,  /**< ADC Channel 7 (PA7) */
    ADC_CHANNEL_8,  /**< ADC Channel 8 (PB0) */
    ADC_CHANNEL_9,  /**< ADC Channel 9 (PB1) */
    ADC_CHANNEL_10, /**< ADC Channel 10 (PC0) */
    ADC_CHANNEL_11, /**< ADC Channel 11 (PC1) */
    ADC_CHANNEL_12, /**< ADC Channel 12 (PC2) */
    ADC_CHANNEL_13, /**< ADC Channel 13 (PC3) */
    ADC_CHANNEL_14, /**< ADC Channel 14 (PC4) */
    ADC_CHANNEL_15, /**< ADC Channel 15 (PC5) */
    ADC_CHANNEL_16, /**< ADC Channel 16 (Internal temperature sensor) */
    ADC_CHANNEL_17, /**< ADC Channel 17 (Internal Vrefint) */
    ADC_CHANNEL_18  /**< ADC Channel 18 (Internal VBAT) */
} t_adc_channel;

/**
 * @brief Enumeration for ADC conversion mode.
 */
typedef enum {
    ADC_MODE_POLLING,    /**< Polling mode for ADC conversion */
    ADC_MODE_INTERRUPT   /**< Interrupt mode for ADC conversion */
} t_adc_mode_t;

/*******************************************************************************
 *                            MCU Config Module Types                          *
 *******************************************************************************/
/**
 * @brief Enumeration for system voltage source.
 */
typedef enum {
    Vsource_3V, /**< 3V system voltage */
    Vsource_5V  /**< 5V system voltage */
} t_sys_volt;

/*******************************************************************************
 *                               LVD Module Types                              *
 *******************************************************************************/
/**
 * @brief Enumeration for Low Voltage Detection (LVD) threshold levels.
 */
typedef enum {
    Volt_0_5V, /**< LVD threshold 0.5V */
    Volt_1V,   /**< LVD threshold 1.0V */
    Volt_1_5V, /**< LVD threshold 1.5V */
    Volt_2V,   /**< LVD threshold 2.0V */
    Volt_2_5V, /**< LVD threshold 2.5V */
    Volt_3V,   /**< LVD threshold 3.0V */
    Volt_3_5V, /**< LVD threshold 3.5V */
    Volt_4V,   /**< LVD threshold 4.0V */
    Volt_4_5V, /**< LVD threshold 4.5V */
    Volt_5V    /**< LVD threshold 5.0V */
} t_lvd_thrthresholdLevel;

/*******************************************************************************
 *                               TT Module Types                               *
 *******************************************************************************/
/**
 * @brief Enumeration for Time-Triggered (TT) OS tick time.
 */
typedef enum {
    TT_TICK_1MS,   /**< 1 millisecond tick */
    TT_TICK_10MS,  /**< 10 millisecond tick */
    TT_TICK_100MS  /**< 100 millisecond tick */
} t_tick_time;

/*******************************************************************************
 *                               I2S Module Types                              *
 *******************************************************************************/
/**
 * @brief Enumeration for I2S channels.
 */
typedef enum {
    I2S_CHANNEL_SPI1, /**< I2S channel associated with SPI1 */
    I2S_CHANNEL_SPI2, /**< I2S channel associated with SPI2 */
    I2S_CHANNEL_SPI3  /**< I2S channel associated with SPI3 */
} t_i2s_channel;

/**
 * @brief Enumeration for I2S operating mode.
 */
typedef enum {
    I2S_MODE_SLAVE_TX,  /**< Slave transmit mode */
    I2S_MODE_SLAVE_RX,  /**< Slave receive mode */
    I2S_MODE_MASTER_TX, /**< Master transmit mode */
    I2S_MODE_MASTER_RX  /**< Master receive mode */
} I2S_Mode_t;

/**
 * @brief Enumeration for I2S standard.
 */
typedef enum {
    I2S_STANDARD_PHILIPS,    /**< I2S Philips standard */
    I2S_STANDARD_MSB,        /**< MSB justified standard */
    I2S_STANDARD_LSB,        /**< LSB justified standard */
    I2S_STANDARD_PCM_SHORT,  /**< PCM short frame synchronization */
    I2S_STANDARD_PCM_LONG    /**< PCM long frame synchronization */
} I2S_Standard_t;

/**
 * @brief Enumeration for I2S data format.
 */
typedef enum {
    I2S_DATAFORMAT_16B,           /**< 16-bit data length */
    I2S_DATAFORMAT_16B_EXTENDED,  /**< 16-bit data length with 32-bit channel frame */
    I2S_DATAFORMAT_24B,           /**< 24-bit data length */
    I2S_DATAFORMAT_32B            /**< 32-bit data length */
} I2S_DataFormat_t;

/**
 * @brief Enumeration for I2S channel mode.
 */
typedef enum {
    I2S_CHANNELMODE_STEREO, /**< Stereo mode */
    I2S_CHANNELMODE_MONO    /**< Mono mode */
} I2S_ChannelMode_t;

/*******************************************************************************
 *                             WiFi Driver Types                               *
 *******************************************************************************/
/**
 * @brief Enumeration for WiFi transmit mode.
 */
typedef enum {
    WIFI_TX_MODE_NORMAL,    /**< Normal transmit mode */
    WIFI_TX_MODE_LOW_POWER  /**< Low power transmit mode */
} t_tx_mode;

/*******************************************************************************
 *                                 API Prototypes                              *
 *******************************************************************************/

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void);
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
// LVD not supported on this MCU (no specific registers found in JSON).
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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Changed return type to tlong for frequency
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
// Internal_EEPROM not supported on this MCU (no dedicated EEPROM registers found in JSON).
// void Internal_EEPROM_Init(void);
// void Internal_EEPROM_Set(tbyte address, tbyte data);
// tbyte Internal_EEPROM_Get(tbyte address);

// TT
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// MCAL_OUTPUT_BUZZER
// MCAL_OUTPUT_BUZZER not supported on this MCU (no specific registers found in JSON).
// void BUZZER_OUTPUT_Init(tbyte buzzer_number);
// void BUZZER_OUTPUT_Start(tbyte NUMBER_BUZZER);
// void BUZZER_OUTPUT_Stop(tbyte NUMBER_BUZZER);

// WDT
// WDT_Init is not explicitly in API.json, but WDT_Reset is. WDT_Init functionality is covered by MCU_Config_Init.
// void WDT_Init(void); // Covered by MCU_Config_Init
// void WDT_Reset(void); // Already declared above

// DAC
// DAC not supported on this MCU (no specific registers found in JSON).
// typedef enum {
//     DAC_CHANNEL_1,
//     DAC_CHANNEL_2
// } dac_channel_t;
// void DAC_Init(dac_channel_t channel);
// void DAC_Enable(dac_channel_t channel);
// void DAC_Disable(dac_channel_t channel);
// void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue);

// I2S
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
void I2S_Enable(t_i2s_channel channel);
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// MQTT Protocol
// MQTT Protocol not supported on this MCU (no specific registers/SDK dependencies found in JSON).
// void MQTT_Init(void);
// void MQTT_Subscribe_Topic(void *handler_args, esp_event_base_t base, tslong event_id, void *event_data);
// void MQTT_Publish_Message(const tsbyte *message);
// void AZURE_IoT_Hub_Client_Init(void);
// void AZURE_Connection_Enable(void);

// HTTP Protocol
// HTTP Protocol not supported on this MCU (no specific registers/SDK dependencies found in JSON).
// void HTTP_Get_Device_ID(char *device_id, size_t size);
// void HTTP_Server_Init(void);
// void HTTP_Server_Start(void);
// void HTTP_Server_Stop(void);
// void HTTP_Reset_SSID_PASSWORD(void);
// esp_err_t HTTP_Config_Handler(httpd_req_t *req); // esp_err_t and httpd_req_t are ESP-IDF specific

// WiFi Driver
// WiFi Driver not supported on this MCU (no specific registers/SDK dependencies found in JSON).
// void WiFi_Init(void);
// void WiFi_Connect(const tsbyte *ssid, const tsbyte *password);
// void WiFi_Enable(void);
// void WiFi_Disable(void);
// void WiFi_SetTxMode(t_tx_mode mode);
// tsword WiFi_Check_Connection(void);
// int WiFi_Check_Internet(void);

// DTC_driver
// DTC_driver not supported on this MCU (no specific registers found in JSON).
// void DTC_Init();
// void DTC_EnableSource(uint8_t source_id, uint8_t channel);
// void DTC_DisableSource(uint8_t source_id);
// void DTC_Start(void);
// void DTC_Stop(void);
// void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int);

#endif /* MCAL_H */