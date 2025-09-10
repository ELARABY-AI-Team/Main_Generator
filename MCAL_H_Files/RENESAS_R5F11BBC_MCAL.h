/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) header file for RENESAS_R5F11BBC.
 *
 * This file contains the definitions for data types, register addresses, and
 * function prototypes for the Microcontroller Abstraction Layer, providing
 * an interface to access the RENESAS_R5F11BBC MCU peripherals.
 *
 * It adheres to the coding standards and API specifications defined for
 * the project.
 */

#ifndef MCAL_H
#define MCAL_H

// --- Core MCU Header and Standard C Library Includes ---
#include <stdint.h>   // For uint8_t, uint16_t, uint32_t
#include <stdbool.h>  // For bool type
#include <stddef.h>   // For size_t
// #include "RENESAS_R5F11BBC.h" // Placeholder for device-specific header, if available.
                               // In a real project, this would be the actual device header.

// --- Data Type Definitions (as per Rules.json) ---
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// Placeholder for external library types if not defined elsewhere and needed by API.json
// For MQTT/HTTP/WiFi APIs, these types are assumed to be from their respective SDKs.
// Since these modules are optionally implemented based on MCU support, their types
// are only defined if the module is active. For this MCU, they are not supported by registers.
#if 0 // These are typically defined by external SDKs, not MCAL.
typedef int16_t tsword; // Example for WiFi_Check_Connection
#endif


// --- Enumeration Definitions (as per API.json and Rules.json) ---

/**
 * @brief Enumeration for system voltage levels.
 */
typedef enum
{
    sys_volt_3v = 0, /**< System voltage 3V */
    sys_volt_5v      /**< System voltage 5V */
} t_sys_volt;

/**
 * @brief Enumeration for Low Voltage Detection (LVD) threshold levels.
 *        Levels are generic as specific register bits are not provided for each level.
 *        Assumed mapping from common Renesas parts, adjust if actual register values
 *        are different.
 */
typedef enum
{
    lvd_threshold_level_0_5v = 0,
    lvd_threshold_level_1v,
    lvd_threshold_level_1_5v,
    lvd_threshold_level_2v,
    lvd_threshold_level_2_5v,
    lvd_threshold_level_3v,
    lvd_threshold_level_3_5v,
    lvd_threshold_level_4v,
    lvd_threshold_level_4_5v,
    lvd_threshold_level_5v
} t_lvd_thrthresholdLevel;

/**
 * @brief Enumeration for UART channels.
 *        Derived from SAU0_SMR00-03 (Serial Array Unit 0, Channels 0-3).
 */
typedef enum
{
    uart_channel_0 = 0, /**< Serial Array Unit 0, Channel 0 */
    uart_channel_1,     /**< Serial Array Unit 0, Channel 1 */
    uart_channel_2,     /**< Serial Array Unit 0, Channel 2 */
    uart_channel_3      /**< Serial Array Unit 0, Channel 3 */
} t_uart_channel;

/**
 * @brief Enumeration for common UART baud rates.
 */
typedef enum
{
    uart_baud_rate_2400 = 0,
    uart_baud_rate_4800,
    uart_baud_rate_9600,
    uart_baud_rate_19200,
    uart_baud_rate_38400,
    uart_baud_rate_57600,
    uart_baud_rate_115200
} t_uart_baud_rate;

/**
 * @brief Enumeration for UART data length.
 */
typedef enum
{
    uart_data_length_7_bit = 0,
    uart_data_length_8_bit,
    uart_data_length_9_bit
} t_uart_data_length;

/**
 * @brief Enumeration for UART stop bits.
 */
typedef enum
{
    uart_stop_bit_1 = 0,
    uart_stop_bit_2
} t_uart_stop_bit;

/**
 * @brief Enumeration for UART parity modes.
 */
typedef enum
{
    uart_parity_none = 0,
    uart_parity_even,
    uart_parity_odd
} t_uart_parity;

/**
 * @brief Enumeration for I2C channels.
 *        Derived from I2C_IICCTL00-01 (Serial Array Unit 0, Channels 0-1 for I2C control).
 */
typedef enum
{
    i2c_channel_0 = 0, /**< I2C Channel 0 (using SAU0 Channel 0 configuration) */
    i2c_channel_1      /**< I2C Channel 1 (using SAU0 Channel 1 configuration) */
} t_i2c_channel;

/**
 * @brief Enumeration for I2C clock speeds.
 */
typedef enum
{
    i2c_clk_speed_100khz = 0,
    i2c_clk_speed_400khz
} t_i2c_clk_speed;

/**
 * @brief Enumeration for I2C device address.
 *        This is a placeholder as the actual address would be a value.
 */
typedef tbyte t_i2c_device_address; // Use tbyte for 7-bit or 10-bit address

/**
 * @brief Enumeration for I2C Acknowledge.
 */
typedef enum
{
    i2c_ack_enable = 0,
    i2c_ack_disable
} t_i2c_ack;

/**
 * @brief Enumeration for I2C data length.
 *        Typically 8 bits for I2C data, but can be customized in some MCUs.
 */
typedef enum
{
    i2c_datalength_8_bit = 0
} t_i2c_datalength;

/**
 * @brief Enumeration for SPI channels.
 *        Derived from SAU0_SMR00-03 (Serial Array Unit 0, Channels 0-3 configurable for SPI).
 */
typedef enum
{
    spi_channel_0 = 0, /**< SPI Channel 0 (using SAU0 Channel 0 configuration) */
    spi_channel_1,     /**< SPI Channel 1 (using SAU0 Channel 1 configuration) */
    spi_channel_2,     /**< SPI Channel 2 (using SAU0 Channel 2 configuration) */
    spi_channel_3      /**< SPI Channel 3 (using SAU0 Channel 3 configuration) */
} t_spi_channel;

/**
 * @brief Enumeration for SPI modes (Master/Slave).
 */
typedef enum
{
    spi_mode_master = 0,
    spi_mode_slave
} t_spi_mode;

/**
 * @brief Enumeration for SPI Clock Polarity (CPOL).
 */
typedef enum
{
    spi_cpol_low = 0,  /**< Clock is low when idle */
    spi_cpol_high      /**< Clock is high when idle */
} t_spi_cpol;

/**
 * @brief Enumeration for SPI Clock Phase (CPHA).
 */
typedef enum
{
    spi_cpha_1_edge = 0, /**< Data sampled on first clock edge */
    spi_cpha_2_edge      /**< Data sampled on second clock edge */
} t_spi_cpha;

/**
 * @brief Enumeration for SPI Data Frame Format (DFF).
 */
typedef enum
{
    spi_dff_8_bit = 0, /**< 8-bit data frame */
    spi_dff_16_bit     /**< 16-bit data frame */
} t_spi_dff;

/**
 * @brief Enumeration for SPI Bit Order.
 */
typedef enum
{
    spi_bit_order_msb_first = 0, /**< Most significant bit first */
    spi_bit_order_lsb_first      /**< Least significant bit first */
} t_spi_bit_order;

/**
 * @brief Enumeration for external interrupt channels.
 *        Derived from GPIO_PIMK0/1 registers, which map to pins P0.x and P1.x.
 *        These are generic channels, actual pin mapping depends on configuration.
 */
typedef enum
{
    external_int_channel_0 = 0, /**< External interrupt channel 0 */
    external_int_channel_1,     /**< External interrupt channel 1 */
    external_int_channel_2,     /**< External interrupt channel 2 */
    external_int_channel_3,     /**< External interrupt channel 3 */
    external_int_channel_4,     /**< External interrupt channel 4 */
    external_int_channel_5,     /**< External interrupt channel 5 */
    external_int_channel_6,     /**< External interrupt channel 6 */
    external_int_channel_7,     /**< External interrupt channel 7 */
    // Add more channels if supported by GPIO_PIMK registers (P1.x for example)
    external_int_channel_8,     // P1.0
    external_int_channel_9,     // P1.1
    external_int_channel_10,    // P1.2
    external_int_channel_11,    // P1.3
    external_int_channel_12,    // P1.4
    external_int_channel_13,    // P1.5
    external_int_channel_14,    // P1.6
    external_int_channel_15     // P1.7
} t_external_int_channel;

/**
 * @brief Enumeration for external interrupt edge detection.
 */
typedef enum
{
    external_int_edge_rising = 0,  /**< Trigger on rising edge */
    external_int_edge_falling,     /**< Trigger on falling edge */
    external_int_edge_both         /**< Trigger on both rising and falling edges */
} t_external_int_edge;

/**
 * @brief Enumeration for GPIO ports.
 *        Derived from GPIO_Px registers.
 */
typedef enum
{
    port_0 = 0,  /**< Port 0 */
    port_1,      /**< Port 1 */
    port_2,      /**< Port 2 */
    port_3,      /**< Port 3 */
    port_4,      /**< Port 4 */
    port_5,      /**< Port 5 */
    port_6,      /**< Port 6 */
    port_7,      /**< Port 7 */
    port_12 = 12,/**< Port 12 */
    port_13,     /**< Port 13 */
    port_14      /**< Port 14 */
} t_port;

/**
 * @brief Enumeration for GPIO pins (0-7 for an 8-bit port).
 */
typedef enum
{
    pin_0 = 0, /**< Pin 0 */
    pin_1,     /**< Pin 1 */
    pin_2,     /**< Pin 2 */
    pin_3,     /**< Pin 3 */
    pin_4,     /**< Pin 4 */
    pin_5,     /**< Pin 5 */
    pin_6,     /**< Pin 6 */
    pin_7      /**< Pin 7 */
} t_pin;

/**
 * @brief Enumeration for GPIO pin direction.
 */
typedef enum
{
    direction_input = 0,  /**< Pin configured as input */
    direction_output      /**< Pin configured as output */
} t_direction;

/**
 * @brief Enumeration for PWM channels.
 *        Derived from TIM0_TO, TIM1_TO registers (Timer Array Unit Outputs).
 */
typedef enum
{
    pwm_channel_tau0_0 = 0, /**< Timer Array Unit 0, Channel 0 output */
    pwm_channel_tau0_1,     /**< Timer Array Unit 0, Channel 1 output */
    pwm_channel_tau0_2,     /**< Timer Array Unit 0, Channel 2 output */
    pwm_channel_tau0_3,     /**< Timer Array Unit 0, Channel 3 output */
    pwm_channel_tau0_4,     /**< Timer Array Unit 0, Channel 4 output */
    pwm_channel_tau0_5,     /**< Timer Array Unit 0, Channel 5 output */
    pwm_channel_tau0_6,     /**< Timer Array Unit 0, Channel 6 output */
    pwm_channel_tau0_7,     /**< Timer Array Unit 0, Channel 7 output */
    pwm_channel_tau1_0,     /**< Timer Array Unit 1, Channel 0 output */
    pwm_channel_tau1_1,     /**< Timer Array Unit 1, Channel 1 output */
    pwm_channel_tau1_2,     /**< Timer Array Unit 1, Channel 2 output */
    pwm_channel_tau1_3      /**< Timer Array Unit 1, Channel 3 output */
} t_pwm_channel;

/**
 * @brief Enumeration for Input Capture Unit (ICU) channels.
 *        Derived from TIM0_TMR and TIM1_TMR registers for capture mode.
 */
typedef enum
{
    icu_channel_tau0_0 = 0, /**< Timer Array Unit 0, Channel 0 input capture */
    icu_channel_tau0_1,     /**< Timer Array Unit 0, Channel 1 input capture */
    icu_channel_tau0_2,     /**< Timer Array Unit 0, Channel 2 input capture */
    icu_channel_tau0_3,     /**< Timer Array Unit 0, Channel 3 input capture */
    icu_channel_tau0_4,     /**< Timer Array Unit 0, Channel 4 input capture */
    icu_channel_tau0_5,     /**< Timer Array Unit 0, Channel 5 input capture */
    icu_channel_tau0_6,     /**< Timer Array Unit 0, Channel 6 input capture */
    icu_channel_tau0_7,     /**< Timer Array Unit 0, Channel 7 input capture */
    icu_channel_tau1_0,     /**< Timer Array Unit 1, Channel 0 input capture */
    icu_channel_tau1_1,     /**< Timer Array Unit 1, Channel 1 input capture */
    icu_channel_tau1_2,     /**< Timer Array Unit 1, Channel 2 input capture */
    icu_channel_tau1_3      /**< Timer Array Unit 1, Channel 3 input capture */
} t_icu_channel;

/**
 * @brief Enumeration for ICU prescaler values. Generic values, actual mapping in .c.
 */
typedef enum
{
    icu_prescaller_div_1 = 0,
    icu_prescaller_div_2,
    icu_prescaller_div_4,
    icu_prescaller_div_8,
    icu_prescaller_div_16,
    icu_prescaller_div_32,
    icu_prescaller_div_64,
    icu_prescaller_div_128
} t_icu_prescaller;

/**
 * @brief Enumeration for ICU edge detection.
 */
typedef enum
{
    icu_edge_rising = 0,  /**< Capture on rising edge */
    icu_edge_falling,     /**< Capture on falling edge */
    icu_edge_both         /**< Capture on both rising and falling edges */
} t_icu_edge;

/**
 * @brief Enumeration for Timer channels.
 *        Derived from TIM0_TMR and TIM1_TMR registers. Also includes Interval Timer.
 */
typedef enum
{
    timer_channel_tau0_0 = 0, /**< Timer Array Unit 0, Channel 0 */
    timer_channel_tau0_1,     /**< Timer Array Unit 0, Channel 1 */
    timer_channel_tau0_2,     /**< Timer Array Unit 0, Channel 2 */
    timer_channel_tau0_3,     /**< Timer Array Unit 0, Channel 3 */
    timer_channel_tau0_4,     /**< Timer Array Unit 0, Channel 4 */
    timer_channel_tau0_5,     /**< Timer Array Unit 0, Channel 5 */
    timer_channel_tau0_6,     /**< Timer Array Unit 0, Channel 6 */
    timer_channel_tau0_7,     /**< Timer Array Unit 0, Channel 7 */
    timer_channel_tau1_0,     /**< Timer Array Unit 1, Channel 0 */
    timer_channel_tau1_1,     /**< Timer Array Unit 1, Channel 1 */
    timer_channel_tau1_2,     /**< Timer Array Unit 1, Channel 2 */
    timer_channel_tau1_3,     /**< Timer Array Unit 1, Channel 3 */
    timer_channel_interval    /**< Interval Timer */
} t_timer_channel;

/**
 * @brief Enumeration for ADC channels.
 *        ADC_ADPC and ADC_ADS suggest channel selection.
 *        Assuming 8 channels based on typical Renesas F1x series.
 */
typedef enum
{
    adc_channel_0 = 0, /**< Analog input channel 0 */
    adc_channel_1,     /**< Analog input channel 1 */
    adc_channel_2,     /**< Analog input channel 2 */
    adc_channel_3,     /**< Analog input channel 3 */
    adc_channel_4,     /**< Analog input channel 4 */
    adc_channel_5,     /**< Analog input channel 5 */
    adc_channel_6,     /**< Analog input channel 6 */
    adc_channel_7      /**< Analog input channel 7 */
} t_adc_channel;

/**
 * @brief Enumeration for ADC operating modes.
 */
typedef enum
{
    adc_mode_single = 0,      /**< Single conversion mode */
    adc_mode_continuous       /**< Continuous conversion mode */
} t_adc_mode_t;

/**
 * @brief Enumeration for tick time periods for the Time-Triggered OS.
 */
typedef enum
{
    tick_time_1ms = 0,  /**< 1 millisecond tick */
    tick_time_10ms,     /**< 10 milliseconds tick */
    tick_time_100ms,    /**< 100 milliseconds tick */
    tick_time_1s        /**< 1 second tick */
} t_tick_time;

/**
 * @brief Enumeration for DAC channels.
 *        Placeholder, as no DAC registers are defined in register_json.
 */
// typedef enum {
//     dac_channel_0 = 0,
//     // Add more DAC channels if supported
// } dac_channel_t;


/**
 * @brief Enumeration for DMA Source Trigger Selectors.
 *        These map to values for DTC_TRGSRx registers.
 *        Specific values would depend on the MCU's TRGSR bit definitions.
 */
typedef enum
{
    dtc_trigger_source_intc = 0x01, // Example: INT_C trigger
    dtc_trigger_source_adc = 0x02,  // Example: ADC end of conversion trigger
    dtc_trigger_source_tau0_0 = 0x03, // Example: TAU0 Channel 0 compare match
    // ... add more as per actual MCU datasheet for DTC_TRGSR registers
} t_dtc_trigger_source;

/**
 * @brief Generic type for TX mode.
 *        Placeholder, as no WiFi registers are defined in register_json.
 */
// typedef enum {
//     tx_mode_normal = 0,
//     tx_mode_power_save
// } t_tx_mode;


// --- Peripheral Register Definitions ---

// GPIO Registers (8-bit)
#define GPIO_P0        (*((volatile tbyte *)0xFF00)) /**< Port Register 0 */
#define GPIO_P1        (*((volatile tbyte *)0xFF01)) /**< Port Register 1 */
#define GPIO_P2        (*((volatile tbyte *)0xFF02)) /**< Port Register 2 */
#define GPIO_P3        (*((volatile tbyte *)0xFF03)) /**< Port Register 3 */
#define GPIO_P4        (*((volatile tbyte *)0xFF04)) /**< Port Register 4 */
#define GPIO_P5        (*((volatile tbyte *)0xFF05)) /**< Port Register 5 */
#define GPIO_P6        (*((volatile tbyte *)0xFF06)) /**< Port Register 6 */
#define GPIO_P7        (*((volatile tbyte *)0xFF07)) /**< Port Register 7 */
#define GPIO_P12       (*((volatile tbyte *)0xFF0C)) /**< Port Register 12 */
#define GPIO_P13       (*((volatile tbyte *)0xFF0D)) /**< Port Register 13 */
#define GPIO_P14       (*((volatile tbyte *)0xFF0E)) /**< Port Register 14 */

#define GPIO_PM0       (*((volatile tbyte *)0xFF20)) /**< Port Mode Register 0 */
#define GPIO_PM1       (*((volatile tbyte *)0xFF21)) /**< Port Mode Register 1 */
#define GPIO_PM2       (*((volatile tbyte *)0xFF22)) /**< Port Mode Register 2 */
#define GPIO_PM3       (*((volatile tbyte *)0xFF23)) /**< Port Mode Register 3 */
#define GPIO_PM4       (*((volatile tbyte *)0xFF24)) /**< Port Mode Register 4 */
#define GPIO_PM5       (*((volatile tbyte *)0xFF25)) /**< Port Mode Register 5 */
#define GPIO_PM6       (*((volatile tbyte *)0xFF26)) /**< Port Mode Register 6 */
#define GPIO_PM7       (*((volatile tbyte *)0xFF27)) /**< Port Mode Register 7 */
#define GPIO_PM12      (*((volatile tbyte *)0xFF2C)) /**< Port Mode Register 12 */
#define GPIO_PM14      (*((volatile tbyte *)0xFF2E)) /**< Port Mode Register 14 */
#define GPIO_PM13      (*((volatile tbyte *)0xF002D)) /**< Port Mode Register 13 */

#define GPIO_PIM0      (*((volatile tbyte *)0xFF34)) /**< Port Input Mode Register 0 */
#define GPIO_PIM1      (*((volatile tbyte *)0xFF35)) /**< Port Input Mode Register 1 */

#define GPIO_POM0      (*((volatile tbyte *)0xFF38)) /**< Port Output Mode Register 0 */
#define GPIO_POM1      (*((volatile tbyte *)0xFF39)) /**< Port Output Mode Register 1 */

#define GPIO_PMC0      (*((volatile tbyte *)0xFF3C)) /**< Port Mode Control Register 0 */
#define GPIO_PMC1      (*((volatile tbyte *)0xFF3D)) /**< Port Mode Control Register 1 */
#define GPIO_PMC13     (*((volatile tbyte *)0xF003D)) /**< Port Mode Control Register 13 */

#define GPIO_PPR       (*((volatile tbyte *)0xF0043)) /**< Port Pin Read Register */

#define GPIO_PIOR0     (*((volatile tbyte *)0xF02C0)) /**< Peripheral I/O Redirection Register 0 */
#define GPIO_PIOR1     (*((volatile tbyte *)0xF02C1)) /**< Peripheral I/O Redirection Register 1 */
#define GPIO_PIOR2     (*((volatile tbyte *)0xF02C2)) /**< Peripheral I/O Redirection Register 2 */
#define GPIO_PIOR3     (*((volatile tbyte *)0xF02C3)) /**< Peripheral I/O Redirection Register 3 */
#define GPIO_GDIDIS    (*((volatile tbyte *)0xF02C4)) /**< Global Digital Input Disable Register */


// ADC Registers (8-bit and 16-bit)
#define ADC_ADM0       (*((volatile tbyte *)0xFF30)) /**< A/D Converter Mode Register 0 */
#define ADC_ADS        (*((volatile tbyte *)0xFF31)) /**< A/D Channel Select Register */
#define ADC_ADCRH      (*((volatile tbyte *)0xFF32)) /**< A/D Converter Result Register High */
#define ADC_ADCRL      (*((volatile tbyte *)0xFF33)) /**< A/D Converter Result Register Low */
#define ADC_ADMK       (*((volatile tbyte *)0xFF50)) /**< A/D Interrupt Mask Register */
#define ADC_PMC        (*((volatile tbyte *)0xFF9B)) /**< Analog Input Setting Register */
#define ADC_ADPC       (*((volatile tbyte *)0xFF9C)) /**< A/D Port Configuration Register */
#define ADC_ADCS       (*((volatile tbyte *)0xF0040)) /**< A/D Converter Start Register */


// Interrupt/Priority Registers (8-bit)
#define INT_NFEN0      (*((volatile tbyte *)0xFF40)) /**< Noise Filter Enable Register 0 */
#define INT_PCMK       (*((volatile tbyte *)0xFF54)) /**< PC Interrupt Mask Register */

#define INT_PR00       (*((volatile tbyte *)0xFF78)) /**< Interrupt Priority Register 00 */
#define INT_PR01       (*((volatile tbyte *)0xFF79)) /**< Interrupt Priority Register 01 */
#define INT_PR02       (*((volatile tbyte *)0xFF7A)) /**< Interrupt Priority Register 02 */
#define INT_PR03       (*((volatile tbyte *)0xFF7B)) /**< Interrupt Priority Register 03 */
#define INT_PR04       (*((volatile tbyte *)0xFF7C)) /**< Interrupt Priority Register 04 */
#define INT_PR05       (*((volatile tbyte *)0xFF7D)) /**< Interrupt Priority Register 05 */
#define INT_PR06       (*((volatile tbyte *)0xFF7E)) /**< Interrupt Priority Register 06 */
#define INT_PR07       (*((volatile tbyte *)0xFF7F)) /**< Interrupt Priority Register 07 */
#define INT_PR10       (*((volatile tbyte *)0xFF80)) /**< Interrupt Priority Register 10 */
#define INT_PR11       (*((volatile tbyte *)0xFF81)) /**< Interrupt Priority Register 11 */
#define INT_PR12       (*((volatile tbyte *)0xFF82)) /**< Interrupt Priority Register 12 */
#define INT_PR13       (*((volatile tbyte *)0xFF83)) /**< Interrupt Priority Register 13 */
#define INT_PR14       (*((volatile tbyte *)0xFF84)) /**< Interrupt Priority Register 14 */
#define INT_PR15       (*((volatile tbyte *)0xFF85)) /**< Interrupt Priority Register 15 */
#define INT_PR16       (*((volatile tbyte *)0xFF86)) /**< Interrupt Priority Register 16 */
#define INT_PR17       (*((volatile tbyte *)0xFF87)) /**< Interrupt Priority Register 17 */
#define INT_PR20       (*((volatile tbyte *)0xFF88)) /**< Interrupt Priority Register 20 */
#define INT_PR21       (*((volatile tbyte *)0xFF89)) /**< Interrupt Priority Register 21 */
#define INT_PR22       (*((volatile tbyte *)0xFF8A)) /**< Interrupt Priority Register 22 */
#define INT_PR23       (*((volatile tbyte *)0xFF8B)) /**< Interrupt Priority Register 23 */
#define INT_PR24       (*((volatile tbyte *)0xFF8C)) /**< Interrupt Priority Register 24 */
#define INT_PR25       (*((volatile tbyte *)0xFF8D)) /**< Interrupt Priority Register 25 */
#define INT_PR26       (*((volatile tbyte *)0xFF8E)) /**< Interrupt Priority Register 26 */
#define INT_PR27       (*((volatile tbyte *)0xFF8F)) /**< Interrupt Priority Register 27 */


// Timer Array Unit 0 (TAU0) Registers (8-bit and 16-bit)
#define TIM0_NFEN1     (*((volatile tbyte *)0xFF41)) /**< Noise Filter Enable Register 1 */
#define TIM0_TIS       (*((volatile tbyte *)0xFF43)) /**< Timer Array Unit 0 Input Switch Control Register */

#define TIM0_TMMK00    (*((volatile tbyte *)0xFF48)) /**< Timer Mask Register 00 */
#define TIM0_TMMK01    (*((volatile tbyte *)0xFF49)) /**< Timer Mask Register 01 */
#define TIM0_TMMK02    (*((volatile tbyte *)0xFF4A)) /**< Timer Mask Register 02 */
#define TIM0_TMMK03    (*((volatile tbyte *)0xFF4B)) /**< Timer Mask Register 03 */
#define TIM0_TMMK04    (*((volatile tbyte *)0xFF4C)) /**< Timer Mask Register 04 */
#define TIM0_TMMK05    (*((volatile tbyte *)0xFF4D)) /**< Timer Mask Register 05 */
#define TIM0_TMMK06    (*((volatile tbyte *)0xFF4E)) /**< Timer Mask Register 06 */
#define TIM0_TMMK07    (*((volatile tbyte *)0xFF4F)) /**< Timer Mask Register 07 */

#define TIM0_TMR00     (*((volatile tbyte *)0xFF90)) /**< Timer Mode Register 00 */
#define TIM0_TMR01     (*((volatile tbyte *)0xFF91)) /**< Timer Mode Register 01 */
#define TIM0_TMR02     (*((volatile tbyte *)0xFF92)) /**< Timer Mode Register 02 */
#define TIM0_TMR03     (*((volatile tbyte *)0xFF93)) /**< Timer Mode Register 03 */
#define TIM0_TMR04     (*((volatile tbyte *)0xFF94)) /**< Timer Mode Register 04 */
#define TIM0_TMR05     (*((volatile tbyte *)0xFF95)) /**< Timer Mode Register 05 */
#define TIM0_TMR06     (*((volatile tbyte *)0xFF96)) /**< Timer Mode Register 06 */
#define TIM0_TMR07     (*((volatile tbyte *)0xFF97)) /**< Timer Mode Register 07 */

#define TIM0_TPS       (*((volatile tbyte *)0xFF98)) /**< Timer Clock Select Register 0 */

#define TIM0_TCR00     (*((volatile tword *)0xF0100)) /**< Timer Count Register 00 */
#define TIM0_TCR01     (*((volatile tword *)0xF0102)) /**< Timer Count Register 01 */
#define TIM0_TCR02     (*((volatile tword *)0xF0104)) /**< Timer Count Register 02 */
#define TIM0_TCR03     (*((volatile tword *)0xF0106)) /**< Timer Count Register 03 */
#define TIM0_TCR04     (*((volatile tword *)0xF0108)) /**< Timer Count Register 04 */
#define TIM0_TCR05     (*((volatile tword *)0xF010A)) /**< Timer Count Register 05 */
#define TIM0_TCR06     (*((volatile tword *)0xF010C)) /**< Timer Count Register 06 */
#define TIM0_TCR07     (*((volatile tword *)0xF010E)) /**< Timer Count Register 07 */

#define TIM0_TDR00     (*((volatile tword *)0xF0110)) /**< Timer Data Register 00 */
#define TIM0_TDR01     (*((volatile tword *)0xF0112)) /**< Timer Data Register 01 */
#define TIM0_TDR02     (*((volatile tword *)0xF0114)) /**< Timer Data Register 02 */
#define TIM0_TDR03     (*((volatile tword *)0xF0116)) /**< Timer Data Register 03 */
#define TIM0_TDR04     (*((volatile tword *)0xF0118)) /**< Timer Data Register 04 */
#define TIM0_TDR05     (*((volatile tword *)0xF011A)) /**< Timer Data Register 05 */
#define TIM0_TDR06     (*((volatile tword *)0xF011C)) /**< Timer Data Register 06 */
#define TIM0_TDR07     (*((volatile tword *)0xF011E)) /**< Timer Data Register 07 */

#define TIM0_TSR00     (*((volatile tbyte *)0xF0120)) /**< Timer Status Register 00 */
#define TIM0_TSR01     (*((volatile tbyte *)0xF0121)) /**< Timer Status Register 01 */
#define TIM0_TSR02     (*((volatile tbyte *)0xF0122)) /**< Timer Status Register 02 */
#define TIM0_TSR03     (*((volatile tbyte *)0xF0123)) /**< Timer Status Register 03 */
#define TIM0_TSR04     (*((volatile tbyte *)0xF0124)) /**< Timer Status Register 04 */
#define TIM0_TSR05     (*((volatile tbyte *)0xF0125)) /**< Timer Status Register 05 */
#define TIM0_TSR06     (*((volatile tbyte *)0xF0126)) /**< Timer Status Register 06 */
#define TIM0_TSR07     (*((volatile tbyte *)0xF0127)) /**< Timer Status Register 07 */

#define TIM0_TS        (*((volatile tbyte *)0xF0128)) /**< Timer Channel Start Register 0 */
#define TIM0_TT        (*((volatile tbyte *)0xF0129)) /**< Timer Channel Stop Register 0 */
#define TIM0_TE        (*((volatile tbyte *)0xF012A)) /**< Timer Channel Enable Status Register 0 */
#define TIM0_TO        (*((volatile tbyte *)0xF012C)) /**< Timer Output Register 0 */
#define TIM0_TOE       (*((volatile tbyte *)0xF012D)) /**< Timer Output Enable Register 0 */
#define TIM0_TOL       (*((volatile tbyte *)0xF012E)) /**< Timer Output Level Register 0 */
#define TIM0_TOM       (*((volatile tbyte *)0xF012F)) /**< Timer Output Mode Register 0 */

// Timer Array Unit 1 (TAU1) Registers (8-bit and 16-bit)
#define TIM1_TMR10     (*((volatile tbyte *)0xF0130)) /**< Timer Mode Register 10 */
#define TIM1_TMR11     (*((volatile tbyte *)0xF0131)) /**< Timer Mode Register 11 */
#define TIM1_TMR12     (*((volatile tbyte *)0xF0132)) /**< Timer Mode Register 12 */
#define TIM1_TMR13     (*((volatile tbyte *)0xF0133)) /**< Timer Mode Register 13 */

#define TIM1_TPS       (*((volatile tbyte *)0xF0138)) /**< Timer Clock Select Register 1 */

#define TIM1_TCR10     (*((volatile tword *)0xF0140)) /**< Timer Count Register 10 */
#define TIM1_TCR11     (*((volatile tword *)0xF0142)) /**< Timer Count Register 11 */
#define TIM1_TCR12     (*((volatile tword *)0xF0144)) /**< Timer Count Register 12 */
#define TIM1_TCR13     (*((volatile tword *)0xF0146)) /**< Timer Count Register 13 */

#define TIM1_TDR10     (*((volatile tword *)0xF0150)) /**< Timer Data Register 10 */
#define TIM1_TDR11     (*((volatile tword *)0xF0152)) /**< Timer Data Register 11 */
#define TIM1_TDR12     (*((volatile tword *)0xF0154)) /**< Timer Data Register 12 */
#define TIM1_TDR13     (*((volatile tword *)0xF0156)) /**< Timer Data Register 13 */

#define TIM1_TSR10     (*((volatile tbyte *)0xF0160)) /**< Timer Status Register 10 */
#define TIM1_TSR11     (*((volatile tbyte *)0xF0161)) /**< Timer Status Register 11 */
#define TIM1_TSR12     (*((volatile tbyte *)0xF0162)) /**< Timer Status Register 12 */
#define TIM1_TSR13     (*((volatile tbyte *)0xF0163)) /**< Timer Status Register 13 */

#define TIM1_TS        (*((volatile tbyte *)0xF0168)) /**< Timer Channel Start Register 1 */
#define TIM1_TT        (*((volatile tbyte *)0xF0169)) /**< Timer Channel Stop Register 1 */
#define TIM1_TE        (*((volatile tbyte *)0xF016A)) /**< Timer Channel Enable Status Register 1 */
#define TIM1_TO        (*((volatile tbyte *)0xF016C)) /**< Timer Output Register 1 */
#define TIM1_TOE       (*((volatile tbyte *)0xF016D)) /**< Timer Output Enable Register 1 */
#define TIM1_TOL       (*((volatile tbyte *)0xF016E)) /**< Timer Output Level Register 1 */
#define TIM1_TOM       (*((volatile tbyte *)0xF016F)) /**< Timer Output Mode Register 1 */


// Interval Timer Registers (8-bit and 16-bit)
#define INTL_TIM_ITMC  (*((volatile tbyte *)0xF0048)) /**< Interval Timer Control Register */
#define INTL_TIM_ITMF  (*((volatile tbyte *)0xF0049)) /**< Interval Timer Flag Register */
#define INTL_TIM_ITCS  (*((volatile tbyte *)0xF0080)) /**< IT Timer Select Register */
#define INTL_TIM_ITCL  (*((volatile tbyte *)0xF0081)) /**< IT Count Register Low */
#define INTL_TIM_ITCH  (*((volatile tbyte *)0xF0082)) /**< IT Count Register High */


// RCC (Clock Control) Registers (8-bit)
#define RCC_CKC        (*((volatile tbyte *)0xFF9A)) /**< System Clock Control Register */
#define RCC_OSMC       (*((volatile tbyte *)0xFF9D)) /**< Subsystem Clock Supply Mode Control Register */
#define RCC_CMC        (*((volatile tbyte *)0xFF9E)) /**< Clock Operation Mode Control Register */
#define RCC_CSC        (*((volatile tbyte *)0xFF9F)) /**< Clock Operation Status Control Register */
#define RCC_OSTS       (*((volatile tbyte *)0xFFA0)) /**< Oscillation Stabilization Time Select Register */
#define RCC_OSTC       (*((volatile tbyte *)0xFFA1)) /**< Oscillation Stabilization Time Counter Status Register */
#define RCC_HOCODIV    (*((volatile tbyte *)0xFFA2)) /**< High-speed On-chip Oscillator Frequency Select Register */
#define RCC_HIOTRM     (*((volatile tbyte *)0xFFA3)) /**< High-speed On-chip Oscillator Trimming Register */
#define RCC_PER0       (*((volatile tbyte *)0xF0000)) /**< Peripheral Enable Register 0 */
#define RCC_PER1       (*((volatile tbyte *)0xF0001)) /**< Peripheral Enable Register 1 */
#define RCC_PCKD       (*((volatile tbyte *)0xF0042)) /**< Port Clock Output Division Register */
#define RCC_HOCOFC     (*((volatile tbyte *)0xF0078)) /**< HOCO Frequency Control Register */


// WDT (Watchdog Timer) Registers (8-bit)
#define WDT_WDM        (*((volatile tbyte *)0xFFC0)) /**< Watchdog Timer Mode Register */
#define WDT_WTT        (*((volatile tbyte *)0xFFC1)) /**< Watchdog Timer Timer Register */


// DTC (DMA) Registers (8-bit and 16-bit)
#define DTC_DTCAD      (*((volatile tword *)0xFFD0)) /**< DTC Transfer Data Address Register */
#define DTC_DTSAD      (*((volatile tword *)0xFFD2)) /**< DTC Transfer Start Address Register */
#define DTC_DRS        (*((volatile tbyte *)0xFFD4)) /**< DTC Transfer Control Register */
#define DTC_DTCD       (*((volatile tword *)0xFFD6)) /**< DTC Transfer Data Register */
#define DTC_DTCNT      (*((volatile tword *)0xFFD8)) /**< DTC Transfer Count Register */
#define DTC_TRGSR0     (*((volatile tbyte *)0xF0070)) /**< DTC Trigger Select Register 0 */
#define DTC_TRGSR1     (*((volatile tbyte *)0xF0071)) /**< DTC Trigger Select Register 1 */
#define DTC_TRGSR2     (*((volatile tbyte *)0xF0072)) /**< DTC Trigger Select Register 2 */
#define DTC_TRGSR3     (*((volatile tbyte *)0xF0073)) /**< DTC Trigger Select Register 3 */


// Serial Array Unit 0 (SAU0) Registers (8-bit and 16-bit)
#define SAU0_STMK0     (*((volatile tbyte *)0xFF58)) /**< Serial Interrupt Mask Register Transmit 0 */
#define SAU0_STMK1     (*((volatile tbyte *)0xFF59)) /**< Serial Interrupt Mask Register Transmit 1 */
#define SAU0_STMK2     (*((volatile tbyte *)0xFF5A)) /**< Serial Interrupt Mask Register Transmit 2 */
#define SAU0_STMK3     (*((volatile tbyte *)0xFF5B)) /**< Serial Interrupt Mask Register Transmit 3 */

#define SAU0_SRMK0     (*((volatile tbyte *)0xFF60)) /**< Serial Interrupt Mask Register Receive 0 */
#define SAU0_SRMK1     (*((volatile tbyte *)0xFF61)) /**< Serial Interrupt Mask Register Receive 1 */
#define SAU0_SRMK2     (*((volatile tbyte *)0xFF62)) /**< Serial Interrupt Mask Register Receive 2 */
#define SAU0_SRMK3     (*((volatile tbyte *)0xFF63)) /**< Serial Interrupt Mask Register Receive 3 */

#define SAU0_SGMK0     (*((volatile tbyte *)0xFF68)) /**< Serial Interrupt Mask Register Error 0 */
#define SAU0_SGMK1     (*((volatile tbyte *)0xFF69)) /**< Serial Interrupt Mask Register Error 1 */
#define SAU0_SGMK2     (*((volatile tbyte *)0xFF6A)) /**< Serial Interrupt Mask Register Error 2 */
#define SAU0_SGMK3     (*((volatile tbyte *)0xFF6B)) /**< Serial Interrupt Mask Register Error 3 */

#define SAU0_SPS0      (*((volatile tbyte *)0xF00B0)) /**< Serial Peripheral Select Register 0 */
#define SAU0_SPS1      (*((volatile tbyte *)0xF00B1)) /**< Serial Peripheral Select Register 1 */

#define SAU0_SIR00     (*((volatile tword *)0xF00B2)) /**< Serial Input Register 00 */
#define SAU0_SIR01     (*((volatile tword *)0xF00B3)) /**< Serial Input Register 01 */
#define SAU0_SIR02     (*((volatile tword *)0xF00B4)) /**< Serial Input Register 02 */
#define SAU0_SIR03     (*((volatile tword *)0xF00B5)) /**< Serial Input Register 03 */

#define SAU0_SMR00     (*((volatile tword *)0xF00B8)) /**< Serial Mode Register 00 */
#define SAU0_SMR01     (*((volatile tword *)0xF00B9)) /**< Serial Mode Register 01 */
#define SAU0_SMR02     (*((volatile tword *)0xF00BA)) /**< Serial Mode Register 02 */
#define SAU0_SMR03     (*((volatile tword *)0xF00BB)) /**< Serial Mode Register 03 */

#define SAU0_SCR00     (*((volatile tword *)0xF00C0)) /**< Serial Control Register 00 */
#define SAU0_SCR01     (*((volatile tword *)0xF00C1)) /**< Serial Control Register 01 */
#define SAU0_SCR02     (*((volatile tword *)0xF00C2)) /**< Serial Control Register 02 */
#define SAU0_SCR03     (*((volatile tword *)0xF00C3)) /**< Serial Control Register 03 */

#define SAU0_SDR00     (*((volatile tword *)0xF00C8)) /**< Serial Data Register 00 */
#define SAU0_SDR01     (*((volatile tword *)0xF00C9)) /**< Serial Data Register 01 */
#define SAU0_SDR02     (*((volatile tword *)0xF00CA)) /**< Serial Data Register 02 */
#define SAU0_SDR03     (*((volatile tword *)0xF00CB)) /**< Serial Data Register 03 */

#define SAU0_SSR00     (*((volatile tword *)0xF00D0)) /**< Serial Status Register 00 */
#define SAU0_SSR01     (*((volatile tword *)0xF00D1)) /**< Serial Status Register 01 */
#define SAU0_SSR02     (*((volatile tword *)0xF00D2)) /**< Serial Status Register 02 */
#define SAU0_SSR03     (*((volatile tword *)0xF00D3)) /**< Serial Status Register 03 */

#define SAU0_SO0       (*((volatile tbyte *)0xF00D4)) /**< Serial Output Register 0 */
#define SAU0_SOE0      (*((volatile tbyte *)0xF00D5)) /**< Serial Output Enable Register 0 */
#define SAU0_SO00      (*((volatile tbyte *)0xF00D6)) /**< Serial Output Register 00 */
#define SAU0_SO01      (*((volatile tbyte *)0xF00D7)) /**< Serial Output Register 01 */
#define SAU0_SO02      (*((volatile tbyte *)0xF00D8)) /**< Serial Output Register 02 */
#define SAU0_SO03      (*((volatile tbyte *)0xF00D9)) /**< Serial Output Register 03 */

#define SAU0_SSE0      (*((volatile tbyte *)0xF00DA)) /**< Serial Stop Enable Register 0 */
#define SAU0_SS0       (*((volatile tbyte *)0xF00DC)) /**< Serial Start Register 0 */
#define SAU0_ST0       (*((volatile tbyte *)0xF00DD)) /**< Serial Stop Register 0 */
#define SAU0_CRC0      (*((volatile tword *)0xF00DE)) /**< Serial CRC Register 0 */
#define SAU0_PFLG      (*((volatile tbyte *)0xF00DF)) /**< Serial Peripheral Flag Register */


// I2C Registers (8-bit)
#define I2C_IICAMK0    (*((volatile tbyte *)0xFF70)) /**< IIC Interrupt Mask Register 0 */
#define I2C_IICAMK1    (*((volatile tbyte *)0xFF71)) /**< IIC Interrupt Mask Register 1 */
#define I2C_IICAMK2    (*((volatile tbyte *)0xFF72)) /**< IIC Interrupt Mask Register 2 */
#define I2C_IICAMK3    (*((volatile tbyte *)0xFF73)) /**< IIC Interrupt Mask Register 3 */

#define I2C_IICCTL0    (*((volatile tbyte *)0xF0047)) /**< IIC Control Register 0 */
#define I2C_IICCTL00   (*((volatile tbyte *)0xF00E0)) /**< IIC Control Register 00 */
#define I2C_IICCTL01   (*((volatile tbyte *)0xF00E1)) /**< IIC Control Register 01 */

#define I2C_IICWL0     (*((volatile tbyte *)0xF00E2)) /**< IIC Wait Time Low Register 0 */
#define I2C_IICWH0     (*((volatile tbyte *)0xF00E3)) /**< IIC Wait Time High Register 0 */

#define I2C_SVA0       (*((volatile tbyte *)0xF00E4)) /**< IIC Slave Address Register 0 */
#define I2C_AM0        (*((volatile tbyte *)0xF00E5)) /**< IIC Address Match Register 0 */


// Flash Registers (8-bit)
#define FLASH_FLAPL    (*((volatile tbyte *)0xF0074)) /**< Flash Access Protection Low Register */
#define FLASH_FLAPH    (*((volatile tbyte *)0xF0075)) /**< Flash Access Protection High Register */
#define FLASH_FLARS    (*((volatile tbyte *)0xF0076)) /**< Flash Access Range Start Register */
#define FLASH_FLARE    (*((volatile tbyte *)0xF0077)) /**< Flash Access Range End Register */
#define FLASH_FLSEC    (*((volatile tbyte *)0xF00EE)) /**< Flash Security Register */
#define FLASH_FLRESD   (*((volatile tbyte *)0xF00EF)) /**< Flash Read-Out Protection Register */


// RTC Registers (8-bit and 16-bit) - No API functions defined in API.json, for reference only
#define RTC_RTCRDY     (*((volatile tbyte *)0xF0044)) /**< RTC Ready Register */
#define RTC_RTCC0      (*((volatile tbyte *)0xF0045)) /**< RTC Control Register 0 */
#define RTC_RTCC1      (*((volatile tbyte *)0xF0046)) /**< RTC Control Register 1 */
#define RTC_RTCL       (*((volatile tword *)0xF004B)) /**< RTC Count Register Low */
#define RTC_RTCH       (*((volatile tword *)0xF004C)) /**< RTC Count Register High */


// Comparator Registers (8-bit) - No API functions defined in API.json, for reference only
#define COMP_COMPM     (*((volatile tbyte *)0xF0060)) /**< Comparator Mode Register */


// LVD (Low Voltage Detection) Registers (8-bit)
#define LVD_LVDVDDF    (*((volatile tbyte *)0xF0061)) /**< LVD Flag Register */
#define LVD_LVDVDDCR   (*((volatile tbyte *)0xF0062)) /**< LVD Control Register */


// CRC Registers (8-bit) - No API functions defined in API.json, for reference only
#define CRC_CRC0CTL    (*((volatile tbyte *)0xF007F)) /**< CRC Control Register 0 */
#define CRC_CRC0DR     (*((volatile tbyte *)0xF01F0)) /**< CRC Data Register 0 */
#define CRC_CRC0ED     (*((volatile tbyte *)0xF01F1)) /**< CRC End Register 0 */


// Peripheral Input Switch Control Registers (8-bit)
#define PERIPH_ISC     (*((volatile tbyte *)0xFF42)) /**< Input Switch Control Register */


// Key Interrupt Registers (8-bit) - No API functions defined in API.json, for reference only
#define KEYINT_KRM     (*((volatile tbyte *)0xF004A)) /**< Key Return Mode Register */


// --- Public API Function Prototypes (as per API.json) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // Defined in WDT category, but also part of MCU_Config_Init sequence
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
// ICU_GetFrequency returns float or tlong? API.json doesn't specify return type for GetFrequency. Assuming tlong for ticks.
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

// Internal_EEPROM not supported on this MCU (no specific registers found).
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

// MCAL_OUTPUT_BUZZER not supported on this MCU (no specific registers found).
// void BUZZER_OUTPUT_Init(tbyte buzzer_number);
// void BUZZER_OUTPUT_Start(tbyte NUMBER_BUZZER);
// void BUZZER_OUTPUT_Stop(tbyte NUMBER_BUZZER);

// WDT
// WDT_Reset prototype already declared in MCU CONFIG.
void WDT_Init(void);

// DAC not supported on this MCU (no specific registers found).
// typedef enum {
//     dac_channel_0,
//     // ... other DAC channels
// } dac_channel_t;
// void DAC_Init(dac_channel_t channel);
// void DAC_Enable(dac_channel_t channel);
// void DAC_Disable(dac_channel_t channel);
// void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue);

// I2S not supported on this MCU (no specific registers found).
// typedef enum { /* ... */ } I2S_Mode_t;
// typedef enum { /* ... */ } I2S_Standard_t;
// typedef enum { /* ... */ } I2S_DataFormat_t;
// typedef enum { /* ... */ } I2S_ChannelMode_t;
// void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size);
// void I2S_Enable(t_i2s_channel channel);
// void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length);
// void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length);

// MQTT Protocol not supported on this MCU directly via registers (requires external module/SDK).
// void MQTT_Init(void);
// void MQTT_Subscribe_Topic(void *handler_args, esp_event_base_t base, tslong event_id, void *event_data);
// void MQTT_Publish_Message(const tsbyte *message);
// void AZURE_IoT_Hub_Client_Init(void);
// void AZURE_Connection_Enable(void);

// HTTP Protocol not supported on this MCU directly via registers (requires external module/SDK).
// typedef struct httpd_req_t httpd_req_t; // Placeholder for httpd_req_t type
// typedef int esp_err_t; // Placeholder for esp_err_t type
// void HTTP_Get_Device_ID(char *device_id, size_t size);
// void HTTP_Server_Init(void);
// void HTTP_Server_Start(void);
// void HTTP_Server_Stop(void);
// void HTTP_Reset_SSID_PASSWORD(void);
// esp_err_t HTTP_Config_Handler(httpd_req_t *req);

// WiFi Driver not supported on this MCU directly via registers (requires external module/SDK).
// typedef enum { /* ... */ } t_tx_mode;
// typedef int tsword; // Placeholder for tsword
// void WiFi_Init(void);
// void WiFi_Connect(const tsbyte *ssid, const tsbyte *password);
// void WiFi_Enable(void);
// void WiFi_Disable(void);
// void WiFi_SetTxMode(t_tx_mode mode);
// tsword WiFi_Check_Connection(void);
// int WiFi_Check_Internet(void);

// DTC_driver
void DTC_Init(void);
void DTC_EnableSource(uint8_t source_id, uint8_t channel);
void DTC_DisableSource(uint8_t source_id);
void DTC_Start(void);
void DTC_Stop(void);
void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int);


#endif // MCAL_H