
// Chunk 1
#ifndef MCAL_H
#define MCAL_H

// Core device header
#include "stm32f401xc.h"  // Core device header as per rules for STM32F401RC
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h> // Included as per core_includes example in Rules.json
#include <stdio.h>  // Included as per core_includes example in Rules.json
#include <stdlib.h> // Included as per core_includes example in Rules.json
#include <math.h>   // Included as per core_includes example in Rules.json

// Data type definitions as per Rules.json
typedef uint8_t tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// MCU CONFIG types
typedef enum {
    Vsource_3V = 0, // Rule: 2V for 3V system voltage
    Vsource_5V      // Rule: 3.5V for 5V system voltage
} t_sys_volt;

// LVD types based on API.json and Rules.json LVD_requirements
typedef enum {
    Volt_0_5V = 0,
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

typedef enum {
    LVD_CHANNEL_DEFAULT = 0 // No specific LVD channels defined in inputs
} t_lvd_channel;

// UART types (placeholders as no specific values/details provided in rules/registers)
typedef enum {
    UART_CHANNEL_1 = 0, // Example channel, not defined in REGISTER_JSON
    UART_CHANNEL_2,
    UART_CHANNEL_3
} t_uart_channel;

typedef enum {
    UART_BAUD_9600 = 0,
    UART_BAUD_19200,
    UART_BAUD_115200
} t_uart_baud_rate;

typedef enum {
    UART_DATA_8BIT = 0,
    UART_DATA_9BIT
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

// I2C types (placeholders as no specific values/details provided in rules/registers)
typedef enum {
    I2C_CHANNEL_1 = 0, // Example channel, not defined in REGISTER_JSON
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum {
    I2C_CLK_STANDARD = 0, // 100 kHz
    I2C_CLK_FAST          // 400 kHz, Rule: Always use fast mode
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // Typically 7-bit or 10-bit address

typedef enum {
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum {
    I2C_DATA_8BIT = 0,
    I2C_DATA_10BIT
} t_i2c_datalength;

// SPI types (placeholders as no specific values/details provided in rules/registers)
typedef enum {
    SPI_CHANNEL_1 = 0, // Example channel, not defined in REGISTER_JSON
    SPI_CHANNEL_2,
    SPI_CHANNEL_3
} t_spi_channel;

typedef enum {
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE
} t_spi_mode;

typedef enum {
    SPI_CPOL_LOW = 0,  // Clock Phase as per rules
    SPI_CPOL_HIGH
} t_spi_cpol;

typedef enum {
    SPI_CPHA_1EDGE = 0, // Clock Polarity as per rules
    SPI_CPHA_2EDGE
} t_spi_cpha;

typedef enum {
    SPI_DFF_8BIT = 0, // Data Frame Format as per rules
    SPI_DFF_16BIT
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt types
typedef enum {
    EXT_INT_CHANNEL_0 = 0,  // EXTI lines 0-15 mapped to SYSCFG_EXTICR1-4
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
    EXT_INT_EDGE_RISING = 0, // Rising edge trigger
    EXT_INT_EDGE_FALLING,    // Falling edge trigger
    EXT_INT_EDGE_BOTH        // Both rising and falling edge trigger
} t_external_int_edge;

// GPIO types based on REGISTER_JSON
typedef enum {
    PORTA = 0, // Corresponds to GPIOA_ registers
    PORTB      // Corresponds to GPIOB_MODER (partial support)
    // Other ports like C, D, E, H are mentioned in SYSCFG_EXTICR, but no GPIOx_MODER or other GPIO registers are provided for them.
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
    GPIO_DIRECTION_INPUT = 0,
    GPIO_DIRECTION_OUTPUT,
    GPIO_DIRECTION_ANALOG,
    GPIO_DIRECTION_ALTERNATE_FUNCTION
} t_direction; // Return type for GPIO_Direction_get

// PWM types (placeholders as no specific values/details provided in rules/registers)
typedef enum {
    PWM_CHANNEL_1 = 0, // Example channel, not defined in REGISTER_JSON
    PWM_CHANNEL_2
} t_pwm_channel;

// ICU types (placeholders as no specific values/details provided in rules/registers)
typedef enum {
    ICU_CHANNEL_1 = 0, // Example channel, not defined in REGISTER_JSON
    ICU_CHANNEL_2
} t_icu_channel;

typedef enum {
    ICU_PRESCALER_1 = 0,
    ICU_PRESCALER_8,
    ICU_PRESCALER_64,
    ICU_PRESCALER_256
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer types (placeholders as no specific values/details provided in rules/registers)
typedef enum {
    TIMER_CHANNEL_1 = 0, // Example channel, not defined in REGISTER_JSON
    TIMER_CHANNEL_2
} t_timer_channel;

// ADC types (placeholders as no specific values/details provided in rules/registers)
typedef enum {
    ADC_CHANNEL_0 = 0, // Example channel, not defined in REGISTER_JSON
    ADC_CHANNEL_1
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE_CONVERSION = 0,
    ADC_MODE_CONTINUOUS
} t_adc_mode_t;

// TT types (placeholders for time-triggered scheduler)
typedef tword t_tick_time; // Time in milliseconds for TT_Init

// API function prototypes as defined in API.json
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
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Adjusted return type to void for action function

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
void UART_ClearFlag(t_uart_channel uart_channel); // Adjusted return type to void for action function

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
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Adjusted return type to void for action function

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
void SPI_ClearFlag(t_spi_channel spi_channel); // Adjusted return type to void for action function

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Adjusted return type to void for action function

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
void ICU_Updatefrequency(t_icu_channel icu_channel);
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Adjusted return type for 'Get' function
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Adjusted return type to void for action function

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Adjusted return type to void for action function

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void); // Adjusted return type to void for action function

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

// Chunk 2
/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File for STM32F401RC.
 *
 * This file contains the declarations for the MCAL API functions and
 * necessary type definitions for the STM32F401RC microcontroller.
 *
 * It includes core MCU header and standard C library headers as per
 * the coding rules.
 */

#ifndef MCAL_H
#define MCAL_H

// Core MCU header - deduced for STM32F401RC
#include "stm32f401xc.h"  // Core device header for STM32F401RC

// Standard C library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Data type definitions as per rules.json
#define tbyte  uint8_t
#define tword  uint16_t
#define tlong  uint32_t

// Macro for volatile register access
#define _VOLATILE_REGISTER(ADDR) ((volatile uint32_t*)(ADDR))

// --- MCU Register Addresses (from REGISTER_JSON) ---

// GPIOB Registers
#define GPIOB_OTYPER_ADDR      0x40020404U
#define GPIOB_OSPEEDR_ADDR     0x40020408U
#define GPIOB_PUPDR_ADDR       0x4002040CU
#define GPIOB_IDR_ADDR         0x40020410U
#define GPIOB_ODR_ADDR         0x40020414U
#define GPIOB_BSRR_ADDR        0x40020418U
#define GPIOB_LCKR_ADDR        0x4002041CU
#define GPIOB_AFRL_ADDR        0x40020420U
#define GPIOB_AFRH_ADDR        0x40020424U

// GPIOC Registers
#define GPIOC_MODER_ADDR       0x40020800U
#define GPIOC_OTYPER_ADDR      0x40020804U
#define GPIOC_OSPEEDR_ADDR     0x40020808U
#define GPIOC_PUPDR_ADDR       0x4002080CU
#define GPIOC_IDR_ADDR         0x40020810U
#define GPIOC_ODR_ADDR         0x40020814U
#define GPIOC_BSRR_ADDR        0x40020818U
#define GPIOC_LCKR_ADDR        0x4002081CU
#define GPIOC_AFRL_ADDR        0x40020820U
#define GPIOC_AFRH_ADDR        0x40020824U

// GPIOD Registers
#define GPIOD_MODER_ADDR       0x40020C00U
#define GPIOD_OTYPER_ADDR      0x40020C04U
#define GPIOD_OSPEEDR_ADDR     0x40020C08U
#define GPIOD_PUPDR_ADDR       0x40020C0CU
#define GPIOD_IDR_ADDR         0x40020C10U
#define GPIOD_ODR_ADDR         0x40020C14U
#define GPIOD_BSRR_ADDR        0x40020C18U
#define GPIOD_LCKR_ADDR        0x40020C1CU
#define GPIOD_AFRL_ADDR        0x40020C20U
#define GPIOD_AFRH_ADDR        0x40020C24U

// GPIOE Registers
#define GPIOE_MODER_ADDR       0x40021000U

// Note: GPIOB_MODER is not explicitly in JSON, but inferred from other GPIOB registers.
// However, the rule states "Use only what is given in the inputs".
// Therefore, only GPIOB_OTYPER, GPIOB_OSPEEDR, GPIOB_PUPDR are configurable as "gpio_config".
// For GPIOB_MODER, it will be treated as non-configurable via "gpio_config" if not explicitly mentioned.
//
// RE-EVALUATION: The 'gpio_config' function in the JSON lists 'GPIOB_OTYPER', 'GPIOB_OSPEEDR', 'GPIOB_PUPDR'
// but implies a general 'gpio_config' capability for the port.
// However, 'GPIOC_MODER', 'GPIOD_MODER', 'GPIOE_MODER' *are* listed.
// This means GPIOB's MODER is *not* provided. This is a critical constraint.
// I can only configure OTYPER, OSPEEDR, PUPDR for GPIOB, but not MODER.
// This is unusual for GPIOs, but I must follow the input strictly.
// If GPIOB_MODER is truly missing from the JSON, I cannot set its mode.
// I will include GPIOB_MODER_ADDR based on standard STM32 offset, but comment it.
// The strict interpretation of "Use only what is given" means I cannot invent GPIOB_MODER.
// Let's re-read "Only use what is given in the inputs below. Do not invent API functions, registers, or rules."
// This means if GPIOB_MODER is NOT in the JSON, I cannot use it.

// After careful re-reading, GPIOB_MODER is indeed MISSING from the register JSON.
// This implies that the mode for GPIOB cannot be configured via MCAL using provided info.
// I will adhere to this and only configure registers explicitly listed.
// This will mean `GPIO_Output_Init` and `GPIO_Input_Init` will not be fully functional for Port B.

// --- Enums and Type Definitions ---

/**
 * @brief Enumeration for system voltage levels.
 */
typedef enum {
    Vsource_3V = 0, ///< System voltage is 3V
    Vsource_5V      ///< System voltage is 5V
} t_sys_volt;

/**
 * @brief Enumeration for Low Voltage Detection (LVD) threshold levels.
 *        Levels are inferred from the rules.json example.
 */
typedef enum {
    Volt_0_5V = 0,
    Volt_1V,
    Volt_1_5V,
    Volt_2V,
    Volt_2_5V, // Added for completeness, assuming increment
    Volt_3V,
    Volt_3_5V,
    Volt_4V,
    Volt_4_5V,
    Volt_5V
} t_lvd_thrthresholdLevel;

/**
 * @brief Enumeration for GPIO port selection.
 */
typedef enum {
    PORTB = 0,
    PORTC,
    PORTD,
    PORTE
    // Note: Other ports (A, F, G, H, I, J, K) are not present in the given register JSON.
} t_port;

/**
 * @brief Enumeration for GPIO pin selection (0-15).
 */
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

/**
 * @brief Enumeration for GPIO direction.
 */
typedef enum {
    DIRECTION_INPUT = 0,
    DIRECTION_OUTPUT
} t_direction;

/**
 * @brief Enumeration for UART channel. Placeholder as no registers provided.
 */
typedef enum {
    UART_CHANNEL_1 = 0 // Placeholder
} t_uart_channel;

/**
 * @brief Enumeration for UART baud rate. Placeholder.
 */
typedef enum {
    UART_BAUD_9600 = 0 // Placeholder
} t_uart_baud_rate;

/**
 * @brief Enumeration for UART data length. Placeholder.
 */
typedef enum {
    UART_DATA_8BIT = 0 // Placeholder
} t_uart_data_length;

/**
 * @brief Enumeration for UART stop bit. Placeholder.
 */
typedef enum {
    UART_STOP_BIT_1 = 0 // Placeholder
} t_uart_stop_bit;

/**
 * @brief Enumeration for UART parity. Placeholder.
 */
typedef enum {
    UART_PARITY_NONE = 0 // Placeholder
} t_uart_parity;

/**
 * @brief Enumeration for I2C channel. Placeholder as no registers provided.
 */
typedef enum {
    I2C_CHANNEL_1 = 0 // Placeholder
} t_i2c_channel;

/**
 * @brief Enumeration for I2C clock speed. Placeholder.
 */
typedef enum {
    I2C_CLK_SPEED_FAST = 0 // Placeholder
} t_i2c_clk_speed;

/**
 * @brief Enumeration for I2C device address. Placeholder.
 */
typedef enum {
    I2C_DEV_ADDR_GENERIC = 0 // Placeholder
} t_i2c_device_address;

/**
 * @brief Enumeration for I2C acknowledge. Placeholder.
 */
typedef enum {
    I2C_ACK_ENABLE = 0 // Placeholder
} t_i2c_ack;

/**
 * @brief Enumeration for I2C data length. Placeholder.
 */
typedef enum {
    I2C_DATA_LEN_8BIT = 0 // Placeholder
} t_i2c_datalength;

/**
 * @brief Enumeration for SPI channel. Placeholder as no registers provided.
 */
typedef enum {
    SPI_CHANNEL_1 = 0 // Placeholder
} t_spi_channel;

/**
 * @brief Enumeration for SPI mode. Placeholder.
 */
typedef enum {
    SPI_MODE_MASTER = 0 // Placeholder
} t_spi_mode;

/**
 * @brief Enumeration for SPI Clock Polarity. Placeholder.
 */
typedef enum {
    SPI_CPOL_LOW = 0 // Placeholder
} t_spi_cpol;

/**
 * @brief Enumeration for SPI Clock Phase. Placeholder.
 */
typedef enum {
    SPI_CPHA_1EDGE = 0 // Placeholder
} t_spi_cpha;

/**
 * @brief Enumeration for SPI Data Frame Format. Placeholder.
 */
typedef enum {
    SPI_DFF_8BIT = 0 // Placeholder
} t_spi_dff;

/**
 * @brief Enumeration for SPI Bit Order. Placeholder.
 */
typedef enum {
    SPI_BIT_ORDER_MSB_FIRST = 0 // Placeholder
} t_spi_bit_order;

/**
 * @brief Enumeration for External Interrupt channel. Placeholder as no registers provided.
 */
typedef enum {
    EXTI_CHANNEL_0 = 0 // Placeholder
} t_external_int_channel;

/**
 * @brief Enumeration for External Interrupt edge. Placeholder.
 */
typedef enum {
    EXTI_EDGE_RISING = 0 // Placeholder
} t_external_int_edge;

/**
 * @brief Enumeration for PWM channel. Placeholder as no registers provided.
 */
typedef enum {
    PWM_CHANNEL_TIM1_CH1 = 0 // PA8,PE9 as per comments rule example. Placeholder.
} t_pwm_channel;

/**
 * @brief Enumeration for ICU channel. Placeholder as no registers provided.
 */
typedef enum {
    ICU_CHANNEL_1 = 0 // Placeholder
} t_icu_channel;

/**
 * @brief Enumeration for ICU prescaler. Placeholder.
 */
typedef enum {
    ICU_PRESCALER_1 = 0 // Placeholder
} t_icu_prescaller;

/**
 * @brief Enumeration for ICU edge. Placeholder.
 */
typedef enum {
    ICU_EDGE_RISING = 0 // Placeholder
} t_icu_edge;

/**
 * @brief Enumeration for Timer channel. Placeholder as no registers provided.
 */
typedef enum {
    TIMER_CHANNEL_1 = 0 // Placeholder
} t_timer_channel;

/**
 * @brief Enumeration for ADC channel. Placeholder as no registers provided.
 */
typedef enum {
    ADC_CHANNEL_0 = 0 // Placeholder
} t_adc_channel;

/**
 * @brief Enumeration for ADC mode. Placeholder.
 */
typedef enum {
    ADC_MODE_SINGLE = 0 // Placeholder
} t_adc_mode_t;

/**
 * @brief Enumeration for TT tick time. Placeholder.
 */
typedef enum {
    TT_TICK_1MS = 0 // Placeholder
} t_tick_time;

// --- API Function Declarations (from API.json) ---

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
void LVD_ClearFlag(t_lvd_channel lvd_channel); // t_lvd_channel not defined in API.json, will use int

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Return type deduced from example
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

// Chunk 3
#ifndef MCAL_H
#define MCAL_H

#include "stm32f401xc.h"  // Core device header for STM32F401RC (deduced from MCU NAME)
#include <stdint.h>       // Required for fixed-width integer types
#include <stdbool.h>      // Required for boolean type
// Other standard C includes (stddef.h, string.h, stdio.h, stdlib.h, math.h)
// are not strictly necessary for the defined APIs with current constraints.

/*
 * @brief  MCAL Data Type Definitions
 *         As per "data_type_definitions" rule.
 */
#define tbyte  uint8_t
#define tword  uint16_t
#define tlong  uint32_t

/*
 * @brief  MCAL Peripheral Register Definitions
 *         Pointers to volatile uint32_t for direct register access.
 *         Only registers explicitly listed in the REGISTER JSON are defined.
 */

// GPIOE Registers
#define GPIOE_OTYPER_REG    ((volatile uint32_t *)0x40021004) // GPIO port output type register for Port E.
#define GPIOE_OSPEEDR_REG   ((volatile uint32_t *)0x40021008) // GPIO port output speed register for Port E.
#define GPIOE_PUPDR_REG     ((volatile uint32_t *)0x4002100C) // GPIO port pull-up/pull-down register for Port E.
#define GPIOE_IDR_REG       ((volatile uint32_t *)0x40021010) // GPIO port input data register for Port E.
#define GPIOE_ODR_REG       ((volatile uint32_t *)0x40021014) // GPIO port output data register for Port E.
#define GPIOE_BSRR_REG      ((volatile uint32_t *)0x40021018) // GPIO port bit set/reset register for Port E.
#define GPIOE_LCKR_REG      ((volatile uint32_t *)0x4002101C) // GPIO port configuration lock register for Port E.
#define GPIOE_AFRL_REG      ((volatile uint32_t *)0x40021020) // GPIO alternate function low register for Port E (pins 0-7).
#define GPIOE_AFRH_REG      ((volatile uint32_t *)0x40021024) // GPIO alternate function high register for Port E (pins 8-15).

// GPIOH Registers
#define GPIOH_MODER_REG     ((volatile uint32_t *)0x40021C00) // GPIO port mode register for Port H.
#define GPIOH_OTYPER_REG    ((volatile uint32_t *)0x40021C04) // GPIO port output type register for Port H.
#define GPIOH_OSPEEDR_REG   ((volatile uint32_t *)0x40021C08) // GPIO port output speed register for Port H.
#define GPIOH_PUPDR_REG     ((volatile uint32_t *)0x40021C0C) // GPIO port pull-up/pull-down register for Port H.
#define GPIOH_IDR_REG       ((volatile uint32_t *)0x40021C10) // GPIO port input data register for Port H.
#define GPIOH_ODR_REG       ((volatile uint32_t *)0x40021C14) // GPIO port output data register for Port H.
#define GPIOH_BSRR_REG      ((volatile uint32_t *)0x40021C18) // GPIO port bit set/reset register for Port H.
#define GPIOH_LCKR_REG      ((volatile uint32_t *)0x40021C1C) // GPIO port configuration lock register for Port H.
#define GPIOH_AFRL_REG      ((volatile uint32_t *)0x40021C20) // GPIO alternate function low register for Port H (pins 0-7).
#define GPIOH_AFRH_REG      ((volatile uint32_t *)0x40021C24) // GPIO alternate function high register for Port H (pins 8-15).

// DMA1 Registers (No DMA APIs in API.json, but registers are defined in REGISTER JSON)
#define DMA1_LISR_REG       ((volatile uint32_t *)0x40026000) // DMA1 low interrupt status register.
#define DMA1_HISR_REG       ((volatile uint32_t *)0x40026004) // DMA1 high interrupt status register.
#define DMA1_LIFCR_REG      ((volatile uint32_t *)0x40026008) // DMA1 low interrupt flag clear register.
#define DMA1_HIFCR_REG      ((volatile uint32_t *)0x4002600C) // DMA1 high interrupt flag clear register.
#define DMA1_S0CR_REG       ((volatile uint32_t *)0x40026010) // DMA1 stream 0 configuration register.
#define DMA1_S0NDTR_REG     ((volatile uint32_t *)0x40026014) // DMA1 stream 0 number of data register.
#define DMA1_S0PAR_REG      ((volatile uint32_t *)0x40026018) // DMA1 stream 0 peripheral address register.
#define DMA1_S0M0AR_REG     ((volatile uint32_t *)0x4002601C) // DMA1 stream 0 memory 0 address register.
#define DMA1_S0M1AR_REG     ((volatile uint32_t *)0x40026020) // DMA1 stream 0 memory 1 address register.
#define DMA1_S0FCR_REG      ((volatile uint32_t *)0x40026024) // DMA1 stream 0 FIFO control register.
#define DMA1_S1CR_REG       ((volatile uint32_t *)0x40026028) // DMA1 stream 1 configuration register.
#define DMA1_S1NDTR_REG     ((volatile uint32_t *)0x4002602C) // DMA1 stream 1 number of data register.
#define DMA1_S1PAR_REG      ((volatile uint32_t *)0x40026030) // DMA1 stream 1 peripheral address register.
#define DMA1_S1M0AR_REG     ((volatile uint32_t *)0x40026034) // DMA1 stream 1 memory 0 address register.
#define DMA1_S1M1AR_REG     ((volatile uint32_t *)0x40026038) // DMA1 stream 1 memory 1 address register.
#define DMA1_S1FCR_REG      ((volatile uint32_t *)0x4002603C) // DMA1 stream 1 FIFO control register.
#define DMA1_S2CR_REG       ((volatile uint32_t *)0x40026040) // DMA1 stream 2 configuration register.
#define DMA1_S2NDTR_REG     ((volatile uint32_t *)0x40026044) // DMA1 stream 2 number of data register.
#define DMA1_S2PAR_REG      ((volatile uint32_t *)0x40026048) // DMA1 stream 2 peripheral address register.
#define DMA1_S2M0AR_REG     ((volatile uint32_t *)0x4002604C) // DMA1 stream 2 memory 0 address register.
#define DMA1_S2M1AR_REG     ((volatile uint32_t *)0x40026050) // DMA1 stream 2 memory 1 address register.
#define DMA1_S2FCR_REG      ((volatile uint32_t *)0x40026054) // DMA1 stream 2 FIFO control register.
#define DMA1_S3CR_REG       ((volatile uint32_t *)0x40026058) // DMA1 stream 3 configuration register.
#define DMA1_S3NDTR_REG     ((volatile uint32_t *)0x4002605C) // DMA1 stream 3 number of data register.
#define DMA1_S3PAR_REG      ((volatile uint32_t *)0x40026060) // DMA1 stream 3 peripheral address register.
#define DMA1_S3M0AR_REG     ((volatile uint32_t *)0x40026064) // DMA1 stream 3 memory 0 address register.
#define DMA1_S3M1AR_REG     ((volatile uint32_t *)0x40026068) // DMA1 stream 3 memory 1 address register.
#define DMA1_S3FCR_REG      ((volatile uint32_t *)0x4002606C) // DMA1 stream 3 FIFO control register.
#define DMA1_S4CR_REG       ((volatile uint32_t *)0x40026070) // DMA1 stream 4 configuration register.
#define DMA1_S4NDTR_REG     ((volatile uint32_t *)0x40026074) // DMA1 stream 4 number of data register.
#define DMA1_S4PAR_REG      ((volatile uint32_t *)0x40026078) // DMA1 stream 4 peripheral address register.
#define DMA1_S4M0AR_REG     ((volatile uint32_t *)0x4002607C) // DMA1 stream 4 memory 0 address register.
#define DMA1_S4M1AR_REG     ((volatile uint32_t *)0x40026080) // DMA1 stream 4 memory 1 address register.
#define DMA1_S4FCR_REG      ((volatile uint32_t *)0x40026084) // DMA1 stream 4 FIFO control register.
#define DMA1_S5CR_REG       ((volatile uint32_t *)0x40026088) // DMA1 stream 5 configuration register.
#define DMA1_S5NDTR_REG     ((volatile uint32_t *)0x4002608C) // DMA1 stream 5 number of data register.
#define DMA1_S5PAR_REG      ((volatile uint32_t *)0x40026090) // DMA1 stream 5 peripheral address register.
#define DMA1_S5M0AR_REG     ((volatile uint32_t *)0x40026094) // DMA1 stream 5 memory 0 address register.
#define DMA1_S5M1AR_REG     ((volatile uint32_t *)0x40026098) // DMA1 stream 5 memory 1 address register.
#define DMA1_S5FCR_REG      ((volatile uint32_t *)0x4002609C) // DMA1 stream 5 FIFO control register.
#define DMA1_S6CR_REG       ((volatile uint32_t *)0x400260A0) // DMA1 stream 6 configuration register.
#define DMA1_S6NDTR_REG     ((volatile uint32_t *)0x400260A4) // DMA1 stream 6 number of data register.
#define DMA1_S6PAR_REG      ((volatile uint32_t *)0x400260A8) // DMA1 stream 6 peripheral address register.


/*
 * @brief  Enumerations for API parameters
 */

// MCU CONFIG
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

// LVD
typedef enum {
    Volt_0_5V = 0, // Using 0_5V to avoid float in enum name
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

typedef enum {
    LVD_CHANNEL_NONE = 0 // Placeholder as no specific LVD channels defined in inputs
} t_lvd_channel;

// UART (Placeholder as no specific UART registers defined in inputs)
typedef enum {
    UART_CHANNEL_NONE = 0
} t_uart_channel;
typedef enum {
    UART_BAUD_RATE_NONE = 0
} t_uart_baud_rate;
typedef enum {
    UART_DATA_LENGTH_NONE = 0
} t_uart_data_length;
typedef enum {
    UART_STOP_BIT_NONE = 0
} t_uart_stop_bit;
typedef enum {
    UART_PARITY_NONE = 0
} t_uart_parity;

// I2C (Placeholder as no specific I2C registers defined in inputs)
typedef enum {
    I2C_CHANNEL_NONE = 0
} t_i2c_channel;
typedef enum {
    I2C_CLK_SPEED_NONE = 0
} t_i2c_clk_speed;
typedef enum {
    I2C_DEVICE_ADDRESS_NONE = 0
} t_i2c_device_address;
typedef enum {
    I2C_ACK_NONE = 0
} t_i2c_ack;
typedef enum {
    I2C_DATALENGTH_NONE = 0
} t_i2c_datalength;

// SPI (Placeholder as no specific SPI registers defined in inputs)
typedef enum {
    SPI_CHANNEL_NONE = 0
} t_spi_channel;
typedef enum {
    SPI_MODE_NONE = 0
} t_spi_mode;
typedef enum {
    SPI_CPOL_NONE = 0
} t_spi_cpol;
typedef enum {
    SPI_CPHA_NONE = 0
} t_spi_cpha;
typedef enum {
    SPI_DFF_NONE = 0
} t_spi_dff;
typedef enum {
    SPI_BIT_ORDER_NONE = 0
} t_spi_bit_order;

// External Interrupt (Placeholder as no specific EXTI registers defined in inputs)
typedef enum {
    EXTERNAL_INT_CHANNEL_NONE = 0
} t_external_int_channel;
typedef enum {
    EXTERNAL_INT_EDGE_NONE = 0
} t_external_int_edge;

// GPIO
typedef enum {
    GPIO_PORT_E = 0, // Port E (PE0-PE15) - MODER register NOT provided in JSON
    GPIO_PORT_H      // Port H (PH0-PH1) - MODER register IS provided in JSON
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

// PWM (Placeholder as no specific PWM registers defined in inputs)
typedef enum {
    PWM_CHANNEL_NONE = 0
} t_pwm_channel;

// ICU (Placeholder as no specific ICU registers defined in inputs)
typedef enum {
    ICU_CHANNEL_NONE = 0
} t_icu_channel;
typedef enum {
    ICU_PRESCALLER_NONE = 0
} t_icu_prescaller;
typedef enum {
    ICU_EDGE_NONE = 0
} t_icu_edge;

// Timer (Placeholder as no specific Timer registers defined in inputs)
typedef enum {
    TIMER_CHANNEL_NONE = 0
} t_timer_channel;

// ADC (Placeholder as no specific ADC registers defined in inputs)
typedef enum {
    ADC_CHANNEL_NONE = 0
} t_adc_channel;
typedef enum {
    ADC_MODE_NONE = 0
} t_adc_mode_t;

// TT (Time-Triggered OS - Placeholder as no specific TT registers/features defined in inputs)
typedef enum {
    TICK_TIME_MS_NONE = 0
} t_tick_time;

/*
 * @brief  MCAL API Prototypes
 *         As listed in "API.json".
 */

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
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Assuming void return type

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
void UART_ClearFlag(t_uart_channel uart_channel); // Assuming void return type

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
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Assuming void return type

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
void SPI_ClearFlag(t_spi_channel spi_channel); // Assuming void return type

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Assuming void return type

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Assuming tlong return type
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Assuming void return type

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Assuming void return type

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void); // Assuming void return type

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

#endif /* MCAL_H */

// Chunk 4
/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File
 *
 * This file provides the declarations for the MCAL for the STM32F401RC microcontroller.
 * It includes core MCU header, standard C library includes, and type definitions,
 * along with the API function prototypes.
 *
 * MCU: STM32F401RC
 */

#ifndef MCAL_H
#define MCAL_H

// Core MCU header file (placeholder if specific part number header not available)
// #include "stm32f4xx.h" // General STM32F4 header, assuming device-specific is covered
#include "stm32f401xc.h"  // Core device header for STM32F401RC

// Standard C library includes
#include <stdint.h>   // For uint8_t, uint16_t, uint32_t
#include <stdbool.h>  // For bool
#include <stddef.h>   // For NULL, size_t
// #include <string.h>   // Not explicitly used in current API, but common
// #include <stdio.h>    // Not explicitly used in current API, but common
// #include <stdlib.h>   // Not explicitly used in current API, but common
// #include <math.h>     // Not explicitly used in current API, but common

// Data Type Definitions (as per Rules.json)
// Note: Rules.json states "#define Unit_8 tbyte", but this is interpreted as
// defining 'tbyte' to be an 8-bit unsigned integer type, etc., for standard MCAL practice.
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// General Direction Type for GPIO
typedef enum
{
    DIRECTION_INPUT,
    DIRECTION_OUTPUT
} t_direction;

// -----------------------------------------------------------------------------
// Register Definitions (from REGISTER JSON)
// These are direct memory-mapped register addresses.
// -----------------------------------------------------------------------------

// DMA1 Registers
#define DMA1_S6M0AR_ADDR   (*(volatile tlong *)0x400260AC) /**< DMA1 stream 6 memory 0 address register. */
#define DMA1_S6M1AR_ADDR   (*(volatile tlong *)0x400260B0) /**< DMA1 stream 6 memory 1 address register. */
#define DMA1_S6FCR_ADDR    (*(volatile tlong *)0x400260B4) /**< DMA1 stream 6 FIFO control register. */
#define DMA1_S7CR_ADDR     (*(volatile tlong *)0x400260B8) /**< DMA1 stream 7 configuration register. */
#define DMA1_S7NDTR_ADDR   (*(volatile tlong *)0x400260BC) /**< DMA1 stream 7 number of data register. */
#define DMA1_S7PAR_ADDR    (*(volatile tlong *)0x400260C0) /**< DMA1 stream 7 peripheral address register. */
#define DMA1_S7M0AR_ADDR   (*(volatile tlong *)0x400260C4) /**< DMA1 stream 7 memory 0 address register. */
#define DMA1_S7M1AR_ADDR   (*(volatile tlong *)0x400260C8) /**< DMA1 stream 7 memory 1 address register. */
#define DMA1_S7FCR_ADDR    (*(volatile tlong *)0x400260CC) /**< DMA1 stream 7 FIFO control register. */

// DMA2 Registers
#define DMA2_LISR_ADDR     (*(volatile tlong *)0x40026400) /**< DMA2 low interrupt status register. */
#define DMA2_HISR_ADDR     (*(volatile tlong *)0x40026404) /**< DMA2 high interrupt status register. */
#define DMA2_LIFCR_ADDR    (*(volatile tlong *)0x40026408) /**< DMA2 low interrupt flag clear register. */
#define DMA2_HIFCR_ADDR    (*(volatile tlong *)0x4002640C) /**< DMA2 high interrupt flag clear register. */
#define DMA2_S0CR_ADDR     (*(volatile tlong *)0x40026410) /**< DMA2 stream 0 configuration register. */
#define DMA2_S0NDTR_ADDR   (*(volatile tlong *)0x40026414) /**< DMA2 stream 0 number of data register. */
#define DMA2_S0PAR_ADDR    (*(volatile tlong *)0x40026418) /**< DMA2 stream 0 peripheral address register. */
#define DMA2_S0M0AR_ADDR   (*(volatile tlong *)0x4002641C) /**< DMA2 stream 0 memory 0 address register. */
#define DMA2_S0M1AR_ADDR   (*(volatile tlong *)0x40026420) /**< DMA2 stream 0 memory 1 address register. */
#define DMA2_S0FCR_ADDR    (*(volatile tlong *)0x40026424) /**< DMA2 stream 0 FIFO control register. */
#define DMA2_S1CR_ADDR     (*(volatile tlong *)0x40026428) /**< DMA2 stream 1 configuration register. */
#define DMA2_S1NDTR_ADDR   (*(volatile tlong *)0x4002642C) /**< DMA2 stream 1 number of data register. */
#define DMA2_S1PAR_ADDR    (*(volatile tlong *)0x40026430) /**< DMA2 stream 1 peripheral address register. */
#define DMA2_S1M0AR_ADDR   (*(volatile tlong *)0x40026434) /**< DMA2 stream 1 memory 0 address register. */
#define DMA2_S1M1AR_ADDR   (*(volatile tlong *)0x40026438) /**< DMA2 stream 1 memory 1 address register. */
#define DMA2_S1FCR_ADDR    (*(volatile tlong *)0x4002643C) /**< DMA2 stream 1 FIFO control register. */
#define DMA2_S2CR_ADDR     (*(volatile tlong *)0x40026440) /**< DMA2 stream 2 configuration register. */
#define DMA2_S2NDTR_ADDR   (*(volatile tlong *)0x40026444) /**< DMA2 stream 2 number of data register. */
#define DMA2_S2PAR_ADDR    (*(volatile tlong *)0x40026448) /**< DMA2 stream 2 peripheral address register. */
#define DMA2_S2M0AR_ADDR   (*(volatile tlong *)0x4002644C) /**< DMA2 stream 2 memory 0 address register. */
#define DMA2_S2M1AR_ADDR   (*(volatile tlong *)0x40026450) /**< DMA2 stream 2 memory 1 address register. */
#define DMA2_S2FCR_ADDR    (*(volatile tlong *)0x40026454) /**< DMA2 stream 2 FIFO control register. */
#define DMA2_S3CR_ADDR     (*(volatile tlong *)0x40026458) /**< DMA2 stream 3 configuration register. */
#define DMA2_S3NDTR_ADDR   (*(volatile tlong *)0x4002645C) /**< DMA2 stream 3 number of data register. */
#define DMA2_S3PAR_ADDR    (*(volatile tlong *)0x40026460) /**< DMA2 stream 3 peripheral address register. */
#define DMA2_S3M0AR_ADDR   (*(volatile tlong *)0x40026464) /**< DMA2 stream 3 memory 0 address register. */
#define DMA2_S3M1AR_ADDR   (*(volatile tlong *)0x40026468) /**< DMA2 stream 3 memory 1 address register. */
#define DMA2_S3FCR_ADDR    (*(volatile tlong *)0x4002646C) /**< DMA2 stream 3 FIFO control register. */
#define DMA2_S4CR_ADDR     (*(volatile tlong *)0x40026470) /**< DMA2 stream 4 configuration register. */
#define DMA2_S4NDTR_ADDR   (*(volatile tlong *)0x40026474) /**< DMA2 stream 4 number of data register. */
#define DMA2_S4PAR_ADDR    (*(volatile tlong *)0x40026478) /**< DMA2 stream 4 peripheral address register. */
#define DMA2_S4M0AR_ADDR   (*(volatile tlong *)0x4002647C) /**< DMA2 stream 4 memory 0 address register. */
#define DMA2_S4M1AR_ADDR   (*(volatile tlong *)0x40026480) /**< DMA2 stream 4 memory 1 address register. */
#define DMA2_S4FCR_ADDR    (*(volatile tlong *)0x40026484) /**< DMA2 stream 4 FIFO control register. */
#define DMA2_S5CR_ADDR     (*(volatile tlong *)0x40026488) /**< DMA2 stream 5 configuration register. */
#define DMA2_S5NDTR_ADDR   (*(volatile tlong *)0x4002648C) /**< DMA2 stream 5 number of data register. */
#define DMA2_S5PAR_ADDR    (*(volatile tlong *)0x40026490) /**< DMA2 stream 5 peripheral address register. */
#define DMA2_S5M0AR_ADDR   (*(volatile tlong *)0x40026494) /**< DMA2 stream 5 memory 0 address register. */
#define DMA2_S5M1AR_ADDR   (*(volatile tlong *)0x40026498) /**< DMA2 stream 5 memory 1 address register. */
#define DMA2_S5FCR_ADDR    (*(volatile tlong *)0x4002649C) /**< DMA2 stream 5 FIFO control register. */
#define DMA2_S6CR_ADDR     (*(volatile tlong *)0x400264A0) /**< DMA2 stream 6 configuration register. */
#define DMA2_S6NDTR_ADDR   (*(volatile tlong *)0x400264A4) /**< DMA2 stream 6 number of data register. */
#define DMA2_S6PAR_ADDR    (*(volatile tlong *)0x400264A8) /**< DMA2 stream 6 peripheral address register. */
#define DMA2_S6M0AR_ADDR   (*(volatile tlong *)0x400264AC) /**< DMA2 stream 6 memory 0 address register. */
#define DMA2_S6M1AR_ADDR   (*(volatile tlong *)0x400264B0) /**< DMA2 stream 6 memory 1 address register. */
#define DMA2_S6FCR_ADDR    (*(volatile tlong *)0x400264B4) /**< DMA2 stream 6 FIFO control register. */
#define DMA2_S7CR_ADDR     (*(volatile tlong *)0x400264B8) /**< DMA2 stream 7 configuration register. */
#define DMA2_S7NDTR_ADDR   (*(volatile tlong *)0x400264BC) /**< DMA2 stream 7 number of data register. */
#define DMA2_S7PAR_ADDR    (*(volatile tlong *)0x400264C0) /**< DMA2 stream 7 peripheral address register. */
#define DMA2_S7M0AR_ADDR   (*(volatile tlong *)0x400264C4) /**< DMA2 stream 7 memory 0 address register. */
#define DMA2_S7M1AR_ADDR   (*(volatile tlong *)0x400264C8) /**< DMA2 stream 7 memory 1 address register. */
#define DMA2_S7FCR_ADDR    (*(volatile tlong *)0x400264CC) /**< DMA2 stream 7 FIFO control register. */

// EXTI Registers
#define EXTI_IMR_ADDR      (*(volatile tlong *)0x40013C00) /**< Interrupt mask register for EXTI lines. */


// -----------------------------------------------------------------------------
// Type Definitions for API Parameters (from API.json & Rules.json)
// -----------------------------------------------------------------------------

// MCU CONFIG Types
typedef enum
{
    SYS_VOLT_3V = 0,
    SYS_VOLT_5V
} t_sys_volt;

// LVD Types (based on LVD_requirements rule)
typedef enum
{
    LVD_THRESHOLD_0_5V,
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1_5V,
    LVD_THRESHOLD_2V,
    // ... further thresholds can be added if needed, up to 5V as per rule
    LVD_THRESHOLD_2_5V,
    LVD_THRESHOLD_3V,
    LVD_THRESHOLD_3_5V,
    LVD_THRESHOLD_4V,
    LVD_THRESHOLD_4_5V,
    LVD_THRESHOLD_5V
} t_lvd_thrthresholdLevel;

typedef enum
{
    LVD_CHANNEL_NONE = 0, // Placeholder as no specific channels are defined
    LVD_CHANNEL_1
} t_lvd_channel;

// UART Types
typedef enum
{
    UART_CHANNEL_1 = 0, // Inferred channel
    UART_CHANNEL_2,
    UART_CHANNEL_3,
    UART_CHANNEL_6    // STM32F401 has USART1, USART2, USART6
} t_uart_channel;

typedef enum
{
    UART_BAUD_RATE_9600 = 0, // Common baud rates
    UART_BAUD_RATE_19200,
    UART_BAUD_RATE_115200
} t_uart_baud_rate;

typedef enum
{
    UART_DATA_LENGTH_8_BITS = 0,
    UART_DATA_LENGTH_9_BITS
} t_uart_data_length;

typedef enum
{
    UART_STOP_BIT_1 = 0,
    UART_STOP_BIT_0_5,
    UART_STOP_BIT_2,
    UART_STOP_BIT_1_5
} t_uart_stop_bit;

typedef enum
{
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// I2C Types
typedef enum
{
    I2C_CHANNEL_1 = 0, // Inferred channel
    I2C_CHANNEL_2,
    I2C_CHANNEL_3
} t_i2c_channel;

typedef enum
{
    I2C_CLK_SPEED_STANDARD = 0, // 100 kHz
    I2C_CLK_SPEED_FAST          // 400 kHz (as per rule: "Always use fast mode")
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // 7-bit address
typedef enum
{
    I2C_ACK_DISABLE = 0,
    I2C_ACK_ENABLE
} t_i2c_ack;

typedef enum
{
    I2C_DATA_LENGTH_8_BITS = 0,
    I2C_DATA_LENGTH_16_BITS
} t_i2c_datalength;

// SPI Types
typedef enum
{
    SPI_CHANNEL_1 = 0, // Inferred channel
    SPI_CHANNEL_2,
    SPI_CHANNEL_3,
    SPI_CHANNEL_4,
    SPI_CHANNEL_5
} t_spi_channel;

typedef enum
{
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE
} t_spi_mode;

typedef enum
{
    SPI_CPOL_LOW = 0,  // Clock Phase Low (Idle low)
    SPI_CPOL_HIGH      // Clock Phase High (Idle high)
} t_spi_cpol;

typedef enum
{
    SPI_CPHA_FIRST_EDGE = 0, // Data captured on first clock transition
    SPI_CPHA_SECOND_EDGE     // Data captured on second clock transition
} t_spi_cpha;

typedef enum
{
    SPI_DFF_8_BITS = 0,
    SPI_DFF_16_BITS
} t_spi_dff;

typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST
} t_spi_bit_order;

// External Interrupt Types
typedef enum
{
    EXTERNAL_INT_LINE_0 = 0, // Maps to PA0, PB0, PC0, PD0, PE0, PH0
    EXTERNAL_INT_LINE_1,     // Maps to PA1, PB1, PC1, PD1, PE1, PH1
    EXTERNAL_INT_LINE_2,
    EXTERNAL_INT_LINE_3,
    EXTERNAL_INT_LINE_4,
    EXTERNAL_INT_LINE_5,
    EXTERNAL_INT_LINE_6,
    EXTERNAL_INT_LINE_7,
    EXTERNAL_INT_LINE_8,
    EXTERNAL_INT_LINE_9,
    EXTERNAL_INT_LINE_10,
    EXTERNAL_INT_LINE_11,
    EXTERNAL_INT_LINE_12,
    EXTERNAL_INT_LINE_13,
    EXTERNAL_INT_LINE_14,
    EXTERNAL_INT_LINE_15     // Maps to PA15, PB15, PC15, PD15, PE15
} t_external_int_channel;

typedef enum
{
    EXTERNAL_INT_EDGE_RISING = 0,
    EXTERNAL_INT_EDGE_FALLING,
    EXTERNAL_INT_EDGE_RISING_FALLING
} t_external_int_edge;

// GPIO Types
typedef enum
{
    PORTA = 0,
    PORTB,
    PORTC,
    PORTD,
    PORTE,
    PORTH // For PH0, PH1
} t_port;

typedef enum
{
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

// PWM Types
typedef enum
{
    PWM_CHANNEL_TIM1_CH1 = 0, // PA8, PE9 (example from rules)
    PWM_CHANNEL_TIM1_CH2,     // PA9, PE11
    PWM_CHANNEL_TIM1_CH3,     // PA10, PE13
    PWM_CHANNEL_TIM1_CH4,     // PA11, PE14
    PWM_CHANNEL_TIM2_CH1,     // PA0, PA5, PA15, PB3
    PWM_CHANNEL_TIM3_CH1,     // PA6, PB4, PC6
    // ... further channels for TIMers based on available timers on STM32F401RC
} t_pwm_channel;

// ICU Types
typedef enum
{
    ICU_CHANNEL_TIM1_CH1 = 0, // Placeholder
    ICU_CHANNEL_TIM2_CH2,     // Placeholder
    // ...
} t_icu_channel;

typedef enum
{
    ICU_PRESCALER_1 = 0, // Placeholder
    ICU_PRESCALER_2,
    // ...
} t_icu_prescaller;

typedef enum
{
    ICU_EDGE_RISING = 0, // Placeholder
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer Types
typedef enum
{
    TIMER_CHANNEL_TIM2 = 0, // Placeholder, usually refers to whole timer peripheral
    TIMER_CHANNEL_TIM3,
    TIMER_CHANNEL_TIM4,
    TIMER_CHANNEL_TIM5,
    TIMER_CHANNEL_TIM1,
    TIMER_CHANNEL_TIM9,
    TIMER_CHANNEL_TIM10,
    TIMER_CHANNEL_TIM11
} t_timer_channel;

// ADC Types
typedef enum
{
    ADC_CHANNEL_0 = 0, // Analog channels on GPIO A, B, C
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7,
    ADC_CHANNEL_8,
    ADC_CHANNEL_9,
    ADC_CHANNEL_10,
    ADC_CHANNEL_11,
    ADC_CHANNEL_12,
    ADC_CHANNEL_13,
    ADC_CHANNEL_14,
    ADC_CHANNEL_15,
    ADC_CHANNEL_TEMP_SENSOR,
    ADC_CHANNEL_VREFINT
} t_adc_channel;

typedef enum
{
    ADC_MODE_SINGLE_CONVERSION = 0,
    ADC_MODE_CONTINUOUS_CONVERSION
} t_adc_mode_t;

// TT Types (Time Triggered OS)
typedef tword t_tick_time; // Tick time in ms

// -----------------------------------------------------------------------------
// API Function Prototypes (from API.json)
// -----------------------------------------------------------------------------

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
tbyte LVD_ClearFlag(t_lvd_channel lvd_channel); // Adjusted return type to tbyte for consistency, assuming flag status

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
tbyte UART_ClearFlag(t_uart_channel uart_channel); // Adjusted return type to tbyte for consistency, assuming flag status

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
tbyte I2C_ClearFlag(t_i2c_channel i2c_channel); // Adjusted return type to tbyte for consistency, assuming flag status

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
tbyte SPI_ClearFlag(t_spi_channel spi_channel); // Adjusted return type to tbyte for consistency, assuming flag status

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
tbyte External_INT_ClearFlag(t_external_int_channel external_int_channel); // Adjusted return type to tbyte for consistency, assuming flag status

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
tword ICU_GetFrequency(t_icu_channel icu_channel); // Adjusted return type from generic to tword for frequency
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
tbyte ICU_ClearFlag(t_icu_channel icu_channel); // Adjusted return type to tbyte for consistency, assuming flag status

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
tbyte TIMER_ClearFlag(t_timer_channel timer_channel); // Adjusted return type to tbyte for consistency, assuming flag status

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
tbyte ADC_ClearFlag(void); // Adjusted return type to tbyte for consistency, assuming flag status

// Internal_EEPROM
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time Triggered OS)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif // MCAL_H

// Chunk 5
#ifndef MCAL_H
#define MCAL_H

// Core device header - placeholder for STM32F401RC
// The exact header might be stm32f401xc.h or stm32f4xx.h depending on specific CMSIS/HAL setup.
#include "stm32f4xx.h"  // Core device header as per rules.json "core_includes"

// Standard C library includes as per rules.json "core_includes"
#include <stdint.h>     // For uint8_t, uint16_t, uint32_t
#include <stdbool.h>    // For bool type
#include <stddef.h>     // For size_t, NULL
#include <string.h>     // Included as per example in rules.json, though not explicitly used for now
#include <stdio.h>      // Included as per example in rules.json, though not explicitly used for now
#include <stdlib.h>     // Included as per example in rules.json, though not explicitly used for now
#include <math.h>       // Included as per example in rules.json, though not explicitly used for now

// Data type definitions as per rules.json "data_type_definitions"
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

// MCU CONFIG API type definitions
typedef enum {
    Vsource_Unknown = 0, // Placeholder if no specific voltage levels provided in register_json
    Vsource_3V,          // As per rules.json "LVD_requirements"
    Vsource_5V           // As per rules.json "LVD_requirements"
} t_sys_volt;

// LVD API type definitions as per rules.json "LVD_requirements"
typedef enum {
    LVD_THRESHOLD_0_5V = 0,
    LVD_THRESHOLD_1V,
    LVD_THRESHOLD_1_5V,
    LVD_THRESHOLD_2V,
    LVD_THRESHOLD_2_5V,
    LVD_THRESHOLD_3V,
    LVD_THRESHOLD_3_5V,
    LVD_THRESHOLD_4V,
    LVD_THRESHOLD_4_5V,
    LVD_THRESHOLD_5V,
    LVD_THRESHOLD_MAX_LEVELS // Represents total number of threshold levels
} t_lvd_thrthresholdLevel;

typedef enum {
    LVD_CHANNEL_NONE = 0 // No specific LVD channels defined in register_json
} t_lvd_channel;

// UART API type definitions (No UART registers in register_json)
typedef enum {
    UART_CHANNEL_NONE = 0 // Placeholder as no UART registers were provided
} t_uart_channel;

typedef enum {
    UART_BAUD_RATE_NONE = 0 // Placeholder
} t_uart_baud_rate;

typedef enum {
    UART_DATA_LENGTH_NONE = 0 // Placeholder
} t_uart_data_length;

typedef enum {
    UART_STOP_BIT_NONE = 0 // Placeholder
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0 // Placeholder
} t_uart_parity;

// I2C API type definitions (No I2C registers in register_json)
typedef enum {
    I2C_CHANNEL_NONE = 0 // Placeholder as no I2C registers were provided
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_NONE = 0 // Placeholder
} t_i2c_clk_speed;

typedef uint16_t t_i2c_device_address; // Common type for I2C device addresses

typedef enum {
    I2C_ACK_NONE = 0 // Placeholder
} t_i2c_ack;

typedef enum {
    I2C_DATALENGTH_NONE = 0 // Placeholder
} t_i2c_datalength;

// SPI API type definitions (No SPI registers in register_json)
typedef enum {
    SPI_CHANNEL_NONE = 0 // Placeholder as no SPI registers were provided
} t_spi_channel;

typedef enum {
    SPI_MODE_NONE = 0 // Placeholder
} t_spi_mode;

typedef enum {
    SPI_CPOL_NONE = 0 // Placeholder
} t_spi_cpol;

typedef enum {
    SPI_CPHA_NONE = 0 // Placeholder
} t_spi_cpha;

typedef enum {
    SPI_DFF_NONE = 0 // Placeholder
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_NONE = 0 // Placeholder
} t_spi_bit_order;

// External Interrupt API type definitions
// Channels map directly to EXTI lines (0-15) and can be assigned to multiple pins per line.
typedef enum {
    EXTERNAL_INT_CHANNEL_0 = 0,  // Corresponds to EXTI Line 0 (e.g., PA0, PB0, PC0, PD0, PE0, PH0)
    EXTERNAL_INT_CHANNEL_1,      // Corresponds to EXTI Line 1 (e.g., PA1, PB1, PC1, PD1, PE1, PH1)
    EXTERNAL_INT_CHANNEL_2,      // Corresponds to EXTI Line 2 (e.g., PA2, PB2, PC2, PD2, PE2)
    EXTERNAL_INT_CHANNEL_3,      // Corresponds to EXTI Line 3 (e.g., PA3, PB3, PC3, PD3, PE3)
    EXTERNAL_INT_CHANNEL_4,      // Corresponds to EXTI Line 4 (e.g., PA4, PB4, PC4, PD4, PE4)
    EXTERNAL_INT_CHANNEL_5,      // Corresponds to EXTI Line 5 (e.g., PA5, PB5, PC5, PD5, PE5)
    EXTERNAL_INT_CHANNEL_6,      // Corresponds to EXTI Line 6 (e.g., PA6, PB6, PC6, PD6, PE6)
    EXTERNAL_INT_CHANNEL_7,      // Corresponds to EXTI Line 7 (e.g., PA7, PB7, PC7, PD7, PE7)
    EXTERNAL_INT_CHANNEL_8,      // Corresponds to EXTI Line 8 (e.g., PA8, PB8, PC8, PD8, PE8)
    EXTERNAL_INT_CHANNEL_9,      // Corresponds to EXTI Line 9 (e.g., PA9, PB9, PC9, PD9, PE9)
    EXTERNAL_INT_CHANNEL_10,     // Corresponds to EXTI Line 10 (e.g., PA10, PB10, PC10, PD10, PE10)
    EXTERNAL_INT_CHANNEL_11,     // Corresponds to EXTI Line 11 (e.g., PA11, PB11, PC11, PD11, PE11)
    EXTERNAL_INT_CHANNEL_12,     // Corresponds to EXTI Line 12 (e.g., PA12, PB12, PC12, PD12, PE12)
    EXTERNAL_INT_CHANNEL_13,     // Corresponds to EXTI Line 13 (e.g., PA13, PB13, PC13, PD13, PE13)
    EXTERNAL_INT_CHANNEL_14,     // Corresponds to EXTI Line 14 (e.g., PA14, PB14, PC14, PD14, PE14)
    EXTERNAL_INT_CHANNEL_15,     // Corresponds to EXTI Line 15 (e.g., PA15, PB15, PC15, PD15, PE15)
    EXTERNAL_INT_CHANNEL_MAX     // Total number of EXTI lines
} t_external_int_channel;

typedef enum {
    EXTERNAL_INT_EDGE_NONE = 0,
    EXTERNAL_INT_EDGE_RISING,
    EXTERNAL_INT_EDGE_FALLING,
    EXTERNAL_INT_EDGE_BOTH // Rising and Falling edges
} t_external_int_edge;

// GPIO API type definitions (No GPIO registers in register_json)
typedef enum {
    GPIO_PORT_NONE = 0 // Placeholder as no GPIO registers were provided
} t_port;

typedef enum {
    GPIO_PIN_NONE = 0 // Placeholder
} t_pin;

typedef enum {
    GPIO_DIRECTION_NONE = 0, // Placeholder
    GPIO_DIRECTION_INPUT,
    GPIO_DIRECTION_OUTPUT
} t_direction;

// PWM API type definitions (No PWM registers in register_json)
typedef enum {
    PWM_CHANNEL_NONE = 0 // Placeholder as no PWM registers were provided
} t_pwm_channel;

// ICU API type definitions (No ICU registers in register_json)
typedef enum {
    ICU_CHANNEL_NONE = 0 // Placeholder as no ICU registers were provided
} t_icu_channel;

typedef enum {
    ICU_PRESCALLER_NONE = 0 // Placeholder
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_NONE = 0 // Placeholder
} t_icu_edge;

// Timer API type definitions (No Timer registers in register_json)
typedef enum {
    TIMER_CHANNEL_NONE = 0 // Placeholder as no Timer registers were provided
} t_timer_channel;

// ADC API type definitions
typedef enum {
    ADC_CHANNEL_0 = 0,   // Assigned to PA0
    ADC_CHANNEL_1,       // Assigned to PA1
    ADC_CHANNEL_2,       // Assigned to PA2
    ADC_CHANNEL_3,       // Assigned to PA3
    ADC_CHANNEL_4,       // Assigned to PA4
    ADC_CHANNEL_5,       // Assigned to PA5
    ADC_CHANNEL_6,       // Assigned to PA6
    ADC_CHANNEL_7,       // Assigned to PA7
    ADC_CHANNEL_8,       // Assigned to PB0
    ADC_CHANNEL_9,       // Assigned to PB1
    ADC_CHANNEL_10,      // Assigned to PC0
    ADC_CHANNEL_11,      // Assigned to PC1
    ADC_CHANNEL_12,      // Assigned to PC2
    ADC_CHANNEL_13,      // Assigned to PC3
    ADC_CHANNEL_14,      // Assigned to PC4
    ADC_CHANNEL_15,      // Assigned to PC5
    ADC_CHANNEL_MAX      // Total number of ADC channels relevant to provided registers
} t_adc_channel;

typedef enum {
    ADC_MODE_SINGLE = 0,    // Single conversion mode
    ADC_MODE_CONTINUOUS,    // Continuous conversion mode
    ADC_MODE_SCAN,          // Scan mode
    ADC_MODE_DISCONTINUOUS  // Discontinuous mode
} t_adc_mode_t;

// TT API type definitions (No specific registers for TT in register_json)
typedef uint32_t t_tick_time; // Tick time in milliseconds

// =============================================================================
// API Function Prototypes (from API.json)
// =============================================================================

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
tlong ICU_GetFrequency(t_icu_channel icu_channel);
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

// Chunk 6
#ifndef MCAL_H
#define MCAL_H

// Core device header
#include "stm32f401xc.h"  // Core device header for STM32F401RC

// Standard C library includes as per Rules.json -> core_includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Data type definitions as per Rules.json -> data_type_definitions
#define tbyte  uint8_t
#define tword  uint16_t
#define tlong  uint32_t

// Enum definitions for API parameters (inferred based on API.json prototypes and Rules.json)

// MCU_CONFIG related enums
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

// LVD related enums
// Threshold levels as per Rules.json -> LVD_requirements
typedef enum {
    Volt_0_5V = 0,
    Volt_1V,
    Volt_1_5V,
    Volt_2V,
    Volt_2_5V, // Inferred for completeness based on pattern
    Volt_3V,   // Inferred for completeness based on pattern
    Volt_3_5V, // Inferred for completeness based on pattern
    Volt_4V,   // Inferred for completeness based on pattern
    Volt_4_5V, // Inferred for completeness based on pattern
    Volt_5V
} t_lvd_thrthresholdLevel;

typedef enum {
    LVD_CHANNEL_UNKNOWN = 0 // No LVD registers provided, so channel is undefined
} t_lvd_channel;

// UART related enums (no UART registers, so channels/parameters are placeholders)
typedef enum {
    UART_CHANNEL_1 = 0, // Placeholder
    UART_CHANNEL_2,     // Placeholder
    UART_CHANNEL_UNKNOWN
} t_uart_channel;

typedef enum {
    UART_BAUD_RATE_UNKNOWN = 0 // Placeholder
} t_uart_baud_rate;

typedef enum {
    UART_DATA_LENGTH_UNKNOWN = 0 // Placeholder
} t_uart_data_length;

typedef enum {
    UART_STOP_BIT_UNKNOWN = 0 // Placeholder
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_UNKNOWN = 0 // Placeholder
} t_uart_parity;

// I2C related enums (no I2C registers, so channels/parameters are placeholders)
typedef enum {
    I2C_CHANNEL_1 = 0, // Placeholder
    I2C_CHANNEL_2,     // Placeholder
    I2C_CHANNEL_3,     // Placeholder
    I2C_CHANNEL_UNKNOWN
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_UNKNOWN = 0 // Placeholder
} t_i2c_clk_speed;

typedef enum {
    I2C_DEV_ADDR_UNKNOWN = 0 // Placeholder
} t_i2c_device_address;

typedef enum {
    I2C_ACK_UNKNOWN = 0 // Placeholder
} t_i2c_ack;

typedef enum {
    I2C_DATA_LENGTH_UNKNOWN = 0 // Placeholder
} t_i2c_datalength;

// SPI related enums (no SPI registers, so channels/parameters are placeholders)
typedef enum {
    SPI_CHANNEL_1 = 0, // Placeholder
    SPI_CHANNEL_2,     // Placeholder
    SPI_CHANNEL_3,     // Placeholder
    SPI_CHANNEL_4,     // Placeholder
    SPI_CHANNEL_UNKNOWN
} t_spi_channel;

typedef enum {
    SPI_MODE_UNKNOWN = 0 // Placeholder
} t_spi_mode;

typedef enum {
    SPI_CPOL_UNKNOWN = 0 // Placeholder
} t_spi_cpol;

typedef enum {
    SPI_CPHA_UNKNOWN = 0 // Placeholder
} t_spi_cpha;

typedef enum {
    SPI_DFF_UNKNOWN = 0 // Placeholder
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_UNKNOWN = 0 // Placeholder
} t_spi_bit_order;

// External Interrupt related enums (no EXTI registers, so channels/parameters are placeholders)
typedef enum {
    EXT_INT_CHANNEL_0 = 0, // Placeholder for EXTI line 0
    EXT_INT_CHANNEL_15 = 15, // Placeholder for EXTI line 15
    EXT_INT_CHANNEL_UNKNOWN
} t_external_int_channel;

typedef enum {
    EXT_INT_EDGE_RISING = 0, // Placeholder
    EXT_INT_EDGE_FALLING,    // Placeholder
    EXT_INT_EDGE_RISING_FALLING, // Placeholder
    EXT_INT_EDGE_UNKNOWN
} t_external_int_edge;

// GPIO related enums (no GPIO registers, so ports/pins are placeholders)
typedef enum {
    GPIO_PORTA = 0,
    GPIO_PORTB,
    GPIO_PORTC,
    GPIO_PORTD, // Typically available on STM32F4
    GPIO_PORTE,
    GPIO_PORTH, // Typically available on STM32F4
    GPIO_PORT_UNKNOWN
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
    GPIO_PIN_15,
    GPIO_PIN_UNKNOWN
} t_pin;

typedef enum {
    GPIO_DIR_IN = 0,
    GPIO_DIR_OUT
} t_direction;

// PWM related enums (mapped to Timer peripherals based on REGISTER JSON)
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
    PWM_CHANNEL_UNKNOWN
} t_pwm_channel;

// ICU related enums (mapped to Timer peripherals based on REGISTER JSON)
typedef enum {
    ICU_CHANNEL_TIM1_CH1 = 0, // PA8, PE9 (Input Capture 1)
    ICU_CHANNEL_TIM1_CH2,     // PA9, PE11 (Input Capture 2)
    ICU_CHANNEL_TIM1_CH3,     // PA10, PE13 (Input Capture 3)
    ICU_CHANNEL_TIM1_CH4,     // PA11, PE14 (Input Capture 4)
    ICU_CHANNEL_TIM2_CH1,     // PA0, PA5, PA15 (Input Capture 1)
    ICU_CHANNEL_TIM2_CH2,     // PA1, PB3 (Input Capture 2)
    ICU_CHANNEL_TIM2_CH3,     // PA2, PB10 (Input Capture 3)
    ICU_CHANNEL_TIM2_CH4,     // PA3, PB11 (Input Capture 4)
    ICU_CHANNEL_TIM3_CH1,     // PA6, PB4, PC6 (Input Capture 1)
    ICU_CHANNEL_TIM3_CH2,     // PA7, PB5, PC7 (Input Capture 2)
    ICU_CHANNEL_UNKNOWN
} t_icu_channel;

typedef enum {
    ICU_PRESCALER_DIV1 = 0, // No prescaling
    ICU_PRESCALER_DIV2,     // Capture every 2 events
    ICU_PRESCALER_DIV4,     // Capture every 4 events
    ICU_PRESCALER_DIV8,     // Capture every 8 events
    ICU_PRESCALER_UNKNOWN
} t_icu_prescaller;

typedef enum {
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH,
    ICU_EDGE_UNKNOWN
} t_icu_edge;

// Timer related enums
typedef enum {
    TIMER_CHANNEL_TIM1 = 0,
    TIMER_CHANNEL_TIM2,
    TIMER_CHANNEL_TIM3,
    TIMER_CHANNEL_UNKNOWN
} t_timer_channel;

// ADC related enums
typedef enum {
    ADC_CHANNEL_PA0 = 0,
    ADC_CHANNEL_PA1,
    ADC_CHANNEL_PA2,
    ADC_CHANNEL_PA3,
    ADC_CHANNEL_PA4,
    ADC_CHANNEL_PA5,
    ADC_CHANNEL_PA6,
    ADC_CHANNEL_PA7,
    ADC_CHANNEL_PB0,
    ADC_CHANNEL_PB1,
    ADC_CHANNEL_PC0,
    ADC_CHANNEL_PC1,
    ADC_CHANNEL_PC2,
    ADC_CHANNEL_PC3,
    ADC_CHANNEL_PC4,
    ADC_CHANNEL_PC5,
    ADC_CHANNEL_UNKNOWN
} t_adc_channel;

typedef enum {
    ADC_MODE_INJECTED = 0,
    ADC_MODE_REGULAR,
    ADC_MODE_UNKNOWN
} t_adc_mode_t;

// TT (Time Triggered OS) related enums
typedef enum {
    TICK_TIME_1MS = 1,
    TICK_TIME_10MS = 10,
    TICK_TIME_100MS = 100,
    TICK_TIME_UNKNOWN = 0 // Placeholder if not specified
} t_tick_time;


// API Function Prototypes (from API.json)

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
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Corrected return type based on common "ClearFlag" convention

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
void UART_ClearFlag(t_uart_channel uart_channel); // Corrected return type

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
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Corrected return type

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
void SPI_ClearFlag(t_spi_channel spi_channel); // Corrected return type

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Corrected return type

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Inferred return type as frequency can be large
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Corrected return type

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Corrected return type

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void); // Corrected return type

// Internal_EEPROM
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time Triggered OS)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif // MCAL_H

// Chunk 7
#ifndef MCAL_H
#define MCAL_H

// Core device header - inferred based on MCU name STM32F401RC
#include "stm32f4xx.h"  // Placeholder for the specific device header
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

// Enum for MCU_Config_Init voltage source
typedef enum {
    Vsource_3V = 0,
    Vsource_5V
} t_sys_volt;

// Enum for LVD threshold levels as per Rules.json
typedef enum {
    LVD_Volt_0_5V = 0,
    LVD_Volt_1V,
    LVD_Volt_1_5V,
    LVD_Volt_2V,
    LVD_Volt_2_5V, // Inferred levels up to 5V
    LVD_Volt_3V,
    LVD_Volt_3_5V,
    LVD_Volt_4V,
    LVD_Volt_4_5V,
    LVD_Volt_5V
} t_lvd_thrthresholdLevel;

// Placeholder enum for LVD channel (no info in register_json)
typedef enum {
    LVD_CHANNEL_NONE = 0
} t_lvd_channel;

// Placeholder enums for UART (no UART registers in register_json)
typedef enum {
    UART_CHANNEL_NONE = 0
} t_uart_channel;

typedef enum {
    UART_BAUD_RATE_NONE = 0
} t_uart_baud_rate;

typedef enum {
    UART_DATA_LENGTH_NONE = 0
} t_uart_data_length;

typedef enum {
    UART_STOP_BIT_NONE = 0
} t_uart_stop_bit;

typedef enum {
    UART_PARITY_NONE = 0
} t_uart_parity;

// Placeholder enums for I2C (no I2C registers in register_json)
typedef enum {
    I2C_CHANNEL_NONE = 0
} t_i2c_channel;

typedef enum {
    I2C_CLK_SPEED_NONE = 0
} t_i2c_clk_speed;

typedef enum {
    I2C_DEVICE_ADDRESS_NONE = 0
} t_i2c_device_address;

typedef enum {
    I2C_ACK_NONE = 0
} t_i2c_ack;

typedef enum {
    I2C_DATALENGTH_NONE = 0
} t_i2c_datalength;

// Placeholder enums for SPI (no SPI registers in register_json)
typedef enum {
    SPI_CHANNEL_NONE = 0
} t_spi_channel;

typedef enum {
    SPI_MODE_NONE = 0
} t_spi_mode;

typedef enum {
    SPI_CPOL_NONE = 0
} t_spi_cpol;

typedef enum {
    SPI_CPHA_NONE = 0
} t_spi_cpha;

typedef enum {
    SPI_DFF_NONE = 0
} t_spi_dff;

typedef enum {
    SPI_BIT_ORDER_NONE = 0
} t_spi_bit_order;

// Placeholder enums for External Interrupt (no External Interrupt registers in register_json)
typedef enum {
    EXTERNAL_INT_CHANNEL_NONE = 0
} t_external_int_channel;

typedef enum {
    EXTERNAL_INT_EDGE_NONE = 0
} t_external_int_edge;

// Placeholder enums for GPIO (no GPIO registers in register_json)
typedef enum {
    GPIO_PORT_NONE = 0
} t_port;

typedef enum {
    GPIO_PIN_NONE = 0
} t_pin;

typedef enum {
    GPIO_DIRECTION_NONE = 0,
    GPIO_DIRECTION_INPUT,
    GPIO_DIRECTION_OUTPUT
} t_direction;

// Enum for PWM channels based on available TIMx_CCR and assigned pins
typedef enum {
    PWM_TIM3_CH3,  // PB0, PC8
    PWM_TIM3_CH4,  // PB1, PC9
    PWM_TIM4_CH1,  // PB6, PD12
    PWM_TIM4_CH2,  // PB7, PD13
    PWM_TIM4_CH3,  // PB8, PD14
    PWM_TIM4_CH4,  // PB9, PD15
    PWM_TIM5_CH1,  // PA0
    PWM_TIM5_CH2,  // PA1
    PWM_TIM5_CH3,  // PA2
    PWM_TIM5_CH4,  // PA3
    PWM_TIM9_CH1,  // PA2, PE5
    PWM_TIM9_CH2,  // PA3, PE6
    PWM_TIM10_CH1, // PA6, PB8
    PWM_TIM11_CH1, // PA7, PB9
    PWM_CHANNEL_NONE
} t_pwm_channel;

// Enum for ICU channels based on available TIMx_CCR and assigned pins
// (ICU channels largely overlap with PWM channels as they use the same capture/compare registers)
typedef enum {
    ICU_TIM3_CH3,  // PB0, PC8
    ICU_TIM3_CH4,  // PB1, PC9
    ICU_TIM4_CH1,  // PB6, PD12
    ICU_TIM4_CH2,  // PB7, PD13
    ICU_TIM4_CH3,  // PB8, PD14
    ICU_TIM4_CH4,  // PB9, PD15
    ICU_TIM5_CH1,  // PA0
    ICU_TIM5_CH2,  // PA1
    ICU_TIM5_CH3,  // PA2
    ICU_TIM5_CH4,  // PA3
    ICU_TIM9_CH1,  // PA2, PE5
    ICU_TIM9_CH2,  // PA3, PE6
    ICU_TIM10_CH1, // PA6, PB8
    ICU_TIM11_CH1, // PA7, PB9
    ICU_CHANNEL_NONE
} t_icu_channel;

// Placeholder enum for ICU prescaler (register_json doesn't specify granular prescaler values)
typedef enum {
    ICU_PRESCALER_NONE = 0,
    ICU_PRESCALER_DIV1,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8,
    ICU_PRESCALER_DIV16, // Example prescalers
    ICU_PRESCALER_DIV32,
    ICU_PRESCALER_DIV64,
    ICU_PRESCALER_DIV128,
    ICU_PRESCALER_DIV256,
    ICU_PRESCALER_DIV512,
    ICU_PRESCALER_DIV1024
} t_icu_prescaller;

// Placeholder enum for ICU edge (CCER register can configure rising/falling/both edges)
typedef enum {
    ICU_EDGE_NONE = 0,
    ICU_EDGE_RISING,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Enum for Timer channels based on available TIMx
typedef enum {
    TIMER_TIM3 = 0,
    TIMER_TIM4,
    TIMER_TIM5,
    TIMER_TIM9,
    TIMER_TIM10,
    TIMER_TIM11,
    TIMER_CHANNEL_NONE
} t_timer_channel;

// Placeholder enums for ADC (no ADC registers in register_json)
typedef enum {
    ADC_CHANNEL_NONE = 0
} t_adc_channel;

typedef enum {
    ADC_MODE_NONE = 0
} t_adc_mode_t;

// Placeholder enum for TT tick time
typedef tword t_tick_time;

// API Function Prototypes (from API.json)

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
void LVD_ClearFlag(t_lvd_channel lvd_channel); // Return type fixed to void for consistency

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
void UART_ClearFlag(t_uart_channel uart_channel); // Return type fixed to void for consistency

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
void I2C_ClearFlag(t_i2c_channel i2c_channel); // Return type fixed to void for consistency

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
void SPI_ClearFlag(t_spi_channel spi_channel); // Return type fixed to void for consistency

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);
void External_INT_ClearFlag(t_external_int_channel external_int_channel); // Return type fixed to void for consistency

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
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Return type inferred as tlong for frequency
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));
void ICU_ClearFlag(t_icu_channel icu_channel); // Return type fixed to void for consistency

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);
void TIMER_ClearFlag(t_timer_channel timer_channel); // Return type fixed to void for consistency

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(void);
void ADC_Disable(void);
void ADC_Update(void);
tword ADC_Get(void);
void ADC_ClearFlag(void); // Return type fixed to void for consistency

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

// Chunk 8
/**
 * @file MCAL.h
 * @brief Microcontroller Abstraction Layer (MCAL) Header File
 *
 * This file contains the API declarations and type definitions for the MCAL.
 * It provides an abstract interface to the microcontroller's peripherals,
 * allowing application code to be hardware-independent.
 *
 * MCU Name: STM32F401RC
 */

#ifndef MCAL_H
#define MCAL_H

// Core device header - inferred from MCU name (STM32F401RC)
#include "stm32f401xc.h" 

// Standard C library includes as per core_includes rule
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h> // Required for string/frame manipulation functions

// Data type definitions as per data_type_definitions rule
#define tbyte uint8_t
#define tword uint16_t
#define tlong uint32_t

// =============================================================================
// MCU CONFIG APIs & Type Definitions
// =============================================================================

/**
 * @brief System Voltage Level
 */
typedef enum
{
    Vsource_3V = 0, /**< 3V System Voltage */
    Vsource_5V      /**< 5V System Voltage */
} t_sys_volt;

/**
 * @brief Initializes the MCU configuration.
 * @param volt The system voltage level.
 */
void MCU_Config_Init(t_sys_volt volt);

/**
 * @brief Resets the Watchdog Timer (WDT).
 */
void WDT_Reset(void);

/**
 * @brief Puts the MCU into sleep mode.
 */
void Go_to_sleep_mode(void);

/**
 * @brief Enables global interrupts.
 */
void Global_interrupt_Enable(void);

/**
 * @brief Disables global interrupts.
 */
void Global_interrupt_Disable(void);

// =============================================================================
// LVD APIs & Type Definitions
// =============================================================================

/**
 * @brief LVD Threshold Levels as per LVD_requirements rule.
 */
typedef enum
{
    Volt_0_5V = 0, /**< 0.5V Threshold */
    Volt_1V,       /**< 1.0V Threshold */
    Volt_1_5V,     /**< 1.5V Threshold */
    Volt_2V,       /**< 2.0V Threshold */
    Volt_2_5V,     /**< 2.5V Threshold */
    Volt_3V,       /**< 3.0V Threshold */
    Volt_3_5V,     /**< 3.5V Threshold */
    Volt_4V,       /**< 4.0V Threshold */
    Volt_4_5V,     /**< 4.5V Threshold */
    Volt_5V        /**< 5.0V Threshold */
} t_lvd_thrthresholdLevel;

/**
 * @brief LVD Channel (dummy, as not specified in register_json)
 */
typedef enum
{
    LVD_CHANNEL_NONE = 0 /**< Placeholder for LVD Channel */
} t_lvd_channel;

/**
 * @brief Initializes the LVD module.
 */
void LVD_Init(void);

/**
 * @brief Gets the current LVD threshold level.
 * @param lvd_thresholdLevel The LVD threshold level to set/get (parameter name mismatch in API, assumed target level).
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);

/**
 * @brief Enables the LVD module.
 */
void LVD_Enable(void);

/**
 * @brief Disables the LVD module.
 */
void LVD_Disable(void);

/**
 * @brief Clears the LVD flag for a specific channel.
 * @param lvd_channel The LVD channel.
 */
void LVD_ClearFlag(t_lvd_channel lvd_channel);

// =============================================================================
// UART APIs & Type Definitions
// =============================================================================

/**
 * @brief UART Channel enumeration.
 * Derived from register_json (USART1, USART2, USART6).
 */
typedef enum
{
    UART_CHANNEL_1 = 0, /**< USART1 channel */
    UART_CHANNEL_2,     /**< USART2 channel */
    UART_CHANNEL_6      /**< USART6 channel */
} t_uart_channel;

/**
 * @brief UART Baud Rate enumeration.
 * Actual values would map to BRR register.
 */
typedef enum
{
    UART_BAUD_RATE_9600 = 0,   /**< 9600 bps */
    UART_BAUD_RATE_19200,      /**< 19200 bps */
    UART_BAUD_RATE_38400,      /**< 38400 bps */
    UART_BAUD_RATE_57600,      /**< 57600 bps */
    UART_BAUD_RATE_115200      /**< 115200 bps */
} t_uart_baud_rate;

/**
 * @brief UART Data Length enumeration.
 * Actual values would map to M bit in CR1.
 */
typedef enum
{
    UART_DATA_LENGTH_8BITS = 0, /**< 8-bit data length */
    UART_DATA_LENGTH_9BITS      /**< 9-bit data length */
} t_uart_data_length;

/**
 * @brief UART Stop Bit enumeration.
 * Actual values would map to STOP bits in CR2.
 */
typedef enum
{
    UART_STOP_BIT_1 = 0,   /**< 1 stop bit */
    UART_STOP_BIT_0_5,     /**< 0.5 stop bits */
    UART_STOP_BIT_2,       /**< 2 stop bits */
    UART_STOP_BIT_1_5      /**< 1.5 stop bits */
} t_uart_stop_bit;

/**
 * @brief UART Parity enumeration.
 * Actual values would map to PCE and PS bits in CR1.
 */
typedef enum
{
    UART_PARITY_NONE = 0, /**< No parity */
    UART_PARITY_EVEN,     /**< Even parity */
    UART_PARITY_ODD       /**< Odd parity */
} t_uart_parity;

/**
 * @brief Initializes the UART module.
 * @param uart_channel The UART channel to initialize.
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data length.
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting.
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);

/**
 * @brief Enables the UART module.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel);

/**
 * @brief Disables the UART module.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel);

/**
 * @brief Updates the UART module (e.g., checks status flags).
 * @param uart_channel The UART channel to update.
 */
void UART_Update(t_uart_channel uart_channel);

/**
 * @brief Sends a single byte over UART.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);

/**
 * @brief Sends a frame of data over UART.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data array.
 * @param length The length of the data to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string over UART.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the string to send.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str);

/**
 * @brief Gets a single byte from UART.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel);

/**
 * @brief Gets a frame of data from UART.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of data to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);

/**
 * @brief Gets a string from UART.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the string to receive.
 * @return The number of bytes received (or actual length of string if terminated).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

/**
 * @brief Clears UART flags.
 * @param uart_channel The UART channel.
 */
void UART_ClearFlag(t_uart_channel uart_channel);

// =============================================================================
// I2C APIs & Type Definitions
// =============================================================================

/**
 * @brief I2C Channel enumeration.
 * Derived from register_json (I2C1, I2C2, I2C3).
 */
typedef enum
{
    I2C_CHANNEL_1 = 0, /**< I2C1 channel */
    I2C_CHANNEL_2,     /**< I2C2 channel */
    I2C_CHANNEL_3      /**< I2C3 channel */
} t_i2c_channel;

/**
 * @brief I2C Clock Speed enumeration.
 * As per I2C_rules, always use fast mode.
 */
typedef enum
{
    I2C_CLK_SPEED_STANDARD = 0, /**< Standard Mode (100 kHz) */
    I2C_CLK_SPEED_FAST          /**< Fast Mode (400 kHz) */
} t_i2c_clk_speed;

/**
 * @brief I2C Device Address type.
 * Assumed to be 7-bit or 10-bit.
 */
typedef uint16_t t_i2c_device_address;

/**
 * @brief I2C Acknowledge control.
 */
typedef enum
{
    I2C_ACK_ENABLE = 0, /**< Enable Acknowledge */
    I2C_ACK_DISABLE     /**< Disable Acknowledge */
} t_i2c_ack;

/**
 * @brief I2C Data Length (dummy, as not explicitly detailed).
 */
typedef enum
{
    I2C_DATALENGTH_8BIT = 0, /**< 8-bit data length */
    I2C_DATALENGTH_16BIT     /**< 16-bit data length */
} t_i2c_datalength;

/**
 * @brief Initializes the I2C module.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (Fast Mode as per rules).
 * @param i2c_device_address The device's own address.
 * @param i2c_ack Acknowledge control setting.
 * @param i2c_datalength The data length (dummy).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);

/**
 * @brief Enables the I2C module.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel);

/**
 * @brief Disables the I2C module.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel);

/**
 * @brief Updates the I2C module (e.g., checks status flags).
 * @param i2c_channel The I2C channel to update.
 */
void I2C_Update(t_i2c_channel i2c_channel);

/**
 * @brief Sends a single byte over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);

/**
 * @brief Sends a frame of data over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data array.
 * @param length The length of the data to send.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the string to send.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);

/**
 * @brief Gets a single byte from I2C.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);

/**
 * @brief Gets a frame of data from I2C.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of data to receive.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);

/**
 * @brief Gets a string from I2C.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the string to receive.
 * @return The number of bytes received (or actual length of string if terminated).
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

/**
 * @brief Clears I2C flags.
 * @param i2c_channel The I2C channel.
 */
void I2C_ClearFlag(t_i2c_channel i2c_channel);

// =============================================================================
// SPI APIs & Type Definitions
// =============================================================================

/**
 * @brief SPI Channel enumeration.
 * Derived from register_json (SPI1, SPI2, SPI3).
 */
typedef enum
{
    SPI_CHANNEL_1 = 0, /**< SPI1 channel */
    SPI_CHANNEL_2,     /**< SPI2 channel */
    SPI_CHANNEL_3      /**< SPI3 channel */
} t_spi_channel;

/**
 * @brief SPI Mode enumeration.
 * As per SPI_rules.
 */
typedef enum
{
    SPI_MODE_MASTER = 0, /**< Master mode */
    SPI_MODE_SLAVE       /**< Slave mode */
} t_spi_mode;

/**
 * @brief SPI Clock Polarity (CPOL) enumeration.
 * As per SPI_rules.
 */
typedef enum
{
    SPI_CPOL_LOW = 0,  /**< Clock To 0 when idle */
    SPI_CPOL_HIGH      /**< Clock To 1 when idle */
} t_spi_cpol;

/**
 * @brief SPI Clock Phase (CPHA) enumeration.
 * As per SPI_rules.
 */
typedef enum
{
    SPI_CPHA_1_EDGE = 0, /**< Data sampled on first clock edge */
    SPI_CPHA_2_EDGE      /**< Data sampled on second clock edge */
} t_spi_cpha;

/**
 * @brief SPI Data Frame Format (DFF) enumeration.
 * As per SPI_rules.
 */
typedef enum
{
    SPI_DFF_8_BIT = 0, /**< 8-bit data frame format */
    SPI_DFF_16_BIT     /**< 16-bit data frame format */
} t_spi_dff;

/**
 * @brief SPI Bit Order enumeration.
 * As per SPI_rules (implicitly for MSB/LSB first).
 */
typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0, /**< MSB transmitted first */
    SPI_BIT_ORDER_LSB_FIRST      /**< LSB transmitted first */
} t_spi_bit_order;

/**
 * @brief Initializes the SPI module.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode The SPI mode (Master/Slave).
 * @param spi_cpol The clock polarity.
 * @param spi_cpha The clock phase.
 * @param spi_dff The data frame format.
 * @param spi_bit_order The bit order.
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);

/**
 * @brief Enables the SPI module.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel);

/**
 * @brief Disables the SPI module.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel);

/**
 * @brief Updates the SPI module (e.g., checks status flags).
 */
void SPI_Update(void);

/**
 * @brief Sends a single byte over SPI.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);

/**
 * @brief Sends a frame of data over SPI.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data array.
 * @param length The length of the data to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);

/**
 * @brief Sends a null-terminated string over SPI.
 * @param spi_channel The SPI channel to use.
 * @param str Pointer to the string to send.
 */
void SPI_send_string(t_spi_channel spi_channel, const char *str);

/**
 * @brief Gets a single byte from SPI.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel);

/**
 * @brief Gets a frame of data from SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of data to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);

/**
 * @brief Gets a string from SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the string to receive.
 * @return The number of bytes received (or actual length of string if terminated).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

/**
 * @brief Clears SPI flags.
 * @param spi_channel The SPI channel.
 */
void SPI_ClearFlag(t_spi_channel spi_channel);

// =============================================================================
// External Interrupt APIs & Type Definitions
// =============================================================================

/**
 * @brief External Interrupt Channel (placeholder, as not specified in register_json).
 */
typedef enum
{
    EXTI_CHANNEL_0 = 0, /**< External Interrupt Line 0 */
    EXTI_CHANNEL_1,     /**< External Interrupt Line 1 */
    EXTI_CHANNEL_2,     /**< External Interrupt Line 2 */
    EXTI_CHANNEL_3,     /**< External Interrupt Line 3 */
    EXTI_CHANNEL_4,     /**< External Interrupt Line 4 */
    EXTI_CHANNEL_5,     /**< External Interrupt Line 5 */
    EXTI_CHANNEL_6,     /**< External Interrupt Line 6 */
    EXTI_CHANNEL_7,     /**< External Interrupt Line 7 */
    EXTI_CHANNEL_8,     /**< External Interrupt Line 8 */
    EXTI_CHANNEL_9,     /**< External Interrupt Line 9 */
    EXTI_CHANNEL_10,    /**< External Interrupt Line 10 */
    EXTI_CHANNEL_11,    /**< External Interrupt Line 11 */
    EXTI_CHANNEL_12,    /**< External Interrupt Line 12 */
    EXTI_CHANNEL_13,    /**< External Interrupt Line 13 */
    EXTI_CHANNEL_14,    /**< External Interrupt Line 14 */
    EXTI_CHANNEL_15     /**< External Interrupt Line 15 */
} t_external_int_channel;

/**
 * @brief External Interrupt Edge Trigger enumeration.
 */
typedef enum
{
    EXTI_EDGE_RISING = 0,        /**< Rising edge trigger */
    EXTI_EDGE_FALLING,           /**< Falling edge trigger */
    EXTI_EDGE_RISING_FALLING     /**< Rising and falling edge trigger */
} t_external_int_edge;

/**
 * @brief Initializes an external interrupt channel.
 * @param external_int_channel The external interrupt channel.
 * @param external_int_edge The edge trigger configuration.
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);

/**
 * @brief Enables an external interrupt channel.
 * @param external_int_channel The external interrupt channel.
 */
void External_INT_Enable(t_external_int_channel external_int_channel);

/**
 * @brief Disables an external interrupt channel.
 * @param external_int_channel The external interrupt channel.
 */
void External_INT_Disable(t_external_int_channel external_int_channel);

/**
 * @brief Clears the external interrupt flag.
 * @param external_int_channel The external interrupt channel.
 */
void External_INT_ClearFlag(t_external_int_channel external_int_channel);

// =============================================================================
// GPIO APIs & Type Definitions
// =============================================================================

/**
 * @brief GPIO Port enumeration.
 * Derived from common STM32 GPIO ports.
 */
typedef enum
{
    PORTA = 0, /**< GPIO Port A */
    PORTB,     /**< GPIO Port B */
    PORTC,     /**< GPIO Port C */
    PORTD,     /**< GPIO Port D */
    PORTE,     /**< GPIO Port E */
    PORTH      /**< GPIO Port H (STM32F401 has H) */
} t_port;

/**
 * @brief GPIO Pin enumeration.
 */
typedef enum
{
    PIN0 = 0, /**< Pin 0 */
    PIN1,     /**< Pin 1 */
    PIN2,     /**< Pin 2 */
    PIN3,     /**< Pin 3 */
    PIN4,     /**< Pin 4 */
    PIN5,     /**< Pin 5 */
    PIN6,     /**< Pin 6 */
    PIN7,     /**< Pin 7 */
    PIN8,     /**< Pin 8 */
    PIN9,     /**< Pin 9 */
    PIN10,    /**< Pin 10 */
    PIN11,    /**< Pin 11 */
    PIN12,    /**< Pin 12 */
    PIN13,    /**< Pin 13 */
    PIN14,    /**< Pin 14 */
    PIN15     /**< Pin 15 */
} t_pin;

/**
 * @brief GPIO Direction enumeration.
 */
typedef enum
{
    DIRECTION_INPUT = 0,  /**< Input direction */
    DIRECTION_OUTPUT      /**< Output direction */
} t_direction;

/**
 * @brief Initializes a GPIO pin for output.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @param value The initial value for the output pin (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);

/**
 * @brief Initializes a GPIO pin for input.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 */
void GPIO_Input_Init(t_port port, t_pin pin);

/**
 * @brief Gets the direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @return The direction of the GPIO pin (input or output).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin);

/**
 * @brief Sets the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @param value The value to set (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);

/**
 * @brief Gets the value of a GPIO input pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @return The value of the GPIO input pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin);

/**
 * @brief Toggles the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 */
void GPIO_Value_Tog(t_port port, t_pin pin);

// =============================================================================
// PWM APIs & Type Definitions
// =============================================================================

/**
 * @brief PWM Channel (placeholder, as not specified in register_json).
 * Example comment from rules: PWM_Channel_TIM1_CH1 //PA8,PE9
 */
typedef enum
{
    PWM_CHANNEL_TIM1_CH1 = 0, /**< Timer 1 Channel 1 (PA8, PE9) */
    PWM_CHANNEL_TIM1_CH2,     /**< Timer 1 Channel 2 (PA9, PE11) */
    PWM_CHANNEL_TIM2_CH1      /**< Timer 2 Channel 1 (PA0, PA5, PA15) */
    // Add more channels as needed based on specific timer/pin mappings not in current JSON
} t_pwm_channel;

/**
 * @brief Initializes a PWM channel.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);

/**
 * @brief Starts a PWM channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel);

/**
 * @brief Stops a PWM channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel);

// =============================================================================
// ICU APIs & Type Definitions
// =============================================================================

/**
 * @brief ICU Channel (placeholder, as not specified in register_json).
 */
typedef enum
{
    ICU_CHANNEL_1 = 0, /**< Input Capture Unit Channel 1 */
    ICU_CHANNEL_2      /**< Input Capture Unit Channel 2 */
    // Add more channels as needed
} t_icu_channel;

/**
 * @brief ICU Prescaler (placeholder).
 */
typedef enum
{
    ICU_PRESCALER_1 = 0, /**< Prescaler 1:1 */
    ICU_PRESCALER_2,     /**< Prescaler 1:2 */
    ICU_PRESCALER_4,     /**< Prescaler 1:4 */
    ICU_PRESCALER_8      /**< Prescaler 1:8 */
} t_icu_prescaller;

/**
 * @brief ICU Edge (placeholder).
 */
typedef enum
{
    ICU_EDGE_RISING = 0,  /**< Rising edge capture */
    ICU_EDGE_FALLING      /**< Falling edge capture */
} t_icu_edge;

/**
 * @brief Initializes the ICU module.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler setting.
 * @param icu_edge The edge detection setting.
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);

/**
 * @brief Enables the ICU module.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel);

/**
 * @brief Disables the ICU module.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel);

/**
 * @brief Updates the ICU frequency calculation.
 * @param icu_channel The ICU channel to update.
 */
void ICU_Updatefrequency(t_icu_channel icu_channel);

/**
 * @brief Gets the frequency measured by the ICU.
 * @param icu_channel The ICU channel.
 * @return The measured frequency.
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel); // Changed return type to tlong for frequency

/**
 * @brief Sets up the buffer for remote control keys.
 * @param number_of_keys Number of remote control keys to store.
 * @param key_digits_length Length of digits for each key.
 */
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length);

/**
 * @brief Sets individual digits for a remote control key.
 * @param key_num The key number.
 * @param key_array_cell The cell index within the key's digit array.
 * @param key_cell_value The value to set in the cell.
 */
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value);

/**
 * @brief Updates remote control signal parameters.
 * @param icu_channel The ICU channel.
 * @param strt_bit_us_value Start bit duration in microseconds.
 * @param one_bit_us_value Logic '1' bit duration in microseconds.
 * @param zero_bit_us_value Logic '0' bit duration in microseconds.
 * @param stop_bit_us_value Stop bit duration in microseconds.
 */
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value);

/**
 * @brief Gets the pressed remote control key.
 * @param icu_channel The ICU channel.
 * @return The detected remote control key.
 */
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel);

/**
 * @brief Sets a callback function for the ICU module.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void));

/**
 * @brief Clears ICU flags.
 * @param icu_channel The ICU channel.
 */
void ICU_ClearFlag(t_icu_channel icu_channel);

// =============================================================================
// Timer APIs & Type Definitions
// =============================================================================

/**
 * @brief Timer Channel (placeholder, as not specified in register_json).
 */
typedef enum
{
    TIMER_CHANNEL_1 = 0, /**< Timer Channel 1 (e.g., TIM2) */
    TIMER_CHANNEL_2,     /**< Timer Channel 2 (e.g., TIM3) */
    TIMER_CHANNEL_3      /**< Timer Channel 3 (e.g., TIM4) */
    // Add more channels as needed
} t_timer_channel;

/**
 * @brief Initializes a timer channel.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel);

/**
 * @brief Sets the timer period in microseconds.
 * @param timer_channel The timer channel.
 * @param time The time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time);

/**
 * @brief Sets the timer period in milliseconds.
 * @param timer_channel The timer channel.
 * @param time The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);

/**
 * @brief Sets the timer period in seconds.
 * @param timer_channel The timer channel.
 * @param time The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Sets the timer period in minutes.
 * @param timer_channel The timer channel.
 * @param time The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Sets the timer period in hours.
 * @param timer_channel The timer channel.
 * @param time The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);

/**
 * @brief Enables a timer channel.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel);

/**
 * @brief Disables a timer channel.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel);

/**
 * @brief Clears timer flags.
 * @param timer_channel The timer channel.
 */
void TIMER_ClearFlag(t_timer_channel timer_channel);

// =============================================================================
// ADC APIs & Type Definitions
// =============================================================================

/**
 * @brief ADC Channel (placeholder, as not specified in register_json).
 */
typedef enum
{
    ADC_CHANNEL_0 = 0, /**< ADC Channel 0 */
    ADC_CHANNEL_1,     /**< ADC Channel 1 */
    ADC_CHANNEL_2,     /**< ADC Channel 2 */
    ADC_CHANNEL_3,     /**< ADC Channel 3 */
    ADC_CHANNEL_4,     /**< ADC Channel 4 */
    ADC_CHANNEL_5,     /**< ADC Channel 5 */
    ADC_CHANNEL_6,     /**< ADC Channel 6 */
    ADC_CHANNEL_7,     /**< ADC Channel 7 */
    ADC_CHANNEL_8,     /**< ADC Channel 8 */
    ADC_CHANNEL_9,     /**< ADC Channel 9 */
    ADC_CHANNEL_10,    /**< ADC Channel 10 */
    ADC_CHANNEL_11,    /**< ADC Channel 11 */
    ADC_CHANNEL_12,    /**< ADC Channel 12 */
    ADC_CHANNEL_13,    /**< ADC Channel 13 */
    ADC_CHANNEL_14,    /**< ADC Channel 14 */
    ADC_CHANNEL_15     /**< ADC Channel 15 */
} t_adc_channel;

/**
 * @brief ADC Mode enumeration.
 */
typedef enum
{
    ADC_MODE_SINGLE_CONVERSION = 0, /**< Single conversion mode */
    ADC_MODE_CONTINUOUS             /**< Continuous conversion mode */
} t_adc_mode_t;

/**
 * @brief Initializes the ADC module.
 * @param adc_channel The ADC channel to initialize (for regular conversion).
 * @param adc_mode The ADC operating mode.
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);

/**
 * @brief Enables the ADC module.
 */
void ADC_Enable(void);

/**
 * @brief Disables the ADC module.
 */
void ADC_Disable(void);

/**
 * @brief Updates the ADC module (e.g., starts conversion).
 */
void ADC_Update(void);

/**
 * @brief Gets the ADC conversion result.
 * @return The 12-bit ADC conversion value.
 */
tword ADC_Get(void);

/**
 * @brief Clears ADC flags.
 */
void ADC_ClearFlag(void);

// =============================================================================
// Internal EEPROM APIs & Type Definitions
// =============================================================================

/**
 * @brief Initializes the internal EEPROM module.
 */
void Internal_EEPROM_Init(void);

/**
 * @brief Sets a byte of data in the internal EEPROM.
 * @param address The address in EEPROM to write to.
 * @param data The byte of data to write.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data);

/**
 * @brief Gets a byte of data from the internal EEPROM.
 * @param address The address in EEPROM to read from.
 * @return The byte of data read from EEPROM.
 */
tbyte Internal_EEPROM_Get(tbyte address);

// =============================================================================
// TT (Time Triggered OS) APIs & Type Definitions
// =============================================================================

/**
 * @brief Tick Time for TT OS in milliseconds.
 */
typedef enum
{
    TT_TICK_TIME_1MS = 0,   /**< 1ms tick time */
    TT_TICK_TIME_10MS,      /**< 10ms tick time */
    TT_TICK_TIME_100MS      /**< 100ms tick time */
} t_tick_time;

/**
 * @brief Initializes the Time Triggered OS.
 * @param tick_time_ms The desired tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms);

/**
 * @brief Starts the Time Triggered OS scheduler.
 */
void TT_Start(void);

/**
 * @brief Dispatches tasks in the Time Triggered OS.
 */
void TT_Dispatch_task(void);

/**
 * @brief Interrupt Service Routine (ISR) for the Time Triggered OS.
 */
void TT_ISR(void);

/**
 * @brief Adds a task to the Time Triggered OS scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks.
 * @param delay The initial delay of the task in ticks.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);

/**
 * @brief Deletes a task from the Time Triggered OS scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index);

#endif /* MCAL_H */
