/***********************************************************************************************************************
* File Name      : STM32F410RC_GPIO.h
* Description    : GPIO peripheral header file for STM32F410RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F410RC_GPIO_H_
#define STM32F410RC_GPIO_H_

/*
 * Include configuration file for basic types (tbyte, tword, etc.)
 * Note: Assumes tbyte, tword, etc. are defined within this header or headers it includes.
 */
#include "STM32F410RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
 * @brief Defines the intended usage context of a GPIO pin.
 */
typedef enum
{
  normal_usage,     /* Pin used for general digital input/output logic */
  communication_usage /* Pin used for peripheral communication functions (e.g., SPI, I2C, UART) */
} t_usage;

/**
 * @brief Defines the direction mode configuration for a GPIO pin.
 */
typedef enum
{
  output, /* Pin configured as a digital output */
  input,  /* Pin configured as a digital input */
  analog  /* Pin configured for analog input/output */
} t_direction;

/**
 * @brief Defines the pull resistor configuration for an input pin.
 * Note: No_pull configuration is not explicitly defined in this enum as per requirements.
 *       The implementation must handle the 'no_pull' case if required by the use case
 *       when initializing inputs.
 */
typedef enum
{
  pull_up,   /* Internal pull-up resistor is enabled */
  pull_down  /* Internal pull-down resistor is enabled */
} t_pull;

/**
 * @brief Defines the output driving connection type for an output pin.
 */
typedef enum
{
  push_pull,  /* Output configured as push-pull */
  open_drain  /* Output configured as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/**
 * @brief Defines the available GPIO ports on the STM32F410RC microcontroller.
 */
typedef enum
{
  port_A, /* PDF Reference: GPIO Port A available on STM32F410RC */
  port_B, /* PDF Reference: GPIO Port B available on STM32F410RC */
  port_C, /* PDF Reference: GPIO Port C available on STM32F410RC */
  port_H  /* PDF Reference: GPIO Port H available on STM32F410RC (often for OSC) */
} tport;

/**
 * @brief Defines the available pin numbers within a GPIO port (0-15 standard).
 */
typedef enum
{
  pin_0,  /* PDF Reference: Pin 0 */
  pin_1,  /* PDF Reference: Pin 1 */
  pin_2,  /* PDF Reference: Pin 2 */
  pin_3,  /* PDF Reference: Pin 3 */
  pin_4,  /* PDF Reference: Pin 4 */
  pin_5,  /* PDF Reference: Pin 5 */
  pin_6,  /* PDF Reference: Pin 6 */
  pin_7,  /* PDF Reference: Pin 7 */
  pin_8,  /* PDF Reference: Pin 8 */
  pin_9,  /* PDF Reference: Pin 9 */
  pin_10, /* PDF Reference: Pin 10 */
  pin_11, /* PDF Reference: Pin 11 */
  pin_12, /* PDF Reference: Pin 12 */
  pin_13, /* PDF Reference: Pin 13 */
  pin_14, /* PDF Reference: Pin 14 */
  pin_15  /* PDF Reference: Pin 15 */
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a specified GPIO pin as an output.
 * This function configures the pin mode, output type, and initial state.
 * Clock gating for the respective GPIO port must be enabled prior to calling this function.
 * @param port: The GPIO port to configure.
 * @param pin: The pin number within the port to configure.
 * @param value: The initial output value (0 for logic low, any non-zero value for logic high).
 * @param usage: Specifies the intended usage context of the pin.
 * @param conn: Specifies the output connection type (push-pull or open-drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a specified GPIO pin as an input.
 * This function configures the pin mode and pull-up/pull-down resistors.
 * Clock gating for the respective GPIO port must be enabled prior to calling this function.
 * @param port: The GPIO port to configure.
 * @param pin: The pin number within the port to configure.
 * @param usage: Specifies the intended usage context of the pin.
 * @param pull: Specifies the pull resistor configuration (pull-up or pull-down).
 *              Note: The absence of pull (no-pull) configuration must be handled by the caller
 *              or the implementation if it's not covered by the provided enum.
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull); /* MISRA Note: t_pull enum does not include no_pull */

/*
 * @brief Gets the current direction configuration of a specified GPIO pin.
 * @param port: The GPIO port of the pin.
 * @param pin: The pin number.
 * @return The configured direction of the pin (output, input, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a specified GPIO pin.
 * This function is typically used for pins configured as digital output.
 * @param port: The GPIO port of the pin.
 * @param pin: The pin number.
 * @param value: The value to set (0 for logic low, any non-zero value for logic high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the input value of a specified GPIO pin.
 * This function is typically used for pins configured as digital input.
 * @param port: The GPIO port of the pin.
 * @param pin: The pin number.
 * @return The input value (0 for logic low, 1 for logic high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a specified GPIO pin.
 * This function is typically used for pins configured as digital output.
 * @param port: The GPIO port of the pin.
 * @param pin: The pin number.
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F410RC_GPIO_H_ */