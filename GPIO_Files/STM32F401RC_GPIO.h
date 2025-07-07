/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for GPIO peripheral drivers for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_Config.h" /* Includes base types like tbyte, tword and potentially other necessary headers */

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
 * @brief Defines the usage context of the GPIO pin.
 *        normal_usage: Used for simple digital input/output logic.
 *        communication_usage: Used for specific communication protocols (e.g., SPI, I2C, UART).
 */
typedef enum
{
  normal_usage      = 0, /* GPIO pin used for general logic control */
  communication_usage = 1  /* GPIO pin used as part of a communication interface */
} t_usage;

/*
 * @brief Defines the direction mode of the GPIO pin.
 *        output: Pin is configured as an output.
 *        input: Pin is configured as an input.
 *        analog: Pin is configured for analog input/output.
 */
typedef enum
{
  output = 0, /* Pin configured as output */
  input  = 1, /* Pin configured as input */
  analog = 2  /* Pin configured for analog mode */
} t_direction;

/*
 * @brief Defines the pull resistor configuration for input pins.
 *        pull_up: Internal pull-up resistor enabled.
 *        pull_down: Internal pull-down resistor enabled.
 */
typedef enum
{
  pull_up   = 0, /* Internal pull-up resistor enabled */
  pull_down = 1  /* Internal pull-down resistor enabled */
} t_pull;

/*
 * @brief Defines the output driver connection type for output pins.
 *        push_pull: Output stage is configured as push-pull.
 *        open_drain: Output stage is configured as open-drain.
 */
typedef enum
{
  push_pull  = 0, /* Output configured as push-pull */
  open_drain = 1  /* Output configured as open-drain */
} t_output_conn;

/* ==================== HARDWARE DEFINITIONS ==================== */

/*
 * @brief Defines the available GPIO ports on the STM32F401RC microcontroller.
 */
typedef enum
{
  port_A = 0, /* PDF Reference */
  port_B = 1, /* PDF Reference */
  port_C = 2, /* PDF Reference */
  port_D = 3, /* PDF Reference */
  port_E = 4, /* PDF Reference */
  port_H = 5  /* PDF Reference - Note: GPIO H typically has fewer pins */
} tport;

/*
 * @brief Defines the available pin numbers within a GPIO port (0 to 15).
 */
typedef enum
{
  pin_0  = 0,  /* PDF Reference */
  pin_1  = 1,  /* PDF Reference */
  pin_2  = 2,  /* PDF Reference */
  pin_3  = 3,  /* PDF Reference */
  pin_4  = 4,  /* PDF Reference */
  pin_5  = 5,  /* PDF Reference */
  pin_6  = 6,  /* PDF Reference */
  pin_7  = 7,  /* PDF Reference */
  pin_8  = 8,  /* PDF Reference */
  pin_9  = 9,  /* PDF Reference */
  pin_10 = 10, /* PDF Reference */
  pin_11 = 11, /* PDF Reference */
  pin_12 = 12, /* PDF Reference */
  pin_13 = 13, /* PDF Reference */
  pin_14 = 14, /* PDF Reference */
  pin_15 = 15  /* PDF Reference */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a GPIO pin as an output.
 * @param port: The GPIO port (e.g., port_A, port_B).
 * @param pin: The pin number within the port (e.g., pin_0, pin_15).
 * @param value: The initial output value (0 for low, 1 for high).
 * @param usage: The usage context (normal_usage, communication_usage).
 * @param conn: The output connection type (push_pull, open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a GPIO pin as an input.
 * @param port: The GPIO port (e.g., port_A, port_B).
 * @param pin: The pin number within the port (e.g., pin_0, pin_15).
 * @param usage: The usage context (normal_usage, communication_usage).
 * @param pull: The pull resistor configuration (pull_up, pull_down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief Gets the current direction configuration of a GPIO pin.
 * @param port: The GPIO port.
 * @param pin: The pin number.
 * @return The direction configuration (output, input, analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a GPIO pin (if configured as output).
 * @param port: The GPIO port.
 * @param pin: The pin number.
 * @param value: The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the input value of a GPIO pin (if configured as input) or the output level (if configured as output).
 * @param port: The GPIO port.
 * @param pin: The pin number.
 * @return The pin value (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a GPIO pin (if configured as output).
 * @param port: The GPIO port.
 * @param pin: The pin number.
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F401RC_GPIO_H_ */