/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : GPIO driver header file for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
 * @enum   t_usage
 * @brief  Defines the intended usage of a GPIO pin.
 */
typedef enum
{
    normal_usage,       /* Normal GPIO functionality */
    communication_usage /* GPIO used for communication protocol (e.g., SPI, I2C, UART) */
} t_usage;

/*
 * @enum   t_direction
 * @brief  Defines the direction mode of a GPIO pin.
 */
typedef enum
{
    output, /* Pin configured as output */
    input,  /* Pin configured as input */
    analog  /* Pin configured as analog input/output */
} t_direction;

/*
 * @enum   t_pull
 * @brief  Defines the pull resistor settings for an input pin.
 */
typedef enum
{
    pull_up,   /* Enable internal pull-up resistor */
    pull_down  /* Enable internal pull-down resistor */
} t_pull;

/*
 * @enum   t_output_conn
 * @brief  Defines the output driving connection type.
 */
typedef enum
{
    push_pull,  /* Push-pull output configuration */
    open_drain  /* Open-drain output configuration */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/*
 * @enum   tport
 * @brief  Enumerates the available GPIO ports on the STM32F401RC.
 */
typedef enum
{
    port_A, /* PDF Reference */
    port_B, /* PDF Reference */
    port_C, /* PDF Reference */
    port_D, /* PDF Reference */
    port_E, /* PDF Reference */
    port_H  /* PDF Reference */
} tport;

/*
 * @enum   tpin
 * @brief  Enumerates the available pin numbers within a GPIO port (0-15).
 */
typedef enum
{
    pin_0,  /* PDF Reference */
    pin_1,  /* PDF Reference */
    pin_2,  /* PDF Reference */
    pin_3,  /* PDF Reference */
    pin_4,  /* PDF Reference */
    pin_5,  /* PDF Reference */
    pin_6,  /* PDF Reference */
    pin_7,  /* PDF Reference */
    pin_8,  /* PDF Reference */
    pin_9,  /* PDF Reference */
    pin_10, /* PDF Reference */
    pin_11, /* PDF Reference */
    pin_12, /* PDF Reference */
    pin_13, /* PDF Reference */
    pin_14, /* PDF Reference */
    pin_15  /* PDF Reference */
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief  Initializes a GPIO pin as output.
 * @param  port: The GPIO port (tport).
 * @param  pin: The GPIO pin number (tpin).
 * @param  value: Initial output value (0 for low, 1 for high).
 * @param  usage: Intended usage of the pin (t_usage).
 * @param  conn: Output connection type (t_output_conn).
 * @retval None
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief  Initializes a GPIO pin as input.
 * @param  port: The GPIO port (tport).
 * @param  pin: The GPIO pin number (tpin).
 * @param  usage: Intended usage of the pin (t_usage).
 * @param  pull: Pull resistor setting (t_pull).
 * @retval None
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief  Gets the current direction mode of a GPIO pin.
 * @param  port: The GPIO port (tport).
 * @param  pin: The GPIO pin number (tpin).
 * @retval The direction mode of the pin (t_direction).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief  Sets the output value of a GPIO pin configured as output.
 * @param  port: The GPIO port (tport).
 * @param  pin: The GPIO pin number (tpin).
 * @param  value: Output value to set (0 for low, 1 for high).
 * @retval None
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief  Gets the input value of a GPIO pin configured as input.
 * @param  port: The GPIO port (tport).
 * @param  pin: The GPIO pin number (tpin).
 * @retval The input value of the pin (0 for low, 1 for high - tbyte).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief  Toggles the output value of a GPIO pin configured as output.
 * @param  port: The GPIO port (tport).
 * @param  pin: The GPIO pin number (tpin).
 * @retval None
 */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */