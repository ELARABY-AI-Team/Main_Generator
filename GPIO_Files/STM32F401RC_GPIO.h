/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : GPIO peripheral header file for STM32F410RC
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
* Include the configuration file which is expected to define
* necessary types like tbyte, tword, etc.
*/
#include "STM32F410RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
* @enum t_usage
* @brief Defines the intended usage type of the GPIO pin.
*/
typedef enum
{
    normal_usage,       /* GPIO used for general-purpose logic */
    communication_usage /* GPIO used for communication protocols */
} t_usage;

/**
* @enum t_direction
* @brief Defines the direction mode of the GPIO pin.
*/
typedef enum
{
    output, /* GPIO configured as an output */
    input,  /* GPIO configured as an input */
    analog  /* GPIO configured as an analog input/output */
} t_direction;

/**
* @enum t_pull
* @brief Defines pull resistor settings for input pins.
*/
typedef enum
{
    pull_up,   /* Enable internal pull-up resistor */
    pull_down  /* Enable internal pull-down resistor */
} t_pull;

/**
* @enum t_output_conn
* @brief Defines the output driving connection type.
*/
typedef enum
{
    push_pull,  /* Output configured as push-pull */
    open_drain  /* Output configured as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/**
* @enum tport
* @brief Defines the available GPIO ports on the STM32F410RC.
*/
typedef enum
{
    port_A, /* PDF Reference */
    port_B, /* PDF Reference */
    port_C, /* PDF Reference */
    port_H  /* PDF Reference */
} tport;

/**
* @enum tpin
* @brief Defines the available pin indices within a GPIO port.
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

/**
* @brief Initializes a GPIO pin as an output.
* @param[in] port: The GPIO port (tport).
* @param[in] pin: The pin number (tpin).
* @param[in] value: The initial output value (0 or 1) (tbyte).
* @param[in] usage: The intended usage (t_usage).
* @param[in] conn: The output connection type (t_output_conn).
*/
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
* @brief Initializes a GPIO pin as an input.
* @param[in] port: The GPIO port (tport).
* @param[in] pin: The pin number (tpin).
* @param[in] usage: The intended usage (t_usage).
* @param[in] pull: The pull resistor configuration (t_pull).
*/
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/**
* @brief Gets the current direction mode of a GPIO pin.
* @param[in] port: The GPIO port (tport).
* @param[in] pin: The pin number (tpin).
* @return The direction mode of the pin (t_direction).
*/
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
* @brief Sets the output value of a GPIO pin configured as output.
* @param[in] port: The GPIO port (tport).
* @param[in] pin: The pin number (tpin).
* @param[in] value: The value to set (0 or 1) (tbyte).
*/
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
* @brief Gets the input value of a GPIO pin configured as input.
* @param[in] port: The GPIO port (tport).
* @param[in] pin: The pin number (tpin).
* @return The current value of the pin (0 or 1) (tbyte).
*/
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
* @brief Toggles the output value of a GPIO pin configured as output.
* @param[in] port: The GPIO port (tport).
* @param[in] pin: The pin number (tpin).
*/
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F410RC_GPIO_H_ */