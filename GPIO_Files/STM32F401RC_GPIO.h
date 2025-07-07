/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for the GPIO peripheral driver for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/* Include necessary configuration and type definitions */
/* Assumes STM32F401RC_Config.h provides types like tbyte */
#include "STM32F401RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
* @brief Enumeration for GPIO usage type
* Defines whether the GPIO is used for general or communication-specific logic
*/
typedef enum
{
    normal_usage,       /* GPIO used for standard digital I/O */
    communication_usage /* GPIO used for specific communication protocol (e.g., UART, SPI) */
} t_usage;

/*
* @brief Enumeration for GPIO direction mode
* Defines the direction mode of the GPIO pin
*/
typedef enum
{
    output, /* GPIO pin is configured as output */
    input,  /* GPIO pin is configured as input */
    analog  /* GPIO pin is configured as analog */
} t_direction;

/*
* @brief Enumeration for GPIO pull resistor settings
* Defines pull resistor settings for input pins
*/
typedef enum
{
    pull_up,   /* Enable internal pull-up resistor */
    pull_down  /* Enable internal pull-down resistor */
    /* No pull-up/pull-down state is typically represented by a separate parameter */
} t_pull;

/*
* @brief Enumeration for GPIO output connection type
* Defines the output driving connection type
*/
typedef enum
{
    push_pull,  /* Output configured as push-pull */
    open_drain  /* Output configured as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/*
* @brief Enumeration for available GPIO ports
* Lists all GPIO ports available on the STM32F401RC
*/
typedef enum
{
    port_A, /* PDF Reference */
    port_B, /* PDF Reference */
    port_C, /* PDF Reference */
    port_D, /* PDF Reference */
    port_E, /* PDF Reference */
    /* Port F, G typically not available/bonded on STM32F401RC packages like RC */
    port_H  /* PDF Reference */
} tport;

/*
* @brief Enumeration for available GPIO pins
* Lists pin numbers within a GPIO port (0-15)
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
* @brief Initializes a GPIO pin as output.
* @param port: The GPIO port (e.g., port_A).
* @param pin: The pin number within the port (e.g., pin_0).
* @param value: Initial output value (0 for low, non-zero for high).
* @param usage: The intended usage of the pin (e.g., normal_usage).
* @param conn: The output connection type (e.g., push_pull).
*/
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
* @brief Initializes a GPIO pin as input.
* @param port: The GPIO port (e.g., port_A).
* @param pin: The pin number within the port (e.g., pin_0).
* @param usage: The intended usage of the pin (e.g., normal_usage).
* @param pull: The pull resistor configuration (e.g., pull_up).
*/
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
* @brief Gets the current direction configuration of a GPIO pin.
* @param port: The GPIO port (e.g., port_A).
* @param pin: The pin number within the port (e.g., pin_0).
* @return The direction of the pin (e.g., output, input, analog).
*/
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
* @brief Sets the output value of a GPIO pin configured as output.
* @param port: The GPIO port (e.g., port_A).
* @param pin: The pin number within the port (e.g., pin_0).
* @param value: Value to set (0 for low, non-zero for high).
*/
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
* @brief Gets the current value of a GPIO pin.
* @param port: The GPIO port (e.g., port_A).
* @param pin: The pin number within the port (e.g., pin_0).
* @return The pin value (0 for low, 1 for high).
*/
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
* @brief Toggles the output value of a GPIO pin configured as output.
* @param port: The GPIO port (e.g., port_A).
* @param pin: The pin number within the port (e.g., pin_0).
*/
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */