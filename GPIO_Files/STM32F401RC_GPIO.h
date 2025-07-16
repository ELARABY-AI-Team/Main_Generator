/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Provides a driver interface for GPIO peripheral.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-14
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

// Include project-specific configuration and main types
#include "STM32F401RC_Config.h"

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
 * @brief Defines the intended usage of a GPIO pin.
 */
typedef enum
{
    normal_usage,      /* Standard general purpose input/output usage */
    communication_usage /* Usage for specific communication protocols */
} t_usage;

/**
 * @brief Defines the operational direction mode of a GPIO pin.
 * @note Limited to standard input, output, or analog modes as per requirement.
 */
typedef enum
{
    output,           /* Configure pin as output */
    input,            /* Configure pin as input */
    analog            /* Configure pin as analog input */
} t_direction;

/**
 * @brief Defines the pull resistor configuration for input pins.
 * @note Limited to pull-up or pull-down as per requirement.
 */
typedef enum
{
    pull_up,          /* Enable internal pull-up resistor */
    pull_down         /* Enable internal pull-down resistor */
} t_pull;

/**
 * @brief Defines the output connection type for output pins.
 */
typedef enum
{
    push_pull,        /* Output configured as push-pull */
    open_drain        /* Output configured as open-drain */
} t_output_conn;

/* ==================== HARDWARE DEFINITIONS ==================== */

/**
 * @brief Defines the available GPIO ports on the STM32F401RC microcontroller (LQFP64 package).
 */
typedef enum
{
    port_A,  /* PDF Reference */
    port_B,  /* PDF Reference */
    port_C,  /* PDF Reference */
    port_H   /* PDF Reference */
} tport;

/**
 * @brief Defines the available pin numbers within a GPIO port.
 */
typedef enum
{
    pin_0,   /* PDF Reference */
    pin_1,   /* PDF Reference */
    pin_2,   /* PDF Reference */
    pin_3,   /* PDF Reference */
    pin_4,   /* PDF Reference */
    pin_5,   /* PDF Reference */
    pin_6,   /* PDF Reference */
    pin_7,   /* PDF Reference */
    pin_8,   /* PDF Reference */
    pin_9,   /* PDF Reference */
    pin_10,  /* PDF Reference */
    pin_11,  /* PDF Reference */
    pin_12,  /* PDF Reference */
    pin_13,  /* PDF Reference */
    pin_14,  /* PDF Reference */
    pin_15   /* PDF Reference */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a GPIO pin for output operation.
 * @param[in] port: The GPIO port to initialize (type tport).
 * @param[in] pin: The pin number to initialize (type tpin).
 * @param[in] value: The initial output value (0 or 1) (type tbyte).
 * @param[in] usage: The intended usage of the pin (type t_usage).
 * @param[in] conn: The output connection type (push-pull or open-drain) (type t_output_conn).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
 * @brief Initializes a GPIO pin for input operation.
 * @param[in] port: The GPIO port to initialize (type tport).
 * @param[in] pin: The pin number to initialize (type tpin).
 * @param[in] usage: The intended usage of the pin (type t_usage).
 * @param[in] pull: The pull resistor configuration (pull-up or pull-down) (type t_pull).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/**
 * @brief Gets the current direction mode of a GPIO pin.
 * @param[in] port: The GPIO port (type tport).
 * @param[in] pin: The pin number (type tpin).
 * @return The current direction mode of the pin (type t_direction).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
 * @brief Sets the output value of a GPIO pin configured as output.
 * @param[in] port: The GPIO port (type tport).
 * @param[in] pin: The pin number (type tpin).
 * @param[in] value: The value to set (0 or 1) (type tbyte).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
 * @brief Gets the input value of a GPIO pin.
 * @param[in] port: The GPIO port (type tport).
 * @param[in] pin: The pin number (type tpin).
 * @return The input value (0 or 1) (type tbyte).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
 * @brief Toggles the output value of a GPIO pin configured as output.
 * @param[in] port: The GPIO port (type tport).
 * @param[in] pin: The pin number (type tpin).
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F401RC_GPIO_H_ */