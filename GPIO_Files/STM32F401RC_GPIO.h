/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO_H_
* Description    : GPIO Driver header file for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_Config.h"

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
 * @brief Defines the intended usage of a GPIO pin.
 */
typedef enum
{
    normal_usage,       /* Purpose: General purpose I/O or standard function */
    communication_usage /* Purpose: Used for communication protocols (e.g., SPI, I2C, UART) */
} t_usage;

/**
 * @brief Defines the direction mode of a GPIO pin.
 */
typedef enum
{
    output, /* Purpose: Pin configured as output */
    input,  /* Purpose: Pin configured as input */
    analog  /* Purpose: Pin configured in analog mode */
} t_direction;

/**
 * @brief Defines the pull resistor settings for input pins.
 */
typedef enum
{
    pull_up,   /* Purpose: Enable internal pull-up resistor */
    pull_down  /* Purpose: Enable internal pull-down resistor */
} t_pull;

/**
 * @brief Defines the output driving connection type.
 */
typedef enum
{
    push_pull,  /* Purpose: Push-pull output configuration */
    open_drain  /* Purpose: Open-drain output configuration */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/**
 * @brief Enumerates the available GPIO ports on the STM32F401RC.
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

/**
 * @brief Enumerates the available pins within a GPIO port.
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
 * @brief Initializes a GPIO pin for output operation.
 * @param[in] port: The GPIO port to configure.
 * @param[in] pin: The pin number within the port.
 * @param[in] value: Initial output value (0 or 1).
 * @param[in] usage: The intended usage (normal or communication).
 * @param[in] conn: The output connection type (push-pull or open-drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
 * @brief Initializes a GPIO pin for input operation.
 * @param[in] port: The GPIO port to configure.
 * @param[in] pin: The pin number within the port.
 * @param[in] usage: The intended usage (normal or communication).
 * @param[in] pull: The pull resistor configuration (pull-up or pull-down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/**
 * @brief Gets the current direction mode of a GPIO pin.
 * @param[in] port: The GPIO port.
 * @param[in] pin: The pin number within the port.
 * @return The current direction mode (output, input, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
 * @brief Sets the output value of a GPIO pin.
 *        This function should only be called for pins configured as output.
 * @param[in] port: The GPIO port.
 * @param[in] pin: The pin number within the port.
 * @param[in] value: The value to set (0 for low, non-zero for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
 * @brief Gets the input value of a GPIO pin.
 *        This function should only be called for pins configured as input.
 * @param[in] port: The GPIO port.
 * @param[in] pin: The pin number within the port.
 * @return The current input value (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
 * @brief Toggles the output value of a GPIO pin.
 *        This function should only be called for pins configured as output.
 * @param[in] port: The GPIO port.
 * @param[in] pin: The pin number within the port.
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F401RC_GPIO_H_ */