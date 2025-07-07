/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for the GPIO driver for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/*
 * Include section
 * All types (tbyte, tword) are assumed to be defined in STM32F401RC_MAIN.h
 */
#include "STM32F401RC_MAIN.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
 * Enum to define GPIO usage type.
 * normal_usage: Standard digital input/output.
 * communication_usage: Pin used for specific communication protocols (e.g., SPI, I2C, UART).
 */
typedef enum
{
    normal_usage,       /* Standard GPIO functionality */
    communication_usage /* Pin used for communication interfaces (Alternative Function) */
} t_usage;

/*
 * Enum to define GPIO direction mode.
 * output: Pin configured as digital output.
 * input: Pin configured as digital input.
 * analog: Pin configured in analog mode.
 */
typedef enum
{
    output, /* Configured as output pin */
    input,  /* Configured as input pin */
    analog  /* Configured in analog mode */
} t_direction;

/*
 * Enum to define internal pull resistor configuration for input pins.
 * pull_up: Internal pull-up resistor enabled.
 * pull_down: Internal pull-down resistor enabled.
 */
typedef enum
{
    pull_up,   /* Internal pull-up resistor enabled */
    pull_down  /* Internal pull-down resistor enabled */
} t_pull;

/*
 * Enum to define output driving connection type for output pins.
 * push_pull: Standard push-pull output mode.
 * open_drain: Open-drain output mode.
 */
typedef enum
{
    push_pull,  /* Push-pull output connection */
    open_drain  /* Open-drain output connection */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/*
 * Enum to define available GPIO ports on STM32F401RC (LQFP64 package).
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
 * Enum to define available pin numbers within each GPIO port.
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
 * @param pin: The pin number (e.g., pin_0).
 * @param value: Initial state of the output pin (0 for low, non-zero for high).
 * @param usage: Usage type (normal_usage or communication_usage).
 * @param conn: Output connection type (push_pull or open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a GPIO pin as input.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number (e.g., pin_0).
 * @param usage: Usage type (normal_usage or communication_usage).
 * @param pull: Pull resistor configuration (pull_up or pull_down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief Gets the current direction configuration of a GPIO pin.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number (e.g., pin_0).
 * @return The direction configuration (output, input, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a GPIO pin.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number (e.g., pin_0).
 * @param value: The value to set (0 for low, non-zero for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the current input value of a GPIO pin.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number (e.g., pin_0).
 * @return The input value (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a GPIO pin.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number (e.g., pin_0).
 */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */