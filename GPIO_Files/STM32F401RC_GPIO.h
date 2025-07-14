/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for the GPIO peripheral driver for STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-14
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/
#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/* Include necessary configuration header */
/* Assumes STM32F401RC_Config.h defines types like tbyte, tword, etc. */
#include "STM32F401RC_Config.h"

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
 * @brief Defines the intended usage context of a GPIO pin.
 */
typedef enum
{
    normal_usage,      /* Pin is used for general purpose I/O logic */
    communication_usage /* Pin is used for communication protocol (e.g., SPI, I2C) - Alternate Function */
} t_usage;

/**
 * @brief Defines the direction mode of a GPIO pin.
 */
typedef enum
{
    output, /* Pin is configured as an output */
    input,  /* Pin is configured as an input */
    analog  /* Pin is configured for analog mode */
} t_direction;

/**
 * @brief Defines the pull resistor settings for input pins.
 * @note Not applicable for output or analog modes.
 */
typedef enum
{
    pull_up,   /* Enable internal pull-up resistor */
    pull_down  /* Enable internal pull-down resistor */
} t_pull;

/**
 * @brief Defines the output driving connection type for output pins.
 * @note Not applicable for input or analog modes.
 */
typedef enum
{
    push_pull,  /* Output configured as push-pull */
    open_drain  /* Output configured as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/**
 * @brief Defines the available GPIO ports on the STM32F401RC.
 */
typedef enum
{
    port_A, /* PDF Reference */
    port_B, /* PDF Reference */
    port_C, /* PDF Reference */
    port_D, /* PDF Reference */
    port_E, /* PDF Reference */ /* Note: Port E pins typically not bonded out on 64-pin packages like RC */
    port_H  /* PDF Reference */ /* Note: Port H pins typically limited (PH0/PH1) and used for OSC on 64-pin packages like RC */
} tport;

/**
 * @brief Defines the available pin numbers within a GPIO port (0 to 15).
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
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @param value The initial output value (0 for low, non-zero for high).
 * @param usage The usage context (normal or communication).
 * @param conn The output connection type (push-pull or open-drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
 * @brief Initializes a GPIO pin as an input.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @param usage The usage context (normal or communication).
 * @param pull The pull resistor configuration (pull-up or pull-down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/**
 * @brief Gets the current direction configuration of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @return The current direction mode (output, input, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
 * @brief Sets the output value of a GPIO pin configured as output.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @param value The value to set (0 for low, non-zero for high).
 * @note This function should only be called for pins configured as output.
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
 * @brief Gets the input value of a GPIO pin configured as input.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @return The pin value (0 if low, 1 if high).
 * @note This function is typically used for pins configured as input.
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
 * @brief Toggles the output value of a GPIO pin configured as output.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @note This function should only be called for pins configured as output.
 */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */