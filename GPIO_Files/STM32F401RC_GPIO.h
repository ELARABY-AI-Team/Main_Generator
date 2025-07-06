/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : GPIO header file for STM32F401RC microcontroller
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/* Include necessary configuration header */
#include "STM32F401RC_Config.h"

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
* @brief Defines whether the GPIO is used for general or communication-specific logic.
*/
typedef enum
{
    normal_usage,       /* General purpose GPIO usage */
    communication_usage /* GPIO used for specific communication protocols (e.g., SPI, I2C, UART, etc.) */
} t_usage;

/*
* @brief Defines the direction mode of the GPIO pin.
*/
typedef enum
{
    output, /* Pin configured as output */
    input,  /* Pin configured as input */
    analog  /* Pin configured as analog */
} t_direction;

/*
* @brief Defines pull resistor settings for input pins.
*/
typedef enum
{
    pull_up,   /* Pull-up resistor enabled */
    pull_down  /* Pull-down resistor enabled */
} t_pull;

/*
* @brief Defines the output driving connection type.
*/
typedef enum
{
    push_pull, /* Push-pull output configuration */
    open_drain /* Open-drain output configuration */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/*
* @brief Defines the available GPIO ports for the STM32F401RC microcontroller.
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
* @brief Defines the available pins for a GPIO port (0-15).
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
* @brief Initializes a GPIO pin as an output.
* @param[in] port : The GPIO port (tport enum).
* @param[in] pin : The pin number (tpin enum).
* @param[in] value : Initial output value (0 or 1).
* @param[in] usage : Pin usage type (t_usage enum).
* @param[in] conn : Output connection type (t_output_conn enum).
*/
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
* @brief Initializes a GPIO pin as an input.
* @param[in] port : The GPIO port (tport enum).
* @param[in] pin : The pin number (tpin enum).
* @param[in] usage : Pin usage type (t_usage enum).
* @param[in] pull : Pull resistor configuration (t_pull enum).
*/
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
* @brief Gets the direction configuration of a GPIO pin.
* @param[in] port : The GPIO port (tport enum).
* @param[in] pin : The pin number (tpin enum).
* @return t_direction : The current direction mode (t_direction enum).
*/
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
* @brief Sets the output value of a GPIO pin.
* @param[in] port : The GPIO port (tport enum).
* @param[in] pin : The pin number (tpin enum).
* @param[in] value : The value to set (0 or 1).
*/
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
* @brief Gets the input value of a GPIO pin.
* @param[in] port : The GPIO port (tport enum).
* @param[in] pin : The pin number (tpin enum).
* @return tbyte : The current pin value (0 or 1).
*/
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
* @brief Toggles the output value of a GPIO pin.
* @param[in] port : The GPIO port (tport enum).
* @param[in] pin : The pin number (tpin enum).
*/
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F401RC_GPIO_H_ */