/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : Header file for GPIO peripheral driver for ATMEGA32.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef ATMEGA32_GPIO_H_
#define ATMEGA32_GPIO_H_

/* Include necessary configuration header which defines required types */
#include "ATMEGA32_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
 * @enum t_usage
 * @brief Defines the intended usage category of a GPIO pin.
 */
typedef enum
{
    normal_usage = 0, /* Define GPIO for standard general purpose use */
    communication_usage = 1 /* Define GPIO for specific communication interfaces (e.g., SPI, I2C, UART) */
} t_usage;

/**
 * @enum t_direction
 * @brief Defines the data direction mode of a GPIO pin.
 */
typedef enum
{
    output = 0, /* Configure pin as output */
    input = 1, /* Configure pin as input */
    analog = 2 /* Configure pin for analog functions (e.g., ADC input) */
} t_direction;

/**
 * @enum t_pull
 * @brief Defines the pull resistor configuration for an input pin.
 */
typedef enum
{
    pull_up = 0, /* Enable internal pull-up resistor */
    pull_down = 1 /* Enable internal pull-down resistor (Note: ATMEGA32 supports pull-up, external pull-down needed) */
} t_pull; /* Note: ATMEGA32 only has internal pull-ups. pull_down enum value kept for potential future compatibility or abstraction. */

/**
 * @enum t_output_conn
 * @brief Defines the output driver connection type for an output pin.
 */
typedef enum
{
    push_pull = 0, /* Standard output driver */
    open_drain = 1 /* Open-drain output driver */
} t_output_conn; /* Note: ATMEGA32 typically uses Push-Pull logic. Open-drain functionality might require external components or specific peripheral configurations. */


/* ==================== HARDWARE DEFINITIONS ==================== */

/**
 * @enum tport
 * @brief Defines the available GPIO ports on the ATMEGA32 microcontroller.
 */
typedef enum
{
    port_A = 0, /* Port A - PDF Reference */
    port_B = 1, /* Port B - PDF Reference */
    port_C = 2, /* Port C - PDF Reference */
    port_D = 3 /* Port D - PDF Reference */
} tport;

/**
 * @enum tpin
 * @brief Defines the available pin numbers within a GPIO port.
 */
typedef enum
{
    pin_0 = 0, /* Pin 0 - PDF Reference */
    pin_1 = 1, /* Pin 1 - PDF Reference */
    pin_2 = 2, /* Pin 2 - PDF Reference */
    pin_3 = 3, /* Pin 3 - PDF Reference */
    pin_4 = 4, /* Pin 4 - PDF Reference */
    pin_5 = 5, /* Pin 5 - PDF Reference */
    pin_6 = 6, /* Pin 6 - PDF Reference */
    pin_7 = 7 /* Pin 7 - PDF Reference */
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a specified GPIO pin as an output.
 * @param port The GPIO port (tport).
 * @param pin The pin number within the port (tpin).
 * @param value The initial output value (tbyte - 0 for low, non-zero for high).
 * @param usage The intended usage of the pin (t_usage).
 * @param conn The output connection type (t_output_conn).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
 * @brief Initializes a specified GPIO pin as an input.
 * @param port The GPIO port (tport).
 * @param pin The pin number within the port (tpin).
 * @param usage The intended usage of the pin (t_usage).
 * @param pull The pull resistor configuration (t_pull).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/**
 * @brief Gets the current direction configuration of a specified GPIO pin.
 * @param port The GPIO port (tport).
 * @param pin The pin number within the port (tpin).
 * @return The direction mode of the pin (t_direction).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
 * @brief Sets the output value of a specified GPIO pin configured as output.
 * @param port The GPIO port (tport).
 * @param pin The pin number within the port (tpin).
 * @param value The output value to set (tbyte - 0 for low, non-zero for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
 * @brief Gets the input value of a specified GPIO pin configured as input.
 * @param port The GPIO port (tport).
 * @param pin The pin number within the port (tpin).
 * @return The input value of the pin (tbyte - 0 for low, non-zero for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
 * @brief Toggles the output value of a specified GPIO pin configured as output.
 * @param port The GPIO port (tport).
 * @param pin The pin number within the port (tpin).
 */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* ATMEGA32_GPIO_H_ */