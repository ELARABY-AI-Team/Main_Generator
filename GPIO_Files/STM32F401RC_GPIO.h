/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for the GPIO peripheral driver on STM32F401RC microcontrollers.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/* ==================== REQUIRED INCLUDES ==================== */
/* Assumed to provide types like tbyte, tword and potential configuration defines */
#include "STM32F401RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
 * @brief Enumeration for GPIO usage type.
 * Defines whether the GPIO is used for general or communication-specific logic.
 */
typedef enum
{
    normal_usage,        /* GPIO pin used for general purpose I/O */
    communication_usage  /* GPIO pin used for communication protocols (e.g., SPI, I2C, UART) */
} t_usage;

/*
 * @brief Enumeration for GPIO pin direction mode.
 * Defines the operational mode of the GPIO pin.
 */
typedef enum
{
    output,              /* GPIO pin configured as an output */
    input,               /* GPIO pin configured as an input */
    analog               /* GPIO pin configured for analog input */
} t_direction;

/*
 * @brief Enumeration for GPIO pull resistor settings.
 * Defines pull resistor configuration for input pins.
 */
typedef enum
{
    pull_up,             /* Configure the internal pull-up resistor */
    pull_down            /* Configure the internal pull-down resistor */
    /* Note: No Pull-up/Pull-down setting is not included as per requirements */
} t_pull;

/*
 * @brief Enumeration for GPIO output connection type.
 * Defines the output driver configuration for output pins.
 */
typedef enum
{
    push_pull,           /* Configure the output driver as push-pull */
    open_drain           /* Configure the output driver as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/*
 * @brief Enumeration for available GPIO ports.
 * Lists the physical GPIO ports present on the STM32F401RC microcontroller.
 */
typedef enum
{
    port_A,              /* GPIO Port A base address: 0x40020000 /* PDF Reference */ */
    port_B,              /* GPIO Port B base address: 0x40020400 /* PDF Reference */ */
    port_C,              /* GPIO Port C base address: 0x40020800 /* PDF Reference */ */
    port_D,              /* GPIO Port D base address: 0x40020C00 /* PDF Reference */ */
    port_E,              /* GPIO Port E base address: 0x40021000 /* PDF Reference */ */
    port_H               /* GPIO Port H base address: 0x40021C00 /* PDF Reference */ */
} tport;

/*
 * @brief Enumeration for GPIO pin numbers.
 * Lists the individual pin numbers available on each GPIO port.
 */
typedef enum
{
    pin_0,               /* Pin number 0 /* PDF Reference */ */
    pin_1,               /* Pin number 1 /* PDF Reference */ */
    pin_2,               /* Pin number 2 /* PDF Reference */ */
    pin_3,               /* Pin number 3 /* PDF Reference */ */
    pin_4,               /* Pin number 4 /* PDF Reference */ */
    pin_5,               /* Pin number 5 /* PDF Reference */ */
    pin_6,               /* Pin number 6 /* PDF Reference */ */
    pin_7,               /* Pin number 7 /* PDF Reference */ */
    pin_8,               /* Pin number 8 /* PDF Reference */ */
    pin_9,               /* Pin number 9 /* PDF Reference */ */
    pin_10,              /* Pin number 10 /* PDF Reference */ */
    pin_11,              /* Pin number 11 /* PDF Reference */ */
    pin_12,              /* Pin number 12 /* PDF Reference */ */
    pin_13,              /* Pin number 13 /* PDF Reference */ */
    pin_14,              /* Pin number 14 /* PDF Reference */ */
    pin_15               /* Pin number 15 /* PDF Reference */ */
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a GPIO pin for output operation.
 * Configures the specified port and pin as an output with defined initial value, usage type, and output connection.
 *
 * @param port The GPIO port (e.g., port_A).
 * @param pin The pin number within the port (e.g., pin_0).
 * @param value The initial output value (0 for low, 1 for high).
 * @param usage The intended usage of the pin (normal_usage or communication_usage).
 * @param conn The output driver connection type (push_pull or open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a GPIO pin for input operation.
 * Configures the specified port and pin as an input with defined usage type and pull resistor setting.
 *
 * @param port The GPIO port (e.g., port_A).
 * @param pin The pin number within the port (e.g., pin_0).
 * @param usage The intended usage of the pin (normal_usage or communication_usage).
 * @param pull The pull resistor configuration (pull_up or pull_down).
 *             Note: Only pull-up and pull-down are supported as per t_pull enum definition.
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief Gets the current direction mode of a GPIO pin.
 * Reads the configuration register to determine if the pin is set as input, output, or analog.
 *
 * @param port The GPIO port (e.g., port_A).
 * @param pin The pin number within the port (e.g., pin_0).
 * @return The current direction mode of the pin (output, input, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a GPIO pin.
 * Writes the specified value (high or low) to an output-configured pin.
 * Has no effect if the pin is not configured as output.
 *
 * @param port The GPIO port (e.g., port_A).
 * @param pin The pin number within the port (e.g., pin_0).
 * @param value The value to set (0 for low, non-zero for high). Typically 0 or 1.
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the input value of a GPIO pin.
 * Reads the current logic level on an input-configured pin.
 * For output pins, typically reads the output data register value.
 *
 * @param port The GPIO port (e.g., port_A).
 * @param pin The pin number within the port (e.g., pin_0).
 * @return The current logic level of the pin (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a GPIO pin.
 * Flips the current state (high to low, or low to high) of an output-configured pin.
 * Has no effect if the pin is not configured as output.
 *
 * @param port The GPIO port (e.g., port_A).
 * @param pin The pin number within the port (e.g., pin_0).
 */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */