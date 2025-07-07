/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO_H_
* Description    : Header file for GPIO peripheral driver on STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/
#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_Config.h" // Assumes tbyte and potentially other types like tword are defined here

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
 * @brief Defines the intended usage context of a GPIO pin.
 */
typedef enum
{
    normal_usage,      // Standard digital input/output
    communication_usage // Used in communication protocols (e.g., SPI, I2C, UART) - implies Alternate Function configuration
} t_usage;

/**
 * @brief Defines the direction mode of the GPIO pin.
 */
typedef enum
{
    output, // Pin configured as digital output
    input,  // Pin configured as digital input
    analog  // Pin configured for analog functions (ADC/DAC)
} t_direction;

/**
 * @brief Defines the pull resistor settings for input pins.
 */
typedef enum
{
    pull_up,   // Enable internal pull-up resistor
    pull_down  // Enable internal pull-down resistor
} t_pull;

/**
 * @brief Defines the output driving connection type for output pins.
 */
typedef enum
{
    push_pull, // Output drives high and low
    open_drain // Output pulls low, requires external pull-up for high state
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/**
 * @brief Defines the available GPIO ports on the STM32F401RC microcontroller.
 */
typedef enum
{
    port_A, /* PDF Reference */ // GPIO Port A
    port_B, /* PDF Reference */ // GPIO Port B
    port_C, /* PDF Reference */ // GPIO Port C
    port_D, /* PDF Reference */ // GPIO Port D
    port_H  /* PDF Reference */ // GPIO Port H (typically PH0/PH1)
    // Ports E, F, G are not typically available on the STM32F401RC 64-pin package
} tport;

/**
 * @brief Defines the available pins within a GPIO port (0 to 15).
 */
typedef enum
{
    pin_0,  /* PDF Reference */ // Pin 0
    pin_1,  /* PDF Reference */ // Pin 1
    pin_2,  /* PDF Reference */ // Pin 2
    pin_3,  /* PDF Reference */ // Pin 3
    pin_4,  /* PDF Reference */ // Pin 4
    pin_5,  /* PDF Reference */ // Pin 5
    pin_6,  /* PDF Reference */ // Pin 6
    pin_7,  /* PDF Reference */ // Pin 7
    pin_8,  /* PDF Reference */ // Pin 8
    pin_9,  /* PDF Reference */ // Pin 9
    pin_10, /* PDF Reference */ // Pin 10
    pin_11, /* PDF Reference */ // Pin 11
    pin_12, /* PDF Reference */ // Pin 12
    pin_13, /* PDF Reference */ // Pin 13
    pin_14, /* PDF Reference */ // Pin 14
    pin_15  /* PDF Reference */ // Pin 15
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a GPIO pin as an output.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number within the port (e.g., pin_5).
 * @param value: Initial output value (0 for low, non-zero for high).
 * @param usage: The intended usage (normal_usage or communication_usage).
 * @param conn: The output connection type (push_pull or open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
 * @brief Initializes a GPIO pin as an input.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number within the port (e.g., pin_5).
 * @param usage: The intended usage (normal_usage or communication_usage).
 * @param pull: The pull resistor setting (pull_up or pull_down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/**
 * @brief Gets the current direction mode of a GPIO pin.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number within the port (e.g., pin_5).
 * @return The current direction (output, input, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
 * @brief Sets the output value of a GPIO pin.
 *        Only effective if the pin is configured as output.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number within the port (e.g., pin_5).
 * @param value: The value to set (0 for low, non-zero for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
 * @brief Gets the input value of a GPIO pin.
 *        If configured as output, returns the output state.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number within the port (e.g., pin_5).
 * @return The pin value (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
 * @brief Toggles the output value of a GPIO pin.
 *        Only effective if the pin is configured as output.
 * @param port: The GPIO port (e.g., port_A).
 * @param pin: The pin number within the port (e.g., pin_5).
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F401RC_GPIO_H_ */