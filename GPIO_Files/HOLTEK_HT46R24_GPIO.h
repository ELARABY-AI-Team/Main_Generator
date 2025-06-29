#ifndef HOLTEK_HT46R24_GPIO_H_
#define HOLTEK_HT46R24_GPIO_H_

/* Include necessary files */
#include "HOLTEK_HT46R24_Config.h" /* For configuration specific settings if any */
#include "HOLTEK_HT46R24_MAIN.h"   /* For standard types like tbyte */

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/**
 * @brief Enum to specify the intended usage of a GPIO pin.
 */
typedef enum {
    normal_usage,       /* Standard digital or analog I/O */
    communication_usage /* Usage for specific communication interfaces (e.g., SPI, I2C, UART) */
} t_usage;

/**
 * @brief Enum to specify the data direction of a GPIO pin.
 */
typedef enum {
    output, /* Pin configured as an output */
    input,  /* Pin configured as an input */
    analog  /* Pin configured for analog input (ADC) */
} t_direction;

/**
 * @brief Enum to specify the pull-up or pull-down configuration for an input pin.
 */
typedef enum {
    pull_up,   /* Internal pull-up resistor enabled */
    pull_down  /* Internal pull-down resistor enabled (if supported) */
} t_pull;

/**
 * @brief Enum to specify the output connection type for an output pin.
 */
typedef enum {
    push_pull,  /* Standard push-pull output configuration */
    open_drain  /* Open-drain output configuration */
} t_output_conn;

/* ==================== HARDWARE DEFINITIONS ==================== */

/**
 * @brief Enum for identifying the available GPIO ports on the HOLTEK HT46R24.
 */
typedef enum {
    PORT_A, /* General Purpose I/O Port A */ /* PDF Reference */
    PORT_B, /* General Purpose I/O Port B */ /* PDF Reference */
    PORT_C, /* General Purpose I/O Port C */ /* PDF Reference */
    PORT_D  /* General Purpose I/O Port D */ /* PDF Reference */
} tport;

/**
 * @brief Enum for identifying the pins within a GPIO port (0-7).
 */
typedef enum {
    PIN_0, /* Pin 0 within a port */ /* PDF Reference */
    PIN_1, /* Pin 1 within a port */ /* PDF Reference */
    PIN_2, /* Pin 2 within a port */ /* PDF Reference */
    PIN_3, /* Pin 3 within a port */ /* PDF Reference */
    PIN_4, /* Pin 4 within a port */ /* PDF Reference */
    PIN_5, /* Pin 5 within a port */ /* PDF Reference */
    PIN_6, /* Pin 6 within a port */ /* PDF Reference */
    PIN_7  /* Pin 7 within a port */ /* PDF Reference */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes a GPIO pin as an output.
 *
 * Configures the direction register and output connection type for the specified pin.
 * Sets the initial output value.
 *
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number within the port (e.g., PIN_0).
 * @param value The initial value to set the output pin to (0 or 1).
 * @param usage The intended usage of the pin (e.g., normal_usage).
 * @param conn The output connection type (e.g., push_pull, open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
 * @brief Initializes a GPIO pin as an input.
 *
 * Configures the direction register and pull-up/pull-down resistor for the specified pin.
 * Note: The HT46R24 primarily features configurable pull-up resistors. Pull-down support may vary.
 *
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number within the port (e.g., PIN_0).
 * @param usage The intended usage of the pin (e.g., normal_usage, communication_usage).
 * @param pull The pull configuration for the input pin (e.g., pull_up, pull_down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/**
 * @brief Gets the current direction configuration of a GPIO pin.
 *
 * Reads the direction register for the specified pin.
 *
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number within the port (e.g., PIN_0).
 * @return The current direction of the pin (input, output, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
 * @brief Sets the output value of a GPIO pin configured as output.
 *
 * Writes a value (0 or 1) to the output data register for the specified pin.
 *
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number within the port (e.g., PIN_0).
 * @param value The value to set the pin to (0 for low, 1 for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
 * @brief Gets the current value of a GPIO pin.
 *
 * Reads the input data register for a pin configured as input,
 * or the output data register for a pin configured as output.
 *
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number within the port (e.g., PIN_0).
 * @return The current value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
 * @brief Toggles the output value of a GPIO pin configured as output.
 *
 * Reads the current value and writes the inverted value back to the output data register.
 *
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number within the port (e.g., PIN_0).
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* HOLTEK_HT46R24_GPIO_H_ */