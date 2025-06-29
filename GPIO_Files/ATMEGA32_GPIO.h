#ifndef ATMEGA32_GPIO_H_
#define ATMEGA32_GPIO_H_

/* Include necessary configuration file for custom types */
/* This file should define types like tbyte, tword, etc. */
#include "ATMEGA32_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */
/* Enum for specifying pin usage context */
typedef enum {
    normal_usage,       /* Standard digital I/O */
    communication_usage /* Pin used for specific peripheral communication (e.g., SPI, I2C, UART) */
} t_usage;

/* Enum for specifying pin direction */
typedef enum {
    output, /* Pin configured as an output */
    input,  /* Pin configured as an input */
    analog  /* Pin configured for analog input (requires specific peripheral configuration) */
} t_direction;

/* Enum for specifying pull-up/down configuration for input pins */
typedef enum {
    pull_up,   /* Internal pull-up resistor enabled */
    pull_down  /* Pull-down configuration (Note: ATMEGA32 has internal pull-ups, pull-down typically requires external resistor or specific peripheral setting not standard GPIO) */
} t_pull;

/* Enum for specifying output connection type */
typedef enum {
    push_pull,  /* Standard active output drive */
    open_drain  /* Open-drain output (Note: ATMEGA32 GPIO pins are typically push-pull, open-drain behavior might require external component or specific configuration/peripheral use like I2C) */
} t_output_conn;

/* ==================== HARDWARE DEFINITIONS ==================== */
/* Enum for ATMEGA32 GPIO Ports */
typedef enum {
    PORT_A, /* Assumed - please verify */
    PORT_B, /* Assumed - please verify */
    PORT_C, /* Assumed - please verify */
    PORT_D  /* Assumed - please verify */
} tport;

/* Enum for ATMEGA32 GPIO Pins within a Port */
typedef enum {
    PIN_0, /* Assumed - please verify */
    PIN_1, /* Assumed - please verify */
    PIN_2, /* Assumed - please verify */
    PIN_3, /* Assumed - please verify */
    PIN_4, /* Assumed - please verify */
    PIN_5, /* Assumed - please verify */
    PIN_6, /* Assumed - please verify */
    PIN_7  /* Assumed - please verify */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */
/*
 * @brief Initializes a GPIO pin as an output.
 * @param[in] port: The GPIO port (e.g., PORT_A).
 * @param[in] pin: The pin number within the port (e.g., PIN_0).
 * @param[in] value: Initial state of the output pin (0 or non-zero).
 * @param[in] usage: The intended usage of the pin (normal_usage or communication_usage).
 * @param[in] conn: The output connection type (push_pull or open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a GPIO pin as an input.
 * @param[in] port: The GPIO port (e.g., PORT_A).
 * @param[in] pin: The pin number within the port (e.g., PIN_0).
 * @param[in] usage: The intended usage of the pin (normal_usage or communication_usage).
 * @param[in] pull: The pull resistor configuration (pull_up or pull_down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief Gets the current direction of a GPIO pin.
 * @param[in] port: The GPIO port (e.g., PORT_A).
 * @param[in] pin: The pin number within the port (e.g., PIN_0).
 * @return t_direction: The current direction of the pin (output, input, or analog - note: analog detection might be limited to direction register state).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a GPIO pin.
 *        This function should only be called for pins configured as output.
 * @param[in] port: The GPIO port (e.g., PORT_A).
 * @param[in] pin: The pin number within the port (e.g., PIN_0).
 * @param[in] value: The value to set (0 for low, non-zero for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the input value of a GPIO pin.
 *        This function should only be called for pins configured as input.
 * @param[in] port: The GPIO port (e.g., PORT_A).
 * @param[in] pin: The pin number within the port (e.g., PIN_0).
 * @return tbyte: The value read from the pin (0 for low, non-zero for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a GPIO pin.
 *        This function should only be called for pins configured as output.
 * @param[in] port: The GPIO port (e.g., PORT_A).
 * @param[in] pin: The pin number within the port (e.g., PIN_0).
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* ATMEGA32_GPIO_H_ */