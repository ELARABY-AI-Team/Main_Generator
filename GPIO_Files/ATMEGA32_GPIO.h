#ifndef ATMEGA32_GPIO_H_
#define ATMEGA32_GPIO_H_

/*
 * @brief This file contains the public interface for the GPIO driver.
 *        It provides declarations for configuring and controlling
 *        General Purpose Input/Output pins on the ATMEGA32 microcontroller.
 */

/* Include necessary configurations for the ATMEGA32 */
#include "ATMEGA32_Config.h"

/* Define standard integer types from ATMEGA32_Config.h */
/* Assuming tbyte and tword are defined as unsigned char and unsigned short respectively */
#ifndef tbyte
#error "tbyte type not defined in ATMEGA32_Config.h"
#endif
#ifndef tword
#error "tword type not defined in ATMEGA32_Config.h"
#endif

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
 * @brief Enum to define the intended usage of a GPIO pin.
 */
typedef enum {
    normal_usage,       /* Used for basic digital input/output */
    communication_usage /* Used for communication protocols like SPI, UART, TWI, etc. */
} t_usage;

/*
 * @brief Enum to define the direction of a GPIO pin.
 */
typedef enum {
    output, /* Pin configured as output */
    input,  /* Pin configured as input */
    analog  /* Pin configured as analog input (disables digital input buffer) */
} t_direction;

/*
 * @brief Enum to define the type of pull resistor for an input pin.
 *        Pull-down resistors are not typically configurable via PORT registers
 *        on ATmega microcontrollers, but included for standard interface.
 *        Pull-up enables the internal pull-up resistor. Pull-down implies no
 *        internal pull-up/down (external pull-down or floating unless driven).
 */
typedef enum {
    pull_up,   /* Internal pull-up resistor enabled */
    pull_down  /* No internal pull (external pull-down or floating/driven) */
} t_pull;

/*
 * @brief Enum to define the output connection type for an output pin.
 *        Open-drain is not directly configurable per pin via PORT registers
 *        on ATmega microcontrollers but can be simulated by toggling direction.
 *        Included for standard interface.
 */
typedef enum {
    push_pull,  /* Standard CMOS output driver (high sink and source capability) */
    open_drain  /* Simulated open-drain (achieved by configuring as input with pull-up) */
} t_output_conn;

/* ==================== HARDWARE DEFINITIONS ==================== */

/*
 * @brief Enum to define the available GPIO ports on the ATMEGA32.
 */
typedef enum {
    PORT_A, /* PDF Reference */
    PORT_B, /* PDF Reference */
    PORT_C, /* PDF Reference */
    PORT_D  /* PDF Reference */
} tport;

/*
 * @brief Enum to define the available pin numbers within a GPIO port (0-7).
 */
typedef enum {
    PIN_0, /* PDF Reference */
    PIN_1, /* PDF Reference */
    PIN_2, /* PDF Reference */
    PIN_3, /* PDF Reference */
    PIN_4, /* PDF Reference */
    PIN_5, /* PDF Reference */
    PIN_6, /* PDF Reference */
    PIN_7  /* PDF Reference */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a GPIO pin as an output.
 *
 * @param port The GPIO port (tport enum).
 * @param pin The pin number within the port (tpin enum).
 * @param value The initial output value (0 or 1).
 * @param usage The intended usage of the pin (normal_usage or communication_usage).
 * @param conn The output connection type (push_pull or open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a GPIO pin as an input.
 *
 * @param port The GPIO port (tport enum).
 * @param pin The pin number within the port (tpin enum).
 * @param usage The intended usage of the pin (normal_usage, communication_usage, or analog).
 * @param pull The type of pull resistor (pull_up or pull_down).
 *             Note: pull_down is not directly supported by internal resistors on ATmega32,
 *             setting to pull_down disables the pull-up.
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief Gets the current direction configuration of a GPIO pin.
 *
 * @param port The GPIO port (tport enum).
 * @param pin The pin number within the port (tpin enum).
 * @return The direction of the pin (output, input). Analog direction is not
 *         directly readable from DDR/PORT registers, so this function returns
 *         input for pins configured as analog input.
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a GPIO pin.
 *        This function should only be called for pins configured as output.
 *
 * @param port The GPIO port (tport enum).
 * @param pin The pin number within the port (tpin enum).
 * @param value The output value to set (0 or 1).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the input value of a GPIO pin.
 *        This function can be called regardless of pin configuration (input/output).
 *        If the pin is output, it reads the value written to the output latch.
 *        If the pin is input, it reads the physical pin state (after synchronizer).
 *
 * @param port The GPIO port (tport enum).
 * @param pin The pin number within the port (tpin enum).
 * @return The pin value (0 or 1).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a GPIO pin.
 *        This function should only be called for pins configured as output.
 *
 * @param port The GPIO port (tport enum).
 * @param pin The pin number within the port (tpin enum).
 */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* ATMEGA32_GPIO_H_ */