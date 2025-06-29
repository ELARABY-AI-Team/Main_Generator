#ifndef _GPIO_H_
#define _GPIO_H_

/**
 * @file GPIO.h
 * @brief Production-ready header file for ATMEGA32 GPIO control.
 *
 * This header provides macros, typedefs, and function declarations
 * for initializing and controlling the GPIO pins on the ATMEGA32
 * microcontroller (Port A, Port B, Port C, Port D).
 *
 * Assumes necessary basic types like uint8_t are defined elsewhere
 * (e.g., included via main.h or stdint.h).
 * Includes avr/io.h for register definitions.
 */

// Include AVR-specific definitions for registers like DDRA, PORTA, PINA etc.
// /* Assumed dependency: avr/io.h provides necessary register definitions. */
#include <avr/io.h>

// Support C++ compilers
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enumeration for the available GPIO ports.
 */
typedef enum {
    PORT_A = 0, ///< Port A
    PORT_B,     ///< Port B
    PORT_C,     ///< Port C
    PORT_D      ///< Port D
} GPIOPort_t;

/**
 * @brief Enumeration for the individual pins within a port.
 */
typedef enum {
    PIN_0 = 0, ///< Pin 0
    PIN_1,     ///< Pin 1
    PIN_2,     ///< Pin 2
    PIN_3,     ///< Pin 3
    PIN_4,     ///< Pin 4
    PIN_5,     ///< Pin 5
    PIN_6,     ///< Pin 6
    PIN_7      ///< Pin 7
} GPIOPin_t;

/**
 * @brief Enumeration for GPIO pin direction.
 */
typedef enum {
    GPIO_DIRECTION_INPUT = 0,  ///< Pin configured as input
    GPIO_DIRECTION_OUTPUT = 1  ///< Pin configured as output
} GPIODirection_t;

/**
 * @brief Enumeration for GPIO pin logic level.
 */
typedef enum {
    GPIO_LEVEL_LOW = 0,  ///< Logic low level (0V)
    GPIO_LEVEL_HIGH = 1  ///< Logic high level (+Vcc or enabled pull-up for input)
} GPIOLevel_t;

/**
 * @brief Macro to enable internal pull-up resistor for an input pin.
 * Use with GPIO_initPin when direction is GPIO_DIRECTION_INPUT.
 */
#define GPIO_PULLUP_ENABLE  GPIO_LEVEL_HIGH

/**
 * @brief Macro to disable internal pull-up resistor for an input pin.
 * Use with GPIO_initPin when direction is GPIO_DIRECTION_INPUT.
 */
#define GPIO_PULLUP_DISABLE GPIO_LEVEL_LOW

/**
 * @brief Initializes a specific GPIO pin with the given direction and initial state/pull-up setting.
 *
 * @param port The GPIO port (e.g., PORT_A, PORT_B).
 * @param pin The pin number within the port (e.g., PIN_0, PIN_7).
 * @param direction The direction for the pin (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 * @param initialStateOrPullupState If direction is GPIO_DIRECTION_OUTPUT, this is the initial output level (GPIO_LEVEL_LOW or GPIO_LEVEL_HIGH).
 *                                   If direction is GPIO_DIRECTION_INPUT, this controls the internal pull-up resistor (GPIO_PULLUP_ENABLE or GPIO_PULLUP_DISABLE).
 */
void GPIO_initPin(GPIOPort_t port, GPIOPin_t pin, GPIODirection_t direction, GPIOLevel_t initialStateOrPullupState);

/**
 * @brief Sets the output level of a GPIO pin configured as output.
 * This function has no effect if the pin is configured as input.
 *
 * @param port The GPIO port (e.g., PORT_A, PORT_B).
 * @param pin The pin number within the port (e.g., PIN_0, PIN_7).
 * @param level The desired output level (GPIO_LEVEL_LOW or GPIO_LEVEL_HIGH).
 */
void GPIO_writePin(GPIOPort_t port, GPIOPin_t pin, GPIOLevel_t level);

/**
 * @brief Reads the logic level of a GPIO pin.
 *
 * @param port The GPIO port (e.g., PORT_A, PORT_B).
 * @param pin The pin number within the port (e.g., PIN_0, PIN_7).
 * @return The current logic level of the pin (GPIO_LEVEL_LOW or GPIO_LEVEL_HIGH).
 */
GPIOLevel_t GPIO_readPin(GPIOPort_t port, GPIOPin_t pin);

/**
 * @brief Toggles the output level of a GPIO pin configured as output.
 * This function has no effect if the pin is configured as input.
 *
 * @param port The GPIO port (e.g., PORT_A, PORT_B).
 * @param pin The pin number within the port (e.g., PIN_0, PIN_7).
 */
void GPIO_togglePin(GPIOPort_t port, GPIOPin_t pin);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _GPIO_H_ */