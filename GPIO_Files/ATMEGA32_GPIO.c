/**
 * @file GPIO.c
 * @brief Source file for the GPIO driver for the ATMEGA32 microcontroller.
 *
 * This file implements the functions declared in GPIO.h for configuring
 * and controlling General Purpose Input/Output (GPIO) pins on the ATMEGA32.
 *
 * @note This driver assumes the ATMEGA32_GPIO.h header correctly defines
 *       the addresses and bit names/positions for the GPIO registers
 *       (DDRx, PORTx, PINx for x = A, B, C, D).
 * @note This driver assumes the ATMEGA32_MAIN.h header correctly defines
 *       standard types like Std_Return_t and standard return codes (OK, ERROR).
 *
 * @author Your Name
 * @date YYYY-MM-DD
 * @version 1.0
 */

//==============================================================================
// Includes
//==============================================================================

#include "ATMEGA32_GPIO.h" // Contains GPIO register definitions, enums, function prototypes
#include "ATMEGA32_MAIN.h" // Contains standard types like Std_Return_t, OK, ERROR
#include <stdint.h>        // For standard integer types like uint8_t
#include <stddef.h>        // For NULL

// Include the AVR-GCC I/O header for register definitions if ATMEGA32_GPIO.h
// doesn't directly provide them but relies on the standard mechanism.
// If ATMEGA32_GPIO.h provides them, this line might be redundant but harmless.
#include <avr/io.h>

//==============================================================================
// Private Definitions
//==============================================================================

// Helper macro to check if a pin number is valid (0-7)
#define IS_VALID_PIN(PIN) ((PIN) >= PIN0 && (PIN) <= PIN7)

// Helper macro to check if a port is valid (A, B, C, D)
#define IS_VALID_PORT(PORT) ((PORT) >= PORTA && (PORT) <= PORTD)

// Helper macro to check if a direction is valid (INPUT, OUTPUT)
#define IS_VALID_DIRECTION(DIR) ((DIR) == INPUT || (DIR) == OUTPUT)

// Helper macro to check if a state is valid (LOW, HIGH)
#define IS_VALID_STATE(STATE) ((STATE) == LOW || (STATE) == HIGH)


//==============================================================================
// Global Variables (if any) - None needed for this driver
//==============================================================================


//==============================================================================
// Function Implementations
//==============================================================================

/**
 * @brief Initializes a specific GPIO pin by setting its data direction.
 *
 * This function sets the direction of the specified pin to either input
 * or output. For input pins, the pull-up resistor is disabled by default.
 * To enable the pull-up, use GPIO_WritePin with the HIGH state after
 * setting the direction to INPUT.
 *
 * @param port      The GPIO port (e.g., PORTA, PORTB).
 * @param pin       The pin number within the port (e.g., PIN0, PIN1).
 * @param direction The desired direction (INPUT or OUTPUT).
 * @return Std_Return_t Status of the operation (OK or ERROR).
 *         Returns ERROR if the provided port, pin, or direction is invalid.
 */
Std_Return_t GPIO_InitPin(Port_t port, Pin_t pin, Direction_t direction)
{
    // Input validation
    if (!IS_VALID_PORT(port) || !IS_VALID_PIN(pin) || !IS_VALID_DIRECTION(direction))
    {
        // Log error or handle appropriately in a real system
        return ERROR;
    }

    // Use a switch statement to select the correct Data Direction Register (DDRx)
    // based on the provided port.
    switch (port)
    {
        case PORTA:
            if (direction == OUTPUT)
            {
                // Set the corresponding bit in DDRA to configure as output
                DDRA |= (1 << pin);
            }
            else // direction == INPUT
            {
                // Clear the corresponding bit in DDRA to configure as input
                DDRA &= ~(1 << pin);
                // Ensure pull-up is disabled by default for input
                PORTA &= ~(1 << pin);
            }
            break;

        case PORTB:
            if (direction == OUTPUT)
            {
                // Set the corresponding bit in DDRB
                DDRB |= (1 << pin);
            }
            else // direction == INPUT
            {
                // Clear the corresponding bit in DDRB
                DDRB &= ~(1 << pin);
                 // Ensure pull-up is disabled
                PORTB &= ~(1 << pin);
            }
            break;

        case PORTC:
            if (direction == OUTPUT)
            {
                // Set the corresponding bit in DDRC
                DDRC |= (1 << pin);
            }
            else // direction == INPUT
            {
                // Clear the corresponding bit in DDRC
                DDRC &= ~(1 << pin);
                 // Ensure pull-up is disabled
                PORTC &= ~(1 << pin);
            }
            break;

        case PORTD:
            if (direction == OUTPUT)
            {
                // Set the corresponding bit in DDRD
                DDRD |= (1 << pin);
            }
            else // direction == INPUT
            {
                // Clear the corresponding bit in DDRD
                DDRD &= ~(1 << pin);
                // Ensure pull-up is disabled
                PORTD &= ~(1 << pin);
            }
            break;

        // Default case is not strictly needed due to IS_VALID_PORT check,
        // but included for robustness.
        default:
            return ERROR;
    }

    return OK;
}

/**
 * @brief Writes a digital state (HIGH or LOW) to a specific GPIO pin.
 *
 * If the pin is configured as output, this function sets the pin's output
 * voltage to high or low.
 * If the pin is configured as input, writing HIGH enables the internal pull-up
 * resistor, and writing LOW disables it.
 *
 * @param port  The GPIO port.
 * @param pin   The pin number.
 * @param state The desired state (LOW or HIGH).
 * @return Std_Return_t Status of the operation (OK or ERROR).
 *         Returns ERROR if the provided port, pin, or state is invalid.
 */
Std_Return_t GPIO_WritePin(Port_t port, Pin_t pin, State_t state)
{
    // Input validation
    if (!IS_VALID_PORT(port) || !IS_VALID_PIN(pin) || !IS_VALID_STATE(state))
    {
        // Log error or handle appropriately
        return ERROR;
    }

    // Use a switch statement to select the correct Port Data Register (PORTx)
    // based on the provided port.
    switch (port)
    {
        case PORTA:
            if (state == HIGH)
            {
                // Set the corresponding bit in PORTA (sets output high or enables pull-up)
                PORTA |= (1 << pin);
            }
            else // state == LOW
            {
                // Clear the corresponding bit in PORTA (sets output low or disables pull-up)
                PORTA &= ~(1 << pin);
            }
            break;

        case PORTB:
            if (state == HIGH)
            {
                PORTB |= (1 << pin);
            }
            else // state == LOW
            {
                PORTB &= ~(1 << pin);
            }
            break;

        case PORTC:
            if (state == HIGH)
            {
                PORTC |= (1 << pin);
            }
            else // state == LOW
            {
                PORTC &= ~(1 << pin);
            }
            break;

        case PORTD:
            if (state == HIGH)
            {
                PORTD |= (1 << pin);
            }
            else // state == LOW
            {
                PORTD &= ~(1 << pin);
            }
            break;

        default:
            return ERROR;
    }

    return OK;
}

/**
 * @brief Toggles the digital state of a specific GPIO pin.
 *
 * This function inverts the state of the corresponding bit in the PORTx register.
 * It is typically used for output pins to switch between HIGH and LOW states.
 * Toggling an input pin's PORTx bit will toggle the state of its internal
 * pull-up resistor.
 *
 * @param port The GPIO port.
 * @param pin  The pin number.
 * @return Std_Return_t Status of the operation (OK or ERROR).
 *         Returns ERROR if the provided port or pin is invalid.
 */
Std_Return_t GPIO_TogglePin(Port_t port, Pin_t pin)
{
    // Input validation
    if (!IS_VALID_PORT(port) || !IS_VALID_PIN(pin))
    {
        // Log error or handle appropriately
        return ERROR;
    }

    // Use a switch statement to select the correct Port Data Register (PORTx)
    // based on the provided port.
    switch (port)
    {
        case PORTA:
            // Toggle the corresponding bit using XOR
            PORTA ^= (1 << pin);
            break;

        case PORTB:
            PORTB ^= (1 << pin);
            break;

        case PORTC:
            PORTC ^= (1 << pin);
            break;

        case PORTD:
            PORTD ^= (1 << pin);
            break;

        default:
            return ERROR;
    }

    return OK;
}

/**
 * @brief Reads the digital state of a specific GPIO pin.
 *
 * This function reads the state of the corresponding bit in the PINx register.
 * If the pin is configured as input, this reflects the external voltage level
 * on the pin.
 * If the pin is configured as output, reading PINx reads the state of the
 * *output buffer*, not the actual external voltage (though they are usually the same).
 *
 * @param port  The GPIO port.
 * @param pin   The pin number.
 * @param state Pointer to a State_t variable where the read state (LOW or HIGH)
 *              will be stored.
 * @return Std_Return_t Status of the operation (OK or ERROR).
 *         Returns ERROR if the provided port or pin is invalid, or if
 *         the state pointer is NULL.
 */
Std_Return_t GPIO_ReadPin(Port_t port, Pin_t pin, State_t *state)
{
    // Input validation
    if (!IS_VALID_PORT(port) || !IS_VALID_PIN(pin) || (state == NULL))
    {
        // Log error or handle appropriately
        return ERROR;
    }

    // Use a switch statement to select the correct Pin Input Register (PINx)
    // based on the provided port.
    switch (port)
    {
        case PORTA:
            // Read the corresponding bit from PINA
            if ((PINA >> pin) & 1) // Check if the bit is set
            {
                *state = HIGH;
            }
            else
            {
                *state = LOW;
            }
            break;

        case PORTB:
            if ((PINB >> pin) & 1)
            {
                *state = HIGH;
            }
            else
            {
                *state = LOW;
            }
            break;

        case PORTC:
            if ((PINC >> pin) & 1)
            {
                *state = HIGH;
            }
            else
            {
                *state = LOW;
            }
            break;

        case PORTD:
            if ((PIND >> pin) & 1)
            {
                *state = HIGH;
            }
            else
            {
                *state = LOW;
            }
            break;

        default:
             // Should not reach here due to IS_VALID_PORT check, but for safety:
            *state = LOW; // Set a default/safe value
            return ERROR;
    }

    return OK;
}

//==============================================================================
// End of File
//==============================================================================