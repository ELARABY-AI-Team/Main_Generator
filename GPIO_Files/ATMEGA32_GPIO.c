/**
 * @file GPIO.c
 * @brief Production-ready GPIO driver implementation for ATMEGA32 microcontroller.
 *
 * This file implements the functions declared in ATMEGA32_GPIO.h, providing
 * a clean and safe interface for controlling GPIO pins.
 *
 * @note This driver assumes the ATMEGA32_GPIO.h header file defines:
 *       - Port_t, Direction_t, State_t, Pullup_t, GPIO_Status_t enums/types.
 *       - GPIO_STATUS_OK and specific error status values.
 *       - Function prototypes matching the implementations below.
 *       - Inclusion of `<avr/io.h>` or equivalent for hardware register definitions (DDRA, PORTA, PINA, etc.).
 *       - Valid ranges for Port_t (0-3 for A-D) and pin numbers (0-7).
 *
 * @author Your Name/Company
 * @date YYYY-MM-DD
 * @version 1.0
 * @copyright Your License (e.g., MIT, GPL, Proprietary)
 */

/*
 * Include necessary headers. Order as requested.
 */
#include "ATMEGA32_MAIN.h"  /* Assumed to contain general MCU includes/defines */
#include "ATMEGA32_GPIO.h" /* Contains GPIO specific types, status, and prototypes */
/* <avr/io.h> is typically included by ATMEGA32_GPIO.h or ATMEGA32_MAIN.h for register definitions */

/**
 * @brief Initializes a single GPIO pin with specified direction and pull-up state.
 *
 * For output pins, the initial state is set to LOW.
 * For input pins, the pull-up resistor is enabled or disabled.
 *
 * @param port      The GPIO port (e.g., PORT_A, PORT_B).
 * @param pin       The pin number within the port (0-7).
 * @param direction The desired direction (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 * @param pullup    The desired pull-up state for input pins (GPIO_PULLUP_ENABLED or GPIO_PULLUP_DISABLED). Ignored for output pins.
 * @return GPIO_Status_t Status of the initialization (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_InitPin(Port_t port, uint8_t pin, Direction_t direction, Pullup_t pullup)
{
    /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    if (pin > 7)
    {
        return GPIO_STATUS_ERROR_INVALID_PIN;
    }
    if (direction != GPIO_DIRECTION_INPUT && direction != GPIO_DIRECTION_OUTPUT)
    {
        return GPIO_STATUS_ERROR_INVALID_DIRECTION;
    }
    if (pullup != GPIO_PULLUP_DISABLED && pullup != GPIO_PULLUP_ENABLED)
    {
        return GPIO_STATUS_ERROR_INVALID_PULLUP;
    }

    /*
     * Configure Direction (DDRx Register):
     * Set bit for output, clear bit for input.
     */
    switch (port)
    {
        case PORT_A:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRA |= (1 << pin); // Set pin as output
            else
                DDRA &= ~(1 << pin); // Set pin as input
            break;
        case PORT_B:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRB |= (1 << pin); // Set pin as output
            else
                DDRB &= ~(1 << pin); // Set pin as input
            break;
        case PORT_C:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRC |= (1 << pin); // Set pin as output
            else
                DDRC &= ~(1 << pin); // Set pin as input
            break;
        case PORT_D:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRD |= (1 << pin); // Set pin as output
            else
                DDRD &= ~(1 << pin); // Set pin as input
            break;
        default:
            /* Should not reach here due to initial validation */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    /*
     * Configure Initial State (PORTx Register):
     * For output: set initial state (default to LOW).
     * For input: enable/disable pull-up resistor.
     */
    switch (port)
    {
        case PORT_A:
            if (direction == GPIO_DIRECTION_OUTPUT)
            {
                PORTA &= ~(1 << pin); // Set initial output state to LOW
            }
            else /* GPIO_DIRECTION_INPUT */
            {
                if (pullup == GPIO_PULLUP_ENABLED)
                    PORTA |= (1 << pin); // Enable pull-up
                else
                    PORTA &= ~(1 << pin); // Disable pull-up
            }
            break;
        case PORT_B:
             if (direction == GPIO_DIRECTION_OUTPUT)
            {
                PORTB &= ~(1 << pin); // Set initial output state to LOW
            }
            else /* GPIO_DIRECTION_INPUT */
            {
                if (pullup == GPIO_PULLUP_ENABLED)
                    PORTB |= (1 << pin); // Enable pull-up
                else
                    PORTB &= ~(1 << pin); // Disable pull-up
            }
            break;
        case PORT_C:
             if (direction == GPIO_DIRECTION_OUTPUT)
            {
                PORTC &= ~(1 << pin); // Set initial output state to LOW
            }
            else /* GPIO_DIRECTION_INPUT */
            {
                if (pullup == GPIO_PULLUP_ENABLED)
                    PORTC |= (1 << pin); // Enable pull-up
                else
                    PORTC &= ~(1 << pin); // Disable pull-up
            }
            break;
        case PORT_D:
             if (direction == GPIO_DIRECTION_OUTPUT)
            {
                PORTD &= ~(1 << pin); // Set initial output state to LOW
            }
            else /* GPIO_DIRECTION_INPUT */
            {
                if (pullup == GPIO_PULLUP_ENABLED)
                    PORTD |= (1 << pin); // Enable pull-up
                else
                    PORTD &= ~(1 << pin); // Disable pull-up
            }
            break;
        default:
            /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    return GPIO_STATUS_OK;
}

/**
 * @brief Writes a specified state (HIGH or LOW) to a GPIO pin.
 *
 * @note This function assumes the pin has been configured as an output.
 *       Writing to a pin configured as input affects the pull-up resistor setting.
 *
 * @param port  The GPIO port.
 * @param pin   The pin number (0-7).
 * @param state The desired state (GPIO_STATE_LOW or GPIO_STATE_HIGH).
 * @return GPIO_Status_t Status of the write operation (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_WritePin(Port_t port, uint8_t pin, State_t state)
{
    /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    if (pin > 7)
    {
        return GPIO_STATUS_ERROR_INVALID_PIN;
    }
     if (state != GPIO_STATE_LOW && state != GPIO_STATE_HIGH)
    {
        return GPIO_STATUS_ERROR_INVALID_STATE;
    }

    /* Write state to PORTx register */
    switch (port)
    {
        case PORT_A:
            if (state == GPIO_STATE_HIGH)
                PORTA |= (1 << pin); // Set pin high
            else
                PORTA &= ~(1 << pin); // Set pin low
            break;
        case PORT_B:
            if (state == GPIO_STATE_HIGH)
                PORTB |= (1 << pin); // Set pin high
            else
                PORTB &= ~(1 << pin); // Set pin low
            break;
        case PORT_C:
            if (state == GPIO_STATE_HIGH)
                PORTC |= (1 << pin); // Set pin high
            else
                PORTC &= ~(1 << pin); // Set pin low
            break;
        case PORT_D:
            if (state == GPIO_STATE_HIGH)
                PORTD |= (1 << pin); // Set pin high
            else
                PORTD &= ~(1 << pin); // Set pin low
            break;
        default:
            /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    return GPIO_STATUS_OK;
}

/**
 * @brief Reads the current state (HIGH or LOW) of a GPIO pin.
 *
 * This function reads the PINx register, which reflects the actual voltage
 * level on the pin, regardless of its direction.
 *
 * @param port  The GPIO port.
 * @param pin   The pin number (0-7).
 * @param state Pointer to a State_t variable to store the read state.
 * @return GPIO_Status_t Status of the read operation (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_ReadPin(Port_t port, uint8_t pin, State_t* state)
{
    /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    if (pin > 7)
    {
        return GPIO_STATUS_ERROR_INVALID_PIN;
    }
    if (state == NULL)
    {
        return GPIO_STATUS_ERROR_NULL_POINTER;
    }

    /* Read state from PINx register */
    uint8_t pin_value;
    switch (port)
    {
        case PORT_A:
            pin_value = (PINA >> pin) & 1;
            break;
        case PORT_B:
            pin_value = (PINB >> pin) & 1;
            break;
        case PORT_C:
            pin_value = (PINC >> pin) & 1;
            break;
        case PORT_D:
            pin_value = (PIND >> pin) & 1;
            break;
        default:
            /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    *state = (pin_value == 1) ? GPIO_STATE_HIGH : GPIO_STATE_LOW;

    return GPIO_STATUS_OK;
}

/**
 * @brief Toggles the current state of a GPIO pin.
 *
 * This function utilizes the ATMEGA32's feature of writing to the PINx register
 * to toggle the corresponding PORTx bit atomically.
 *
 * @note This function assumes the pin has been configured as an output.
 *       Toggling an input pin's PORTx bit toggles its pull-up state.
 *
 * @param port  The GPIO port.
 * @param pin   The pin number (0-7).
 * @return GPIO_Status_t Status of the toggle operation (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_TogglePin(Port_t port, uint8_t pin)
{
     /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    if (pin > 7)
    {
        return GPIO_STATUS_ERROR_INVALID_PIN;
    }

    /*
     * Toggle pin state by writing '1' to the corresponding bit in the PINx register.
     * This is an AVR-specific feature for atomic toggling of PORTx output bits.
     */
    switch (port)
    {
        case PORT_A:
            PINA = (1 << pin);
            break;
        case PORT_B:
            PINB = (1 << pin);
            break;
        case PORT_C:
            PINC = (1 << pin);
            break;
        case PORT_D:
            PIND = (1 << pin);
            break;
        default:
             /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    return GPIO_STATUS_OK;
}

/**
 * @brief Writes an entire byte value to the PORTx register of a GPIO port.
 *
 * This function sets the state of all 8 pins on the specified port simultaneously.
 * For pins configured as outputs, this sets their HIGH/LOW state.
 * For pins configured as inputs, this enables/disables their pull-up resistors.
 *
 * @param port  The GPIO port.
 * @param value The 8-bit value to write to the PORTx register.
 * @return GPIO_Status_t Status of the write operation (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_WritePort(Port_t port, uint8_t value)
{
    /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    /* Write value to PORTx register */
    switch (port)
    {
        case PORT_A:
            PORTA = value;
            break;
        case PORT_B:
            PORTB = value;
            break;
        case PORT_C:
            PORTC = value;
            break;
        case PORT_D:
            PORTD = value;
            break;
        default:
            /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    return GPIO_STATUS_OK;
}

/**
 * @brief Reads the current state of all 8 pins on a GPIO port.
 *
 * This function reads the PINx register, which reflects the actual voltage
 * levels on the pins.
 *
 * @param port  The GPIO port.
 * @param value Pointer to a uint8_t variable to store the read byte.
 * @return GPIO_Status_t Status of the read operation (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_ReadPort(Port_t port, uint8_t* value)
{
    /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    if (value == NULL)
    {
        return GPIO_STATUS_ERROR_NULL_POINTER;
    }

    /* Read value from PINx register */
    switch (port)
    {
        case PORT_A:
            *value = PINA;
            break;
        case PORT_B:
            *value = PINB;
            break;
        case PORT_C:
            *value = PINC;
            break;
        case PORT_D:
            *value = PIND;
            break;
        default:
            /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }

    return GPIO_STATUS_OK;
}

/* --- Optional/Helper Functions (Can be added based on need) --- */

/**
 * @brief Sets the direction of a single GPIO pin.
 *
 * @note This function only modifies the DDRx register bit.
 *       It does not affect the state or pull-up setting in PORTx.
 *
 * @param port      The GPIO port.
 * @param pin       The pin number (0-7).
 * @param direction The desired direction (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 * @return GPIO_Status_t Status of the operation (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_SetPinDirection(Port_t port, uint8_t pin, Direction_t direction)
{
    /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    if (pin > 7)
    {
        return GPIO_STATUS_ERROR_INVALID_PIN;
    }
    if (direction != GPIO_DIRECTION_INPUT && direction != GPIO_DIRECTION_OUTPUT)
    {
        return GPIO_STATUS_ERROR_INVALID_DIRECTION;
    }

     /* Configure Direction (DDRx Register) */
    switch (port)
    {
        case PORT_A:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRA |= (1 << pin); // Set pin as output
            else
                DDRA &= ~(1 << pin); // Set pin as input
            break;
        case PORT_B:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRB |= (1 << pin); // Set pin as output
            else
                DDRB &= ~(1 << pin); // Set pin as input
            break;
        case PORT_C:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRC |= (1 << pin); // Set pin as output
            else
                DDRC &= ~(1 << pin); // Set pin as input
            break;
        case PORT_D:
            if (direction == GPIO_DIRECTION_OUTPUT)
                DDRD |= (1 << pin); // Set pin as output
            else
                DDRD &= ~(1 << pin); // Set pin as input
            break;
        default:
            /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    return GPIO_STATUS_OK;
}

/**
 * @brief Sets the pull-up state for a single GPIO pin configured as input.
 *
 * @note This function only modifies the PORTx register bit. It is typically
 *       used after setting the direction to input. Setting the PORTx bit
 *       for an output pin changes its state, not pull-up.
 *
 * @param port      The GPIO port.
 * @param pin       The pin number (0-7).
 * @param pullup    The desired pull-up state (GPIO_PULLUP_ENABLED or GPIO_PULLUP_DISABLED).
 * @return GPIO_Status_t Status of the operation (GPIO_STATUS_OK or an error code).
 */
GPIO_Status_t GPIO_SetPinPullup(Port_t port, uint8_t pin, Pullup_t pullup)
{
    /* Input Validation */
    if (port < PORT_A || port > PORT_D)
    {
        return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    if (pin > 7)
    {
        return GPIO_STATUS_ERROR_INVALID_PIN;
    }
     if (pullup != GPIO_PULLUP_DISABLED && pullup != GPIO_PULLUP_ENABLED)
    {
        return GPIO_STATUS_ERROR_INVALID_PULLUP;
    }

    /* Configure Pull-up (PORTx Register) */
    /* Note: This will set/clear the output state if pin is configured as output */
    switch (port)
    {
        case PORT_A:
            if (pullup == GPIO_PULLUP_ENABLED)
                PORTA |= (1 << pin); // Enable pull-up (or set output high)
            else
                PORTA &= ~(1 << pin); // Disable pull-up (or set output low)
            break;
        case PORT_B:
            if (pullup == GPIO_PULLUP_ENABLED)
                PORTB |= (1 << pin); // Enable pull-up (or set output high)
            else
                PORTB &= ~(1 << pin); // Disable pull-up (or set output low)
            break;
        case PORT_C:
            if (pullup == GPIO_PULLUP_ENABLED)
                PORTC |= (1 << pin); // Enable pull-up (or set output high)
            else
                PORTC &= ~(1 << pin); // Disable pull-up (or set output low)
            break;
        case PORT_D:
            if (pullup == GPIO_PULLUP_ENABLED)
                PORTD |= (1 << pin); // Enable pull-up (or set output high)
            else
                PORTD &= ~(1 << pin); // Disable pull-up (or set output low)
            break;
        default:
            /* Should not reach here */
            return GPIO_STATUS_ERROR_INVALID_PORT;
    }
    return GPIO_STATUS_OK;
}