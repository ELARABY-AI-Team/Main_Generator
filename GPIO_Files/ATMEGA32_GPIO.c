/**
 * @file    GPIO.c
 * @brief   GPIO Driver implementation for ATMEGA32 microcontroller.
 * @date    [Date of creation]
 * @author  [Your Name]
 *
 * @details This file provides the implementation for controlling GPIO pins
 *          and ports on the ATMEGA32. It includes functions for
 *          initialization, setting pin states, reading pin states,
 *          configuring port directions, writing/reading full ports,
 *          and managing internal pull-up resistors.
 *          Adheres to safe and standard embedded C practices.
 */

//==============================================================================
// Includes
//==============================================================================

// Include device-specific headers first
#include "ATMEGA32_MAIN.h" // Assumed to contain core MCU definitions, potentially including <avr/io.h>
#include "ATMEGA32_GPIO.h" // Assumed to contain GPIO function prototypes, enums, and type definitions

// Include standard libraries needed (if any, like stdint.h for uint8_t)
#include <stdint.h> // For uint8_t
#include <stddef.h> // For NULL

// --- ASSUMPTIONS ---
// 1. "ATMEGA32_MAIN.h" or a file it includes provides access to the
//    AVR-GCC specific register definitions (DDRA, PORTA, PINA, DDRB, PORTB, PINB, etc.).
//    A typical AVR setup would include <avr/io.h> for this. We assume it's covered.
// 2. "ATMEGA32_GPIO.h" defines the enums and types used in this file:
//    - GPIO_Port_t (e.g., enum { GPIO_Port_A, GPIO_Port_B, GPIO_Port_C, GPIO_Port_D })
//    - GPIO_Pin_t (e.g., uint8_t, expected 0-7)
//    - GPIO_Direction_t (e.g., enum { GPIO_Direction_Input, GPIO_Direction_Output })
//    - GPIO_State_t (e.g., enum { GPIO_State_Low, GPIO_State_High, GPIO_State_Error })
//    - GPIO_Status_t (e.g., enum { GPIO_Status_Ok, GPIO_Status_Error_InvalidPort, GPIO_Status_Error_InvalidPin, ... })
// 3. The system clock is configured elsewhere (e.g., in ATMEGA32_MAIN.c or system initialization).
//    GPIO functionality does not require specific clock setup beyond basic MCU operation.
// 4. This driver does not handle alternate pin functions (e.g., ADC, SPI, Timer outputs).
//    It only provides basic digital input/output control. Configuring alternate functions
//    must be done by the respective peripheral drivers after setting the pin direction
//    appropriately via this driver (often output for peripheral signals, input for MISO/analog).

//==============================================================================
// Defines and Macros
//==============================================================================

// No specific local macros needed beyond those assumed from header

//==============================================================================
// Private Variables
//==============================================================================

// No private variables needed for this stateless driver

//==============================================================================
// Private Function Prototypes
//==============================================================================

// Helper function to get the address of DDR, PORT, and PIN registers based on port enum
static volatile uint8_t* GPIO_GetDDRRegister(GPIO_Port_t port);
static volatile uint8_t* GPIO_GetPORTRegister(GPIO_Port_t port);
static volatile uint8_t* GPIO_GetPINRegister(GPIO_Port_t port);

//==============================================================================
// Function Implementations
//==============================================================================

/**
 * @brief   Initializes a specific GPIO pin direction.
 * @param   port      The GPIO port (e.g., GPIO_Port_A).
 * @param   pin       The pin number (0-7).
 * @param   direction The direction (GPIO_Direction_Input or GPIO_Direction_Output).
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Configures the corresponding bit in the Data Direction Register (DDRx).
 *          - Setting DDRx bit to 1 configures the pin as output.
 *          - Clearing DDRx bit to 0 configures the pin as input.
 */
GPIO_Status_t GPIO_InitPin(GPIO_Port_t port, GPIO_Pin_t pin, GPIO_Direction_t direction)
{
    volatile uint8_t* ddr_reg = GPIO_GetDDRRegister(port);

    // 1. Validate inputs
    if (ddr_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }
    if (pin > 7) // Pins are 0 through 7
    {
        return GPIO_Status_Error_InvalidPin;
    }

    // 2. Configure the pin direction
    // Using bitwise operations for safety (read-modify-write)
    if (direction == GPIO_Direction_Output)
    {
        // Set the corresponding bit in the DDR register for output
        *ddr_reg |= (1 << pin);
    }
    else if (direction == GPIO_Direction_Input)
    {
        // Clear the corresponding bit in the DDR register for input
        *ddr_reg &= ~(1 << pin);

        // Optional: Ensure pull-up is disabled by default for input (PORTx bit low)
        // This prevents unexpected current draw if the line is floating initially.
        // If pull-up is needed, it should be explicitly enabled using GPIO_EnablePullUp.
        volatile uint8_t* port_reg = GPIO_GetPORTRegister(port);
        if (port_reg != NULL) // Should not be NULL if ddr_reg wasn't
        {
            *port_reg &= ~(1 << pin);
        }
    }
    else
    {
        // Invalid direction specified
        return GPIO_Status_Error_InvalidDirection;
    }

    return GPIO_Status_Ok;
}

/**
 * @brief   Sets the output state of a specific GPIO pin.
 * @param   port  The GPIO port (e.g., GPIO_Port_A).
 * @param   pin   The pin number (0-7).
 * @param   state The desired state (GPIO_State_Low or GPIO_State_High).
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Sets the corresponding bit in the Port Output Register (PORTx).
 *          - Setting PORTx bit to 1 drives the pin high (if configured as output).
 *          - Clearing PORTx bit to 0 drives the pin low (if configured as output).
 *          This function *assumes* the pin has been configured as output using
 *          GPIO_InitPin(..., GPIO_Direction_Output). Setting PORTx for an input pin
 *          controls the internal pull-up resistor.
 */
GPIO_Status_t GPIO_WritePin(GPIO_Port_t port, GPIO_Pin_t pin, GPIO_State_t state)
{
    volatile uint8_t* port_reg = GPIO_GetPORTRegister(port);

    // 1. Validate inputs
    if (port_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }
    if (pin > 7) // Pins are 0 through 7
    {
        return GPIO_Status_Error_InvalidPin;
    }

    // 2. Set the pin state
    // Using bitwise operations for safety (read-modify-write)
    if (state == GPIO_State_High)
    {
        // Set the corresponding bit in the PORT register (drives high or enables pull-up)
        *port_reg |= (1 << pin);
    }
    else if (state == GPIO_State_Low)
    {
        // Clear the corresponding bit in the PORT register (drives low or disables pull-up)
        *port_reg &= ~(1 << pin);
    }
    else
    {
        // Invalid state specified
        return GPIO_Status_Error_InvalidState;
    }

    return GPIO_Status_Ok;
}

/**
 * @brief   Reads the actual state of a specific GPIO pin.
 * @param   port The GPIO port (e.g., GPIO_Port_A).
 * @param   pin  The pin number (0-7).
 * @return  GPIO_State_t indicating the pin state (GPIO_State_Low or GPIO_State_High)
 *          or GPIO_State_Error if input is invalid.
 *
 * @details Reads the corresponding bit in the Port Input Register (PINx).
 *          - If the pin is configured as input, PINx reflects the external voltage level.
 *          - If the pin is configured as output, PINx reflects the voltage level being driven
 *            by the output buffer (which should match the state in the PORTx register).
 */
GPIO_State_t GPIO_ReadPin(GPIO_Port_t port, GPIO_Pin_t pin)
{
    volatile uint8_t* pin_reg = GPIO_GetPINRegister(port);

    // 1. Validate inputs
    if (pin_reg == NULL)
    {
        return GPIO_State_Error; // Indicate error with a special state value
    }
    if (pin > 7) // Pins are 0 through 7
    {
        return GPIO_State_Error; // Indicate error with a special state value
    }

    // 2. Read the pin state
    // Check if the corresponding bit in the PIN register is set
    if ((*pin_reg >> pin) & 1)
    {
        return GPIO_State_High;
    }
    else
    {
        return GPIO_State_Low;
    }
}

/**
 * @brief   Toggles the output state of a specific GPIO pin.
 * @param   port The GPIO port (e.g., GPIO_Port_A).
 * @param   pin  The pin number (0-7).
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Toggles the corresponding bit in the Port Output Register (PORTx).
 *          The ATMEGA32 has a specific feature where writing a '1' to a PINx
 *          bit toggles the corresponding PORTx bit. This is used here for an
 *          atomic toggle operation.
 *          This function *assumes* the pin has been configured as output.
 */
GPIO_Status_t GPIO_TogglePin(GPIO_Port_t port, GPIO_Pin_t pin)
{
    volatile uint8_t* pin_reg = GPIO_GetPINRegister(port);

    // 1. Validate inputs
    if (pin_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }
    if (pin > 7) // Pins are 0 through 7
    {
        return GPIO_Status_Error_InvalidPin;
    }

    // 2. Toggle the pin state using the PIN register write feature
    // Writing a '1' to a bit in PINx toggles the corresponding bit in PORTx
    *pin_reg = (1 << pin);

    return GPIO_Status_Ok;
}


/**
 * @brief   Sets the direction for all pins of a specific GPIO port simultaneously.
 * @param   port          The GPIO port (e.g., GPIO_Port_A).
 * @param   direction_byte An 8-bit value where each bit corresponds to a pin
 *                        (1 for output, 0 for input).
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Writes the 8-bit direction_byte directly to the Data Direction Register (DDRx).
 */
GPIO_Status_t GPIO_SetPortDirection(GPIO_Port_t port, uint8_t direction_byte)
{
    volatile uint8_t* ddr_reg = GPIO_GetDDRRegister(port);

    // 1. Validate inputs
    if (ddr_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }

    // 2. Set the port direction
    *ddr_reg = direction_byte;

    // 3. Optional: Clear the corresponding PORTx register bits for inputs
    //    to disable pull-ups by default. This prevents floating inputs
    //    from enabling pull-ups if they were previously outputs set high.
    //    This should only clear bits where the corresponding DDR bit is 0.
    volatile uint8_t* port_reg = GPIO_GetPORTRegister(port);
    if (port_reg != NULL) // Should not be NULL if ddr_reg wasn't
    {
        // Only clear bits in PORTx where the corresponding bit in direction_byte is 0 (input)
        *port_reg &= direction_byte; // Clear bits that are input (0 in direction_byte)
    }


    return GPIO_Status_Ok;
}

/**
 * @brief   Writes an 8-bit value to all pins of a specific GPIO port simultaneously.
 * @param   port  The GPIO port (e.g., GPIO_Port_A).
 * @param   value The 8-bit value to write to the port's PORTx register.
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Writes the 8-bit value directly to the Port Output Register (PORTx).
 *          For pins configured as output, this sets their state. For pins
 *          configured as input, this enables (bit 1) or disables (bit 0) the
 *          internal pull-up resistor.
 *          This function *assumes* the port directions are configured appropriately
 *          using GPIO_SetPortDirection or GPIO_InitPin.
 */
GPIO_Status_t GPIO_WritePort(GPIO_Port_t port, uint8_t value)
{
    volatile uint8_t* port_reg = GPIO_GetPORTRegister(port);

    // 1. Validate inputs
    if (port_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }

    // 2. Write the value to the port's output register
    *port_reg = value;

    return GPIO_Status_Ok;
}

/**
 * @brief   Reads the actual state of all pins of a specific GPIO port simultaneously.
 * @param   port  The GPIO port (e.g., GPIO_Port_A).
 * @param   value Pointer to a uint8_t variable where the read port value will be stored.
 *                Each bit corresponds to a pin's state.
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Reads the 8-bit value from the Port Input Register (PINx).
 *          This register reflects the actual voltage level on each pin,
 *          regardless of whether it's configured as input or output.
 *          On output pins, PINx reads the driven voltage. On input pins,
 *          PINx reads the external signal level.
 */
GPIO_Status_t GPIO_ReadPort(GPIO_Port_t port, uint8_t *value)
{
    volatile uint8_t* pin_reg = GPIO_GetPINRegister(port);

    // 1. Validate inputs
    if (pin_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }
    if (value == NULL)
    {
        return GPIO_Status_Error_NullPointer;
    }

    // 2. Read the value from the port's input register
    *value = *pin_reg;

    return GPIO_Status_Ok;
}

/**
 * @brief   Enables the internal pull-up resistor for a specific input pin.
 * @param   port The GPIO port (e.g., GPIO_Port_A).
 * @param   pin  The pin number (0-7).
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Sets the corresponding bit in the Port Output Register (PORTx).
 *          Setting a bit in PORTx *while the pin is configured as input*
 *          (DDRx bit is 0) enables the internal pull-up resistor (~50kOhm).
 *          This function *assumes* the pin has been configured as input
 *          using GPIO_InitPin(..., GPIO_Direction_Input).
 */
GPIO_Status_t GPIO_EnablePullUp(GPIO_Port_t port, GPIO_Pin_t pin)
{
    volatile uint8_t* port_reg = GPIO_GetPORTRegister(port);

    // 1. Validate inputs
    if (port_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }
    if (pin > 7) // Pins are 0 through 7
    {
        return GPIO_Status_Error_InvalidPin;
    }

    // 2. Set the corresponding bit in the PORT register to enable pull-up
    //    (Requires the pin to be configured as input via DDRx)
    *port_reg |= (1 << pin);

    return GPIO_Status_Ok;
}

/**
 * @brief   Disables the internal pull-up resistor for a specific input pin.
 * @param   port The GPIO port (e.g., GPIO_Port_A).
 * @param   pin  The pin number (0-7).
 * @return  GPIO_Status_t indicating success or failure.
 *
 * @details Clears the corresponding bit in the Port Output Register (PORTx).
 *          Clearing a bit in PORTx *while the pin is configured as input*
 *          (DDRx bit is 0) disables the internal pull-up resistor.
 *          This function *assumes* the pin has been configured as input.
 */
GPIO_Status_t GPIO_DisablePullUp(GPIO_Port_t port, GPIO_Pin_t pin)
{
    volatile uint8_t* port_reg = GPIO_GetPORTRegister(port);

    // 1. Validate inputs
    if (port_reg == NULL)
    {
        return GPIO_Status_Error_InvalidPort;
    }
    if (pin > 7) // Pins are 0 through 7
    {
        return GPIO_Status_Error_InvalidPin;
    }

    // 2. Clear the corresponding bit in the PORT register to disable pull-up
    //    (Requires the pin to be configured as input via DDRx)
    *port_reg &= ~(1 << pin);

    return GPIO_Status_Ok;
}


//==============================================================================
// Private Function Implementations
//==============================================================================

/**
 * @brief   Helper function to get the address of the DDR register for a given port.
 * @param   port The GPIO port.
 * @return  Pointer to the volatile 8-bit DDR register, or NULL if the port is invalid.
 */
static volatile uint8_t* GPIO_GetDDRRegister(GPIO_Port_t port)
{
    // Assumes GPIO_Port_t maps to valid values corresponding to ports A, B, C, D
    // and that DDRA, DDRB, DDRC, DDRD are defined (e.g., via <avr/io.h>)
    switch (port)
    {
        case GPIO_Port_A: return &DDRA;
        case GPIO_Port_B: return &DDRB;
        case GPIO_Port_C: return &DDRC;
        case GPIO_Port_D: return &DDRD;
        default:          return NULL; // Invalid port
    }
}

/**
 * @brief   Helper function to get the address of the PORT register for a given port.
 * @param   port The GPIO port.
 * @return  Pointer to the volatile 8-bit PORT register, or NULL if the port is invalid.
 */
static volatile uint8_t* GPIO_GetPORTRegister(GPIO_Port_t port)
{
    // Assumes GPIO_Port_t maps to valid values corresponding to ports A, B, C, D
    // and that PORTA, PORTB, PORTC, PORTD are defined (e.g., via <avr/io.h>)
    switch (port)
    {
        case GPIO_Port_A: return &PORTA;
        case GPIO_Port_B: return &PORTB;
        case GPIO_Port_C: return &PORTC;
        case GPIO_Port_D: return &PORTD;
        default:          return NULL; // Invalid port
    }
}

/**
 * @brief   Helper function to get the address of the PIN register for a given port.
 * @param   port The GPIO port.
 * @return  Pointer to the volatile 8-bit PIN register, or NULL if the port is invalid.
 */
static volatile uint8_t* GPIO_GetPINRegister(GPIO_Port_t port)
{
    // Assumes GPIO_Port_t maps to valid values corresponding to ports A, B, C, D
    // and that PINA, PINB, PINC, PIND are defined (e.g., via <avr/io.h>)
    switch (port)
    {
        case GPIO_Port_A: return &PINA;
        case GPIO_Port_B: return &PINB;
        case GPIO_Port_C: return &PINC;
        case GPIO_Port_D: return &PIND;
        default:          return NULL; // Invalid port
    }
}

//==============================================================================
// End of File
//==============================================================================