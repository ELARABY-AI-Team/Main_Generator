/***********************************************************************************************************************
* File Name      : ATMEGA32_GPIO.c
* Description    : ATMEGA32 GPIO Driver Implementation
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "ATMEGA32_GPIO.h"

/*
 * @brief Sets the logic value of a specific GPIO pin.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number within the port (e.g., pin_0, pin_1).
 * @param value The logic value to set (0 for low, non-zero for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            if(value & (1 << pin)) SET_BIT(PORTA, pin); else CLR_BIT(PORTA, pin); /* PDF Reference - PORTx */
            break;
        case port_B:
            if(value & (1 << pin)) SET_BIT(PORTB, pin); else CLR_BIT(PORTB, pin); /* PDF Reference - PORTx */
            break;
        case port_C:
            if(value & (1 << pin)) SET_BIT(PORTC, pin); else CLR_BIT(PORTC, pin); /* PDF Reference - PORTx */
            break;
        case port_D:
            if(value & (1 << pin)) SET_BIT(PORTD, pin); else CLR_BIT(PORTD, pin); /* PDF Reference - PORTx */
            break;
        default:
            break;
    }
}

/*
 * @brief Gets the logic value of a specific GPIO pin.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number within the port (e.g., pin_0, pin_1).
 * @return The logic value of the pin (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();
    tbyte ret_val = 0;
    switch(port)
    {
        case port_A:
            ret_val = GET_BIT(PINA, pin); /* PDF Reference - PINx */
            break;
        case port_B:
            ret_val = GET_BIT(PINB, pin); /* PDF Reference - PINx */
            break;
        case port_C:
            ret_val = GET_BIT(PINC, pin); /* PDF Reference - PINx */
            break;
        case port_D:
            ret_val = GET_BIT(PIND, pin); /* PDF Reference - PINx */
            break;
        default:
            break;
    }
    return ret_val;
}

/*
 * @brief Toggles the logic value of a specific GPIO pin.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number within the port (e.g., pin_0, pin_1).
 */
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            TOG_BIT(PORTA, pin); /* PDF Reference - PORTx */
            break;
        case port_B:
            TOG_BIT(PORTB, pin); /* PDF Reference - PORTx */
            break;
        case port_C:
            TOG_BIT(PORTC, pin); /* PDF Reference - PORTx */
            break;
        case port_D:
            TOG_BIT(PORTD, pin); /* PDF Reference - PORTx */
            break;
        default:
            break;
    }
}

/*
 * @brief Gets the direction configuration of a specific GPIO pin.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number within the port (e.g., pin_0, pin_1).
 * @return The direction configuration (input or output).
 */
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();
    t_direction ret_val = input; // Assuming input maps to 0, output maps to 1
    switch(port)
    {
        case port_A:
            ret_val = (t_direction)GET_BIT(DDRA, pin); /* PDF Reference - DDRx */
            break;
        case port_B:
            ret_val = (t_direction)GET_BIT(DDRB, pin); /* PDF Reference - DDRx */
            break;
        case port_C:
            ret_val = (t_direction)GET_BIT(DDRC, pin); /* PDF Reference - DDRx */
            break;
        case port_D:
            ret_val = (t_direction)GET_BIT(DDRD, pin); /* PDF Reference - DDRx */
            break;
        default:
            break;
    }
    return ret_val;
}

/*
 * @brief Initializes a specific GPIO pin as an output.
 *
 * Sets the direction, initial value, usage, and connection type.
 * Note: Usage and connection type settings cannot be implemented for general
 *       GPIO based on the provided ATMEGA32 PDF content, as there are no
 *       corresponding registers described for these settings on arbitrary pins.
 *       The do-while loop requirement for waiting for direction set is also
 *       omitted as DDR registers do not have a busy state documented in the PDF.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number within the port (e.g., pin_0, pin_1).
 * @param value The initial logic value (0 for low, non-zero for high).
 * @param usage The intended usage (general_usage, communication_usage).
 * @param conn The output connection type (standard_connection, open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();
    // The requirement (do method) while(GPIO_Direction_get(...) != output);
    // is omitted as DDR registers do not have a busy state documented in the PDF.

    // Logic for t_usage and t_output_conn is omitted as the provided PDF
    // does not describe corresponding registers for general GPIO pins.

    switch(port)
    {
        case port_A:
            // Ensure pull-up is off (if was input) and set initial output level
            (value & (1 << pin)) ? SET_BIT(PORTA, pin) : CLR_BIT(PORTA, pin); /* PDF Reference - PORTx */
            // Set as output
            SET_BIT(DDRA, pin); /* PDF Reference - DDRx */
            break;
        case port_B:
            // Ensure pull-up is off (if was input) and set initial output level
            (value & (1 << pin)) ? SET_BIT(PORTB, pin) : CLR_BIT(PORTB, pin); /* PDF Reference - PORTx */
            // Set as output
            SET_BIT(DDRB, pin); /* PDF Reference - DDRx */
            break;
        case port_C:
            // Ensure pull-up is off (if was input) and set initial output level
            (value & (1 << pin)) ? SET_BIT(PORTC, pin) : CLR_BIT(PORTC, pin); /* PDF Reference - PORTx */
            // Set as output
            SET_BIT(DDRC, pin); /* PDF Reference - DDRC */
            break;
        case port_D:
            // Ensure pull-up is off (if was input) and set initial output level
            (value & (1 << pin)) ? SET_BIT(PORTD, pin) : CLR_BIT(PORTD, pin); /* PDF Reference - PORTx */
            // Set as output
            SET_BIT(DDRD, pin); /* PDF Reference - DDRx */
            break;
        default:
            break;
    }
}

/*
 * @brief Initializes a specific GPIO pin as an input.
 *
 * Sets the direction, usage, and pull-up configuration.
 * Note: Usage setting cannot be implemented for general GPIO based on the
 *       provided ATMEGA32 PDF content, as there are no corresponding registers
 *       described for this setting on arbitrary pins.
 *       The do-while loop requirement for waiting for direction set is also
 *       omitted as DDR registers do not have a busy state documented in the PDF.
 *       Assumes 'pull' non-zero means enable pull-up.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number within the port (e.g., pin_0, pin_1).
 * @param usage The intended usage (general_usage, communication_usage).
 * @param pull The pull-up configuration (e.g., pull_up, no_pull).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();
    // The requirement (do method) while(GPIO_Direction_get(...) != input);
    // is omitted as DDR registers do not have a busy state documented in the PDF.

    // Logic for t_usage is omitted as the provided PDF does not describe
    // corresponding registers for general GPIO pins.

    // Pull-up control via PORTx when DDRx is input. Assuming pull is a byte value
    // where non-zero enables the pull-up for the pin.

    switch(port)
    {
        case port_A:
            // Set pull-up configuration via PORTx (active when pin is input)
            (pull & (1 << pin)) ? SET_BIT(PORTA, pin) : CLR_BIT(PORTA, pin); /* PDF Reference - PORTx */ /* Assumed pull enum mapping */
            // Set as input
            CLR_BIT(DDRA, pin); /* PDF Reference - DDRx */
            break;
        case port_B:
            // Set pull-up configuration via PORTx (active when pin is input)
            (pull & (1 << pin)) ? SET_BIT(PORTB, pin) : CLR_BIT(PORTB, pin); /* PDF Reference - PORTx */ /* Assumed pull enum mapping */
            // Set as input
            CLR_BIT(DDRB, pin); /* PDF Reference - DDRB */
            break;
        case port_C:
            // Set pull-up configuration via PORTx (active when pin is input)
            (pull & (1 << pin)) ? SET_BIT(PORTC, pin) : CLR_BIT(PORTC, pin); /* PDF Reference - PORTx */ /* Assumed pull enum mapping */
            // Set as input
            CLR_BIT(DDRC, pin); /* PDF Reference - DDRC */
            break;
        case port_D:
            // Set pull-up configuration via PORTx (active when pin is input)
            (pull & (1 << pin)) ? SET_BIT(PORTD, pin) : CLR_BIT(PORTD, pin); /* PDF Reference - PORTx */ /* Assumed pull enum mapping */
            // Set as input
            CLR_BIT(DDRD, pin); /* PDF Reference - DDRD */
            break;
        default:
            break;
    }
}