/***********************************************************************************************************************
* File Name      : ATMEGA32_GPIO.c
* Description    : ATMEGA32 GPIO Driver Implementation
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "ATMEGA32_GPIO.h"

/*
 * Assuming necessary register definitions (PORTA, DDRA, PINA, PORTB, DDRB, PINB, PORTC, DDRC, PINC, PORTD, DDRD, PIND),
 * bit manipulation macros (SET_BIT, CLR_BIT, TOG_BIT, GET_BIT),
 * WDT_Reset() function, and the required enums/typedefs (tport, tpin, tbyte,
 * t_direction, t_usage, t_output_conn, t_pull) are provided by or included
 * through "ATMEGA32_GPIO.h".
 *
 * As per rules 4 & 5, placeholder registers (Px, PMx, PUx, POMx) are NOT used.
 * Real ATMEGA32 registers (PORTx, DDRx, PINx) are used.
 * This means the 'usage' parameter and certain 'pull'/'conn' options
 * (like pull_down, open_drain, communication_usage tied to POMx)
 * which do not map to a simple, generic register bit on ATMEGA32 GPIOs
 * are effectively ignored in the register configuration logic, adhering to rule 4/5.
 */


void GPIO_Value_Set(tport port, tpin pin, tbyte value)

    WDT_Reset();
    switch(port)

        case port_A:
            if(value & (1 << pin)) SET_BIT(PORTA, pin); else CLR_BIT(PORTA, pin);
            break;
        case port_B:
            if(value & (1 << pin)) SET_BIT(PORTB, pin); else CLR_BIT(PORTB, pin);
            break;
        case port_C:
            if(value & (1 << pin)) SET_BIT(PORTC, pin); else CLR_BIT(PORTC, pin);
            break;
        case port_D:
            if(value & (1 << pin)) SET_BIT(PORTD, pin); else CLR_BIT(PORTD, pin);
            break;
        default:
            /* Handle error or ignore invalid port */
            break;


tbyte GPIO_Value_Get(tport port, tpin pin)

    WDT_Reset();
    tbyte ret_val = 0; /* Initialize */
    switch(port)

        case port_A:
            ret_val = GET_BIT(PINA, pin); /* Read from PINx for input state */
            break;
        case port_B:
            ret_val = GET_BIT(PINB, pin); /* Read from PINx for input state */
            break;
        case port_C:
            ret_val = GET_BIT(PINC, pin); /* Read from PINx for input state */
            break;
        case port_D:
            ret_val = GET_BIT(PIND, pin); /* Read from PINx for input state */
            break;
        default:
            /* Handle error or ignore invalid port */
            break;

    return ret_val;


void GPIO_Value_Tog(tport port, tpin pin)

    WDT_Reset();
    switch(port)

        case port_A:
            TOG_BIT(PORTA, pin); /* Toggle PORTx to change output state */
            break;
        case port_B:
            TOG_BIT(PORTB, pin); /* Toggle PORTx to change output state */
            break;
        case port_C:
            TOG_BIT(PORTC, pin); /* Toggle PORTx to change output state */
            break;
        case port_D:
            TOG_BIT(PORTD, pin); /* Toggle PORTx to change output state */
            break;
        default:
            /* Handle error or ignore invalid port */
            break;


t_direction GPIO_Direction_get(tport port, tpin pin)

    WDT_Reset();
    t_direction ret_val = (t_direction)0; /* Initialize, assuming output=0, input=1 */
    switch(port)

        case port_A:
            ret_val = (t_direction)GET_BIT(DDRA, pin);
            break;
        case port_B:
            ret_val = (t_direction)GET_BIT(DDRB, pin);
            break;
        case port_C:
            ret_val = (t_direction)GET_BIT(DDRC, pin);
            break;
        case port_D:
            ret_val = (t_direction)GET_BIT(DDRD, pin);
            break;
        default:
            /* Handle error or ignore invalid port */
            break;

    return ret_val;


void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)

    WDT_Reset();
    /*
     * Note: The 'usage' and 'conn' parameters are effectively ignored
     * as their implied register (POMx) is not a real ATMEGA32 register,
     * and rule 4/5 requires using real registers only.
     * The 'do method' while loop mentioned in description is not implemented
     * as it conflicts with the required code structure and is not standard.
     * Pull-up is disabled for output pins by clearing PORTx bit.
     */
    switch(port)

        case port_A:
            /* Disable pull-up and set initial output state */
            if(value & (1 << pin)) SET_BIT(PORTA, pin); else CLR_BIT(PORTA, pin);
            /* Set direction to output */
            CLR_BIT(DDRA, pin);
            break;
        case port_B:
            /* Disable pull-up and set initial output state */
            if(value & (1 << pin)) SET_BIT(PORTB, pin); else CLR_BIT(PORTB, pin);
            /* Set direction to output */
            CLR_BIT(DDRB, pin);
            break;
        case port_C:
            /* Disable pull-up and set initial output state */
            if(value & (1 << pin)) SET_BIT(PORTC, pin); else CLR_BIT(PORTC, pin);
            /* Set direction to output */
            CLR_BIT(DDRC, pin);
            break;
        case port_D:
            /* Disable pull-up and set initial output state */
            if(value & (1 << pin)) SET_BIT(PORTD, pin); else CLR_BIT(PORTD, pin);
            /* Set direction to output */
            CLR_BIT(DDRD, pin);
            break;
        default:
            /* Handle error or ignore invalid port */
            break;


void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)

    WDT_Reset();
    /*
     * Note: The 'usage' parameter is effectively ignored as its implied register (POMx)
     * is not a real ATMEGA32 register, and rule 4/5 requires using real registers only.
     * 'pull_down' and 'open_drain' types are ignored as they don't map to a simple
     * register bit like 'pull_up' does (via PORTx when DDRx=0).
     * The 'do method' while loop mentioned in description is not implemented
     * as it conflicts with the required code structure and is not standard.
     */
    switch(port)

        case port_A:
            /* Set direction to input */
            SET_BIT(DDRA, pin);
            /* Configure pull-up based on t_pull */
            if (pull == pull_up) SET_BIT(PORTA, pin); /* Enable pull-up */
            else CLR_BIT(PORTA, pin);               /* Disable pull-up (for no_pull, pull_down, open_drain) */
            break;
        case port_B:
            /* Set direction to input */
            SET_BIT(DDRB, pin);
            /* Configure pull-up based on t_pull */
            if (pull == pull_up) SET_BIT(PORTB, pin); /* Enable pull-up */
            else CLR_BIT(PORTB, pin);               /* Disable pull-up */
            break;
        case port_C:
            /* Set direction to input */
            SET_BIT(DDRC, pin);
            /* Configure pull-up based on t_pull */
            if (pull == pull_up) SET_BIT(PORTC, pin); /* Enable pull-up */
            else CLR_BIT(PORTC, pin);               /* Disable pull-up */
            break;
        case port_D:
            /* Set direction to input */
            SET_BIT(DDRD, pin);
            /* Configure pull-up based on t_pull */
            if (pull == pull_up) SET_BIT(PORTD, pin); /* Enable pull-up */
            else CLR_BIT(PORTD, pin);               /* Disable pull-up */
            break;
        default:
            /* Handle error or ignore invalid port */
            break;