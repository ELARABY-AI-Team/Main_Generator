/***********************************************************************************************************************
* File Name      : ATMEGA32_GPIO.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "ATMEGA32_GPIO.h"

/* Assumed WDT_Reset() macro/function exists */
/* Assumed SET_BIT, CLR_BIT, TOG_BIT, GET_BIT macros exist */
/* Assumed tport, tpin, tbyte, t_direction, t_usage, t_pull, t_output_conn enums/typedefs are defined in ATMEGA32_GPIO.h */
/* Assumed t_direction: input=0, output=1 based on DDR bit value */
/* Assumed t_pull: pull_up_disabled=0, pull_up_enabled=1 based on PORT bit value for input pin */
/* Assumed register names PORTA, DDRA, PINA, PORTB, DDRB, PINB, PORTC, DDRC, PINC, PORTD, DDRD, PIND are defined */

void GPIO_Value_Set(tport port, tpin pin, tbyte value)

    /* PDF Reference: Function body structure, switch/case */
    WDT_Reset();
    switch(port)

        case port_A:
            /* PDF Reference: PORTA, bits PA0-PA7 control output state */
            if(value & (1 << pin)) SET_BIT(PORTA, pin); else CLR_BIT(PORTA, pin);
            break;
        case port_B:
            /* PDF Reference: PORTB, bits PB0-PB7 control output state */
            if(value & (1 << pin)) SET_BIT(PORTB, pin); else CLR_BIT(PORTB, pin);
            break;
        case port_C:
            /* PDF Reference: PORTC, bits PC0-PC7 control output state */
            if(value & (1 << pin)) SET_BIT(PORTC, pin); else CLR_BIT(PORTC, pin);
            break;
        case port_D:
            /* PDF Reference: PORTD, bits PD0-PD7 control output state */
            if(value & (1 << pin)) SET_BIT(PORTD, pin); else CLR_BIT(PORTD, pin);
            break;
        default:
            break;


}


tbyte GPIO_Value_Get(tport port, tpin pin)

    /* PDF Reference: Function body structure, switch/case, returning local variable */
    WDT_Reset();
    tbyte ret_val = 0; // Initialize return value

    switch(port)

        case port_A:
            /* PDF Reference: PINA, bits PINA0-PINA7 read input state */
            ret_val = GET_BIT(PINA, pin);
            break;
        case port_B:
            /* PDF Reference: PINB, bits PINB0-PINB7 read input state */
            ret_val = GET_BIT(PINB, pin);
            break;
        case port_C:
            /* PDF Reference: PINC, bits PINC0-PINC7 read input state */
            ret_val = GET_BIT(PINC, pin);
            break;
        case port_D:
            /* PDF Reference: PIND, bits PIND0-PIND7 read input state */
            ret_val = GET_BIT(PIND, pin);
            break;
        default:
            break;


    return ret_val;
}


void GPIO_Value_Tog(tport port, tpin pin)

    /* PDF Reference: Function body structure, switch/case */
    WDT_Reset();
    switch(port)

        case port_A:
            /* PDF Reference: PORTA, bits PA0-PA7 control output state (toggle) */
            TOG_BIT(PORTA, pin);
            break;
        case port_B:
            /* PDF Reference: PORTB, bits PB0-PB7 control output state (toggle) */
            TOG_BIT(PORTB, pin);
            break;
        case port_C:
            /* PDF Reference: PORTC, bits PC0-PC7 control output state (toggle) */
            TOG_BIT(PORTC, pin);
            break;
        case port_D:
            /* PDF Reference: PORTD, bits PD0-PD7 control output state (toggle) */
            TOG_BIT(PORTD, pin);
            break;
        default:
            break;


}


t_direction GPIO_Direction_get(tport port, tpin pin)

    /* PDF Reference: Function body structure, switch/case, returning local variable */
    WDT_Reset();
    t_direction ret_val = 0; // Initialize return value (assuming input=0)

    switch(port)

        case port_A:
            /* PDF Reference: DDRA, bits DDA0-DDA7 control direction (0=input, 1=output) */
            ret_val = GET_BIT(DDRA, pin);
            break;
        case port_B:
            /* PDF Reference: DDRB, bits DDB0-DDB7 control direction (0=input, 1=output) */
            ret_val = GET_BIT(DDRB, pin);
            break;
        case port_C:
            /* PDF Reference: DDRC, bits DDC0-DDC7 control direction (0=input, 1=output) */
            ret_val = GET_BIT(DDRC, pin);
            break;
        case port_D:
            /* PDF Reference: DDRD, bits DDD0-DDD7 control direction (0=input, 1=output) */
            ret_val = GET_BIT(DDRD, pin);
            break;
        default:
            break;


    /* Assumed t_direction mapping: 0=input, 1=output. Return matches DDR bit value directly. */
    return ret_val;
}


void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)

    /* PDF Reference: Function body structure, switch/case */
    WDT_Reset();

    // The prompt's instructions for usage and output_conn (using POMx)
    // do not map to real ATMEGA32 hardware registers described in the PDF.
    // Ignoring usage and conn parameters and associated logic (like POMx and the do/while loop)
    // as per core rule 5: "search for and use the real hardware register names"
    // and core rule 4: "DO NOT use placeholder registers like ... POMx".
    // Setting initial value via PORTx and direction via DDRx as per PDF.

    switch(port)

        case port_A:
            /* PDF Reference: PORTA bits set initial output value when configured as output */
            if(value & (1 << pin)) SET_BIT(PORTA, pin); else CLR_BIT(PORTA, pin);
            /* PDF Reference: DDRA bits set direction (1=output) */
            SET_BIT(DDRA, pin);
            break;
        case port_B:
            /* PDF Reference: PORTB bits set initial output value when configured as output */
            if(value & (1 << pin)) SET_BIT(PORTB, pin); else CLR_BIT(PORTB, pin);
            /* PDF Reference: DDRB bits set direction (1=output) */
            SET_BIT(DDRB, pin);
            break;
        case port_C:
            /* PDF Reference: PORTC bits set initial output value when configured as output */
            if(value & (1 << pin)) SET_BIT(PORTC, pin); else CLR_BIT(PORTC, pin);
            /* PDF Reference: DDRC bits set direction (1=output) */
            SET_BIT(DDRC, pin);
            break;
        case port_D:
            /* PDF Reference: PORTD bits set initial output value when configured as output */
            if(value & (1 << pin)) SET_BIT(PORTD, pin); else CLR_BIT(PORTD, pin);
            /* PDF Reference: DDRD bits set direction (1=output) */
            SET_BIT(DDRD, pin);
            break;
        default:
            break;


}


void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)

    /* PDF Reference: Function body structure, switch/case */
    WDT_Reset();

    // The prompt's instructions for usage (using POMx)
    // do not map to real ATMEGA32 hardware registers described in the PDF.
    // Ignoring usage parameter and associated logic (like POMx and the do/while loop)
    // as per core rule 5: "search for and use the real hardware register names"
    // and core rule 4: "DO NOT use placeholder registers like ... POMx".
    // Configuring pull-up via PORTx and direction via DDRx as per PDF.

    switch(port)

        case port_A:
            /* PDF Reference: DDRA bits set direction (0=input) */
            CLR_BIT(DDRA, pin);
            /* PDF Reference: PORTA bits configure pull-up when configured as input */
            if (pull == pull_up_enabled) SET_BIT(PORTA, pin); else CLR_BIT(PORTA, pin);
            break;
        case port_B:
            /* PDF Reference: DDRB bits set direction (0=input) */
            CLR_BIT(DDRB, pin);
            /* PDF Reference: PORTB bits configure pull-up when configured as input */
            if (pull == pull_up_enabled) SET_BIT(PORTB, pin); else CLR_BIT(PORTB, pin);
            break;
        case port_C:
            /* PDF Reference: DDRC bits set direction (0=input) */
            CLR_BIT(DDRC, pin);
            /* PDF Reference: PORTC bits configure pull-up when configured as input */
            if (pull == pull_up_enabled) SET_BIT(PORTC, pin); else CLR_BIT(PORTC, pin);
            break;
        case port_D:
            /* PDF Reference: DDRD bits set direction (0=input) */
            CLR_BIT(DDRD, pin);
            /* PDF Reference: PORTD bits configure pull-up when configured as input */
            if (pull == pull_up_enabled) SET_BIT(PORTD, pin); else CLR_BIT(PORTD, pin);
            break;
        default:
            break;


}