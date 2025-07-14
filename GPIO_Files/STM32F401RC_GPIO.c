/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : STM32F401RC GPIO Driver Implementation
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-14
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"

// Assuming SET_BIT, CLR_BIT, TOG_BIT, GET_BIT macros,
// GPIO peripheral base addresses (GPIOA, etc.),
// GPIO_TypeDef structure definition,
// tport, tpin, tbyte, t_direction, t_usage, t_output_conn, t_pull enums/typedefs,
// and WDT_Reset() are provided in STM32F401RC_GPIO.h or included headers.
// Based on the examples and PDF, the macros are assumed to operate on the GPIO peripheral base address pointer (e.g., GPIOA)
// and handle the specific register access (BSRR, ODR, IDR) internally for the intended operation (SET/CLR/TOG/GET).
// Example macro interpretation aligning with PDF atomic operations where possible:
// #define SET_BIT(PORT_BASE, BIT)   ((PORT_BASE)->BSRR = (1U << (BIT)))          /* PDF Reference (BSy) */
// #define CLR_BIT(PORT_BASE, BIT)   ((PORT_BASE)->BSRR = (1U << ((BIT) + 16)))   /* PDF Reference (BRy) */
// #define TOG_BIT(PORT_BASE, BIT)   ((PORT_BASE)->ODR ^= (1U << (BIT)))          /* PDF Reference (ODR) */
// #define GET_BIT(PORT_BASE, BIT)   (((PORT_BASE)->IDR >> (BIT)) & 1U)           /* PDF Reference (IDR) */


void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            if(value) SET_BIT(GPIOA, pin); else CLR_BIT(GPIOA, pin);
            break;
        case port_B:
            if(value) SET_BIT(GPIOB, pin); else CLR_BIT(GPIOB, pin);
            break;
        case port_C:
            if(value) SET_BIT(GPIOC, pin); else CLR_BIT(GPIOC, pin);
            break;
        case port_D:
            if(value) SET_BIT(GPIOD, pin); else CLR_BIT(GPIOD, pin);
            break;
        case port_E:
            if(value) SET_BIT(GPIOE, pin); else CLR_BIT(GPIOE, pin);
            break;
        case port_H: // Note: STM32F401 has GPIOH0 and GPIOH1
            if(value) SET_BIT(GPIOH, pin); else CLR_BIT(GPIOH, pin);
            break;
        default:
            break;
    }
}


tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();
    tbyte ret_val = 0;
    switch(port)
    {
        case port_A:
            ret_val = GET_BIT(GPIOA, pin);
            break;
        case port_B:
            ret_val = GET_BIT(GPIOB, pin);
            break;
        case port_C:
            ret_val = GET_BIT(GPIOC, pin);
            break;
        case port_D:
            ret_val = GET_BIT(GPIOD, pin);
            break;
        case port_E:
            ret_val = GET_BIT(GPIOE, pin);
            break;
        case port_H:
            ret_val = GET_BIT(GPIOH, pin);
            break;
        default:
            break;
    }
    return ret_val;
}


void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            TOG_BIT(GPIOA, pin);
            break;
        case port_B:
            TOG_BIT(GPIOB, pin);
            break;
        case port_C:
            TOG_BIT(GPIOC, pin);
            break;
        case port_D:
            TOG_BIT(GPIOD, pin);
            break;
        case port_E:
            TOG_BIT(GPIOE, pin);
            break;
        case port_H:
            TOG_BIT(GPIOH, pin);
            break;
        default:
            break;
    }
}


t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();
    t_direction ret_val = direction_unknown; /* Assumed enum value */
    uint32_t mode;
    switch(port)
    {
        case port_A:
            mode = (GPIOA->MODER >> (pin * 2)) & 0x03; /* PDF Reference (MODER) */
            if (mode == 0x00) ret_val = input; /* PDF Reference */ /* Assumed enum mapping */
            else if (mode == 0x01) ret_val = output; /* PDF Reference */ /* Assumed enum mapping */
            else ret_val = direction_unknown; /* Assumed enum value for other modes */
            break;
        case port_B:
            mode = (GPIOB->MODER >> (pin * 2)) & 0x03; /* PDF Reference (MODER) */
            if (mode == 0x00) ret_val = input; /* PDF Reference */ /* Assumed enum mapping */
            else if (mode == 0x01) ret_val = output; /* PDF Reference */ /* Assumed enum mapping */
            else ret_val = direction_unknown; /* Assumed enum value for other modes */
            break;
        case port_C:
            mode = (GPIOC->MODER >> (pin * 2)) & 0x03; /* PDF Reference (MODER) */
            if (mode == 0x00) ret_val = input; /* PDF Reference */ /* Assumed enum mapping */
            else if (mode == 0x01) ret_val = output; /* PDF Reference */ /* Assumed enum mapping */
            else ret_val = direction_unknown; /* Assumed enum value for other modes */
            break;
        case port_D:
            mode = (GPIOD->MODER >> (pin * 2)) & 0x03; /* PDF Reference (MODER) */
            if (mode == 0x00) ret_val = input; /* PDF Reference */ /* Assumed enum mapping */
            else if (mode == 0x01) ret_val = output; /* PDF Reference */ /* Assumed enum mapping */
            else ret_val = direction_unknown; /* Assumed enum value for other modes */
            break;
        case port_E:
            mode = (GPIOE->MODER >> (pin * 2)) & 0x03; /* PDF Reference (MODER) */
            if (mode == 0x00) ret_val = input; /* PDF Reference */ /* Assumed enum mapping */
            else if (mode == 0x01) ret_val = output; /* PDF Reference */ /* Assumed enum mapping */
            else ret_val = direction_unknown; /* Assumed enum value for other modes */
            break;
        case port_H: // Note: STM32F401 has GPIOH0 and GPIOH1
            mode = (GPIOH->MODER >> (pin * 2)) & 0x03; /* PDF Reference (MODER) */
            if (mode == 0x00) ret_val = input; /* PDF Reference */ /* Assumed enum mapping */
            else if (mode == 0x01) ret_val = output; /* PDF Reference */ /* Assumed enum mapping */
            else ret_val = direction_unknown; /* Assumed enum value for other modes */
            break;
        default:
            break;
    }
    return ret_val;
}


void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();
    // The instruction specifies a do-while loop waiting for the direction to be output.
    // This function configures the pin as output, so the loop re-attempts configuration
    // if the target state is not immediately reached.
    // Note: The 'usage' parameter and associated placeholder register instructions ('POMx', 'PMx') are ignored
    // as they conflict with setting standard GPIO Output mode (MODER 01) using documented registers from the PDF.
    do {
        switch(port)
        {
            case port_A:
                // Disable pull-up/down (PUPDR 00)
                CLR_BIT(GPIOA->PUPDR, pin * 2);     /* PDF Reference (PUPDR) */
                CLR_BIT(GPIOA->PUPDR, pin * 2 + 1); /* PDF Reference */
                // Set initial value (ODR via BSRR)
                if(value) SET_BIT(GPIOA, pin); else CLR_BIT(GPIOA, pin); // Using macros assumed to handle BSRR
                // Configure Output Type (OTYPER) - Push-pull (0) or Open-drain (1)
                if(conn == open_drain) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin); /* PDF Reference (OTYPER) */ /* Assumed enum mapping */
                // Set as general purpose output (MODER 01)
                SET_BIT(GPIOA->MODER, pin * 2);     /* PDF Reference (MODER) */
                CLR_BIT(GPIOA->MODER, pin * 2 + 1); /* PDF Reference */
                break;
            case port_B:
                CLR_BIT(GPIOB->PUPDR, pin * 2);
                CLR_BIT(GPIOB->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(value) SET_BIT(GPIOB, pin); else CLR_BIT(GPIOB, pin);
                if(conn == open_drain) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin); /* PDF Reference (OTYPER) */ /* Assumed enum mapping */
                SET_BIT(GPIOB->MODER, pin * 2);
                CLR_BIT(GPIOB->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                break;
            case port_C:
                CLR_BIT(GPIOC->PUPDR, pin * 2);
                CLR_BIT(GPIOC->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(value) SET_BIT(GPIOC, pin); else CLR_BIT(GPIOC, pin);
                if(conn == open_drain) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin); /* PDF Reference (OTYPER) */ /* Assumed enum mapping */
                SET_BIT(GPIOC->MODER, pin * 2);
                CLR_BIT(GPIOC->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                break;
            case port_D:
                CLR_BIT(GPIOD->PUPDR, pin * 2);
                CLR_BIT(GPIOD->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(value) SET_BIT(GPIOD, pin); else CLR_BIT(GPIOD, pin);
                if(conn == open_drain) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin); /* PDF Reference (OTYPER) */ /* Assumed enum mapping */
                SET_BIT(GPIOD->MODER, pin * 2);
                CLR_BIT(GPIOD->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                break;
            case port_E:
                CLR_BIT(GPIOE->PUPDR, pin * 2);
                CLR_BIT(GPIOE->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(value) SET_BIT(GPIOE, pin); else CLR_BIT(GPIOE, pin);
                if(conn == open_drain) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin); /* PDF Reference (OTYPER) */ /* Assumed enum mapping */
                SET_BIT(GPIOE->MODER, pin * 2);
                CLR_BIT(GPIOE->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                break;
            case port_H: // Note: STM32F401 has GPIOH0 and GPIOH1
                CLR_BIT(GPIOH->PUPDR, pin * 2);
                CLR_BIT(GPIOH->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(value) SET_BIT(GPIOH, pin); else CLR_BIT(GPIOH, pin);
                if(conn == open_drain) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin); /* PDF Reference (OTYPER) */ /* Assumed enum mapping */
                SET_BIT(GPIOH->MODER, pin * 2);
                CLR_BIT(GPIOH->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                break;
            default:
                break;
        }
    } while (GPIO_Direction_get(port, pin) != output);
}


void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();
    // The instruction specifies a do-while loop waiting for the direction to be input.
    // This function configures the pin as input, so the loop re-attempts configuration
    // if the target state is not immediately reached.
    // Note: The 'usage' parameter and associated placeholder register instructions ('POMx', 'PMx') are ignored
    // as they conflict with setting standard GPIO Input mode (MODER 00) using documented registers from the PDF.
    do {
        switch(port)
        {
            case port_A:
                // Set as input (MODER 00)
                CLR_BIT(GPIOA->MODER, pin * 2);     /* PDF Reference (MODER) */
                CLR_BIT(GPIOA->MODER, pin * 2 + 1); /* PDF Reference */
                // Configure Pull-up/down (PUPDR) - No Pull (00), Pull-up (01), Pull-down (10)
                CLR_BIT(GPIOA->PUPDR, pin * 2);     /* PDF Reference (PUPDR) */
                CLR_BIT(GPIOA->PUPDR, pin * 2 + 1); /* PDF Reference */
                if(pull == pull_up) SET_BIT(GPIOA->PUPDR, pin * 2); else if(pull == pull_down) SET_BIT(GPIOA->PUPDR, pin * 2 + 1); /* PDF Reference */ /* Assumed enum mapping */
                break;
            case port_B:
                CLR_BIT(GPIOB->MODER, pin * 2);
                CLR_BIT(GPIOB->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                CLR_BIT(GPIOB->PUPDR, pin * 2);
                CLR_BIT(GPIOB->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(pull == pull_up) SET_BIT(GPIOB->PUPDR, pin * 2); else if(pull == pull_down) SET_BIT(GPIOB->PUPDR, pin * 2 + 1); /* PDF Reference */ /* Assumed enum mapping */
                break;
            case port_C:
                CLR_BIT(GPIOC->MODER, pin * 2);
                CLR_BIT(GPIOC->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                CLR_BIT(GPIOC->PUPDR, pin * 2);
                CLR_BIT(GPIOC->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(pull == pull_up) SET_BIT(GPIOC->PUPDR, pin * 2); else if(pull == pull_down) SET_BIT(GPIOC->PUPDR, pin * 2 + 1); /* PDF Reference */ /* Assumed enum mapping */
                break;
            case port_D:
                CLR_BIT(GPIOD->MODER, pin * 2);
                CLR_BIT(GPIOD->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                CLR_BIT(GPIOD->PUPDR, pin * 2);
                CLR_BIT(GPIOD->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(pull == pull_up) SET_BIT(GPIOD->PUPDR, pin * 2); else if(pull == pull_down) SET_BIT(GPIOD->PUPDR, pin * 2 + 1); /* PDF Reference */ /* Assumed enum mapping */
                break;
            case port_E:
                CLR_BIT(GPIOE->MODER, pin * 2);
                CLR_BIT(GPIOE->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                CLR_BIT(GPIOE->PUPDR, pin * 2);
                CLR_BIT(GPIOE->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(pull == pull_up) SET_BIT(GPIOE->PUPDR, pin * 2); else if(pull == pull_down) SET_BIT(GPIOE->PUPDR, pin * 2 + 1); /* PDF Reference */ /* Assumed enum mapping */
                break;
            case port_H: // Note: STM32F401 has GPIOH0 and GPIOH1
                CLR_BIT(GPIOH->MODER, pin * 2);
                CLR_BIT(GPIOH->MODER, pin * 2 + 1); /* PDF Reference (MODER) */
                CLR_BIT(GPIOH->PUPDR, pin * 2);
                CLR_BIT(GPIOH->PUPDR, pin * 2 + 1); /* PDF Reference (PUPDR) */
                if(pull == pull_up) SET_BIT(GPIOH->PUPDR, pin * 2); else if(pull == pull_down) SET_BIT(GPIOH->PUPDR, pin * 2 + 1); /* PDF Reference */ /* Assumed enum mapping */
                break;
            default:
                break;
        }
    } while (GPIO_Direction_get(port, pin) != input);
}