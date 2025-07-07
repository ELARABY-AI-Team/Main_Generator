/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : GPIO driver implementation for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "STM32F401RC_GPIO.h"

// Assumed definitions from STM32F401RC_GPIO.h or included headers:
// - GPIO_TypeDef structure pointer definitions for GPIOA, GPIOB, ..., GPIOH
// - SET_BIT, CLR_BIT, TOG_BIT, GET_BIT macros
// - WDT_Reset() function declaration
// - Enum definitions for tport, tpin, t_direction, t_usage, t_output_conn, t_pull
// - typedef for tbyte (uint8_t)

/***********************************************************************************************************************
* Function Name  : GPIO_Value_Set
* Description    : Sets or clears the specified GPIO pin value.
* Arguments      : port - The GPIO port (e.g., port_a, port_b)
*                : pin  - The pin number (e.g., pin_0, pin_15)
*                : value - The value to set (0 or non-zero)
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();

    switch(port)
    {
        case port_a:
            if (value & (1U << pin)) { SET_BIT(GPIOA->ODR, pin); } else { CLR_BIT(GPIOA->ODR, pin); }
            break;
        case port_b:
            if (value & (1U << pin)) { SET_BIT(GPIOB->ODR, pin); } else { CLR_BIT(GPIOB->ODR, pin); }
            break;
        case port_c:
            if (value & (1U << pin)) { SET_BIT(GPIOC->ODR, pin); } else { CLR_BIT(GPIOC->ODR, pin); }
            break;
        case port_d:
            if (value & (1U << pin)) { SET_BIT(GPIOD->ODR, pin); } else { CLR_BIT(GPIOD->ODR, pin); }
            break;
        case port_e:
            if (value & (1U << pin)) { SET_BIT(GPIOE->ODR, pin); } else { CLR_BIT(GPIOE->ODR, pin); }
            break;
        case port_h:
            if (value & (1U << pin)) { SET_BIT(GPIOH->ODR, pin); } else { CLR_BIT(GPIOH->ODR, pin); }
            break;
        default:
            // Invalid port, do nothing
            break;
    }
}

/***********************************************************************************************************************
* Function Name  : GPIO_Value_Get
* Description    : Gets the current value of the specified GPIO pin.
* Arguments      : port - The GPIO port (e.g., port_a, port_b)
*                : pin  - The pin number (e.g., pin_0, pin_15)
* Return Value   : tbyte - The pin value (0 or 1)
***********************************************************************************************************************/
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();

    tbyte ret_val = 0; // Default return 0 for invalid port

    switch(port)
    {
        case port_a:
            ret_val = GET_BIT(GPIOA->IDR, pin);
            break;
        case port_b:
            ret_val = GET_BIT(GPIOB->IDR, pin);
            break;
        case port_c:
            ret_val = GET_BIT(GPIOC->IDR, pin);
            break;
        case port_d:
            ret_val = GET_BIT(GPIOD->IDR, pin);
            break;
        case port_e:
            ret_val = GET_BIT(GPIOE->IDR, pin);
            break;
        case port_h:
            ret_val = GET_BIT(GPIOH->IDR, pin);
            break;
        default:
            // Invalid port, return default 0
            break;
    }

    return ret_val;
}

/***********************************************************************************************************************
* Function Name  : GPIO_Value_Tog
* Description    : Toggles the specified GPIO pin value.
* Arguments      : port - The GPIO port (e.g., port_a, port_b)
*                : pin  - The pin number (e.g., pin_0, pin_15)
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();

    switch(port)
    {
        case port_a:
            TOG_BIT(GPIOA->ODR, pin);
            break;
        case port_b:
            TOG_BIT(GPIOB->ODR, pin);
            break;
        case port_c:
            TOG_BIT(GPIOC->ODR, pin);
            break;
        case port_d:
            TOG_BIT(GPIOD->ODR, pin);
            break;
        case port_e:
            TOG_BIT(GPIOE->ODR, pin);
            break;
        case port_h:
            TOG_BIT(GPIOH->ODR, pin);
            break;
        default:
            // Invalid port, do nothing
            break;
    }
}

/***********************************************************************************************************************
* Function Name  : GPIO_Direction_get
* Description    : Gets the current direction of the specified GPIO pin.
* Arguments      : port - The GPIO port (e.g., port_a, port_b)
*                : pin  - The pin number (e.g., pin_0, pin_15)
* Return Value   : t_direction - The pin direction (input or output). Returns input for non-GPIO modes or invalid port.
***********************************************************************************************************************/
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();

    uint32_t moder_val;
    t_direction dir = input; // Default to input for safety/invalid port/non-gpio modes

    switch(port)
    {
        case port_a:
            moder_val = (GPIOA->MODER >> (2 * pin)) & 0x3UL;
            if (moder_val == 0x1UL) dir = output; // 01 is General purpose output mode
            // 00 is Input mode (handled by default)
            // 10 (AF) and 11 (Analog) will also result in default 'input' return, which is reasonable for a simple driver.
            break;
        case port_b:
            moder_val = (GPIOB->MODER >> (2 * pin)) & 0x3UL;
            if (moder_val == 0x1UL) dir = output;
            break;
        case port_c:
            moder_val = (GPIOC->MODER >> (2 * pin)) & 0x3UL;
            if (moder_val == 0x1UL) dir = output;
            break;
        case port_d:
            moder_val = (GPIOD->MODER >> (2 * pin)) & 0x3UL;
            if (moder_val == 0x1UL) dir = output;
            break;
        case port_e:
            moder_val = (GPIOE->MODER >> (2 * pin)) & 0x3UL;
            if (moder_val == 0x1UL) dir = output;
            break;
        case port_h:
            moder_val = (GPIOH->MODER >> (2 * pin)) & 0x3UL;
            if (moder_val == 0x1UL) dir = output;
            break;
        default:
            // Invalid port, return default input
            break;
    }
    return dir;
}


/***********************************************************************************************************************
* Function Name  : GPIO_Output_Init
* Description    : Initializes the specified GPIO pin as output.
* Arguments      : port  - The GPIO port (e.g., port_a, port_b)
*                : pin   - The pin number (e.g., pin_0, pin_15)
*                : value - The initial value to set (0 or non-zero)
*                : usage - The usage (general_usage or communication_usage for open-drain type)
*                : conn  - Output connection type (parameter exists in signature but rules use 'usage' for type)
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();

    // Loop to ensure direction is set correctly, as per requirement
    // This loop can potentially hang if configuration fails or port is invalid.
    do {
        switch(port)
        {
            case port_a:
                // Disable pull-up/pull-down (PUPDR = 00) as per rule 5
                GPIOA->PUPDR &= ~(0x3UL << (2 * pin));

                // Set initial value (ODR)
                if (value & (1U << pin)) { SET_BIT(GPIOA->ODR, pin); } else { CLR_BIT(GPIOA->ODR, pin); }

                // Set output type (OTYPER) based on usage (communication_usage implies Open-Drain)
                if (usage == communication_usage) { SET_BIT(GPIOA->OTYPER, pin); } else { CLR_BIT(GPIOA->OTYPER, pin); }

                // Set as output (MODER = 01)
                GPIOA->MODER = (GPIOA->MODER & ~(0x3UL << (2 * pin))) | (0x1UL << (2 * pin));
                break;
            case port_b:
                GPIOB->PUPDR &= ~(0x3UL << (2 * pin));
                if (value & (1U << pin)) { SET_BIT(GPIOB->ODR, pin); } else { CLR_BIT(GPIOB->ODR, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOB->OTYPER, pin); } else { CLR_BIT(GPIOB->OTYPER, pin); }
                GPIOB->MODER = (GPIOB->MODER & ~(0x3UL << (2 * pin))) | (0x1UL << (2 * pin));
                break;
            case port_c:
                GPIOC->PUPDR &= ~(0x3UL << (2 * pin));
                if (value & (1U << pin)) { SET_BIT(GPIOC->ODR, pin); } else { CLR_BIT(GPIOC->ODR, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOC->OTYPER, pin); } else { CLR_BIT(GPIOC->OTYPER, pin); }
                GPIOC->MODER = (GPIOC->MODER & ~(0x3UL << (2 * pin))) | (0x1UL << (2 * pin));
                break;
            case port_d:
                GPIOD->PUPDR &= ~(0x3UL << (2 * pin));
                if (value & (1U << pin)) { SET_BIT(GPIOD->ODR, pin); } else { CLR_BIT(GPIOD->ODR, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOD->OTYPER, pin); } else { CLR_BIT(GPIOD->OTYPER, pin); }
                GPIOD->MODER = (GPIOD->MODER & ~(0x3UL << (2 * pin))) | (0x1UL << (2 * pin));
                break;
            case port_e:
                GPIOE->PUPDR &= ~(0x3UL << (2 * pin));
                if (value & (1U << pin)) { SET_BIT(GPIOE->ODR, pin); } else { CLR_BIT(GPIOE->ODR, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOE->OTYPER, pin); } else { CLR_BIT(GPIOE->OTYPER, pin); }
                GPIOE->MODER = (GPIOE->MODER & ~(0x3UL << (2 * pin))) | (0x1UL << (2 * pin));
                break;
            case port_h:
                 GPIOH->PUPDR &= ~(0x3UL << (2 * pin));
                if (value & (1U << pin)) { SET_BIT(GPIOH->ODR, pin); } else { CLR_BIT(GPIOH->ODR, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOH->OTYPER, pin); } else { CLR_BIT(GPIOH->OTYPER, pin); }
                GPIOH->MODER = (GPIOH->MODER & ~(0x3UL << (2 * pin))) | (0x1UL << (2 * pin));
                break;
            default:
                 // Invalid port - configuration not possible, loop condition will not be met.
                break;
        }
        // conn parameter is unused per rules for OTYPER config which uses 'usage'.
    } while (GPIO_Direction_get(port, pin) != output); // Wait until direction is confirmed output
}

/***********************************************************************************************************************
* Function Name  : GPIO_Input_Init
* Description    : Initializes the specified GPIO pin as input.
* Arguments      : port  - The GPIO port (e.g., port_a, port_b)
*                : pin   - The pin number (e.g., pin_0, pin_15)
*                : usage - The usage (parameter exists in signature, not used for config per rules for this func)
*                : pull  - The pull configuration (pull_none, pull_up, pull_down)
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();

     // Loop to ensure direction is set correctly, as per requirement
    // This loop can potentially hang if configuration fails or port is invalid.
    do {
        switch(port)
        {
            case port_a:
                // Configure pull (PUPDR) based on 'pull' parameter
                GPIOA->PUPDR &= ~(0x3UL << (2 * pin)); // Clear existing pull config
                GPIOA->PUPDR |= ((uint32_t)pull << (2 * pin)); // Set new pull config (00, 01, or 10)

                // Set as input (MODER = 00)
                GPIOA->MODER &= ~(0x3UL << (2 * pin)); // Clear bits to set 00 (Input)
                break;
            case port_b:
                GPIOB->PUPDR &= ~(0x3UL << (2 * pin));
                GPIOB->PUPDR |= ((uint32_t)pull << (2 * pin));
                GPIOB->MODER &= ~(0x3UL << (2 * pin));
                break;
            case port_c:
                GPIOC->PUPDR &= ~(0x3UL << (2 * pin));
                GPIOC->PUPDR |= ((uint32_t)pull << (2 * pin));
                GPIOC->MODER &= ~(0x3UL << (2 * pin));
                break;
            case port_d:
                GPIOD->PUPDR &= ~(0x3UL << (2 * pin));
                GPIOD->PUPDR |= ((uint32_t)pull << (2 * pin));
                GPIOD->MODER &= ~(0x3UL << (2 * pin));
                break;
            case port_e:
                GPIOE->PUPDR &= ~(0x3UL << (2 * pin));
                GPIOE->PUPDR |= ((uint32_t)pull << (2 * pin));
                GPIOE->MODER &= ~(0x3UL << (2 * pin));
                break;
            case port_h:
                GPIOH->PUPDR &= ~(0x3UL << (2 * pin));
                GPIOH->PUPDR |= ((uint32_t)pull << (2 * pin));
                GPIOH->MODER &= ~(0x3UL << (2 * pin));
                break;
            default:
                // Invalid port - configuration not possible, loop condition will not be met.
                break;
        }
        // usage parameter is unused per rules for this function (OTYPER config rule was only for Output_Init).
    } while (GPIO_Direction_get(port, pin) != input); // Wait until direction is confirmed input
}