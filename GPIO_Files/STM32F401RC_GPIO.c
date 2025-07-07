/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : STM32F401RC GPIO Driver Implementation
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

// Include the necessary header file. This header is assumed to define:
// - tport, tpin, tbyte, t_direction, t_usage, t_pull enums/types.
// - Macros: SET_BIT, CLR_BIT, TOG_BIT, GET_BIT.
// - Register symbols/pointers for each port (A-E, H) following the naming convention GPIOx_Px, GPIOx_PMx, GPIOx_PUx, GPIOx_POMx.
// - WDT_Reset() function declaration.
// - Include necessary standard types (e.g., from standard_types.h).
#include "STM32F401RC_GPIO.h"

// Implement GPIO_Value_Set function.
// Sets the logic level of a specific GPIO pin.
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOA_Px, pin);
            }
            else
            {
                CLR_BIT(GPIOA_Px, pin);
            }
            break;
        case port_B:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOB_Px, pin);
            }
            else
            {
                CLR_BIT(GPIOB_Px, pin);
            }
            break;
        case port_C:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOC_Px, pin);
            }
            else
            {
                CLR_BIT(GPIOC_Px, pin);
            }
            break;
        case port_D:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOD_Px, pin);
            }
            else
            {
                CLR_BIT(GPIOD_Px, pin);
            }
            break;
        case port_E:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOE_Px, pin);
            }
            else
            {
                CLR_BIT(GPIOE_Px, pin);
            }
            break;
        case port_H:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOH_Px, pin);
            }
            else
            {
                CLR_BIT(GPIOH_Px, pin);
            }
            break;
        default:
            // Handle unknown/invalid port.
            break;
    }
}

// Implement GPIO_Value_Get function.
// Gets the current logic level of a specific GPIO pin.
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();
    switch(port)
    {
        case port_A: return GET_BIT(GPIOA_Px, pin);
        case port_B: return GET_BIT(GPIOB_Px, pin);
        case port_C: return GET_BIT(GPIOC_Px, pin);
        case port_D: return GET_BIT(GPIOD_Px, pin);
        case port_E: return GET_BIT(GPIOE_Px, pin);
        case port_H: return GET_BIT(GPIOH_Px, pin);
        default:
            // Handle unknown/invalid port.
            return 0; // Return 0 (low) for unknown port bit state.
    }
}

// Implement GPIO_Value_Tog function.
// Toggles the current logic level of a specific GPIO pin.
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();
    switch(port)
    {
        case port_A: TOG_BIT(GPIOA_Px, pin); break;
        case port_B: TOG_BIT(GPIOB_Px, pin); break;
        case port_C: TOG_BIT(GPIOC_Px, pin); break;
        case port_D: TOG_BIT(GPIOD_Px, pin); break;
        case port_E: TOG_BIT(GPIOE_Px, pin); break;
        case port_H: TOG_BIT(GPIOH_Px, pin); break;
        default:
            // Handle unknown/invalid port.
            break;
    }
}

// Implement GPIO_Direction_get function.
// Gets the current direction configuration (Input/Output) of a specific GPIO pin.
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();
    switch(port)
    {
        case port_A: return (t_direction)GET_BIT(GPIOA_PMx, pin);
        case port_B: return (t_direction)GET_BIT(GPIOB_PMx, pin);
        case port_C: return (t_direction)GET_BIT(GPIOC_PMx, pin);
        case port_D: return (t_direction)GET_BIT(GPIOD_PMx, pin);
        case port_E: return (t_direction)GET_BIT(GPIOE_PMx, pin);
        case port_H: return (t_direction)GET_BIT(GPIOH_PMx, pin);
        default:
            // Handle unknown/invalid port.
            return (t_direction)0; // Return output (0) as default for unknown port direction.
    }
}

// Implement GPIO_Output_Init function.
// Initializes a specific GPIO pin as an output.
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();
    // Configure the pin as output and loop until the direction is confirmed as output.
    // The loop condition and configuration steps inside the do-block follow the strict rules provided.
    do
    {
        switch(port)
        {
            case port_A:
                CLR_BIT(GPIOA_PUx, pin); // Disable pull-up as per rule.
                if (value & (1U << pin))
                {
                    SET_BIT(GPIOA_Px, pin); // Set initial value as per rule.
                }
                else
                {
                    CLR_BIT(GPIOA_Px, pin); // Set initial value as per rule.
                }
                // Set output type based on usage as per rule (communication_usage = open-drain, general_usage = push-pull).
                if (usage == communication_usage)
                {
                    SET_BIT(GPIOA_POMx, pin);
                }
                else
                {
                    CLR_BIT(GPIOA_POMx, pin);
                }
                CLR_BIT(GPIOA_PMx, pin); // Set mode as output (0) as per rule.
                break;
            case port_B:
                CLR_BIT(GPIOB_PUx, pin);
                if (value & (1U << pin)) { SET_BIT(GPIOB_Px, pin); } else { CLR_BIT(GPIOB_Px, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOB_POMx, pin); } else { CLR_BIT(GPIOB_POMx, pin); }
                CLR_BIT(GPIOB_PMx, pin);
                break;
            case port_C:
                CLR_BIT(GPIOC_PUx, pin);
                if (value & (1U << pin)) { SET_BIT(GPIOC_Px, pin); } else { CLR_BIT(GPIOC_Px, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOC_POMx, pin); } else { CLR_BIT(GPIOC_POMx, pin); }
                CLR_BIT(GPIOC_PMx, pin);
                break;
            case port_D:
                CLR_BIT(GPIOD_PUx, pin);
                if (value & (1U << pin)) { SET_BIT(GPIOD_Px, pin); } else { CLR_BIT(GPIOD_Px, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOD_POMx, pin); } else { CLR_BIT(GPIOD_POMx, pin); }
                CLR_BIT(GPIOD_PMx, pin);
                break;
            case port_E:
                CLR_BIT(GPIOE_PUx, pin);
                if (value & (1U << pin)) { SET_BIT(GPIOE_Px, pin); } else { CLR_BIT(GPIOE_Px, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOE_POMx, pin); } else { CLR_BIT(GPIOE_POMx, pin); }
                CLR_BIT(GPIOE_PMx, pin);
                break;
            case port_H:
                CLR_BIT(GPIOH_PUx, pin);
                if (value & (1U << pin)) { SET_BIT(GPIOH_Px, pin); } else { CLR_BIT(GPIOH_Px, pin); }
                if (usage == communication_usage) { SET_BIT(GPIOH_POMx, pin); } else { CLR_BIT(GPIOH_POMx, pin); }
                CLR_BIT(GPIOH_PMx, pin);
                break;
            default:
                // Handle unknown/invalid port - configuration steps are skipped.
                break;
        }
    } while(GPIO_Direction_get(port, pin) != output); // Loop condition as per rule.
}

// Implement GPIO_Input_Init function.
// Initializes a specific GPIO pin as an input.
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();
    // Configure the pin as input and loop until the direction is confirmed as input.
    // The loop condition and configuration steps inside the do-block follow the strict rules provided.
    do
    {
        switch(port)
        {
            case port_A:
                SET_BIT(GPIOA_PUx, pin); // Enable pull-up as per rule (ignores the specific state of 'pull' parameter).
                // Set output type based on usage as per rule (unusual for input but requested).
                if (usage == communication_usage)
                {
                    SET_BIT(GPIOA_POMx, pin);
                }
                else
                {
                    CLR_BIT(GPIOA_POMx, pin);
                }
                SET_BIT(GPIOA_PMx, pin); // Set mode as input (1) as per rule.
                break;
            case port_B:
                SET_BIT(GPIOB_PUx, pin);
                if (usage == communication_usage) { SET_BIT(GPIOB_POMx, pin); } else { CLR_BIT(GPIOB_POMx, pin); }
                SET_BIT(GPIOB_PMx, pin);
                break;
            case port_C:
                SET_BIT(GPIOC_PUx, pin);
                if (usage == communication_usage) { SET_BIT(GPIOC_POMx, pin); } else { CLR_BIT(GPIOC_POMx, pin); }
                SET_BIT(GPIOC_PMx, pin);
                break;
            case port_D:
                SET_BIT(GPIOD_PUx, pin);
                if (usage == communication_usage) { SET_BIT(GPIOD_POMx, pin); } else { CLR_BIT(GPIOD_POMx, pin); }
                SET_BIT(GPIOD_PMx, pin);
                break;
            case port_E:
                SET_BIT(GPIOE_PUx, pin);
                if (usage == communication_usage) { SET_BIT(GPIOE_POMx, pin); } else { CLR_BIT(GPIOE_POMx, pin); }
                SET_BIT(GPIOE_PMx, pin);
                break;
            case port_H:
                SET_BIT(GPIOH_PUx, pin);
                if (usage == communication_usage) { SET_BIT(GPIOH_POMx, pin); } else { CLR_BIT(GPIOH_POMx, pin); }
                SET_BIT(GPIOH_PMx, pin);
                break;
            default:
                // Handle unknown/invalid port - configuration steps are skipped.
                break;
        }
    } while(GPIO_Direction_get(port, pin) != input); // Loop condition as per rule.
}