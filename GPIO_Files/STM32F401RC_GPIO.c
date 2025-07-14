/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : GPIO driver implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-14
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"
// Assuming WDT_Reset, SET_BIT, CLR_BIT, TOG_BIT, GET_BIT macros/functions are defined in STM32F401RC_GPIO.h or a header it includes.
// Assuming GPIO peripheral base addresses (GPIOA, GPIOB, etc.) are defined and accessible (e.g., via CMSIS headers included by STM32F401RC_GPIO.h).
// Assuming the enum values for t_direction (0=input, 1=output, 2=alternate_function, 3=analog), t_output_conn (0=push_pull, 1=open_drain), and t_pull (0=no_pull, 1=pull_up, 2=pull_down) are defined in STM32F401RC_GPIO.h, matching the PDF configurations.
// Assuming the enum value for communication_usage in t_usage is defined and non-zero, and general_usage is 0.


void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            if(value) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin); /* PDF Reference */
            break;
        case port_B:
            if(value) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin); /* PDF Reference */
            break;
        case port_C:
            if(value) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin); /* PDF Reference */
            break;
        case port_D:
            if(value) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin); /* PDF Reference */
            break;
        case port_E:
            if(value) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin); /* PDF Reference */
            break;
        case port_H:
            if(value) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin); /* PDF Reference */
            break;
        default:
            break;
    }
}

tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();
    tbyte ret_val = 0; /* Strict Rule 16 */
    switch(port)
    {
        case port_A:
            ret_val = GET_BIT(GPIOA->IDR, pin); /* PDF Reference */
            break;
        case port_B:
            ret_val = GET_BIT(GPIOB->IDR, pin); /* PDF Reference */
            break;
        case port_C:
            ret_val = GET_BIT(GPIOC->IDR, pin); /* PDF Reference */
            break;
        case port_D:
            ret_val = GET_BIT(GPIOD->IDR, pin); /* PDF Reference */
            break;
        case port_E:
            ret_val = GET_BIT(GPIOE->IDR, pin); /* PDF Reference */
            break;
        case port_H:
            ret_val = GET_BIT(GPIOH->IDR, pin); /* PDF Reference */
            break;
        default:
            break;
    }
    return ret_val; /* Strict Rule 16 */
}

void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            TOG_BIT(GPIOA->ODR, pin); /* PDF Reference */
            break;
        case port_B:
            TOG_BIT(GPIOB->ODR, pin); /* PDF Reference */
            break;
        case port_C:
            TOG_BIT(GPIOC->ODR, pin); /* PDF Reference */
            break;
        case port_D:
            TOG_BIT(GPIOD->ODR, pin); /* PDF Reference */
            break;
        case port_E:
            TOG_BIT(GPIOE->ODR, pin); /* PDF Reference */
            break;
        case port_H:
            TOG_BIT(GPIOH->ODR, pin); /* PDF Reference */
            break;
        default:
            break;
    }
}

t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();
    t_direction ret_val = input; /* Strict Rule 16, PDF Reference (reset state) */
    uint32_t moder_val;

    switch(port)
    {
        case port_A:
            moder_val = (GPIOA->MODER >> (pin * 2)) & 0x03; /* PDF Reference */
            if (moder_val == 0x00) ret_val = input; /* PDF Reference */
            else if (moder_val == 0x01) ret_val = output; /* PDF Reference */
            else if (moder_val == 0x02) ret_val = alternate_function; /* PDF Reference */
            else if (moder_val == 0x03) ret_val = analog; /* PDF Reference */
            break;
        case port_B:
            moder_val = (GPIOB->MODER >> (pin * 2)) & 0x03; /* PDF Reference */
            if (moder_val == 0x00) ret_val = input; /* PDF Reference */
            else if (moder_val == 0x01) ret_val = output; /* PDF Reference */
            else if (moder_val == 0x02) ret_val = alternate_function; /* PDF Reference */
            else if (moder_val == 0x03) ret_val = analog; /* PDF Reference */
            break;
        case port_C:
            moder_val = (GPIOC->MODER >> (pin * 2)) & 0x03; /* PDF Reference */
            if (moder_val == 0x00) ret_val = input; /* PDF Reference */
            else if (moder_val == 0x01) ret_val = output; /* PDF Reference */
            else if (moder_val == 0x02) ret_val = alternate_function; /* PDF Reference */
            else if (moder_val == 0x03) ret_val = analog; /* PDF Reference */
            break;
        case port_D:
            moder_val = (GPIOD->MODER >> (pin * 2)) & 0x03; /* PDF Reference */
            if (moder_val == 0x00) ret_val = input; /* PDF Reference */
            else if (moder_val == 0x01) ret_val = output; /* PDF Reference */
            else if (moder_val == 0x02) ret_val = alternate_function; /* PDF Reference */
            else if (moder_val == 0x03) ret_val = analog; /* PDF Reference */
            break;
        case port_E:
            moder_val = (GPIOE->MODER >> (pin * 2)) & 0x03; /* PDF Reference */
            if (moder_val == 0x00) ret_val = input; /* PDF Reference */
            else if (moder_val == 0x01) ret_val = output; /* PDF Reference */
            else if (moder_val == 0x02) ret_val = alternate_function; /* PDF Reference */
            else if (moder_val == 0x03) ret_val = analog; /* PDF Reference */
            break;
        case port_H:
            moder_val = (GPIOH->MODER >> (pin * 2)) & 0x03; /* PDF Reference */
            if (moder_val == 0x00) ret_val = input; /* PDF Reference */
            else if (moder_val == 0x01) ret_val = output; /* PDF Reference */
            else if (moder_val == 0x02) ret_val = alternate_function; /* PDF Reference */
            else if (moder_val == 0x03) ret_val = analog; /* PDF Reference */
            break;
        default:
            break;
    }
    return ret_val; /* Strict Rule 16 */
}


void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();
    // Note: The prompt includes "(do method) while(GPIO_Direction_get(...) != output);" but this conflicts
    // with the required strict formatting and the example structure in rule 8.
    // Following the structure in rule 8 and the explicit register manipulation instructions.

    switch(port)
    {
        case port_A:
            // Disable pull-up/pull-down (00: No pull-up, pull-down) /* PDF Reference */
            GPIOA->PUPDR &= ~((0x03) << (pin * 2));
            // Set initial value /* PDF Reference */
            if(value) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin);
            // Set output type (Push-pull=0 or Open-drain=1) based on 'usage'
            // Prompt description: If usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin);
            // Interpreting POMx as OTYPER as per output configuration context.
            // This mapping assumes communication_usage implies Open-drain (OTy=1), general_usage implies Push-pull (OTy=0).
            if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); /* PDF Reference (OTYPER mapping assumed for usage) */
            else CLR_BIT(GPIOA->OTYPER, pin); /* PDF Reference (OTYPER mapping assumed for usage) */
            // Note: The 'conn' parameter is not used in the explicit instructions for this function's body in the prompt.

            // Set as output (General purpose output mode = 01) /* PDF Reference */
            GPIOA->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            GPIOA->MODER |= (0x01 << (pin * 2));   // Set to 01 (Output mode)
            break;
        case port_B:
            // Disable pull-up/pull-down (00: No pull-up, pull-down) /* PDF Reference */
            GPIOB->PUPDR &= ~((0x03) << (pin * 2));
            // Set initial value /* PDF Reference */
            if(value) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin);
             // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage) */
            if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin);
            else CLR_BIT(GPIOB->OTYPER, pin);
            // Set as output (General purpose output mode = 01) /* PDF Reference */
            GPIOB->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            GPIOB->MODER |= (0x01 << (pin * 2));   // Set to 01 (Output mode)
            break;
        case port_C:
             // Disable pull-up/pull-down (00: No pull-up, pull-down) /* PDF Reference */
            GPIOC->PUPDR &= ~((0x03) << (pin * 2));
            // Set initial value /* PDF Reference */
            if(value) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin);
             // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage) */
            if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin);
            else CLR_BIT(GPIOC->OTYPER, pin);
            // Set as output (General purpose output mode = 01) /* PDF Reference */
            GPIOC->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            GPIOC->MODER |= (0x01 << (pin * 2));   // Set to 01 (Output mode)
            break;
        case port_D:
             // Disable pull-up/pull-down (00: No pull-up, pull-down) /* PDF Reference */
            GPIOD->PUPDR &= ~((0x03) << (pin * 2));
            // Set initial value /* PDF Reference */
            if(value) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin);
             // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage) */
            if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin);
            else CLR_BIT(GPIOD->OTYPER, pin);
            // Set as output (General purpose output mode = 01) /* PDF Reference */
            GPIOD->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            GPIOD->MODER |= (0x01 << (pin * 2));   // Set to 01 (Output mode)
            break;
        case port_E:
             // Disable pull-up/pull-down (00: No pull-up, pull-down) /* PDF Reference */
            GPIOE->PUPDR &= ~((0x03) << (pin * 2));
            // Set initial value /* PDF Reference */
            if(value) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin);
             // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage) */
            if(usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin);
            else CLR_BIT(GPIOE->OTYPER, pin);
            // Set as output (General purpose output mode = 01) /* PDF Reference */
            GPIOE->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            GPIOE->MODER |= (0x01 << (pin * 2));   // Set to 01 (Output mode)
            break;
        case port_H:
             // Disable pull-up/pull-down (00: No pull-up, pull-down) /* PDF Reference */
            GPIOH->PUPDR &= ~((0x03) << (pin * 2));
            // Set initial value /* PDF Reference */
            if(value) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
             // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage) */
            if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin);
            else CLR_BIT(GPIOH->OTYPER, pin);
            // Set as output (General purpose output mode = 01) /* PDF Reference */
            GPIOH->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            GPIOH->MODER |= (0x01 << (pin * 2));   // Set to 01 (Output mode)
            break;
        default:
            break;
    }
}

void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();
    // Note: The prompt includes "(do method) while(GPIO_Direction_get(...) != input);" but this conflicts
    // with the required strict formatting and the example structure in rule 8.
    // Following the structure in rule 8 and the explicit register manipulation instructions.

    switch(port)
    {
        case port_A:
            // Set pull-up/pull-down based on 'pull' (00: No pull, 01: Pull-up, 10: Pull-down) /* PDF Reference */
            GPIOA->PUPDR &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            if(pull == pull_up) GPIOA->PUPDR |= (0x01 << (pin * 2)); // 01: Pull-up
            else if(pull == pull_down) GPIOA->PUPDR |= (0x02 << (pin * 2)); // 10: Pull-down

            // Prompt description: If usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin);
            // Interpreting POMx as OTYPER again, even though this register setting is illogical for an input pin.
            // Following prompt strictly for 'usage' logic.
            // Assuming communication_usage implies OTYPER=1 (Open-drain), general_usage implies OTYPER=0 (Push-pull).
            if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); /* PDF Reference (OTYPER mapping assumed for usage, illogical for input) */
            else CLR_BIT(GPIOA->OTYPER, pin); /* PDF Reference (OTYPER mapping assumed for usage, illogical for input) */
            // Note: OTYPER affects the output driver, which is disabled in input mode. This configuration has no effect on input behavior.

            // Set as input (Input = 00) /* PDF Reference */
            GPIOA->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits (sets to 00: Input mode)
            break;
        case port_B:
            // Set pull-up/pull-down based on 'pull' /* PDF Reference */
            GPIOB->PUPDR &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            if(pull == pull_up) GPIOB->PUPDR |= (0x01 << (pin * 2)); // 01: Pull-up
            else if(pull == pull_down) GPIOB->PUPDR |= (0x02 << (pin * 2)); // 10: Pull-down
            // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage, illogical for input) */
            if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin);
            else CLR_BIT(GPIOB->OTYPER, pin);
             // Set as input (Input = 00) /* PDF Reference */
            GPIOB->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits (sets to 00: Input mode)
            break;
        case port_C:
             // Set pull-up/pull-down based on 'pull' /* PDF Reference */
            GPIOC->PUPDR &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            if(pull == pull_up) GPIOC->PUPDR |= (0x01 << (pin * 2)); // 01: Pull-up
            else if(pull == pull_down) GPIOC->PUPDR |= (0x02 << (pin * 2)); // 10: Pull-down
            // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage, illogical for input) */
            if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin);
            else CLR_BIT(GPIOC->OTYPER, pin);
             // Set as input (Input = 00) /* PDF Reference */
            GPIOC->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits (sets to 00: Input mode)
            break;
        case port_D:
             // Set pull-up/pull-down based on 'pull' /* PDF Reference */
            GPIOD->PUPDR &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            if(pull == pull_up) GPIOD->PUPDR |= (0x01 << (pin * 2)); // 01: Pull-up
            else if(pull == pull_down) GPIOD->PUPDR |= (0x02 << (pin * 2)); // 10: Pull-down
            // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage, illogical for input) */
            if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin);
            else CLR_BIT(GPIOD->OTYPER, pin);
             // Set as input (Input = 00) /* PDF Reference */
            GPIOD->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits (sets to 00: Input mode)
            break;
        case port_E:
             // Set pull-up/pull-down based on 'pull' /* PDF Reference */
            GPIOE->PUPDR &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            if(pull == pull_up) GPIOE->PUPDR |= (0x01 << (pin * 2)); // 01: Pull-up
            else if(pull == pull_down) GPIOE->PUPDR |= (0x02 << (pin * 2)); // 10: Pull-down
            // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage, illogical for input) */
            if(usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin);
            else CLR_BIT(GPIOE->OTYPER, pin);
             // Set as input (Input = 00) /* PDF Reference */
            GPIOE->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits (sets to 00: Input mode)
            break;
        case port_H:
             // Set pull-up/pull-down based on 'pull' /* PDF Reference */
            GPIOH->PUPDR &= ~((0x03) << (pin * 2)); // Clear the 2 bits
            if(pull == pull_up) GPIOH->PUPDR |= (0x01 << (pin * 2)); // 01: Pull-up
            else if(pull == pull_down) GPIOH->PUPDR |= (0x02 << (pin * 2)); // 10: Pull-down
            // Set output type based on 'usage' /* PDF Reference (OTYPER mapping assumed for usage, illogical for input) */
            if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin);
            else CLR_BIT(GPIOH->OTYPER, pin);
             // Set as input (Input = 00) /* PDF Reference */
            GPIOH->MODER &= ~((0x03) << (pin * 2)); // Clear the 2 bits (sets to 00: Input mode)
            break;
        default:
            break;
    }
}