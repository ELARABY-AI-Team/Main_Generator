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

// Assume necessary headers are included via the project's include paths or STM32F401RC_GPIO.h
// Specifically, this driver requires access to GPIO_TypeDef structure and base addresses (GPIOA, GPIOB, etc.)
// which are typically found in device-specific header files like "stm32f4xx.h" or similar provided by the MCU vendor SDK.
#include "STM32F401RC_GPIO.h"
// Assume <stdint.h> is included elsewhere for uint32_t type definitions if not in STM32F401RC_GPIO.h.
// Assuming GPIO_TypeDef and GPIOx base addresses (GPIOA, GPIOB, etc.) are defined and available
// through the included header or project settings.

/***********************************************************************************************************************
* Function Name  : GPIO_Value_Set
* Description    : Sets or clears the output value of a specific GPIO pin.
* Arguments      : port - The GPIO port (tport enum)
*                : pin - The pin number (tpin enum)
*                : value - The desired state. Evaluation uses (value & (1 << pin)).
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();
    GPIO_TypeDef *GPIOx = NULL; // Pointer to the GPIO port registers

    // Select the correct GPIO port based on the enum value
    switch(port)
    {
        case PORT_A: GPIOx = GPIOA; break;
        case PORT_B: GPIOx = GPIOB; break;
        case PORT_C: GPIOx = GPIOC; break;
        case PORT_D: GPIOx = GPIOD; break; // Assuming PORT_D is defined and exists in tport enum
        case PORT_E: GPIOx = GPIOE; break; // Assuming PORT_E is defined and exists in tport enum
        case PORT_H: GPIOx = GPIOH; break; // Assuming PORT_H is defined and exists in tport enum
        default:
            // Invalid port provided. Exit function.
            return;
    }

    // Basic validation for pin number (tpin enum values should typically be 0-15, but safety check)
    if (pin > 15)
    {
        return;
    }

    // Set or clear the bit in the Output Data Register (ODR) based on the value parameter
    // Requirement: Evaluate value using (value & (1 << pin))
    if ((value & (1U << pin))) // Check if the 'pin' bit is set within the 'value' byte
    {
        SET_BIT(GPIOx->ODR, pin);
    }
    else
    {
        CLR_BIT(GPIOx->ODR, pin);
    }
}

/***********************************************************************************************************************
* Function Name  : GPIO_Value_Get
* Description    : Reads the input value of a specific GPIO pin.
* Arguments      : port - The GPIO port (tport enum)
*                : pin - The pin number (tpin enum)
* Return Value   : tbyte - The pin value (0 or 1)
***********************************************************************************************************************/
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();
    GPIO_TypeDef *GPIOx = NULL;

    // Select the correct GPIO port
    switch(port)
    {
        case PORT_A: GPIOx = GPIOA; break;
        case PORT_B: GPIOx = GPIOB; break;
        case PORT_C: GPIOx = GPIOC; break;
        case PORT_D: GPIOx = GPIOD; break;
        case PORT_E: GPIOx = GPIOE; break;
        case PORT_H: GPIOx = GPIOH; break;
        default:
            // Invalid port. Return 0.
            return 0;
    }

    // Basic validation for pin number
    if (pin > 15)
    {
        return 0; // Invalid pin. Return 0.
    }

    // Read the bit from the Input Data Register (IDR)
    return (tbyte)GET_BIT(GPIOx->IDR, pin);
}

/***********************************************************************************************************************
* Function Name  : GPIO_Value_Tog
* Description    : Toggles the output value of a specific GPIO pin.
* Arguments      : port - The GPIO port (tport enum)
*                : pin - The pin number (tpin enum)
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();
    GPIO_TypeDef *GPIOx = NULL;

    // Select the correct GPIO port
    switch(port)
    {
        case PORT_A: GPIOx = GPIOA; break;
        case PORT_B: GPIOx = GPIOB; break;
        case PORT_C: GPIOx = GPIOC; break;
        case PORT_D: GPIOx = GPIOD; break;
        case PORT_E: GPIOx = GPIOE; break;
        case PORT_H: GPIOx = GPIOH; break;
        default:
            // Invalid port. Exit function.
            return;
    }

    // Basic validation for pin number
    if (pin > 15)
    {
        return; // Invalid pin. Exit function.
    }

    // Toggle the bit in the Output Data Register (ODR)
    TOG_BIT(GPIOx->ODR, pin);
}

/***********************************************************************************************************************
* Function Name  : GPIO_Direction_get
* Description    : Gets the current direction (mode) of a specific GPIO pin.
* Arguments      : port - The GPIO port (tport enum)
*                : pin - The pin number (tpin enum)
* Return Value   : t_direction - The pin's mode (input, output, or analog). Alternate Function mode is treated as input.
***********************************************************************************************************************/
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();
    GPIO_TypeDef *GPIOx = NULL;
    uint32_t moder_val; // Variable to hold the 2 mode bits for the pin

    // Select the correct GPIO port
    switch(port)
    {
        case PORT_A: GPIOx = GPIOA; break;
        case PORT_B: GPIOx = GPIOB; break;
        case PORT_C: GPIOx = GPIOC; break;
        case PORT_D: GPIOx = GPIOD; break;
        case PORT_E: GPIOx = GPIOE; break;
        case PORT_H: GPIOx = GPIOH; break;
        default:
            // Invalid port. Return input as a safe default.
            return input;
    }

    // Basic validation for pin number
    if (pin > 15)
    {
        return input; // Invalid pin. Return input.
    }

    // Read the 2 bits corresponding to the pin's mode from the MODER register
    // Bits for pin 'n' are at positions (n * 2) and (n * 2) + 1
    moder_val = (GPIOx->MODER >> (pin * 2)) & 0x03; // Shift right and mask to get the 2 bits

    // Map the 2-bit value to the t_direction enum
    switch(moder_val)
    {
        case 0x00: return input;  // 00: Input mode
        case 0x01: return output; // 01: General purpose output mode
        case 0x10: return input;  // 10: Alternate function mode (treat as input per requirements)
        case 0x11: return analog; // 11: Analog mode
        default:
            // Should not happen with a 2-bit value, but return input as default
            return input;
    }
}

/***********************************************************************************************************************
* Function Name  : GPIO_Output_Init
* Description    : Initializes a GPIO pin as an output.
* Arguments      : port - The GPIO port (tport enum)
*                : pin - The pin number (tpin enum)
*                : value - The initial output state. Evaluation uses (value & (1 << pin)).
*                : usage - Not used in this implementation as per requirements.
*                : conn - Output connection type (push_pull or open_drain).
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();
    GPIO_TypeDef *GPIOx = NULL;

    // Select the correct GPIO port
    switch(port)
    {
        case PORT_A: GPIOx = GPIOA; break;
        case PORT_B: GPIOx = GPIOB; break;
        case PORT_C: GPIOx = GPIOC; break;
        case PORT_D: GPIOx = GPIOD; break;
        case PORT_E: GPIOx = GPIOE; break;
        case PORT_H: GPIOx = GPIOH; break;
        default:
            // Invalid port. Exit function.
            return;
    }

    // Basic validation for pin number
    if (pin > 15)
    {
        return; // Invalid pin. Exit function.
    }

    // Wait until the pin direction is reported as output.
    // WARNING: This is a blocking infinite loop if the direction never becomes output.
    // In a real system, a timeout or different synchronization mechanism might be required.
     while(GPIO_Direction_get(port, pin) != output)
    {
        // Loop body is intentionally empty.
        // Consider adding a small delay or yield mechanism if needed in a multitasking OS.
    }

    // Set the initial pin value using the ODR register
    // Requirement: Evaluate value using (value & (1 << pin))
    if ((value & (1U << pin))) // Check if the 'pin' bit is set within the 'value' byte
    {
        SET_BIT(GPIOx->ODR, pin);
    }
    else
    {
        CLR_BIT(GPIOx->ODR, pin);
    }

    // Configure the Output Type Register (OTYPER) for push-pull or open-drain
    // OTYPER bit 'n': 0=Push-pull, 1=Open-drain
    if (conn == open_drain)
    {
        SET_BIT(GPIOx->OTYPER, pin);
    }
    else // push_pull
    {
        CLR_BIT(GPIOx->OTYPER, pin);
    }

    // Configure the Pull-up/Pull-down Register (PUPDR) to disable pull-up/pull-down
    // PUPDR bits (n*2+1):(n*2): 00=No pull, 01=Pull-up, 10=Pull-down
    // To set to No Pull (00), clear both bits.
    CLR_BIT(GPIOx->PUPDR, (pin * 2));
    CLR_BIT(GPIOx->PUPDR, (pin * 2) + 1);

    // Configure the Mode Register (MODER) to set the pin as General Purpose Output
    // MODER bits (n*2+1):(n*2): 00=Input, 01=Output, 10=AF, 11=Analog
    // To set to Output (01): clear both bits, then set the LSB.
    CLR_BIT(GPIOx->MODER, (pin * 2));     // Clear bit (pin*2)
    CLR_BIT(GPIOx->MODER, (pin * 2) + 1); // Clear bit (pin*2 + 1)
    SET_BIT(GPIOx->MODER, (pin * 2));     // Set bit (pin*2) -> Resulting mode is 01
}

/***********************************************************************************************************************
* Function Name  : GPIO_Input_Init
* Description    : Initializes a GPIO pin as an input.
* Arguments      : port - The GPIO port (tport enum)
*                : pin - The pin number (tpin enum)
*                : usage - Not used in this implementation as per requirements.
*                : pull - Pull-up/pull-down configuration (no_pull, pull_up, or pull_down).
* Return Value   : None
***********************************************************************************************************************/
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();
    GPIO_TypeDef *GPIOx = NULL;

    // Select the correct GPIO port
    switch(port)
    {
        case PORT_A: GPIOx = GPIOA; break;
        case PORT_B: GPIOx = GPIOB; break;
        case PORT_C: GPIOx = GPIOC; break;
        case PORT_D: GPIOx = GPIOD; break;
        case PORT_E: GPIOx = GPIOE; break;
        case PORT_H: GPIOx = GPIOH; break;
        default:
            // Invalid port. Exit function.
            return;
    }

    // Basic validation for pin number
    if (pin > 15)
    {
        return; // Invalid pin. Exit function.
    }

    // Wait until the pin direction is reported as input.
    // WARNING: This is a blocking infinite loop if the direction never becomes input.
    // In a real system, a timeout or different synchronization mechanism might be required.
    while(GPIO_Direction_get(port, pin) != input)
    {
        // Loop body is intentionally empty.
        // Consider adding a small delay or yield mechanism
    }

    // Configure the Pull-up/Pull-down Register (PUPDR)
    // PUPDR bits (n*2+1):(n*2): 00=No pull, 01=Pull-up, 10=Pull-down
    // First, clear the 2 bits for the pin to set to No Pull (00)
    CLR_BIT(GPIOx->PUPDR, (pin * 2));
    CLR_BIT(GPIOx->PUPDR, (pin * 2) + 1);

    // Then, set bits based on the desired pull configuration
    switch(pull)
    {
        case pull_up:
            SET_BIT(GPIOx->PUPDR, (pin * 2)); // Set LSB to 1 -> 01 (Pull-up)
            break;
        case pull_down:
            SET_BIT(GPIOx->PUPDR, (pin * 2) + 1); // Set MSB to 1 -> 10 (Pull-down)
            break;
        case no_pull:
        default:
            // No pull-up/pull-down (00) is already configured by the initial clear
            break;
    }

    // Configure the Mode Register (MODER) to set the pin as Input
    // MODER bits (n*2+1):(n*2): 00=Input, 01=Output, 10=AF, 11=Analog
    // To set to Input (00): clear both bits.
    // These bits were already cleared as part of the PUPDR configuration approach above,
    // but explicitly clearing them here ensures the mode is set to input regardless
    // of previous PUPDR state handling or potential optimization by the compiler.
    CLR_BIT(GPIOx->MODER, (pin * 2));     // Clear bit (pin*2)
    CLR_BIT(GPIOx->MODER, (pin * 2) + 1); // Clear bit (pin*2 + 1)
}