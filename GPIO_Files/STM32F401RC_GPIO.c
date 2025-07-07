/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : GPIO driver implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"
// This implementation assumes that "STM32F401RC_GPIO.h" or headers included by it
// provide the necessary CMSIS structures and base addresses (like GPIO_TypeDef, GPIOA, GPIOB, etc.),
// the required enums (tport, tpin, tbyte, t_direction, t_usage, t_pull),
// and the macros SET_BIT, CLR_BIT, TOG_BIT, GET_BIT for single bit operations,
// as well as the declaration for WDT_Reset().

void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();
    GPIO_TypeDef* GPIOx;

    switch(port)
    {
        case port_A: GPIOx = GPIOA; break;
        case port_B: GPIOx = GPIOB; break;
        case port_C: GPIOx = GPIOC; break;
        case port_D: GPIOx = GPIOD; break;
        case port_E: GPIOx = GPIOE; break;
        case port_H: GPIOx = GPIOH; break;
        default: return; // Invalid port, do nothing
    }

    // ODR (Output Data Register) is a 1-bit register per pin for setting value
    if (value & (1 << pin))
    {
        SET_BIT(GPIOx->ODR, pin);
    }
    else
    {
        CLR_BIT(GPIOx->ODR, pin);
    }
}

tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();
    GPIO_TypeDef* GPIOx;
    tbyte value = 0; // generate a vaiable inside the function and return it.

    switch(port)
    {
        case port_A: GPIOx = GPIOA; break;
        case port_B: GPIOx = GPIOB; break;
        case port_C: GPIOx = GPIOC; break;
        case port_D: GPIOx = GPIOD; break;
        case port_E: GPIOx = GPIOE; break;
        case port_H: GPIOx = GPIOH; break;
        default: return 0; // Invalid port, return default value
    }

    // IDR (Input Data Register) is a 1-bit register per pin for getting value
    if (GET_BIT(GPIOx->IDR, pin))
    {
        value = 1;
    }
    else
    {
        value = 0;
    }

    return value;
}

void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();
    GPIO_TypeDef* GPIOx;

    switch(port)
    {
        case port_A: GPIOx = GPIOA; break;
        case port_B: GPIOx = GPIOB; break;
        case port_C: GPIOx = GPIOC; break;
        case port_D: GPIOx = GPIOD; break;
        case port_E: GPIOx = GPIOE; break;
        case port_H: GPIOx = GPIOH; break;
        default: return; // Invalid port, do nothing
    }

    // ODR (Output Data Register) is a 1-bit register per pin for toggling value
    TOG_BIT(GPIOx->ODR, pin);
}

t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();
    GPIO_TypeDef* GPIOx;
    uint32_t moder_val = 0;
    t_direction direction = (t_direction)0; // generate a vaiable inside the function and return it.

    switch(port)
    {
        case port_A: GPIOx = GPIOA; break;
        case port_B: GPIOx = GPIOB; break;
        case port_C: GPIOx = GPIOC; break;
        case port_D: GPIOx = GPIOD; break;
        case port_E: GPIOx = GPIOE; break;
        case port_H: GPIOx = GPIOH; break;
        default: return (t_direction)0; // Invalid port, return default/invalid direction
    }

    // MODER (Mode Register) uses 2 bits per pin. Read the 2-bit field for the pin.
    moder_val = (GPIOx->MODER >> (2 * pin)) & 0x03UL;

    // Assuming t_direction enum values correspond to the 2-bit MODER field values:
    // 00 (Input) -> input
    // 01 (Output) -> output
    // 10 (Alternate function) -> alternate_function
    // 11 (Analog) -> analog
    direction = (t_direction)moder_val;

    return direction;
}

void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();
    GPIO_TypeDef* GPIOx;

    switch(port)
    {
        case port_A: GPIOx = GPIOA; break;
        case port_B: GPIOx = GPIOB; break;
        case port_C: GPIOx = GPIOC; break;
        case port_D: GPIOx = GPIOD; break;
        case port_E: GPIOx = GPIOE; break;
        case port_H: GPIOx = GPIOH; break;
        default: return; // Invalid port, do nothing
    }

    // --- Configuration Steps ---

    // 1. Disable pull-up/down (set PUPDR bits to 00 - No pull-up, no pull-down)
    // PUPDR (Pull-up/pull-down Register) uses 2 bits per pin. Clear the 2 bits at position 2*pin.
    GPIOx->PUPDR &= ~(0x03UL << (2 * pin));

    // 2. Set initial output value using ODR (1 bit)
    if (value & (1 << pin))
    {
        SET_BIT(GPIOx->ODR, pin);
    }
    else
    {
        CLR_BIT(GPIOx->ODR, pin);
    }

    // 3. Set output type using OTYPER (1 bit) based on usage mapping requested
    // OTYPER (Output Type Register) uses 1 bit per pin.
    // If usage is communication_usage, set OTYPER bit to 1 (Open-drain).
    // Otherwise (non_communication_usage or others), clear OTYPER bit to 0 (Push-pull).
    if (usage == communication_usage)
    {
        SET_BIT(GPIOx->OTYPER, pin); // Set bit for Open-drain
    }
    else
    {
        CLR_BIT(GPIOx->OTYPER, pin); // Clear bit for Push-pull
    }

    // Note: OSPEEDR (Output Speed Register) configuration is not requested.

    // 4. Set mode as output (set MODER bits to 01 - General purpose output mode)
    // MODER (Mode Register) uses 2 bits per pin. Clear the 2 bits, then set the lower bit (01).
    GPIOx->MODER &= ~(0x03UL << (2 * pin)); // Clear current mode bits
    GPIOx->MODER |= (0x01UL << (2 * pin));  // Set mode to 01 (General purpose output mode)


    do
    {
        WDT_Reset(); // Keep watchdog happy inside the loop
    } while(GPIO_Direction_get(port, pin) != output);
}

void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();
    GPIO_TypeDef* GPIOx;

    switch(port)
    {
        case port_A: GPIOx = GPIOA; break;
        case port_B: GPIOx = GPIOB; break;
        case port_C: GPIOx = GPIOC; break;
        case port_D: GPIOx = GPIOD; break;
        case port_E: GPIOx = GPIOE; break;
        case port_H: GPIOx = GPIOH; break;
        default: return; // Invalid port, do nothing
    }

    // --- Configuration Steps ---

    // 1. Set pull-up/down based on 'pull' parameter (set PUPDR bits 00, 01, or 10)
    // PUPDR (Pull-up/pull-down Register) uses 2 bits per pin.
    // Clear the 2 bits at position 2*pin, then set based on the value of 'pull'.
    // Assuming t_pull enum maps directly to the 2-bit PUPDR field values:
    // pull_no = 00 (0)
    // pull_up = 01 (1)
    // pull_down = 10 (2)
    GPIOx->PUPDR &= ~(0x03UL << (2 * pin));      // Clear current pull config bits
    GPIOx->PUPDR |= ((uint32_t)pull << (2 * pin)); // Set pull config based on t_pull value


    // 2. Set output type using OTYPER (1 bit) based on usage mapping requested
    // OTYPER (Output Type Register) uses 1 bit per pin.
    // If usage is communication_usage, set OTYPER bit to 1 (Open-drain).
    // Otherwise, clear OTYPER bit to 0 (Push-pull).
    if (usage == communication_usage)
    {
        SET_BIT(GPIOx->OTYPER, pin); // Set bit for Open-drain
    }
    else
    {
        CLR_BIT(GPIOx->OTYPER, pin); // Clear bit for Push-pull
    }

    // Note: OSPEEDR (Output Speed Register) configuration is not requested.

    // 3. Set mode as input (set MODER bits to 00 - General purpose input mode)
    // MODER (Mode Register) uses 2 bits per pin. Clear the 2 bits at position 2*pin (which sets it to 00).
    GPIOx->MODER &= ~(0x03UL << (2 * pin)); // Clear current mode bits (sets mode to 00 - Input)


    do
    {
        WDT_Reset(); // Keep watchdog happy inside the loop
    } while(GPIO_Direction_get(port, pin) != input);
}

// End of file