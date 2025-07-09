/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : GPIO driver implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"

/*
 * Assume the following are defined elsewhere, likely in the header file
 * or included CMSIS headers, matching the structure pointers for GPIO
 * peripherals and the macro definitions.
 *
 * extern GPIO_TypeDef *GPIOA;
 * extern GPIO_TypeDef *GPIOB;
 * extern GPIO_TypeDef *GPIOC;
 * extern GPIO_TypeDef *GPIOD;
 * extern GPIO_TypeDef *GPIOE;
 * extern GPIO_TypeDef *GPIOH;
 *
 * Assume GPIO_TypeDef structure includes:
 * __IO uint32_t MODER;   // GPIO port mode register
 * __IO uint32_t OTYPER;  // GPIO port output type register
 * __IO uint32_t OSPEEDR; // GPIO port output speed register
 * __IO uint32_t PUPDR;   // GPIO port pull-up/pull-down register
 * __IO uint32_t IDR;     // GPIO port input data register
 * __IO uint32_t ODR;     // GPIO port output data register
 * __IO uint32_t BSRR;    // GPIO port bit set/reset register
 * __IO uint32_t LCKR;    // GPIO port configuration lock register
 * __IO uint32_t AFR[2];  // GPIO alternate function registers
 *
 * Assume WDT_Reset() is defined elsewhere.
 *
 * Assume macros SET_BIT, CLR_BIT, TOG_BIT, GET_BIT are defined elsewhere,
 * operating on a register pointer and a bit number (0-31).
 * Example: SET_BIT(reg_ptr, bit_num)
 *
 * Note: The instructions imply single-bit access to MODER and PUPDR using
 * bit index 'pin'. This is contrary to the STM32 architecture which uses
 * 2 bits per pin shifted by pin*2. This implementation strictly follows
 * the provided instructions to use the single-bit macros on the given pin index.
 */


/**
 * @brief Set the output value of a specific GPIO pin.
 *
 * This function sets or clears the output data register (ODR) bit
 * corresponding to the specified pin based on the given value.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number (e.g., pin_0, pin_15).
 * @param value The desired output value (non-zero for high, zero for low).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            if(value & (1 << pin)) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin);
            break;
        case port_B:
            if(value & (1 << pin)) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin);
            break;
        case port_C:
            if(value & (1 << pin)) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin);
            break;
        case port_D:
            if(value & (1 << pin)) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin);
            break;
        case port_E:
            if(value & (1 << pin)) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin);
            break;
        case port_H:
            if(value & (1 << pin)) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
            break;
        default:
            break;
    }
}


/**
 * @brief Get the input value of a specific GPIO pin.
 *
 * This function reads the state of the input data register (IDR) bit
 * corresponding to the specified pin.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number (e.g., pin_0, pin_15).
 * @return The input value of the pin (1 for high, 0 for low).
 */
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();
    tbyte ret_val = 0;
    switch(port)
    {
        case port_A:
            ret_val = GET_BIT(GPIOA->IDR, pin);
            break;
        case port_B:
            ret_val = GET_BIT(GPIOB->IDR, pin);
            break;
        case port_C:
            ret_val = GET_BIT(GPIOC->IDR, pin);
            break;
        case port_D:
            ret_val = GET_BIT(GPIOD->IDR, pin);
            break;
        case port_E:
            ret_val = GET_BIT(GPIOE->IDR, pin);
            break;
        case port_H:
            ret_val = GET_BIT(GPIOH->IDR, pin);
            break;
        default:
            break;
    }
    return ret_val;
}


/**
 * @brief Toggle the output value of a specific GPIO pin.
 *
 * This function toggles the state of the output data register (ODR) bit
 * corresponding to the specified pin.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number (e.g., pin_0, pin_15).
 */
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();
    switch(port)
    {
        case port_A:
            TOG_BIT(GPIOA->ODR, pin);
            break;
        case port_B:
            TOG_BIT(GPIOB->ODR, pin);
            break;
        case port_C:
            TOG_BIT(GPIOC->ODR, pin);
            break;
        case port_D:
            TOG_BIT(GPIOD->ODR, pin);
            break;
        case port_E:
            TOG_BIT(GPIOE->ODR, pin);
            break;
        case port_H:
            TOG_BIT(GPIOH->ODR, pin);
            break;
        default:
            break;
    }
}


/**
 * @brief Get the direction (mode) of a specific GPIO pin.
 *
 * This function reads the state of the mode register (MODER) bit
 * corresponding to the specified pin.
 *
 * Note: This implementation follows the instruction to use single-bit access
 * on MODER which deviates from the standard STM32 architecture.
 * It returns 'input' if the queried bit is 1, and 'output' if 0,
 * as per the initialization function instructions.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number (e.g., pin_0, pin_15).
 * @return The direction of the pin (input or output).
 */
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();
    t_direction ret_val = input; // Default to input
    switch(port)
    {
        case port_A:
            // Interpretation based on Init functions: SET_BIT=input, CLR_BIT=output
            ret_val = (GET_BIT(GPIOA->MODER, pin)) ? input : output;
            break;
        case port_B:
            ret_val = (GET_BIT(GPIOB->MODER, pin)) ? input : output;
            break;
        case port_C:
            ret_val = (GET_BIT(GPIOC->MODER, pin)) ? input : output;
            break;
        case port_D:
            ret_val = (GET_BIT(GPIOD->MODER, pin)) ? input : output;
            break;
        case port_E:
            ret_val = (GET_BIT(GPIOE->MODER, pin)) ? input : output;
            break;
        case port_H:
            ret_val = (GET_BIT(GPIOH->MODER, pin)) ? input : output;
            break;
        default:
            break;
    }
    return ret_val;
}


/**
 * @brief Initialize a specific GPIO pin as output.
 *
 * This function configures the mode, output type, pull-up/down,
 * and initial value for a GPIO pin intended for output.
 * It includes a loop to wait for the direction to be confirmed.
 *
 * Note: This implementation follows the instruction to use single-bit access
 * on MODER and PUPDR which deviates from the standard STM32 architecture.
 * It also uses the single-bit MODER setting logic specified (CLR_BIT for output).
 * The t_output_conn parameter is unused as per instructions.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number (e.g., pin_0, pin_15).
 * @param value The initial output value (non-zero for high, zero for low).
 * @param usage The pin usage (communication_usage or general_usage) influencing output type.
 * @param conn The output connection type (parameter present but unused).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();
    do
    {
        switch(port)
        {
            case port_A:
                CLR_BIT(GPIOA->PUPDR, pin); // Disable pull-up (following instruction)
                if(value & (1 << pin)) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin); // Set initial value
                if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin); // Set output type (SET=Open-drain, CLR=Push-pull assuming STM32 convention and usage mapping)
                CLR_BIT(GPIOA->MODER, pin); // Set as output (following instruction - contradicts STM32 MODER=01)
                break;
            case port_B:
                CLR_BIT(GPIOB->PUPDR, pin);
                if(value & (1 << pin)) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin);
                CLR_BIT(GPIOB->MODER, pin);
                break;
            case port_C:
                CLR_BIT(GPIOC->PUPDR, pin);
                if(value & (1 << pin)) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin);
                CLR_BIT(GPIOC->MODER, pin);
                break;
            case port_D:
                CLR_BIT(GPIOD->PUPDR, pin);
                if(value & (1 << pin)) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin);
                CLR_BIT(GPIOD->MODER, pin);
                break;
            case port_E:
                CLR_BIT(GPIOE->PUPDR, pin);
                if(value & (1 << pin)) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin);
                CLR_BIT(GPIOE->MODER, pin);
                break;
            case port_H:
                CLR_BIT(GPIOH->PUPDR, pin);
                if(value & (1 << pin)) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin);
                CLR_BIT(GPIOH->MODER, pin);
                break;
            default:
                break;
        }
    } while(GPIO_Direction_get(port, pin) != output);
}


/**
 * @brief Initialize a specific GPIO pin as input.
 *
 * This function configures the mode, output type (though less common for input,
 * relevant for open-drain), and pull-up/down for a GPIO pin intended for input.
 * It includes a loop to wait for the direction to be confirmed.
 *
 * Note: This implementation follows the instruction to use single-bit access
 * on MODER and PUPDR which deviates from the standard STM32 architecture.
 * It also uses the single-bit MODER setting logic specified (SET_BIT for input)
 * and only handles enabling pull-up based on the single PUPDR bit logic.
 * The t_pull parameter is unused as only enabling pull-up via SET_BIT is instructed.
 *
 * @param port The GPIO port (e.g., port_A, port_B).
 * @param pin The pin number (e.g., pin_0, pin_15).
 * @param usage The pin usage (communication_usage or general_usage) influencing output type.
 * @param pull The pull-up/down configuration (parameter present but only pull-up is handled).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)
{
    WDT_Reset();
    do
    {
        switch(port)
        {
            case port_A:
                SET_BIT(GPIOA->PUPDR, pin); // Enable pull-up (following instruction - only pull-up or no-pull based on single bit)
                // Output type relevant for input in open-drain configurations (e.g., I2C)
                if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin); // SET=Open-drain, CLR=Push-pull assumed
                SET_BIT(GPIOA->MODER, pin); // Set as input (following instruction - contradicts STM32 MODER=00)
                break;
            case port_B:
                SET_BIT(GPIOB->PUPDR, pin);
                if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin);
                SET_BIT(GPIOB->MODER, pin);
                break;
            case port_C:
                SET_BIT(GPIOC->PUPDR, pin);
                if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin);
                SET_BIT(GPIOC->MODER, pin);
                break;
            case port_D:
                SET_BIT(GPIOD->PUPDR, pin);
                if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin);
                SET_BIT(GPIOD->MODER, pin);
                break;
            case port_E:
                SET_BIT(GPIOE->PUPDR, pin);
                if(usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin);
                SET_BIT(GPIOE->MODER, pin);
                break;
            case port_H:
                SET_BIT(GPIOH->PUPDR, pin);
                if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin);
                SET_BIT(GPIOH->MODER, pin);
                break;
            default:
                break;
        }
    } while(GPIO_Direction_get(port, pin) != input);
}