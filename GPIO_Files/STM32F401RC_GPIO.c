/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : STM32F401RC GPIO driver implementation.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"

// Assume necessary type definitions (tport, tpin, tbyte, t_direction, t_usage, t_pull)
// and macros (SET_BIT, CLR_BIT, TOG_BIT, GET_BIT)
// and WDT_Reset() and GPIO Peripheral base addresses (GPIOA, etc.) are in STM32F401RC_GPIO.h or included by it.
// Assume t_direction: input maps to MODER 00, output maps to MODER 01 for comparison in while loops.
// Assume t_pull: pull_none maps to PUPDR 00, pull_up maps to PUPDR 01, pull_down maps to PUPDR 10 for mapping.
// Assume t_usage: general_usage maps to OTYPER 0 (Push-Pull), communication_usage maps to OTYPER 1 (Open-Drain) for mapping.


void GPIO_Value_Set(tport port, tpin pin, tbyte value)

    WDT_Reset();
    switch(port)
    
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
        case port_H:
            if(value & (1 << pin)) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
            break;
        default:
            break;
    


tbyte GPIO_Value_Get(tport port, tpin pin)

    WDT_Reset();
    tbyte ret_val = 0; // Default to 0

    switch(port)
    
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
        case port_H:
            ret_val = GET_BIT(GPIOH->IDR, pin);
            break;
        default:
            // ret_val remains 0
            break;
    
    return ret_val;


void GPIO_Value_Tog(tport port, tpin pin)

    WDT_Reset();
    switch(port)
    
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
        case port_H:
            TOG_BIT(GPIOH->ODR, pin);
            break;
        default:
            break;
    


t_direction GPIO_Direction_get(tport port, tpin pin)

    WDT_Reset();
    // Default to a value that is neither 'input' (0) nor 'output' (1) based on MODER bits 00=Input, 01=Output
    t_direction ret_val = (t_direction)0xFF;

    switch(port)
    
        case port_A:
            // Extract the 2-bit mode field for the pin (bits 2*pin and 2*pin+1)
            ret_val = (t_direction)((GPIOA->MODER >> (2 * pin)) & 0x03);
            break;
        case port_B:
            ret_val = (t_direction)((GPIOB->MODER >> (2 * pin)) & 0x03);
            break;
        case port_C:
            ret_val = (t_direction)((GPIOC->MODER >> (2 * pin)) & 0x03);
            break;
        case port_D:
            ret_val = (t_direction)((GPIOD->MODER >> (2 * pin)) & 0x03);
            break;
        case port_H:
            ret_val = (t_direction)((GPIOH->MODER >> (2 * pin)) & 0x03);
            break;
        default:
            // ret_val remains 0xFF
            break;
    
    return ret_val;


void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)

    WDT_Reset();
    do
    
        switch(port)
        
            case port_A:
                // Disable pull-up/down (00)
                CLR_BIT(GPIOA->PUPDR, 2 * pin);
                CLR_BIT(GPIOA->PUPDR, 2 * pin + 1);
                // Set initial value
                if(value & (1 << pin)) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin);
                // Configure Output Type (Push-Pull:0, Open-Drain:1 in OTYPER) based on usage
                if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin);
                // Set mode to Output (01)
                CLR_BIT(GPIOA->MODER, 2 * pin + 1); // Set bit 2*pin+1 to 0
                SET_BIT(GPIOA->MODER, 2 * pin);     // Set bit 2*pin to 1
                break;
            case port_B:
                CLR_BIT(GPIOB->PUPDR, 2 * pin);
                CLR_BIT(GPIOB->PUPDR, 2 * pin + 1);
                if(value & (1 << pin)) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin);
                CLR_BIT(GPIOB->MODER, 2 * pin + 1);
                SET_BIT(GPIOB->MODER, 2 * pin);
                break;
            case port_C:
                CLR_BIT(GPIOC->PUPDR, 2 * pin);
                CLR_BIT(GPIOC->PUPDR, 2 * pin + 1);
                if(value & (1 << pin)) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin);
                CLR_BIT(GPIOC->MODER, 2 * pin + 1);
                SET_BIT(GPIOC->MODER, 2 * pin);
                break;
            case port_D:
                CLR_BIT(GPIOD->PUPDR, 2 * pin);
                CLR_BIT(GPIOD->PUPDR, 2 * pin + 1);
                if(value & (1 << pin)) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin);
                if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin);
                CLR_BIT(GPIOD->MODER, 2 * pin + 1);
                SET_BIT(GPIOD->MODER, 2 * pin);
                break;
            case port_H:
                 CLR_BIT(GPIOH->PUPDR, 2 * pin);
                 CLR_BIT(GPIOH->PUPDR, 2 * pin + 1);
                 if(value & (1 << pin)) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
                 if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin);
                 CLR_BIT(GPIOH->MODER, 2 * pin + 1);
                 SET_BIT(GPIOH->MODER, 2 * pin);
                 break;
            default:
                break;
        
    } while(GPIO_Direction_get(port, pin) != output);


void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)

    WDT_Reset();
    do
    
        switch(port)
        
            case port_A:
                // Configure Pull-up/down (PUPDR)
                CLR_BIT(GPIOA->PUPDR, 2 * pin);     // Clear bits first
                CLR_BIT(GPIOA->PUPDR, 2 * pin + 1);
                if(pull == pull_up) SET_BIT(GPIOA->PUPDR, 2 * pin);       // Set 01 for Pull-up
                else if(pull == pull_down) SET_BIT(GPIOA->PUPDR, 2 * pin + 1); // Set 10 for Pull-down
                // Configure Output Type (OTYPER) - As per instruction, mapping placeholder POMx to OTYPER
                // This configures Open-Drain/Push-Pull even for input? Adhering strictly to the rule.
                if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin);
                // Set mode to Input (00)
                CLR_BIT(GPIOA->MODER, 2 * pin);     // Set bit 2*pin to 0
                CLR_BIT(GPIOA->MODER, 2 * pin + 1); // Set bit 2*pin+1 to 0
                break;
            case port_B:
                CLR_BIT(GPIOB->PUPDR, 2 * pin);
                CLR_BIT(GPIOB->PUPDR, 2 * pin + 1);
                if(pull == pull_up) SET_BIT(GPIOB->PUPDR, 2 * pin);
                else if(pull == pull_down) SET_BIT(GPIOB->PUPDR, 2 * pin + 1);
                if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin);
                CLR_BIT(GPIOB->MODER, 2 * pin);
                CLR_BIT(GPIOB->MODER, 2 * pin + 1);
                break;
            case port_C:
                CLR_BIT(GPIOC->PUPDR, 2 * pin);
                CLR_BIT(GPIOC->PUPDR, 2 * pin + 1);
                if(pull == pull_up) SET_BIT(GPIOC->PUPDR, 2 * pin);
                else if(pull == pull_down) SET_BIT(GPIOC->PUPDR, 2 * pin + 1);
                if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin);
                CLR_BIT(GPIOC->MODER, 2 * pin);
                CLR_BIT(GPIOC->MODER, 2 * pin + 1);
                break;
            case port_D:
                CLR_BIT(GPIOD->PUPDR, 2 * pin);
                CLR_BIT(GPIOD->PUPDR, 2 * pin + 1);
                if(pull == pull_up) SET_BIT(GPIOD->PUPDR, 2 * pin);
                else if(pull == pull_down) SET_BIT(GPIOD->PUPDR, 2 * pin + 1);
                if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin);
                CLR_BIT(GPIOD->MODER, 2 * pin);
                CLR_BIT(GPIOD->MODER, 2 * pin + 1);
                break;
            case port_H:
                CLR_BIT(GPIOH->PUPDR, 2 * pin);
                CLR_BIT(GPIOH->PUPDR, 2 * pin + 1);
                if(pull == pull_up) SET_BIT(GPIOH->PUPDR, 2 * pin);
                else if(pull == pull_down) SET_BIT(GPIOH->PUPDR, 2 * pin + 1);
                if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin);
                CLR_BIT(GPIOH->MODER, 2 * pin);
                CLR_BIT(GPIOH->MODER, 2 * pin + 1);
                break;
            default:
                break;
        
    } while(GPIO_Direction_get(port, pin) != input);