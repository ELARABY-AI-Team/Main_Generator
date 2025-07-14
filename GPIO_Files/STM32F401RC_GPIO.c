/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-14
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"

/* Assumed include for hardware register definitions (e.g., GPIOA, GPIOB, etc.) */
/* Assumed include for WDT_Reset() */
/* Assumed include for SET_BIT, CLR_BIT, TOG_BIT, GET_BIT macros */
/* Assumed include for standard types (tport, tpin, tbyte, t_direction, t_usage, t_output_conn, t_pull) */


void GPIO_Value_Set(tport port, tpin pin, tbyte value)

    WDT_Reset(); /* PDF Reference */
    switch(port)
    
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
    


tbyte GPIO_Value_Get(tport port, tpin pin)

    WDT_Reset(); /* PDF Reference */
    tbyte ret_val = 0; /* PDF Reference */
    switch(port)
    
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
    
    return ret_val; /* PDF Reference */


void GPIO_Value_Tog(tport port, tpin pin)

    WDT_Reset(); /* PDF Reference */
    switch(port)
    
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
    


t_direction GPIO_Direction_get(tport port, tpin pin)

    WDT_Reset(); /* PDF Reference */
    t_direction ret_val = 0; /* PDF Reference */ /* Assumed GPIO config - please verify: t_direction enum mapping (e.g., 0 for input) */
    uint32_t moder_bits; /* PDF Reference */
    switch(port)
    
        case port_A:
            moder_bits = (GET_BIT(GPIOA->MODER, pin * 2 + 1) << 1) | GET_BIT(GPIOA->MODER, pin * 2); /* PDF Reference */
            if (moder_bits == 0x00) ret_val = input; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 00 is Input */
            else if (moder_bits == 0x01) ret_val = output; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 01 is Output */
            break;
        case port_B:
            moder_bits = (GET_BIT(GPIOB->MODER, pin * 2 + 1) << 1) | GET_BIT(GPIOB->MODER, pin * 2); /* PDF Reference */
            if (moder_bits == 0x00) ret_val = input; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 00 is Input */
            else if (moder_bits == 0x01) ret_val = output; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 01 is Output */
            break;
        case port_C:
            moder_bits = (GET_BIT(GPIOC->MODER, pin * 2 + 1) << 1) | GET_BIT(GPIOC->MODER, pin * 2); /* PDF Reference */
            if (moder_bits == 0x00) ret_val = input; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 00 is Input */
            else if (moder_bits == 0x01) ret_val = output; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 01 is Output */
            break;
        case port_D:
            moder_bits = (GET_BIT(GPIOD->MODER, pin * 2 + 1) << 1) | GET_BIT(GPIOD->MODER, pin * 2); /* PDF Reference */
            if (moder_bits == 0x00) ret_val = input; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 00 is Input */
            else if (moder_bits == 0x01) ret_val = output; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 01 is Output */
            break;
        case port_E:
            moder_bits = (GET_BIT(GPIOE->MODER, pin * 2 + 1) << 1) | GET_BIT(GPIOE->MODER, pin * 2); /* PDF Reference */
            if (moder_bits == 0x00) ret_val = input; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 00 is Input */
            else if (moder_bits == 0x01) ret_val = output; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 01 is Output */
            break;
        case port_H:
            moder_bits = (GET_BIT(GPIOH->MODER, pin * 2 + 1) << 1) | GET_BIT(GPIOH->MODER, pin * 2); /* PDF Reference */
            if (moder_bits == 0x00) ret_val = input; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 00 is Input */
            else if (moder_bits == 0x01) ret_val = output; /* PDF Reference */ /* Assumed GPIO config - please verify: MODER 01 is Output */
            break;
        default:
            break;
    
    return ret_val; /* PDF Reference */


void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)

    WDT_Reset(); /* PDF Reference */
    do
    
        switch(port)
        
            case port_A:
                CLR_BIT(GPIOA->PUPDR, pin*2 + 1); CLR_BIT(GPIOA->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */
                if(value) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin); /* PDF Reference */
                if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER (Open-Drain vs Push-Pull) */
                CLR_BIT(GPIOA->MODER, pin*2 + 1); SET_BIT(GPIOA->MODER, pin*2); /* PDF Reference */ /* MODER = 01 (General purpose output mode) */
                break;
            case port_B:
                CLR_BIT(GPIOB->PUPDR, pin*2 + 1); CLR_BIT(GPIOB->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */
                if(value) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin); /* PDF Reference */
                if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER (Open-Drain vs Push-Pull) */
                CLR_BIT(GPIOB->MODER, pin*2 + 1); SET_BIT(GPIOB->MODER, pin*2); /* PDF Reference */ /* MODER = 01 (General purpose output mode) */
                break;
            case port_C:
                CLR_BIT(GPIOC->PUPDR, pin*2 + 1); CLR_BIT(GPIOC->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */
                if(value) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin); /* PDF Reference */
                if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER (Open-Drain vs Push-Pull) */
                CLR_BIT(GPIOC->MODER, pin*2 + 1); SET_BIT(GPIOC->MODER, pin*2); /* PDF Reference */ /* MODER = 01 (General purpose output mode) */
                break;
            case port_D:
                CLR_BIT(GPIOD->PUPDR, pin*2 + 1); CLR_BIT(GPIOD->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */
                if(value) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin); /* PDF Reference */
                if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER (Open-Drain vs Push-Pull) */
                CLR_BIT(GPIOD->MODER, pin*2 + 1); SET_BIT(GPIOD->MODER, pin*2); /* PDF Reference */ /* MODER = 01 (General purpose output mode) */
                break;
            case port_E:
                CLR_BIT(GPIOE->PUPDR, pin*2 + 1); CLR_BIT(GPIOE->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */
                if(value) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin); /* PDF Reference */
                if(usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER (Open-Drain vs Push-Pull) */
                CLR_BIT(GPIOE->MODER, pin*2 + 1); SET_BIT(GPIOE->MODER, pin*2); /* PDF Reference */ /* MODER = 01 (General purpose output mode) */
                break;
            case port_H:
                CLR_BIT(GPIOH->PUPDR, pin*2 + 1); CLR_BIT(GPIOH->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */
                if(value) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin); /* PDF Reference */
                if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER (Open-Drain vs Push-Pull) */
                CLR_BIT(GPIOH->MODER, pin*2 + 1); SET_BIT(GPIOH->MODER, pin*2); /* PDF Reference */ /* MODER = 01 (General purpose output mode) */
                break;
            default:
                break;
        
    while(GPIO_Direction_get(port, pin) != output); /* PDF Reference */ /* Assumed GPIO config - please verify: t_direction enum mapping (e.g., 1 for output) */


void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)

    WDT_Reset(); /* PDF Reference */
    do
    
        switch(port)
        
            case port_A:
                if(pull == pull_up) 
                
                    CLR_BIT(GPIOA->PUPDR, pin*2 + 1); SET_BIT(GPIOA->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 01 (Pull-up) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else if(pull == pull_down)
                
                    SET_BIT(GPIOA->PUPDR, pin*2 + 1); CLR_BIT(GPIOA->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 10 (Pull-down) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else 
                
                    CLR_BIT(GPIOA->PUPDR, pin*2 + 1); CLR_BIT(GPIOA->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                if(usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER even for input */
                CLR_BIT(GPIOA->MODER, pin*2 + 1); CLR_BIT(GPIOA->MODER, pin*2); /* PDF Reference */ /* MODER = 00 (Input) */
                break;
            case port_B:
                if(pull == pull_up)
                
                    CLR_BIT(GPIOB->PUPDR, pin*2 + 1); SET_BIT(GPIOB->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 01 (Pull-up) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else if(pull == pull_down)
                
                    SET_BIT(GPIOB->PUPDR, pin*2 + 1); CLR_BIT(GPIOB->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 10 (Pull-down) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else
                
                    CLR_BIT(GPIOB->PUPDR, pin*2 + 1); CLR_BIT(GPIOB->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                if(usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER even for input */
                CLR_BIT(GPIOB->MODER, pin*2 + 1); CLR_BIT(GPIOB->MODER, pin*2); /* PDF Reference */ /* MODER = 00 (Input) */
                break;
            case port_C:
                if(pull == pull_up)
                
                    CLR_BIT(GPIOC->PUPDR, pin*2 + 1); SET_BIT(GPIOC->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 01 (Pull-up) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else if(pull == pull_down)
                
                    SET_BIT(GPIOC->PUPDR, pin*2 + 1); CLR_BIT(GPIOC->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 10 (Pull-down) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else
                
                    CLR_BIT(GPIOC->PUPDR, pin*2 + 1); CLR_BIT(GPIOC->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                if(usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER even for input */
                CLR_BIT(GPIOC->MODER, pin*2 + 1); CLR_BIT(GPIOC->MODER, pin*2); /* PDF Reference */ /* MODER = 00 (Input) */
                break;
            case port_D:
                if(pull == pull_up)
                
                    CLR_BIT(GPIOD->PUPDR, pin*2 + 1); SET_BIT(GPIOD->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 01 (Pull-up) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else if(pull == pull_down)
                
                    SET_BIT(GPIOD->PUPDR, pin*2 + 1); CLR_BIT(GPIOD->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 10 (Pull-down) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else
                
                    CLR_BIT(GPIOD->PUPDR, pin*2 + 1); CLR_BIT(GPIOD->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                if(usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER even for input */
                CLR_BIT(GPIOD->MODER, pin*2 + 1); CLR_BIT(GPIOD->MODER, pin*2); /* PDF Reference */ /* MODER = 00 (Input) */
                break;
            case port_E:
                if(pull == pull_up)
                
                    CLR_BIT(GPIOE->PUPDR, pin*2 + 1); SET_BIT(GPIOE->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 01 (Pull-up) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else if(pull == pull_down)
                
                    SET_BIT(GPIOE->PUPDR, pin*2 + 1); CLR_BIT(GPIOE->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 10 (Pull-down) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else
                
                    CLR_BIT(GPIOE->PUPDR, pin*2 + 1); CLR_BIT(GPIOE->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                if(usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER even for input */
                CLR_BIT(GPIOE->MODER, pin*2 + 1); CLR_BIT(GPIOE->MODER, pin*2); /* PDF Reference */ /* MODER = 00 (Input) */
                break;
            case port_H:
                if(pull == pull_up)
                
                    CLR_BIT(GPIOH->PUPDR, pin*2 + 1); SET_BIT(GPIOH->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 01 (Pull-up) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else if(pull == pull_down)
                
                    SET_BIT(GPIOH->PUPDR, pin*2 + 1); CLR_BIT(GPIOH->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 10 (Pull-down) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                else
                
                    CLR_BIT(GPIOH->PUPDR, pin*2 + 1); CLR_BIT(GPIOH->PUPDR, pin*2); /* PDF Reference */ /* PUPDR = 00 (No pull) */ /* Assumed GPIO config - please verify: mapping pull enum to PUPDR bits */
                
                if(usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin); /* PDF Reference */ /* Assumed GPIO config - please verify: mapping usage to OTYPER even for input */
                CLR_BIT(GPIOH->MODER, pin*2 + 1); CLR_BIT(GPIOH->MODER, pin*2); /* PDF Reference */ /* MODER = 00 (Input) */
                break;
            default:
                break;
        
    while(GPIO_Direction_get(port, pin) != input); /* PDF Reference */ /* Assumed GPIO config - please verify: t_direction enum mapping (e.g., 0 for input) */