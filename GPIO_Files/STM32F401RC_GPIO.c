/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"
#include "stm32f4xx.h" // Required for GPIO peripheral register access (GPIOA, etc.)

// Assume necessary definitions for tport, tpin, tbyte, t_direction, t_usage, t_output_conn, t_pull
// Assume macros SET_BIT, CLR_BIT, TOG_BIT, GET_BIT are defined elsewhere
// Assume WDT_Reset() is defined elsewhere

/*
 * Note: Instructions regarding 't_usage' and registers like 'POMx' or single-bit
 * configuration via 'PMx' or 'PUx' are inconsistent with the provided PDF
 * which describes GPIO configuration using MODER (2 bits/pin), OTYPER (1 bit/pin),
 * and PUPDR (2 bits/pin) registers. This implementation will use the register
 * names and bit configurations found in the PDF (RM0368) for core GPIO setup
 * (Mode, Type, Pull-up/down) and will ignore the conflicting 't_usage' related
 * instructions as they cannot be mapped to the provided PDF register details.
 *
 * PDF Reference: General-purpose I/Os (GPIO) section (RM0368)
 * GPIOx_MODER: 2 bits per pin for mode (00:Input, 01:Output, 10:AF, 11:Analog)
 * GPIOx_OTYPER: 1 bit per pin for output type (0:Push-pull, 1:Open-drain)
 * GPIOx_PUPDR: 2 bits per pin for pull-up/down (00:No, 01:Pull-up, 10:Pull-down)
 * GPIOx_IDR: Input Data Register (read-only)
 * GPIOx_ODR: Output Data Register (read/write)
 * GPIOx_BSRR: Bit Set/Reset Register (write-only for atomic set/reset)
 */

void GPIO_Value_Set(tport port, tpin pin, tbyte value)

    WDT_Reset();
    switch(port)
    
        case port_A:
            /* PDF Reference: GPIOA->ODR */
            if(value & (1 << pin)) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin);
            break;
        case port_B:
            /* PDF Reference: GPIOB->ODR */
            if(value & (1 << pin)) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin);
            break;
        case port_C:
            /* PDF Reference: GPIOC->ODR */
            if(value & (1 << pin)) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin);
            break;
        case port_D:
            /* PDF Reference: GPIOD->ODR */
            if(value & (1 << pin)) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin);
            break;
        case port_E:
            /* PDF Reference: GPIOE->ODR */
            if(value & (1 << pin)) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin);
            break;
        case port_H:
            /* PDF Reference: GPIOH->ODR */
            if(value & (1 << pin)) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
            break;
        default:
            break;
    


tbyte GPIO_Value_Get(tport port, tpin pin)

    WDT_Reset();
    tbyte ret_val = 0;
    switch(port)
    
        case port_A:
            /* PDF Reference: GPIOA->IDR */
            ret_val = GET_BIT(GPIOA->IDR, pin);
            break;
        case port_B:
            /* PDF Reference: GPIOB->IDR */
            ret_val = GET_BIT(GPIOB->IDR, pin);
            break;
        case port_C:
            /* PDF Reference: GPIOC->IDR */
            ret_val = GET_BIT(GPIOC->IDR, pin);
            break;
        case port_D:
            /* PDF Reference: GPIOD->IDR */
            ret_val = GET_BIT(GPIOD->IDR, pin);
            break;
        case port_E:
            /* PDF Reference: GPIOE->IDR */
            ret_val = GET_BIT(GPIOE->IDR, pin);
            break;
        case port_H:
            /* PDF Reference: GPIOH->IDR */
            ret_val = GET_BIT(GPIOH->IDR, pin);
            break;
        default:
            break;
    
    return ret_val;


void GPIO_Value_Tog(tport port, tpin pin)

    WDT_Reset();
    switch(port)
    
        case port_A:
            /* PDF Reference: GPIOA->ODR */ // Toggling ODR bit directly is not atomic, BSRR is preferred for atomic set/reset
            TOG_BIT(GPIOA->ODR, pin);
            break;
        case port_B:
            /* PDF Reference: GPIOB->ODR */ // Toggling ODR bit directly is not atomic, BSRR is preferred for atomic set/reset
            TOG_BIT(GPIOB->ODR, pin);
            break;
        case port_C:
            /* PDF Reference: GPIOC->ODR */ // Toggling ODR bit directly is not atomic, BSRR is preferred for atomic set/reset
            TOG_BIT(GPIOC->ODR, pin);
            break;
        case port_D:
            /* PDF Reference: GPIOD->ODR */ // Toggling ODR bit directly is not atomic, BSRR is preferred for atomic set/reset
            TOG_BIT(GPIOD->ODR, pin);
            break;
        case port_E:
            /* PDF Reference: GPIOE->ODR */ // Toggling ODR bit directly is not atomic, BSRR is preferred for atomic set/reset
            TOG_BIT(GPIOE->ODR, pin);
            break;
        case port_H:
            /* PDF Reference: GPIOH->ODR */ // Toggling ODR bit directly is not atomic, BSRR is preferred for atomic set/reset
            TOG_BIT(GPIOH->ODR, pin);
            break;
        default:
            break;
    


t_direction GPIO_Direction_get(tport port, tpin pin)

    WDT_Reset();
    t_direction ret_val = input; /* Default/Error state */
    uint32_t mode;
    switch(port)
    
        case port_A:
            /* PDF Reference: GPIOA->MODER (bits 2y:2y+1) */
            mode = (GPIOA->MODER >> (pin * 2)) & 0x03;
            if (mode == 0x00) ret_val = input; /* PDF Reference: 00: Input */
            else if (mode == 0x01) ret_val = output; /* PDF Reference: 01: General purpose output mode */
            /* Ignore AF (10) and Analog (11) modes as per t_direction enum implied by usage */
            break;
        case port_B:
            /* PDF Reference: GPIOB->MODER (bits 2y:2y+1) */
            mode = (GPIOB->MODER >> (pin * 2)) & 0x03;
            if (mode == 0x00) ret_val = input; /* PDF Reference: 00: Input */
            else if (mode == 0x01) ret_val = output; /* PDF Reference: 01: General purpose output mode */
            /* Ignore AF (10) and Analog (11) modes */
            break;
        case port_C:
            /* PDF Reference: GPIOC->MODER (bits 2y:2y+1) */
            mode = (GPIOC->MODER >> (pin * 2)) & 0x03;
            if (mode == 0x00) ret_val = input; /* PDF Reference: 00: Input */
            else if (mode == 0x01) ret_val = output; /* PDF Reference: 01: General purpose output mode */
            /* Ignore AF (10) and Analog (11) modes */
            break;
        case port_D:
            /* PDF Reference: GPIOD->MODER (bits 2y:2y+1) */
            mode = (GPIOD->MODER >> (pin * 2)) & 0x03;
            if (mode == 0x00) ret_val = input; /* PDF Reference: 00: Input */
            else if (mode == 0x01) ret_val = output; /* PDF Reference: 01: General purpose output mode */
            /* Ignore AF (10) and Analog (11) modes */
            break;
        case port_E:
            /* PDF Reference: GPIOE->MODER (bits 2y:2y+1) */
            mode = (GPIOE->MODER >> (pin * 2)) & 0x03;
            if (mode == 0x00) ret_val = input; /* PDF Reference: 00: Input */
            else if (mode == 0x01) ret_val = output; /* PDF Reference: 01: General purpose output mode */
            /* Ignore AF (10) and Analog (11) modes */
            break;
        case port_H:
            /* PDF Reference: GPIOH->MODER (bits 2y:2y+1) */
            mode = (GPIOH->MODER >> (pin * 2)) & 0x03;
            if (mode == 0x00) ret_val = input; /* PDF Reference: 00: Input */
            else if (mode == 0x01) ret_val = output; /* PDF Reference: 01: General purpose output mode */
            /* Ignore AF (10) and Analog (11) modes */
            break;
        default:
            break;
    
    return ret_val;


void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)

    WDT_Reset();
    do
    switch(port)
    
        case port_A:
            /* PDF Reference: GPIOA->PUPDR (bits 2y:2y+1) - Set to 00: No pull-up, pull-down */
            GPIOA->PUPDR &= ~((uint32_t)0x03 << (pin * 2));
            /* PDF Reference: GPIOA->BSRR - Atomic set/reset for initial value */
            if (value & (1 << pin)) SET_BIT(GPIOA->BSRR, pin); else SET_BIT(GPIOA->BSRR, pin + 16);
            /* PDF Reference: GPIOA->OTYPER (bit y) - Set output type (0:Push-pull, 1:Open-drain) */
            if (conn == open_drain) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin);
            /* PDF Reference: GPIOA->MODER (bits 2y:2y+1) - Set mode to 01: General purpose output mode */
            GPIOA->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits
            GPIOA->MODER |= ((uint32_t)0x01 << (pin * 2));  // Set mode to 01 (GP Output)
            break;
        case port_B:
            /* PDF Reference: GPIOB->PUPDR (bits 2y:2y+1) - Set to 00: No pull-up, pull-down */
            GPIOB->PUPDR &= ~((uint32_t)0x03 << (pin * 2));
            /* PDF Reference: GPIOB->BSRR - Atomic set/reset for initial value */
            if (value & (1 << pin)) SET_BIT(GPIOB->BSRR, pin); else SET_BIT(GPIOB->BSRR, pin + 16);
            /* PDF Reference: GPIOB->OTYPER (bit y) - Set output type (0:Push-pull, 1:Open-drain) */
            if (conn == open_drain) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin);
            /* PDF Reference: GPIOB->MODER (bits 2y:2y+1) - Set mode to 01: General purpose output mode */
            GPIOB->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits
            GPIOB->MODER |= ((uint32_t)0x01 << (pin * 2));  // Set mode to 01 (GP Output)
            break;
        case port_C:
            /* PDF Reference: GPIOC->PUPDR (bits 2y:2y+1) - Set to 00: No pull-up, pull-down */
            GPIOC->PUPDR &= ~((uint32_t)0x03 << (pin * 2));
            /* PDF Reference: GPIOC->BSRR - Atomic set/reset for initial value */
            if (value & (1 << pin)) SET_BIT(GPIOC->BSRR, pin); else SET_BIT(GPIOC->BSRR, pin + 16);
            /* PDF Reference: GPIOC->OTYPER (bit y) - Set output type (0:Push-pull, 1:Open-drain) */
            if (conn == open_drain) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin);
            /* PDF Reference: GPIOC->MODER (bits 2y:2y+1) - Set mode to 01: General purpose output mode */
            GPIOC->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits
            GPIOC->MODER |= ((uint32_t)0x01 << (pin * 2));  // Set mode to 01 (GP Output)
            break;
        case port_D:
            /* PDF Reference: GPIOD->PUPDR (bits 2y:2y+1) - Set to 00: No pull-up, pull-down */
            GPIOD->PUPDR &= ~((uint32_t)0x03 << (pin * 2));
            /* PDF Reference: GPIOD->BSRR - Atomic set/reset for initial value */
            if (value & (1 << pin)) SET_BIT(GPIOD->BSRR, pin); else SET_BIT(GPIOD->BSRR, pin + 16);
            /* PDF Reference: GPIOD->OTYPER (bit y) - Set output type (0:Push-pull, 1:Open-drain) */
            if (conn == open_drain) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin);
            /* PDF Reference: GPIOD->MODER (bits 2y:2y+1) - Set mode to 01: General purpose output mode */
            GPIOD->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits
            GPIOD->MODER |= ((uint32_t)0x01 << (pin * 2));  // Set mode to 01 (GP Output)
            break;
        case port_E:
            /* PDF Reference: GPIOE->PUPDR (bits 2y:2y+1) - Set to 00: No pull-up, pull-down */
            GPIOE->PUPDR &= ~((uint32_t)0x03 << (pin * 2));
            /* PDF Reference: GPIOE->BSRR - Atomic set/reset for initial value */
            if (value & (1 << pin)) SET_BIT(GPIOE->BSRR, pin); else SET_BIT(GPIOE->BSRR, pin + 16);
            /* PDF Reference: GPIOE->OTYPER (bit y) - Set output type (0:Push-pull, 1:Open-drain) */
            if (conn == open_drain) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin);
            /* PDF Reference: GPIOE->MODER (bits 2y:2y+1) - Set mode to 01: General purpose output mode */
            GPIOE->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits
            GPIOE->MODER |= ((uint32_t)0x01 << (pin * 2));  // Set mode to 01 (GP Output)
            break;
        case port_H:
            /* PDF Reference: GPIOH->PUPDR (bits 2y:2y+1) - Set to 00: No pull-up, pull-down */
            GPIOH->PUPDR &= ~((uint32_t)0x03 << (pin * 2));
            /* PDF Reference: GPIOH->BSRR - Atomic set/reset for initial value */
            if (value & (1 << pin)) SET_BIT(GPIOH->BSRR, pin); else SET_BIT(GPIOH->BSRR, pin + 16);
            /* PDF Reference: GPIOH->OTYPER (bit y) - Set output type (0:Push-pull, 1:Open-drain) */
            if (conn == open_drain) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin);
            /* PDF Reference: GPIOH->MODER (bits 2y:2y+1) - Set mode to 01: General purpose output mode */
            GPIOH->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits
            GPIOH->MODER |= ((uint32_t)0x01 << (pin * 2));  // Set mode to 01 (GP Output)
            break;
        default:
            break;
    
    while(GPIO_Direction_get(port, pin) != output);


void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull)

    WDT_Reset();
    do
    switch(port)
    
        case port_A:
            /* PDF Reference: GPIOA->PUPDR (bits 2y:2y+1) - Set pull-up/down */
            GPIOA->PUPDR &= ~((uint32_t)0x03 << (pin * 2)); // Clear pull-up/down bits first
            if (pull == pull_up) GPIOA->PUPDR |= ((uint32_t)0x01 << (pin * 2)); /* PDF Reference: 01: Pull-up */
            else if (pull == pull_down) GPIOA->PUPDR |= ((uint32_t)0x02 << (pin * 2)); /* PDF Reference: 10: Pull-down */
            /* If pull == no_pull (00), bits remain cleared */
            /* PDF Reference: GPIOA->MODER (bits 2y:2y+1) - Set mode to 00: Input */
            GPIOA->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits (Sets mode to 00: Input)
            break;
        case port_B:
            /* PDF Reference: GPIOB->PUPDR (bits 2y:2y+1) - Set pull-up/down */
            GPIOB->PUPDR &= ~((uint32_t)0x03 << (pin * 2)); // Clear pull-up/down bits first
            if (pull == pull_up) GPIOB->PUPDR |= ((uint32_t)0x01 << (pin * 2)); /* PDF Reference: 01: Pull-up */
            else if (pull == pull_down) GPIOB->PUPDR |= ((uint32_t)0x02 << (pin * 2)); /* PDF Reference: 10: Pull-down */
            /* If pull == no_pull (00), bits remain cleared */
            /* PDF Reference: GPIOB->MODER (bits 2y:2y+1) - Set mode to 00: Input */
            GPIOB->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits (Sets mode to 00: Input)
            break;
        case port_C:
            /* PDF Reference: GPIOC->PUPDR (bits 2y:2y+1) - Set pull-up/down */
            GPIOC->PUPDR &= ~((uint32_t)0x03 << (pin * 2)); // Clear pull-up/down bits first
            if (pull == pull_up) GPIOC->PUPDR |= ((uint32_t)0x01 << (pin * 2)); /* PDF Reference: 01: Pull-up */
            else if (pull == pull_down) GPIOC->PUPDR |= ((uint32_t)0x02 << (pin * 2)); /* PDF Reference: 10: Pull-down */
            /* If pull == no_pull (00), bits remain cleared */
            /* PDF Reference: GPIOC->MODER (bits 2y:2y+1) - Set mode to 00: Input */
            GPIOC->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits (Sets mode to 00: Input)
            break;
        case port_D:
            /* PDF Reference: GPIOD->PUPDR (bits 2y:2y+1) - Set pull-up/down */
            GPIOD->PUPDR &= ~((uint32_t)0x03 << (pin * 2)); // Clear pull-up/down bits first
            if (pull == pull_up) GPIOD->PUPDR |= ((uint32_t)0x01 << (pin * 2)); /* PDF Reference: 01: Pull-up */
            else if (pull == pull_down) GPIOD->PUPDR |= ((uint32_t)0x02 << (pin * 2)); /* PDF Reference: 10: Pull-down */
            /* If pull == no_pull (00), bits remain cleared */
            /* PDF Reference: GPIOD->MODER (bits 2y:2y+1) - Set mode to 00: Input */
            GPIOD->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits (Sets mode to 00: Input)
            break;
        case port_E:
            /* PDF Reference: GPIOE->PUPDR (bits 2y:2y+1) - Set pull-up/down */
            GPIOE->PUPDR &= ~((uint32_t)0x03 << (pin * 2)); // Clear pull-up/down bits first
            if (pull == pull_up) GPIOE->PUPDR |= ((uint32_t)0x01 << (pin * 2)); /* PDF Reference: 01: Pull-up */
            else if (pull == pull_down) GPIOE->PUPDR |= ((uint32_t)0x02 << (pin * 2)); /* PDF Reference: 10: Pull-down */
            /* If pull == no_pull (00), bits remain cleared */
            /* PDF Reference: GPIOE->MODER (bits 2y:2y+1) - Set mode to 00: Input */
            GPIOE->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits (Sets mode to 00: Input)
            break;
        case port_H:
            /* PDF Reference: GPIOH->PUPDR (bits 2y:2y+1) - Set pull-up/down */
            GPIOH->PUPDR &= ~((uint32_t)0x03 << (pin * 2)); // Clear pull-up/down bits first
            if (pull == pull_up) GPIOH->PUPDR |= ((uint32_t)0x01 << (pin * 2)); /* PDF Reference: 01: Pull-up */
            else if (pull == pull_down) GPIOH->PUPDR |= ((uint32_t)0x02 << (pin * 2)); /* PDF Reference: 10: Pull-down */
            /* If pull == no_pull (00), bits remain cleared */
            /* PDF Reference: GPIOH->MODER (bits 2y:2y+1) - Set mode to 00: Input */
            GPIOH->MODER &= ~((uint32_t)0x03 << (pin * 2)); // Clear mode bits (Sets mode to 00: Input)
            break;
        default:
            break;
    
    while(GPIO_Direction_get(port, pin) != input);

/***********************************************************************************************************************
* End of File
***********************************************************************************************************************/