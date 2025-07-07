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

// Assume WDT_Reset() is defined elsewhere
extern void WDT_Reset(void);

// Assume standard peripheral access header is included by STM32F401RC_GPIO.h
// This header would define GPIOA, GPIOB, etc. pointers and their register members (MODER, ODR, IDR, PUPDR, OTYPER).
// Assume macros like SET_BIT, CLR_BIT, TOG_BIT, GET_BIT are defined in STM32F401RC_GPIO.h
// Assume enums and types like tport, tpin, tbyte, t_direction, t_usage, t_output_conn, t_pull are defined in STM32F401RC_GPIO.h

/**Functions Implementation ============================================================================*/

void GPIO_Value_Set(tport port, tpin pin, tbyte value) {
    WDT_Reset();
    switch(port) {
        case port_A:
            if (value & (1U << pin)) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin);
            break;
        case port_B:
            if (value & (1U << pin)) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin);
            break;
        case port_C:
            if (value & (1U << pin)) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin);
            break;
        case port_D:
            if (value & (1U << pin)) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin);
            break;
        case port_E:
            if (value & (1U << pin)) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin);
            break;
        case port_H:
            if (value & (1U << pin)) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
            break;
        default:
            break;
    }
}

tbyte GPIO_Value_Get(tport port, tpin pin) {
    WDT_Reset();
    switch(port) {
        case port_A: return GET_BIT(GPIOA->IDR, pin);
        case port_B: return GET_BIT(GPIOB->IDR, pin);
        case port_C: return GET_BIT(GPIOC->IDR, pin);
        case port_D: return GET_BIT(GPIOD->IDR, pin);
        case port_E: return GET_BIT(GPIOE->IDR, pin);
        case port_H: return GET_BIT(GPIOH->IDR, pin);
        default:
            return 0;
    }
}

void GPIO_Value_Tog(tport port, tpin pin) {
    WDT_Reset();
    switch(port) {
        case port_A: TOG_BIT(GPIOA->ODR, pin); break;
        case port_B: TOG_BIT(GPIOB->ODR, pin); break;
        case port_C: TOG_BIT(GPIOC->ODR, pin); break;
        case port_D: TOG_BIT(GPIOD->ODR, pin); break;
        case port_E: TOG_BIT(GPIOE->ODR, pin); break;
        case port_H: TOG_BIT(GPIOH->ODR, pin); break;
        default:
            break;
    }
}

t_direction GPIO_Direction_get(tport port, tpin pin) {
    WDT_Reset();
    uint32_t moder_val = 0;
    switch(port) {
        case port_A: moder_val = GPIOA->MODER; break;
        case port_B: moder_val = GPIOB->MODER; break;
        case port_C: moder_val = GPIOC->MODER; break;
        case port_D: moder_val = GPIOD->MODER; break;
        case port_E: moder_val = GPIOE->MODER; break;
        case port_H: moder_val = GPIOH->MODER; break;
        default:
            // Assuming input is the default/error state for direction (e.g., enum value 0)
            return (t_direction)input; 
    }
    // Extract the 2-bit mode configuration for the pin (00: Input, 01: Output, 10: Alternate, 11: Analog)
    uint32_t mode = (moder_val >> (pin * 2)) & 0x03U;
    // Assuming t_direction enum values match the 2-bit mode values (input=0, output=1, alternate_function=2, analog=3)
    return (t_direction)mode; 
}

void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn) {
    WDT_Reset();
    // Loop until the direction is confirmed as output, as required by the rules.
    do {
        switch(port) {
            case port_A:
                // Clear Pull-up/Pull-down configuration (2 bits per pin)
                GPIOA->PUPDR &= ~(0x03UL << (pin * 2));
                // Set initial output value
                if (value & (1U << pin)) SET_BIT(GPIOA->ODR, pin); else CLR_BIT(GPIOA->ODR, pin);
                // Set Output Type (Push-pull or Open-drain - 1 bit per pin)
                // Based on rule: communication_usage -> Open-drain (OTYPER bit 1), general_usage -> Push-pull (OTYPER bit 0)
                if (usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin);
                // Set Mode to Output (01 - 2 bits per pin)
                GPIOA->MODER &= ~(0x03UL << (pin * 2)); // Clear existing mode bits
                GPIOA->MODER |= (0x01UL << (pin * 2)); // Set Output mode (01)
                break;
            case port_B:
                GPIOB->PUPDR &= ~(0x03UL << (pin * 2));
                if (value & (1U << pin)) SET_BIT(GPIOB->ODR, pin); else CLR_BIT(GPIOB->ODR, pin);
                if (usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin);
                GPIOB->MODER &= ~(0x03UL << (pin * 2));
                GPIOB->MODER |= (0x01UL << (pin * 2));
                break;
            case port_C:
                GPIOC->PUPDR &= ~(0x03UL << (pin * 2));
                if (value & (1U << pin)) SET_BIT(GPIOC->ODR, pin); else CLR_BIT(GPIOC->ODR, pin);
                if (usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin);
                GPIOC->MODER &= ~(0x03UL << (pin * 2));
                GPIOC->MODER |= (0x01UL << (pin * 2));
                break;
            case port_D:
                GPIOD->PUPDR &= ~(0x03UL << (pin * 2));
                if (value & (1U << pin)) SET_BIT(GPIOD->ODR, pin); else CLR_BIT(GPIOD->ODR, pin);
                if (usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin);
                GPIOD->MODER &= ~(0x03UL << (pin * 2));
                GPIOD->MODER |= (0x01UL << (pin * 2));
                break;
            case port_E:
                GPIOE->PUPDR &= ~(0x03UL << (pin * 2));
                if (value & (1U << pin)) SET_BIT(GPIOE->ODR, pin); else CLR_BIT(GPIOE->ODR, pin);
                if (usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin);
                GPIOE->MODER &= ~(0x03UL << (pin * 2));
                GPIOE->MODER |= (0x01UL << (pin * 2));
                break;
            case port_H:
                GPIOH->PUPDR &= ~(0x03UL << (pin * 2));
                if (value & (1U << pin)) SET_BIT(GPIOH->ODR, pin); else CLR_BIT(GPIOH->ODR, pin);
                if (usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin);
                GPIOH->MODER &= ~(0x03UL << (pin * 2));
                GPIOH->MODER |= (0x01UL << (pin * 2));
                break;
            default:
                // Invalid port. Break switch. do-while might loop infinitely depending on GPIO_Direction_get default.
                break;
        }
    } while(GPIO_Direction_get(port, pin) != output);
}

void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull) {
    WDT_Reset();
    // Loop until the direction is confirmed as input, as required by the rules.
    do {
        switch(port) {
            case port_A:
                // Set Pull-up/Pull-down configuration based on t_pull (2 bits per pin)
                // no_pull (00), pull_up (01), pull_down (10)
                GPIOA->PUPDR &= ~(0x03UL << (pin * 2)); // Clear existing pull bits
                if (pull == pull_up) GPIOA->PUPDR |= (0x01UL << (pin * 2)); // Set Pull-up (01)
                else if (pull == pull_down) GPIOA->PUPDR |= (0x02UL << (pin * 2)); // Set Pull-down (10)
                // Set Output Type (less relevant for pure input, but rule exists - 1 bit per pin)
                // Based on rule: communication_usage -> Open-drain (OTYPER bit 1), general_usage -> Push-pull (OTYPER bit 0)
                if (usage == communication_usage) SET_BIT(GPIOA->OTYPER, pin); else CLR_BIT(GPIOA->OTYPER, pin);
                // Set Mode to Input (00 - 2 bits per pin)
                GPIOA->MODER &= ~(0x03UL << (pin * 2)); // Set Input mode (00)
                break;
            case port_B:
                GPIOB->PUPDR &= ~(0x03UL << (pin * 2));
                if (pull == pull_up) GPIOB->PUPDR |= (0x01UL << (pin * 2));
                else if (pull == pull_down) GPIOB->PUPDR |= (0x02UL << (pin * 2));
                if (usage == communication_usage) SET_BIT(GPIOB->OTYPER, pin); else CLR_BIT(GPIOB->OTYPER, pin);
                GPIOB->MODER &= ~(0x03UL << (pin * 2));
                break;
            case port_C:
                GPIOC->PUPDR &= ~(0x03UL << (pin * 2));
                if (pull == pull_up) GPIOC->PUPDR |= (0x01UL << (pin * 2));
                else if (pull == pull_down) GPIOC->PUPDR |= (0x02UL << (pin * 2));
                if (usage == communication_usage) SET_BIT(GPIOC->OTYPER, pin); else CLR_BIT(GPIOC->OTYPER, pin);
                GPIOC->MODER &= ~(0x03UL << (pin * 2));
                break;
            case port_D:
                GPIOD->PUPDR &= ~(0x03UL << (pin * 2));
                if (pull == pull_up) GPIOD->PUPDR |= (0x01UL << (pin * 2));
                else if (pull == pull_down) GPIOD->PUPDR |= (0x02UL << (pin * 2));
                if (usage == communication_usage) SET_BIT(GPIOD->OTYPER, pin); else CLR_BIT(GPIOD->OTYPER, pin);
                GPIOD->MODER &= ~(0x03UL << (pin * 2));
                break;
            case port_E:
                GPIOE->PUPDR &= ~(0x03UL << (pin * 2));
                if (pull == pull_up) GPIOE->PUPDR |= (0x01UL << (pin * 2));
                else if (pull == pull_down) GPIOE->PUPDR |= (0x02UL << (pin * 2));
                if (usage == communication_usage) SET_BIT(GPIOE->OTYPER, pin); else CLR_BIT(GPIOE->OTYPER, pin);
                GPIOE->MODER &= ~(0x03UL << (pin * 2));
                break;
            case port_H:
                GPIOH->PUPDR &= ~(0x03UL << (pin * 2));
                if (pull == pull_up) GPIOH->PUPDR |= (0x01UL << (pin * 2));
                else if (pull == pull_down) GPIOH->PUPDR |= (0x02UL << (pin * 2));
                if (usage == communication_usage) SET_BIT(GPIOH->OTYPER, pin); else CLR_BIT(GPIOH->OTYPER, pin);
                GPIOH->MODER &= ~(0x03UL << (pin * 2));
                break;
            default:
                // Invalid port. Break switch. do-while will execute once because GPIO_Direction_get default is input.
                break;
        }
    } while(GPIO_Direction_get(port, pin) != input);
}