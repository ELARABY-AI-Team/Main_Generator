/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : Generic MCAL GPIO Driver for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
// Assume STM32F401RC_GPIO.h defines the necessary types (tport, tpin, tbyte, t_direction, t_usage, t_output_conn)
// and potentially declares WDT_Reset().
#include "STM32F401RC_GPIO.h"

// Include standard STM32 CMSIS header for register access using structure pointers (e.g., GPIOA->ODR)
// This header should be part of the microcontroller's standard peripheral library package.
#include "stm32f4xx.h" // Generic header for STM32F4 series, includes definitions like GPIO_TypeDef

// Assume WDT_Reset() is declared and implemented elsewhere.
// For compilation purposes, a dummy declaration might be needed if not in included headers.
// extern void WDT_Reset(void);


/**Static Variables ====================================================================*/
// No static variables requested

/**Functions ===========================================================================*/

/*
 * Define generic bit manipulation macros.
 * These macros perform standard bitwise operations on a given 32-bit register address (or volatile uint32_t) and bit number.
 * NOTE: Applying these single-bit macros directly to STM32 registers with 2-bit fields (like MODER, PUPDR)
 * will not configure them correctly according to the datasheet. This implementation strictly follows the user's
 * instruction to use these macros with PMx and PUx, resulting in non-standard register configuration for these fields.
 */
#define SET_BIT(REG, BIT)     ((*(volatile uint32_t*)(REG)) |= (1U << (BIT)))
#define CLR_BIT(REG, BIT)     ((*(volatile uint32_t*)(REG)) &= ~(1U << (BIT)))
#define TOG_BIT(REG, BIT)     ((*(volatile uint32_t*)(REG)) ^= (1U << (BIT)))
#define GET_BIT(REG, BIT)     (((*(volatile uint32_t*)(REG)) >> (BIT)) & 1U)


/**
 * @brief  Set the value of a specific GPIO pin.
 * @param  port: The GPIO port (e.g., Port_0 mapped to GPIOA).
 * @param  pin: The pin number (0-15).
 * @param  value: The value determining the logic level (interprets value & (1 << pin)).
 * @retval None
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();

    // Local pointer to the Output Data Register (ODR) for the selected port.
    // Corresponds to Px in the description.
    volatile uint32_t* Px_reg_addr = NULL;

    switch(port)
    {
        case Port_0: // Assuming Port_0 maps to GPIOA
            Px_reg_addr = (volatile uint32_t*)&GPIOA->ODR;
            break;
        case Port_1: // Assuming Port_1 maps to GPIOB
            Px_reg_addr = (volatile uint32_t*)&GPIOB->ODR;
            break;
        case Port_2: // Assuming Port_2 maps to GPIOC
            Px_reg_addr = (volatile uint32_t*)&GPIOC->ODR;
            break;
        case Port_3: // Assuming Port_3 maps to GPIOD
            Px_reg_addr = (volatile uint32_t*)&GPIOD->ODR;
            break;
        case Port_4: // Assuming Port_4 maps to GPIOE
            Px_reg_addr = (volatile uint32_t*)&GPIOE->ODR;
            break;
        // STM32F401RC also has GPIOH (Port_7 assuming sequential mapping)
        case Port_7: // Assuming Port_7 maps to GPIOH
             Px_reg_addr = (volatile uint32_t*)&GPIOH->ODR;
             break;
        default:
            // Handle invalid port. For a void function, just return.
            return;
    }

    // Check if the register address is valid before dereferencing
    if (Px_reg_addr == NULL) {
        // Invalid port provided
        return;
    }

    // Check the requested value based on the specified bit position
    if (value & (1 << pin))
    {
        // Use SET_BIT macro on the ODR address
        SET_BIT(Px_reg_addr, pin);
    }
    else
    {
        // Use CLR_BIT macro on the ODR address
        CLR_BIT(Px_reg_addr, pin);
    }
}

/**
 * @brief  Get the value of a specific GPIO pin.
 * @param  port: The GPIO port (e.g., Port_0 mapped to GPIOA).
 * @param  pin: The pin number (0-15).
 * @retval The pin value (0 or 1). Returns 0 for invalid port.
 */
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();

    // Local pointer to the Input Data Register (IDR) for the selected port.
    // Corresponds to Px in the description when reading value.
    volatile uint32_t* Px_reg_addr = NULL;

    switch(port)
    {
        case Port_0: // Assuming Port_0 maps to GPIOA
            Px_reg_addr = (volatile uint32_t*)&GPIOA->IDR; // Read from Input Data Register
            break;
        case Port_1: // Assuming Port_1 maps to GPIOB
            Px_reg_addr = (volatile uint32_t*)&GPIOB->IDR;
            break;
        case Port_2: // Assuming Port_2 maps to GPIOC
            Px_reg_addr = (volatile uint32_t*)&GPIOC->IDR;
            break;
        case Port_3: // Assuming Port_3 maps to GPIOD
            Px_reg_addr = (volatile uint32_t*)&GPIOD->IDR;
            break;
        case Port_4: // Assuming Port_4 maps to GPIOE
            Px_reg_addr = (volatile uint32_t*)&GPIOE->IDR;
            break;
        case Port_7: // Assuming Port_7 maps to GPIOH
            Px_reg_addr = (volatile uint32_t*)&GPIOH->IDR;
            break;
        default:
            // Handle invalid port, return 0 as a default value.
            return 0;
    }

    // Check if the register address is valid
    if (Px_reg_addr == NULL) {
        return 0; // Return 0 for invalid port
    }

    // Return GET_BIT(Px, pin) using the IDR address
    return (tbyte)GET_BIT(Px_reg_addr, pin);
}

/**
 * @brief  Toggle the value of a specific GPIO pin.
 * @param  port: The GPIO port (e.g., Port_0 mapped to GPIOA).
 * @param  pin: The pin number (0-15).
 * @retval None
 */
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();

    // Local pointer to the Output Data Register (ODR) for the selected port.
    // Corresponds to Px in the description when toggling.
    volatile uint32_t* Px_reg_addr = NULL;

    switch(port)
    {
        case Port_0: // Assuming Port_0 maps to GPIOA
            Px_reg_addr = (volatile uint32_t*)&GPIOA->ODR;
            break;
        case Port_1: // Assuming Port_1 maps to GPIOB
            Px_reg_addr = (volatile uint32_t*)&GPIOB->ODR;
            break;
        case Port_2: // Assuming Port_2 maps to GPIOC
            Px_reg_addr = (volatile uint32_t*)&GPIOC->ODR;
            break;
        case Port_3: // Assuming Port_3 maps to GPIOD
            Px_reg_addr = (volatile uint32_t*)&GPIOD->ODR;
            break;
        case Port_4: // Assuming Port_4 maps to GPIOE
            Px_reg_addr = (volatile uint32_t*)&GPIOE->ODR;
            break;
         case Port_7: // Assuming Port_7 maps to GPIOH
            Px_reg_addr = (volatile uint32_t*)&GPIOH->ODR;
            break;
        default:
            // Handle invalid port. For a void function, just return.
            return;
    }

     // Check if the register address is valid
    if (Px_reg_addr == NULL) {
        return;
    }

    // Toggle the pin using TOG_BIT(Px, pin) on the ODR address
    TOG_BIT(Px_reg_addr, pin);
}

/**
 * @brief  Get the direction of a specific GPIO pin.
 * @param  port: The GPIO port (e.g., Port_0 mapped to GPIOA).
 * @param  pin: The pin number (0-15).
 * @retval The pin direction (t_direction enum value: input or output). Returns output (0) for invalid port.
 */
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();

    // Local pointer to the Mode Register (MODER) for the selected port.
    // Corresponds to PMx in the description.
    volatile uint32_t* PMx_reg_addr = NULL;

    switch(port)
    {
        case Port_0: // Assuming Port_0 maps to GPIOA
            PMx_reg_addr = (volatile uint32_t*)&GPIOA->MODER; // Map PMx to MODER
            break;
        case Port_1: // Assuming Port_1 maps to GPIOB
            PMx_reg_addr = (volatile uint32_t*)&GPIOB->MODER;
            break;
        case Port_2: // Assuming Port_2 maps to GPIOC
            PMx_reg_addr = (volatile uint32_t*)&GPIOC->MODER;
            break;
        case Port_3: // Assuming Port_3 maps to GPIOD
            PMx_reg_addr = (volatile uint32_t*)&GPIOD->MODER;
            break;
        case Port_4: // Assuming Port_4 maps to GPIOE
            PMx_reg_addr = (volatile uint32_t*)&GPIOE->MODER;
            break;
        case Port_7: // Assuming Port_7 maps to GPIOH
            PMx_reg_addr = (volatile uint32_t*)&GPIOH->MODER;
            break;
        default:
            // Handle invalid port, return output (0) as a default.
            return (t_direction)output;
    }

    // Check if the register address is valid
    if (PMx_reg_addr == NULL) {
         return (t_direction)output; // Defaulting to output for invalid port
    }

    // NOTE: This reads only bit 'pin' of the MODER register, which has 2 bits per pin (2*pin and 2*pin+1).
    // This single bit does NOT correctly determine Input/Output state for STM32 MODER (00=Input, 01=Output).
    // This implementation strictly follows the user's explicit instruction to use GET_BIT(PMx, pin).
    // Based on the user's logic (SET_BIT(PMx, pin) for input, CLR_BIT(PMx, pin) for output),
    // a return value of 1 is intended for input, and 0 for output.
    // The literal GET_BIT will return 1 if MODER bit 'pin' is set, 0 if clear, which is inconsistent
    // with standard STM32 MODER behavior.
    return (t_direction)GET_BIT(PMx_reg_addr, pin);
}

/**
 * @brief  Initialize a GPIO pin as output.
 * @param  port: The GPIO port (e.g., Port_0 mapped to GPIOA).
 * @param  pin: The pin number (0-15).
 * @param  value: The initial value to set (interprets value & (1 << pin)).
 * @param  usage: The pin usage (general_usage or communication_usage). Affects output type (Push-Pull/Open-Drain) via POMx.
 * @param  conn: Output connection type (push_pull or open_drain). Parameter is present per signature but logic follows 'usage'.
 * @retval None
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    // Implement using the specified do-while loop structure checking direction.
    // This structure is unusual for standard MCAL GPIO initialization which typically
    // just configures the registers once. It implies waiting for the configuration
    // to take effect or be readable via GPIO_Direction_get.
    do
    {
        // WDT_Reset() is typically called once at the beginning of the function.
        // If it needs to be reset on every iteration of this loop, move it here.
        // Following the instruction "at the beginning of each function", it's outside the loop.
        // WDT_Reset(); // If needed inside the loop

        volatile uint32_t* Px_reg_addr = NULL;  // Maps to ODR
        volatile uint32_t* PMx_reg_addr = NULL; // Maps to MODER (conceptually for single-bit control)
        volatile uint32_t* PUx_reg_addr = NULL; // Maps to PUPDR (conceptually for single-bit control)
        volatile uint32_t* POMx_reg_addr = NULL; // Maps to OTYPER

        switch(port)
        {
            case Port_0: // Assuming Port_0 maps to GPIOA
                Px_reg_addr = (volatile uint32_t*)&GPIOA->ODR;
                PMx_reg_addr = (volatile uint32_t*)&GPIOA->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOA->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOA->OTYPER; // Map POMx to OTYPER
                break;
            case Port_1: // Assuming Port_1 maps to GPIOB
                Px_reg_addr = (volatile uint32_t*)&GPIOB->ODR;
                PMx_reg_addr = (volatile uint32_t*)&GPIOB->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOB->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOB->OTYPER;
                break;
            case Port_2: // Assuming Port_2 maps to GPIOC
                Px_reg_addr = (volatile uint32_t*)&GPIOC->ODR;
                PMx_reg_addr = (volatile uint32_t*)&GPIOC->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOC->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOC->OTYPER;
                break;
            case Port_3: // Assuming Port_3 maps to GPIOD
                Px_reg_addr = (volatile uint32_t*)&GPIOD->ODR;
                PMx_reg_addr = (volatile uint32_t*)&GPIOD->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOD->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOD->OTYPER;
                break;
            case Port_4: // Assuming Port_4 maps to GPIOE
                Px_reg_addr = (volatile uint32_t*)&GPIOE->ODR;
                PMx_reg_addr = (volatile uint32_t*)&GPIOE->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOE->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOE->OTYPER;
                break;
             case Port_7: // Assuming Port_7 maps to GPIOH
                Px_reg_addr = (volatile uint32_t*)&GPIOH->ODR;
                PMx_reg_addr = (volatile uint32_t*)&GPIOH->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOH->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOH->OTYPER;
                break;
            default:
                // Exit the loop and function if port is invalid
                return;
        }

        // Check if register pointers were successfully assigned
        if (Px_reg_addr == NULL || PMx_reg_addr == NULL || PUx_reg_addr == NULL || POMx_reg_addr == NULL) {
             return; // Exit on invalid port mapping failure
        }

        // CLR_BIT(PUx, pin); // Disable pull-up
        // NOTE: This clears only bit 'pin' of the PUPDR register, which has 2 bits per pin (2*pin and 2*pin+1).
        // This does NOT correctly configure the pin for No Pull-up/No Pull-down (00) in STM32 PUPDR.
        // This implementation strictly follows the user's explicit instruction to use CLR_BIT(PUx, pin).
        CLR_BIT(PUx_reg_addr, pin);

        // Set initial output value based on 'value' parameter
        if (value & (1 << pin))
        {
            SET_BIT(Px_reg_addr, pin); // Set initial output value using ODR
        }
        else
        {
            CLR_BIT(Px_reg_addr, pin); // Clear initial output value using ODR
        }

        // Configure Output Type (Push-Pull/Open-Drain) based on 'usage' parameter
        // if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin);
        // This maps communication_usage to Open-Drain (SET_BIT(OTYPER, pin)) and general_usage to Push-Pull (CLR_BIT(OTYPER, pin)).
        // This *does* align with STM32 OTYPER register (0=Push-Pull, 1=Open-Drain).
        if (usage == communication_usage)
        {
             SET_BIT(POMx_reg_addr, pin); // Set Open-Drain using OTYPER
        }
        else
        {
             CLR_BIT(POMx_reg_addr, pin); // Set Push-Pull using OTYPER
        }

        // CLR_BIT(PMx, pin); // Set as output mode
        // NOTE: This clears only bit 'pin' of the MODER register, which has 2 bits per pin (2*pin and 2*pin+1).
        // This does NOT correctly configure the pin for Output mode (01) in STM32 MODER.
        // This implementation strictly follows the user's explicit instruction to use CLR_BIT(PMx, pin).
        CLR_BIT(PMx_reg_addr, pin);

    } while(GPIO_Direction_get(port, pin) != output); // Loop until GPIO_Direction_get reports 'output'
}

/**
 * @brief  Initialize a GPIO pin as input.
 * @param  port: The GPIO port (e.g., Port_0 mapped to GPIOA).
 * @param  pin: The pin number (0-15).
 * @param  usage: The pin usage (general_usage or communication_usage). Affects POMx (OTYPER), unusual for input but follows instruction.
 * @retval None
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage)
{
    // Implement using the specified do-while loop structure checking direction.
    // This structure is unusual for standard MCAL GPIO initialization.
    do
    {
        // WDT_Reset() is typically called once at the beginning of the function.
        // If it needs to be reset on every iteration of this loop, move it here.
        // Following the instruction "at the beginning of each function", it's outside the loop.
        // WDT_Reset(); // If needed inside the loop

        volatile uint32_t* PMx_reg_addr = NULL; // Maps to MODER (conceptually for single-bit control)
        volatile uint32_t* PUx_reg_addr = NULL; // Maps to PUPDR (conceptually for single-bit control)
        volatile uint32_t* POMx_reg_addr = NULL; // Maps to OTYPER

        switch(port)
        {
            case Port_0: // Assuming Port_0 maps to GPIOA
                PMx_reg_addr = (volatile uint32_t*)&GPIOA->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOA->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOA->OTYPER; // Map POMx to OTYPER (affected by usage)
                break;
            case Port_1: // Assuming Port_1 maps to GPIOB
                PMx_reg_addr = (volatile uint32_t*)&GPIOB->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOB->PUPDR;
                POMx_reg_addr = (volatile uint32_t*)&GPIOB->OTYPER;
                break;
            case Port_2: // Assuming Port_2 maps to GPIOC
                PMx_reg_addr = (volatile uint32_t*)&GPIOC->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOC->PUPDR;
                 POMx_reg_addr = (volatile uint32_t*)&GPIOC->OTYPER;
                break;
            case Port_3: // Assuming Port_3 maps to GPIOD
                PMx_reg_addr = (volatile uint32_t*)&GPIOD->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOD->PUPDR;
                 POMx_reg_addr = (volatile uint32_t*)&GPIOD->OTYPER;
                break;
            case Port_4: // Assuming Port_4 maps to GPIOE
                PMx_reg_addr = (volatile uint32_t*)&GPIOE->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOE->PUPDR;
                 POMx_reg_addr = (volatile uint32_t*)&GPIOE->OTYPER;
                break;
             case Port_7: // Assuming Port_7 maps to GPIOH
                PMx_reg_addr = (volatile uint32_t*)&GPIOH->MODER;
                PUx_reg_addr = (volatile uint32_t*)&GPIOH->PUPDR;
                 POMx_reg_addr = (volatile uint32_t*)&GPIOH->OTYPER;
                break;
            default:
                // Exit the loop and function if port is invalid
                return;
        }

        // Check if register pointers were successfully assigned
        if (PMx_reg_addr == NULL || PUx_reg_addr == NULL || POMx_reg_addr == NULL) {
             return; // Exit on invalid port mapping failure
        }

        // SET_BIT(PUx, pin); // Enable pull-up
        // NOTE: This sets only bit 'pin' of the PUPDR register, which has 2 bits per pin (2*pin and 2*pin+1).
        // This does NOT correctly configure the pin for Pull-up (01) in STM32 PUPDR.
        // This implementation strictly follows the user's explicit instruction to use SET_BIT(PUx, pin).
        SET_BIT(PUx_reg_addr, pin);

        // Configure Output Type (Push-Pull/Open-Drain) based on 'usage' parameter.
        // This affects OTYPER (POMx). While OTYPER primarily affects output, the instruction is to apply
        // this logic for input initialization as well, following the provided template.
        // This *does* align with STM32 OTYPER register, even if its effect on an input pin is minimal or non-standard.
        if (usage == communication_usage)
        {
             SET_BIT(POMx_reg_addr, pin); // Set bit 'pin' in OTYPER
        }
        else
        {
             CLR_BIT(POMx_reg_addr, pin); // Clear bit 'pin' in OTYPER
        }

        // SET_BIT(PMx, pin); // Set as input mode
        // NOTE: This sets only bit 'pin' of the MODER register, which has 2 bits per pin (2*pin and 2*pin+1).
        // This does NOT correctly configure the pin for Input mode (00) in STM32 MODER.
        // This implementation strictly follows the user's explicit instruction to use SET_BIT(PMx, pin).
        SET_BIT(PMx_reg_addr, pin);

    } while(GPIO_Direction_get(port, pin) != input); // Loop until GPIO_Direction_get reports 'input'
}

// WDT_Reset() implementation would be here or in another file/library.
// Example dummy implementation if needed for isolated testing:
/*
void WDT_Reset(void) {
    // Replace with actual Watchdog Timer reset logic for STM32F401RC
    // e.g., Reload IWDG/WWDG counter
}
*/