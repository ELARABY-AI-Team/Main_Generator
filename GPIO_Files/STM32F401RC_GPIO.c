/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : Generic MCAL GPIO driver implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"
// Assuming WDT_Reset is declared in a Watchdog driver header or similar
extern void WDT_Reset(void);

/**Static Variables ====================================================================*/
// Static variables can go here if needed. None explicitly required by the prompt.

/**Functions ===========================================================================*/

/* Implement the following functions using MCAL style, macros, and MCU logic awareness: */

/**
 * @brief Sets the value of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @param value: The desired logic level (non-zero for high, zero for low).
 *               The actual bit representing the level is checked using value & (1 << pin).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset(); // Reset watchdog

    // Ensure pin is within valid range (0-15) - Basic validation
    if (pin > Pin_15) {
        // Handle error: Invalid pin number
        // Potentially log error, assert, or return
        return;
    }

    switch(port)
    {
        case Port_A:
            // Use the ODR (Output Data Register) for setting/clearing the output level.
            // This corresponds to Px in the description for data operations.
            if (value & (1U << pin)) // Check the bit corresponding to the pin in the value byte
            {
                SET_BIT(GPIOA->ODR, pin);
            }
            else
            {
                CLR_BIT(GPIOA->ODR, pin);
            }
            break;

        case Port_B:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOB->ODR, pin);
            }
            else
            {
                CLR_BIT(GPIOB->ODR, pin);
            }
            break;

        case Port_C:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOC->ODR, pin);
            }
            else
            {
                CLR_BIT(GPIOC->ODR, pin);
            }
            break;

        case Port_D:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOD->ODR, pin);
            }
            else
            {
                CLR_BIT(GPIOD->ODR, pin);
            }
            break;

        case Port_E:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOE->ODR, pin);
            }
            else
            {
                CLR_BIT(GPIOE->ODR, pin);
            }
            break;

        case Port_H:
            if (value & (1U << pin))
            {
                SET_BIT(GPIOH->ODR, pin);
            }
            else
            {
                CLR_BIT(GPIOH->ODR, pin);
            }
            break;

        default:
            // Handle error: Invalid port
            // Potentially log error, assert, or return
            break;
    }
}

/**
 * @brief Gets the current value of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @return The logic level of the pin (1 for high, 0 for low).
 */
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset(); // Reset watchdog
    tbyte ret_val = 0; // Default return value

    // Ensure pin is within valid range (0-15)
    if (pin > Pin_15) {
        // Handle error: Invalid pin number
        // Potentially log error, assert, or return 0
        return 0;
    }

    switch(port)
    {
        case Port_A:
            // Use the IDR (Input Data Register) to get the pin state.
            // This corresponds to Px in the description for data operations.
            ret_val = (tbyte)GET_BIT(GPIOA->IDR, pin);
            break;

        case Port_B:
            ret_val = (tbyte)GET_BIT(GPIOB->IDR, pin);
            break;

        case Port_C:
            ret_val = (tbyte)GET_BIT(GPIOC->IDR, pin);
            break;

        case Port_D:
            ret_val = (tbyte)GET_BIT(GPIOD->IDR, pin);
            break;

        case Port_E:
            ret_val = (tbyte)GET_BIT(GPIOE->IDR, pin);
            break;

        case Port_H:
            ret_val = (tbyte)GET_BIT(GPIOH->IDR, pin);
            break;

        default:
            // Handle error: Invalid port
            // Potentially log error, assert
            break;
    }

    return ret_val;
}

/**
 * @brief Toggles the value of a specific GPIO pin.
 *        Only applicable if the pin is configured as output.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 */
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset(); // Reset watchdog

    // Ensure pin is within valid range (0-15)
    if (pin > Pin_15) {
        // Handle error: Invalid pin number
        // Potentially log error, assert, or return
        return;
    }

    switch(port)
    {
        case Port_A:
            // Use the ODR (Output Data Register) for toggling.
            // This corresponds to Px in the description for data operations.
            TOG_BIT(GPIOA->ODR, pin);
            break;

        case Port_B:
            TOG_BIT(GPIOB->ODR, pin);
            break;

        case Port_C:
            TOG_BIT(GPIOC->ODR, pin);
            break;

        case Port_D:
            TOG_BIT(GPIOD->ODR, pin);
            break;

        case Port_E:
            TOG_BIT(GPIOE->ODR, pin);
            break;

        case Port_H:
            TOG_BIT(GPIOH->ODR, pin);
            break;

        default:
            // Handle error: Invalid port
            // Potentially log error, assert
            break;
    }
}

/**
 * @brief Gets the current direction (mode) of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @return The direction of the pin (input or output).
 */
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset(); // Reset watchdog
    t_direction ret_dir = input; // Default return value (Input)

    // Ensure pin is within valid range (0-15)
    if (pin > Pin_15) {
        // Handle error: Invalid pin number
        // Potentially log error, assert, or return input as a safe default
        return input;
    }

    // The MODER register (Mode Register) determines the pin direction/mode.
    // It uses 2 bits per pin: 00=Input, 01=Output, 10=Alternate Function, 11=Analog.
    // The request asks for GET_BIT(PMx, pin), which is simplified. We will get the 2 bits
    // for the pin from MODER and interpret them as input (00) or output (01).
    uint32_t mode_bits;
    volatile GPIO_TypeDef* GPIOx;

    switch(port)
    {
        case Port_A: GPIOx = GPIOA; break;
        case Port_B: GPIOx = GPIOB; break;
        case Port_C: GPIOx = GPIOC; break;
        case Port_D: GPIOx = GPIOD; break;
        case Port_E: GPIOx = GPIOE; break;
        case Port_H: GPIOx = GPIOH; break;
        default:
            // Handle error: Invalid port
            // Potentially log error, assert
            return input; // Return input as a safe default for invalid port
    }

    // Read the 2 relevant bits from the MODER register for the given pin.
    mode_bits = (GPIOx->MODER >> (pin * 2)) & 0x03U;

    // Interpret the 2 bits: 00 is Input, 01 is Output.
    if (mode_bits == 0x01U) // 01b = General purpose output mode
    {
        ret_dir = output;
    }
    else // Includes Input (00b), Alternate function (10b), Analog (11b).
         // Per the request, we only distinguish between input and output.
         // Anything not explicit output (01) is considered input by this function.
    {
        ret_dir = input;
    }

    return ret_dir;
}

/**
 * @brief Initializes a specific GPIO pin as output.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @param value: The initial output value (non-zero for high, zero for low).
 *               The actual bit representing the level is checked using value & (1 << pin).
 * @param usage: The intended usage (e.g., general_usage, communication_usage).
 *               Mapped to output type (Push-pull/Open-drain).
 * @param conn: The output connection type (e.g., push_pull, open_drain).
 *              Note: The implementation maps 'usage' to OTYPER, not 'conn' directly as per instructions.
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset(); // Reset watchdog

    // Ensure pin is within valid range (0-15)
    if (pin > Pin_15) {
        // Handle error: Invalid pin number
        // Potentially log error, assert, or return
        return;
    }

    volatile GPIO_TypeDef* GPIOx;

    // The loop body will attempt the configuration.
    do {
        // Find the base address of the GPIO peripheral
        switch(port)
        {
            case Port_A: GPIOx = GPIOA; break;
            case Port_B: GPIOx = GPIOB; break;
            case Port_C: GPIOx = GPIOC; break;
            case Port_D: GPIOx = GPIOD; break;
            case Port_E: GPIOx = GPIOE; break;
            case Port_H: GPIOx = GPIOH; break;
            default:
                // Handle error: Invalid port
                // Potentially log error, assert
                return; // Exit function if port is invalid
        }

        // Configure the pin:

        // 1. Disable pull-up/pull-down (PUPDR - 2 bits per pin)
        // CLR_BIT(PUx, pin); -> Clear the 2 bits corresponding to the pin
        GPIOx->PUPDR &= ~(0x03UL << (pin * 2)); // 00b = No pull-up, no pull-down

        // 2. Set initial output value (ODR - 1 bit per pin)
        // if(value & (1 << pin)) -> SET_BIT(Px, pin); else -> CLR_BIT(Px, pin);
        if (value & (1U << pin))
        {
            SET_BIT(GPIOx->ODR, pin);
        }
        else
        {
            CLR_BIT(GPIOx->ODR, pin);
        }

        // 3. Set output type (OTYPER - 1 bit per pin) based on 'usage'
        // if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin);
        // OTYPER: 0 = Push-pull, 1 = Open-drain. Communication usage often implies Open-drain.
        if (usage == communication_usage)
        {
            SET_BIT(GPIOx->OTYPER, pin); // Set Open-drain
        }
        else // Assume general_usage implies Push-pull
        {
            CLR_BIT(GPIOx->OTYPER, pin); // Clear Push-pull
        }
        // Note: The 'conn' parameter is unused in the requested logic,
        // as the output type is determined by 'usage'.

        // 4. Set direction as output (MODER - 2 bits per pin)
        // CLR_BIT(PMx, pin); -> This instruction implies clearing the bit.
        // For MODER, we need to set the mode to 01b (General purpose output mode).
        // This requires clearing the existing 2 bits and setting 01.
        GPIOx->MODER &= ~(0x03UL << (pin * 2)); // Clear existing mode bits
        GPIOx->MODER |= (0x01UL << (pin * 2));  // Set 01b for Output mode

        // Add default case to the switch if needed (though it's handled before the do-while)
        // This structure places the switch outside the do-while, so a default is not strictly needed *inside* the loop body switch
        // but the selection of GPIOx needs a default. Let's move the switch outside and use the pointer inside the do-while.

    } while(GPIO_Direction_get(port, pin) != output); // Check if direction is now output
}

/**
 * @brief Initializes a specific GPIO pin as input.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @param usage: The intended usage (e.g., general_usage, communication_usage).
 *               Mapped to output type (Push-pull/Open-drain) in a seemingly incorrect way for input pins.
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage)
{
    WDT_Reset(); // Reset watchdog

    // Ensure pin is within valid range (0-15)
    if (pin > Pin_15) {
        // Handle error: Invalid pin number
        // Potentially log error, assert, or return
        return;
    }

    volatile GPIO_TypeDef* GPIOx;

    do {
        // Find the base address of the GPIO peripheral
        switch(port)
        {
            case Port_A: GPIOx = GPIOA; break;
            case Port_B: GPIOx = GPIOB; break;
            case Port_C: GPIOx = GPIOC; break;
            case Port_D: GPIOx = GPIOD; break;
            case Port_E: GPIOx = GPIOE; break;
            case Port_H: GPIOx = GPIOH; break;
            default:
                // Handle error: Invalid port
                // Potentially log error, assert
                return; // Exit function if port is invalid
        }

        // Configure the pin:

        // 1. Enable pull-up (PUPDR - 2 bits per pin)
        // SET_BIT(PUx, pin); -> This instruction implies setting the bit.
        // For PUPDR, we need to set the mode to 01b (Pull-up).
        // This requires clearing the existing 2 bits and setting 01.
        GPIOx->PUPDR &= ~(0x03UL << (pin * 2)); // Clear existing pull-up/pull-down bits
        GPIOx->PUPDR |= (0x01UL << (pin * 2));  // Set 01b for Pull-up

        // 2. Set output type (OTYPER - 1 bit per pin) based on 'usage' - Note: OTYPER has NO EFFECT on input pins!
        // if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin);
        // OTYPER: 0 = Push-pull, 1 = Open-drain. Applies only in output mode.
        if (usage == communication_usage)
        {
             SET_BIT(GPIOx->OTYPER, pin); // Set OTYPER bit (no effect for input)
        }
        else // Assume general_usage
        {
             CLR_BIT(GPIOx->OTYPER, pin); // Clear OTYPER bit (no effect for input)
        }

        // 3. Set direction as input (MODER - 2 bits per pin)
        // SET_BIT(PMx, pin); -> This instruction implies setting the bit.
        // For MODER, we need to set the mode to 00b (Input mode).
        // This requires clearing the existing 2 bits. Setting 00 explicitly after clearing is redundant.
        GPIOx->MODER &= ~(0x03UL << (pin * 2)); // Clear existing mode bits (sets 00b for Input mode)

        // Add default case to the switch if needed (handled before the do-while)

    } while(GPIO_Direction_get(port, pin) != input); // Check if direction is now input
}

/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Generic MCAL GPIO driver header for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H
#define STM32F401RC_GPIO_H

/**Includes ============================================================================*/
#include "stm32f4xx.h" // Required for GPIO_TypeDef and peripheral base addresses (GPIOA, etc.)
// Include the WDT header file if it exists, or declare WDT_Reset here.
// Example: #include "WDT_driver.h"

/**Defines =============================================================================*/

// Generic Bit Manipulation Macros
// Using volatile cast for register access
#define SET_BIT(REG, BIT)   ((REG) |= (1UL << (BIT)))
#define CLR_BIT(REG, BIT)   ((REG) &= ~(1UL << (BIT)))
#define TOG_BIT(REG, BIT)   ((REG) ^= (1UL << (BIT)))
#define GET_BIT(REG, BIT)   (((REG) >> (BIT)) & 1UL)

/**Type Definitions ====================================================================*/

// Enumeration for GPIO Ports
typedef enum
{
    Port_A, // Corresponds to GPIOA
    Port_B, // Corresponds to GPIOB
    Port_C, // Corresponds to GPIOC
    Port_D, // Corresponds to GPIOD
    Port_E, // Corresponds to GPIOE
    Port_H, // Corresponds to GPIOH
    Port_MAX // Sentinel value
} tport;

// Enumeration for GPIO Pins (0 to 15)
typedef enum
{
    Pin_0,
    Pin_1,
    Pin_2,
    Pin_3,
    Pin_4,
    Pin_5,
    Pin_6,
    Pin_7,
    Pin_8,
    Pin_9,
    Pin_10,
    Pin_11,
    Pin_12,
    Pin_13,
    Pin_14,
    Pin_15,
    Pin_MAX // Sentinel value
} tpin;

// Type definition for a byte (8-bit unsigned)
typedef unsigned char tbyte;

// Enumeration for GPIO Direction (Input/Output)
typedef enum
{
    input,
    output
} t_direction;

// Enumeration for Pin Usage (influences Output Type configuration)
typedef enum
{
    general_usage,      // Typically Push-pull output
    communication_usage // Typically Open-drain output (e.g., I2C)
} t_usage;

// Enumeration for Output Connection Type (Push-pull/Open-drain)
// Note: In the provided implementation logic, this enum is defined but 'usage'
// dictates the OTYPER setting in init functions, not 'conn' directly.
typedef enum
{
    push_pull,
    open_drain
} t_output_conn;

/**Function Prototypes =================================================================*/

/**
 * @brief Sets the value of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @param value: The desired logic level (non-zero for high, zero for low).
 *               The actual bit representing the level is checked using value & (1 << pin).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/**
 * @brief Gets the current value of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @return The logic level of the pin (1 for high, 0 for low).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/**
 * @brief Toggles the value of a specific GPIO pin.
 *        Only applicable if the pin is configured as output.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 */
void GPIO_Value_Tog(tport port, tpin pin);

/**
 * @brief Gets the current direction (mode) of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @return The direction of the pin (input or output).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/**
 * @brief Initializes a specific GPIO pin as output.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @param value: The initial output value (non-zero for high, zero for low).
 *               The actual bit representing the level is checked using value & (1 << pin).
 * @param usage: The intended usage (e.g., general_usage, communication_usage).
 *               Mapped to output type (Push-pull/Open-drain).
 * @param conn: The output connection type (e.g., push_pull, open_drain).
 *              Note: The implementation maps 'usage' to OTYPER, not 'conn' directly as per instructions.
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/**
 * @brief Initializes a specific GPIO pin as input.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (e.g., Pin_0 to Pin_15).
 * @param usage: The intended usage (e.g., general_usage, communication_usage).
 *               Mapped to output type (Push-pull/Open-drain) in a seemingly incorrect way for input pins.
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage);

#endif /* STM32F401RC_GPIO_H */