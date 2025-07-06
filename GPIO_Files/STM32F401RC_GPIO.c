/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.c
* Description    : Generic MCAL GPIO driver implementation for STM32F401RC
* Author         : Technology Inovation Software Team (Implementation based on user requirements)
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F401RC_GPIO.h"
#include "WDT.h" // Assuming WDT_Reset() is defined here

/* Standard integer types */
#include <stdint.h>

/**Static Variables ====================================================================*/
/* No static variables required by the prompt */

/**Functions ===========================================================================*/

/*
 * Note on Register Mapping:
 * The prompt uses generic register names (Px, PMx, PUx, POMx) with specific bit-level logic.
 * This implementation maps them to the actual STM32F401RC registers as follows:
 * - Px (for setting/toggling) -> ODR (Output Data Register)
 * - Px (for getting)         -> IDR (Input Data Register)
 * - PMx (for direction)      -> MODER (Mode Register) - Note: MODER uses 2 bits per pin, mapping is handled in functions.
 *   (PMx bit 0 = Output, 1 = Input based on prompt logic)
 * - PUx (for pull-up)        -> PUPDR (Pull-up/Pull-down Register) - Note: PUPDR uses 2 bits per pin, mapping is handled.
 *   (PUx bit 0 = No pull, 1 = Pull-up based on prompt logic)
 * - POMx (for output connection/type) -> OTYPER (Output Type Register) - Note: OTYPER uses 1 bit per pin.
 *   (POMx bit 0 = Push-pull, 1 = Open-drain based on prompt logic related to usage)
 *
 * Note on bit manipulation macros:
 * SET_BIT(REG, BIT) -> REG |= (1U << (BIT))
 * CLR_BIT(REG, BIT) -> REG &= ~(1U << (BIT))
 * TOG_BIT(REG, BIT) -> REG ^= (1U << (BIT))
 * GET_BIT(REG, BIT) -> ((REG >> (BIT)) & 1U)
 * These macros are assumed to be defined in STM32F401RC_GPIO.h or included by it.
 *
 * Note on MODER/PUPDR manipulation:
 * Since MODER and PUPDR use 2 bits per pin, setting/clearing a 'bit' in the fictional PMx/PUx
 * requires clearing the 2 relevant bits and then setting the correct bit(s) for the desired mode (00, 01).
 * The prompt's logic is implemented by mapping the single-bit PMx/PUx requirement to the 2-bit STM32 registers.
 *
 * Note on OTYPER manipulation:
 * OTYPER uses 1 bit per pin, which matches the fictional POMx structure.
 *
 * Note on do-while loop in Init functions:
 * The requirement to use a do-while loop checking the direction *after* attempting to set it
 *
 * Note on t_output_conn parameter:
 * logic description provided by the prompt. The logic uses `t_usage` to control `POMx` (OTYPER).
 */

/* Implement the following functions using MCAL style, macros, and MCU logic awareness: */

/**
 * @brief Sets the value of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_A, Port_B).
 * @param pin: The pin number (0-15).
 * @param value: A byte where the bit corresponding to 'pin' determines the desired state (non-zero for high, zero for low).
 *               Logic High is determined by (value & (1 << pin)) being non-zero.
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();

    switch (port)
    {
    case Port_A:
        if (value & (1U << pin))
        {
            SET_BIT(GPIOA->ODR, pin); // Map Px for setting to ODR
        }
        else
        {
            CLR_BIT(GPIOA->ODR, pin); // Map Px for setting to ODR
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
    case Port_H: // STM32F401RC has Port H
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
        /* Handle unknown port */
        break;
    }
}

/**
 * @brief Gets the current value of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_0, Port_1).
 * @param pin: The pin number (0-15).
 * @return tbyte: The state of the pin (1 if high, 0 if low).
 */
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();

    tbyte pin_value = 0; // Default value in case of unknown port

    switch (port)
    {
    case Port_A:
        pin_value = (tbyte)GET_BIT(GPIOA->IDR, pin); // Map Px for getting to IDR
        break;
    case Port_B:
        pin_value = (tbyte)GET_BIT(GPIOB->IDR, pin);
        break;
    case Port_C:
        pin_value = (tbyte)GET_BIT(GPIOC->IDR, pin);
        break;
    case Port_D:
        pin_value = (tbyte)GET_BIT(GPIOD->IDR, pin);
        break;
    case Port_E:
        pin_value = (tbyte)GET_BIT(GPIOE->IDR, pin);
        break;
    case Port_H: // STM32F401RC has Port H
        pin_value = (tbyte)GET_BIT(GPIOH->IDR, pin);
        break;
    default:
        /* Handle unknown port, pin_value remains 0 */
        break;
    }

    return pin_value;
}

/**
 * @brief Toggles the value of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_0, Port_1).
 * @param pin: The pin number (0-15).
 */
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();

    switch (port)
    {
    case Port_A:
        TOG_BIT(GPIOA->ODR, pin); // Map Px for toggling to ODR
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
    case Port_H: // STM32F401RC has Port H
        TOG_BIT(GPIOH->ODR, pin);
        break;
    default:
        /* Handle unknown port */
        break;
    }
}

/**
 * @brief Gets the current direction configuration of a specific GPIO pin.
 * @param port: The GPIO port (e.g., Port_0, Port_1).
 * @param pin: The pin number (0-15).
 * @return t_direction: The direction of the pin (input or output).
 */
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();

    t_direction direction = input; // Default to input in case of unknown port

    uint32_t moder_value = 0;
    uint32_t pin_mode_bits = 0;

    switch (port)
    {
    case Port_A:
        moder_value = GPIOA->MODER;
        break;
    case Port_B:
        moder_value = GPIOB->MODER;
        break;
    case Port_C:
        moder_value = GPIOC->MODER;
        break;
    case Port_D:
        moder_value = GPIOD->MODER;
        break;
    case Port_E:
        moder_value = GPIOE->MODER;
        break;
    case Port_H: // STM32F401RC has Port H
        moder_value = GPIOH->MODER;
        break;
    default:
        /* Handle unknown port, direction remains input */
        return direction;
    }

    // Get the 2 mode bits for the specific pin
    pin_mode_bits = (moder_value >> (2U * pin)) & 0x03U;

    // Map STM32 MODER bits to fictional PMx bit (0=Output, 1=Input)
    // MODER 00 (Input) -> PMx 1 (Input)
    // MODER 01 (Output) -> PMx 0 (Output)
    // MODER 10 (Alt Fun) -> Consider as not input/output for this mapping? Default to input? Let's map 00->input, 01->output.
    // MODER 11 (Analog)  -> Consider as not input/output? Default to input?

    if (pin_mode_bits == 0x01U) // 01: General purpose output mode
    {
        direction = output;
    }
    else // 00: Input mode (also 10, 11 will fall here, mapping them to input as per the fictional PMx logic)
    {
        direction = input;
    }

    return direction;
}

/**
 * @brief Initializes a specific GPIO pin as an output.
 * @param port: The GPIO port (e.g., Port_0, Port_1).
 * @param pin: The pin number (0-15).
 * @param value: The initial value to set on the output pin (uses bit corresponding to 'pin').
 * @param usage: The usage type (e.g., communication_usage).
 * @param conn: The output connection type (parameter required by signature, but not used in logic per prompt).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    WDT_Reset();

    uint32_t pin_mask = (1U << pin);
    uint32_t pin_mode_shift = (2U * pin);
    uint32_t pin_pull_shift = (2U * pin);

    // Attempt to set direction, then check if it became output
    do
    {
        switch (port)
        {
        case Port_A:
            // CLR_BIT(PUx, pin); // Disable pull-up -> Map to PUPDR 00 (No pull-up/down)
            GPIOA->PUPDR &= ~(0x03U << pin_pull_shift);

            // if(value & (1 << pin)) -> SET_BIT(Px, pin); else -> CLR_BIT(Px, pin); // Set initial value
            if (value & pin_mask)
            {
                SET_BIT(GPIOA->ODR, pin);
            }
            else
            {
                CLR_BIT(GPIOA->ODR, pin);
            }

            // if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin); // Set output type (Open-drain/Push-pull)
            // Map POMx to OTYPER (1=Open-drain, 0=Push-pull)
            if (usage == communication_usage)
            {
                SET_BIT(GPIOA->OTYPER, pin); // Set bit for Open-drain
            }
            else
            {
                CLR_BIT(GPIOA->OTYPER, pin); // Clear bit for Push-pull
            }

            // CLR_BIT(PMx, pin); // Set as output -> Map to MODER 01 (General purpose output mode)
            GPIOA->MODER &= ~(0x03U << pin_mode_shift); // Clear mode bits
            GPIOA->MODER |= (0x01U << pin_mode_shift);  // Set output mode (01)
            break;

        case Port_B:
            GPIOB->PUPDR &= ~(0x03U << pin_pull_shift);
            if (value & pin_mask) { SET_BIT(GPIOB->ODR, pin); } else { CLR_BIT(GPIOB->ODR, pin); }
            if (usage == communication_usage) { SET_BIT(GPIOB->OTYPER, pin); } else { CLR_BIT(GPIOB->OTYPER, pin); }
            GPIOB->MODER &= ~(0x03U << pin_mode_shift); GPIOB->MODER |= (0x01U << pin_mode_shift);
            break;

        case Port_C:
            GPIOC->PUPDR &= ~(0x03U << pin_pull_shift);
            if (value & pin_mask) { SET_BIT(GPIOC->ODR, pin); } else { CLR_BIT(GPIOC->ODR, pin); }
            if (usage == communication_usage) { SET_BIT(GPIOC->OTYPER, pin); } else { CLR_BIT(GPIOC->OTYPER, pin); }
            GPIOC->MODER &= ~(0x03U << pin_mode_shift); GPIOC->MODER |= (0x01U << pin_mode_shift);
            break;

        case Port_D:
            GPIOD->PUPDR &= ~(0x03U << pin_pull_shift);
            if (value & pin_mask) { SET_BIT(GPIOD->ODR, pin); } else { CLR_BIT(GPIOD->ODR, pin); }
            if (usage == communication_usage) { SET_BIT(GPIOD->OTYPER, pin); } else { CLR_BIT(GPIOD->OTYPER, pin); }
            GPIOD->MODER &= ~(0x03U << pin_mode_shift); GPIOD->MODER |= (0x01U << pin_mode_shift);
            break;

        case Port_E:
            GPIOE->PUPDR &= ~(0x03U << pin_pull_shift);
            if (value & pin_mask) { SET_BIT(GPIOE->ODR, pin); } else { CLR_BIT(GPIOE->ODR, pin); }
            if (usage == communication_usage) { SET_BIT(GPIOE->OTYPER, pin); } else { CLR_BIT(GPIOE->OTYPER, pin); }
            GPIOE->MODER &= ~(0x03U << pin_mode_shift); GPIOE->MODER |= (0x01U << pin_mode_shift);
            break;

        case Port_H: // STM32F401RC has Port H
            GPIOH->PUPDR &= ~(0x03U << pin_pull_shift);
            if (value & pin_mask) { SET_BIT(GPIOH->ODR, pin); } else { CLR_BIT(GPIOH->ODR, pin); }
            if (usage == communication_usage) { SET_BIT(GPIOH->OTYPER, pin); } else { CLR_BIT(GPIOH->OTYPER, pin); }
            GPIOH->MODER &= ~(0x03U << pin_mode_shift); GPIOH->MODER |= (0x01U << pin_mode_shift);
            break;

        default:
            /* Handle unknown port */
            break;
        }
    } while (GPIO_Direction_get(port, pin) != output);
}

/**
 * @brief Initializes a specific GPIO pin as an input.
 * @param port: The GPIO port (e.g., Port_0, Port_1).
 * @param pin: The pin number (0-15).
 * @param usage: The usage type (e.g., communication_usage).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage)
{
    WDT_Reset();

    uint32_t pin_mode_shift = (2U * pin);
    uint32_t pin_pull_shift = (2U * pin);

    // Attempt to set direction, then check if it became input
    do
    {
        switch (port)
        {
        case Port_A:
            // SET_BIT(PUx, pin); // Enable pull-up -> Map to PUPDR 01 (Pull-up mode)
            GPIOA->PUPDR &= ~(0x03U << pin_pull_shift); // Clear pull bits
            GPIOA->PUPDR |= (0x01U << pin_pull_shift);  // Set pull-up mode (01)

            // Map POMx to OTYPER (1=Open-drain, 0=Push-pull)
            if (usage == communication_usage)
            {
                SET_BIT(GPIOA->OTYPER, pin); // Set bit for Open-drain (config applied even in input mode)
            }
            else
            {
                CLR_BIT(GPIOA->OTYPER, pin); // Clear bit for Push-pull (config applied even in input mode)
            }

            // SET_BIT(PMx, pin); // Set as input -> Map to MODER 00 (Input mode)
            GPIOA->MODER &= ~(0x03U << pin_mode_shift); // Clear mode bits (sets to 00 - Input mode)
            break;

        case Port_B:
            GPIOB->PUPDR &= ~(0x03U << pin_pull_shift); GPIOB->PUPDR |= (0x01U << pin_pull_shift);
            if (usage == communication_usage) { SET_BIT(GPIOB->OTYPER, pin); } else { CLR_BIT(GPIOB->OTYPER, pin); }
            GPIOB->MODER &= ~(0x03U << pin_mode_shift);
            break;

        case Port_C:
            GPIOC->PUPDR &= ~(0x03U << pin_pull_shift); GPIOC->PUPDR |= (0x01U << pin_pull_shift);
            if (usage == communication_usage) { SET_BIT(GPIOC->OTYPER, pin); } else { CLR_BIT(GPIOC->OTYPER, pin); }
            GPIOC->MODER &= ~(0x03U << pin_mode_shift);
            break;

        case Port_D:
            GPIOD->PUPDR &= ~(0x03U << pin_pull_shift); GPIOD->PUPDR |= (0x01U << pin_pull_shift);
            if (usage == communication_usage) { SET_BIT(GPIOD->OTYPER, pin); } else { CLR_BIT(GPIOD->OTYPER, pin); }
            GPIOD->MODER &= ~(0x03U << pin_mode_shift);
            break;

        case Port_E:
            GPIOE->PUPDR &= ~(0x03U << pin_pull_shift); GPIOE->PUPDR |= (0x01U << pin_pull_shift);
            if (usage == communication_usage) { SET_BIT(GPIOE->OTYPER, pin); } else { CLR_BIT(GPIOE->OTYPER, pin); }
            GPIOE->MODER &= ~(0x03U << pin_mode_shift);
            break;

        case Port_H: // STM32F401RC has Port H
            GPIOH->PUPDR &= ~(0x03U << pin_pull_shift); GPIOH->PUPDR |= (0x01U << pin_pull_shift);
            if (usage == communication_usage) { SET_BIT(GPIOH->OTYPER, pin); } else { CLR_BIT(GPIOH->OTYPER, pin); }
            GPIOH->MODER &= ~(0x03U << pin_mode_shift);
            break;

        default:
            /* Handle unknown port */
            break;
        }
    } while (GPIO_Direction_get(port, pin) != input);
}


/**
 * @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
 */

/*
 * --- Placeholder Definitions (replace with actual includes and definitions from your project) ---
 * These would typically be in STM32F401RC_GPIO.h or system headers.
 * Added here only for completeness and context if STM32F401RC_GPIO.h is not available.
 */
#if !defined(STM32F401RC_GPIO_H)
#define STM32F401RC_GPIO_H

#include <stdint.h> // For uint32_t, etc.

/* Basic Type Definitions */
typedef uint8_t tbyte;
typedef uint8_t tpin;

/* Enum for Ports (adjust based on actual MCU ports) */
typedef enum
{
    Port_A,
    Port_B,
    Port_C,
    Port_D,
    Port_E,
    Port_H, // STM32F401RC has A, B, C, D, E, H
    NUM_PORTS
} tport;

/* Enum for Direction */
typedef enum
{
    input,
    output
} t_direction;

/* Enum for Usage */
typedef enum
{
    general_usage,
    communication_usage
} t_usage;

/* Enum for Output Connection (as required by signature, logic unused per prompt) */
typedef enum
{
    push_pull,
    open_drain // Matches OTYPER logic
} t_output_conn;


/* --- Placeholder: GPIO Peripheral Register Structure --- */
// This is a simplified structure mimicking the relevant registers.
// Replace with actual CMSIS definitions (e.g., from stm32f4xx.h)
typedef struct
{
    volatile uint32_t MODER;   /*!< GPIO port mode register,                     Address offset: 0x00 */
    volatile uint32_t OTYPER;  /*!< GPIO port output type register,              Address offset: 0x04 */
    volatile uint32_t OSPEEDR; /*!< GPIO port output speed register,             Address offset: 0x08 */
    volatile uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C */
    volatile uint32_t IDR;     /*!< GPIO port input data register,               Address offset: 0x10 */
    volatile uint32_t ODR;     /*!< GPIO port output data register,              Address offset: 0x14 */
    volatile uint32_t BSRR;    /*!< GPIO port bit set/reset register,            Address offset: 0x18 */
    volatile uint32_t LCKR;    /*!< GPIO port configuration lock register,       Address offset: 0x1C */
    volatile uint32_t AFR[2];  /*!< GPIO alternate function registers,           Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/* --- Placeholder: Peripheral Base Addresses --- */
// Replace with actual base addresses from linker script or device header
#define GPIOA_BASE        (0x40020000U)
#define GPIOB_BASE        (0x40020400U)
#define GPIOC_BASE        (0x40020800U)
#define GPIOD_BASE        (0x40020C00U)
#define GPIOE_BASE        (0x40021000U)
#define GPIOH_BASE        (0x40021C00U) // Port H address for F401

/* --- Placeholder: Peripheral Pointers --- */
#define GPIOA             ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB             ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC             ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD             ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE             ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH             ((GPIO_TypeDef *) GPIOH_BASE)


/* --- Placeholder: Bit Manipulation Macros --- */
#define SET_BIT(REG, BIT)       ((REG) |= (1U << (BIT)))
#define CLR_BIT(REG, BIT)       ((REG) &= ~(1U << (BIT)))
#define TOG_BIT(REG, BIT)       ((REG) ^= (1U << (BIT)))
#define GET_BIT(REG, BIT)       (((REG) >> (BIT)) & 1U)

#endif /* STM32F401RC_GPIO_H */

/*
 * --- Placeholder Definition for WDT_Reset() ---
 * This function is assumed to be defined elsewhere, e.g., in WDT.h and WDT.c
 */
#if !defined(WDT_H)
#define WDT_H
void WDT_Reset(void); // Simple declaration
#endif // WDT_H

/*
 * Dummy WDT_Reset implementation if not provided elsewhere, for compilation purposes.
 * REMOVE this if you have a real WDT driver.
 */
#ifndef HAS_REAL_WDT
void WDT_Reset(void)
{
    // Dummy implementation: do nothing or print a message
    // This should be replaced by actual WDT reset logic
}
#endif // HAS_REAL_WDT