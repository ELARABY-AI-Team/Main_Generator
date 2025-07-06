/***********************************************************************************************************************
* File Name      : STM32F410RC_GPIO.c
* Description    : Generic MCAL GPIO driver implementation for STM32F410RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F410RC_GPIO.h" // Assumed to define types, macros, register maps, and WDT_Reset()
#include <stdint.h>           // For standard integer types (uint32_t etc.)

// Assume WDT_Reset() function is defined elsewhere and linked
// void WDT_Reset(void);

// Assume the following types and enums are defined in "STM32F410RC_GPIO.h":
// typedef unsigned char tbyte;
// typedef enum { Port_0, Port_1, Port_2, /* Add other ports if needed */ Invalid_Port } tport; // Mapping to GPIOA, GPIOB, GPIOC...
// typedef uint8_t tpin; // 0-15
// typedef enum { input, output, alternate_function, analog, unknown_direction } t_direction; // Need states for MODER 00, 01, 10, 11
// typedef enum { general_usage, communication_usage /* Add other usage types */ } t_usage;
// typedef enum { push_pull, open_drain /* Add other connection types */ } t_output_conn; // Not used in function logic description based on prompt

// Assume the following bit manipulation macros are defined in "STM32F410RC_GPIO.h":
// #define SET_BIT(reg, bit)  ((*(volatile uint32_t*)(reg)) |= (1UL << (bit)))
// #define CLR_BIT(reg, bit)  ((*(volatile uint32_t*)(reg)) &= ~(1UL << (bit)))
// #define TOG_BIT(reg, bit)  ((*(volatile uint32_t*)(reg)) ^= (1UL << (bit)))
// #define GET_BIT(reg, bit)  (((*(volatile uint32_t*)(reg)) >> (bit)) & 1UL)

// Define MCU-specific register addresses and offsets for STM32F410RC (typical values, confirm with datasheet)
// These would ideally be in a device-specific header file
#define GPIOA_BASE 0x40020000UL
#define GPIOB_BASE 0x40020400UL
#define GPIOC_BASE 0x40020800UL
// Add other ports if available/supported by the MCU (e.g., GPIOH_BASE 0x40021C00UL)

#define MODER_OFFSET 0x00UL // GPIO port mode register
#define OTYPER_OFFSET 0x04UL // GPIO port output type register
#define OSPEEDR_OFFSET 0x08UL // GPIO port output speed register
#define PUPDR_OFFSET 0x0CUL // GPIO port pull-up/pull-down register
#define IDR_OFFSET 0x10UL // GPIO port input data register
#define ODR_OFFSET 0x14UL // GPIO port output data register
#define BSRR_OFFSET 0x18UL // GPIO port bit set/reset register (Note: Prompt requests SET_BIT/CLR_BIT on ODR)
#define LCKR_OFFSET 0x1CUL // GPIO port configuration lock register
#define AFR_OFFSET 0x20UL // GPIO alternate function registers (AFR[0] and AFR[1])


/**Static Variables ====================================================================*/
// No static variables required by the prompt

/**Functions ===========================================================================*/

// Helper function to get GPIO base address based on port enum
// Returns NULL for invalid port
static volatile uint32_t* GPIO_Get_Base_Addr(tport port)
{
    switch (port)
    {
        case Port_0: return (volatile uint32_t*)GPIOA_BASE;
        case Port_1: return (volatile uint32_t*)GPIOB_BASE;
        case Port_2: return (volatile uint32_t*)GPIOC_BASE;
        // Add cases for other supported ports (Port_3 etc.)
        default: return (volatile uint32_t*)0; // Indicate invalid port
    }
}


/* Implement the following functions using MCAL style, macros, and MCU logic awareness: */

/**
 * @brief Sets the logic level of a specific GPIO pin.
 *        Uses the ODR register and standard bit manipulation macros.
 *        Does not assume 1 = HIGH; uses value & (1 << pin) to determine logic.
 *
 * @param port: The GPIO port (e.g., Port_0 for GPIOA).
 * @param pin: The pin number (0-15).
 * @param value: A tbyte where the relevant bit (1 << pin) indicates the desired state.
 *               e.g., if pin is 5, value=0xFF sets high, value=0x00 sets low.
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset(); // Kick the watchdog

    volatile uint32_t* port_base = GPIO_Get_Base_Addr(port);

    if (port_base != (volatile uint32_t*)0 && pin < 16) // Validate port and pin
    {
        volatile uint32_t* odr_addr = port_base + ODR_OFFSET;

        // Check the specific bit corresponding to the pin in the 'value' byte
        if (value & (1 << pin))
        {
            SET_BIT(odr_addr, pin);
        }
        else
        {
            CLR_BIT(odr_addr, pin);
        }
    }
    else
    {
        // Handle invalid port or pin, e.g., log an error or assert
    }
}

/**
 * @brief Gets the current logic level of a specific GPIO pin from the Input Data Register (IDR).
 *
 * @param port: The GPIO port (e.g., Port_0 for GPIOA).
 * @param pin: The pin number (0-15).
 * @return tbyte: The state of the pin (0 or 1). Returns 0 for invalid port/pin.
 */
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset(); // Kick the watchdog

    volatile uint32_t* port_base = GPIO_Get_Base_Addr(port);
    tbyte pin_value = 0; // Default to low or error value

    if (port_base != (volatile uint32_t*)0 && pin < 16) // Validate port and pin
    {
         volatile uint32_t* idr_addr = port_base + IDR_OFFSET;
         pin_value = (tbyte)GET_BIT(idr_addr, pin);
    }
    else
    {
        // Handle invalid port or pin
    }

    return pin_value;
}

/**
 * @brief Toggles the current logic level of a specific GPIO pin in the Output Data Register (ODR).
 *
 * @param port: The GPIO port (e.g., Port_0 for GPIOA).
 * @param pin: The pin number (0-15).
 */
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset(); // Kick the watchdog

    volatile uint32_t* port_base = GPIO_Get_Base_Addr(port);

    if (port_base != (volatile uint32_t*)0 && pin < 16) // Validate port and pin
    {
        volatile uint32_t* odr_addr = port_base + ODR_OFFSET;
        TOG_BIT(odr_addr, pin);
    }
    else
    {
        // Handle invalid port or pin
    }
}

/**
 * @brief Gets the current direction configuration of a specific GPIO pin from the Mode Register (MODER).
 *        Interprets the 2-bit mode field for the pin.
 *
 * @param port: The GPIO port (e.g., Port_0 for GPIOA).
 * @param pin: The pin number (0-15).
 * @return t_direction: The configured direction (input, output, alternate_function, analog, or unknown_direction).
 */
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset(); // Kick the watchdog

    volatile uint32_t* port_base = GPIO_Get_Base_Addr(port);
    t_direction direction = unknown_direction; // Default or error value

    if (port_base != (volatile uint32_t*)0 && pin < 16) // Validate port and pin
    {
        volatile uint32_t* moder_addr = port_base + MODER_OFFSET;
        uint32_t mode_field = (*moder_addr >> (pin * 2)) & 0b11; // Extract 2-bit mode field

        // Map 2-bit mode field to t_direction enum (STM32 MODER: 00=Input, 01=Output, 10=AF, 11=Analog)
        switch (mode_field)
        {
            case 0b00: direction = input; break;
            case 0b01: direction = output; break;
            case 0b10: direction = alternate_function; break;
            case 0b11: direction = analog; break;
            default:   direction = unknown_direction; break; // Should not happen with 2 bits
        }
    }
    else
    {
        // Handle invalid port or pin
    }

    return direction;
}


/**
 * @brief Initializes a specific GPIO pin as an output.
 *        Configures pull-up (disables), initial output value, output type (push-pull/open-drain via usage/POMx), and sets mode to Output.
 *
 * @param port: The GPIO port (e.g., Port_0 for GPIOA).
 * @param pin: The pin number (0-15).
 * @param value: The initial logic level for the output pin (used with value & (1 << pin)).
 * @param usage: The intended usage (e.g., general_usage, communication_usage) affecting output type.
 *               communication_usage is assumed to configure Open-Drain (OTYPER bit set),
 *               general_usage configures Push-Pull (OTYPER bit cleared).
 * @param conn: The output connection type (push_pull, open_drain). Note: This parameter
 *              is not directly used in the function's logic *as described by the prompt*,
 *              which links usage to POMx. The implementation uses 'usage' to determine OTYPER.
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    // Note: Proper GPIO initialization usually requires enabling the GPIO peripheral clock first.
    // This driver assumes the clock is already enabled or handled externally.

    WDT_Reset(); // Kick the watchdog

    volatile uint32_t* port_base = GPIO_Get_Base_Addr(port);

    if (port_base != (volatile uint32_t*)0 && pin < 16) // Validate port and pin
    {
        volatile uint32_t* moder_addr = port_base + MODER_OFFSET;
        volatile uint32_t* otyper_addr = port_base + OTYPER_OFFSET;
        volatile uint32_t* pupdr_addr = port_base + PUPDR_OFFSET;
        volatile uint32_t* odr_addr = port_base + ODR_OFFSET;

        // Implement initialization steps as described in the prompt *within the do-while loop*
        // The do-while loop waiting for direction change is unusual but requested.
        do
        {
            // CLR_BIT(PUx, pin); // Disable pull-up/pull-down -> Set PUPDR to 00 (No Pull-up/Pull-down)
            // PUPDR uses 2 bits per pin. Need to clear the 2 bits for the pin.
            *pupdr_addr &= ~(0b11UL << (pin * 2)); // Clear PUPDR[2*pin+1:2*pin]

            // if(value & (1 << pin)) -> SET_BIT(Px, pin); else -> CLR_BIT(Px, pin); // Set initial value in ODR
            // Px refers to the Output Data Register (ODR) here
            if (value & (1 << pin))
            {
                SET_BIT(odr_addr, pin);
            }
            else
            {
                CLR_BIT(odr_addr, pin);
            }

            // if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin);
            // Assuming POMx maps to OTYPER (Output Type Register), where bit 1 means Open-Drain, 0 means Push-Pull.
            if (usage == communication_usage)
            {
                // Configure as Open-Drain (set OTYPER bit)
                SET_BIT(otyper_addr, pin);
            }
            else // Assuming general_usage or other implies Push-Pull
            {
                // Configure as Push-Pull (clear OTYPER bit)
                CLR_BIT(otyper_addr, pin);
            }
            // Note: The 'conn' parameter (push_pull/open_drain) is redundant if 'usage' also controls OTYPER.
            // The prompt logic only mentions 'usage'.

            // CLR_BIT(PMx, pin); // Set as output
            // PMx refers to the Mode Register (MODER). Output mode is 01 (b01).
            // MODER uses 2 bits per pin. Need to set MODER[2*pin+1:2*pin] to 01.
            *moder_addr &= ~(0b11UL << (pin * 2)); // Clear MODER[2*pin+1:2*pin]
            *moder_addr |= (0b01UL << (pin * 2));  // Set MODER[2*pin+1:2*pin] to 01 (Output mode)

        } while (GPIO_Direction_get(port, pin) != output); // Busy-wait until direction is reported as output
    }
    else
    {
        // Handle invalid port or pin
    }
}

/**
 * @brief Initializes a specific GPIO pin as an input.
 *        Configures pull-up (enables), output type (via usage/POMx - less relevant for input but included as per prompt logic), and sets mode to Input.
 *
 * @param port: The GPIO port (e.g., Port_0 for GPIOA).
 * @param pin: The pin number (0-15).
 * @param usage: The intended usage (e.g., general_usage, communication_usage) potentially affecting output type config
 *               (though less relevant for pure input). Included as per prompt logic linking usage to POMx/OTYPER.
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage)
{
     // Note: Proper GPIO initialization usually requires enabling the GPIO peripheral clock first.
    // This driver assumes the clock is already enabled or handled externally.

    WDT_Reset(); // Kick the watchdog

    volatile uint32_t* port_base = GPIO_Get_Base_Addr(port);

    if (port_base != (volatile uint32_t*)0 && pin < 16) // Validate port and pin
    {
        volatile uint32_t* moder_addr = port_base + MODER_OFFSET;
        volatile uint32_t* otyper_addr = port_base + OTYPER_OFFSET; // OTYPER config included as per prompt linking usage to POMx
        volatile uint32_t* pupdr_addr = port_base + PUPDR_OFFSET;

        // Implement initialization steps as described in the prompt *within the do-while loop*
        // The do-while loop waiting for direction change is unusual but requested.
        do
        {
            // SET_BIT(PUx, pin); // Enable pull-up -> Set PUPDR to 01 (Pull-up)
            // PUPDR uses 2 bits per pin. Need to set PUPDR[2*pin+1:2*pin] to 01.
            *pupdr_addr &= ~(0b11UL << (pin * 2)); // Clear PUPDR[2*pin+1:2*pin]
            *pupdr_addr |= (0b01UL << (pin * 2));  // Set PUPDR[2*pin+1:2*pin] to 01 (Pull-up mode)

            // if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin);
            // Assuming POMx maps to OTYPER. While OTYPER is primarily for outputs,
            // the prompt's logic includes this for input init based on 'usage'.
             if (usage == communication_usage)
            {
                // Configure as Open-Drain (set OTYPER bit) - Unusual for input, but following prompt logic
                SET_BIT(otyper_addr, pin);
            }
            else // Assuming general_usage or other implies Push-Pull
            {
                // Configure as Push-Pull (clear OTYPER bit) - Default value, less impact on input
                CLR_BIT(otyper_addr, pin);
            }

            // SET_BIT(PMx, pin); // Set as input
            // PMx refers to the Mode Register (MODER). Input mode is 00 (b00).
            // MODER uses 2 bits per pin. Need to set MODER[2*pin+1:2*pin] to 00.
            *moder_addr &= ~(0b11UL << (pin * 2)); // Clear MODER[2*pin+1:2*pin] (This effectively sets it to 00 for Input)

        } while (GPIO_Direction_get(port, pin) != input); // Busy-wait until direction is reported as input
    }
    else
    {
        // Handle invalid port or pin
    }
}

/***********************************************************************************************************************
* End of File
***********************************************************************************************************************/