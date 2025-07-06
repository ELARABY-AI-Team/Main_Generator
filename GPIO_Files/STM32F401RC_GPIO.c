/***********************************************************************************************************************
* File Name      : STM32F410RC_GPIO.c
* Description    : MCAL GPIO driver implementation for STM32F410RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "STM32F410RC_GPIO.h"
// Assuming WDT_Reset() is defined in a global utility or WDT driver header
// If not, provide a placeholder or include the correct header
// #include "WDT.h" // Example include for WDT

/**Static Variables ====================================================================*/

/**Functions ===========================================================================*/

/* Helper macro to get the base address of a GPIO peripheral based on port enum */
#define GET_GPIO_PORT_BASE(port) \
    ((port) == Port_0 ? GPIOA_BASE_ADDRESS : \
     (port) == Port_1 ? GPIOB_BASE_ADDRESS : \
     (port) == Port_2 ? GPIOC_BASE_ADDRESS : \
     (port) == Port_3 ? GPIOH_BASE_ADDRESS : \
     0) // Should not happen with valid port enum

/* Placeholder for WDT_Reset() if not defined elsewhere */
#ifndef WDT_Reset
#define WDT_Reset() ((void)0) // Define as empty if not provided externally
#endif

/***********************************************************************************************************************
* Function Name: GPIO_Value_Set
* Description  : Sets or Clears the output value of a specific GPIO pin.
* Arguments    : port - The GPIO port (e.g., Port_0 for GPIOA)
*              : pin  - The pin number within the port (0-15)
*              : value - The desired value byte. The bit corresponding to 'pin' determines the output level.
* Return Value : None
***********************************************************************************************************************/
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset();

    /* Get the base address of the ODR register for the specified port */
    volatile uint32_t* port_odr_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_ODR_OFFSET);

    switch(port)
    {
        case Port_0: /* GPIOA */
        case Port_1: /* GPIOB */
        case Port_2: /* GPIOC */
        case Port_3: /* GPIOH */
            /* Check the bit corresponding to the pin in the value byte */
            if (value & (1 << pin))
            {
                SET_BIT(*port_odr_address, pin);
            }
            else
            {
                CLR_BIT(*port_odr_address, pin);
            }
            break;

        default:
            /* Handle invalid port */
            break;
    }
}

/***********************************************************************************************************************
* Function Name: GPIO_Value_Get
* Description  : Reads the input value of a specific GPIO pin.
* Arguments    : port - The GPIO port (e.g., Port_0 for GPIOA)
*              : pin  - The pin number within the port (0-15)
* Return Value : tbyte - The value of the pin (0 or 1)
***********************************************************************************************************************/
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset();

    /* Get the base address of the IDR register for the specified port */
    volatile uint32_t* port_idr_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_IDR_OFFSET);
    tbyte pin_value = 0; // Default return value

    switch(port)
    {
        case Port_0: /* GPIOA */
        case Port_1: /* GPIOB */
        case Port_2: /* GPIOC */
        case Port_3: /* GPIOH */
            pin_value = GET_BIT(*port_idr_address, pin);
            break;

        default:
            /* Handle invalid port */
            break;
    }
    return pin_value;
}

/***********************************************************************************************************************
* Function Name: GPIO_Value_Tog
* Description  : Toggles the output value of a specific GPIO pin.
* Arguments    : port - The GPIO port (e.g., Port_0 for GPIOA)
*              : pin  - The pin number within the port (0-15)
* Return Value : None
***********************************************************************************************************************/
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset();

    /* Get the base address of the ODR register for the specified port */
    volatile uint32_t* port_odr_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_ODR_OFFSET);

    switch(port)
    {
        case Port_0: /* GPIOA */
        case Port_1: /* GPIOB */
        case Port_2: /* GPIOC */
        case Port_3: /* GPIOH */
            TOG_BIT(*port_odr_address, pin);
            break;

        default:
            /* Handle invalid port */
            break;
    }
}

/***********************************************************************************************************************
* Function Name: GPIO_Direction_get
* Description  : Gets the direction (Input/Output) of a specific GPIO pin.
* Arguments    : port - The GPIO port (e.g., Port_0 for GPIOA)
*              : pin  - The pin number within the port (0-15)
* Return Value : t_direction - The direction of the pin (input or output).
*                Note: Returns input for any mode other than General Purpose Output (MODER = 0b01).
***********************************************************************************************************************/
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset();

    /* Get the base address of the MODER register for the specified port */
    volatile uint32_t* port_moder_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_MODER_OFFSET);
    t_direction direction = input; // Default return value (input mode is 00)

    switch(port)
    {
        case Port_0: /* GPIOA */
        case Port_1: /* GPIOB */
        case Port_2: /* GPIOC */
        case Port_3: /* GPIOH */
            /* Read the two bits corresponding to the pin in the MODER register */
            uint32_t moder_bits = (*port_moder_address >> (2 * pin)) & 0x03;

            /* Interpret MODER bits: 00=Input, 01=Output, 10=Alternate, 11=Analog */
            /* According to request's implied logic (0=output, 1=input for PMx bit): */
            /* Map 0b01 (Output) to 'output' enum, anything else to 'input' enum for this function's purpose. */
            if (moder_bits == 0b01) // General Purpose Output mode
            {
                direction = output;
            }
            else // Input (00), Alternate (10), Analog (11) are treated as 'input' by this function
            {
                 direction = input;
            }
            break;

        default:
            /* Handle invalid port - default direction is input */
            break;
    }
    return direction;
}

/***********************************************************************************************************************
* Function Name: GPIO_Output_Init
* Description  : Initializes a specific GPIO pin as Output.
* Arguments    : port  - The GPIO port (e.g., Port_0 for GPIOA)
*              : pin   - The pin number within the port (0-15)
*              : value - The initial output value byte. The bit corresponding to 'pin' sets the initial level.
*              : usage - The intended usage (general_usage implies push-pull, communication_usage implies open-drain)
*              : conn  - Not used in the requested logic (part of function signature)
* Return Value : None
***********************************************************************************************************************/
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn)
{
    /* Configure the pin and then wait until the direction is successfully set to output */
    do
    {
        WDT_Reset(); // Reset WDT inside the loop

        switch(port)
        {
            case Port_0: /* GPIOA */
            case Port_1: /* GPIOB */
            case Port_2: /* GPIOC */
            case Port_3: /* GPIOH */
            {
                /* Get base addresses for required registers */
                volatile uint32_t* port_moder_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_MODER_OFFSET);
                volatile uint32_t* port_otyper_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_OTYPER_OFFSET);
                volatile uint32_t* port_odr_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_ODR_OFFSET);
                volatile uint32_t* port_pupdr_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_PUPDR_OFFSET);

                /* 1. CLR_BIT(PUx, pin); // Disable pull-up/pull-down */
                /* Translate to PUPDR register (2 bits per pin): Set bits 2*pin and 2*pin+1 to 00 (No pull-up/down) */
                CLR_BIT(*port_pupdr_address, 2 * pin);
                CLR_BIT(*port_pupdr_address, 2 * pin + 1);

                /* 2. if(value & (1 << pin)) -> SET_BIT(Px, pin); else -> CLR_BIT(Px, pin); // Set initial value */
                /* Translate to ODR register (1 bit per pin) */
                if (value & (1 << pin))
                {
                    SET_BIT(*port_odr_address, pin);
                }
                else
                {
                    CLR_BIT(*port_odr_address, pin);
                }

                /* 3. if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin); // Set output type */
                /* Translate to OTYPER register (1 bit per pin): 0=Push-pull, 1=Open-drain */
                if (usage == communication_usage)
                {
                    SET_BIT(*port_otyper_address, pin); // Set bit to 1 for Open-drain
                }
                else // general_usage or other
                {
                    CLR_BIT(*port_otyper_address, pin); // Clear bit to 0 for Push-pull
                }

                /* 4. CLR_BIT(PMx, pin); // Set as output */
                /* Translate to MODER register (2 bits per pin): Set bits 2*pin and 2*pin+1 to 01 (General Purpose Output mode) */
                CLR_BIT(*port_moder_address, 2 * pin + 1); // Clear bit 2*pin+1 (bit 1 of the pair)
                SET_BIT(*port_moder_address, 2 * pin);     // Set bit 2*pin (bit 0 of the pair)
                break;
            }

            default:
                /* Handle invalid port */
                break;
        }
    }
    while(GPIO_Direction_get(port, pin) != output); // Wait until the direction is confirmed as output
}

/***********************************************************************************************************************
* Function Name: GPIO_Input_Init
* Description  : Initializes a specific GPIO pin as Input.
* Arguments    : port  - The GPIO port (e.g., Port_0 for GPIOA)
*              : pin   - The pin number within the port (0-15)
*              : usage - The intended usage (communication_usage seems to affect output type based on description,
*                        but applied to input init as per request)
* Return Value : None
***********************************************************************************************************************/
void GPIO_Input_Init(tport port, tpin pin, t_usage usage)
{
    /* Configure the pin and then wait until the direction is successfully set to input */
    do
    {
        WDT_Reset(); // Reset WDT inside the loop

        switch(port)
        {
            case Port_0: /* GPIOA */
            case Port_1: /* GPIOB */
            case Port_2: /* GPIOC */
            case Port_3: /* GPIOH */
            {
                /* Get base addresses for required registers */
                volatile uint32_t* port_moder_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_MODER_OFFSET);
                volatile uint32_t* port_otyper_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_OTYPER_OFFSET); // Included as per request logic, though OTYPER is for output
                volatile uint32_t* port_pupdr_address = (volatile uint32_t *)(GET_GPIO_PORT_BASE(port) + GPIO_PUPDR_OFFSET);

                /* 1. SET_BIT(PUx, pin); // Enable pull-up */
                /* Translate to PUPDR register (2 bits per pin): Set bits 2*pin and 2*pin+1 to 01 (Pull-up enabled) */
                CLR_BIT(*port_pupdr_address, 2 * pin + 1); // Clear bit 2*pin+1 (bit 1 of the pair)
                SET_BIT(*port_pupdr_address, 2 * pin);     // Set bit 2*pin (bit 0 of the pair)


                /* 2. if usage == communication_usage -> SET_BIT(POMx, pin); else -> CLR_BIT(POMx, pin); */
                /* Translate to OTYPER register (1 bit per pin): 0=Push-pull, 1=Open-drain */
                /* Note: Configuring OTYPER for an input pin has no effect on pin behavior. Included as per request. */
                if (usage == communication_usage)
                {
                    SET_BIT(*port_otyper_address, pin); // Set bit to 1 for Open-drain (ignored in input mode)
                }
                else // general_usage or other
                {
                    CLR_BIT(*port_otyper_address, pin); // Clear bit to 0 for Push-pull (ignored in input mode)
                }

                /* 3. SET_BIT(PMx, pin); // Set as input */
                /* Translate to MODER register (2 bits per pin): Set bits 2*pin and 2*pin+1 to 00 (Input mode) */
                CLR_BIT(*port_moder_address, 2 * pin);     // Clear bit 2*pin (bit 0 of the pair)
                CLR_BIT(*port_moder_address, 2 * pin + 1); // Clear bit 2*pin+1 (bit 1 of the pair)
                break;
            }

            default:
                /* Handle invalid port */
                break;
        }
    }
    while(GPIO_Direction_get(port, pin) != input); // Wait until the direction is confirmed as input
}

/***********************************************************************************************************************
* End of File
***********************************************************************************************************************/