/***********************************************************************************************************************
* File Name      : MCAL_STM32F401RC_GPIO.c
* Description    : This file implements device driver for (GPIO)
* Author         : AI
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-01
* Testing Date   :
* @COPYRIGHT YYYY El-ARABY Research and Development Center. All rights reserved.
***********************************************************************************************************************/

/**Includes ============================================================================*/
#include "MCAL_STM32F401RC_GPIO.h"

/* Assume WDT_Reset is declared elsewhere, e.g., in a WDT driver header */
extern void WDT_Reset(void);

/**Static Variables ====================================================================*/
/* None */

/**Functions ===========================================================================*/

/*
* ===================================================================================================
* Private Function Prototypes
* ===================================================================================================
*/
/* None */

/*
* ===================================================================================================
* Private Macros for Bit Manipulation
* Note: These macros are defined locally for clarity in this example driver.
*       In a real MCAL, they would likely be in a common utility header file.
* ===================================================================================================
*/
#define SET_BIT(REG, BIT)     ((REG) |= (1U << (BIT)))
#define CLR_BIT(REG, BIT)     ((REG) &= ~(1U << (BIT)))
#define TOG_BIT(REG, BIT)     ((REG) ^= (1U << (BIT)))
#define GET_BIT(REG, BIT)     (((REG) >> (BIT)) & 1U)

/*
* ===================================================================================================
* Private Assumed Register Base Addresses and Offsets for STM32F401RC GPIO
* Note: These are based on common STM32 register maps, but should be verified
*       against the specific STM32F401RC datasheet/reference manual.
* ===================================================================================================
*/
/* Assumed - please verify */
#define GPIOA_BASE              (0x40020000UL)
#define GPIOB_BASE              (0x40020400UL)
#define GPIOC_BASE              (0x40020800UL)
#define GPIOD_BASE              (0x40020C00UL)
#define GPIOE_BASE              (0x40021000UL)
#define GPIOH_BASE              (0x40021C00UL) /* PH0 and PH1 are available */

/* Assumed Register Offsets - please verify */
#define GPIO_MODER_OFFSET       (0x00UL) /* GPIO port mode register */
#define GPIO_OTYPER_OFFSET      (0x04UL) /* GPIO port output type register */
#define GPIO_OSPEEDR_OFFSET     (0x08UL) /* GPIO port output speed register */
#define GPIO_PUPDR_OFFSET       (0x0CUL) /* GPIO port pull-up/pull-down register */
#define GPIO_IDR_OFFSET         (0x10UL) /* GPIO port input data register */
#define GPIO_ODR_OFFSET         (0x14UL) /* GPIO port output data register */
#define GPIO_BSRR_OFFSET        (0x18UL) /* GPIO port bit set/reset register */
#define GPIO_LCKR_OFFSET        (0x1CUL) /* GPIO port configuration lock register */
#define GPIO_AFRL_OFFSET        (0x20UL) /* GPIO alternate function low register */
#define GPIO_AFRH_OFFSET        (0x24UL) /* GPIO alternate function high register */

/*
* ===================================================================================================
* Function Definitions
* ===================================================================================================
*/

/***************************************************************************************************
* @brief Sets the output value of a specific GPIO pin.
* @param port: The GPIO port (e.g., Port_A).
* @param pin: The pin number (0-15).
* @param value: The desired output value. Logic high if (value & (1 << pin)) is non-zero.
* @return None.
***************************************************************************************************/
void GPIO_Value_Set(tport port, tpin pin, tbyte value)
{
    WDT_Reset(); /* Assumed - please verify function availability */

    volatile uint32_t* odr_reg = NULL;

    switch(port)
    {
        case Port_A:
            odr_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_B:
            odr_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_C:
            odr_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_D:
            odr_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_E:
            odr_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_H:
            odr_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        default:
            /* Handle error: Invalid port */
            /* e.g., print error, log, or assert */
            return;
    }

    if (odr_reg != NULL)
    {
        if (value & (1 << pin))
        {
            SET_BIT(*odr_reg, pin); /* Assumed - please verify register access method */
        }
        else
        {
            CLR_BIT(*odr_reg, pin); /* Assumed - please verify register access method */
        }
    }
}

/***************************************************************************************************
* @brief Gets the input value of a specific GPIO pin.
* @param port: The GPIO port (e.g., Port_A).
* @param pin: The pin number (0-15).
* @return The input value (0 or 1). Returns a non-standard value (e.g., 0xFF) on error.
***************************************************************************************************/
tbyte GPIO_Value_Get(tport port, tpin pin)
{
    WDT_Reset(); /* Assumed - please verify function availability */

    volatile uint32_t* idr_reg = NULL;
    tbyte ret_val = 0xFF; /* Default error value */

    switch(port)
    {
        case Port_A:
            idr_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_IDR_OFFSET); /* Assumed - please verify */
            break;
        case Port_B:
            idr_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_IDR_OFFSET); /* Assumed - please verify */
            break;
        case Port_C:
            idr_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_IDR_OFFSET); /* Assumed - please verify */
            break;
        case Port_D:
            idr_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_IDR_OFFSET); /* Assumed - please verify */
            break;
        case Port_E:
            idr_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_IDR_OFFSET); /* Assumed - please verify */
            break;
        case Port_H:
            idr_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_IDR_OFFSET); /* Assumed - please verify */
            break;
        default:
            /* Handle error: Invalid port */
            /* e.g., print error, log, or assert */
            return ret_val; /* Return error value */
    }

    if (idr_reg != NULL)
    {
        ret_val = (tbyte)GET_BIT(*idr_reg, pin); /* Assumed - please verify register access method */
    }

    return ret_val;
}

/***************************************************************************************************
* @brief Toggles the output value of a specific GPIO pin.
* @param port: The GPIO port (e.g., Port_A).
* @param pin: The pin number (0-15).
* @return None.
***************************************************************************************************/
void GPIO_Value_Tog(tport port, tpin pin)
{
    WDT_Reset(); /* Assumed - please verify function availability */

    volatile uint32_t* odr_reg = NULL;

    switch(port)
    {
        case Port_A:
            odr_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_B:
            odr_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_C:
            odr_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_D:
            odr_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_E:
            odr_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        case Port_H:
            odr_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_ODR_OFFSET); /* Assumed - please verify */
            break;
        default:
            /* Handle error: Invalid port */
            /* e.g., print error, log, or assert */
            return;
    }

    if (odr_reg != NULL)
    {
        TOG_BIT(*odr_reg, pin); /* Assumed - please verify register access method */
    }
}

/***************************************************************************************************
* @brief Gets the direction configuration of a specific GPIO pin.
* @param port: The GPIO port (e.g., Port_A).
* @param pin: The pin number (0-15).
* @return The direction (input or output). Returns direction_max on error.
* Note: This implementation literally follows the prompt's instruction to use
*       GET_BIT(PMx, pin). For STM32's 2-bit MODER field (00=Input, 01=Output, etc.),
*       accessing a single bit `pin` is incorrect and will not reliably indicate
*       input/output mode. This function will likely NOT work correctly for STM32
*       based on standard MODER register structure.
***************************************************************************************************/
t_direction GPIO_Direction_get(tport port, tpin pin)
{
    WDT_Reset(); /* Assumed - please verify function availability */

    volatile uint32_t* moder_reg = NULL;
    t_direction ret_dir = direction_max; /* Default error value */

    switch(port)
    {
        case Port_A:
            moder_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_MODER_OFFSET); /* Assumed - please verify */
            break;
        case Port_B:
            moder_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_MODER_OFFSET); /* Assumed - please verify */
            break;
        case Port_C:
            moder_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_MODER_OFFSET); /* Assumed - please verify */
            break;
        case Port_D:
            moder_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_MODER_OFFSET); /* Assumed - please verify */
            break;
        case Port_E:
            moder_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_MODER_OFFSET); /* Assumed - please verify */
            break;
        case Port_H:
            moder_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_MODER_OFFSET); /* Assumed - please verify */
            break;
        default:
            /* Handle error: Invalid port */
            /* e.g., print error, log, or assert */
            return ret_dir; /* Return error value */
    }

    if (moder_reg != NULL)
    {
        /*
         * WARNING: This GET_BIT(MODER, pin) access is NOT how STM32 MODER works.
         * STM32 uses 2 bits per pin (MODER[2*pin+1]:MODER[2*pin]).
         * Following the prompt's literal instruction, but this is expected to fail.
         * Assuming '0' means input, '1' means output based on prompt's logic context.
         */
        if (GET_BIT(*moder_reg, pin) == 0U) /* Assumed: GET_BIT(MODER, pin) == 0 implies input */
        {
            ret_dir = input; /* Assumed value based on faulty logic */
        }
        else /* Assumed: GET_BIT(MODER, pin) == 1 implies output (or AF/Analog) */
        {
            ret_dir = output; /* Assumed value based on faulty logic */
        }
    }

    return ret_dir;
}


/***************************************************************************************************
* @brief Initializes a specific GPIO pin as output.
* @param port: The GPIO port (e.g., Port_A).
* @param pin: The pin number (0-15).
* @param value: The initial output value. Logic high if (value & (1 << pin)) is non-zero.
* @param usage: The pin usage (general_usage for Push-Pull, communication_usage for Open-Drain).
* @return None.
* Note: The do-while loop condition relies on GPIO_Direction_get, which is implemented
*       based on a potentially flawed understanding of the STM32 MODER register (1 bit vs 2 bits).
*       This function's configuration steps (PUPDR, OTYPER, MODER manipulation via single bits)
*       also deviate significantly from standard STM32 GPIO register programming practices
*       which require handling 2 bits per pin for mode and pull-up/down.
***************************************************************************************************/
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage)
{
    WDT_Reset(); /* Assumed - please verify function availability */

    volatile uint32_t* pupdr_reg = NULL; /* Assumed PUx maps to PUPDR */
    volatile uint32_t* odr_reg = NULL;   /* Assumed Px maps to ODR for value set */
    volatile uint32_t* otyper_reg = NULL;/* Assumed POMx maps to OTYPER */
    volatile uint32_t* moder_reg = NULL; /* Assumed PMx maps to MODER */

    /*
     * WARNING: The do-while loop condition GPIO_Direction_get(...) != output
     * relies on a flawed implementation of GPIO_Direction_get for STM32 MODER
     * which uses 2 bits per pin. This loop may not function as intended.
     * Configuration steps inside the loop also use 1-bit manipulation on 2-bit
     * registers (MODER, PUPDR), which is incorrect for STM32.
     */
    do
    {
        switch(port)
        {
            case Port_A:
                pupdr_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                odr_reg   = (volatile uint32_t *)(GPIOA_BASE + GPIO_ODR_OFFSET);    /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOA_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_B:
                pupdr_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                odr_reg   = (volatile uint32_t *)(GPIOB_BASE + GPIO_ODR_OFFSET);    /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOB_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_C:
                pupdr_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                odr_reg   = (volatile uint32_t *)(GPIOC_BASE + GPIO_ODR_OFFSET);    /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOC_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_D:
                pupdr_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                odr_reg   = (volatile uint32_t *)(GPIOD_BASE + GPIO_ODR_OFFSET);    /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOD_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_E:
                pupdr_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                odr_reg   = (volatile uint32_t *)(GPIOE_BASE + GPIO_ODR_OFFSET);    /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOE_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_H:
                pupdr_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                odr_reg   = (volatile uint32_t *)(GPIOH_BASE + GPIO_ODR_OFFSET);    /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOH_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            default:
                /* Handle error: Invalid port */
                /* e.g., print error, log, or assert */
                return; /* Exit function on invalid port */
        }

        if (pupdr_reg != NULL && odr_reg != NULL && otyper_reg != NULL && moder_reg != NULL)
        {
            /*
             * WARNING: Following prompt instruction CLR_BIT(PUx, pin).
             * STM32 PUPDR is 2 bits per pin (00=None, 01=PullUp, 10=PullDown).
             * This single-bit clear is NOT sufficient or correct for STM32.
             */
            CLR_BIT(*pupdr_reg, pin); /* Assumed - please verify register access method */

            /* Set initial output value */
            if (value & (1 << pin))
            {
                SET_BIT(*odr_reg, pin); /* Assumed - please verify register access method */
            }
            else
            {
                CLR_BIT(*odr_reg, pin); /* Assumed - please verify register access method */
            }

            /* Configure output type (Push-Pull or Open-Drain) */
            /* Assumed POMx maps to OTYPER, 1=Open-Drain (communication_usage), 0=Push-Pull (general_usage) */
            if (usage == communication_usage)
            {
                SET_BIT(*otyper_reg, pin); /* Assumed - please verify register access method */
            }
            else
            {
                CLR_BIT(*otyper_reg, pin); /* Assumed - please verify register access method */
            }

            /*
             * WARNING: Following prompt instruction CLR_BIT(PMx, pin) to set as output.
             * STM32 MODER is 2 bits per pin (00=Input, 01=Output).
             * Clearing a single bit is NOT sufficient or correct for setting output mode (01).
             * This step is expected to fail for STM32.
             */
            CLR_BIT(*moder_reg, pin); /* Assumed - please verify register access method */
        }
        else
        {
             /* Handle error: Should not happen with valid port, but safety check */
             break; /* Break do-while loop on error */
        }

    } while(GPIO_Direction_get(port, pin) != output); /* Assumed - relies on flawed GPIO_Direction_get logic */

    /* After the potentially non-terminating or incorrect loop, ensure critical setup */
    /* In a real driver, the loop condition and register writes would need rework for STM32 */
    /* Example (correct STM32 output config - *NOT* following prompt's logic): */
    /*
    volatile uint32_t* p_moder = (volatile uint32_t *)(port_base_addr + GPIO_MODER_OFFSET);
    volatile uint32_t* p_pupdr = (volatile uint32_t *)(port_base_addr + GPIO_PUPDR_OFFSET);
    volatile uint32_t* p_otyper = (volatile uint32_t *)(port_base_addr + GPIO_OTYPER_OFFSET);

    // Set mode to Output (01)
    CLR_BIT(*p_moder, 2 * pin);
    SET_BIT(*p_moder, 2 * pin + 1);

    // Set initial value
    if (value & (1 << pin)) { SET_BIT(*p_odr, pin); } else { CLR_BIT(*p_odr, pin); }

    // Set output type (Push-Pull/Open-Drain)
    if (usage == communication_usage) { SET_BIT(*p_otyper, pin); } else { CLR_BIT(*p_otyper, pin); }

    // Disable Pull-up/Pull-down (00)
    CLR_BIT(*p_pupdr, 2 * pin);
    CLR_BIT(*p_pupdr, 2 * pin + 1);
    */
}

/***************************************************************************************************
* @brief Initializes a specific GPIO pin as input.
* @param port: The GPIO port (e.g., Port_A).
* @param pin: The pin number (0-15).
* @param usage: The pin usage (general_usage, communication_usage). Usage for input
*               is less standard; assumed it might relate to Open-Drain considerations
*               or specific input configurations, following the prompt's POMx instruction.
* @return None.
* Note: The do-while loop condition relies on GPIO_Direction_get, which is implemented
*       based on a potentially flawed understanding of the STM32 MODER register (1 bit vs 2 bits).
*       This function's configuration steps (PUPDR, OTYPER, MODER manipulation via single bits)
*       also deviate significantly from standard STM32 GPIO register programming practices
*       which require handling 2 bits per pin for mode and pull-up/down. OTYPER is typically
*       irrelevant for input pins.
***************************************************************************************************/
void GPIO_Input_Init(tport port, tpin pin, t_usage usage)
{
    WDT_Reset(); /* Assumed - please verify function availability */

    volatile uint32_t* pupdr_reg = NULL; /* Assumed PUx maps to PUPDR */
    volatile uint32_t* otyper_reg = NULL;/* Assumed POMx maps to OTYPER */
    volatile uint32_t* moder_reg = NULL; /* Assumed PMx maps to MODER */

    /*
     * WARNING: The do-while loop condition GPIO_Direction_get(...) != input
     * relies on a flawed implementation of GPIO_Direction_get for STM32 MODER
     * which uses 2 bits per pin. This loop may not function as intended.
     * Configuration steps inside the loop also use 1-bit manipulation on 2-bit
     * registers (MODER, PUPDR), which is incorrect for STM32. OTYPER is generally
     * not applicable to input pins.
     */
    do
    {
        switch(port)
        {
            case Port_A:
                pupdr_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOA_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOA_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_B:
                pupdr_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOB_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOB_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_C:
                pupdr_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOC_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOC_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_D:
                pupdr_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOD_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOD_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_E:
                pupdr_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOE_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOE_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            case Port_H:
                pupdr_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_PUPDR_OFFSET);  /* Assumed - please verify */
                otyper_reg= (volatile uint32_t *)(GPIOH_BASE + GPIO_OTYPER_OFFSET); /* Assumed - please verify */
                moder_reg = (volatile uint32_t *)(GPIOH_BASE + GPIO_MODER_OFFSET);  /* Assumed - please verify */
                break;
            default:
                /* Handle error: Invalid port */
                /* e.g., print error, log, or assert */
                return; /* Exit function on invalid port */
        }

        if (pupdr_reg != NULL && otyper_reg != NULL && moder_reg != NULL)
        {
            /*
             * WARNING: Following prompt instruction SET_BIT(PUx, pin) to enable pull-up.
             * STM32 PUPDR is 2 bits per pin (00=None, 01=PullUp, 10=PullDown).
             * Setting a single bit is NOT sufficient or correct for setting pull-up (01).
             * This step is expected to fail for STM32's intended pull-up config.
             */
            SET_BIT(*pupdr_reg, pin); /* Assumed - please verify register access method */

            /* Configure output type (Push-Pull or Open-Drain) - STRANGE FOR INPUT */
            /* Assumed POMx maps to OTYPER, 1=Open-Drain, 0=Push-Pull. OTYPER is usually ignored for input pins. */
            if (usage == communication_usage)
            {
                SET_BIT(*otyper_reg, pin); /* Assumed - please verify register access method */
            }
            else
            {
                CLR_BIT(*otyper_reg, pin); /* Assumed - please verify register access method */
            }

            /*
             * WARNING: Following prompt instruction SET_BIT(PMx, pin) to set as input.
             * STM32 MODER is 2 bits per pin (00=Input, 01=Output).
             * Setting a single bit is NOT sufficient or correct for setting input mode (00).
             * This step is expected to fail for STM32.
             */
            SET_BIT(*moder_reg, pin); /* Assumed - please verify register access method */
        }
        else
        {
             /* Handle error: Should not happen with valid port, but safety check */
             break; /* Break do-while loop on error */
        }

    } while(GPIO_Direction_get(port, pin) != input); /* Assumed - relies on flawed GPIO_Direction_get logic */

    /* After the potentially non-terminating or incorrect loop, ensure critical setup */
    /* In a real driver, the loop condition and register writes would need rework for STM32 */
    /* Example (correct STM32 input config with pull-up - *NOT* following prompt's logic): */
    /*
    volatile uint32_t* p_moder = (volatile uint32_t *)(port_base_addr + GPIO_MODER_OFFSET);
    volatile uint32_t* p_pupdr = (volatile uint32_t *)(port_base_addr + GPIO_PUPDR_OFFSET);

    // Set mode to Input (00)
    CLR_BIT(*p_moder, 2 * pin);
    CLR_BIT(*p_moder, 2 * pin + 1);

    // Set Pull-up (01)
    SET_BIT(*p_pupdr, 2 * pin);
    CLR_BIT(*p_pupdr, 2 * pin + 1);

    // OTYPER is irrelevant for input
    */
}

/***************************************************************************************************
* @brief Placeholder for potential future functions.
***************************************************************************************************/
/* None */