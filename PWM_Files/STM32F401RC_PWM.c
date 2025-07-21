/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready PWM implementation for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h" // Includes necessary bare-metal register definitions and TRD_Channel_t enum
#include <stdint.h>          // For standard integer types like uint32_t

/*
*   STM32F401RC Clock Configuration Assumptions:
*   - The system clock (HCLK) is assumed to be 84 MHz.
*   - APB1 Prescaler is assumed to be configured such that TIMxCLK = HCLK (84 MHz).
*     This occurs if APB1 prescaler is greater than 1, causing TIMxCLK to be 2x APB1 clock.
*     If APB1 clock is HCLK/2 = 42MHz, then TIMxCLK = 2 * 42MHz = 84MHz.
*   - APB2 Prescaler is assumed to be configured such that TIMxCLK = HCLK (84 MHz).
*     This occurs if APB2 prescaler is 1, causing TIMxCLK to be APB2 clock.
*     If APB2 clock is HCLK = 84MHz, then TIMxCLK = 84MHz.
*     (TIM1, TIM2, TIM3, TIM4, TIM10, TIM11) are assumed to be 84 MHz.
*/
static const uint32_t TIM_CLOCK_FREQ = 84000000UL; /* Assumed SystemCoreClock / Timer Bus Clock Frequency */

/*
*   Timer Reservation Policy:
*   As per requirements, at least 2 Timers and their channels are reserved for OS or delay purposes.
*   For STM32F401RC, general-purpose timers are used for this as dedicated basic timers (like TIM6/7)
*   are not extensively covered or are not available for typical PWM applications in the provided context.
*   The following timers are explicitly reserved and excluded from this PWM driver:
*   - TIM5: A 32-bit general-purpose timer.
*   - TIM9: A 16-bit general-purpose timer.
*   This ensures that the application has resources available for other critical functionalities
*   without conflict with the PWM module.
*/

/**
 * @brief Represents the configuration of a single PWM channel.
 *
 * This structure holds all necessary hardware details for configuring
 * a specific timer channel for PWM output.
 */
typedef struct
{
    uint33_t         TIM_Base;                  /* Base address of the Timer peripheral (e.g., TIM1_BASE) */
    uint8_t          ChannelNumber;             /* Channel number (1, 2, 3, or 4) */
    GPIO_Port_Name_t PortName;                  /* Generic GPIO port name (e.g., PORT_A) */
    uint8_t          PinNumber;                 /* GPIO pin number (0-15) */
    uint8_t          AlternateFunctionNumber;   /* Alternate function (AF) number for the pin */
} PWM_Channel_Config_t;


/**
 * @brief Array mapping logical TRD_Channel_t to physical PWM hardware configurations.
 *
 * This array defines the timer, channel, GPIO port, pin, and alternate function
 * for each PWM channel supported by this driver.
 *
 * NOTE ON PHYSICAL PIN SHARING:
 * Some physical pins (e.g., PA8, PB9) are capable of being used by multiple timers
 * with different alternate functions. While this array lists distinct logical channels
 * (e.g., TRD_TIM1_CH1 and TRD_TIM10_CH1 both use PA8 but with different AFs),
 * it is critical for the application layer to ensure that a single physical pin is
 * NOT configured for output by more than one timer simultaneously at runtime.
 * This array focuses on mapping all *valid* PWM hardware configurations for the MCU.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (APB2 Timer, 16-bit, Advanced-control)
    // AF for TIM1 is AF1
    { TIM1_BASE, 1, PORT_A, 8,  GPIO_AF1_TIM1 }, // TRD_TIM1_CH1, PA8, AF1 /* Assumed PWM config */
    { TIM1_BASE, 2, PORT_A, 9,  GPIO_AF1_TIM1 }, // TRD_TIM1_CH2, PA9, AF1 /* Assumed PWM config */
    { TIM1_BASE, 3, PORT_A, 10, GPIO_AF1_TIM1 }, // TRD_TIM1_CH3, PA10, AF1 /* Assumed PWM config */
    { TIM1_BASE, 4, PORT_A, 11, GPIO_AF1_TIM1 }, // TRD_TIM1_CH4, PA11, AF1 /* Assumed PWM config */

    // TIM2 Channels (APB1 Timer, 32-bit, General-purpose)
    // AF for TIM2 is AF1
    { TIM2_BASE, 1, PORT_A, 0,  GPIO_AF1_TIM2 }, // TRD_TIM2_CH1, PA0, AF1 /* Assumed PWM config - Pin 0, verify usage context */
    { TIM2_BASE, 2, PORT_A, 1,  GPIO_AF1_TIM2 }, // TRD_TIM2_CH2, PA1, AF1 /* Assumed PWM config - Pin 1, verify usage context */
    { TIM2_BASE, 3, PORT_A, 2,  GPIO_AF1_TIM2 }, // TRD_TIM2_CH3, PA2, AF1 /* Assumed PWM config */
    { TIM2_BASE, 4, PORT_A, 3,  GPIO_AF1_TIM2 }, // TRD_TIM2_CH4, PA3, AF1 /* Assumed PWM config */

    // TIM3 Channels (APB1 Timer, 16-bit, General-purpose)
    // AF for TIM3 is AF2
    { TIM3_BASE, 1, PORT_A, 6,  GPIO_AF2_TIM3 }, // TRD_TIM3_CH1, PA6, AF2 /* Assumed PWM config */
    { TIM3_BASE, 2, PORT_A, 7,  GPIO_AF2_TIM3 }, // TRD_TIM3_CH2, PA7, AF2 /* Assumed PWM config */
    { TIM3_BASE, 3, PORT_B, 0,  GPIO_AF2_TIM3 }, // TRD_TIM3_CH3, PB0, AF2 /* Assumed PWM config - Pin 0, verify usage context */
    { TIM3_BASE, 4, PORT_B, 1,  GPIO_AF2_TIM3 }, // TRD_TIM3_CH4, PB1, AF2 /* Assumed PWM config - Pin 1, verify usage context */

    // TIM4 Channels (APB1 Timer, 16-bit, General-purpose)
    // AF for TIM4 is AF2
    { TIM4_BASE, 1, PORT_B, 6,  GPIO_AF2_TIM4 }, // TRD_TIM4_CH1, PB6, AF2 /* Assumed PWM config */
    { TIM4_BASE, 2, PORT_B, 7,  GPIO_AF2_TIM4 }, // TRD_TIM4_CH2, PB7, AF2 /* Assumed PWM config */
    { TIM4_BASE, 3, PORT_B, 8,  GPIO_AF2_TIM4 }, // TRD_TIM4_CH3, PB8, AF2 /* Assumed PWM config */
    { TIM4_BASE, 4, PORT_B, 9,  GPIO_AF2_TIM4 }, // TRD_TIM4_CH4, PB9, AF2 /* Assumed PWM config */

    // TIM10 Channel (APB2 Timer, 16-bit, General-purpose)
    // AF for TIM10 is AF3
    { TIM10_BASE, 1, PORT_A, 8, GPIO_AF3_TIM10 }, // TRD_TIM10_CH1, PA8, AF3 /* Assumed PWM config - Physical pin conflict with TRD_TIM1_CH1 (PA8 AF1). Ensure only one is used at a time. */

    // TIM11 Channel (APB2 Timer, 16-bit, General-purpose)
    // AF for TIM11 is AF3
    { TIM11_BASE, 1, PORT_B, 9, GPIO_AF3_TIM11 }  // TRD_TIM11_CH1, PB9, AF3 /* Assumed PWM config - Physical pin conflict with TRD_TIM4_CH4 (PB9 AF2). Ensure only one is used at a time. */
};


/**Functions ===========================================================================*/

/**
 * @brief Retrieves the GPIO Port Base Address from a generic port name.
 * @param port_name The generic GPIO_Port_Name_t enum value.
 * @return The base address of the GPIO port. Returns 0 if invalid.
 */
static uint32_t Get_GPIO_Port_Base(GPIO_Port_Name_t port_name)
{
    switch(port_name)
    {
        case PORT_A: return GPIOA_BASE;
        case PORT_B: return GPIOB_BASE;
        case PORT_C: return GPIOC_BASE;
        case PORT_D: return GPIOD_BASE; // Limited availability on STM32F401RC, typically not used for general I/O
        case PORT_E: return GPIOE_BASE; // Limited availability on STM32F401RC, typically not used for general I/O
        case PORT_H: return GPIOH_BASE; // Limited availability on STM32F401RC, mostly H0/H1
        default: return 0; // Invalid port
    }
}

/**
 * @brief Retrieves the RCC AHB1ENR bit for enabling a GPIO port clock.
 * @param port_name The generic GPIO_Port_Name_t enum value.
 * @return The bit mask for RCC_AHB1ENR. Returns 0 if invalid.
 */
static uint32_t Get_GPIO_Clock_Bit(GPIO_Port_Name_t port_name)
{
    switch(port_name)
    {
        case PORT_A: return RCC_AHB1ENR_GPIOAEN;
        case PORT_B: return RCC_AHB1ENR_GPIOBEN;
        case PORT_C: return RCC_AHB1ENR_GPIOCEN;
        case PORT_D: return RCC_AHB1ENR_GPIODEN;
        case PORT_E: return RCC_AHB1ENR_GPIOEEN;
        case PORT_H: return RCC_AHB1ENR_GPIOHEN;
        default: return 0; // Invalid port
    }
}

/**
 * @brief Retrieves the RCC APBxENR bit for enabling a Timer peripheral clock.
 * @param tim_base The base address of the Timer peripheral.
 * @return The bit mask for the corresponding RCC APBxENR. Returns 0 if invalid.
 */
static uint32_t Get_Timer_Clock_Bit(uint32_t tim_base)
{
    if (tim_base == TIM1_BASE) return RCC_APB2ENR_TIM1EN; /* PDF Reference */
    if (tim_base == TIM2_BASE) return RCC_APB1ENR_TIM2EN; /* PDF Reference */
    if (tim_base == TIM3_BASE) return RCC_APB1ENR_TIM3EN; /* PDF Reference */
    if (tim_base == TIM4_BASE) return RCC_APB1ENR_TIM4EN; /* PDF Reference */
    // TIM5 and TIM9 are reserved as per driver policy.
    if (tim_base == TIM10_BASE) return RCC_APB2ENR_TIM10EN; /* PDF Reference */
    if (tim_base == TIM11_BASE) return RCC_APB2ENR_TIM11EN; /* PDF Reference */
    return 0; // Invalid or reserved timer
}

/**
 * @brief Determines if a timer is an Advanced Control Timer (TIM1).
 * @param tim_base The base address of the Timer peripheral.
 * @return 1 if TIM1, 0 otherwise.
 */
static uint8_t Is_Advanced_Control_Timer(uint32_t tim_base)
{
    return (tim_base == TIM1_BASE);
}

/**
 * @brief Determines if a timer is a 32-bit timer (TIM2, TIM5).
 * @param tim_base The base address of the Timer peripheral.
 * @return 1 if 32-bit, 0 otherwise.
 */
static uint8_t Is_32Bit_Timer(uint32_t tim_base)
{
    return (tim_base == TIM2_BASE || tim_base == TIM5_BASE); /* PDF Reference */
}

/**
 * @brief Finds the PWM channel configuration in the pwm_channel_map array.
 * @param TRD_Channel The logical PWM channel identifier.
 * @return A pointer to the PWM_Channel_Config_t structure, or NULL if not found.
 */
static const PWM_Channel_Config_t* Find_PWM_Channel_Config(TRD_Channel_t TRD_Channel)
{
    // TRD_PWM_CHANNEL_COUNT must be defined as the number of elements in pwm_channel_map
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT)
    {
        return NULL; // Invalid channel index
    }
    return &pwm_channel_map[TRD_Channel];
}

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The logical PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = Find_PWM_Channel_Config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    uint32_t tim_base = config->TIM_Base;
    uint8_t channel_num = config->ChannelNumber;
    uint32_t gpio_port_base = Get_GPIO_Port_Base(config->PortName);
    uint8_t pin_num = config->PinNumber;
    uint8_t af_num = config->AlternateFunctionNumber;

    if (gpio_port_base == 0 || Get_Timer_Clock_Bit(tim_base) == 0)
    {
        return; // Invalid configuration derived
    }

    /* 1. Enable GPIO clock */
    // Ensure clock is enabled for the GPIO port
    RCC->AHB1ENR |= Get_GPIO_Clock_Bit(config->PortName); /* PDF Reference */

    /* 2. Configure GPIO pin for Alternate Function */
    // Clear MODER bits (2 bits per pin) to set to 00 (Input mode initially, or just clear current)
    // Then set MODER to Alternate Function mode (10b)
    uint32_t* moder_reg = (uint32_t*)(gpio_port_base + GPIO_MODER_OFFSET);
    *moder_reg &= ~(GPIO_MODER_MODER0_Msk << (pin_num * 2)); /* PDF Reference */
    *moder_reg |= (GPIO_MODER_ALTERNATE_FUNCTION << (pin_num * 2)); /* PDF Reference */

    // Configure Alternate Function register (AFRL for pins 0-7, AFRH for pins 8-15)
    if (pin_num < 8)
    {
        uint32_t* afrl_reg = (uint32_t*)(gpio_port_base + GPIO_AFRL_OFFSET);
        *afrl_reg &= ~(GPIO_AFR_AFSEL0_Msk << (pin_num * 4)); /* PDF Reference */
        *afrl_reg |= (af_num << (pin_num * 4)); /* PDF Reference */
    }
    else
    {
        uint32_t* afrh_reg = (uint32_t*)(gpio_port_base + GPIO_AFRH_OFFSET);
        *afrh_reg &= ~(GPIO_AFR_AFSEL8_Msk << ((pin_num - 8) * 4)); /* PDF Reference */
        *afrh_reg |= (af_num << ((pin_num - 8) * 4)); /* PDF Reference */
    }

    /* 3. Configure GPIO Output Type to Push-Pull (00b) */
    // OTYPER bit for pin must be 0 for Push-Pull
    uint32_t* otyper_reg = (uint32_t*)(gpio_port_base + GPIO_OTYPER_OFFSET);
    *otyper_reg &= ~(GPIO_OTYPER_OT0_Msk << pin_num); /* PDF Reference */

    /* 4. Configure GPIO Output Speed to Very High Speed (11b) */
    uint32_t* ospeedr_reg = (uint32_t*)(gpio_port_base + GPIO_OSPEEDR_OFFSET);
    *ospeedr_reg &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk << (pin_num * 2)); /* PDF Reference */
    *ospeedr_reg |= (GPIO_OSPEEDR_VERY_HIGH_SPEED << (pin_num * 2)); /* PDF Reference - Very High Speed */

    /* 5. Configure GPIO Pull-up/Pull-down to No Pull (00b) */
    uint32_t* pupdr_reg = (uint32_t*)(gpio_port_base + GPIO_PUPDR_OFFSET);
    *pupdr_reg &= ~(GPIO_PUPDR_PUPDR0_Msk << (pin_num * 2)); /* PDF Reference */
    *pupdr_reg |= (GPIO_PUPDR_NO_PULL << (pin_num * 2)); /* PDF Reference - No pull-up/pull-down */

    /* 6. Enable Timer peripheral clock */
    if (tim_base >= APB2PERIPH_BASE) // APB2 timers (TIM1, TIM9, TIM10, TIM11)
    {
        RCC->APB2ENR |= Get_Timer_Clock_Bit(tim_base); /* PDF Reference */
    }
    else // APB1 timers (TIM2, TIM3, TIM4, TIM5)
    {
        RCC->APB1ENR |= Get_Timer_Clock_Bit(tim_base); /* PDF Reference */
    }

    /* 7. Configure Timer in PWM Mode 1 */
    // Clear CCMR register bits for the channel and set output compare mode to PWM Mode 1
    // And enable Output Compare Preload Enable (OCxPE)
    volatile uint32_t* ccmr_reg;
    uint32_t ccmr_clear_mask = 0;
    uint32_t ccmr_set_mask = 0;

    // Direct register access: TIMx_CCMR1 for channels 1,2; TIMx_CCMR2 for channels 3,4
    if (channel_num == 1 || channel_num == 2) {
        ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR1_OFFSET);
        if (channel_num == 1)
        {
            // Clear OC1M (bits 4-6), OC1PE (bit 3), CC1S (bits 0-1)
            ccmr_clear_mask = TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE_Msk | TIM_CCMR1_CC1S_Msk;
            ccmr_set_mask = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_CC1S_OUTPUT; /* PDF Reference */
        }
        else // channel_num == 2
        {
            // Clear OC2M (bits 12-14), OC2PE (bit 11), CC2S (bits 8-9)
            ccmr_clear_mask = TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC2PE_Msk | TIM_CCMR1_CC2S_Msk;
            ccmr_set_mask = TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE | TIM_CCMR1_CC2S_OUTPUT; /* PDF Reference */
        }
    } else if (channel_num == 3 || channel_num == 4) {
        // Note: TIM9/10/11 do not have CCMR2. This case is for TIM1-4.
        ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR2_OFFSET);
        if (channel_num == 3)
        {
            // Clear OC3M (bits 4-6), OC3PE (bit 3), CC3S (bits 0-1)
            ccmr_clear_mask = TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC3PE_Msk | TIM_CCMR2_CC3S_Msk;
            ccmr_set_mask = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_CC3S_OUTPUT; /* PDF Reference */
        }
        else // channel_num == 4
        {
            // Clear OC4M (bits 12-14), OC4PE (bit 11), CC4S (bits 8-9)
            ccmr_clear_mask = TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE_Msk | TIM_CCMR2_CC4S_Msk;
            ccmr_set_mask = TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE | TIM_CCMR2_CC4S_OUTPUT; /* PDF Reference */
        }
    }
    else
    {
        return; // Invalid channel number for PWM
    }

    *ccmr_reg &= ~ccmr_clear_mask; /* PDF Reference */
    *ccmr_reg |= ccmr_set_mask;    /* PDF Reference */

    /* 8. Enable Auto-Reload Preload (ARPE) */
    // Direct register access: TIMx_CR1
    volatile uint32_t* cr1_reg = (uint32_t*)(tim_base + TIM_CR1_OFFSET);
    *cr1_reg |= TIM_CR1_ARPE; /* PDF Reference */

    /* 9. Configure clock division and Counter mode */
    // Clear CKD bits (bits 8-9) and set to 00 (No clock division)
    *cr1_reg &= ~TIM_CR1_CKD_Msk; /* PDF Reference */
    *cr1_reg |= TIM_CR1_CKD_DIV1; // Set tDTS = tCK_INT (No clock division) /* PDF Reference */

    // For general purpose timers, only upcounting mode is relevant for basic PWM.
    // For advanced timers, ensure edge-aligned mode (CMS=00) and upcounting (DIR=0).
    // Clear CMS bits (bits 5-6) set to edge-aligned mode (00b)
    *cr1_reg &= ~TIM_CR1_CMS_Msk; /* PDF Reference */
    // Clear DIR bit (bit 4) set to upcounting mode (0)
    *cr1_reg &= ~TIM_CR1_DIR_Msk;   /* PDF Reference */

    /* 10. Generate an Update Event to load preload registers (UG bit) */
    // Direct register access: TIMx_EGR
    volatile uint32_t* egr_reg = (uint32_t*)(tim_base + TIM_EGR_OFFSET);
    *egr_reg |= TIM_EGR_UG; /* PDF Reference */

    /* 11. Enable the specific Capture/Compare Output (CCxE) */
    // Direct register access: TIMx_CCER
    volatile uint32_t* ccer_reg = (uint32_t*)(tim_base + TIM_CCER_OFFSET);
    uint32_t ccer_enable_mask = 0;
    if (channel_num == 1) ccer_enable_mask = TIM_CCER_CC1E; /* PDF Reference */
    else if (channel_num == 2) ccer_enable_mask = TIM_CCER_CC2E; /* PDF Reference */
    else if (channel_num == 3) ccer_enable_mask = TIM_CCER_CC3E; /* PDF Reference */
    else if (channel_num == 4) ccer_enable_mask = TIM_CCER_CC4E; /* PDF Reference */

    *ccer_reg |= ccer_enable_mask; /* PDF Reference */

    /* 12. For TIM1 (Advanced Control Timer), enable Main Output (MOE) */
    if (Is_Advanced_Control_Timer(tim_base))
    {
        // Direct register access: TIMx_BDTR
        volatile uint32_t* bdtr_reg = (uint32_t*)(tim_base + TIM_BDTR_OFFSET);
        *bdtr_reg |= TIM_BDTR_MOE; /* PDF Reference */
    }
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The logical PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, uint32_t frequency, uint8_t duty)
{
    const PWM_Channel_Config_t* config = Find_PWM_Channel_Config(TRD_Channel);
    if (config == NULL || frequency == 0)
    {
        return; // Invalid channel or frequency
    }

    uint32_t tim_base = config->TIM_Base;

    // Calculate ARR and PSC based on desired frequency and timer clock
    // f_PWM = f_TIM / ((PSC + 1) * (ARR + 1))
    // For optimal resolution, we aim for the largest possible ARR value.
    uint32_t arr_max = Is_32Bit_Timer(tim_base) ? 0xFFFFFFFFUL : 0xFFFFUL; /* PDF Reference - ARR max values */

    uint32_t prescaler = 0;
    uint32_t auto_reload_val = 0;

    // Iterate to find suitable prescaler and ARR values
    // This simple loop prioritizes minimizing (PSC + 1) for higher frequency accuracy,
    // and then maximizing (ARR + 1) for higher duty cycle resolution.
    // A more sophisticated algorithm could be used for wider frequency ranges.
    for (prescaler = 0; prescaler <= 0xFFFF; prescaler++) /* PDF Reference - PSC max is 65535 */
    {
        uint32_t timer_freq_after_psc = TIM_CLOCK_FREQ / (prescaler + 1);
        if (frequency > timer_freq_after_psc) continue; // Cannot achieve desired frequency with this prescaler

        auto_reload_val = (timer_freq_after_psc / frequency) - 1;

        if (auto_reload_val <= arr_max)
        {
            // Found a valid combination, break and use these values
            break;
        }
    }

    if (prescaler > 0xFFFF || auto_reload_val > arr_max)
    {
        // Could not find a suitable PSC/ARR combination for the desired frequency
        return;
    }

    // Set Prescaler and Auto-Reload Register
    volatile uint32_t* psc_reg = (uint32_t*)(tim_base + TIM_PSC_OFFSET);
    volatile uint32_t* arr_reg = (uint32_t*)(tim_base + TIM_ARR_OFFSET);
    *psc_reg = prescaler; /* PDF Reference */
    *arr_reg = auto_reload_val; /* PDF Reference */

    // Calculate Compare Value (Duty Cycle)
    // CCR = (duty / 100.0) * (ARR + 1)
    uint32_t compare_val = (uint32_t)(((uint32_t)duty * (auto_reload_val + 1)) / 100UL);
    if (duty == 0) compare_val = 0; // Ensure 0% duty
    // If CCRx > ARR, then OCxREF is held high (100% duty)
    if (duty >= 100) compare_val = auto_reload_val + 1; // Ensure 100% duty, as per PDF if CCRx > ARR then OCxREF is held at '1'

    // Set Compare Capture Register for the specific channel
    switch (config->ChannelNumber)
    {
        case 1: *(volatile uint32_t*)(tim_base + TIM_CCR1_OFFSET) = compare_val; break; /* PDF Reference */
        case 2: *(volatile uint32_t*)(tim_base + TIM_CCR2_OFFSET) = compare_val; break; /* PDF Reference */
        case 3: *(volatile uint32_t*)(tim_base + TIM_CCR3_OFFSET) = compare_val; break; /* PDF Reference */
        case 4: *(volatile uint32_t*)(tim_base + TIM_CCR4_OFFSET) = compare_val; break; /* PDF Reference */
        default: return; // Should not happen with valid config
    }
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The logical PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = Find_PWM_Channel_Config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    uint32_t tim_base = config->TIM_Base;

    /* Enable counter */
    // Direct register access: TIMx_CR1
    volatile uint32_t* cr1_reg = (uint32_t*)(tim_base + TIM_CR1_OFFSET);
    *cr1_reg |= TIM_CR1_CEN; /* PDF Reference */

    /* For TIM1 (Advanced Control Timer), enable Main Output (MOE) */
    // This is required for advanced timers for the output to be enabled on the pin.
    if (Is_Advanced_Control_Timer(tim_base))
    {
        // Direct register access: TIMx_BDTR
        volatile uint32_t* bdtr_reg = (uint32_t*)(tim_base + TIM_BDTR_OFFSET);
        *bdtr_reg |= TIM_BDTR_MOE; /* PDF Reference */
    }
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The logical PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = Find_PWM_Channel_Config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    uint32_t tim_base = config->TIM_Base;
    uint8_t channel_num = config->ChannelNumber;

    /* Disable counter */
    // Direct register access: TIMx_CR1
    volatile uint32_t* cr1_reg = (uint32_t*)(tim_base + TIM_CR1_OFFSET);
    *cr1_reg &= ~TIM_CR1_CEN; /* PDF Reference */

    /* For TIM1 (Advanced Control Timer), clear Main Output (MOE) */
    if (Is_Advanced_Control_Timer(tim_base))
    {
        // Direct register access: TIMx_BDTR
        volatile uint32_t* bdtr_reg = (uint32_t*)(tim_base + TIM_BDTR_OFFSET);
        *bdtr_reg &= ~TIM_BDTR_MOE; /* PDF Reference */
    }

    /* Set output to inactive level (Force Inactive Mode) and disable CCxE */
    volatile uint32_t* ccmr_reg = NULL;
    uint32_t ccmr_clear_mask = 0;
    uint32_t ccmr_set_mask = 0;
    volatile uint32_t* ccer_reg = (uint32_t*)(tim_base + TIM_CCER_OFFSET);
    uint32_t ccer_disable_mask = 0;

    if (channel_num == 1)
    {
        ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR1_OFFSET);
        ccmr_clear_mask = TIM_CCMR1_OC1M_Msk; // Clear OC1M bits
        ccmr_set_mask = TIM_CCMR1_OC1M_FORCE_INACT; /* PDF Reference */
        ccer_disable_mask = TIM_CCER_CC1E;
    }
    else if (channel_num == 2)
    {
        ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR1_OFFSET);
        ccmr_clear_mask = TIM_CCMR1_OC2M_Msk; // Clear OC2M bits
        ccmr_set_mask = TIM_CCMR1_OC2M_FORCE_INACT; /* PDF Reference */
        ccer_disable_mask = TIM_CCER_CC2E;
    }
    else if (channel_num == 3)
    {
        ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR2_OFFSET); // Note: TIM9/10/11 do not have CCMR2.
        ccmr_clear_mask = TIM_CCMR2_OC3M_Msk; // Clear OC3M bits
        ccmr_set_mask = TIM_CCMR2_OC3M_FORCE_INACT; /* PDF Reference */
        ccer_disable_mask = TIM_CCER_CC3E;
    }
    else if (channel_num == 4)
    {
        ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR2_OFFSET); // Note: TIM9/10/11 do not have CCMR2.
        ccmr_clear_mask = TIM_CCMR2_OC4M_Msk; // Clear OC4M bits
        ccmr_set_mask = TIM_CCMR2_OC4M_FORCE_INACT; /* PDF Reference */
        ccer_disable_mask = TIM_CCER_CC4E;
    }
    else
    {
        return; // Should not happen with valid config
    }

    // Set OCxM to Force Inactive to ensure output goes low (assuming active high PWM)
    *ccmr_reg &= ~ccmr_clear_mask; /* PDF Reference */
    *ccmr_reg |= ccmr_set_mask;    /* PDF Reference */

    // Disable the capture/compare channel output
    *ccer_reg &= ~ccer_disable_mask; /* PDF Reference */

    // Generate an Update Event to apply new mode immediately
    volatile uint32_t* egr_reg = (uint32_t*)(tim_base + TIM_EGR_OFFSET);
    *egr_reg |= TIM_EGR_UG; /* PDF Reference */
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through all configured PWM channels and disables their
 *        corresponding timers and reconfigures GPIO pins to input floating mode.
 */
void PWM_PowerOff(void)
{
    // Arrays to track enabled GPIO and Timer clocks to disable them once
    // A single bit in RCC registers controls all pins/channels for a peripheral.
    uint32_t enabled_gpio_clocks = 0;
    uint32_t enabled_apb1_timer_clocks = 0;
    uint32_t enabled_apb2_timer_clocks = 0;

    for (TRD_Channel_t i = (TRD_Channel_t)0; i < TRD_PWM_CHANNEL_COUNT; i++)
    {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];

        uint32_t tim_base = config->TIM_Base;
        uint8_t channel_num = config->ChannelNumber;
        uint32_t gpio_port_base = Get_GPIO_Port_Base(config->PortName);
        uint8_t pin_num = config->PinNumber;

        // Skip if configuration invalid or for reserved timers not in Get_Timer_Clock_Bit
        if (gpio_port_base == 0 || Get_Timer_Clock_Bit(tim_base) == 0)
        {
            continue;
        }

        /* Stop the individual PWM channel */
        // Disable counter (CEN bit in CR1)
        volatile uint32_t* cr1_reg = (uint32_t*)(tim_base + TIM_CR1_OFFSET);
        *cr1_reg &= ~TIM_CR1_CEN; /* PDF Reference */

        // For TIM1 (Advanced Control Timer), clear Main Output (MOE)
        if (Is_Advanced_Control_Timer(tim_base))
        {
            volatile uint32_t* bdtr_reg = (uint32_t*)(tim_base + TIM_BDTR_OFFSET);
            *bdtr_reg &= ~TIM_BDTR_MOE; /* PDF Reference */
        }

        // Set output to inactive level (Force Inactive Mode) and disable CCxE
        volatile uint32_t* ccmr_reg = NULL;
        uint32_t ccmr_clear_mask = 0;
        uint32_t ccmr_set_mask = 0;
        volatile uint32_t* ccer_reg = (uint32_t*)(tim_base + TIM_CCER_OFFSET);
        uint32_t ccer_disable_mask = 0;

        if (channel_num == 1)
        {
            ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR1_OFFSET); ccmr_clear_mask = TIM_CCMR1_OC1M_Msk; ccmr_set_mask = TIM_CCMR1_OC1M_FORCE_INACT;
            ccer_disable_mask = TIM_CCER_CC1E;
        }
        else if (channel_num == 2)
        {
            ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR1_OFFSET); ccmr_clear_mask = TIM_CCMR1_OC2M_Msk; ccmr_set_mask = TIM_CCMR1_OC2M_FORCE_INACT;
            ccer_disable_mask = TIM_CCER_CC2E;
        }
        else if (channel_num == 3)
        {
            ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR2_OFFSET); ccmr_clear_mask = TIM_CCMR2_OC3M_Msk; ccmr_set_mask = TIM_CCMR2_OC3M_FORCE_INACT;
            ccer_disable_mask = TIM_CCER_CC3E;
        }
        else if (channel_num == 4)
        {
            ccmr_reg = (uint32_t*)(tim_base + TIM_CCMR2_OFFSET); ccmr_clear_mask = TIM_CCMR2_OC4M_Msk; ccmr_set_mask = TIM_CCMR2_OC4M_FORCE_INACT;
            ccer_disable_mask = TIM_CCER_CC4E;
        }

        if (ccmr_reg != NULL)
        {
            *ccmr_reg &= ~ccmr_clear_mask; /* PDF Reference */
            *ccmr_reg |= ccmr_set_mask;    /* PDF Reference */
            *ccer_reg &= ~ccer_disable_mask; /* PDF Reference */ // Disable the output
            volatile uint32_t* egr_reg = (uint32_t*)(tim_base + TIM_EGR_OFFSET);
            *egr_reg |= TIM_EGR_UG; /* PDF Reference - Apply update immediately */
        }

        /* Reconfigure GPIO pin to Input Floating mode */
        // Clear MODER bits for the pin (sets to 00: Input mode)
        volatile uint32_t* moder_reg = (uint32_t*)(gpio_port_base + GPIO_MODER_OFFSET);
        *moder_reg &= ~(GPIO_MODER_MODER0_Msk << (pin_num * 2)); /* PDF Reference */

        // Clear Alternate Function register bits for the pin
        if (pin_num < 8)
        {
            volatile uint32_t* afrl_reg = (uint32_t*)(gpio_port_base + GPIO_AFRL_OFFSET);
            *afrl_reg &= ~(GPIO_AFR_AFSEL0_Msk << (pin_num * 4)); /* PDF Reference */
        }
        else
        {
            volatile uint32_t* afrh_reg = (uint32_t*)(gpio_port_base + GPIO_AFRH_OFFSET);
            *afrh_reg &= ~(GPIO_AFR_AFSEL8_Msk << ((pin_num - 8) * 4)); /* PDF Reference */
        }
        // Ensure no pull-up/pull-down (00: No pull-up, pull-down)
        volatile uint32_t* pupdr_reg = (uint32_t*)(gpio_port_base + GPIO_PUPDR_OFFSET);
        *pupdr_reg &= ~(GPIO_PUPDR_PUPDR0_Msk << (pin_num * 2)); /* PDF Reference */

        // Store clock enable bits to disable later, only once per peripheral
        enabled_gpio_clocks |= Get_GPIO_Clock_Bit(config->PortName);
        if (tim_base >= APB2PERIPH_BASE)
        {
            enabled_apb2_timer_clocks |= Get_Timer_Clock_Bit(tim_base);
        }
        else
        {
            enabled_apb1_timer_clocks |= Get_Timer_Clock_Bit(tim_base);
        }
    }

    /* Disable all used Timer peripheral clocks */
    // Direct register access for RCC APB registers
    RCC->APB1ENR &= ~enabled_apb1_timer_clocks; /* PDF Reference */
    RCC->APB2ENR &= ~enabled_apb2_timer_clocks; /* PDF Reference */

    /* Disable all used GPIO peripheral clocks */
    // Direct register access for RCC AHB1 register
    RCC->AHB1ENR &= ~enabled_gpio_clocks; /* PDF Reference */
}