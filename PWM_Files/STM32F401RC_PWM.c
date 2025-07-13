/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
// Assuming STM32F401RC_PWM.h includes necessary peripheral headers like
// "stm32f401rc.h" or equivalent bare-metal register definitions,
// as well as definitions for TIM_TypeDef, GPIO_TypeDef, RCC_TypeDef,
// PWM_Channel_Config_t, TRD_Channel_t, and base addresses for peripherals.
// Also assuming tlong is uint32_t and tbyte is uint8_t.

// Reserved TIM10 and TIM11 for potential OS/delay use, as per requirements.
// Their channels are therefore excluded from the pwm_channel_map.

/**Hardware Mapping ===========================================================================*/
// This array maps the logical TRD_Channel_t enumeration to physical timer and GPIO configurations.
// Only common and valid PWM-capable pins for STM32F401RC are included, excluding reserved timers.
// Format: { TIMx, ChannelNumber, PortName, PinNumber, AlternateFunctionNumber }
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    { TIM1, 1, GPIOA, GPIO_Pin_8,  GPIO_AF_TIM1  }, /* TIM1_CH1 on PA8, AF1 (Advanced Timer) */
    { TIM1, 2, GPIOA, GPIO_Pin_9,  GPIO_AF_TIM1  }, /* TIM1_CH2 on PA9, AF1 */
    { TIM1, 3, GPIOA, GPIO_Pin_10, GPIO_AF_TIM1  }, /* TIM1_CH3 on PA10, AF1 */
    { TIM1, 4, GPIOA, GPIO_Pin_11, GPIO_AF_TIM1  }, /* TIM1_CH4 on PA11, AF1 */

    { TIM2, 1, GPIOA, GPIO_Pin_0,  GPIO_AF_TIM2  }, /* TIM2_CH1 on PA0, AF1 (General Purpose Timer, 32-bit) */
    { TIM2, 2, GPIOA, GPIO_Pin_1,  GPIO_AF_TIM2  }, /* TIM2_CH2 on PA1, AF1 */
    { TIM2, 3, GPIOA, GPIO_Pin_2,  GPIO_AF_TIM2  }, /* TIM2_CH3 on PA2, AF1 */
    { TIM2, 4, GPIOA, GPIO_Pin_3,  GPIO_AF_TIM2  }, /* TIM2_CH4 on PA3, AF1 */

    { TIM3, 1, GPIOA, GPIO_Pin_6,  GPIO_AF_TIM3  }, /* TIM3_CH1 on PA6, AF2 (General Purpose Timer, 16-bit) */
    { TIM3, 2, GPIOA, GPIO_Pin_7,  GPIO_AF_TIM3  }, /* TIM3_CH2 on PA7, AF2 */
    { TIM3, 3, GPIOB, GPIO_Pin_0,  GPIO_AF_TIM3  }, /* TIM3_CH3 on PB0, AF2 */
    { TIM3, 4, GPIOB, GPIO_Pin_1,  GPIO_AF_TIM3  }, /* TIM3_CH4 on PB1, AF2 */
    { TIM3, 1, GPIOC, GPIO_Pin_6,  GPIO_AF_TIM3  }, /* TIM3_CH1 on PC6, AF2 (Alternative mapping) */
    { TIM3, 2, GPIOC, GPIO_Pin_7,  GPIO_AF_TIM3  }, /* TIM3_CH2 on PC7, AF2 */
    { TIM3, 3, GPIOC, GPIO_Pin_8,  GPIO_AF_TIM3  }, /* TIM3_CH3 on PC8, AF2 */
    { TIM3, 4, GPIOC, GPIO_Pin_9,  GPIO_AF_TIM3  }, /* TIM3_CH4 on PC9, AF2 */

    { TIM4, 1, GPIOB, GPIO_Pin_6,  GPIO_AF_TIM4  }, /* TIM4_CH1 on PB6, AF2 (General Purpose Timer, 16-bit) */
    { TIM4, 2, GPIOB, GPIO_Pin_7,  GPIO_AF_TIM4  }, /* TIM4_CH2 on PB7, AF2 */
    { TIM4, 3, GPIOB, GPIO_Pin_8,  GPIO_AF_TIM4  }, /* TIM4_CH3 on PB8, AF2 */
    { TIM4, 4, GPIOB, GPIO_Pin_9,  GPIO_AF_TIM4  }, /* TIM4_CH4 on PB9, AF2 */

    // TIM5 channels (often on PA0-PA3, AF2) and TIM9 channels (PA2, PA3, AF3) are excluded
    // TIM10 and TIM11 are excluded as reserved for OS/delay.
};

/**Functions ===========================================================================*/

/**
 * @brief Get the PWM channel configuration from the map.
 * @param TRD_Channel: Logical channel identifier.
 * @param config: Pointer to the configuration structure to fill.
 * @return 0 on success, -1 on invalid channel.
 */
static int get_pwm_config(TRD_Channel_t TRD_Channel, PWM_Channel_Config_t *config)
{
    if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))
    {
        // Invalid channel index
        return -1;
    }
    *config = pwm_channel_map[TRD_Channel];
    return 0;
}

/**
 * @brief Enable the clock for a given timer peripheral.
 * @param TIMx: Pointer to the timer peripheral.
 */
static void enable_tim_clk(TIM_TypeDef *TIMx)
{
    // Assuming RCC base address and register definitions are available
    if (TIMx == TIM1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* PDF Reference */ /* Assumed bus mapping for TIM1 */
    }
    else if (TIMx == TIM2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* PDF Reference */ /* Assumed bus mapping for TIM2 */
    }
    else if (TIMx == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* PDF Reference */ /* Assumed bus mapping for TIM3 */
    }
    else if (TIMx == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* PDF Reference */ /* Assumed bus mapping for TIM4 */
    }
    else if (TIMx == TIM5)
    {
         // TIM5 clock enable - excluded from map, but include for completeness if needed elsewhere
         // RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; /* PDF Reference */ /* Assumed bus mapping for TIM5 */
    }
    else if (TIMx == TIM9)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* PDF Reference */ /* Assumed bus mapping for TIM9 */
    }
    // TIM10/11 excluded as reserved
}

/**
 * @brief Enable the clock for a given GPIO port.
 * @param GPIOx: Pointer to the GPIO port.
 */
static void enable_gpio_clk(GPIO_TypeDef *GPIOx)
{
    // Assuming RCC base address and register definitions are available
    if (GPIOx == GPIOA)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* PDF Reference */ /* Assumed bus mapping for GPIOA */
    }
    else if (GPIOx == GPIOB)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* PDF Reference */ /* Assumed bus mapping for GPIOB */
    }
    else if (GPIOx == GPIOC)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* PDF Reference */ /* Assumed bus mapping for GPIOC */
    }
    else if (GPIOx == GPIOD)
    {
         // GPIOD clock enable - not available on STM32F401RC according to PDF, exclude.
         // RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* Assumed bus mapping for GPIOD */
    }
    else if (GPIOx == GPIOE)
    {
         // GPIOE clock enable - not available on STM32F401RC according to PDF, exclude.
         // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; /* Assumed bus mapping for GPIOE */
    }
    else if (GPIOx == GPIOH)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; /* PDF Reference */ /* Assumed bus mapping for GPIOH */
    }
}


/**
 * @brief Initialize the PWM hardware and configure the timer and GPIOs for the given channel.
 * @param TRD_Channel: The logical channel identifier.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    PWM_Channel_Config_t config;
    if (get_pwm_config(TRD_Channel, &config) != 0)
    {
        // Handle invalid channel error, maybe return a status or assert
        return;
    }

    // 1. Enable clocks for Timer and GPIO
    enable_tim_clk(config.TIMx);
    enable_gpio_clk(config.GPIOx);

    // 2. Configure GPIO pin for Alternate Function
    uint32_t pin_index = 0;
    // Determine pin index from PinNumber bitmask (e.g. 0x0001 -> 0, 0x0002 -> 1, ...)
    if (config.PinNumber > 0)
    {
       // Find the bit position
       uint32_t temp_pin = config.PinNumber;
       while((temp_pin & 0x01) == 0)
       {
          pin_index++;
          temp_pin >>= 1;
       }
    } else {
        // Should not happen based on map definition, but defensive check
        return;
    }

    // Configure Pin as Alternate Function Output (MODER = 10)
    config.GPIOx->MODER &= ~(0x03 << (pin_index * 2)); /* PDF Reference */
    config.GPIOx->MODER |= (0x02 << (pin_index * 2)); /* PDF Reference */

    // Configure Output Type as Push-Pull (OTYPER = 0) - Reset state, but explicit is safer
    config.GPIOx->OTYPER &= ~(0x01 << pin_index); /* PDF Reference */

    // Configure Output Speed (e.g., High Speed = 10)
    config.GPIOx->OSPEEDR &= ~(0x03 << (pin_index * 2)); /* PDF Reference */
    config.GPIOx->OSPEEDR |= (0x02 << (pin_index * 2)); /* PDF Reference */ /* Assumed speed */

    // Configure Pull-up/Pull-down as No Pull-up/Pull-down (PUPDR = 00) - Reset state
    config.GPIOx->PUPDR &= ~(0x03 << (pin_index * 2)); /* PDF Reference */

    // Configure Alternate Function (AFRL for pin 0-7, AFRH for pin 8-15)
    if (pin_index < 8)
    {
        config.GPIOx->AFRL &= ~(0x0F << (pin_index * 4)); /* PDF Reference */
        config.GPIOx->AFRL |= (config.AlternateFunctionNumber << (pin_index * 4)); /* PDF Reference */
    }
    else
    {
        config.GPIOx->AFRH &= ~(0x0F << ((pin_index - 8) * 4)); /* PDF Reference */
        config.GPIOx->AFRH |= (config.AlternateFunctionNumber << ((pin_index - 8) * 4)); /* PDF Reference */
    }

    // 3. Configure Timer Time-base and PWM mode

    // Stop the timer before configuration
    config.TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

    // Set timer to Upcounting mode, Edge-aligned, enable ARR preload
    config.TIMx->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR | TIM_CR1_UDIS | TIM_CR1_URS | TIM_CR1_OPM); /* PDF Reference */
    config.TIMx->CR1 |= TIM_CR1_ARPE; /* PDF Reference */

    // Set a default frequency and duty cycle (e.g., 1kHz, 50%)
    // Assuming APB1 timer clock = 84MHz, APB2 timer clock = 84MHz
    // Need to adjust based on actual clock configuration if different.
    uint32_t timer_clock = 84000000; /* Assumed timer clock frequency */
    if (config.TIMx == TIM2 || config.TIMx == TIM3 || config.TIMx == TIM4 || config.TIMx == TIM5)
    {
        // These timers are on APB1. If APB1 prescaler > 1, timer clock is 2*APB1 clock.
        // Assuming 42MHz APB1, so timer clock is 84MHz.
        timer_clock = 84000000; /* Assumed timer clock frequency */
    } else { // TIM1, TIM9
        // These timers are on APB2. If APB2 prescaler > 1, timer clock is 2*APB2 clock.
        // Assuming 84MHz APB2, so timer clock is 168MHz / 2 = 84MHz (max for F401)
        // Max F401 clock is 84MHz. APB2 freq is <= 84MHz. Timer clock is APB2 freq if APB2 prescaler is 1,
        timer_clock = 84000000; /* Assumed timer clock frequency */
    }


    uint32_t default_frequency = 1000; // 1kHz
    tbyte default_duty = 50; // 50%

    uint64_t total_ticks = timer_clock / default_frequency;
    uint16_t psc = 0;
    uint32_t arr = 0;
    uint32_t ccr_val = 0;

    // Calculate optimal PSC and ARR
    // Try to keep ARR high for better duty cycle resolution
    if (total_ticks > 0)
    {
        // Find PSC that results in ARR <= 65535 for 16-bit timers
        if (config.TIMx == TIM2 || config.TIMx == TIM5) // 32-bit timers
        {
             psc = (total_ticks - 1) / (0xFFFFFFFFUL + 1); // Should be 0 if total_ticks fits in 32 bits
             if (psc > 0xFFFF) psc = 0xFFFF;
             arr = total_ticks / (psc + 1) - 1;
        }
        else // 16-bit timers (TIM1, TIM3, TIM4, TIM9, TIM10, TIM11)
        {
            psc = (total_ticks - 1) / 65536;
            if (psc > 0xFFFF) psc = 0xFFFF;
            arr = total_ticks / (psc + 1) - 1;
        }

        // Calculate CCR value for 50% duty cycle
        ccr_val = (arr + 1) * default_duty / 100;
    } else {
        // Frequency is too high or 0. Set to minimum possible or error.
        // Setting to 0 period means output is always low (0% duty).
        psc = 0;
        arr = 0;
        ccr_val = 0;
    }

    config.TIMx->PSC = psc; /* PDF Reference */
    config.TIMx->ARR = arr; /* PDF Reference */


    // Configure Channel N for PWM Mode 1 (OCnM = 110), enable preload (OCnPE = 1)
    volatile uint33_t* ccmr = (config.Channel == 1 || config.Channel == 2) ? &config.TIMx->CCMR1 : &config.TIMx->CCMR2; /* PDF Reference */
    uint32_t ccmr_shift = (config.Channel == 1 || config.Channel == 3) ? 0 : 8; // Shift for CC1/3 or CC2/4 settings
    uint32_t ccer_shift = (config.Channel - 1) * 4; // Shift for CC1E, CC1P, etc. in CCER /* PDF Reference */

    // Clear existing mode and preload bits
    *ccmr &= ~(0x78 << ccmr_shift); /* PDF Reference */
    // Set PWM mode 1 (110) and enable preload (1)
    *ccmr |= (0x6 << (ccmr_shift + 1)); // OCnM = 110 /* PDF Reference */
    *ccmr |= (0x1 << (ccmr_shift + 3)); // OCnPE = 1 /* PDF Reference */

    // Configure Capture/Compare Enable Register (CCER)
    // Enable output (CCnE = 1), set output polarity (CCnP = 0 for active high)
    config.TIMx->CCER &= ~(0xF << ccer_shift); // Clear CCnE, CCnP, CCnNE, CCnNP /* PDF Reference */
    config.TIMx->CCER |= (0x1 << ccer_shift); // Set CCnE = 1 /* PDF Reference */
    // config.TIMx->CCER |= (0x0 << (ccer_shift + 2)); // CCnP = 0 (Active High) - Reset state, implicit /* PDF Reference */


    // Set initial duty cycle value
    switch (config.Channel)
    {
        case 1: config.TIMx->CCR1 = ccr_val; break; /* PDF Reference */
        case 2: config.TIMx->CCR2 = ccr_val; break; /* PDF Reference */
        case 3: config.TIMx->CCR3 = ccr_val; break; /* PDF Reference */
        case 4: config.TIMx->CCR4 = ccr_val; break; /* PDF Reference */
        default: return; // Should not happen based on map
    }

    // Generate an update event to load the Prescaler and ARR registers,
    // and the CCR preload value into the active registers.
    config.TIMx->EGR |= TIM_EGR_UG; /* PDF Reference */

    // For TIM1 (Advanced Timer), enable the main output (MOE bit in BDTR)
    if (config.TIMx == TIM1)
    {
        config.TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // The timer is now configured and ready, but not started.
}

/**
 * @brief Set the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel: The logical channel identifier.
 * @param frequency: The desired PWM frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    PWM_Channel_Config_t config;
    if (get_pwm_config(TRD_Channel, &config) != 0)
    {
        // Handle invalid channel error
        return;
    }

    if (frequency == 0 || duty > 100)
    {
        // Invalid parameters
        return;
    }

    // Assuming APB1 timer clock = 84MHz, APB2 timer clock = 84MHz
    // Need to adjust based on actual clock configuration if different.
    uint32_t timer_clock = 84000000; /* Assumed timer clock frequency */
    if (config.TIMx == TIM2 || config.TIMx == TIM3 || config.TIMx == TIM4 || config.TIMx == TIM5)
    {
        // These timers are on APB1. If APB1 prescaler > 1, timer clock is 2*APB1 clock.
        // Assuming 42MHz APB1, so timer clock is 84MHz.
        timer_clock = 84000000; /* Assumed timer clock frequency */
    } else { // TIM1, TIM9
        // These timers are on APB2. If APB2 prescaler > 1, timer clock is 2*APB2 clock.
        // Assuming 84MHz APB2, so timer clock is 168MHz / 2 = 84MHz (max for F401)
        timer_clock = 84000000; /* Assumed timer clock frequency */
    }


    uint64_t total_ticks = timer_clock / frequency;
    uint16_t psc = 0;
    uint32_t arr = 0;
    uint32_t ccr_val = 0;

    if (total_ticks == 0)
    {
         // Frequency is too high, cannot achieve. Error or set minimum period.
         // Setting minimum period (PSC=0, ARR=0) results in max frequency, ~42MHz or ~84MHz depending on bus
         psc = 0;
         arr = 0; // Period = 1
         ccr_val = 0; // 0% duty
    } else {
        // Calculate optimal PSC and ARR
        // Try to keep ARR high for better duty cycle resolution
        if (config.TIMx == TIM2 || config.TIMx == TIM5) // 32-bit timers
        {
             psc = (total_ticks - 1) / (0xFFFFFFFFUL); // Find minimum psc for 32-bit ARR
             if (psc > 0xFFFF) psc = 0xFFFF;
             arr = total_ticks / (psc + 1) - 1;
        }
        else // 16-bit timers (TIM1, TIM3, TIM4, TIM9, TIM10, TIM11)
        {
            psc = (total_ticks - 1) / 65536; // Find minimum psc for 16-bit ARR
            if (psc > 0xFFFF) psc = 0xFFFF;
            arr = total_ticks / (psc + 1) - 1;
        }

        // Calculate CCR value based on new ARR and desired duty cycle
        ccr_val = (arr + 1) * duty / 100;
    }

    // Ensure timer is stopped while changing core parameters (optional but safer)
    uint32_t cr1_reg = config.TIMx->CR1; // Save CR1 state
    config.TIMx->CR1 &= ~TIM_CR1_CEN; // Disable counter /* PDF Reference */

    // Set new PSC, ARR, and CCR values
    config.TIMx->PSC = psc; /* PDF Reference */
    config.TIMx->ARR = arr; /* PDF Reference */

    switch (config.Channel)
    {
        case 1: config.TIMx->CCR1 = ccr_val; break; /* PDF Reference */
        case 2: config.TIMx->CCR2 = ccr_val; break; /* PDF Reference */
        case 3: config.TIMx->CCR3 = ccr_val; break; /* PDF Reference */
        case 4: config.TIMx->CCR4 = ccr_val; break; /* PDF Reference */
        default: break; // Should not happen
    }

    // Generate update event to load the new PSC, ARR, and CCR preload values
    config.TIMx->EGR |= TIM_EGR_UG; /* PDF Reference */

    // Restore timer state (re-enable counter if it was running)
    config.TIMx->CR1 = cr1_reg;
}

/**
 * @brief Start the PWM signal generation on the specified channel.
 * @param TRD_Channel: The logical channel identifier.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    PWM_Channel_Config_t config;
    if (get_pwm_config(TRD_Channel, &config) != 0)
    {
        // Handle invalid channel error
        return;
    }

    // Enable the counter
    config.TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference */

    // For TIM1 (Advanced Timer), enable the main output (MOE bit in BDTR)
    // General purpose timers (TIM2-5, TIM9-11) do not have BDTR/MOE,
    // their output is enabled by CCxE which is set in PWM_Init.
    if (config.TIMx == TIM1)
    {
        config.TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }
}

/**
 * @brief Stop the PWM signal output on the specified channel.
 * @param TRD_Channel: The logical channel identifier.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    PWM_Channel_Config_t config;
    if (get_pwm_config(TRD_Channel, &config) != 0)
    {
        // Handle invalid channel error
        return;
    }

    // Disable the counter
    config.TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

    // For TIM1 (Advanced Timer), disable the main output (MOE bit in BDTR)
    // General purpose timers (TIM2-5, TIM9-11) do not have BDTR/MOE.
    // To stop output on these, you would typically disable the specific channel output enable (CCxE)
    // or just disable the counter, which stops the clock. Disabling the counter is sufficient
    // as the output is tied to the timer operation.
    if (config.TIMx == TIM1)
    {
        config.TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */
    }
}

/**
 * @brief Disable all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through the configured channels and disables
 *        their associated timer and GPIO clocks.
 */
void PWM_PowerOff(void)
{
    // Iterate through all configured channels to disable their clocks
    for (size_t i = 0; i < sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]); ++i)
    {
        PWM_Channel_Config_t config = pwm_channel_map[i];

        // Disable the counter
        config.TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

        // Disable main output for TIM1
        if (config.TIMx == TIM1)
        {
            config.TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */
        }
        // For other timers, output disable is handled by CEN=0 or could clear CCxE,
        // but disabling the clock is sufficient for power off.

        // Disable Timer clock
        if (config.TIMx == TIM1)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; /* PDF Reference */
        }
        else if (config.TIMx == TIM2)
        {
            RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; /* PDF Reference */
        }
        else if (config.TIMx == TIM3)
        {
            RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; /* PDF Reference */
        }
        else if (config.TIMx == TIM4)
        {
            RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; /* PDF Reference */
        }
        // TIM5, TIM9-11 handled if needed in the map/config. Current map only has TIM1-4.
        else if (config.TIMx == TIM9)
        {
            RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN; /* PDF Reference */
        }


        // Disable GPIO clock (Need to be careful not to disable a port needed by other peripherals)
        // A safer approach would track which ports are used by *this module* only,
        // or require external clock disabling. For this implementation, we disable
        // the clock for each GPIO port listed in the map.
        // Note: This assumes no other peripheral *critical* for basic operation
        // relies on these specific GPIO ports being clocked at this moment.
        if (config.GPIOx == GPIOA)
        {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN; /* PDF Reference */
        }
        else if (config.GPIOx == GPIOB)
        {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN; /* PDF Reference */
        }
        else if (config.GPIOx == GPIOC)
        {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN; /* PDF Reference */
        }
        else if (config.GPIOx == GPIOH)
        {
             // GPIOH, only PH0/PH1 available according to PDF, not used in map.
             // RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN; /* PDF Reference */
        }
    }
}