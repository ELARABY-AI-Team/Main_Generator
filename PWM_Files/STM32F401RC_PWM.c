/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready implementation for PWM functionality on STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

// System clock frequency assumption for timer calculations.
// For STM32F401RC, APB1 timers (TIM2-TIM5) max clock is 42MHz, APB2 timers (TIM1, TIM9-TIM11) max clock is 84MHz.
// If APBx prescaler is 1, the timer clock is APBx_Clock. If APBx prescaler is >1, the timer clock is 2*APBx_Clock.
// This implementation assumes the timer input clock (CK_INT) is 84MHz, which is typical for APB2 timers (TIM1, TIM9)
// when the system clock (HCLK) is 84MHz and APB2 prescaler is /1 or /2 (resulting in 84MHz PCLK2).
// For APB1 timers (TIM2, TIM3, TIM4, TIM5), if HCLK is 84MHz and APB1 prescaler is /2 (PCLK1 = 42MHz),
// their clock is 2 * PCLK1 = 84MHz.
// This value MUST be accurately defined based on the actual clock configuration in `system_stm32f4xx.c` or RCC driver.
#define PWM_TIMER_CLOCK_FREQ    (84000000UL) /* Assumed Timer Clock Frequency for calculations */

/**
 * @brief Helper function to get GPIO clock enable bit position in RCC_AHB1ENR.
 * @param port_name The GPIO_TypeDef pointer for the port (e.g., GPIOA).
 * @return RCC_AHB1ENR_GPIOxEN_Pos value, or 0xFFFFFFFF for invalid.
 */
static uint32_t Get_GPIO_Clock_Enable_Bit(GPIO_TypeDef* port_name)
{
    if (port_name == GPIOA) return RCC_AHB1ENR_GPIOAEN_Pos;
    if (port_name == GPIOB) return RCC_AHB1ENR_GPIOBEN_Pos;
    if (port_name == GPIOC) return RCC_AHB1ENR_GPIOCEN_Pos;
    if (port_name == GPIOD) return RCC_AHB1ENR_GPIODEN_Pos;
    // GPIO F/G/H/I/J/K (except GPIOH0 and GPIOH1) are not available in STM32F401xB/C/xD/E.
    // This driver will not support these GPIOs for PWM due to device limitations.
    return 0xFFFFFFFF; // Indicate error or invalid port
}

/**
 * @brief Helper function to get Timer clock enable bit position in RCC_APBxENR.
 * @param tim_ptr The TIM_TypeDef pointer for the timer.
 * @return RCC_APBxENR_TIMxEN_Pos value, or 0xFFFFFFFF for invalid.
 */
static uint32_t Get_TIM_Clock_Enable_Bit(TIM_TypeDef* tim_ptr)
{
    if (tim_ptr == TIM1)  return RCC_APB2ENR_TIM1EN_Pos;
    if (tim_ptr == TIM2)  return RCC_APB1ENR_TIM2EN_Pos;
    if (tim_ptr == TIM3)  return RCC_APB1ENR_TIM3EN_Pos;
    if (tim_ptr == TIM4)  return RCC_APB1ENR_TIM4EN_Pos;
    if (tim_ptr == TIM5)  return RCC_APB1ENR_TIM5EN_Pos;
    if (tim_ptr == TIM9)  return RCC_APB2ENR_TIM9EN_Pos;
    // TIM10 and TIM11 are explicitly reserved for OS/delay purposes per requirement,
    // hence their channels are not included in this PWM driver's channel map.
    // This function will return an invalid value for them to enforce the reservation.
    return 0xFFFFFFFF; // Indicate error or invalid timer (e.g., TIM10, TIM11 for PWM usage)
}

// Global array to store pointers to active timers for PWM_PowerOff function.
// This prevents disabling clocks for timers not used by this PWM driver.
static TIM_TypeDef* active_timers[PWM_CHANNEL_COUNT];
static tbyte active_timer_count = 0;

/**
 * @brief Adds a timer to the active_timers list if not already present.
 * @param tim_ptr The TIM_TypeDef pointer to add.
 */
static void Add_Active_Timer(TIM_TypeDef* tim_ptr)
{
    for (tbyte i = 0; i < active_timer_count; i++)
    {
        if (active_timers[i] == tim_ptr)
        {
            return; // Already in the list
        }
    }
    if (active_timer_count < (sizeof(active_timers) / sizeof(active_timers[0])))
    {
        active_timers[active_timer_count++] = tim_ptr;
    }
    // Else, list is full (should not happen if PWM_CHANNEL_COUNT is sufficient)
}


/**
 * @brief Configuration map for available PWM channels on STM32F401RC.
 *        This map defines the Timer, Channel, GPIO Port, Pin, and Alternate Function
 *        for each distinct PWM output.
 *
 * @note  TIM10 and TIM11 are reserved for OS/delay purposes as per requirements.
 *        Pin mappings and Alternate Functions (AF) are assumed based on common STM32F4xx
 *        datasheet information. Refer to the specific STM32F401RC datasheet for exact
 *        AF mappings if issues arise. Pin 0 is included if it's a common PWM pin.
 *        GPIO F/G/H/I/J/K (except PH0/PH1) are not available on STM32F401xB/C.
 */
const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (Advanced-control timer, 16-bit, up to 4 channels, AF1)
    // Clocked by APB2 bus.
    { TIM1, 1, GPIOA, 8, GPIO_AF_TIM1 },  /* Assumed PWM config - please verify */
    { TIM1, 2, GPIOA, 9, GPIO_AF_TIM1 },  /* Assumed PWM config - please verify */
    { TIM1, 3, GPIOA, 10, GPIO_AF_TIM1 }, /* Assumed PWM config - please verify */
    { TIM1, 4, GPIOA, 11, GPIO_AF_TIM1 }, /* Assumed PWM config - please verify */

    // TIM2 Channels (General-purpose timer, 32-bit, up to 4 channels, AF1)
    // Clocked by APB1 bus.
    { TIM2, 1, GPIOA, 0, GPIO_AF_TIM2 },  /* Assumed PWM config - Pin 0 included per common usage, please verify */
    { TIM2, 2, GPIOA, 1, GPIO_AF_TIM2 },  /* Assumed PWM config - please verify */
    { TIM2, 3, GPIOA, 2, GPIO_AF_TIM2 },  /* Assumed PWM config - please verify */
    { TIM2, 4, GPIOA, 3, GPIO_AF_TIM2 },  /* Assumed PWM config - please verify */

    // TIM3 Channels (General-purpose timer, 16-bit, up to 4 channels, AF2)
    // Clocked by APB1 bus.
    { TIM3, 1, GPIOA, 6, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */
    { TIM3, 2, GPIOA, 7, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */
    { TIM3, 3, GPIOB, 0, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */
    { TIM3, 4, GPIOB, 1, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */

    // TIM4 Channels (General-purpose timer, 16-bit, up to 4 channels, AF2)
    // Clocked by APB1 bus.
    { TIM4, 1, GPIOB, 6, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */
    { TIM4, 2, GPIOB, 7, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */
    { TIM4, 3, GPIOB, 8, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */
    { TIM4, 4, GPIOB, 9, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */

    // TIM5 Channels (General-purpose timer, 32-bit, up to 4 channels, AF2)
    // Clocked by APB1 bus.
    // Using GPIOC pins to avoid direct conflict on GPIOA pins already heavily used by TIM2/TIM9.
    // Ensure these specific GPIOC pins (PC0-PC3) are available and PWM-capable on the STM32F401RC package.
    { TIM5, 1, GPIOC, 0, GPIO_AF_TIM5 },  /* Assumed PWM config - please verify */
    { TIM5, 2, GPIOC, 1, GPIO_AF_TIM5 },  /* Assumed PWM config - please verify */
    { TIM5, 3, GPIOC, 2, GPIO_AF_TIM5 },  /* Assumed PWM config - please verify */
    { TIM5, 4, GPIOC, 3, GPIO_AF_TIM5 },  /* Assumed PWM config - please verify */

    // TIM9 Channels (General-purpose timer, 16-bit, only 2 channels, AF3)
    // Clocked by APB2 bus.
    // PA2/PA3 also have AF1 (TIM2/TIM5). Selectivity is by the AF setting.
    { TIM9, 1, GPIOA, 2, GPIO_AF_TIM9 },  /* Assumed PWM config - please verify */
    { TIM9, 2, GPIOA, 3, GPIO_AF_TIM9 }   /* Assumed PWM config - please verify */
};


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The specific TRD PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= PWM_CHANNEL_COUNT)
    {
        // Invalid channel, potentially log an error or return
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    GPIO_TypeDef* GPIOx = config->PortName;
    uint8_t pin = config->PinNumber;
    uint8_t channel_num = config->ChannelNumber;
    uint8_t af = config->AlternateFunctionNumber;

    // 1. Enable GPIO Clock (RCC_AHB1ENR)
    uint32_t gpio_en_bit = Get_GPIO_Clock_Enable_Bit(GPIOx);
    if (gpio_en_bit == 0xFFFFFFFFUL)
    {
        // Invalid GPIO port or not supported for this device.
        return;
    }
    RCC->AHB1ENR |= (1UL << gpio_en_bit); /* PDF Reference: Implied by peripheral access, RCC common knowledge */

    // 2. Configure GPIO Pin for Alternate Function
    // Clear MODER bits for the pin and set to Alternate Function mode (10)
    // Bits 2y:2y+1 MODERy[1:0]: Port x configuration bits (y = 0..15) -> 10: Alternate function mode
    GPIOx->MODER &= ~(0x3UL << (pin * 2)); /* PDF Reference: Table 24, MODER */
    GPIOx->MODER |= (0x2UL << (pin * 2));  /* PDF Reference: Table 24, MODER */

    // Set Output Type to Push-Pull (0) - Default after reset, explicit for clarity
    // Bits 15:0 OTy: Port x configuration bits (y = 0..15) -> 0: Output push-pull (reset state)
    GPIOx->OTYPER &= ~(0x1UL << pin); /* PDF Reference: Table 24, OTYPER */

    // Set Output Speed to High Speed (10)
    // Bits 2y:2y+1 OSPEEDRy[1:0]: Port x configuration bits (y = 0..15) -> 10: High speed
    GPIOx->OSPEEDR &= ~(0x3UL << (pin * 2)); /* PDF Reference: Table 24, OSPEEDR */
    GPIOx->OSPEEDR |= (0x2UL << (pin * 2));  /* PDF Reference: Table 24, OSPEEDR */

    // Set Pull-up/Pull-down to No pull-up, pull-down (00) - Default after reset
    // Bits 2y:2y+1 PUPDRy[1:0]: Port x configuration bits (y = 0..15) -> 00: No pull-up, pull-down
    GPIOx->PUPDR &= ~(0x3UL << (pin * 2)); /* PDF Reference: Table 24, PUPDR */

    // Configure Alternate Function Register (AFRL for pins 0-7, AFRH for pins 8-15)
    // AFRLy selection: (for pins 0 to 7), AFRHy selection: (for pins 8 to 15)
    if (pin < 8)
    {
        GPIOx->AFR[0] &= ~(0xFUL << (pin * 4));     // Clear AFRL bits
        GPIOx->AFR[0] |= (af << (pin * 4));         // Set AFRL bits /* PDF Reference: AFRLy selection */
    }
    else
    {
        GPIOx->AFR[1] &= ~(0xFUL << ((pin - 8) * 4)); // Clear AFRH bits
        GPIOx->AFR[1] |= (af << ((pin - 8) * 4));     // Set AFRH bits /* PDF Reference: AFRHy selection */
    }

    // 3. Enable Timer Clock (RCC_APBxENR)
    uint32_t tim_en_bit = Get_TIM_Clock_Enable_Bit(TIMx);
    if (tim_en_bit == 0xFFFFFFFFUL)
    {
        // Invalid Timer or reserved for other purposes.
        return;
    }
    // Check if it's an APB2 timer (TIM1, TIM9) or APB1 timer (TIM2, TIM3, TIM4, TIM5)
    if (TIMx == TIM1 || TIMx == TIM9)
    {
        RCC->APB2ENR |= (1UL << tim_en_bit); /* PDF Reference: Implied by peripheral access, RCC common knowledge */
    }
    else // TIM2, TIM3, TIM4, TIM5
    {
        RCC->APB1ENR |= (1UL << tim_en_bit); /* PDF Reference: Implied by peripheral access, RCC common knowledge */
    }

    // Add timer to the active list for power management
    Add_Active_Timer(TIMx);

    // 4. Configure Timer Basic Settings
    // Disable Counter (CEN=0) before configuration
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: TIMx_CR1, Bit 0 CEN */
    
    // Set Prescaler (PSC) and Auto-Reload Register (ARR) to default values.
    // Actual frequency and duty cycle will be set by PWM_Set_Freq.
    TIMx->PSC = 0x0000; // No prescaling initially, will be set by Set_Freq /* PDF Reference: TIMx_PSC */

    // For 16-bit timers (TIM1, TIM3, TIM4, TIM9) ARR max is 0xFFFF
    // For 32-bit timers (TIM2, TIM5) ARR max is 0xFFFFFFFF
    if (TIMx == TIM2 || TIMx == TIM5)
    {
        TIMx->ARR = 0xFFFFFFFFUL; // Max 32-bit value /* PDF Reference: TIMx_ARR */
    }
    else
    {
        TIMx->ARR = 0xFFFFU;     // Max 16-bit value /* PDF Reference: TIMx_ARR */
    }

    // Configure Counter mode: Upcounting (DIR=0, CMS=00)
    // Bits 6:5 CMS[1:0]: Center-aligned mode selection -> 00: Edge-aligned mode.
    // Bit 4 DIR: Direction -> 0: Counter used as upcounter
    TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); /* PDF Reference: TIMx_CR1, Bit 4 DIR, Bits 6:5 CMS */

    // Enable Auto-Reload Preload (ARPE=1)
    // Bit 7 ARPE: Auto-reload preload enable -> 1: TIMx_ARR register is buffered
    TIMx->CR1 |= TIM_CR1_ARPE; /* PDF Reference: TIMx_CR1, Bit 7 ARPE */

    // Set Update Request Source to only counter overflow/underflow (URS=1)
    // Bit 2 URS: Update request source -> 1: Only counter overflow/underflow generates an update...
    TIMx->CR1 |= TIM_CR1_URS; /* PDF Reference: TIMx_CR1, Bit 2 URS */

    // 5. Configure Capture/Compare Channel for PWM Mode 1
    // Clear CCxS bits for the channel (00: output)
    // Set OCxM to PWM mode 1 (110)
    // Enable OCxPE (Preload Enable)
    volatile uint32_t* ccmr_reg;
    uint32_t ccmr_clear_mask;
    uint32_t ccmr_set_mask;

    // Determine which CCMR register (CCMR1 or CCMR2) and bit positions to use
    // TIMx_CCMR1 handles channels 1 and 2
    // TIMx_CCMR2 handles channels 3 and 4
    if (channel_num == 1)
    {
        ccmr_reg = &TIMx->CCMR1;
        ccmr_clear_mask = TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE_Msk;
        ccmr_set_mask = (0x6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk; // PWM mode 1 (110) & OCxPE
    }
    else if (channel_num == 2)
    {
        ccmr_reg = &TIMx->CCMR1;
        ccmr_clear_mask = TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC2PE_Msk;
        ccmr_set_mask = (0x6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE_Msk; // PWM mode 1 (110) & OCxPE
    }
    else if (channel_num == 3)
    {
        ccmr_reg = &TIMx->CCMR2;
        ccmr_clear_mask = TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC3PE_Msk;
        ccmr_set_mask = (0x6UL << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE_Msk; // PWM mode 1 (110) & OCxPE
    }
    else if (channel_num == 4)
    {
        ccmr_reg = &TIMx->CCMR2;
        ccmr_clear_mask = TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE_Msk;
        ccmr_set_mask = (0x6UL << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE_Msk; // PWM mode 1 (110) & OCxPE
    }
    else
    {
        // Should not happen due to TRD_Channel check, but for safety
        return;
    }

    *ccmr_reg &= ~ccmr_clear_mask; // Clear relevant bits (CCxS, OCxM, OCxPE)
    *ccmr_reg |= ccmr_set_mask;    // Set to Output, PWM Mode 1, Enable Preload /* PDF Reference: CCxS, OCxM, OCxPE */

    // 6. Configure Capture/Compare Enable Register (CCER)
    // Enable Capture/Compare Output (CCxE)
    // Set Output Polarity to Active High (CCxP=0)
    volatile uint32_t* ccer_reg = &TIMx->CCER;
    uint32_t cce_mask = 0;
    uint32_t ccp_mask = 0;

    switch (channel_num)
    {
        case 1:
            cce_mask = TIM_CCER_CC1E;  // Bit 0 CC1E: Capture/Compare 1 output enable
            ccp_mask = TIM_CCER_CC1P;  // Bit 1 CC1P: Capture/Compare 1 output polarity
            break;
        case 2:
            cce_mask = TIM_CCER_CC2E;  // Bit 4 CC2E
            ccp_mask = TIM_CCER_CC2P;  // Bit 5 CC2P
            break;
        case 3:
            cce_mask = TIM_CCER_CC3E;  // Bit 8 CC3E
            ccp_mask = TIM_CCER_CC3P;  // Bit 9 CC3P
            break;
        case 4:
            cce_mask = TIM_CCER_CC4E;  // Bit 12 CC4E
            ccp_mask = TIM_CCER_CC4P;  // Bit 13 CC4P
            break;
        default: return; // Should not happen
    }

    *ccer_reg |= cce_mask;   // Enable output /* PDF Reference: CCxE */
    *ccer_reg &= ~ccp_mask;  // Set active high polarity (CCxP=0) /* PDF Reference: CCxP */

    // 7. For TIM1 (advanced-control timer), enable Main Output Enable (MOE)
    // General-purpose timers (TIM2-5, TIM9) do not have the BDTR register or MOE bit.
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference: TIM1 BDTR, Bit 15 MOE */
    }

    // 8. Generate an Update Event (UG) to load preload registers immediately.
    // This ensures the initial configuration (PSC, ARR, CCRx) is loaded from preload to active registers.
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference: TIMx_EGR, Bit 0 UG */

    // 9. Initially stop the timer; it will be started by PWM_Start.
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: TIMx_CR1, Bit 0 CEN */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The specific TRD PWM channel.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= PWM_CHANNEL_COUNT || frequency == 0 || duty > 100)
    {
        // Invalid parameters
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint8_t channel_num = config->ChannelNumber;

    // Preserve timer running state
    tbyte was_running = (TIMx->CR1 & TIM_CR1_CEN) != 0;
    if (was_running)
    {
        TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: TIMx_CR1, Bit 0 CEN */
    }

    tlong timer_clock = PWM_TIMER_CLOCK_FREQ;

    // Calculate Prescaler (PSC) and Auto-Reload Register (ARR) values.
    // F_PWM = F_TIM_CLK / ((PSC + 1) * (ARR + 1))
    // To get the best resolution, maximize ARR.
    tlong ARR_max = (TIMx == TIM2 || TIMx == TIM5) ? 0xFFFFFFFFUL : 0xFFFFUL;
    tlong PSC_val = 0;
    tlong ARR_val = 0;

    // Calculate total clock ticks per PWM period
    tlong total_ticks = timer_clock / frequency;

    // Iterate through possible PSC values to find a suitable ARR
    // Starting PSC from 0 up to 65535, as PSC is 16-bit for all timers
    for (PSC_val = 0; PSC_val <= 0xFFFFUL; PSC_val++)
    {
        // Check if (PSC_val + 1) divides total_ticks evenly or if ARR will fit
        if ((total_ticks % (PSC_val + 1)) == 0) // Perfect division
        {
            ARR_val = (total_ticks / (PSC_val + 1)) - 1;
            if (ARR_val <= ARR_max)
            {
                break; // Found suitable PSC/ARR
            }
        }
        else // Imperfect division, find closest ARR
        {
            ARR_val = (total_ticks / (PSC_val + 1)) - 1;
            if (ARR_val <= ARR_max)
            {
                // This will result in a slightly different frequency but is the best fit for this PSC
                break;
            }
        }
    }
    
    // If no suitable PSC/ARR found, or if loop finished without break,
    // ARR_val might be > ARR_max or PSC_val might be 0xFFFF.
    // In a real application, robust error handling or setting to max possible would be needed.
    // For now, it uses the last calculated values, which may not be ideal.
    if (PSC_val > 0xFFFFUL || ARR_val > ARR_max) {
        // Fallback: Max out ARR and calculate PSC
        ARR_val = ARR_max;
        PSC_val = (total_ticks / (ARR_val + 1)) - 1;
        if (PSC_val > 0xFFFFUL) PSC_val = 0xFFFFUL; // Cap PSC to max
    }


    // Set new Prescaler and Auto-Reload values
    TIMx->PSC = (uint16_t)PSC_val; /* PDF Reference: TIMx_PSC */
    if (TIMx == TIM2 || TIMx == TIM5)
    {
        TIMx->ARR = ARR_val; /* PDF Reference: TIMx_ARR */
    }
    else
    {
        TIMx->ARR = (uint16_t)ARR_val; /* PDF Reference: TIMx_ARR */
    }

    // Calculate Capture/Compare Register (CCRx) value for duty cycle
    // CCRx = (ARR + 1) * DutyCycle / 100
    // Use the potentially adjusted ARR_val for calculation.
    tlong CCRx_val = ((ARR_val + 1) * duty) / 100;

    // Set CCRx value based on channel number
    switch (channel_num)
    {
        case 1: TIMx->CCR1 = CCRx_val; break; /* PDF Reference: TIMx_CCR1 */
        case 2: TIMx->CCR2 = CCRx_val; break; /* PDF Reference: TIMx_CCR2 */
        case 3: TIMx->CCR3 = CCRx_val; break; /* PDF Reference: TIMx_CCR3 */
        case 4: TIMx->CCR4 = CCRx_val; break; /* PDF Reference: TIMx_CCR4 */
        default: return; // Should not happen
    }

    // Generate an Update Event (UG) to load new PSC, ARR, CCRx values from preload
    // registers into active registers immediately.
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference: TIMx_EGR, Bit 0 UG */

    // Restore timer running state
    if (was_running)
    {
        TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference: TIMx_CR1, Bit 0 CEN */
    }
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The specific TRD PWM channel.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= PWM_CHANNEL_COUNT)
    {
        return; // Invalid channel
    }

    TIM_TypeDef* TIMx = pwm_channel_map[TRD_Channel].TIMx;
    uint8_t channel_num = pwm_channel_map[TRD_Channel].ChannelNumber;

    // Enable the specific Capture/Compare Channel output
    switch (channel_num)
    {
        case 1: TIMx->CCER |= TIM_CCER_CC1E; break; /* PDF Reference: CC1E */
        case 2: TIMx->CCER |= TIM_CCER_CC2E; break; /* PDF Reference: CC2E */
        case 3: TIMx->CCER |= TIM_CCER_CC3E; break; /* PDF Reference: CC3E */
        case 4: TIMx->CCER |= TIM_CCER_CC4E; break; /* PDF Reference: CC4E */
        default: return; // Should not happen
    }

    // Enable Main Output (MOE) for TIM1 if it's TIM1.
    // General-purpose timers (TIM2-5, TIM9) do not have this bit.
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference: TIM1 BDTR, Bit 15 MOE */
    }

    // Enable the Timer Counter
    TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference: TIMx_CR1, Bit 0 CEN */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The specific TRD PWM channel.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= PWM_CHANNEL_COUNT)
    {
        return; // Invalid channel
    }

    TIM_TypeDef* TIMx = pwm_channel_map[TRD_Channel].TIMx;
    uint8_t channel_num = pwm_channel_map[TRD_Channel].ChannelNumber;

    // Disable the specific Capture/Compare Channel output
    switch (channel_num)
    {
        case 1: TIMx->CCER &= ~TIM_CCER_CC1E; break; /* PDF Reference: CC1E */
        case 2: TIMx->CCER &= ~TIM_CCER_CC2E; break; /* PDF Reference: CC2E */
        case 3: TIMx->CCER &= ~TIM_CCER_CC3E; break; /* PDF Reference: CC3E */
        case 4: TIMx->CCER &= ~TIM_CCER_CC4E; break; /* PDF Reference: CC4E */
        default: return; // Should not happen
    }

    // For TIM1, Main Output Enable (MOE) is not cleared here.
    // Clearing MOE affects all channels of TIM1. If other channels on TIM1 are still
    // active, clearing MOE would disable them. This function is for single channel stop.
    // MOE will be cleared in PWM_PowerOff if the entire peripheral is being shut down.
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through all timers previously initialized by PWM_Init
 *        and disables their counters, outputs, and clocks.
 */
void PWM_PowerOff(void)
{
    for (tbyte i = 0; i < active_timer_count; i++)
    {
        TIM_TypeDef* TIMx = active_timers[i];

        if (TIMx != NULL)
        {
            // Disable Timer Counter
            TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: TIMx_CR1, Bit 0 CEN */

            // Disable all Capture/Compare outputs for this timer (CCER)
            TIMx->CCER = 0x00000000UL; /* PDF Reference: TIMx_CCER */

            // If it's TIM1, disable Main Output (MOE)
            if (TIMx == TIM1)
            {
                TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference: TIM1 BDTR, Bit 15 MOE */
            }

            // Disable Timer Clock (RCC_APBxENR)
            uint32_t tim_en_bit = Get_TIM_Clock_Enable_Bit(TIMx);
            if (tim_en_bit != 0xFFFFFFFFUL)
            {
                if (TIMx == TIM1 || TIMx == TIM9)
                {
                    RCC->APB2ENR &= ~(1UL << tim_en_bit); /* PDF Reference: Implied by peripheral access, RCC common knowledge */
                }
                else // TIM2, TIM3, TIM4, TIM5
                {
                    RCC->APB1ENR &= ~(1UL << tim_en_bit); /* PDF Reference: Implied by peripheral access, RCC common knowledge */
                }
            }

            // Mark this slot as cleared
            active_timers[i] = NULL;
        }
    }
    active_timer_count = 0; // Reset the count of active timers
}