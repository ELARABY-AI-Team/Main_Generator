/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM driver for STM32F401RC microcontroller
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/


#include "STM32F401RC_PWM.h" // Includes stm32f4xx.h and custom types as specified

/*
 * Note on Timer Reservation:
 * As per the requirement, TIM2 and TIM5 (which are 32-bit general-purpose timers)
 * are reserved and excluded from this PWM implementation. This allows them to be
 * used for other purposes, such as an Operating System's tick or precise delay functions,
 * which often benefit from the higher resolution or dedicated timing capabilities.
 * The PWM implementation will utilize TIM1, TIM3, TIM4, TIM9, TIM10, and TIM11.
 */

// --- Internal Helper Defines for Register Access ---

// RCC AHB1 peripheral clock enable register (RCC_AHB1ENR) bit positions
#define RCC_AHB1ENR_GPIOAEN     (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN     (1UL << 1)
#define RCC_AHB1ENR_GPIOCEN     (1UL << 2)
#define RCC_AHB1ENR_GPIODEN     (1UL << 3)  // GPIO D is available on some F401RC packages
#define RCC_AHB1ENR_GPIOEEN     (1UL << 4)  // GPIO E is available on some F401RC packages
#define RCC_AHB1ENR_GPIOHEN     (1UL << 7)  // GPIOH0 and GPIOH1 are available on STM32F401xB/C/xD/E

// RCC APB1 peripheral clock enable register (RCC_APB1ENR) bit positions
#define RCC_APB1ENR_TIM2EN      (1UL << 0)  // Reserved
#define RCC_APB1ENR_TIM3EN      (1UL << 1)
#define RCC_APB1ENR_TIM4EN      (1UL << 2)
#define RCC_APB1ENR_TIM5EN      (1UL << 3)  // Reserved

// RCC APB2 peripheral clock enable register (RCC_APB2ENR) bit positions
#define RCC_APB2ENR_TIM1EN      (1UL << 0)
#define RCC_APB2ENR_TIM9EN      (1UL << 16)
#define RCC_APB2ENR_TIM10EN     (1UL << 17)
#define RCC_APB2ENR_TIM11EN     (1UL << 18)

// GPIO Register Bit Masks and Positions (from RM0368 Rev 5)
#define GPIO_MODER_MODE_Pos(pin)    (pin * 2U)
#define GPIO_MODER_MODE_Msk(pin)    (0x3UL << GPIO_MODER_MODE_Pos(pin))
#define GPIO_MODER_AF_MODE          (0x2UL) // Alternate Function Mode (10)

#define GPIO_OTYPER_OT_Pos(pin)     (pin)
#define GPIO_OTYPER_OT_Msk(pin)     (0x1UL << GPIO_OTYPER_OT_Pos(pin))
#define GPIO_OTYPER_PUSHPULL        (0x0UL) // Push-Pull (0)

#define GPIO_OSPEEDR_OSPEED_Pos(pin) (pin * 2U)
#define GPIO_OSPEEDR_OSPEED_Msk(pin) (0x3UL << GPIO_OSPEEDR_OSPEED_Pos(pin))
#define GPIO_OSPEEDR_VHIGH_SPEED    (0x3UL) // Very High Speed (11)

#define GPIO_PUPDR_PUPD_Pos(pin)    (pin * 2U)
#define GPIO_PUPDR_PUPD_Msk(pin)    (0x3UL << GPIO_PUPDR_PUPD_Pos(pin))
#define GPIO_PUPDR_NO_PULL          (0x0UL) // No Pull-up, Pull-down (00)

#define GPIO_AFR_AFR_Pos(pin)       ((pin % 8U) * 4U)
#define GPIO_AFR_AFR_Msk(pin)       (0xFUL << GPIO_AFR_AFR_Pos(pin))

// Alternate Function values (from STM32F4 reference manual, common for general purpose timers)
#define GPIO_AF_TIM1    (0x01U) // AF1 (used for TIM1, TIM2)
#define GPIO_AF_TIM3    (0x02U) // AF2 (used for TIM3, TIM4, TIM5)
#define GPIO_AF_TIM4    (0x02U) // AF2 (used for TIM3, TIM4, TIM5)
#define GPIO_AF_TIM9    (0x03U) // AF3 (used for TIM9, TIM10, TIM11)
#define GPIO_AF_TIM10   (0x03U) // AF3 (used for TIM9, TIM10, TIM11)
#define GPIO_AF_TIM11   (0x03U) // AF3 (used for TIM9, TIM10, TIM11)


// Timer Register Bit Masks and Positions (common for general purpose timers, from RM0368 Rev 5)
// TIMx_CR1 (Control Register 1)
#define TIM_CR1_CEN_Msk             (0x1UL << 0U)  // Counter enable
#define TIM_CR1_UDIS_Msk            (0x1UL << 1U)  // Update disable
#define TIM_CR1_URS_Msk             (0x1UL << 2U)  // Update request source (only counter overflow/underflow)
#define TIM_CR1_DIR_Msk             (0x1UL << 4U)  // Direction (0: upcounter)
#define TIM_CR1_CMS_Msk             (0x3UL << 5U)  // Center-aligned mode selection (00: Edge-aligned)
#define TIM_CR1_ARPE_Msk            (0x1UL << 7U)  // Auto-reload preload enable
#define TIM_CR1_CKD_Msk             (0x3UL << 8U)  // Clock division (00: no clock division)

// TIMx_CCMR1/CCMR2 (Capture/Compare Mode Registers) - Output Compare Mode fields
// For OCxM: Bits 6:4 for Channel 1/3, Bits 14:12 for Channel 2/4
#define TIM_CCMR_OCM_Pos(channel)    (((channel) % 2 == 1) ? 4U : 12U)
#define TIM_CCMR_OCM_Msk(channel)    (0x7UL << TIM_CCMR_OCM_Pos(channel))
#define TIM_OCM_PWM1                 (0x6UL)        // PWM mode 1 (110)

// For OCxPE: Bits 3 for Channel 1/3, Bits 11 for Channel 2/4
#define TIM_CCMR_OCPE_Pos(channel)   (((channel) % 2 == 1) ? 3U : 11U)
#define TIM_CCMR_OCPE_Msk(channel)   (0x1UL << TIM_CCMR_OCPE_Pos(channel))
#define TIM_OCPE_ENABLE              (0x1UL)        // Preload enabled

// TIMx_CCER (Capture/Compare Enable Register)
// For CCxE: Bits 0, 4, 8, 12 for Channels 1, 2, 3, 4 respectively
#define TIM_CCER_CCE_Pos(channel)    (((channel) - 1U) * 4U)
#define TIM_CCER_CCE_Msk(channel)    (0x1UL << TIM_CCER_CCE_Pos(channel))
#define TIM_CCE_ENABLE               (0x1UL)        // Enable Capture/Compare output

// For CCxP: Bits 1, 5, 9, 13 for Channels 1, 2, 3, 4 respectively
#define TIM_CCER_CCP_Pos(channel)    (((channel) - 1U) * 4U + 1U)
#define TIM_CCER_CCP_Msk(channel)    (0x1UL << TIM_CCER_CCP_Pos(channel))
#define TIM_CCP_ACTIVE_HIGH          (0x0UL)        // Output Polarity (0: active high)

// For CCxNE: Bits 2, 6, 10, 14 for Channels 1, 2, 3, 4 respectively (only for TIM1 complementary outputs)
#define TIM_CCER_CCNE_Pos(channel)   (((channel) - 1U) * 4U + 2U)
#define TIM_CCER_CCNE_Msk(channel)   (0x1UL << TIM_CCER_CCNE_Pos(channel))

// For CCxNP: Bits 3, 7, 11, 15 for Channels 1, 2, 3, 4 respectively (only for TIM1 complementary outputs)
#define TIM_CCER_CCNP_Pos(channel)   (((channel) - 1U) * 4U + 3U)
#define TIM_CCER_CCNP_Msk(channel)   (0x1UL << TIM_CCER_CCNP_Pos(channel))

// TIMx_BDTR (Break and Dead-Time Register) - only for TIM1
#define TIM_BDTR_MOE_Msk            (0x1UL << 15U) // Main Output Enable
#define TIM_MOE_ENABLE              (0x1UL)        // Main output enabled

// TIMx_EGR (Event Generation Register)
#define TIM_EGR_UG_Msk              (0x1UL << 0U)  // Update Generation

/**
 * @brief Production-ready definition of the PWM channel configuration array.
 * This array maps each TRD_Channel_t enum entry to its specific hardware configuration.
 *
 * Each entry follows the format:
 *   {TIMx, ChannelNumber, PortName, PinNumber, AlternateFunctionNumber, TimerClockFreq}
 *
 * Timer Reservation Note:
 * TIM2 and TIM5 are intentionally excluded as per requirements to reserve them for
 * potential OS or high-precision delay purposes.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (Advanced-control timer, APB2 bus)
    // TIM1 is a 16-bit timer. AF1 is typical for TIM1 outputs.
    // Timer clock frequency assumed to be HCLK (84MHz) due to common APB prescaler setup.
    {TIM1, 1, GPIOA, 8,  GPIO_AF_TIM1, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA8 is TIM1_CH1, please verify */
    {TIM1, 2, GPIOA, 9,  GPIO_AF_TIM1, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA9 is TIM1_CH2, please verify */
    {TIM1, 3, GPIOA, 10, GPIO_AF_TIM1, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA10 is TIM1_CH3, please verify */
    {TIM1, 4, GPIOA, 11, GPIO_AF_TIM1, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA11 is TIM1_CH4, please verify */

    // TIM3 Channels (General-purpose timer, APB1 bus)
    // TIM3 is a 16-bit timer. AF2 is typical for TIM3 outputs.
    // Pin 0 is included as it is commonly PWM-capable on STM32.
    {TIM3, 1, GPIOA, 6,  GPIO_AF_TIM3, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA6 is TIM3_CH1, please verify */
    {TIM3, 2, GPIOA, 7,  GPIO_AF_TIM3, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA7 is TIM3_CH2, please verify */
    {TIM3, 3, GPIOB, 0,  GPIO_AF_TIM3, TIM_CLOCK_FREQ}, /* Assumed PWM config - PB0 is TIM3_CH3, please verify */
    {TIM3, 4, GPIOB, 1,  GPIO_AF_TIM3, TIM_CLOCK_FREQ}, /* Assumed PWM config - PB1 is TIM3_CH4, please verify */

    // TIM4 Channels (General-purpose timer, APB1 bus)
    // TIM4 is a 16-bit timer. AF2 is typical for TIM4 outputs.
    // Note: PB8 and PB9 are common shared pins with other timers (TIM10, TIM11).
    // For this array, we define their primary common usage for TIM4.
    {TIM4, 1, GPIOB, 6,  GPIO_AF_TIM4, TIM_CLOCK_FREQ}, /* Assumed PWM config - PB6 is TIM4_CH1, please verify */
    {TIM4, 2, GPIOB, 7,  GPIO_AF_TIM4, TIM_CLOCK_FREQ}, /* Assumed PWM config - PB7 is TIM4_CH2, please verify */
    {TIM4, 3, GPIOB, 8,  GPIO_AF_TIM4, TIM_CLOCK_FREQ}, /* Assumed PWM config - PB8 is TIM4_CH3 (AF2); also TIM10_CH1(AF3) possible, please verify */
    {TIM4, 4, GPIOB, 9,  GPIO_AF_TIM4, TIM_CLOCK_FREQ}, /* Assumed PWM config - PB9 is TIM4_CH4 (AF2); also TIM11_CH1(AF3) possible, please verify */

    // TIM9 Channels (General-purpose timer, APB2 bus)
    // TIM9 is a 16-bit timer. AF3 is typical for TIM9 outputs.
    {TIM9, 1, GPIOA, 2,  GPIO_AF_TIM9, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA2 is TIM9_CH1, please verify */
    {TIM9, 2, GPIOA, 3,  GPIO_AF_TIM9, TIM_CLOCK_FREQ}, /* Assumed PWM config - PA3 is TIM9_CH2, please verify */

    // TIM10 Channel (General-purpose timer, APB2 bus) - only one channel (CH1)
    // TIM10 is a 16-bit timer. AF3 is typical for TIM10_CH1.
    // Note: PB8 is shared with TIM4_CH3. Different Alternate Functions (AF2 vs AF3) allow this.
    {TIM10, 1, GPIOB, 8, GPIO_AF_TIM10, TIM_CLOCK_FREQ}, /* Assumed PWM config - PB8 is TIM10_CH1 (AF3); also TIM4_CH3(AF2) possible, please verify */

    // TIM11 Channel (General-purpose timer, APB2 bus) - only one channel (CH1)
    // TIM11 is a 16-bit timer. AF3 is typical for TIM11_CH1.
    // Note: PB9 is shared with TIM4_CH4. Different Alternate Functions (AF2 vs AF3) allow this.
    {TIM11, 1, GPIOB, 9, GPIO_AF_TIM11, TIM_CLOCK_FREQ}  /* Assumed PWM config - PB9 is TIM11_CH1 (AF3); also TIM4_CH4(AF2) possible, please verify */
};


/**
 * @brief Retrieves the PWM channel configuration based on the TRD_Channel_t enum.
 * @param channel The TRD_Channel_t enum value.
 * @return Pointer to the PWM_Channel_Config_t struct, or NULL if invalid.
 */
static const PWM_Channel_Config_t* get_pwm_config(TRD_Channel_t channel)
{
    if (channel >= NUM_PWM_CHANNELS)
    {
        // Handle error: Invalid channel index
        return NULL;
    }
    return &pwm_channel_map[channel];
}

/**
 * @brief Enables the clock for the specified GPIO port.
 * @param port Pointer to the GPIO_TypeDef structure.
 */
static void enable_gpio_clock(GPIO_TypeDef *port)
{
    // Enable clock for GPIOA
    if (port == GPIOA)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* PDF Reference */
    }
    // Enable clock for GPIOB
    else if (port == GPIOB)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* PDF Reference */
    }
    // Enable clock for GPIOC
    else if (port == GPIOC)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* PDF Reference */
    }
    // Enable clock for GPIOD (if used and available on specific F401RC package)
    else if (port == GPIOD)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* PDF Reference */
    }
    // Enable clock for GPIOE (if used and available on specific F401RC package)
    else if (port == GPIOE)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; /* PDF Reference */
    }
    // Enable clock for GPIOH (if used and available on specific F401RC package)
    else if (port == GPIOH)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; /* PDF Reference */
    }
    // A small delay might be needed for the clock to stabilize (typically 2 APB cycles)
    // Read the register to ensure the clock is enabled
    (void)RCC->AHB1ENR;
}

/**
 * @brief Enables the clock for the specified Timer peripheral.
 * @param tim Pointer to the TIM_TypeDef structure.
 */
static void enable_timer_clock(TIM_TypeDef *tim)
{
    // Enable clock for TIM1 (APB2)
    if (tim == TIM1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* PDF Reference */
    }
    // Enable clock for TIM3 (APB1)
    else if (tim == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* PDF Reference */
    }
    // Enable clock for TIM4 (APB1)
    else if (tim == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* PDF Reference */
    }
    // Enable clock for TIM9 (APB2)
    else if (tim == TIM9)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* PDF Reference */
    }
    // Enable clock for TIM10 (APB2)
    else if (tim == TIM10)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; /* PDF Reference */
    }
    // Enable clock for TIM11 (APB2)
    else if (tim == TIM11)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; /* PDF Reference */
    }
    // A small delay might be needed for the clock to stabilize (typically 2 APB cycles)
    // Read the register to ensure the clock is enabled
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;
}

/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The desired PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = get_pwm_config(TRD_Channel);
    if (config == NULL)
    {
        // Handle invalid channel: Log error or return specific error code
        return;
    }

    // 1. Enable GPIO peripheral clock
    enable_gpio_clock(config->PortName);

    // 2. Configure GPIO pin for Alternate Function mode
    // Clear MODER bits for the pin (reset to Input mode (00) first, then set to AF (10))
    config->PortName->MODER &= ~GPIO_MODER_MODE_Msk(config->PinNumber); /* PDF Reference */
    // Set MODER bits to Alternate Function mode (10)
    config->PortName->MODER |= (GPIO_MODER_AF_MODE << GPIO_MODER_MODE_Pos(config->PinNumber)); /* PDF Reference */

    // Configure Output Type to Push-Pull (0)
    config->PortName->OTYPER &= ~GPIO_OTYPER_OT_Msk(config->PinNumber); /* PDF Reference */
    config->PortName->OTYPER |= (GPIO_OTYPER_PUSHPULL << GPIO_OTYPER_OT_Pos(config->PinNumber)); /* PDF Reference */

    // Configure Output Speed to Very High Speed (11)
    config->PortName->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED_Msk(config->PinNumber); /* PDF Reference */
    config->PortName->OSPEEDR |= (GPIO_OSPEEDR_VHIGH_SPEED << GPIO_OSPEEDR_OSPEED_Pos(config->PinNumber)); /* PDF Reference */

    // Configure Pull-up/Pull-down to No Pull (00)
    config->PortName->PUPDR &= ~GPIO_PUPDR_PUPD_Msk(config->PinNumber); /* PDF Reference */
    config->PortName->PUPDR |= (GPIO_PUPDR_NO_PULL << GPIO_PUPDR_PUPD_Pos(config->PinNumber)); /* PDF Reference */

    // Configure Alternate Function (AFR[L/H] for pins 0-7 and 8-15 respectively)
    if (config->PinNumber < 8)
    {
        config->PortName->AFRL &= ~GPIO_AFR_AFR_Msk(config->PinNumber); /* PDF Reference */
        config->PortName->AFRL |= ((uint32_t)config->AlternateFunction << GPIO_AFR_AFR_Pos(config->PinNumber)); /* PDF Reference */
    }
    else
    {
        config->PortName->AFRH &= ~GPIO_AFR_AFR_Msk(config->PinNumber); /* PDF Reference */
        config->PortName->AFRH |= ((uint32_t)config->AlternateFunction << GPIO_AFR_AFR_Pos(config->PinNumber)); /* PDF Reference */
    }

    // 3. Enable Timer peripheral clock
    enable_timer_clock(config->TIMx);

    // 4. Configure Timer for PWM generation
    // Disable counter before configuration to ensure safe updates
    config->TIMx->CR1 &= ~TIM_CR1_CEN_Msk; /* PDF Reference */

    // Reset TIMx_CR1 to a known state (excluding CEN which is cleared above)
    // Set ARPE (Auto-reload preload enable) for buffering ARR
    // Set URS (Update Request Source) so only overflow/underflow generates UEV, not UG bit
    // Set DIR=0 (upcounting mode)
    // Set CMS=00 (edge-aligned mode)
    // Set CKD=00 (no clock division for internal timer clock)
    config->TIMx->CR1 &= ~(TIM_CR1_DIR_Msk | TIM_CR1_CMS_Msk | TIM_CR1_CKD_Msk | TIM_CR1_URS_Msk | TIM_CR1_ARPE_Msk); /* PDF Reference */
    config->TIMx->CR1 |= (TIM_CR1_ARPE_Msk | TIM_CR1_URS_Msk); /* PDF Reference */


    // Configure Output Compare Mode (OCxM) and Output Compare Preload Enable (OCxPE)
    volatile uint32_t *ccmr_reg = NULL;
    uint8_t channel_bit_offset = 0; // Offset within CCMR1 or CCMR2
    
    // Determine which CCMR register to use based on channel number
    if (config->ChannelNumber == 1 || config->ChannelNumber == 2)
    {
        ccmr_reg = &config->TIMx->CCMR1; /* PDF Reference */
        channel_bit_offset = (config->ChannelNumber == 1) ? 0 : 8;
    }
    else if (config->ChannelNumber == 3 || config->ChannelNumber == 4)
    {
        ccmr_reg = &config->TIMx->CCMR2; /* PDF Reference */
        channel_bit_offset = (config->ChannelNumber == 3) ? 0 : 8;
    }
    else
    {
        // For TIM10/11, they only have Channel 1, so CCMR1 is always used.
        // This case is for an invalid channel number for general purpose timers that have 4 channels
        // or a timer that doesn't fit the expected channel patterns.
        if (config->TIMx != TIM10 && config->TIMx != TIM11) {
             return; // Should not happen with valid pwm_channel_map, but good for robustness
        }
        ccmr_reg = &config->TIMx->CCMR1; // For TIM10/11, ChannelNumber will be 1
        channel_bit_offset = 0;
    }

    // Clear CCxS (capture/compare selection) bits to configure channel as output (00)
    *ccmr_reg &= ~(0x3UL << channel_bit_offset); /* PDF Reference */

    // Clear OCxM (Output Compare Mode) and OCxPE (Output Compare Preload Enable) bits
    *ccmr_reg &= ~(TIM_CCMR_OCM_Msk(config->ChannelNumber) | TIM_CCMR_OCPE_Msk(config->ChannelNumber)); /* PDF Reference */
    // Set OCxM to PWM Mode 1 (110) and enable OCxPE (preload for duty cycle)
    *ccmr_reg |= (TIM_OCM_PWM1 << TIM_CCMR_OCM_Pos(config->ChannelNumber)); /* PDF Reference */
    *ccmr_reg |= (TIM_OCPE_ENABLE << TIM_CCMR_OCPE_Pos(config->ChannelNumber)); /* PDF Reference */


    // Configure Capture/Compare Enable Register (CCER)
    // Clear relevant bits: CCxE (output enable), CCxP (polarity), CCxNE/CCxNP (complementary for TIM1)
    uint32_t ccer_channel_mask = TIM_CCER_CCE_Msk(config->ChannelNumber) | TIM_CCER_CCP_Msk(config->ChannelNumber);

    // For TIM1 (Advanced-control timer), also handle complementary output settings (CCxNE, CCxNP)
    // General purpose timers (TIM3, TIM4, TIM9, TIM10, TIM11) do not have complementary outputs;
    // their CCxNE/CCxNP bits are typically reserved or implicitly 0.
    if (config->TIMx == TIM1) {
        ccer_channel_mask |= (TIM_CCER_CCNE_Msk(config->ChannelNumber) | TIM_CCER_CCNP_Msk(config->ChannelNumber));
    }
    config->TIMx->CCER &= ~ccer_channel_mask; /* PDF Reference */

    // Enable output (CCxE = 1) and set polarity to Active High (CCxP = 0)
    config->TIMx->CCER |= (TIM_CCE_ENABLE << TIM_CCER_CCE_Pos(config->ChannelNumber)); /* PDF Reference */
    config->TIMx->CCER |= (TIM_CCP_ACTIVE_HIGH << TIM_CCER_CCP_Pos(config->ChannelNumber)); /* PDF Reference */

    // For TIM1 (Advanced-control timer), Main Output Enable (MOE) must be set in BDTR
    if (config->TIMx == TIM1)
    {
        config->TIMx->BDTR |= TIM_BDTR_MOE_Msk; /* PDF Reference */
    }

    // Generate an update event to load all preload registers (PSC, ARR, CCRx) into active registers.
    config->TIMx->EGR |= TIM_EGR_UG_Msk; /* PDF Reference */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    const PWM_Channel_Config_t* config = get_pwm_config(TRD_Channel);
    if (config == NULL || frequency == 0 || duty > 100)
    {
        // Handle invalid input: log error or return specific error code
        return;
    }

    // Temporarily disable the counter to ensure safe configuration updates.
    // This prevents glitches if a new period/duty cycle is applied mid-cycle.
    config->TIMx->CR1 &= ~TIM_CR1_CEN_Msk; /* PDF Reference */

    uint32_t timer_clock_freq = config->TimerClockFreq;
    uint32_t auto_reload = 0;
    uint16_t prescaler = 0;

    // Determine max ARR value based on timer's bit-width (16-bit for selected timers)
    uint32_t max_arr_value = 0xFFFFUL; // For TIM1, TIM3, TIM4, TIM9, TIM10, TIM11

    // Calculate optimal Prescaler (PSC) and Auto-Reload Register (ARR) values
    // Goal: Maximize ARR for better resolution, while keeping PSC as small as possible.
    // Iteratively find PSC that allows ARR to fit within register limits.
    // Frequency = Timer_Clock_Freq / ((ARR + 1) * (PSC + 1))
    // (ARR + 1) * (PSC + 1) = Timer_Clock_Freq / Frequency
    // (ARR + 1) = (Timer_Clock_Freq / Frequency) / (PSC + 1)
    // ARR = (Timer_Clock_Freq / (Frequency * (PSC + 1))) - 1
    
    // Start with minimum prescaler (0, means division by 1)
    for (prescaler = 0; prescaler <= 0xFFFFUL; prescaler++) /* PDF Reference */
    {
        // Calculate the theoretical (ARR + 1) value. Use 64-bit for intermediate calculation to prevent overflow.
        uint64_t total_clocks_per_period = (uint64_t)timer_clock_freq / frequency;
        
        // Calculate the potential ARR value
        uint66_t potential_arr = total_clocks_per_period / (prescaler + 1);

        if (potential_arr > 0 && potential_arr <= (max_arr_value + 1UL))
        {
            auto_reload = (uint32_t)potential_arr - 1UL; /* PDF Reference */
            break; // Found suitable values
        }
    }

    // Validate if a suitable (PSC, ARR) pair was found
    if (prescaler > 0xFFFFUL)
    {
        // Could not find suitable PSC/ARR combination for the desired frequency
        // This means the frequency is either too low or too high for the timer's capabilities.
        return;
    }

    // Apply the calculated prescaler and auto-reload values
    config->TIMx->PSC = prescaler; /* PDF Reference */
    config->TIMx->ARR = auto_reload; /* PDF Reference */

    // Calculate Capture Compare Register (CCRx) value for duty cycle
    // Duty_Cycle = (CCRx + 1) / (ARR + 1) * 100%
    // CCRx = ((ARR + 1) * Duty_Cycle / 100) - 1 (approximately)
    // To avoid floating point, we can use integer arithmetic and approximate:
    uint32_t ccr_value = ((auto_reload + 1UL) * duty) / 100UL; /* PDF Reference */

    // Ensure CCR value does not exceed ARR value, as CCRx >= ARR+1 for 100% duty (or ARR for 99.x%)
    // And ensure it's not less than 0.
    if (ccr_value > auto_reload)
    {
        ccr_value = auto_reload; // Max duty cycle, ensures CCRx is not greater than ARR
    }
    if (duty == 0) { // For 0% duty cycle, set CCR to 0. Some timers behave differently for 0%.
        ccr_value = 0;
    }

    // Set CCRx register based on channel number
    switch (config->ChannelNumber)
    {
        case 1: config->TIMx->CCR1 = ccr_value; break; /* PDF Reference */
        case 2: config->TIMx->CCR2 = ccr_value; break; /* PDF Reference */
        case 3: config->TIMx->CCR3 = ccr_value; break; /* PDF Reference */
        case 4: config->TIMx->CCR4 = ccr_value; break; /* PDF Reference */
        default: return; // Should theoretically not happen with valid config->ChannelNumber
    }

    // Generate an update event to load all preload registers (PSC, ARR, CCRx) into active registers.
    config->TIMx->EGR |= TIM_EGR_UG_Msk; /* PDF Reference */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = get_pwm_config(TRD_Channel);
    if (config == NULL)
    {
        // Handle invalid channel
        return;
    }

    // Enable the counter (CEN bit)
    config->TIMx->CR1 |= TIM_CR1_CEN_Msk; /* PDF Reference */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = get_pwm_config(TRD_Channel);
    if (config == NULL)
    {
        // Handle invalid channel
        return;
    }

    // Option 1: Disable the counter (CEN bit)
    config->TIMx->CR1 &= ~TIM_CR1_CEN_Msk; /* PDF Reference */

    // Option 2 (Complementary to Option 1, or can be used alone):
    // Force the duty cycle to 0% to ensure the output is low.
    // This makes the stop more immediate and predictable, even if the counter isn't explicitly halted.
    switch (config->ChannelNumber)
    {
        case 1: config->TIMx->CCR1 = 0; break; /* PDF Reference */
        case 2: config->TIMx->CCR2 = 0; break; /* PDF Reference */
        case 3: config->TIMx->CCR3 = 0; break; /* PDF Reference */
        case 4: config->TIMx->CCR4 = 0; break; /* PDF Reference */
        default: return;
    }
    // Generate an update event to immediately apply the 0% duty cycle
    config->TIMx->EGR |= TIM_EGR_UG_Msk; /* PDF Reference */
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function will stop all active PWM channels, disable timer clocks,
 *        and reset associated GPIO pins to their default input floating state.
 */
void PWM_PowerOff(void)
{
    // First, stop all individual PWM channels (sets duty to 0% and disables counter)
    for (TRD_Channel_t i = (TRD_Channel_t)0; i < NUM_PWM_CHANNELS; i = (TRD_Channel_t)(i + 1))
    {
        PWM_Stop(i);
    }

    // Now, explicitly disable all used timer clocks to save power
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN); /* PDF Reference */
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN); /* PDF Reference */

    // Ensure clock gates are actually disabled by reading the register
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;

    // Finally, reset the GPIO pins used for PWM to their default input floating state.
    // This is done by clearing MODER and PUPDR bits to 00.
    // Collect all unique GPIO ports used in the map to avoid redundant operations
    GPIO_TypeDef* unique_ports[6] = {NULL}; // Max 6 possible ports (A, B, C, D, E, H) for F401RC
    uint8_t port_count = 0;

    for (TRD_Channel_t i = (TRD_Channel_t)0; i < NUM_PWM_CHANNELS; i = (TRD_Channel_t)(i + 1))
    {
        const PWM_Channel_Config_t* config = get_pwm_config(i);
        if (config == NULL) continue;

        uint8_t found = 0;
        for (uint8_t j = 0; j < port_count; j++)
        {
            if (unique_ports[j] == config->PortName)
            {
                found = 1;
                break;
            }
        }
        if (!found && (port_count < sizeof(unique_ports)/sizeof(unique_ports[0])))
        {
            unique_ports[port_count++] = config->PortName;
        }
    }

    for (uint8_t i = 0; i < port_count; i++)
    {
        if (unique_ports[i] != NULL)
        {
            // Set the specific pin used for PWM back to Input mode (00) and No Pull (00)
            // It's safer to only modify the pins that were actually used for PWM,
            // rather than affecting the entire port.
            for (TRD_Channel_t ch_idx = (TRD_Channel_t)0; ch_idx < NUM_PWM_CHANNELS; ch_idx = (TRD_Channel_t)(ch_idx + 1))
            {
                const PWM_Channel_Config_t* config = get_pwm_config(ch_idx);
                if (config != NULL && config->PortName == unique_ports[i])
                {
                    uint8_t pin = config->PinNumber;
                    // Clear MODER bits to set to Input mode (00)
                    unique_ports[i]->MODER &= ~GPIO_MODER_MODE_Msk(pin); /* PDF Reference */
                    // Clear PUPDR bits to set to No Pull-up, Pull-down (00)
                    unique_ports[i]->PUPDR &= ~GPIO_PUPDR_PUPD_Msk(pin); /* PDF Reference */
                    // Clear OTYPER bit to set to Push-Pull (0) - default reset state
                    unique_ports[i]->OTYPER &= ~GPIO_OTYPER_OT_Msk(pin); /* PDF Reference */
                    // Clear OSPEEDR bits to set to Low Speed (00) - default reset state
                    unique_ports[i]->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED_Msk(pin); /* PDF Reference */
                    // Clear Alternate Function bits to AF0 (System function) - default reset state
                    if (pin < 8) {
                        unique_ports[i]->AFRL &= ~GPIO_AFR_AFR_Msk(pin); /* PDF Reference */
                    } else {
                        unique_ports[i]->AFRH &= ~GPIO_AFR_AFR_Msk(pin); /* PDF Reference */
                    }
                }
            }
            // Note: Disabling GPIO clocks is generally not done unless the entire GPIO port
            // is guaranteed to be unused, as other peripherals might depend on the same port's clock.
            // Setting pins to input floating mode is usually sufficient for power reduction.
        }
    }
}