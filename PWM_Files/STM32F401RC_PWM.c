/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready implementation for PWM on STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h" // Includes necessary definitions like TRD_Channel_t and register headers
#include "stm32f4xx.h"       // Required for STM32F4xx peripheral register definitions
#include "system_stm32f4xx.h" // Required for SystemCoreClock variable

/*
 * @note: Per requirement, two timers (TIM2 and TIM3) are reserved and excluded
 *        from this PWM implementation. These timers are commonly used for OS
 *        ticks, delays, or other system-level timing purposes.
 *        The available timers for PWM in this implementation are TIM1, TIM4,
 *        TIM5, TIM9, TIM10, TIM11. Not all channels of these timers might be
 *        broken out on the specific package or board, or might conflict with
 *        other peripherals. The following configuration defines which channels
 *        are supported by this driver based on typical usage and pin availability.
 */

// Forward declaration for internal clock calculation helper
static uint32_t GetTimerClockFrequency(TIM_TypeDef *TIMx);

// Structure to map a TRD_Channel_t enum to its hardware configuration
typedef struct {
    TRD_Channel_t     channel_id;
    TIM_TypeDef      *timer_base;
    uint32_t          timer_channel;   // Timer channel number (1 to 4)
    GPIO_TypeDef     *gpio_port;
    uint33_t          gpio_pin_mask;   // Pin mask (e.g., GPIO_PIN_0)
    uint8_t           gpio_pin_num;    // Pin number (0 to 15)
    uint8_t           gpio_af_num;     // Alternate Function number
    uint32_t          rcc_timer_enr_bit; // RCC APBxENR bit for the timer
    uint32_t          rcc_gpio_enr_bit;  // RCC AHB1ENR bit for the GPIO port
    uint8_t           apb_bus;           // Which APB bus the timer is on (1 or 2)
} PwmChannelConfig_t;

// Define the supported PWM channels and their configurations
// Reserved timers TIM2 and TIM3 are *not* included in this array.
static const PwmChannelConfig_t pwm_channel_configs[] = {
    // Example configurations - Verify against specific board/datasheet for pin usage
    // TIM1 - APB2 (max 168MHz)
    { TRD_CHANNEL_TIM1_CH1, TIM1, 1, GPIOA, GPIO_PIN_8,  8,  GPIO_AF_TIM1, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN, 2 }, // PA8
    { TRD_CHANNEL_TIM1_CH2, TIM1, 2, GPIOA, GPIO_PIN_9,  9,  GPIO_AF_TIM1, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN, 2 }, // PA9
    { TRD_CHANNEL_TIM1_CH3, TIM1, 3, GPIOA, GPIO_PIN_10, 10, GPIO_AF_TIM1, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN, 2 }, // PA10
    { TRD_CHANNEL_TIM1_CH4, TIM1, 4, GPIOA, GPIO_PIN_11, 11, GPIO_AF_TIM1, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN, 2 }, // PA11

    // TIM4 - APB1 (max 84MHz)
    { TRD_CHANNEL_TIM4_CH1, TIM4, 1, GPIOB, GPIO_PIN_6,  6,  GPIO_AF_TIM4, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN, 1 }, // PB6
    { TRD_CHANNEL_TIM4_CH2, TIM4, 2, GPIOB, GPIO_PIN_7,  7,  GPIO_AF_TIM4, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN, 1 }, // PB7
    { TRD_CHANNEL_TIM4_CH3, TIM4, 3, GPIOB, GPIO_PIN_8,  8,  GPIO_AF_TIM4, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN, 1 }, // PB8
    { TRD_CHANNEL_TIM4_CH4, TIM4, 4, GPIOB, GPIO_PIN_9,  9,  GPIO_AF_TIM4, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN, 1 }, // PB9

    // TIM5 - APB1 (max 84MHz) - 32-bit timer
    { TRD_CHANNEL_TIM5_CH1, TIM5, 1, GPIOA, GPIO_PIN_0,  0,  GPIO_AF_TIM5, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN, 1 }, // PA0
    { TRD_CHANNEL_TIM5_CH2, TIM5, 2, GPIOA, GPIO_PIN_1,  1,  GPIO_AF_TIM5, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN, 1 }, // PA1
    { TRD_CHANNEL_TIM5_CH3, TIM5, 3, GPIOA, GPIO_PIN_2,  2,  GPIO_AF_TIM5, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN, 1 }, // PA2
    { TRD_CHANNEL_TIM5_CH4, TIM5, 4, GPIOA, GPIO_PIN_3,  3,  GPIO_AF_TIM5, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN, 1 }, // PA3

    // TIM9 - APB2 (max 168MHz) - 2 channels
    { TRD_CHANNEL_TIM9_CH1, TIM9, 1, GPIOA, GPIO_PIN_2,  2,  GPIO_AF_TIM9, RCC_APB2ENR_TIM9EN,  RCC_AHB1ENR_GPIOAEN, 2 }, // PA2
    { TRD_CHANNEL_TIM9_CH2, TIM9, 2, GPIOA, GPIO_PIN_3,  3,  GPIO_AF_TIM9, RCC_APB2ENR_TIM9EN,  RCC_AHB1ENR_GPIOAEN, 2 }, // PA3

    // TIM10 - APB2 (max 168MHz) - 1 channel
    { TRD_CHANNEL_TIM10_CH1, TIM10, 1, GPIOB, GPIO_PIN_8, 8, GPIO_AF_TIM10, RCC_APB2ENR_TIM10EN, RCC_AHB1ENR_GPIOBEN, 2 }, // PB8

    // TIM11 - APB2 (max 168MHz) - 1 channel
    { TRD_CHANNEL_TIM11_CH1, TIM11, 1, GPIOB, GPIO_PIN_9, 9, GPIO_AF_TIM11, RCC_APB2ENR_TIM11EN, RCC_AHB1ENR_GPIOBEN, 2 }, // PB9

    // Add other potentially available channels here if needed, excluding TIM2 and TIM3
};

#define NUM_PWM_CHANNELS (sizeof(pwm_channel_configs) / sizeof(PwmChannelConfig_t))


/**
 * @brief Helper function to find the configuration for a given PWM channel ID.
 * @param channel_id: The TRD_Channel_t identifier.
 * @return Pointer to the PwmChannelConfig_t structure, or NULL if not found (reserved or invalid).
 */
static const PwmChannelConfig_t* get_pwm_config(TRD_Channel_t channel_id)
{
    for (uint32_t i = 0; i < NUM_PWM_CHANNELS; i++) {
        if (pwm_channel_configs[i].channel_id == channel_id) {
            return &pwm_channel_configs[i];
        }
    }
    // Return NULL if channel_id is not found in the supported list (e.g., reserved or invalid)
    return NULL;
}

/**
 * @brief Helper function to get the timer clock frequency based on the APB prescaler.
 *        Assumes SystemCoreClock is correctly set.
 * @param TIMx: Pointer to the Timer peripheral (e.g., TIM1, TIM4).
 * @return Timer clock frequency in Hz.
 */
static uint32_t GetTimerClockFrequency(TIM_TypeDef *TIMx)
{
    uint32_t pclk_freq = 0;
    uint32_t apb_prescaler = 0;
    const PwmChannelConfig_t *config = NULL;

    // Find the config to determine the APB bus
    for(uint32_t i = 0; i < NUM_PWM_CHANNELS; i++)
    {
        if(pwm_channel_configs[i].timer_base == TIMx)
        {
            config = &pwm_channel_configs[i];
            break;
        }
    }

    if (config == NULL) {
        // This should not happen if the caller passes a valid timer base
        return 0;
    }

    if (config->apb_bus == 1) { // APB1 timers (TIM2, TIM3, TIM4, TIM5)
        // PCLK1 frequency = HCLK / APB1 prescaler
        // Timer clock is PCLK1 * 2 if APB1 prescaler is > 1, otherwise PCLK1
        pclk_freq = SystemCoreClock >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos]; // Get HCLK
        apb_prescaler = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos; // Get APB1 prescaler
        pclk_freq >>= APBPrescTable[apb_prescaler]; // Get PCLK1

        // Timer clock is PCLK1 * 2 if prescaler is not 1
        if (apb_prescaler > 4) // Prescaler > 1 (codes 8, 10, 12, 16 represent div 2, 4, 8, 16)
        {
            pclk_freq *= 2;
        }
    } else { // APB2 timers (TIM1, TIM9, TIM10, TIM11)
        // PCLK2 frequency = HCLK / APB2 prescaler
        // Timer clock is PCLK2 * 2 if APB2 prescaler is > 1, otherwise PCLK2
        pclk_freq = SystemCoreClock >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos]; // Get HCLK
        apb_prescaler = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos; // Get APB2 prescaler
        pclk_freq >>= APBPrescTable[apb_prescaler]; // Get PCLK2

        // Timer clock is PCLK2 * 2 if prescaler is not 1
        if (apb_prescaler > 4) // Prescaler > 1
        {
            pclk_freq *= 2;
        }
    }

    return pclk_freq;
}

// Lookup table for APB prescaler decoding
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4}; /* 0->1, 4->2, 5->4, 6->8, 7->16 */
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9}; /* 0->1, 8->2, 9->4, 10->8, 11->16, 12->64, 13->128, 14->256, 15->512 */


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIO for the given channel.
 * @param TRD_Channel: The specific PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PwmChannelConfig_t *config = get_pwm_config(TRD_Channel);

    // Check if the channel is supported (not reserved or invalid)
    if (config == NULL) {
        // Channel is reserved or invalid, do nothing.
        return;
    }

    TIM_TypeDef *TIMx = config->timer_base;
    GPIO_TypeDef *GPIOx = config->gpio_port;
    uint32_t timer_channel = config->timer_channel;
    uint32_t gpio_pin_mask = config->gpio_pin_mask;
    uint8_t gpio_pin_num = config->gpio_pin_num;
    uint8_t gpio_af_num = config->gpio_af_num;

    // 1. Enable GPIO clock
    RCC->AHB1ENR |= config->rcc_gpio_enr_bit;

    // Configure GPIO pin for Alternate Function (PWM output)
    // Mode: Alternate Function (0b10)
    GPIOx->MODER = (GPIOx->MODER & ~(GPIO_MODER_MODE0 << (gpio_pin_num * 2))) | (GPIO_MODER_MODE0_1 << (gpio_pin_num * 2));
    // Output Type: Push-Pull (0b0)
    GPIOx->OTYPER &= ~gpio_pin_mask;
    // Speed: High speed (0b11)
    GPIOx->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (gpio_pin_num * 2)); // Set both bits to '1' for high speed
    // Pull-up/Pull-down: No pull-up/pull-down (0b00)
    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (gpio_pin_num * 2));
    // Alternate Function selection
    if (gpio_pin_num < 8) {
        GPIOx->AFR[0] = (GPIOx->AFR[0] & ~(0xF << (gpio_pin_num * 4))) | (gpio_af_num << (gpio_pin_num * 4));
    } else {
        GPIOx->AFR[1] = (GPIOx->AFR[1] & ~(0xF << ((gpio_pin_num - 8) * 4))) | (gpio_af_num << ((gpio_pin_num - 8) * 4));
    }

    // 2. Enable Timer clock
    if (config->apb_bus == 1) {
        RCC->APB1ENR |= config->rcc_timer_enr_bit;
    } else { // APB2
        RCC->APB2ENR |= config->rcc_timer_enr_bit;
    }

    // 3. Configure Timer for PWM mode
    // Ensure timer is stopped during configuration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Configure Time-Base (PSC and ARR will be set by Set_Freq, set initial values)
    // ARR should be the maximum value for the timer type initially for flexibility.
    // PSC set to 0 initially.
    if (TIMx == TIM2 || TIMx == TIM5) // 32-bit timers
    {
        TIMx->PSC = 0;
        TIMx->ARR = 0xFFFFFFFFU;
    }
    else // 16-bit timers
    {
        TIMx->PSC = 0;
        TIMx->ARR = 0xFFFFU;
    }
    TIMx->CNT = 0; // Reset counter

    // Set Counter Mode: Up-counting
    TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);

    // Enable Auto-Reload Preload (ARPE)
    TIMx->CR1 |= TIM_CR1_ARPE;

    // Configure Output Compare mode for PWM (Mode 1: active high)
    // Select the correct CCMR register (CCMR1 for CH1/2, CCMR2 for CH3/4)
    volatile uint32_t* ccmr;
    uint32_t ccmr_offset; // Offset within CCMR register (0 for CH1/3, 8 for CH2/4)

    if (timer_channel == 1 || timer_channel == 2) {
        ccmr = &TIMx->CCMR1;
        ccmr_offset = (timer_channel == 1) ? 0 : 8;
    } else { // timer_channel == 3 or 4
        ccmr = &TIMx->CCMR2;
        ccmr_offset = (timer_channel == 3) ? 0 : 8;
    }

    // Clear existing OCxM and OCxPE bits
    *ccmr &= ~((TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE) << ccmr_offset);
    // Set OCxM to PWM Mode 1 (0b110) and enable output compare preload (OCxPE)
    *ccmr |= (((TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1PE) << ccmr_offset);

    // Configure Capture/Compare Enable Register (CCER)
    // Enable the output channel (CCxE) and set polarity (CCxP=0 for active high)
    volatile uint33_t* ccer = &TIMx->CCER;
    uint32_t ccer_offset = (timer_channel - 1) * 4; // Offset for CCxE, CCxP, CCxNE, CCxNP

    // Clear existing bits for the channel
    *ccer &= ~(TIM_CCER_CC1E << ccer_offset); // Clear CCxE, CCxP, CCxNE, CCxNP (safe clear all)
    // Set CCxE (Capture/Compare Output Enable)
    *ccer |= (TIM_CCER_CC1E << ccer_offset);


    // For advanced timers (like TIM1), enable Main Output Enable (MOE) in BDTR later in PWM_Start
    // For general purpose timers, CCER_CCxE is sufficient to enable the output.
    if (TIMx == TIM1) {
        // Enable Output Automatic enable (BDOE) - may not be needed if no break/deadtime
        // TIMx->BDTR |= TIM_BDTR_BDOE;
        // MOE will be enabled in PWM_Start
    }

    // Generate an update event to load the prescaler and ARR value registers (optional here, will be done in Set_Freq)
    TIMx->EGR |= TIM_EGR_UG;

    // Clear update interrupt flag
    TIMx->SR &= ~TIM_SR_UIF;

    // Wait for the update flag to be set and cleared (optional)
    // while((TIMx->SR & TIM_SR_UIF) == 0);
    // TIMx->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel: The specific PWM channel.
 * @param frequency: The desired frequency in Hz.
 * @param duty: The desired duty cycle in percent (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    const PwmChannelConfig_t *config = get_pwm_config(TRD_Channel);

    // Check if the channel is supported (not reserved or invalid)
    if (config == NULL) {
        // Channel is reserved or invalid, do nothing.
        return;
    }

    if (frequency == 0) {
        // Frequency cannot be zero, stop the channel or return.
        // Stopping the channel is safer.
        PWM_Stop(TRD_Channel);
        return;
    }

    if (duty > 100) duty = 100; // Cap duty cycle at 100%

    TIM_TypeDef *TIMx = config->timer_base;
    uint32_t timer_channel = config->timer_channel;

    // Get timer clock frequency
    uint32_t timer_clock = GetTimerClockFrequency(TIMx);

    // Calculate total ticks per period (frequency)
    // ticks_per_period = Timer_Clock / frequency
    uint64_t ticks_per_period = (uint64_t)timer_clock / frequency;

    uint32_t prescaler = 0;
    uint32_t arr = 0;
    uint32_t max_arr;

    if (TIMx == TIM2 || TIMx == TIM5) { // 32-bit timers
        max_arr = 0xFFFFFFFFU;
    } else { // 16-bit timers
        max_arr = 0xFFFFU;
    }

    // Find the optimal prescaler and ARR to achieve the desired frequency.
    // Prioritize maximizing ARR for better duty cycle resolution.
    // (PSC + 1) * (ARR + 1) = ticks_per_period
    // Let ARR = max_arr, solve for PSC+1: (PSC + 1) = ticks_per_period / (max_arr + 1)
    // If calculated PSC+1 is 0, it means ticks_per_period is very small, can use PSC=0.
    uint32_t psc_plus_1 = (ticks_per_period > max_arr) ? (uint32_t)(ticks_per_period / (max_arr + 1)) : 0;

    if (psc_plus_1 > 0xFFFFU) {
        // Calculated prescaler is too large for 16-bit PSC register.
        // Use max prescaler and calculate resulting minimum frequency.
        // In a production system, you might want to indicate this error.
        prescaler = 0xFFFFU;
        arr = (uint32_t)(ticks_per_period / (prescaler + 1)) - 1;
    } else {
        prescaler = psc_plus_1;
        // Recalculate ARR based on chosen prescaler
        if ((prescaler + 1) == 0) prescaler = 0; // Should not happen but safety check
        arr = (uint32_t)(ticks_per_period / (prescaler + 1)) - 1;
    }

    // Ensure calculated ARR is within bounds and non-negative
    if (arr > max_arr) arr = max_arr;
    if (arr == (uint32_t)-1) arr = 0; // Handle case where ticks_per_period / (prescaler + 1) = 0

    // Set Prescaler and Auto-Reload Register values
    TIMx->PSC = prescaler;
    TIMx->ARR = arr;

    // Calculate the Capture/Compare Register (CCR) value for duty cycle
    // CCRx = (ARR + 1) * duty / 100
    uint64_t pulse_ticks = ((uint64_t)arr + 1) * duty / 100;
    uint32_t ccr = (uint32_t)pulse_ticks;

    // Adjust for 0% and 100% duty cycle edge cases for PWM Mode 1 (active high)
    // 0% duty: CCRx should be 0 (pulse_ticks calculated above handles this)
    // 100% duty: CCRx should be >= ARR + 1 (results in continuous high)
    if (duty == 100) {
         ccr = arr + 1;
    } else if (duty == 0) {
         ccr = 0;
    }


    // Set the Capture/Compare Register value for the specific channel
    switch (timer_channel) {
        case 1: TIMx->CCR1 = ccr; break;
        case 2: TIMx->CCR2 = ccr; break;
        case 3: TIMx->CCR3 = ccr; break;
        case 4: TIMx->CCR4 = ccr; break;
        default: break; // Should not happen
    }

    // Generate an update event to load the new PSC, ARR, and CCR values
    // This is important to ensure the changes take effect immediately after setting.
    TIMx->EGR |= TIM_EGR_UG;

    // Clear the update flag
    TIMx->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel: The specific PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PwmChannelConfig_t *config = get_pwm_config(TRD_Channel);

    // Check if the channel is supported (not reserved or invalid)
    if (config == NULL) {
        // Channel is reserved or invalid, do nothing.
        return;
    }

    TIM_TypeDef *TIMx = config->timer_base;
    uint32_t timer_channel = config->timer_channel;
    uint32_t ccer_offset = (timer_channel - 1) * 4; // Offset for CCxE, CCxP, CCxNE, CCxNP

    // Enable the specific output compare channel in CCER
    TIMx->CCER |= (TIM_CCER_CC1E << ccer_offset);

    // For advanced timers (TIM1), enable the main output.
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE; // Main Output Enable
    }

    // Enable the timer counter
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel: The specific PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PwmChannelConfig_t *config = get_pwm_config(TRD_Channel);

    // Check if the channel is supported (not reserved or invalid)
    if (config == NULL) {
        // Channel is reserved or invalid, do nothing.
        return;
    }

    TIM_TypeDef *TIMx = config->timer_base;
    GPIO_TypeDef *GPIOx = config->gpio_port;
    uint32_t timer_channel = config->timer_channel;
    uint8_t gpio_pin_num = config->gpio_pin_num;
    uint32_t ccer_offset = (timer_channel - 1) * 4; // Offset for CCxE, CCxP, CCxNE, CCxNP

    // Disable the specific output compare channel in CCER
    TIMx->CCER &= ~(TIM_CCER_CC1E << ccer_offset);

    // For advanced timers (TIM1), disable the main output if no other channels are active
    // (Simplified: just disable for this channel's timer if needed, though usually MOE is global)
    // A robust implementation might track if other channels on TIM1 are still active.
    if (TIMx == TIM1) {
        TIMx->BDTR &= ~TIM_BDTR_MOE; // Main Output Disable
    }

    // Stop the timer counter only if this is the only channel using this timer,
    // or if stopping the timer is desired behavior per channel.
    // For per-channel stop, usually you just disable the output enable in CCER.
    // If stopping the timer completely is desired, check if any other channel using this timer is active.
    // Simpler approach: just disable the output enable, don't stop the timer counter itself.
    // If the requirement is to stop the timer counter per channel call, this is inefficient.
    // To truly stop the timer, you might need a ref-counting mechanism or a dedicated `PWM_StopTimer(TIM_TypeDef *TIMx)` function.
    // Sticking to the explicit function signature: disable the channel output.

    // Reconfigure the GPIO pin back to a safe state (e.g., Analog input for power saving)
    // Mode: Analog (0b11)
    GPIOx->MODER |= (GPIO_MODER_MODE0 << (gpio_pin_num * 2));
    // Other settings (OTYPER, OSPEEDR, PUPDR) are less critical in Analog mode, but can be reset.
    GPIOx->OTYPER &= ~config->gpio_pin_mask; // Default to Push-Pull
    GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (gpio_pin_num * 2)); // Default to Low Speed
    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (gpio_pin_num * 2)); // Default to No pull-up/pull-down
    // Alternate Function: Reset AF selection (optional, mode switch is dominant)
    if (gpio_pin_num < 8) {
        GPIOx->AFR[0] &= ~(0xF << (gpio_pin_num * 4));
    } else {
        GPIOx->AFR[1] &= ~(0xF << ((gpio_pin_num - 8) * 4));
    }
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        Stops all configured PWM channels and disables their clocks.
 */
void PWM_PowerOff(void)
{
    // Iterate through all *supported* PWM channel configurations
    for (uint32_t i = 0; i < NUM_PWM_CHANNELS; i++) {
        const PwmChannelConfig_t *config = &pwm_channel_configs[i];

        TIM_TypeDef *TIMx = config->timer_base;
        GPIO_TypeDef *GPIOx = config->gpio_port;
        uint32_t timer_channel = config->timer_channel;
        uint8_t gpio_pin_num = config->gpio_pin_num;
        uint32_t ccer_offset = (timer_channel - 1) * 4;

        // Disable the specific output compare channel in CCER
        TIMx->CCER &= ~(TIM_CCER_CC1E << ccer_offset);

        // Disable Main Output for TIM1 if this channel belongs to TIM1
        if (TIMx == TIM1) {
            // Note: This disables MOE for all channels on TIM1.
            // A more sophisticated approach would check if *any* channel on TIM1 is enabled before disabling MOE.
            // For a full power off, disabling it is fine.
             TIMx->BDTR &= ~TIM_BDTR_MOE;
        }

        // Reconfigure the GPIO pin back to Analog input mode for power saving
        // Mode: Analog (0b11)
        GPIOx->MODER |= (GPIO_MODER_MODE0 << (gpio_pin_num * 2));
        // Clear other settings
        GPIOx->OTYPER &= ~config->gpio_pin_mask;
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (gpio_pin_num * 2));
        GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (gpio_pin_num * 2));
         if (gpio_pin_num < 8) {
            GPIOx->AFR[0] &= ~(0xF << (gpio_pin_num * 4));
        } else {
            GPIOx->AFR[1] &= ~(0xF << ((gpio_pin_num - 8) * 4));
        }
    }

    // Disable clocks for all supported Timers and GPIO ports that were used.
    // Need to iterate through the unique timers/ports used in the configs.
    // A more efficient way is to just disable all supported timer clocks.
    // For GPIO, disable clocks only if *all* pins on a port are in power-off state,
    // which is hard to track here. Safer to just disable timer clocks and reconfigure pins.

    // Disable clocks for timers that are *not* TIM2 or TIM3
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN);
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN);

    // GPIO clocks are generally left enabled if other peripherals or user code
    // might be using other pins on those ports. Reconfiguring the specific pins
    // to Analog mode is usually sufficient for power saving related to those pins.
}