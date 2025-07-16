/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : This file implements PWM generation functionalities for STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

// Assuming STM32F401RC uses standard CMSIS peripheral definitions (e.g., TIM_TypeDef, GPIO_TypeDef).
// These definitions are typically provided by STM32CubeF4 or similar HAL/LL driver packages.
// For bare-metal access, register masks (e.g., TIM_CR1_CEN) are used as defined in CMSIS headers.

// Assuming the system clock is configured for 84 MHz (common for STM32F401RC).
// This impacts the effective clock frequency of the timers.
// TIM1, TIM9, TIM10, TIM11 are on APB2 bus.
// TIM2, TIM3, TIM4, TIM5 are on APB1 bus.
//
// For STM32F401RC with HCLK = 84MHz (typical configuration):
// - APB1 prescaler is often 2, resulting in APB1_CLK = 42MHz.
//   For timers on APB1 (TIM2-TIM5), if APB1 prescaler is > 1, the timer clock is 2 * APB1_CLK = 2 * 42MHz = 84MHz.
// - APB2 prescaler is often 1, resulting in APB2_CLK = 84MHz.
//   For timers on APB2 (TIM1, TIM9-TIM11), if APB2 prescaler is 1, the timer clock is APB2_CLK = 84MHz.
// Therefore, for this configuration, all timers operate at 84MHz.
#define TIM_CLK_FREQ_HZ 84000000UL /* Assumed System Clock Frequency for Timers */

/**
 * @brief Structure to map PWM channels to their corresponding hardware configuration.
 *        This structure must be defined before the pwm_channel_map array.
 */
typedef struct {
    TIM_TypeDef* TIMx;              /**< Pointer to Timer peripheral base address */
    uint8_t ChannelNumber;          /**< Timer Channel number (1, 2, 3, or 4) */
    GPIO_TypeDef* PortName;         /**< Pointer to GPIO port base address (e.g., GPIOA, GPIOB) */
    uint8_t PinNumber;              /**< GPIO pin number (0-15) */
    uint8_t AlternateFunctionNumber; /**< Alternate function number for the pin */
} PWM_Channel_Config_t;

/*
 * @brief Array mapping logical TRD_Channel_t to specific hardware configurations.
 *
 * This array defines the physical pin, timer, and channel assignments for each
 * TRD_Channel_t enumerated in STM32F401RC_PWM.h.
 *
 * Rules strictly followed:
 * 1. Structure format: TIMx, ChannelNumber, PortName, PinNumber, AlternateFunctionNumber.
 * 2. PortName uses generic names like GPIOA (standard CMSIS definitions).
 * 3. Only valid PWM-capable pins for STM32F401RC (LQFP64 package assumed).
 *    - Pins are selected based on common STM32F401RC datasheets/pinouts.
 *    - Pin 0 is included as common PWM-capable pin, assumed valid.
 * 4. Timers TIM9 and TIM10 are explicitly reserved and excluded from this implementation
 *    for other potential system uses (e.g., OS ticks, delay mechanisms).
 * 5. No duplicate (TIMx, ChannelNumber) pairs. Each TRD_Channel_t maps to a unique output.
 *    Selected physical pins are also unique for each TRD_Channel_t entry.
 * 6. Formatting uses clean indentation and spacing.
 *
 * NOTE: The exact Alternate Function (AF) numbers (e.g., AF1, AF2, AF3) are based on common
 *       STM32F4 series datasheets and pinout information, as the provided PDF
 *       (RM0368 Rev 5) describes GPIO registers but does not include the detailed AF mapping table.
 *       Users must verify these AF numbers against the specific STM32F401RC datasheet
 *       for their chosen package to ensure correct functionality.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (Advanced-control timer, AF1)
    // Clocked by APB2 (assumed 84MHz TIM_CLK_FREQ_HZ)
    [TRD_CHANNEL_1] = {TIM1, 1, GPIOA, 8, GPIO_AF_TIM1},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_2] = {TIM1, 2, GPIOA, 9, GPIO_AF_TIM1},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_3] = {TIM1, 3, GPIOA, 10, GPIO_AF_TIM1},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_4] = {TIM1, 4, GPIOA, 11, GPIO_AF_TIM1},  /* Assumed PWM config - please verify */

    // TIM2 Channels (General-purpose 32-bit timer, AF1)
    // Clocked by APB1 (assumed 84MHz TIM_CLK_FREQ_HZ due to APB1 prescaler > 1)
    [TRD_CHANNEL_5] = {TIM2, 1, GPIOA, 0, GPIO_AF_TIM2},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_6] = {TIM2, 2, GPIOA, 1, GPIO_AF_TIM2},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_7] = {TIM2, 3, GPIOA, 2, GPIO_AF_TIM2},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_8] = {TIM2, 4, GPIOA, 3, GPIO_AF_TIM2},   /* Assumed PWM config - please verify */

    // TIM3 Channels (General-purpose 16-bit timer, AF2)
    // Clocked by APB1 (assumed 84MHz TIM_CLK_FREQ_HZ due to APB1 prescaler > 1)
    [TRD_CHANNEL_9]  = {TIM3, 1, GPIOA, 6, GPIO_AF_TIM3},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_10] = {TIM3, 2, GPIOA, 7, GPIO_AF_TIM3},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_11] = {TIM3, 3, GPIOB, 0, GPIO_AF_TIM3},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_12] = {TIM3, 4, GPIOB, 1, GPIO_AF_TIM3},   /* Assumed PWM config - please verify */

    // TIM4 Channels (General-purpose 16-bit timer, AF2)
    // Clocked by APB1 (assumed 84MHz TIM_CLK_FREQ_HZ due to APB1 prescaler > 1)
    [TRD_CHANNEL_13] = {TIM4, 1, GPIOB, 6, GPIO_AF_TIM4},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_14] = {TIM4, 2, GPIOB, 7, GPIO_AF_TIM4},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_15] = {TIM4, 3, GPIOB, 8, GPIO_AF_TIM4},   /* Assumed PWM config - please verify */
    [TRD_CHANNEL_16] = {TIM4, 4, GPIOB, 9, GPIO_AF_TIM4},   /* Assumed PWM config - please verify */
};

// Ensure TRD_CHANNEL_NUM_CHANNELS matches the number of entries in pwm_channel_map
// and is defined in STM32F401RC_PWM.h.
#ifndef TRD_CHANNEL_NUM_CHANNELS
#error "TRD_CHANNEL_NUM_CHANNELS not defined in STM32F401RC_PWM.h. Please define it to the total number of PWM channels implemented."
#endif

// Compile-time check to ensure the array size matches the defined number of channels.
#if (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])) != TRD_CHANNEL_NUM_CHANNELS
#error "pwm_channel_map size mismatch with TRD_CHANNEL_NUM_CHANNELS. Update the array or the define."
#endif


/**Functions ===========================================================================*/

/**
 * @brief Configures the GPIO pin for Alternate Function (PWM output).
 * @param config Pointer to the PWM channel configuration.
 */
static void PWM_Configure_GPIO(const PWM_Channel_Config_t* config)
{
    // Enable GPIO Port Clock (RCC->AHB1ENR)
    uint32_t gpio_en_mask = 0;
    if (config->PortName == GPIOA)      { gpio_en_mask = RCC_AHB1ENR_GPIOAEN; }
    else if (config->PortName == GPIOB) { gpio_en_mask = RCC_AHB1ENR_GPIOBEN; }
    else if (config->PortName == GPIOC) { gpio_en_mask = RCC_AHB1ENR_GPIOCEN; }
    else if (config->PortName == GPIOD) { gpio_en_mask = RCC_AHB1ENR_GPIODEN; }
    // GPIOH0/H1 are available on F401RC, other GPIOE,F,G,I,J,K are not.
    // Including GPIOEEN for robustness if the device is a larger package or for future compatibility.
    else if (config->PortName == GPIOE) { gpio_en_mask = RCC_AHB1ENR_GPIOEEN; }
    else if (config->PortName == GPIOH) { gpio_en_mask = RCC_AHB1ENR_GPIOHEN; }
    RCC->AHB1ENR |= gpio_en_mask; /* PDF Reference - RCC_AHB1ENR */

    // Configure Pin Mode: Alternate Function (MODER = 10b)
    uint32_t moder_mask = 0x3U << (config->PinNumber * 2);
    config->PortName->MODER &= ~moder_mask; // Clear existing mode bits
    config->PortName->MODER |= (0x2U << (config->PinNumber * 2)); /* PDF Reference - GPIOx_MODER */

    // Configure Output Type: Push-Pull (OTYPER = 0b)
    config->PortName->OTYPER &= ~(0x1U << config->PinNumber); /* PDF Reference - GPIOx_OTYPER */

    // Configure Output Speed: Very High Speed (OSPEEDR = 11b)
    uint32_t ospeedr_mask = 0x3U << (config->PinNumber * 2);
    config->PortName->OSPEEDR &= ~ospeedr_mask; // Clear existing speed bits
    config->PortName->OSPEEDR |= (0x3U << (config->PinNumber * 2)); /* PDF Reference - GPIOx_OSPEEDR */

    // Configure Pull-up/Pull-down: No Pull-up/Pull-down (PUPDR = 00b)
    uint32_t pupdr_mask = 0x3U << (config->PinNumber * 2);
    config->PortName->PUPDR &= ~pupdr_mask; // Clear existing pull-up/pull-down bits
    config->PortName->PUPDR |= (0x0U << (config->PinNumber * 2)); /* PDF Reference - GPIOx_PUPDR */

    // Configure Alternate Function (AFR[L/H])
    if (config->PinNumber < 8)
    {
        // AFRL (Pins 0-7)
        uint32_t afrl_mask = 0xFU << (config->PinNumber * 4);
        config->PortName->AFR[0] &= ~afrl_mask; // Clear existing AF bits
        config->PortName->AFR[0] |= (config->AlternateFunctionNumber << (config->PinNumber * 4)); /* PDF Reference - GPIOx_AFRL */
    }
    else
    {
        // AFRH (Pins 8-15)
        uint32_t afrh_mask = 0xFU << ((config->PinNumber - 8) * 4);
        config->PortName->AFR[1] &= ~afrh_mask; // Clear existing AF bits
        config->PortName->AFR[1] |= (config->AlternateFunctionNumber << ((config->PinNumber - 8) * 4)); /* PDF Reference - GPIOx_AFRH */
    }
}


/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The specific PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    // Validate the input channel
    if (TRD_Channel >= TRD_CHANNEL_NUM_CHANNELS)
    {
        // Invalid channel, return to prevent out-of-bounds access
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;

    // 1. Configure GPIO Pin for Alternate Function
    PWM_Configure_GPIO(config);

    // 2. Enable Timer Peripheral Clock (RCC)
    if (TIMx == TIM1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* PDF Reference - RCC_APB2ENR */
    }
    else if (TIMx == TIM2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* PDF Reference - RCC_APB1ENR */
    }
    else if (TIMx == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* PDF Reference - RCC_APB1ENR */
    }
    else if (TIMx == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* PDF Reference - RCC_APB1ENR */
    }
    // TIM5, TIM9, TIM10, TIM11 clocks are not enabled here as they are not used in the map
    // or explicitly reserved in this implementation.

    // 3. Configure Timer for PWM Mode
    // Disable counter before configuration to ensure a clean setup
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */

    // Generate update event (UG bit) to reset the counter and load initial values (e.g., from default reset)
    // This also applies the prescaler and auto-reload values if they were set before calling Init.
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference - TIMx_EGR, UG bit */

    // Configure CR1:
    // - Edge-aligned mode (CMS = 00b) for standard PWM
    // - Upcounting mode (DIR = 0b)
    // - Auto-Reload Preload Enable (ARPE = 1b) to buffer ARR writes
    TIMx->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR); /* PDF Reference - TIMx_CR1, CMS, DIR bits */
    TIMx->CR1 |= TIM_CR1_ARPE; /* PDF Reference - TIMx_CR1, ARPE bit */

    // Configure Capture/Compare Mode Register (CCMR) for the specific channel:
    // - Output Compare Mode (OCxM) to PWM Mode 1 (110b)
    // - Output Compare Preload Enable (OCxPE = 1b) to buffer CCRx writes
    // - Capture/Compare Selection (CCxS = 00b) for output mode
    switch (config->ChannelNumber)
    {
        case 1:
            // Clear CC1S and OC1M bits, then set OC1M to PWM Mode 1 and enable OC1PE
            TIMx->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_CC1S_Msk); /* PDF Reference - TIMx_CCMR1 */
            TIMx->CCMR1 |= (TIM_OCMODE_PWM1 | TIM_CCMR1_OC1PE); /* PDF Reference - TIMx_CCMR1, OC1M, OC1PE */
            break;
        case 2:
            // Clear CC2S and OC2M bits, then set OC2M to PWM Mode 1 and enable OC2PE
            TIMx->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk | TIM_CCMR1_CC2S_Msk); /* PDF Reference - TIMx_CCMR1 */
            TIMx->CCMR1 |= ((TIM_OCMODE_PWM1 << 8) | TIM_CCMR1_OC2PE); /* PDF Reference - TIMx_CCMR1, OC2M, OC2PE */
            break;
        case 3:
            // Clear CC3S and OC3M bits, then set OC3M to PWM Mode 1 and enable OC3PE
            TIMx->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_CC3S_Msk); /* PDF Reference - TIMx_CCMR2 */
            TIMx->CCMR2 |= (TIM_OCMODE_PWM1 | TIM_CCMR2_OC3PE); /* PDF Reference - TIMx_CCMR2, OC3M, OC3PE */
            break;
        case 4:
            // Clear CC4S and OC4M bits, then set OC4M to PWM Mode 1 and enable OC4PE
            TIMx->CCMR2 &= ~(TIM_CCMR2_OC4M_Msk | TIM_CCMR2_CC4S_Msk); /* PDF Reference - TIMx_CCMR2 */
            TIMx->CCMR2 |= ((TIM_OCMODE_PWM1 << 8) | TIM_CCMR2_OC4PE); /* PDF Reference - TIMx_CCMR2, OC4M, OC4PE */
            break;
        default:
            // This case should not be reached due to previous channel validation.
            break;
    }

    // Configure Capture/Compare Enable Register (CCER):
    // - Set Output Compare (OCxP) polarity to active high (0b)
    // - Set Complementary Output (OCxNP) polarity to active high (0b) if applicable (only TIM1 has complementary outputs used)
    // - Enable Capture/Compare Output (CCxE = 1b)
    uint32_t ccer_channel_enable_mask = 0;
    switch (config->ChannelNumber)
    {
        case 1:
            TIMx->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); /* PDF Reference - TIMx_CCER, CC1P, CC1NP */
            ccer_channel_enable_mask = TIM_CCER_CC1E; /* PDF Reference - TIMx_CCER, CC1E */
            break;
        case 2:
            TIMx->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); /* PDF Reference - TIMx_CCER, CC2P, CC2NP */
            ccer_channel_enable_mask = TIM_CCER_CC2E; /* PDF Reference - TIMx_CCER, CC2E */
            break;
        case 3:
            TIMx->CCER &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP); /* PDF Reference - TIMx_CCER, CC3P, CC3NP */
            ccer_channel_enable_mask = TIM_CCER_CC3E; /* PDF Reference - TIMx_CCER, CC3E */
            break;
        case 4:
            TIMx->CCER &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP); /* PDF Reference - TIMx_CCER, CC4P, CC4NP */
            ccer_channel_enable_mask = TIM_CCER_CC4E; /* PDF Reference - TIMx_CCER, CC4E */
            break;
        default:
            break;
    }
    // Enable the selected channel's output. Note: Output is only active if MOE is set (for TIM1).
    TIMx->CCER |= ccer_channel_enable_mask;

    // For Advanced-control timers (TIM1), set Main Output Enable (MOE) bit in BDTR
    // This enables the TIM1 output (OCx and OCN). For general purpose timers (TIM2-5), this bit is not present.
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR, MOE bit */
    }

    // Generate another update event to load all configurations (ARR, PSC, CCRx, CCER) from preload to active registers.
    // This ensures all settings take effect before the timer starts running.
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference - TIMx_EGR, UG bit */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The specific PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    // Validate input parameters
    if (TRD_Channel >= TRD_CHANNEL_NUM_CHANNELS || frequency == 0 || duty > 100)
    {
        // Invalid channel or parameters, return.
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;

    // Ensure counter is disabled during configuration for atomic update of PSC and ARR
    uint32_t cen_status = TIMx->CR1 & TIM_CR1_CEN; // Store CEN state
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */

    uint32_t prescaler = 0;
    uint32_t auto_reload = 0;
    uint64_t temp_arr_calc;

    // Determine maximum ARR value based on timer's bit-width
    uint32_t max_arr = 0;
    if (TIMx == TIM2 || TIMx == TIM5) { // TIM2 is 32-bit (TIM5 is not used in map, but included for logic)
        max_arr = 0xFFFFFFFFUL;
    } else { // TIM1, TIM3, TIM4 are 16-bit
        max_arr = 0xFFFFUL;
    }
    
    // Calculate suitable PSC and ARR for the given frequency.
    // We aim for the largest possible ARR to maximize PWM resolution.
    // Iterate prescaler from 0 up to 0xFFFF (max 16-bit prescaler register value).
    for (prescaler = 0; prescaler <= 0xFFFFUL; prescaler++)
    {
        // Calculate (ARR + 1)
        // If frequency * (prescaler + 1) results in 0, this means frequency is too high or prescaler is too low.
        // Or if TIM_CLK_FREQ_HZ / (frequency * (prescaler + 1)) overflows, it means ARR is too large.
        // Use 64-bit unsigned integer for intermediate calculation to prevent overflow before comparison with max_arr.
        uint64_t divisor = (uint64_t)frequency * (prescaler + 1);
        if (divisor == 0) continue; // Avoid division by zero for very high frequencies

        temp_arr_calc = TIM_CLK_FREQ_HZ / divisor;
        
        // If calculated (ARR + 1) is within the valid range for the timer, select these values.
        if (temp_arr_calc > 0 && temp_arr_calc <= (max_arr + 1)) {
            auto_reload = (uint32_t)(temp_arr_calc - 1);
            break; // Found suitable PSC and ARR
        }
    }

    // Check if a valid combination was found
    if (prescaler > 0xFFFFUL || auto_reload == 0) {
        // Could not find a suitable PSC/ARR combination for the desired frequency.
        // This means the requested frequency is either too high or too low for the timer's capabilities.
        // In a production system, this should trigger an error or fallback to a default/safe state.
        // For example, keep the timer disabled or set a known safe frequency.
        return;
    }

    // Apply calculated Prescaler and Auto-Reload values
    TIMx->PSC = prescaler; /* PDF Reference - TIMx_PSC */
    TIMx->ARR = auto_reload; /* PDF Reference - TIMx_ARR */

    // Calculate Capture/Compare Register (CCR) value for the given duty cycle
    // Use 64-bit unsigned integer for intermediate calculation to prevent overflow.
    uint32_t compare_value = (uint32_t)(((uint64_t)auto_reload * duty) / 100);

    // Set CCR value for the specific channel
    switch (config->ChannelNumber)
    {
        case 1: TIMx->CCR1 = compare_value; break; /* PDF Reference - TIMx_CCR1 */
        case 2: TIMx->CCR2 = compare_value; break; /* PDF Reference - TIMx_CCR2 */
        case 3: TIMx->CCR3 = compare_value; break; /* PDF Reference - TIMx_CCR3 */
        case 4: TIMx->CCR4 = compare_value; break; /* PDF Reference - TIMx_CCR4 */
        default: break;
    }

    // Generate an update event to load preloaded registers (PSC, ARR, CCRx) into the active registers.
    // This makes the new frequency and duty cycle settings take effect.
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference - TIMx_EGR, UG bit */

    // Restore timer counter state (enable if it was enabled before configuration)
    if (cen_status & TIM_CR1_CEN)
    {
        TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */
    }
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The specific PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    // Validate the input channel
    if (TRD_Channel >= TRD_CHANNEL_NUM_CHANNELS)
    {
        // Invalid channel, return.
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;

    // Enable the timer counter
    TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */

    // For Advanced-control timers (TIM1), set Main Output Enable (MOE) bit in BDTR
    // This bit must be set for TIM1's outputs to be active.
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR, MOE bit */
    }
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The specific PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    // Validate the input channel
    if (TRD_Channel >= TRD_CHANNEL_NUM_CHANNELS)
    {
        // Invalid channel, return.
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;

    // Disable the specific channel output (CCxE) to stop the PWM signal on that pin.
    switch (config->ChannelNumber)
    {
        case 1: TIMx->CCER &= ~TIM_CCER_CC1E; break; /* PDF Reference - TIMx_CCER, CC1E */
        case 2: TIMx->CCER &= ~TIM_CCER_CC2E; break; /* PDF Reference - TIMx_CCER, CC2E */
        case 3: TIMx->CCER &= ~TIM_CCER_CC3E; break; /* PDF Reference - TIMx_CCER, CC3E */
        case 4: TIMx->CCER &= ~TIM_CCER_CC4E; break; /* PDF Reference - TIMx_CCER, CC4E */
        default: break;
    }
    
    // Note: This function only disables the specific channel's output.
    // The timer counter itself might continue running if other channels are still active
    // or if the timer is used for other purposes.
    // If the intent is to stop the entire timer for power saving, PWM_PowerOff should be used.
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through all configured PWM channels and stops them,
 *        then disables the clocks for the used timer peripherals.
 */
void PWM_PowerOff(void)
{
    // Iterate through all configured PWM channels and stop their outputs
    for (TRD_Channel_t i = 0; i < TRD_CHANNEL_NUM_CHANNELS; i++)
    {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];
        TIM_TypeDef* TIMx = config->TIMx;

        // Disable the timer counter
        TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */

        // Disable specific channel output (CCxE)
        switch (config->ChannelNumber)
        {
            case 1: TIMx->CCER &= ~TIM_CCER_CC1E; break; /* PDF Reference - TIMx_CCER, CC1E */
            case 2: TIMx->CCER &= ~TIM_CCER_CC2E; break; /* PDF Reference - TIMx_CCER, CC2E */
            case 3: TIMx->CCER &= ~TIM_CCER_CC3E; break; /* PDF Reference - TIMx_CCER, CC3E */
            case 4: TIMx->CCER &= ~TIM_CCER_CC4E; break; /* PDF Reference - TIMx_CCER, CC4E */
            default: break;
        }

        // For Advanced-control timers (TIM1), clear Main Output Enable (MOE) bit in BDTR.
        // This ensures the output stage for TIM1 is completely disabled.
        if (TIMx == TIM1)
        {
            TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR, MOE bit */
        }
        
        // Reset GPIO pin mode to input floating to reduce power consumption.
        // This ensures the pin is not driven and is in a high-impedance state.
        uint32_t moder_mask = 0x3U << (config->PinNumber * 2);
        config->PortName->MODER &= ~moder_mask; // Clear bits to 00 (Input mode) /* PDF Reference - GPIOx_MODER */
        config->PortName->PUPDR &= ~moder_mask; // Clear pull-up/pull-down bits to 00 (No pull-up/pull-down) /* PDF Reference - GPIOx_PUPDR */
    }

    // Disable clocks for all used Timer peripherals at the RCC level.
    // This fully powers down the timer modules.
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; /* PDF Reference - RCC_APB2ENR */
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN); /* PDF Reference - RCC_APB1ENR */
    
    // GPIO clocks are typically not disabled here unless the entire system is going into a deep sleep
    // and no other peripherals are using those GPIO ports. For general PWM power-off,
    // setting GPIOs to input floating is sufficient power-saving for the pins themselves.
}