/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : STM32F410RC PWM driver implementation
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "STM32F410RC_PWM.h"
#include "stm32f4xx.h" // Include CMSIS header for STM32F4 register definitions
#include <stdint.h>
#include <stddef.h> // For NULL

// Assuming tlong and tbyte are project-specific typedefs.
// Using standard types here for clarity, assume header maps them.
// typedef uint32_t tlong;
// typedef uint8_t tbyte;


/**Static Variables ====================================================================*/

/**
 * @brief Structure to hold configuration details for each PWM channel.
 */
typedef struct
{
    TIM_TypeDef *TIMx;       /**< Pointer to the Timer peripheral base address */
    uint32_t TIM_Channel;    /**< Timer Channel number (1, 2, 3, or 4) */
    GPIO_TypeDef *GPIOx;     /**< Pointer to the GPIO port base address */
    uint32_t GPIO_Pin;       /**< GPIO pin number */
    uint32_t GPIO_AF;        /**< GPIO Alternate Function number */
    uint32_t RCC_APB_ENR;    /**< RCC APB peripheral clock enable register address */
    uint32_t RCC_APB_Mask;   /**< RCC APB peripheral clock enable mask for the timer */
    uint32_t RCC_AHB1_ENR;   /**< RCC AHB1 peripheral clock enable register address */
    uint32_t RCC_AHB1_Mask;  /**< RCC AHB1 peripheral clock enable mask for the GPIO */
} PWM_ChannelConfig_t;


/**
 * @brief Configuration table for available PWM channels.
 * Maps TRD_Channel_t enum to specific hardware resources.
 * NOTE: This table must be populated with the actual GPIO/Timer/AF configurations
 *       used on the specific board design for the STM32F410RC.
 *       Example uses common TIM3/TIM4 channels on PA6/PA7/PB0/PB1.
 */
static const PWM_ChannelConfig_t PWM_ChannelConfig[] =
{
    // Example configurations - VERIFY THESE WITH YOUR HARDWARE DESIGN
    { TIM3, 1, GPIOA, GPIO_PIN_6, GPIO_AF2_TIM3, (uint32_t)&RCC->APB1ENR, RCC_APB1ENR_TIM3EN, (uint32_t)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN }, // Example: TIM3_CH1 on PA6 (AF2)
    { TIM3, 2, GPIOA, GPIO_PIN_7, GPIO_AF2_TIM3, (uint32_t)&RCC->APB1ENR, RCC_APB1ENR_TIM3EN, (uint32_t)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN }, // Example: TIM3_CH2 on PA7 (AF2)
    { TIM3, 3, GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3, (uint32_t)&RCC->APB1ENR, RCC_APB1ENR_TIM3EN, (uint32_t)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN }, // Example: TIM3_CH3 on PB0 (AF2)
    { TIM3, 4, GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3, (uint32_t)&RCC->APB1ENR, RCC_APB1ENR_TIM3EN, (uint32_t)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN }, // Example: TIM3_CH4 on PB1 (AF2)
    { TIM4, 1, GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4, (uint32_t)&RCC->APB1ENR, RCC_APB1ENR_TIM4EN, (uint32_t)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN }, // Example: TIM4_CH1 on PB6 (AF2)
    { TIM4, 2, GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4, (uint32_t)&RCC->APB1ENR, RCC_APB1ENR_TIM4EN, (uint32_t)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN }, // Example: TIM4_CH2 on PB7 (AF2)
    // Add other channels as needed based on TRD_Channel_t definition and hardware
};

// Macro to get the number of defined channels in the config table
#define PWM_CHANNEL_COUNT (sizeof(PWM_ChannelConfig) / sizeof(PWM_ChannelConfig[0]))


/**
 * @brief Internal helper function to get the channel configuration.
 * @param TRD_Channel The channel enum value.
 * @return Pointer to the configuration struct, or NULL if invalid channel.
 */
static const PWM_ChannelConfig_t* PWM_GetChannelConfig(TRD_Channel_t TRD_Channel)
{
    // Cast to uint32_t for comparison with array index
    uint32_t index = (uint32_t)TRD_Channel;
    if (index < PWM_CHANNEL_COUNT)
    {
        return &PWM_ChannelConfig[index];
    }
    return NULL; // Invalid channel
}


/**
 * @brief Internal helper function to get the timer clock frequency.
 * @param TIMx Pointer to the Timer peripheral base address.
 * @return Timer clock frequency in Hz.
 */
static uint32_t PWM_GetTimerClockFreq(TIM_TypeDef *TIMx)
{
    RCC_ClkInitTypeDef rcc_clks;
    RCC_OscInitTypeDef rcc_osc;
    uint32_t pclk1_freq, pclk2_freq;
    uint32_t flashtemp = 0;
    uint32_t preftemp = 0;

    // Get system clocks
    // NOTE: This requires SystemCoreClock and the functions
    //       RCC_GetClockConfig and RCC_GetOscConfig which are typically
    //       provided by STM32Cube or CMSIS system files.
    //       If not using Cube/CMSIS functions, this needs to be implemented
    //       by reading RCC->CFGR register based on SystemCoreClock value.
    //       For bare-metal, we will read CFGR directly based on SystemCoreClock assumption.

    uint32_t SystemCoreClock = 16000000; // Assume default HSI clock if not set elsewhere
                                         // PRODUCTION NOTE: SystemCoreClock must be correctly
                                         // updated by the system initialization code.

    // Read PCLK1 and PCLK2 prescalers from RCC->CFGR
    preftemp = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos; // APB1 Prescaler
    if (preftemp < 4) // Division by 1
    {
        pclk1_freq = SystemCoreClock;
    }
    else // Division by 2, 4, 8, or 16
    {
        pclk1_freq = SystemCoreClock >> (preftemp - 3); // 4->/2, 5->/4, 6->/8, 7->/16
    }

    preftemp = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos; // APB2 Prescaler
    if (preftemp < 4) // Division by 1
    {
        pclk2_freq = SystemCoreClock;
    }
    else // Division by 2, 4, 8, or 16
    {
        pclk2_freq = SystemCoreClock >> (preftemp - 3); // 4->/2, 5->/4, 6->/8, 7->/16
    }


    // Timers connected to APB1 domain (TIM2, TIM3, TIM4, TIM5)
    if ((TIMx == TIM2) || (TIMx == TIM3) || (TIMx == TIM4) || (TIMx == TIM5))
    {
        // If APB1 prescaler is 1, timer clock is PCLK1.
        // If APB1 prescaler is > 1, timer clock is 2 * PCLK1.
        if (((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) < 4)
        {
             return pclk1_freq;
        }
        else
        {
            return pclk1_freq * 2;
        }
    }
    // Timers connected to APB2 domain (TIM1, TIM9, TIM10, TIM11)
    else if ((TIMx == TIM1) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
    {
        // If APB2 prescaler is 1, timer clock is PCLK2.
        // If APB2 prescaler is > 1, timer clock is 2 * PCLK2.
         if (((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos) < 4)
        {
             return pclk2_freq;
        }
        else
        {
            return pclk2_freq * 2;
        }
    }
    else
    {
        // Unknown timer
        return 0; // Or some error indicator
    }
}


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for the specified channel.
 * Configures GPIO and timer peripheral registers.
 * @param TRD_Channel The channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_ChannelConfig_t* config = PWM_GetChannelConfig(TRD_Channel);

    if (config == NULL)
    {
        // Invalid channel provided. Handle error (e.g., assert, return).
        // For production, maybe a log or error flag.
        return;
    }

    // 1. Enable GPIO clock
    // Use direct register access for the specified RCC AHB1 ENR
    *((volatile uint32_t*)config->RCC_AHB1_ENR) |= config->RCC_AHB1_Mask;

    // Ensure clock is stable (optional, but good practice)
    (void)(*((volatile uint32_t*)config->RCC_AHB1_ENR));


    // 2. Configure GPIO pin for Alternate Function
    // MODER: Set pin to Alternate Function mode (0b10)
    uint32_t pin_pos = config->GPIO_Pin; // Assuming GPIO_PIN_X is a bitmask like (1 << X)
    uint32_t pin_idx = 0;
    // Determine pin index (0-15) from mask
    if (pin_pos > 0) {
        while (!((pin_pos >> pin_idx) & 1)) {
            pin_idx++;
        }
    } else {
        // Invalid pin mask (0). Handle error.
        return;
    }

    // Clear and set MODER bits
    config->GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (pin_idx * 2)); // Clear bits (0b00)
    config->GPIOx->MODER |= (GPIO_MODER_MODER0_1 << (pin_idx * 2)); // Set to AF mode (0b10)

    // OTYPER: Push-Pull (0b0)
    config->GPIOx->OTYPER &= ~(GPIO_OTYPER_OT0 << pin_idx);

    // OSPEEDR: High speed (0b10 or 0b11) - choose appropriate speed
    config->GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0 << (pin_idx * 2)); // Clear bits
    config->GPIOx->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR0_1 << (pin_idx * 2)); // Set to High speed (0b10) - adjust if needed

    // PUPDR: No Pull-up/Pull-down (0b00)
    config->GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin_idx * 2)); // Clear bits

    // AFR: Set Alternate Function number
    // AFR[0] for pins 0-7, AFR[1] for pins 8-15
    if (pin_idx < 8)
    {
        config->GPIOx->AFR[0] &= ~(0xF << (pin_idx * 4)); // Clear 4 bits for the pin
        config->GPIOx->AFR[0] |= (config->GPIO_AF << (pin_idx * 4)); // Set AF
    }
    else
    {
        config->GPIOx->AFR[1] &= ~(0xF << ((pin_idx - 8) * 4)); // Clear 4 bits for the pin
        config->GPIOx->AFR[1] |= (config->GPIO_AF << ((pin_idx - 8) * 4)); // Set AF
    }

    // 3. Enable Timer clock
    // Use direct register access for the specified RCC APB ENR
    *((volatile uint32_t*)config->RCC_APB_ENR) |= config->RCC_APB_Mask;

    // Ensure clock is stable (optional, but good practice)
    (void)(*((volatile uint32_t*)config->RCC_APB_ENR));


    // 4. Configure Timer Peripheral for PWM
    TIM_TypeDef *TIMx = config->TIMx;

    // Disable timer counter during configuration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set timer to Up-counting mode, disable clock division, disable buffer mode
    TIMx->CR1 = (TIMx->CR1 & ~(TIM_CR1_CMS | TIM_CR1_CKD | TIM_CR1_DIR)) | TIM_CR1_ARPE; // Enable auto-reload preload

    // Set PSC and ARR to default/safe values initially. Set_Freq will configure later.
    TIMx->PSC = 0; // No prescaler initially
    TIMx->ARR = 0xFFFF; // Max period initially

    // Configure PWM mode for the specific channel
    volatile uint32_t *ccmr_reg;
    uint32_t ccmr_shift;
    uint32_t ccmr_mask = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE; // Mask for OCxM and OCxPE bits (adjust for other channels)

    switch (config->TIM_Channel)
    {
        case 1:
            ccmr_reg = &TIMx->CCMR1;
            ccmr_shift = 0; // Bits 0-6 for CC1
            break;
        case 2:
            ccmr_reg = &TIMx->CCMR1;
            ccmr_shift = 8; // Bits 8-14 for CC2
            break;
        case 3:
            ccmr_reg = &TIMx->CCMR2;
            ccmr_shift = 0; // Bits 0-6 for CC3
            break;
        case 4:
            ccmr_reg = &TIMx->CCMR2;
            ccmr_shift = 8; // Bits 8-14 for CC4
            break;
        default:
            // Invalid channel number, should not happen with correct config table
            return;
    }

    // Clear existing mode bits and set PWM Mode 1 (0b110) with preload enabled (OCxPE)
    // OCxM bits: 0b110 for PWM Mode 1
    *ccmr_reg &= ~(ccmr_mask << ccmr_shift);
    *ccmr_reg |= (((TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1PE) << ccmr_shift); // 0b110 for mode, 0b1 for preload

    // Configure Output Compare Enable in CCER
    // CCxE bit: 0b1 for output enabled
    uint32_t ccer_mask = (TIM_CCER_CC1E << ((config->TIM_Channel - 1) * 4)); // Mask for CCxE bit
    TIMx->CCER &= ~ccer_mask; // Clear enable bit
    // Do NOT enable output here, PWM_Start will do that.

    // Set initial duty cycle (e.g., 0%)
    volatile uint32_t *ccr_reg;
     switch (config->TIM_Channel)
    {
        case 1: ccr_reg = &TIMx->CCR1; break;
        case 2: ccr_reg = &TIMx->CCR2; break;
        case 3: ccr_reg = &TIMx->CCR3; break;
        case 4: ccr_reg = &TIMx->CCR4; break;
        default: return; // Should not happen
    }
    *ccr_reg = 0; // Set initial pulse to 0

    // Generate an update event to load the configuration
    TIMx->EGR = TIM_EGR_UG;

    // Timer counter remains disabled until PWM_Start is called
}

/**
 * @brief Configures the PWM frequency and duty cycle.
 * @param TRD_Channel The channel to configure.
 * @param frequency Desired frequency in Hz.
 * @param duty Desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, uint32_t frequency, uint8_t duty)
{
    const PWM_ChannelConfig_t* config = PWM_GetChannelConfig(TRD_Channel);

    if (config == NULL || frequency == 0 || duty > 100)
    {
        // Invalid channel, frequency or duty. Handle error.
        return;
    }

    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t timer_clock = PWM_GetTimerClockFreq(TIMx);

    if (timer_clock == 0)
    {
        // Could not determine timer clock frequency. Handle error.
        return;
    }

    // Calculate ARR and PSC
    // Timer Frequency = TimerClock / ((PSC + 1) * (ARR + 1))
    // Target: TimerClock / ((PSC + 1) * (ARR + 1)) = frequency
    // (PSC + 1) * (ARR + 1) = TimerClock / frequency

    uint32_t total_ticks = timer_clock / frequency;

    // Find PSC and ARR. Prioritize smaller PSC for higher ARR resolution.
    uint32_t psc = 0;
    uint32_t arr = 0;
    uint32_t temp_arr_plus_1;

    // Iterate through possible PSC values
    for (psc = 0; psc <= 0xFFFF; psc++) // PSC is 16-bit
    {
        if ((psc + 1) == 0) continue; // Avoid division by zero

        temp_arr_plus_1 = total_ticks / (psc + 1);

        // Check if division is exact and result fits in ARR (16-bit for TIM3/TIM4)
        if ((total_ticks % (psc + 1)) == 0 && temp_arr_plus_1 > 0 && temp_arr_plus_1 <= 0x10000)
        {
            arr = temp_arr_plus_1 - 1;
            break; // Found a valid combination
        }
    }

    if (psc > 0xFFFF || arr > 0xFFFF)
    {
        // Could not find a suitable PSC/ARR combination for the desired frequency.
        // Frequency is likely too low or too high. Handle error.
        return;
    }


    // Calculate Pulse (CCRx value) for duty cycle
    // Pulse = (ARR + 1) * (duty / 100.0)
    // Using integer math: Pulse = ((ARR + 1) * duty) / 100
    uint32_t pulse = ((arr + 1) * duty) / 100;

    // Make sure pulse doesn't exceed ARR+1 (100% duty should set CCRx = ARR + 1?)
    // Or typically CCRx is set to ARR for 100% in up-counting PWM mode 1
    // Let's follow the definition: active while CNT < CCRx.
    // 0% duty -> pulse = 0
    // 100% duty -> pulse = ARR + 1. If CNT counts up to ARR and then resets,
    // CNT < ARR + 1 is always true for CNT values 0 to ARR.
    // So setting CCRx to ARR+1 or ARR effectively gives 100% duty in up-counting mode 1.
    // Setting CCRx = ARR is more common for 100% duty. Let's use ARR for 100%.
    if (duty == 100) {
         pulse = arr; // For 100% duty, output is high when CNT < ARR (i.e., always high except during the update)
    } else if (duty == 0) {
        pulse = 0; // For 0% duty, output is high when CNT < 0 (never)
    } else {
        // For 1-99%, pulse is calculated relative to ARR + 1
        pulse = ((arr + 1) * duty) / 100;
    }


    // Update timer registers
    // Ensure timer is disabled or update generation handles loading
    // TIM_CR1_UDIS can prevent update event from loading registers immediately.
    // Setting PSC and ARR can cause an update event depending on TIM_CR1_URS.
    // Explicit UG generation is safer after setting everything.

    // Disable counter temporarily if needed for safe update, depends on specific usage
    // If ARPE is enabled (set in Init), ARR is buffered and loaded at the next UG.
    // If OCxPE is enabled (set in Init), CCRx is buffered and loaded at the next UG.
    // PSC update is controlled by URS bit in CR1 and UG bit in EGR.
    // With URS=0 (default), UG in EGR forces an update, loading PSC, ARR (if buffered), CCRx (if buffered).
    // So, we can set the values and then generate an update.

    TIMx->PSC = psc;
    TIMx->ARR = arr;

    volatile uint32_t *ccr_reg;
     switch (config->TIM_Channel)
    {
        case 1: ccr_reg = &TIMx->CCR1; break;
        case 2: ccr_reg = &TIMx->CCR2; break;
        case 3: ccr_reg = &TIMx->CCR3; break;
        case 4: ccr_reg = &TIMx->CCR4; break;
        default: return; // Should not happen
    }
    *ccr_reg = pulse;

    // Generate update event to load the new values from preload registers
    // This happens at the end of the period if URS=0 (default)
    // Or immediately if UG bit is set in EGR. Setting UG is common after config changes.
    TIMx->EGR = TIM_EGR_UG;

    // Re-enable the counter if it was stopped for configuration (Init doesn't start it)
    // If Start was called before, it will remain running.
}

/**
 * @brief Starts PWM signal generation for the selected channel.
 * @param TRD_Channel The channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PWM_ChannelConfig_t* config = PWM_GetChannelConfig(TRD_Channel);

    if (config == NULL)
    {
        // Invalid channel. Handle error.
        return;
    }

    TIM_TypeDef *TIMx = config->TIMx;

    // Enable the specific Capture/Compare output channel
    uint32_t ccer_mask = (TIM_CCER_CC1E << ((config->TIM_Channel - 1) * 4)); // Mask for CCxE bit
    TIMx->CCER |= ccer_mask;

    // For TIM1/TIM8 (advanced timers), enable the main output latch (MOE)
    // TIM3/TIM4 do not have this bit.
    // if (TIMx == TIM1 || TIMx == TIM8) {
    //     TIMx->BDTR |= TIM_BDTR_MOE;
    // }


    // Enable the counter
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Stops the PWM signal on the specified channel.
 * The timer might keep running if other channels are active.
 * @param TRD_Channel The channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PWM_ChannelConfig_t* config = PWM_GetChannelConfig(TRD_Channel);

    if (config == NULL)
    {
        // Invalid channel. Handle error.
        return;
    }

    TIM_TypeDef *TIMx = config->TIMx;

    // Disable the specific Capture/Compare output channel
    uint32_t ccer_mask = (TIM_CCER_CC1E << ((config->TIM_Channel - 1) * 4)); // Mask for CCxE bit
    TIMx->CCER &= ~ccer_mask;

    // For TIM1/TIM8 (advanced timers), check if MOE should be disabled.
    // Usually, you only disable MOE in PowerOff or if *all* advanced channels are off.
    // We won't disable MOE here for per-channel stop on advanced timers.

    // Do NOT disable the timer counter here, as it might be used by other channels.
    // PWM_PowerOff handles stopping the timer counter.
}

/**
 * @brief Fully disables all PWM timers and output channels to save power.
 * Resets associated peripherals.
 */
void PWM_PowerOff(void)
{
    const PWM_ChannelConfig_t* config;
    TIM_TypeDef *current_tim = NULL; // Keep track of timers to reset each one once

    // Iterate through all possible channels defined in the config
    for (uint32_t i = 0; i < PWM_CHANNEL_COUNT; i++)
    {
        config = &PWM_ChannelConfig[i]; // Get config directly from the array

        // Only process each timer once
        if (config->TIMx != current_tim)
        {
            current_tim = config->TIMx;

            if (current_tim != NULL)
            {
                 // Disable timer counter
                current_tim->CR1 &= ~TIM_CR1_CEN;

                // Disable all output channels on this timer (clear all CCxE bits)
                current_tim->CCER &= ~TIM_CCER_CC1E;
                current_tim->CCER &= ~TIM_CCER_CC2E;
                current_tim->CCER &= ~TIM_CCER_CC3E;
                current_tim->CCER &= ~TIM_CCER_CC4E;

                // For TIM1/TIM8, disable Main Output Enable (MOE)
                // if (current_tim == TIM1 || current_tim == TIM8) {
                //     current_tim->BDTR &= ~TIM_BDTR_MOE;
                // }

                // Reset the timer peripheral registers
                // This requires knowing which RSTR register corresponds to the timer
                // Use direct register access based on timer address
                if (current_tim == TIM2) RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST; else
                if (current_tim == TIM3) RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST; else
                if (current_tim == TIM4) RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST; else
                if (current_tim == TIM5) RCC->APB1RSTR |= RCC_APB1RSTR_TIM5RST; else
                if (current_tim == TIM1) RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; else
                if (current_tim == TIM9) RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST; else
                if (current_tim == TIM10) RCC->APB2RSTR |= RCC_APB2RSTR_TIM10RST; else
                if (current_tim == TIM11) RCC->APB2RSTR |= RCC_APB2RSTR_TIM11RST; // Add other timers if used
                // Release the reset
                if (current_tim == TIM2) RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST; else
                if (current_tim == TIM3) RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST; else
                if (current_tim == TIM4) RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST; else
                if (current_tim == TIM5) RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM5RST; else
                if (current_tim == TIM1) RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST; else
                if (current_tim == TIM9) RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST; else
                if (current_tim == TIM10) RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM10RST; else
                if (current_tim == TIM11) RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM11RST; // Add other timers if used


                // Disable the timer clock
                 if (current_tim == TIM2) RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; else
                if (current_tim == TIM3) RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; else
                if (current_tim == TIM4) RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; else
                if (current_tim == TIM5) RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN; else
                if (current_tim == TIM1) RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; else
                if (current_tim == TIM9) RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN; else
                if (current_tim == TIM10) RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN; else
                if (current_tim == TIM11) RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN; // Add other timers if used

                // Optionally, configure GPIO pins back to a default state (e.g., Input, no pull)
                // This would require tracking which pins were used and configuring them back.
                // timer output stops the PWM signal on the pin anyway.
                // If truly necessary for ultra-low power, iterate GPIOs used and set MODER to Input (0b00), PUPDR to No Pull.
            }
        }
    }
}

// Helper macro for GPIO pin index determination (needed for MODER/OSPEEDR/PUPDR/AFR calculations)
#define GPIO_PIN_0 ((uint16_t)0x0001)  /* Pin 0 selected */
#define GPIO_PIN_1 ((uint16_t)0x0002)  /* Pin 1 selected */
#define GPIO_PIN_2 ((uint16_t)0x0004)  /* Pin 2 selected */
#define GPIO_PIN_3 ((uint16_t)0x0008)  /* Pin 3 selected */
#define GPIO_PIN_4 ((uint16_t)0x0010)  /* Pin 4 selected */
#define GPIO_PIN_5 ((uint16_t)0x0020)  /* Pin 5 selected */
#define GPIO_PIN_6 ((uint16_t)0x0040)  /* Pin 6 selected */
#define GPIO_PIN_7 ((uint16_t)0x0080)  /* Pin 7 selected */
#define GPIO_PIN_8 ((uint16_t)0x0100)  /* Pin 8 selected */
#define GPIO_PIN_9 ((uint16_t)0x0200)  /* Pin 9 selected */
#define GPIO_PIN_10 ((uint16_t)0x0400) /* Pin 10 selected */
#define GPIO_PIN_11 ((uint16_t)0x0800) /* Pin 11 selected */
#define GPIO_PIN_12 ((uint16_t)0x1000) /* Pin 12 selected */
#define GPIO_PIN_13 ((uint16_t)0x2000) /* Pin 13 selected */
#define GPIO_PIN_14 ((uint16_t)0x4000) /* Pin 14 selected */
#define GPIO_PIN_15 ((uint16_t)0x8000) /* Pin 15 selected */
#define GPIO_PIN_All ((uint16_t)0xFFFF)/* All pins selected */

// Define AF values used in the example config
#define GPIO_AF2_TIM3 ((uint8_t)0x02) /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TIM4 ((uint8_t)0x02) /* TIM4 Alternate Function mapping */

/* Helper macro to get pin index from bit mask */
static inline uint32_t GetPinIndex(uint32_t pin_mask)
{
    uint32_t index = 0;
    if (pin_mask > 0) {
        while (!((pin_mask >> index) & 1)) {
            index++;
            if (index >= 16) return 16; // Error: Pin not found or invalid mask
        }
    } else {
        return 16; // Error: Invalid pin mask
    }
    return index;
}

// Re-implementing the GPIO configuration within Init using the helper
// Original Init function has the logic, just need to ensure the pin_idx logic is sound.
// The GetPinIndex helper simplifies the logic. Let's replace the inline logic with the helper.

/**
 * @brief Initializes the PWM hardware for the specified channel.
 * Configures GPIO and timer peripheral registers.
 * @param TRD_Channel The channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_ChannelConfig_t* config = PWM_GetChannelConfig(TRD_Channel);

    if (config == NULL)
    {
        // Invalid channel provided.
        return;
    }

    uint32_t pin_idx = GetPinIndex(config->GPIO_Pin);
    if (pin_idx >= 16) {
        // Invalid pin mask in config table
        return;
    }

    // 1. Enable GPIO clock
    *((volatile uint32_t*)config->RCC_AHB1_ENR) |= config->RCC_AHB1_Mask;
    (void)(*((volatile uint32_t*)config->RCC_AHB1_ENR)); // Ensure clock is stable

    // 2. Configure GPIO pin for Alternate Function
    // Clear and set MODER bits for the specific pin
    config->GPIOx->MODER &= ~(0x3UL << (pin_idx * 2)); // Clear bits
    config->GPIOx->MODER |= (0x2UL << (pin_idx * 2)); // Set to AF mode (0b10)

    // OTYPER: Push-Pull (0b0) - Clear the bit
    config->GPIOx->OTYPER &= ~(0x1UL << pin_idx);

    // OSPEEDR: High speed (0b10) - Clear and set bits
    config->GPIOx->OSPEEDR &= ~(0x3UL << (pin_idx * 2)); // Clear bits
    config->GPIOx->OSPEEDR |= (0x2UL << (pin_idx * 2)); // Set to High speed (0b10)

    // PUPDR: No Pull-up/Pull-down (0b00) - Clear bits
    config->GPIOx->PUPDR &= ~(0x3UL << (pin_idx * 2)); // Clear bits

    // AFR: Set Alternate Function number
    if (pin_idx < 8)
    {
        config->GPIOx->AFR[0] &= ~(0xFUL << (pin_idx * 4)); // Clear 4 bits
        config->GPIOx->AFR[0] |= (config->GPIO_AF << (pin_idx * 4)); // Set AF
    }
    else
    {
        config->GPIOx->AFR[1] &= ~(0xFUL << ((pin_idx - 8) * 4)); // Clear 4 bits
        config->GPIOx->AFR[1] |= (config->GPIO_AF << ((pin_idx - 8) * 4)); // Set AF
    }

    // 3. Enable Timer clock
    *((volatile uint32_t*)config->RCC_APB_ENR) |= config->RCC_APB_Mask;
    (void)(*((volatile uint32_t*)config->RCC_APB_ENR)); // Ensure clock is stable

    // 4. Configure Timer Peripheral for PWM
    TIM_TypeDef *TIMx = config->TIMx;

    // Reset timer (optional here, can be done in PowerOff, Init is for specific channel)
    // Resetting the timer in Init might affect other channels already initialized on the same timer.
    // Better to configure just the required channel bits.

    // Disable timer counter during configuration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set timer to Up-counting mode (DIR=0), disable clock division (CKD=0), disable buffer mode (CMS=0)
    // Enable auto-reload preload (ARPE=1)
    TIMx->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_CKD | TIM_CR1_DIR);
    TIMx->CR1 |= TIM_CR1_ARPE;

    // Configure PWM mode for the specific channel
    volatile uint32_t *ccmr_reg;
    uint32_t ccmr_shift;

    switch (config->TIM_Channel)
    {
        case 1:
            ccmr_reg = &TIMx->CCMR1;
            ccmr_shift = 0; // CC1 is bits 0-7
            break;
        case 2:
            ccmr_reg = &TIMx->CCMR1;
            ccmr_shift = 8; // CC2 is bits 8-15
            break;
        case 3:
            ccmr_reg = &TIMx->CCMR2;
            ccmr_shift = 0; // CC3 is bits 0-7
            break;
        case 4:
            ccmr_reg = &TIMx->CCMR2;
            ccmr_shift = 8; // CC4 is bits 8-15
            break;
        default:
            // Invalid channel number, should not happen with correct config table
            return;
    }

    // Clear existing CCx channel configuration bits (OCxM, OCxPE)
    *ccmr_reg &= ~(0xFFUL << ccmr_shift); // Clear 8 bits (OCxM and OCxPE)

    // Set PWM Mode 1 (0b110) and Preload Enable (0b1)
    *ccmr_reg |= ((0x6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE) << ccmr_shift; // Mode 1 (0b110) + Preload

    // Configure Capture/Compare Enable register (CCER)
    // Clear the CCxE, CCxNE, CCxP, CCxNP bits for this channel
    uint32_t ccer_channel_mask = (0xFUL << ((config->TIM_Channel - 1) * 4)); // CC1 uses bits 0-3, CC2 4-7, etc.
    TIMx->CCER &= ~ccer_channel_mask;
    // Set the CCxE bit (Output Enable) - Do NOT enable here, Start function does it.
    // Set the CCxP bit for output polarity (0 for active high, 1 for active low). Default is 0.

    // Set initial duty cycle (e.g., 0%)
    volatile uint32_t *ccr_reg;
     switch (config->TIM_Channel)
    {
        case 1: ccr_reg = &TIMx->CCR1; break;
        case 2: ccr_reg = &TIMx->CCR2; break;
        case 3: ccr_reg = &TIMx->CCR3; break;
        case 4: ccr_reg = &TIMx->CCR4; break;
        default: return; // Should not happen
    }
    *ccr_reg = 0; // Set initial pulse to 0

    // Set PSC and ARR to default/safe values initially. Set_Freq will configure later.
    TIMx->PSC = 0;
    TIMx->ARR = 0xFFFF;

    // Generate an update event to load the configuration immediately
    // This also clears the counter
    TIMx->EGR = TIM_EGR_UG;

    // Wait for the update flag to be set (optional, ensures config is loaded)
    // while((TIMx->SR & TIM_SR_UIF) == 0);
    // TIMx->SR &= ~TIM_SR_UIF; // Clear update flag

    // Timer counter remains disabled until PWM_Start is called
}