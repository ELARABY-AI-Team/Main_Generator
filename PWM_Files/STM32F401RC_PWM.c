/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include "stm32f4xx.h" // Include CMSIS for register access

// Define custom types assuming their mapping from the header
// typedef uint32_t tlong;
// typedef uint8_t tbyte;

// Timer Clock Source: Timers on APB1 (TIM2,3,4,5,6,7,12,13,14) or APB2 (TIM1,8,9,10,11)
// The clock frequency depends on the APBx prescaler. If the prescaler is 1, the timer clock is APBx clock.
// If the prescaler is > 1, the timer clock is 2x APBx clock.
// A robust implementation would calculate this from RCC registers (e.g., reading RCC->CFGR)
// Assume APB1 timer clock = 84MHz (if APB1 prescaler is 1, or < 84MHz but prescaler > 1)
// Assume APB2 timer clock = 84MHz (if APB2 prescaler is 1, or < 84MHz but prescaler > 1)
// For STM32F401RC, SystemCoreClock can be up to 84MHz. APB1 max is 42MHz, APB2 max is 84MHz.
// If APB1 prescaler > 1, TIM clock = 2 * APB1 clock (max 84MHz). If APB1 prescaler = 1, TIM clock = APB1 clock (max 42MHz).
#define TIMER_CLK_FREQ    ((tlong)84000000) // Hz

// Define Timer reservation
// Reserved for OS/delay: TIM9, TIM10. These timers and their channels are excluded from this driver.
// Available for PWM in this driver example: TIM1, TIM2, TIM3, TIM4 (subset of channels).

// Internal structure to map logical channels to hardware configuration
typedef struct {
    TIM_TypeDef *TIMx;        // Pointer to the timer instance (e.g., TIM1, TIM2)
    uint32_t Timer_CLK_EN_BIT; // RCC clock enable bit for the timer
    uint32_t RSTR_BIT;         // RCC reset register bit for the timer (APB1RSTR or APB2RSTR)
    GPIO_TypeDef *GPIOx;      // Pointer to the GPIO port (e.g., GPIOA, GPIOB)
    uint32_t GPIO_CLK_EN_BIT;  // RCC clock enable bit for the GPIO port
    uint32_t GPIO_Pin;         // Pin number (GPIO_Pin_0 to GPIO_Pin_15)
    uint8_t  GPIO_AF;          // Alternate function number
    uint8_t  TIM_Channel;      // Timer channel number (1, 2, 3, 4)
    uint32_t CCR_Offset;       // Offset to the Capture/Compare Register (e.g., &TIMx->CCR1 relative to TIMx)
    uint16_t Max_ARR;          // Maximum value for ARR (65535 for 16-bit, 0xFFFFFFFF for 32-bit)
} ChannelMap_t;

// Mapping of logical channels to hardware specifics
// Note: Only a few channels are mapped as examples. Add more as needed.
static const ChannelMap_t ChannelMap[] = {
    // TRD_TIM1_CH1 -> TIM1, Channel 1, PA8, AF1
    { TIM1, RCC_APB2ENR_TIM1EN, RCC_APB2RSTR_TIM1RST, GPIOA, RCC_AHB1ENR_GPIOAEN, GPIO_PIN_8, GPIO_AF1_TIM1, 1, offsetof(TIM_TypeDef, CCR1), 65535 },
    // TRD_TIM2_CH1 -> TIM2, Channel 1, PA0, AF1
    { TIM2, RCC_APB1ENR_TIM2EN, RCC_APB1RSTR_TIM2RST, GPIOA, RCC_AHB1ENR_GPIOAEN, GPIO_PIN_0, GPIO_AF1_TIM2, 1, offsetof(TIM_TypeDef, CCR1), 0xFFFFFFFF }, // TIM2 is 32-bit
    // TRD_TIM3_CH1 -> TIM3, Channel 1, PA6, AF2
    { TIM3, RCC_APB1ENR_TIM3EN, RCC_APB1RSTR_TIM3RST, GPIOA, RCC_AHB1ENR_GPIOAEN, GPIO_PIN_6, GPIO_AF2_TIM3, 1, offsetof(TIM_TypeDef, CCR1), 65535 },
    // TRD_TIM4_CH1 -> TIM4, Channel 1, PB6, AF2
    { TIM4, RCC_APB1ENR_TIM4EN, RCC_APB1RSTR_TIM4RST, GPIOB, RCC_AHB1ENR_GPIOBEN, GPIO_PIN_6, GPIO_AF2_TIM4, 1, offsetof(TIM_TypeDef, CCR1), 65535 },
    // Add more channels here following the pattern
    // Example: TRD_TIM3_CH2 -> TIM3, Channel 2, PA7, AF2
    // { TIM3, RCC_APB1ENR_TIM3EN, RCC_APB1RSTR_TIM3RST, GPIOA, RCC_AHB1ENR_GPIOAEN, GPIO_PIN_7, GPIO_AF2_TIM3, 2, offsetof(TIM_TypeDef, CCR2), 65535 },
};

// Helper function to get channel map
static const ChannelMap_t* getChannelMap(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_CHANNEL_MAX) {
        return NULL; // Invalid channel
    }
    // Check if the requested channel is in our mapped list (simple linear search)
    for (uint32_t i = 0; i < sizeof(ChannelMap) / sizeof(ChannelMap_t); ++i) {
        // This assumes a direct correlation between enum index and array index, which is not strictly safe
        // A more robust way would be to add the TRD_Channel_t enum value to the struct
        // This requires the enum in the header to match the order in the array.
        // A better approach:
        // typedef struct { TRD_Channel_t channel_id; TIM_TypeDef *TIMx; ... } ChannelMap_t;
        // Then iterate and compare channel_id.
        if (TRD_Channel == (TRD_Channel_t)i) {
             return &ChannelMap[i];
        }
    }
     return NULL; // Channel not found in the mapping
}


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    const ChannelMap_t* map = getChannelMap(TRD_Channel);
    if (!map) {
        // Handle invalid channel, e.g., log an error or assert
        return;
    }

    // 1. Enable Timer clock
    if (map->TIMx == TIM1 || map->TIMx == TIM8 || map->TIMx == TIM9 || map->TIMx == TIM10 || map->TIMx == TIM11) {
         // APB2 Timer
        RCC->APB2ENR |= map->Timer_CLK_EN_BIT;
    } else {
        // APB1 Timer (TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14)
        RCC->APB1ENR |= map->Timer_CLK_EN_BIT;
    }


    // 2. Enable GPIO clock
    RCC->AHB1ENR |= map->GPIO_CLK_EN_BIT;

    // Wait for clocks to stabilize (optional but good practice)
    // __IO uint32_t tmpreg = map->GPIOx->MODER; (Reading register after setting clock bit ensures clock is ready)
    // (void)tmpreg;
    // tmpreg = map->TIMx->CR1;
    // (void)tmpreg;


    // 3. Configure GPIO pin for Alternate Function
    // Clear MODER bits for the pin (set to Input mode first or directly to Alternate function mode 10)
    map->GPIOx->MODER &= ~(GPIO_MODER_MODER0_Msk << (map->GPIO_Pin * 2));
    // Set MODER bits to Alternate Function mode (10)
    map->GPIOx->MODER |= (GPIO_MODER_MODER0_1 << (map->GPIO_Pin * 2));

    // Configure OTYPER (Output Type): Push-pull (0) for PWM
    map->GPIOx->OTYPER &= ~(GPIO_OTYPER_OT_0 << map->GPIO_Pin); // Push-pull

    // Configure OSPEEDR (Output Speed): High speed (11) for PWM
    map->GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk << (map->GPIO_Pin * 2));
    map->GPIOx->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR0_1 | GPIO_OSPEEDR_OSPEEDR0_0) << (map->GPIO_Pin * 2); // High speed

    // Configure PUPDR (Pull-up/Pull-down): No pull-up/pull-down (00) typically
    map->GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk << (map->GPIO_Pin * 2));
    // map->GPIOx->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << (map->GPIO_Pin * 2)); // Example: Pull-up

    // Configure AFR (Alternate Function Register)
    // Determine whether to use AFR[0] (low) or AFR[1] (high)
    if (map->GPIO_Pin < 8) {
        // AFR[0] (Pins 0-7)
        map->GPIOx->AFR[0] &= ~(0xF << (map->GPIO_Pin * 4)); // Clear current AF bits
        map->GPIOx->AFR[0] |= (map->GPIO_AF << (map->GPIO_Pin * 4)); // Set new AF
    } else {
        // AFR[1] (Pins 8-15)
        map->GPIOx->AFR[1] &= ~(0xF << ((map->GPIO_Pin - 8) * 4)); // Clear current AF bits
        map->GPIOx->AFR[1] |= (map->GPIO_AF << ((map->GPIO_Pin - 8) * 4)); // Set new AF
    }

    // 4. Configure Timer Base (assuming upcounting mode)
    map->TIMx->CR1 &= ~TIM_CR1_CMS; // Set Center-aligned mode to Edge-aligned mode (00)
    map->TIMx->CR1 &= ~TIM_CR1_DIR; // Set Direction to Upcounting (0)
    map->TIMx->CR1 &= ~TIM_CR1_CKD; // Set clock division to 0 (no division)

    // 5. Configure Timer Channel for PWM Output
    volatile uint32_t* CCMRx;
    uint32_t ccmr_channel_shift; // Shift for the channel's bits in CCMRx

    // Determine which CCMR register (1 or 2) and the bit shift based on channel number
    if (map->TIM_Channel == 1 || map->TIM_Channel == 2) {
        CCMRx = &map->TIMx->CCMR1;
        ccmr_channel_shift = (map->TIM_Channel == 1) ? 0 : 8; // Bits 0-7 for CH1, 8-15 for CH2
    } else { // Channel 3 or 4
        CCMRx = &map->TIMx->CCMR2;
        ccmr_channel_shift = (map->TIM_Channel == 3) ? 0 : 8; // Bits 0-7 for CH3, 8-15 for CH4
    }

    // Clear OCxM (Output Compare mode) and OCxPE (Output Compare Preload Enable) bits
    *CCMRx &= ~(TIM_CCMR1_OC1M_Msk << ccmr_channel_shift); // Clear mode bits (6 bits total, but 3 for mode, 3 for others)
    *CCMRx &= ~(TIM_CCMR1_OC1PE_Msk << ccmr_channel_shift); // Clear preload enable bit

    // Set OCxM to PWM Mode 1 (110)
    // For CCMR1: OC1M[2:0] are bits 6:4, OC2M[2:0] are bits 14:12
    // For CCMR2: OC3M[2:0] are bits 6:4, OC4M[2:0] are bits 14:12
    // The mask and shift logic needs care. Let's use direct bit manipulation based on channel number.
     switch (map->TIM_Channel) {
        case 1:
            map->TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;     // Clear OC1M
            map->TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Set OC1M to 110 (PWM Mode 1)
            map->TIMx->CCMR1 |= TIM_CCMR1_OC1PE;     // Enable Output Compare 1 Preload
            break;
        case 2:
            map->TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;     // Clear OC2M
            map->TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // Set OC2M to 110 (PWM Mode 1)
            map->TIMx->CCMR1 |= TIM_CCMR1_OC2PE;     // Enable Output Compare 2 Preload
            break;
        case 3:
            map->TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;     // Clear OC3M
            map->TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Set OC3M to 110 (PWM Mode 1)
            map->TIMx->CCMR2 |= TIM_CCMR2_OC3PE;     // Enable Output Compare 3 Preload
            break;
        case 4:
            map->TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;     // Clear OC4M
            map->TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // Set OC4M to 110 (PWM Mode 1)
            map->TIMx->CCMR2 |= TIM_CCMR2_OC4PE;     // Enable Output Compare 4 Preload
            break;
    }

    // 6. Enable Capture/Compare output
    // Clear CCxE bits for the channel
    map->TIMx->CCER &= ~(TIM_CCER_CC1E << ((map->TIM_Channel - 1) * 4)); // CC1E, CC2E, CC3E, CC4E bits are 0, 4, 8, 12
    // Set CCxE bit to enable the output signal
    map->TIMx->CCER |= (TIM_CCER_CC1E << ((map->TIM_Channel - 1) * 4));

    // For advanced timers (TIM1), enable the main output (MOE)
    if (map->TIMx == TIM1) {
        map->TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Generate an update event to load the prescaler and ARR settings immediately
    map->TIMx->EGR |= TIM_EGR_UG;
}

/**
 * @brief Sets the PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    const ChannelMap_t* map = getChannelMap(TRD_Channel);
    if (!map || frequency == 0 || duty > 100) {
        // Handle invalid channel, zero frequency, or invalid duty cycle
        // A frequency of 0 is not possible. Duty cycle must be 0-100.
        return;
    }

    tlong timer_clk = TIMER_CLK_FREQ; // Use the defined timer clock frequency

    // Calculate Target_Ticks = clock frequency / desired frequency
    // This is the total number of timer clock cycles in one PWM period.
    tlong target_ticks = timer_clk / frequency;

    // Find PSC and ARR
    // (PSC + 1) * (ARR + 1) = target_ticks
    // We want to maximize ARR for resolution, so minimize PSC+1
    // ARR = (target_ticks / (PSC + 1)) - 1
    // We need ARR <= Max_ARR (which is map->Max_ARR)
    // (target_ticks / (PSC + 1)) - 1 <= map->Max_ARR
    // target_ticks / (PSC + 1) <= map->Max_ARR + 1
    // PSC + 1 >= target_ticks / (map->Max_ARR + 1)
    // Use integer division for ceiling-like effect: PSC + 1 = (target_ticks + map->Max_ARR) / (map->Max_ARR + 1)
    // Check if target_ticks is too small (frequency too high)
    if (target_ticks < 1) {
        // Frequency is higher than timer clock, impossible
        // Handle error
        return;
    }

    tlong psc_plus_1;
    tlong calculated_arr;

    // Find the smallest PSC that results in a valid ARR <= Max_ARR
    // Iterate PSC from 0 upwards
    for (psc_plus_1 = 1; psc_plus_1 <= 65536; ++psc_plus_1) { // PSC can be up to 65535 (so PSC+1 up to 65536)
        calculated_arr = (target_ticks / psc_plus_1); // Calculate ARR+1 first
        if (calculated_arr > 0 && calculated_arr <= (map->Max_ARR + 1)) {
             // Valid PSC+1 and ARR+1 found
             calculated_arr -= 1; // Get the actual ARR value
             break; // Exit loop
        }
        if (psc_plus_1 == 65536) { // Checked all possible PSC values
             // Could not find a suitable PSC/ARR combination
             // Handle error (frequency/resolution impossible)
             return;
        }
    }

    uint16_t final_psc = (uint16_t)(psc_plus_1 - 1);
    uint32_t final_arr = (uint32_t)calculated_arr; // Use uint32_t to accommodate 32-bit timers

    // Calculate CCR based on the duty cycle and ARR
    // CCR = (ARR + 1) * duty / 100
    tlong calculated_ccr = ( (final_arr + 1) * duty ) / 100;

    // Special cases for 0% and 100% duty cycle
    if (duty == 0) {
        calculated_ccr = 0;
    } else if (duty == 100) {
        // For PWM Mode 1, CCR >= ARR results in 100% duty cycle
        calculated_ccr = final_arr + 1; // Or simply final_arr, depending on TIMx->CR1->ARPE setting and desired behavior
                                       // A common approach is to set CCR to ARR+1 for 100% if using ARR preloading.
                                       // Let's set to ARR+1 to be explicit about staying high until counter resets.
    }

    // Apply the new PSC, ARR, and CCR values
    // Update PSC and ARR. These updates take effect at the next Update Event (counter rollover) if URS=0 in CR1.
    map->TIMx->PSC = final_psc;
    map->TIMx->ARR = final_arr;

    // Update the Capture/Compare Register (CCR). This takes effect at the next Update Event if OCxPE is enabled.
    volatile uint32_t* ccr_reg = (volatile uint32_t*)((uint32_t)map->TIMx + map->CCR_Offset);
    *ccr_reg = calculated_ccr;

    // An update event can be generated manually if needed for immediate effect (TIMx->EGR |= TIM_EGR_UG;)
    // However, relying on the next counter rollover is standard for smooth PWM changes.
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    const ChannelMap_t* map = getChannelMap(TRD_Channel);
    if (!map) {
        // Handle invalid channel
        return;
    }

    // Enable the counter
    map->TIMx->CR1 |= TIM_CR1_CEN;

    // For advanced timers (TIM1), ensure the main output is enabled
    if (map->TIMx == TIM1) {
        map->TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Re-enable the specific channel output if it was stopped with PWM_Stop
    // Note: PWM_Init enables it, PWM_Stop disables it. Start just needs to ensure CEN and MOE (if TIM1) are set.
    // If PWM_Stop only disabled CCxE, maybe re-enabling CCxE here is necessary if Start can follow Stop.
    map->TIMx->CCER |= (TIM_CCER_CC1E << ((map->TIM_Channel - 1) * 4));
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    const ChannelMap_t* map = getChannelMap(TRD_Channel);
    if (!map) {
        // Handle invalid channel
        return;
    }

    // Disable the Capture/Compare output for the specific channel.
    // This stops the PWM signal on the pin. The timer counter might continue running.
    map->TIMx->CCER &= ~(TIM_CCER_CC1E << ((map->TIM_Channel - 1) * 4));

    // Note: For TIM1, BDTR->MOE controls ALL outputs. Disabling it here would affect other channels on TIM1.
    // The requirement is to stop *the* specified channel, so just disabling CCxE is appropriate.
    // If BDTR->MOE needed to be disabled when *all* TIM1 channels are stopped, state tracking would be needed.
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function resets the timer peripherals used by this driver.
 */
void PWM_PowerOff(void) {
    // Iterate through all mapped channels and reset their timers
    // Resetting is safer than just disabling clocks as it returns registers to default state.
     for (uint32_t i = 0; i < sizeof(ChannelMap) / sizeof(ChannelMap_t); ++i) {
        const ChannelMap_t* map = &ChannelMap[i];

        // Assert reset for the timer peripheral
         if (map->TIMx == TIM1 || map->TIMx == TIM8 || map->TIMx == TIM9 || map->TIMx == TIM10 || map->TIMx == TIM11) {
             // APB2 Timer
             RCC->APB2RSTR |= map->RSTR_BIT;
         } else {
             // APB1 Timer
             RCC->APB1RSTR |= map->RSTR_BIT;
         }
    }

    // De-assert reset for the timer peripherals
     for (uint32_t i = 0; i < sizeof(ChannelMap) / sizeof(ChannelMap_t); ++i) {
         const ChannelMap_t* map = &ChannelMap[i];

         if (map->TIMx == TIM1 || map->TIMx == TIM8 || map->TIMx == TIM9 || map->TIMx == TIM10 || map->TIMx == TIM11) {
              // APB2 Timer
             RCC->APB2RSTR &= ~map->RSTR_BIT;
         } else {
             // APB1 Timer
             RCC->APB1RSTR &= ~map->RSTR_BIT;
         }
     }

     // Optionally, disable the clocks for the timers. Reset implies this, but explicitly disabling is also fine.
     // Disabling clocks prevents accidental access if reset bits are cleared later without re-init.
    for (uint32_t i = 0; i < sizeof(ChannelMap) / sizeof(ChannelMap_t); ++i) {
        const ChannelMap_t* map = &ChannelMap[i];

        if (map->TIMx == TIM1 || map->TIMx == TIM8 || map->TIMx == TIM9 || map->TIMx == TIM10 || map->TIMx == TIM11) {
             RCC->APB2ENR &= ~map->Timer_CLK_EN_BIT;
         } else {
             RCC->APB1ENR &= ~map->Timer_CLK_EN_BIT;
         }
    }

     // Optionally, disable GPIO clocks or reconfigure pins to a low-power state (e.g., analog input).
     // Disabling GPIO clocks is generally not done per-peripheral as multiple peripherals might share a port.
     // Reconfiguring pins requires tracking which pins were used by active channels, which adds complexity.
     // Resetting the timer peripherals is usually sufficient for power down concerning the timers themselves.
}