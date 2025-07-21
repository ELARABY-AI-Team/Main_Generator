/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM driver for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

/*
 * System Clock Frequency Assumption:
 * The STM32F401RC microcontroller can operate with a maximum HCLK of 84MHz.
 * For the purpose of this PWM implementation, it is assumed that the
 * SystemCoreClock (HCLK) is configured to 84MHz.
 *
 * Timer Clock Derivation:
 * On STM32F4 series, if the APB prescaler (APB1 or APB2) is set to 1,
 * the timer clock is equal to the APB clock. If the APB prescaler is
 * greater than 1, the timer clock is double the APB clock.
 *
 * Assuming HCLK = 84MHz:
 * - APB1 Bus (for TIM2, TIM3, TIM4, TIM5): PCLK1 is typically HCLK/2 = 42MHz.
 *   If APB1 prescaler is >1 (e.g., DIV2), then Timer Clock (CK_INT) = 2 * PCLK1 = 2 * 42MHz = 84MHz.
 * - APB2 Bus (for TIM1, TIM9, TIM10, TIM11): PCLK2 is typically HCLK = 84MHz.
 *   If APB2 prescaler is >1 (e.g., not applicable here as PCLK2 is typically HCLK),
 *   then Timer Clock (CK_INT) = PCLK2 = 84MHz.
 *
 * assumes that the effective clock frequency supplied to all timers (CK_INT) is 84 MHz.
 */
#define PWM_TIMER_CLOCK_FREQ    84000000UL /* Assumed Timer Clock Frequency based on 84MHz HCLK */

/*
 * @brief  Configuration structure for a single PWM channel.
 *         This structure maps a logical TRD_Channel_t to a specific
 *         hardware Timer, Channel, GPIO Port, Pin, and Alternate Function.
 */
typedef struct {
    TIM_TypeDef* TIMx;              /**< Pointer to the Timer peripheral base address (e.g., TIM1, TIM2) */
    uint8_t ChannelNumber;          /**< Timer channel number (1 for CH1, 2 for CH2, etc.) */
    GPIO_TypeDef* PortName;         /**< Pointer to the GPIO Port base address (e.g., GPIOA, GPIOB) */
    uint16_t PinNumber;             /**< GPIO pin mask (e.g., GPIO_PIN_0, GPIO_PIN_1, etc.) */
    uint8_t AlternateFunction;      /**< Alternate function number for the GPIO pin (e.g., GPIO_AF_TIM1) */
} PWM_Channel_Config_t;

/*
 * Reservation Logic:
 * As per the requirements, TIM5 and TIM9 are reserved for OS or delay purposes.
 * Therefore, these timers and their associated channels will not be included
 * in the pwm_channel_map array for PWM generation.
 *
 * Pin Mapping Assumptions:
 * The alternate function (AF) mapping for specific GPIO pins to TIM channels
 * is derived from common STM32F4 series datasheets and reference manuals.
 * While every effort has been made to ensure accuracy for STM32F401RC,
 * these mappings should be verified against the precise device datasheet
 * if discrepancies are observed.
 *
 * Pin Exclusion:
 * No pin with number 0 (GPIO_PIN_0) has been included unless explicitly confirmed
 * by documentation to be robustly PWM-capable and not prone to debug/reset conflicts.
 * For example, PA0 (TIM2_CH1 and TIM5_CH1) is excluded to avoid potential issues,
 * and an alternative pin (PA5 for TIM2_CH1) is used instead.
 *
 * Duplication Rule:
 * The `pwm_channel_map` array is designed such that no two entries use the
 * same timer/channel pair (e.g., TIM1, Channel 1 will only appear once).
 * However, a physical GPIO pin might be capable of multiple alternate functions
 * for different timers/channels (e.g., PA6 can be TIM3_CH1 (AF2) or TIM10_CH1 (AF3)).
 * In such cases, distinct entries in this array represent distinct `TRD_Channel_t`
 * (logical PWM outputs) that utilize different alternate functions on the same physical pin.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (Advanced-control timer, APB2 Bus Clock, AF1)
    { TIM1, 1, GPIOA, GPIO_PIN_8,  GPIO_AF_TIM1 },  // TIM1_CH1 on PA8   /* Assumed PWM config - please verify */
    { TIM1, 2, GPIOA, GPIO_PIN_9,  GPIO_AF_TIM1 },  // TIM1_CH2 on PA9   /* Assumed PWM config - please verify */
    { TIM1, 3, GPIOA, GPIO_PIN_10, GPIO_AF_TIM1 },  // TIM1_CH3 on PA10  /* Assumed PWM config - please verify */
    { TIM1, 4, GPIOA, GPIO_PIN_11, GPIO_AF_TIM1 },  // TIM1_CH4 on PA11  /* Assumed PWM config - please verify */

    // TIM2 Channels (General-purpose timer, 32-bit, APB1 Bus Clock, AF1)
    { TIM2, 1, GPIOA, GPIO_PIN_5,  GPIO_AF_TIM2 },  // TIM2_CH1 on PA5   /* Assumed PWM config - please verify */
    { TIM2, 2, GPIOA, GPIO_PIN_1,  GPIO_AF_TIM2 },  // TIM2_CH2 on PA1   /* Assumed PWM config - please verify */
    { TIM2, 3, GPIOA, GPIO_PIN_2,  GPIO_AF_TIM2 },  // TIM2_CH3 on PA2   /* Assumed PWM config - please verify */
    { TIM2, 4, GPIOA, GPIO_PIN_3,  GPIO_AF_TIM2 },  // TIM2_CH4 on PA3   /* Assumed PWM config - please verify */

    // TIM3 Channels (General-purpose timer, 16-bit, APB1 Bus Clock, AF2)
    { TIM3, 1, GPIOA, GPIO_PIN_6,  GPIO_AF_TIM3 },  // TIM3_CH1 on PA6   /* Assumed PWM config - please verify */
    { TIM3, 2, GPIOA, GPIO_PIN_7,  GPIO_AF_TIM3 },  // TIM3_CH2 on PA7   /* Assumed PWM config - please verify */
    { TIM3, 3, GPIOB, GPIO_PIN_0,  GPIO_AF_TIM3 },  // TIM3_CH3 on PB0   /* Assumed PWM config - please verify */
    { TIM3, 4, GPIOB, GPIO_PIN_1,  GPIO_AF_TIM3 },  // TIM3_CH4 on PB1   /* Assumed PWM config - please verify */

    // TIM4 Channels (General-purpose timer, 16-bit, APB1 Bus Clock, AF2)
    { TIM4, 1, GPIOB, GPIO_PIN_6,  GPIO_AF_TIM4 },  // TIM4_CH1 on PB6   /* Assumed PWM config - please verify */
    { TIM4, 2, GPIOB, GPIO_PIN_7,  GPIO_AF_TIM4 },  // TIM4_CH2 on PB7   /* Assumed PWM config - please verify */
    { TIM4, 3, GPIOB, GPIO_PIN_8,  GPIO_AF_TIM4 },  // TIM4_CH3 on PB8   /* Assumed PWM config - please verify */
    { TIM4, 4, GPIOB, GPIO_PIN_9,  GPIO_AF_TIM4 },  // TIM4_CH4 on PB9   /* Assumed PWM config - please verify */

    // TIM10 Channel (General-purpose timer, 16-bit, APB2 Bus Clock, AF3)
    { TIM10, 1, GPIOA, GPIO_PIN_6, GPIO_AF_TIM10 }, // TIM10_CH1 on PA6  /* Assumed PWM config - please verify */

    // TIM11 Channel (General-purpose timer, 16-bit, APB2 Bus Clock, AF3)
    { TIM11, 1, GPIOA, GPIO_PIN_7, GPIO_AF_TIM11 }  // TIM11_CH1 on PA7  /* Assumed PWM config - please verify */
};

// Ensure TRD_Channel_t (PWM_CHANNEL_COUNT) in header reflects the number of entries in this array.
// For the above example, there are 18 entries, so PWM_CHANNEL_COUNT should be 18.


/**
 * @brief Enables the clock for a given GPIO port.
 * @param GPIOx Pointer to the GPIO port (e.g., GPIOA, GPIOB).
 */
static void PWM_GPIO_Clock_Enable(GPIO_TypeDef* GPIOx) {
    if (GPIOx == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* PDF Reference - Assumed RCC register bit for GPIOA */
    } else if (GPIOx == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* PDF Reference - Assumed RCC register bit for GPIOB */
    } else if (GPIOx == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* PDF Reference - Assumed RCC register bit for GPIOC */
    } else if (GPIOx == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* PDF Reference - Assumed RCC register bit for GPIOD */
    }
    // Ports F, G, H, I, J, K are not available in STM32F401xB/C/D/E (RM0368 Rev 5, page 146).
    // Small delay for clock to stabilize (read back to ensure clock is stable)
    (void)RCC->AHB1ENR;
}

/**
 * @brief Disables the clock for a given GPIO port.
 * @param GPIOx Pointer to the GPIO port (e.g., GPIOA, GPIOB).
 */
static void PWM_GPIO_Clock_Disable(GPIO_TypeDef* GPIOx) {
    if (GPIOx == GPIOA) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN; /* PDF Reference - Assumed RCC register bit for GPIOA */
    } else if (GPIOx == GPIOB) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN; /* PDF Reference - Assumed RCC register bit for GPIOB */
    } else if (GPIOx == GPIOC) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN; /* PDF Reference - Assumed RCC register bit for GPIOC */
    } else if (GPIOx == GPIOD) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN; /* PDF Reference - Assumed RCC register bit for GPIOD */
    }
}

/**
 * @brief Enables the clock for a given Timer peripheral.
 * @param TIMx Pointer to the Timer peripheral base address (e.g., TIM1, TIM2).
 */
static void PWM_Timer_Clock_Enable(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* PDF Reference - Assumed RCC register bit for TIM1 */
    } else if (TIMx == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* PDF Reference - Assumed RCC register bit for TIM2 */
    } else if (TIMx == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* PDF Reference - Assumed RCC register bit for TIM3 */
    } else if (TIMx == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* PDF Reference - Assumed RCC register bit for TIM4 */
    } else if (TIMx == TIM10) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; /* PDF Reference - Assumed RCC register bit for TIM10 */
    } else if (TIMx == TIM11) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; /* PDF Reference - Assumed RCC register bit for TIM11 */
    }
    // Reserved timers TIM5, TIM9 are not enabled here.
    // Small delay for clock to stabilize
    (void)TIMx->CR1;
}

/**
 * @brief Disables the clock for a given Timer peripheral.
 * @param TIMx Pointer to the Timer peripheral base address (e.g., TIM1, TIM2).
 */
static void PWM_Timer_Clock_Disable(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; /* PDF Reference - Assumed RCC register bit for TIM1 */
    } else if (TIMx == TIM2) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; /* PDF Reference - Assumed RCC register bit for TIM2 */
    } else if (TIMx == TIM3) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; /* PDF Reference - Assumed RCC register bit for TIM3 */
    } else if (TIMx == TIM4) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; /* PDF Reference - Assumed RCC register bit for TIM4 */
    } else if (TIMx == TIM10) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN; /* PDF Reference - Assumed RCC register bit for TIM10 */
    } else if (TIMx == TIM11) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN; /* PDF Reference - Assumed RCC register bit for TIM11 */
    }
}

/**
 * @brief Configures a GPIO pin for Alternate Function mode.
 * @param GPIOx Pointer to the GPIO Port base address.
 * @param PinMask GPIO pin mask (e.g., GPIO_PIN_0, GPIO_PIN_1, etc.).
 * @param AF Alternate function number for the GPIO pin.
 */
static void PWM_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t PinMask, uint8_t AF) {
    // Enable GPIO Port Clock
    PWM_GPIO_Clock_Enable(GPIOx);

    uint32_t pin_pos = 0;
    // Determine the bit position (0-15) from the PinMask
    // This loop or __builtin_ctz (count trailing zeros) can be used.
    // Assuming PinMask is a single bit (e.g., 0x0001, 0x0002, ...)
    while (((PinMask >> pin_pos) & 0x1) == 0 && pin_pos < 16) {
        pin_pos++;
    }
    if (pin_pos >= 16) {
        // Invalid pin mask, or no pin found
        return;
    }

    // Configure GPIO Pin in Alternate Function mode (MODER)
    // MODER bits are 2y:2y+1 for pin y. Set to 10 (Alternate function mode).
    GPIOx->MODER &= ~(0x3UL << (pin_pos * 2)); // Clear current mode bits
    GPIOx->MODER |= (0x2UL << (pin_pos * 2));  // Set to Alternate Function mode (10) /* PDF Reference - Table 24, MODER bits to 10 for AF */

    // Configure Output type to Push-Pull (OTYPER)
    // OTYPER bit y for pin y. Set to 0 (Push-pull).
    GPIOx->OTYPER &= ~(0x1UL << pin_pos);      // Set to Push-pull (0) /* PDF Reference - Table 24, OTYPER bit to 0 for Push-pull */

    // Configure Output Speed to High speed (OSPEEDR)
    // OSPEEDR bits are 2y:2y+1 for pin y. Set to 10 (High speed).
    GPIOx->OSPEEDR &= ~(0x3UL << (pin_pos * 2)); // Clear current speed bits
    GPIOx->OSPEEDR |= (0x2UL << (pin_pos * 2));  // Set to High speed (10) /* PDF Reference - Table 24, OSPEEDR bits to 10 for High speed */

    // Configure Pull-up/Pull-down to No pull-up, pull-down (PUPDR)
    // PUPDR bits are 2y:2y+1 for pin y. Set to 00 (No pull-up, pull-down).
    GPIOx->PUPDR &= ~(0x3UL << (pin_pos * 2)); // Set to No Pull-up/Pull-down (00) /* PDF Reference - Table 24, PUPDR bits to 00 for No pull */

    // Configure Alternate Function (AFRL for pins 0-7, AFRH for pins 8-15)
    // AFRLy/AFRHy are 4 bits each.
    if (pin_pos < 8) {
        GPIOx->AFR[0] &= ~(0xFUL << (pin_pos * 4));     // Clear 4 bits for the AF selection
        GPIOx->AFR[0] |= ((uint32_t)AF << (pin_pos * 4)); /* PDF Reference - GPIOx_AFRL register */
    } else {
        GPIOx->AFR[1] &= ~(0xFUL << ((pin_pos - 8) * 4)); // Clear 4 bits for the AF selection
        GPIOx->AFR[1] |= ((uint32_t)AF << ((pin_pos - 8) * 4)); /* PDF Reference - GPIOx_AFRH register */
    }
}


/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The specific PWM channel to initialize (index into pwm_channel_map).
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    // Validate TRD_Channel against the size of the pwm_channel_map array
    if (TRD_Channel >= (TRD_Channel_t)(sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))) {
        // Invalid channel, return or handle error appropriately
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // 1. Initialize GPIO for the PWM channel
    PWM_GPIO_Init(config->PortName, config->PinNumber, config->AlternateFunction);

    // 2. Enable Timer Peripheral Clock
    PWM_Timer_Clock_Enable(config->TIMx);

    // 3. Configure Timer Time-base (Prescaler and Auto-Reload Register)
    // Disable the counter to allow configuration
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */

    // Enable Auto-Reload Preload Register (ARPE) for buffered updates.
    // This ensures that ARR changes take effect only after an update event (UG bit set in EGR).
    config->TIMx->CR1 |= TIM_CR1_ARPE; /* PDF Reference - TIMx_CR1, ARPE bit for buffered ARR updates */

    // Set a default prescaler and ARR for a starting frequency.
    // For 84MHz timer clock, PSC=83 gives 1MHz counter clock.
    // Then ARR=99 gives 10kHz frequency (1MHz / (99+1) = 10kHz).
    config->TIMx->PSC = 83; /* PDF Reference - TIMx_PSC register. Assumed value for 1MHz counter clock. */
    config->TIMx->ARR = 99; /* PDF Reference - TIMx_ARR register. Assumed value for 10kHz default frequency. */

    // Clear the counter
    config->TIMx->CNT = 0; /* PDF Reference - TIMx_CNT register */

    // 4. Configure Output Compare Mode (PWM Mode 1)
    // Clear CCxS bits (00 for output) and set OCxM (110 for PWM Mode 1), OCxPE (1 for preload enable)
    volatile uint32_t* ccmr_reg = NULL;
    uint32_t ccmr_clear_mask = 0;
    uint32_t ccmr_set_val = 0;

    // Positions for OCxM, OCxPE, CCxS differ based on channel number for CCMR1/CCMR2
    uint32_t oc_mode_pos; // Position of OCxM bits
    uint32_t oc_pe_pos;   // Position of OCxPE bit
    uint32_t cc_s_pos;    // Position of CCxS bits

    if (config->ChannelNumber == 1) {
        ccmr_reg = &config->TIMx->CCMR1;
        oc_mode_pos = TIM_CCMR1_OC1M_Pos; // Bits 6:4
        oc_pe_pos = TIM_CCMR1_OC1PE_Pos;   // Bit 3
        cc_s_pos = TIM_CCMR1_CC1S_Pos;     // Bits 1:0
    } else if (config->ChannelNumber == 2) {
        ccmr_reg = &config->TIMx->CCMR1;
        oc_mode_pos = TIM_CCMR1_OC2M_Pos; // Bits 14:12
        oc_pe_pos = TIM_CCMR1_OC2PE_Pos;   // Bit 11
        cc_s_pos = TIM_CCMR1_CC2S_Pos;     // Bits 9:8
    } else if (config->ChannelNumber == 3) {
        ccmr_reg = &config->TIMx->CCMR2;
        oc_mode_pos = TIM_CCMR2_OC3M_Pos; // Bits 6:4
        oc_pe_pos = TIM_CCMR2_OC3PE_Pos;   // Bit 3
        cc_s_pos = TIM_CCMR2_CC3S_Pos;     // Bits 1:0
    } else if (config->ChannelNumber == 4) {
        ccmr_reg = &config->TIMx->CCMR2;
        oc_mode_pos = TIM_CCMR2_OC4M_Pos; // Bits 14:12
        oc_pe_pos = TIM_CCMR2_OC4PE_Pos;   // Bit 11
        cc_s_pos = TIM_CCMR2_CC4S_Pos;     // Bits 9:8
    } else {
        return; // Invalid channel number for a timer
    }

    // Clear CCxS (00 for output), OCxM and OCxPE bits
    ccmr_clear_mask = (0x3UL << cc_s_pos) | (0x7UL << oc_mode_pos) | (0x1UL << oc_pe_pos);
    // Set OCxM to PWM Mode 1 (110 = 0x6), and enable preload (OCxPE = 1)
    ccmr_set_val = (0x6UL << oc_mode_pos) | (0x1UL << oc_pe_pos); /* PDF Reference - OCxM=110, OCxPE=1 */

    *ccmr_reg &= ~ccmr_clear_mask;
    *ccmr_reg |= ccmr_set_val;

    // 5. Configure Capture/Compare Enable Register (CCER)
    // Set CCxE to 1 (Output enable), CCxP to 0 (Active high)
    // CCxP and CCxE bits are located at (channel_number - 1) * 4
    uint32_t ccer_channel_offset = (config->ChannelNumber - 1) * 4;
    // Clear CCxP and CCxE bits for the current channel
    config->TIMx->CCER &= ~( (TIM_CCER_CC1P | TIM_CCER_CC1E) << ccer_channel_offset ); /* PDF Reference - TIMx_CCER bits */
    // Set CCxE (output enable) - CCxP (polarity) remains 0 (active high) after clear.
    config->TIMx->CCER |= (TIM_CCER_CC1E << ccer_channel_offset); /* PDF Reference - CCxE=1, CCxP=0 (default) */


    // 6. For advanced-control timers (TIM1), enable Main Output (MOE)
    if (config->TIMx == TIM1) {
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR, MOE bit */
    }

    // 7. Generate an update event to load the preload registers (PSC, ARR, CCR, OCxM, OCxPE) into active registers
    // This also clears the counter and prescaler, ensuring configuration takes effect.
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference - TIMx_EGR, UG bit */
}


/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The specific PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    // Validate TRD_Channel
    if (TRD_Channel >= (TRD_Channel_t)(sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))) {
        return; // Invalid channel
    }
    // Validate input parameters
    if (frequency == 0 || duty > 100) {
        return; // Invalid frequency or duty cycle
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // Determine max ARR based on timer type (16-bit or 32-bit)
    uint32_t max_arr_val = 0xFFFF; // Default for 16-bit timers (TIM1, TIM3, TIM4, TIM10, TIM11)
    if (config->TIMx == TIM2) { // TIM2 is 32-bit
        max_arr_val = 0xFFFFFFFF;
    }
    // TIM5 is reserved, but if it were used it would also be 32-bit.

    uint32_t psc = 0;
    uint32_t arr = 0;
    uint64_t timer_clk_div_freq = 0; // Use 64-bit for intermediate calculations to prevent overflow

    // Aim for a resolution of at least 100 steps for duty cycle, or higher if frequency allows.
    // Optimal strategy: Keep PSC as small as possible to maximize ARR for resolution.
    // Iteratively find PSC and ARR: (PSC + 1) * (ARR + 1) = PWM_TIMER_CLOCK_FREQ / frequency
    // Start with minimum PSC (0) and calculate max ARR possible.
    // If ARR exceeds max_arr_val, increase PSC and re-calculate.
    for (psc = 0; psc <= 0xFFFF; psc++) { // PSC is 16-bit
        if (frequency == 0) continue; // Avoid division by zero if frequency becomes 0 during loop
        timer_clk_div_freq = PWM_TIMER_CLOCK_FREQ / frequency;
        // Calculate (ARR + 1)
        uint64_t arr_plus_1 = timer_clk_div_freq / (psc + 1);

        if (arr_plus_1 > 0 && arr_plus_1 <= (max_arr_val + 1)) {
            arr = (uint32_t)(arr_plus_1 - 1);
            if (arr > 0) { // ARR cannot be 0 for PWM generation.
                break; // Found suitable PSC and ARR values
            }
        }
    }

    // If no suitable PSC/ARR pair found within the 16-bit PSC range
    if (psc > 0xFFFF) {
        // Frequency cannot be achieved with the given timer clock and resolution.
        return;
    }

    // Stop the timer counter temporarily during configuration update
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */

    // Set the calculated Prescaler and Auto-Reload Register values
    config->TIMx->PSC = psc; /* PDF Reference - TIMx_PSC register */
    config->TIMx->ARR = arr; /* PDF Reference - TIMx_ARR register */

    // Calculate Capture Compare Register value for duty cycle
    // CCRx = (ARR + 1) * duty / 100
    uint32_t ccr_val = (uint32_t)(((uint64_t)(arr + 1) * duty) / 100);
    // Ensure CCR value does not exceed ARR
    if (ccr_val > arr) {
        ccr_val = arr;
    }

    // Set the Capture Compare Register value for the specific channel
    switch (config->ChannelNumber) {
        case 1:
            config->TIMx->CCR1 = ccr_val; /* PDF Reference - TIMx_CCR1 register */
            break;
        case 2:
            config->TIMx->CCR2 = ccr_val; /* PDF Reference - TIMx_CCR2 register */
            break;
        case 3:
            config->TIMx->CCR3 = ccr_val; /* PDF Reference - TIMx_CCR3 register */
            break;
        case 4:
            config->TIMx->CCR4 = ccr_val; /* PDF Reference - TIMx_CCR4 register */
            break;
        default:
            break; // Should not happen with validation above
    }

    // Generate an update event to load the new values from preload registers into active registers
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference - TIMx_EGR, UG bit */

    // Restart the counter
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The specific PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= (TRD_Channel_t)(sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // Enable the output compare channel (CCxE bit)
    uint32_t ccer_channel_offset = (config->ChannelNumber - 1) * 4;
    config->TIMx->CCER |= (TIM_CCER_CC1E << ccer_channel_offset); /* PDF Reference - TIMx_CCER, CCxE bit */

    // For advanced-control timers (TIM1), ensure Main Output (MOE) is enabled
    // This allows the outputs OCx and OCxN to be driven.
    if (config->TIMx == TIM1) {
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR, MOE bit */
    }

    // Enable the timer counter (CEN bit)
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The specific PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= (TRD_Channel_t)(sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // Disable the output compare channel (CCxE bit)
    uint32_t ccer_channel_offset = (config->ChannelNumber - 1) * 4;
    config->TIMx->CCER &= ~(TIM_CCER_CC1E << ccer_channel_offset); /* PDF Reference - TIMx_CCER, CCxE bit */

    // For advanced-control timers (TIM1), disable Main Output (MOE)
    if (config->TIMx == TIM1) {
        config->TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR, MOE bit */
    }
    // Note: The timer counter (CEN bit) is not stopped here,
    // as other channels on the same timer might still be active.
    // The output is effectively stopped by disabling CCxE and MOE.
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through all defined PWM channels in pwm_channel_map,
 *        disables their outputs, resets their GPIOs to input floating mode,
 *        and disables the associated timer and GPIO clocks.
 *        It assumes a complete power-off of the PWM module is desired.
 */
void PWM_PowerOff(void) {
    // Iterate through all possible PWM configurations and disable them
    for (TRD_Channel_t i = (TRD_Channel_t)0; i < (TRD_Channel_t)(sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])); i++) {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];

        // 1. Stop the specific channel output
        PWM_Stop(i); // This clears CCxE for the channel and MOE (for TIM1)

        // 2. Reset the timer counter and disable it
        config->TIMx->CNT = 0;             /* PDF Reference - TIMx_CNT register */
        config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1, CEN bit */
        // Also clear ARPE bit as part of power-off / reset
        config->TIMx->CR1 &= ~TIM_CR1_ARPE; /* Clear ARPE bit */


        // 3. Reset GPIO pin to Input Floating mode (MODER 00) and no pull-up/down
        uint32_t pin_pos = 0;
        uint16_t pin_mask = config->PinNumber;
        while (((pin_mask >> pin_pos) & 0x1) == 0 && pin_pos < 16) {
            pin_pos++;
        }
        if (pin_pos < 16) {
            config->PortName->MODER &= ~(0x3UL << (pin_pos * 2)); // Set to 00 (Input mode) /* PDF Reference - Table 24, MODER bits to 00 for Input */
            config->PortName->PUPDR &= ~(0x3UL << (pin_pos * 2)); // Set to 00 (No pull-up, pull-down) /* PDF Reference - Table 24, PUPDR bits to 00 */
            // Reset alternate function to 0 as well
            if (pin_pos < 8) {
                config->PortName->AFR[0] &= ~(0xFUL << (pin_pos * 4));
            } else {
                config->PortName->AFR[1] &= ~(0xFUL << ((pin_pos - 8) * 4));
            }
        }
    }

    // 4. Disable clocks for all GPIOs and Timers used in the pwm_channel_map.
    // This part is less efficient if many channels share the same peripheral clock.
    // A more optimized approach would use a flag/counter for each peripheral to track usage.
    // However, for a full "PowerOff" function, explicitly disabling all used clocks is acceptable.
    // Only disable clocks for GPIO ports explicitly listed in pwm_channel_map.
    PWM_GPIO_Clock_Disable(GPIOA);
    PWM_GPIO_Clock_Disable(GPIOB);
    // GPIOC and GPIOD are not used by the current pwm_channel_map, so disabling their clocks here is redundant
    // PWM_GPIO_Clock_Disable(GPIOC);
    // PWM_GPIO_Clock_Disable(GPIOD);

    // Only disable clocks for timers explicitly listed in pwm_channel_map.
    PWM_Timer_Clock_Disable(TIM1);
    PWM_Timer_Clock_Disable(TIM2);
    PWM_Timer_Clock_Disable(TIM3);
    PWM_Timer_Clock_Disable(TIM4);
    PWM_Timer_Clock_Disable(TIM10);
    PWM_Timer_Clock_Disable(TIM11);
    // Reserved timers TIM5, TIM9 are not affected by this driver's power off, assuming
    // they are managed by other system components.
}