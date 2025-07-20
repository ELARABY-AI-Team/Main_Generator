/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "stm32f4xx.h"
#include "pwm.h" // Assumed to contain TRD_Channel_t definition

typedef uint32_t tlong;
typedef uint8_t tbyte;

// Peripheral Base Addresses (assumed clock frequencies for STM32F401RC)
// Assuming SYSCLK = 84MHz, APB1 Prescaler = /2 (PCLK1 = 42MHz), APB2 Prescaler = /1 (PCLK2 = 84MHz).
// For STM32F4, if APBx prescaler is > 1, the timer clock is 2 * PCLKx.
// If APBx prescaler is 1, the timer clock is PCLKx.
// So, TIM2,3,4,5 (APB1) get 2 * 42MHz = 84MHz.
// TIM1,9,10,11 (APB2) get 1 * 84MHz = 84MHz.
// All timers effectively run at 84MHz.
#define TIMER_CLOCK_FREQ 84000000UL 

// Define TIM Channel numbers (matching HAL defines for clarity and bit positions in CCER)
#define TIM_CHANNEL_1                    ((uint32_t)0x00000000U) // CCER Bit 0
#define TIM_CHANNEL_2                    ((uint32_t)0x00000004U) // CCER Bit 4
#define TIM_CHANNEL_3                    ((uint32_t)0x00000008U) // CCER Bit 8
#define TIM_CHANNEL_4                    ((uint32_t)0x0000000CU) // CCER Bit 12

// Define GPIO AF numbers for STM32F401RC (matching HAL defines)
#define GPIO_AF_TIM1                     ((uint8_t)0x01U) // AF1
#define GPIO_AF_TIM2                     ((uint8_t)0x01U) // AF1
#define GPIO_AF_TIM3                     ((uint8_t)0x02U) // AF2
#define GPIO_AF_TIM4                     ((uint8_t)0x02U) // AF2
#define GPIO_AF_TIM5                     ((uint8_t)0x02U) // AF2
#define GPIO_AF_TIM9                     ((uint8_t)0x03U) // AF3
#define GPIO_AF_TIM10                    ((uint8_t)0x03U) // AF3
#define GPIO_AF_TIM11                    ((uint8_t)0x03U) // AF3

// PWM_Channel_Config_t struct definition
typedef struct {
    TIM_TypeDef *TIMx;
    uint8_t AF_Number; // GPIO Alternate Function number
    uint32_t ChannelNumber; // TIM_CHANNEL_1, TIM_CHANNEL_2, etc. (for CCER bit offset)
    GPIO_TypeDef *PortName;
    uint16_t PinNumber;
} PWM_Channel_Config_t;

// Map of all valid PWM-capable timers and channels for STM32F401RC
// For pins with multiple AF options, one common option is chosen for demonstration.
static const PWM_Channel_Config_t pwm_channel_map[] = {
    // TIM1 - Advanced Control Timer (APB2) - 16-bit
    {TIM1, GPIO_AF_TIM1, TIM_CHANNEL_1, GPIOA, GPIO_PIN_8},
    {TIM1, GPIO_AF_TIM1, TIM_CHANNEL_2, GPIOA, GPIO_PIN_9},
    {TIM1, GPIO_AF_TIM1, TIM_CHANNEL_3, GPIOA, GPIO_PIN_10},
    {TIM1, GPIO_AF_TIM1, TIM_CHANNEL_4, GPIOA, GPIO_PIN_11},

    // TIM2 - General Purpose Timer (APB1) - 32-bit
    {TIM2, GPIO_AF_TIM2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_0},
    {TIM2, GPIO_AF_TIM2, TIM_CHANNEL_2, GPIOA, GPIO_PIN_1},
    {TIM2, GPIO_AF_TIM2, TIM_CHANNEL_3, GPIOA, GPIO_PIN_2},
    {TIM2, GPIO_AF_TIM2, TIM_CHANNEL_4, GPIOA, GPIO_PIN_3},

    // TIM3 - General Purpose Timer (APB1) - 16-bit
    {TIM3, GPIO_AF_TIM3, TIM_CHANNEL_1, GPIOA, GPIO_PIN_6},
    {TIM3, GPIO_AF_TIM3, TIM_CHANNEL_2, GPIOA, GPIO_PIN_7},
    {TIM3, GPIO_AF_TIM3, TIM_CHANNEL_3, GPIOB, GPIO_PIN_0},
    {TIM3, GPIO_AF_TIM3, TIM_CHANNEL_4, GPIOB, GPIO_PIN_1},

    // TIM4 - General Purpose Timer (APB1) - 16-bit
    {TIM4, GPIO_AF_TIM4, TIM_CHANNEL_1, GPIOB, GPIO_PIN_6},
    {TIM4, GPIO_AF_TIM4, TIM_CHANNEL_2, GPIOB, GPIO_PIN_7},
    {TIM4, GPIO_AF_TIM4, TIM_CHANNEL_3, GPIOB, GPIO_PIN_8},
    {TIM4, GPIO_AF_TIM4, TIM_CHANNEL_4, GPIOB, GPIO_PIN_9},

    // TIM5 - General Purpose Timer (APB1) - 32-bit
    {TIM5, GPIO_AF_TIM5, TIM_CHANNEL_1, GPIOA, GPIO_PIN_0}, // Shared with TIM2_CH1
    {TIM5, GPIO_AF_TIM5, TIM_CHANNEL_2, GPIOA, GPIO_PIN_1}, // Shared with TIM2_CH2
    {TIM5, GPIO_AF_TIM5, TIM_CHANNEL_3, GPIOA, GPIO_PIN_2}, // Shared with TIM2_CH3, TIM9_CH1
    {TIM5, GPIO_AF_TIM5, TIM_CHANNEL_4, GPIOA, GPIO_PIN_3}, // Shared with TIM2_CH4, TIM9_CH2

    // TIM9 - General Purpose Timer (APB2) - 16-bit
    {TIM9, GPIO_AF_TIM9, TIM_CHANNEL_1, GPIOA, GPIO_PIN_2}, // Shared with TIM2_CH3, TIM5_CH3
    {TIM9, GPIO_AF_TIM9, TIM_CHANNEL_2, GPIOA, GPIO_PIN_3}, // Shared with TIM2_CH4, TIM5_CH4

    // TIM10 - General Purpose Timer (APB2) - 16-bit
    {TIM10, GPIO_AF_TIM10, TIM_CHANNEL_1, GPIOB, GPIO_PIN_8}, // Shared with TIM4_CH3

    // TIM11 - General Purpose Timer (APB2) - 16-bit
    {TIM11, GPIO_AF_TIM11, TIM_CHANNEL_1, GPIOB, GPIO_PIN_9}  // Shared with TIM2_CH1/2, TIM4_CH4
};

// Helper function to get the timer channel configuration
static const PWM_Channel_Config_t* get_pwm_config(TRD_Channel_t trd_channel) {
    if (trd_channel >= (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))) {
        return (const PWM_Channel_Config_t*)0; // Invalid channel index
    }
    return &pwm_channel_map[trd_channel];
}

// Function to determine if a timer is 32-bit (TIM2, TIM5)
static uint8_t is_timer_32bit(TIM_TypeDef *TIMx) {
    return (TIMx == TIM2 || TIMx == TIM5);
}

// Internal function to set TIM base configuration
static void TIM_Base_SetConfig(TIM_TypeDef *TIMx, uint32_t Prescaler, uint32_t Period) {
    // Clear Counter Mode and Clock Division bits, then set to default values for PWM
    // Counter Mode: Up-counting (CMS=00, DIR=0)
    // Clock Division: No division (CKD=00)
    TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);
    // Auto-Reload Preload Enable (ARPE) should be set for PWM
    TIMx->CR1 |= TIM_CR1_ARPE;

    // Set the Autoreload value
    TIMx->ARR = Period;

    // Set the Prescaler value
    TIMx->PSC = Prescaler;

    // Generate an update event to reload the Prescaler and ARR value immediately
    TIMx->EGR = TIM_EGR_UG;
}

// Internal function to configure an output compare channel for PWM mode
static void TIM_ConfigurePWMChannelMode(TIM_TypeDef *TIMx, uint32_t ChannelNumber) {
    uint32_t tmpccmr;
    uint32_t tmpccer;
    uint32_t ccmr_shift_offset; // 0 for CH1/3, 8 for CH2/4
    uint32_t ccer_shift_offset; // 0 for CH1, 4 for CH2, 8 for CH3, 12 for CH4

    if (ChannelNumber == TIM_CHANNEL_1) {
        tmpccmr = TIMx->CCMR1;
        tmpccer = TIMx->CCER;
        ccmr_shift_offset = 0;
        ccer_shift_offset = 0;
    } else if (ChannelNumber == TIM_CHANNEL_2) {
        tmpccmr = TIMx->CCMR1;
        tmpccer = TIMx->CCER;
        ccmr_shift_offset = 8;
        ccer_shift_offset = 4;
    } else if (ChannelNumber == TIM_CHANNEL_3) {
        tmpccmr = TIMx->CCMR2;
        tmpccer = TIMx->CCER;
        ccmr_shift_offset = 0;
        ccer_shift_offset = 8;
    } else { // TIM_CHANNEL_4
        tmpccmr = TIMx->CCMR2;
        tmpccer = TIMx->CCER;
        ccmr_shift_offset = 8;
        ccer_shift_offset = 12;
    }

    // Disable the Channel: Clear the CCxE Bit in CCER before configuration
    TIMx->CCER &= ~(1UL << ccer_shift_offset); // CC1E=bit0, CC2E=bit4, CC3E=bit8, CC4E=bit12

    // Clear output compare mode bits and capture/compare selection bits
    // OCxM[2:0] (Bits 6:4 for CH1, 14:12 for CH2, etc.) for PWM Mode 1 (0b110)
    // CCxS[1:0] (Bits 1:0 for CH1, 9:8 for CH2, etc.) for Output (0b00)
    tmpccmr &= ~((0x7UL << (4 + ccmr_shift_offset)) | (0x3UL << ccmr_shift_offset));
    
    // Set PWM Mode 1 (0x6UL) and configure as output (0x0UL for CCxS)
    tmpccmr |= (0x6UL << (4 + ccmr_shift_offset)); // Set OCxM to PWM Mode 1

    // Set the Preload enable bit (OCxPE)
    tmpccmr |= (1UL << (3 + ccmr_shift_offset)); // OCxPE bit is bit 3 within the channel's 8-bit block

    // Configure the Output Fast mode (OCxFE - disabled, 0)
    tmpccmr &= ~(1UL << (2 + ccmr_shift_offset)); // OCxFE bit is bit 2 within the channel's 8-bit block

    if (ChannelNumber == TIM_CHANNEL_1 || ChannelNumber == TIM_CHANNEL_2) {
        TIMx->CCMR1 = tmpccmr;
    } else {
        TIMx->CCMR2 = tmpccmr;
    }

    // Reset the Output Polarity level and set active high (CCxP = 0)
    tmpccer &= ~(1UL << (1 + ccer_shift_offset)); // CCxP bit (0 = active high)
    TIMx->CCER = tmpccer; // Apply polarity settings
}


void PWM_Init(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (!config) {
        return; // Invalid channel
    }

    // 1. Enable GPIO Clock
    if (config->PortName == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (config->PortName == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (config->PortName == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (config->PortName == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else if (config->PortName == GPIOE) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    }

    // 2. Configure GPIO Pin for Alternate Function (AF)
    uint32_t pin_mode_shift = (config->PinNumber % 16) * 2;
    uint32_t pin_af_register_idx = config->PinNumber / 8; // AFR[0] for pins 0-7, AFR[1] for pins 8-15
    uint32_t pin_af_shift = (config->PinNumber % 8) * 4;

    // Set pin mode to Alternate Function (10b)
    config->PortName->MODER &= ~(0x3UL << pin_mode_shift); // Clear mode bits
    config->PortName->MODER |= (0x2UL << pin_mode_shift);  // Set to Alternate Function mode

    // Set output type to Push-Pull (0b)
    config->PortName->OTYPER &= ~(0x1UL << config->PinNumber); // Clear OTYPER bit (0 = Push-pull)

    // Set output speed to High (10b)
    config->PortName->OSPEEDR &= ~(0x3UL << pin_mode_shift); // Clear speed bits
    config->PortName->OSPEEDR |= (0x2UL << pin_mode_shift);  // Set to High speed

    // Set Pull-up/Pull-down to No Pull-up/Pull-down (00b)
    config->PortName->PUPDR &= ~(0x3UL << pin_mode_shift); // Clear PUPDR bits

    // Set Alternate Function mapping
    if (pin_af_register_idx == 0) { // AFR[0] for Px0-Px7
        config->PortName->AFR[0] &= ~(0xFUL << pin_af_shift); // Clear AF bits
        config->PortName->AFR[0] |= (config->AF_Number << pin_af_shift); // Set AF
    } else { // AFR[1] for Px8-Px15
        config->PortName->AFR[1] &= ~(0xFUL << pin_af_shift); // Clear AF bits
        config->PortName->AFR[1] |= (config->AF_Number << pin_af_shift); // Set AF
    }

    // 3. Enable Timer Clock
    if (config->TIMx == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    } else if (config->TIMx == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    } else if (config->TIMx == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    } else if (config->TIMx == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    } else if (config->TIMx == TIM5) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    } else if (config->TIMx == TIM9) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    } else if (config->TIMx == TIM10) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    } else if (config->TIMx == TIM11) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    }

    // 4. Configure Time Base for a default frequency (e.g., 10kHz with 84MHz clock)
    uint32_t default_freq = 10000; // Hz
    uint32_t prescaler = 0;
    uint32_t period = 0;
    uint32_t period_max = is_timer_32bit(config->TIMx) ? 0xFFFFFFFFUL : 0xFFFFUL;

    // Calculate optimal prescaler and period
    for (prescaler = 0; prescaler <= 0xFFFF; prescaler++) {
        uint32_t calculated_period = TIMER_CLOCK_FREQ / (default_freq * (prescaler + 1));
        if (calculated_period > 0) { // Ensure period is at least 1
            if ((calculated_period - 1) <= period_max) {
                period = calculated_period - 1;
                break;
            }
        }
    }
    // Fallback if loop doesn't find a perfect fit (e.g., too high freq for max period)
    if (period == 0) { // This means (calculated_period - 1) was often negative or zero.
        prescaler = TIMER_CLOCK_FREQ / default_freq / period_max;
        period = period_max;
        if (prescaler == 0) prescaler = 1; // Minimum prescaler is 1 (value 0)
        period = (TIMER_CLOCK_FREQ / (default_freq * (prescaler + 1))) - 1;
        if (period == 0) period = 1;
    }

    TIM_Base_SetConfig(config->TIMx, prescaler, period);

    // 5. Configure PWM Output Compare Mode for the channel
    TIM_ConfigurePWMChannelMode(config->TIMx, config->ChannelNumber);

    // Set initial duty cycle to 0%
    PWM_Set_Freq(TRD_Channel, default_freq, 0); 

    // For advanced timers (TIM1), enable the Main Output (MOE bit)
    if (config->TIMx == TIM1) {
        config->TIMx->BDTR |= TIM_BDTR_MOE;
    }
}

void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (!config || frequency == 0) {
        return; // Invalid channel or frequency
    }

    uint32_t period_max = is_timer_32bit(config->TIMx) ? 0xFFFFFFFFUL : 0xFFFFUL;
    uint32_t prescaler = 0;
    uint32_t period = 0;

    // Find appropriate prescaler and period
    for (prescaler = 0; prescaler <= 0xFFFF; prescaler++) { // Prescaler is 16-bit value
        uint32_t calculated_period = TIMER_CLOCK_FREQ / (frequency * (prescaler + 1));
        if (calculated_period > 0) {
            if ((calculated_period - 1) <= period_max) {
                period = calculated_period - 1;
                break;
            }
        }
    }
    // Fallback if loop doesn't find a perfect fit (e.g., too high freq for max period)
    if (period == 0) {
        prescaler = TIMER_CLOCK_FREQ / frequency / period_max;
        period = period_max;
        if (prescaler == 0) prescaler = 1;
        period = (TIMER_CLOCK_FREQ / (frequency * (prescaler + 1))) - 1;
        if (period == 0) period = 1;
    }

    // Update PSC and ARR registers
    config->TIMx->PSC = prescaler;
    config->TIMx->ARR = period;

    // Calculate Capture Compare Register value for duty cycle
    uint32_t pulse = (period + 1) * duty / 100;

    // Set CCR value based on channel
    if (config->ChannelNumber == TIM_CHANNEL_1) {
        config->TIMx->CCR1 = pulse;
    } else if (config->ChannelNumber == TIM_CHANNEL_2) {
        config->TIMx->CCR2 = pulse;
    } else if (config->ChannelNumber == TIM_CHANNEL_3) {
        config->TIMx->CCR3 = pulse;
    } else { // TIM_CHANNEL_4
        config->TIMx->CCR4 = pulse;
    }

    // Generate an update event to apply changes immediately
    config->TIMx->EGR = TIM_EGR_UG;
}

void PWM_Start(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (!config) {
        return; // Invalid channel
    }

    // Enable the Capture compare channel output (CCxE bit in CCER)
    // TIM_CHANNEL_1 corresponds to CCER bit 0 (CC1E)
    // TIM_CHANNEL_2 corresponds to CCER bit 4 (CC2E)
    // TIM_CHANNEL_3 corresponds to CCER bit 8 (CC3E)
    // TIM_CHANNEL_4 corresponds to CCER bit 12 (CC4E)
    config->TIMx->CCER |= (1UL << (config->ChannelNumber));

    // Enable the Counter
    config->TIMx->CR1 |= TIM_CR1_CEN;
}

void PWM_Stop(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (!config) {
        return; // Invalid channel
    }

    // Disable the Capture compare channel output (CCxE bit in CCER)
    config->TIMx->CCER &= ~(1UL << (config->ChannelNumber));

    // Disable the Counter (only if no other channels on same timer are active)
    config->TIMx->CR1 &= ~TIM_CR1_CEN;
}

void PWM_PowerOff(void) {
    // Iterate through all possible channels and stop them
    for (int i = 0; i < (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])); i++) {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];

        // Disable the Capture compare channel output
        config->TIMx->CCER &= ~(1UL << (config->ChannelNumber));

        // Disable the Counter
        config->TIMx->CR1 &= ~TIM_CR1_CEN;
        
        // For advanced timers (TIM1), disable the Main Output (MOE bit)
        if (config->TIMx == TIM1) {
            config->TIMx->BDTR &= ~TIM_BDTR_MOE;
        }

        // Reset GPIO pin to input floating state
        uint32_t pin_mode_shift = (config->PinNumber % 16) * 2;
        config->PortName->MODER &= ~(0x3UL << pin_mode_shift); // Clear mode bits (00b = Input)
        config->PortName->PUPDR &= ~(0x3UL << pin_mode_shift); // No pull-up/pull-down
    }

    // Disable all associated timer clocks
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN);
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN);

    // Disable all associated GPIO clocks (potentially over-disabling if other peripherals use them)
    RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN);
}