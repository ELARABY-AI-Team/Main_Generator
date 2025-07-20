/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM bare-metal implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "stm32f401xe.h" // Standard Peripheral Header for STM32F401RC
// The pwm.h file is assumed to be provided by the user and contains TRD_Channel_t.
// For compilation purposes, a local definition is provided here.
// In a real project, this would be removed and pwm.h included.

#ifndef TRD_CHANNEL_T_DEFINED
#define TRD_CHANNEL_T_DEFINED
typedef enum
{
    TRD_CHANNEL_PWM_0 = 0, // Maps to TIM3_CH1, PA6
    TRD_CHANNEL_PWM_1,     // Maps to TIM3_CH2, PA7
    TRD_CHANNEL_PWM_2,     // Maps to TIM3_CH3, PB0
    TRD_CHANNEL_PWM_3,     // Maps to TIM3_CH4, PB1
    TRD_CHANNEL_MAX
} TRD_Channel_t;

typedef uint32_t tlong;
typedef uint8_t tbyte;
#endif // TRD_CHANNEL_T_DEFINED

/**
 * @brief Structure to define configuration for a PWM channel.
 */
typedef struct {
    TIM_TypeDef *TIMx;         ///< Pointer to the TIM peripheral instance (e.g., TIM3)
    uint32_t ChannelRegisterOffset; ///< Register offset for the specific channel's CCER bit and CCMRx bits.
                                ///< (e.g., 0x00 for CH1, 0x04 for CH2, 0x08 for CH3, 0x0C for CH4 based on CCER shifts)
    GPIO_TypeDef *PortName;    ///< Pointer to the GPIO Port (e.g., GPIOA, GPIOB)
    uint16_t PinNumber;        ///< GPIO Pin number (e.g., GPIO_PIN_6 for PA6)
    uint8_t AFNumber;          ///< Alternate Function number (0-15) for the GPIO pin
    uint32_t RCC_APB_ENR_BIT;  ///< RCC APB Bus Enable Register bit for the Timer (e.g., RCC_APB1ENR_TIM3EN)
    uint32_t RCC_AHB_ENR_BIT;  ///< RCC AHB Bus Enable Register bit for the GPIO Port (e.g., RCC_AHB1ENR_GPIOAEN)
} PWM_Channel_Config_t;

// --- GPIO Register Masks and Values ---
#define GPIO_MODE_INPUT             0x00U // Input Mode
#define GPIO_MODE_OUTPUT            0x01U // General Purpose Output Mode
#define GPIO_MODE_AF                0x02U // Alternate Function Mode
#define GPIO_MODE_ANALOG            0x03U // Analog Mode

#define GPIO_SPEED_LOW              0x00U
#define GPIO_SPEED_MEDIUM           0x01U
#define GPIO_SPEED_HIGH             0x02U
#define GPIO_SPEED_VERY_HIGH        0x03U

#define GPIO_PUPD_NO_PUPD           0x00U
#define GPIO_PUPD_PU                0x01U
#define GPIO_PUPD_PD                0x02U

// --- TIM Register Masks and Values (for TIM3 specific) ---
// TIMx_CR1 (Control Register 1)
#define TIM_CR1_CEN_Pos             (0U)
#define TIM_CR1_CEN_Msk             (0x1UL << TIM_CR1_CEN_Pos)     // Counter enable
#define TIM_CR1_DIR_Pos             (4U)
#define TIM_CR1_DIR_Msk             (0x1UL << TIM_CR1_DIR_Pos)     // Direction (0=Up, 1=Down)
#define TIM_CR1_CMS_Pos             (5U)
#define TIM_CR1_CMS_Msk             (0x3UL << TIM_CR1_CMS_Pos)     // Center-aligned mode selection
#define TIM_CR1_ARPE_Pos            (7U)
#define TIM_CR1_ARPE_Msk            (0x1UL << TIM_CR1_ARPE_Pos)    // Auto-reload preload enable

// TIMx_CCMR1 (Capture/Compare Mode Register 1) - for Channels 1 & 2
#define TIM_CCMR1_OC1M_Pos          (4U)
#define TIM_CCMR1_OC1M_Msk          (0x7UL << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_PWM1         (0x6UL << TIM_CCMR1_OC1M_Pos)  // PWM mode 1
#define TIM_CCMR1_OC1PE_Pos         (3U)
#define TIM_CCMR1_OC1PE_Msk         (0x1UL << TIM_CCMR1_OC1PE_Pos) // Output compare 1 preload enable

#define TIM_CCMR1_OC2M_Pos          (12U)
#define TIM_CCMR1_OC2M_Msk          (0x7UL << TIM_CCMR1_OC2M_Pos)
#define TIM_CCMR1_OC2M_PWM1         (0x6UL << TIM_CCMR1_OC2M_Pos)
#define TIM_CCMR1_OC2PE_Pos         (11U)
#define TIM_CCMR1_OC2PE_Msk         (0x1UL << TIM_CCMR1_OC2PE_Pos)

// TIMx_CCMR2 (Capture/Compare Mode Register 2) - for Channels 3 & 4
#define TIM_CCMR2_OC3M_Pos          (4U)
#define TIM_CCMR2_OC3M_Msk          (0x7UL << TIM_CCMR2_OC3M_Pos)
#define TIM_CCMR2_OC3M_PWM1         (0x6UL << TIM_CCMR2_OC3M_Pos)
#define TIM_CCMR2_OC3PE_Pos         (3U)
#define TIM_CCMR2_OC3PE_Msk         (0x1UL << TIM_CCMR2_OC3PE_Pos)

#define TIM_CCMR2_OC4M_Pos          (12U)
#define TIM_CCMR2_OC4M_Msk          (0x7UL << TIM_CCMR2_OC4M_Pos)
#define TIM_CCMR2_OC4M_PWM1         (0x6UL << TIM_CCMR2_OC4M_Pos)
#define TIM_CCMR2_OC4PE_Pos         (11U)
#define TIM_CCMR2_OC4PE_Msk         (0x1UL << TIM_CCMR2_OC4PE_Pos)

// TIMx_CCER (Capture/Compare Enable Register)
#define TIM_CCER_CC1E_Pos           (0U)
#define TIM_CCER_CC1E_Msk           (0x1UL << TIM_CCER_CC1E_Pos)   // Capture/Compare 1 output enable
#define TIM_CCER_CC1P_Pos           (1U)
#define TIM_CCER_CC1P_Msk           (0x1UL << TIM_CCER_CC1P_Pos)   // Output Polarity (0=High, 1=Low)

#define TIM_CCER_CC2E_Pos           (4U)
#define TIM_CCER_CC2E_Msk           (0x1UL << TIM_CCER_CC2E_Pos)
#define TIM_CCER_CC2P_Pos           (5U)
#define TIM_CCER_CC2P_Msk           (0x1UL << TIM_CCER_CC2P_Pos)

#define TIM_CCER_CC3E_Pos           (8U)
#define TIM_CCER_CC3E_Msk           (0x1UL << TIM_CCER_CC3E_Pos)
#define TIM_CCER_CC3P_Pos           (9U)
#define TIM_CCER_CC3P_Msk           (0x1UL << TIM_CCER_CC3P_Pos)

#define TIM_CCER_CC4E_Pos           (12U)
#define TIM_CCER_CC4E_Msk           (0x1UL << TIM_CCER_CC4E_Pos)
#define TIM_CCER_CC4P_Pos           (13U)
#define TIM_CCER_CC4P_Msk           (0x1UL << TIM_CCER_CC4P_Pos)

// TIMx_EGR (Event Generation Register)
#define TIM_EGR_UG_Pos              (0U)
#define TIM_EGR_UG_Msk              (0x1UL << TIM_EGR_UG_Pos)      // Update generation

// --- RCC Peripheral Enable Register Bits ---
// RCC_AHB1ENR (AHB1 Peripheral Clock Enable Register)
#define RCC_AHB1ENR_GPIOAEN_Pos     (0U)
#define RCC_AHB1ENR_GPIOAEN_Msk     (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOBEN_Pos     (1U)
#define RCC_AHB1ENR_GPIOBEN_Msk     (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos)

// RCC_APB1ENR (APB1 Peripheral Clock Enable Register)
#define RCC_APB1ENR_TIM3EN_Pos      (1U)
#define RCC_APB1ENR_TIM3EN_Msk      (0x1UL << RCC_APB1ENR_TIM3EN_Pos)

// --- System Core Clock Frequency ---
// Assuming HCLK = 84MHz for STM32F401RC, and APB1 prescaler = 2.
// For Timers on APB1 (like TIM3), if APB1 prescaler > 1, the timer clock is 2x APB1 clock.
// PCLK1 = 84MHz / 2 = 42MHz. So, TIM3_CLK = 2 * 42MHz = 84MHz.
#define APB1_TIMER_CLOCK_HZ         (84000000UL)

/**
 * @brief Array mapping logical TRD_Channel_t to specific PWM hardware configurations.
 */
static const PWM_Channel_Config_t pwm_channel_map[TRD_CHANNEL_MAX] = {
    // TRD_CHANNEL_PWM_0: TIM3_CH1, PA6, AF2
    {TIM3, TIM_CCER_CC1E_Pos, GPIOA, GPIO_PIN_6, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk},
    // TRD_CHANNEL_PWM_1: TIM3_CH2, PA7, AF2
    {TIM3, TIM_CCER_CC2E_Pos, GPIOA, GPIO_PIN_7, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk},
    // TRD_CHANNEL_PWM_2: TIM3_CH3, PB0, AF2
    {TIM3, TIM_CCER_CC3E_Pos, GPIOB, GPIO_PIN_0, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk},
    // TRD_CHANNEL_PWM_3: TIM3_CH4, PB1, AF2
    {TIM3, TIM_CCER_CC4E_Pos, GPIOB, GPIO_PIN_1, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk}
};

/**
 * @brief Initializes a specific PWM channel with default settings.
 *        This function enables clocks, configures GPIO for Alternate Function,
 *        and sets up the Timer peripheral for PWM generation.
 * @param TRD_Channel The logical PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_MAX)
    {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    
    // 1. Enable GPIO Clock
    if (config->RCC_AHB_ENR_BIT == RCC_AHB1ENR_GPIOAEN_Msk)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;
    }
    else if (config->RCC_AHB_ENR_BIT == RCC_AHB1ENR_GPIOBEN_Msk)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN_Msk;
    }
    // Add more GPIO port enables if other ports are used

    // 2. Configure GPIO Pin for Alternate Function (AF)
    uint32_t pin_idx = 0;
    while (!((1U << pin_idx) & config->PinNumber) && (pin_idx < 16)) {
        pin_idx++;
    }

    // Set pin mode to Alternate Function
    config->PortName->MODER &= ~(0x3UL << (pin_idx * 2U));
    config->PortName->MODER |= (GPIO_MODE_AF << (pin_idx * 2U));

    // Set Alternate Function (AFR[0] for pins 0-7, AFR[1] for pins 8-15)
    if (pin_idx < 8)
    {
        config->PortName->AFR[0] &= ~(0xFUL << (pin_idx * 4U));
        config->PortName->AFR[0] |= ((uint32_t)config->AFNumber << (pin_idx * 4U));
    }
    else
    {
        config->PortName->AFR[1] &= ~(0xFUL << ((pin_idx - 8U) * 4U));
        config->PortName->AFR[1] |= ((uint32_t)config->AFNumber << ((pin_idx - 8U) * 4U));
    }

    // Set Output Type to Push-Pull (default, 0x0)
    config->PortName->OTYPER &= ~(0x1UL << pin_idx);

    // Set Output Speed to Very High
    config->PortName->OSPEEDR &= ~(0x3UL << (pin_idx * 2U));
    config->PortName->OSPEEDR |= (GPIO_SPEED_VERY_HIGH << (pin_idx * 2U));

    // Set Pull-up/Pull-down to No PUPD (default, 0x0)
    config->PortName->PUPDR &= ~(0x3UL << (pin_idx * 2U));


    // 3. Enable Timer Clock
    RCC->APB1ENR |= config->RCC_APB_ENR_BIT;

    // 4. Configure Timer Base (TIM3)
    // Disable the timer counter during configuration
    config->TIMx->CR1 &= ~TIM_CR1_CEN_Msk;

    // Set Counter Mode to Up-counting and enable Auto-Reload Preload
    // Clear CMS (Center-aligned mode) and DIR (Direction) bits for Up-counting
    config->TIMx->CR1 &= ~(TIM_CR1_CMS_Msk | TIM_CR1_DIR_Msk);
    config->TIMx->CR1 |= TIM_CR1_ARPE_Msk; // Enable auto-reload preload

    // Default Prescaler and Period. Actual frequency set by PWM_Set_Freq.
    // Set a default prescaler and period for a 1MHz counter clock to start.
    config->TIMx->PSC = (APB1_TIMER_CLOCK_HZ / 1000000UL) - 1; // Counter clock = 1MHz
    config->TIMx->ARR = 1000 - 1; // Default period of 1ms (1kHz frequency)

    // 5. Configure PWM Channel
    uint32_t ccmr_val;
    if (config->ChannelRegisterOffset == TIM_CCER_CC1E_Pos) // Channel 1
    {
        ccmr_val = config->TIMx->CCMR1;
        ccmr_val &= ~TIM_CCMR1_OC1M_Msk;       // Clear Output Compare Mode bits
        ccmr_val |= TIM_CCMR1_OC1M_PWM1;       // Set PWM Mode 1
        ccmr_val |= TIM_CCMR1_OC1PE_Msk;       // Enable Output Compare Preload
        config->TIMx->CCMR1 = ccmr_val;

        config->TIMx->CCR1 = config->TIMx->ARR / 2; // Default 50% duty cycle
    }
    else if (config->ChannelRegisterOffset == TIM_CCER_CC2E_Pos) // Channel 2
    {
        ccmr_val = config->TIMx->CCMR1;
        ccmr_val &= ~TIM_CCMR1_OC2M_Msk;
        ccmr_val |= TIM_CCMR1_OC2M_PWM1;
        ccmr_val |= TIM_CCMR1_OC2PE_Msk;
        config->TIMx->CCMR1 = ccmr_val;

        config->TIMx->CCR2 = config->TIMx->ARR / 2;
    }
    else if (config->ChannelRegisterOffset == TIM_CCER_CC3E_Pos) // Channel 3
    {
        ccmr_val = config->TIMx->CCMR2;
        ccmr_val &= ~TIM_CCMR2_OC3M_Msk;
        ccmr_val |= TIM_CCMR2_OC3M_PWM1;
        ccmr_val |= TIM_CCMR2_OC3PE_Msk;
        config->TIMx->CCMR2 = ccmr_val;

        config->TIMx->CCR3 = config->TIMx->ARR / 2;
    }
    else if (config->ChannelRegisterOffset == TIM_CCER_CC4E_Pos) // Channel 4
    {
        ccmr_val = config->TIMx->CCMR2;
        ccmr_val &= ~TIM_CCMR2_OC4M_Msk;
        ccmr_val |= TIM_CCMR2_OC4M_PWM1;
        ccmr_val |= TIM_CCMR2_OC4PE_Msk;
        config->TIMx->CCMR2 = ccmr_val;

        config->TIMx->CCR4 = config->TIMx->ARR / 2;
    }
    // No TIM_BDTR_MOE for general purpose timers like TIM3

    // Generate an update event to apply all configurations
    config->TIMx->EGR |= TIM_EGR_UG_Msk;
}

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 * @param TRD_Channel The logical PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= TRD_CHANNEL_MAX || frequency == 0)
    {
        return; // Invalid channel or frequency
    }

    if (duty > 100) duty = 100;

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    tlong prescaler = 0;
    tlong period = 0;

    // Calculate optimal Prescaler and Period
    // Target_ticks = APB1_TIMER_CLOCK_HZ / frequency
    // (PSC + 1) * (ARR + 1) = Target_ticks
    // Since ARR and PSC are 16-bit registers (max value 65535), we need to find values that fit.

    // Calculate maximum possible period value
    tlong max_period = 0xFFFF; // 16-bit timer

    // Calculate the total ticks for one PWM period
    tlong target_ticks = APB1_TIMER_CLOCK_HZ / frequency;

    if (target_ticks == 0) // Frequency is too high for the timer clock
    {
        period = 0; // Smallest possible period
        prescaler = 0;
    }
    else if (target_ticks <= (max_period + 1)) // Period fits with prescaler = 0
    {
        period = target_ticks - 1;
        prescaler = 0;
    }
    else // Need to increase prescaler
    {
        prescaler = (target_ticks / (max_period + 1));
        if (target_ticks % (max_period + 1) != 0) {
            prescaler++; // Increment prescaler if there's a remainder to ensure period fits
        }
        prescaler--; // Subtract 1 for register value

        if (prescaler > 0xFFFF) { // Prescaler too large
            prescaler = 0xFFFF;
        }

        period = (APB1_TIMER_CLOCK_HZ / (frequency * (prescaler + 1))) - 1;
        if (period > max_period) { // Fallback for very low frequencies
            period = max_period;
        }
    }
    
    // Ensure calculated values fit within 16-bit registers
    if (prescaler > 0xFFFF) prescaler = 0xFFFF;
    if (period > 0xFFFF) period = 0xFFFF;

    // Set Prescaler and Period (ARR)
    config->TIMx->PSC = (uint16_t)prescaler;
    config->TIMx->ARR = (uint16_t)period;

    // Calculate Capture Compare Register (CCR) value for duty cycle
    tlong pulse = (period + 1) * duty / 100;
    if (pulse > period + 1) pulse = period + 1; // Cap at max period value
    if (duty == 0) pulse = 0;
    if (duty == 100) pulse = period + 1;

    // Update CCR register based on channel
    if (config->ChannelRegisterOffset == TIM_CCER_CC1E_Pos)
    {
        config->TIMx->CCR1 = (uint16_t)pulse;
    }
    else if (config->ChannelRegisterOffset == TIM_CCER_CC2E_Pos)
    {
        config->TIMx->CCR2 = (uint16_t)pulse;
    }
    else if (config->ChannelRegisterOffset == TIM_CCER_CC3E_Pos)
    {
        config->TIMx->CCR3 = (uint16_t)pulse;
    }
    else if (config->ChannelRegisterOffset == TIM_CCER_CC4E_Pos)
    {
        config->TIMx->CCR4 = (uint16_t)pulse;
    }

    // Generate an update event to immediately apply new PSC, ARR, and CCR values
    config->TIMx->EGR |= TIM_EGR_UG_Msk;
}

/**
 * @brief Starts the PWM signal generation for a specific channel.
 * @param TRD_Channel The logical PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_MAX)
    {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Enable the Capture/Compare output for the selected channel
    config->TIMx->CCER |= (0x1UL << config->ChannelRegisterOffset);

    // Enable the Timer Counter
    config->TIMx->CR1 |= TIM_CR1_CEN_Msk;
}

/**
 * @brief Stops the PWM signal generation for a specific channel.
 * @param TRD_Channel The logical PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_MAX)
    {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Disable the Capture/Compare output for the selected channel
    config->TIMx->CCER &= ~(0x1UL << config->ChannelRegisterOffset);

    // Disable the Timer Counter
    config->TIMx->CR1 &= ~TIM_CR1_CEN_Msk;
}

/**
 * @brief Powers off all configured PWM channels.
 *        Disables timer counters, channel outputs, GPIO alternate functions,
 *        and peripheral clocks for all associated hardware.
 */
void PWM_PowerOff(void)
{
    for (TRD_Channel_t i = (TRD_Channel_t)0; i < TRD_CHANNEL_MAX; i++)
    {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];

        // 1. Disable Capture/Compare output for the channel
        config->TIMx->CCER &= ~(0x1UL << config->ChannelRegisterOffset);

        // 2. Disable Timer Counter
        config->TIMx->CR1 &= ~TIM_CR1_CEN_Msk;
        
        // 3. Reset GPIO Pin mode to Input (default power-on state for many applications)
        uint32_t pin_idx = 0;
        while (!((1U << pin_idx) & config->PinNumber) && (pin_idx < 16)) {
            pin_idx++;
        }
        config->PortName->MODER &= ~(0x3UL << (pin_idx * 2U)); // Clear mode bits
        config->PortName->MODER |= (GPIO_MODE_INPUT << (pin_idx * 2U)); // Set to Input mode

        // Reset Alternate Function (AFR[0] for pins 0-7, AFR[1] for pins 8-15)
        if (pin_idx < 8)
        {
            config->PortName->AFR[0] &= ~(0xFUL << (pin_idx * 4U)); // Clear AF bits
        }
        else
        {
            config->PortName->AFR[1] &= ~(0xFUL << ((pin_idx - 8U) * 4U)); // Clear AF bits
        }
        
        // Reset Output Type, Speed, Pull-up/Pull-down to default (optional, Input mode generally ignores these)
        config->PortName->OTYPER &= ~(0x1UL << pin_idx);
        config->PortName->OSPEEDR &= ~(0x3UL << (pin_idx * 2U));
        config->PortName->PUPDR &= ~(0x3UL << (pin_idx * 2U));
    }

    // 4. Disable Peripheral Clocks (GPIO and TIM)
    // Assuming all configured channels use TIM3 and GPIOA/GPIOB.
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN_Msk;
    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN_Msk;
    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN_Msk;
}