/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready implementation for PWM driver targeting STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/* Include necessary headers */
#include "STM32F401RC_PWM.h"
#include "stm32f4xx.h" // Provides definitions for STM32F4xx peripherals
#include <stdint.h>    // Provides standard integer types (uint32_t, uint8_t etc)

/* Define custom types if not already defined in STM32F401RC_PWM.h */
#ifndef T_BYTE_DEF
#define T_BYTE_DEF
typedef uint8_t tbyte;
#endif

#ifndef T_LONG_DEF
#define T_LONG_DEF
typedef uint32_t tlong;
#endif

/*
 * @brief   Enum defining the available PWM channels.
 *          This enum is designed to map to the configuration array below.
 *          The enum values are expected to be contiguous starting from 0.
 *          Note: TIM2 and TIM3 channels are intentionally omitted from this driver
 *          implementation to reserve them for other potential uses like
 *          OS timekeeping, complex timing, or other non-PWM features,
 *          as per the requirement to reserve timers if the MCU lacks dedicated ones.
 *          TIM6 and TIM7 are basic timers and cannot generate PWM outputs.
 */
typedef enum
{
    PWM_CHANNEL_TIM1_CH1 = 0,
    PWM_CHANNEL_TIM1_CH2,
    PWM_CHANNEL_TIM1_CH3,
    PWM_CHANNEL_TIM1_CH4,
    PWM_CHANNEL_TIM4_CH1,
    PWM_CHANNEL_TIM4_CH2,
    PWM_CHANNEL_TIM4_CH3,
    PWM_CHANNEL_TIM4_CH4,
    PWM_CHANNEL_TIM5_CH1,
    PWM_CHANNEL_TIM5_CH2,
    PWM_CHANNEL_TIM5_CH3,
    PWM_CHANNEL_TIM5_CH4,
    // Add more channels if needed from non-reserved timers (TIM1, TIM4, TIM5)

    PWM_CHANNEL_COUNT // Total number of defined channels
} t_pwm_channel;

/* Static configuration structure for each PWM channel */
typedef struct
{
    TIM_TypeDef*      TIMx;       /* Pointer to the Timer peripheral */
    GPIO_TypeDef*     GPIOx;      /* Pointer to the GPIO port */
    uint32_t          GPIO_Pin;   /* GPIO Pin number (e.g., GPIO_Pin_0) */
    uint32_t          GPIO_AF;    /* GPIO Alternate Function selection */
    uint32_t          TIM_Channel;/* Timer Channel number (TIM_Channel_1 to TIM_Channel_4) */
    uint32_t          RCC_APBENR; /* RCC APB peripheral clock enable register (APB1ENR or APB2ENR) */
    uint32_t          RCC_APBENR_TIM_Mask; /* Mask for enabling the timer clock in RCC_APBENR */
    uint32_t          RCC_AHB1ENR_GPIO_Mask; /* Mask for enabling the GPIO clock in RCC_AHB1ENR */
} pwm_config_t;


/*
 * @brief   Configuration array for the defined PWM channels.
 *          Maps t_pwm_channel enum value to specific hardware registers and settings.
 *          This array must be kept in sync with the t_pwm_channel enum.
 */
static const pwm_config_t pwm_configs[PWM_CHANNEL_COUNT] =
{
    /* TIM1 Channels (APB2 Timer) */
    { TIM1, GPIOA, GPIO_PIN_8,  GPIO_AF_TIM1,  TIM_CHANNEL_1, RCC->APB2ENR, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN }, // TIM1_CH1 on PA8
    { TIM1, GPIOA, GPIO_PIN_9,  GPIO_AF_TIM1,  TIM_CHANNEL_2, RCC->APB2ENR, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN }, // TIM1_CH2 on PA9
    { TIM1, GPIOA, GPIO_PIN_10, GPIO_AF_TIM1,  TIM_CHANNEL_3, RCC->APB2ENR, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN }, // TIM1_CH3 on PA10
    { TIM1, GPIOA, GPIO_PIN_11, GPIO_AF_TIM1,  TIM_CHANNEL_4, RCC->APB2ENR, RCC_APB2ENR_TIM1EN,  RCC_AHB1ENR_GPIOAEN }, // TIM1_CH4 on PA11

    /* TIM4 Channels (APB1 Timer) */
    { TIM4, GPIOB, GPIO_PIN_6,  GPIO_AF_TIM4,  TIM_CHANNEL_1, RCC->APB1ENR, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN }, // TIM4_CH1 on PB6
    { TIM4, GPIOB, GPIO_PIN_7,  GPIO_AF_TIM4,  TIM_CHANNEL_2, RCC->APB1ENR, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN }, // TIM4_CH2 on PB7
    { TIM4, GPIOB, GPIO_PIN_8,  GPIO_AF_TIM4,  TIM_CHANNEL_3, RCC->APB1ENR, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN }, // TIM4_CH3 on PB8
    { TIM4, GPIOB, GPIO_PIN_9,  GPIO_AF_TIM4,  TIM_CHANNEL_4, RCC->APB1ENR, RCC_APB1ENR_TIM4EN,  RCC_AHB1ENR_GPIOBEN }, // TIM4_CH4 on PB9

    /* TIM5 Channels (APB1 Timer - 32-bit counter) */
    { TIM5, GPIOA, GPIO_PIN_0,  GPIO_AF_TIM5,  TIM_CHANNEL_1, RCC->APB1ENR, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN }, // TIM5_CH1 on PA0
    { TIM5, GPIOA, GPIO_PIN_1,  GPIO_AF_TIM5,  TIM_CHANNEL_2, RCC->APB1ENR, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN }, // TIM5_CH2 on PA1
    { TIM5, GPIOA, GPIO_PIN_2,  GPIO_AF_TIM5,  TIM_CHANNEL_3, RCC->APB1ENR, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN }, // TIM5_CH3 on PA2
    { TIM5, GPIOA, GPIO_PIN_3,  GPIO_AF_TIM5,  TIM_CHANNEL_4, RCC->APB1ENR, RCC_APB1ENR_TIM5EN,  RCC_AHB1ENR_GPIOAEN }, // TIM5_CH4 on PA3

    // Add configurations for additional channels here
};

/*
 * @brief   Helper function to get the timer clock frequency.
 *          Depends on the APB prescaler configuration.
 * @param   TIMx: Pointer to the Timer peripheral.
 * @return  Timer clock frequency in Hz. Returns 0 if TIMx is invalid.
 */
static tlong get_timer_clock_freq(TIM_TypeDef* TIMx)
{
    tlong timer_clock = 0;
    tlong apb_prescaler = 0;

    // Determine if timer is on APB1 or APB2
    if (TIMx == TIM2 || TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5 || TIMx == TIM6 || TIMx == TIM7)
    {
        // Timer is on APB1
        apb_prescaler = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07; // Read PPRE1 bits
        timer_clock = SystemCoreClock / (1 << (apb_prescaler & 0x04 ? (apb_prescaler & 0x03) + 1 : 0)); // Calculate APB1 clock
        if (apb_prescaler > 4) // If APB prescaler is > 1, timer clock is multiplied by 2
        {
            timer_clock *= 2;
        }
    }
    else if (TIMx == TIM1)
    {
        // Timer is on APB2
        apb_prescaler = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x07; // Read PPRE2 bits
        timer_clock = SystemCoreClock / (1 << (apb_prescaler & 0x04 ? (apb_prescaler & 0x03) + 1 : 0)); // Calculate APB2 clock
        if (apb_prescaler > 4) // If APB prescaler is > 1, timer clock is multiplied by 2
        {
            timer_clock *= 2;
        }
    }
    // Note: TIM6 and TIM7 are on APB1 but cannot do PWM output.

    return timer_clock;
}


/**Functions ===========================================================================*/

/**
 * @brief   Initializes the PWM hardware for a specific channel.
 *          Configures the timer and GPIOs.
 * @param   TRD_Channel: The PWM channel to initialize (type t_pwm_channel).
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    // Ensure the channel index is within the valid range
    if (TRD_Channel >= (TRD_Channel_t)PWM_CHANNEL_COUNT)
    {
        // Invalid channel, do nothing or handle error
        return;
    }

    const pwm_config_t* config = &pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    GPIO_TypeDef* GPIOx = config->GPIOx;
    uint32_t pin = config->GPIO_Pin;
    uint32_t channel = config->TIM_Channel;

    /* 1. Enable GPIO clock */
    // Check which AHB1ENR bit corresponds to the GPIO port
    if (GPIOx == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (GPIOx == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (GPIOx == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (GPIOx == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (GPIOx == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    else if (GPIOx == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;


    /* 2. Configure GPIO pin for Alternate Function (AF) */
    // Clear MODER bits for the pin and set to Alternate Function (10)
    GPIOx->MODER &= ~(GPIO_MODER_MODE0 << (pin * 2));
    GPIOx->MODER |= (GPIO_MODER_MODE0_1 << (pin * 2)); // Set bit 1

    // Clear OTYPER bit for the pin and set to Push-Pull (0)
    GPIOx->OTYPER &= ~(GPIO_OTYPER_OT0 << pin); // Already reset value, explicit for clarity

    // Clear OSPEEDR bits for the pin and set to High Speed (11)
    GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (pin * 2));
    GPIOx->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0_0 | GPIO_OSPEEDR_OSPEED0_1) << (pin * 2); // Set bits 0 and 1

    // Clear PUPDR bits for the pin and set to No Pull-up/Pull-down (00)
    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (pin * 2)); // Already reset value, explicit for clarity

    // Set the Alternate Function (AFR) register
    // For pin 0-7 use AFR[0], for pin 8-15 use AFR[1]
    uint32_t pin_index = pin; // Assuming GPIO_PIN_x is 0 to 15
    if (pin_index < 8)
    {
        GPIOx->AFR[0] &= ~(0xF << (pin_index * 4)); // Clear AF bits
        GPIOx->AFR[0] |= (config->GPIO_AF << (pin_index * 4)); // Set AF bits
    }
    else
    {
        pin_index -= 8;
        GPIOx->AFR[1] &= ~(0xF << (pin_index * 4)); // Clear AF bits
        GPIOx->AFR[1] |= (config->GPIO_AF << (pin_index * 4)); // Set AF bits
    }

    /* 3. Enable Timer clock */
    // Use masks from the config structure for flexibility
    if (config->RCC_APBENR == RCC->APB1ENR)
    {
        RCC->APB1ENR |= config->RCC_APBENR_TIM_Mask;
    }
    else if (config->RCC_APBENR == RCC->APB2ENR)
    {
        RCC->APB2ENR |= config->RCC_APBENR_TIM_Mask;
    }

    /* 4. Configure Timer for PWM mode */

    // Disable the timer counter before configuration
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set the timer to Upcounting mode
    TIMx->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR); // Clear Center-aligned mode and Direction (sets to Upcounting)

    // Enable the Auto-Reload Preload Enable (ARPE) bit
    TIMx->CR1 |= TIM_CR1_ARPE;

    // Configure the Output Compare mode for the selected channel
    // Need to select the correct CCMR register (CCMR1 or CCMR2) based on channel number
    volatile uint32_t* ccmr_reg = (channel <= TIM_CHANNEL_2) ? &TIMx->CCMR1 : &TIMx->CCMR2;
    uint32_t ccmr_shift = (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_3) ? 0 : 8; // Shift for CCxR bits

    // Clear existing CCxS and OCxM bits for the channel
    *ccmr_reg &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE); // Use CCMR1 masks, applies to CCMR2 with appropriate shift
    *ccmr_reg &= ~(TIM_CCMR1_OC1M_Msk << ccmr_shift); // Clear OCxM bits
    *ccmr_reg &= ~(TIM_CCMR1_CC1S_Msk << ccmr_shift); // Clear CCxS bits (ensure output mode)

    // Configure the channel in PWM Mode 1 (OCxM = 110)
    // Set OCxM[2:0] to 110 for PWM Mode 1
    *ccmr_reg |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) << ccmr_shift;

    // Enable Output Compare Preload enable (OCxPE) for the channel
    *ccmr_reg |= (TIM_CCMR1_OC1PE << ccmr_shift);

    // Configure Output Compare Fast enable (OCxFE) if needed (optional, good for high frequencies)
    // *ccmr_reg |= (TIM_CCMR1_OC1FE << ccmr_shift);

    // Set Output Polarity (default is high) (CCER register)
    // TIMx->CCER &= ~(TIM_CCER_CC1P << ((channel - 1) * 4)); // Clear polarity bit (default high)

    // Generate an update event to load all configurations (except CR1 bits)
    TIMx->EGR |= TIM_EGR_UG;
    // Clear the Update interrupt flag (optional, but good practice)
    TIMx->SR &= ~TIM_SR_UIF;

    // For TIM1 (advanced timer), enable the Main Output Enable (MOE) bit in BDTR
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Timer remains disabled after init. Start with PWM_Start().
}

/**
 * @brief   Sets the desired PWM frequency and duty cycle for the selected channel.
 *          Calculates and updates the timer's ARR, PSC, and CCR registers.
 * @param   TRD_Channel: The PWM channel (type t_pwm_channel).
 * @param   frequency: The desired frequency in Hz.
 * @param   duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    // Ensure channel is valid
    if (TRD_Channel >= (TRD_Channel_t)PWM_CHANNEL_COUNT)
    {
        // Invalid channel
        return;
    }

    const pwm_config_t* config = &pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint32_t channel = config->TIM_Channel;

    // Get the timer clock frequency
    tlong timer_clock = get_timer_clock_freq(TIMx);

    // Handle invalid frequency requests
    if (frequency == 0 || timer_clock == 0)
    {
        // Cannot generate PWM with 0 frequency or 0 timer clock.
        // Stop the channel output.
        PWM_Stop(TRD_Channel);
        return;
    }

    // Clamp duty cycle to 0-100
    if (duty > 100)
    {
        duty = 100;
    }

    /*
     * Calculate PSC and ARR values
     * Timer Frequency (f_TIM) = Timer_Clock
     * PWM Frequency (f_PWM) = f_TIM / ((PSC + 1) * (ARR + 1))
     * Target (PSC + 1) * (ARR + 1) = f_TIM / f_PWM
     * To maximize resolution, we want to maximize ARR.
     * Max ARR for 16-bit timers (TIM1, TIM4): 65535 (0xFFFF)
     * Max ARR for 32-bit timers (TIM5): 4294967295 (0xFFFFFFFF)
     */

    uint32_t auto_reload = 0;
    uint32_t prescaler = 0;
    uint64_t target_arr_psc; // Use 64-bit for calculation to avoid overflow

    target_arr_psc = (uint64_t)timer_clock / frequency;

    if (TIMx == TIM5)
    {
        // 32-bit timer (TIM5)
        // Try to achieve the target frequency with minimal prescaler (PSC=0) first
        if (target_arr_psc <= 0xFFFFFFFFUL) // Check if ARR fits in 32 bits with PSC=0
        {
             prescaler = 0;
             auto_reload = (uint32_t)target_arr_psc - 1;
        }
        else
        {
            // If target > max ARR, calculate PSC
            // We need (PSC + 1) * (ARR + 1) = target_arr_psc
            // To maximize ARR, we can set ARR to the max possible value for 32-bit: 0xFFFFFFFF
            // (PSC + 1) * (0xFFFFFFFF + 1) = target_arr_psc
            // PSC + 1 = target_arr_psc / (0xFFFFFFFF + 1)  <- This division is problematic as 0xFFFFFFFF+1 wraps around
            // Let's find PSC + 1 such that (target_arr_psc / (PSC + 1)) - 1 is the largest possible ARR <= 0xFFFFFFFF
            // Simpler approach: Iterate PSC from 0 upwards until (target_arr_psc / (PSC + 1)) fits into ARR (32-bit).
            // Or, calculate minimal PSC: (target_arr_psc / 0xFFFFFFFF)
            prescaler = (uint32_t)(target_arr_psc / 0x100000000ULL); // Calculate minimal prescaler (approx)
            if (prescaler > 0xFFFF) prescaler = 0xFFFF; // PSC is only 16-bit

            // Recalculate ARR based on the chosen prescaler
            if (prescaler == 0xFFFF) // Avoid division by 0x10000
            {
                 auto_reload = (uint32_t)(target_arr_psc / 0x10000ULL) - 1;
            }
            else
            {
                 auto_reload = (uint32_t)(target_arr_psc / (prescaler + 1)) - 1;
            }

            // Ensure ARR doesn't exceed 32-bit max, though calculation should prevent this
            if (auto_reload > 0xFFFFFFFFUL) auto_reload = 0xFFFFFFFFUL;
        }

        // Ensure minimal ARR
        if (auto_reload < 1) auto_reload = 1;

    }
    else // 16-bit timers (TIM1, TIM4)
    {
        // Max ARR for 16-bit timers is 0xFFFF
        // Try to achieve the target frequency with maximal ARR (0xFFFF) first
        if (target_arr_psc > 0xFFFF)
        {
            // Calculate PSC to bring the frequency down
            // (PSC + 1) = target_arr_psc / (0xFFFF + 1)
            prescaler = (uint32_t)(target_arr_psc / 0x10000ULL);
            if (prescaler > 0xFFFF) prescaler = 0xFFFF; // PSC is 16-bit

            // Recalculate ARR based on the chosen prescaler
            if (prescaler == 0xFFFF) // Avoid division by 0x10000
            {
                auto_reload = (uint32_t)(target_arr_psc / 0x10000ULL) - 1;
            }
            else
            {
                 auto_reload = (uint32_t)(target_arr_psc / (prescaler + 1)) - 1;
            }

            // Ensure ARR fits in 16 bits
             if (auto_reload > 0xFFFF) auto_reload = 0xFFFF;

        }
        else
        {
            // target_arr_psc fits within 16 bits with PSC=0
            prescaler = 0;
            auto_reload = (uint32_t)target_arr_psc - 1;
        }

        // Ensure minimal ARR
        if (auto_reload < 1) auto_reload = 1;

         // Ensure prescaler fits in 16 bits
        if (prescaler > 0xFFFF) prescaler = 0xFFFF;
    }


    /* 5. Calculate Compare value (CCR) based on duty cycle */
    // CCR = (duty / 100.0) * (ARR + 1)
    // Using integer arithmetic: CCR = (duty * (ARR + 1)) / 100
    uint32_t compare_value = (uint32_t)(((uint64_t)duty * (auto_reload + 1)) / 100);

    // Ensure compare value does not exceed ARR + 1 (which is effectively 100%)
    if (compare_value > auto_reload + 1)
    {
        compare_value = auto_reload + 1; // Max duty cycle
    }


    /* 6. Update Timer registers */
    // Disable the timer counter briefly or rely on update event
    // TIMx->CR1 &= ~TIM_CR1_CEN; // Optional, safer to update while running if using preload

    TIMx->PSC = prescaler;
    TIMx->ARR = auto_reload;

    // Update the specific channel's CCR register
    switch (channel)
    {
        case TIM_CHANNEL_1: TIMx->CCR1 = compare_value; break;
        case TIM_CHANNEL_2: TIMx->CCR2 = compare_value; break;
        case TIM_CHANNEL_3: TIMx->CCR3 = compare_value; break;
        case TIM_CHANNEL_4: TIMx->CCR4 = compare_value; break;
        default: break; // Should not happen with valid channel
    }

    // Generate an update event to load new PSC, ARR, and CCR values into active registers
    TIMx->EGR |= TIM_EGR_UG;
    // Wait for the update flag or clear it immediately if not waiting
    // TIMx->SR &= ~TIM_SR_UIF; // Clear the Update interrupt flag

    // Re-enable the timer counter if it was stopped (handled by PWM_Start/Stop functions)
    // TIMx->CR1 |= TIM_CR1_CEN; // Optional
}

/**
 * @brief   Enables and starts PWM signal generation on the specified channel.
 * @param   TRD_Channel: The PWM channel (type t_pwm_channel).
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    // Ensure channel is valid
    if (TRD_Channel >= (TRD_Channel_t)PWM_CHANNEL_COUNT)
    {
        // Invalid channel
        return;
    }

    const pwm_config_t* config = &pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint32_t channel = config->TIM_Channel;

    // Enable the Capture/Compare output for the specific channel
    // The CCxE bit is at different positions based on the channel (1, 2, 3, 4)
    // Position is (channel - 1) * 4 bits
    TIMx->CCER |= (TIM_CCER_CC1E << ((channel - 1) * 4));

    // For TIM1 (advanced timer), ensure the Main Output Enable (MOE) bit is set
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Enable the timer counter
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   Stops the PWM signal output on the specified channel.
 *          The timer itself might keep running if other channels are active.
 * @param   TRD_Channel: The PWM channel (type t_pwm_channel).
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     // Ensure channel is valid
    if (TRD_Channel >= (TRD_Channel_t)PWM_CHANNEL_COUNT)
    {
        // Invalid channel
        return;
    }

    const pwm_config_t* config = &pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint32_t channel = config->TIM_Channel;

    // Disable the Capture/Compare output for the specific channel
    TIMx->CCER &= ~(TIM_CCER_CC1E << ((channel - 1) * 4));

    // Note: We do NOT disable the timer counter (TIMx->CR1_CEN) here,
    // as other channels on the same timer might still be active.
    // The counter will only be disabled if all channels using this timer are stopped
    // OR via the PWM_PowerOff function.

    // For TIM1, check if any other channel is active before clearing MOE.
    // This check requires keeping track of active channels per timer, which adds complexity.
    // A simpler approach for 'Stop' is just disabling the individual channel output (CCxE).
    // MOE is typically managed by the first channel starting and the last channel stopping on TIM1.
}

/**
 * @brief   Disables all PWM peripherals and outputs controlled by this driver
 *          to reduce power consumption. Resets relevant GPIOs.
 */
void PWM_PowerOff(void)
{
    // Iterate through all defined channels to disable outputs and reset GPIOs
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++)
    {
        const pwm_config_t* config = &pwm_configs[i];
        TIM_TypeDef* TIMx = config->TIMx;
        GPIO_TypeDef* GPIOx = config->GPIOx;
        uint32_t pin = config->GPIO_Pin;
        uint32_t channel = config->TIM_Channel;

        // Disable the Capture/Compare output for the channel
        TIMx->CCER &= ~(TIM_CCER_CC1E << ((channel - 1) * 4));

        // Reset GPIO pin to Analog Input (low power consumption)
        // Clear MODER bits for the pin and set to Analog (11)
        GPIOx->MODER &= ~(GPIO_MODER_MODE0 << (pin * 2));
        GPIOx->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1) << (pin * 2); // Set bits 0 and 1

        // Ensure no pull-up/pull-down
        GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (pin * 2));
    }

    // Disable all timer clocks used by this driver
    // Check if any channel on TIM1, TIM4, TIM5 was initialized/used before disabling its clock?
    // A simpler approach is to just disable the clocks for the timers included in the config array.
    // This ensures all timers potentially used by this driver are powered off.
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; // Disable TIM1 clock
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; // Disable TIM4 clock
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN; // Disable TIM5 clock

    // Also disable TIM1 Main Output Enable if TIM1 is used
    // This should happen if TIM1EN is disabled, but explicit clear is safer.
    // Only needed if TIM1 was actually used.
    // A check is more robust, but for a full power off, just clear it if TIM1EN was disabled.
    // If TIM1EN is already 0, writing to BDTR might be harmless but it's better to only touch registers when clock is ON.
    // Re-initializing TIM1 after power-off would require re-setting MOE in BDTR during init/start.
}

/* End of File *******************************************************************************************************/