/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Bare-metal PWM implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "stm32f401xe.h" // Device header for STM32F401RC register definitions
#include "pwm.h"        // Contains TRD_Channel_t as per user requirement
#include <stdint.h>     // For standard integer types

// Custom type definitions as per requirement
typedef uint32_t tlong;
typedef uint8_t tbyte;

// PWM Channel Configuration Structure
typedef struct {
    TIM_TypeDef *TIMx;      // Pointer to the Timer peripheral (e.g., TIM2, TIM3)
    uint32_t ChannelNumber; // TIM_CHANNEL_1, TIM_CHANNEL_2, etc. (defined below)
    GPIO_TypeDef *PortName; // Pointer to the GPIO port (e.g., GPIOA, GPIOB)
    uint16_t PinNumber;     // The pin number (0-15) on the specified GPIO port
    uint8_t AFNumber;       // The Alternate Function number for the pin
} PWM_Channel_Config_t;

// Constants and Macros
// SystemCoreClock is assumed to be configured by the system startup code (e.g., in SystemInit()).
// For STM32F401RC, with HCLK at 84MHz, APB1 at 42MHz (prescaler /2), and APB2 at 84MHz (prescaler /1),
// all Timers (on APB1 and APB2) will receive a clock of 84MHz due to timer clock multiplication (2x PCLK).
#define TIMER_CLOCK_FREQ 84000000UL

// Standard TIM Channel Defines (these are bit offsets/masks commonly used for configuration)
// They represent the starting bit position of the channel's control bits in various registers (CCER, CCMRx).
#define TIM_CHANNEL_1 ((uint32_t)0x00000000U) // Corresponds to CCER bits 0 (CC1E), 1 (CC1P)
#define TIM_CHANNEL_2 ((uint32_t)0x00000004U) // Corresponds to CCER bits 4 (CC2E), 5 (CC2P)
#define TIM_CHANNEL_3 ((uint32_t)0x00000008U) // Corresponds to CCER bits 8 (CC3E), 9 (CC3P)
#define TIM_CHANNEL_4 ((uint32_t)0x0000000CU) // Corresponds to CCER bits 12 (CC4E), 13 (CC4P)

// PWM Mode 1 configuration for OCxM bits (0b110)
#define TIM_OCMODE_PWM1 ((uint32_t)0x00000060U) // Value for OCxM[2:0] = 110 (PWM mode 1)

// Output Polarity (CCxP bit) - Active High
#define TIM_OCPOLARITY_HIGH ((uint32_t)0x00000000U) // CCxP = 0 (Active high)

// GPIO Mode and Speed definitions
#define GPIO_MODER_ALTERNATE_FUNCTION ((uint32_t)0x00000002U) // 0b10 for Alternate Function Mode
#define GPIO_MODER_ANALOG_MODE        ((uint32_t)0x00000003U) // 0b11 for Analog Mode
#define GPIO_OSPEEDR_HIGH             ((uint32_t)0x00000003U) // 0b11 for High Speed
#define GPIO_PUPDR_NOPULL             ((uint32_t)0x00000000U) // 0b00 for No Pull-up/Pull-down

// Alternate Function (AF) numbers for specific timers on STM32F401RC
#define GPIO_AF1_TIM2   ((uint8_t)0x01)
#define GPIO_AF2_TIM3   ((uint8_t)0x02)
#define GPIO_AF2_TIM4   ((uint8_t)0x02)
#define GPIO_AF3_TIM10  ((uint8_t)0x03) // TIM10_CH1 is typically on PB8

// Array of all valid PWM-capable timers and channels for STM32F401RC
// This maps the logical TRD_Channel_t to specific physical hardware configurations.
// Ensure TRD_CHANNEL_COUNT matches the number of entries here.
static const PWM_Channel_Config_t pwm_channel_map[TRD_CHANNEL_COUNT] = {
    [TRD_CHANNEL_PWM1] = {
        .TIMx = TIM2, .ChannelNumber = TIM_CHANNEL_1, .PortName = GPIOA, .PinNumber = 0, .AFNumber = GPIO_AF1_TIM2
    },
    [TRD_CHANNEL_PWM2] = {
        .TIMx = TIM3, .ChannelNumber = TIM_CHANNEL_1, .PortName = GPIOA, .PinNumber = 6, .AFNumber = GPIO_AF2_TIM3
    },
    [TRD_CHANNEL_PWM3] = {
        .TIMx = TIM4, .ChannelNumber = TIM_CHANNEL_1, .PortName = GPIOB, .PinNumber = 6, .AFNumber = GPIO_AF2_TIM4
    },
    [TRD_CHANNEL_PWM4] = {
        .TIMx = TIM10, .ChannelNumber = TIM_CHANNEL_1, .PortName = GPIOB, .PinNumber = 8, .AFNumber = GPIO_AF3_TIM10
    }
};

// Internal function prototypes
static void RCC_Enable_GPIO_Clock(GPIO_TypeDef *GPIOx);
static void RCC_Enable_TIM_Clock(TIM_TypeDef *TIMx);
static void RCC_Disable_GPIO_Clock(GPIO_TypeDef *GPIOx);
static void RCC_Disable_TIM_Clock(TIM_TypeDef *TIMx);
static void GPIO_Config_AF(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t AF);
static void TIM_Base_SetConfig(TIM_TypeDef *TIMx, uint32_t Prescaler, uint32_t Period);
static void TIM_OC_SetConfig(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t OCMode, uint32_t OCPolarity);
static void TIM_Set_Pulse(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t Pulse);

/**
 * @brief Enables the clock for the given GPIO port.
 * @param GPIOx Pointer to the GPIO port register structure (e.g., GPIOA, GPIOB).
 */
static void RCC_Enable_GPIO_Clock(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    else if (GPIOx == GPIOB)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    }
    // Add other GPIO ports here if used by pwm_channel_map entries
    // else if (GPIOx == GPIOC) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; }
    // ...
}

/**
 * @brief Disables the clock for the given GPIO port.
 * @param GPIOx Pointer to the GPIO port register structure (e.g., GPIOA, GPIOB).
 */
static void RCC_Disable_GPIO_Clock(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
    }
    else if (GPIOx == GPIOB)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
    }
    // Add other GPIO ports here if used by pwm_channel_map entries
    // else if (GPIOx == GPIOC) { RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN; }
    // ...
}

/**
 * @brief Enables the clock for the given TIM peripheral.
 * @param TIMx Pointer to the TIM peripheral register structure (e.g., TIM2, TIM3).
 */
static void RCC_Enable_TIM_Clock(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }
    else if (TIMx == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    }
    else if (TIMx == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    else if (TIMx == TIM5)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }
    else if (TIMx == TIM1) // TIM1 is on APB2
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if (TIMx == TIM9) // TIM9 is on APB2
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    }
    else if (TIMx == TIM10) // TIM10 is on APB2
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    }
    else if (TIMx == TIM11) // TIM11 is on APB2
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    }
}

/**
 * @brief Disables the clock for the given TIM peripheral.
 * @param TIMx Pointer to the TIM peripheral register structure (e.g., TIM2, TIM3).
 */
static void RCC_Disable_TIM_Clock(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM2)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
    }
    else if (TIMx == TIM3)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
    }
    else if (TIMx == TIM4)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
    }
    else if (TIMx == TIM5)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
    }
    else if (TIMx == TIM1)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
    }
    else if (TIMx == TIM9)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;
    }
    else if (TIMx == TIM10)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
    }
    else if (TIMx == TIM11)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;
    }
}

/**
 * @brief Configures a GPIO pin for Alternate Function mode.
 * @param GPIOx Pointer to the GPIO port register structure.
 * @param Pin The pin number (0-15).
 * @param AF The Alternate Function number.
 */
static void GPIO_Config_AF(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t AF)
{
    uint32_t pin_mode_shift = Pin * 2;
    uint32_t afr_shift = (Pin % 8) * 4;

    // Clear and set MODER bits for Alternate Function mode (0b10)
    GPIOx->MODER &= ~(GPIO_MODER_MODER0 << pin_mode_shift); // Clear current mode bits
    GPIOx->MODER |= (GPIO_MODER_ALTERNATE_FUNCTION << pin_mode_shift); // Set AF mode

    // Set output speed to High (0b11)
    GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0 << pin_mode_shift); // Clear current speed bits
    GPIOx->OSPEEDR |= (GPIO_OSPEEDR_HIGH << pin_mode_shift);      // Set High Speed

    // Set No Pull-up/Pull-down (0b00)
    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << pin_mode_shift); // Clear current pull bits
    GPIOx->PUPDR |= (GPIO_PUPDR_NOPULL << pin_mode_shift);  // Set No Pull-up/Pull-down

    // Configure Alternate Function Register (AFRL for pins 0-7, AFRH for pins 8-15)
    if (Pin < 8)
    {
        GPIOx->AFR[0] &= ~((uint32_t)0xF << afr_shift); // Clear AF bits
        GPIOx->AFR[0] |= ((uint32_t)AF << afr_shift);   // Set AF
    }
    else
    {
        GPIOx->AFR[1] &= ~((uint32_t)0xF << afr_shift); // Clear AF bits
        GPIOx->AFR[1] |= ((uint32_t)AF << afr_shift);   // Set AF
    }
}

/**
 * @brief Configures the TIM base unit (Prescaler and Period/ARR).
 * @param TIMx Pointer to the TIM peripheral register structure.
 * @param Prescaler The prescaler value (0-65535).
 * @param Period The auto-reload value (0 to MAX_REG_VALUE for 16-bit or 32-bit timers).
 */
static void TIM_Base_SetConfig(TIM_TypeDef *TIMx, uint32_t Prescaler, uint32_t Period)
{
    // Disable counter before configuration to ensure settings apply cleanly
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set the Prescaler value
    TIMx->PSC = (uint16_t)Prescaler; // PSC is always 16-bit

    // Set the Auto-Reload Register (ARR) value
    // Note: TIM2 and TIM5 are 32-bit timers, others are 16-bit on F401RC.
    // The `Period` parameter is `uint32_t`, which correctly handles both.
    TIMx->ARR = Period;

    // Configure TIMx->CR1 register:
    // - CMS (Center-aligned mode selection): 00 (Edge-aligned mode)
    // - DIR (Direction): 0 (Upcounting)
    // - CKD (Clock Division): 00 (t_DTS = t_CK_INT)
    // - ARPE (Auto-reload preload enable): 1 (ARR register is buffered)
    // - URS (Update Request Source): 1 (Only counter overflow/underflow generates an update interrupt)
    TIMx->CR1 = (TIM_CR1_ARPE | TIM_CR1_URS); // All other bits are 0 by default for basic operation.
}

/**
 * @brief Configures a TIM Output Compare channel for PWM mode.
 * @param TIMx Pointer to the TIM peripheral register structure.
 * @param Channel The TIM channel (TIM_CHANNEL_1, TIM_CHANNEL_2, etc.).
 * @param OCMode Output Compare mode (TIM_OCMODE_PWM1).
 * @param OCPolarity Output Compare polarity (TIM_OCPOLARITY_HIGH).
 */
static void TIM_OC_SetConfig(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t OCMode, uint32_t OCPolarity)
{
    uint32_t tmpccmrx;
    uint32_t tmpccer = TIMx->CCER;
    uint32_t ccer_channel_shift = Channel; // TIM_CHANNEL_x values are already correct shifts for CCER bits

    // Clear CCxE bit for the channel to safely reconfigure
    tmpccer &= ~((uint32_t)TIM_CCER_CC1E << ccer_channel_shift);

    // Determine which CCMR register and which half to modify
    if (Channel == TIM_CHANNEL_1)
    {
        tmpccmrx = TIMx->CCMR1;
        // Clear OC1M, CC1S, OC1PE bits
        tmpccmrx &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S | TIM_CCMR1_OC1PE);
        // Set OC1M (PWM Mode) and OC1PE (Preload Enable)
        tmpccmrx |= (OCMode | TIM_CCMR1_OC1PE); // OCMode (0x60) and OC1PE (0x08) align for OC1
        TIMx->CCMR1 = tmpccmrx;
    }
    else if (Channel == TIM_CHANNEL_2)
    {
        tmpccmrx = TIMx->CCMR1;
        // Clear OC2M, CC2S, OC2PE bits
        tmpccmrx &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S | TIM_CCMR1_OC2PE);
        // Set OC2M (PWM Mode) and OC2PE (Preload Enable). Shift by 8 for Channel 2.
        tmpccmrx |= ((OCMode | TIM_CCMR1_OC1PE) << 8U); // Reuse OC1PE mask, apply correct shift for OC2
        TIMx->CCMR1 = tmpccmrx;
    }
    else if (Channel == TIM_CHANNEL_3)
    {
        tmpccmrx = TIMx->CCMR2;
        // Clear OC3M, CC3S, OC3PE bits
        tmpccmrx &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S | TIM_CCMR2_OC3PE);
        // Set OC3M (PWM Mode) and OC3PE (Preload Enable)
        tmpccmrx |= (OCMode | TIM_CCMR2_OC3PE); // OCMode (0x60) and OC3PE (0x08) align for OC3
        TIMx->CCMR2 = tmpccmrx;
    }
    else if (Channel == TIM_CHANNEL_4)
    {
        tmpccmrx = TIMx->CCMR2;
        // Clear OC4M, CC4S, OC4PE bits
        tmpccmrx &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_CC4S | TIM_CCMR2_OC4PE);
        // Set OC4M (PWM Mode) and OC4PE (Preload Enable). Shift by 8 for Channel 4.
        tmpccmrx |= ((OCMode | TIM_CCMR2_OC3PE) << 8U); // Reuse OC3PE mask, apply correct shift for OC4
        TIMx->CCMR2 = tmpccmrx;
    }

    // Set Output Polarity (CCxP bit)
    tmpccer |= (OCPolarity << ccer_channel_shift);

    // Write to TIMx CCER
    TIMx->CCER = tmpccer;
}

/**
 * @brief Sets the pulse value (duty cycle) for a TIM Output Compare channel.
 * @param TIMx Pointer to the TIM peripheral register structure.
 * @param Channel The TIM channel (TIM_CHANNEL_1, TIM_CHANNEL_2, etc.).
 * @param Pulse The pulse value to be loaded into CCRx.
 */
static void TIM_Set_Pulse(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t Pulse)
{
    // The Pulse value is directly written to the CCRx register.
    // The hardware will load this new value into the active register at the next update event
    // because OCxPE (Output Compare Preload Enable) is set in TIM_OC_SetConfig.
    switch (Channel)
    {
        case TIM_CHANNEL_1:
            TIMx->CCR1 = Pulse;
            break;
        case TIM_CHANNEL_2:
            TIMx->CCR2 = Pulse;
            break;
        case TIM_CHANNEL_3:
            TIMx->CCR3 = Pulse;
            break;
        case TIM_CHANNEL_4:
            TIMx->CCR4 = Pulse;
            break;
        default:
            // Should not happen with proper channel validation
            break;
    }
}

/**
 * @brief Initializes a specific PWM channel.
 * This function configures the necessary GPIO and Timer peripherals for PWM output.
 * It sets up the timer in PWM mode 1 with a default frequency of 1kHz and 50% duty cycle.
 * @param TRD_Channel The logical PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        // Invalid channel, return without action
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // 1. Enable GPIO Clock for the specified port
    RCC_Enable_GPIO_Clock(config->PortName);

    // 2. Configure GPIO pin for Alternate Function mode
    GPIO_Config_AF(config->PortName, config->PinNumber, config->AFNumber);

    // 3. Enable Timer Clock for the specified timer instance
    RCC_Enable_TIM_Clock(config->TIMx);

    // 4. Set TIM Base Configuration: Default to 1kHz for initial setup
    // For 1kHz frequency, with a 84MHz timer clock:
    // We can choose a prescaler (PSC) to get a manageable counter frequency.
    // Example: A counter clock of 1MHz (prescaler = 84-1 = 83)
    // Then, for 1kHz output frequency: period (ARR) = (1MHz / 1kHz) - 1 = 1000 - 1 = 999
    uint32_t prescaler = (TIMER_CLOCK_FREQ / 1000000UL) - 1; // PSC for 1MHz counter clock
    uint32_t period = (1000000UL / 1000UL) - 1;             // ARR for 1kHz output

    // Ensure prescaler and period are within valid ranges for the timer
    if (prescaler > 0xFFFFUL) prescaler = 0xFFFFUL;
    if (period > 0xFFFFUL && (config->TIMx != TIM2 && config->TIMx != TIM5)) period = 0xFFFFUL; // 16-bit timers

    TIM_Base_SetConfig(config->TIMx, prescaler, period);

    // 5. Configure PWM Output Channel: Mode (PWM1), Polarity (Active High)
    TIM_OC_SetConfig(config->TIMx, config->ChannelNumber, TIM_OCMODE_PWM1, TIM_OCPOLARITY_HIGH);

    // Set initial pulse value (duty cycle). For 50%, it's (period + 1) / 2
    uint32_t initial_pulse = (period + 1) / 2;
    TIM_Set_Pulse(config->TIMx, config->ChannelNumber, initial_pulse);

    // 6. Generate an Update Event to load all buffered registers (PSC, ARR, CCRx)
    // into the active registers immediately.
    config->TIMx->EGR |= TIM_EGR_UG;
}

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 * @param TRD_Channel The logical PWM channel.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT || frequency == 0 || duty > 100)
    {
        // Invalid channel, frequency, or duty cycle
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t Channel = config->ChannelNumber;

    // Temporarily stop the timer counter to ensure smooth parameter update
    // Store CR1 state, then disable CEN bit
    uint32_t cr1_backup = TIMx->CR1;
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Calculate Prescaler and Period (ARR)
    // (TIMER_CLOCK_FREQ / (Prescaler + 1)) / (Period + 1) = Frequency
    // Aim to keep (Period + 1) as large as possible for higher resolution.
    // Try to find a Prescaler such that Period fits in 16-bit for 16-bit timers.
    uint32_t prescaler = 0;
    uint32_t period = 0;
    uint32_t timer_max_arr = (TIMx == TIM2 || TIMx == TIM5) ? 0xFFFFFFFFUL : 0xFFFFUL;

    // Calculate minimal prescaler and corresponding period
    period = (TIMER_CLOCK_FREQ / frequency) - 1;

    // Adjust prescaler if period exceeds maximum or if resolution can be improved
    if (period > timer_max_arr)
    {
        prescaler = (period / timer_max_arr) + 1;
        period = (TIMER_CLOCK_FREQ / (prescaler + 1) / frequency) - 1;
    }

    // Ensure prescaler does not exceed 16-bit
    if (prescaler > 0xFFFFUL)
    {
        // This case indicates frequency is too low for the given timer clock
        // or a very high resolution is being attempted.
        // Cap prescaler and period to maximum values to generate *some* output,
        // though not exact.
        prescaler = 0xFFFFUL;
        period = timer_max_arr;
        // Optionally, return an error code or log a warning here.
    }


    // Set Prescaler and Period (ARR) registers
    TIMx->PSC = (uint16_t)prescaler;
    TIMx->ARR = period;

    // Calculate Pulse (CCRx) based on duty cycle
    uint32_t pulse = ((tlong)(period + 1) * duty) / 100;

    // Ensure pulse does not exceed period (100% duty cycle)
    if (pulse > period) {
        pulse = period;
    }

    // Set CCRx register
    TIM_Set_Pulse(TIMx, Channel, pulse);

    // Generate an update event to apply the new PSC, ARR, and CCRx values
    TIMx->EGR |= TIM_EGR_UG;

    // Restore timer counter state if it was running before the update
    if (cr1_backup & TIM_CR1_CEN)
    {
        TIMx->CR1 |= TIM_CR1_CEN;
    }
}

/**
 * @brief Starts the PWM signal generation on a specific channel.
 * @param TRD_Channel The logical PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t Channel = config->ChannelNumber;

    // Enable the Capture/Compare Channel Output (CCxE bit)
    // TIM_CHANNEL_x defines are already the correct bit offsets for CCER
    TIMx->CCER |= ((uint32_t)TIM_CCER_CC1E << Channel);

    // Enable the Timer Peripheral (CEN bit)
    TIMx->CR1 |= TIM_CR1_CEN;

    // For advanced timers (TIM1, TIM8), Main Output Enable (MOE) needs to be set.
    // Our chosen timers (TIM2, TIM3, TIM4, TIM10) are general-purpose and don't require this.
    // If TIM1 were used:
    // if (TIMx == TIM1) {
    //     TIMx->BDTR |= TIM_BDTR_MOE;
    // }
}

/**
 * @brief Stops the PWM signal generation on a specific channel.
 * @param TRD_Channel The logical PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t Channel = config->ChannelNumber;

    // Disable the Capture/Compare Channel Output (CCxE bit)
    TIMx->CCER &= ~((uint32_t)TIM_CCER_CC1E << Channel);

    // Disable the Timer Peripheral (CEN bit)
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // For advanced timers (TIM1, TIM8), disable Main Output Enable (MOE).
    // If TIM1 were used:
    // if (TIMx == TIM1) {
    //     TIMx->BDTR &= ~TIM_BDTR_MOE;
    // }
}

/**
 * @brief Powers off all configured PWM channels and their associated peripherals.
 * This includes disabling GPIOs, timers, and their clocks.
 */
void PWM_PowerOff(void)
{
    // Iterate through all possible logical PWM channels
    for (TRD_Channel_t i = (TRD_Channel_t)0; i < TRD_CHANNEL_COUNT; i++)
    {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];
        TIM_TypeDef *TIMx = config->TIMx;
        GPIO_TypeDef *GPIOx = config->PortName;
        uint32_t Channel = config->ChannelNumber;
        uint16_t Pin = config->PinNumber;

        // 1. Stop the PWM channel output and counter
        // Disable the Capture/Compare Channel Output (CCxE bit)
        TIMx->CCER &= ~((uint32_t)TIM_CCER_CC1E << Channel);

        // Disable the Timer Peripheral (CEN bit)
        TIMx->CR1 &= ~TIM_CR1_CEN;

        // 2. Configure GPIO pin to Analog mode (0b11) for power saving
        // This clears the alternate function setting and sets to analog.
        uint32_t pin_mode_shift = Pin * 2;
        GPIOx->MODER &= ~(GPIO_MODER_MODER0 << pin_mode_shift); // Clear MODER bits
        GPIOx->MODER |= (GPIO_MODER_ANALOG_MODE << pin_mode_shift); // Set to Analog Mode (0b11)

        // Optionally, reset speed and pull-up/down to default (no effect in Analog mode typically)
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0 << pin_mode_shift); // Clear speed
        GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << pin_mode_shift);       // Clear pull-up/down
    }

    // 3. Disable all used Timer peripheral clocks
    // This part can be optimized by only disabling clocks for timers that were actually used,
    for (TRD_Channel_t i = (TRD_Channel_t)0; i < TRD_CHANNEL_COUNT; i++)
    {
        RCC_Disable_TIM_Clock(pwm_channel_map[i].TIMx);
    }

    // 4. Disable all used GPIO clocks
    // Same as for timers, in a real system, you might track which GPIOs are enabled.
    RCC_Disable_GPIO_Clock(GPIOA);
    RCC_Disable_GPIO_Clock(GPIOB);
    // Add other GPIO ports if they were used in pwm_channel_map
    // RCC_Disable_GPIO_Clock(GPIOC);
    // ...
}