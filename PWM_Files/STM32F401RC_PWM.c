/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : This file implements bare-metal PWM generation for STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-17
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pwm.h"
#include "stm32f401xe.h" // For direct register access (e.g., TIM_TypeDef, GPIO_TypeDef)
#include "system_stm32f4xx.h" // For SystemCoreClock
#include <stdint.h>         // For standard integer types (e.g., uint32_t, uint8_t)

/*
 * NOTE: The following type definitions (tlong, tbyte) are assumed to be
 *       provided by a common 'types.h' or directly in 'pwm.h'.
 *       For this implementation, they are aliased to standard C types.
 *       DO NOT REDEFINE if they are already present in pwm.h or another included header.
 */
#ifndef TLONG_TYPEDEF_DEFINED
#define TLONG_TYPEDEF_DEFINED
typedef uint32_t tlong; // Assuming tlong maps to uint32_t for frequency/period
#endif
#ifndef TBYTE_TYPEDEF_DEFINED
#define TBYTE_TYPEDEF_DEFINED
typedef uint8_t  tbyte; // Assuming tbyte maps to uint8_t for duty cycle (0-100%)
#endif


/* Private typedef -----------------------------------------------------------*/

/**
  * @brief  PWM Channel Configuration Structure definition
  *         This structure holds the hardware mapping and configuration
  *         details for a specific PWM channel.
  */
typedef struct
{
    TIM_TypeDef *TIMx;       /*!< Pointer to Timer peripheral (e.g., TIM1, TIM2) */
    uint8_t ChannelNumber;   /*!< Logical Timer Channel number (1, 2, 3, or 4) */
    GPIO_TypeDef *PortName;  /*!< Pointer to GPIO Port (e.g., GPIOA, GPIOB) */
    uint16_t PinNumber;      /*!< GPIO Pin number (e.g., GPIO_PIN_0 - as bitmask) */
    uint8_t AFNumber;        /*!< Alternate Function number (e.g., GPIO_AF1_TIM2) */
} PWM_Channel_Config_t;


/* Private define ------------------------------------------------------------*/

// Logical Timer Channel numbers (used for `ChannelNumber` in `PWM_Channel_Config_t`)
#define TIM_CHANNEL_1_L             1U
#define TIM_CHANNEL_2_L             2U
#define TIM_CHANNEL_3_L             3U
#define TIM_CHANNEL_4_L             4U

// GPIO Pin Numbers as bitmasks (e.g., (1U << 0) for Pin 0)
#define GPIO_PIN_0                  ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                  ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                  ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                  ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                  ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                  ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                  ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                  ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                  ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                  ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                 ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                 ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                 ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                 ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                 ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                 ((uint16_t)0x8000)  /* Pin 15 selected   */

// Alternate Function defines for STM32F401RC (Refer to datasheet/RM for exact values)
// AF1: TIM1, TIM2
// AF2: TIM3, TIM4, TIM5
// AF3: TIM9, TIM10, TIM11
#define GPIO_AF_TIM1                ((uint8_t)0x01) // AF1
#define GPIO_AF_TIM2                ((uint8_t)0x01) // AF1
#define GPIO_AF_TIM3                ((uint8_t)0x02) // AF2
#define GPIO_AF_TIM4                ((uint8_t)0x02) // AF2
#define GPIO_AF_TIM5                ((uint8_t)0x02) // AF2
#define GPIO_AF_TIM9                ((uint8_t)0x03) // AF3
#define GPIO_AF_TIM10               ((uint8_t)0x03) // AF3
#define GPIO_AF_TIM11               ((uint8_t)0x03) // AF3

// PWM Mode 1: In upcounting, channel is active as long as TIMx_CNT < TIMx_CCRx. Else inactive.
// This corresponds to 0b110 in OCxM bits.
#define TIM_OCMODE_PWM1             (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) // This define is for OC1M, but the bit pattern is generic

// Output Polarity (Active High) - CCxP bit is 0 for active high
#define TIM_OCPOLARITY_ACTIVE_HIGH  0x00000000U // No bit set for CCxP

/* Private macros ------------------------------------------------------------*/

// Macro to get pin source (0-15) from GPIO_PIN_X bitmask.
// Uses __builtin_ctz (Count Trailing Zeros), a GCC/Clang extension.
#define GPIO_PIN_TO_PIN_SOURCE(PIN_N) ((uint32_t)__builtin_ctz(PIN_N))

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  Array mapping logical TRD_Channel_t to physical PWM hardware configurations.
  *         This array lists the specific Timer, Channel, GPIO Port, Pin, and Alternate
  *         Function for each defined `TRD_Channel_t`.
  *         Ensure this matches the `TRD_Channel_t` enum defined in `pwm.h`.
  */
static const PWM_Channel_Config_t pwm_channel_map[TRD_CHANNEL_COUNT] =
{
    // Example Mappings for STM32F401RC
    // TRD_CHANNEL_PWM1: TIM2_CH1 (PA0), AF1
    {TIM2, TIM_CHANNEL_1_L, GPIOA, GPIO_PIN_0, GPIO_AF_TIM2},
    // TRD_CHANNEL_PWM2: TIM3_CH1 (PA6), AF2
    {TIM3, TIM_CHANNEL_1_L, GPIOA, GPIO_PIN_6, GPIO_AF_TIM3},
    // TRD_CHANNEL_PWM3: TIM4_CH1 (PB6), AF2
    {TIM4, TIM_CHANNEL_1_L, GPIOB, GPIO_PIN_6, GPIO_AF_TIM4},
    // TRD_CHANNEL_PWM4: TIM1_CH1 (PA8), AF1 (Advanced Timer)
    {TIM1, TIM_CHANNEL_1_L, GPIOA, GPIO_PIN_8, GPIO_AF_TIM1},
    // Add more channels here if TRD_Channel_t enum has more entries.
    // Ensure unique combinations or clear handling of shared pins/timers if applicable.
};

/* Private function prototypes -----------------------------------------------*/

static void _PWM_ConfigureGPIO(GPIO_TypeDef *GPIOx, uint16_t pin_mask, uint8_t af_num);
static void _PWM_EnableTimerClock(TIM_TypeDef *TIMx);
static void _PWM_SetTimerRegisters(TIM_TypeDef *TIMx, uint32_t prescaler, uint32_t period_arr);
static void _PWM_SetChannelRegisters(TIM_TypeDef *TIMx, uint8_t channel_number_L, uint32_t pulse_ccr, uint32_t mode);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes a specific PWM channel with default settings.
  *         This function configures the necessary GPIO and Timer peripherals.
  * @param  TRD_Channel The logical PWM channel to initialize.
  * @retval None
  */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        // Handle invalid channel: log error, assert, or return an error status if API allowed.
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // 1. Enable Timer and GPIO Clocks
    // GPIO clocks need to be enabled for configuration
    if (config->PortName == GPIOA)      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (config->PortName == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // Add more GPIO ports if used by other channels.

    _PWM_EnableTimerClock(config->TIMx);

    // 2. Configure GPIO Pin for Alternate Function
    _PWM_ConfigureGPIO(config->PortName, config->PinNumber, config->AFNumber);

    // 3. De-initialize the timer (soft reset of critical registers)
    // It's good practice to ensure a known state before configuration.
    // Disable counter first.
    config->TIMx->CR1 &= ~TIM_CR1_CEN;
    config->TIMx->CR1 = 0x0000;
    config->TIMx->CR2 = 0x0000;
    config->TIMx->SMCR = 0x0000;
    config->TIMx->DIER = 0x0000;
    config->TIMx->CCMR1 = 0x0000;
    config->TIMx->CCMR2 = 0x0000;
    config->TIMx->CCER = 0x0000;
    config->TIMx->CNT = 0x0000; // Reset counter
    config->TIMx->PSC = 0x0000; // Reset prescaler
    config->TIMx->ARR = 0x0000; // Reset auto-reload
    if (config->TIMx == TIM1) { // BDTR exists only for advanced timers like TIM1
        config->TIMx->BDTR = 0x0000; // Reset BDTR
    }

    // 4. Configure Timer Base and PWM Channel with initial values
    // These values will be refined by PWM_Set_Freq.
    // Set a default prescaler and period for a 1MHz internal timer clock, 1kHz period (1ms)
    _PWM_SetTimerRegisters(config->TIMx, (SystemCoreClock / 1000000U) - 1U, 1000U - 1U);
    _PWM_SetChannelRegisters(config->TIMx, config->ChannelNumber, 0, TIM_OCMODE_PWM1); // 0% duty, PWM Mode 1

    // 5. Enable Main Output (MOE) for advanced timers (TIM1)
    // BDTR_MOE must be set for TIM1, TIM8 (not on F401RC) to enable outputs.
    if (config->TIMx == TIM1)
    {
        config->TIMx->BDTR |= TIM_BDTR_MOE;
    }
}

/**
  * @brief  Sets the frequency and duty cycle for a specific PWM channel.
  * @param  TRD_Channel The logical PWM channel to configure.
  * @param  frequency The desired PWM frequency in Hz. Must be > 0.
  * @param  duty The desired duty cycle in percentage (0-100).
  * @retval None
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT || frequency == 0)
    {
        // Invalid channel or frequency, handle error.
        return;
    }
    if (duty > 100)
    {
        duty = 100; // Cap duty cycle at 100%
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;

    // Determine the timer clock frequency.
    // On STM32F401RC, assuming standard clock configuration (e.g., HCLK=84MHz, APB1/2 prescalers configured
    // to feed timers at HCLK), the timer clock is SystemCoreClock.
    uint32_t timer_clock_freq = SystemCoreClock; // Max 84MHz for F401RC timers.

    uint32_t prescaler = 0;
    uint32_t period_arr = 0;
    uint32_t pulse_ccr = 0;

    // Calculate Prescaler (PSC) and Auto-Reload Register (ARR) for desired frequency.
    // (Timer_Clock_Freq / (Prescaler + 1)) / (Period_ARR + 1) = Desired_Frequency
    // Prioritize maximum resolution by keeping ARR as high as possible.

    uint32_t max_arr;
    // TIM2 and TIM5 are 32-bit timers on STM32F4. Others are 16-bit.
    if (TIMx == TIM2 || TIMx == TIM5) {
        max_arr = 0xFFFFFFFFU; // Max value for a 32-bit register
    } else {
        max_arr = 0xFFFFU;     // Max value for a 16-bit register
    }

    // Calculate total timer ticks per period
    tlong total_ticks = timer_clock_freq / frequency;

    // Try to find a prescaler and period that fit
    if (total_ticks == 0) { // Avoid division by zero for frequency too high
        return;
    }

    // If total_ticks is too large for max_arr, we need to apply a prescaler
    if (total_ticks > (max_arr + 1)) {
        prescaler = (total_ticks / (max_arr + 1));
        // Check if prescaler itself exceeds max_arr (0xFFFF for PSC register)
        if (prescaler > 0xFFFFU) {
             prescaler = 0xFFFFU; // Cap prescaler at its maximum
        }
    } else {
        prescaler = 0; // No prescaler needed (or minimal)
    }

    // Recalculate period_arr with the chosen prescaler
    period_arr = (timer_clock_freq / (frequency * (prescaler + 1U))) - 1U;

    // Ensure period_arr does not exceed max_arr (important for 16-bit timers)
    if (period_arr > max_arr) {
        period_arr = max_arr;
        // This means we can't achieve the exact requested frequency.
        // The effective frequency will be `timer_clock_freq / ((prescaler + 1) * (max_arr + 1))`
        // For production code, might return an error or log a warning.
    }

    // Update Prescaler and ARR registers
    _PWM_SetTimerRegisters(TIMx, prescaler, period_arr);

    // Calculate Capture Compare Register (CCR) value based on duty cycle
    // CCR = (ARR + 1) * Duty / 100
    pulse_ccr = (uint32_t)(((tlong)period_arr + 1U) * duty / 100U);

    // Set CCR value for the specific channel
    _PWM_SetChannelRegisters(TIMx, config->ChannelNumber, pulse_ccr, TIM_OCMODE_PWM1);

    // Generate an update event to apply new PSC and ARR values immediately
    TIMx->EGR |= TIM_EGR_UG;
}


/**
  * @brief  Starts the PWM generation on a specific channel.
  *         The timer must be initialized and configured (frequency, duty cycle) first.
  * @param  TRD_Channel The logical PWM channel to start.
  * @retval None
  */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        // Invalid channel, handle error.
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint8_t channel_number_L = config->ChannelNumber;

    // Enable the Capture compare channel output (CCxE bit in TIMx_CCER)
    uint32_t ccer_enable_mask = 0;
    switch (channel_number_L) {
        case TIM_CHANNEL_1_L: ccer_enable_mask = TIM_CCER_CC1E; break;
        case TIM_CHANNEL_2_L: ccer_enable_mask = TIM_CCER_CC2E; break;
        case TIM_CHANNEL_3_L: ccer_enable_mask = TIM_CCER_CC3E; break;
        case TIM_CHANNEL_4_L: ccer_enable_mask = TIM_CCER_CC4E; break;
        default: return; // Should not happen with valid TRD_Channel_t
    }
    TIMx->CCER |= ccer_enable_mask;

    // For advanced timers (TIM1), enable Main Output (MOE bit in TIMx_BDTR)
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Start the timer counter (CEN bit in TIMx_CR1)
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Stops the PWM generation on a specific channel.
  *         This disables the output of the specified channel. The timer
  *         counter itself may continue to run if other channels are active.
  * @param  TRD_Channel The logical PWM channel to stop.
  * @retval None
  */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        // Invalid channel, handle error.
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint8_t channel_number_L = config->ChannelNumber;

    // Disable the Capture compare channel output (CCxE bit in TIMx_CCER)
    uint32_t ccer_enable_mask = 0;
    switch (channel_number_L) {
        case TIM_CHANNEL_1_L: ccer_enable_mask = TIM_CCER_CC1E; break;
        case TIM_CHANNEL_2_L: ccer_enable_mask = TIM_CCER_CC2E; break;
        case TIM_CHANNEL_3_L: ccer_enable_mask = TIM_CCER_CC3E; break;
        case TIM_CHANNEL_4_L: ccer_enable_mask = TIM_CCER_CC4E; break;
        default: return; // Should not happen with valid TRD_Channel_t
    }
    TIMx->CCER &= ~ccer_enable_mask;

    // For advanced timers (TIM1), check if all channels are disabled before disabling MOE.
    // If other channels are still active, MOE should remain enabled.
    if (TIMx == TIM1) {
        if ((TIMx->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)) == 0) {
            TIMx->BDTR &= ~TIM_BDTR_MOE; // Disable Main Output if all channels are off
        }
    }

    // Optional: Stop the timer counter if no other channels on this TIMx are active.
    // A more complex system might track active channels per timer instance.
    // To completely stop the timer: TIMx->CR1 &= ~TIM_CR1_CEN;
}

/**
  * @brief  Powers off all PWM channels by disabling associated timers and resetting GPIOs.
  *         This function performs a hard shutdown of all PWM resources.
  * @retval None
  */
void PWM_PowerOff(void)
{
    // Iterate through all defined PWM channels to shut them down
    for (uint32_t i = 0; i < TRD_CHANNEL_COUNT; i++)
    {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];
        TIM_TypeDef *TIMx = config->TIMx;
        GPIO_TypeDef *GPIOx = config->PortName;
        uint16_t pin_mask = config->PinNumber;
        uint32_t pin_source = GPIO_PIN_TO_PIN_SOURCE(pin_mask);

        // 1. Stop the specific channel output (CCxE)
        uint32_t ccer_enable_mask = 0;
        switch (config->ChannelNumber) {
            case TIM_CHANNEL_1_L: ccer_enable_mask = TIM_CCER_CC1E; break;
            case TIM_CHANNEL_2_L: ccer_enable_mask = TIM_CCER_CC2E; break;
            case TIM_CHANNEL_3_L: ccer_enable_mask = TIM_CCER_CC3E; break;
            case TIM_CHANNEL_4_L: ccer_enable_mask = TIM_CCER_CC4E; break;
            default: break;
        }
        TIMx->CCER &= ~ccer_enable_mask;

        // 2. Disable timer counter (CEN)
        TIMx->CR1 &= ~TIM_CR1_CEN;

        // 3. Disable Main Output (MOE) for advanced timers
        if (TIMx == TIM1) {
            TIMx->BDTR &= ~TIM_BDTR_MOE;
        }

        // 4. Reset Timer registers to default values (minimal set for clean state)
        TIMx->CR1 = 0x0000;
        TIMx->CR2 = 0x0000;
        TIMx->SMCR = 0x0000;
        TIMx->DIER = 0x0000;
        TIMx->CCMR1 = 0x0000;
        TIMx->CCMR2 = 0x0000;
        TIMx->CCER = 0x0000;
        TIMx->CNT = 0x0000;
        TIMx->PSC = 0x0000;
        TIMx->ARR = 0x0000;
        TIMx->CCR1 = 0x0000;
        TIMx->CCR2 = 0x0000;
        TIMx->CCR3 = 0x0000;
        TIMx->CCR4 = 0x0000;

        // 5. Reset GPIO pin to default Input mode (floating)
        // MODER: Clear current mode bits (2 bits per pin) and set to 00 (Input mode)
        GPIOx->MODER &= ~(3U << (pin_source * 2U));
        // OSPEEDR: Clear current speed bits
        GPIOx->OSPEEDR &= ~(3U << (pin_source * 2U));
        // PUPDR: Clear pull-up/down bits (sets to no pull-up/down)
        GPIOx->PUPDR &= ~(3U << (pin_source * 2U));
        // AFR: Clear Alternate Function bits
        if (pin_source < 8) { // AFR[0] for pins 0-7
            GPIOx->AFR[0] &= ~(0xFU << (pin_source * 4U));
        } else { // AFR[1] for pins 8-15
            GPIOx->AFR[1] &= ~(0xFU << ((pin_source - 8U) * 4U));
        }
    }

    // 6. Disable all relevant Timer peripheral clocks in RCC.
    // This is a global disable and assumes these timers are not used by other modules.
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN);
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN);

    // GPIO clocks are typically left enabled unless entering deep sleep modes.
    // Disabling them here would require careful consideration of other GPIO uses.
    // RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOHEN);
}


/* Private function implementations ------------------------------------------*/

/**
  * @brief  Configures a GPIO pin for Alternate Function mode for timer output.
  * @param  GPIOx Pointer to the GPIO port (e.g., GPIOA, GPIOB).
  * @param  pin_mask Bitmask for the pin (e.g., GPIO_PIN_0 - 1U << 0).
  * @param  af_num Alternate Function number.
  * @retval None
  */
static void _PWM_ConfigureGPIO(GPIO_TypeDef *GPIOx, uint16_t pin_mask, uint8_t af_num)
{
    uint32_t pin_source = GPIO_PIN_TO_PIN_SOURCE(pin_mask); // Get pin index (0-15)

    // Set pin mode to Alternate Function (0b10)
    // Clear current mode bits and set to AF mode
    GPIOx->MODER &= ~(3U << (pin_source * 2U));
    GPIOx->MODER |= (2U << (pin_source * 2U));

    // Set output type to Push-Pull (0b0) - default for AF output
    GPIOx->OTYPER &= ~(1U << pin_source); // 0: Push-pull

    // Set output speed to Very High (0b11)
    GPIOx->OSPEEDR |= (3U << (pin_source * 2U)); // 11: Very High speed

    // No pull-up, no pull-down (0b00)
    GPIOx->PUPDR &= ~(3U << (pin_source * 2U)); // 00: No pull-up, no pull-down

    // Set Alternate Function
    if (pin_source < 8) { // AFR[0] for pins 0-7 (4 bits per pin)
        GPIOx->AFR[0] &= ~(0xFU << (pin_source * 4U)); // Clear current AF bits
        GPIOx->AFR[0] |= ((uint32_t)af_num << (pin_source * 4U)); // Set AF
    } else { // AFR[1] for pins 8-15 (4 bits per pin)
        GPIOx->AFR[1] &= ~(0xFU << ((pin_source - 8U) * 4U)); // Clear current AF bits
        GPIOx->AFR[1] |= ((uint32_t)af_num << ((pin_source - 8U) * 4U)); // Set AF
    }
}

/**
  * @brief  Enables the clock for the specified TIM peripheral.
  * @param  TIMx Pointer to the Timer peripheral.
  * @retval None
  */
static void _PWM_EnableTimerClock(TIM_TypeDef *TIMx)
{
    // Enable peripheral clock in RCC for the specific timer instance
    if (TIMx == TIM1)      RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (TIMx == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (TIMx == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (TIMx == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (TIMx == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    else if (TIMx == TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    else if (TIMx == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    else if (TIMx == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    // Add dummy read to ensure clock is propagated
    (void)TIMx->CR1;
}

/**
  * @brief  Configures the TIM Prescaler and Auto-Reload Register (Period).
  * @param  TIMx Pointer to the Timer peripheral.
  * @param  prescaler The prescaler value (PSC register value, 0-65535).
  * @param  period_arr The auto-reload value (ARR register value).
  * @retval None
  */
static void _PWM_SetTimerRegisters(TIM_TypeDef *TIMx, uint32_t prescaler, uint32_t period_arr)
{
    // Disable counter before modifying core registers to avoid glitches
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set Prescaler and Auto-Reload Register
    TIMx->PSC = prescaler;
    TIMx->ARR = period_arr;

    // Set CR1: Up-counting mode (DIR=0, CMS=0), enable auto-reload preload (ARPE=1)
    TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Clear direction and center-aligned mode bits
    TIMx->CR1 |= TIM_CR1_ARPE;                 // Enable auto-reload preload

    // Generate an update event to reload the prescaler and ARR values immediately
    TIMx->EGR |= TIM_EGR_UG;
}

/**
  * @brief  Configures a specific Capture Compare Channel for PWM.
  * @param  TIMx Pointer to the Timer peripheral.
  * @param  channel_number_L Logical Channel number (1, 2, 3, or 4).
  * @param  pulse_ccr The pulse width value (CCRx register value).
  * @param  mode The PWM mode (e.g., TIM_OCMODE_PWM1).
  * @retval None
  */
static void _PWM_SetChannelRegisters(TIM_TypeDef *TIMx, uint8_t channel_number_L, uint32_t pulse_ccr, uint32_t mode)
{
    volatile uint32_t *ccmr_reg;
    volatile uint32_t *ccr_reg;
    uint32_t ccmr_mode_mask;
    uint32_t ccmr_preload_mask;
    uint32_t ccer_enable_bit;
    uint32_t ccer_polarity_bit;
    uint32_t ccmr_offset_shift = 0; // Shift for channel 2 & 4 in CCMR registers

    // Determine which CCMR register (CCMR1 or CCMR2) and bit fields to use,
    // and identify the corresponding CCR and CCER bits.
    switch (channel_number_L)
    {
        case TIM_CHANNEL_1_L:
            ccmr_reg = &TIMx->CCMR1;
            ccr_reg = &TIMx->CCR1;
            ccer_enable_bit = TIM_CCER_CC1E;
            ccer_polarity_bit = TIM_CCER_CC1P;
            ccmr_mode_mask = TIM_CCMR1_OC1M;
            ccmr_preload_mask = TIM_CCMR1_OC1PE;
            ccmr_offset_shift = 0; // No shift for channel 1
            break;
        case TIM_CHANNEL_2_L:
            ccmr_reg = &TIMx->CCMR1;
            ccr_reg = &TIMx->CCR2;
            ccer_enable_bit = TIM_CCER_CC2E;
            ccer_polarity_bit = TIM_CCER_CC2P;
            ccmr_mode_mask = TIM_CCMR1_OC2M;
            ccmr_preload_mask = TIM_CCMR1_OC2PE;
            ccmr_offset_shift = 8U; // Shift by 8 bits for channel 2 in CCMR1
            break;
        case TIM_CHANNEL_3_L:
            ccmr_reg = &TIMx->CCMR2;
            ccr_reg = &TIMx->CCR3;
            ccer_enable_bit = TIM_CCER_CC3E;
            ccer_polarity_bit = TIM_CCER_CC3P;
            ccmr_mode_mask = TIM_CCMR2_OC3M;
            ccmr_preload_mask = TIM_CCMR2_OC3PE;
            ccmr_offset_shift = 0; // No shift for channel 3
            break;
        case TIM_CHANNEL_4_L:
            ccmr_reg = &TIMx->CCMR2;
            ccr_reg = &TIMx->CCR4;
            ccer_enable_bit = TIM_CCER_CC4E;
            ccer_polarity_bit = TIM_CCER_CC4P;
            ccmr_mode_mask = TIM_CCMR2_OC4M;
            ccmr_preload_mask = TIM_CCMR2_OC4PE;
            ccmr_offset_shift = 8U; // Shift by 8 bits for channel 4 in CCMR2
            break;
        default:
            return; // Invalid channel number
    }

    // Temporarily disable channel output to apply changes safely
    TIMx->CCER &= ~ccer_enable_bit;

    // Configure the Capture Compare Mode Register (CCMR)
    // Clear existing mode and preload bits, then set PWM mode 1 and enable preload.
    *ccmr_reg &= ~(ccmr_mode_mask | ccmr_preload_mask); // Clear OCxM and OCxPE
    *ccmr_reg |= ((mode << ccmr_offset_shift) | (ccmr_preload_mask)); // Set mode and preload enable

    // Set the Capture Compare Register (CCR) value (pulse width)
    *ccr_reg = pulse_ccr;

    // Configure polarity (CCER register): Active High
    // Clear the polarity bit (sets to active high as 0)
    TIMx->CCER &= ~ccer_polarity_bit;
    // If TIM_OCPOLARITY_ACTIVE_HIGH were a non-zero value for active low, it would be ORed here.
}

/***********************************************************************************************************************
* END OF FILE
***********************************************************************************************************************/