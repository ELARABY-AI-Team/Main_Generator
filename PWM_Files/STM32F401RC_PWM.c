/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Provides PWM generation functionalities for STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

// Note: This implementation assumes that the necessary STM32F401RC header
// (e.g., stm32f4xx.h, providing TIM_TypeDef, GPIO_TypeDef, RCC_TypeDef, etc.,
// and defines like RCC_APB1ENR_TIM2EN, GPIO_AF_TIMx, etc.) is included
// either directly or indirectly by "STM32F401RC_PWM.h".
// It also assumes SystemCoreClock is correctly updated by the system startup code.

#include "STM32F401RC_PWM.h"

// Assume tlong and tbyte are defined in STM32F401RC_PWM.h
// e.g.:
// typedef uint32_t tlong;
// typedef uint8_t tbyte;

// Assume TRD_Channel_t is an enum defined in STM32F401RC_PWM.h
// listing the available PWM channels, e.g.:
// typedef enum {
//     TRD_TIM1_CH1_PA8,
//     TRD_TIM1_CH2_PA9,
//     // ... other channels
//     TRD_TIM4_CH4_PB9,
//     TRD_CHANNEL_COUNT // Helper for array size
// } TRD_Channel_t;


// --- Timer Reservation Note ---
// As per requirements, TIM5 and TIM9 are reserved for potential OS or delay
// purposes. They and their channels are excluded from this PWM implementation.
// Available Timers for PWM: TIM1 (APB2), TIM2 (APB1), TIM3 (APB1), TIM4 (APB1),
//                           TIM10 (APB2 - 1ch), TIM11 (APB2 - 1ch)
// This implementation focuses on TIM1, TIM2, TIM3, TIM4 as they offer multiple channels.
// TIM10 and TIM11 are less common for general multi-channel PWM and are omitted
// to simplify the channel mapping unless explicitly required.

// --- Channel Mapping Structure ---
// Structure to hold configuration details for each PWM channel
typedef struct {
    TIM_TypeDef *TIMx;          // Pointer to the Timer instance
    uint32_t RCC_APB_EN_BIT;  // RCC APB enable bit for the timer
    uint8_t APB_Bus;          // 1 for APB1, 2 for APB2
    uint8_t Channel;          // Timer Channel number (1, 2, 3, 4)
    GPIO_TypeDef *GPIOx;        // Pointer to the GPIO port
    uint8_t GPIO_Pin_Num;     // GPIO pin number (0-15)
    uint8_t GPIO_AF;          // GPIO Alternate Function number
    uint32_t RCC_AHB1_EN_BIT; // RCC AHB1 enable bit for the GPIO port
} PWM_Channel_Info_t;

// Array holding configuration for supported PWM channels.
// The index of this array must correspond to the TRD_Channel_t enum value.
// Ensure TRD_Channel_t enum starts from 0 and matches this order.
const PWM_Channel_Info_t ChannelInfo[] = {
    // TRD_TIM1_CH1_PA8
    { TIM1, RCC_APB2ENR_TIM1EN, 2, 1, GPIOA, 8, GPIO_AF_TIM1, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM1_CH2_PA9
    { TIM1, RCC_APB2ENR_TIM1EN, 2, 2, GPIOA, 9, GPIO_AF_TIM1, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM1_CH3_PA10
    { TIM1, RCC_APB2ENR_TIM1EN, 2, 3, GPIOA, 10, GPIO_AF_TIM1, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM1_CH4_PA11

    // TRD_TIM2_CH1_PA0
    { TIM2, RCC_APB1ENR_TIM2EN, 1, 1, GPIOA, 0, GPIO_AF_TIM2, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM2_CH2_PA1
    { TIM2, RCC_APB1ENR_TIM2EN, 1, 2, GPIOA, 1, GPIO_AF_TIM2, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM2_CH3_PA2
    { TIM2, RCC_APB1ENR_TIM2EN, 1, 3, GPIOA, 2, GPIO_AF_TIM2, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM2_CH4_PA3
    { TIM2, RCC_APB1ENR_TIM2EN, 1, 4, GPIOA, 3, GPIO_AF_TIM2, RCC_AHB1ENR_GPIOAEN },

    // TRD_TIM3_CH1_PA6
    { TIM3, RCC_APB1ENR_TIM3EN, 1, 1, GPIOA, 6, GPIO_AF_TIM3, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM3_CH2_PA7
    { TIM3, RCC_APB1ENR_TIM3EN, 1, 2, GPIOA, 7, GPIO_AF_TIM3, RCC_AHB1ENR_GPIOAEN },
    // TRD_TIM3_CH3_PB0
    { TIM3, RCC_APB1ENR_TIM3EN, 1, 3, GPIOB, 0, GPIO_AF_TIM3, RCC_AHB1ENR_GPIOBEN },
    // TRD_TIM3_CH4_PB1
    { TIM3, RCC_APB1ENR_TIM3EN, 1, 4, GPIOB, 1, GPIO_AF_TIM3, RCC_AHB1ENR_GPIOBEN },

    // TRD_TIM4_CH1_PB6
    { TIM4, RCC_APB1ENR_TIM4EN, 1, 1, GPIOB, 6, GPIO_AF_TIM4, RCC_AHB1ENR_GPIOBEN },
    // TRD_TIM4_CH2_PB7
    { TIM4, RCC_APB1ENR_TIM4EN, 1, 2, GPIOB, 7, GPIO_AF_TIM4, RCC_AHB1ENR_GPIOBEN },
    // TRD_TIM4_CH3_PB8
    { TIM4, RCC_APB1ENR_TIM4EN, 1, 3, GPIOB, 8, GPIO_AF_TIM4, RCC_AHB1ENR_GPIOBEN },
    // TRD_TIM4_CH4_PB9
    { TIM4, RCC_APB1ENR_TIM4EN, 1, 4, GPIOB, 9, GPIO_AF_TIM4, RCC_AHB1ENR_GPIOBEN },

    // Add other channels if supported and needed, matching TRD_Channel_t enum
    // For example, if TIM10/TIM11 were needed:
    // { TIM10, RCC_APB2ENR_TIM10EN, 2, 1, PB8, 8, GPIO_AF_TIM10, RCC_AHB1ENR_GPIOBEN }, // Note: PB8 is also TIM4_CH3
    // { TIM11, RCC_APB2ENR_TIM11EN, 2, 1, PB9, 9, GPIO_AF_TIM11, RCC_AHB1ENR_GPIOBEN }, // Note: PB9 is also TIM4_CH4
};

// --- Helper Function to get Timer Clock Frequency ---
// Reads RCC configuration to determine the clock frequency provided to a specific timer.
// Handles APB prescalers which might double the timer clock if prescaler > 1.
static uint32_t GetTimerClockFrequency(TIM_TypeDef *TIMx, uint8_t apb_bus)
{
    uint32_t timer_clock;
    uint32_t pclk;
    uint32_t apb_prescaler;

    if (apb_bus == 1) { // APB1
        pclk = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
        apb_prescaler = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    } else { // APB2
        pclk = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
        apb_prescaler = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    }

    // If APB prescaler is division by 1 (0xx) timer clock is PCLK
    // If APB prescaler is division by >1 (1xx), timer clock is PCLK * 2
    if (apb_prescaler >= 4) { // Corresponds to div by 2, 4, 8, 16 (binary 100, 101, 110, 111)
        timer_clock = pclk * 2;
    } else {
        timer_clock = pclk;
    }

    return timer_clock;
}

// APB prescaler decoding table (copied/derived from standard CMSIS/HAL headers)
// Index 0-7 corresponds to PPRE bits value (000-111)
// Value is the right shift amount for SystemCoreClock
const uint8_t APBPrescTable[] = {0, 0, 0, 0, 1, 2, 3, 4}; // 0xx: /1, 100: /2, 101: /4, 110: /8, 111: /16

/**Functions ===========================================================================*/

/**
 * @brief Initialize the PWM hardware and configure the timer and GPIOs for the given channel.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= sizeof(ChannelInfo) / sizeof(ChannelInfo[0])) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Info_t *info = &ChannelInfo[TRD_Channel];
    TIM_TypeDef *TIMx = info->TIMx;
    GPIO_TypeDef *GPIOx = info->GPIOx;
    uint8_t pinNum = info->GPIO_Pin_Num;
    uint32_t pinMask = (1U << pinNum); // Use 1U for unsigned shift

    // 1. Enable GPIO clock
    RCC->AHB1ENR |= info->RCC_AHB1_EN_BIT;

    // 2. Configure GPIO pin for Alternate Function (AF)
    // Set pin mode to Alternate Function (10)
    GPIOx->MODER &= ~(0x3U << (pinNum * 2)); // Clear mode bits
    GPIOx->MODER |= (0x2U << (pinNum * 2));  // Set mode to AF (10)

    // Set output type to Push-Pull (0)
    GPIOx->OTYPER &= ~pinMask; // Clear OTYPE bit (0 for Push-Pull)

    // Set output speed to High (10)
    GPIOx->OSPEEDR &= ~(0x3U << (pinNum * 2)); // Clear speed bits
    GPIOx->OSPEEDR |= (0x2U << (pinNum * 2));  // Set speed to High (10)

    // Set pull-up/pull-down to No pull-up/down (00)
    GPIOx->PUPDR &= ~(0x3U << (pinNum * 2)); // Clear pull bits
    // PUPDR |= (0x0U << (pinNum * 2)); // Set to No pull-up/down (00) - redundant if cleared to 0

    // Set Alternate Function (AFRL for pins 0-7, AFRH for pins 8-15)
    if (pinNum < 8) {
        GPIOx->AFR[0] &= ~(0xFU << (pinNum * 4)); // Clear AF bits for low register
        GPIOx->AFR[0] |= ((uint32_t)info->GPIO_AF << (pinNum * 4)); // Set AF bits
    } else {
        GPIOx->AFR[1] &= ~(0xFU << ((pinNum - 8) * 4)); // Clear AF bits for high register
        GPIOx->AFR[1] |= ((uint32_t)info->GPIO_AF << ((pinNum - 8) * 4)); // Set AF bits
    }

    // 3. Enable Timer clock
    if (info->APB_Bus == 1) {
        RCC->APB1ENR |= info->RCC_APB_EN_BIT;
    } else {
        RCC->APB2ENR |= info->RCC_APB_EN_BIT;
    }

    // Delay after enabling clock (recommended in some appnotes)
    volatile uint32_t dummy_read = RCC->APB1ENR; // Read back register to ensure clock is active
    if (info->APB_Bus == 2) dummy_read = RCC->APB2ENR;


    // 4. Configure Timer for PWM
    // Stop the timer
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set counter mode to Upcounting (CMS = 00, DIR = 0)
    TIMx->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);

    // Enable Auto-Reload preload (ARPE)
    TIMx->CR1 |= TIM_CR1_ARPE;

    // Configure Capture/Compare (CC) channel for PWM Mode 1
    // OCM = 110 (PWM Mode 1), OCPE = 1 (Output compare preload enable)
    uint32_t cc_enable_bit = 0; // Used to store CCMR register value
    volatile uint32_t *ccmr_reg;

    if (info->Channel == 1 || info->Channel == 2) {
        ccmr_reg = &TIMx->CCMR1;
        cc_enable_bit = (info->Channel == 1) ? 0 : 8; // Bits [7:0] for CH1, [15:8] for CH2
    } else { // Channel 3 or 4
        ccmr_reg = &TIMx->CCMR2;
        cc_enable_bit = (info->Channel == 3) ? 0 : 8; // Bits [7:0] for CH3, [15:8] for CH4
    }

    // Clear previous configuration for this channel
    *ccmr_reg &= ~(0xFFU << cc_enable_bit); // Clear OCM and OCPE bits

    // Set PWM Mode 1 (0b110 for OCM) and enable Output Compare Preload (OCPE=1)
    *ccmr_reg |= ((TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2) << cc_enable_bit); // Set OCM to 110
    *ccmr_reg |= ((TIM_CCMR1_OC1PE) << cc_enable_bit);                  // Set OCPE to 1

    // Configure Capture/Compare Enable Register (CCER)
    // CCxE = 1 (Output enabled), CCxP = 0 (Output polarity high)
    // Leave CCxNE and CCxNP as 0 for standard non-complementary output
    uint32_t ccer_bit = (info->Channel - 1) * 4; // 4 bits per channel in CCER

    // Clear CCxE and CCxP bits for this channel
    TIMx->CCER &= ~(0x3U << ccer_bit); // Clear bits for CCx stream (CCxE, CCxP)

    // Set CCxE (Output Enable) - CCxP (Polarity) is 0 by default
    // TIMx->CCER |= (1U << ccer_bit); // Don't enable yet, done in PWM_Start

    // For TIM1 (and TIM8 if present), enable Main Output Enable (MOE) in BDTR
    // This is required for the output pin to actually toggle.
    if (TIMx == TIM1) {
         // BDTR configuration might be needed for dead-time, break, etc.
         // For simple PWM, just MOE is needed.
         // BDTR bits are reset to 0, MOE is bit 15.
         TIMx->BDTR |= TIM_BDTR_MOE; // Enable Main Output
    }

    // Generate Update Event to load configurations
    TIMx->EGR |= TIM_EGR_UG;
    // Clear the update flag (optional but good practice)
    // TIMx->SR &= ~TIM_SR_UIF;

    // Initial values for PSC, ARR, CCR should be set in Set_Freq
    // but clearing them here ensures a known state.
    TIMx->PSC = 0;
    TIMx->ARR = 0;
    switch (info->Channel) {
        case 1: TIMx->CCR1 = 0; break;
        case 2: TIMx->CCR2 = 0; break;
        case 3: TIMx->CCR3 = 0; break;
        case 4: TIMx->CCR4 = 0; break;
    }

    // The timer is still stopped after init. PWM_Start will enable CR1_CEN.
}

/**
 * @brief Set the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= sizeof(ChannelInfo) / sizeof(ChannelInfo[0])) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Info_t *info = &ChannelInfo[TRD_Channel];
    TIM_TypeDef *TIMx = info->TIMx;

    // Get timer clock frequency
    uint32_t timer_clock = GetTimerClockFrequency(TIMx, info->APB_Bus);

    // Calculate PSC and ARR values
    uint32_t psc = 0;
    uint32_t arr = 0;
    // Determine max ARR based on timer type (TIM2 is 32-bit, others 16-bit)
    // TIM5 is also 32-bit, but reserved.
    uint32_t max_arr = (TIMx == TIM2) ? 0xFFFFFFFFU : 0xFFFFU;

    if (frequency == 0 || timer_clock == 0) {
        // Avoid division by zero or invalid states.
        // Set a very low frequency (e.g., max prescaler and period)
        // Or just leave previous values, but setting to a known low state is safer.
        psc = 0xFFFFU;
        arr = max_arr;
    } else {
        // Total timer ticks per period = Timer Clock / Frequency
        uint64_t total_ticks = (uint64_t)timer_clock / frequency;

        // Calculate minimum possible ARR for PSC=0
        // If total_ticks exceeds max_arr + 1, a prescaler is needed.
        if (total_ticks == 0) total_ticks = 1; // Should not happen if freq > 0 and clock > 0, but safety

        if (total_ticks > max_arr + 1) {
            // Need prescaler. Calculate PSC to bring total_ticks into ARR range.
            // psc = (timer_clock / frequency) / (arr + 1) - 1
            // To maximize resolution (large ARR), use largest possible ARR first.
            // Let's iterate PSC until ARR fits or we find a better fit.
            // Simpler approach: psc = (total_ticks - 1) / (max_arr + 1) seems too simple.
            // Let's try increasing PSC until ARR is within range.
            // timer_clock / ( (psc + 1) * (arr + 1) ) = frequency
            // (psc + 1) * (arr + 1) = timer_clock / frequency = total_ticks
            // psc + 1 = total_ticks / (arr + 1)
            // arr + 1 = total_ticks / (psc + 1)

            // Try to find a PSC that results in ARR <= max_arr
            psc = (uint32_t)(total_ticks / (max_arr + 1));
            // If total_ticks % (max_arr + 1) is 0, we might be able to use psc-1?
            // Let's stick to simple calculation. psc = total_ticks / max_arr is another option.
            // Using psc = total_ticks / (max_arr + 1) might result in arr=max_arr+1 sometimes.
            // Let's try psc = (total_ticks + max_arr) / (max_arr + 1) - 1
            // psc = (uint32_t)((total_ticks + max_arr) / (max_arr + 1) - 1);
            // A safer way is:
            psc = (uint32_t)(total_ticks / (max_arr + 1));
            if (total_ticks % (max_arr + 1) == 0 && psc > 0) {
                 // If division is exact, we can use a smaller PSC potentially?
                 // Or maybe the calculated psc is just right.
                 // Let's just calculate ARR with this PSC.
            }


            if (psc > 0xFFFFU) psc = 0xFFFFU; // Clamp PSC to its max value

            // Calculate ARR based on the determined PSC
            // arr + 1 = total_ticks / (psc + 1)
            // Make sure (psc + 1) is not zero. PSC is unsigned, always >= 0.
            if (psc + 1 == 0) { // Should not happen for uint32_t psc
                arr = max_arr;
            } else {
                arr = (uint32_t)(total_ticks / (psc + 1));
                if (arr > 0) arr -= 1; // arr = (total_ticks / (psc + 1)) - 1;
                else arr = 0; // if total_ticks / (psc + 1) is 0 or 1
            }

             if (arr > max_arr) arr = max_arr; // Safety clamp

        } else {
            // No prescaler needed (or total_ticks is very small)
            psc = 0;
            arr = (uint32_t)total_ticks;
            if (arr > 0) arr -= 1; // arr = total_ticks - 1;
            else arr = 0; // if total_ticks is 0 or 1
        }
    }

    // Clamp duty cycle to 0-100
    uint8_t clamped_duty = duty;
    if (clamped_duty > 100) {
        clamped_duty = 100;
    }

    // Calculate CCR value based on duty cycle
    // CCR = (ARR + 1) * duty / 100
    uint32_t ccr;
    if (clamped_duty == 0) {
        ccr = 0;
    } else if (clamped_duty == 100) {
        ccr = arr + 1; // For PWM1 mode, CCR >= ARR + 1 results in 100% duty
    } else {
        // Use 64-bit intermediate to avoid overflow during multiplication
        ccr = (uint32_t)(((uint64_t)(arr + 1) * clamped_duty) / 100);
        // Clamp CCR to max ARR value if calculation exceeds it (shouldn't happen with duty <= 100)
        if (ccr > arr + 1) ccr = arr + 1; // Safety clamp, CCR can be ARR+1 for 100%
    }


    // Apply calculated values to timer registers
    // Temporarily disable counter to write PSC and ARR if needed (safer, but UG also works)
    // TIMx->CR1 &= ~TIM_CR1_CEN; // Not strictly necessary with UG

    TIMx->PSC = psc;
    TIMx->ARR = arr;

    // Write CCR value to the appropriate channel register
    switch (info->Channel) {
        case 1: TIMx->CCR1 = ccr; break;
        case 2: TIMx->CCR2 = ccr; break;
        case 3: TIMx->CCR3 = ccr; break;
        case 4: TIMx->CCR4 = ccr; break;
    }

    // Generate an update event to load the new PSC, ARR, and CCR values
    TIMx->EGR |= TIM_EGR_UG;

    // Clear the update interrupt flag if it was set by UG (optional)
    // TIMx->SR &= ~TIM_SR_UIF;

    // TIMx->CR1 |= TIM_CR1_CEN; // Re-enable counter if stopped (not needed if not stopped)
}

/**
 * @brief Enable and start PWM signal generation on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= sizeof(ChannelInfo) / sizeof(ChannelInfo[0])) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Info_t *info = &ChannelInfo[TRD_Channel];
    TIM_TypeDef *TIMx = info->TIMx;

    // Enable the specific channel output in CCER
    // CCxE = 1 (Output enabled)
    uint32_t ccer_bit = (info->Channel - 1) * 4; // 4 bits per channel in CCER
    TIMx->CCER |= (1U << ccer_bit);

    // For TIM1 (and TIM8), ensure Main Output Enable (MOE) is set in BDTR
    // This is set in PWM_Init, but ensure it's not cleared unintentionally.
    // It affects *all* outputs on the timer, not just this channel.
    if (TIMx == TIM1) {
         TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Enable the timer counter
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Stop the PWM signal output on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= sizeof(ChannelInfo) / sizeof(ChannelInfo[0])) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Info_t *info = &ChannelInfo[TRD_Channel];
    TIM_TypeDef *TIMx = info->TIMx;

    // Disable the specific channel output in CCER
    // CCxE = 0 (Output disabled)
    uint32_t ccer_bit = (info->Channel - 1) * 4; // 4 bits per channel in CCER
    TIMx->CCER &= ~(1U << ccer_bit);

    // Note: This function only stops the *output* of the specified channel.
    // The timer counter (CR1_CEN) is left running as other channels on the
    // same timer might still be active.
    // For TIM1, BDTR_MOE is also left enabled. Disabling MOE here would stop
    // all TIM1 outputs. A more complex logic would be needed to only disable MOE
    // when the last active channel on TIM1 is stopped.
}

/**
 * @brief Disable all PWM peripherals and outputs to reduce power consumption.
 * This function disables the clocks for the Timers used for PWM and resets
 * their corresponding GPIO pins to their default state (Input floating).
 * Reserved timers (TIM5, TIM9) are not affected by this function.
 */
void PWM_PowerOff(void)
{
    // Disable clocks for all Timers potentially used for PWM
    // Excluding reserved timers (TIM5, TIM9).
    // APB1 Timers: TIM2, TIM3, TIM4
    // APB2 Timers: TIM1, TIM10, TIM11 (Only TIM1 listed in ChannelInfo, add others if used)
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN);
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN); // Add RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN if TIM10/11 used

    // Configure GPIO pins back to a safe state (e.g., Input, no pull)
    // Iterate through all configured PWM channels and reset their GPIO pins.
    for (size_t i = 0; i < sizeof(ChannelInfo) / sizeof(ChannelInfo[0]); ++i) {
        const PWM_Channel_Info_t *info = &ChannelInfo[i];
        GPIO_TypeDef *GPIOx = info->GPIOx;
        uint8_t pinNum = info->GPIO_Pin_Num;

        // Set pin mode to Input (00)
        GPIOx->MODER &= ~(0x3U << (pinNum * 2)); // Clear mode bits
        // MODER |= (0x0U << (pinNum * 2)); // Set mode to Input (00) - redundant

        // Set pull-up/pull-down to No pull-up/down (00)
        GPIOx->PUPDR &= ~(0x3U << (pinNum * 2)); // Clear pull bits
        // PUPDR |= (0x0U << (pinNum * 2)); // Set to No pull-up/down (00) - redundant
    }

    // Note: Disabling the timer clock also stops the counter and resets registers.
    // Individual channel outputs in CCER are also effectively disabled.
    // For TIM1, the BDTR_MOE bit is reset when the TIM1 clock is disabled.
}

// Optional: Add a function to get the configuration of a channel for debugging
/*
const PWM_Channel_Info_t* PWM_GetChannelInfo(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= sizeof(ChannelInfo) / sizeof(ChannelInfo[0])) {
        return NULL; // Invalid channel
    }
    return &ChannelInfo[TRD_Channel];
}
*/