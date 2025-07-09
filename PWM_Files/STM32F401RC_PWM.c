/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready PWM implementation for STM32F401RC
* Author         : Technology Inovation Software Team (based on requirements)
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

/*
 * Development Note:
 * This implementation assumes a specific system clock configuration where the Timer clocks
 * (TIM3, TIM4, TIM5) are derived from APB1 and are running at 84 MHz. In a production system,
 * the actual timer clock frequency should be calculated dynamically based on the RCC clock
 * configuration registers (SYSCLK source, HCLK, PCLK1, PCLK2 prescalers).
 *
 * Timer Reservation:
 * As per requirements, Timers TIM1 and TIM2 are reserved for potential OS/delay/other
 * critical system functions and are excluded from this general-purpose PWM implementation.
 *
 * t_pwm_channel interpretation:
 * The requirement "enum named is `t_pwm_channel` use it inside the function IMPLEMENTATION"
 * is interpreted as using the type `TRD_Channel_t` (assumed to be defined in
 * "STM32F401RC_PWM.h") as the channel identifier type within the function implementations.
 * A local typedef alias is used (`t_pwm_channel`) to align with the naming request,
 * although using `TRD_Channel_t` directly is standard C practice.
 */

#include "STM32F401RC_PWM.h" // Assumes this header defines TRD_Channel_t
#include "stm32f4xx.h"       // For STM32F401 register definitions

// Assume tbyte is uint8_t and tlong is uint32_t or uint64_t for frequency/period calculations.
// In a real project, these should be defined in a common types header.
#ifndef tbyte
typedef uint8_t tbyte;
#endif
#ifndef tlong
typedef uint32_t tlong; // Frequency values typically fit in 32-bit
#endif

// Local type alias as per requirement interpretation
typedef TRD_Channel_t t_pwm_channel;

// Structure to hold configuration for each PWM channel
typedef struct {
    TIM_TypeDef *TIM_Instance;       // Pointer to the Timer peripheral
    uint32_t TIM_Channel_Num;        // Timer Channel Number (e.g., 1, 2, 3, 4)
    GPIO_TypeDef *GPIO_Port;         // Pointer to the GPIO port
    uint16_t GPIO_Pin;               // GPIO Pin number (e.g., GPIO_PIN_6 is 6)
    uint8_t GPIO_AF;                 // GPIO Alternate Function number
    uint32_t RCC_APB_Peripheral_EN;  // RCC APBxENR bit for the Timer (e.g., RCC_APB1ENR_TIM3EN)
    uint32_t RCC_AHB1_Peripheral_EN; // RCC AHB1ENR bit for the GPIO Port (e.g., RCC_AHB1ENR_GPIOAEN)
    uint32_t TIM_CCR_Offset;         // Offset for CCR register (TIMx->CCR1, TIMx->CCR2, ...)
    uint32_t TIM_CCMR_Offset;        // Offset for CCMR register (TIMx->CCMR1, TIMx->CCMR2)
    uint32_t TIM_CCMR_Shift;         // Shift for OCxM/OCxPE bits in CCMR
    uint32_t TIM_CCER_Shift;         // Shift for CCxE/CCxP bits in CCER
    uint32_t TIM_Max_ARR;            // Maximum possible ARR value (0xFFFF for 16-bit, 0xFFFFFFFF for 32-bit)
} PWM_ChannelConfig_t;

// Configuration array mapping TRD_Channel_t to hardware settings
// This array MUST be kept in sync with the TRD_Channel_t enum in the header.
// GPIO_PIN_x definitions are typically found in stm32f4xx_hal_gpio.h or similar,
// or can be defined manually as (1 << x).
// GPIO_AFx_TIMy definitions are typically found in stm32f4xx_hal_gpio_ex.h or similar.
// RCC_APBxENR_TIMyEN definitions are typically found in stm32f4xx_hal_rcc.h or similar.
// We define the pin numbers directly (e.g., 6 for PA6) and use hardcoded AF values and RCC bits for bare-metal access.
// Ensure these match your specific STM32F401RC pinout and AF mapping.
#define GPIO_PIN_0  0
#define GPIO_PIN_1  1
#define GPIO_PIN_2  2
#define GPIO_PIN_3  3
#define GPIO_PIN_6  6
#define GPIO_PIN_7  7
#define GPIO_PIN_8  8
#define GPIO_PIN_9  9

#define GPIO_AF2_TIM3 2
#define GPIO_AF2_TIM4 2
#define GPIO_AF2_TIM5 2


const PWM_ChannelConfig_t g_pwmChannelConfigs[TRD_CHANNEL_COUNT] = {
    // TIM3 Channels (16-bit timer on APB1) - Clock assumed 84MHz
    { TIM3, 1, GPIOA, GPIO_PIN_6, GPIO_AF2_TIM3, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOAEN, offsetof(TIM_TypeDef, CCR1), offsetof(TIM_TypeDef, CCMR1), 0, 0, 0xFFFF },
    { TIM3, 2, GPIOA, GPIO_PIN_7, GPIO_AF2_TIM3, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOAEN, offsetof(TIM_TypeDef, CCR2), offsetof(TIM_TypeDef, CCMR1), 8, 4, 0xFFFF },
    { TIM3, 3, GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOBEN, offsetof(TIM_TypeDef, CCR3), offsetof(TIM_TypeDef, CCMR2), 0, 8, 0xFFFF },
    { TIM3, 4, GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOBEN, offsetof(TIM_TypeDef, CCR4), offsetof(TIM_TypeDef, CCMR2), 8, 12, 0xFFFF },

    // TIM4 Channels (16-bit timer on APB1) - Clock assumed 84MHz
    { TIM4, 1, GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, offsetof(TIM_TypeDef, CCR1), offsetof(TIM_TypeDef, CCMR1), 0, 0, 0xFFFF },
    { TIM4, 2, GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, offsetof(TIM_TypeDef, CCR2), offsetof(TIM_TypeDef, CCMR1), 8, 4, 0xFFFF },
    { TIM4, 3, GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, offsetof(TIM_TypeDef, CCR3), offsetof(TIM_TypeDef, CCMR2), 0, 8, 0xFFFF },
    { TIM4, 4, GPIOB, GPIO_PIN_9, GPIO_AF2_TIM4, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, offsetof(TIM_TypeDef, CCR4), offsetof(TIM_TypeDef, CCMR2), 8, 12, 0xFFFF },

    // TIM5 Channels (32-bit timer on APB1) - Clock assumed 84MHz
    // Note: GPIOA Pin numbers must be explicitly defined or use a standard header
    { TIM5, 1, GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOAEN, offsetof(TIM_TypeDef, CCR1), offsetof(TIM_TypeDef, CCMR1), 0, 0, 0xFFFFFFFF },
    { TIM5, 2, GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOAEN, offsetof(TIM_TypeDef, CCR2), offsetof(TIM_TypeDef, CCMR1), 8, 4, 0xFFFFFFFF },
    { TIM5, 3, GPIOA, GPIO_PIN_2, GPIO_AF2_TIM5, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOAEN, offsetof(TIM_TypeDef, CCR3), offsetof(TIM_TypeDef, CCMR2), 0, 8, 0xFFFFFFFF },
    { TIM5, 4, GPIOA, GPIO_PIN_3, GPIO_AF2_TIM5, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOAEN, offsetof(TIM_TypeDef, CCR4), offsetof(TIM_TypeDef, CCMR2), 8, 12, 0xFFFFFFFF },
};

// Helper function to get the timer clock frequency
// This function currently hardcodes the clock value based on a common assumption for F401RC.
// For a production system, this should read RCC registers to determine the actual clock.
static uint32_t get_timer_clock_freq(TIM_TypeDef *TIMx) {
    // In a real application, this function would calculate the actual timer clock
    // by reading RCC_CFGR, determining SYSCLK, HCLK, PCLK1, PCLK2, and
    // the timer clock multiplier (2x if APB prescaler > 1).
    // For the STM32F401RC, APB1 timers (TIM2, TIM3, TIM4, TIM5) max PCLK1 is 42MHz.
    // APB2 timers (TIM1, TIM9, TIM10, TIM11) max PCLK2 is 84MHz.
    // If the APB prescaler is > 1, the timer clock is 2 * PCLK.
    // Assuming HCLK=84MHz, APB1 prescaler=2 (PCLK1=42MHz), APB2 prescaler=1 (PCLK2=84MHz):
    // Timer clocks on APB1 (TIM3, TIM4, TIM5) = 2 * 42MHz = 84MHz.
    // Timer clocks on APB2 (TIM1, TIM9, TIM10, TIM11) = 1 * 84MHz = 84MHz.
    // If APB1 prescaler=1 (PCLK1=84MHz - not possible for F401 max), timer clock would be 84MHz.
    // If APB2 prescaler=2 (PCLK2=42MHz), timer clock would be 84MHz.
    // A common clock tree configuration results in 84MHz for APB1 timers and 84MHz (or 168MHz if PCLK2 is 84 and prescaler > 1) for APB2 timers.
    (void)TIMx; // Avoid unused parameter warning
    return 84000000; // 84 MHz assumed timer clock for TIM3, TIM4, TIM5
}


/**Functions ===========================================================================*/


/***********************************************************************************************************************
* Function Name  : PWM_Init
* Description    : Initializes the PWM hardware (GPIO, Timer clock, Timer basic settings).
* Parameters     : TRD_Channel_t TRD_Channel: The specific PWM channel to initialize.
* Return Value   : None.
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    t_pwm_channel channel = TRD_Channel; // Use local alias as per requirement interpretation

    if (channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel
        return;
    }

    const PWM_ChannelConfig_t *config = &g_pwmChannelConfigs[channel];

    // 1. Enable GPIO clock
    RCC->AHB1ENR |= config->RCC_AHB1_Peripheral_EN;

    // 2. Configure GPIO pin for Alternate Function (AF) mode
    uint32_t pin_mode_shift = config->GPIO_Pin * 2;
    config->GPIO_Port->MODER &= ~(0x3U << pin_mode_shift); // Clear mode bits (00)
    config->GPIO_Port->MODER |= (0x2U << pin_mode_shift);  // Set mode to Alternate Function (10)

    // 3. Configure GPIO pin Output Type (Push-Pull) - OTYPER bit 0 = Push-pull
    config->GPIO_Port->OTYPER &= ~(0x1U << config->GPIO_Pin);

    // 4. Configure GPIO pin Output Speed (High speed) - OSPEEDR bits 11 = High speed
    config->GPIO_Port->OSPEEDR &= ~(0x3U << pin_mode_shift);
    config->GPIO_Port->OSPEEDR |= (0x3U << pin_mode_shift);

    // 5. Configure GPIO pin Pull-up/Pull-down (No pull-up/pull-down) - PUPDR bits 00 = No pull-up/pull-down
    config->GPIO_Port->PUPDR &= ~(0x3U << pin_mode_shift);

    // 6. Configure GPIO pin Alternate Function mapping (AFRL for Pins 0-7, AFRH for Pins 8-15)
    if (config->GPIO_Pin < 8) {
        uint32_t pin_af_shift = config->GPIO_Pin * 4;
        config->GPIO_Port->AFR[0] &= ~(0xFU << pin_af_shift); // Clear AF bits
        config->GPIO_Port->AFR[0] |= (config->GPIO_AF << pin_af_shift); // Set AF
    } else {
        uint32_t pin_af_shift = (config->GPIO_Pin - 8) * 4;
        config->GPIO_Port->AFR[1] &= ~(0xFU << pin_af_shift); // Clear AF bits
        config->GPIO_Port->AFR[1] |= (config->GPIO_AF << pin_af_shift); // Set AF
    }

    // 7. Enable Timer clock
    // Check if Timer is on APB1 or APB2 based on base address
    if ((uint32_t)config->TIM_Instance >= APB2PERIPH_BASE) {
         RCC->APB2ENR |= config->RCC_APB_Peripheral_EN; // TIM1, TIM9, TIM10, TIM11
    } else { // Assumes APB1
         RCC->APB1ENR |= config->RCC_APB_Peripheral_EN; // TIM2, TIM3, TIM4, TIM5
    }
    // Provide a small delay for the peripheral clock to stabilize (optional but good practice)
    volatile uint32_t dummy = config->TIM_Instance->CR1;
    (void)dummy;

    // 8. Configure Timer basic settings:
    // - Stop the timer counter (CEN=0)
    // - Upcounting mode (DIR=0)
    // - Auto-Reload Preload Enable (ARPE=1)
    // - No clock division (CKD=00)
    config->TIM_Instance->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_DIR | TIM_CR1_CKD);
    config->TIM_Instance->CR1 |= TIM_CR1_ARPE;

    // 9. Configure Output Compare Mode (PWM Mode 1) and Preload Enable
    // Access the correct CCMR register (CCMR1 for Ch1/2, CCMR2 for Ch3/4)
    // Bits OCxM[2:0] (bits 6:4 or 14:12) = 110 for PWM Mode 1
    // Bit OCxPE (bit 3 or 11) = 1 for Output Compare Preload Enable
    volatile uint32_t *ccmr_reg = (volatile uint32_t *)((uint32_t)config->TIM_Instance + config->TIM_CCMR_Offset);
    uint32_t ccmr_mask = (0x7U << (config->TIM_CCMR_Shift + 4)) | (0x1U << (config->TIM_CCMR_Shift + 3)); // Mask for OCxM and OCxPE
    uint32_t ccmr_value = (0x6U << (config->TIM_CCMR_Shift + 4)) | (0x1U << (config->TIM_CCMR_Shift + 3)); // Value for PWM1 and OCxPE

    *ccmr_reg &= ~ccmr_mask;  // Clear relevant bits
    *ccmr_reg |= ccmr_value; // Set PWM Mode 1 and OCxPE

    // 10. Configure Output Polarity (Active High)
    // CCER register, CCxP bit (bit 1, 5, 9, 13) - 0 for active high (default)
    config->TIM_Instance->CCER &= ~(0x1U << (config->TIM_CCER_Shift + 1)); // Clear CCxP bit

    // 11. Disable output initially (Enable in PWM_Start)
    // CCER register, CCxE bit (bit 0, 4, 8, 12) - 0 to disable output
    config->TIM_Instance->CCER &= ~(0x1U << config->TIM_CCER_Shift);

    // 12. Generate an update event to load configurations (PSC, ARR, CCMR settings)
    // Note: This can cause a single-cycle pulse/glitch. For glitch-free updates,
    // this should be done after setting PSC/ARR/CCR in PWM_Set_Freq or use Update Disable (UDIS).
    config->TIM_Instance->EGR = TIM_EGR_UG;

    // 13. Clear the update interrupt flag (just in case it was set by UG)
    config->TIM_Instance->SR &= ~TIM_SR_UIF;
}

/***********************************************************************************************************************
* Function Name  : PWM_Set_Freq
* Description    : Sets the desired PWM frequency and duty cycle for the selected channel.
* Parameters     : TRD_Channel_t TRD_Channel: The specific PWM channel.
*                : tlong frequency: The desired frequency in Hz.
*                : tbyte duty: The desired duty cycle in percentage (0-100).
* Return Value   : None.
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    t_pwm_channel channel = TRD_Channel; // Use local alias

    // Validate inputs
    if (channel >= TRD_CHANNEL_COUNT || frequency == 0 || duty > 100) {
        // Invalid channel, frequency cannot be zero, duty cycle must be 0-100
        return;
    }

    const PWM_ChannelConfig_t *config = &g_pwmChannelConfigs[channel];
    uint32_t timer_clock = get_timer_clock_freq(config->TIM_Instance);

    // Calculate the total number of timer clock ticks required for one PWM period: Timer_Clock / frequency
    uint64_t total_ticks = (uint64_t)timer_clock / frequency;

    // Ensure total_ticks is at least 1 for any valid frequency
    if (total_ticks == 0) {
         // Frequency is too high for the timer clock. Set minimum period (ARR=0, PSC=0), max frequency.
         // This might not be exactly the requested frequency, but it's the fastest possible.
         total_ticks = 1; // ARR = 0, PSC = 0 results in 1 tick period
    }

    uint32_t psc = 0;
    uint32_t arr = 0;

    // Calculate PSC and ARR for the desired total_ticks (Period = (PSC + 1) * (ARR + 1))
    // We aim to maximize ARR for better duty cycle resolution while keeping PSC and ARR within limits.
    // A simple approach: Calculate PSC to bring (PSC+1)*(ARR+1) near the max ARR limit.
    uint64_t max_arr_plus_1 = (uint64_t)config->TIM_Max_ARR + 1;

    // Calculate PSC: (total_ticks / max_arr_plus_1)
    // If total_ticks fits within max_arr_plus_1, psc will be 0.
    // If total_ticks is larger, psc will be >= 1.
    psc = (uint32_t)((total_ticks -1) / max_arr_plus_1);

    // Ensure PSC is within the 16-bit limit for the register
    if (psc > 0xFFFF) {
        // Frequency is too low, or total_ticks calculation overflowed 64-bit? (Unlikely for F401)
        // Set PSC to max value as a fallback. This will result in the lowest possible frequency.
        psc = 0xFFFF;
    }

    // Calculate ARR: (total_ticks / (psc + 1)) - 1
    // Use 64-bit division if total_ticks is large
    arr = (uint32_t)((total_ticks / (psc + 1)) - 1);

    // Ensure calculated ARR is within the actual max ARR limit for the timer
     if (arr > config->TIM_Max_ARR) {
        // This should ideally not happen with the calculation method above if psc is correct.
        // If it does, it suggests an issue or boundary case. Cap ARR at max.
        arr = config->TIM_Max_ARR;
     }

    // Write PSC and ARR values to the registers
    config->TIM_Instance->PSC = psc;
    config->TIM_Instance->ARR = arr;

    // Calculate and write the Capture/Compare Register (CCR) value
    // CCR = (Duty / 100) * (ARR + 1)
    uint32_t ccr_value;
    uint64_t arr_plus_1 = (uint64_t)arr + 1; // Use 64-bit for intermediate calculation if ARR is 32-bit

    if (duty == 0) {
        ccr_value = 0; // 0% duty cycle
    } else if (duty == 100) {
        ccr_value = (uint32_t)arr_plus_1; // 100% duty cycle means pulse is active for entire period
                                          // Note: Some configurations might use ARR for 100% duty
                                          // depending on timer mode and desired behavior.
                                          // For edge-aligned PWM1, CCR = ARR+1 means output is always active.
    } else {
        // Use 64-bit arithmetic to prevent overflow during multiplication (duty * (ARR+1))
        ccr_value = (uint32_t)((uint64_t)duty * arr_plus_1 / 100);
    }

    // Write CCR value to the appropriate CCR register
    // The config struct stores the offset to the specific CCR register (CCR1, CCR2, etc.)
    volatile uint32_t *ccr_reg = (volatile uint32_t *)((uint32_t)config->TIM_Instance + config->TIM_CCR_Offset);
    *ccr_reg = ccr_value;

    // Generate an update event to load the new PSC, ARR, and CCR values immediately
    // Note: This can cause a brief perturbation (glitch) in the output signal as registers update.
    // For glitch-free updates, you would typically disable the Update Generation (UDIS bit in CR1),
    // configure all registers, then re-enable updates and generate one event via software (UG bit).
    config->TIM_Instance->EGR = TIM_EGR_UG;
}

/***********************************************************************************************************************
* Function Name  : PWM_Start
* Description    : Enables and starts the PWM signal generation on the specified channel.
* Parameters     : TRD_Channel_t TRD_Channel: The specific PWM channel.
* Return Value   : None.
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    t_pwm_channel channel = TRD_Channel; // Use local alias

    if (channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel
        return;
    }

    const PWM_ChannelConfig_t *config = &g_pwmChannelConfigs[channel];

    // Enable the specific channel's output in the Capture/Compare Enable Register (CCER)
    // CCxE bit (bit 0, 4, 8, 12) - set to 1
    config->TIM_Instance->CCER |= (0x1U << config->TIM_CCER_Shift);

    // Enable the Main Output (MOE) for advanced timers (TIM1, TIM8).
    // Our implemented timers (TIM3, TIM4, TIM5) are general purpose timers and do not have/require MOE.
    // If TIM1 were used, the BDTR_MOE bit would need to be set here.
    // Example (if TIM1 was NOT reserved):
    // if (config->TIM_Instance == TIM1) {
    //    config->TIM_Instance->BDTR |= TIM_BDTR_MOE; // Enable Main Output
    // }

    // Start the timer counter by setting the Counter Enable bit (CEN) in CR1
    config->TIM_Instance->CR1 |= TIM_CR1_CEN;
}

/***********************************************************************************************************************
* Function Name  : PWM_Stop
* Description    : Stops the PWM signal output on the specified channel.
* Parameters     : TRD_Channel_t TRD_Channel: The specific PWM channel.
* Return Value   : None.
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    t_pwm_channel channel = TRD_Channel; // Use local alias

    if (channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel
        return;
    }

    const PWM_ChannelConfig_t *config = &g_pwmChannelConfigs[channel];

    // Disable the specific channel's output in the Capture/Compare Enable Register (CCER)
    // CCxE bit (bit 0, 4, 8, 12) - clear to 0
    config->TIM_Instance->CCER &= ~(0x1U << config->TIM_CCER_Shift);

    // Note: This function only disables the output for the specified channel.
    // The timer counter itself might continue running if other channels on the same
    // timer instance are still active or the timer is used for other purposes.
    // If the requirement were to stop the entire timer, TIMx->CR1 &= ~TIM_CR1_CEN; would be used,
    // but this would affect all channels on that timer.
}

/***********************************************************************************************************************
* Function Name  : PWM_PowerOff
* Description    : Disables all PWM peripherals and outputs handled by this module to reduce power consumption.
* Parameters     : None.
* Return Value   : None.
***********************************************************************************************************************/
void PWM_PowerOff(void)
{
    // Disable outputs and reconfigure GPIOs for all implemented channels
    for (t_pwm_channel channel = (t_pwm_channel)0; channel < TRD_CHANNEL_COUNT; channel++)
    {
         const PWM_ChannelConfig_t *config = &g_pwmChannelConfigs[channel];

         // Disable the specific channel output (CCER, CCxE bit)
         config->TIM_Instance->CCER &= ~(0x1U << config->TIM_CCER_Shift);

         // Reconfigure the GPIO pin to a low-power state (e.g., Input, No Pull)
         // This disconnects the Alternate Function mapping and reduces potential leakage.
         uint32_t pin_mode_shift = config->GPIO_Pin * 2;
         config->GPIO_Port->MODER &= ~(0x3U << pin_mode_shift); // Clear mode bits (sets to Input - 00)

         // Clear Alternate Function mapping in AFRL/AFRH
         if (config->GPIO_Pin < 8) {
             uint32_t pin_af_shift = config->GPIO_Pin * 4;
             config->GPIO_Port->AFR[0] &= ~(0xFU << pin_af_shift);
         } else {
             uint32_t pin_af_shift = (config->GPIO_Pin - 8) * 4;
             config->GPIO_Port->AFR[1] &= ~(0xFU << pin_af_shift);
         }

         // Clear Pull-up/Pull-down (PUPDR bits to 00 = No pull)
         config->GPIO_Port->PUPDR &= ~(0x3U << pin_mode_shift);
    }

    // Stop the timer counters for all timers used by this module
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM5->CR1 &= ~TIM_CR1_CEN;

    // Disable Main Output (MOE) for advanced timers if applicable (TIM1 is reserved in this impl)
    // If TIM1 was used and its MOE bit might be set, clear it here:
    // if (TIM1->BDTR & TIM_BDTR_MOE) { TIM1->BDTR &= ~TIM_BDTR_MOE; }


    // Disable Timer clocks in RCC to reduce power
    // Only disable clocks for timers managed by this module.
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN);
    // Add other RCC_APBxENR_TIMxEN bits here if other timers (like TIM9, TIM10, TIM11 if used) were enabled.
    // RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN); // Only disable if actually used
}

/*
// Example of how TRD_Channel_t might be defined in STM32F401RC_PWM.h
// This section is commented out as it belongs in the header file.
// It is included here only for context regarding the enum definition.

#ifndef STM32F401RC_PWM_H
#define STM32F401RC_PWM_H

// Standard includes (adjust paths as needed)
#include <stdint.h> // For uint8_t, uint32_t etc. (if not defined elsewhere)
#include <stddef.h> // For offsetof

// Define tbyte and tlong if they aren't defined globally
// Ensure these types are appropriate for your system (e.g., 8-bit for duty, 32/64-bit for frequency)
#ifndef tbyte
typedef uint8_t tbyte;
#endif
#ifndef tlong
typedef uint32_t tlong;
#endif


// Reserved Timers: TIM1, TIM2 (for OS, delay, etc.)
// Available Timers for this module: TIM3, TIM4, TIM5, (TIM9, TIM10, TIM11 could also be added)
// Define the enum mapping abstract channels to specific timer/pin configurations.
// The order MUST match the g_pwmChannelConfigs array in the .c file.
typedef enum {
    // TIM3 Channels
    TRD_CHANNEL_TIM3_CH1, // PA6
    TRD_CHANNEL_TIM3_CH2, // PA7
    TRD_CHANNEL_TIM3_CH3, // PB0
    TRD_CHANNEL_TIM3_CH4, // PB1

    // TIM4 Channels
    TRD_CHANNEL_TIM4_CH1, // PB6
    TRD_CHANNEL_TIM4_CH2, // PB7
    TRD_CHANNEL_TIM4_CH3, // PB8
    TRD_CHANNEL_TIM4_CH4, // PB9

    // TIM5 Channels (32-bit timer)
    TRD_CHANNEL_TIM5_CH1, // PA0
    TRD_CHANNEL_TIM5_CH2, // PA1
    TRD_CHANNEL_TIM5_CH3, // PA2
    TRD_CHANNEL_TIM5_CH4, // PA3

    // Add other available channels here if needed (e.g., TIM9, TIM10, TIM11)

    TRD_CHANNEL_COUNT // Keep track of total channels defined
} TRD_Channel_t;


// Function Prototypes
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif // STM32F401RC_PWM_H
*/